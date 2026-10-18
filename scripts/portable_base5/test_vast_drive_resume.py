import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import vast_drive_resume as resume


class Response(io.BytesIO):
    def __init__(self, body, offset, total, status=206):
        super().__init__(body)
        self.status = status
        self.headers = {'Content-Length': str(len(body)),
                        'Content-Range': 'bytes %d-%d/%d' % (offset, offset+len(body)-1, total)}


class Tests(unittest.TestCase):
    def make_inputs(self, directory, body=b'abcdefghijklmno', old_bytes=None, allow_slow=False):
        archive = directory/'trusted.tar.zst'; archive.write_bytes(body)
        proof = dict(complete=True, verified_decompressed_members=True, source_package_unchanged=True,
                     archive=str(archive), bytes=len(body), sha256=hashlib.sha256(body).hexdigest())
        pack = directory/'pack.json'; pack.write_text(json.dumps(proof))
        old = directory/'old.tar.zst'; new = directory/'new.tar.zst'
        with mock.patch.object(resume.fetch, 'validate_spec', return_value=60):
            plan = resume.make_plan(archive, pack, 'test_file_id_valid', old, new, allow_slow)
        partial = Path(str(old)+'.partial')
        partial.write_bytes(body if old_bytes is None else old_bytes)
        old_receipt = Path(str(old)+'.transfer.json')
        old_receipt.write_text(json.dumps(dict(complete=False, state='failed', destination=str(old),
            file_id=plan['file_id'], expected_sha256=plan['sha256'], expected_bytes=len(body))))
        return plan, archive, pack, partial, old_receipt

    def execute(self, plan, body, corrupt=False, bad_status=False):
        plan = dict(plan, access_token='TEST_SHORT_LIVED_TOKEN_NOT_REAL')
        calls = []
        def opener(request, timeout):
            self.assertEqual(timeout, 8)
            offset, end = map(int, request.headers['Range'][6:].split('-'))
            calls.append((offset, end))
            chunk = body[offset:end+1]
            if corrupt: chunk = bytes([chunk[0] ^ 1])+chunk[1:]
            return Response(chunk, offset, len(body), 200 if bad_status else 206)
        with mock.patch.object(resume.fetch, 'validate_spec', return_value=60), contextlib.redirect_stdout(io.StringIO()):
            result = resume.recover(plan, opener)
        return result, calls

    def test_local_full_sha_and_range_manifest(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, archive, pack, _, _ = self.make_inputs(Path(folder))
            self.assertEqual([b['bytes'] for b in p['blocks']], [4,4,4,3])
            self.assertEqual(p['network_deadline_seconds'], 120)
            self.assertFalse(p['allow_slow_completion'])
            archive.write_bytes(b'changed')
            with mock.patch.object(resume.fetch, 'validate_spec', return_value=60), self.assertRaises(ValueError):
                resume.make_plan(archive, pack, p['file_id'], p['old_destination'], p['destination'])

    def test_only_mismatched_range_downloaded_old_files_preserved(self):
        body = b'abcdefghijklmno'
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, old, old_receipt = self.make_inputs(Path(folder), old_bytes=body[:8]+b'XXXX'+body[12:])
            old_hash, receipt_bytes = resume.digest(old), old_receipt.read_bytes()
            result, calls = self.execute(p, body)
            self.assertTrue(result['complete'])
            self.assertEqual(calls, [(8,11)])
            self.assertEqual(result['reused_bytes'], 11)
            self.assertEqual(result['missing_bytes'], 4)
            self.assertEqual(Path(p['destination']).read_bytes(), body)
            self.assertEqual(resume.digest(old), old_hash)
            self.assertEqual(old_receipt.read_bytes(), receipt_bytes)
            self.assertNotIn('access_token', Path(p['destination']+'.transfer.json').read_text())

    def test_short_ranges_and_sparse_holes_are_not_reused(self):
        body = b'abcdefghijklmno'
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, _, _ = self.make_inputs(Path(folder), old_bytes=body[:4]+b'\0'*4)
            result, calls = self.execute(p, body)
            self.assertTrue(result['complete'])
            self.assertEqual(sorted(calls), [(4,7),(8,11),(12,14)])
            self.assertEqual(result['reused_bytes'], 4)

    def test_all_verified_partial_requires_no_network(self):
        body = b'abcdefghijklmno'
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, _, _ = self.make_inputs(Path(folder))
            result, calls = self.execute(p, body)
            self.assertTrue(result['complete']); self.assertEqual(calls, [])
            self.assertEqual(result['network_received_bytes'], 0)

    def test_wrong_sha_or_http_status_never_publishes(self):
        for keyword in ('corrupt', 'bad_status'):
            with self.subTest(case=keyword), tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
                p, _, _, old, old_receipt = self.make_inputs(Path(folder), old_bytes=b'')
                old_bytes, receipt_bytes = old.read_bytes(), old_receipt.read_bytes()
                result, _ = self.execute(p, b'abcdefghijklmno', **{keyword: True})
                self.assertFalse(result['complete']); self.assertFalse(Path(p['destination']).exists())
                self.assertTrue(Path(p['destination']+'.partial').exists())
                self.assertEqual(old.read_bytes(), old_bytes); self.assertEqual(old_receipt.read_bytes(), receipt_bytes)

    def test_bad_failure_binding_rejected_before_network_or_new_file(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, _, old_receipt = self.make_inputs(Path(folder))
            proof = json.loads(old_receipt.read_text()); proof['file_id'] = 'another_file'; old_receipt.write_text(json.dumps(proof))
            with mock.patch.object(resume.fetch, 'validate_spec', return_value=60), self.assertRaises(ValueError):
                resume.recover(p, mock.Mock(side_effect=AssertionError('network forbidden')))
            self.assertFalse(Path(p['destination']+'.partial').exists())

    def test_existing_output_and_symlink_old_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, old, _ = self.make_inputs(Path(folder))
            Path(p['destination']).write_bytes(b'old output')
            with mock.patch.object(resume.fetch, 'validate_spec', return_value=60), self.assertRaises(FileExistsError):
                resume.recover(p)
            Path(p['destination']).unlink(); old.unlink(); old.symlink_to('/not/a/file')
            with mock.patch.object(resume.fetch, 'validate_spec', return_value=60), self.assertRaises(ValueError):
                resume.recover(p)

    def test_range_manifest_policy_and_deadline_guards(self):
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(resume.fetch, 'RANGE_BYTES', 4):
            p, _, _, _, _ = self.make_inputs(Path(folder), allow_slow=True)
            self.assertTrue(p['allow_slow_completion'])
            for change in (dict(network_deadline_seconds=121), dict(destination=p['old_destination']),
                           dict(blocks=p['blocks'][:-1]), dict(allow_slow_completion='yes'),
                           dict(blocks=[dict(p['blocks'][0], bytes=0)]+p['blocks'][1:]),
                           dict(blocks=[dict(p['blocks'][0], bytes=-1)]+p['blocks'][1:]),
                           dict(blocks=p['blocks']+[dict(offset=p['bytes'], bytes=0, sha256='0'*64)])):
                with self.subTest(change=change), mock.patch.object(resume.fetch, 'validate_spec', return_value=60), self.assertRaises(ValueError):
                    resume.validate_contract(dict(p, **change))

    def test_default_plan_no_oauth_or_ssh(self):
        with tempfile.TemporaryDirectory() as folder:
            p, archive, pack, _, _ = self.make_inputs(Path(folder))
            argv = ['vast_drive_resume.py','--archive',str(archive),'--pack-receipt',str(pack),
                    '--file-id',p['file_id'],'--old-destination','/workspace/old.tar.zst','--destination','/workspace/new.tar.zst']
            with mock.patch.object(resume.sys, 'argv', argv), mock.patch.object(resume.fetch, 'refresh_access_token') as oauth, \
                    mock.patch.object(resume.subprocess, 'run') as process, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(resume.main(), 0)
            oauth.assert_not_called(); process.assert_not_called()


if __name__ == '__main__':
    unittest.main()
