import contextlib
import hashlib
import importlib.util
import io
import json
import pathlib
import tempfile
import unittest
from unittest import mock

SPEC = importlib.util.spec_from_file_location('vast_drive_fetch', pathlib.Path(__file__).with_name('vast_drive_fetch.py'))
fetch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fetch)


class Response(io.BytesIO):
    def __init__(self, body, start, end, total, status=206, length=None, content_range=None):
        super().__init__(body)
        self.status = status
        self.headers = {'Content-Length': str(len(body) if length is None else length),
                        'Content-Range': content_range or 'bytes %d-%d/%d' % (start, end, total)}


class Tests(unittest.TestCase):
    def contract(self, destination='/workspace/new.tar.zst', body=b'abcdefghijklmno'):
        return dict(file_id='valid_Drive_file_123', sha256=hashlib.sha256(body).hexdigest(),
                    bytes=len(body), destination=destination, access_token='TEST_TOKEN_NOT_REAL')

    def test_spec_and_budget(self):
        self.assertEqual(fetch.validate_spec(self.contract()), 61)
        p = self.contract(); p['bytes'] = 64 * 1024**3
        self.assertEqual(fetch.validate_spec(p), 900)
        for field, value in [('file_id', '../unsafe'), ('sha256', 'bad'), ('bytes', 0),
                             ('bytes', True), ('destination', '/tmp/new.tar.zst'),
                             ('destination', '/workspace/../tmp/new.tar.zst'),
                             ('destination', '/workspace/new.zip')]:
            p = self.contract(); p[field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                fetch.validate_spec(p)

    def test_range_headers(self):
        headers = {'Content-Length': '4', 'Content-Range': 'bytes 0-3/10'}
        fetch.validate_range(206, headers, 0, 3, 10)
        for status, wrong in [(200, headers), (206, {}),
                              (206, dict(headers, **{'Content-Length': '5'})),
                              (206, dict(headers, **{'Content-Range': 'bytes 1-4/10'}))]:
            with self.subTest(status=status, headers=wrong), self.assertRaises(ValueError):
                fetch.validate_range(status, wrong, 0, 3, 10)

    def test_destination_existing_and_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder)/'new.tar.zst'
            self.assertEqual(fetch.validate_destination(path)[0], path)
            path.write_bytes(b'old')
            with self.assertRaises(FileExistsError): fetch.validate_destination(path)
            path.unlink(); pathlib.Path(str(path)+'.partial').symlink_to('/no/such/file')
            with self.assertRaises(FileExistsError): fetch.validate_destination(path)
            alias = pathlib.Path(folder)/'alias'; alias.symlink_to(folder)
            with self.assertRaises(ValueError): fetch.validate_destination(alias/'other.tar.zst')

    def test_speed_gate_startup_and_sustained_low(self):
        gate = fetch.SpeedGate()
        self.assertEqual(gate.update(100, 0), (None, False))
        for t in range(0, 20, 2):
            rate, blocked = gate.update(200+t, (t+1)*1000000)
            self.assertFalse(blocked)
        self.assertTrue(gate.update(220, 21000000)[1])

    def test_speed_gate_recovery_resets_low_interval(self):
        gate = fetch.SpeedGate()
        for t in range(0, 16, 2): gate.update(t, (t+1)*1000000)
        self.assertFalse(gate.update(16, 300000000)[1])
        self.assertIsNone(gate.low_since)

    def execute_stub(self, body, corrupt=False, broken_headers=False, truncate=False, oversized=False):
        folder = tempfile.TemporaryDirectory(); self.addCleanup(folder.cleanup)
        dest = pathlib.Path(folder.name)/'archive.tar.zst'
        p = self.contract(str(dest), body)
        calls = []
        def opener(request, timeout):
            self.assertEqual(timeout, 8)
            calls.append(request)
            start, end = map(int, request.headers['Range'][6:].split('-'))
            chunk = body[start:end+1]
            if corrupt: chunk = bytes([chunk[0] ^ 1]) + chunk[1:]
            if truncate: chunk = chunk[:-1]
            if oversized: chunk += b'!'
            return Response(chunk, start, end, len(body), length=end-start+1,
                            content_range='bytes 99-100/101' if broken_headers else None)
        with mock.patch.object(fetch, 'validate_spec', return_value=60), mock.patch.object(fetch, 'RANGE_BYTES', 4), contextlib.redirect_stdout(io.StringIO()):
            result = fetch.fetch_archive(p, opener=opener)
        return dest, result, calls

    def test_parallel_range_download_and_sha_publish(self):
        body = b'abcdefghijklmno'
        dest, result, calls = self.execute_stub(body)
        self.assertTrue(result['complete'])
        self.assertEqual(dest.read_bytes(), body)
        self.assertEqual(result['received_bytes'], len(body))
        self.assertEqual(len(calls), 4)
        self.assertFalse(pathlib.Path(str(dest)+'.partial').exists())
        receipt = pathlib.Path(str(dest)+'.transfer.json').read_text()
        self.assertNotIn('TEST_TOKEN_NOT_REAL', receipt)
        self.assertTrue(json.loads(receipt)['complete'])

    def test_corrupt_or_wrong_or_truncated_keeps_partial(self):
        for keyword in ('corrupt', 'broken_headers', 'truncate', 'oversized'):
            with self.subTest(case=keyword):
                dest, result, _ = self.execute_stub(b'abcdefghijklmno', **{keyword: True})
                self.assertFalse(result['complete'])
                self.assertFalse(dest.exists())
                self.assertTrue(pathlib.Path(str(dest)+'.partial').exists())
                self.assertEqual(json.loads(pathlib.Path(str(dest)+'.transfer.json').read_text())['state'], 'failed')

    def test_atomic_publish_refuses_existing_archive(self):
        with tempfile.TemporaryDirectory() as folder:
            dest = pathlib.Path(folder)/'archive.tar.zst'; partial = pathlib.Path(str(dest)+'.partial')
            dest.write_bytes(b'old'); partial.write_bytes(b'new')
            with self.assertRaises(FileExistsError): fetch.publish_no_overwrite(partial, dest)
            self.assertEqual(dest.read_bytes(), b'old')
            self.assertEqual(partial.read_bytes(), b'new')

    def test_default_plan_does_not_touch_credentials_or_network(self):
        p = self.contract()
        argv = ['vast_drive_fetch.py', '--file-id', p['file_id'], '--sha256', p['sha256'],
                '--bytes', str(p['bytes']), '--destination', p['destination']]
        with mock.patch.object(fetch.sys, 'argv', argv), \
                mock.patch.object(fetch, 'refresh_access_token') as token, \
                mock.patch.object(fetch.subprocess, 'run') as process, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(fetch.main(), 0)
        token.assert_not_called(); process.assert_not_called()

    def test_missing_fields_fail_without_network_or_secret_error_text(self):
        output = io.StringIO()
        with mock.patch.object(fetch.sys, 'argv', ['vast_drive_fetch.py']), \
                mock.patch.object(fetch, 'refresh_access_token') as token, \
                mock.patch.object(fetch.subprocess, 'run') as process, \
                contextlib.redirect_stdout(output):
            self.assertEqual(fetch.main(), 1)
        self.assertEqual(json.loads(output.getvalue())['error_type'], 'ValueError')
        token.assert_not_called(); process.assert_not_called()


if __name__ == '__main__':
    unittest.main()
