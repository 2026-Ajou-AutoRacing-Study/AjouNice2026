"""CPU-only fixtures/subprocess stubs; no Drive, credentials, Vast or training."""
import importlib.util
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

SPEC = importlib.util.spec_from_file_location('upload_training_archive',
    Path(__file__).with_name('upload_training_archive.py'))
u = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(u)


class UploadTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.archive = self.root / 'sunny15.tar.zst'
        self.archive.write_bytes(b'QUALIFIED ARCHIVE')
        self.pack = self.root / 'pack_receipt'; self.pack.mkdir()
        digest = u.hashes(self.archive)['sha256']
        self.intent = dict(schema='base5_qualified_training_archive_v1',
                           output=str(self.archive), expected_scenes=5000)
        self.complete = dict(event='qualified_training_archive_complete', complete=True,
            archive=str(self.archive), frames=5000, bytes=self.archive.stat().st_size,
            sha256=digest, verified_decompressed_members=True, source_package_unchanged=True)
        self.manifest = dict(archive=str(self.archive), bytes=self.archive.stat().st_size,
                             sha256=digest, members={'dataset/manifest.json': {'sha256': 'payload'}})
        self.save('intent.json', self.intent)
        self.save('COMPLETE.json', self.complete)
        self.save('archive_manifest.json', self.manifest)
        (self.pack / 'SHA256SUMS').write_text(digest + '  ' + self.archive.name + '\n')
        self.receipt = self.root / 'upload_receipt'

    def save(self, name, value):
        (self.pack / name).write_text(json.dumps(value))

    def args(self, run=False):
        return u.parser().parse_args(['--archive', str(self.archive), '--pack-receipt', str(self.pack),
            '--destination', 'ajou_drive_private:dataset/Base5/sunny15_fixture/',
            '--receipt-dir', str(self.receipt)] + (['--run'] if run else []))

    def remote(self, contract):
        return dict(Name=self.archive.name, Path=self.archive.name, IsDir=False,
                    Size=contract['bytes'], ID='drive_file_123', Hashes={'MD5': contract['md5']})

    def test_default_plan_does_not_write_or_start_subprocess(self):
        with patch.object(u.subprocess, 'run', side_effect=AssertionError('no subprocess')), \
                patch.object(u.subprocess, 'Popen', side_effect=AssertionError('no upload')):
            plan = u.upload(self.args())
        self.assertFalse(plan['execute'])
        self.assertFalse(self.receipt.exists())
        self.assertEqual(plan['minimum_bytes_per_second'], 8_000_000)
        self.assertEqual(plan['frames'], 5000)

    def test_corrupt_archive_blocks_before_receipt(self):
        self.archive.write_bytes(b'CHANGED')
        with self.assertRaisesRegex(ValueError, 'size/SHA256'):
            u.upload(self.args(True))
        self.assertFalse(self.receipt.exists())

    def test_partial_or_unverified_pack_refused(self):
        self.save('INCOMPLETE.json', dict(complete=False))
        with self.assertRaisesRegex(ValueError, 'Incomplete pack'):
            u.prepare(self.args())
        (self.pack / 'INCOMPLETE.json').unlink()
        self.complete['verified_decompressed_members'] = False
        self.save('COMPLETE.json', self.complete)
        with self.assertRaisesRegex(ValueError, 'Verified complete'):
            u.prepare(self.args())

    def test_wrong_pack_path_or_counts_or_checksum_refused(self):
        self.manifest['archive'] = str(self.root / 'other.tar.zst')
        self.save('archive_manifest.json', self.manifest)
        with self.assertRaisesRegex(ValueError, 'different archive'):
            u.prepare(self.args())
        self.manifest['archive'] = str(self.archive); self.save('archive_manifest.json', self.manifest)
        self.complete['frames'] = 4999; self.save('COMPLETE.json', self.complete)
        with self.assertRaisesRegex(ValueError, 'scene counts'):
            u.prepare(self.args())
        self.complete['frames'] = 5000; self.save('COMPLETE.json', self.complete)
        (self.pack / 'SHA256SUMS').write_text('invalid\n')
        with self.assertRaisesRegex(ValueError, 'SHA256SUMS'):
            u.prepare(self.args())

    def test_symlink_input_and_old_receipt_refused(self):
        link = self.root / 'linked.tar.zst'; link.symlink_to(self.archive)
        args = self.args(); args.archive = link
        with self.assertRaisesRegex(ValueError, 'Symlink'):
            u.prepare(args)
        self.receipt.mkdir()
        with self.assertRaisesRegex(ValueError, 'Fresh upload'):
            u.prepare(self.args())

    def test_destination_root_and_traversal_refused(self):
        for destination in ('remote:', 'remote:/', 'remote:../other/', 'remote:a//b/',
                            'remote:a/b', 'remote:a/./b/', 'local/folder/', '-remote:a/'):
            with self.subTest(destination=destination), self.assertRaises(ValueError):
                u.destination_folder(destination)

    def test_success_uses_one_upload_and_returns_verified_file_id(self):
        contract = u.prepare(self.args())
        summary = dict(transferred_bytes=contract['bytes'], elapsed_seconds=1.)
        with patch.object(u, 'check_rclone', return_value='rclone v1.73.1'), \
                patch.object(u, 'listing', side_effect=[[], [self.remote(contract)]]), \
                patch.object(u, 'monitor_upload', return_value=summary) as upload:
            result = u.upload(self.args(True))
        self.assertEqual(upload.call_count, 1)
        self.assertEqual(result['remote']['file_id'], 'drive_file_123')
        self.assertTrue((self.receipt / 'COMPLETE.json').exists())
        self.assertFalse((self.receipt / 'INCOMPLETE.json').exists())
        self.assertFalse(result['training_started'])

    def test_existing_remote_archive_never_starts_upload(self):
        contract = u.prepare(self.args())
        with patch.object(u, 'check_rclone', return_value='rclone v1.73.1'), \
                patch.object(u, 'listing', return_value=[self.remote(contract)]), \
                patch.object(u, 'monitor_upload') as upload:
            with self.assertRaisesRegex(ValueError, 'already exists'):
                u.upload(self.args(True))
        upload.assert_not_called()
        self.assertTrue((self.receipt / 'INCOMPLETE.json').exists())
        self.assertFalse((self.receipt / 'COMPLETE.json').exists())

    def test_slow_upload_never_retries_or_marks_complete(self):
        with patch.object(u, 'check_rclone', return_value='rclone v1.73.1'), \
                patch.object(u, 'listing', return_value=[]), \
                patch.object(u, 'monitor_upload', side_effect=ValueError('Sustained upload below 8 MB/s')) as upload:
            with self.assertRaisesRegex(ValueError, 'below 8'):
                u.upload(self.args(True))
        self.assertEqual(upload.call_count, 1)
        self.assertFalse((self.receipt / 'COMPLETE.json').exists())
        receipt = json.loads((self.receipt / 'INCOMPLETE.json').read_text())
        self.assertFalse(receipt['automatic_retries'])
        self.assertTrue(receipt['remote_partial_may_exist'])

    def test_remote_md5_size_id_and_duplicate_checks(self):
        contract = u.prepare(self.args())
        for override in ({'Size': 0}, {'ID': ''}, {'Hashes': {'MD5': 'wrong'}}, {'IsDir': True}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                u.verify_remote(contract, [dict(self.remote(contract), **override)])
        with self.assertRaisesRegex(ValueError, 'duplicated'):
            u.verify_remote(contract, [self.remote(contract)] * 2)

    def test_changed_source_after_upload_refused(self):
        contract = u.prepare(self.args())
        self.archive.write_bytes(b'MODIFIED AFTER SEND')
        with self.assertRaisesRegex(ValueError, 'changed during upload'):
            u.verify_pins(contract)

    def test_only_one_nonrecursive_copy_with_no_overwrite_flags(self):
        command = u.upload_command(u.prepare(self.args()))
        self.assertEqual(command[1], 'copyto')
        for flag in ('--immutable', '--ignore-existing', '--checksum', '--use-json-log'):
            self.assertIn(flag, command)
        self.assertEqual(command[command.index('--retries') + 1], '1')
        self.assertNotIn('sync', command)
        self.assertNotIn('--delete-after', command)

    def test_scoped_listing_missing_folder_and_auth_error(self):
        contract = u.prepare(self.args())
        with patch.object(u.subprocess, 'run', return_value=subprocess.CompletedProcess([], 3, b'', b'')):
            self.assertEqual(u.listing(contract, allow_missing=True), [])
        with patch.object(u.subprocess, 'run', return_value=subprocess.CompletedProcess([], 1, b'', b'SECRET')):
            with self.assertRaisesRegex(ValueError, '^Scoped Drive listing failed \\(exit 1\\)$'):
                u.listing(contract, allow_missing=True)

    def test_old_rclone_fails_before_remote_or_upload(self):
        with patch.object(u.subprocess, 'run', return_value=subprocess.CompletedProcess(
                [], 0, b'rclone v1.50.2\n- os/arch: linux/amd64\n', b'')), \
                patch.object(u, 'listing') as listing, patch.object(u, 'monitor_upload') as monitor:
            with self.assertRaisesRegex(ValueError, 'structured stats'):
                u.upload(self.args(True))
        listing.assert_not_called(); monitor.assert_not_called()

    def test_new_rclone_version_is_qualified_locally(self):
        with patch.object(u.subprocess, 'run', return_value=subprocess.CompletedProcess(
                [], 0, b'rclone v1.73.1\n- os/arch: linux/amd64\n', b'')):
            self.assertEqual(u.check_rclone(u.prepare(self.args())), 'rclone v1.73.1')

    def test_timeout_does_not_save_raw_credential_response(self):
        with patch.object(u, 'check_rclone', side_effect=subprocess.TimeoutExpired('secret_command', 5)):
            with self.assertRaises(subprocess.TimeoutExpired):
                u.upload(self.args(True))
        saved = (self.receipt / 'INCOMPLETE.json').read_text()
        self.assertIn('TimeoutExpired', saved)
        self.assertNotIn('secret_command', saved)

    def monitor_fixture(self, rate):
        contract = u.prepare(self.args())
        contract.update(bytes=rate * 10, upload_deadline_seconds=70.)
        self.receipt.mkdir()
        chunks = [(json.dumps(dict(stats={'bytes': now * rate}, msg='Bearer SECRET')) + '\n').encode()
                  for now in (2, 4, 6, 8, 10)] + [b'']
        clock = [0.]
        process = Mock()
        process.stdout.fileno.return_value = 42
        process.wait.return_value = 0

        class Selector:
            def __init__(self):
                self.active = True
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False
            def register(self, obj, event):
                self.obj = obj
            def unregister(self, obj):
                self.active = False
            def get_map(self):
                return {42: self.obj} if self.active else {}
            def select(self, timeout):
                return [(Mock(fileobj=self.obj), 1)]

        def next_chunk(fd, size):
            clock[0] += 2
            return chunks.pop(0)
        return contract, process, Selector, clock, next_chunk

    def test_actual_monitor_parses_stats_and_ignores_auth_text(self):
        contract, process, selector, clock, read = self.monitor_fixture(8_000_000)
        with patch.object(u.subprocess, 'Popen', return_value=process) as popen, \
                patch.object(u.selectors, 'DefaultSelector', selector), \
                patch.object(u.os, 'read', side_effect=read), \
                patch.object(u.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch('builtins.print'):
            summary = u.monitor_upload(contract, self.receipt)
        self.assertEqual(summary['transferred_bytes'], 80_000_000)
        self.assertEqual(summary['payload_seconds'], 10)
        self.assertTrue(popen.call_args.kwargs['start_new_session'])
        self.assertNotIn('SECRET', (self.receipt / 'progress.jsonl').read_text())
        process.stdout.close.assert_called_once()

    def test_actual_monitor_slow_stats_stop_only_its_child(self):
        contract, process, selector, clock, read = self.monitor_fixture(7_000_000)
        with patch.object(u.subprocess, 'Popen', return_value=process), \
                patch.object(u.selectors, 'DefaultSelector', selector), \
                patch.object(u.os, 'read', side_effect=read), \
                patch.object(u.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(u, 'stop_own_child') as stop, patch('builtins.print'):
            with self.assertRaisesRegex(ValueError, 'below 8 MB/s'):
                u.monitor_upload(contract, self.receipt)
        stop.assert_called_once_with(process)
        self.assertEqual(clock[0], 10)


class RateGuardTest(unittest.TestCase):
    def test_startup_excluded_and_exact_8mbps_allowed(self):
        guard = u.RateGuard(0, 100)
        for now in (2, 4, 6, 8):
            guard.observe(now, {'bytes': 0})
        for now in (10, 12, 14, 16, 18, 20):
            guard.observe(now, {'bytes': int((now - 8) * 8_000_000)})
        self.assertEqual(guard.payload_start, 8)
        self.assertEqual(guard.minimum_window_rate, 8_000_000)

    def test_sustained_7mbps_stops_at_first_window(self):
        guard = u.RateGuard(0, 100)
        for now in (2, 4, 6, 8):
            guard.observe(now, {'bytes': now * 7_000_000})
        with self.assertRaisesRegex(ValueError, 'below 8 MB/s'):
            guard.observe(10, {'bytes': 70_000_000})

    def test_fast_start_does_not_hide_later_stall(self):
        guard = u.RateGuard(0, 100)
        for now in (2, 4, 6, 8, 10):
            guard.observe(now, {'bytes': now * 20_000_000})
        with self.assertRaisesRegex(ValueError, 'below 8 MB/s'):
            for now in (12, 14, 16, 18, 20):
                guard.observe(now, {'bytes': 200_000_000})

    def test_no_payload_no_stats_and_hard_deadline_are_bounded(self):
        guard = u.RateGuard(0, 100)
        with self.assertRaisesRegex(ValueError, 'startup allowance'):
            guard.tick(20.01)
        guard.observe(2, {'bytes': 40_000_000})
        with self.assertRaisesRegex(ValueError, 'stats stopped'):
            guard.tick(12.01)
        with self.assertRaisesRegex(ValueError, 'hard deadline'):
            guard.tick(100.01)

    def test_bad_counter_refused(self):
        for value in (None, True, -1, float('nan'), float('inf')):
            guard = u.RateGuard(0, 100)
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'byte counter'):
                guard.observe(2, {'bytes': value})
        guard = u.RateGuard(0, 100); guard.observe(2, {'bytes': 5})
        with self.assertRaisesRegex(ValueError, 'byte counter'):
            guard.observe(4, {'bytes': 4})

    def test_finalization_is_not_misclassified_as_payload_stall(self):
        guard = u.RateGuard(0, 70, expected_bytes=80_000_000)
        for now in (2, 4, 6, 8, 10):
            guard.observe(now, {'bytes': now * 8_000_000})
        guard.tick(25)  # Drive may finalize metadata after all bytes were sent.
        guard.observe(26, {'bytes': 80_000_000})
        self.assertEqual(guard.summary(26)['payload_seconds'], 10)
        with self.assertRaisesRegex(ValueError, 'hard deadline'):
            guard.tick(70.01)

    def test_counter_larger_than_one_archive_refused(self):
        guard = u.RateGuard(0, 70, expected_bytes=80_000_000)
        with self.assertRaisesRegex(ValueError, 'single expected payload'):
            guard.observe(2, {'bytes': 80_000_001})

    def test_own_child_only_sigint_then_term(self):
        process = Mock(pid=123456)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('owned', 3), 0]
        with patch.object(u.os, 'killpg') as kill:
            u.stop_own_child(process)
        self.assertEqual(kill.call_args_list[0].args, (123456, signal.SIGINT))
        self.assertEqual(kill.call_args_list[1].args, (123456, signal.SIGTERM))


if __name__ == '__main__':
    unittest.main()
