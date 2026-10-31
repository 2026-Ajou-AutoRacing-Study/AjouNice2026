"""CPU fixtures only; never start/stop ROS, a collector or MORAI."""
import contextlib
import fcntl
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import control


class ControlTests(unittest.TestCase):
    def setUp(self):
        previous = os.umask(0o077)
        self.addCleanup(os.umask, previous)

    def fixture(self, folder):
        workspace = Path(folder) / control.live.WORKSPACE
        workspace.mkdir(mode=0o700)
        session = workspace / 'work/ros-session-v2'
        session.mkdir(parents=True, mode=0o700)
        (session / 'capture.lock').write_text('')
        output = workspace / 'collections/sunny13_5k'
        output.mkdir(parents=True, mode=0o700)
        text = workspace / 'MORAI_START.txt'
        text.write_text('DO NOT TOUCH BELOW\noperator private commands\n')
        ctx = dict(workspace=workspace, session=session)
        return ctx, output, text

    def json_file(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        path.write_text(json.dumps(value))

    def native_session(self, root, name='chunk_0000/capture'):
        folder = root / name
        self.json_file(folder / 'session_contract.json', {'source': 'fixture'})
        self.json_file(folder / 'status.json', {'stage': 'complete'})
        (folder / 'ledger.jsonl').write_text('{"status":"accepted_candidate"}\n')
        self.json_file(folder / 'shutdown_safety.json', dict(park_verified=True, gear=1,
            brake_held_during_requests=True, request_fields=dict(option=2, gear=1)))
        (folder / '.runner.lock').write_text('')
        return folder

    def paused(self, folder):
        ctx, output, text = self.fixture(folder)
        self.native_session(output)
        self.json_file(output / 'status.json', dict(stage='paused', current=None,
            requested=5000, finished=16, updated_at=control.time.time() + 1))
        (output / 'PAUSE').write_text('original pause request\n')
        return ctx, output, text

    def snapshot(self, root):
        return {str(path.relative_to(root)): (path.read_bytes(), path.stat().st_ino,
                path.stat().st_mtime_ns, path.stat().st_mode)
                for path in root.rglob('*') if path.is_file()}

    def invoke(self, ctx, argv):
        stream = io.StringIO()
        with patch.object(control.live, 'load', return_value=ctx), \
                patch.object(control.live, 'ready') as ready, contextlib.redirect_stdout(stream):
            result = control.main(['--workspace', str(ctx['workspace']), *argv])
            ready.assert_called_once_with(ctx)
        return result, json.loads(stream.getvalue())

    def test_deployed_v2_executor_precedes_stale_bootstrap_live_module(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'control.py').write_bytes(Path(control.__file__).read_bytes())
            (root / 'live.py').write_text('raise RuntimeError("stale bootstrap must not load")\n')
            (root / 'live_v2.py').write_text('WORKSPACE="fixture_approved_v2"\n')
            result = subprocess.run([sys.executable, '-B', str(root / 'control.py'), '--help'],
                                    capture_output=True, text=True, check=True)
            self.assertIn('resume-paused', result.stdout)
            self.assertFalse((root / '__pycache__').exists())

    def test_default_actions_are_readonly_without_runtime_probe_or_output_creation(self):
        for action in ('pause', 'resume-paused'):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as folder:
                ctx, output, _ = self.fixture(folder)
                output.rmdir()
                before = self.snapshot(ctx['workspace'])
                with patch.object(control.live, 'service_owned') as probe:
                    result, report = self.invoke(ctx, [action])
                self.assertEqual(result, 0)
                self.assertFalse(report['files_written'])
                self.assertFalse(report['capture_started'])
                probe.assert_not_called()
                self.assertEqual(self.snapshot(ctx['workspace']), before)

    def test_pause_creates_only_private_marker_after_owned_active_capture(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, text = self.fixture(folder)
            old_text = text.read_bytes()
            with patch.object(control.live, 'service_owned', return_value=True):
                result = control.pause(ctx)
            self.assertTrue(result['created'])
            self.assertEqual((root / 'PAUSE').stat().st_mode & 0o777, 0o600)
            self.assertEqual(json.loads((root / 'PAUSE').read_text())['reason'], 'operator_graceful_pause')
            self.assertEqual(text.read_bytes(), old_text)
            self.assertFalse(result['signals_sent'])
            self.assertFalse(result['capture_started'])

    def test_pause_reuses_existing_marker_without_changing_its_bytes_or_mtime(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.fixture(folder)
            marker = root / 'PAUSE'
            marker.write_bytes(b'preserve user pause\n')
            old = self.snapshot(ctx['workspace'])
            with patch.object(control.live, 'service_owned', return_value=True):
                result = control.pause(ctx)
            self.assertFalse(result['created'])
            self.assertEqual(self.snapshot(ctx['workspace']), old)

    def test_pause_without_approved_active_capture_never_creates_marker(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.fixture(folder)
            with patch.object(control.live, 'service_owned', return_value=False):
                with self.assertRaisesRegex(ValueError, 'No active capture'):
                    control.pause(ctx)
            self.assertFalse((root / 'PAUSE').exists())

    def test_pause_refuses_symlink_or_writable_marker(self):
        for kind in ('symlink', 'writable'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as folder:
                ctx, root, text = self.fixture(folder)
                marker = root / 'PAUSE'
                if kind == 'symlink':
                    marker.symlink_to(text)
                else:
                    marker.write_text('preserve')
                    marker.chmod(0o666)
                with patch.object(control.live, 'service_owned', return_value=True):
                    with self.assertRaises(ValueError):
                        control.pause(ctx)
                self.assertEqual(text.read_text(), 'DO NOT TOUCH BELOW\noperator private commands\n')

    def test_safe_resume_only_renames_marker_and_returns_manual_command(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            before = self.snapshot(ctx['workspace'])
            marker = root / 'PAUSE'
            old = marker.stat()
            with patch.object(control.live, 'service_owned', return_value=False):
                report = control.resume_paused(ctx)
            audit = Path(report['archived_marker'])
            self.assertFalse(marker.exists())
            self.assertEqual(audit.read_text(), 'original pause request\n')
            self.assertEqual(audit.stat().st_ino, old.st_ino)
            self.assertEqual(audit.stat().st_mtime_ns, old.st_mtime_ns)
            after = self.snapshot(ctx['workspace'])
            prior_key = str(marker.relative_to(ctx['workspace']))
            audit_key = str(audit.relative_to(ctx['workspace']))
            self.assertEqual(after.pop(audit_key), before.pop(prior_key))
            self.assertEqual(after, before)
            self.assertEqual(report['next_manual_command'][0], 'bash')
            self.assertIn('--confirmed-environment', report['next_manual_command'])
            self.assertEqual(len(report['evidence']['native_sessions']), 1)
            self.assertFalse(report['capture_started'])

    def test_resume_requires_existing_capture_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            (ctx['session'] / 'capture.lock').unlink()
            with self.assertRaises(FileNotFoundError):
                control.resume_paused(ctx)
            self.assertTrue((root / 'PAUSE').exists())
            self.assertFalse((ctx['session'] / 'capture.lock').exists())

    def test_resume_refuses_active_parent_or_child_lock(self):
        for kind in ('parent', 'child'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as folder:
                ctx, root, _ = self.paused(folder)
                path = ctx['session'] / 'capture.lock' if kind == 'parent' else root / 'chunk_0000/capture/.runner.lock'
                with path.open() as lock:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    with patch.object(control.live, 'service_owned', return_value=False):
                        with self.assertRaisesRegex(ValueError, 'still active'):
                            control.resume_paused(ctx)
                self.assertTrue((root / 'PAUSE').exists())

    def test_resume_requires_paused_finished_parent_without_stop(self):
        cases = ('stage', 'current', 'empty', 'stop', 'dangling_stop')
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as folder:
                ctx, root, _ = self.paused(folder)
                status = json.loads((root / 'status.json').read_text())
                if case == 'stage':
                    status['stage'] = 'halted'
                elif case == 'current':
                    status['current'] = 'still active child'
                elif case == 'empty':
                    status['finished'] = 0
                elif case == 'stop':
                    (root / 'STOP').write_text('safety stop')
                else:
                    (root / 'STOP').symlink_to(root / 'missing')
                self.json_file(root / 'status.json', status)
                with patch.object(control.live, 'service_owned', return_value=False):
                    with self.assertRaises(ValueError):
                        control.resume_paused(ctx)
                self.assertTrue((root / 'PAUSE').exists())

    def test_resume_checks_all_recovered_native_attempts_not_only_first_safe_batch(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            recovery = self.native_session(root, 'chunk_0000/recovery/cell/round_0/near_raw')
            self.json_file(recovery / 'shutdown_safety.json', dict(park_verified=False))
            with patch.object(control.live, 'service_owned', return_value=False):
                with self.assertRaisesRegex(ValueError, 'final Park'):
                    control.resume_paused(ctx)
            self.assertTrue((root / 'PAUSE').exists())

    def test_resume_rejects_missing_park_or_extra_unbound_shutdown(self):
        for case in ('missing', 'unbound'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as folder:
                ctx, root, _ = self.paused(folder)
                if case == 'missing':
                    (root / 'chunk_0000/capture/shutdown_safety.json').unlink()
                else:
                    self.json_file(root / 'unbound/shutdown_safety.json', dict(park_verified=True))
                with patch.object(control.live, 'service_owned', return_value=False):
                    with self.assertRaisesRegex(ValueError, 'do not match'):
                        control.resume_paused(ctx)
                self.assertTrue((root / 'PAUSE').exists())

    def test_resume_rejects_incomplete_native_status_and_partial_park_records(self):
        for case in ('status', 'gear', 'brake', 'request'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as folder:
                ctx, root, _ = self.paused(folder)
                native = root / 'chunk_0000/capture'
                if case == 'status':
                    self.json_file(native / 'status.json', dict(stage='halted_exit_safety'))
                else:
                    path = native / 'shutdown_safety.json'
                    record = json.loads(path.read_text())
                    record.pop({'gear': 'gear', 'brake': 'brake_held_during_requests',
                                'request': 'request_fields'}[case])
                    self.json_file(path, record)
                with patch.object(control.live, 'service_owned', return_value=False):
                    with self.assertRaises(ValueError):
                        control.resume_paused(ctx)
                self.assertTrue((root / 'PAUSE').exists())

    def test_resume_refuses_hidden_symlink_session(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            (root / 'hidden_session').symlink_to(root / 'chunk_0000/capture', target_is_directory=True)
            with patch.object(control.live, 'service_owned', return_value=False):
                with self.assertRaisesRegex(ValueError, 'Symlink'):
                    control.resume_paused(ctx)
            self.assertTrue((root / 'PAUSE').exists())

    def test_resume_rejects_pause_modified_during_evidence_check(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            real = control.resume_evidence
            def changed(path, stack):
                result = real(path, stack)
                (root / 'PAUSE').write_text('new operator request')
                return result
            with patch.object(control.live, 'service_owned', return_value=False), \
                    patch.object(control, 'resume_evidence', side_effect=changed):
                with self.assertRaisesRegex(ValueError, 'PAUSE changed'):
                    control.resume_paused(ctx)
            self.assertEqual((root / 'PAUSE').read_text(), 'new operator request')

    def test_resume_does_not_overwrite_colliding_audit_file(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root, _ = self.paused(folder)
            audit = root / 'PAUSE.resumed_123_fixture.json'
            audit.write_text('existing operator evidence')
            with patch.object(control.live, 'service_owned', return_value=False), \
                    patch.object(control.time, 'time_ns', return_value=123), \
                    patch.object(control.uuid, 'uuid4', return_value=SimpleNamespace(hex='fixture')):
                with self.assertRaisesRegex(ValueError, 'already exists'):
                    control.resume_paused(ctx)
            self.assertEqual(audit.read_text(), 'existing operator evidence')
            self.assertTrue((root / 'PAUSE').exists())


if __name__ == '__main__':
    unittest.main()
