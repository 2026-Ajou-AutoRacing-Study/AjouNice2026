"""CPU read-only progress fixtures; no network, ROS or simulator activity."""
import contextlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import status


class StatusTests(unittest.TestCase):
    def setUp(self):
        previous = os.umask(0o077)
        self.addCleanup(os.umask, previous)

    def fixture(self, folder):
        root = Path(folder) / status.live.WORKSPACE
        root.mkdir(mode=0o700)
        return dict(workspace=root), root / 'collections/sunny13_5k'

    def state(self, root, **changes):
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        value = dict(stage='running', requested=5000, accepted_candidate=128, review=2,
                     elapsed_sec=3600, eta_sec=9000, updated_at=1000,
                     current='chunk_0008', checkpoint_only=True)
        value.update(changes)
        (root / 'status.json').write_text(json.dumps(value))

    def snapshot_files(self, root):
        return {str(path.relative_to(root)): (path.read_bytes(), path.stat().st_mtime_ns,
                path.stat().st_mode) for path in root.rglob('*') if path.is_file()}

    def test_no_output_is_normal_waiting_without_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            view = status.snapshot(ctx)
            self.assertIn('준비 대기', status.render(view))
            self.assertEqual(view['target'], 5000)
            self.assertFalse(root.exists())
            self.assertFalse(view['files_written'])
            self.assertFalse(view['capture_started'])

    def test_one_snapshot_shows_korean_counts_and_keeps_every_file_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            self.state(root)
            (root / 'quota.json').write_text(json.dumps(dict(target=5000, accepted=128,
                                                            deficit=2, pending=4870)))
            txt = ctx['workspace'] / 'MORAI_START.txt'
            txt.write_text('DO NOT TOUCH BELOW\noperator private section\n')
            before = self.snapshot_files(ctx['workspace'])
            text = status.render(status.snapshot(ctx, now=1010))
            for expected in ('승인 128 / 5,000장', '검토·부족 2장', '1시간 0분 0초',
                             '2시간 30분 0초', 'chunk_0008', '10초 전', '부모 체크포인트'):
                self.assertIn(expected, text)
            self.assertEqual(before, self.snapshot_files(ctx['workspace']))

    def test_old_status_is_explicitly_not_an_alive_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            self.state(root)
            self.assertIn('정상 동작을 보증하지 않는다', status.render(status.snapshot(ctx, now=1100)))

    def test_paused_and_complete_do_not_claim_a_running_eta(self):
        for stage in ('paused', 'complete', 'complete_with_deficit', 'halted'):
            with self.subTest(stage=stage), tempfile.TemporaryDirectory() as folder:
                ctx, root = self.fixture(folder)
                self.state(root, stage=stage, current=None)
                text = status.render(status.snapshot(ctx, now=1000))
                self.assertIn('예상 잔여 진행하지 않음', text)
                self.assertIn('현재 작업: 없음', text)

    def test_rejects_wrong_campaign_counts_and_nonfinite_times(self):
        for change in (dict(requested=3), dict(accepted_candidate=5001), dict(review=-1),
                       dict(accepted_candidate=True), dict(elapsed_sec=float('nan')),
                       dict(eta_sec=-10), dict(updated_at=None)):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as folder:
                ctx, root = self.fixture(folder)
                self.state(root, **change)
                with self.assertRaises(ValueError):
                    status.snapshot(ctx)

    def test_rejects_wrong_quota_without_using_it_as_latest_progress(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            self.state(root)
            (root / 'quota.json').write_text('{"target":5000,"accepted":5,"deficit":0,"pending":0}')
            with self.assertRaises(ValueError):
                status.snapshot(ctx)

    def test_rejects_root_and_status_symlinks(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            root.parent.mkdir(mode=0o700)
            external = Path(folder) / 'external'
            external.mkdir(mode=0o700)
            root.symlink_to(external, target_is_directory=True)
            with self.assertRaises(ValueError):
                status.snapshot(ctx)
            root.unlink()
            root.mkdir(mode=0o700)
            (root / 'status.json').symlink_to(external / 'missing.json')
            with self.assertRaises(ValueError):
                status.snapshot(ctx)

    def test_main_once_never_calls_sleep_or_starts_subprocess(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            stream = io.StringIO()
            with patch.object(status.live, 'load', return_value=ctx) as load, \
                    patch.object(status.live, 'ready') as ready, \
                    patch.object(status.time, 'sleep') as sleep, \
                    patch.object(subprocess, 'Popen') as child, \
                    contextlib.redirect_stdout(stream):
                self.assertEqual(status.main(['--workspace', str(ctx['workspace'])]), 0)
            load.assert_called_once_with(ctx['workspace'], verify=False)
            ready.assert_called_once_with(ctx)
            sleep.assert_not_called()
            child.assert_not_called()
            self.assertIn('준비 대기', stream.getvalue())

    def test_watch_read_error_does_not_reprint_old_success_as_current(self):
        with tempfile.TemporaryDirectory() as folder:
            ctx, root = self.fixture(folder)
            stream = io.StringIO()
            with patch.object(status.live, 'load', return_value=ctx), \
                    patch.object(status.live, 'ready'), \
                    patch.object(status, 'snapshot', side_effect=ValueError('broken file')), \
                    patch.object(status.time, 'sleep', side_effect=KeyboardInterrupt) as sleep, \
                    contextlib.redirect_stdout(stream):
                with self.assertRaises(KeyboardInterrupt):
                    status.main(['--workspace', str(ctx['workspace']), '--watch'])
            self.assertIn('읽기 오류 1회', stream.getvalue())
            self.assertIn('이전 값을 최신 정보로 표시하지 않음', stream.getvalue())
            self.assertNotIn('승인 ', stream.getvalue())
            sleep.assert_called_once_with(5)

    def test_prefers_deployed_v2_without_importing_historical_executor(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'status.py').write_bytes(Path(status.__file__).read_bytes())
            (root / 'live.py').write_text('raise RuntimeError("stale executor must not load")\n')
            (root / 'live_v2.py').write_text('WORKSPACE="v2_fixture"\n')
            result = subprocess.run([sys.executable, '-B', str(root / 'status.py'), '--help'],
                                    capture_output=True, text=True, check=True)
            self.assertIn('--watch', result.stdout)
            self.assertFalse((root / '__pycache__').exists())


if __name__ == '__main__':
    unittest.main()
