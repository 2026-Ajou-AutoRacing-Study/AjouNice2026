import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).parent))
from base5_wandb_relay import allowed

SPEC = importlib.util.spec_from_file_location('oneclick', Path(__file__).with_name('base5_vast_oneclick.py'))
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


class OneclickTests(unittest.TestCase):
    def test_logging_relay_rejects_unrelated_destinations(self):
        self.assertTrue(allowed('api.wandb.ai', 443))
        self.assertTrue(allowed('storage.googleapis.com', 443))
        self.assertFalse(allowed('wandb.ai.attacker.test', 443))
        self.assertFalse(allowed('example.com', 443))
        self.assertFalse(allowed('api.wandb.ai', 80))

    def fixture(self, root):
        c = dict(launcher.DEFAULTS)
        for name in ('source', 'data', 'work'):
            c[name] = str(root / name)
            Path(c[name]).mkdir()
        c['initializer'] = str(root / 'initializer.pth')
        Path(c['initializer']).write_bytes(b'audited-init')
        c['config'] = 'config.py'
        (Path(c['source']) / c['config']).write_text('model = dict(compile_camera=True)')
        (Path(c['data']) / 'manifest.json').write_text('{}')
        h = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
        identity = dict(epochs=24, batch=4, workers=4, accumulate=1, train_count=4255,
                        val_enabled=True, lr=1e-4, source_hashes={'config.py': h(Path(c['source']) / c['config'])},
                        initial_checkpoint_sha256=h(c['initializer']),
                        config_sha256=h(Path(c['source']) / c['config']),
                        dataset_manifest_sha256=h(Path(c['data']) / 'manifest.json'))
        (Path(c['work']) / 'run_manifest.json').write_text(json.dumps(dict(contract_sha256=c['contract'], identity=identity)))
        return c

    def test_preflight_accepts_only_qualified_manifest(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            self.assertEqual(launcher.validate_manifest(c)['contract_sha256'], c['contract'])
            c['contract'] = 'changed'
            with self.assertRaisesRegex(ValueError, 'contract'):
                launcher.validate_manifest(c)

    def test_source_mutation_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            (Path(c['source']) / c['config']).write_text('changed')
            with self.assertRaisesRegex(ValueError, 'Frozen source'):
                launcher.validate_manifest(c)

    def test_initializer_and_dataset_mutation_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            Path(c['initializer']).write_bytes(b'cost-checkpoint')
            with self.assertRaisesRegex(ValueError, 'Initializer'):
                launcher.validate_manifest(c)
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            (Path(c['data']) / 'manifest.json').write_text('{"changed":true}')
            with self.assertRaisesRegex(ValueError, 'Dataset manifest'):
                launcher.validate_manifest(c)

    def test_started_run_requires_explicit_resume(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp)); w = Path(c['work'])
            (w / 'fresh_start.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'already started'):
                launcher.validate_manifest(c)
            with self.assertRaisesRegex(ValueError, 'No completed checkpoint'):
                launcher.validate_manifest(c, True)
            (w / 'last_checkpoint').write_text(str(w / 'epoch_1.pth'))
            launcher.validate_manifest(c, True)

    def test_check_command_never_contains_run(self):
        c = launcher.DEFAULTS
        command = launcher.train_command(c)
        self.assertIn('--wandb', command)
        self.assertNotIn('--run', command)
        self.assertNotIn('--resume', command)
        self.assertIn('--resume', launcher.train_command(c, True))

    def test_check_mode_starts_no_tmux(self):
        with patch.object(launcher.shutil, 'which', return_value='/usr/bin/tmux'), \
             patch.object(launcher, 'check_drive') as drive, \
             patch.object(launcher, 'logging_relay'), \
             patch.object(launcher, 'save'), \
             patch.object(launcher, 'ssh') as ssh, \
             patch.object(launcher, 'checked') as checked:
            launcher.start_local(launcher.DEFAULTS, check=True)
            drive.assert_called_once()
            checked.assert_not_called()
            self.assertIn('--remote-check', ssh.call_args.args[1])

    def test_start_orchestrates_both_sessions_after_preflight(self):
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(launcher, 'ROOT', Path(temp)), \
             patch.object(launcher.shutil, 'which', return_value='/usr/bin/tmux'), \
             patch.object(launcher, 'logging_relay'), \
             patch.object(launcher, 'check_drive') as drive, \
             patch.object(launcher, 'ssh') as ssh, \
             patch.object(launcher.subprocess, 'run', return_value=subprocess.CompletedProcess([], 1)), \
             patch.object(launcher.time, 'sleep'), \
             patch.object(launcher, 'checked') as checked:
            launcher.start_local(launcher.DEFAULTS)
            drive.assert_called_once()
            self.assertIn('--remote-check', ssh.call_args_list[0].args[1])
            self.assertIn('--remote-start', ssh.call_args_list[1].args[1])
            self.assertIn('new-session', checked.call_args_list[0].args[0])
            self.assertIn('has-session', checked.call_args_list[1].args[0])

    def test_failed_drive_preflight_starts_no_jobs(self):
        with patch.object(launcher.shutil, 'which', return_value='/usr/bin/tmux'), \
             patch.object(launcher, 'logging_relay'), \
             patch.object(launcher, 'check_drive', side_effect=ValueError('offline')), \
             patch.object(launcher, 'ssh') as ssh, \
             patch.object(launcher, 'checked') as checked:
            with self.assertRaises(ValueError):
                launcher.start_local(launcher.DEFAULTS)
            checked.assert_not_called()
            self.assertEqual(ssh.call_count, 1)
            self.assertIn('--remote-check', ssh.call_args.args[1])

    def test_duplicate_remote_training_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            with patch.object(launcher.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)), \
                 patch.object(launcher, 'check_remote') as check:
                with self.assertRaisesRegex(ValueError, 'already exists'):
                    launcher.start_remote(c)
                check.assert_not_called()

    def test_missing_backups_prevent_monitor_completion(self):
        statuses = [dict(state='finished', backup_errors=[], pending_checkpoints=['best.pth'], uploaded_count=1),
                    dict(state='finished', backup_errors=[], pending_checkpoints=[], uploaded_count=2)]
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(launcher, 'ROOT', Path(temp)), \
             patch.object(launcher.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)) as run, \
             patch.object(launcher, 'remote_status', side_effect=statuses), \
             patch.object(launcher.time, 'sleep') as sleep:
            self.assertEqual(launcher.backup_poll_worker(launcher.DEFAULTS), 0)
            self.assertEqual(run.call_count, 2)
            sleep.assert_called_once_with(120)

    def test_backup_failure_retries_without_training_control(self):
        results = [subprocess.CompletedProcess([], 1), subprocess.CompletedProcess([], 0)]
        status = dict(state='failed', backup_errors=[], pending_checkpoints=[], uploaded_count=1)
        with tempfile.TemporaryDirectory() as temp, \
             patch.object(launcher, 'ROOT', Path(temp)), \
             patch.object(launcher.subprocess, 'run', side_effect=results) as run, \
             patch.object(launcher, 'remote_status', return_value=status), \
             patch.object(launcher.time, 'sleep') as sleep:
            self.assertEqual(launcher.backup_poll_worker(launcher.DEFAULTS), 0)
            self.assertEqual(run.call_count, 2)
            sleep.assert_called_once_with(120)
            self.assertTrue(all('--backup-once' in call.args[0] for call in run.call_args_list))

    def test_remote_worker_preflight_failure_is_recorded_without_training(self):
        with tempfile.TemporaryDirectory() as temp:
            c = self.fixture(Path(temp))
            c['contract'] = 'bad'
            with patch.object(launcher.subprocess, 'Popen') as popen:
                self.assertEqual(launcher.remote_worker(c), 1)
                popen.assert_not_called()
            status = json.loads((Path(c['work']) / 'oneclick_status.json').read_text())
            self.assertEqual(status['state'], 'failed')
            self.assertEqual(status['error_type'], 'ValueError')


if __name__ == '__main__':
    unittest.main()
