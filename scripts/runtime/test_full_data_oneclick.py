import importlib.util
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('oneclick', HERE/'full_data_oneclick.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def config(self):
        return dict(schema='base5_full_data_launcher_v1', expected_scenes=30750, variant='cosine',
            source='/workspace/oldsource', data='/workspace/new/index.json', work='/workspace/new/run',
            initializer='/workspace/init.pth', remote_script='/workspace/new/full_data_oneclick.py',
            remote_config='/workspace/new/config.json', prepare_script='/workspace/new/full_data_prepare.py',
            epochs=8, seed=261015, compile_camera=False, epochs_user_approved=False,
            start_authority='/workspace/new/USER_START_AUTHORITY.json',
            val_logging_script='/workspace/new/base5_val_logging.py',
            val_logging_schema='base5_val_logging_v1',val_logging_sha256='a'*64)

    def test_draft_run_rejected(self):
        with self.assertRaisesRegex(ValueError, 'still draft'):
            m.command(self.config(), run=True)

    def test_epoch_only_not_start_permission(self):
        c = self.config()
        c['epochs_user_approved'] = True
        with self.assertRaisesRegex(ValueError, 'No separate'):
            m.command(c, run=True)

    def test_prepare_is_never_run(self):
        self.assertNotIn('--run', m.command(self.config()))

    def test_resume_keeps_original_epoch_boundary_mode(self):
        self.assertEqual(m.command(self.config(), resume=True)[-2:], ['--resume', 'auto'])

    def test_three_class_supplement_count_only_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.json'
            c = self.config()
            path.write_text(json.dumps(c))
            self.assertEqual(m.load_config(path), c)
            c['expected_scenes'] = 30000
            path.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError, 'explicit full-data'):
                m.load_config(path)

    def test_incomplete_data_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'index.json'
            path.write_text(json.dumps(dict(schema='base5_full_cohort_index_v1', complete=False)))
            c = self.config()
            c.update(data=str(path), dataset_manifest_sha256=m.sha(path))
            with self.assertRaisesRegex(ValueError, 'not ready'):
                m.data_gate(c)

    def test_old_30749_configuration_is_not_reused_implicitly(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.json'
            c = self.config()
            c['expected_scenes'] = 30749
            path.write_text(json.dumps(c))
            with self.assertRaisesRegex(ValueError, 'explicit full-data'):
                m.load_config(path)

    def test_unbound_or_external_val_logging_cannot_reuse_this_entry(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.json'
            for change in ({'val_logging_schema':'unreviewed'},
                           {'val_logging_sha256':''},
                           {'val_logging_script':'/outside/base5_val_logging.py'}):
                c=self.config();c.update(change)
                path.write_text(json.dumps(c))
                with self.assertRaisesRegex(ValueError,'validation-logging'):
                    m.load_config(path)

    def test_direct_backup_prefix_only_changes_proxy_environment(self):
        with patch.dict(os.environ, dict(HTTP_PROXY='proxy', HTTPS_PROXY='proxy', ALL_PROXY='proxy',
            http_proxy='proxy', https_proxy='proxy', all_proxy='proxy', UNRELATED='keep')):
            exec(m.direct_backup_prefix(), {})
            for key in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
                self.assertNotIn(key, os.environ)
            self.assertEqual(os.environ['UNRELATED'], 'keep')

    def test_direct_backup_requires_explicit_route(self):
        with self.assertRaisesRegex(ValueError, 'direct backup route'):
            m.backup_once(dict(drive_backup_route='relay'), None)

    def test_only_setup_checked_deadline_scales(self):
        original = Mock()
        transport = SimpleNamespace(checked=original, ssh=Mock())
        prepare = '/workspace/runtime/full_data_prepare.py'
        m.preparation_timeouts(transport, prepare)
        transport.checked(['python', '-u', prepare], timeout=180)
        self.assertEqual(original.call_args.kwargs['timeout'], 900)
        transport.checked(['python', '-u', prepare, '--run'], timeout=180)
        self.assertEqual(original.call_args.kwargs['timeout'], 180)
        transport.checked(['rclone', 'copy'], timeout=1200)
        self.assertEqual(original.call_args.kwargs['timeout'], 1200)

    def test_only_setup_ssh_deadline_scales(self):
        original = Mock()
        transport = SimpleNamespace(checked=Mock(), ssh=original)
        m.preparation_timeouts(transport, '/workspace/runtime/full_data_prepare.py')
        for action in ('--remote-prepare', '--remote-check', '--remote-start'):
            transport.ssh({}, 'python /runtime/main.py '+action, timeout=240)
            self.assertEqual(original.call_args.kwargs['timeout'], 1200)
        transport.ssh({}, 'python /runtime/main.py --remote-status', timeout=240)
        self.assertEqual(original.call_args.kwargs['timeout'], 240)
        transport.ssh({}, 'python -c "print(123)"', timeout=60)
        self.assertEqual(original.call_args.kwargs['timeout'], 60)


if __name__ == '__main__':
    unittest.main()
