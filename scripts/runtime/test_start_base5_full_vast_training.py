"""Exercise the shell dispatcher with a fake launcher Python, never SSH/GPU."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).with_name('start_base5_full_vast_training.sh')


class DispatcherTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.capture = self.root/'dispatched.json'
        self.config = self.root/'launcher.json'
        self.config.write_text(json.dumps(dict(schema='base5_performance_launcher_v1')))
        # Schema parsing uses the real interpreter; launch dispatch is captured.
        # No production Python entrypoint or remote transport is ever executed.
        fake = self.root/'python3'
        fake.write_text(f'#!{sys.executable}\n'
            'import json, os, sys\n'
            'from pathlib import Path\n'
            'args = sys.argv[1:]\n'
            'if args[:2] == ["-B", "-c"]:\n'
            f'    os.execv({sys.executable!r}, [{sys.executable!r}, *args])\n'
            'Path(os.environ["DISPATCH_CAPTURE"]).write_text(json.dumps(args))\n')
        fake.chmod(0o755)
        self.env = dict(os.environ, PATH=str(self.root)+os.pathsep+os.environ['PATH'],
                        DISPATCH_CAPTURE=str(self.capture),
                        BASE5_FULL_TRAINING_CONFIG=str(self.config))

    def call(self, *args):
        return subprocess.run(['bash', str(SCRIPT), *args], env=self.env,
                              text=True, capture_output=True, timeout=10)

    def dispatched(self):
        return json.loads(self.capture.read_text())

    def test_help_does_not_open_a_config_or_dispatch(self):
        self.config.unlink()
        for option in ('--help', '-h'):
            result = self.call(option)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('read-only remote status', result.stdout)
            self.assertFalse(self.capture.exists())

    def test_missing_config_or_path_argument_fails_closed(self):
        self.config.unlink()
        self.assertEqual(self.call().returncode, 2)
        self.assertEqual(self.call('--config').returncode, 2)
        self.assertFalse(self.capture.exists())

    def test_unknown_or_old5k_schema_has_no_fallback(self):
        for schema in ('unknown', 'base5_vast_training_v1'):
            self.config.write_text(json.dumps(dict(schema=schema)))
            result = self.call('--run')
            self.assertEqual(result.returncode, 2)
            self.assertIn('no fallback', result.stderr)
            self.assertFalse(self.capture.exists())

    def test_invalid_json_or_missing_schema_fails_closed(self):
        for content in ('broken', '{}', '[]', '{"schema": null}'):
            self.config.write_text(content)
            self.assertEqual(self.call('--run').returncode, 2)
            self.assertFalse(self.capture.exists())

    def test_new_schema_no_args_is_status_without_start_or_resume(self):
        result = self.call()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.dispatched()
        self.assertEqual(Path(args[1]).name, 'base5_performance_vast_training.py')
        self.assertEqual(args[2:], ['--config', str(self.config)])
        self.assertNotIn('--run', args)
        self.assertNotIn('--resume', args)

    def test_status_is_read_only(self):
        self.assertEqual(self.call('--status').returncode, 0)
        self.assertEqual(self.dispatched()[2:], ['--config', str(self.config)])

    def test_transition_and_later_child_resume_are_distinct(self):
        self.assertEqual(self.call('--run').returncode, 0)
        self.assertEqual(self.dispatched()[4:], ['--run'])
        self.assertEqual(self.call('--run', '--resume').returncode, 0)
        self.assertEqual(self.dispatched()[4:], ['--run', '--resume'])

    def test_explicit_old_full_schema_selects_frozen_entry(self):
        old = self.root/'old launcher.json'
        old.write_text(json.dumps(dict(schema='base5_full_data_launcher_v1')))
        result = self.call('--config', str(old), '--run', '--resume')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.dispatched()
        self.assertEqual(Path(args[1]).name, 'full_data_oneclick.py')
        self.assertEqual(args[2:], ['--config', str(old), '--run', '--resume'])

    def test_nonstart_actions_are_forwarded_without_run(self):
        for action in ('--check', '--prepare'):
            self.assertEqual(self.call(action).returncode, 0)
            self.assertEqual(self.dispatched()[4:], [action])

    def test_status_cannot_hide_a_start_action(self):
        self.assertEqual(self.call('--status', '--run').returncode, 2)
        self.assertFalse(self.capture.exists())

    def test_default_points_to_qualified_successor_and_shell_syntax(self):
        self.assertIn('base5_speed_repair_261021_v1/continuation_v2/launcher_speed_ready.json', SCRIPT.read_text())
        result = subprocess.run(['bash', '-n', str(SCRIPT)], text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__': unittest.main()
