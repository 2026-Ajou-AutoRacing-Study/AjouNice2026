"""Readiness-only fake-clock tests: no Docker, GPU, ROS or wall-clock sleeps."""
from pathlib import Path
import subprocess
import unittest


RUNNER = Path(__file__).with_name('run_bevfusion.sh')


class ReadinessTests(unittest.TestCase):
    def run_wait(self, timeout, failures):
        source = RUNNER.read_text()
        function = source.split('wait_for_selected_server() {', 1)[1].split('\ncleanup()', 1)[0]
        script = '''set -eu
SECONDS=0
ready=false
attempt=0
sleep() { SECONDS=$((SECONDS + 2)); }
check_server_identity() {
  attempt=$((attempt + 1))
  (( attempt > FAILURES ))
}
'''
        script += 'FAILURES=' + str(failures) + '\n'
        script += 'BEVFUSION_READY_TIMEOUT_SEC=' + timeout + '\n'
        script += 'wait_for_selected_server() {' + function
        script += '\nwait_for_selected_server\n'
        return subprocess.run(['bash', '-c', script], text=True, capture_output=True, timeout=5)

    def test_slow_loading_exceeds_old_two_minutes(self):
        result = self.run_wait('300', 80)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Model ready after 160s', result.stdout)

    def test_explicit_unlimited_still_requires_ready(self):
        result = self.run_wait('0', 200)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('no deadline', result.stdout)
        self.assertIn('Model ready after 400s', result.stdout)

    def test_deadline_fails_without_claiming_ready(self):
        result = self.run_wait('10', 200)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('timed out after 10s', result.stderr)
        self.assertNotIn('Model ready after', result.stdout)

    def test_bad_optional_setting_uses_default(self):
        result = self.run_wait('bad', 1)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('using 300 seconds', result.stderr)

    def test_zero_padded_timeout_is_decimal(self):
        result = self.run_wait('000008', 200)
        self.assertIn('timed out after 8s', result.stderr)


if __name__ == '__main__':
    unittest.main()
