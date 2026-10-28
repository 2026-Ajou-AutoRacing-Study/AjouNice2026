"""Black-box checks for the field console; no ROS master or vehicle required.

These tests exercise only processes created by the test.  They deliberately do
not require the displayed wording to be stable: lossless evidence, retained
errors, bounded output, and subprocess lifecycle are the deployment contracts.
"""

import contextlib
import errno
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().with_name('field_console.py')
KOREAN = re.compile('[가-힣]')
ERROR = re.compile(r'오류|실패|ERROR|FATAL|예외', re.IGNORECASE)


def process_alive(pid):
    """A reaped or zombie child is not running vehicle/application work."""
    try:
        return Path('/proc/{}/stat'.format(pid)).read_text().split()[2] != 'Z'
    except (FileNotFoundError, ProcessLookupError):
        return False


class FieldConsoleCLI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='field-console-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.logs = self.root / 'logs'

    def command(self, mode='run', log_dir=None, *extra):
        return [sys.executable, str(SCRIPT), mode, '--component', 'test',
                '--log-dir', str(self.logs if log_dir is None else log_dir),
                '--interval', '0.1'] + list(extra)

    def run_child(self, code, **kwargs):
        return subprocess.run(self.command('run', None, '--', sys.executable,
                                          '-u', '-c', code),
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=8, **kwargs)

    def screen(self, result):
        return (result.stdout + result.stderr).decode('utf-8', errors='replace')

    def raw_logs(self):
        return sorted(self.logs.glob('*_raw.log'))

    def test_run_keeps_complete_stdout_and_stderr_evidence(self):
        result = self.run_child("import os; os.write(1,b'OUT-one\\n'); "
                                "os.write(2,b'ERR-two\\n'); os.write(1,b'partial')")
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertEqual(len(self.raw_logs()), 1)
        evidence = self.raw_logs()[0].read_bytes()
        self.assertIn(b'OUT-one\n', evidence)
        self.assertIn(b'ERR-two\n', evidence)
        self.assertTrue(evidence.endswith(b'partial'), evidence)
        self.assertRegex(self.screen(result), KOREAN)

    def test_filter_flushes_short_lived_error_at_eof(self):
        payload = b'[ERROR] one-off failure\n[INFO] next ordinary line\n'
        result = subprocess.run(self.command('filter'), input=payload,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=5)
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertEqual(self.raw_logs()[0].read_bytes(), payload)
        self.assertRegex(self.screen(result), ERROR)

    def test_invalid_utf8_and_a_trailing_fragment_remain_lossless(self):
        payload = b'[WARN] vendor byte: \xff\xfe\nlast fragment without newline'
        result = subprocess.run(self.command('filter'), input=payload,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=5)
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertEqual(self.raw_logs()[0].read_bytes(), payload)
        self.assertRegex(self.screen(result), KOREAN)

    def test_nonzero_child_exit_is_not_masked(self):
        result = self.run_child('import sys; print("before failure"); sys.exit(7)')
        self.assertEqual(result.returncode, 7, self.screen(result))
        self.assertRegex(self.screen(result), ERROR)
        self.assertIn(b'before failure', self.raw_logs()[0].read_bytes())

    def test_traceback_is_retained_when_a_later_info_line_arrives(self):
        result = self.run_child(
            'import sys,time; '
            'sys.stderr.write("Traceback (most recent call last):\\n"); '
            'sys.stderr.write("RuntimeError: synthetic regression\\n"); '
            'sys.stderr.flush(); print("[INFO] still here",flush=True); time.sleep(.23)')
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertRegex(self.screen(result), ERROR)
        self.assertIn(b'Traceback', self.raw_logs()[0].read_bytes())
        self.assertIn(b'RuntimeError', self.raw_logs()[0].read_bytes())

    def test_missing_executable_is_visible_and_nonzero(self):
        result = subprocess.run(self.command('run', None, '--',
                                             str(self.root / 'does-not-exist')),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertRegex(self.screen(result), ERROR)

    def test_log_path_failure_prevents_starting_child(self):
        invalid = self.root / 'not-a-directory'
        invalid.write_text('occupied', encoding='utf-8')
        marker = self.root / 'must-not-be-created'
        code = 'from pathlib import Path; Path({!r}).touch()'.format(str(marker))
        result = subprocess.run(self.command('run', invalid, '--', sys.executable,
                                             '-c', code),
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(marker.exists(), 'child ran without raw-log evidence')
        self.assertRegex(self.screen(result), ERROR)

    def test_run_names_do_not_overwrite_previous_raw_evidence(self):
        first = self.run_child('print("first unique evidence")')
        second = self.run_child('print("second unique evidence")')
        self.assertEqual(first.returncode, 0, self.screen(first))
        self.assertEqual(second.returncode, 0, self.screen(second))
        self.assertEqual(len(self.raw_logs()), 2)
        data = {p.read_bytes() for p in self.raw_logs()}
        self.assertEqual(data, {b'first unique evidence\n', b'second unique evidence\n'})

    def test_repeated_diagnostics_are_aggregated_without_losing_raw_events(self):
        result = self.run_child(
            'import time; '
            '[(print("[WARN] TF lookup failed",flush=True), time.sleep(.002)) '
            'for _ in range(200)]')
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertEqual(self.raw_logs()[0].read_bytes().count(b'TF lookup failed'), 200)
        self.assertLess(len(self.screen(result).splitlines()), 35,
                        'display is still emitting per-message diagnostics')

    def test_terminal_controls_are_preserved_raw_but_cannot_escape_on_screen(self):
        payload = b'[ERROR] unsafe\x1b[2J\rRESET\x00\x1b]0;bad-title\x07\n'
        result = subprocess.run(self.command('filter'), input=payload,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=5)
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertEqual(self.raw_logs()[0].read_bytes(), payload)
        self.assertNotIn(b'\x00', result.stdout + result.stderr)
        self.assertNotIn(b'\x07', result.stdout + result.stderr)
        self.assertNotIn(b'\x1b[2J', result.stdout + result.stderr)
        self.assertNotIn(b'\x1b]0;', result.stdout + result.stderr)

    def test_unrecognized_and_silent_input_does_not_claim_observed_health(self):
        for code in ('print("opaque vendor heartbeat")', 'import time; time.sleep(.25)'):
            with self.subTest(code=code):
                result = self.run_child(code)
                self.assertEqual(result.returncode, 0, self.screen(result))
                screen = self.screen(result)
                self.assertRegex(screen, KOREAN)
                # A zero exit status can be reported as a normal *process exit*;
                # it must not become a claim that the unseen ROS stack is healthy.
                self.assertNotRegex(screen, r'스택\s*정상|센서\s*정상|통신\s*정상|주행\s*정상|상태\s*정상')

    def test_demo_needs_no_ros_and_writes_lossless_evidence(self):
        result = subprocess.run([sys.executable, str(SCRIPT), 'demo',
                                 '--log-dir', str(self.logs)],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=10)
        self.assertEqual(result.returncode, 0, self.screen(result))
        self.assertRegex(self.screen(result), KOREAN)
        self.assertTrue(self.raw_logs(), 'demo must also demonstrate where evidence is retained')
        self.assertTrue(any(path.stat().st_size for path in self.raw_logs()))

    def test_nonfinite_or_zero_interval_cannot_start_a_child(self):
        marker = self.root / 'invalid-config-child'
        code = 'from pathlib import Path; Path({!r}).touch()'.format(str(marker))
        for value in ('nan', 'inf', '0'):
            with self.subTest(interval=value):
                result = subprocess.run(
                    self.command('run', None, '--interval', value, '--',
                                 sys.executable, '-c', code),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(marker.exists())

    @unittest.skipUnless(os.name == 'posix' and Path('/proc').exists(), 'Linux shutdown escalation')
    def test_unresponsive_managed_child_is_killed_after_grace_timeout(self):
        ready = self.root / 'ignoring-signal-pid'
        code = '\n'.join([
            'import os,signal,time',
            'from pathlib import Path',
            'signal.signal(signal.SIGTERM,signal.SIG_IGN)',
            'signal.signal(signal.SIGINT,signal.SIG_IGN)',
            'Path({!r}).write_text(str(os.getpid()))'.format(str(ready)),
            'time.sleep(30)',
        ])
        wrapper = subprocess.Popen(
            self.command('run', None, '--stop-timeout', '.2', '--',
                         sys.executable, '-u', '-c', code),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        child_pid = None
        try:
            deadline = time.monotonic() + 4
            while not ready.exists() and wrapper.poll() is None and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue(ready.exists(), 'managed child never reached readiness')
            child_pid = int(ready.read_text())
            before_stop = time.monotonic()
            wrapper.send_signal(signal.SIGTERM)
            wrapper.communicate(timeout=3)
            self.assertLess(time.monotonic() - before_stop, 2.5)
            self.assertFalse(process_alive(child_pid), 'unresponsive managed child is still running')
            self.assertNotEqual(wrapper.returncode, 0, 'forced kill was reported as successful completion')
        finally:
            if wrapper.poll() is None:
                wrapper.kill()
                wrapper.communicate(timeout=2)
            if child_pid is not None and process_alive(child_pid):
                try:
                    os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    @unittest.skipUnless(os.name == 'posix' and Path('/proc').exists(), 'Linux process-group contract')
    def test_signals_forward_to_only_the_managed_process_group_and_wait_for_cleanup(self):
        for requested_signal in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=requested_signal):
                self._signal_trial(requested_signal)

    def _signal_trial(self, requested_signal):
        ready = self.root / ('ready-' + str(requested_signal))
        stopped = self.root / ('stopped-' + str(requested_signal))
        grand_stopped = self.root / ('grand-stopped-' + str(requested_signal))
        grand_code = (
            'import signal,time; from pathlib import Path; '
            'signal.signal(signal.SIGINT,lambda s,f:(Path({!r}).write_text(str(s)),exit(0))); '
            'signal.signal(signal.SIGTERM,lambda s,f:(Path({!r}).write_text(str(s)),exit(0))); '
            'time.sleep(30)'
        ).format(str(grand_stopped), str(grand_stopped))
        code = '\n'.join([
            'import os,signal,subprocess,sys,time',
            'from pathlib import Path',
            'child=subprocess.Popen([sys.executable,"-u","-c",{!r}])'.format(grand_code),
            'def stop(signum,frame):',
            '    child.wait(timeout=3)',
            '    Path({!r}).write_text(str(signum))'.format(str(stopped)),
            '    sys.exit(0)',
            'signal.signal(signal.SIGINT,stop)',
            'signal.signal(signal.SIGTERM,stop)',
            'time.sleep(.15)',
            'Path({!r}).write_text(str(os.getpid())+" "+str(child.pid))'.format(str(ready)),
            'print("[INFO] synthetic child ready",flush=True)',
            'time.sleep(30)',
        ])
        sentinel = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        wrapper = subprocess.Popen(
            self.command('run', None, '--', sys.executable, '-u', '-c', code),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        managed_pids = []
        try:
            deadline = time.monotonic() + 4
            while not ready.exists() and wrapper.poll() is None and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue(ready.exists(), 'managed child never reached readiness')
            managed_pids = [int(pid) for pid in ready.read_text().split()]
            wrapper.send_signal(requested_signal)
            output, error = wrapper.communicate(timeout=6)
            screen = (output + error).decode('utf-8', errors='replace')
            self.assertIsNone(sentinel.poll(), 'wrapper signalled an unrelated process')
            self.assertTrue(stopped.exists(), 'wrapper did not wait for child cleanup: ' + screen)
            self.assertTrue(grand_stopped.exists(), 'signal did not reach the managed process group')
            forwarded_signal = signal.SIGINT if requested_signal == signal.SIGHUP else requested_signal
            self.assertEqual(stopped.read_text(), str(int(forwarded_signal)))
            self.assertEqual(grand_stopped.read_text(), str(int(forwarded_signal)))
            self.assertFalse(any(process_alive(pid) for pid in managed_pids))
        finally:
            if wrapper.poll() is None:
                wrapper.kill()
                wrapper.communicate(timeout=2)
            for pid in managed_pids:
                if process_alive(pid):
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            sentinel.terminate()
            sentinel.wait(timeout=2)

    @unittest.skipUnless(os.name == 'posix' and Path('/proc').exists(), 'Linux pipe ownership')
    def test_direct_child_exit_is_not_hidden_by_a_grandchild_holding_stdout(self):
        ready = self.root / 'orphan-grandchild-ready'
        grand_code = '\n'.join([
            'import os,signal,time',
            'from pathlib import Path',
            'signal.signal(signal.SIGTERM,signal.SIG_IGN)',
            'Path({!r}).write_text(str(os.getpid()))'.format(str(ready)),
            'time.sleep(30)',
        ])
        child_code = '\n'.join([
            'import subprocess,sys,time',
            'from pathlib import Path',
            'subprocess.Popen([sys.executable,"-u","-c",{!r}])'.format(grand_code),
            'while not Path({!r}).exists(): time.sleep(.01)'.format(str(ready)),
            'sys.exit(7)',
        ])
        sentinel = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        wrapper = subprocess.Popen(
            self.command('run', None, '--stop-timeout', '.2', '--',
                         sys.executable, '-u', '-c', child_code),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        grand_pid = None
        try:
            before = time.monotonic()
            stdout, stderr = wrapper.communicate(timeout=3)
            self.assertLess(time.monotonic() - before, 2.5)
            self.assertEqual(wrapper.returncode, 7, (stdout + stderr).decode(errors='replace'))
            self.assertTrue(ready.exists())
            grand_pid = int(ready.read_text())
            self.assertFalse(process_alive(grand_pid), 'grandchild still holds the managed output pipe')
            self.assertIsNone(sentinel.poll(), 'orphan cleanup reached an unrelated process')
        finally:
            if wrapper.poll() is None:
                wrapper.kill()
                wrapper.communicate(timeout=2)
            if grand_pid is None and ready.exists():
                grand_pid = int(ready.read_text())
            if grand_pid is not None and process_alive(grand_pid):
                try:
                    os.kill(grand_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            sentinel.terminate()
            sentinel.wait(timeout=2)

    @unittest.skipUnless(os.name == 'posix' and Path('/proc').exists(), 'Linux pipe closure')
    def test_child_can_close_stdout_and_still_receive_orderly_shutdown(self):
        ready = self.root / 'closed-stdout-ready'
        stopped = self.root / 'closed-stdout-stopped'
        code = '\n'.join([
            'import os,signal,sys,time',
            'from pathlib import Path',
            'def stop(signum,frame):',
            '    Path({!r}).write_text(str(signum))'.format(str(stopped)),
            '    sys.exit(0)',
            'signal.signal(signal.SIGINT,stop)',
            'os.close(1)',
            'os.close(2)',
            'Path({!r}).write_text(str(os.getpid()))'.format(str(ready)),
            'time.sleep(30)',
        ])
        wrapper = subprocess.Popen(
            self.command('run', None, '--stop-timeout', '.2', '--',
                         sys.executable, '-u', '-c', code),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
        child_pid = None
        try:
            deadline = time.monotonic() + 4
            while not ready.exists() and wrapper.poll() is None and time.monotonic() < deadline:
                time.sleep(.02)
            self.assertTrue(ready.exists(), 'managed child never reached readiness')
            child_pid = int(ready.read_text())
            time.sleep(.25)  # Reader has observed EOF, but application remains alive.
            self.assertIsNone(wrapper.poll())
            wrapper.send_signal(signal.SIGINT)
            stdout, stderr = wrapper.communicate(timeout=3)
            self.assertEqual(wrapper.returncode, 0, (stdout + stderr).decode(errors='replace'))
            self.assertEqual(stopped.read_text(), str(int(signal.SIGINT)))
            self.assertFalse(process_alive(child_pid))
        finally:
            if wrapper.poll() is None:
                wrapper.kill()
                wrapper.communicate(timeout=2)
            if child_pid is not None and process_alive(child_pid):
                try:
                    os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


class FieldConsoleFailureIsolation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('field_console_failure_tests', SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.module
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='field-console-failure-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def arguments(self, code):
        return SimpleNamespace(mode='run', command=[sys.executable, '-u', '-c', code],
                               log_dir=str(self.root / 'logs'), component='test',
                               width=120, interval=.1, stop_timeout=.3)

    def delayed_marker(self, marker, exit_code=0):
        return '\n'.join([
            'import sys,time',
            'from pathlib import Path',
            'print("[INFO] child started",flush=True)',
            'time.sleep(.25)',
            'Path({!r}).write_text("child continued after failure")'.format(str(marker)),
            'print("[INFO] child finished",flush=True)',
            'sys.exit({})'.format(exit_code),
        ])

    def test_midrun_raw_write_or_flush_failure_does_not_stop_the_application(self):
        class FullFile:
            def __init__(self, failing_operation):
                self.failing_operation = failing_operation
                self.closed = False

            def write(self, data):
                if self.failing_operation == 'write':
                    raise OSError(errno.ENOSPC, 'No space left on device')
                return len(data)

            def flush(self):
                if self.failing_operation == 'flush':
                    raise OSError(errno.ENOSPC, 'No space left on device')

            def close(self):
                self.closed = True

        for operation in ('write', 'flush'):
            for exit_code in (0, 7):
                with self.subTest(operation=operation, exit_code=exit_code):
                    marker = self.root / ('continued-{}-{}'.format(operation, exit_code))
                    handle = FullFile(operation)
                    output, error = io.StringIO(), io.StringIO()
                    with patch.object(self.module, 'new_log', return_value=(self.root / 'raw.log', handle)):
                        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
                            result = self.module.run(self.arguments(self.delayed_marker(marker, exit_code)))
                    self.assertTrue(marker.exists(), 'raw logging failure stopped the application')
                    self.assertEqual(result, 2 if exit_code == 0 else exit_code)
                    self.assertIn('원본 로그 저장 실패', output.getvalue() + error.getvalue())
                    self.assertTrue(handle.closed)

    def test_broken_stdout_uses_stderr_without_stopping_the_application(self):
        class ClosedDisplay:
            def write(self, data):
                raise BrokenPipeError(errno.EPIPE, 'closed test display')

            def flush(self):
                raise BrokenPipeError(errno.EPIPE, 'closed test display')

        marker = self.root / 'continued-after-display-failure'
        error = io.StringIO()
        with contextlib.redirect_stdout(ClosedDisplay()), contextlib.redirect_stderr(error):
            result = self.module.run(self.arguments(self.delayed_marker(marker)))
        self.assertEqual(result, 0)
        self.assertTrue(marker.exists())
        self.assertRegex(error.getvalue(), KOREAN)
        raw = list((self.root / 'logs').glob('*_raw.log'))
        self.assertEqual(len(raw), 1)
        self.assertIn(b'child finished', raw[0].read_bytes())

    def test_both_displays_can_fail_without_stopping_application_or_raw_capture(self):
        class ClosedDisplay:
            def write(self, data):
                raise OSError(errno.EIO, 'closed test terminal')

            def flush(self):
                raise OSError(errno.EIO, 'closed test terminal')

        marker = self.root / 'continued-without-display'
        with contextlib.redirect_stdout(ClosedDisplay()), contextlib.redirect_stderr(ClosedDisplay()):
            result = self.module.run(self.arguments(self.delayed_marker(marker)))
        self.assertEqual(result, 0)
        self.assertTrue(marker.exists())
        self.assertIn(b'child finished', next((self.root / 'logs').glob('*_raw.log')).read_bytes())


class FieldConsoleClassification(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('field_console_classification_tests', SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.module
        spec.loader.exec_module(cls.module)

    def test_roslaunch_parameter_names_are_not_python_exceptions(self):
        for line in (' * /controller/max_lat_error: 5.0',
                     '* /planner/error: 1.0', 'max_lat_error: 5.0',
                     'threshold_error: false'):
            with self.subTest(line=line):
                self.assertNotEqual(self.module.classify(line, 'planning').level, 'ERROR')

    def test_python_and_qualified_ros_exceptions_remain_errors(self):
        for line in ('RuntimeError: bad state', 'ValueError: bad parameter',
                     'rospy.exceptions.ROSInterruptException: rospy shutdown',
                     'Traceback (most recent call last):'):
            with self.subTest(line=line):
                self.assertEqual(self.module.classify(line, 'planning').level, 'ERROR')

    def test_failed_inference_or_preprocessing_is_not_downgraded_by_warn_prefix(self):
        for line in ('[WARN] BEVFusion request failed: timeout',
                     '[WARN] BEVFusion preprocessing failed: invalid array'):
            with self.subTest(line=line):
                self.assertEqual(self.module.classify(line, 'bevfusion').level, 'ERROR')

    def test_expected_ego_disabled_configuration_is_not_an_error(self):
        event = self.module.classify('[WARN] Skipping Ego Vehicle Status UDP receiver (type 770) by configuration', 'udp')
        self.assertEqual(event.level, 'INFO')
        self.assertEqual(event.code, 'ego_off')

    def test_competition_signal_failopen_is_not_reported_as_green_observation(self):
        event = self.module.classify('[WARN] missing/stale signal: competition fail-open GO', 'planning')
        self.assertEqual(event.level, 'WARN')
        self.assertEqual(event.code, 'signal_bypass')
        self.assertIn('녹색 관측을 뜻하지 않음', self.module.event_text(event))

    def test_vehicle_only_corridor_failure_is_not_reported_as_expected_configuration(self):
        event = self.module.classify('[WARN] vehicle-only corridor FAIL-OPEN: state missing', 'tracking')
        self.assertEqual(event.level, 'ERROR')
        self.assertEqual(event.code, 'corridor_failed')

    def test_recorded_mpt_failure_signatures_distinguish_braking_and_overlap(self):
        rows = [
            ('[WARN] [MPT] Optimization failed; braking on previous path (clear=59.20 stop=58.20)',
             'mpt_braking', 'WARN', '59.20m'),
            ('[WARN] [MPT] Measured footprint already overlaps static obstacle clearance; stopping',
             'mpt_overlap', 'ERROR', '실제 충돌 확정 아님'),
            ('[WARN] [MPT] Optimization failed; emergency brake, previous path clear distance 0.00 < 1.06',
             'mpt_emergency', 'ERROR', '0.00m'),
        ]
        for line, code, level, detail in rows:
            with self.subTest(code=code):
                event = self.module.classify(line, 'planning')
                self.assertEqual((event.code, event.level), (code, level))
                self.assertIn(detail, self.module.event_text(event))

    def test_real_error_severity_is_not_demoted_by_mpt_braking_rule(self):
        event = self.module.classify(
            '[ERROR] [MPT] Optimization failed; braking on previous path (clear=2.0 stop=1.0)',
            'planning')
        self.assertEqual(event.level, 'ERROR')


class FieldTerminalAliasCLI(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='field-terminal-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.repo = SCRIPT.parents[2]
        self.script = SCRIPT.with_name('field_terminal.sh')
        self.argv = self.root / 'mock-roslaunch-argv.json'
        self.mock = self.root / 'mock-roslaunch'
        self.mock.write_text(
            '#!{}\nimport json,os,sys\n'.format(sys.executable) +
            'from pathlib import Path\n' +
            'Path(os.environ["MOCK_ARGV"]).write_text(json.dumps(sys.argv[1:]))\n' +
            'print("[INFO] mocked roslaunch, no ROS was started",flush=True)\n' +
            'sys.exit(int(os.environ.get("MOCK_EXIT","0")))\n', encoding='utf-8')
        self.mock.chmod(0o700)
        self.log_dir = self.root / 'field-logs'
        self.log_dir.mkdir()
        pointer = self.root / 'log-pointer'
        pointer.write_text(str(self.log_dir / 'test-session') + '\n', encoding='utf-8')
        self.environment = dict(os.environ)
        self.environment.update({
            'AJOUNICE_ROOT': str(self.repo),
            'AJOUNICE_FIELD_LOG_DIR': str(self.log_dir),
            'AJOUNICE_FIELD_LOG_POINTER': str(pointer),
            'AJOUNICE_FIELD_LOG_INTERVAL': '.1',
            'MOCK_ARGV': str(self.argv),
            'MOCK_ROSLAUNCH': str(self.mock),
            # The real overlay may prepend /opt/ros/bin to PATH.  This exported
            # shell function has priority over PATH, guaranteeing no real ROS.
            'BASH_FUNC_roslaunch%%': '() { "$MOCK_ROSLAUNCH" "$@"; }',
        })

    def invoke(self, *arguments):
        return subprocess.run(['bash', str(self.script)] + list(arguments),
                              env=self.environment, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=10)

    def test_actual_alias_expansion_preserves_quoted_arguments_without_executing_them(self):
        injection = self.root / 'injection-must-not-run'
        arguments = ['text:=a b', 'quote:=a"b\'c',
                     'literal:=$(touch {})'.format(injection), 'semi:=x;y']
        result = self.invoke('planning', *arguments)
        self.assertEqual(result.returncode, 0, (result.stdout + result.stderr).decode(errors='replace'))
        self.assertEqual(json.loads(self.argv.read_text()),
                         ['ajounice_planning_integration', 'competition_planning.launch'] + arguments)
        self.assertFalse(injection.exists(), 'an argument was interpreted as shell code')
        self.assertEqual(len(list(self.log_dir.glob('*_console/*_raw.log'))), 1)

    def test_shell_wrapper_preserves_mocked_roslaunch_failure_status(self):
        self.environment['MOCK_EXIT'] = '17'
        result = self.invoke('traffic', 'example:=value')
        self.assertEqual(result.returncode, 17, (result.stdout + result.stderr).decode(errors='replace'))
        self.assertEqual(json.loads(self.argv.read_text()),
                         ['traffic_light_detection', 'traffic_light_node.launch', 'example:=value'])

    def test_unsupported_entry_does_not_start_the_ros_mock(self):
        result = self.invoke('not-a-supported-command')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv.exists())


if __name__ == '__main__':
    unittest.main()
