"""Persist startup failures without starting or killing ROS/MORAI processes."""
import json
import io
import io
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch, Mock

import yaml

from planning_scenario_validator import gui, runner, runtime, startup_audit
from planning_scenario_validator.config import ROOT, load_campaign


EXAMPLE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/localization_baseline.yaml'


class StartupFailureAuditTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.cfg = {'output': self.root, 'name': 'startup-fixture'}

    def read(self, name):
        return json.loads((self.root / name).read_text())

    def test_console_disk_failure_does_not_stop_reading_child_output(self):
        process = Mock(stdout=io.StringIO('first\nsecond\n'))
        handle = Mock()
        handle.write.side_effect = OSError('fixture full disk')
        handle.close.side_effect = OSError('fixture flush error')
        received = []
        with patch.object(Path, 'open', return_value=handle):
            startup_audit.stream_console(process, received.append, self.root / 'console.log')
        self.assertIn('first', received)
        self.assertIn('second', received)

    def assert_timestamp(self, report):
        self.assertTrue(any(report.get(key) for key in
                            ('recorded_at', 'updated_at', 'time', 'timestamp', 'failed_at')),
                        report)

    def test_error_phase_type_message_and_traceback_are_persisted(self):
        startup_audit.mark_startup_phase(self.cfg, 'ROS_MASTER_CHECK')
        self.assertEqual(self.read('startup_status.json')['phase'], 'ROS_MASTER_CHECK')
        try:
            raise runtime.RuntimeFailure('fixture existing /Ego_topic publisher')
        except runtime.RuntimeFailure as error:
            startup_audit.record_startup_failure(self.cfg, error)
        report = self.read('startup_error.json')
        self.assertEqual(report['phase'], 'ROS_MASTER_CHECK')
        self.assertEqual(report['error_type'], 'RuntimeFailure')
        self.assertIn('/Ego_topic', report['error'])
        self.assert_timestamp(report)
        text = (self.root / 'stack_logs/startup_error.log').read_text()
        self.assertIn('Traceback', text)
        self.assertIn('RuntimeFailure', text)
        self.assertIn('/Ego_topic', text)

    def test_error_does_not_erase_previous_case_results(self):
        previous = {'schema_version': 2, 'run_status': 'INTERRUPTED',
                    'results': {'previous': {'status': 'PASS', 'reason': 'SUCCESS'}},
                    'current': None}
        (self.root / 'campaign_state.json').write_text(json.dumps(previous))
        startup_audit.mark_startup_phase(self.cfg, 'STACK_START')
        try:
            raise runtime.RuntimeFailure('fixture UDP startup failed')
        except runtime.RuntimeFailure as error:
            startup_audit.record_startup_failure(self.cfg, error)
        state = self.read('campaign_state.json')
        self.assertEqual(state['results'], previous['results'])
        self.assertEqual(state['startup_error']['phase'], 'STACK_START')
        self.assertIn('UDP startup failed', state['startup_error']['error'])

    def test_successful_completion_records_exit_code_without_error_file(self):
        startup_audit.mark_startup_phase(self.cfg, 'CASE_RUN')
        startup_audit.complete_startup(self.cfg, 0)
        report = self.read('startup_status.json')
        self.assertEqual(report['phase'], 'CAMPAIGN_FINISHED')
        self.assertEqual(report['exit_code'], 0)
        self.assert_timestamp(report)
        self.assertFalse((self.root / 'startup_error.json').exists())

    def test_console_stream_is_saved_verbatim_and_delivered_to_gui_callback(self):
        text = 'fixture startup\n실행 준비 오류: fixture conflict\nlast line without newline'
        process = SimpleNamespace(stdout=io.StringIO(text))
        lines = []
        logfile = self.root / 'stack_logs/runner_console.log'
        startup_audit.stream_console(process, lines.append, logfile)
        self.assertEqual(logfile.read_text(), text)
        self.assertEqual(lines, ['fixture startup', '실행 준비 오류: fixture conflict',
                                 'last line without newline'])

    def test_gui_completion_shows_error_and_phase_instead_of_exit_code_alone(self):
        state = {'results': {}, 'startup_error': {
            'phase': 'ROS_MASTER_CHECK', 'error': 'fixture duplicate /Ego_topic publisher'}}
        label = gui.completion_label(2, state)
        self.assertIn('ROS_MASTER_CHECK', label)
        self.assertIn('duplicate /Ego_topic', label)
        self.assertIn('startup_error.log', label)
        self.assertIn('2', label)

    def test_completed_retry_clears_active_error_but_keeps_failure_evidence_and_case_results(self):
        previous = {'run_status': 'INTERRUPTED',
                    'results': {'previous': {'status': 'PASS', 'reason': 'SUCCESS'}}}
        state_path = self.root / 'campaign_state.json'
        state_path.write_text(json.dumps(previous))
        startup_audit.mark_startup_phase(self.cfg, 'ROS_MASTER_CHECK')
        try:
            raise runtime.RuntimeFailure('fixture failed previous attempt')
        except runtime.RuntimeFailure as error:
            startup_audit.record_startup_failure(self.cfg, error)
        history = (self.root / 'startup_error.json').read_bytes()
        self.assertIn('startup_error', self.read('campaign_state.json'))
        startup_audit.mark_startup_phase(self.cfg, 'CASE_RUN')
        startup_audit.complete_startup(self.cfg, 0)
        state = self.read('campaign_state.json')
        self.assertEqual(state['results'], previous['results'])
        self.assertNotIn('startup_error', state)
        self.assertNotIn('failure_stage', state)
        self.assertEqual((self.root / 'startup_error.json').read_bytes(), history)
        self.assertTrue((self.root / 'stack_logs/startup_error.log').is_file())
        self.assertEqual(self.read('startup_status.json')['exit_code'], 0)

    def test_conflict_guard_collects_all_blocking_publishers_without_killing_them(self):
        publishers = [
            ['/Ego_topic', ['/manual_udp']],
            ['/ctrl_cmd', ['/manual_controller']],
            ['/planning/trajectory', ['/manual_planner']],
            ['/rosout', ['/rosout']]]
        master = SimpleNamespace(getSystemState=lambda: (publishers, [], []))
        with patch.dict(sys.modules, {'rosgraph': SimpleNamespace(Master=lambda name: master)}), \
             patch.object(runtime.os, 'killpg') as kill:
            with self.assertRaises(runtime.RuntimeFailure) as caught:
                runtime.check_no_conflicting_publishers()
        self.assertEqual(caught.exception.conflicting_publishers, {
            '/Ego_topic': ['/manual_udp'], '/ctrl_cmd': ['/manual_controller'],
            '/planning/trajectory': ['/manual_planner']})
        self.assertNotIn('/rosout', caught.exception.conflicting_publishers)
        kill.assert_not_called()

    def test_existing_roscore_housekeeping_publishers_are_not_a_conflict(self):
        master = SimpleNamespace(getSystemState=lambda: (
            [['/rosout', ['/rosout']], ['/rosout_agg', ['/rosout']]], [], []))
        with patch.dict(sys.modules, {'rosgraph': SimpleNamespace(Master=lambda name: master)}), \
             patch.object(runtime.os, 'killpg') as kill:
            runtime.check_no_conflicting_publishers()
        kill.assert_not_called()

    def campaign(self):
        document = yaml.safe_load(EXAMPLE.read_text())
        document['campaign_name'] = 'startup_guard_fixture'
        document['output_root'] = str(self.root)
        document['stack'].update(detector='od_gt', traffic_light=True,
                                 traffic_light_runtime='system', planviz=False)
        document['recording'].update(bag_enabled=False, video={'enabled': False})
        path = self.root / 'source.yaml'
        path.write_text(yaml.safe_dump(document))
        cfg = load_campaign(path, check_install=False)
        cfg['output'].mkdir(parents=True)
        return cfg

    def campaign_patches(self, stack):
        stack.enter_context(patch.object(runner, 'resolve_contract', return_value={'preset': 'od_gt'}))
        stack.enter_context(patch.object(runner, 'resolve_localization_contract',
            return_value={'fingerprint': 'f' * 64}))
        stack.enter_context(patch.object(runner, 'check_free_disk'))
        stack.enter_context(patch.object(runner.signal, 'signal'))

    def test_passing_traffic_preflight_then_existing_master_conflict_is_audited(self):
        cfg = self.campaign()
        publishers = [['/Ego_topic', ['/manual_udp']], ['/ctrl_cmd', ['/manual_controller']]]
        master = SimpleNamespace(getPid=lambda: 123,
            getSystemState=lambda: (publishers, [], []))
        with ExitStack() as patches:
            self.campaign_patches(patches)
            patches.enter_context(patch.dict(sys.modules,
                {'rosgraph': SimpleNamespace(Master=lambda name: master)}))
            passed = patches.enter_context(patch.object(runner, '_check_traffic_light'))
            start = patches.enter_context(patch.object(runtime.Processes, 'start'))
            persistent = patches.enter_context(patch.object(runner, '_start_persistent'))
            with self.assertRaises(runtime.RuntimeFailure) as caught:
                runner._run_campaign(cfg)
        passed.assert_called_once()
        start.assert_not_called()
        persistent.assert_not_called()
        self.assertIn('/Ego_topic', str(caught.exception))
        report = json.loads((cfg['output'] / 'startup_error.json').read_text())
        self.assertEqual(report['phase'], 'ROS_MASTER_CHECK')
        self.assertEqual(report['conflicting_publishers'], {
            '/Ego_topic': ['/manual_udp'], '/ctrl_cmd': ['/manual_controller']})
        state = json.loads((cfg['output'] / 'campaign_state.json').read_text())
        self.assertEqual(state['results'], {})
        self.assertEqual(state['startup_error']['phase'], 'ROS_MASTER_CHECK')

    def test_contract_failure_before_process_creation_keeps_original_exception(self):
        cfg = self.campaign()
        error = ValueError('fixture contract mismatch')
        with patch.object(runner, 'resolve_contract', side_effect=error), \
             patch.object(runner, 'Processes') as processes:
            with self.assertRaises(ValueError) as caught:
                runner._run_campaign(cfg)
        self.assertIs(caught.exception, error)
        processes.assert_not_called()
        report = json.loads((cfg['output'] / 'startup_error.json').read_text())
        self.assertEqual(report['phase'], 'CONTRACT_CHECK')
        self.assertEqual(report['error_type'], 'ValueError')
        self.assertEqual(report['error'], 'fixture contract mismatch')

    def test_audit_write_failure_does_not_replace_the_original_startup_error(self):
        cfg = self.campaign()
        error = ValueError('fixture original contract mismatch')
        with patch.object(runner, 'resolve_contract', side_effect=error), \
             patch.object(runner, 'record_startup_failure', side_effect=OSError('fixture disk error')), \
             patch.object(runner, 'Processes') as processes:
            with self.assertRaises(ValueError) as caught:
                runner._run_campaign(cfg)
        self.assertIs(caught.exception, error)
        processes.assert_not_called()

    def traffic_cfg(self):
        return {'output': self.root, 'stack': {
            'traffic_light': True, 'traffic_light_runtime': 'system',
            'traffic_light_python': '/usr/bin/python3', 'traffic_light_device': '0'}}

    def test_probe_pass_text_does_not_hide_process_crash_returncode(self):
        # A shutdown-time crash may happen after the probe prints JSON PASS.
        stdout = 'probe started\n' + json.dumps({'status': 'PASS', 'gpu': {'name': 'fixture'}}) + '\n'
        process = SimpleNamespace(returncode=-11, stdout=stdout)
        with patch.object(runner.subprocess, 'run', return_value=process):
            with self.assertRaisesRegex(runtime.RuntimeFailure, '-11'):
                runner._check_traffic_light(self.traffic_cfg())
        receipt = self.read('stack_logs/traffic_light_preflight.json')
        self.assertEqual(receipt['status'], 'FAIL')
        self.assertEqual(receipt['returncode'], -11)
        self.assertEqual(receipt['probe_report_status'], 'PASS')
        self.assertTrue(receipt['reported_pass_but_process_failed'])
        self.assertEqual((self.root / 'stack_logs/traffic_light_preflight.log').read_text(), stdout)
        self.assertEqual(receipt['python'], '/usr/bin/python3')
        self.assertEqual(receipt['device'], '0')
        self.assertTrue(receipt['command'])
        self.assertTrue(receipt['started_at'])
        self.assertTrue(receipt['finished_at'])

    def test_healthy_probe_exit_zero_and_json_pass_are_both_recorded(self):
        stdout = 'probe started\n' + json.dumps({'status': 'PASS', 'gpu': {'name': 'fixture'}}) + '\n'
        process = SimpleNamespace(returncode=0, stdout=stdout)
        with patch.object(runner.subprocess, 'run', return_value=process):
            runner._check_traffic_light(self.traffic_cfg())
        receipt = self.read('stack_logs/traffic_light_preflight.json')
        self.assertEqual(receipt['status'], 'PASS')
        self.assertEqual(receipt['returncode'], 0)
        self.assertEqual(receipt['probe_report_status'], 'PASS')
        self.assertFalse(receipt['reported_pass_but_process_failed'])

    def test_probe_timeout_keeps_partial_stdout_and_error_receipt(self):
        partial = b'fixture partial GPU diagnostic\n'
        error = subprocess.TimeoutExpired('fixture probe', 120, output=partial)
        with patch.object(runner.subprocess, 'run', side_effect=error):
            with self.assertRaises(runtime.RuntimeFailure):
                runner._check_traffic_light(self.traffic_cfg())
        receipt = self.read('stack_logs/traffic_light_preflight.json')
        self.assertEqual(receipt['status'], 'ERROR')
        self.assertIn('timed out', receipt['error'])
        self.assertTrue(receipt['finished_at'])
        self.assertEqual((self.root / 'stack_logs/traffic_light_preflight.log').read_text(),
                         partial.decode('utf-8'))


if __name__ == '__main__':
    unittest.main()
