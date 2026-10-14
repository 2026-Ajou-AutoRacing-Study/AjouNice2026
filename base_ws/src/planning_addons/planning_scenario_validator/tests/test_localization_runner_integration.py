"""Candidate launch, exclusive control and measured shutdown integration.

No ROS master or simulator is started. The native monitor's measurement gates
are tested separately in test_localization_safety_runtime.py; these tests verify
the runner does not silently bypass their evidence or proceed after a failure.
"""
import json
import os
import sys
import tempfile
import unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import yaml

from planning_scenario_validator import runner
from planning_scenario_validator.config import ROOT, load_campaign
from planning_scenario_validator.localization_contract import MR_PINNED_ARGS
from planning_scenario_validator.runtime import SafetyFailure


EXAMPLE = ROOT / 'base_ws/src/planning_addons/planning_scenario_validator/config/missions/gps_shadow/base/example.yaml'


class LocalizationRunnerIntegrationTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.path = self.root / 'candidate.yaml'
        self.document = yaml.safe_load(EXAMPLE.read_text())
        self.document['output_root'] = str(self.root / 'results')
        self.document['stack'].update(detector='od_gt', traffic_light=False, planviz=False)
        self.document['recording'].update(bag_enabled=False, video={'enabled': False})

    def cfg(self, *, safety=False):
        if safety:
            self.document['validation']['localization_safety'] = {'enabled': True}
        self.path.write_text(yaml.safe_dump(self.document))
        return load_campaign(self.path, check_install=False)

    def processes(self):
        process = Mock()
        process.log_dir = self.root / 'stack_logs'
        process.log_dir.mkdir(exist_ok=True)
        process.env = dict(os.environ)
        process.commands = []
        process.running = {'planning': Mock(), 'udp_control': Mock(), 'controller': Mock()}
        for child in process.running.values():
            child.poll.return_value = None
        return process

    def monitor(self):
        monitor = Mock()
        monitor.begin_case.side_effect = lambda path: Path(path).touch()
        monitor.drive_evidence.return_value = {'drive_observation_available': True}
        monitor.drive_started_at = None
        monitor.start_driving.side_effect = lambda tick: setattr(monitor, 'drive_started_at', tick)
        monitor.disarm.side_effect = lambda: setattr(monitor, 'drive_started_at', None)
        monitor.collision_count, monitor.goal_min, monitor.max_speed = 0, 0.1, 5.0
        monitor.control_source = '/longi_pi_autoware'
        monitor.stamps = {'control': 123.0}
        monitor.wait_stationary.return_value = {'park_verified': True, 'speed_mps': 0.0}
        monitor.run_until_done.return_value = ('SUCCESS', 1.0)
        return monitor

    def run_case(self, cfg, *, drive_reason='SUCCESS', stop_error=None,
                 startup_error=None, watchdog_error=None, terminal_watchdog_error=None):
        cfg['output'].mkdir(parents=True, exist_ok=True)
        case = cfg['cases'][0]
        state = runner._load_state(cfg, False)
        processes, monitor = self.processes(), self.monitor()
        if startup_error:
            monitor.wait_stationary.side_effect = startup_error
        if watchdog_error:
            monitor.localization_watchdog.side_effect = watchdog_error
            def drive(case, validation, tick, progress):
                progress({'elapsed_sec': 0.1})
                return drive_reason, 1.0
            monitor.run_until_done.side_effect = drive
        else:
            monitor.run_until_done.return_value = (drive_reason, 1.0)
        if terminal_watchdog_error:
            monitor.localization_watchdog.side_effect = terminal_watchdog_error
        if drive_reason == 'COLLISION':
            monitor.collision_count = 1
        with ExitStack() as patches:
            patches.enter_context(patch.object(runner, 'prepare_scenario', return_value=(case.source, (0, 0))))
            for name in ('BrakeHold', '_load_scene', '_start_case_stack', 'publish_brake',
                         '_verify_control_owner', '_capture_runtime_receipt', '_upload'):
                patches.enter_context(patch.object(runner, name))
            patches.enter_context(patch.object(runner, 'request_route', return_value='route-fixture'))
            patches.enter_context(patch.object(runner.subprocess, 'run'))
            patches.enter_context(patch.object(runner.subprocess, 'check_output', return_value='git-fixture'))
            patches.enter_context(patch('rospy.wait_for_message'))
            stop = patches.enter_context(patch.object(runner, '_stop_and_verify',
                return_value={'status': 'VERIFIED', 'stop': {'park_verified': True}}))
            if stop_error:
                stop.side_effect = stop_error
            recorder = patches.enter_context(patch.object(runner, 'Recording'))
            recorder.return_value.stop.return_value = (None, None)
            result = runner._run_case(cfg, processes, monitor, case, state)
        return result, state, processes, monitor, stop

    def test_explicit_candidate_scalars_and_paths_reach_actual_pose_launch(self):
        config = self.root / 'candidate_global.yaml'
        config.write_text('frequency: 40\n')
        self.document['stack']['localization_args'] = {
            'control_translation_tau_sec': 0.15, 'control_yaw_tau_sec': 0.25,
            'use_quantized_gnss_control_state': True,
            'gnss_quantization_input_mode': 'cell_transition',
            'global_ekf_gt_config': str(config)}
        cfg = self.cfg()
        processes, monitor = self.processes(), self.monitor()
        runner._start_case_stack(cfg, processes, monitor)
        pose = processes.start.call_args_list[0].args
        self.assertEqual(pose[0], 'pose')
        command = pose[1]
        self.assertEqual(command[:3], ['roslaunch', 'bringup', 'localization_gnss_imu_gt_ctra.launch'])
        self.assertIn('global_ekf_gt_config:=' + str(config), command)
        self.assertIn('control_translation_tau_sec:=0.15', command)
        self.assertIn('control_yaw_tau_sec:=0.25', command)
        self.assertIn('gnss_quantization_input_mode:=cell_transition', command)
        self.assertIn('use_quantized_gnss_control_state:=true', command)
        for name, value in MR_PINNED_ARGS.items():
            expected = runner._bool(value) if isinstance(value, bool) else str(value)
            self.assertIn(name + ':=' + expected, command)
        self.assertIn('sensor_config:=' + str(cfg['network']['sensor_json']), command)

    def test_gt_correction_is_opt_in_and_legacy_launch_is_unchanged(self):
        self.document['stack']['pose_source'] = 'maptf_morai'
        processes, monitor = self.processes(), self.monitor()
        runner._start_case_stack(self.cfg(), processes, monitor)
        self.assertIn('maptf_morai.launch', processes.start.call_args_list[0].args[1])
        self.document['stack']['gt_pose_args'] = {
            'use_heading_correction': True, 'heading_yaw_offset_deg': -1.347}
        processes = self.processes()
        runner._start_case_stack(self.cfg(), processes, monitor)
        command = processes.start.call_args_list[0].args[1]
        self.assertIn('localization_morai_gt.launch', command)
        self.assertIn('use_heading_correction:=true', command)
        self.assertIn('heading_yaw_offset_deg:=-1.347', command)
        self.assertIn('use_sim_time:=false', command)

    def test_controller_ownership_requires_one_correct_publisher_and_new_packet(self):
        monitor = self.monitor()
        owners = ['/longi_pi_autoware']
        master = SimpleNamespace(getSystemState=lambda: ([['/ctrl_cmd', owners]], [], []))
        module = SimpleNamespace(Master=lambda name: master)
        with patch.dict(sys.modules, {'rosgraph': module}):
            runner._verify_control_owner(monitor, after=100)
            owners.append('/external_controller')
            with self.assertRaisesRegex(SafetyFailure, '소유권'):
                runner._verify_control_owner(monitor)
            owners.pop()
            monitor.control_source = '/external_controller'
            with self.assertRaisesRegex(SafetyFailure, '새로운 명령'):
                runner._verify_control_owner(monitor)
            monitor.control_source = '/longi_pi_autoware'
            with self.assertRaisesRegex(SafetyFailure, '새로운 명령'):
                runner._verify_control_owner(monitor, after=123)

    def test_runtime_receipt_preserves_graph_commands_and_actual_parameter_dump(self):
        for pose_source, topic in [('localization', '/localization/control_kinematic_state'),
                                   ('maptf_morai', '/localization/kinematic_state')]:
            with self.subTest(pose_source=pose_source):
                cfg = self.cfg()
                cfg['stack']['pose_source'] = pose_source
                cfg['localization_contract'] = {'fingerprint': 'identity-fixture'}
                processes = self.processes()
                processes.commands = [{'name': 'pose', 'command': ['roslaunch', 'bringup', 'fixture.launch']}]
                graph = ([['/ctrl_cmd', ['/longi_pi_autoware']]], [], [])
                master = SimpleNamespace(getSystemState=lambda: graph,
                    getTopicTypes=lambda: [['/ctrl_cmd', 'morai_msgs/CtrlCmd']])
                with patch.dict(sys.modules, {'rosgraph': SimpleNamespace(Master=lambda name: master)}), \
                     patch.object(runner.subprocess, 'run') as dump, \
                     patch.object(runner.subprocess, 'check_output', side_effect=[
                         'git-fixture\n', 'configs/localization/candidate.yaml\n']) as git:
                    runner._capture_runtime_receipt(cfg, processes, self.root)
                command = dump.call_args.args[0]
                self.assertEqual(command[:2], ['rosparam', 'dump'])
                self.assertEqual(Path(command[2]).name, 'ros_parameters_after_control.yaml')
                self.assertTrue(dump.call_args.kwargs['check'])
                receipt = json.loads((self.root / 'runtime_receipt.json').read_text())
                self.assertEqual(receipt['localization_contract'], cfg['localization_contract'])
                self.assertEqual(receipt['commands'], processes.commands)
                self.assertEqual(receipt['controller_state_topic'], topic)
                self.assertEqual(receipt['ros_system_state'][0][0][0], '/ctrl_cmd')
                self.assertEqual(receipt['stage'], 'control_ready')
                self.assertEqual(receipt['python_executable'], sys.executable)
                self.assertEqual(Path(receipt['runner_module']), Path(runner.__file__).resolve())
                self.assertEqual(receipt['git_sha'], 'git-fixture')
                self.assertEqual(receipt['tracked_modified_files'], ['configs/localization/candidate.yaml'])
                self.assertEqual(git.call_args_list[0].args[0], ['git', 'rev-parse', 'HEAD'])
                self.assertEqual(git.call_args_list[1].args[0], ['git', 'diff', '--name-only', 'HEAD'])

    def test_stop_then_park_is_measured_before_control_sender_shutdown(self):
        cfg = self.cfg(safety=True)
        processes, monitor = self.processes(), self.monitor()
        calls = []
        processes.stop.side_effect = lambda name: calls.append(('stop', name))
        processes.start.side_effect = lambda name, command: calls.append(('start', name, command))
        def verify(**kwargs):
            calls.append(('verify', kwargs))
            return {'park_verified': bool(kwargs.get('require_park'))}
        monitor.verify_stopped.side_effect = verify
        with patch.object(runner, 'publish_brake') as brake, \
             patch.object(runner, 'BrakeHold') as hold:
            result = runner._stop_and_verify(cfg, processes, monitor)
        self.assertEqual(result['status'], 'VERIFIED')
        self.assertTrue(result['stop']['park_verified'])
        self.assertEqual(calls[0], ('stop', 'parking_prepare'))
        self.assertEqual([entry[0] for entry in calls[1:]], ['verify', 'stop', 'start', 'verify', 'stop'])
        self.assertNotIn('require_park', calls[1][1])
        self.assertTrue(calls[4][1]['require_park'])
        self.assertTrue(any('parking_hold.launch' in part for part in calls[3][2]))
        hold.return_value.start.assert_called_once()
        hold.return_value.stop.assert_called_once()
        brake.assert_called_once()

    def test_unconfirmed_stop_does_not_claim_park_or_leave_owned_sender(self):
        for fail_index in (1, 2):
            with self.subTest(fail_index=fail_index):
                cfg = self.cfg(safety=True)
                processes, monitor = self.processes(), self.monitor()
                replies = [SafetyFailure('fresh feedback absent')] if fail_index == 1 else [
                    {'park_verified': False}, SafetyFailure('P feedback absent')]
                monitor.verify_stopped.side_effect = replies
                with patch.object(runner, 'publish_brake'), \
                     patch.object(runner, 'BrakeHold') as hold, \
                     self.assertRaisesRegex(SafetyFailure, 'feedback absent'):
                    runner._stop_and_verify(cfg, processes, monitor)
                processes.stop.assert_called_with('udp_control')
                if fail_index == 1:
                    hold.assert_not_called()
                else:
                    hold.return_value.stop.assert_called_once()

    def test_startup_safety_failure_never_launches_driving_control(self):
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True),
            startup_error=SafetyFailure('actual Ego moving'))
        self.assertEqual(result['reason'], 'SAFETY_FAILURE')
        self.assertEqual(result['status'], 'ERROR')
        self.assertTrue(result['halt_campaign'])
        self.assertNotIn('controller', [call.args[0] for call in processes.start.call_args_list])
        self.assertNotIn('udp_control', [call.args[0] for call in processes.start.call_args_list])
        monitor.start_driving.assert_not_called()
        stop.assert_called_once()

    def test_stop_failure_is_separate_from_original_drive_success_and_halts_campaign(self):
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True),
            stop_error=SafetyFailure('no measured P stop'))
        self.assertEqual(result['drive_reason'], 'SUCCESS')
        self.assertEqual(result['reason'], 'SAFETY_STOP_UNCONFIRMED')
        self.assertEqual(result['status'], 'ERROR')
        self.assertTrue(result['halt_campaign'])
        self.assertEqual(result['safety']['shutdown']['status'], 'UNCONFIRMED')

    def test_collision_can_halt_localization_campaign_after_verified_stop(self):
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True), drive_reason='COLLISION')
        self.assertEqual(result['reason'], 'COLLISION')
        self.assertEqual(result['status'], 'FAIL')
        self.assertTrue(result['halt_campaign'])
        self.assertEqual(result['safety']['shutdown']['status'], 'VERIFIED')

    def test_stream_safety_failure_is_fatal_even_with_system_error_continue(self):
        self.document['continue_on_system_error'] = True
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True),
            watchdog_error=SafetyFailure('gps stream stale'))
        self.assertEqual(result['reason'], 'SAFETY_FAILURE')
        self.assertTrue(result['halt_campaign'])
        monitor.localization_watchdog.assert_called_once()

    def test_successful_guarded_run_launches_controller_last_and_uses_smooth_state(self):
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True))
        launches = processes.start.call_args_list
        self.assertEqual(launches[-1].args[0], 'controller')
        self.assertIn('kinematic_state_topic:=/localization/control_kinematic_state', launches[-1].args[1])
        self.assertLess([call.args[0] for call in launches].index('udp_control'), len(launches) - 1)
        self.assertEqual(result['status'], 'PASS')
        self.assertFalse(result['halt_campaign'])
        self.assertTrue(result['safety']['startup']['park_verified'])
        self.assertEqual(result['safety']['shutdown']['status'], 'VERIFIED')

    def test_terminal_success_without_progress_still_checks_final_stream_freshness(self):
        result, state, processes, monitor, stop = self.run_case(self.cfg(safety=True),
            terminal_watchdog_error=SafetyFailure('final GPS stream stale'))
        monitor.run_until_done.assert_called_once()
        monitor.localization_watchdog.assert_called_once()
        self.assertEqual(result['reason'], 'SAFETY_FAILURE')
        self.assertEqual(result['status'], 'ERROR')
        self.assertTrue(result['halt_campaign'])
        stop.assert_called_once()

    def test_changed_localization_contract_refuses_campaign_before_launching_ros(self):
        cfg = self.cfg(safety=True)
        cfg['localization_contract'] = {'fingerprint': 'old-heading-or-config'}
        with patch.object(runner, 'resolve_localization_contract',
                          return_value={'fingerprint': 'different-heading-or-config'}), \
             patch.object(runner, 'Processes') as processes, \
             patch.object(runner, '_start_master') as master, \
             self.assertRaisesRegex(runner.RuntimeFailure, '기존 실행'):
            runner._run_campaign(cfg)
        processes.assert_not_called()
        master.assert_not_called()

    def test_legacy_results_without_localization_receipt_cannot_resume_opted_in_experiment(self):
        cfg = self.cfg(safety=True)
        cfg['output'].mkdir(parents=True)
        state = runner._load_state(cfg, False)
        case = cfg['cases'][0]
        state['results'][case.name] = {'source_sha256': case.source_sha256,
            'status': 'PASS', 'reason': 'SUCCESS', 'attempts': 1,
            'result_path': str(self.root / 'old_result.json')}
        runner._write_state(cfg, state)
        with patch.object(runner, 'resolve_localization_contract', return_value={'fingerprint': 'new-contract'}), \
             patch.object(runner, 'Processes') as processes, \
             self.assertRaisesRegex(runner.RuntimeFailure, '측위 계약이 없는'):
            runner._run_campaign(cfg)
        processes.assert_not_called()

    def test_campaign_halt_flag_prevents_second_scene_even_if_pass_and_continue_enabled(self):
        self.document['continue_on_system_error'] = True
        cfg = self.cfg(safety=True)
        self.assertGreater(len(cfg['cases']), 1)
        visited = []
        def fake_case(config, processes, monitor, case, state):
            visited.append(case.name)
            state['results'][case.name] = {'source_sha256': case.source_sha256,
                'status': 'PASS', 'reason': 'SUCCESS', 'attempts': 1,
                'result_path': str(self.root / 'none.json')}
            return {'status': 'PASS', 'reason': 'SUCCESS', 'halt_campaign': True}
        with patch.dict(sys.modules, {'rospy': SimpleNamespace(init_node=Mock())}), \
             patch.object(runner, '_env', return_value={}), patch.object(runner, 'Processes'), \
             patch.object(runner, 'LiveMonitor'), patch.object(runner, '_start_master'), \
             patch.object(runner, '_start_persistent'), patch.object(runner, '_check_traffic_light'), \
             patch.object(runner, 'check_free_disk'), patch.object(runner.signal, 'signal'), \
             patch.object(runner, 'resolve_localization_contract', return_value={'fingerprint': 'fixture'}), \
             patch.object(runner, '_run_case', side_effect=fake_case):
            self.assertEqual(runner._run_campaign(cfg), 1)
        self.assertEqual(visited, [cfg['cases'][0].name])
        state = json.loads((cfg['output'] / 'campaign_state.json').read_text())
        self.assertEqual(state['run_status'], 'INTERRUPTED')


if __name__ == '__main__':
    unittest.main()
