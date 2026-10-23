"""Model telemetry contract, basic completion audit and frozen-list regressions."""
import copy
import csv
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import rosbag
import yaml
from std_msgs.msg import String

from planning_scenario_validator import gui, models, runner, recording_options as options
from planning_scenario_validator.config import ROOT, load_campaign
from planning_scenario_validator.recording_audit import CORE, audit_bag, profile_contract_warnings
from planning_scenario_validator.runtime import Recording
from planning_scenario_validator.topic_contract import all_telemetry_topics, model_topics, validate_trial_remaps


class LatencyContractTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.doc = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        self.doc['output_root'] = str(self.root / 'results')
        self.doc['recording']['video']['enabled'] = False
        self.path = self.root / 'campaign.yaml'

    def cfg(self, preset='base5_e12'):
        self.doc['stack'].update(detector='bevfusion', bevfusion_model=preset, traffic_light=False)
        self.path.write_text(yaml.safe_dump(self.doc))
        return load_campaign(self.path, check_install=False)

    def bag(self, topics):
        path = self.root / 'fixture.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            for topic in dict.fromkeys(topics):
                bag.write(topic, String(data='fixture'))
        return path

    def test_basic_all_model_diagnostics_and_measurement_without_raw_sensors(self):
        cfg = self.cfg()
        self.assertTrue(set(all_telemetry_topics()) <= set(cfg['recording']['topics']))
        self.assertIn(model_topics('base5_e24')['measurement_objects'], cfg['recording']['topics'])
        self.assertFalse(set(options.SENSORS) & set(cfg['recording']['topics']))
        self.assertEqual(profile_contract_warnings(cfg), [])

    def test_both_models_use_same_contract_in_launcher_recording_and_audit(self):
        for preset in ('base5_e12', 'base5_e24', 'e25'):
            with self.subTest(preset=preset):
                cfg = self.cfg(preset)
                contract = model_topics(preset)
                if preset != 'e25':
                    validate_trial_remaps(ROOT, preset)
                report = audit_bag(self.bag(CORE + [contract['status'], contract['latency_ms']]), cfg, driving_started=True)
                self.assertTrue(report['complete_for_drive'])
                self.assertTrue(report['complete_for_latency'])
                self.assertEqual(report['status'], 'OK')
                other = model_topics('e25' if preset != 'e25' else 'base5_e24')
                self.assertEqual(report['topics'][other['latency_ms']]['message_count'], 0)
                self.assertNotIn(other['latency_ms'], report['missing_required'])

    def test_requested_but_unpublished_telemetry_is_warning_not_claimed_recorded(self):
        cfg = self.cfg()
        report = audit_bag(self.bag(CORE), cfg, driving_started=True)
        topic = model_topics('base5_e24')['latency_ms']
        self.assertIn(topic, cfg['recording']['topics'])
        self.assertFalse(report['topics'][topic]['recorded'])
        self.assertEqual(report['topics'][topic]['message_count'], 0)
        self.assertFalse(report['complete_for_latency'])
        self.assertEqual(report['status'], 'WARN')

    def test_lightweight_intentional_exclusions_are_not_missing_required(self):
        options.apply_mode(self.doc, options.LIGHT)
        cfg = self.cfg()
        cfg['stack']['traffic_light'] = True
        report = audit_bag(self.bag(cfg['recording']['topics']), cfg, driving_started=True)
        self.assertEqual(report['missing_required'], [])
        self.assertEqual(report['status'], 'OK')
        self.assertFalse(report['complete_for_latency'])

    def test_old_frozen_profile_missing_diagnostics_is_reported_without_rewrite(self):
        cfg = self.cfg()
        frozen = self.root / 'old_topic_profile.json'
        old_topics = [t for t in cfg['recording']['topics'] if t not in
                      (model_topics('base5_e24')['status'], model_topics('base5_e24')['latency_ms'],
                       model_topics('base5_e24')['measurement_objects'])]
        frozen.write_text(json.dumps(old_topics))
        original = frozen.read_bytes()
        cfg['recording'].update(topic_profile=str(frozen), profile_preset='basic', topics=old_topics)
        # profile_preset explicitly identifies a copied basic list, irrespective of filename.
        self.assertTrue(profile_contract_warnings(cfg))
        report = audit_bag(self.bag(old_topics), cfg, driving_started=True)
        self.assertEqual(frozen.read_bytes(), original)
        self.assertFalse(report['complete_for_latency'])
        self.assertIn(model_topics('base5_e24')['latency_ms'], report['not_requested_required'])

    def test_latency_preset_gui_roundtrip_patterns_no_sensor_no_forced_video(self):
        values = gui.form_values(self.doc)
        values['recording_mode'] = options.LATENCY
        self.doc = gui.operator_document(self.doc, self.doc, values, self.doc['scenarios'].get('include'))
        cfg = self.cfg()
        self.assertEqual(gui.form_values(self.doc)['recording_mode'], options.LATENCY)
        self.assertFalse(cfg['recording']['video']['enabled'])
        self.assertFalse(set(options.SENSORS) & set(cfg['recording']['topics']))
        self.assertTrue(cfg['recording']['topic_regex'])
        contract = model_topics('base5_e24')
        topics = CORE + [contract['status'], contract['latency_ms']]
        missing = audit_bag(self.bag(topics), cfg, driving_started=True)
        self.assertEqual(missing['status'], 'WARN')
        topics += ['/planning/behavior_path_planner/debug/total_time/processing_time_ms',
                   '/planning/motion_velocity_planner/debug/processing_time_ms',
                   contract['frame_timing']] + options.DOWNSTREAM_TIMING
        actual = audit_bag(self.bag(topics), cfg, driving_started=True)
        self.assertTrue(actual['complete_for_latency'])
        self.assertEqual(actual['planner_processing_time']['message_count'], 2)
        only_selector = audit_bag(self.bag(CORE + [contract['status'], contract['latency_ms'], contract['frame_timing']]
                                  + options.DOWNSTREAM_TIMING + ['/planning/scenario_selector/debug/processing_time_ms']),
                                  cfg, driving_started=True)
        self.assertFalse(only_selector['complete_for_analysis'])

    def test_latency_recorder_regex_does_not_capture_raw_sensors_or_all_topics(self):
        options.apply_mode(self.doc, options.LATENCY)
        cfg = self.cfg()
        recorder = Recording(cfg['recording'], self.root, {})
        process = Mock()
        process.poll.return_value = None
        with patch('planning_scenario_validator.runtime.subprocess.Popen', return_value=process) as start, \
             patch('planning_scenario_validator.runtime.time.sleep'), \
             patch('planning_scenario_validator.runtime.stop_group'):
            recorder.start('test')
            Path(str(recorder.bag_prefix) + '.bag').touch()
            recorder.stop('test_P')
        command = start.call_args.args[0]
        self.assertIn('--regex', command)
        self.assertNotIn('--all', command)
        self.assertTrue(all(pattern in command for pattern in cfg['recording']['topic_regex']))

    def test_basic_PASS_and_driving_flag_survive_missing_telemetry_audit(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                self.doc['campaign_name'] = 'audit_missing' if missing else 'audit_complete'
                cfg = self.cfg()
                cfg['output'].mkdir(parents=True)
                state = runner._load_state(cfg, False)
                case = cfg['cases'][0]
                contract = model_topics('base5_e24')
                topics = CORE if missing else CORE + [contract['status'], contract['latency_ms']]
                path = self.bag(topics)
                processes = Mock()
                processes.log_dir = self.root / 'stack_logs'
                processes.log_dir.mkdir(exist_ok=True)
                processes.env = dict(os.environ)
                processes.running = {'planning': Mock(), 'bevfusion': Mock()}
                monitor = Mock()
                monitor.begin_case.side_effect = lambda p: Path(p).touch()
                monitor.run_until_done.return_value = ('SUCCESS', 1.0)
                monitor.drive_evidence.return_value = {'drive_observation_available': True}
                monitor.drive_started_at = 123
                monitor.disarm.side_effect = lambda: setattr(monitor, 'drive_started_at', None)
                monitor.collision_count, monitor.goal_min, monitor.max_speed = 0, 0.1, 5.0
                with patch.object(runner, 'prepare_scenario', return_value=(case.source, (0, 0))), \
                     patch.object(runner, 'BrakeHold'), patch.object(runner, '_load_scene'), \
                     patch.object(runner, '_start_case_stack'), patch.object(runner, 'publish_brake'), \
                     patch.object(runner, 'request_route', return_value='test'), \
                     patch.object(runner.subprocess, 'run'), patch.object(runner.subprocess, 'check_output', return_value='test-sha'), \
                     patch('rospy.wait_for_message'), patch.object(runner, 'Recording') as record:
                    record.return_value.stop.return_value = (path, None)
                    result = runner._run_case(cfg, processes, monitor, case, state)
                self.assertEqual(result['status'], 'PASS')
                launches = processes.start.call_args_list
                self.assertEqual([call.args[0] for call in launches], ['udp_control', 'controller'])
                control_command = launches[0].args[1]
                for argument in ('enable_sensors:=false', 'enable_lidar:=false',
                                 'enable_ego_network:=false', 'enable_control:=true'):
                    self.assertIn(argument, control_command)
                stops = [call.args[0] for call in processes.stop.call_args_list]
                self.assertLess(stops.index('controller'), stops.index('udp_control'))
                self.assertEqual(result['reason'], 'SUCCESS')
                self.assertTrue(result['recording_audit']['driving_started'])
                self.assertEqual(result['recording_audit']['status'], 'WARN' if missing else 'OK')
                with (cfg['output'] / 'summary.csv').open() as stream:
                    row = next(csv.DictReader(stream))
                self.assertEqual(row['status'], 'PASS')
                self.assertEqual(row['recording_warning'], str(missing))
                label = gui.completion_label(0, state)
                self.assertEqual('녹화 경고' in label, missing)

    def test_persistent_receivers_never_enable_drive_sender(self):
        cfg = self.cfg()
        processes, monitor = Mock(), Mock()
        monitor.wait.side_effect = runner.RuntimeFailure('stop after sensor startup')
        with self.assertRaises(runner.RuntimeFailure):
            runner._start_persistent(cfg, processes, monitor)
        launches = processes.start.call_args_list
        self.assertEqual([call.args[0] for call in launches], ['udp', 'sensing'])
        self.assertIn('enable_control:=false', launches[0].args[1])

    def test_route_failure_never_starts_control_transport_or_controller(self):
        cfg = self.cfg()
        cfg['recording']['bag_enabled'] = False
        cfg['output'].mkdir(parents=True)
        state = runner._load_state(cfg, False)
        case = cfg['cases'][0]
        processes, monitor = Mock(), Mock()
        processes.log_dir = self.root / 'stack_logs'
        processes.log_dir.mkdir()
        processes.env = dict(os.environ)
        processes.running = {'planning': Mock(), 'bevfusion': Mock()}
        monitor.begin_case.side_effect = lambda path: Path(path).touch()
        monitor.drive_evidence.return_value = {}
        monitor.drive_started_at = None
        monitor.collision_count, monitor.goal_min, monitor.max_speed = 0, float('inf'), 0
        with patch.object(runner, 'prepare_scenario', return_value=(case.source, (0, 0))), \
             patch.object(runner, 'BrakeHold'), patch.object(runner, '_load_scene'), \
             patch.object(runner, '_start_case_stack'), patch.object(runner, 'publish_brake'), \
             patch.object(runner, 'request_route', side_effect=runner.ScenarioFailure('NO_TRAJECTORY', 'fixture')), \
             patch.object(runner.subprocess, 'run'), \
             patch.object(runner.subprocess, 'check_output', return_value='test-sha'):
            result = runner._run_case(cfg, processes, monitor, case, state)
        self.assertEqual(result['reason'], 'NO_TRAJECTORY')
        processes.start.assert_not_called()
        monitor.start_driving.assert_not_called()


if __name__ == '__main__':
    unittest.main()
