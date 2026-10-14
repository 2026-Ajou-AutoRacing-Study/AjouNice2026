"""Pinned model, GUI/save/resume and real indexed-bag contracts."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from planning_scenario_validator import executions, gui, models, runner
from planning_scenario_validator.config import ROOT, load_campaign, validate_campaign_document
from planning_scenario_validator.recording_audit import audit_bag, CORE, SENSORS
from planning_scenario_validator.runtime import LiveMonitor, RuntimeFailure


class ModelsAndAuditTests(unittest.TestCase):
    def setUp(self):
        self.document = gui.load_document(gui.MISSIONS / 'full_course/base/base5_e24_analysis_sample.yaml')
        self.sensor = ROOT / 'configs/sensors/Sensor_competition_local.json'

    def test_new_execution_defaults_to_promoted_e24_without_rewriting_yaml(self):
        old = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        self.assertNotIn('bevfusion_model', old['stack'])
        self.assertEqual(models.selected_model(old['stack']), 'base5_e24')
        self.assertEqual(gui.build_document(old, gui.form_values(old)), old)
        self.assertEqual(models.PRESETS['e25']['thresholds'], [0.4, 0.15, 0.1])

    def test_gui_operator_round_trip_preserves_model_analysis_and_fixed_parameters(self):
        old = gui.load_document(gui.MISSIONS / 'full_course/base/example.yaml')
        old['stack']['bevfusion_model'] = 'e25'
        values = gui.form_values(old)
        values['bevfusion_model'] = models.PRESETS['base5_e24']['label']
        values['recording_mode'] = '인지 분석 (센서+영상)'
        changed = gui.operator_document(old, old, values, None)
        self.assertEqual(changed['stack']['bevfusion_model'], 'base5_e24')
        self.assertTrue(changed['recording']['include_sensors'])
        self.assertTrue(changed['recording']['video']['enabled'])
        self.assertEqual(changed['default_case'], old['default_case'])
        text = gui.updated_yaml(yaml.safe_dump(old, allow_unicode=True, sort_keys=False), changed)
        restored = yaml.safe_load(text)
        self.assertEqual(gui.form_values(restored)['bevfusion_model'], values['bevfusion_model'])
        self.assertEqual(gui.form_values(restored)['recording_mode'], values['recording_mode'])

    def test_model_choice_conflicts_with_external_yaml_edit_are_rejected(self):
        old = gui.load_document(gui.MISSIONS / 'full_course/base/example.yaml')
        old['stack']['bevfusion_model'] = 'e25'
        latest = copy.deepcopy(old)
        latest['stack']['bevfusion_model'] = 'base5_e9'
        values = gui.form_values(old)
        values['bevfusion_model'] = models.PRESETS['base5_e24']['label']
        with self.assertRaisesRegex(ValueError, '양쪽'):
            gui.operator_document(old, latest, values, None)

    def test_bad_preset_and_arbitrary_checkpoint_config_are_refused(self):
        for stack in ({'bevfusion_model': 'bad'}, {'bevfusion_checkpoint': 'unknown.pth'},
                      {'bevfusion_config': 'other.py'}):
            with self.assertRaises(ValueError):
                models.selected_model(stack)

    def test_analysis_cannot_silently_omit_sensors_or_video(self):
        with tempfile.TemporaryDirectory() as temp:
            for part in ('include_sensors', 'video'):
                bad = copy.deepcopy(self.document)
                bad['recording'][part] = False if part == 'include_sensors' else {'enabled': False}
                path = Path(temp) / 'bad.yaml'
                path.write_text(yaml.safe_dump(bad))
                with self.assertRaisesRegex(ValueError, '인지 분석'):
                    load_campaign(path, check_install=False)

    def test_e24_checkpoint_and_config_mismatch_are_refused(self):
        with patch.object(models, 'sha256', return_value='wrong'):
            with self.assertRaisesRegex(ValueError, 'config 계약'):
                models.resolve_contract(self.document['stack'], self.sensor, ROOT)
        real_hash = models.sha256
        with patch.object(models, 'sha256', side_effect=lambda p: 'wrong' if str(p).endswith('base5_e24_inference.pth') else real_hash(p)):
            with self.assertRaisesRegex(ValueError, 'checkpoint SHA256'):
                models.resolve_contract(self.document['stack'], self.sensor, ROOT)

    def test_e24_command_exact_options_sensor_and_canonical_bridge(self):
        contract = models.resolve_contract(self.document['stack'], self.sensor, ROOT)
        server, bridge = models.trial_commands(contract, ROOT, 'owned-test')
        for option in ('--base5-runtime', '--point-intensity-source', 'ros255', '--amp', '--parallel-camera'):
            self.assertIn(option, server)
        self.assertNotIn('--compile-parts', server)
        self.assertIn('{}:/config/active_sensor.json:ro'.format(self.sensor.resolve()), server)
        self.assertIn('sensor_config:=' + str(self.sensor.resolve()), bridge)
        self.assertIn('output_objects:=' + models.CANONICAL, bridge)
        self.assertIn('pedestrian_score_threshold:=0.1', bridge)
        self.assertTrue(contract['checkpoint_verified'])
        self.assertEqual(contract['thresholds'], {'vehicle': 0.4, 'pedestrian': 0.1, 'obstacle': 0.1})

    def test_e9_is_calibrated_but_not_the_production_default(self):
        self.assertIn('비교용', models.PRESETS['base5_e9']['label'])
        self.assertEqual(models.PRESETS['base5_e9']['thresholds'], [0.3, 0.2, 0.12])
        self.assertNotEqual(models.DEFAULT_MODEL, 'base5_e9')
        contract = models.resolve_contract(dict(detector='bevfusion', bevfusion_model='base5_e9'), self.sensor, ROOT)
        _, bridge = models.trial_commands(contract, ROOT, 'comparison-test')
        self.assertTrue(any('base5_e9_calibrated_trial.launch' in part for part in bridge))

    def test_e24_uses_the_portable_installed_path_not_ignored_evaluation_artifacts(self):
        contract = models.resolve_contract(self.document['stack'], self.sensor, ROOT)
        self.assertEqual(contract['checkpoint_path'], str(Path.home() / 'bevfusion_artifacts/checkpoints/base5_e24_inference.pth'))
        self.assertNotIn('artifacts/evaluation', contract['checkpoint_path'])

    def test_e24_runtime_promotes_nearest_and_gravity_without_changing_e9(self):
        e24=models.resolve_contract(self.document['stack'],self.sensor,ROOT)
        self.assertTrue(e24['model_config'].endswith('nearest_depth_runtime.py'))
        self.assertEqual(e24['depth_raster_policy'],'nearest_positive')
        self.assertEqual(e24['box_center_output_origin'],'gravity_center_v1')
        _,bridge=models.trial_commands(e24,ROOT,'isolated-test')
        self.assertIn('box_center_output_origin:=gravity_center_v1',bridge)
        e9=models.resolve_contract(dict(detector='bevfusion',bevfusion_model='base5_e9'),self.sensor,ROOT)
        self.assertTrue(e9['model_config'].endswith('pedestrian_merged_runtime.py'))
        self.assertEqual(e9['box_center_output_origin'],'legacy_passthrough')
        self.assertIsNone(e9['depth_raster_policy'])

    def test_explicit_e25_selection_passes_model_argument_to_promoted_runner(self):
        with tempfile.TemporaryDirectory() as t:
            cfg = dict(network=dict(sensor_json=str(self.sensor), ego_network_json='network',
                                   has_lidar=True, camera_count=4),
                       stack=dict(detector='bevfusion', bevfusion_model='e25', planviz=False),
                       document={}, output=Path(t))
            processes = unittest.mock.Mock()
            monitor = SimpleNamespace(stamps={'ego': 1}, wait=unittest.mock.Mock())
            with patch.object(runner.shutil, 'which', return_value=None):
                runner._start_persistent(cfg, processes, monitor)
            args = next(call.args[1] for call in processes.start.call_args_list if call.args[0] == 'bevfusion')
            self.assertEqual(args[-3:], ['--model', 'e25', str(self.sensor)])

    def test_changed_checkpoint_path_in_receipt_is_rejected_before_loading(self):
        contract = models.resolve_contract(self.document['stack'], self.sensor, ROOT)
        contract['checkpoint_path'] = '/tmp/wrong.pth'
        with self.assertRaisesRegex(ValueError, '계약 내용/hash'):
            models.verify_contract(contract, self.document['stack'], self.sensor, ROOT)

    def test_resume_refuses_model_or_core_setting_change(self):
        with tempfile.TemporaryDirectory() as temp:
            doc = copy.deepcopy(self.document)
            doc['output_root'] = temp
            path = Path(temp) / 'source.yaml'
            path.write_text(yaml.safe_dump(doc))
            cfg = executions.prepare_execution(path, check_install=False)
            self.assertEqual(cfg['model_contract']['preset'], 'base5_e24')
            self.assertTrue((cfg['output'] / 'model_configs/bridge.yaml').is_file())
            self.assertEqual(executions.load_execution(cfg['output'], check_install=False)['stack']['bevfusion_model'], 'base5_e24')
            altered = dict(cfg['model_contract'], fingerprint='changed')
            with patch('planning_scenario_validator.models.resolve_contract', return_value=altered):
                with self.assertRaisesRegex(ValueError, 'resume'):
                    executions.load_execution(cfg['output'], check_install=True)

    def test_actual_bag_message_counts_and_optional_clock(self):
        import rosbag
        from std_msgs.msg import String
        cfg = load_campaign(gui.MISSIONS / 'full_course/base/base5_e24_analysis_sample.yaml', check_install=False)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'test.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                for topic in CORE + SENSORS + ['/perception/base5_trial/status', '/perception/base5_trial/latency_ms',
                                               '/perception/traffic_light_recognition/traffic_signals']:
                    bag.write(topic, String(data='fixture'))
                bag.write('/Ego_topic', String(data='second'))
            result = audit_bag(path, cfg, driving_started=True)
            self.assertTrue(result['complete_for_drive'])
            self.assertTrue(result['re_inference_possible'])
            self.assertFalse(result['clock_recorded'])
            self.assertEqual(result['topics']['/Ego_topic']['message_count'], 2)
            self.assertFalse(result['topics']['/clock']['recorded'])

    def test_empty_requested_sensor_topic_is_not_claimed_recorded(self):
        import rosbag
        from std_msgs.msg import String
        cfg = load_campaign(gui.MISSIONS / 'full_course/base/base5_e24_analysis_sample.yaml', check_install=False)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'test.bag'
            with rosbag.Bag(str(path), 'w') as bag:
                bag.write('/Ego_topic', String(data='fixture'))
            result = audit_bag(path, cfg, driving_started=False)
            self.assertFalse(result['complete_for_drive'])
            self.assertFalse(result['re_inference_possible'])
            self.assertIn('/sensor/lidar1', result['missing_required'])
            self.assertEqual(result['topics']['/sensor/lidar1']['message_count'], 0)

    def test_duplicate_publisher_guard_never_accepts_two_owners(self):
        spec = importlib.util.spec_from_file_location('trial_runner_test', PACKAGE / 'scripts/run_bevfusion_trial.py')
        trial = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(trial)
        with patch.object(trial, 'publishers', return_value=['/e25', '/base5_trial/bevfusion_ros_bridge']):
            with self.assertRaisesRegex(RuntimeError, 'publisher'):
                trial.guard('/base5_trial/bevfusion_ros_bridge')
        with patch.object(trial, 'publishers', return_value=['/base5_trial/bevfusion_ros_bridge']):
            trial.guard('/base5_trial/bevfusion_ros_bridge')

    def test_planning_process_exit_is_not_misreported_as_trajectory_timeout(self):
        from types import SimpleNamespace
        monitor = LiveMonitor.__new__(LiveMonitor)
        monitor.wait = lambda predicate, seconds, label: predicate()
        with self.assertRaisesRegex(RuntimeFailure, 'planning 스택'):
            monitor.wait_route_and_trajectory([], after=0, timeout=75, process=SimpleNamespace(poll=lambda: 1))

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_real_gui_model_and_analysis_save_reload_and_running_lock(self):
        failures = []
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / 'mine.yaml'
            source.write_text(yaml.safe_dump(self.document))

            def ready(app):
                try:
                    app['actions']['open_yaml'](source)
                    app['fields']['bevfusion_model'].set(models.PRESETS['base5_e9']['label'])
                    app['save_button'].invoke()
                    app['actions']['open_yaml'](source)
                    self.assertEqual(app['fields']['bevfusion_model'].get(), models.PRESETS['base5_e9']['label'])
                    self.assertEqual(yaml.safe_load(source.read_text())['stack']['bevfusion_model'], 'base5_e9')
                    app['actions']['set_running_ui'](True)
                    self.assertEqual(str(app['field_widgets']['bevfusion_model'].cget('state')), 'disabled')
                except BaseException as exc:
                    failures.append(exc)
                finally:
                    app['root'].destroy()
            with patch.object(gui, 'USERS', root), patch.dict(os.environ, {'XDG_CONFIG_HOME': str(root / 'state')}), \
                 patch('tkinter.messagebox.showerror') as errors:
                gui.main(on_ready=ready)
                errors.assert_not_called()
            if failures:
                raise failures[0]
