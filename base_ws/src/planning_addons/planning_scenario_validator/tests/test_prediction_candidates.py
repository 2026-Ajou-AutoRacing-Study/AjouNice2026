"""Explicit map-horizon opt-in reaches launch without starting ROS or MORAI."""
import copy
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from planning_scenario_validator import executions, runner
from planning_scenario_validator.config import (
    ConfigError, load_campaign, prediction_horizon, prediction_mixed_horizon_enabled,
    validate_campaign_document, prediction_highway_horizon, prediction_highway_mode,
)


class PredictionCandidateTests(unittest.TestCase):
    def setUp(self):
        self.document = yaml.safe_load((PACKAGE / 'config/missions/highway/base/example.yaml').read_text())
        self.document['stack'].update(pose_source='maptf_morai', detector='od_gt',
                                      traffic_light=False, planviz=False)
        self.document['recording'].update(bag_enabled=False, video={'enabled': False})

    def test_absent_flag_is_off_without_mutating_source(self):
        original = copy.deepcopy(self.document)
        validate_campaign_document(self.document)
        self.assertFalse(prediction_mixed_horizon_enabled(self.document['stack']))
        self.assertEqual(original, self.document)

    def test_true_and_false_are_valid_explicit_choices(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                self.document['stack']['prediction_mixed_horizon_enabled'] = enabled
                validate_campaign_document(self.document)
                self.assertIs(prediction_mixed_horizon_enabled(self.document['stack']), enabled)

    def test_absent_native_horizon_remains_three_without_mutating_source(self):
        original = copy.deepcopy(self.document)
        validate_campaign_document(self.document)
        self.assertEqual(prediction_horizon(self.document['stack']), 3.0)
        self.assertEqual(self.document, original)

    def test_valid_native_horizons(self):
        for value in (0.1, 0.3, 3, 6, 6.0, 10):
            with self.subTest(value=value):
                self.document['stack']['prediction_horizon'] = value
                validate_campaign_document(self.document)
                self.assertEqual(prediction_horizon(self.document['stack']), float(value))

    def test_highway_horizon_defaults_to_eight_without_enabling_old_campaigns(self):
        before = copy.deepcopy(self.document)
        self.assertEqual(prediction_highway_horizon(self.document['stack']), 8.0)
        self.assertFalse(prediction_mixed_horizon_enabled(self.document['stack']))
        self.assertEqual(prediction_highway_mode(self.document['stack']), 'append')
        self.assertEqual(before, self.document)

    def test_highway_mode_requires_an_explicit_known_strategy(self):
        for value in ('append', 'native'):
            self.document['stack']['prediction_highway_mode'] = value
            validate_campaign_document(self.document)
            self.assertEqual(prediction_highway_mode(self.document['stack']), value)
        for value in (None, True, 0, [], {}, 'auto', 'Native'):
            with self.subTest(value=value):
                self.document['stack']['prediction_highway_mode'] = value
                with self.assertRaisesRegex(ConfigError, 'prediction_highway_mode'):
                    validate_campaign_document(self.document)

    def test_highway_horizon_is_bounded_to_valid_message_length(self):
        for value in (3.1, 6, 8.0, 9.9):
            self.document['stack']['prediction_highway_horizon'] = value
            validate_campaign_document(self.document)
            self.assertEqual(prediction_highway_horizon(self.document['stack']), value)
        for value in (None, True, '8', 3, 8.05, 10, float('nan'), float('inf'), 10**1000):
            with self.subTest(value=value):
                self.document['stack']['prediction_highway_horizon'] = value
                with self.assertRaisesRegex(ConfigError, 'prediction_highway_horizon'):
                    validate_campaign_document(self.document)

    def test_mixed_eight_second_choice_reaches_launch_and_survives_snapshot(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'candidate.yaml'
            self.document['stack'].update(prediction_mixed_horizon_enabled=True,
                                           prediction_highway_horizon=8.0,
                                           prediction_highway_mode='native')
            self.document['scenarios']['include'] = ['high_baseline_01.json']
            self.document['output_root'] = str(Path(temporary) / 'results')
            path.write_text(yaml.safe_dump(self.document))
            cfg = load_campaign(path, check_install=False)
            processes, monitor = Mock(), Mock()
            runner._start_case_stack(cfg, processes, monitor)
            command = next(c.args[1] for c in processes.start.call_args_list if c.args[0] == 'tracking')
            self.assertIn('prediction_mixed_horizon_enabled:=true', command)
            self.assertIn('prediction_highway_horizon:=8.0', command)
            self.assertIn('prediction_highway_mode:=native', command)
            self.assertIn('prediction_horizon:=3.0', command)
            prepared = executions.prepare_execution(path, check_install=False)
            self.document['stack']['prediction_highway_horizon'] = 6.0
            path.write_text(yaml.safe_dump(self.document))
            resumed = executions.load_execution(prepared['output'], check_install=False)
            self.assertEqual(prediction_highway_horizon(resumed['stack']), 8.0)
            self.assertEqual(prediction_highway_mode(resumed['stack']), 'native')

    def test_invalid_native_horizons_rejected_before_loading_or_driving(self):
        for value in (None, True, False, '3', '6.0', [], {}, 0, -1,
                      0.05, 3.05, 10.1, 10 ** 1000, float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value):
                self.document['stack']['prediction_horizon'] = value
                with self.assertRaisesRegex(ConfigError, 'prediction_horizon'):
                    validate_campaign_document(self.document)

    def test_non_boolean_flag_is_rejected_before_loading_or_driving(self):
        for value in (None, 0, 1, 'false', 'true', [], {}):
            with self.subTest(value=value):
                self.document['stack']['prediction_mixed_horizon_enabled'] = value
                with self.assertRaisesRegex(ConfigError, 'prediction_mixed_horizon_enabled'):
                    validate_campaign_document(self.document)

    def test_explicit_choice_reaches_actual_tracking_launch(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'candidate.yaml'
            for value in (None, False, True):
                with self.subTest(value=value):
                    if value is None:
                        self.document['stack'].pop('prediction_mixed_horizon_enabled', None)
                    else:
                        self.document['stack']['prediction_mixed_horizon_enabled'] = value
                    path.write_text(yaml.safe_dump(self.document))
                    cfg = load_campaign(path, check_install=False)
                    processes, monitor = Mock(), Mock()
                    runner._start_case_stack(cfg, processes, monitor)
                    command = next(call.args[1] for call in processes.start.call_args_list
                                   if call.args[0] == 'tracking')
                    self.assertEqual(command[:3], ['roslaunch', 'perception_object_pipeline',
                                                   'object_pipeline.launch'])
                    self.assertIn('prediction_mixed_horizon_enabled:=' +
                                  ('true' if value is True else 'false'), command)
                    self.assertIn('prediction_model:=map_based', command)
                    self.assertIn('start_detector:=false', command)
                    self.assertIn('prediction_horizon:=3.0', command)

    def test_native_six_second_horizon_reaches_actual_tracking_launch(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'candidate.yaml'
            self.document['stack'].update(prediction_horizon=6.0,
                                           prediction_mixed_horizon_enabled=False)
            path.write_text(yaml.safe_dump(self.document))
            cfg = load_campaign(path, check_install=False)
            processes, monitor = Mock(), Mock()
            runner._start_case_stack(cfg, processes, monitor)
            command = next(call.args[1] for call in processes.start.call_args_list
                           if call.args[0] == 'tracking')
            self.assertIn('prediction_horizon:=6.0', command)
            self.assertIn('prediction_mixed_horizon_enabled:=false', command)

    def test_frozen_execution_preserves_native_horizon_after_source_edit(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'candidate.yaml'
            self.document['stack'].update(prediction_horizon=6.0,
                                           prediction_mixed_horizon_enabled=False)
            self.document['scenarios']['include'] = ['high_baseline_01.json']
            self.document['output_root'] = str(Path(temporary) / 'results')
            path.write_text(yaml.safe_dump(self.document))
            original = path.read_bytes()
            prepared = executions.prepare_execution(path, check_install=False)
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(prediction_horizon(prepared['stack']), 6.0)
            self.document['stack']['prediction_horizon'] = 3.0
            path.write_text(yaml.safe_dump(self.document))
            resumed = executions.load_execution(prepared['output'], check_install=False)
            self.assertEqual(prediction_horizon(resumed['stack']), 6.0)
            self.assertFalse(prediction_mixed_horizon_enabled(resumed['stack']))

    def test_frozen_execution_preserves_choice_after_source_yaml_changes(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'candidate.yaml'
            self.document['stack']['prediction_mixed_horizon_enabled'] = True
            self.document['scenarios']['include'] = ['high_baseline_01.json']
            self.document['output_root'] = str(Path(temporary) / 'results')
            path.write_text(yaml.safe_dump(self.document))
            original = path.read_bytes()
            prepared = executions.prepare_execution(path, check_install=False)
            self.assertEqual(path.read_bytes(), original)
            self.assertTrue(prepared['stack']['prediction_mixed_horizon_enabled'])
            self.document['stack']['prediction_mixed_horizon_enabled'] = False
            path.write_text(yaml.safe_dump(self.document))
            resumed = executions.load_execution(prepared['output'], check_install=False)
            self.assertTrue(resumed['stack']['prediction_mixed_horizon_enabled'])


if __name__ == '__main__':
    unittest.main()
