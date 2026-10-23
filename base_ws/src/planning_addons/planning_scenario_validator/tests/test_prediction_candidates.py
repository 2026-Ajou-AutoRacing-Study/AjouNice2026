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
    ConfigError, load_campaign, prediction_mixed_horizon_enabled,
    validate_campaign_document,
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
