"""Resolved production pair and frozen rollback/resume; no ROS/live stack."""
from pathlib import Path
from unittest.mock import Mock, patch
import copy
import unittest

import yaml

from planning_scenario_validator import config, executions, gui, runner
from planning_scenario_validator.config import ConfigError, traffic_temporal_options
import test_traffic_light_controls as controls


class TrafficProductionPairTest(unittest.TestCase):
    save = controls.TrafficControlsTests.save
    load = controls.TrafficControlsTests.load

    def setUp(self):
        controls.TrafficControlsTests.setUp(self)
        self.document['stack']['traffic_light_profile'] = 'v5_candidate'
        check = patch.object(config, 'runtime_settings', return_value=('/mock/python', '0'))
        check.start(); self.addCleanup(check.stop)

    def commands(self, cfg):
        processes = Mock()
        assets = None if cfg['stack'].get('traffic_light_profile') == 'legacy' else {'root': '/mock/models'}
        with patch.object(runner, 'runtime_settings', return_value=('/mock/python', '0')), \
             patch.object(runner, 'candidate_assets', return_value=assets):
            runner._start_case_stack(cfg, processes, Mock())
        return {call.args[0]: call.args[1] for call in processes.start.call_args_list}

    def test_fresh_v5_pair_defaults_and_explicit_conservative_zero(self):
        cfg = self.load()
        temporal = cfg['traffic_light_temporal_options']
        self.assertEqual(temporal, dict(enable_signal_compliance_guard=True,
            signal_compliance_unknown_grace_sec=.3, preserve_unreadable_color_evidence=True,
            planner_compliance_observations=True, stamped_vote_window_size=0,
            signal_compliance_required_green_observations=1))
        self.document['stack']['signal_compliance_unknown_grace_sec'] = 0.
        temporal = self.load()['traffic_light_temporal_options']
        self.assertEqual(temporal['signal_compliance_unknown_grace_sec'], 0.)
        self.assertFalse(temporal['preserve_unreadable_color_evidence'])
        self.assertTrue(temporal['planner_compliance_observations'])

    def test_legacy_off_and_guard_off_never_inherit_positive_grace(self):
        for changes, direct in (({'traffic_light_profile': 'legacy'}, False),
                                ({'traffic_light': False}, False),
                                ({'enable_signal_compliance_guard': False}, True)):
            with self.subTest(changes=changes):
                stack = dict(traffic_light=True, traffic_light_profile='v5_candidate',
                             signal_compliance_unknown_grace_sec=.3)
                stack.update(changes)
                result = traffic_temporal_options(stack)
                self.assertEqual(result['signal_compliance_unknown_grace_sec'], 0.)
                self.assertFalse(result['preserve_unreadable_color_evidence'])
                self.assertEqual(result['planner_compliance_observations'], direct)
                self.assertEqual(result['stamped_vote_window_size'], 0)

    def test_independent_unpaired_marker_direct_or_vote_fields_are_rejected(self):
        for changes in ({'preserve_unreadable_color_evidence': False},
                        {'signal_compliance_unknown_grace_sec': 0., 'preserve_unreadable_color_evidence': True},
                        {'planner_compliance_observations': False},
                        {'stamped_vote_window_size': 3}, {'stamped_vote_window_size': 7}):
            with self.subTest(changes=changes):
                self.document['stack'].update(changes)
                with self.assertRaises(ConfigError):
                    self.load()
                for key in changes:
                    del self.document['stack'][key]

    def test_types_and_bounds_fail_before_runtime_even_when_traffic_is_off(self):
        for invalid in (True, -.001, .300001, float('nan'), float('inf'), '0.3', None):
            document = copy.deepcopy(self.document)
            document['stack'].update(traffic_light=False, signal_compliance_unknown_grace_sec=invalid)
            with self.subTest(invalid=invalid), self.assertRaisesRegex(ConfigError, 'unknown_grace'):
                config.validate_campaign_document(document)
        for key in ('enable_signal_compliance_guard', 'preserve_unreadable_color_evidence',
                    'planner_compliance_observations'):
            for invalid in (0, 1, 'false', None):
                with self.subTest(key=key, invalid=invalid), self.assertRaisesRegex(ConfigError, key):
                    traffic_temporal_options({'traffic_light': False, key: invalid})

    def test_old_v5_snapshot_missing_grace_resolves_zero_without_rewriting_input(self):
        path = self.save('config_snapshot.yaml')
        before = path.read_bytes()
        cfg = config.load_campaign(path, check_install=False)
        self.assertEqual(path.read_bytes(), before)
        self.assertNotIn('signal_compliance_unknown_grace_sec', cfg['document']['stack'])
        self.assertNotIn('preserve_unreadable_color_evidence', cfg['document']['stack'])
        self.assertEqual(cfg['traffic_light_temporal_options']['signal_compliance_unknown_grace_sec'], 0.)
        self.assertFalse(cfg['traffic_light_temporal_options']['preserve_unreadable_color_evidence'])
        self.assertTrue(cfg['traffic_light_temporal_options']['planner_compliance_observations'])
        commands = self.commands(cfg)
        self.assertIn('signal_compliance_unknown_grace_sec:=0.0', commands['planning'])
        self.assertIn('preserve_unreadable_color_evidence:=false', commands['traffic_light'])

    def test_legacy_resume_without_profile_keeps_legacy_and_zero_in_memory_only(self):
        del self.document['stack']['traffic_light_profile']
        path = self.save('config_snapshot.yaml'); before = path.read_bytes()
        cfg = config.load_campaign(path, check_install=False)
        self.assertEqual(cfg['stack']['traffic_light_profile'], 'legacy')
        self.assertNotIn('traffic_light_profile', cfg['document']['stack'])
        self.assertEqual(path.read_bytes(), before)
        commands = self.commands(cfg)
        self.assertIn('traffic_light_legacy.launch', commands['traffic_light'])
        self.assertIn('planner_compliance_observations:=false', commands['traffic_light'])
        self.assertIn('signal_compliance_unknown_grace_sec:=0.0', commands['planning'])

    def test_new_execution_freezes_the_resolved_pair_and_resumes_it_exactly(self):
        self.document['stack']['signal_compliance_required_green_observations'] = 2
        path = self.save(); original = path.read_bytes()
        with patch.object(executions, 'runtime_settings', return_value=('/mock/python', '0')):
            cfg = executions.prepare_execution(path, check_install=False)
        stack = cfg['document']['stack']
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(stack['traffic_light_profile'], 'v5_candidate')
        self.assertEqual(stack['signal_compliance_unknown_grace_sec'], .3)
        self.assertTrue(stack['preserve_unreadable_color_evidence'])
        self.assertTrue(stack['planner_compliance_observations'])
        self.assertTrue(stack['enable_signal_compliance_guard'])
        self.assertEqual(stack['stamped_vote_window_size'], 0)
        self.assertEqual(stack['signal_compliance_required_green_observations'], 2)
        frozen = cfg['path'].read_bytes()
        resumed = executions.load_execution(cfg['output'], check_install=False)
        self.assertEqual(resumed['traffic_light_temporal_options'], cfg['traffic_light_temporal_options'])
        self.assertEqual(cfg['path'].read_bytes(), frozen)

    def test_gui_noop_preserves_advanced_grace_and_derived_frozen_fields(self):
        self.document['stack'].update(traffic_temporal_options(self.document['stack']))
        values = gui.form_values(self.document)
        self.assertEqual(gui.build_document(self.document, values), self.document)
        self.assertEqual(gui.operator_document(self.document, self.document, values, None), self.document)

    def test_runner_explicitly_forwards_both_sides_for_each_public_profile(self):
        cases = ({}, {'traffic_light_profile': 'legacy'}, {'traffic_light': False},
                 {'enable_signal_compliance_guard': False})
        for changes in cases:
            with self.subTest(changes=changes):
                stack = copy.deepcopy(self.document['stack']); stack.update(changes)
                doc = copy.deepcopy(self.document); doc['stack'] = stack
                self.path.write_text(yaml.safe_dump(doc))
                cfg = config.load_campaign(self.path, check_install=False)
                temporal = cfg['traffic_light_temporal_options']; commands = self.commands(cfg)
                self.assertIn('signal_compliance_unknown_grace_sec:=' + str(temporal['signal_compliance_unknown_grace_sec']),
                              commands['planning'])
                self.assertIn('enable_signal_compliance_guard:=' + str(temporal['enable_signal_compliance_guard']).lower(),
                              commands['planning'])
                if stack['traffic_light']:
                    for key in ('preserve_unreadable_color_evidence', 'planner_compliance_observations'):
                        self.assertIn(key + ':=' + str(temporal[key]).lower(), commands['traffic_light'])
                    self.assertIn('stamped_vote_window_size:=0', commands['traffic_light'])
                else:
                    self.assertNotIn('traffic_light', commands)


if __name__ == '__main__':
    unittest.main()
