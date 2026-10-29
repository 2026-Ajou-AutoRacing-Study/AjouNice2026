#!/usr/bin/env python3
"""Highway current-lane forecast integration without a ROS master."""
import copy
from dataclasses import replace
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml
from autoware_perception_msgs.msg import ObjectClassification
from roslaunch.config import ROSLaunchConfig
from roslaunch.xmlloader import XmlLoader

import test_mixed_horizon_node as helpers
from test_highway_lane_policy import road
from map_based_prediction_node import MixedHorizonExtension
from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork, LaneletRecord, MapPredictionConfig,
)
from perception_object_pipeline.mixed_horizon import (
    MixedHorizonSelector, build_highway_horizons,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class HighwayCurrentLaneNodeTest(unittest.TestCase):
    setUp = helpers.MixedHorizonNodeTest.setUp
    track = helpers.MixedHorizonNodeTest.track
    message = helpers.MixedHorizonNodeTest.message
    predict = helpers.MixedHorizonNodeTest.predict

    def fork_network(self, fork_x=50., end_x=200.):
        self.network = LaneletNetwork([
            road(1, -10., fork_x, 0., following=(4, 3)),
            road(3, fork_x, end_x, 0.),
            LaneletRecord(4, ((fork_x, 0., 0.), (end_x, 15., 0.)), (),
                          (('speed_limit', '150'),)),
            road(2, -10., 10., 30., following=(5, 6), speed='50'),
            road(5, 10., 200., 30., speed='50'),
            LaneletRecord(6, ((10., 30., 0.), (200., 45., 0.)), (),
                          (('speed_limit', '50'),)),
        ], MapPredictionConfig(maximum_start_lanelets=3))

    def native(self, enabled=True):
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.,
                                        current_lanelet_only=True)
        return MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 10.),
            selector, mode='native', highway_current_lane_only=enabled,
            highway_lane_switch_margin_m=.2)

    def assert_current_fields_unchanged(self, original, selected):
        old, new = copy.deepcopy(original), copy.deepcopy(selected)
        old.kinematics.predicted_paths = []
        new.kinematics.predicted_paths = []
        self.assertEqual(helpers.wire(old), helpers.wire(new))

    def test_close_confidence_highway_fork_outputs_one_geometric_continuation(self):
        self.fork_network()
        message = self.message([self.track(1)])
        before = helpers.wire(message)
        original = self.predict(message, self.native(False))
        selected = self.predict(message, self.native(True))
        old_paths = original.objects[0].kinematics.predicted_paths
        self.assertEqual(len(old_paths), 2)
        self.assertLess(abs(old_paths[0].confidence - old_paths[1].confidence), .02)
        paths = selected.objects[0].kinematics.predicted_paths
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].path), 101)
        self.assertAlmostEqual(paths[0].path[-1].position.y, 0.)
        self.assert_current_fields_unchanged(original.objects[0], selected.objects[0])
        self.assertEqual(before, helpers.wire(message))

    def test_polygon_lane_wins_even_if_adjacent_candidate_has_higher_score(self):
        self.network = LaneletNetwork([road(1, -10., 200., 0.), road(2, -10., 200., 3.5)],
                                     MapPredictionConfig(maximum_start_lanelets=3))
        message = self.message([self.track(1, y=1.6)])
        candidates = self.network.lane_candidates(0., 1.6, 0.)
        ranked = tuple(sorted((replace(candidate, score=.01 if candidate.lanelet_id == 1 else .99)
                               for candidate in candidates), key=lambda candidate: candidate.score,
                              reverse=True))
        with patch.object(self.network, 'lane_candidates', return_value=ranked):
            original = self.predict(message, self.native(False))
            selected = self.predict(message, self.native(True))
        self.assertAlmostEqual(original.objects[0].kinematics.predicted_paths[0].path[-1].position.y, 3.5)
        self.assertEqual(len(selected.objects[0].kinematics.predicted_paths), 1)
        self.assertAlmostEqual(selected.objects[0].kinematics.predicted_paths[0].path[-1].position.y, 0.)

    def test_boundary_jitter_then_real_lane_entry_keeps_appropriate_one_path(self):
        self.network = LaneletNetwork([road(1, -10., 200., 0.), road(2, -10., 200., 3.5)],
                                     MapPredictionConfig(maximum_start_lanelets=3))
        extension = self.native(True)
        message = self.message([self.track(1)])
        for stamp, y, expected_lane_y in ((10., 0., 0.), (10.1, 1.8, 0.),
                                          (10.2, 2.05, 3.5), (10.3, 1.72, 3.5)):
            with self.subTest(stamp=stamp, y=y):
                message.header.stamp = helpers.rospy.Time.from_sec(stamp)
                message.objects[0].kinematics.pose_with_covariance.pose.position.y = y
                output = self.predict(message, extension)
                paths = output.objects[0].kinematics.predicted_paths
                self.assertEqual(len(paths), 1)
                self.assertAlmostEqual(paths[0].path[-1].position.y, expected_lane_y)

    def test_terminal_selected_lane_preserves_short_map_without_padding_or_long_ctrv(self):
        self.network = LaneletNetwork([road(1, -10., 20., 0.)])
        message = self.message([self.track(1)])
        message.objects[0].kinematics.twist_with_covariance.twist.angular.z = .2
        output = self.predict(message, self.native(True))
        paths = output.objects[0].kinematics.predicted_paths
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].path), 21)
        self.assertEqual(paths[0].path[-1].position.x, 20.)
        self.assertEqual(paths[0].path[-1].position.y, 0.)

    def test_missing_road_remains_baseline_three_second_ctrv(self):
        self.fork_network()
        message = self.message([self.track(1, y=100.)])
        original = self.predict(message)
        selected = self.predict(message, self.native(True))
        self.assertEqual(helpers.wire(original), helpers.wire(selected))
        self.assertEqual(len(selected.objects[0].kinematics.predicted_paths[0].path), 31)

    def test_tiny_negative_speed_keeps_current_lane_without_multiple_fork_paths(self):
        self.fork_network(fork_x=10.)
        message = self.message([self.track(1, speed=-.1)])
        paths = self.predict(message, self.native(True)).objects[0].kinematics.predicted_paths
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].path), 101)
        self.assertEqual(paths[0].path[-1].position.x, 0.)
        self.assertEqual(paths[0].path[-1].position.y, 0.)

    def test_current_lane_selection_exception_keeps_original_object_and_forecasts(self):
        self.fork_network()
        message = self.message([self.track(1)])
        original = self.predict(message, self.native(False))
        extension = self.native(True)
        with patch.object(extension.lane_policy, 'select', side_effect=RuntimeError('selection fault')), \
                patch('map_based_prediction_node.rospy.logwarn_throttle'):
            selected = self.predict(message, extension)
        self.assertEqual(helpers.wire(original), helpers.wire(selected))

    def test_native_exception_retains_selected_current_lane_short_map(self):
        self.fork_network(fork_x=10.)
        message = self.message([self.track(1)])
        predict = self.network.predict

        def fail_long(**kwargs):
            if kwargs['horizons'][-1] > 3.:
                raise RuntimeError('native prediction fault')
            return predict(**kwargs)

        with patch.object(self.network, 'predict', side_effect=fail_long), \
                patch('map_based_prediction_node.rospy.logwarn_throttle'):
            selected = self.predict(message, self.native(True))
        paths = selected.objects[0].kinematics.predicted_paths
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].path), 31)
        self.assertAlmostEqual(paths[0].path[-1].position.y, 0.)

    def test_nonhighway_fork_remains_wire_exact_with_both_hypotheses(self):
        self.fork_network()
        message = self.message([self.track(1, y=30.)])
        original = self.predict(message)
        selected = self.predict(message, self.native(True))
        self.assertEqual(len(original.objects[0].kinematics.predicted_paths), 2)
        self.assertEqual(helpers.wire(original), helpers.wire(selected))

    def test_highway_overlap_selects_actual_nonhighway_lane_without_expanding_horizon(self):
        self.network = LaneletNetwork([road(1, -10., 200., 0.),
                                      road(2, -10., 200., 3.5, speed='59')],
                                     MapPredictionConfig(maximum_start_lanelets=3))
        message = self.message([self.track(1, y=2.05)])
        paths = self.predict(message, self.native(True)).objects[0].kinematics.predicted_paths
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].path), 31)
        self.assertAlmostEqual(paths[0].path[-1].position.y, 3.5)

    def test_leaving_geographic_highway_scope_forgets_previous_boundary_choice(self):
        self.network = LaneletNetwork([road(1, -10., 200., 0.),
                                      road(2, -10., 200., 3.5, speed='59')],
                                     MapPredictionConfig(maximum_start_lanelets=3))
        extension = self.native(True)
        message = self.message([self.track(1)])
        for stamp, y, expected_lane_y in ((10., 0., 0.), (10.1, 1.8, 0.),
                                          (10.2, 3.5, 3.5), (10.5, 3.5, 3.5),
                                          (10.6, 1.8, 3.5)):
            with self.subTest(stamp=stamp, y=y):
                message.header.stamp = helpers.rospy.Time.from_sec(stamp)
                message.objects[0].kinematics.pose_with_covariance.pose.position.y = y
                paths = self.predict(message, extension).objects[0].kinematics.predicted_paths
                self.assertEqual(len(paths), 1)
                self.assertAlmostEqual(paths[0].path[-1].position.y, expected_lane_y)

    def seam_network(self):
        self.network = LaneletNetwork([
            road(1, -10., 10., 0., following=(2,)),
            road(2, 10., 30., 0., following=(4, 3), speed='59'),
            road(3, 30., 200., 0., speed='59'),
            LaneletRecord(4, ((30., 0., 0.), (200., 15., 0.)), (),
                          (('speed_limit', '59'),)),
        ], MapPredictionConfig(maximum_start_lanelets=3))

    def paths_at(self, extension, message, stamp, x):
        message.header.stamp = helpers.rospy.Time.from_sec(stamp)
        message.objects[0].kinematics.pose_with_covariance.pose.position.x = x
        return self.predict(message, extension).objects[0].kinematics.predicted_paths

    def test_brief_highway_neighborhood_gap_keeps_one_short_path_then_expires(self):
        self.seam_network()
        extension = self.native(True)
        message = self.message([self.track(1)])
        self.assertEqual(len(self.paths_at(extension, message, 10., 9.)), 1)
        gap = self.paths_at(extension, message, 10.2, 14.)
        self.assertEqual(len(gap), 1)
        self.assertEqual(len(gap[0].path), 31)
        expired = self.paths_at(extension, message, 10.31, 15.)
        self.assertEqual(len(expired), 2)
        self.assertEqual([len(path.path) for path in expired], [31, 31])

    def test_scope_grace_does_not_leak_to_another_uuid(self):
        self.seam_network()
        extension = self.native(True)
        message = self.message([self.track(1)])
        self.paths_at(extension, message, 10., 9.)
        message.objects[0].object_id.uuid = list(bytes([2]) * 16)
        self.assertEqual(len(self.paths_at(extension, message, 10.2, 14.)), 2)

    def test_scope_grace_is_cleared_when_uuid_disappears(self):
        self.seam_network()
        extension = self.native(True)
        message = self.message([self.track(1)])
        self.paths_at(extension, message, 10., 9.)
        empty = self.message([])
        empty.header.stamp = helpers.rospy.Time.from_sec(10.1)
        self.predict(empty, extension)
        self.assertEqual(len(self.paths_at(extension, message, 10.2, 14.)), 2)

    def test_scope_grace_is_cleared_on_clock_regression(self):
        self.seam_network()
        extension = self.native(True)
        message = self.message([self.track(1)])
        self.paths_at(extension, message, 10., 9.)
        self.assertEqual(len(self.paths_at(extension, message, 9.9, 14.)), 2)

    def test_scope_grace_is_not_extended_by_each_new_gap_frame(self):
        self.seam_network()
        extension = self.native(True)
        message = self.message([self.track(1)])
        self.paths_at(extension, message, 10., 9.)
        self.assertEqual(len(self.paths_at(extension, message, 10.15, 14.)), 1)
        self.assertEqual(len(self.paths_at(extension, message, 10.28, 14.5)), 1)
        self.assertEqual(len(self.paths_at(extension, message, 10.4, 15.)), 2)

    def test_mixed_objects_preserve_nonhighway_nonvehicles_fallback_order_and_input(self):
        self.fork_network()
        message = self.message([
            self.track(1), self.track(2, y=30.),
            self.track(3, label=ObjectClassification.PEDESTRIAN),
            self.track(4, label=ObjectClassification.UNKNOWN), self.track(5, y=100.),
        ])
        before = helpers.wire(message)
        original = self.predict(message, self.native(False))
        selected = self.predict(message, self.native(True))
        self.assertEqual([len(o.kinematics.predicted_paths) for o in original.objects], [2, 2, 1, 1, 1])
        self.assertEqual([len(o.kinematics.predicted_paths) for o in selected.objects], [1, 2, 1, 1, 1])
        self.assert_current_fields_unchanged(original.objects[0], selected.objects[0])
        for old, new in zip(original.objects[1:], selected.objects[1:]):
            self.assertEqual(helpers.wire(old), helpers.wire(new))
        self.assertEqual(before, helpers.wire(message))

    def test_direct_constructor_default_off_is_wire_exact_compatibility(self):
        self.fork_network()
        message = self.message([self.track(1)])
        implicit = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 10.),
            MixedHorizonSelector(self.network, entry_confirm_sec=0.,
                                 current_lanelet_only=True), mode='native')
        self.assertEqual(helpers.wire(self.predict(message, implicit)),
                         helpers.wire(self.predict(message, self.native(False))))
        self.assertEqual(len(self.predict(message, implicit).objects[0].kinematics.predicted_paths), 2)


class HighwayCurrentLaneStartupTest(unittest.TestCase):
    setUp = helpers.MixedHorizonStartupTest.setUp
    start = helpers.MixedHorizonStartupTest.start

    def test_yaml_enables_current_lane_and_false_override_preserves_horizon(self):
        policy = yaml.safe_load((PACKAGE_ROOT / 'config/map_based_prediction.yaml').read_text())
        self.assertIs(policy['highway_current_lane_only'], True)
        self.assertEqual(policy['highway_lane_switch_margin_m'], .2)
        parameters = {'~' + key: value for key, value in policy.items()}
        parameters['~mixed_horizon_enabled'] = True
        node, _, _, warning, _ = self.start(parameters)
        self.assertTrue(node.mixed_extension.highway_current_lane_only)
        self.assertEqual(node.mixed_extension.extended_horizons[-1], 10.)
        warning.assert_not_called()
        parameters['~highway_current_lane_only'] = False
        node, _, _, warning, _ = self.start(parameters)
        self.assertFalse(node.mixed_extension.highway_current_lane_only)
        self.assertEqual(node.mixed_extension.extended_horizons[-1], 10.)
        warning.assert_not_called()

    def test_invalid_margin_disables_only_current_lane_policy_not_ten_second_horizon(self):
        for value in (-1., 1.01, float('nan'), float('inf'), True):
            with self.subTest(value=value):
                node, publisher, subscriber, _, _ = self.start({
                    '~highway_prediction_horizon': 10., '~highway_prediction_mode': 'native',
                    '~highway_current_lane_only': True, '~highway_lane_switch_margin_m': value,
                })
                self.assertIsNotNone(node.mixed_extension)
                self.assertFalse(node.mixed_extension.highway_current_lane_only)
                self.assertEqual(node.mixed_extension.extended_horizons[-1], 10.)
                publisher.assert_called_once()
                subscriber.assert_called_once()


def test_current_lane_and_margin_forward_through_all_four_launches():
    names = ('prediction_highway_current_lane_only', 'prediction_highway_lane_switch_margin_m')
    repo = PACKAGE_ROOT.parents[2]
    launches = [
        (PACKAGE_ROOT / 'launch/tracking_prediction.launch', 'object_pipeline.launch'),
        (PACKAGE_ROOT / 'launch/object_pipeline.launch', 'object_prediction.launch'),
        (repo / 'base_ws/src/planning_addons/ajounice_planning_integration/launch/'
         'morai_planning_stack.launch', 'object_pipeline.launch'),
    ]
    for path, child in launches:
        launch = ET.parse(path).getroot()
        include = next(item for item in launch.iter('include') if child in item.attrib['file'])
        for name in names:
            assert launch.find("arg[@name='" + name + "']").attrib['default'] == ''
            assert include.find("arg[@name='" + name + "']").attrib['value'] == '$(arg ' + name + ')'
    leaf = ET.parse(PACKAGE_ROOT / 'launch/object_prediction.launch').getroot()
    node = next(item for item in leaf.iter('node') if item.attrib['name'] == 'map_based_prediction')
    config = next(item for item in node.findall('rosparam')
                  if 'map_based_prediction.yaml' in item.attrib.get('file', ''))
    for name in names:
        assert leaf.find("arg[@name='" + name + "']").attrib['default'] == ''
        private_name = name[len('prediction_'):]
        override = node.find("param[@name='" + private_name + "']")
        assert list(node).index(config) < list(node).index(override)
        assert override.attrib['value'] == '$(arg ' + name + ')'
        assert override.attrib['if'] == "$(eval arg('" + name + "') != '')"


def test_actual_roslaunch_leaf_defaults_and_explicit_current_lane_rollback():
    repo = PACKAGE_ROOT.parents[2]
    for override, expected in (('', True), ('false', False)):
        config = ROSLaunchConfig()
        arguments = ['prediction_model:=map_based', 'prediction_mixed_horizon_enabled:=true']
        if override:
            arguments.extend(['prediction_highway_current_lane_only:=' + override,
                              'prediction_highway_lane_switch_margin_m:=0.3'])
        with patch.dict(os.environ, {'AJOUNICE_ROOT': str(repo)}):
            XmlLoader().load(str(PACKAGE_ROOT / 'launch/object_prediction.launch'),
                             config, argv=arguments, verbose=False)
        assert config.params['/map_based_prediction/highway_current_lane_only'].value is expected
        assert config.params['/map_based_prediction/highway_lane_switch_margin_m'].value == (.3 if override else .2)
        assert config.params['/map_based_prediction/highway_prediction_horizon'].value == 10.
        assert config.params['/map_based_prediction/prediction_horizon'].value == 3.
