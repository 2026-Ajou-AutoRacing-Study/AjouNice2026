#!/usr/bin/env python3
"""Wire-level optional extension and failure isolation, without a ROS master."""
import copy
from contextlib import ExitStack
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rospy
from autoware_perception_msgs.msg import ObjectClassification, TrackedObject, TrackedObjects

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_based_prediction_node import (
    MapBasedPredictionNode, MixedHorizonExtension, make_map_based_predicted_objects,
)
from perception_object_pipeline.kinematic_prediction import build_prediction_horizons, yaw_to_quaternion, predict_pose
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord, MapPredictionConfig, PredictedTrajectory
from perception_object_pipeline.mixed_horizon import MixedHorizonSelector, build_highway_horizons


def wire(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return stream.getvalue()


class MixedHorizonNodeTest(unittest.TestCase):
    def setUp(self):
        self.base = build_prediction_horizons(.1, 3.)
        self.long = build_prediction_horizons(.1, 6.)
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),)),
            LaneletRecord(2, ((-10., 30., 0.), (200., 30., 0.)), (), (('speed_limit', '50'),))],
            MapPredictionConfig(maximum_start_lanelets=1))
        self.selector = SimpleNamespace(
            highway_lanelet_ids=frozenset({1}), prune=lambda *args: None,
            decide=lambda *args: SimpleNamespace(extend=True, reason='test_highway'))
        self.extension = MixedHorizonExtension(self.network, self.base, self.long, self.selector)

    def track(self, identity, y=0., label=ObjectClassification.CAR, speed=10.):
        track = TrackedObject()
        track.object_id.uuid = list(bytes([identity]) * 16)
        track.classification = [ObjectClassification(label=label, probability=.99)]
        track.kinematics.pose_with_covariance.pose.position.y = y
        track.kinematics.pose_with_covariance.pose.orientation = yaw_to_quaternion(0.)
        track.kinematics.twist_with_covariance.twist.linear.x = speed
        track.existence_probability = .85
        return track

    def message(self, tracks):
        message = TrackedObjects()
        message.header.frame_id = 'map'
        message.header.stamp = rospy.Time.from_sec(10.)
        message.objects = tracks
        return message

    def predict(self, message, extension=None):
        return make_map_based_predicted_objects(message, self.base, self.network, extension)[0]

    def test_off_and_ineligible_on_are_wire_exact_and_input_immutable(self):
        message = self.message([self.track(1), self.track(2, y=30.),
                                self.track(3, label=ObjectClassification.PEDESTRIAN)])
        original = wire(message)
        baseline = self.predict(message)
        self.selector.decide = lambda *args: SimpleNamespace(extend=False, reason='not_highway')
        self.assertEqual(wire(baseline), wire(self.predict(message, self.extension)))
        self.assertEqual(original, wire(message))

    def test_mixed_objects_preserve_prefix_confidence_current_state_and_order(self):
        message = self.message([self.track(1), self.track(2, y=30.),
                                self.track(3, label=ObjectClassification.PEDESTRIAN)])
        baseline = self.predict(message)
        output = self.predict(message, self.extension)
        self.assertEqual([len(o.kinematics.predicted_paths[0].path) for o in output.objects], [61, 31, 31])
        self.assertEqual(self.extension.extended_objects, 1)
        self.assertEqual(wire(output.objects[1]), wire(baseline.objects[1]))
        self.assertEqual(wire(output.objects[2]), wire(baseline.objects[2]))
        for original, extended in zip(baseline.objects, output.objects):
            trimmed = copy.deepcopy(extended)
            for old_path, new_path in zip(original.kinematics.predicted_paths, trimmed.kinematics.predicted_paths):
                self.assertEqual(old_path.confidence, new_path.confidence)
                new_path.path = new_path.path[:len(old_path.path)]
            self.assertEqual(wire(original), wire(trimmed))

    def test_highway8_and_other3_are_wire_exact_when_suffix_is_removed(self):
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1), self.track(2, y=30.),
                                self.track(3, label=ObjectClassification.PEDESTRIAN),
                                self.track(4, speed=-10.)])
        before = wire(message)
        baseline = self.predict(message)
        output = self.predict(message, extension)
        self.assertEqual([len(o.kinematics.predicted_paths[0].path) for o in output.objects],
                         [81, 31, 31, 31])
        self.assertEqual(output.objects[0].kinematics.predicted_paths[0].path[-1].position.x, 80.)
        for old, new in zip(baseline.objects, output.objects):
            for old_path, new_path in zip(old.kinematics.predicted_paths,
                                          new.kinematics.predicted_paths):
                self.assertEqual(old_path.confidence, new_path.confidence)
                new_path.path = new_path.path[:31]
        self.assertEqual(wire(baseline), wire(output))
        self.assertEqual(before, wire(message))

    def test_highway8_does_not_guess_a_new_branch_after_three_seconds(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (50., 0., 0.)), (3, 4), (('speed_limit', '150'),)),
            LaneletRecord(3, ((50., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),)),
            LaneletRecord(4, ((50., 0., 0.), (200., 20., 0.)), (), (('speed_limit', '150'),)),
        ], MapPredictionConfig(maximum_start_lanelets=1))
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1)])
        baseline = self.predict(message)
        self.assertEqual(wire(baseline), wire(self.predict(message, extension)))
        self.assertEqual(extension.reasons.get('route_not_extendable'), 1)

    def test_highway8_preserves_all_preselected_branch_hypotheses(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (10., 0., 0.)), (3, 4), (('speed_limit', '150'),)),
            LaneletRecord(3, ((10., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),)),
            LaneletRecord(4, ((10., 0., 0.), (200., 20., 0.)), (), (('speed_limit', '150'),)),
        ], MapPredictionConfig(maximum_start_lanelets=1))
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1)])
        baseline = self.predict(message)
        output = self.predict(message, extension)
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths), 2)
        for old, new in zip(baseline.objects[0].kinematics.predicted_paths,
                             output.objects[0].kinematics.predicted_paths):
            self.assertEqual(len(new.path), 81)
            self.assertEqual(new.confidence, old.confidence)
            new.path = new.path[:31]
        self.assertEqual(wire(output), wire(baseline))

    def test_highway8_never_extends_into_a_nonhighway_successor(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (50., 0., 0.)), (2,), (('speed_limit', '150'),)),
            LaneletRecord(2, ((50., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '50'),)),
        ], MapPredictionConfig(maximum_start_lanelets=1))
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1)])
        self.assertEqual(wire(self.predict(message, extension)), wire(self.predict(message)))
        self.assertEqual(extension.reasons.get('route_not_extendable'), 1)

    def test_highway8_entry_confirmation_and_immediate_exit_remain_per_object(self):
        selector = MixedHorizonSelector(self.network)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1)])

        def count(stamp, y):
            message.header.stamp = rospy.Time.from_sec(stamp)
            message.objects[0].kinematics.pose_with_covariance.pose.position.y = y
            return len(self.predict(message, extension).objects[0].kinematics.predicted_paths[0].path)

        self.assertEqual(count(10., 0.), 31)
        self.assertEqual(count(10.49, 0.), 31)
        self.assertEqual(count(10.5, 0.), 81)
        self.assertEqual(count(10.6, 30.), 31)
        self.assertEqual(count(10.7, 0.), 31)
        self.assertEqual(count(11.2, 0.), 81)

    def test_highway8_insufficient_geometry_never_pads_to_81_poses(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (70., 0., 0.)), (), (('speed_limit', '150'),)),
        ], MapPredictionConfig(maximum_start_lanelets=1))
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=0.)
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), selector)
        message = self.message([self.track(1)])
        output = self.predict(message, extension)
        self.assertEqual(wire(output), wire(self.predict(message)))
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 31)

    def test_highway8_memory_error_is_optional_and_preserves_input_and_baseline(self):
        extension = MixedHorizonExtension(
            self.network, self.base, build_highway_horizons(self.base, .1, 8.), self.selector)
        message = self.message([self.track(1), self.track(2, y=30.)])
        before = wire(message)
        with patch.object(self.network, 'extend_trajectories', side_effect=MemoryError('fault injection')):
            with patch('map_based_prediction_node.rospy.logwarn_throttle'):
                self.assertEqual(wire(self.predict(message, extension)), wire(self.predict(message)))
        self.assertEqual(before, wire(message))
        self.assertEqual(extension.extension_errors, 2)

    def test_selector_exception_retains_object_and_other_objects(self):
        message = self.message([self.track(1), self.track(2, y=30.)])
        self.selector.decide = lambda *args: (_ for _ in ()).throw(RuntimeError('fault injection'))
        with patch('map_based_prediction_node.rospy.logwarn_throttle'):
            output = self.predict(message, self.extension)
        self.assertEqual(wire(output), wire(self.predict(message)))
        self.assertEqual(self.extension.extension_errors, 2)

    def test_prune_exception_disables_extension_for_whole_frame(self):
        message = self.message([self.track(1)])
        self.selector.prune = lambda *args: (_ for _ in ()).throw(RuntimeError('fault injection'))
        with patch('map_based_prediction_node.rospy.logwarn_throttle'):
            self.assertEqual(wire(self.predict(message, self.extension)), wire(self.predict(message)))
        self.assertEqual(self.extension.extension_errors, 1)

    def test_optional_prune_failure_does_not_permanently_disable_following_frames(self):
        message = self.message([self.track(1)])
        self.selector.prune = lambda *args: (_ for _ in ()).throw(RuntimeError('fault injection'))
        with patch('map_based_prediction_node.rospy.logwarn_throttle'):
            self.assertEqual(wire(self.predict(message, self.extension)), wire(self.predict(message)))
        self.selector.prune = lambda *args: None
        output = self.predict(message, self.extension)
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 61)
        self.assertEqual(self.extension.extension_errors, 0)

    def test_temporary_no_map_path_resets_real_selector_entry_confirmation(self):
        selector = MixedHorizonSelector(self.network)
        extension = MixedHorizonExtension(self.network, self.base, self.long, selector)
        message = self.message([self.track(1)])

        def poses_at(stamp):
            message.header.stamp = rospy.Time.from_sec(stamp)
            return len(self.predict(message, extension).objects[0].kinematics.predicted_paths[0].path)

        self.assertEqual(poses_at(10.), 31)
        self.assertEqual(poses_at(10.5), 61)
        with patch.object(self.network, 'predict', return_value=()):
            self.assertEqual(poses_at(10.6), 31)
        self.assertEqual(poses_at(10.7), 31)
        self.assertEqual(poses_at(11.2), 61)

    def test_temporary_nonvehicle_class_resets_real_selector_entry_confirmation(self):
        selector = MixedHorizonSelector(self.network)
        extension = MixedHorizonExtension(self.network, self.base, self.long, selector)
        message = self.message([self.track(1)])

        def poses_at(stamp):
            message.header.stamp = rospy.Time.from_sec(stamp)
            return len(self.predict(message, extension).objects[0].kinematics.predicted_paths[0].path)

        self.assertEqual(poses_at(10.), 31)
        self.assertEqual(poses_at(10.5), 61)
        message.objects[0].classification[0].label = ObjectClassification.UNKNOWN
        self.assertEqual(poses_at(10.6), 31)
        message.objects[0].classification[0].label = ObjectClassification.CAR
        self.assertEqual(poses_at(10.7), 31)
        self.assertEqual(poses_at(11.2), 61)

    def test_extension_exception_never_suppresses_original_object(self):
        message = self.message([self.track(1)])
        with patch.object(self.network, 'extend_trajectories', side_effect=RuntimeError('fault injection')):
            with patch('map_based_prediction_node.rospy.logwarn_throttle'):
                self.assertEqual(wire(self.predict(message, self.extension)), wire(self.predict(message)))

    def test_malformed_extension_is_rejected_without_changing_baseline(self):
        message = self.message([self.track(1)])
        with patch.object(self.network, 'extend_trajectories', return_value=()):
            self.assertEqual(wire(self.predict(message, self.extension)), wire(self.predict(message)))

    def test_required_predictor_failure_is_not_masked_as_optional_fallback(self):
        message = self.message([self.track(1)])
        with patch.object(self.network, 'predict', side_effect=ValueError('required map failure')):
            with self.assertRaisesRegex(ValueError, 'required map failure'):
                self.predict(message, self.extension)

    def test_empty_frame_still_publishes_and_prunes(self):
        message = self.message([])
        self.assertEqual(wire(self.predict(message, self.extension)), wire(self.predict(message)))


class NativeHighwayHorizonTest(unittest.TestCase):
    # Reuse only message builders, not the legacy append assertions.
    setUp = MixedHorizonNodeTest.setUp
    track = MixedHorizonNodeTest.track
    message = MixedHorizonNodeTest.message
    predict = MixedHorizonNodeTest.predict

    def native(self, *, confirm=0.):
        selector = MixedHorizonSelector(self.network, entry_confirm_sec=confirm,
                                        current_lanelet_only=True)
        return MixedHorizonExtension(self.network, self.base,
                                     build_highway_horizons(self.base, .1, 8.),
                                     selector, mode='native')

    def replace_network(self, records):
        self.network = LaneletNetwork(records, MapPredictionConfig(maximum_start_lanelets=1))

    def assert_fields_unchanged(self, before, after):
        old, new = copy.deepcopy(before), copy.deepcopy(after)
        old.kinematics.predicted_paths = []
        new.kinematics.predicted_paths = []
        self.assertEqual(wire(old), wire(new))

    def test_native_highway8_preserves_current_fields_input_and_all_nonhighway_objects(self):
        message = self.message([self.track(1), self.track(2, y=30.),
                                self.track(3, label=ObjectClassification.PEDESTRIAN),
                                self.track(4, label=ObjectClassification.UNKNOWN)])
        before = wire(message)
        baseline = self.predict(message)
        extension = self.native()
        output, mapped, fallback, count = make_map_based_predicted_objects(
            message, self.base, self.network, extension)
        self.assertEqual([len(o.kinematics.predicted_paths[0].path) for o in output.objects],
                         [81, 31, 31, 31])
        self.assertEqual((mapped, fallback, count), (2, 2, 4))
        for old, new in zip(baseline.objects, output.objects):
            self.assert_fields_unchanged(old, new)
            for path in new.kinematics.predicted_paths:
                self.assertTrue(all(all(math.isfinite(v) for v in
                                       (p.position.x, p.position.y, p.position.z,
                                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
                                    for p in path.path))
        for old, new in zip(baseline.objects[1:], output.objects[1:]):
            self.assertEqual(wire(old), wire(new))
        self.assertEqual(before, wire(message))
        self.assertEqual((extension.extended_objects, extension.native_map_objects,
                          extension.native_kinematic_objects), (1, 1, 0))

    def test_new_fork_after_three_seconds_is_predicted_natively_not_shortened(self):
        self.replace_network([
            LaneletRecord(1, ((-10., 0., 0.), (50., 0., 0.)), (3, 4), (('speed_limit', '150'),)),
            LaneletRecord(3, ((50., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),)),
            LaneletRecord(4, ((50., 0., 0.), (200., 20., 0.)), (), (('speed_limit', '50'),)),
        ])
        message = self.message([self.track(1)])
        baseline = self.predict(message)
        extension = self.native()
        output = self.predict(message, extension)
        self.assertEqual(len(baseline.objects[0].kinematics.predicted_paths), 1)
        self.assertEqual([len(p.path) for p in output.objects[0].kinematics.predicted_paths], [81, 81])
        self.assert_fields_unchanged(baseline.objects[0], output.objects[0])
        self.assertEqual(extension.native_map_objects, 1)
        self.assertEqual(extension.native_kinematic_objects, 0)

    def test_future_nonhighway_lane_does_not_change_current_highway_region(self):
        self.replace_network([
            LaneletRecord(1, ((-10., 0., 0.), (10., 0., 0.)), (2,), (('speed_limit', '150'),)),
            LaneletRecord(2, ((10., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '50'),)),
        ])
        message = self.message([self.track(1)])
        extension = self.native()
        output = self.predict(message, extension)
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 81)
        self.assertEqual(extension.native_map_objects, 1)

    def test_map_end_retains_exact_three_second_map_instead_of_long_ctrv(self):
        self.replace_network([
            LaneletRecord(1, ((-10., 0., 0.), (70., 0., 0.)), (), (('speed_limit', '150'),)),
        ])
        tracked = self.track(1)
        tracked.kinematics.twist_with_covariance.twist.angular.z = .2
        message = self.message([tracked])
        baseline = self.predict(message)
        extension = self.native()
        output, mapped, fallback, _ = make_map_based_predicted_objects(
            message, self.base, self.network, extension)
        self.assertEqual(wire(output), wire(baseline))
        self.assertEqual((mapped, fallback), (1, 0))
        self.assertEqual(extension.extended_objects, 0)
        self.assertEqual(extension.native_short_objects, 1)
        self.assertEqual(extension.reasons.get('native_short_no_map_paths'), 1)

    def test_stopped_slow_and_reverse_keep_baseline_ctrv_three_seconds(self):
        for speed in (0., .1, .49, -10.):
            with self.subTest(speed=speed):
                tracked = self.track(1, speed=speed)
                message = self.message([tracked])
                baseline = self.predict(message)
                extension = self.native()
                output = self.predict(message, extension)
                self.assertEqual(wire(output), wire(baseline))
                self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 31)
                self.assertEqual(extension.extended_objects, 0)

    def test_native_never_expands_nonvehicle_even_with_supplied_map_context(self):
        extension = self.native()
        message = self.message([self.track(1, label=ObjectClassification.PEDESTRIAN)])
        extension.begin(message)
        trajectories = self.network.predict(0., 0., 0., 0., 10., self.base)
        self.assertIs(extension.extend(message.objects[0], trajectories, message.header.stamp), trajectories)
        self.assertEqual(extension.output_horizons, tuple(self.base))
        self.assertEqual(extension.extended_objects, 0)

    def test_map_only_two_seconds_keeps_exact_baseline_ctrv_three_seconds(self):
        self.replace_network([
            LaneletRecord(1, ((-10., 0., 0.), (20., 0., 0.)), (), (('speed_limit', '150'),)),
            LaneletRecord(2, ((-10., 30., 0.), (20., 30., 0.)), (), (('speed_limit', '50'),)),
        ])
        message = self.message([self.track(1), self.track(2, y=30.)])
        baseline = self.predict(message)
        extension = self.native()
        output = self.predict(message, extension)
        self.assertEqual([len(o.kinematics.predicted_paths[0].path) for o in output.objects], [31, 31])
        self.assertEqual(wire(baseline), wire(output))
        self.assertEqual(extension.extended_objects, 0)

    def test_native_rejects_the_existing_half_meter_endpoint_clamp_tolerance(self):
        self.replace_network([
            LaneletRecord(1, ((-10., 0., 0.), (79.8, 0., 0.)), (), (('speed_limit', '150'),)),
        ])
        extension = self.native()
        native = self.network.predict(0., 0., 0., 0., 10., extension.extended_horizons)
        self.assertTrue(native)
        self.assertEqual(native[0].poses[-1][0], 79.8)
        output = self.predict(self.message([self.track(1)]), extension)
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 31)
        self.assertEqual(output.objects[0].kinematics.predicted_paths[0].path[-1].position.x, 30.)
        self.assertEqual(extension.reasons.get('native_short_invalid_map_paths'), 1)

    def test_malformed_or_padded_native_map_samples_preserve_short_map(self):
        extension = self.native()
        genuine = self.network.predict(0., 0., 0., 0., 10., extension.extended_horizons)[0]
        variants = [
            PredictedTrajectory(genuine.poses[:-1], 1., genuine.lanelet_ids),
            PredictedTrajectory(genuine.poses[:-1] + ((float('nan'), 0., 0., 0.),), 1., genuine.lanelet_ids),
            PredictedTrajectory(genuine.poses[:-1] + (genuine.poses[-2],), 1., genuine.lanelet_ids),
            PredictedTrajectory(genuine.poses, float('nan'), genuine.lanelet_ids),
            PredictedTrajectory(genuine.poses, 2., genuine.lanelet_ids),
        ]
        original = self.network.predict
        for malformed in variants:
            with self.subTest(malformed=malformed):
                message = self.message([self.track(1)])
                before = wire(message)
                def predict(**values):
                    return (malformed,) if values['horizons'][-1] > 3. else original(**values)
                with patch.object(self.network, 'predict', side_effect=predict):
                    output = self.predict(message, extension)
                path = output.objects[0].kinematics.predicted_paths[0]
                self.assertEqual(len(path.path), 31)
                self.assertEqual(path.path[-1].position.x, 30.)
                self.assertEqual(extension.native_short_objects, 1)
                self.assertEqual(before, wire(message))

    def test_native_map_exception_and_memory_error_preserve_three_second_map(self):
        original = self.network.predict
        for error in (RuntimeError('native failure'), MemoryError('fault injection')):
            with self.subTest(error=error):
                extension = self.native()
                def predict(**values):
                    if values['horizons'][-1] > 3.:
                        raise error
                    return original(**values)
                message = self.message([self.track(1)])
                with patch.object(self.network, 'predict', side_effect=predict), \
                        patch('map_based_prediction_node.rospy.logwarn_throttle'):
                    output = self.predict(message, extension)
                self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 31)
                self.assertEqual(extension.extension_errors, 1)
                self.assertEqual(extension.native_short_objects, 1)
                self.assertEqual(extension.reasons.get('native_short_exception'), 1)

    def test_native_confirmation_and_current_lane_exit_still_apply(self):
        extension = self.native(confirm=.5)
        message = self.message([self.track(1)])
        def count(stamp, y=0.):
            message.header.stamp = rospy.Time.from_sec(stamp)
            message.objects[0].kinematics.pose_with_covariance.pose.position.y = y
            return len(self.predict(message, extension).objects[0].kinematics.predicted_paths[0].path)
        self.assertEqual(count(10.), 31)
        self.assertEqual(count(10.5), 81)
        self.assertEqual(count(10.6, 30.), 31)
        self.assertEqual(count(10.7), 31)
        self.assertEqual(count(11.2), 81)

    def test_one_invalid_native_motion_does_not_reset_other_confirmed_tracks(self):
        extension = self.native(confirm=.5)
        message = self.message([self.track(1), self.track(2)])
        self.predict(message, extension)
        message.header.stamp = rospy.Time.from_sec(10.5)
        self.assertEqual([len(o.kinematics.predicted_paths[0].path)
                          for o in self.predict(message, extension).objects], [81, 81])
        message.header.stamp = rospy.Time.from_sec(10.6)
        message.objects[0].kinematics.pose_with_covariance.pose.position.z = float('nan')
        self.assertEqual([len(o.kinematics.predicted_paths[0].path)
                          for o in self.predict(message, extension).objects], [31, 81])


class MixedHorizonStartupTest(unittest.TestCase):
    def setUp(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),))
        ], MapPredictionConfig(maximum_start_lanelets=1))

    def start(self, parameters=None, network=None, load_error=None, import_error=False):
        values = {'~map_file': 'unused.osm', '~projector_file': 'unused.yaml',
                  '~mixed_horizon_enabled': True,
                  # Isolate optional extension from the separate subscriber-
                  # activated diagnostics publisher; covered explicitly below.
                  '~diagnostics_auto_on_subscribe': False}
        values.update(parameters or {})

        def parameter(name, *defaults):
            return values[name] if name in values else defaults[0]

        with ExitStack() as patches:
            patches.enter_context(patch('map_based_prediction_node.rospy.get_param', side_effect=parameter))
            publisher = patches.enter_context(patch('map_based_prediction_node.rospy.Publisher'))
            subscriber = patches.enter_context(patch('map_based_prediction_node.rospy.Subscriber'))
            patches.enter_context(patch('map_based_prediction_node.rospy.loginfo'))
            warning = patches.enter_context(patch('map_based_prediction_node.rospy.logwarn'))
            loader = patches.enter_context(patch('map_based_prediction_node.LaneletNetwork.load',
                                                  return_value=network or self.network,
                                                  side_effect=load_error))
            if import_error:
                import builtins
                original_import = builtins.__import__

                def optional_import(name, *args, **kwargs):
                    if name == 'perception_object_pipeline.mixed_horizon':
                        raise ImportError('optional module unavailable')
                    return original_import(name, *args, **kwargs)

                patches.enter_context(patch('builtins.__import__', side_effect=optional_import))
            node = MapBasedPredictionNode()
            return node, publisher, subscriber, warning, loader

    def test_optional_on_has_only_existing_required_map_load(self):
        node, publisher, subscriber, warning, loader = self.start()
        self.assertIsNotNone(node.mixed_extension)
        self.assertEqual(node.mixed_extension.selector.highway_lanelet_ids, frozenset({1}))
        loader.assert_called_once()
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_explicit_highway8_starts_with_80_future_samples(self):
        node, publisher, subscriber, warning, _ = self.start({'~highway_prediction_horizon': 8.})
        self.assertIsNotNone(node.mixed_extension)
        self.assertEqual(len(node.mixed_extension.extended_horizons), 80)
        self.assertEqual(node.mixed_extension.extended_horizons[-1], 8.)
        self.assertEqual(node.mixed_extension.extended_horizons[:30], tuple(node.horizons))
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_native_mode_uses_current_lane_scope(self):
        node, publisher, subscriber, warning, _ = self.start(
            {'~highway_prediction_horizon': 8., '~highway_prediction_mode': 'native'})
        self.assertEqual(node.mixed_extension.mode, 'native')
        self.assertTrue(node.mixed_extension.selector.current_lanelet_only)
        self.assertEqual(node.mixed_extension.selector.entry_confirm_sec, 0.)
        self.assertEqual(len(node.mixed_extension.extended_horizons), 80)
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_native_rotary_baseline_can_be_configured_to_six_seconds(self):
        node, _, _, warning, _ = self.start(
            {'~prediction_horizon': 6., '~highway_prediction_horizon': 8.,
             '~highway_prediction_mode': 'native'})
        self.assertEqual(node.horizons[-1], 6.)
        self.assertEqual(node.mixed_extension.extended_horizons[-1], 8.)
        warning.assert_not_called()

    def test_native_startup_has_eight_second_paths_on_first_highway_frame(self):
        node, _, _, _, _ = self.start(
            {'~highway_prediction_horizon': 8., '~highway_prediction_mode': 'native'})
        track = MixedHorizonNodeTest.track(self, 1)
        message = MixedHorizonNodeTest.message(self, [track])
        output = make_map_based_predicted_objects(
            message, node.horizons, node.network, node.mixed_extension)[0]
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 81)

    def test_invalid_optional_mode_keeps_ordinary_predictor_alive(self):
        for mode in ('invalid', None, True, [], 8):
            with self.subTest(mode=mode):
                node, publisher, subscriber, warning, _ = self.start(
                    {'~highway_prediction_mode': mode})
                self.assertIsNone(node.mixed_extension)
                publisher.assert_called_once()
                subscriber.assert_called_once()
                warning.assert_called_once()

    def test_highway8_coexists_with_subscriber_activated_diagnostics(self):
        node, publisher, subscriber, warning, _ = self.start(
            {'~highway_prediction_horizon': 8., '~diagnostics_auto_on_subscribe': True})
        self.assertEqual(node.mixed_extension.extended_horizons[-1], 8.)
        self.assertEqual(publisher.call_count, 2)
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_invalid_optional_highway_horizon_still_starts_baseline(self):
        for value in (None, 'invalid', True, float('nan'), float('inf'), -1., 0.,
                      2.9, 3., 8.05, 10., 1.e308):
            with self.subTest(value=value):
                node, publisher, subscriber, warning, loader = self.start(
                    {'~highway_prediction_horizon': value})
                self.assertIsNone(node.mixed_extension)
                self.assertEqual(node.horizons[-1], 3.)
                loader.assert_called_once()
                publisher.assert_called_once()
                subscriber.assert_called_once()
                warning.assert_called_once()

    def test_invalid_optional_horizon_is_not_read_when_feature_is_off(self):
        node, publisher, subscriber, warning, _ = self.start(
            {'~mixed_horizon_enabled': False, '~highway_prediction_horizon': float('inf')})
        self.assertIsNone(node.mixed_extension)
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_missing_optional_metadata_still_starts_ordinary_predictor(self):
        source = LaneletNetwork([LaneletRecord(1, ((-10., 0., 0.), (200., 0., 0.)))])
        node, publisher, subscriber, warning, _ = self.start(network=source)
        self.assertIsNone(node.mixed_extension)
        self.assertTrue(node.network.predict(0., 0., 0., 0., 10., node.horizons))
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_called_once()

    def test_missing_optional_module_still_starts_ordinary_predictor(self):
        node, publisher, subscriber, warning, _ = self.start(import_error=True)
        self.assertIsNone(node.mixed_extension)
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_called_once()

    def test_incompatible_optional_horizon_keeps_explicit_baseline_configuration(self):
        for values in ({'~prediction_horizon': 4.0}, {'~prediction_time_step': .2}):
            with self.subTest(values=values):
                node, publisher, subscriber, warning, _ = self.start(values)
                self.assertIsNone(node.mixed_extension)
                self.assertEqual(node.horizons[-1], values.get('~prediction_horizon', 3.0))
                self.assertEqual(node.horizons[0], values.get('~prediction_time_step', .1))
                publisher.assert_called_once()
                subscriber.assert_called_once()
                warning.assert_called_once()

    def test_feature_off_does_not_import_optional_module(self):
        node, publisher, subscriber, warning, _ = self.start(
            {'~mixed_horizon_enabled': False}, import_error=True)
        self.assertIsNone(node.mixed_extension)
        publisher.assert_called_once()
        subscriber.assert_called_once()
        warning.assert_not_called()

    def test_required_map_failure_is_not_hidden_by_optional_mode(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), self.assertRaisesRegex(RuntimeError, 'required map failed'):
                self.start({'~mixed_horizon_enabled': enabled},
                           load_error=RuntimeError('required map failed'))


if __name__ == '__main__':
    unittest.main()
