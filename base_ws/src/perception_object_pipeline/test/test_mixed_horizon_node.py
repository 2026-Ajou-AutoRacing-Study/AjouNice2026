#!/usr/bin/env python3
"""Wire-level optional extension and failure isolation, without a ROS master."""
import copy
from contextlib import ExitStack
import io
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
from perception_object_pipeline.kinematic_prediction import build_prediction_horizons, yaw_to_quaternion
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord, MapPredictionConfig
from perception_object_pipeline.mixed_horizon import MixedHorizonSelector


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


class MixedHorizonStartupTest(unittest.TestCase):
    def setUp(self):
        self.network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (200., 0., 0.)), (), (('speed_limit', '150'),))
        ], MapPredictionConfig(maximum_start_lanelets=1))

    def start(self, parameters=None, network=None, load_error=None, import_error=False):
        values = {'~map_file': 'unused.osm', '~projector_file': 'unused.yaml',
                  '~mixed_horizon_enabled': True}
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
