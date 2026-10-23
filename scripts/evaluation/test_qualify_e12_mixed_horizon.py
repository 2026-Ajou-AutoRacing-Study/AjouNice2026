"""Deterministic protocol tests; no ROS master, simulator or controller."""
from copy import deepcopy
import importlib.util
import unittest

from qualify_e12_mixed_horizon import (
    NODE_PATH, build_message, compare_wire_prefix, qualify_case,
    synthetic_uuid, trim_prefix, wire,
)


class MixedHorizonQualificationTest(unittest.TestCase):
    def output(self, poses=31):
        from autoware_perception_msgs.msg import PredictedObject, PredictedObjects, PredictedPath
        from geometry_msgs.msg import Pose
        message = PredictedObjects()
        message.header.seq = 42
        message.header.frame_id = 'map'
        obj = PredictedObject()
        obj.object_id.uuid = synthetic_uuid('case', 0)
        obj.existence_probability = .7
        obj.shape.dimensions.x = 4.5
        obj.kinematics.initial_twist_with_covariance.covariance[8] = 1.23
        path = PredictedPath()
        path.confidence = .77
        for i in range(poses):
            pose = Pose()
            pose.position.x = float(i)
            pose.orientation.w = 1.
            path.path.append(pose)
        obj.kinematics.predicted_paths = [path]
        message.objects = [obj]
        return message

    def test_trim_is_copy_and_preserves_every_non_suffix_byte(self):
        baseline, mixed = self.output(), self.output(61)
        before = wire(mixed)
        extended, first, second = compare_wire_prefix(baseline, mixed)
        self.assertEqual(extended, 1)
        self.assertEqual(first, second)
        self.assertEqual(wire(mixed), before)
        self.assertEqual(len(trim_prefix(mixed).objects[0].kinematics.predicted_paths[0].path), 31)

    def test_header_uuid_class_state_confidence_and_prefix_differences_are_rejected(self):
        baseline = self.output()
        mutations = [
            lambda m: setattr(m.header, 'frame_id', 'wrong'),
            lambda m: setattr(m.header, 'seq', 43),
            lambda m: m.objects[0].object_id.uuid.__setitem__(0, 0),
            lambda m: setattr(m.objects[0], 'existence_probability', .9),
            lambda m: setattr(m.objects[0].shape.dimensions, 'x', 10.),
            lambda m: m.objects[0].kinematics.initial_twist_with_covariance.covariance.__setitem__(8, 2.),
            lambda m: setattr(m.objects[0].kinematics.predicted_paths[0], 'confidence', .78),
            lambda m: setattr(m.objects[0].kinematics.predicted_paths[0].path[3].position, 'x', 99.),
        ]
        for mutation in mutations:
            mixed = self.output(61)
            mutation(mixed)
            with self.assertRaisesRegex(ValueError, 'Serialized prefix'):
                compare_wire_prefix(baseline, mixed)

    def test_partial_extension_and_truncated_paths_are_rejected(self):
        baseline, mixed = self.output(), self.output(61)
        baseline.objects[0].kinematics.predicted_paths.append(
            deepcopy(baseline.objects[0].kinematics.predicted_paths[0]))
        mixed.objects[0].kinematics.predicted_paths.append(
            deepcopy(baseline.objects[0].kinematics.predicted_paths[0]))
        with self.assertRaisesRegex(ValueError, 'Partially extended'):
            compare_wire_prefix(baseline, mixed)
        with self.assertRaisesRegex(ValueError, 'invalid output length'):
            compare_wire_prefix(self.output(), self.output(60))

    def test_synthetic_uuid_is_stable_nonzero_and_case_isolated(self):
        self.assertEqual(synthetic_uuid('a', 0), synthetic_uuid('a', '0'))
        self.assertNotEqual(synthetic_uuid('a', 0), synthetic_uuid('b', 0))
        self.assertNotEqual(synthetic_uuid('a', 0), synthetic_uuid('a', 1))
        self.assertEqual(len(synthetic_uuid('a', 0)), 16)
        self.assertTrue(any(synthetic_uuid('a', 0)))

    def track(self, identity, label, velocity=(5., 0.)):
        return dict(id=identity, tracker_label=label, label={0: 2, 1: 0, 3: 1}[label],
            confirmed=True, associated=True, center=[10., 0., 0.], yaw=0.,
            velocity=list(velocity), yaw_rate=.1, size=[4., 2., 1.5], score=.7)

    def test_message_adapter_preserves_source_order_shapes_and_unknown_stationary(self):
        raw = dict(stamp=1., native_index=9, tracks=[self.track(2, 0), self.track(1, 1),
                                                   self.track(3, 3)])
        normalized = dict(objects=[dict(id='1', center=[20., 30., 0.], yaw=0.,
            body_velocity=[5., 0.], yaw_rate=.1)])
        tf = dict(parent='map', child='morai_map', xyz=[10., 30., 0.], rpy=[0., 0., 0.])
        message = build_message('test', 0, raw, normalized, tf)
        self.assertEqual(message.header.seq, 9)
        self.assertEqual(message.header.stamp.to_sec(), 1.)
        self.assertEqual([o.classification[0].label for o in message.objects], [0, 1, 7])
        unknown = message.objects[0]
        self.assertEqual(unknown.kinematics.twist_with_covariance.twist.linear.x, 0.)
        self.assertEqual(unknown.kinematics.twist_with_covariance.twist.angular.z, 0.)
        self.assertEqual(unknown.shape.dimensions.x, 4.)
        self.assertEqual(message.objects[1].kinematics.pose_with_covariance.pose.position.x, 20.)

    def test_real_public_converter_complete_case_with_dwell_and_all_classes(self):
        from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord
        spec = importlib.util.spec_from_file_location('qualification_test_actual_map_node', NODE_PATH)
        node = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(node)
        network = LaneletNetwork((LaneletRecord(1, ((0., 0., 0.), (100., 0., 0.)), (),
            (('subtype', 'road'), ('speed_limit', '150'))),))
        frames = []
        for index in range(8):
            tracks = [self.track(1, 1), self.track(2, 0), self.track(3, 3)]
            tracks[0]['center'][0] += .5*index
            frames.append(dict(stamp=index/10., native_index=index, token=str(index),
                tracks=tracks, scoring_eligible=True, gt=[]))
        transform = dict(parent='map', child='morai_map', xyz=[0., 0., 0.], rpy=[0., 0., 0.])
        summary, rows, gt = qualify_case('synthetic', dict(frames=frames, gt_trajectories=[]),
                                       transform, network, node)
        self.assertEqual(summary['counts']['all_input_frames'], 8)
        self.assertEqual(summary['counts']['input_objects'], 24)
        self.assertEqual(summary['counts']['extended_objects'], 3)
        self.assertEqual(summary['decision_reasons']['entry_confirmation'], 5)
        self.assertEqual(summary['decision_reasons']['highway_confirmed'], 3)
        self.assertTrue(summary['serialized_prefix_exact'])
        self.assertEqual(len(rows), 8)
        self.assertEqual(gt, [])


if __name__ == '__main__':
    unittest.main()
