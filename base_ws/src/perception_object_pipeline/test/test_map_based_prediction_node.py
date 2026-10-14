#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from autoware_perception_msgs.msg import (
    ObjectClassification,
    TrackedObject,
    TrackedObjects,
)

from map_based_prediction_node import make_map_based_predicted_objects
from perception_object_pipeline.kinematic_prediction import yaw_to_quaternion
from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    LaneletRecord,
    MapPredictionConfig,
)


class MapBasedPredictionNodeTest(unittest.TestCase):
    def _track(self, label, x=1.0, y=0.0, speed=2.0):
        tracked = TrackedObject()
        tracked.classification = [ObjectClassification(label=label, probability=1.0)]
        tracked.kinematics.pose_with_covariance.pose.position.x = x
        tracked.kinematics.pose_with_covariance.pose.position.y = y
        tracked.kinematics.pose_with_covariance.pose.orientation = yaw_to_quaternion(0.0)
        tracked.kinematics.twist_with_covariance.twist.linear.x = speed
        return tracked

    def test_vehicle_map_path_and_pedestrian_fallback(self):
        network = LaneletNetwork(
            [LaneletRecord(1, ((0.0, 0.0, 0.0), (20.0, 0.0, 0.0)))],
            MapPredictionConfig(maximum_start_lanelets=1),
        )
        message = TrackedObjects()
        message.header.frame_id = "map"
        message.objects = [
            self._track(ObjectClassification.CAR),
            self._track(ObjectClassification.PEDESTRIAN, y=4.0),
        ]
        output, mapped, fallback, path_count = make_map_based_predicted_objects(
            message, [0.5, 1.0, 1.5], network
        )
        self.assertEqual((mapped, fallback, path_count), (1, 1, 2))
        self.assertEqual(len(output.objects), 2)
        self.assertEqual(len(output.objects[0].kinematics.predicted_paths[0].path), 4)
        self.assertEqual(len(output.objects[1].kinematics.predicted_paths[0].path), 4)

    def _straight_network(self):
        return LaneletNetwork(
            [LaneletRecord(1, ((-100.0, 0.0, 0.0), (100.0, 0.0, 0.0)))],
            MapPredictionConfig(maximum_start_lanelets=1),
        )

    def _prediction(self, tracked, horizons=(1.0, 2.0, 3.0)):
        message = TrackedObjects()
        message.header.frame_id = "map"
        message.objects = [tracked]
        return make_map_based_predicted_objects(
            message, horizons, self._straight_network()
        )

    def test_signed_reverse_speed_uses_kinematic_fallback_not_forward_lane(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=-2.0)
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (0, 1))
        poses = output.objects[0].kinematics.predicted_paths[0].path
        self.assertEqual([pose.position.x for pose in poses], [10.0, 8.0, 6.0, 4.0])
        self.assertEqual([pose.position.y for pose in poses], [0.0] * 4)
        self.assertEqual(output.objects[0].kinematics.initial_twist_with_covariance.twist.linear.x, -2.0)

    def test_reverse_with_yaw_rate_preserves_ctrv_not_cartesian_cv(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=-2.0)
        tracked.kinematics.twist_with_covariance.twist.angular.z = 0.2
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (0, 1))
        endpoint = output.objects[0].kinematics.predicted_paths[0].path[-1]
        self.assertAlmostEqual(endpoint.position.x, 10.0 - 10.0 * math.sin(0.6))
        self.assertAlmostEqual(endpoint.position.y, 10.0 * (math.cos(0.6) - 1.0))
        self.assertNotEqual(endpoint.position.y, 0.0)

    def test_stopped_vehicle_remains_stationary_even_with_yaw_rate(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=0.0)
        tracked.kinematics.twist_with_covariance.twist.angular.z = 0.2
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (0, 1))
        for pose in output.objects[0].kinematics.predicted_paths[0].path:
            self.assertEqual((pose.position.x, pose.position.y), (10.0, 0.0))

    def test_forward_start_is_not_delayed_by_a_predictor_maturity_gate(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=0.6)
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (1, 0))
        self.assertAlmostEqual(output.objects[0].kinematics.predicted_paths[0].path[-1].position.x, 11.8)

    def test_pi_axis_and_signed_speed_preserve_straight_world_motion(self):
        forward = self._track(ObjectClassification.CAR, x=10.0, speed=2.0)
        equivalent = self._track(ObjectClassification.CAR, x=10.0, speed=-2.0)
        equivalent.kinematics.pose_with_covariance.pose.orientation = yaw_to_quaternion(math.pi)
        forward_output, mapped, fallback, _ = self._prediction(forward)
        equivalent_output, reverse_mapped, reverse_fallback, _ = self._prediction(equivalent)
        self.assertEqual((mapped, fallback), (1, 0))
        self.assertEqual((reverse_mapped, reverse_fallback), (0, 1))
        first = forward_output.objects[0].kinematics.predicted_paths[0].path
        second = equivalent_output.objects[0].kinematics.predicted_paths[0].path
        for a, b in zip(first, second):
            self.assertAlmostEqual(a.position.x, b.position.x)
            self.assertAlmostEqual(a.position.y, b.position.y)

    def test_wrong_world_velocity_is_not_repaired_by_road_direction(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=2.0)
        tracked.kinematics.pose_with_covariance.pose.orientation = yaw_to_quaternion(math.pi)
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (0, 1))
        # This could be real oncoming motion or a bad young-track velocity.
        # The predictor has no evidence that permits forcing it along +X.
        self.assertAlmostEqual(output.objects[0].kinematics.predicted_paths[0].path[-1].position.x, 4.0)

    def test_perfect_current_state_does_not_predict_future_acceleration(self):
        tracked = self._track(ObjectClassification.CAR, x=10.0, speed=2.0)
        output, mapped, fallback, _ = self._prediction(tracked)
        self.assertEqual((mapped, fallback), (1, 0))
        horizon, acceleration = 3.0, 2.0
        future_gt = 10.0 + 2.0 * horizon + 0.5 * acceleration * horizon ** 2
        predicted_x = output.objects[0].kinematics.predicted_paths[0].path[-1].position.x
        self.assertAlmostEqual(abs(future_gt - predicted_x), 9.0)


if __name__ == "__main__":
    unittest.main()
