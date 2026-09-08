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

from object_prediction_node import make_predicted_objects, yaw_to_quaternion
from perception_object_pipeline.time_conversion import duration_to_seconds


class ObjectPredictionTest(unittest.TestCase):
    def _track(self, label, x=1.0, y=2.0, yaw=0.0, vx=2.0, vy=0.0, omega=0.0):
        tracked = TrackedObject()
        tracked.object_id.uuid = bytes([0] * 15 + [7])
        tracked.existence_probability = 0.8
        tracked.classification = [ObjectClassification(label=label, probability=0.9)]
        tracked.kinematics.pose_with_covariance.pose.position.x = x
        tracked.kinematics.pose_with_covariance.pose.position.y = y
        tracked.kinematics.pose_with_covariance.pose.orientation = yaw_to_quaternion(yaw)
        tracked.kinematics.twist_with_covariance.twist.linear.x = vx
        tracked.kinematics.twist_with_covariance.twist.linear.y = vy
        tracked.kinematics.twist_with_covariance.twist.angular.z = omega
        tracked.shape.dimensions.x = 4.0
        return tracked

    def test_vehicle_ctrv_and_identity_preservation(self):
        message = TrackedObjects()
        message.header.frame_id = "map"
        message.objects = [
            self._track(ObjectClassification.CAR, omega=0.2)
        ]
        output = make_predicted_objects(message, [0.5, 1.0, 1.5])
        self.assertEqual(output.header.frame_id, "map")
        self.assertEqual(output.objects[0].object_id.uuid[15], 7)
        path = output.objects[0].kinematics.predicted_paths[0]
        self.assertEqual(len(path.path), 4)
        self.assertAlmostEqual(duration_to_seconds(path.time_step), 0.5)
        self.assertAlmostEqual(path.path[0].position.x, 1.0)
        self.assertGreater(path.path[-1].position.x, path.path[0].position.x)
        self.assertGreater(path.path[-1].position.y, path.path[0].position.y)

    def test_pedestrian_uses_lateral_velocity(self):
        message = TrackedObjects()
        message.objects = [
            self._track(ObjectClassification.PEDESTRIAN, vx=0.0, vy=1.0)
        ]
        output = make_predicted_objects(message, [1.0])
        pose = output.objects[0].kinematics.predicted_paths[0].path[1]
        self.assertAlmostEqual(pose.position.x, 1.0)
        self.assertAlmostEqual(pose.position.y, 3.0)
        self.assertAlmostEqual(pose.orientation.z, math.sqrt(0.5), places=6)


if __name__ == "__main__":
    unittest.main()
