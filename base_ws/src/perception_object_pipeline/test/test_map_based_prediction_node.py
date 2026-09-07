#!/usr/bin/env python3

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


if __name__ == "__main__":
    unittest.main()
