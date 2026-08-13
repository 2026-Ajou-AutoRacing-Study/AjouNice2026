#!/usr/bin/env python3

import math
import unittest

from autoware_perception_msgs.msg import (
    DetectedObjectKinematics,
    ObjectClassification,
)
from std_msgs.msg import Header

from perception_object_pipeline.detected_objects import make_detected_objects


class DetectedObjectsTest(unittest.TestCase):
    def test_box_mapping_and_semantics(self):
        header = Header(frame_id="velodyne")
        message = make_detected_objects(
            header,
            [[1.0, 2.0, 3.0, 4.0, 2.0, 1.5, math.pi / 2.0]],
            [1.4],
            ["vehicle"],
        )

        self.assertEqual(message.header.frame_id, "velodyne")
        self.assertEqual(len(message.objects), 1)
        obj = message.objects[0]
        self.assertAlmostEqual(obj.existence_probability, 1.0)
        self.assertEqual(obj.classification[0].label, ObjectClassification.CAR)
        self.assertEqual(
            obj.kinematics.orientation_availability,
            DetectedObjectKinematics.AVAILABLE,
        )
        self.assertFalse(obj.kinematics.has_twist)
        self.assertAlmostEqual(obj.shape.dimensions.x, 4.0)
        self.assertAlmostEqual(obj.kinematics.pose_with_covariance.pose.orientation.z, math.sqrt(0.5))

    def test_ambiguous_orientation_and_negative_dimension_clamp(self):
        message = make_detected_objects(
            Header(frame_id="base_link"),
            [[0.0, 0.0, 0.0, -1.0, 0.5, 1.7, 0.0]],
            [-0.2],
            ["pedestrian"],
        )
        obj = message.objects[0]
        self.assertEqual(obj.existence_probability, 0.0)
        self.assertEqual(obj.classification[0].label, ObjectClassification.PEDESTRIAN)
        self.assertEqual(
            obj.kinematics.orientation_availability,
            DetectedObjectKinematics.SIGN_UNKNOWN,
        )
        self.assertEqual(obj.shape.dimensions.x, 0.0)


if __name__ == "__main__":
    unittest.main()
