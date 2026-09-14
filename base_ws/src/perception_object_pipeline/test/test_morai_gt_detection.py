#!/usr/bin/env python3

import math
import unittest

from autoware_perception_msgs.msg import (
    DetectedObjectKinematics,
    ObjectClassification,
)
from morai_msgs.msg import ObjectStatus, ObjectStatusList

from perception_object_pipeline.morai_gt_detection import (
    make_morai_gt_detections,
    vehicle_bbox_center_offset,
)


def make_status(*, heading=0.0, length=4.5, width=1.8, height=1.5):
    status = ObjectStatus()
    status.heading = heading
    status.position.x = 10.0
    status.position.y = 20.0
    status.position.z = 0.75
    status.size.x = length
    status.size.y = width
    status.size.z = height
    return status


class MoraiGtDetectionTest(unittest.TestCase):
    def test_vehicle_actor_reference_is_shifted_to_bbox_center(self):
        message = ObjectStatusList()
        message.npc_list = [make_status(heading=90.0)]

        output = make_morai_gt_detections(message)

        self.assertEqual(output.header.frame_id, "morai_map")
        self.assertEqual(len(output.objects), 1)
        obj = output.objects[0]
        self.assertAlmostEqual(obj.kinematics.pose_with_covariance.pose.position.x, 10.0)
        self.assertAlmostEqual(obj.kinematics.pose_with_covariance.pose.position.y, 21.34)
        self.assertAlmostEqual(
            obj.kinematics.pose_with_covariance.pose.orientation.z,
            math.sqrt(0.5),
        )
        self.assertEqual(obj.classification[0].label, ObjectClassification.CAR)
        self.assertFalse(obj.kinematics.has_twist)

    def test_non_vehicle_is_not_shifted_and_keeps_ambiguous_heading(self):
        message = ObjectStatusList()
        message.pedestrian_list = [make_status(heading=45.0, length=0.6)]

        obj = make_morai_gt_detections(message).objects[0]

        self.assertAlmostEqual(obj.kinematics.pose_with_covariance.pose.position.x, 10.0)
        self.assertAlmostEqual(obj.kinematics.pose_with_covariance.pose.position.y, 20.0)
        self.assertEqual(
            obj.kinematics.orientation_availability,
            DetectedObjectKinematics.SIGN_UNKNOWN,
        )
        self.assertEqual(
            obj.classification[0].label, ObjectClassification.PEDESTRIAN
        )

    def test_unknown_large_vehicle_profile_is_not_guessed(self):
        self.assertEqual(vehicle_bbox_center_offset(12.0, 2.5, 3.5), 0.0)


if __name__ == "__main__":
    unittest.main()
