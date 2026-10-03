#!/usr/bin/env python3

import math
import io
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
    def test_sim_velocity_body_axes_and_metadata(self):
        for heading in (0.0, 90.0, -90.0, 179.0):
            message = ObjectStatusList()
            status = make_status(heading=heading)
            status.unique_id = 23
            status.velocity.x, status.velocity.y = 33.333, -0.4
            message.npc_list = [status]
            baseline = make_morai_gt_detections(message).objects[0]
            obj = make_morai_gt_detections(message, include_velocity=True).objects[0]
            self.assertEqual(obj.shape, baseline.shape)
            self.assertEqual(obj.classification, baseline.classification)
            self.assertEqual(obj.kinematics.pose_with_covariance, baseline.kinematics.pose_with_covariance)
            self.assertTrue(obj.kinematics.has_twist)
            self.assertTrue(obj.kinematics.has_twist_covariance)
            self.assertAlmostEqual(obj.kinematics.twist_with_covariance.twist.linear.x, 33.333)
            self.assertAlmostEqual(obj.kinematics.twist_with_covariance.twist.linear.y, -0.4)
            self.assertEqual(obj.kinematics.twist_with_covariance.covariance[0], 1.0)
            # Tracker applies world pose yaw, which includes the map TF yaw.
            world_yaw = math.radians(heading) + 0.31
            twist = obj.kinematics.twist_with_covariance.twist.linear
            world_vx = math.cos(world_yaw) * twist.x - math.sin(world_yaw) * twist.y
            self.assertAlmostEqual(world_vx, math.cos(world_yaw) * 33.333 + math.sin(world_yaw) * 0.4)

    def test_default_serialization_and_invalid_velocity(self):
        message = ObjectStatusList()
        message.npc_list = [make_status()]
        before = io.BytesIO()
        make_morai_gt_detections(message).serialize(before)
        for value in (float('nan'), float('inf'), 51.0):
            message.npc_list[0].velocity.x = value
            after = io.BytesIO()
            make_morai_gt_detections(message, include_velocity=False).serialize(after)
            self.assertEqual(before.getvalue(), after.getvalue())
            self.assertFalse(make_morai_gt_detections(message, include_velocity=True).objects[0].kinematics.has_twist)

    def test_stationary_and_non_vehicle_velocity(self):
        message = ObjectStatusList()
        message.npc_list = [make_status()]
        message.pedestrian_list = [make_status(length=0.6)]
        objects = make_morai_gt_detections(message, include_velocity=True).objects
        self.assertTrue(objects[0].kinematics.has_twist)
        self.assertEqual(objects[0].kinematics.twist_with_covariance.twist.linear.x, 0.0)
        self.assertFalse(objects[1].kinematics.has_twist)

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
