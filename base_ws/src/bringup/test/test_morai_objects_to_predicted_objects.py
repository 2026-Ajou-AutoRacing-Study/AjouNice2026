#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest

import rospy
from autoware_perception_msgs.msg import ObjectClassification, Shape
from geometry_msgs.msg import TransformStamped
from morai_msgs.msg import ObjectStatus, ObjectStatusList


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "validation",
    "scripts",
    "morai_objects_to_predicted_objects.py",
)
SPEC = importlib.util.spec_from_file_location("morai_object_bridge", SCRIPT_PATH)
BRIDGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BRIDGE)


def detected_object(unique_id, heading=0.0, velocity_x=0.0):
    output = ObjectStatus()
    output.unique_id = unique_id
    output.heading = heading
    output.position.x = 1.0
    output.position.y = 2.0
    output.position.z = 0.0
    output.velocity.x = velocity_x
    output.size.x = 4.0
    output.size.y = 2.0
    output.size.z = 2.0
    return output


def identity_transform():
    output = TransformStamped()
    output.header.frame_id = "map"
    output.child_frame_id = "morai_map"
    output.transform.rotation.w = 1.0
    return output


class MoraiObjectBridgeTest(unittest.TestCase):
    def test_npc_fields_and_object_frame_velocity(self):
        message = ObjectStatusList()
        message.header.frame_id = "morai_map"
        message.header.stamp = rospy.Time(12.0)
        message.npc_list = [
            detected_object(42, heading=90.0, velocity_x=10.0)
        ]

        output = BRIDGE.make_predicted_objects(
            message, identity_transform()
        )

        self.assertEqual(output.header.frame_id, "map")
        self.assertEqual(len(output.objects), 1)
        converted = output.objects[0]
        self.assertEqual(
            converted.classification[0].label, ObjectClassification.CAR
        )
        self.assertAlmostEqual(
            converted.classification[0].probability, 1.0
        )
        self.assertEqual(converted.shape.type, Shape.BOUNDING_BOX)
        self.assertAlmostEqual(converted.shape.dimensions.x, 4.0)
        self.assertAlmostEqual(converted.shape.dimensions.y, 2.0)
        self.assertAlmostEqual(converted.shape.dimensions.z, 2.0)
        self.assertAlmostEqual(
            converted.kinematics.initial_pose_with_covariance.pose.position.z,
            1.0,
        )
        self.assertAlmostEqual(
            converted.kinematics.initial_pose_with_covariance.pose.orientation.z,
            math.sqrt(0.5),
            places=6,
        )
        self.assertAlmostEqual(
            converted.kinematics.initial_twist_with_covariance.twist.linear.x,
            0.0,
            places=6,
        )
        self.assertAlmostEqual(
            converted.kinematics.initial_twist_with_covariance.twist.linear.y,
            10.0,
            places=6,
        )
        self.assertEqual(len(converted.object_id.uuid), 16)

    def test_global_kinematics_can_be_selected_explicitly(self):
        message = ObjectStatusList()
        message.header.frame_id = "morai_map"
        message.npc_list = [
            detected_object(42, heading=90.0, velocity_x=10.0)
        ]

        converted = BRIDGE.make_predicted_objects(
            message,
            identity_transform(),
            kinematics_in_object_frame=False,
        ).objects[0]
        velocity = (
            converted.kinematics.initial_twist_with_covariance.twist.linear
        )

        self.assertAlmostEqual(velocity.x, 10.0, places=6)
        self.assertAlmostEqual(velocity.y, 0.0, places=6)

    def test_tf_rotates_pose_heading_and_velocity(self):
        message = ObjectStatusList()
        message.header.frame_id = "morai_map"
        message.npc_list = [detected_object(7, velocity_x=10.0)]
        transform = identity_transform()
        transform.transform.translation.x = 10.0
        transform.transform.translation.y = 20.0
        transform.transform.rotation.z = math.sqrt(0.5)
        transform.transform.rotation.w = math.sqrt(0.5)

        converted = BRIDGE.make_predicted_objects(
            message, transform
        ).objects[0]
        pose = converted.kinematics.initial_pose_with_covariance.pose
        velocity = (
            converted.kinematics.initial_twist_with_covariance.twist.linear
        )

        self.assertAlmostEqual(pose.position.x, 8.0, places=6)
        self.assertAlmostEqual(pose.position.y, 21.0, places=6)
        self.assertAlmostEqual(pose.orientation.z, math.sqrt(0.5), places=6)
        self.assertAlmostEqual(velocity.x, 0.0, places=6)
        self.assertAlmostEqual(velocity.y, 10.0, places=6)

    def test_all_morai_categories_are_forwarded(self):
        message = ObjectStatusList()
        message.header.frame_id = "map"
        message.npc_list = [detected_object(1)]
        message.pedestrian_list = [detected_object(2)]
        message.obstacle_list = [detected_object(3)]

        output = BRIDGE.make_predicted_objects(message)
        labels = [
            predicted.classification[0].label
            for predicted in output.objects
        ]

        self.assertEqual(
            labels,
            [
                ObjectClassification.CAR,
                ObjectClassification.PEDESTRIAN,
                ObjectClassification.UNKNOWN,
            ],
        )
        self.assertEqual(len({bytes(obj.object_id.uuid) for obj in output.objects}), 3)


if __name__ == "__main__":
    unittest.main()
