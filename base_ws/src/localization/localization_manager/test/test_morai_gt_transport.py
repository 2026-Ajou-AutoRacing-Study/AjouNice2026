#!/usr/bin/env python3

import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import AccelWithCovarianceStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry


class MoraiGtTransportTest(unittest.TestCase):
    def setUp(self):
        self.odom = None
        self.acceleration = None
        self.odom_subscriber = rospy.Subscriber(
            "/localization/kinematic_state", Odometry, self.odom_callback
        )
        self.acceleration_subscriber = rospy.Subscriber(
            "/localization/acceleration",
            AccelWithCovarianceStamped,
            self.acceleration_callback,
        )
        self.ego_publisher = rospy.Publisher(
            "/Ego_topic", EgoVehicleStatus, queue_size=10
        )

    def odom_callback(self, message):
        self.odom = message

    def acceleration_callback(self, message):
        self.acceleration = message

    def test_odometry_and_acceleration_share_source_stamp(self):
        deadline = time.monotonic() + 10.0
        while (
            self.ego_publisher.get_num_connections() < 2
            and time.monotonic() < deadline
        ):
            rospy.sleep(0.05)
        self.assertGreaterEqual(self.ego_publisher.get_num_connections(), 2)

        sequence = 0
        while (
            (self.odom is None or self.acceleration is None)
            and time.monotonic() < deadline
        ):
            sequence += 1
            ego = EgoVehicleStatus()
            ego.header.seq = sequence
            ego.header.stamp = rospy.Time.now()
            ego.header.frame_id = "morai_map"
            ego.position.x = 10.0
            ego.position.y = 20.0
            ego.position.z = 0.5
            ego.velocity.x = 3.0
            ego.velocity.y = -0.2
            ego.acceleration.x = 1.25
            ego.acceleration.y = -0.5
            ego.acceleration.z = 0.125
            ego.heading = 30.0
            self.ego_publisher.publish(ego)
            rospy.sleep(0.05)

        self.assertIsNotNone(self.odom)
        self.assertIsNotNone(self.acceleration)
        self.assertEqual(self.odom.header.frame_id, "map")
        self.assertEqual(self.odom.child_frame_id, "base_link")
        self.assertEqual(self.acceleration.header.frame_id, "base_link")
        self.assertFalse(self.acceleration.header.stamp.is_zero())
        self.assertEqual(self.acceleration.header.stamp, self.odom.header.stamp)
        self.assertTrue(
            math.isclose(self.acceleration.accel.accel.linear.x, 1.25)
        )
        self.assertTrue(
            math.isclose(self.acceleration.accel.accel.linear.y, -0.5)
        )
        self.assertTrue(
            math.isclose(self.acceleration.accel.accel.linear.z, 0.125)
        )


if __name__ == "__main__":
    rospy.init_node("morai_gt_transport_test")
    rostest.rosrun(
        "localization_manager",
        "morai_gt_transport_test",
        MoraiGtTransportTest,
    )
