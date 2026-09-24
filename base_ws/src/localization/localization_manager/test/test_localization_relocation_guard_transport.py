#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RelocationGuardTransportTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.resets = []
        self.state_pub = rospy.Publisher("/test/state", Odometry, queue_size=10)
        self.orientation_pub = rospy.Publisher("/test/orientation", Imu, queue_size=10)
        self.gnss_pub = rospy.Publisher("/test/gnss", Odometry, queue_size=10)
        self.reset_sub = rospy.Subscriber(
            "/test/reset", PoseWithCovarianceStamped, self.reset_callback
        )
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if (
                self.state_pub.get_num_connections() > 0
                and self.orientation_pub.get_num_connections() > 0
                and self.gnss_pub.get_num_connections() > 0
            ):
                break
            rospy.sleep(0.02)

    def reset_callback(self, message):
        with self.lock:
            self.resets.append(message)

    @staticmethod
    def odometry(stamp, x, y):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def orientation(stamp):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.orientation.w = 1.0
        return message

    def publish_sample(self, x):
        stamp = rospy.Time.now()
        self.state_pub.publish(self.odometry(stamp, 0.0, 0.0))
        self.orientation_pub.publish(self.orientation(stamp))
        rospy.sleep(0.01)
        self.gnss_pub.publish(self.odometry(stamp, x, 0.0))
        rospy.sleep(0.03)

    def test_three_persistent_samples_publish_one_reset(self):
        self.publish_sample(20.0)
        self.publish_sample(20.0)
        with self.lock:
            self.assertEqual(len(self.resets), 0)
        self.publish_sample(20.0)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            with self.lock:
                if self.resets:
                    break
            rospy.sleep(0.02)
        with self.lock:
            self.assertEqual(len(self.resets), 1)
            reset = self.resets[0]
        self.assertEqual(reset.header.frame_id, "map")
        self.assertAlmostEqual(reset.pose.pose.position.x, 20.0)
        self.assertAlmostEqual(reset.pose.pose.orientation.w, 1.0)
        self.assertGreater(reset.pose.covariance[0], 0.0)


if __name__ == "__main__":
    rospy.init_node("test_localization_relocation_guard_transport")
    rostest.rosrun(
        "localization_manager",
        "localization_relocation_guard_transport",
        RelocationGuardTransportTest,
    )
