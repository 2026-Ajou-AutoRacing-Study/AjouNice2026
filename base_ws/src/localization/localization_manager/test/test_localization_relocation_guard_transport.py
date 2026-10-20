#!/usr/bin/env python3

import threading
import time
import unittest
import json

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class RelocationGuardTransportTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.resets = []
        self.statuses = []
        self.state_pub = rospy.Publisher("/test/state", Odometry, queue_size=10)
        self.orientation_pub = rospy.Publisher("/test/orientation", Imu, queue_size=10)
        self.gnss_pub = rospy.Publisher("/test/gnss", Odometry, queue_size=10)
        self.motion_pub = rospy.Publisher("/test/motion", TwistWithCovarianceStamped, queue_size=10)
        self.reset_sub = rospy.Subscriber(
            "/test/reset", PoseWithCovarianceStamped, self.reset_callback
        )
        self.status_sub = rospy.Subscriber("/test/relocation_status", String, self.status_callback)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if (
                self.state_pub.get_num_connections() > 0
                and self.orientation_pub.get_num_connections() > 0
                and self.gnss_pub.get_num_connections() > 0
                and self.motion_pub.get_num_connections() > 0
            ):
                break
            rospy.sleep(0.02)

    def reset_callback(self, message):
        with self.lock:
            self.resets.append(message)

    def status_callback(self, message):
        with self.lock:
            self.statuses.append(json.loads(message.data))

    @staticmethod
    def odometry(stamp, x, y):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def orientation(stamp):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.orientation.w = 1.0
        return message

    def publish_sample(self, x, state_x=0.0, vx=0.0, valid_state=True):
        stamp = rospy.Time.now()
        state = self.odometry(stamp, state_x, 0.0)
        if not valid_state:
            state.header.frame_id = "odom"
        self.state_pub.publish(state)
        self.orientation_pub.publish(self.orientation(stamp))
        motion = TwistWithCovarianceStamped()
        motion.header.stamp = stamp
        motion.header.frame_id = "base_link"
        motion.twist.twist.linear.x = vx
        self.motion_pub.publish(motion)
        rospy.sleep(0.01)
        self.gnss_pub.publish(self.odometry(stamp, x, 0.0))
        rospy.sleep(0.03)

    def test_three_persistent_samples_publish_one_reset(self):
        # Warm up continuous allowed motion telemetry before a stationary reset.
        for _ in range(5):
            self.publish_sample(0.0)
        # A relocation restarts the GNSS stability hold before counting candidates.
        for _ in range(4):
            self.publish_sample(20.0, valid_state=False)
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
        for _ in range(5):
            self.publish_sample(20.0)
        with self.lock:
            self.assertEqual(len(self.resets), 1)
        # Outcome acknowledgement requires two fresh Global poses at the target.
        for _ in range(3):
            self.publish_sample(20.0, state_x=20.0)
        with self.lock:
            self.assertTrue(any(s['state'] == 'acknowledged' for s in self.statuses))
        for _ in range(4):
            self.publish_sample(40.0, state_x=20.0, vx=2.0)
        with self.lock:
            self.assertEqual(len(self.resets), 1)
            self.assertTrue(any(s['reason'] == 'stationary_motion_hold_not_met' for s in self.statuses))
        # Wait out cooldown with fresh inputs, then intentionally withhold ack.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.publish_sample(20.0, state_x=20.0)
        for _ in range(4):
            self.publish_sample(60.0, state_x=20.0, valid_state=False)
        for _ in range(3):
            self.publish_sample(60.0, state_x=20.0)
        with self.lock:
            self.assertEqual(len(self.resets), 2)
        deadline = time.time() + 2.3
        while time.time() < deadline:
            self.publish_sample(60.0, state_x=20.0)
        with self.lock:
            self.assertEqual(len(self.resets), 2)
            self.assertTrue(any(s['state'] == 'timeout_blocked' for s in self.statuses))
            self.assertTrue(self.statuses[-1]['reset_blocked'])


if __name__ == "__main__":
    rospy.init_node("test_localization_relocation_guard_transport")
    rostest.rosrun(
        "localization_manager",
        "localization_relocation_guard_transport",
        RelocationGuardTransportTest,
    )
