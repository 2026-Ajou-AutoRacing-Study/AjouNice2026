#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from autoware_adapi_v1_msgs.msg import OperationModeState


class FixedAutonomousOperationModeTest(unittest.TestCase):
    def setUp(self):
        self.messages = []
        self.subscriber = rospy.Subscriber(
            "/test/system/operation_mode/state",
            OperationModeState,
            self.messages.append,
            queue_size=10,
        )

    def test_fixed_state_and_increasing_stamp(self):
        deadline = time.monotonic() + 10.0
        while len(self.messages) < 3 and time.monotonic() < deadline:
            rospy.sleep(0.02)
        self.assertGreaterEqual(len(self.messages), 3)

        for message in self.messages:
            self.assertEqual(message.mode, OperationModeState.AUTONOMOUS)
            self.assertTrue(message.is_autoware_control_enabled)
            self.assertFalse(message.is_in_transition)
            self.assertFalse(message.is_stop_mode_available)
            self.assertTrue(message.is_autonomous_mode_available)
            self.assertFalse(message.is_local_mode_available)
            self.assertFalse(message.is_remote_mode_available)

        stamps = [
            message.stamp.sec + message.stamp.nanosec * 1.0e-9
            for message in self.messages
        ]
        self.assertTrue(all(second > first for first, second in zip(stamps, stamps[1:])))


if __name__ == "__main__":
    rospy.init_node("fixed_autonomous_operation_mode_test")
    rostest.rosrun(
        "ajounice_planning_integration",
        "fixed_autonomous_operation_mode_test",
        FixedAutonomousOperationModeTest,
    )
