#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from autoware_adapi_v1_msgs.msg import OperationModeState


class OperationModeRelayTest(unittest.TestCase):
    def setUp(self):
        self.output = None
        self.subscriber = rospy.Subscriber(
            "/test/api/operation_mode/state",
            OperationModeState,
            self.callback,
        )
        self.publisher = rospy.Publisher(
            "/test/system/operation_mode/state",
            OperationModeState,
            queue_size=1,
            latch=True,
        )

    def callback(self, message):
        self.output = message

    def test_relay_preserves_complete_state(self):
        deadline = time.monotonic() + 10.0
        while self.publisher.get_num_connections() < 1 and time.monotonic() < deadline:
            rospy.sleep(0.05)
        self.assertGreaterEqual(self.publisher.get_num_connections(), 1)

        source = OperationModeState()
        now = rospy.Time.now()
        source.stamp.sec = now.secs
        source.stamp.nanosec = now.nsecs
        source.mode = OperationModeState.AUTONOMOUS
        source.is_autoware_control_enabled = True
        source.is_in_transition = False
        source.is_stop_mode_available = True
        source.is_autonomous_mode_available = True
        source.is_local_mode_available = False
        source.is_remote_mode_available = False
        self.publisher.publish(source)

        while self.output is None and time.monotonic() < deadline:
            rospy.sleep(0.05)
        self.assertIsNotNone(self.output)
        self.assertEqual(self.output, source)


if __name__ == "__main__":
    rospy.init_node("operation_mode_relay_test")
    rostest.rosrun(
        "ajounice_planning_integration",
        "operation_mode_relay_test",
        OperationModeRelayTest,
    )
