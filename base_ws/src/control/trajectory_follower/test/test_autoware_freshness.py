#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class AutowareFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.lateral_valid = None
        self.failure_reason = ""
        self.trajectory_pub = rospy.Publisher(
            "/planning/trajectory", Trajectory, queue_size=1
        )
        self.state_pub = rospy.Publisher(
            "/localization/kinematic_state", Odometry, queue_size=1
        )
        self.command_sub = rospy.Subscriber("/ctrl_cmd", CtrlCmd, self._on_command)
        self.valid_sub = rospy.Subscriber(
            "/control/autoware/controller_valid", Bool, self._on_valid
        )
        self.failure_sub = rospy.Subscriber(
            "/control/autoware/failure_reason", String, self._on_failure
        )

    def _on_command(self, message):
        self.commands.append((time.monotonic(), message))

    def _on_valid(self, message):
        self.lateral_valid = message.data

    def _on_failure(self, message):
        self.failure_reason = message.data

    @staticmethod
    def _trajectory(stamp):
        message = Trajectory()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        for index in range(40):
            point = TrajectoryPoint()
            point.pose.position.x = float(index)
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 5.0
            message.points.append(point)
        return message

    @staticmethod
    def _state(stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        return message

    def _wait_for_connections(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.trajectory_pub.get_num_connections() >= 2 and self.state_pub.get_num_connections() >= 2:
                return
            rospy.sleep(0.05)
        self.fail("controller subscribers did not connect")

    def _publish_for(self, duration_s, trajectory_stamp_fn, state_stamp_fn):
        deadline = time.monotonic() + duration_s
        rate = rospy.Rate(30)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.trajectory_pub.publish(self._trajectory(trajectory_stamp_fn()))
            self.state_pub.publish(self._state(state_stamp_fn()))
            rate.sleep()

    def _recent_commands(self, since):
        return [message for observed, message in self.commands if observed >= since]

    def _assert_full_brake_since(self, since, context):
        commands = self._recent_commands(since)
        self.assertTrue(commands, "no commands observed during {}".format(context))
        self.assertTrue(
            all(
                command.accel == 0.0
                and command.brake == 1.0
                and command.steering == 0.0
                for command in commands
            ),
            "{} did not remain at full-brake stop".format(context),
        )

    def test_stale_republish_stops_and_fresh_input_recovers(self):
        self._wait_for_connections()

        self._publish_for(1.0, rospy.Time.now, rospy.Time.now)
        fresh_commands = self._recent_commands(time.monotonic() - 0.4)
        self.assertTrue(
            any(command.accel > 0.0 and command.brake == 0.0 for command in fresh_commands),
            "fresh trajectory/state never produced a forward command",
        )
        self.assertTrue(self.lateral_valid)

        stale_stamp = rospy.Time.now() - rospy.Duration(5.0)
        stale_phase = time.monotonic()
        self._publish_for(0.8, lambda: stale_stamp, rospy.Time.now)
        self._assert_full_brake_since(stale_phase + 0.4, "stale trajectory replay")
        self.assertFalse(self.lateral_valid)
        self.assertEqual(self.failure_reason, "stale or unavailable trajectory/state")

        future_stamp = rospy.Time.now() + rospy.Duration(5.0)
        future_phase = time.monotonic()
        self._publish_for(0.5, lambda: future_stamp, rospy.Time.now)
        self._assert_full_brake_since(future_phase + 0.25, "future trajectory replay")

        zero_phase = time.monotonic()
        self._publish_for(0.5, rospy.Time, rospy.Time.now)
        self._assert_full_brake_since(zero_phase + 0.25, "zero-stamp trajectory replay")

        recovery_phase = time.monotonic()
        self._publish_for(0.8, rospy.Time.now, rospy.Time.now)
        recovered_commands = self._recent_commands(recovery_phase + 0.3)
        self.assertTrue(
            any(command.accel > 0.0 and command.brake == 0.0 for command in recovered_commands),
            "controller did not recover after fresh source stamps returned",
        )
        self.assertTrue(self.lateral_valid)


if __name__ == "__main__":
    rospy.init_node("autoware_freshness_test")
    rostest.rosrun(
        "trajectory_follower", "autoware_freshness_test", AutowareFreshnessTest
    )
