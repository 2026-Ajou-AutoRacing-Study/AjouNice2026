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
        for index in range(400):
            point = TrajectoryPoint()
            point.pose.position.x = float(index)
            point.pose.orientation.w = 1.0
            point.longitudinal_velocity_mps = 5.0
            message.points.append(point)
        return message

    @staticmethod
    def _state(stamp, x=0.0):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = x
        return message

    def _wait_for_connections(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if self.trajectory_pub.get_num_connections() >= 2 and self.state_pub.get_num_connections() >= 2:
                return
            rospy.sleep(0.05)
        self.fail("controller subscribers did not connect")

    def _publish_for(self, duration_s, trajectory_stamp_fn, state_stamp_fn, state_x=0.0):
        deadline = time.monotonic() + duration_s
        rate = rospy.Rate(30)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.trajectory_pub.publish(self._trajectory(trajectory_stamp_fn()))
            self.state_pub.publish(self._state(state_stamp_fn(), state_x))
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

    def test_normal_high_speed_with_short_state_gap_does_not_trigger_jump_brake(self):
        self._wait_for_connections()
        for speed_kmh, gap in ((30.0, 0.35), (80.0, 0.35), (150.0, 0.35), (150.0, 0.45)):
            with self.subTest(speed_kmh=speed_kmh, gap=gap):
                self._publish_for(1.3, rospy.Time.now, rospy.Time.now)
                speed = speed_kmh / 3.6
                started = time.monotonic()
                while time.monotonic() - started < 1.3:
                    elapsed = time.monotonic() - started
                    stamp = rospy.Time.now()
                    trajectory = self._trajectory(stamp)
                    for point in trajectory.points:
                        point.longitudinal_velocity_mps = speed + 5.0
                    self.trajectory_pub.publish(trajectory)
                    if not (0.3 <= elapsed < 0.3 + gap):
                        state = self._state(stamp, speed * elapsed)
                        state.twist.twist.linear.x = speed
                        self.state_pub.publish(state)
                    rospy.sleep(0.02)
                commands = self._recent_commands(started + 0.3 + gap + 0.1)
                self.assertTrue(commands)
                self.assertFalse(any(c.brake == 1.0 for c in commands),
                                 "normal travel after a short input gap triggered full brake")

    def test_pose_jump_brakes_for_one_second_and_repeated_jump_extends_hold(self):
        self._wait_for_connections()
        self._publish_for(1.2, rospy.Time.now, rospy.Time.now)
        # A displacement below the threshold must not activate the hold.
        self._publish_for(0.3, rospy.Time.now, rospy.Time.now, state_x=6.0)
        self.assertTrue(any(c.accel > 0.0 and c.brake == 0.0
                            for c in self._recent_commands(time.monotonic() - 0.15)))
        # Exactly 7 m, with a still-valid forward trajectory: braking must come
        # from the jump guard rather than a missing-state/trajectory watchdog.
        jumped_at = time.monotonic()
        self._publish_for(0.95, rospy.Time.now, rospy.Time.now, state_x=13.0)
        self._assert_full_brake_since(jumped_at + 0.08, "7 m pose jump")
        self.assertTrue(self.lateral_valid)
        self._publish_for(0.6, rospy.Time.now, rospy.Time.now, state_x=13.0)
        self.assertTrue(any(c.accel > 0.0 and c.brake == 0.0
                            for c in self._recent_commands(time.monotonic() - 0.15)))
        # A second jump during a hold restarts the full one-second interval.
        self._publish_for(0.6, rospy.Time.now, rospy.Time.now, state_x=20.0)
        repeated_at = time.monotonic()
        self._publish_for(0.95, rospy.Time.now, rospy.Time.now, state_x=27.0)
        self._assert_full_brake_since(repeated_at + 0.08, "repeated pose jump")
        self._publish_for(0.6, rospy.Time.now, rospy.Time.now, state_x=27.0)
        self.assertTrue(any(c.accel > 0.0 and c.brake == 0.0
                            for c in self._recent_commands(time.monotonic() - 0.15)))

    def test_stale_republish_stops_and_fresh_input_recovers(self):
        self._wait_for_connections()

        self._publish_for(1.3, rospy.Time.now, rospy.Time.now)
        fresh_commands = self._recent_commands(time.monotonic() - 0.4)
        self.assertTrue(
            any(command.accel > 0.0 and command.brake == 0.0 for command in fresh_commands),
            "fresh trajectory/state never produced a forward command",
        )
        self.assertTrue(self.lateral_valid)

        stale_stamp = rospy.Time.now() - rospy.Duration(15.0)
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
