#!/usr/bin/python3
"""Bounded automatic MORAI excitation run for lateral tire identification.

This is deliberately an open-road simulation experiment, not a route-following
autonomous-driving controller. It uses EgoVehicleStatus only to hold a modest
speed while applying a bounded multisine steering input.
"""

import argparse
import math
import time

import rosgraph
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import String


class AutomaticExcitationPublisher:
    """Single-owner, speed-limited lateral-excitation command publisher."""

    def __init__(self, args):
        self.args = args
        rospy.init_node("mpc_develop_automatic_excitation")
        self.ctrl_cmd_topic = "/ctrl_cmd"
        self.status_topic = "/mpc_develop/command_status"
        self.ego_topic = "/Ego_topic"
        self.publisher = rospy.Publisher(self.ctrl_cmd_topic, CtrlCmd, queue_size=1)
        self.status_publisher = rospy.Publisher(self.status_topic, String, queue_size=5)
        self.last_ego = None
        self.last_ego_received_at = None
        self.aborted = False
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self._ego_callback, queue_size=1)
        self._check_exclusive_owner()
        rospy.on_shutdown(self._brake_hold)

    def _check_exclusive_owner(self):
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        owners = next((nodes for topic, nodes in publishers if topic == self.ctrl_cmd_topic), [])
        foreign = [node for node in owners if node != rospy.get_name()]
        if foreign:
            raise RuntimeError("refusing to publish {}: existing publisher(s): {}".format(
                self.ctrl_cmd_topic, ", ".join(foreign)))

    def _ego_callback(self, message):
        self.last_ego = message
        self.last_ego_received_at = time.monotonic()

    def _speed(self):
        if self.last_ego is None or self.last_ego_received_at is None:
            raise RuntimeError("no Ego feedback available")
        if time.monotonic() - self.last_ego_received_at > self.args.feedback_timeout:
            raise RuntimeError("Ego feedback timed out")
        return math.hypot(self.last_ego.velocity.x, self.last_ego.velocity.y)

    def _publish(self, accel, brake, steering, state):
        accel = min(max(float(accel), 0.0), self.args.max_accel)
        brake = min(max(float(brake), 0.0), self.args.max_brake)
        steering = min(max(float(steering), -self.args.max_steering), self.args.max_steering)
        if brake > 0.0:
            accel = 0.0
        command = CtrlCmd()
        command.longlCmdType = 1
        command.accel = accel
        command.brake = brake
        command.steering = steering
        self.publisher.publish(command)
        self.status_publisher.publish(String(data=(
            "AUTOMATIC_{} accel={:.3f} brake={:.3f} steering_rad={:.5f}".format(
                state, accel, brake, steering))))

    def _brake_hold(self):
        if not hasattr(self, "publisher"):
            return
        for _ in range(5):
            self._publish(0.0, self.args.brake_hold, 0.0, "BRAKE")
            time.sleep(1.0 / self.args.rate_hz)

    def _wait_for_ready(self):
        try:
            self.last_ego = rospy.wait_for_message(
                self.ego_topic, EgoVehicleStatus, timeout=self.args.feedback_timeout)
            self.last_ego_received_at = time.monotonic()
        except rospy.ROSException as error:
            raise RuntimeError("{} did not arrive: {}".format(self.ego_topic, error))
        deadline = time.monotonic() + self.args.feedback_timeout
        while self.publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            if rospy.is_shutdown():
                raise RuntimeError("ROS shutdown before MORAI command sender connected")
            rospy.sleep(0.1)
        if self.publisher.get_num_connections() == 0:
            raise RuntimeError("no /ctrl_cmd subscriber; start the MORAI UDP bridge first")

    def _speed_command(self):
        speed = self._speed()
        if speed is None:
            raise RuntimeError("lost Ego feedback")
        if speed > self.args.max_speed:
            raise RuntimeError("speed {:.2f} m/s exceeds safety limit {:.2f} m/s".format(
                speed, self.args.max_speed))
        error = self.args.target_speed - speed
        if error > self.args.speed_deadband:
            return min(self.args.max_accel, max(0.04, self.args.speed_kp * error)), 0.0
        if error < -self.args.speed_deadband:
            return 0.0, min(self.args.max_brake, self.args.brake_kp * -error)
        return 0.0, 0.0

    def _run_for(self, seconds, state, steering_function=None):
        rate = rospy.Rate(self.args.rate_hz)
        started = time.monotonic()
        low_speed_since = None
        while not rospy.is_shutdown() and time.monotonic() - started < seconds:
            elapsed = time.monotonic() - started
            if steering_function is None:
                accel, brake = 0.0, self.args.brake_hold
                steering = 0.0
            else:
                accel, brake = self._speed_command()
                steering = steering_function(elapsed)
                speed = self._speed()
                if elapsed >= self.args.low_speed_grace_sec and speed < self.args.minimum_excitation_speed:
                    if low_speed_since is None:
                        low_speed_since = time.monotonic()
                    elif time.monotonic() - low_speed_since >= self.args.low_speed_timeout:
                        raise RuntimeError(
                            "excitation speed remained below {:.2f} m/s for {:.1f} s".format(
                                self.args.minimum_excitation_speed,
                                self.args.low_speed_timeout))
                else:
                    low_speed_since = None
            self._publish(accel, brake, steering, state)
            rate.sleep()

    def run(self):
        self._wait_for_ready()
        rospy.logwarn("automatic excitation starts: target=%.1f m/s, steer amplitude=%.3f rad, duration=%.1f s",
                      self.args.target_speed, self.args.steer_amplitude, self.args.excitation_sec)
        self._run_for(self.args.initial_brake_sec, "INITIAL_BRAKE")

        reached_at = None
        deadline = time.monotonic() + self.args.accelerate_timeout
        rate = rospy.Rate(self.args.rate_hz)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            speed = self._speed()
            if speed is not None and speed >= self.args.target_speed - self.args.speed_deadband:
                reached_at = time.monotonic()
                break
            accel, brake = self._speed_command()
            self._publish(accel, brake, 0.0, "ACCELERATE")
            rate.sleep()
        if reached_at is None:
            raise RuntimeError("target speed was not reached within {:.1f} s".format(
                self.args.accelerate_timeout))

        self._run_for(self.args.settle_sec, "SETTLE", lambda _: 0.0)

        def multisine(elapsed):
            # The 1.6 normalizer ensures |delta| never exceeds steer_amplitude.
            return self.args.steer_amplitude * (
                math.sin(2.0 * math.pi * 0.20 * elapsed) +
                0.60 * math.sin(2.0 * math.pi * 0.37 * elapsed)) / 1.60

        self._run_for(self.args.excitation_sec, "EXCITATION", multisine)
        self._brake_hold()
        rospy.logwarn("automatic excitation completed; vehicle brake-held")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="required acknowledgement before any command is published")
    parser.add_argument("--target-speed", type=float, default=8.0)
    parser.add_argument("--max-speed", type=float, default=12.0)
    parser.add_argument("--steer-amplitude", type=float, default=0.05)
    parser.add_argument("--excitation-sec", type=float, default=60.0)
    parser.add_argument("--settle-sec", type=float, default=4.0)
    parser.add_argument("--initial-brake-sec", type=float, default=2.0)
    parser.add_argument("--accelerate-timeout", type=float, default=45.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--max-accel", type=float, default=0.20)
    parser.add_argument("--max-brake", type=float, default=0.45)
    parser.add_argument("--brake-hold", type=float, default=0.35)
    parser.add_argument("--max-steering", type=float, default=0.10)
    parser.add_argument("--speed-kp", type=float, default=0.06)
    parser.add_argument("--brake-kp", type=float, default=0.10)
    parser.add_argument("--speed-deadband", type=float, default=0.35)
    parser.add_argument("--minimum-excitation-speed", type=float, default=3.0)
    parser.add_argument("--low-speed-grace-sec", type=float, default=3.0)
    parser.add_argument("--low-speed-timeout", type=float, default=3.0)
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; this node sends vehicle commands")
    positive = (args.target_speed, args.max_speed, args.steer_amplitude,
                args.excitation_sec, args.accelerate_timeout, args.feedback_timeout,
                args.rate_hz, args.max_accel, args.max_brake, args.brake_hold,
                args.max_steering, args.minimum_excitation_speed,
                args.low_speed_grace_sec, args.low_speed_timeout)
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        parser.error("all limits and durations must be finite and positive")
    if args.target_speed >= args.max_speed:
        parser.error("--target-speed must be smaller than --max-speed")
    if args.steer_amplitude > args.max_steering:
        parser.error("--steer-amplitude cannot exceed --max-steering")
    if args.brake_hold > args.max_brake:
        parser.error("--brake-hold cannot exceed --max-brake")
    return args


if __name__ == "__main__":
    try:
        AutomaticExcitationPublisher(parse_args()).run()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("mpc_develop automatic excitation: %s", error)
        raise SystemExit(1)
