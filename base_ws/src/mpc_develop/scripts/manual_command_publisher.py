#!/usr/bin/python3
"""Interactive, single-owner MORAI CtrlCmd publisher for identification runs.

This node deliberately starts disarmed.  It publishes no /ctrl_cmd message
until the operator types ``arm`` and refuses to start if another node already
publishes the configured command topic.
"""

import math
import sys
import threading
import time

import rosgraph
import rospy
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String


class ManualCommandPublisher:
    """Publish bounded type-1 pedal/physical-steering commands from stdin."""

    def __init__(self):
        rospy.init_node("mpc_develop_manual_command")

        self.ctrl_cmd_topic = rospy.get_param("~ctrl_cmd_topic", "/ctrl_cmd")
        self.status_topic = rospy.get_param(
            "~status_topic", "/mpc_develop/command_status")
        self.rate_hz = float(rospy.get_param("~rate_hz", 20.0))
        self.brake_hold = float(rospy.get_param("~brake_hold", 0.35))
        self.max_accel = float(rospy.get_param("~max_accel", 1.0))
        self.max_brake = float(rospy.get_param("~max_brake", 1.0))
        self.max_steering_rad = float(
            rospy.get_param("~max_steering_rad", math.radians(40.0)))
        self.shutdown_brake_cycles = int(
            rospy.get_param("~shutdown_brake_cycles", 5))
        self.allow_existing_publisher = bool(
            rospy.get_param("~allow_existing_publisher", False))

        self._validate_parameters()
        self._check_existing_owner()

        self.publisher = rospy.Publisher(self.ctrl_cmd_topic, CtrlCmd, queue_size=1)
        self.status_publisher = rospy.Publisher(self.status_topic, String, queue_size=5)
        self.lock = threading.Lock()
        self.armed = False
        self.command = (0.0, self.brake_hold, 0.0)
        self.stop_requested = False
        rospy.on_shutdown(self._on_shutdown)

    def _validate_parameters(self):
        values = (self.rate_hz, self.brake_hold, self.max_accel,
                  self.max_brake, self.max_steering_rad)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("command bounds and rate must be finite")
        if self.rate_hz <= 0.0 or self.max_accel <= 0.0 or self.max_brake <= 0.0:
            raise ValueError("rate_hz, max_accel, and max_brake must be positive")
        if self.max_steering_rad <= 0.0 or self.max_steering_rad > math.pi / 2.0:
            raise ValueError("max_steering_rad must be within (0, pi/2]")
        if not 0.0 <= self.brake_hold <= self.max_brake:
            raise ValueError("brake_hold must be in [0, max_brake]")

    def _check_existing_owner(self):
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        owners = next((nodes for topic, nodes in publishers if topic == self.ctrl_cmd_topic), [])
        foreign_owners = [owner for owner in owners if owner != rospy.get_name()]
        if foreign_owners and not self.allow_existing_publisher:
            raise RuntimeError(
                "refusing to publish {}: existing publisher(s): {}".format(
                    self.ctrl_cmd_topic, ", ".join(foreign_owners)))
        if foreign_owners:
            rospy.logwarn("allow_existing_publisher=true: %s already has %s",
                          self.ctrl_cmd_topic, ", ".join(foreign_owners))

    @staticmethod
    def _bounded(value, lower, upper, name):
        if not math.isfinite(value):
            raise ValueError("{} must be finite".format(name))
        return max(lower, min(upper, value))

    def _set_command(self, accel, brake, steering):
        accel = self._bounded(accel, 0.0, self.max_accel, "accel")
        brake = self._bounded(brake, 0.0, self.max_brake, "brake")
        steering = self._bounded(
            steering, -self.max_steering_rad, self.max_steering_rad, "steering")
        # Type-1 MORAI pedals must never request throttle and brake together.
        if brake > 0.0:
            accel = 0.0
        with self.lock:
            self.command = (accel, brake, steering)
        rospy.loginfo("command set: accel=%.3f brake=%.3f steer=%.4f rad",
                      accel, brake, steering)

    def _publish_command(self, force=False):
        with self.lock:
            armed = self.armed
            accel, brake, steering = self.command
        if armed or force:
            command = CtrlCmd()
            command.longlCmdType = 1
            command.accel = accel
            command.brake = brake
            command.steering = steering
            command.velocity = 0.0
            command.acceleration = 0.0
            self.publisher.publish(command)
        state = "ARMED" if armed else "DISARMED"
        self.status_publisher.publish(String(
            data="{} accel={:.3f} brake={:.3f} steering_rad={:.5f}".format(
                state, accel, brake, steering)))

    def _arm(self):
        if self.publisher.get_num_connections() == 0:
            rospy.logwarn("cannot arm: no /ctrl_cmd subscriber; start the MORAI UDP sender first")
            return
        with self.lock:
            self.command = (0.0, self.brake_hold, 0.0)
            self.armed = True
        rospy.logwarn("ARMED: holding brake %.3f; use 'set' only after confirming MORAI is ready",
                      self.brake_hold)

    def _disarm(self):
        with self.lock:
            self.command = (0.0, self.brake_hold, 0.0)
            was_armed = self.armed
            self.armed = False
        if was_armed:
            self._publish_command(force=True)
        rospy.logwarn("DISARMED: sent brake hold once; MORAI command ownership is released")

    def _handle_line(self, line):
        tokens = line.strip().split()
        if not tokens:
            return
        action = tokens[0].lower()
        try:
            if action in ("help", "?"):
                self._print_help()
            elif action == "arm" and len(tokens) == 1:
                self._arm()
            elif action in ("disarm", "stop") and len(tokens) == 1:
                self._disarm()
            elif action == "hold" and len(tokens) == 1:
                self._set_command(0.0, self.brake_hold, 0.0)
            elif action == "zero" and len(tokens) == 1:
                self._set_command(0.0, 0.0, 0.0)
            elif action == "set" and len(tokens) == 4:
                self._set_command(float(tokens[1]), float(tokens[2]), float(tokens[3]))
            elif action in ("quit", "exit") and len(tokens) == 1:
                self._disarm()
                self.stop_requested = True
                rospy.signal_shutdown("operator requested exit")
            else:
                rospy.logwarn("invalid command: %s", line.strip())
                self._print_help()
        except ValueError as error:
            rospy.logwarn("invalid command value: %s", error)

    @staticmethod
    def _print_help():
        rospy.loginfo(
            "commands: arm | set <accel_0to1> <brake_0to1> <steering_rad> | "
            "zero | hold | stop | quit")

    def _stdin_loop(self):
        self._print_help()
        while not rospy.is_shutdown() and not self.stop_requested:
            try:
                line = input("mpc_develop> ")
            except EOFError:
                rospy.logwarn("stdin closed; disarming")
                self._disarm()
                return
            except KeyboardInterrupt:
                rospy.signal_shutdown("operator interrupted stdin")
                return
            self._handle_line(line)

    def _on_shutdown(self):
        if not hasattr(self, "publisher"):
            return
        with self.lock:
            was_armed = self.armed
            self.armed = False
            self.command = (0.0, self.brake_hold, 0.0)
        if was_armed:
            for _ in range(max(0, self.shutdown_brake_cycles)):
                self._publish_command(force=True)
                time.sleep(1.0 / self.rate_hz)

    def run(self):
        if sys.stdin.isatty():
            thread = threading.Thread(target=self._stdin_loop, daemon=True)
            thread.start()
        else:
            rospy.logwarn("stdin is not interactive; command node remains disarmed")
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._publish_command()
            rate.sleep()


if __name__ == "__main__":
    try:
        ManualCommandPublisher().run()
    except (RuntimeError, ValueError) as error:
        rospy.logfatal("mpc_develop manual command publisher: %s", error)
        raise SystemExit(1)
