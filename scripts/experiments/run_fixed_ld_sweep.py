#!/usr/bin/env python3
"""Run repeatable fixed-lookahead Pure Pursuit experiments in MORAI."""

import argparse
import math
import os
import signal
import subprocess
import sys
import time
from decimal import Decimal

import rospy
from autoware_planning_msgs.msg import Trajectory
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from morai_msgs.srv import MoraiScenarioLoadSrv, MoraiScenarioLoadSrvRequest
from nav_msgs.msg import Odometry


class TopicState:
    def __init__(self):
        self.ego = None
        self.ego_received = 0.0
        self.trajectory_received = 0.0
        self.odometry_received = 0.0

    def ego_callback(self, message):
        self.ego = message
        self.ego_received = time.monotonic()

    def trajectory_callback(self, _message):
        self.trajectory_received = time.monotonic()

    def odometry_callback(self, _message):
        self.odometry_received = time.monotonic()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sweep fixed Pure Pursuit lookahead distance in MORAI"
    )
    parser.add_argument("--ld-start", type=float, default=2.0)
    parser.add_argument("--ld-end", type=float, default=12.0)
    parser.add_argument("--ld-step", type=float, default=1.0)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--scenario", default="loadstart.json")
    parser.add_argument("--start-x", type=float, default=-133.431)
    parser.add_argument("--start-y", type=float, default=-431.796)
    parser.add_argument("--start-tolerance", type=float, default=0.5)
    parser.add_argument("--stationary-speed", type=float, default=0.5)
    parser.add_argument("--reset-timeout", type=float, default=20.0)
    parser.add_argument("--run-timeout", type=float, default=300.0)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--max-lateral-deviation", type=float, default=15.0)
    parser.add_argument("--off-route-duration", type=float, default=3.0)
    parser.add_argument(
        "--no-final-reset",
        action="store_true",
        help="leave the vehicle at the final run's end position",
    )
    return parser.parse_args()


def validate_args(args):
    if args.ld_start <= 0 or args.ld_end < args.ld_start:
        raise ValueError("LD range must be positive and increasing")
    if not 0.1 <= args.ld_step <= 1.0:
        raise ValueError("ld-step must be between 0.1 and 1.0 m")
    if args.repeat <= 0:
        raise ValueError("repeat must be positive")
    for name in (
        "start_tolerance",
        "stationary_speed",
        "reset_timeout",
        "run_timeout",
        "startup_timeout",
        "max_lateral_deviation",
        "off_route_duration",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            raise ValueError("{} must be finite and positive".format(name))


def start_launch(arguments):
    return subprocess.Popen(
        ["roslaunch"] + arguments,
        start_new_session=True,
    )


def stop_process(process, timeout=8.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def publish_stop(publisher):
    message = CtrlCmd()
    message.longlCmdType = 1
    message.accel = 0.0
    message.brake = 1.0
    message.steering = 0.0
    rate = rospy.Rate(20)
    for _ in range(20):
        if rospy.is_shutdown():
            break
        publisher.publish(message)
        rate.sleep()


def fresh(received, maximum_age=1.0):
    return received > 0.0 and time.monotonic() - received <= maximum_age


def ld_values(args):
    start = Decimal(str(args.ld_start))
    end = Decimal(str(args.ld_end))
    step = Decimal(str(args.ld_step))
    values = []
    current = start
    while current <= end:
        values.append(float(current))
        current += step
    return values


def ld_label(value):
    text = "{:.10f}".format(value).rstrip("0").rstrip(".")
    integer, separator, fraction = text.partition(".")
    label = "ld{}".format(integer.zfill(2))
    return "{}p{}".format(label, fraction) if separator else label


def wait_for_pipeline(state, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if (
            fresh(state.ego_received)
            and fresh(state.trajectory_received)
            and fresh(state.odometry_received)
        ):
            return
        rospy.sleep(0.1)
    raise RuntimeError(
        "required topics are unavailable: /Ego_topic, "
        "/control/trajectory, /localization/kinematic_state"
    )


def reset_scenario(service, state, args):
    request = MoraiScenarioLoadSrvRequest()
    request.request.file_name = args.scenario
    request.request.load_network_connection_data = True
    request.request.delete_all = False
    request.request.load_ego_vehicle_data = True
    request.request.load_surrounding_vehicle_data = True
    request.request.load_pedestrian_data = True
    request.request.load_obstacle_data = True
    request.request.set_pause = False
    response = service(request)
    if not response.response.result:
        raise RuntimeError("Scenario Load UDP sender rejected the request")

    deadline = time.monotonic() + args.reset_timeout
    stable_samples = 0
    while time.monotonic() < deadline and not rospy.is_shutdown():
        message = state.ego
        if message is None or not fresh(state.ego_received):
            stable_samples = 0
            rospy.sleep(0.1)
            continue
        distance = math.hypot(
            message.position.x - args.start_x,
            message.position.y - args.start_y,
        )
        speed = math.sqrt(
            message.velocity.x ** 2
            + message.velocity.y ** 2
            + message.velocity.z ** 2
        )
        if distance <= args.start_tolerance and speed <= args.stationary_speed:
            stable_samples += 1
            if stable_samples >= 5:
                rospy.sleep(args.settle_time)
                wait_for_pipeline(state, args.startup_timeout)
                return
        else:
            stable_samples = 0
        rospy.sleep(0.1)
    raise RuntimeError("vehicle did not settle at the loadstart position")


def wait_for_node(node_name, process, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if process.poll() is not None:
            raise RuntimeError(
                "{} launch exited early with status {}".format(
                    node_name, process.returncode
                )
            )
        try:
            output = subprocess.check_output(
                ["rosnode", "list"], stderr=subprocess.DEVNULL, text=True
            )
            if node_name in output.splitlines():
                return
        except subprocess.CalledProcessError:
            pass
        rospy.sleep(0.1)
    raise RuntimeError("{} did not start".format(node_name))


def run_once(ld_value, repeat_index, state, stop_publisher, args):
    label = "{}_run{:02d}".format(ld_label(ld_value), repeat_index)
    validator = None
    follower = None
    try:
        rospy.loginfo("[%s] resetting scenario", label)
        reset_scenario(scenario_service, state, args)

        rospy.loginfo("[%s] starting validator", label)
        validator = start_launch(
            [
                "trajectory_validator",
                "record.launch",
                "run_label:={}".format(label),
                "max_lateral_deviation_m:={}".format(
                    args.max_lateral_deviation
                ),
                "off_route_duration_s:={}".format(args.off_route_duration),
            ]
        )
        wait_for_node("/trajectory_validator_recorder", validator, args.startup_timeout)

        rospy.loginfo("[%s] starting fixed-LD follower (LD=%g m)", label, ld_value)
        follower = start_launch(
            [
                "trajectory_follower",
                "pp_pi.launch",
                "lookahead_distance_m:={:.10g}".format(ld_value),
            ]
        )
        wait_for_node("/purepursuit", follower, args.startup_timeout)

        deadline = time.monotonic() + args.run_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            validator_status = validator.poll()
            follower_status = follower.poll()
            if validator_status is not None:
                if validator_status != 0:
                    raise RuntimeError(
                        "validator exited with status {}".format(validator_status)
                    )
                rospy.loginfo("[%s] recording completed", label)
                return
            if follower_status is not None:
                raise RuntimeError(
                    "follower exited before recording completed (status {})".format(
                        follower_status
                    )
                )
            rospy.sleep(0.2)
        raise RuntimeError("run timed out after {:.1f}s".format(args.run_timeout))
    finally:
        stop_process(follower)
        publish_stop(stop_publisher)
        stop_process(validator)


if __name__ == "__main__":
    parsed_args = parse_args()
    try:
        validate_args(parsed_args)
        rospy.init_node("fixed_ld_sweep", anonymous=False)
        topic_state = TopicState()
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, topic_state.ego_callback)
        rospy.Subscriber(
            "/control/trajectory", Trajectory, topic_state.trajectory_callback
        )
        rospy.Subscriber(
            "/localization/kinematic_state",
            Odometry,
            topic_state.odometry_callback,
        )
        stop_command_publisher = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        rospy.loginfo("waiting for MORAI Scenario Load UDP service")
        rospy.wait_for_service("/morai/scenario_load_udp", parsed_args.startup_timeout)
        scenario_service = rospy.ServiceProxy(
            "/morai/scenario_load_udp", MoraiScenarioLoadSrv
        )
        wait_for_pipeline(topic_state, parsed_args.startup_timeout)

        lookahead_values = ld_values(parsed_args)
        total = len(lookahead_values) * parsed_args.repeat
        completed = 0
        for lookahead in lookahead_values:
            for repeat in range(1, parsed_args.repeat + 1):
                run_once(
                    lookahead,
                    repeat,
                    topic_state,
                    stop_command_publisher,
                    parsed_args,
                )
                completed += 1
                rospy.loginfo("completed %d/%d runs", completed, total)

        if not parsed_args.no_final_reset:
            rospy.loginfo("returning vehicle to loadstart")
            reset_scenario(scenario_service, topic_state, parsed_args)
        rospy.loginfo("fixed LD sweep completed: %d runs", completed)
    except (rospy.ROSException, rospy.ServiceException, RuntimeError, ValueError) as error:
        rospy.logerr("fixed LD sweep failed: %s", error)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
