#!/usr/bin/env python3

"""Record compact planner diagnostics from multiple ROS topics into one CSV file."""

import csv
import json
import math
import os
import re
import subprocess
import threading
import time
from collections import deque
from datetime import datetime

import rospy
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_planning_msgs.msg import Path, Trajectory
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from tier4_planning_msgs.msg import StopReasonArray


BEHAVIOR_NAMES = {
    0: "UNKNOWN",
    1: "NONE",
    2: "SLOW_DOWN",
    3: "STOP",
    4: "SHIFT_LEFT",
    5: "SHIFT_RIGHT",
    6: "TURN_LEFT",
    7: "TURN_RIGHT",
}

OPERATION_MODE_NAMES = {
    0: "UNKNOWN",
    1: "STOP",
    2: "AUTONOMOUS",
    3: "LOCAL",
    4: "REMOTE",
}

DEFAULT_TOPICS = {
    "bpp_status": "/planning/debug/bpp/module_status",
    "bvp_status": "/planning/debug/behavior_velocity_planner/status",
    "stop_reasons": "/planning/scenario_planning/status/stop_reasons",
    "planning_factors": "/planning/planning_factors/behavior_velocity_planner",
    "velocity_smoother_status": "/planning/debug/velocity_smoother/status",
    "odometry": "/localization/kinematic_state",
    "acceleration": "/localization/acceleration",
    "bvp_path": "/planning/scenario_planning/lane_driving/behavior_planning/path",
    "path_smoother_path": (
        "/planning/scenario_planning/lane_driving/motion_planning/"
        "path_smoother/path"
    ),
    "path_optimizer_trajectory": (
        "/planning/scenario_planning/lane_driving/motion_planning/"
        "path_optimizer/trajectory"
    ),
    "mvp_raw_trajectory": (
        "/planning/scenario_planning/lane_driving/motion_planning/"
        "motion_velocity_planner/raw_trajectory"
    ),
    "control_trajectory": "/control/trajectory",
    "ctrl_cmd": "/ctrl_cmd",
    "ego_status": "/Ego_topic",
    "operation_mode": "/system/operation_mode/state",
    "rosout": "/rosout_agg",
}

CSV_FIELDS = [
    "wall_time_iso",
    "ros_time",
    "elapsed_sec",
    "git_branch",
    "git_commit",
    "git_dirty",
    "scenario_note",
    "metadata_json",
    "bpp_status",
    "bpp_status_age_sec",
    "bvp_status",
    "bvp_status_age_sec",
    "stop_reasons_stamp",
    "stop_reasons_age_sec",
    "stop_reason_count",
    "stop_reasons_json",
    "nearest_stop_reason",
    "nearest_stop_distance_m",
    "nearest_stop_x",
    "nearest_stop_y",
    "planning_factors_stamp",
    "planning_factors_age_sec",
    "planning_factor_count",
    "planning_factors_json",
    "primary_factor_module",
    "primary_factor_behavior",
    "primary_factor_detail",
    "primary_factor_distance_m",
    "primary_factor_velocity_mps",
    "velocity_smoother_status",
    "velocity_smoother_status_age_sec",
    "odom_stamp",
    "odom_age_sec",
    "odom_x",
    "odom_y",
    "odom_z",
    "odom_yaw_rad",
    "odom_vx_mps",
    "odom_vy_mps",
    "odom_speed_mps",
    "odom_yaw_rate_rps",
    "acceleration_stamp",
    "acceleration_age_sec",
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",
] + [
    "{}_{}".format(prefix, suffix)
    for prefix, include_acceleration in (
        ("bvp_path", False),
        ("path_smoother_path", False),
        ("path_optimizer_trajectory", True),
        ("mvp_raw_trajectory", True),
        ("control_trajectory", True),
    )
    for suffix in (
        "stamp",
        "age_sec",
        "point_count",
        "closest_index",
        "closest_velocity_mps",
        *(("closest_acceleration_mps2",) if include_acceleration else ()),
        "min_velocity_ahead_mps",
        "max_velocity_ahead_mps",
        "first_zero_index",
        "first_zero_distance_m",
        "first_zero_x",
        "first_zero_y",
    )
] + [
    "ctrl_cmd_age_sec",
    "ctrl_cmd_type",
    "ctrl_cmd_accel",
    "ctrl_cmd_brake",
    "ctrl_cmd_steering",
    "ctrl_cmd_velocity",
    "ctrl_cmd_acceleration",
    "ego_stamp",
    "ego_age_sec",
    "ego_x",
    "ego_y",
    "ego_z",
    "ego_velocity_x",
    "ego_velocity_y",
    "ego_velocity_z",
    "ego_speed_raw",
    "ego_heading_deg",
    "ego_accel_pedal",
    "ego_brake_pedal",
    "ego_wheel_angle",
    "operation_mode_age_sec",
    "operation_mode_stamp",
    "operation_mode",
    "autoware_control_enabled",
    "operation_mode_in_transition",
    "rosout_json",
]


def _round(value, digits=6):
    if value is None or value == "":
        return ""
    return round(float(value), digits)


def _stamp_to_sec(stamp):
    if stamp is None:
        return ""
    try:
        return _round(stamp.to_sec())
    except (AttributeError, TypeError, ValueError):
        return ""


def _yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _json(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
        default=str,
    )


def _message_stamp(message):
    header = getattr(message, "header", None)
    return _stamp_to_sec(getattr(header, "stamp", None))


def _git_value(repo_root, args):
    if not repo_root or not os.path.isdir(repo_root):
        return ""
    try:
        return subprocess.check_output(
            ["git"] + list(args),
            cwd=repo_root,
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _open_next_numbered_csv(output_directory):
    """Create log_data_01.csv, incrementing from the largest existing number."""
    expanded_directory = os.path.abspath(os.path.expanduser(output_directory))
    os.makedirs(expanded_directory, exist_ok=True)
    pattern = re.compile(r"^log_data_(\d+)\.csv$")
    existing_numbers = []
    for filename in os.listdir(expanded_directory):
        match = pattern.match(filename)
        if match:
            existing_numbers.append(int(match.group(1)))

    number = max(existing_numbers, default=0) + 1
    while True:
        output_path = os.path.join(
            expanded_directory, "log_data_{:02d}.csv".format(number)
        )
        try:
            output_file = open(
                output_path,
                "x",
                newline="",
                encoding="utf-8",
            )
            return output_path, output_file
        except FileExistsError:
            number += 1


class PlannerCsvLogger:
    def __init__(self):
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._latest = {}
        self._arrival_times = {}
        self._recent_rosout = deque(maxlen=max(1, int(rospy.get_param("~max_rosout_per_row", 200))))
        self._subscribers = []
        self._rows_written = 0
        self._closed = False
        self._file = None
        self._writer = None
        self.output_path = ""
        self._start_monotonic = time.monotonic()
        self._start_wall_time = datetime.now().astimezone().isoformat()

        self.rate_hz = max(float(rospy.get_param("~rate_hz", 10.0)), 0.1)
        self.zero_velocity_threshold = max(
            float(rospy.get_param("~zero_velocity_threshold", 0.01)), 0.0
        )
        self.flush_every_n = max(int(rospy.get_param("~flush_every_n", 1)), 1)
        self.scenario_note = str(rospy.get_param("~scenario_note", ""))
        self.auto_start = bool(rospy.get_param("~auto_start", False))
        self.topics = {
            key: str(rospy.get_param("~topics/{}".format(key), topic))
            for key, topic in DEFAULT_TOPICS.items()
        }

        default_repo_root = os.environ.get("AJOUNICE_ROOT", "")
        self.repo_root = str(rospy.get_param("~repo_root", default_repo_root))
        self.git_branch = _git_value(self.repo_root, ["branch", "--show-current"])
        self.git_commit = _git_value(self.repo_root, ["rev-parse", "HEAD"])
        self.git_dirty = bool(_git_value(self.repo_root, ["status", "--porcelain"]))

        default_output_dir = (
            os.path.join(self.repo_root, "planner_logs")
            if self.repo_root
            else os.path.join(os.path.expanduser("~"), ".ros", "planner_logs")
        )
        requested_output_dir = str(rospy.get_param("~output_dir", "")).strip()
        self.output_directory = requested_output_dir or default_output_dir

        rosout_level_name = str(rospy.get_param("~rosout_min_level", "INFO")).upper()
        self.rosout_min_level = {
            "DEBUG": Log.DEBUG,
            "INFO": Log.INFO,
            "WARN": Log.WARN,
            "WARNING": Log.WARN,
            "ERROR": Log.ERROR,
            "FATAL": Log.FATAL,
        }.get(rosout_level_name, Log.INFO)

        self._metadata_json = ""

        self._subscribe("bpp_status", String)
        self._subscribe("bvp_status", String)
        self._subscribe("stop_reasons", StopReasonArray)
        self._subscribe("planning_factors", PlanningFactorArray)
        self._subscribe("velocity_smoother_status", String)
        self._subscribe("odometry", Odometry)
        self._subscribe("acceleration", AccelWithCovarianceStamped)
        self._subscribe("bvp_path", Path)
        self._subscribe("path_smoother_path", Path)
        self._subscribe("path_optimizer_trajectory", Trajectory)
        self._subscribe("mvp_raw_trajectory", Trajectory)
        self._subscribe("control_trajectory", Trajectory)
        self._subscribe("ctrl_cmd", CtrlCmd)
        self._subscribe("ego_status", EgoVehicleStatus)
        self._subscribe("operation_mode", OperationModeState)
        self._subscribers.append(
            rospy.Subscriber(
                self.topics["rosout"],
                Log,
                self._on_rosout,
                queue_size=100,
            )
        )

        self._start_service = rospy.Service("~start", Trigger, self._on_start_service)
        self._stop_service = rospy.Service("~stop", Trigger, self._on_stop_service)
        self._timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_timer)
        rospy.on_shutdown(self.close)

        if self.auto_start:
            success, message = self._start_recording()
            if not success:
                raise RuntimeError(message)
        else:
            rospy.loginfo(
                "planner_csv_logger: ready; call %s or use the RViz panel",
                rospy.resolve_name("~start"),
            )

    def _make_metadata_json(self):
        return _json(
            {
                "logger_started_at": self._start_wall_time,
                "repo_root": self.repo_root,
                "git_branch": self.git_branch,
                "git_commit": self.git_commit,
                "git_dirty": self.git_dirty,
                "rate_hz": self.rate_hz,
                "zero_velocity_threshold": self.zero_velocity_threshold,
                "topics": self.topics,
                "behavior_velocity_planner_params": rospy.get_param(
                    "/behavior_velocity_planner", {}
                ),
            }
        )

    def _start_recording(self):
        with self._write_lock:
            if self._closed:
                return False, "Logger node is shutting down"
            if self._file is not None:
                return False, "Already recording: {}".format(self.output_path)

            self.git_branch = _git_value(self.repo_root, ["branch", "--show-current"])
            self.git_commit = _git_value(self.repo_root, ["rev-parse", "HEAD"])
            self.git_dirty = bool(_git_value(self.repo_root, ["status", "--porcelain"]))
            self._rows_written = 0
            self._start_monotonic = time.monotonic()
            self._start_wall_time = datetime.now().astimezone().isoformat()
            self.output_path, self._file = _open_next_numbered_csv(
                self.output_directory
            )
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
            self._file.flush()
            self._metadata_json = self._make_metadata_json()

            with self._lock:
                self._recent_rosout.clear()

            message = "Recording: {}".format(self.output_path)
            rospy.loginfo("planner_csv_logger: %s", message)
            return True, message

    def _stop_recording(self):
        with self._write_lock:
            if self._file is None:
                return False, "Logging is not active"

            output_path = self.output_path
            rows_written = self._rows_written
            self._file.flush()
            self._file.close()
            self._file = None
            self._writer = None
            message = "Saved {} rows: {}".format(rows_written, output_path)
            rospy.loginfo("planner_csv_logger: %s", message)
            return True, message

    def _on_start_service(self, _request):
        success, message = self._start_recording()
        return TriggerResponse(success=success, message=message)

    def _on_stop_service(self, _request):
        success, message = self._stop_recording()
        return TriggerResponse(success=success, message=message)

    def _subscribe(self, key, message_type):
        topic = self.topics[key]
        if not topic:
            return
        self._subscribers.append(
            rospy.Subscriber(
                topic,
                message_type,
                lambda message, message_key=key: self._store(message_key, message),
                queue_size=10,
                tcp_nodelay=True,
            )
        )

    def _store(self, key, message):
        with self._lock:
            self._latest[key] = message
            self._arrival_times[key] = time.monotonic()

    def _on_rosout(self, message):
        if int(message.level) < int(self.rosout_min_level):
            return
        entry = {
            "stamp": _stamp_to_sec(message.header.stamp),
            "level": int(message.level),
            "node": message.name,
            "message": message.msg,
            "file": message.file,
            "function": message.function,
            "line": int(message.line),
        }
        with self._lock:
            self._recent_rosout.append(entry)

    def _snapshot(self):
        with self._lock:
            latest = dict(self._latest)
            arrivals = dict(self._arrival_times)
            rosout = list(self._recent_rosout)
            self._recent_rosout.clear()
        return latest, arrivals, rosout

    @staticmethod
    def _age(arrivals, key, now):
        arrival = arrivals.get(key)
        return "" if arrival is None else _round(max(now - arrival, 0.0), 3)

    def _stop_reason_summary(self, message):
        output = []
        nearest = None
        if message is None:
            return output, nearest
        for reason in message.stop_reasons:
            factors = []
            for factor in reason.stop_factors:
                item = {
                    "distance_m": _round(factor.dist_to_stop_pose),
                    "stop_pose": {
                        "x": _round(factor.stop_pose.position.x),
                        "y": _round(factor.stop_pose.position.y),
                        "z": _round(factor.stop_pose.position.z),
                    },
                    "factor_points": [
                        {
                            "x": _round(point.x),
                            "y": _round(point.y),
                            "z": _round(point.z),
                        }
                        for point in factor.stop_factor_points
                    ],
                }
                factors.append(item)
                candidate = {
                    "reason": reason.reason,
                    "distance_m": float(factor.dist_to_stop_pose),
                    "x": float(factor.stop_pose.position.x),
                    "y": float(factor.stop_pose.position.y),
                }
                if nearest is None or abs(candidate["distance_m"]) < abs(nearest["distance_m"]):
                    nearest = candidate
            output.append({"reason": reason.reason, "factors": factors})
        return output, nearest

    def _planning_factor_summary(self, message):
        output = []
        primary = None
        if message is None:
            return output, primary
        for factor in message.factors:
            behavior = BEHAVIOR_NAMES.get(int(factor.behavior), str(int(factor.behavior)))
            control_points = []
            for control in factor.control_points:
                control_item = {
                    "x": _round(control.pose.position.x),
                    "y": _round(control.pose.position.y),
                    "z": _round(control.pose.position.z),
                    "velocity_mps": _round(control.velocity),
                    "distance_m": _round(control.distance),
                    "shift_length_m": _round(control.shift_length),
                }
                control_points.append(control_item)
                candidate = {
                    "module": factor.module,
                    "behavior": behavior,
                    "detail": factor.detail,
                    "distance_m": float(control.distance),
                    "velocity_mps": float(control.velocity),
                }
                candidate_key = (
                    0 if behavior == "STOP" else 1,
                    abs(candidate["distance_m"]),
                )
                primary_key = None
                if primary is not None:
                    primary_key = (
                        0 if primary["behavior"] == "STOP" else 1,
                        abs(primary["distance_m"]),
                    )
                if primary is None or candidate_key < primary_key:
                    primary = candidate
            output.append(
                {
                    "module": factor.module,
                    "behavior": behavior,
                    "detail": factor.detail,
                    "is_driving_forward": bool(factor.is_driving_forward),
                    "control_points": control_points,
                    "is_safe": bool(factor.safety_factors.is_safe),
                    "safety_detail": factor.safety_factors.detail,
                }
            )
        return output, primary

    def _path_summary(self, message, odometry):
        empty = {
            "stamp": "",
            "point_count": 0,
            "closest_index": "",
            "closest_velocity": "",
            "closest_acceleration": "",
            "min_velocity_ahead": "",
            "max_velocity_ahead": "",
            "first_zero_index": "",
            "first_zero_distance": "",
            "first_zero_x": "",
            "first_zero_y": "",
        }
        if message is None or not message.points:
            return empty

        points = message.points
        closest_index = 0
        if odometry is not None:
            ego = odometry.pose.pose.position
            closest_index = min(
                range(len(points)),
                key=lambda index: (
                    (points[index].pose.position.x - ego.x) ** 2
                    + (points[index].pose.position.y - ego.y) ** 2
                ),
            )

        velocities = [float(point.longitudinal_velocity_mps) for point in points]
        ahead_velocities = velocities[closest_index:]
        first_zero_index = next(
            (
                index
                for index in range(closest_index, len(points))
                if velocities[index] <= self.zero_velocity_threshold
            ),
            None,
        )
        first_zero_distance = ""
        first_zero_x = ""
        first_zero_y = ""
        if first_zero_index is not None:
            first_zero_distance = 0.0
            for index in range(closest_index + 1, first_zero_index + 1):
                previous = points[index - 1].pose.position
                current = points[index].pose.position
                first_zero_distance += math.hypot(
                    current.x - previous.x, current.y - previous.y
                )
            first_zero_x = points[first_zero_index].pose.position.x
            first_zero_y = points[first_zero_index].pose.position.y

        closest_acceleration = getattr(
            points[closest_index], "acceleration_mps2", ""
        )
        return {
            "stamp": _message_stamp(message),
            "point_count": len(points),
            "closest_index": closest_index,
            "closest_velocity": _round(velocities[closest_index]),
            "closest_acceleration": _round(closest_acceleration) if closest_acceleration != "" else "",
            "min_velocity_ahead": _round(min(ahead_velocities)),
            "max_velocity_ahead": _round(max(ahead_velocities)),
            "first_zero_index": "" if first_zero_index is None else first_zero_index,
            "first_zero_distance": _round(first_zero_distance),
            "first_zero_x": _round(first_zero_x),
            "first_zero_y": _round(first_zero_y),
        }

    def _add_path_summary(
        self,
        row,
        prefix,
        topic_key,
        latest,
        arrivals,
        odometry,
        now_monotonic,
        include_acceleration=False,
    ):
        summary = self._path_summary(latest.get(topic_key), odometry)
        row.update(
            {
                "{}_stamp".format(prefix): summary["stamp"],
                "{}_age_sec".format(prefix): self._age(
                    arrivals, topic_key, now_monotonic
                ),
                "{}_point_count".format(prefix): summary["point_count"],
                "{}_closest_index".format(prefix): summary["closest_index"],
                "{}_closest_velocity_mps".format(prefix): summary[
                    "closest_velocity"
                ],
                "{}_min_velocity_ahead_mps".format(prefix): summary[
                    "min_velocity_ahead"
                ],
                "{}_max_velocity_ahead_mps".format(prefix): summary[
                    "max_velocity_ahead"
                ],
                "{}_first_zero_index".format(prefix): summary[
                    "first_zero_index"
                ],
                "{}_first_zero_distance_m".format(prefix): summary[
                    "first_zero_distance"
                ],
                "{}_first_zero_x".format(prefix): summary["first_zero_x"],
                "{}_first_zero_y".format(prefix): summary["first_zero_y"],
            }
        )
        if include_acceleration:
            row["{}_closest_acceleration_mps2".format(prefix)] = summary[
                "closest_acceleration"
            ]

    def _build_row(self, latest, arrivals, rosout, now_monotonic):
        row = {field: "" for field in CSV_FIELDS}
        row.update(
            {
                "wall_time_iso": datetime.now().astimezone().isoformat(),
                "ros_time": _round(rospy.Time.now().to_sec()),
                "elapsed_sec": _round(now_monotonic - self._start_monotonic, 3),
                "git_branch": self.git_branch,
                "git_commit": self.git_commit,
                "git_dirty": self.git_dirty,
                "scenario_note": self.scenario_note,
                "metadata_json": self._metadata_json if self._rows_written == 0 else "",
                "rosout_json": _json(rosout) if rosout else "",
            }
        )

        bpp_status = latest.get("bpp_status")
        row["bpp_status"] = bpp_status.data if bpp_status is not None else ""
        row["bpp_status_age_sec"] = self._age(arrivals, "bpp_status", now_monotonic)

        bvp_status = latest.get("bvp_status")
        row["bvp_status"] = bvp_status.data if bvp_status is not None else ""
        row["bvp_status_age_sec"] = self._age(arrivals, "bvp_status", now_monotonic)

        stop_reasons = latest.get("stop_reasons")
        stop_summary, nearest_stop = self._stop_reason_summary(stop_reasons)
        row.update(
            {
                "stop_reasons_stamp": _message_stamp(stop_reasons),
                "stop_reasons_age_sec": self._age(arrivals, "stop_reasons", now_monotonic),
                "stop_reason_count": len(stop_summary),
                "stop_reasons_json": _json(stop_summary) if stop_summary else "",
            }
        )
        if nearest_stop is not None:
            row.update(
                {
                    "nearest_stop_reason": nearest_stop["reason"],
                    "nearest_stop_distance_m": _round(nearest_stop["distance_m"]),
                    "nearest_stop_x": _round(nearest_stop["x"]),
                    "nearest_stop_y": _round(nearest_stop["y"]),
                }
            )

        planning_factors = latest.get("planning_factors")
        factor_summary, primary_factor = self._planning_factor_summary(planning_factors)
        row.update(
            {
                "planning_factors_stamp": _message_stamp(planning_factors),
                "planning_factors_age_sec": self._age(
                    arrivals, "planning_factors", now_monotonic
                ),
                "planning_factor_count": len(factor_summary),
                "planning_factors_json": _json(factor_summary) if factor_summary else "",
            }
        )
        if primary_factor is not None:
            row.update(
                {
                    "primary_factor_module": primary_factor["module"],
                    "primary_factor_behavior": primary_factor["behavior"],
                    "primary_factor_detail": primary_factor["detail"],
                    "primary_factor_distance_m": _round(primary_factor["distance_m"]),
                    "primary_factor_velocity_mps": _round(primary_factor["velocity_mps"]),
                }
            )

        smoother_status = latest.get("velocity_smoother_status")
        row["velocity_smoother_status"] = (
            smoother_status.data if smoother_status is not None else ""
        )
        row["velocity_smoother_status_age_sec"] = self._age(
            arrivals, "velocity_smoother_status", now_monotonic
        )

        odometry = latest.get("odometry")
        if odometry is not None:
            position = odometry.pose.pose.position
            linear = odometry.twist.twist.linear
            row.update(
                {
                    "odom_stamp": _message_stamp(odometry),
                    "odom_x": _round(position.x),
                    "odom_y": _round(position.y),
                    "odom_z": _round(position.z),
                    "odom_yaw_rad": _round(
                        _yaw_from_quaternion(odometry.pose.pose.orientation)
                    ),
                    "odom_vx_mps": _round(linear.x),
                    "odom_vy_mps": _round(linear.y),
                    "odom_speed_mps": _round(math.hypot(linear.x, linear.y)),
                    "odom_yaw_rate_rps": _round(odometry.twist.twist.angular.z),
                }
            )
        row["odom_age_sec"] = self._age(arrivals, "odometry", now_monotonic)

        acceleration = latest.get("acceleration")
        if acceleration is not None:
            linear = acceleration.accel.accel.linear
            row.update(
                {
                    "acceleration_stamp": _message_stamp(acceleration),
                    "accel_x_mps2": _round(linear.x),
                    "accel_y_mps2": _round(linear.y),
                    "accel_z_mps2": _round(linear.z),
                }
            )
        row["acceleration_age_sec"] = self._age(
            arrivals, "acceleration", now_monotonic
        )

        for prefix, topic_key, include_acceleration in (
            ("bvp_path", "bvp_path", False),
            ("path_smoother_path", "path_smoother_path", False),
            (
                "path_optimizer_trajectory",
                "path_optimizer_trajectory",
                True,
            ),
            ("mvp_raw_trajectory", "mvp_raw_trajectory", True),
            ("control_trajectory", "control_trajectory", True),
        ):
            self._add_path_summary(
                row,
                prefix,
                topic_key,
                latest,
                arrivals,
                odometry,
                now_monotonic,
                include_acceleration,
            )

        ctrl_cmd = latest.get("ctrl_cmd")
        if ctrl_cmd is not None:
            row.update(
                {
                    "ctrl_cmd_type": int(ctrl_cmd.longlCmdType),
                    "ctrl_cmd_accel": _round(ctrl_cmd.accel),
                    "ctrl_cmd_brake": _round(ctrl_cmd.brake),
                    "ctrl_cmd_steering": _round(ctrl_cmd.steering),
                    "ctrl_cmd_velocity": _round(ctrl_cmd.velocity),
                    "ctrl_cmd_acceleration": _round(ctrl_cmd.acceleration),
                }
            )
        row["ctrl_cmd_age_sec"] = self._age(arrivals, "ctrl_cmd", now_monotonic)

        ego_status = latest.get("ego_status")
        if ego_status is not None:
            velocity = ego_status.velocity
            row.update(
                {
                    "ego_stamp": _message_stamp(ego_status),
                    "ego_x": _round(ego_status.position.x),
                    "ego_y": _round(ego_status.position.y),
                    "ego_z": _round(ego_status.position.z),
                    "ego_velocity_x": _round(velocity.x),
                    "ego_velocity_y": _round(velocity.y),
                    "ego_velocity_z": _round(velocity.z),
                    "ego_speed_raw": _round(
                        math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                    ),
                    "ego_heading_deg": _round(ego_status.heading),
                    "ego_accel_pedal": _round(ego_status.accel),
                    "ego_brake_pedal": _round(ego_status.brake),
                    "ego_wheel_angle": _round(ego_status.wheel_angle),
                }
            )
        row["ego_age_sec"] = self._age(arrivals, "ego_status", now_monotonic)

        operation_mode = latest.get("operation_mode")
        if operation_mode is not None:
            row.update(
                {
                    "operation_mode_stamp": _stamp_to_sec(operation_mode.stamp),
                    "operation_mode": OPERATION_MODE_NAMES.get(
                        int(operation_mode.mode), str(int(operation_mode.mode))
                    ),
                    "autoware_control_enabled": bool(
                        operation_mode.is_autoware_control_enabled
                    ),
                    "operation_mode_in_transition": bool(
                        operation_mode.is_in_transition
                    ),
                }
            )
        row["operation_mode_age_sec"] = self._age(
            arrivals, "operation_mode", now_monotonic
        )
        return row

    def _on_timer(self, _event):
        if not self._write_lock.acquire(False):
            return
        try:
            if self._closed or self._file is None:
                return
            now_monotonic = time.monotonic()
            latest, arrivals, rosout = self._snapshot()
            row = self._build_row(latest, arrivals, rosout, now_monotonic)
            self._writer.writerow(row)
            self._rows_written += 1
            if self._rows_written % self.flush_every_n == 0:
                self._file.flush()
        except Exception as error:  # Keep the logger alive during a scenario run.
            rospy.logerr_throttle(1.0, "planner_csv_logger write failed: %s", error)
        finally:
            self._write_lock.release()

    def close(self):
        self._closed = True
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.shutdown()
        if self._file is not None:
            self._stop_recording()


def main():
    rospy.init_node("planner_csv_logger")
    PlannerCsvLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
