#!/usr/bin/env python3
"""Observer-only trajectory validator with immutable CSV run artifacts."""

import csv
import hashlib
import json
import math
import os
import shutil
import sys
import threading
from datetime import datetime

import rospy
import yaml
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from trajectory_validator.srv import SetHistoryVisibility, SetHistoryVisibilityResponse


SAMPLE_COLUMNS = [
    "sample_index", "stamp_sec", "dt_sec", "ego_x_m", "ego_y_m", "ego_z_m", "ego_yaw_rad",
    "ref_s_m", "ref_x_m", "ref_y_m", "ref_z_m", "ref_yaw_rad", "ref_lanelet_id",
    "ref_curvature_1pm", "ref_speed_mps", "ref_acceleration_mps2", "remaining_distance_m",
    "lat_err_m", "heading_err_rad", "ref_distance_m", "long_vel_mps", "lat_vel_mps",
    "long_acc_mps2", "lat_acc_mps2", "ctrl_stamp_sec", "ctrl_age_sec", "longl_cmd_type",
    "front_steer_rad", "steering_rate_cmd_radps", "accel_cmd", "brake_cmd", "velocity_cmd_mps",
    "acceleration_cmd_mps2", "lookahead_distance_m", "control_tick_elapsed_ms", "control_deadline_miss",
    "ego_valid", "reference_valid", "ctrl_valid", "projection_valid",
]
REFERENCE_COLUMNS = [
    "s_m", "x_m", "y_m", "z_m", "yaw_rad", "lanelet_id", "speed_limit_kph", "requested_speed_kph",
    "curvature", "target_speed_mps", "acceleration_mps2", "time_from_start_sec",
]
EVENT_COLUMNS = ["event_index", "stamp_sec", "event_type", "severity", "progress_s_m", "ego_x_m", "ego_y_m", "detail"]
INDEX_COLUMNS = [
    "run_id", "run_label", "route_id", "controller", "map_version", "frame_id", "control_rate_hz",
    "ego_state_mode", "reference_checksum", "created_at", "status", "mean_ade_m", "fde_m",
    "lateral_rms_m", "heading_rms_rad", "speed_rms_mps", "artifact_relpath",
]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sha256(filename):
    digest = hashlib.sha256()
    with open(filename, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value):
    return "" if value is None or not math.isfinite(float(value)) else "%.9g" % float(value)


class Reference:
    def __init__(self, message):
        self.stamp = message.header.stamp
        self.frame = message.header.frame_id
        self.points = []
        s = 0.0
        for message_point in message.points:
            x, y, z = message_point.pose.position.x, message_point.pose.position.y, message_point.pose.position.z
            if self.points:
                s += math.hypot(x - self.points[-1]["x"], y - self.points[-1]["y"])
            self.points.append({
                "s": s, "x": x, "y": y, "z": z, "yaw": yaw_from_quaternion(message_point.pose.orientation),
                "speed": message_point.longitudinal_velocity_mps, "acceleration": message_point.acceleration_mps2,
            })
        self.total_length = s

    def valid(self):
        return len(self.points) >= 2

    def same_content(self, other):
        """Ignore header-stamp-only republishes; detect any geometry/profile change."""
        if other is None or len(self.points) != len(other.points):
            return False
        for left, right in zip(self.points, other.points):
            for key in ("x", "y", "z", "yaw", "speed", "acceleration"):
                if abs(left[key] - right[key]) > 1e-6:
                    return False
        return True

    def at_s(self, s):
        if s <= 0.0:
            return dict(self.points[0])
        if s >= self.total_length:
            return dict(self.points[-1])
        index = 1
        while index < len(self.points) and self.points[index]["s"] < s:
            index += 1
        a, b = self.points[index - 1], self.points[index]
        fraction = (s - a["s"]) / max(b["s"] - a["s"], 1e-9)
        return {
            "s": s, "x": a["x"] + fraction * (b["x"] - a["x"]), "y": a["y"] + fraction * (b["y"] - a["y"]),
            "z": a["z"] + fraction * (b["z"] - a["z"]), "yaw": a["yaw"] + fraction * wrap(b["yaw"] - a["yaw"]),
            "speed": a["speed"] + fraction * (b["speed"] - a["speed"]),
            "acceleration": a["acceleration"] + fraction * (b["acceleration"] - a["acceleration"]),
            "index": index - 1,
        }

    def project(self, x, y):
        best = None
        for index in range(len(self.points) - 1):
            a, b = self.points[index], self.points[index + 1]
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            squared = dx * dx + dy * dy
            if squared <= 1e-12:
                continue
            fraction = min(1.0, max(0.0, ((x - a["x"]) * dx + (y - a["y"]) * dy) / squared))
            px, py = a["x"] + fraction * dx, a["y"] + fraction * dy
            distance = math.hypot(x - px, y - py)
            if best is None or distance < best["distance"]:
                value = self.at_s(a["s"] + fraction * (b["s"] - a["s"]))
                ex, ey = x - px, y - py
                value.update({"distance": distance, "lateral": -math.sin(value["yaw"]) * ex + math.cos(value["yaw"]) * ey})
                best = value
        return best


class Validator:
    def __init__(self):
        self.lock = threading.RLock()
        self.config = rospy.get_param("~", {})
        self.validator_cfg = self.config.get("validator", {})
        # The reference header owns the world frame.  The controller publishes a
        # normalized ego-state Odometry, so validator config never duplicates
        # generator map settings or the controller's MORAI/localization toggle.
        self.frame = None
        self.source = "unknown"
        self.gate = self.validator_cfg.get("validation_gate", {})
        self.ctrl_timeout = float(self.validator_cfg.get("ctrl_timeout_sec", 0.30))
        self.safe_stop_grace = float(self.validator_cfg.get("safe_stop_grace_sec", 0.50))
        self.projection_jump = float(self.validator_cfg.get("projection_jump_m", 15.0))
        self.history_root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "history")
        self.controller_dir = self.validator_cfg.get("history_controller", "pp_pi")
        self.history_dir = os.path.join(self.history_root, self.controller_dir)
        os.makedirs(self.history_dir, exist_ok=True)
        self.reference = None
        self.report_rows = []
        self.generator_status = {}
        self.run_config_text = ""
        self.run_config = {}
        self.last_ctrl = None
        self.last_ctrl_stamp = None
        self.last_ctrl_steer = None
        self.last_ctrl_steer_stamp = None
        self.debug = {"lookahead": None, "elapsed_ms": None, "deadline": False}
        self.state = "ARMED"
        self.current_path = []
        self.selected_history = {}
        self.run = None
        self.sample_file = None
        self.sample_writer = None
        self.event_file = None
        self.event_writer = None
        self.metric_values = {key: [] for key in ("ade", "lat", "heading", "speed", "steer")}
        self.deadline_miss_count = 0
        self.last_sample = None
        self.last_progress = None
        self.publishers = self._make_publishers()
        # The reference trajectory is published latched.  A callback can therefore
        # run synchronously while rospy.Subscriber is being constructed, so every
        # publisher it uses must exist before registering any subscriber.
        self.marker_pub = rospy.Publisher("/validation/markers", MarkerArray, queue_size=10, latch=True)
        rospy.Subscriber("/planning/trajectory/reference", Trajectory, self.reference_callback, queue_size=1)
        rospy.Subscriber("/planning/trajectory/generator_status", String, self.generator_status_callback, queue_size=1)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_callback, queue_size=100)
        rospy.Subscriber("/control/purepursuit_pi/run_config", String, self.run_config_callback, queue_size=1)
        rospy.Subscriber("/control/purepursuit_pi/debug/lookahead_distance", Float32, self.lookahead_callback, queue_size=10)
        rospy.Subscriber("/control/purepursuit_pi/debug/control_tick_elapsed_ms", Float32, self.elapsed_callback, queue_size=10)
        rospy.Subscriber("/control/purepursuit_pi/debug/control_deadline_miss", Bool, self.deadline_callback, queue_size=10)
        rospy.Subscriber("/control/purepursuit_pi/debug/state", String, self.controller_state_callback, queue_size=10)
        rospy.Subscriber("/control/purepursuit_pi/ego_state", Odometry, self.ego_callback, queue_size=100)
        self.history_service = rospy.Service("/trajectory_validator/history/set_visible", SetHistoryVisibility, self.history_visibility)
        rospy.loginfo("trajectory_validator armed: normalized ego topic=/control/purepursuit_pi/ego_state, history=%s", self.history_dir)

    def _make_publishers(self):
        topics = [
            "lateral/lat_err", "lateral/heading_err", "lateral/steering_angle_cmd", "lateral/steering_rate_cmd",
            "lateral/lat_vel", "lateral/lat_acc", "lateral/lookahead_distance", "lateral/curvature_ref", "lateral/lat_err_abs",
            "longitudinal/long_vel", "longitudinal/target_vel", "longitudinal/long_vel_err", "longitudinal/long_acc",
            "longitudinal/target_acc", "longitudinal/accel_cmd", "longitudinal/brake_cmd", "longitudinal/remaining_distance_m",
            "metrics/mean_ade", "metrics/fde", "metrics/lateral_rms", "metrics/heading_rms", "metrics/speed_rms",
            "metrics/max_lat_err", "metrics/p95_lat_err", "metrics/steer_saturation_rate", "metrics/control_deadline_miss_count",
        ]
        return {topic: rospy.Publisher("/validation/" + topic, Float32, queue_size=20) for topic in topics}

    def _publish(self, topic, value):
        if value is not None and math.isfinite(float(value)):
            self.publishers[topic].publish(Float32(data=float(value)))

    def reference_callback(self, message):
        if not message.header.frame_id:
            rospy.logerr_throttle(1.0, "trajectory_validator: reference frame_id is empty")
            return
        reference = Reference(message)
        if not reference.valid():
            rospy.logerr("trajectory_validator: reference has fewer than two points")
            return
        with self.lock:
            if self.reference and reference.same_content(self.reference):
                # trajectory_generator republishes an immutable snapshot at low rate.
                # Its header stamp is not a new route/profile and must not split a run.
                return
            if self.run:
                self.abort("reference_changed", rospy.Time.now())
            self.reference = reference
            self.frame = reference.frame
            self.current_path = []
            self.last_progress = None
        self._publish_markers()

    def generator_status_callback(self, message):
        try:
            status = json.loads(message.data)
            if not status.get("ok", False):
                rospy.logerr("trajectory_generator reported failure: %s", status.get("error", "unknown"))
                return
            report = status.get("profile_report_path", "")
            if not report or not os.path.isfile(report):
                raise ValueError("missing profile_report_path")
            rows = []
            with open(report, newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                if reader.fieldnames != REFERENCE_COLUMNS:
                    raise ValueError("profile report schema differs from reference.csv contract")
                rows = list(reader)
            if len(rows) < 2:
                raise ValueError("profile report has fewer than two rows")
            with self.lock:
                self.generator_status, self.report_rows = status, rows
        except (ValueError, OSError, json.JSONDecodeError) as error:
            rospy.logerr("trajectory_validator cannot use generator report: %s", error)

    def run_config_callback(self, message):
        try:
            parsed = yaml.safe_load(message.data) or {}
            snapshot = parsed.get("run_snapshot", {})
            with self.lock:
                if self.run and message.data != self.run_config_text:
                    self.abort("controller_config_changed", rospy.Time.now())
                self.run_config_text, self.run_config = message.data, parsed
                self.source = snapshot.get("ego_state_source", "unknown")
        except yaml.YAMLError as error:
            rospy.logerr("trajectory_validator received invalid run_config YAML: %s", error)

    def ctrl_callback(self, message):
        with self.lock:
            stamp = rospy.Time.now()
            self.last_ctrl, self.last_ctrl_stamp = message, stamp

    def lookahead_callback(self, message): self.debug["lookahead"] = message.data
    def elapsed_callback(self, message): self.debug["elapsed_ms"] = message.data
    def deadline_callback(self, message): self.debug["deadline"] = bool(message.data)

    def controller_state_callback(self, message):
        # A controller started before its first Ego/TF message legitimately publishes
        # SAFE_STOP.  Do not turn that already-latched pre-start state into a false
        # aborted run before the timer can observe the new input cache.
        # FINISHED is the controller's normal finite-reference terminal state.
        # The validator owns the independent finish gate and must be allowed to
        # close the artifact as complete when that state arrives first.
        if (self.run and message.data.startswith("SAFE_STOP") and "FINISHED" not in message.data and
                rospy.Time.now().to_sec() - self.run["started_stamp_sec"] > self.safe_stop_grace):
            with self.lock:
                self.abort("safe_stop", rospy.Time.now())

    def ego_callback(self, message):
        if self.frame and message.header.frame_id != self.frame:
            rospy.logerr_throttle(1.0, "trajectory_validator: normalized ego frame %s differs from reference %s", message.header.frame_id, self.frame)
            return
        pose = message.pose.pose
        self.process_ego({
            "stamp": message.header.stamp,
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "yaw": yaw_from_quaternion(pose.orientation),
            "long_vel": message.twist.twist.linear.x, "lat_vel": message.twist.twist.linear.y,
        })

    def _gate_value(self, name, default): return float(self.gate.get(name, default))

    def _near_start(self, ego):
        start = self.reference.points[0]
        distance = math.hypot(ego["x"] - start["x"], ego["y"] - start["y"])
        heading = abs(wrap(ego["yaw"] - start["yaw"]))
        return (distance <= self._gate_value("start_distance_m", 3.0) and
                heading <= math.radians(self._gate_value("start_heading_tolerance_deg", 45.0)) and
                abs(ego["long_vel"]) >= self._gate_value("start_speed_mps", 0.5))

    def _near_finish(self, ego, projection):
        finish = self.reference.points[-1]
        distance = math.hypot(ego["x"] - finish["x"], ego["y"] - finish["y"])
        heading = abs(wrap(ego["yaw"] - finish["yaw"]))
        return (projection["s"] >= self.reference.total_length - self._gate_value("finish_distance_m", 3.0) and
                distance <= self._gate_value("finish_distance_m", 3.0) and
                heading <= math.radians(self._gate_value("finish_heading_tolerance_deg", 45.0)) and
                abs(ego["long_vel"]) <= self._gate_value("finish_speed_mps", 1.0))

    def process_ego(self, ego):
        with self.lock:
            if not self.reference or not self.reference.valid():
                return
            projection = self.reference.project(ego["x"], ego["y"])
            if projection is None:
                return
            if self.state == "ARMED":
                if not self.run_config_text:
                    rospy.logwarn_throttle(2.0, "trajectory_validator waiting for latched PP run_config")
                elif not self.report_rows:
                    rospy.logwarn_throttle(2.0, "trajectory_validator waiting for generator profile report")
                elif self._near_start(ego):
                    self.start(ego, projection)
            if self.state == "RECORDING":
                self.record(ego, projection)
                if self._near_finish(ego, projection):
                    self.finish(ego, projection)
            self._publish_markers(ego, projection)

    def _run_label(self):
        return self.run_config.get("run_snapshot", {}).get("run_label", "pp_unknown")

    def start(self, ego, projection):
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_id = "%s__%s" % (now, self._run_label())
        temporary = os.path.join(self.history_dir, ".in_progress_" + run_id)
        final = os.path.join(self.history_dir, run_id)
        suffix = 1
        while os.path.exists(temporary) or os.path.exists(final):
            temporary = os.path.join(self.history_dir, ".in_progress_%s_%02d" % (run_id, suffix))
            final = os.path.join(self.history_dir, "%s_%02d" % (run_id, suffix)); suffix += 1
        os.makedirs(temporary)
        parameters = os.path.join(temporary, "parameters.yaml")
        with open(parameters, "w", encoding="utf-8") as stream:
            stream.write(self.run_config_text)
            if not self.run_config_text.endswith("\n"): stream.write("\n")
        reference_file = os.path.join(temporary, "reference.csv")
        with open(reference_file, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=REFERENCE_COLUMNS); writer.writeheader(); writer.writerows(self.report_rows)
        self.sample_file = open(os.path.join(temporary, "samples.csv"), "w", newline="", encoding="utf-8")
        self.sample_writer = csv.DictWriter(self.sample_file, fieldnames=SAMPLE_COLUMNS); self.sample_writer.writeheader()
        self.event_file = open(os.path.join(temporary, "events.csv"), "w", newline="", encoding="utf-8")
        self.event_writer = csv.DictWriter(self.event_file, fieldnames=EVENT_COLUMNS); self.event_writer.writeheader()
        self.run = {"id": os.path.basename(final), "temporary": temporary, "final": final, "started_at": datetime.now().isoformat(), "started_stamp_sec": ego["stamp"].to_sec(),
                    "reference_checksum": sha256(reference_file), "generator_checksum": self.generator_status.get("profile_checksum", ""), "events": 0}
        self.metric_values = {key: [] for key in ("ade", "lat", "heading", "speed", "steer")}
        self.deadline_miss_count = 0; self.last_sample = None; self.last_progress = None; self.current_path = []
        self.state = "RECORDING"
        self.event("armed", "info", ego["stamp"], projection, ego, "auto gate ready")
        self.event("start", "info", ego["stamp"], projection, ego, "automatic start gate passed")
        rospy.loginfo("trajectory_validator started %s", self.run["id"])

    def _closest_report(self, s):
        if not self.report_rows: return {}
        # resampling makes the index estimate enough, while retaining the exact report metadata.
        ratio = s / max(self.reference.total_length, 1e-9)
        return self.report_rows[min(len(self.report_rows) - 1, max(0, int(round(ratio * (len(self.report_rows) - 1)))))]

    def record(self, ego, projection):
        stamp_sec = ego["stamp"].to_sec()
        dt = None if self.last_sample is None else stamp_sec - self.last_sample["stamp_sec"]
        long_acc = lat_acc = None
        if self.last_sample and 1e-3 <= dt <= 1.0:
            long_acc = (ego["long_vel"] - self.last_sample["long_vel"]) / dt
            lat_acc = (ego["lat_vel"] - self.last_sample["lat_vel"]) / dt
        report = self._closest_report(projection["s"])
        curvature = float(report.get("curvature", 0.0)) if report else 0.0
        ctrl_valid = self.last_ctrl is not None and self.last_ctrl_stamp is not None and (ego["stamp"] - self.last_ctrl_stamp).to_sec() <= self.ctrl_timeout
        steering_rate = None
        if ctrl_valid and self.last_ctrl_steer is not None and self.last_ctrl_steer_stamp is not None:
            ctrl_dt = (self.last_ctrl_stamp - self.last_ctrl_steer_stamp).to_sec()
            if ctrl_dt > 1e-4: steering_rate = (self.last_ctrl.front_steer - self.last_ctrl_steer) / ctrl_dt
        if ctrl_valid:
            self.last_ctrl_steer, self.last_ctrl_steer_stamp = self.last_ctrl.front_steer, self.last_ctrl_stamp
        heading = wrap(ego["yaw"] - projection["yaw"])
        if self.last_progress is not None and projection["s"] + 1.0 < self.last_progress:
            self.event("progress_regression", "warning", ego["stamp"], projection, ego, "finite route progress moved backward")
        if self.last_progress is not None and projection["s"] - self.last_progress > self.projection_jump:
            self.event("projection_jump", "warning", ego["stamp"], projection, ego, "projection jump exceeds configured threshold")
        self.last_progress = projection["s"]
        if not ctrl_valid: self.event("ctrl_stale", "warning", ego["stamp"], projection, ego, "no recent /ctrl_cmd")
        if self.debug["deadline"]: self.deadline_miss_count += 1; self.event("control_deadline_miss", "warning", ego["stamp"], projection, ego, "controller reported deadline miss")
        row = {
            "sample_index": len(self.current_path), "stamp_sec": finite(stamp_sec), "dt_sec": finite(dt),
            "ego_x_m": finite(ego["x"]), "ego_y_m": finite(ego["y"]), "ego_z_m": finite(ego["z"]), "ego_yaw_rad": finite(ego["yaw"]),
            "ref_s_m": finite(projection["s"]), "ref_x_m": finite(projection["x"]), "ref_y_m": finite(projection["y"]), "ref_z_m": finite(projection["z"]), "ref_yaw_rad": finite(projection["yaw"]),
            "ref_lanelet_id": report.get("lanelet_id", ""), "ref_curvature_1pm": finite(curvature), "ref_speed_mps": finite(projection["speed"]), "ref_acceleration_mps2": finite(projection["acceleration"]),
            "remaining_distance_m": finite(self.reference.total_length - projection["s"]), "lat_err_m": finite(projection["lateral"]), "heading_err_rad": finite(heading), "ref_distance_m": finite(projection["distance"]),
            "long_vel_mps": finite(ego["long_vel"]), "lat_vel_mps": finite(ego["lat_vel"]), "long_acc_mps2": finite(long_acc), "lat_acc_mps2": finite(lat_acc),
            "ctrl_stamp_sec": finite(self.last_ctrl_stamp.to_sec()) if ctrl_valid else "", "ctrl_age_sec": finite((ego["stamp"] - self.last_ctrl_stamp).to_sec()) if ctrl_valid else "",
            "longl_cmd_type": self.last_ctrl.longlCmdType if ctrl_valid else "", "front_steer_rad": finite(self.last_ctrl.front_steer) if ctrl_valid else "", "steering_rate_cmd_radps": finite(steering_rate),
            "accel_cmd": finite(self.last_ctrl.accel) if ctrl_valid else "", "brake_cmd": finite(self.last_ctrl.brake) if ctrl_valid else "", "velocity_cmd_mps": finite(self.last_ctrl.velocity / 3.6) if ctrl_valid else "", "acceleration_cmd_mps2": finite(self.last_ctrl.acceleration) if ctrl_valid else "",
            "lookahead_distance_m": finite(self.debug["lookahead"]), "control_tick_elapsed_ms": finite(self.debug["elapsed_ms"]), "control_deadline_miss": int(self.debug["deadline"]),
            "ego_valid": 1, "reference_valid": 1, "ctrl_valid": int(ctrl_valid), "projection_valid": 1,
        }
        self.sample_writer.writerow(row); self.sample_file.flush()
        self.metric_values["ade"].append(projection["distance"]); self.metric_values["lat"].append(projection["lateral"]); self.metric_values["heading"].append(heading); self.metric_values["speed"].append(projection["speed"] - ego["long_vel"])
        if ctrl_valid:
            self.metric_values["steer"].append(abs(self.last_ctrl.front_steer))
        self.last_sample = {"stamp_sec": stamp_sec, "long_vel": ego["long_vel"], "lat_vel": ego["lat_vel"]}
        self.current_path.append((ego["x"], ego["y"], ego["z"]))
        self._publish_sample(projection, ego, heading, long_acc, lat_acc, steering_rate, ctrl_valid)

    def _publish_sample(self, projection, ego, heading, long_acc, lat_acc, steering_rate, ctrl_valid):
        values = {
            "lateral/lat_err": projection["lateral"], "lateral/heading_err": heading, "lateral/lat_vel": ego["lat_vel"], "lateral/lat_acc": lat_acc,
            "lateral/lat_err_abs": abs(projection["lateral"]), "lateral/curvature_ref": float(self._closest_report(projection["s"]).get("curvature", 0.0)), "lateral/lookahead_distance": self.debug["lookahead"],
            "longitudinal/long_vel": ego["long_vel"], "longitudinal/target_vel": projection["speed"], "longitudinal/long_vel_err": projection["speed"] - ego["long_vel"], "longitudinal/long_acc": long_acc,
            "longitudinal/target_acc": projection["acceleration"], "longitudinal/remaining_distance_m": self.reference.total_length - projection["s"],
        }
        if ctrl_valid:
            values.update({"lateral/steering_angle_cmd": self.last_ctrl.front_steer, "lateral/steering_rate_cmd": steering_rate, "longitudinal/accel_cmd": self.last_ctrl.accel, "longitudinal/brake_cmd": self.last_ctrl.brake})
        for topic, value in values.items(): self._publish(topic, value)
        self._publish_metrics(final=False)

    def event(self, event_type, severity, stamp, projection=None, ego=None, detail=""):
        if not self.event_writer: return
        self.event_writer.writerow({"event_index": self.run["events"], "stamp_sec": finite(stamp.to_sec()), "event_type": event_type, "severity": severity,
                                   "progress_s_m": finite(projection["s"]) if projection else "", "ego_x_m": finite(ego["x"]) if ego else "", "ego_y_m": finite(ego["y"]) if ego else "", "detail": detail})
        self.event_file.flush(); self.run["events"] += 1

    def metrics(self, fde=None):
        def rms(values): return math.sqrt(sum(value * value for value in values) / len(values)) if values else None
        lat_abs = sorted(abs(value) for value in self.metric_values["lat"])
        percentile = None if not lat_abs else lat_abs[min(len(lat_abs) - 1, int(math.ceil(0.95 * len(lat_abs))) - 1)]
        steer_cap = float(self.run_config.get("run_snapshot", {}).get("steering_limit_rad", 0.35))
        saturation = None if not self.metric_values["steer"] else sum(value >= 0.98 * steer_cap for value in self.metric_values["steer"]) / float(len(self.metric_values["steer"]))
        return {"mean_ade": sum(self.metric_values["ade"]) / len(self.metric_values["ade"]) if self.metric_values["ade"] else None,
                "fde": fde, "lateral_rms": rms(self.metric_values["lat"]), "heading_rms": rms(self.metric_values["heading"]), "speed_rms": rms(self.metric_values["speed"]),
                "max_lat_err": max(lat_abs) if lat_abs else None, "p95_lat_err": percentile, "steer_saturation_rate": saturation, "control_deadline_miss_count": self.deadline_miss_count}

    def _publish_metrics(self, final=False, fde=None):
        values = self.metrics(fde)
        for name, value in values.items(): self._publish("metrics/" + name, value)
        return values

    def finish(self, ego, projection):
        fde = math.hypot(ego["x"] - self.reference.points[-1]["x"], ego["y"] - self.reference.points[-1]["y"])
        self.event("finish", "info", ego["stamp"], projection, ego, "finite route finish gate passed")
        self.finalize("complete", self._publish_metrics(final=True, fde=fde))

    def abort(self, reason, stamp):
        if not self.run: return
        projection = self.reference.project(self.current_path[-1][0], self.current_path[-1][1]) if self.current_path else None
        ego = {"x": self.current_path[-1][0], "y": self.current_path[-1][1]} if self.current_path else None
        self.event("aborted", "error", stamp, projection, ego, reason)
        self.finalize("aborted", self._publish_metrics(final=True))

    def finalize(self, status, metrics):
        if not self.run: return
        self.sample_file.close(); self.event_file.close()
        snapshot = self.run_config.get("run_snapshot", {})
        metadata = {
            "run_label": self._run_label(), "run_id": self.run["id"], "route_id": self.generator_status.get("route_id", "unknown"),
            "map_version": self.generator_status.get("map_version", "unknown"), "frame_id": self.frame,
            "controller": snapshot.get("controller", "purepursuit_pi"), "control_rate_hz": snapshot.get("resolved_control_rate_hz"),
            "ego_state_topic": snapshot.get("normalized_ego_state_topic", "/control/purepursuit_pi/ego_state"), "ego_state_mode": self.source,
            "vehicle_model": snapshot.get("vehicle_model", "unknown"), "wheelbase_m": snapshot.get("wheelbase_m"),
            "morai_longlCmdType": 1, "generator_profile_checksum": self.run["generator_checksum"], "reference_checksum": self.run["reference_checksum"],
            "start_condition": "gate_%sm_heading%g_speed%g" % (self._gate_value("start_distance_m", 3.0), self._gate_value("start_heading_tolerance_deg", 45.0), self._gate_value("start_speed_mps", 0.5)),
            "created_at": self.run["started_at"], "status": status,
        }
        with open(os.path.join(self.run["temporary"], "metadata.yaml"), "w", encoding="utf-8") as stream: yaml.safe_dump(metadata, stream, sort_keys=False)
        with open(os.path.join(self.run["temporary"], "metrics.yaml"), "w", encoding="utf-8") as stream: yaml.safe_dump({key: (None if value is None else float(value)) for key, value in metrics.items()}, stream, sort_keys=False)
        os.rename(self.run["temporary"], self.run["final"])
        index = os.path.join(self.history_dir, "index.csv")
        new = not os.path.exists(index)
        with open(index, "a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=INDEX_COLUMNS)
            if new: writer.writeheader()
            writer.writerow({"run_id": self.run["id"], "run_label": self._run_label(), "route_id": metadata["route_id"], "controller": "purepursuit_pi", "map_version": metadata["map_version"], "frame_id": self.frame,
                             "control_rate_hz": metadata["control_rate_hz"], "ego_state_mode": self.source, "reference_checksum": self.run["reference_checksum"], "created_at": metadata["created_at"], "status": status,
                             "mean_ade_m": finite(metrics["mean_ade"]), "fde_m": finite(metrics["fde"]), "lateral_rms_m": finite(metrics["lateral_rms"]), "heading_rms_rad": finite(metrics["heading_rms"]), "speed_rms_mps": finite(metrics["speed_rms"]), "artifact_relpath": os.path.relpath(self.run["final"], self.history_root)})
        rospy.loginfo("trajectory_validator finalized %s (%s)", self.run["id"], status)
        self.run = self.sample_file = self.sample_writer = self.event_file = self.event_writer = None
        self.state = "ARMED"; self.last_sample = None; self.last_progress = None
        self._publish_markers()

    def history_visibility(self, request):
        if not request.run_id or "/" in request.run_id or ".." in request.run_id:
            return SetHistoryVisibilityResponse(False, "invalid run_id")
        run_dir = os.path.join(self.history_dir, request.run_id)
        marker_id = int(hashlib.sha256(request.run_id.encode("utf-8")).hexdigest()[:7], 16)
        if request.visible:
            samples = os.path.join(run_dir, "samples.csv")
            if not os.path.isfile(samples): return SetHistoryVisibilityResponse(False, "samples.csv not found")
            try:
                if request.run_id not in self.selected_history:
                    points = []
                    with open(samples, newline="", encoding="utf-8") as stream:
                        for row in csv.DictReader(stream):
                            if row.get("ego_x_m") and row.get("ego_y_m"):
                                points.append((float(row["ego_x_m"]), float(row["ego_y_m"]), float(row.get("ego_z_m") or 0.0)))
                    self.selected_history[request.run_id] = points
                self._publish_markers()
                return SetHistoryVisibilityResponse(True, "visible")
            except (OSError, ValueError) as error:
                return SetHistoryVisibilityResponse(False, str(error))
        self.selected_history.pop(request.run_id, None)
        delete = Marker(); delete.header.frame_id = self.frame; delete.ns = "history/" + request.run_id; delete.id = marker_id; delete.action = Marker.DELETE
        self.marker_pub.publish(MarkerArray(markers=[delete]))
        return SetHistoryVisibilityResponse(True, "hidden")

    def _line_marker(self, namespace, marker_id, points, color, width, stamp):
        marker = Marker(); marker.header.frame_id, marker.header.stamp = self.frame, stamp
        marker.ns, marker.id, marker.type, marker.action = namespace, marker_id, Marker.LINE_STRIP, Marker.ADD
        marker.scale.x = width; marker.color.r, marker.color.g, marker.color.b, marker.color.a = color[0], color[1], color[2], color[3]
        marker.points = [Point(x=x, y=y, z=z + 0.12) for x, y, z in points]
        return marker

    def _publish_markers(self, ego=None, projection=None):
        markers, stamp = [], rospy.Time.now()
        if self.reference:
            markers.append(self._line_marker("validation_target", 0, [(p["x"], p["y"], p["z"]) for p in self.reference.points], (0.0, 0.85, 1.0, 1.0), 0.28, stamp))
            for marker_id, point, color in ((1, self.reference.points[0], (0.1, 1.0, 0.1, 0.95)), (2, self.reference.points[-1], (1.0, 0.1, 0.1, 0.95))):
                sphere = Marker(); sphere.header.frame_id, sphere.header.stamp = self.frame, stamp; sphere.ns, sphere.id, sphere.type, sphere.action = "validation_gate", marker_id, Marker.SPHERE, Marker.ADD
                sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z, sphere.pose.orientation.w = point["x"], point["y"], point["z"] + 0.25, 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.8; sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color; markers.append(sphere)
        if self.current_path: markers.append(self._line_marker("validation_current", 0, self.current_path, (1.0, 0.85, 0.0, 1.0), 0.28, stamp))
        palette = [(0.95, 0.2, 0.8, 0.55), (0.2, 1.0, 0.45, 0.55), (1.0, 0.5, 0.15, 0.55), (0.55, 0.3, 1.0, 0.55)]
        for index, (run_id, points) in enumerate(sorted(self.selected_history.items())):
            marker_id = int(hashlib.sha256(run_id.encode("utf-8")).hexdigest()[:7], 16)
            markers.append(self._line_marker("history/" + run_id, marker_id, points, palette[index % len(palette)], 0.20, stamp))
        if ego and projection:
            error = Marker(); error.header.frame_id, error.header.stamp = self.frame, stamp; error.ns, error.id, error.type, error.action = "validation_error", 0, Marker.LINE_STRIP, Marker.ADD
            error.scale.x = 0.06; error.color.r, error.color.g, error.color.b, error.color.a = 1.0, 0.0, 0.0, 1.0
            error.points = [Point(x=ego["x"], y=ego["y"], z=ego["z"] + 0.2), Point(x=projection["x"], y=projection["y"], z=projection["z"] + 0.2)]; markers.append(error)
        self.marker_pub.publish(MarkerArray(markers=markers))


if __name__ == "__main__":
    rospy.init_node("trajectory_validator")
    try:
        Validator()
        rospy.spin()
    except Exception as error:
        rospy.logfatal("trajectory_validator startup failed: %s", error)
        raise
