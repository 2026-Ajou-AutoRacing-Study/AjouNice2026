import copy
import hashlib
import json
import math
import os
import re
import tempfile
import threading
from datetime import datetime
from pathlib import Path

import rospy
import yaml
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Odometry, Path as RosPath
from visualization_msgs.msg import MarkerArray

from trajectory_validator.csv_io import RunCsvWriter
from trajectory_validator.metrics import (
    TrajectoryMatcher,
    metric_summary,
    yaw_from_quaternion,
)
from trajectory_validator.msg import TrajectoryDebug
from trajectory_validator.visualization import (
    make_ego_pose,
    make_markers,
    make_path,
)


SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def normalized_frame(frame):
    return frame.lstrip("/")


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def resolve_repo_file(root, value, label, required=True):
    if not value:
        if required:
            raise ValueError("{} is required".format(label))
        return None
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("{} must stay inside repository".format(label)) from error
    if required and not path.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, path))
    return path


def resolve_repo_directory(root, value):
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("history_dir must stay inside repository") from error
    path.mkdir(parents=True, exist_ok=True)
    return path


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def yaml_file(path):
    if path is None or not path.is_file():
        return None
    with path.open() as stream:
        value = yaml.safe_load(stream)
    return value if value is not None else {}


def atomic_json(path, document):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(document, stream, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def finite_trajectory(message):
    if len(message.points) < 2:
        return False
    for point in message.points:
        values = (
            point.pose.position.x,
            point.pose.position.y,
            point.pose.position.z,
            point.pose.orientation.x,
            point.pose.orientation.y,
            point.pose.orientation.z,
            point.pose.orientation.w,
            point.longitudinal_velocity_mps,
        )
        if not all(math.isfinite(value) for value in values):
            return False
    return True


def finite_kinematic_state(message):
    pose = message.pose.pose
    linear = message.twist.twist.linear
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        linear.x,
        linear.y,
        linear.z,
    )
    quaternion_norm = (
        pose.orientation.x ** 2
        + pose.orientation.y ** 2
        + pose.orientation.z ** 2
        + pose.orientation.w ** 2
    )
    return (
        bool(normalized_frame(message.header.frame_id))
        and all(math.isfinite(value) for value in values)
        and math.isfinite(quaternion_norm)
        and quaternion_norm > 1e-12
    )


class TrajectoryRecorder:
    def __init__(self):
        self.lock = threading.RLock()
        self.root = repository_root()
        self.started_wall = datetime.now().astimezone()
        self.started_ros = rospy.Time.now()
        self.last_sample_ros = None
        self.finalized = False
        self.end_reason = "shutdown"

        self.input_trajectory_value = rospy.get_param("~input_trajectory_csv")
        self.run_params = rospy.get_param("~run_params")
        self.map_version = rospy.get_param("~map_version")
        self.follower_config_value = rospy.get_param("~follower_config_file", "")
        self.input_trajectory_path = resolve_repo_file(
            self.root, self.input_trajectory_value, "input_trajectory_csv"
        )
        self.follower_config_path = resolve_repo_file(
            self.root,
            self.follower_config_value,
            "follower_config_file",
            required=False,
        )
        self.traj_sample = self.input_trajectory_path.stem
        if not SAFE_NAME.fullmatch(self.traj_sample):
            raise ValueError("trajectory sample basename contains unsupported characters")
        if not self.run_params or not SAFE_NAME.fullmatch(self.run_params):
            raise ValueError("run_params must match [A-Za-z0-9_-]+")
        if not self.map_version or not SAFE_NAME.fullmatch(self.map_version):
            raise ValueError("map_version must match [A-Za-z0-9_-]+")

        self.trajectory_topic = rospy.get_param("~topics/trajectory", "/control/trajectory")
        self.kinematic_state_topic = rospy.get_param(
            "~topics/kinematic_state", "/localization/kinematic_state"
        )
        self.ego_topic = rospy.get_param("~topics/ego", "/Ego_topic")
        self.ctrl_cmd_topic = rospy.get_param("~topics/ctrl_cmd", "/ctrl_cmd")
        self.collision_topic = rospy.get_param("~topics/collision", "/CollisionData")
        self.schema_version = int(rospy.get_param("~storage/schema_version", 1))
        self.overwrite = bool(rospy.get_param("~storage/overwrite", False))
        self.history_root = resolve_repo_directory(
            self.root,
            rospy.get_param(
                "~storage/history_dir",
                "base_ws/src/control/trajectory_validator/history",
            ),
        )
        self.sample_rate_hz = float(rospy.get_param("~recording/sample_rate_hz", 30.0))
        self.input_timeout_s = float(rospy.get_param("~recording/input_timeout_s", 0.5))
        self.sync_tolerance_s = float(rospy.get_param("~recording/sync_tolerance_s", 0.05))
        self.goal_tolerance_m = float(rospy.get_param("~recording/goal_tolerance_m", 3.0))
        for label, value in (
            ("sample_rate_hz", self.sample_rate_hz),
            ("input_timeout_s", self.input_timeout_s),
            ("sync_tolerance_s", self.sync_tolerance_s),
            ("goal_tolerance_m", self.goal_tolerance_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(label))
        if self.schema_version != 1:
            raise ValueError("only schema_version 1 is supported")

        base_run_id = "{}_{}".format(
            self.started_wall.strftime("%m%d_%H%M%S"), self.run_params
        )
        self.run_directory = self.history_root / self.traj_sample
        self.run_directory.mkdir(parents=True, exist_ok=True)
        self.run_id = self._unique_run_id(base_run_id)
        self.csv_path = self.run_directory / (self.run_id + ".csv")
        self.json_path = self.run_directory / (self.run_id + ".json")
        self.writer = RunCsvWriter(
            self.csv_path, self.schema_version, self.traj_sample, self.run_id
        )

        self.gt_trajectory = None
        self.matcher = None
        self.ego_trajectory = Trajectory()
        self.kinematic_message = None
        self.ego_message = None
        self.ctrl_message = None
        self.trajectory_received = None
        self.kinematic_received = None
        self.ego_received = None
        self.ctrl_received = None
        self.collision_detected = False
        self.samples = []

        self.debug_publisher = rospy.Publisher(
            "/trajectory_validator/debug", TrajectoryDebug, queue_size=1
        )
        self.gt_path_publisher = rospy.Publisher(
            "/trajectory_validator/gt_path", RosPath, queue_size=1, latch=True
        )
        self.ego_path_publisher = rospy.Publisher(
            "/trajectory_validator/ego_path", RosPath, queue_size=1
        )
        self.ego_pose_publisher = rospy.Publisher(
            "/trajectory_validator/ego_pose", PoseStamped, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/trajectory_validator/markers", MarkerArray, queue_size=1
        )
        self.trajectory_subscriber = rospy.Subscriber(
            self.trajectory_topic, Trajectory, self.trajectory_callback, queue_size=1
        )
        self.kinematic_subscriber = rospy.Subscriber(
            self.kinematic_state_topic,
            Odometry,
            self.kinematic_callback,
            queue_size=20,
        )
        self.ego_subscriber = rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=20
        )
        self.ctrl_subscriber = rospy.Subscriber(
            self.ctrl_cmd_topic, CtrlCmd, self.ctrl_callback, queue_size=20
        )
        self.collision_subscriber = rospy.Subscriber(
            self.collision_topic, CollisionData, self.collision_callback, queue_size=5
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.sample_rate_hz), self.sample_timer
        )
        rospy.on_shutdown(self.finalize)
        atomic_json(self.json_path, self._metadata("recording"))
        rospy.loginfo(
            "trajectory_validator recording %s -> %s at %.1f Hz",
            self.traj_sample,
            self.run_id,
            self.sample_rate_hz,
        )

    def _unique_run_id(self, base):
        candidates = [base] + ["{}_{:02d}".format(base, index) for index in range(1, 100)]
        for candidate in candidates:
            csv_path = self.run_directory / (candidate + ".csv")
            json_path = self.run_directory / (candidate + ".json")
            if self.overwrite or (not csv_path.exists() and not json_path.exists()):
                return candidate
        raise RuntimeError("could not allocate unique run ID")

    def trajectory_callback(self, message):
        if not finite_trajectory(message):
            rospy.logwarn_throttle(1.0, "trajectory_validator rejected invalid trajectory")
            return
        with self.lock:
            if self.finalized:
                return
            self.trajectory_received = rospy.Time.now()
            if self.gt_trajectory is not None:
                if len(message.points) != len(self.gt_trajectory.points):
                    rospy.logerr_throttle(
                        1.0, "trajectory changed during recording; ignoring replacement"
                    )
                return
            self.gt_trajectory = copy.deepcopy(message)
            if not self.gt_trajectory.header.frame_id:
                self.gt_trajectory.header.frame_id = "map"
            self.ego_trajectory.header.frame_id = self.gt_trajectory.header.frame_id
            self.matcher = TrajectoryMatcher(self.gt_trajectory)
            self.writer.write_gt(self.gt_trajectory)
            stamp = rospy.Time.now()
            self.gt_path_publisher.publish(make_path(self.gt_trajectory, stamp))
            rospy.loginfo(
                "trajectory_validator captured %d GT points in frame %s",
                len(self.gt_trajectory.points),
                self.gt_trajectory.header.frame_id,
            )

    def kinematic_callback(self, message):
        if not finite_kinematic_state(message):
            rospy.logwarn_throttle(
                1.0, "trajectory_validator rejected invalid kinematic state"
            )
            return
        with self.lock:
            if self.gt_trajectory is not None and normalized_frame(
                message.header.frame_id
            ) != normalized_frame(self.gt_trajectory.header.frame_id):
                rospy.logerr_throttle(
                    1.0,
                    "trajectory_validator frame mismatch: trajectory=%s state=%s",
                    self.gt_trajectory.header.frame_id,
                    message.header.frame_id,
                )
                return
            self.kinematic_message = copy.deepcopy(message)
            self.kinematic_received = rospy.Time.now()

    def ego_callback(self, message):
        values = (
            message.position.x,
            message.position.y,
            message.position.z,
            message.heading,
            message.velocity.x,
            message.velocity.y,
            message.velocity.z,
            message.acceleration.x,
            message.acceleration.y,
            message.acceleration.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, "trajectory_validator rejected non-finite ego state")
            return
        with self.lock:
            self.ego_message = copy.deepcopy(message)
            self.ego_received = rospy.Time.now()

    def ctrl_callback(self, message):
        values = (
            message.accel,
            message.brake,
            message.steering,
            message.velocity,
            message.acceleration,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, "trajectory_validator rejected non-finite ctrl command")
            return
        with self.lock:
            self.ctrl_message = copy.deepcopy(message)
            self.ctrl_received = rospy.Time.now()

    def collision_callback(self, message):
        objects = tuple(message.collision_object) + tuple(message.collision_objecta)
        with self.lock:
            # MORAI can publish one obj_type=-1 placeholder while reporting
            # "no collision".  Only non-negative object types are real events.
            if any(obj.type >= 0 for obj in objects):
                self.collision_detected = True

    def _sample_valid(self, now):
        if any(
            value is None
            for value in (
                self.gt_trajectory,
                self.kinematic_message,
                self.ego_message,
                self.ctrl_message,
                self.trajectory_received,
                self.kinematic_received,
                self.ego_received,
                self.ctrl_received,
            )
        ):
            return False
        fresh = all(
            (now - stamp).to_sec() <= self.input_timeout_s
            for stamp in (
                self.trajectory_received,
                self.kinematic_received,
                self.ego_received,
                self.ctrl_received,
            )
        )
        if normalized_frame(self.gt_trajectory.header.frame_id) != normalized_frame(
            self.kinematic_message.header.frame_id
        ):
            return False
        ego_stamp = self.ego_message.header.stamp
        state_stamp = self.kinematic_message.header.stamp
        ego_synchronized = (
            ego_stamp.is_zero()
            or state_stamp.is_zero()
            or abs((ego_stamp - state_stamp).to_sec()) <= self.sync_tolerance_s
        )
        return fresh and ego_synchronized

    def _append_ego_point(self, ego, kinematic, elapsed):
        pose = kinematic.pose.pose
        linear = kinematic.twist.twist.linear
        yaw = yaw_from_quaternion(pose.orientation)
        point = TrajectoryPoint()
        point.time_from_start = rospy.Duration(elapsed)
        point.pose = copy.deepcopy(pose)
        point.longitudinal_velocity_mps = math.sqrt(
            linear.x ** 2 + linear.y ** 2 + linear.z ** 2
        )
        point.acceleration_mps2 = math.sqrt(
            ego.acceleration.x ** 2
            + ego.acceleration.y ** 2
            + ego.acceleration.z ** 2
        )
        self.ego_trajectory.points.append(point)
        return yaw, point

    def sample_timer(self, _event):
        should_stop = False
        with self.lock:
            if self.finalized:
                return
            if (
                self.gt_trajectory is None
                or self.kinematic_message is None
                or self.ego_message is None
            ):
                return
            now = rospy.Time.now()
            elapsed = max(0.0, (now - self.started_ros).to_sec())
            dt = (
                0.0
                if self.last_sample_ros is None
                else max(0.0, (now - self.last_sample_ros).to_sec())
            )
            self.last_sample_ros = now
            ego = self.ego_message
            kinematic = self.kinematic_message
            command = self.ctrl_message if self.ctrl_message is not None else CtrlCmd()
            ego_yaw, ego_point = self._append_ego_point(ego, kinematic, elapsed)
            ego_velocity = float(ego_point.longitudinal_velocity_mps)
            match = self.matcher.match(
                ego_point.pose.position.x,
                ego_point.pose.position.y,
                ego_yaw,
                ego_velocity,
            )
            final = self.gt_trajectory.points[-1].pose.position
            reached_goal = (
                math.hypot(
                    ego_point.pose.position.x - final.x,
                    ego_point.pose.position.y - final.y,
                )
                <= self.goal_tolerance_m
            )
            sample_valid = self._sample_valid(now)
            sample_index = len(self.samples)
            sample = {
                "sample_index": sample_index,
                "ros_time_ns": now.to_nsec(),
                "elapsed_time_s": elapsed,
                "dt_s": dt,
                **match,
                "ego_unique_id": ego.unique_id,
                "ego_x_m": ego_point.pose.position.x,
                "ego_y_m": ego_point.pose.position.y,
                "ego_z_m": ego_point.pose.position.z,
                "ego_yaw_rad": ego_yaw,
                "ego_heading_raw": ego.heading,
                "ego_velocity_x_mps": kinematic.twist.twist.linear.x,
                "ego_velocity_y_mps": kinematic.twist.twist.linear.y,
                "ego_velocity_z_mps": kinematic.twist.twist.linear.z,
                "ego_velocity_mps": ego_velocity,
                "ego_acceleration_x_mps2": ego.acceleration.x,
                "ego_acceleration_y_mps2": ego.acceleration.y,
                "ego_acceleration_z_mps2": ego.acceleration.z,
                "ego_acceleration_mps2": ego_point.acceleration_mps2,
                "ego_accel": ego.accel,
                "ego_brake": ego.brake,
                "ego_wheel_angle_raw": ego.wheel_angle,
                "actual_steering_rad": math.radians(ego.wheel_angle),
                "ego_lateral_offset_m": ego.lateral_offset,
                "cmd_longl_type": command.longlCmdType,
                "cmd_accel": command.accel,
                "cmd_brake": command.brake,
                "cmd_steering_rad": command.steering,
                "cmd_velocity_mps": command.velocity,
                "cmd_acceleration_mps2": command.acceleration,
                "collision_detected": self.collision_detected,
                "reached_goal": reached_goal,
                "sample_valid": sample_valid,
            }
            self.samples.append(sample)
            self.writer.write_sample(sample)
            debug = self._debug_message(sample, now)
            self.debug_publisher.publish(debug)
            self.ego_path_publisher.publish(make_path(self.ego_trajectory, now))
            self.ego_pose_publisher.publish(make_ego_pose(debug, now))
            self.marker_publisher.publish(make_markers(debug, now))
            if self.collision_detected:
                self.end_reason = "collision"
                should_stop = True
            elif reached_goal:
                self.end_reason = "goal_reached"
                should_stop = True
        if should_stop:
            rospy.signal_shutdown(self.end_reason)

    def _debug_message(self, sample, stamp):
        debug = TrajectoryDebug()
        debug.header.seq = sample["sample_index"]
        debug.header.stamp = stamp
        debug.header.frame_id = self.gt_trajectory.header.frame_id
        debug.traj_sample = self.traj_sample
        debug.run_id = self.run_id
        debug.sample_index = sample["sample_index"]
        debug.elapsed_time_s = sample["elapsed_time_s"]
        debug.gt_trajectory = self.gt_trajectory
        debug.gt_trajectory.header.stamp = stamp
        debug.ego_trajectory = self.ego_trajectory
        debug.ego_trajectory.header.stamp = stamp
        for field in (
            "ego_unique_id",
            "ego_x_m",
            "ego_y_m",
            "ego_z_m",
            "ego_yaw_rad",
            "ego_heading_raw",
            "ego_velocity_x_mps",
            "ego_velocity_y_mps",
            "ego_velocity_z_mps",
            "ego_velocity_mps",
            "ego_acceleration_x_mps2",
            "ego_acceleration_y_mps2",
            "ego_acceleration_z_mps2",
            "ego_acceleration_mps2",
            "ego_accel",
            "ego_brake",
            "ego_wheel_angle_raw",
            "actual_steering_rad",
            "ego_lateral_offset_m",
            "cmd_longl_type",
            "cmd_accel",
            "cmd_brake",
            "cmd_steering_rad",
            "cmd_velocity_mps",
            "cmd_acceleration_mps2",
            "matched_gt_index",
            "matched_gt_station_m",
            "target_velocity_mps",
            "lateral_error_m",
            "longitudinal_error_m",
            "heading_error_rad",
            "velocity_error_mps",
            "collision_detected",
            "reached_goal",
            "sample_valid",
        ):
            setattr(debug, field, sample[field])
        return debug

    def _metadata(self, status):
        map_path = self.root / "kcity_map/lanelet_map/lanelet2_map_{}.osm".format(
            self.map_version
        )
        input_relative = str(self.input_trajectory_path.relative_to(self.root))
        follower_relative = (
            str(self.follower_config_path.relative_to(self.root))
            if self.follower_config_path and self.follower_config_path.is_file()
            else None
        )
        duration = self.samples[-1]["elapsed_time_s"] if self.samples else 0.0
        document = {
            "schema_version": self.schema_version,
            "traj_sample": self.traj_sample,
            "run_id": self.run_id,
            "status": status,
            "started_at": self.started_wall.isoformat(),
            "finished_at": datetime.now().astimezone().isoformat()
            if status != "recording"
            else None,
            "frame_id": self.gt_trajectory.header.frame_id
            if self.gt_trajectory is not None
            else None,
            "input_trajectory": {
                "file": input_relative,
                "sha256": sha256_file(self.input_trajectory_path),
                "point_count": len(self.gt_trajectory.points)
                if self.gt_trajectory is not None
                else None,
            },
            "map": {
                "version": self.map_version,
                "file": str(map_path.relative_to(self.root)) if map_path.is_file() else None,
                "sha256": sha256_file(map_path) if map_path.is_file() else None,
            },
            "controller": {
                "run_params": self.run_params.split("_"),
                "config_file": follower_relative,
                "config_sha256": sha256_file(self.follower_config_path)
                if self.follower_config_path and self.follower_config_path.is_file()
                else None,
                "parameters": yaml_file(self.follower_config_path),
            },
            "recording": {
                "sample_rate_hz": self.sample_rate_hz,
                "sample_count": len(self.samples),
                "duration_s": duration,
                "end_reason": self.end_reason if status != "recording" else None,
            },
            "result": metric_summary(self.samples),
            "integrity": {
                "csv_sha256": sha256_file(self.csv_path)
                if status != "recording" and self.csv_path.is_file()
                else None,
                "gt_row_count": self.writer.gt_count,
                "sample_row_count": self.writer.sample_count,
            },
        }
        return document

    def finalize(self):
        with self.lock:
            if self.finalized:
                return
            self.finalized = True

        # Stop future timer firings after publishing the finalized flag.  A timer
        # callback that was waiting for the lock will now return without touching
        # the CSV, while a callback already holding the lock finishes first.
        self.timer.shutdown()

        with self.lock:
            self.writer.close()
            atomic_json(self.json_path, self._metadata("complete"))
            rospy.loginfo(
                "trajectory_validator saved %d GT + %d samples: %s",
                self.writer.gt_count,
                self.writer.sample_count,
                self.csv_path,
            )


def main():
    rospy.init_node("trajectory_validator")
    try:
        TrajectoryRecorder()
    except Exception as error:
        rospy.logfatal("trajectory_validator: %s", error)
        raise SystemExit(1)
    rospy.spin()
