import copy
import hashlib
import json
import math
import os
import re
import shutil
import tempfile
import threading
from collections import deque
from datetime import datetime
from pathlib import Path

import rospy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Odometry, Path as RosPath
from trajectory_follower.msg import PurePursuitDebug
from visualization_msgs.msg import MarkerArray

from trajectory_validator.csv_io import TrackingCsvWriter
from trajectory_validator.metrics import (
    TrajectoryMatcher,
    compact_evaluation_summary,
    metric_summary,
    yaw_from_quaternion,
)
from trajectory_validator.msg import TrajectoryDebug
from trajectory_validator.summary_svg import render_run_summary_svg
from trajectory_validator.visualization import (
    make_ego_pose,
    make_markers,
    make_path,
)


SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")
TRAJECTORY_SAMPLE_NAME = re.compile(
    r"^[0-9]+_[0-9]+_(?:[0-9]+|[0-9]+p[0-9]+)kph$"
)


def normalized_frame(frame):
    return frame.lstrip("/")


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def resolve_repo_directory(root, value):
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("history_dir must stay inside repository") from error
    path.mkdir(parents=True, exist_ok=True)
    return path


def resolve_repo_file(root, value, label):
    if not isinstance(value, str) or not value:
        raise ValueError("{} must be a non-empty repository-relative path".format(label))
    path = (root / value).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("{} must stay inside repository".format(label)) from error
    if not path.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, path))
    return path


def map_version_from_path(path):
    match = re.search(r"(?:^|_)v([0-9]+)\.osm$", Path(path).name, re.IGNORECASE)
    return "v{}".format(match.group(1)) if match else None


def discover_trajectory_file(root):
    parameter = "/trajectory_publisher/trajectory_csv"
    if not rospy.has_param(parameter):
        raise RuntimeError(
            "{} is unavailable; start trajectory_generator before trajectory_validator".format(
                parameter
            )
        )
    value = rospy.get_param(parameter)
    if not isinstance(value, str) or not value:
        raise ValueError("{} must be a non-empty string".format(parameter))
    package_root = root / "base_ws/src/control/trajectory_generator"
    path = (package_root / value).resolve()
    try:
        path.relative_to(package_root)
    except ValueError as error:
        raise ValueError("{} must stay inside trajectory_generator".format(parameter)) from error
    if not path.is_file():
        raise FileNotFoundError("trajectory CSV does not exist: {}".format(path))
    return value, path


def parameter_snapshot(namespace):
    value = rospy.get_param(namespace, {})
    return value if isinstance(value, dict) else {}


def detect_map_version(parameter_snapshots):
    for name in ("/map_version", "/lanelet2_map_loader/map_version"):
        if rospy.has_param(name):
            value = str(rospy.get_param(name))
            if SAFE_NAME.fullmatch(value):
                return value
    version_pattern = re.compile(r"(?:^|[/_])(v[0-9]+)(?:[/_.]|$)", re.IGNORECASE)
    for snapshot in parameter_snapshots:
        for value in flatten_values(snapshot):
            match = version_pattern.search(str(value))
            if match:
                return match.group(1)
    return None


def flatten_values(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from flatten_values(child)
    elif isinstance(value, (list, tuple)):
        for child in value:
            yield from flatten_values(child)
    else:
        yield value


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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


def ensure_reference_trajectory(source_path, sample_directory):
    source_hash = sha256_file(source_path)
    reference_path = sample_directory / "reference_{}.csv".format(source_hash[:12])
    if not reference_path.is_file():
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=reference_path.name + ".", suffix=".tmp", dir=str(sample_directory)
        )
        try:
            with os.fdopen(descriptor, "wb") as destination, source_path.open(
                "rb"
            ) as source:
                shutil.copyfileobj(source, destination)
            os.chmod(temporary_name, 0o644)
            os.replace(temporary_name, reference_path)
        except Exception:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise
    if sha256_file(reference_path) != source_hash:
        raise ValueError("history reference trajectory hash mismatch")
    return reference_path, source_hash


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
    angular = message.twist.twist.angular
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
        angular.z,
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
        self.node_started_ros = rospy.Time.now()
        self.recording_started_ros = None
        self.last_sample_ros = None
        self.finalized = False
        self.end_reason = "shutdown"

        self.trajectory_csv_value, self.input_trajectory_path = discover_trajectory_file(
            self.root
        )
        self.generator_params = parameter_snapshot("/trajectory_publisher")
        self.controller_type = rospy.get_param(
            "~controller/type", "autoware_pure_pursuit"
        )
        self.purepursuit_parameter_namespace = rospy.get_param(
            "~controller/purepursuit_parameter_namespace", "/purepursuit_autoware"
        )
        self.longitudinal_parameter_namespace = rospy.get_param(
            "~controller/longitudinal_parameter_namespace", "/longi_pi_autoware"
        )
        self.purepursuit_params = parameter_snapshot(
            self.purepursuit_parameter_namespace
        )
        self.longi_pi_params = parameter_snapshot(self.longitudinal_parameter_namespace)
        detected_map_version = detect_map_version(
            (self.generator_params, self.purepursuit_params, self.longi_pi_params)
        )
        self.traj_sample = self.input_trajectory_path.stem
        if not TRAJECTORY_SAMPLE_NAME.fullmatch(self.traj_sample):
            raise ValueError(
                "trajectory sample basename must be "
                "<start_id>_<end_id>_<velocity>kph "
                "(use 'p' for a decimal point)"
            )

        self.trajectory_topic = rospy.get_param("~topics/trajectory", "/control/trajectory")
        self.kinematic_state_topic = rospy.get_param(
            "~topics/kinematic_state", "/localization/kinematic_state"
        )
        self.ego_topic = rospy.get_param("~topics/ego", "/Ego_topic")
        self.ctrl_cmd_topic = rospy.get_param("~topics/ctrl_cmd", "/ctrl_cmd")
        self.collision_topic = rospy.get_param("~topics/collision", "/CollisionData")
        self.pp_debug_topic = rospy.get_param(
            "~topics/purepursuit_debug",
            "/purepursuit_autoware/debug/controller_state",
        )
        self.schema_version = int(rospy.get_param("~storage/schema_version", 5))
        self.kinematic_velocity_scale_to_mps = float(
            rospy.get_param("~input/kinematic_velocity_scale_to_mps", 1.0)
        )
        self.overwrite = bool(rospy.get_param("~storage/overwrite", False))
        self.run_label = str(rospy.get_param("~storage/run_label", "")).strip()
        if self.run_label and not SAFE_NAME.fullmatch(self.run_label):
            raise ValueError("run_label may contain only letters, digits, '_' and '-'")
        self.history_root = resolve_repo_directory(
            self.root,
            rospy.get_param(
                "~storage/history_dir",
                "base_ws/src/control/trajectory_validator/history",
            ),
        )
        self.summary_enabled = bool(rospy.get_param("~summary_svg/enabled", True))
        self.summary_filename = rospy.get_param(
            "~summary_svg/filename", "summary.svg"
        )
        if (
            not isinstance(self.summary_filename, str)
            or Path(self.summary_filename).name != self.summary_filename
            or not self.summary_filename.lower().endswith(".svg")
        ):
            raise ValueError("summary_svg/filename must be an SVG basename")
        self.summary_margin_m = float(
            rospy.get_param("~summary_svg/map_margin_m", 20.0)
        )
        if not math.isfinite(self.summary_margin_m) or self.summary_margin_m < 0.0:
            raise ValueError("summary_svg/map_margin_m must be finite and non-negative")
        self.summary_map_path = resolve_repo_file(
            self.root,
            rospy.get_param(
                "~summary_svg/map_file",
                "kcity_map/lanelet_map/lanelet2_map_v44.osm",
            ),
            "summary_svg/map_file",
        )
        self.summary_projector_path = resolve_repo_file(
            self.root,
            rospy.get_param(
                "~summary_svg/projector_file",
                "kcity_map/lanelet_map/map_projector_info_v1.yaml",
            ),
            "summary_svg/projector_file",
        )
        self.map_version = map_version_from_path(self.summary_map_path) or detected_map_version
        self.summary_artifact = {
            "status": "pending" if self.summary_enabled else "disabled",
            "file": self.summary_filename,
        }
        self.sample_rate_hz = float(rospy.get_param("~recording/sample_rate_hz", 50.0))
        self.input_timeout_s = float(rospy.get_param("~recording/input_timeout_s", 0.5))
        self.sync_tolerance_s = float(rospy.get_param("~recording/sync_tolerance_s", 0.05))
        self.command_pair_tolerance_s = float(
            rospy.get_param("~recording/command_pair_tolerance_s", 0.10)
        )
        self.stop_on_collision = bool(
            rospy.get_param("~recording/stop_on_collision", False)
        )
        self.goal_tolerance_m = float(rospy.get_param("~recording/goal_tolerance_m", 3.0))
        self.max_lateral_deviation_m = float(
            rospy.get_param("~recording/max_lateral_deviation_m", 15.0)
        )
        self.off_route_duration_s = float(
            rospy.get_param("~recording/off_route_duration_s", 3.0)
        )
        self.goal_progress_margin_m = float(
            rospy.get_param("~recording/goal_progress_margin_m", 5.0)
        )
        self.controller_debug_timeout_s = float(
            rospy.get_param("~recording/controller_debug_timeout_s", 0.20)
        )
        for label, value in (
            ("sample_rate_hz", self.sample_rate_hz),
            ("input_timeout_s", self.input_timeout_s),
            ("sync_tolerance_s", self.sync_tolerance_s),
            ("command_pair_tolerance_s", self.command_pair_tolerance_s),
            ("goal_tolerance_m", self.goal_tolerance_m),
            ("goal_progress_margin_m", self.goal_progress_margin_m),
            ("controller_debug_timeout_s", self.controller_debug_timeout_s),
            (
                "kinematic_velocity_scale_to_mps",
                self.kinematic_velocity_scale_to_mps,
            ),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(label))
        if self.schema_version != 5:
            raise ValueError("new recordings require schema_version 5")

        # Keep every run grouped below its trajectory sample directory.  The
        # controller suffix makes runs distinguishable while browsing history.
        base_run_id = "{}_pp_autoware".format(self.started_wall.strftime("%m%d_%H%M%S"))
        self.sample_directory = self.history_root / self.traj_sample
        self.sample_directory.mkdir(parents=True, exist_ok=True)
        self.reference_trajectory_path, self.trajectory_sha256 = (
            ensure_reference_trajectory(
                self.input_trajectory_path, self.sample_directory
            )
        )
        self.run_id = self._unique_run_id(base_run_id)
        self.run_directory = self.sample_directory / self.run_id
        self.run_directory.mkdir(exist_ok=self.overwrite)
        self.tracking_csv_path = self.run_directory / "tracking_samples.csv"
        self.json_path = self.run_directory / "metadata.json"
        self.writer = TrackingCsvWriter(self.tracking_csv_path)

        self.gt_trajectory = None
        self.matcher = None
        self.final_station_m = None
        self.ego_trajectory = Trajectory()
        self.kinematic_message = None
        self.ego_message = None
        self.ctrl_message = None
        self.trajectory_received = None
        self.kinematic_received = None
        self.ego_received = None
        self.ctrl_received = None
        self.control_pair = None
        self.last_recorded_command_ros_ns = None
        self.collision_detected = False
        self.off_route_started_ros = None
        self.samples = []
        self.pp_debug_messages = deque(maxlen=100)

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
        self.pp_debug_subscriber = rospy.Subscriber(
            self.pp_debug_topic,
            PurePursuitDebug,
            self.pp_debug_callback,
            queue_size=20,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.sample_rate_hz), self.sample_timer
        )
        rospy.on_shutdown(self.finalize)
        atomic_json(self.json_path, self._metadata("recording"))
        rospy.loginfo(
            "trajectory_validator armed %s -> %s at %.1f Hz; waiting for synchronized "
            "trajectory follower data",
            self.traj_sample,
            self.run_id,
            self.sample_rate_hz,
        )

    def _capture_follower_parameters(self):
        purepursuit = parameter_snapshot(self.purepursuit_parameter_namespace)
        longi_pi = parameter_snapshot(self.longitudinal_parameter_namespace)
        if not purepursuit or not longi_pi:
            rospy.loginfo_throttle(
                2.0,
                "trajectory_validator armed; waiting for %s and %s runtime parameters",
                self.purepursuit_parameter_namespace,
                self.longitudinal_parameter_namespace,
            )
            return False
        self.generator_params = parameter_snapshot("/trajectory_publisher")
        self.purepursuit_params = purepursuit
        self.longi_pi_params = longi_pi
        self.map_version = detect_map_version(
            (self.generator_params, self.purepursuit_params, self.longi_pi_params)
        )
        self.map_version = map_version_from_path(self.summary_map_path) or self.map_version
        return True

    def _unique_run_id(self, base):
        candidates = [base] + ["{}_{:02d}".format(base, index) for index in range(1, 100)]
        for candidate in candidates:
            run_directory = self.sample_directory / candidate
            legacy_csv = self.sample_directory / (candidate + ".csv")
            legacy_json = self.sample_directory / (candidate + ".json")
            if self.overwrite or (
                not run_directory.exists() and not legacy_csv.exists() and not legacy_json.exists()
            ):
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
            self.final_station_m = self.matcher.stations[-1]
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
            linear = self.kinematic_message.twist.twist.linear
            linear.x *= self.kinematic_velocity_scale_to_mps
            linear.y *= self.kinematic_velocity_scale_to_mps
            linear.z *= self.kinematic_velocity_scale_to_mps
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
            if (
                self.kinematic_message is not None
                and self.ego_message is not None
                and self.kinematic_received is not None
                and self.ego_received is not None
            ):
                self.control_pair = {
                    "kinematic": copy.deepcopy(self.kinematic_message),
                    "ego": copy.deepcopy(self.ego_message),
                    "command": copy.deepcopy(message),
                    "state_received": max(self.kinematic_received, self.ego_received),
                    "command_received": self.ctrl_received,
                }

    def collision_callback(self, message):
        objects = tuple(message.collision_object) + tuple(message.collision_objecta)
        with self.lock:
            # MORAI can publish one obj_type=-1 placeholder while reporting
            # "no collision".  Only non-negative object types are real events.
            if any(obj.type >= 0 for obj in objects):
                self.collision_detected = True

    def pp_debug_callback(self, message):
        values = (
            message.lookahead_distance_m,
            message.path_curvature_1pm,
            message.target_curvature_1pm,
            message.lateral_error_m,
            message.steering_command_rad,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(
                1.0, "trajectory_validator rejected non-finite Pure Pursuit debug"
            )
            return
        with self.lock:
            self.pp_debug_messages.append((rospy.Time.now(), copy.deepcopy(message)))

    def _purepursuit_debug_snapshot(self, command_received):
        if not self.pp_debug_messages:
            return {
                "pp_controller_valid": False,
                "pp_debug_fresh": False,
                "pp_lookahead_distance_m": "",
                "pp_path_curvature_1pm": "",
                "pp_target_curvature_1pm": "",
                "pp_lateral_error_m": "",
            }
        received, message = min(
            self.pp_debug_messages,
            key=lambda item: abs((item[0] - command_received).to_sec()),
        )
        age = abs((received - command_received).to_sec())
        return {
            "pp_controller_valid": bool(message.controller_valid),
            "pp_debug_fresh": age <= self.controller_debug_timeout_s,
            "pp_lookahead_distance_m": message.lookahead_distance_m,
            "pp_path_curvature_1pm": message.path_curvature_1pm,
            "pp_target_curvature_1pm": message.target_curvature_1pm,
            "pp_lateral_error_m": message.lateral_error_m,
        }

    def _inputs_valid(self, now):
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
        # This is the same signed base-link longitudinal velocity used by the
        # Pure Pursuit LD scheduler, so the recorded v can reproduce its input.
        point.longitudinal_velocity_mps = linear.x
        point.acceleration_mps2 = math.sqrt(
            ego.acceleration.x ** 2
            + ego.acceleration.y ** 2
            + ego.acceleration.z ** 2
        )
        self.ego_trajectory.points.append(point)
        return yaw, point

    def _reached_goal(self, ego_point, match):
        """Require final-point proximity and progress through the trajectory."""
        final = self.gt_trajectory.points[-1].pose.position
        final_distance_m = math.hypot(
            ego_point.pose.position.x - final.x,
            ego_point.pose.position.y - final.y,
        )
        final_progress_start_m = max(
            0.0, self.final_station_m - self.goal_progress_margin_m
        )
        return (
            final_distance_m <= self.goal_tolerance_m
            and match["matched_gt_station_m"] >= final_progress_start_m
        )

    def sample_timer(self, _event):
        should_stop = False
        with self.lock:
            if self.finalized:
                return
            if (
                self.gt_trajectory is None
                or self.control_pair is None
            ):
                return
            now = rospy.Time.now()
            inputs_valid = self._inputs_valid(now)
            if self.recording_started_ros is None:
                if not inputs_valid:
                    rospy.loginfo_throttle(
                        2.0,
                        "trajectory_validator armed; waiting for fresh synchronized "
                        "trajectory, kinematic state, Ego status, and ctrl_cmd",
                    )
                    return
                if not self._capture_follower_parameters():
                    return
                self.recording_started_ros = now
                self.started_wall = datetime.now().astimezone()
                rospy.loginfo("trajectory_validator recording started")
            elapsed = max(0.0, (now - self.recording_started_ros).to_sec())
            dt = (
                0.0
                if self.last_sample_ros is None
                else max(0.0, (now - self.last_sample_ros).to_sec())
            )
            self.last_sample_ros = now
            pair = self.control_pair
            command_ros_ns = pair["command_received"].to_nsec()
            if command_ros_ns == self.last_recorded_command_ros_ns:
                return
            self.last_recorded_command_ros_ns = command_ros_ns
            ego = pair["ego"]
            kinematic = pair["kinematic"]
            command = pair["command"]
            ego_yaw, ego_point = self._append_ego_point(ego, kinematic, elapsed)
            ego_velocity = float(ego_point.longitudinal_velocity_mps)
            match = self.matcher.match(
                ego_point.pose.position.x,
                ego_point.pose.position.y,
                ego_yaw,
                ego_velocity,
            )
            reached_goal = self._reached_goal(ego_point, match)
            sample_index = len(self.samples)
            state_received = pair["state_received"]
            command_received = pair["command_received"]
            state_to_command_delay = (command_received - state_received).to_sec()
            control_pair_valid = (
                inputs_valid
                and 0.0 <= state_to_command_delay <= self.command_pair_tolerance_s
            )
            purepursuit_debug = self._purepursuit_debug_snapshot(command_received)
            sample = {
                "sample_index": sample_index,
                "control_index": sample_index,
                "ros_time_ns": now.to_nsec(),
                "elapsed_time_s": elapsed,
                "dt_s": dt,
                "state_received_ros_ns": state_received.to_nsec(),
                "command_received_ros_ns": command_received.to_nsec(),
                "state_to_command_delay_s": state_to_command_delay,
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
                "ego_yaw_rate_rps": kinematic.twist.twist.angular.z,
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
                "steering_error_rad": math.radians(ego.wheel_angle) - command.steering,
                "collision_detected": self.collision_detected,
                "reached_goal": reached_goal,
                "control_pair_valid": control_pair_valid,
                **purepursuit_debug,
            }
            self.samples.append(sample)
            self.writer.write(sample)
            debug = self._debug_message(sample, now)
            self.debug_publisher.publish(debug)
            self.ego_path_publisher.publish(make_path(self.ego_trajectory, now))
            self.ego_pose_publisher.publish(make_ego_pose(debug, now))
            self.marker_publisher.publish(make_markers(debug, now))
            if self.collision_detected and self.stop_on_collision:
                self.end_reason = "collision"
                should_stop = True
            elif reached_goal:
                self.end_reason = "goal_reached"
                should_stop = True
            elif off_route:
                self.end_reason = "off_route"
                rospy.logerr(
                    "trajectory recording stopped: lateral deviation %.2f m "
                    "exceeded %.2f m for %.2f s",
                    lateral_deviation,
                    self.max_lateral_deviation_m,
                    self.off_route_duration_s,
                )
                should_stop = True
        if should_stop:
            rospy.signal_shutdown(self.end_reason)

    def _debug_message(self, sample, stamp):
        debug = TrajectoryDebug()
        debug.header.seq = sample["control_index"]
        debug.header.stamp = stamp
        debug.header.frame_id = self.gt_trajectory.header.frame_id
        debug.traj_sample = self.traj_sample
        debug.run_id = self.run_id
        debug.sample_index = sample["control_index"]
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
            "matched_gt_x_m",
            "matched_gt_y_m",
            "matched_gt_yaw_rad",
            "target_velocity_mps",
            "lateral_error_m",
            "longitudinal_error_m",
            "heading_error_rad",
            "velocity_error_mps",
            "steering_error_rad",
            "collision_detected",
            "reached_goal",
        ):
            setattr(debug, field, sample[field])
        debug.control_pair_valid = sample["control_pair_valid"]
        return debug

    def _metadata(self, status):
        input_relative = str(self.input_trajectory_path.relative_to(self.root))
        duration = self.samples[-1]["elapsed_time_s"] if self.samples else 0.0
        document = {
            "schema_version": self.schema_version,
            "traj_sample": self.traj_sample,
            "run_id": self.run_id,
            "run_label": self.run_label or None,
            "status": status,
            "started_at": self.started_wall.isoformat(),
            "armed_at_ros_ns": self.node_started_ros.to_nsec(),
            "recording_started_ros_ns": self.recording_started_ros.to_nsec()
            if self.recording_started_ros is not None
            else None,
            "finished_at": datetime.now().astimezone().isoformat()
            if status != "recording"
            else None,
            "frame_id": self.gt_trajectory.header.frame_id
            if self.gt_trajectory is not None
            else None,
            "input_trajectory": {
                "file": input_relative,
                "publisher_value": self.trajectory_csv_value,
                "sha256": self.trajectory_sha256,
                "point_count": len(self.gt_trajectory.points)
                if self.gt_trajectory is not None
                else None,
            },
            "reference_trajectory": {
                "file": str(self.reference_trajectory_path.relative_to(self.root)),
                "sha256": self.trajectory_sha256,
            },
            "map": {
                "version": self.map_version,
                "file": str(self.summary_map_path.relative_to(self.root)),
                "sha256": sha256_file(self.summary_map_path),
                "projector_file": str(self.summary_projector_path.relative_to(self.root)),
                "projector_sha256": sha256_file(self.summary_projector_path),
            },
            "controller": {
                "type": self.controller_type,
                "purepursuit_parameter_namespace": self.purepursuit_parameter_namespace,
                "longitudinal_parameter_namespace": self.longitudinal_parameter_namespace,
                "purepursuit_parameters": self.purepursuit_params,
                "longi_pi_parameters": self.longi_pi_params,
            },
            "purepursuit_debug": {
                "topic": self.pp_debug_topic,
                "timeout_s": self.controller_debug_timeout_s,
            },
            "trajectory_publisher_parameters": self.generator_params,
            "recording": {
                "kinematic_velocity_scale_to_mps": self.kinematic_velocity_scale_to_mps,
                "sample_rate_hz": self.sample_rate_hz,
                "sample_count": len(self.samples),
                "command_pair_tolerance_s": self.command_pair_tolerance_s,
                "response_delay_s": self.response_delay_s,
                "response_tolerance_s": self.response_tolerance_s,
                "duration_s": duration,
                "end_reason": self.end_reason if status != "recording" else None,
            },
            "result": metric_summary(self.samples),
            "artifacts": {"summary_svg": self.summary_artifact},
            "integrity": {
                "tracking_samples_csv_sha256": sha256_file(self.tracking_csv_path)
                if status != "recording" and self.tracking_csv_path.is_file()
                else None,
                "tracking_sample_row_count": self.writer.count,
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
            if self.summary_enabled:
                try:
                    self.summary_artifact = render_run_summary_svg(
                        self.run_directory,
                        self.summary_map_path,
                        self.summary_projector_path,
                        filename=self.summary_filename,
                        margin_m=self.summary_margin_m,
                        gt_points=[
                            (point.pose.position.x, point.pose.position.y)
                            for point in self.gt_trajectory.points
                        ],
                    )
                except Exception as error:
                    self.summary_artifact = {
                        "status": "failed",
                        "file": self.summary_filename,
                        "error": str(error),
                    }
                    rospy.logerr("trajectory_validator summary image failed: %s", error)
            atomic_json(self.json_path, self._metadata("complete"))
            summary = compact_evaluation_summary(
                self.samples, self.gt_trajectory
            )
            atomic_json(self.summary_path, summary)
            rospy.loginfo(
                "trajectory_validator saved %d compact tracking samples + summary=%s: %s",
                self.writer.count,
                self.summary_artifact["status"],
                self.run_directory,
            )


def main():
    rospy.init_node("trajectory_validator")
    try:
        TrajectoryRecorder()
    except Exception as error:
        rospy.logfatal("trajectory_validator: %s", error)
        raise SystemExit(1)
    rospy.spin()
