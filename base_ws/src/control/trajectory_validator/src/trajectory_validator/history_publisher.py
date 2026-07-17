import argparse
import hashlib
import json
import re
from pathlib import Path

import rospy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import MarkerArray

from trajectory_validator.csv_io import (
    as_bool,
    as_float,
    as_int,
    load_run_csv,
)
from trajectory_validator.metrics import quaternion_from_yaw
from trajectory_validator.msg import TrajectoryDebug
from trajectory_validator.visualization import (
    make_ego_pose,
    make_markers,
    make_path,
)


SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def gt_trajectory_from_rows(rows, frame_id):
    trajectory = Trajectory()
    trajectory.header.frame_id = frame_id
    for expected_index, row in enumerate(rows):
        if as_int(row, "gt_index") != expected_index:
            raise ValueError("GT indices are not contiguous from zero")
        point = TrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(
            as_float(row, "gt_time_from_start_s")
        )
        point.pose.position.x = as_float(row, "gt_x_m")
        point.pose.position.y = as_float(row, "gt_y_m")
        point.pose.position.z = as_float(row, "gt_z_m")
        point.pose.orientation.x = as_float(row, "gt_qx")
        point.pose.orientation.y = as_float(row, "gt_qy")
        point.pose.orientation.z = as_float(row, "gt_qz")
        point.pose.orientation.w = as_float(row, "gt_qw", 1.0)
        point.longitudinal_velocity_mps = as_float(
            row, "gt_longitudinal_velocity_mps"
        )
        point.lateral_velocity_mps = as_float(row, "gt_lateral_velocity_mps")
        point.acceleration_mps2 = as_float(row, "gt_acceleration_mps2")
        point.heading_rate_rps = as_float(row, "gt_heading_rate_rps")
        point.front_wheel_angle_rad = as_float(row, "gt_front_wheel_angle_rad")
        point.rear_wheel_angle_rad = as_float(row, "gt_rear_wheel_angle_rad")
        trajectory.points.append(point)
    if len(trajectory.points) < 2:
        raise ValueError("history GT has fewer than two points")
    return trajectory


def append_ego_point(trajectory, row):
    point = TrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(as_float(row, "elapsed_time_s"))
    point.pose.position.x = as_float(row, "ego_x_m")
    point.pose.position.y = as_float(row, "ego_y_m")
    point.pose.position.z = as_float(row, "ego_z_m")
    qx, qy, qz, qw = quaternion_from_yaw(as_float(row, "ego_yaw_rad"))
    point.pose.orientation.x = qx
    point.pose.orientation.y = qy
    point.pose.orientation.z = qz
    point.pose.orientation.w = qw
    point.longitudinal_velocity_mps = as_float(row, "ego_velocity_mps")
    point.acceleration_mps2 = as_float(row, "ego_acceleration_mps2")
    trajectory.points.append(point)


def debug_from_row(row, gt_trajectory, ego_trajectory, traj_sample, run_id, stamp):
    debug = TrajectoryDebug()
    debug.header.seq = as_int(row, "sample_index")
    debug.header.stamp = stamp
    debug.header.frame_id = gt_trajectory.header.frame_id
    debug.traj_sample = traj_sample
    debug.run_id = run_id
    debug.sample_index = as_int(row, "sample_index")
    debug.elapsed_time_s = as_float(row, "elapsed_time_s")
    debug.gt_trajectory = gt_trajectory
    debug.ego_trajectory = ego_trajectory
    float_fields = (
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
        "cmd_accel",
        "cmd_brake",
        "cmd_steering_rad",
        "cmd_velocity_mps",
        "cmd_acceleration_mps2",
        "matched_gt_station_m",
        "target_velocity_mps",
        "lateral_error_m",
        "longitudinal_error_m",
        "heading_error_rad",
        "velocity_error_mps",
    )
    for field in float_fields:
        setattr(debug, field, as_float(row, field))
    debug.ego_unique_id = as_int(row, "ego_unique_id")
    debug.cmd_longl_type = as_int(row, "cmd_longl_type")
    debug.matched_gt_index = as_int(row, "matched_gt_index")
    debug.collision_detected = as_bool(row, "collision_detected")
    debug.reached_goal = as_bool(row, "reached_goal")
    debug.sample_valid = as_bool(row, "sample_valid")
    return debug


class HistoryPublisher:
    def __init__(self, traj_sample, run_id):
        if not SAFE_NAME.fullmatch(traj_sample) or not SAFE_NAME.fullmatch(run_id):
            raise ValueError("traj sample and run ID must match [A-Za-z0-9_-]+")
        root = repository_root()
        run_directory = (
            root / "base_ws/src/control/trajectory_validator/history" / traj_sample
        )
        self.csv_path = run_directory / (run_id + ".csv")
        self.json_path = run_directory / (run_id + ".json")
        if not self.csv_path.is_file() or not self.json_path.is_file():
            raise FileNotFoundError("history CSV/JSON pair does not exist")
        with self.json_path.open() as stream:
            self.metadata = json.load(stream)
        if self.metadata.get("traj_sample") != traj_sample or self.metadata.get("run_id") != run_id:
            raise ValueError("history metadata identity does not match requested run")
        expected_hash = self.metadata.get("integrity", {}).get("csv_sha256")
        if expected_hash and expected_hash != sha256_file(self.csv_path):
            raise ValueError("history CSV SHA-256 does not match metadata")
        gt_rows, self.sample_rows = load_run_csv(self.csv_path)
        if not self.sample_rows:
            raise ValueError("history run has no SAMPLE rows")
        frame_id = self.metadata.get("frame_id") or "map"
        self.gt_trajectory = gt_trajectory_from_rows(gt_rows, frame_id)
        self.ego_trajectory = Trajectory()
        self.ego_trajectory.header.frame_id = frame_id
        self.traj_sample = traj_sample
        self.run_id = run_id

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

    def run(self):
        rate = rospy.Rate(30.0)
        for row in self.sample_rows:
            if rospy.is_shutdown():
                return
            append_ego_point(self.ego_trajectory, row)
            stamp = rospy.Time.now()
            self.gt_trajectory.header.stamp = stamp
            self.ego_trajectory.header.stamp = stamp
            debug = debug_from_row(
                row,
                self.gt_trajectory,
                self.ego_trajectory,
                self.traj_sample,
                self.run_id,
                stamp,
            )
            self.debug_publisher.publish(debug)
            self.gt_path_publisher.publish(make_path(self.gt_trajectory, stamp))
            self.ego_path_publisher.publish(make_path(self.ego_trajectory, stamp))
            self.ego_pose_publisher.publish(make_ego_pose(debug, stamp))
            self.marker_publisher.publish(make_markers(debug, stamp))
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Publish one trajectory validation history run")
    parser.add_argument("--traj-sample", required=True)
    parser.add_argument("--run-id", required=True)
    arguments = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("trajectory_history_publisher")
    try:
        publisher = HistoryPublisher(arguments.traj_sample, arguments.run_id)
        publisher.run()
    except Exception as error:
        rospy.logfatal("publish_history: %s", error)
        raise SystemExit(1)
