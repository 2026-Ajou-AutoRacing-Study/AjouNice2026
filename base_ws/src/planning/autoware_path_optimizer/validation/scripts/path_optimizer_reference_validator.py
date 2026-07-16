#!/usr/bin/env python3

import json
import math
import os
from datetime import datetime, timezone

import numpy as np
import rospy
from autoware_planning_msgs.msg import Path, PathPoint, Trajectory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from autoware_path_optimizer.mpt_optimizer import MPTParameters, ModelPredictiveTrajectory
from autoware_path_optimizer.trajectory_utils import TrajectorySample, curvature


INPUT_TOPIC = "/planning/scenario_planning/lane_driving/motion_planning/path_smoother/path"
OUTPUT_TOPIC = "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory"
ODOMETRY_TOPIC = "/localization/kinematic_state"


def read_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def assign_pose(target, value):
    position = value["position"]
    orientation = value["orientation"]
    target.position.x = float(position["x"])
    target.position.y = float(position["y"])
    target.position.z = float(position.get("z", 0.0))
    target.orientation.x = float(orientation.get("x", 0.0))
    target.orientation.y = float(orientation.get("y", 0.0))
    target.orientation.z = float(orientation.get("z", 0.0))
    target.orientation.w = float(orientation.get("w", 1.0))


def pose_to_dict(value):
    return {
        "position": {"x": value.position.x, "y": value.position.y, "z": value.position.z},
        "orientation": {
            "x": value.orientation.x,
            "y": value.orientation.y,
            "z": value.orientation.z,
            "w": value.orientation.w,
        },
    }


def path_message(value):
    message = Path()
    message.header.frame_id = str(value.get("frame_id", "map"))
    for source in value.get("points", []):
        item = PathPoint()
        assign_pose(item.pose, source["pose"])
        item.longitudinal_velocity_mps = float(source.get("longitudinal_velocity_mps", 0.0))
        item.lateral_velocity_mps = float(source.get("lateral_velocity_mps", 0.0))
        item.heading_rate_rps = float(source.get("heading_rate_rps", 0.0))
        item.is_final = bool(source.get("is_final", False))
        message.points.append(item)
    for name in ("left_bound", "right_bound"):
        for source in value.get(name, []):
            getattr(message, name).append(
                Point(x=float(source["x"]), y=float(source["y"]), z=float(source.get("z", 0.0)))
            )
    return message


def odometry_message(value):
    message = Odometry()
    message.header.frame_id = str(value.get("frame_id", "map"))
    message.child_frame_id = str(value.get("child_frame_id", "base_link"))
    assign_pose(message.pose.pose, value["pose"])
    linear = value.get("linear_velocity", {})
    angular = value.get("angular_velocity", {})
    message.twist.twist.linear.x = float(linear.get("x", 0.0))
    message.twist.twist.linear.y = float(linear.get("y", 0.0))
    message.twist.twist.linear.z = float(linear.get("z", 0.0))
    message.twist.twist.angular.x = float(angular.get("x", 0.0))
    message.twist.twist.angular.y = float(angular.get("y", 0.0))
    message.twist.twist.angular.z = float(angular.get("z", 0.0))
    return message


def serialize_trajectory(message):
    return {
        "frame_id": message.header.frame_id,
        "points": [
            {
                "pose": pose_to_dict(item.pose),
                "longitudinal_velocity_mps": float(item.longitudinal_velocity_mps),
                "lateral_velocity_mps": float(item.lateral_velocity_mps),
                "acceleration_mps2": float(item.acceleration_mps2),
                "heading_rate_rps": float(item.heading_rate_rps),
                "front_wheel_angle_rad": float(item.front_wheel_angle_rad),
                "rear_wheel_angle_rad": float(item.rear_wheel_angle_rad),
            }
            for item in message.points
        ],
    }


def positions(document):
    return np.asarray(
        [
            [item["pose"]["position"]["x"], item["pose"]["position"]["y"]]
            for item in document["points"]
        ],
        dtype=float,
    )


def cumulative(points):
    if len(points) == 0:
        return np.asarray([], dtype=float)
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))


def point_to_polyline_distance(point, polyline):
    if len(polyline) < 2:
        return float(np.linalg.norm(point - polyline[0]))
    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    ratios = np.divide(
        np.einsum("ij,ij->i", point - starts, vectors),
        lengths_sq,
        out=np.zeros_like(lengths_sq),
        where=lengths_sq > 1.0e-12,
    )
    projected = starts + vectors * np.clip(ratios, 0.0, 1.0)[:, None]
    return float(np.min(np.linalg.norm(projected - point, axis=1)))


def first_stop(document):
    for index, point in enumerate(document["points"]):
        if abs(float(point["longitudinal_velocity_mps"])) < 1.0e-3:
            position = point["pose"]["position"]
            return index, np.asarray([position["x"], position["y"]], dtype=float)
    return None, None


def yaw_from_dict(point):
    value = point["pose"]["orientation"]
    return math.atan2(
        2.0 * (value.get("w", 1.0) * value.get("z", 0.0) + value.get("x", 0.0) * value.get("y", 0.0)),
        1.0 - 2.0 * (value.get("y", 0.0) ** 2 + value.get("z", 0.0) ** 2),
    )


def candidate_samples(document):
    return [
        TrajectorySample(
            x=float(point["pose"]["position"]["x"]),
            y=float(point["pose"]["position"]["y"]),
            z=float(point["pose"]["position"].get("z", 0.0)),
            yaw=yaw_from_dict(point),
            longitudinal_velocity_mps=float(point["longitudinal_velocity_mps"]),
        )
        for point in document["points"]
    ]


def compare(reference, candidate, tolerances):
    input_points = positions(reference["input_path"])
    reference_points = positions(reference["output_trajectory"])
    candidate_points = positions(candidate["output_trajectory"])
    reference_stations = cumulative(reference_points)
    candidate_stations = cumulative(candidate_points)
    distances = [point_to_polyline_distance(point, reference_points) for point in candidate_points]
    distances.extend(point_to_polyline_distance(point, candidate_points) for point in reference_points)
    reference_effect = [
        point_to_polyline_distance(point, input_points) for point in reference_points
    ]
    candidate_effect = [
        point_to_polyline_distance(point, input_points) for point in candidate_points
    ]
    strong_effect_threshold = max(0.05, 0.5 * max(reference_effect))
    strong_effect_indices = [
        index
        for index, effect in enumerate(reference_effect)
        if effect >= strong_effect_threshold
    ]
    strong_effect_errors = [
        point_to_polyline_distance(reference_points[index], candidate_points)
        for index in strong_effect_indices
    ]
    nearest = [
        int(np.argmin(np.linalg.norm(candidate_points - point, axis=1)))
        for point in reference_points
    ]
    reference_velocity = np.asarray(
        [point["longitudinal_velocity_mps"] for point in reference["output_trajectory"]["points"]]
    )
    candidate_velocity = np.asarray(
        [point["longitudinal_velocity_mps"] for point in candidate["output_trajectory"]["points"]]
    )
    reference_stop_index, reference_stop = first_stop(reference["output_trajectory"])
    candidate_stop_index, candidate_stop = first_stop(candidate["output_trajectory"])
    stop_error = (
        float(np.linalg.norm(reference_stop - candidate_stop))
        if reference_stop is not None and candidate_stop is not None
        else (0.0 if reference_stop is None and candidate_stop is None else math.inf)
    )
    input_path = reference["input_path"]
    samples = candidate_samples(candidate["output_trajectory"])
    left = [(point["x"], point["y"], point.get("z", 0.0)) for point in input_path["left_bound"]]
    right = [(point["x"], point["y"], point.get("z", 0.0)) for point in input_path["right_bound"]]
    optimizer = ModelPredictiveTrajectory(MPTParameters())
    lower, upper = optimizer.lateral_bounds(samples, left, right)
    boundary_violation = float(max(np.max(lower), np.max(-upper), 0.0))
    curvatures = curvature(samples)
    max_curvature = float(max(abs(value) for value in curvatures)) if curvatures else 0.0
    max_inferred_steer = float(math.atan(MPTParameters().wheel_base * max_curvature))
    diagnostics = {
        "reference_point_count": int(len(reference_points)),
        "candidate_point_count": int(len(candidate_points)),
        "reference_length_m": float(reference_stations[-1]),
        "candidate_length_m": float(candidate_stations[-1]),
        "length_error_m": float(abs(reference_stations[-1] - candidate_stations[-1])),
        "start_error_m": float(np.linalg.norm(reference_points[0] - candidate_points[0])),
        "end_error_m": float(np.linalg.norm(reference_points[-1] - candidate_points[-1])),
        "geometry_rmse_m": float(math.sqrt(np.mean(np.square(distances)))),
        "geometry_max_error_m": float(max(distances)),
        "reference_input_to_output_rmse_m": float(
            math.sqrt(np.mean(np.square(reference_effect)))
        ),
        "reference_input_to_output_max_m": float(max(reference_effect)),
        "candidate_input_to_output_rmse_m": float(
            math.sqrt(np.mean(np.square(candidate_effect)))
        ),
        "candidate_input_to_output_max_m": float(max(candidate_effect)),
        "strong_effect_threshold_m": float(strong_effect_threshold),
        "strong_effect_point_count": len(strong_effect_indices),
        "strong_effect_geometry_rmse_m": float(
            math.sqrt(np.mean(np.square(strong_effect_errors)))
        )
        if strong_effect_errors
        else 0.0,
        "strong_effect_geometry_max_error_m": float(max(strong_effect_errors))
        if strong_effect_errors
        else 0.0,
        "velocity_rmse_mps": float(
            math.sqrt(np.mean(np.square(reference_velocity - candidate_velocity[nearest])))
        ),
        "reference_first_zero_index": reference_stop_index,
        "candidate_first_zero_index": candidate_stop_index,
        "first_stop_pose_error_m": stop_error,
        "boundary_violation_m": boundary_violation,
        "max_curvature_1pm": max_curvature,
        "max_inferred_steer_rad": max_inferred_steer,
    }
    checks = {
        "finite_geometry": bool(np.all(np.isfinite(candidate_points))),
        "geometry_rmse": diagnostics["geometry_rmse_m"] <= tolerances["geometry_rmse_m"],
        "geometry_max_error": diagnostics["geometry_max_error_m"] <= tolerances["geometry_max_error_m"],
        "start_pose": diagnostics["start_error_m"] <= tolerances["endpoint_error_m"],
        "end_pose": diagnostics["end_error_m"] <= tolerances["endpoint_error_m"],
        "length": diagnostics["length_error_m"] <= tolerances["length_error_m"],
        "velocity": diagnostics["velocity_rmse_mps"] <= tolerances["velocity_rmse_mps"],
        "stop_pose": diagnostics["first_stop_pose_error_m"] <= tolerances["stop_pose_error_m"],
        "inside_drivable_area": boundary_violation <= tolerances["boundary_violation_m"],
        "steer_limit": max_inferred_steer <= tolerances["max_steer_rad"],
        "optimizer_effect": diagnostics["reference_input_to_output_max_m"]
        >= tolerances["minimum_optimizer_effect_m"],
        "strong_effect_geometry_rmse": diagnostics["strong_effect_geometry_rmse_m"]
        <= tolerances["strong_effect_geometry_rmse_m"],
        "strong_effect_geometry_max_error": diagnostics[
            "strong_effect_geometry_max_error_m"
        ]
        <= tolerances["strong_effect_geometry_max_error_m"],
    }
    return diagnostics, checks


class ReferenceValidator:
    def __init__(self):
        self.reference_path = rospy.get_param("~reference_json")
        self.candidate_path = rospy.get_param("~candidate_json")
        self.report_path = rospy.get_param("~report_json")
        self.shutdown_on_result = bool(rospy.get_param("~shutdown_on_result", False))
        self.reference = read_json(self.reference_path)
        self.completed = False
        self.tolerances = {
            "geometry_rmse_m": float(rospy.get_param("~geometry_rmse_tolerance_m", 0.15)),
            "geometry_max_error_m": float(rospy.get_param("~geometry_max_tolerance_m", 0.5)),
            "endpoint_error_m": float(rospy.get_param("~endpoint_tolerance_m", 0.3)),
            "length_error_m": float(rospy.get_param("~length_tolerance_m", 1.0)),
            "velocity_rmse_mps": float(rospy.get_param("~velocity_rmse_tolerance_mps", 0.2)),
            "stop_pose_error_m": float(rospy.get_param("~stop_pose_tolerance_m", 0.75)),
            "boundary_violation_m": float(rospy.get_param("~boundary_violation_tolerance_m", 0.02)),
            "max_steer_rad": float(rospy.get_param("~max_steer_tolerance_rad", 0.63)),
            "minimum_optimizer_effect_m": float(
                rospy.get_param("~minimum_optimizer_effect_m", 0.0)
            ),
            "strong_effect_geometry_rmse_m": float(
                rospy.get_param("~strong_effect_geometry_rmse_tolerance_m", 0.15)
            ),
            "strong_effect_geometry_max_error_m": float(
                rospy.get_param("~strong_effect_geometry_max_tolerance_m", 0.25)
            ),
        }
        self.input_message = path_message(self.reference["input_path"])
        self.odometry_message = odometry_message(self.reference["scenario"]["odometry"])
        self.input_pub = rospy.Publisher(INPUT_TOPIC, Path, queue_size=1)
        self.odom_pub = rospy.Publisher(ODOMETRY_TOPIC, Odometry, queue_size=1)
        self.result_pub = rospy.Publisher(
            "/validation/path_optimizer/result", String, queue_size=1, latch=True
        )
        rospy.Subscriber(OUTPUT_TOPIC, Trajectory, self.on_candidate, queue_size=1)
        rospy.Timer(rospy.Duration(0.2), self.publish_inputs)

    def publish_inputs(self, _event):
        if self.completed:
            return
        stamp = rospy.Time.now()
        self.input_message.header.stamp = stamp
        self.odometry_message.header.stamp = stamp
        self.odom_pub.publish(self.odometry_message)
        self.input_pub.publish(self.input_message)

    def on_candidate(self, message):
        if self.completed or not message.points:
            return
        candidate = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "format": "autoware_path_optimizer",
            "format_version": 1,
            "source": "ros1_autoware_path_optimizer_mvp",
            "scenario": self.reference["scenario"],
            "input_path": self.reference["input_path"],
            "output_trajectory": serialize_trajectory(message),
        }
        diagnostics, checks = compare(self.reference, candidate, self.tolerances)
        report = {
            "passed": all(checks.values()),
            "scenario": self.reference["scenario"].get("name", "path_optimizer"),
            "reference": self.reference_path,
            "candidate": self.candidate_path,
            "tolerances": self.tolerances,
            "diagnostics": diagnostics,
            "checks": checks,
        }
        write_json(self.candidate_path, candidate)
        write_json(self.report_path, report)
        self.result_pub.publish(String(data=json.dumps(report, sort_keys=True)))
        self.completed = True
        rospy.loginfo(
            "PATH OPTIMIZER VALIDATION: %s | RMSE %.4f m, max %.4f m, strong RMSE %.4f m, strong max %.4f m, input change %.4f m, stop %.4f m",
            "PASS" if report["passed"] else "FAIL",
            diagnostics["geometry_rmse_m"],
            diagnostics["geometry_max_error_m"],
            diagnostics["strong_effect_geometry_rmse_m"],
            diagnostics["strong_effect_geometry_max_error_m"],
            diagnostics["reference_input_to_output_max_m"],
            diagnostics["first_stop_pose_error_m"],
        )
        if self.shutdown_on_result:
            rospy.signal_shutdown("path optimizer validation complete")


def main():
    rospy.init_node("path_optimizer_reference_validator")
    ReferenceValidator()
    rospy.spin()


if __name__ == "__main__":
    main()
