#!/usr/bin/env python3

import json
import math
import os
from datetime import datetime, timezone

import numpy as np
import rospy
from autoware_planning_msgs.msg import Path, PathPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def read_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


def copy_position(target, source):
    target.x = float(source["x"])
    target.y = float(source["y"])
    target.z = float(source.get("z", 0.0))


def copy_orientation(target, source):
    target.x = float(source.get("x", 0.0))
    target.y = float(source.get("y", 0.0))
    target.z = float(source.get("z", 0.0))
    target.w = float(source.get("w", 1.0))


def path_message(document):
    path = Path()
    path.header.frame_id = document.get("frame_id", "map")
    path.header.stamp = rospy.Time.now()
    for value in document["points"]:
        point = PathPoint()
        copy_position(point.pose.position, value["pose"]["position"])
        copy_orientation(point.pose.orientation, value["pose"]["orientation"])
        point.longitudinal_velocity_mps = float(value["longitudinal_velocity_mps"])
        point.lateral_velocity_mps = float(value.get("lateral_velocity_mps", 0.0))
        point.heading_rate_rps = float(value.get("heading_rate_rps", 0.0))
        point.is_final = bool(value.get("is_final", False))
        path.points.append(point)
    for source in document.get("left_bound", []):
        point = Point()
        copy_position(point, source)
        path.left_bound.append(point)
    for source in document.get("right_bound", []):
        point = Point()
        copy_position(point, source)
        path.right_bound.append(point)
    return path


def odometry_message(document):
    message = Odometry()
    message.header.frame_id = document.get("frame_id", "map")
    message.header.stamp = rospy.Time.now()
    message.child_frame_id = document.get("child_frame_id", "base_link")
    copy_position(message.pose.pose.position, document["pose"]["position"])
    copy_orientation(message.pose.pose.orientation, document["pose"]["orientation"])
    linear = document.get("linear_velocity", {})
    angular = document.get("angular_velocity", {})
    message.twist.twist.linear.x = float(linear.get("x", 0.0))
    message.twist.twist.linear.y = float(linear.get("y", 0.0))
    message.twist.twist.linear.z = float(linear.get("z", 0.0))
    message.twist.twist.angular.x = float(angular.get("x", 0.0))
    message.twist.twist.angular.y = float(angular.get("y", 0.0))
    message.twist.twist.angular.z = float(angular.get("z", 0.0))
    return message


def serialize_path(message):
    def serialize_point(value):
        return {"x": float(value.x), "y": float(value.y), "z": float(value.z)}

    return {
        "frame_id": message.header.frame_id,
        "points": [
            {
                "pose": {
                    "position": serialize_point(item.pose.position),
                    "orientation": {
                        "x": float(item.pose.orientation.x),
                        "y": float(item.pose.orientation.y),
                        "z": float(item.pose.orientation.z),
                        "w": float(item.pose.orientation.w),
                    },
                },
                "longitudinal_velocity_mps": float(item.longitudinal_velocity_mps),
                "lateral_velocity_mps": float(item.lateral_velocity_mps),
                "heading_rate_rps": float(item.heading_rate_rps),
                "is_final": bool(item.is_final),
            }
            for item in message.points
        ],
        "left_bound": [serialize_point(item) for item in message.left_bound],
        "right_bound": [serialize_point(item) for item in message.right_bound],
    }


def positions(document):
    return np.asarray(
        [
            [
                item["pose"]["position"]["x"],
                item["pose"]["position"]["y"],
                item["pose"]["position"].get("z", 0.0),
            ]
            for item in document["points"]
        ],
        dtype=float,
    )


def cumulative(points):
    if len(points) == 0:
        return np.asarray([], dtype=float)
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))))


def point_to_polyline_distance(point, polyline):
    if len(polyline) < 2:
        return float(np.linalg.norm(point[:2] - polyline[0, :2]))
    starts = polyline[:-1, :2]
    vectors = polyline[1:, :2] - starts
    lengths_sq = np.einsum("ij,ij->i", vectors, vectors)
    relative = point[:2] - starts
    ratios = np.divide(
        np.einsum("ij,ij->i", relative, vectors),
        lengths_sq,
        out=np.zeros_like(lengths_sq),
        where=lengths_sq > 1.0e-12,
    )
    ratios = np.clip(ratios, 0.0, 1.0)
    projected = starts + vectors * ratios[:, None]
    return float(np.min(np.linalg.norm(projected - point[:2], axis=1)))


def first_stop(document):
    for index, point in enumerate(document["points"]):
        if abs(float(point["longitudinal_velocity_mps"])) < 1.0e-3:
            return index, np.asarray(
                [
                    point["pose"]["position"]["x"],
                    point["pose"]["position"]["y"],
                ],
                dtype=float,
            )
    return None, None


def bound_error(reference, candidate, key):
    reference_points = np.asarray(
        [[point["x"], point["y"]] for point in reference.get(key, [])], dtype=float
    )
    candidate_points = np.asarray(
        [[point["x"], point["y"]] for point in candidate.get(key, [])], dtype=float
    )
    if reference_points.shape != candidate_points.shape:
        return math.inf
    if len(reference_points) == 0:
        return 0.0
    return float(np.max(np.linalg.norm(reference_points - candidate_points, axis=1)))


def compare(reference, candidate, tolerances):
    reference_points = positions(reference)
    candidate_points = positions(candidate)
    reference_stations = cumulative(reference_points)
    candidate_stations = cumulative(candidate_points)
    distances = [point_to_polyline_distance(point, reference_points) for point in candidate_points]
    distances.extend(point_to_polyline_distance(point, candidate_points) for point in reference_points)
    reference_velocity = np.asarray(
        [point["longitudinal_velocity_mps"] for point in reference["points"]], dtype=float
    )
    candidate_velocity = np.asarray(
        [point["longitudinal_velocity_mps"] for point in candidate["points"]], dtype=float
    )
    # Compare velocity at the nearest physical path point. Arc-length interpolation
    # is misleading at an inserted zero-velocity point because a millimetre station
    # shift blends a discontinuous stop value with the preceding speed.
    nearest_candidate_indices = [
        int(np.argmin(np.linalg.norm(candidate_points[:, :2] - point[:2], axis=1)))
        for point in reference_points
    ]
    sampled_velocity = candidate_velocity[nearest_candidate_indices]
    reference_stop_index, reference_stop = first_stop(reference)
    candidate_stop_index, candidate_stop = first_stop(candidate)
    stop_error = (
        float(np.linalg.norm(reference_stop - candidate_stop))
        if reference_stop is not None and candidate_stop is not None
        else (0.0 if reference_stop is None and candidate_stop is None else math.inf)
    )
    spacing = np.diff(candidate_stations)
    diagnostics = {
        "reference_point_count": int(len(reference_points)),
        "candidate_point_count": int(len(candidate_points)),
        "reference_length_m": float(reference_stations[-1]),
        "candidate_length_m": float(candidate_stations[-1]),
        "length_error_m": float(abs(reference_stations[-1] - candidate_stations[-1])),
        "start_error_m": float(np.linalg.norm(reference_points[0, :2] - candidate_points[0, :2])),
        "end_error_m": float(np.linalg.norm(reference_points[-1, :2] - candidate_points[-1, :2])),
        "geometry_rmse_m": float(math.sqrt(np.mean(np.square(distances)))),
        "geometry_max_error_m": float(max(distances)),
        "velocity_rmse_mps": float(math.sqrt(np.mean(np.square(reference_velocity - sampled_velocity)))),
        "reference_first_zero_index": reference_stop_index,
        "candidate_first_zero_index": candidate_stop_index,
        "first_stop_pose_error_m": stop_error,
        "candidate_mean_spacing_m": float(np.mean(spacing)) if len(spacing) else 0.0,
        "candidate_max_spacing_m": float(np.max(spacing)) if len(spacing) else 0.0,
        "left_bound_max_error_m": bound_error(reference, candidate, "left_bound"),
        "right_bound_max_error_m": bound_error(reference, candidate, "right_bound"),
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
        "left_bound_copy": diagnostics["left_bound_max_error_m"] <= 1.0e-5,
        "right_bound_copy": diagnostics["right_bound_max_error_m"] <= 1.0e-5,
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
            "endpoint_error_m": float(rospy.get_param("~endpoint_tolerance_m", 0.25)),
            "length_error_m": float(rospy.get_param("~length_tolerance_m", 0.75)),
            "velocity_rmse_mps": float(rospy.get_param("~velocity_rmse_tolerance_mps", 0.15)),
            "stop_pose_error_m": float(rospy.get_param("~stop_pose_tolerance_m", 0.75)),
        }
        self.input_message = path_message(self.reference["input_path"])
        self.odometry_message = odometry_message(self.reference["scenario"]["odometry"])
        self.input_publisher = rospy.Publisher(
            "/planning/scenario_planning/lane_driving/behavior_planning/path",
            Path,
            queue_size=1,
            latch=True,
        )
        self.odometry_publisher = rospy.Publisher(
            "/localization/kinematic_state", Odometry, queue_size=1, latch=True
        )
        self.result_publisher = rospy.Publisher(
            "/validation/path_smoother/result", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/path_smoother/path",
            Path,
            self.on_candidate,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_input)

    def publish_input(self, _event):
        if self.completed:
            return
        stamp = rospy.Time.now()
        self.input_message.header.stamp = stamp
        self.odometry_message.header.stamp = stamp
        self.odometry_publisher.publish(self.odometry_message)
        self.input_publisher.publish(self.input_message)

    def on_candidate(self, message):
        if self.completed or not message.points:
            return
        candidate_output = serialize_path(message)
        candidate = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "format": "autoware_path_smoother",
            "format_version": 1,
            "scenario": self.reference["scenario"],
            "source": "ros1_autoware_path_smoother_port",
            "input_path": self.reference["input_path"],
            "output_path": candidate_output,
        }
        diagnostics, checks = compare(
            self.reference["output_path"], candidate_output, self.tolerances
        )
        passed = all(checks.values())
        report = {
            "passed": passed,
            "scenario": self.reference["scenario"].get("name", "path_smoother"),
            "reference": self.reference_path,
            "candidate": self.candidate_path,
            "tolerances": self.tolerances,
            "checks": checks,
            "diagnostics": diagnostics,
        }
        write_json(self.candidate_path, candidate)
        write_json(self.report_path, report)
        self.completed = True
        self.result_publisher.publish(String(data=json.dumps(report, sort_keys=True)))
        rospy.loginfo(
            "PATH SMOOTHER VALIDATION: %s | geometry RMSE %.4f m, max %.4f m, "
            "length error %.4f m, stop error %.4f m, velocity RMSE %.4f m/s",
            "PASS" if passed else "FAIL",
            diagnostics["geometry_rmse_m"],
            diagnostics["geometry_max_error_m"],
            diagnostics["length_error_m"],
            diagnostics["first_stop_pose_error_m"],
            diagnostics["velocity_rmse_mps"],
        )
        if self.shutdown_on_result:
            rospy.signal_shutdown("validation complete")


def main():
    rospy.init_node("path_smoother_reference_validator")
    ReferenceValidator()
    rospy.spin()


if __name__ == "__main__":
    main()
