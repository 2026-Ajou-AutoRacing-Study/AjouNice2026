#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime, timezone

import rospy
from autoware_planning_msgs.msg import Trajectory


def cumulative_lengths(points):
    lengths = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        lengths.append(
            lengths[-1]
            + math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        )
    return lengths


def first_stop_index(points, epsilon):
    return next(
        (
            index
            for index, point in enumerate(points)
            if abs(point.longitudinal_velocity_mps) <= epsilon
        ),
        None,
    )


def yaw_from_quaternion(value):
    siny_cosp = 2.0 * (value.w * value.z + value.x * value.y)
    cosy_cosp = 1.0 - 2.0 * (value.y * value.y + value.z * value.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalized_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def rounded(value):
    return round(float(value), 9)


class PathOptimizerMvpPipelineValidator:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "path_optimizer/trajectory",
        )
        self.output_topic = rospy.get_param(
            "~output_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory",
        )
        self.report_json = os.path.expanduser(
            rospy.get_param(
                "~report_json",
                "~/AJOUNICE2026/local_autoware_planning_sim/results/"
                "motion_velocity_planner/kcity_path_optimizer_mvp.report.json",
            )
        )
        default_points_csv = self.report_json
        if default_points_csv.endswith(".report.json"):
            default_points_csv = default_points_csv[: -len(".report.json")] + ".points.csv"
        else:
            default_points_csv += ".points.csv"
        self.points_csv = os.path.expanduser(
            rospy.get_param("~points_csv", default_points_csv)
        )
        self.timeout = max(float(rospy.get_param("~timeout", 40.0)), 1.0)
        self.geometry_tolerance = max(
            float(rospy.get_param("~geometry_tolerance", 1.0e-6)), 0.0
        )
        self.zero_epsilon = max(
            float(rospy.get_param("~zero_velocity_epsilon", 1.0e-3)), 0.0
        )
        self.require_same_first_stop = bool(
            rospy.get_param("~require_same_first_stop", True)
        )
        self.shutdown_on_result = bool(
            rospy.get_param("~shutdown_on_result", False)
        )
        self.input_trajectory = None
        self.output_trajectory = None
        self.started_at = rospy.Time.now()

        rospy.Subscriber(
            self.input_topic, Trajectory, self.on_input, queue_size=1
        )
        rospy.Subscriber(
            self.output_topic, Trajectory, self.on_output, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self.on_timer)

    def on_input(self, message):
        if message.points:
            self.input_trajectory = message

    def on_output(self, message):
        if message.points:
            self.output_trajectory = message

    def compare(self):
        input_points = self.input_trajectory.points
        output_points = self.output_trajectory.points
        paired_count = min(len(input_points), len(output_points))
        xy_errors = [
            math.hypot(
                input_points[index].pose.position.x
                - output_points[index].pose.position.x,
                input_points[index].pose.position.y
                - output_points[index].pose.position.y,
            )
            for index in range(paired_count)
        ]
        input_velocities = [
            float(point.longitudinal_velocity_mps) for point in input_points
        ]
        output_velocities = [
            float(point.longitudinal_velocity_mps) for point in output_points
        ]
        velocity_increase_violations = sum(
            1
            for input_velocity, output_velocity in zip(
                input_velocities, output_velocities
            )
            if output_velocity > input_velocity + 1.0e-6
        )
        upstream_zero_violations = sum(
            1
            for input_velocity, output_velocity in zip(
                input_velocities, output_velocities
            )
            if abs(input_velocity) <= self.zero_epsilon
            and abs(output_velocity) > self.zero_epsilon
        )
        input_stop_index = first_stop_index(input_points, self.zero_epsilon)
        output_stop_index = first_stop_index(output_points, self.zero_epsilon)
        input_lengths = cumulative_lengths(input_points)
        output_lengths = cumulative_lengths(output_points)
        same_first_stop = input_stop_index == output_stop_index
        point_count_equal = len(input_points) == len(output_points)
        max_xy_error = max(xy_errors) if xy_errors else None
        geometry_preserved = (
            point_count_equal
            and max_xy_error is not None
            and max_xy_error <= self.geometry_tolerance
        )
        passed = (
            geometry_preserved
            and velocity_increase_violations == 0
            and upstream_zero_violations == 0
            and (same_first_stop or not self.require_same_first_stop)
        )

        return {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "scenario": "kcity_v44_path_optimizer_to_motion_velocity_planner",
            "passed": passed,
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "input_frame_id": self.input_trajectory.header.frame_id,
            "output_frame_id": self.output_trajectory.header.frame_id,
            "input_points": len(input_points),
            "output_points": len(output_points),
            "point_count_equal": point_count_equal,
            "geometry_tolerance_m": self.geometry_tolerance,
            "max_xy_error_m": max_xy_error,
            "input_max_velocity_mps": max(input_velocities),
            "output_max_velocity_mps": max(output_velocities),
            "velocity_increase_violations": velocity_increase_violations,
            "upstream_zero_count": sum(
                1 for value in input_velocities if abs(value) <= self.zero_epsilon
            ),
            "upstream_zero_preservation_violations": upstream_zero_violations,
            "input_first_stop_index": input_stop_index,
            "output_first_stop_index": output_stop_index,
            "input_first_stop_station_m": (
                None if input_stop_index is None else input_lengths[input_stop_index]
            ),
            "output_first_stop_station_m": (
                None if output_stop_index is None else output_lengths[output_stop_index]
            ),
            "same_first_stop": same_first_stop,
            "points_csv": self.points_csv,
        }

    def point_table_rows(self):
        input_points = self.input_trajectory.points
        output_points = self.output_trajectory.points
        input_lengths = cumulative_lengths(input_points)
        output_lengths = cumulative_lengths(output_points)
        rows = []
        for index in range(max(len(input_points), len(output_points))):
            input_point = input_points[index] if index < len(input_points) else None
            output_point = output_points[index] if index < len(output_points) else None
            row = {
                "point_index": index,
                "input_station_m": "" if input_point is None else rounded(input_lengths[index]),
                "output_station_m": "" if output_point is None else rounded(output_lengths[index]),
            }
            for prefix, point in (("input", input_point), ("output", output_point)):
                if point is None:
                    row.update(
                        {
                            prefix + "_time_from_start_s": "",
                            prefix + "_position_x_m": "",
                            prefix + "_position_y_m": "",
                            prefix + "_position_z_m": "",
                            prefix + "_orientation_x": "",
                            prefix + "_orientation_y": "",
                            prefix + "_orientation_z": "",
                            prefix + "_orientation_w": "",
                            prefix + "_yaw_rad": "",
                            prefix + "_longitudinal_velocity_mps": "",
                            prefix + "_lateral_velocity_mps": "",
                            prefix + "_acceleration_mps2": "",
                            prefix + "_heading_rate_rps": "",
                            prefix + "_front_wheel_angle_rad": "",
                            prefix + "_rear_wheel_angle_rad": "",
                            prefix + "_is_stop": "",
                        }
                    )
                    continue
                pose = point.pose
                row.update(
                    {
                        prefix + "_time_from_start_s": rounded(point.time_from_start.to_sec()),
                        prefix + "_position_x_m": rounded(pose.position.x),
                        prefix + "_position_y_m": rounded(pose.position.y),
                        prefix + "_position_z_m": rounded(pose.position.z),
                        prefix + "_orientation_x": rounded(pose.orientation.x),
                        prefix + "_orientation_y": rounded(pose.orientation.y),
                        prefix + "_orientation_z": rounded(pose.orientation.z),
                        prefix + "_orientation_w": rounded(pose.orientation.w),
                        prefix + "_yaw_rad": rounded(yaw_from_quaternion(pose.orientation)),
                        prefix + "_longitudinal_velocity_mps": rounded(
                            point.longitudinal_velocity_mps
                        ),
                        prefix + "_lateral_velocity_mps": rounded(
                            point.lateral_velocity_mps
                        ),
                        prefix + "_acceleration_mps2": rounded(point.acceleration_mps2),
                        prefix + "_heading_rate_rps": rounded(point.heading_rate_rps),
                        prefix + "_front_wheel_angle_rad": rounded(
                            point.front_wheel_angle_rad
                        ),
                        prefix + "_rear_wheel_angle_rad": rounded(
                            point.rear_wheel_angle_rad
                        ),
                        prefix + "_is_stop": int(
                            abs(point.longitudinal_velocity_mps) <= self.zero_epsilon
                        ),
                    }
                )

            if input_point is None or output_point is None:
                row.update(
                    {
                        "xy_position_error_m": "",
                        "z_position_error_m": "",
                        "yaw_error_rad": "",
                        "longitudinal_velocity_delta_mps": "",
                        "lateral_velocity_delta_mps": "",
                        "acceleration_delta_mps2": "",
                        "upstream_stop_preserved": "",
                    }
                )
            else:
                input_pose = input_point.pose
                output_pose = output_point.pose
                row.update(
                    {
                        "xy_position_error_m": rounded(
                            math.hypot(
                                output_pose.position.x - input_pose.position.x,
                                output_pose.position.y - input_pose.position.y,
                            )
                        ),
                        "z_position_error_m": rounded(
                            output_pose.position.z - input_pose.position.z
                        ),
                        "yaw_error_rad": rounded(
                            normalized_angle(
                                yaw_from_quaternion(output_pose.orientation)
                                - yaw_from_quaternion(input_pose.orientation)
                            )
                        ),
                        "longitudinal_velocity_delta_mps": rounded(
                            output_point.longitudinal_velocity_mps
                            - input_point.longitudinal_velocity_mps
                        ),
                        "lateral_velocity_delta_mps": rounded(
                            output_point.lateral_velocity_mps
                            - input_point.lateral_velocity_mps
                        ),
                        "acceleration_delta_mps2": rounded(
                            output_point.acceleration_mps2
                            - input_point.acceleration_mps2
                        ),
                        "upstream_stop_preserved": int(
                            abs(input_point.longitudinal_velocity_mps) > self.zero_epsilon
                            or abs(output_point.longitudinal_velocity_mps)
                            <= self.zero_epsilon
                        ),
                    }
                )
            rows.append(row)
        return rows

    def write_point_table(self):
        rows = self.point_table_rows()
        directory = os.path.dirname(self.points_csv)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.points_csv, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    def write_report(self, report):
        directory = os.path.dirname(self.report_json)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.report_json, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")

    def finish(self, report):
        self.timer.shutdown()
        if "error" not in report:
            self.write_point_table()
        self.write_report(report)
        if "error" in report:
            rospy.logerr(
                "PATH OPTIMIZER + MVP VALIDATION: FAIL error=%s report=%s",
                report["error"],
                self.report_json,
            )
        else:
            rospy.loginfo(
                "PATH OPTIMIZER + MVP VALIDATION: %s points=%d/%d "
                "max_xy_error=%.6f m first_stop=%s/%s report=%s table=%s",
                "PASS" if report["passed"] else "FAIL",
                report["input_points"],
                report["output_points"],
                report["max_xy_error_m"],
                report["input_first_stop_index"],
                report["output_first_stop_index"],
                self.report_json,
                self.points_csv,
            )
        if self.shutdown_on_result:
            rospy.signal_shutdown("path optimizer + MVP validation complete")

    def on_timer(self, _event):
        if self.input_trajectory is not None and self.output_trajectory is not None:
            self.finish(self.compare())
            return
        if (rospy.Time.now() - self.started_at).to_sec() >= self.timeout:
            self.finish(
                {
                    "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                    "scenario": "kcity_v44_path_optimizer_to_motion_velocity_planner",
                    "passed": False,
                    "error": "timed out waiting for non-empty trajectories",
                    "input_received": self.input_trajectory is not None,
                    "output_received": self.output_trajectory is not None,
                }
            )


def main():
    rospy.init_node("path_optimizer_mvp_pipeline_validator")
    PathOptimizerMvpPipelineValidator()
    rospy.spin()


if __name__ == "__main__":
    main()
