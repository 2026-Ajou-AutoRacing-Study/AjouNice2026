#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys
import time

import rospy
import yaml
import rosgraph
import rospkg

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    Shape,
)


DEFAULT_INPUT_TRAJECTORY_TOPIC = (
    "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory"
)
DEFAULT_OBJECTS_TOPIC = "/perception/object_recognition/objects"
DEFAULT_ODOMETRY_TOPIC = "/localization/kinematic_state"
DEFAULT_OUTPUT_TRAJECTORY_TOPIC = (
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "motion_velocity_planner/trajectory"
)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def make_trajectory(num_points, spacing, default_speed):
    trajectory = Trajectory()
    trajectory.header = Header(frame_id="map")
    for i in range(num_points):
        point = TrajectoryPoint()
        point.pose.position.x = float(i) * spacing
        point.pose.position.y = 0.0
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = float(default_speed)
        trajectory.points.append(point)
    return trajectory


def make_odometry(speed):
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.orientation = quaternion_from_yaw(0.0)
    msg.twist.twist.linear.x = float(speed)
    return msg


def make_object(case, object_id):
    obj = PredictedObject()
    obj.object_id.uuid = [object_id] + [0] * 15
    obj.existence_probability = float(case.get("existence_probability", 1.0))

    cls = ObjectClassification()
    cls.label = int(case.get("label", ObjectClassification.CAR))
    cls.probability = float(case.get("classification_probability", 1.0))
    obj.classification.append(cls)

    obj.shape.type = Shape.BOUNDING_BOX
    obj.shape.dimensions.x = float(case.get("object_length", 4.5))
    obj.shape.dimensions.y = float(case.get("object_width", 2.0))
    obj.shape.dimensions.z = float(case.get("object_height", 1.6))

    pose = obj.kinematics.initial_pose_with_covariance.pose
    pose.position.x = float(case.get("object_x", 30.0))
    pose.position.y = float(case.get("object_y", 0.0))
    pose.orientation = quaternion_from_yaw(float(case.get("object_yaw", math.pi)))

    obj.kinematics.initial_twist_with_covariance.twist.linear.x = float(
        case.get("object_speed", 0.0)
    )
    return obj


def station_for_point(path_point):
    return float(path_point.pose.position.x)


def cumulative_stations(points):
    stations = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        stations.append(
            stations[-1]
            + math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        )
    return stations


def summarize_path(path_msg, zero_epsilon, stop_velocity_epsilon):
    if path_msg is None or not path_msg.points:
        return {
            "received_output": False,
            "stop_x": None,
            "min_velocity": None,
            "max_velocity": None,
            "zero_count": 0,
        }

    velocities = [float(point.longitudinal_velocity_mps) for point in path_msg.points]
    stop_x = None
    for point, velocity in zip(path_msg.points, velocities):
        if abs(velocity) <= zero_epsilon:
            stop_x = station_for_point(point)
            break

    first_slow_x = None
    for point, velocity in zip(path_msg.points, velocities):
        if velocity <= stop_velocity_epsilon:
            first_slow_x = station_for_point(point)
            break

    return {
        "received_output": True,
        "stop_x": stop_x,
        "first_slow_x": first_slow_x,
        "min_velocity": min(velocities),
        "max_velocity": max(velocities),
        "zero_count": sum(1 for value in velocities if abs(value) <= zero_epsilon),
    }


class MvpCaseRunner:
    def __init__(self, args, cases):
        self.args = args
        self.cases = cases
        self.latest_output = None
        self.expected_stamps = set()

        rospy.init_node("mvp_validation_runner", anonymous=True)

        self.trajectory_pub = rospy.Publisher(
            args.input_trajectory_topic, Trajectory, queue_size=1, latch=True
        )
        self.odom_pub = rospy.Publisher(args.odometry_topic, Odometry, queue_size=1, latch=True)
        self.objects_pub = rospy.Publisher(args.objects_topic, PredictedObjects, queue_size=1, latch=True)
        self.output_sub = rospy.Subscriber(
            args.output_trajectory_topic, Trajectory, self.output_cb
        )

    def output_cb(self, msg):
        if msg.header.stamp.to_nsec() in self.expected_stamps:
            self.latest_output = msg

    def publish_case_inputs(self, case, object_id):
        now = rospy.Time.now()
        trajectory = make_trajectory(
            num_points=int(case.get("path_points", self.args.path_points)),
            spacing=float(case.get("path_spacing", self.args.path_spacing)),
            default_speed=float(case.get("path_speed", self.args.path_speed)),
        )
        odom = make_odometry(float(case.get("ego_speed", self.args.ego_speed)))
        objects = PredictedObjects()
        objects.header.frame_id = "map"

        trajectory.header.stamp = now
        self.expected_stamps.add(now.to_nsec())
        odom.header.stamp = now
        objects.header.stamp = now
        if bool(case.get("object_enabled", True)):
            objects.objects = [make_object(case, object_id)]

        self.objects_pub.publish(objects)
        self.odom_pub.publish(odom)
        self.trajectory_pub.publish(trajectory)

    def run_case(self, case, object_id):
        self.latest_output = None
        self.expected_stamps = set()
        start = time.time()
        rate = rospy.Rate(self.args.publish_hz)
        while not rospy.is_shutdown() and time.time() - start < self.args.case_timeout:
            self.publish_case_inputs(case, object_id)
            rate.sleep()

        summary = summarize_path(
            self.latest_output,
            zero_epsilon=self.args.zero_epsilon,
            stop_velocity_epsilon=self.args.stop_velocity_epsilon,
        )
        expected_stop_x = case.get("expected_stop_x")
        tolerance = float(case.get("tolerance_m", self.args.tolerance_m))
        if expected_stop_x is None:
            passed = summary["received_output"] and summary["stop_x"] is None
        elif summary["stop_x"] is None:
            passed = False
        else:
            passed = abs(float(summary["stop_x"]) - float(expected_stop_x)) <= tolerance

        result = {
            "case": case.get("name", "unnamed"),
            "expected_stop_x": expected_stop_x,
            "tolerance_m": tolerance,
            "passed": passed,
            "object_x": case.get("object_x"),
            "object_y": case.get("object_y"),
            "object_speed": case.get("object_speed"),
        }
        result.update(summary)
        if self.latest_output is not None:
            stations = cumulative_stations(self.latest_output.points)
            result["_profile_points"] = [
                {
                    "point_index": index,
                    "station_m": stations[index],
                    "position_x_m": point.pose.position.x,
                    "position_y_m": point.pose.position.y,
                    "position_z_m": point.pose.position.z,
                    "longitudinal_velocity_mps": point.longitudinal_velocity_mps,
                }
                for index, point in enumerate(self.latest_output.points)
            ]
        return result

    def run(self):
        results = []
        for idx, case in enumerate(self.cases, start=1):
            if rospy.is_shutdown():
                break
            result = self.run_case(case, idx)
            results.append(result)
            rospy.loginfo(
                "MVP validation case=%s expected_stop_x=%s observed_stop_x=%s passed=%s",
                result["case"],
                result["expected_stop_x"],
                result["stop_x"],
                result["passed"],
            )
        return results


def load_cases(path):
    with open(path, "r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    cases = payload.get("cases", payload)
    if not isinstance(cases, list):
        raise ValueError("case file must contain a top-level 'cases' list")
    return cases


def default_case_file():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("motion_velocity_planner")
    return os.path.join(pkg_path, "config", "validation", "mvp_cases.yaml")


def ensure_output_dir(path):
    os.makedirs(path, exist_ok=True)
    return path


def write_results(results, output_dir):
    ensure_output_dir(output_dir)
    json_path = os.path.join(output_dir, "mvp_validation_results.json")
    csv_path = os.path.join(output_dir, "mvp_validation_results.csv")
    profiles_path = os.path.join(output_dir, "mvp_validation_profiles.csv")

    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(
            [
                {key: value for key, value in result.items() if not key.startswith("_")}
                for result in results
            ],
            stream,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")

    fields = [
        "case",
        "passed",
        "expected_stop_x",
        "stop_x",
        "first_slow_x",
        "min_velocity",
        "max_velocity",
        "zero_count",
        "object_x",
        "object_y",
        "object_speed",
        "tolerance_m",
    ]
    with open(csv_path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            writer.writerow({field: result.get(field) for field in fields})

    profile_fields = [
        "case",
        "point_index",
        "station_m",
        "position_x_m",
        "position_y_m",
        "position_z_m",
        "longitudinal_velocity_mps",
    ]
    with open(profiles_path, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=profile_fields)
        writer.writeheader()
        for result in results:
            for point in result.get("_profile_points", []):
                row = {"case": result["case"]}
                row.update(point)
                writer.writerow(row)

    return json_path, csv_path, profiles_path


def parse_args():
    parser = argparse.ArgumentParser(description="Run motion_velocity_planner validation cases.")
    parser.add_argument("--case-file", default=None, help="YAML file with validation cases.")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(
            os.path.expanduser("~"),
            "AJOUNICE2026",
            "local_autoware_planning_sim",
            "results",
            "motion_velocity_planner",
        ),
    )
    parser.add_argument(
        "--input-trajectory-topic",
        "--input-path-topic",
        dest="input_trajectory_topic",
        default=DEFAULT_INPUT_TRAJECTORY_TOPIC,
    )
    parser.add_argument("--objects-topic", default=DEFAULT_OBJECTS_TOPIC)
    parser.add_argument("--odometry-topic", default=DEFAULT_ODOMETRY_TOPIC)
    parser.add_argument(
        "--output-trajectory-topic",
        "--output-path-topic",
        dest="output_trajectory_topic",
        default=DEFAULT_OUTPUT_TRAJECTORY_TOPIC,
    )
    parser.add_argument("--path-points", type=int, default=121)
    parser.add_argument("--path-spacing", type=float, default=1.0)
    parser.add_argument("--path-speed", type=float, default=8.0)
    parser.add_argument("--ego-speed", type=float, default=5.0)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--case-timeout", type=float, default=1.0)
    parser.add_argument("--tolerance-m", type=float, default=0.25)
    parser.add_argument("--zero-epsilon", type=float, default=1e-3)
    parser.add_argument("--stop-velocity-epsilon", type=float, default=0.5)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    case_file = args.case_file or default_case_file()
    cases = load_cases(case_file)

    if not rosgraph.is_master_online():
        raise RuntimeError("ROS master is not running; start roscore or use the validation launch file")

    runner = MvpCaseRunner(args, cases)
    results = runner.run()
    json_path, csv_path, profiles_path = write_results(results, args.output_dir)

    failed = [result for result in results if not result["passed"]]
    print(
        json.dumps(
            [
                {key: value for key, value in result.items() if not key.startswith("_")}
                for result in results
            ],
            indent=2,
            sort_keys=True,
        )
    )
    print("wrote:", json_path)
    print("wrote:", csv_path)
    print("wrote:", profiles_path)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
