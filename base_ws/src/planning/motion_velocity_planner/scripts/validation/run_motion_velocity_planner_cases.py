#!/usr/bin/env python3

import argparse
import csv
import json
import math
import os
import sys
import time
from copy import deepcopy

import rospy
import yaml
import rosgraph
import rospkg

from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from autoware_internal_planning_msgs.msg import PathWithLaneId, PathPointWithLaneId
from autoware_planning_msgs.msg import PathPoint
from autoware_perception_msgs.msg import (
    ObjectClassification,
    PredictedObject,
    PredictedObjects,
    Shape,
)


DEFAULT_INPUT_PATH_TOPIC = "/behavior_path_planner/output/path"
DEFAULT_OBJECTS_TOPIC = "/perception/objects"
DEFAULT_ODOMETRY_TOPIC = "/odometry"
DEFAULT_OUTPUT_PATH_TOPIC = "/motion_velocity_planner/output/path"


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def make_path(num_points, spacing, default_speed):
    path = PathWithLaneId()
    path.header = Header(frame_id="map")
    for i in range(num_points):
        p = PathPoint()
        p.pose.position.x = float(i) * spacing
        p.pose.position.y = 0.0
        p.pose.orientation = quaternion_from_yaw(0.0)
        p.longitudinal_velocity_mps = float(default_speed)

        item = PathPointWithLaneId()
        item.point = deepcopy(p)
        item.lane_ids = [10]
        path.points.append(item)

    length = (num_points - 1) * spacing
    path.left_bound = [Point(x=0.0, y=2.5), Point(x=length, y=2.5)]
    path.right_bound = [Point(x=0.0, y=-2.5), Point(x=length, y=-2.5)]
    return path


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
    return float(path_point.point.pose.position.x)


def summarize_path(path_msg, zero_epsilon, stop_velocity_epsilon):
    if path_msg is None or not path_msg.points:
        return {
            "received_output": False,
            "stop_x": None,
            "min_velocity": None,
            "max_velocity": None,
            "zero_count": 0,
        }

    velocities = [float(p.point.longitudinal_velocity_mps) for p in path_msg.points]
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

        rospy.init_node("mvp_validation_runner", anonymous=True)

        self.path_pub = rospy.Publisher(args.input_path_topic, PathWithLaneId, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher(args.odometry_topic, Odometry, queue_size=1, latch=True)
        self.objects_pub = rospy.Publisher(args.objects_topic, PredictedObjects, queue_size=1, latch=True)
        self.output_sub = rospy.Subscriber(args.output_path_topic, PathWithLaneId, self.output_cb)

    def output_cb(self, msg):
        self.latest_output = msg

    def publish_case_inputs(self, case, object_id):
        now = rospy.Time.now()
        path = make_path(
            num_points=int(case.get("path_points", self.args.path_points)),
            spacing=float(case.get("path_spacing", self.args.path_spacing)),
            default_speed=float(case.get("path_speed", self.args.path_speed)),
        )
        odom = make_odometry(float(case.get("ego_speed", self.args.ego_speed)))
        objects = PredictedObjects()
        objects.header.frame_id = "map"

        path.header.stamp = now
        odom.header.stamp = now
        objects.header.stamp = now
        if bool(case.get("object_enabled", True)):
            objects.objects = [make_object(case, object_id)]

        self.objects_pub.publish(objects)
        self.odom_pub.publish(odom)
        self.path_pub.publish(path)

    def run_case(self, case, object_id):
        self.latest_output = None
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

    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, sort_keys=True)
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

    return json_path, csv_path


def parse_args():
    parser = argparse.ArgumentParser(description="Run motion_velocity_planner validation cases.")
    parser.add_argument("--case-file", default=None, help="YAML file with validation cases.")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(
            os.path.expanduser("~"),
            "AjouNice2026",
            "artifacts",
            "mvp_validation",
        ),
    )
    parser.add_argument("--input-path-topic", default=DEFAULT_INPUT_PATH_TOPIC)
    parser.add_argument("--objects-topic", default=DEFAULT_OBJECTS_TOPIC)
    parser.add_argument("--odometry-topic", default=DEFAULT_ODOMETRY_TOPIC)
    parser.add_argument("--output-path-topic", default=DEFAULT_OUTPUT_PATH_TOPIC)
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
    json_path, csv_path = write_results(results, args.output_dir)

    failed = [result for result in results if not result["passed"]]
    print(json.dumps(results, indent=2, sort_keys=True))
    print("wrote:", json_path)
    print("wrote:", csv_path)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
