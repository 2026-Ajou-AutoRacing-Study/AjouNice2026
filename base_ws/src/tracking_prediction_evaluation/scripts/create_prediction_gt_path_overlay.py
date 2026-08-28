#!/usr/bin/env python3
"""Create an RViz marker bag containing future GT paths matched to predictions."""

from __future__ import annotations

import argparse
import copy
import json
import math
from collections import Counter
from pathlib import Path

import rosbag
import rospy
from autoware_perception_msgs.msg import ObjectClassification
from geometry_msgs.msg import Point
from scipy.optimize import linear_sum_assignment
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.pipeline_evaluator import (
    VEHICLE_LABELS,
    dominant_label,
    message_timestamp,
)
from tracking_prediction_evaluation.static_transform import (
    load_morai_to_map_transform,
)
from tracking_prediction_evaluation.trajectory import (
    interpolate_point,
    read_morai_trajectories,
    transform_segments_planar,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("prediction_bag", type=Path)
    parser.add_argument("--output-bag", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument(
        "--prediction-topic", default="/perception/object_recognition/objects"
    )
    parser.add_argument(
        "--marker-topic", default="/evaluation/visualization/gt_paths"
    )
    parser.add_argument("--horizon-sec", type=float, default=3.0)
    parser.add_argument("--time-step-sec", type=float, default=0.1)
    parser.add_argument("--maximum-association-distance-m", type=float, default=2.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def class_name_from_label(label):
    if label in VEHICLE_LABELS:
        return "vehicle"
    if label == ObjectClassification.PEDESTRIAN:
        return "pedestrian"
    if label == ObjectClassification.UNKNOWN:
        return "obstacle"
    return None


def prediction_observations(message):
    observations = []
    for obj in message.objects:
        class_name = class_name_from_label(dominant_label(obj))
        if class_name is None or not obj.kinematics.predicted_paths:
            continue
        pose = obj.kinematics.initial_pose_with_covariance.pose
        observations.append(
            (class_name, float(pose.position.x), float(pose.position.y))
        )
    return observations


def active_gt_by_class(segments, timestamp, bracket_sec):
    output = {"vehicle": [], "pedestrian": [], "obstacle": []}
    for segment in segments:
        point = interpolate_point(segment.points, timestamp, bracket_sec)
        if point is not None:
            output.setdefault(segment.class_name, []).append((segment, point))
    return output


def associate(gt_items, observations, maximum_distance_m):
    if not gt_items or not observations:
        return []
    distances = []
    for _, point in gt_items:
        distances.append(
            [math.hypot(point.x - x, point.y - y) for _, x, y in observations]
        )
    rows, columns = linear_sum_assignment(distances)
    matches = []
    for row, column in zip(rows, columns):
        distance = float(distances[int(row)][int(column)])
        if distance <= maximum_distance_m:
            matches.append((int(row), int(column), distance))
    return matches


def sample_future(segment, timestamp, horizon_sec, step_sec, bracket_sec):
    sample_count = int(math.floor(horizon_sec / step_sec + 1.0e-9))
    points = []
    for index in range(sample_count + 1):
        point = interpolate_point(
            segment.points, timestamp + index * step_sec, bracket_sec
        )
        if point is None:
            break
        points.append(point)
    return points


def marker_header(source_header, stamp):
    header = copy.deepcopy(source_header)
    header.frame_id = "map"
    header.stamp = stamp
    return header


def append_path_markers(output, header, marker_id, points, lifetime):
    purple = ColorRGBA(0.72, 0.10, 1.0, 1.0)
    line = Marker()
    line.header = copy.deepcopy(header)
    line.ns = "matched_gt_future_paths"
    line.id = marker_id
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.pose.orientation.w = 1.0
    line.scale.x = 0.10
    line.color = purple
    line.lifetime = lifetime
    line.points = [Point(x=p.x, y=p.y, z=p.z + 0.12) for p in points]
    output.markers.append(line)

    for horizon_index, horizon in enumerate((1.0, 2.0, 3.0), start=1):
        target = min(points, key=lambda p: abs(p.timestamp - (points[0].timestamp + horizon)))
        if abs(target.timestamp - (points[0].timestamp + horizon)) > 0.06:
            continue
        sphere = Marker()
        sphere.header = copy.deepcopy(header)
        sphere.ns = "matched_gt_horizons"
        sphere.id = marker_id * 10 + horizon_index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=target.x, y=target.y, z=target.z + 0.12)
        sphere.pose.orientation.w = 1.0
        diameter = 0.48 + 0.08 * (horizon_index - 1)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = diameter
        sphere.color = purple
        sphere.lifetime = lifetime
        output.markers.append(sphere)


def main():
    args = parse_args()
    if args.horizon_sec <= 0.0 or args.time_step_sec <= 0.0:
        raise ValueError("horizon and time step must be positive")
    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    object_status_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle", "pedestrian", "obstacle"),
        minimum_points=2,
    )
    calibrated_segments, calibration = transform_segments_to_calibrated_bbox_center(
        object_status_segments, exclude_unsupported=False
    )
    segments = transform_segments_planar(
        calibrated_segments, translation_xyz, transform_yaw
    )

    args.output_bag.parent.mkdir(parents=True, exist_ok=True)
    counts = Counter()
    lifetime = rospy.Duration(0.5)
    with rosbag.Bag(str(args.prediction_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w", compression=rosbag.Compression.LZ4
    ) as destination:
        for _, message, bag_stamp in source.read_messages(
            topics=[args.prediction_topic]
        ):
            counts["prediction_frames"] += 1
            timestamp = message_timestamp(message, bag_stamp)
            stamp = rospy.Time.from_sec(timestamp)
            header = marker_header(message.header, stamp)
            markers = MarkerArray()
            clear = Marker()
            clear.header = copy.deepcopy(header)
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            observations = prediction_observations(message)
            counts["prediction_objects_with_paths"] += len(observations)
            active = active_gt_by_class(
                segments, timestamp, args.maximum_interpolation_bracket_sec
            )
            marker_id = 0
            for class_name in ("vehicle", "pedestrian", "obstacle"):
                class_observations = [
                    item for item in observations if item[0] == class_name
                ]
                matches = associate(
                    active.get(class_name, []),
                    class_observations,
                    args.maximum_association_distance_m,
                )
                counts["matched_{}_objects".format(class_name)] += len(matches)
                for gt_index, _, _ in matches:
                    segment = active[class_name][gt_index][0]
                    future = sample_future(
                        segment,
                        timestamp,
                        args.horizon_sec,
                        args.time_step_sec,
                        args.maximum_interpolation_bracket_sec,
                    )
                    if len(future) < 2:
                        counts["insufficient_future"] += 1
                        continue
                    append_path_markers(markers, header, marker_id, future, lifetime)
                    marker_id += 1
                    counts["published_gt_paths"] += 1
            destination.write(args.marker_topic, markers, stamp)

    report = {
        "schema_version": 1,
        "raw_bag": str(args.raw_bag.resolve()),
        "prediction_bag": str(args.prediction_bag.resolve()),
        "output_bag": str(args.output_bag.resolve()),
        "marker_topic": args.marker_topic,
        "horizon_sec": args.horizon_sec,
        "time_step_sec": args.time_step_sec,
        "maximum_association_distance_m": args.maximum_association_distance_m,
        "counts": dict(counts),
        "bbox_reference_calibration": calibration,
    }
    report_path = args.report or args.output_bag.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("GT future overlay bag: {}".format(args.output_bag))
    print("published GT paths: {}".format(counts["published_gt_paths"]))
    print("report: {}".format(report_path))


if __name__ == "__main__":
    main()
