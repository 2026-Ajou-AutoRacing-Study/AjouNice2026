#!/usr/bin/env python3
"""Audit a static-obstacle benchmark rosbag per scenario asset.

This is a diagnostic companion to ``evaluate_rosbag_detection.py``.  It keeps
the production timestamp/coordinate contract, but maps each static MORAI GT
track back to the named actor in a scenario audit file and reports visibility,
recall and nearby class confusion per asset.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.detection_evaluator import (
    build_evaluation_frames,
    read_detection_frames,
)
from tracking_prediction_evaluation.pipeline_evaluator import (
    interpolate_ego,
    message_timestamp,
    read_ego_points,
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
    parser.add_argument("detection_bag", type=Path)
    parser.add_argument("scenario_audit", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--score-threshold", type=float, default=0.10)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--minimum-observable-points", type=int, default=5)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--maximum-lidar-time-difference-sec", type=float, default=0.10)
    parser.add_argument("--point-count-box-tolerance-m", type=float, default=0.03)
    parser.add_argument("--sensor-to-ego-x-m", type=float, default=0.855)
    parser.add_argument("--sensor-to-ego-y-m", type=float, default=0.0)
    parser.add_argument("--sensor-to-ego-z-m", type=float, default=1.388)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def nearest(items, timestamps, timestamp, maximum_difference):
    right = bisect.bisect_left(timestamps, timestamp)
    candidates = range(max(0, right - 2), min(len(items), right + 2))
    if not candidates:
        return None
    index = min(candidates, key=lambda value: abs(timestamps[value] - timestamp))
    if abs(timestamps[index] - timestamp) > maximum_difference:
        return None
    return items[index]


def cloud_xyz(message):
    from sensor_msgs import point_cloud2

    values = list(
        point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
    )
    return np.asarray(values, dtype=np.float32).reshape((-1, 3))


def count_points(points, gt, ego, ego_z, args):
    cosine = math.cos(ego.yaw)
    sine = math.sin(ego.yaw)
    dx_map = float(gt.x) - float(ego.x)
    dy_map = float(gt.y) - float(ego.y)
    center_x = cosine * dx_map + sine * dy_map - args.sensor_to_ego_x_m
    center_y = -sine * dx_map + cosine * dy_map - args.sensor_to_ego_y_m
    center_z = (
        float(gt.z)
        + 0.5 * float(gt.height)
        - float(ego_z)
        - args.sensor_to_ego_z_m
    )
    yaw = float(gt.yaw) - float(ego.yaw)
    relative = points - np.asarray((center_x, center_y, center_z), dtype=np.float32)
    box_cosine = math.cos(yaw)
    box_sine = math.sin(yaw)
    local_x = box_cosine * relative[:, 0] + box_sine * relative[:, 1]
    local_y = -box_sine * relative[:, 0] + box_cosine * relative[:, 1]
    tolerance = args.point_count_box_tolerance_m
    return int(
        np.count_nonzero(
            (np.abs(local_x) <= 0.5 * float(gt.length) + tolerance)
            & (np.abs(local_y) <= 0.5 * float(gt.width) + tolerance)
            & (np.abs(relative[:, 2]) <= 0.5 * float(gt.height) + tolerance)
        )
    )


def percentile(values, q):
    return float(np.percentile(values, q)) if values else None


def main():
    args = parse_args()
    translation, transform_yaw = load_morai_to_map_transform(args.morai_static_tf)
    raw_segments = read_morai_trajectories(
        args.raw_bag, include_classes=("obstacle",)
    )
    calibrated_segments, _ = transform_segments_to_calibrated_bbox_center(raw_segments)
    segments = transform_segments_planar(calibrated_segments, translation, transform_yaw)
    ego_points = read_ego_points(
        args.raw_bag, "/Ego_topic", translation, transform_yaw
    )
    detections = read_detection_frames(
        args.detection_bag, "/perception/object_recognition/detection/objects"
    )
    frames, frame_counts = build_evaluation_frames(
        segments,
        ego_points,
        detections,
        maximum_range_m=args.maximum_range_m,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
        sensor_to_ego_x_m=args.sensor_to_ego_x_m,
        sensor_to_ego_y_m=args.sensor_to_ego_y_m,
        sensor_to_ego_z_m=args.sensor_to_ego_z_m,
    )

    audit = json.loads(args.scenario_audit.read_text(encoding="utf-8"))
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    assets = []
    for item in audit["objects"]:
        x = float(item["position"]["x"])
        y = float(item["position"]["y"])
        assets.append(
            {
                "name": item["name"],
                "data_id": int(item["data_id"]),
                "x": cosine * x - sine * y + translation[0],
                "y": sine * x + cosine * y + translation[1],
            }
        )

    import rosbag

    lidar = []
    raw_heights = []
    with rosbag.Bag(str(args.raw_bag), "r") as bag:
        for topic, message, bag_time in bag.read_messages(
            topics=["/sensor/lidar1", "/Ego_topic"]
        ):
            timestamp = message_timestamp(message, bag_time)
            if topic == "/sensor/lidar1":
                lidar.append((timestamp, message))
            else:
                raw_heights.append(
                    (timestamp, float(message.position.z) + float(translation[2]))
                )
    lidar_times = [item[0] for item in lidar]
    height_times = [item[0] for item in raw_heights]

    rows = defaultdict(
        lambda: {
            "gt_frames": 0,
            "valid_box_frames": 0,
            "zero_point_frames": 0,
            "sparse_1to4_frames": 0,
            "observable_5plus_frames": 0,
            "same_class_tp_1m": 0,
            "same_class_tp_2m": 0,
            "observable_same_class_tp_1m": 0,
            "observable_same_class_tp_2m": 0,
            "nearby_prediction_classes_1m": Counter(),
            "matched_center_errors_2m": [],
            "matched_scores_2m": [],
            "point_counts": [],
            "ranges_m": [],
            "dimensions": [],
        }
    )

    for frame in frames:
        lidar_item = nearest(
            lidar,
            lidar_times,
            frame.timestamp,
            args.maximum_lidar_time_difference_sec,
        )
        for segment, gt in frame.gt:
            asset = min(
                assets,
                key=lambda item: math.hypot(gt.x - item["x"], gt.y - item["y"]),
            )
            row = rows[asset["name"]]
            row["data_id"] = asset["data_id"]
            row["gt_frames"] += 1
            row["ranges_m"].append(math.hypot(gt.x - frame.ego.x, gt.y - frame.ego.y))
            valid = gt.length > 0.0 and gt.width > 0.0 and gt.height > 0.0
            point_count = None
            if valid:
                row["valid_box_frames"] += 1
                row["dimensions"].append((gt.length, gt.width, gt.height))
                if lidar_item is not None:
                    lidar_timestamp, lidar_message = lidar_item
                    lidar_ego = interpolate_ego(
                        ego_points,
                        lidar_timestamp,
                        args.maximum_interpolation_bracket_sec,
                    )
                    lidar_gt = interpolate_point(
                        segment.points,
                        lidar_timestamp,
                        args.maximum_interpolation_bracket_sec,
                    )
                    height_item = nearest(
                        raw_heights,
                        height_times,
                        lidar_timestamp,
                        args.maximum_interpolation_bracket_sec,
                    )
                    if lidar_ego is not None and lidar_gt is not None and height_item:
                        point_count = count_points(
                            cloud_xyz(lidar_message),
                            lidar_gt,
                            lidar_ego,
                            height_item[1],
                            args,
                        )
                        row["point_counts"].append(point_count)
                        if point_count == 0:
                            row["zero_point_frames"] += 1
                        elif point_count < args.minimum_observable_points:
                            row["sparse_1to4_frames"] += 1
                        else:
                            row["observable_5plus_frames"] += 1

            candidates = []
            for prediction in frame.predictions:
                if prediction.score < args.score_threshold:
                    continue
                distance = math.hypot(prediction.x - gt.x, prediction.y - gt.y)
                candidates.append((distance, prediction))
            if candidates:
                distance, prediction = min(candidates, key=lambda item: item[0])
                if distance < 1.0:
                    row["nearby_prediction_classes_1m"][prediction.class_name] += 1

            obstacle_candidates = [
                (distance, prediction)
                for distance, prediction in candidates
                if prediction.class_name == "obstacle"
            ]
            if obstacle_candidates:
                distance, prediction = min(obstacle_candidates, key=lambda item: item[0])
                if distance < 1.0:
                    row["same_class_tp_1m"] += 1
                    if point_count is not None and point_count >= args.minimum_observable_points:
                        row["observable_same_class_tp_1m"] += 1
                if distance < 2.0:
                    row["same_class_tp_2m"] += 1
                    row["matched_center_errors_2m"].append(distance)
                    row["matched_scores_2m"].append(prediction.score)
                    if point_count is not None and point_count >= args.minimum_observable_points:
                        row["observable_same_class_tp_2m"] += 1

    output_rows = []
    for asset in assets:
        row = rows[asset["name"]]
        gt = row["gt_frames"]
        observable = row["observable_5plus_frames"]
        dimensions = row.pop("dimensions")
        point_counts = row.pop("point_counts")
        ranges = row.pop("ranges_m")
        errors = row.pop("matched_center_errors_2m")
        scores = row.pop("matched_scores_2m")
        row["nearby_prediction_classes_1m"] = dict(
            row["nearby_prediction_classes_1m"]
        )
        row["recall_1m"] = row["same_class_tp_1m"] / gt if gt else None
        row["recall_2m"] = row["same_class_tp_2m"] / gt if gt else None
        row["observable_recall_1m"] = (
            row["observable_same_class_tp_1m"] / observable if observable else None
        )
        row["observable_recall_2m"] = (
            row["observable_same_class_tp_2m"] / observable if observable else None
        )
        row["range_p50_m"] = percentile(ranges, 50)
        row["range_p90_m"] = percentile(ranges, 90)
        row["point_count_p50"] = percentile(point_counts, 50)
        row["center_error_p50_m"] = percentile(errors, 50)
        row["matched_score_p50"] = percentile(scores, 50)
        row["median_dimensions_lwh_m"] = (
            [float(value) for value in np.median(dimensions, axis=0)]
            if dimensions
            else None
        )
        output_rows.append({"asset": asset["name"], **row})

    report = {
        "protocol": {
            "name": "morai-static-obstacle-asset-audit-v1",
            "score_threshold": args.score_threshold,
            "maximum_range_m": args.maximum_range_m,
            "minimum_observable_points": args.minimum_observable_points,
            "note": "Per-GT diagnostic recall; use the standard evaluator for global precision/AP.",
        },
        "alignment": frame_counts,
        "assets": output_rows,
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "detection_bag": str(args.detection_bag.resolve()),
            "scenario_audit": str(args.scenario_audit.resolve()),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
