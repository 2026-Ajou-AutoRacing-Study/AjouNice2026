#!/usr/bin/env python3
"""Evaluate online detections with Base3-compatible LiDAR visibility slices."""

from __future__ import annotations

import argparse
import bisect
import json
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.detection_evaluator import (
    EvaluationFrame,
    build_evaluation_frames,
    evaluate_detection_frames,
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


@dataclass(frozen=True)
class EgoHeight:
    timestamp: float
    z: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("detection_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--lidar-topic", default="/sensor/lidar1")
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--maximum-lidar-time-difference-sec", type=float, default=0.10)
    parser.add_argument("--minimum-observable-points", type=int, default=5)
    parser.add_argument("--point-count-box-tolerance-m", type=float, default=0.03)
    parser.add_argument("--production-score-threshold", type=float, default=0.10)
    parser.add_argument("--sensor-to-ego-x-m", type=float, default=0.855)
    parser.add_argument("--sensor-to-ego-y-m", type=float, default=0.0)
    parser.add_argument("--sensor-to-ego-z-m", type=float, default=1.388)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def read_lidar_and_ego_height(args, translation_z):
    import rosbag

    lidar = []
    ego_height_by_timestamp = {}
    with rosbag.Bag(str(args.raw_bag), "r") as bag:
        for topic, message, bag_time in bag.read_messages(
            topics=[args.lidar_topic, args.ego_topic]
        ):
            timestamp = message_timestamp(message, bag_time)
            if topic == args.lidar_topic:
                lidar.append((timestamp, message))
            else:
                ego_height_by_timestamp[timestamp] = EgoHeight(
                    timestamp=timestamp,
                    z=float(message.position.z) + float(translation_z),
                )
    return tuple(lidar), tuple(
        ego_height_by_timestamp[key] for key in sorted(ego_height_by_timestamp)
    )


def interpolate_height(points, timestamp, maximum_gap):
    if not points:
        return None
    times = [point.timestamp for point in points]
    right = bisect.bisect_left(times, timestamp)
    if right < len(points) and abs(points[right].timestamp - timestamp) < 1.0e-9:
        return points[right].z
    if right == 0 or right >= len(points):
        return None
    before, after = points[right - 1], points[right]
    dt = after.timestamp - before.timestamp
    if dt <= 0.0 or dt > maximum_gap:
        return None
    ratio = (timestamp - before.timestamp) / dt
    return before.z + ratio * (after.z - before.z)


def nearest_lidar(lidar, lidar_times, timestamp, maximum_difference):
    right = bisect.bisect_left(lidar_times, timestamp)
    candidates = range(max(0, right - 2), min(len(lidar), right + 2))
    if not candidates:
        return None
    index = min(candidates, key=lambda value: abs(lidar_times[value] - timestamp))
    difference = abs(lidar_times[index] - timestamp)
    return (lidar[index], difference) if difference <= maximum_difference else None


def cloud_xyz(message):
    from sensor_msgs import point_cloud2

    values = list(
        point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
    )
    return np.asarray(values, dtype=np.float32).reshape((-1, 3))


def count_points(points, gt, ego, ego_z, args):
    """Count points in the unmodified GT box with Base3's 3 cm tolerance."""
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


def filtered_frame(frame, keep):
    return EvaluationFrame(
        timestamp=frame.timestamp,
        ego=frame.ego,
        gt=tuple(item for item, accepted in zip(frame.gt, keep) if accepted),
        predictions=frame.predictions,
    )


def filtered_frame_ignoring_excluded(frame, keep, ignore_distance_m=1.0):
    """Filter GT and suppress predictions that match only an excluded GT."""
    kept_gt = tuple(item for item, accepted in zip(frame.gt, keep) if accepted)
    ignored_gt = tuple(item for item, accepted in zip(frame.gt, keep) if not accepted)
    predictions = []
    ignored_predictions = 0
    for prediction in frame.predictions:
        kept_distance = min(
            (
                math.hypot(prediction.x - gt.x, prediction.y - gt.y)
                for segment, gt in kept_gt
                if segment.class_name == prediction.class_name
            ),
            default=math.inf,
        )
        ignored_distance = min(
            (
                math.hypot(prediction.x - gt.x, prediction.y - gt.y)
                for segment, gt in ignored_gt
                if segment.class_name == prediction.class_name
            ),
            default=math.inf,
        )
        if ignored_distance <= ignore_distance_m and ignored_distance < kept_distance:
            ignored_predictions += 1
            continue
        predictions.append(prediction)
    return (
        EvaluationFrame(frame.timestamp, frame.ego, kept_gt, tuple(predictions)),
        ignored_predictions,
    )


def compact_metrics(metrics):
    output = {"mean_ap": metrics["overall"]["mean_ap"], "classes": {}}
    for name, value in metrics["overall"]["classes"].items():
        operating = value["production_operating_point_1m"]
        output["classes"][name] = {
            "gt": value["gt"],
            "predictions": value["predictions"],
            "mean_ap": value["mean_ap"],
            "tp_1m": operating["tp"],
            "fp_1m": operating["fp"],
            "fn_1m": operating["fn"],
            "precision_1m": operating["precision"],
            "recall_1m": operating["recall"],
            "f1_1m": operating["f1"],
            "mATE_2m": value["tp_quality_2m"]["mATE_m"],
        }
    return output


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(args.morai_static_tf)
    raw_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle", "pedestrian", "obstacle"),
    )
    calibrated_segments, calibration = transform_segments_to_calibrated_bbox_center(
        raw_segments
    )
    segments = transform_segments_planar(
        calibrated_segments, translation_xyz, transform_yaw
    )
    ego_points = read_ego_points(
        args.raw_bag, args.ego_topic, translation_xyz, transform_yaw
    )
    detections = read_detection_frames(args.detection_bag, args.detection_topic)
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
    lidar, ego_heights = read_lidar_and_ego_height(args, translation_xyz[2])
    lidar_times = tuple(item[0] for item in lidar)

    slice_names = ("valid", "observable_5plus", "sparse_1to4", "zero_point")
    slice_frames = {name: [] for name in slice_names}
    ignored_slice_frames = {name: [] for name in slice_names}
    ignored_prediction_counts = Counter()
    point_bins = defaultdict(Counter)
    invalid = Counter()
    lidar_time_differences = []
    missing_lidar_frames = 0
    missing_ego_height_frames = 0

    for frame in frames:
        lidar_match = nearest_lidar(
            lidar,
            lidar_times,
            frame.timestamp,
            args.maximum_lidar_time_difference_sec,
        )
        ego_z = interpolate_height(
            ego_heights, frame.timestamp, args.maximum_interpolation_bracket_sec
        )
        if lidar_match is None:
            missing_lidar_frames += 1
            counts = [None] * len(frame.gt)
        elif ego_z is None:
            missing_ego_height_frames += 1
            counts = [None] * len(frame.gt)
        else:
            (lidar_timestamp, cloud), difference = lidar_match
            lidar_time_differences.append(float(difference))
            points = cloud_xyz(cloud)
            lidar_ego = interpolate_ego(
                ego_points,
                lidar_timestamp,
                args.maximum_interpolation_bracket_sec,
            )
            lidar_ego_z = interpolate_height(
                ego_heights,
                lidar_timestamp,
                args.maximum_interpolation_bracket_sec,
            )
            counts = []
            for segment, gt in frame.gt:
                lidar_gt = interpolate_point(
                    segment.points,
                    lidar_timestamp,
                    args.maximum_interpolation_bracket_sec,
                )
                valid = (
                    lidar_gt is not None
                    and lidar_ego is not None
                    and lidar_ego_z is not None
                    and lidar_gt.length > 0.0
                    and lidar_gt.width > 0.0
                    and lidar_gt.height > 0.0
                )
                if not valid:
                    invalid[segment.class_name] += 1
                    counts.append(None)
                    continue
                value = count_points(
                    points, lidar_gt, lidar_ego, lidar_ego_z, args
                )
                counts.append(value)
                if value == 0:
                    point_bins[segment.class_name]["0"] += 1
                elif value < args.minimum_observable_points:
                    point_bins[segment.class_name]["1-4"] += 1
                else:
                    point_bins[segment.class_name]["5+"] += 1

        valid_keep = [value is not None for value in counts]
        observable_keep = [
            value is not None and value >= args.minimum_observable_points
            for value in counts
        ]
        sparse_keep = [
            value is not None and 0 < value < args.minimum_observable_points
            for value in counts
        ]
        zero_keep = [value == 0 for value in counts]
        for name, keep in zip(
            slice_names, (valid_keep, observable_keep, sparse_keep, zero_keep)
        ):
            slice_frames[name].append(filtered_frame(frame, keep))
            ignored_frame, ignored_count = filtered_frame_ignoring_excluded(
                frame, keep
            )
            ignored_slice_frames[name].append(ignored_frame)
            ignored_prediction_counts[name] += ignored_count

    metrics = {
        "global": evaluate_detection_frames(
            frames,
            production_score_threshold=args.production_score_threshold,
        )
    }
    for name in slice_names:
        metrics[name] = evaluate_detection_frames(
            tuple(slice_frames[name]),
            production_score_threshold=args.production_score_threshold,
        )
        metrics[name + "_ignore_excluded"] = evaluate_detection_frames(
            tuple(ignored_slice_frames[name]),
            production_score_threshold=args.production_score_threshold,
        )

    difference_array = np.asarray(lidar_time_differences, dtype=np.float64)
    report = {
        "protocol": {
            "name": "morai-online-lidar-visibility-v1",
            "maximum_range_m": args.maximum_range_m,
            "minimum_observable_points": args.minimum_observable_points,
            "point_count_box_tolerance_m": args.point_count_box_tolerance_m,
            "production_score_threshold": args.production_score_threshold,
            "observable_note": (
                "Predictions associated with excluded sparse GT are retained as false "
                "positives. This is a conservative counterpart to Base3 filtered labels."
            ),
            "ignore_excluded_note": (
                "The *_ignore_excluded slices suppress same-class predictions within "
                "1 m of an excluded GT when no retained GT is closer. Use these for "
                "slice precision/AP and the strict slices for conservative accounting."
            ),
        },
        "alignment": {
            **frame_counts,
            "raw_lidar_frames": len(lidar),
            "missing_lidar_frames": missing_lidar_frames,
            "missing_ego_height_frames": missing_ego_height_frames,
            "ignored_predictions_by_slice": dict(ignored_prediction_counts),
            "lidar_time_difference_abs_sec": {
                "count": int(len(difference_array)),
                "p50": float(np.percentile(difference_array, 50)) if len(difference_array) else None,
                "p95": float(np.percentile(difference_array, 95)) if len(difference_array) else None,
                "maximum": float(np.max(difference_array)) if len(difference_array) else None,
            },
        },
        "gt_visibility_counts": {
            name: {
                "zero_point": int(point_bins[name]["0"]),
                "sparse_1to4": int(point_bins[name]["1-4"]),
                "observable_5plus": int(point_bins[name]["5+"]),
                "invalid_box": int(invalid[name]),
            }
            for name in ("vehicle", "pedestrian", "obstacle")
        },
        "metrics": metrics,
        "compact": {name: compact_metrics(value) for name, value in metrics.items()},
        "gt_reference_calibration": calibration,
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "detection_bag": str(args.detection_bag.resolve()),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output),
        "alignment": report["alignment"],
        "gt_visibility_counts": report["gt_visibility_counts"],
        "compact": report["compact"],
    }, indent=2))


if __name__ == "__main__":
    main()
