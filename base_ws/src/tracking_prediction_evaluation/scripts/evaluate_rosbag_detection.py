#!/usr/bin/env python3
"""Evaluate canonical online 3D detections against MORAI rosbag GT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.detection_evaluator import (
    build_evaluation_frames,
    evaluate_detection_frames,
    read_detection_frames,
)
from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points
from tracking_prediction_evaluation.static_transform import (
    load_morai_to_map_transform,
)
from tracking_prediction_evaluation.trajectory import (
    read_morai_trajectories,
    transform_segments_planar,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("detection_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument(
        "--classes",
        nargs="+",
        choices=("vehicle", "pedestrian", "obstacle"),
        default=("vehicle", "pedestrian", "obstacle"),
    )
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument(
        "--maximum-interpolation-bracket-sec", type=float, default=0.15
    )
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


def _bag_topic_count(path, topic):
    import rosbag

    with rosbag.Bag(str(path), "r") as bag:
        info = bag.get_type_and_topic_info().topics.get(topic)
        return int(info.message_count) if info is not None else 0


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    raw_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=tuple(args.classes),
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
    source_frames = read_detection_frames(args.detection_bag, args.detection_topic)
    frames, frame_counts = build_evaluation_frames(
        segments,
        ego_points,
        source_frames,
        maximum_range_m=args.maximum_range_m,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
        sensor_to_ego_x_m=args.sensor_to_ego_x_m,
        sensor_to_ego_y_m=args.sensor_to_ego_y_m,
        sensor_to_ego_z_m=args.sensor_to_ego_z_m,
    )
    metrics = evaluate_detection_frames(
        frames,
        class_names=tuple(args.classes),
        production_score_threshold=args.production_score_threshold,
    )
    lidar_frames = _bag_topic_count(args.raw_bag, "/sensor/lidar1")
    detection_message_count = _bag_topic_count(
        args.detection_bag, args.detection_topic
    )
    report = {
        "protocol": {
            "name": "morai-online-center-distance-v1",
            "center_thresholds_m": [0.5, 1.0, 2.0, 4.0],
            "bev_iou_thresholds": [0.3, 0.5],
            "distance_bins_m": [[0.0, 10.0], [10.0, 20.0], [20.0, 30.0]],
            "maximum_range_m": args.maximum_range_m,
            "production_score_threshold": args.production_score_threshold,
            "ap_min_recall": 0.1,
            "ap_min_precision": 0.1,
            "gt_reference": "bbox-center-calibrated-v1",
            "note": (
                "AP is conditioned on published detection frames. Runtime "
                "frame coverage is reported separately."
            ),
        },
        "runtime_coverage": {
            "raw_lidar_frames": lidar_frames,
            "detection_messages": detection_message_count,
            "message_to_lidar_ratio": (
                detection_message_count / lidar_frames if lidar_frames else None
            ),
            **frame_counts,
        },
        "gt_reference_calibration": calibration,
        "metrics": metrics,
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "detection_bag": str(args.detection_bag.resolve()),
            "object_topic": args.object_topic,
            "detection_topic": args.detection_topic,
            "trajectory_segment_count": len(segments),
            "detection_frame_count": len(source_frames),
            "evaluated_frame_count": len(frames),
            "morai_static_tf": str(args.morai_static_tf.resolve()),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    compact = {
        "output": str(args.output),
        "evaluated_frames": len(frames),
        "frame_coverage": report["runtime_coverage"]["message_to_lidar_ratio"],
        "mean_ap": metrics["overall"]["mean_ap"],
        "classes": {
            name: {
                "gt": value["gt"],
                "mean_ap": value["mean_ap"],
                "production_f1_1m": value["production_operating_point_1m"]["f1"],
            }
            for name, value in metrics["overall"]["classes"].items()
        },
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
