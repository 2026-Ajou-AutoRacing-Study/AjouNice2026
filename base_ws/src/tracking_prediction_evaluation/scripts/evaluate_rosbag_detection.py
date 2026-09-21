#!/usr/bin/env python3
"""Evaluate canonical online 3D detections against MORAI rosbag GT."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_asset_catalog_bbox_center,
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
    parser.add_argument(
        "--gt-reference",
        choices=("bbox-center-calibrated-v1", "asset-catalog-bbox-center-v1"),
        default="bbox-center-calibrated-v1",
        help=(
            "GT geometry contract. The default preserves published legacy "
            "metrics; asset-catalog-bbox-center-v1 uses audited per-asset boxes."
        ),
    )
    parser.add_argument(
        "--asset-bbox-catalog",
        type=Path,
        default=Path(
            "dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json"
        ),
    )
    parser.add_argument(
        "--gt-box-override",
        type=Path,
        help=(
            "optional JSON class-level ObjectStatus pivot/size override; "
            "intended for controlled bags containing one audited asset per class"
        ),
    )
    return parser.parse_args()


def apply_gt_box_override(segments, path):
    """Apply explicit asset geometry to reconstructed ObjectStatus tracks.

    The override is deliberately class-level and opt-in.  It is suitable for
    controlled evaluation bags such as the official sample CargoBox bag, where
    the obstacle class contains one known asset but ObjectStatus.name is empty.
    """
    if path is None:
        return tuple(segments), None
    contract = json.loads(path.read_text(encoding="utf-8"))
    profiles = {
        str(row["class_name"]): row for row in contract.get("profiles", [])
    }
    transformed = []
    applied_segments = 0
    for segment in segments:
        profile = profiles.get(segment.class_name)
        if profile is None:
            transformed.append(segment)
            continue
        offset = [float(value) for value in profile["pivot_to_bbox_center_local_m"]]
        size = [float(value) for value in profile["bbox_size_m"]]
        if len(offset) != 3 or len(size) != 3 or any(value <= 0.0 for value in size):
            raise ValueError("invalid GT box override profile for {}".format(segment.class_name))
        points = []
        for point in segment.points:
            cosine = math.cos(point.yaw)
            sine = math.sin(point.yaw)
            points.append(
                replace(
                    point,
                    x=point.x + cosine * offset[0] - sine * offset[1],
                    y=point.y + sine * offset[0] + cosine * offset[1],
                    z=point.z + offset[2],
                    length=size[0],
                    width=size[1],
                    height=size[2],
                )
            )
        transformed.append(replace(segment, points=tuple(points)))
        applied_segments += 1
    report = {
        "path": str(path.resolve()),
        "schema_version": contract.get("schema_version"),
        "description": contract.get("description"),
        "profiles": contract.get("profiles", []),
        "applied_segments": applied_segments,
    }
    return tuple(transformed), report


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
    if (
        args.gt_reference == "asset-catalog-bbox-center-v1"
        and args.gt_box_override is not None
    ):
        raise ValueError(
            "--gt-box-override cannot be combined with asset catalog GT; "
            "use the override with the legacy GT reference"
        )
    if args.gt_reference == "asset-catalog-bbox-center-v1":
        calibrated_segments, calibration = (
            transform_segments_to_asset_catalog_bbox_center(
                raw_segments, args.asset_bbox_catalog
            )
        )
    else:
        calibrated_segments, calibration = (
            transform_segments_to_calibrated_bbox_center(raw_segments)
        )
    if not calibrated_segments:
        raise RuntimeError(
            "{} found no supported GT trajectory segments".format(
                args.gt_reference
            )
        )
    calibrated_segments, gt_box_override = apply_gt_box_override(
        calibrated_segments, args.gt_box_override
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
            "gt_reference": args.gt_reference,
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
        "gt_box_override": gt_box_override,
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
            "asset_bbox_catalog": (
                str(args.asset_bbox_catalog.resolve())
                if args.gt_reference == "asset-catalog-bbox-center-v1"
                else None
            ),
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
