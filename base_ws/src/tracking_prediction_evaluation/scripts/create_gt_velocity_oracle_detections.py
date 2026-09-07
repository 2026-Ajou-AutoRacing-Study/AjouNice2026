#!/usr/bin/env python3
"""Copy recorded detections and inject causally reconstructed MORAI GT velocity."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.evaluator import estimate_motion_state
from tracking_prediction_evaluation.pipeline_evaluator import (
    active_gt,
    interpolate_ego,
    match_estimates,
    message_timestamp,
    read_detection_frames,
    read_ego_points,
    transform_sensor_estimates_to_map,
)
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
    parser.add_argument("pipeline_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-association-distance-m", type=float, default=3.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--history-sec", type=float, default=0.5)
    parser.add_argument("--sensor-to-ego-x-m", type=float, default=0.855)
    parser.add_argument("--sensor-to-ego-y-m", type=float, default=0.0)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def causal_state(segment, origin, timestamp, history_sec):
    history = [
        point
        for point in segment.points
        if timestamp - history_sec <= point.timestamp <= timestamp + 1.0e-9
    ]
    if not history or abs(history[-1].timestamp - timestamp) > 1.0e-9:
        history.append(origin)
    return estimate_motion_state(history)


def main():
    args = parse_args()
    import rosbag

    translation_xyz, transform_yaw = load_morai_to_map_transform(args.morai_static_tf)
    raw_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle",),
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
    detection_frames = read_detection_frames(
        args.pipeline_bag, args.detection_topic
    )

    counters = {
        "frames": 0,
        "detections": 0,
        "matched": 0,
        "injected": 0,
        "missing_ego_frames": 0,
        "missing_motion_state": 0,
    }
    args.output_bag.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(args.pipeline_bag), "r") as source, rosbag.Bag(
        str(args.output_bag), "w"
    ) as output:
        for topic, message, bag_timestamp in source.read_messages(
            topics=[args.detection_topic]
        ):
            timestamp = message_timestamp(message, bag_timestamp)
            modified = copy.deepcopy(message)
            counters["frames"] += 1
            counters["detections"] += len(modified.objects)
            ego = interpolate_ego(
                ego_points, timestamp, args.maximum_interpolation_bracket_sec
            )
            estimates = detection_frames.get(timestamp, ())
            if ego is None or len(estimates) != len(modified.objects):
                counters["missing_ego_frames"] += 1
                output.write(topic, modified, bag_timestamp)
                continue
            estimates_map = transform_sensor_estimates_to_map(
                estimates,
                ego,
                args.sensor_to_ego_x_m,
                args.sensor_to_ego_y_m,
            )
            gt_items = active_gt(
                segments,
                timestamp,
                args.maximum_interpolation_bracket_sec,
                ego,
                args.maximum_range_m,
            )
            matches, _, _ = match_estimates(
                gt_items,
                estimates_map,
                args.maximum_association_distance_m,
            )
            counters["matched"] += len(matches)
            for gt_index, detection_index, _ in matches:
                segment, origin = gt_items[gt_index]
                state = causal_state(segment, origin, timestamp, args.history_sec)
                if state is None:
                    counters["missing_motion_state"] += 1
                    continue
                estimate = estimates_map[detection_index]
                world_vx = state.speed * math.cos(state.yaw)
                world_vy = state.speed * math.sin(state.yaw)
                cosine = math.cos(estimate.yaw)
                sine = math.sin(estimate.yaw)
                body_vx = cosine * world_vx + sine * world_vy
                body_vy = -sine * world_vx + cosine * world_vy
                kinematics = modified.objects[detection_index].kinematics
                kinematics.twist_with_covariance.twist.linear.x = body_vx
                kinematics.twist_with_covariance.twist.linear.y = body_vy
                kinematics.has_twist = True
                kinematics.has_twist_covariance = False
                counters["injected"] += 1
            output.write(topic, modified, bag_timestamp)

    report = {
        "raw_bag": str(args.raw_bag.resolve()),
        "pipeline_bag": str(args.pipeline_bag.resolve()),
        "output_bag": str(args.output_bag.resolve()),
        "supported_gt_segments": calibration["supported_segment_count"],
        "counters": counters,
        "notes": (
            "Oracle only: PointPainting boxes are unchanged. GT velocity is "
            "causally reconstructed from past bbox-center positions and "
            "expressed in each detected box body frame."
        ),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
