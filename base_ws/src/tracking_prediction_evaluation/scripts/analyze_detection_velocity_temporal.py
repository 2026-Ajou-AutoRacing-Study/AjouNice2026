#!/usr/bin/env python3
"""Measure temporal stability of detector velocity on persistent MORAI GT tracks."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.detection_evaluator import (
    build_evaluation_frames,
    read_detection_frames,
)
from tracking_prediction_evaluation.pipeline_evaluator import (
    _causal_motion_state,
    read_ego_points,
    statistics,
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
    parser.add_argument("detection_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--score-threshold", type=float, default=0.40)
    parser.add_argument("--association-distance-m", type=float, default=1.0)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-adjacent-gap-sec", type=float, default=0.25)
    parser.add_argument("--morai-heading-correction-deg", type=float, default=-1.347)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def greedy_vehicle_matches(frame, score_threshold, maximum_distance):
    candidates = []
    for gt_index, (segment, gt) in enumerate(frame.gt):
        if segment.class_name != "vehicle":
            continue
        for prediction_index, prediction in enumerate(frame.predictions):
            if prediction.class_name != "vehicle" or prediction.score < score_threshold:
                continue
            if prediction.velocity_x is None or prediction.velocity_y is None:
                continue
            distance = math.hypot(prediction.x - gt.x, prediction.y - gt.y)
            if distance <= maximum_distance:
                candidates.append((distance, gt_index, prediction_index))

    matches = []
    used_gt = set()
    used_predictions = set()
    for distance, gt_index, prediction_index in sorted(candidates):
        if gt_index in used_gt or prediction_index in used_predictions:
            continue
        used_gt.add(gt_index)
        used_predictions.add(prediction_index)
        segment, gt = frame.gt[gt_index]
        prediction = frame.predictions[prediction_index]
        matches.append((segment, gt, prediction, distance))
    return matches


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    raw_segments = read_morai_trajectories(
        args.raw_bag,
        topic="/Object_topic",
        include_classes=("vehicle",),
        heading_correction_deg=args.morai_heading_correction_deg,
    )
    calibrated_segments, calibration = transform_segments_to_calibrated_bbox_center(
        raw_segments
    )
    segments = transform_segments_planar(
        calibrated_segments, translation_xyz, transform_yaw
    )
    ego_points = read_ego_points(
        args.raw_bag,
        "/Ego_topic",
        translation_xyz,
        transform_yaw,
        args.morai_heading_correction_deg,
    )
    detection_frames = read_detection_frames(
        args.detection_bag,
        "/perception/object_recognition/detection/objects",
    )
    frames, frame_counts = build_evaluation_frames(
        segments,
        ego_points,
        detection_frames,
        maximum_range_m=args.maximum_range_m,
    )

    track_rows = defaultdict(list)
    signed_longitudinal_errors = []
    lateral_errors = []
    center_errors = []
    for frame in frames:
        for segment, _, prediction, center_error in greedy_vehicle_matches(
            frame, args.score_threshold, args.association_distance_m
        ):
            motion = _causal_motion_state(segment, frame.timestamp)
            if motion is None:
                continue
            cosine = math.cos(motion.yaw)
            sine = math.sin(motion.yaw)
            predicted_longitudinal = (
                cosine * prediction.velocity_x + sine * prediction.velocity_y
            )
            predicted_lateral = (
                -sine * prediction.velocity_x + cosine * prediction.velocity_y
            )
            signed_error = predicted_longitudinal - motion.speed
            row = {
                "timestamp": frame.timestamp,
                "predicted_longitudinal_mps": predicted_longitudinal,
                "predicted_lateral_mps": predicted_lateral,
                "gt_longitudinal_mps": motion.speed,
                "signed_longitudinal_error_mps": signed_error,
            }
            track_rows[segment.key].append(row)
            signed_longitudinal_errors.append(signed_error)
            lateral_errors.append(abs(predicted_lateral))
            center_errors.append(center_error)

    prediction_steps = []
    gt_steps = []
    residual_steps = []
    prediction_acceleration = []
    gt_acceleration = []
    adjacent_pairs = 0
    for rows in track_rows.values():
        rows.sort(key=lambda row: row["timestamp"])
        for previous, current in zip(rows, rows[1:]):
            dt = current["timestamp"] - previous["timestamp"]
            if dt <= 1.0e-6 or dt > args.maximum_adjacent_gap_sec:
                continue
            prediction_step = (
                current["predicted_longitudinal_mps"]
                - previous["predicted_longitudinal_mps"]
            )
            gt_step = (
                current["gt_longitudinal_mps"]
                - previous["gt_longitudinal_mps"]
            )
            residual_step = (
                current["signed_longitudinal_error_mps"]
                - previous["signed_longitudinal_error_mps"]
            )
            prediction_steps.append(abs(prediction_step))
            gt_steps.append(abs(gt_step))
            residual_steps.append(abs(residual_step))
            prediction_acceleration.append(abs(prediction_step / dt))
            gt_acceleration.append(abs(gt_step / dt))
            adjacent_pairs += 1

    report = {
        "contract": {
            "class": "vehicle",
            "score_threshold": args.score_threshold,
            "association_distance_m": args.association_distance_m,
            "maximum_range_m": args.maximum_range_m,
            "maximum_adjacent_gap_sec": args.maximum_adjacent_gap_sec,
            "morai_heading_correction_deg": args.morai_heading_correction_deg,
            "velocity_frame": (
                "detector body velocity transformed to map, then projected onto "
                "the causal GT heading"
            ),
        },
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "detection_bag": str(args.detection_bag.resolve()),
            "gt_calibration": calibration,
            **frame_counts,
        },
        "matched_samples": len(signed_longitudinal_errors),
        "matched_tracks": len(track_rows),
        "adjacent_pairs": adjacent_pairs,
        "signed_longitudinal_error_mps": statistics(signed_longitudinal_errors),
        "absolute_longitudinal_error_mps": statistics(
            [abs(value) for value in signed_longitudinal_errors]
        ),
        "absolute_lateral_velocity_mps": statistics(lateral_errors),
        "center_error_m": statistics(center_errors),
        "adjacent_prediction_speed_step_mps": statistics(prediction_steps),
        "adjacent_gt_speed_step_mps": statistics(gt_steps),
        "adjacent_velocity_error_step_mps": statistics(residual_steps),
        "apparent_prediction_acceleration_mps2": statistics(prediction_acceleration),
        "apparent_gt_acceleration_mps2": statistics(gt_acceleration),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "matched_samples": report["matched_samples"],
                "adjacent_pairs": adjacent_pairs,
                "signed_bias_mps": report["signed_longitudinal_error_mps"].get("mean"),
                "speed_mae_mps": report["absolute_longitudinal_error_mps"].get("mean"),
                "error_step_p50_mps": report["adjacent_velocity_error_step_mps"].get("p50"),
                "error_step_p95_mps": report["adjacent_velocity_error_step_mps"].get("p95"),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
