#!/usr/bin/env python3
"""Compare detector and position-tracker velocity on common causal GT samples."""

from __future__ import annotations

import argparse
import bisect
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
    active_gt,
    interpolate_ego,
    match_estimates,
    read_ego_points,
    read_output_frames,
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
    parser.add_argument("p6_detection_bag", type=Path)
    parser.add_argument("p7_detection_bag", type=Path)
    parser.add_argument("position_tracker_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--score-threshold", type=float, default=0.40)
    parser.add_argument("--association-distance-m", type=float, default=1.0)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-tracker-time-gap-sec", type=float, default=0.075)
    parser.add_argument("--acceleration-lookback-sec", type=float, default=0.20)
    parser.add_argument("--morai-heading-correction-deg", type=float, default=-1.347)
    parser.add_argument(
        "--morai-static-tf", type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"))
    return parser.parse_args()


def acceleration_slice(value):
    if value <= 0.5:
        return "steady_le_0_5_mps2"
    if value <= 2.0:
        return "moderate_0_5_to_2_mps2"
    return "high_gt_2_mps2"


def detector_rows(bag_path, segments, ego_points, args):
    frames, _ = build_evaluation_frames(
        segments,
        ego_points,
        read_detection_frames(
            bag_path, "/perception/object_recognition/detection/objects"),
        maximum_range_m=args.maximum_range_m,
    )
    rows = {}
    for frame in frames:
        candidates = []
        for gt_index, (segment, gt) in enumerate(frame.gt):
            if segment.class_name != "vehicle":
                continue
            for prediction_index, prediction in enumerate(frame.predictions):
                if (prediction.class_name != "vehicle"
                        or prediction.score < args.score_threshold
                        or prediction.velocity_x is None
                        or prediction.velocity_y is None):
                    continue
                distance = math.hypot(
                    prediction.x - gt.x, prediction.y - gt.y)
                if distance <= args.association_distance_m:
                    candidates.append(
                        (distance, gt_index, prediction_index))
        used_gt = set()
        used_prediction = set()
        for _, gt_index, prediction_index in sorted(candidates):
            if gt_index in used_gt or prediction_index in used_prediction:
                continue
            used_gt.add(gt_index)
            used_prediction.add(prediction_index)
            segment, _ = frame.gt[gt_index]
            prediction = frame.predictions[prediction_index]
            state = _causal_motion_state(segment, frame.timestamp)
            previous_state = _causal_motion_state(
                segment, frame.timestamp - args.acceleration_lookback_sec)
            if state is None or previous_state is None:
                continue
            cosine = math.cos(state.yaw)
            sine = math.sin(state.yaw)
            longitudinal = (
                cosine * prediction.velocity_x
                + sine * prediction.velocity_y)
            acceleration = abs(
                state.speed - previous_state.speed
            ) / args.acceleration_lookback_sec
            rows[(segment.key, frame.timestamp)] = {
                "gt_speed": state.speed,
                "gt_yaw": state.yaw,
                "gt_acceleration": acceleration,
                "estimate": longitudinal,
            }
    return rows


def nearest_timestamp(timestamps, target, maximum_gap):
    index = bisect.bisect_left(timestamps, target)
    candidates = timestamps[max(0, index - 1):index + 1]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda value: abs(value - target))
    return nearest if abs(nearest - target) <= maximum_gap else None


def tracker_estimate_for_segment(
    timestamp, segment_key, tracking_frames, tracking_timestamps,
    track_first_seen, segments, ego_points, args,
):
    tracker_timestamp = nearest_timestamp(
        tracking_timestamps, timestamp, args.maximum_tracker_time_gap_sec)
    if tracker_timestamp is None:
        return None
    ego = interpolate_ego(ego_points, tracker_timestamp, 0.15)
    gt_items = active_gt(
        segments, tracker_timestamp, 0.15, ego, args.maximum_range_m)
    estimates = tracking_frames[tracker_timestamp]
    matches, _, _ = match_estimates(
        gt_items, estimates, args.association_distance_m)
    for gt_index, estimate_index, _ in matches:
        segment, _ = gt_items[gt_index]
        if segment.key != segment_key:
            continue
        estimate = estimates[estimate_index]
        vx = (
            estimate.speed * math.cos(estimate.yaw)
            - estimate.lateral_speed * math.sin(estimate.yaw))
        vy = (
            estimate.speed * math.sin(estimate.yaw)
            + estimate.lateral_speed * math.cos(estimate.yaw))
        return (
            vx, vy,
            tracker_timestamp - track_first_seen[estimate.object_id],
        )
    return None


def summarize(rows):
    names = ("p6", "p7", "position_tracker")
    result = {
        name: statistics([abs(row[name]) for row in rows])
        for name in names
    }
    result["oracle_best_of_three"] = statistics([
        min(abs(row[name]) for name in names) for row in rows
    ])
    for detector_weight in (0.25, 0.5, 0.75):
        name = f"p7_tracker_blend_detector_{detector_weight:g}"
        result[name] = statistics([
            abs(detector_weight * row["p7"]
                + (1.0 - detector_weight) * row["position_tracker"])
            for row in rows
        ])
    for threshold in (0.25, 0.5, 1.0, 2.0):
        name = f"p7_until_track_age_{threshold:g}s_then_tracker"
        result[name] = statistics([
            abs(row["p7"] if row["track_age_sec"] <= threshold
                else row["position_tracker"])
            for row in rows
        ])
    result["win_fraction"] = {
        name: sum(
            abs(row[name]) == min(abs(row[candidate]) for candidate in names)
            for row in rows
        ) / len(rows) if rows else None
        for name in names
    }
    result["count"] = len(rows)
    return result


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf)
    raw_segments = read_morai_trajectories(
        args.raw_bag,
        topic="/Object_topic",
        include_classes=("vehicle",),
        heading_correction_deg=args.morai_heading_correction_deg,
    )
    calibrated_segments, calibration = \
        transform_segments_to_calibrated_bbox_center(raw_segments)
    segments = transform_segments_planar(
        calibrated_segments, translation_xyz, transform_yaw)
    ego_points = read_ego_points(
        args.raw_bag, "/Ego_topic", translation_xyz, transform_yaw,
        args.morai_heading_correction_deg)
    p6_rows = detector_rows(
        args.p6_detection_bag, segments, ego_points, args)
    p7_rows = detector_rows(
        args.p7_detection_bag, segments, ego_points, args)
    tracking_frames, _ = read_output_frames(
        args.position_tracker_bag,
        "/perception/object_recognition/tracking/objects",
        "/perception/object_recognition/objects",
    )
    tracking_timestamps = sorted(tracking_frames)
    track_first_seen = {}
    for timestamp, estimates in sorted(tracking_frames.items()):
        for estimate in estimates:
            track_first_seen.setdefault(estimate.object_id, timestamp)

    common = []
    p7_by_segment = defaultdict(list)
    for (segment_key, timestamp), row in p7_rows.items():
        p7_by_segment[segment_key].append((timestamp, row))
    for (segment_key, p6_timestamp), p6 in p6_rows.items():
        candidates = p7_by_segment.get(segment_key, ())
        if not candidates:
            continue
        p7_timestamp, p7 = min(
            candidates, key=lambda item: abs(item[0] - p6_timestamp))
        if abs(p7_timestamp - p6_timestamp) > 1.0e-4:
            continue
        tracker_velocity = tracker_estimate_for_segment(
            p6_timestamp, segment_key, tracking_frames,
            tracking_timestamps, track_first_seen, segments, ego_points, args)
        if tracker_velocity is None:
            continue
        cosine = math.cos(p6["gt_yaw"])
        sine = math.sin(p6["gt_yaw"])
        tracker_longitudinal = (
            cosine * tracker_velocity[0] + sine * tracker_velocity[1])
        common.append({
            "acceleration": p6["gt_acceleration"],
            "p6": p6["estimate"] - p6["gt_speed"],
            "p7": p7["estimate"] - p7["gt_speed"],
            "position_tracker": tracker_longitudinal - p6["gt_speed"],
            "track_age_sec": tracker_velocity[2],
        })

    sliced = defaultdict(list)
    track_age_sliced = defaultdict(list)
    for row in common:
        sliced[acceleration_slice(row["acceleration"])].append(row)
        if row["track_age_sec"] <= 0.5:
            track_age_sliced["early_le_0_5s"].append(row)
        elif row["track_age_sec"] <= 2.0:
            track_age_sliced["settling_0_5_to_2s"].append(row)
        else:
            track_age_sliced["mature_gt_2s"].append(row)
    report = {
        "contract": {
            "common_samples_only": True,
            "gt_identity_used_for_evaluation_only": True,
            "acceleration_lookback_sec": args.acceleration_lookback_sec,
            "maximum_tracker_time_gap_sec": args.maximum_tracker_time_gap_sec,
            "gt_calibration": calibration,
        },
        "all": summarize(common),
        "by_gt_acceleration": {
            name: summarize(rows) for name, rows in sorted(sliced.items())
        },
        "by_track_age": {
            name: summarize(rows)
            for name, rows in sorted(track_age_sliced.items())
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "output": str(args.output),
        "common_samples": len(common),
        "mae": {
            name: report["all"][name].get("mean")
            for name in ("p6", "p7", "position_tracker", "oracle_best_of_three")
        },
    }, indent=2))


if __name__ == "__main__":
    main()
