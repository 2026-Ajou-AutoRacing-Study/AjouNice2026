#!/usr/bin/env python3
"""Separate center noise, association, and velocity-head error.

The probe reuses recorded detector output.  It estimates object velocity from
two detected centers separated by a configurable 0.3--0.6 second baseline in
two ways:

* ``gt_identity`` groups detections with the reconstructed MORAI GT identity;
* ``causal_association`` groups detections using only past detected centers.

Both estimates are compared with the displacement of the *current* GT actor
over the exact same timestamps.  The first path isolates detector center
noise.  The difference between the second and first paths measures the extra
cost of online association.  This is an offline diagnostic, not a deployable
tracker or an OD score.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.detection_evaluator import (
    build_evaluation_frames,
    read_detection_frames,
)
from tracking_prediction_evaluation.pipeline_evaluator import (
    _causal_motion_state,
    match_estimates,
    read_ego_points,
    statistics,
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
class Observation:
    timestamp: float
    x: float
    y: float
    gt_key: str | None = None
    yaw: float = 0.0


@dataclass
class DetectionTrack:
    track_id: int
    observations: list[Observation] = field(default_factory=list)
    last_velocity: tuple[float, float] | None = None


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("detection_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--score-threshold", type=float, default=0.40)
    parser.add_argument("--gt-match-distance-m", type=float, default=1.0)
    parser.add_argument("--track-association-gate-m", type=float, default=4.0)
    parser.add_argument("--track-maximum-gap-sec", type=float, default=0.5)
    parser.add_argument("--minimum-baseline-sec", type=float, default=0.3)
    parser.add_argument("--target-baseline-sec", type=float, default=0.45)
    parser.add_argument("--maximum-baseline-sec", type=float, default=0.6)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--morai-heading-correction-deg", type=float, default=-1.347)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def _prediction_xy(track: DetectionTrack, timestamp: float):
    last = track.observations[-1]
    if len(track.observations) < 2:
        return last.x, last.y
    previous = track.observations[-2]
    source_dt = last.timestamp - previous.timestamp
    if source_dt <= 1.0e-6:
        return last.x, last.y
    target_dt = max(0.0, timestamp - last.timestamp)
    return (
        last.x + (last.x - previous.x) / source_dt * target_dt,
        last.y + (last.y - previous.y) / source_dt * target_dt,
    )


def associate_tracks(active_tracks, detections, timestamp, gate_m):
    """Return deterministic track-to-detection pairs without using GT."""
    tracks = list(sorted(active_tracks.values(), key=lambda item: item.track_id))
    if not tracks or not detections:
        return [], set(), set()
    distances = np.empty((len(tracks), len(detections)), dtype=np.float64)
    for row, track in enumerate(tracks):
        predicted_x, predicted_y = _prediction_xy(track, timestamp)
        for column, detection in enumerate(detections):
            distances[row, column] = math.hypot(
                predicted_x - detection.x, predicted_y - detection.y
            )
    rows, columns = linear_sum_assignment(distances)
    pairs = []
    used_tracks = set()
    used_detections = set()
    for row, column in zip(rows, columns):
        if distances[row, column] > gate_m:
            continue
        track = tracks[int(row)]
        pairs.append((track.track_id, int(column)))
        used_tracks.add(track.track_id)
        used_detections.add(int(column))
    return pairs, used_tracks, used_detections


def choose_past(observations, timestamp, minimum, target, maximum):
    candidates = [
        item
        for item in observations
        if minimum <= timestamp - item.timestamp <= maximum
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda item: abs(timestamp - item.timestamp - target))


def pair_error(previous, current, target_previous, target_current):
    dt = current.timestamp - previous.timestamp
    estimate = np.asarray(
        ((current.x - previous.x) / dt, (current.y - previous.y) / dt),
        dtype=np.float64,
    )
    target = np.asarray(
        (
            (target_current.x - target_previous.x) / dt,
            (target_current.y - target_previous.y) / dt,
        ),
        dtype=np.float64,
    )
    return {
        "dt": dt,
        "vector_error": float(np.linalg.norm(estimate - target)),
        "speed_error": abs(float(np.linalg.norm(estimate) - np.linalg.norm(target))),
        "estimate_speed": float(np.linalg.norm(estimate)),
        "target_speed": float(np.linalg.norm(target)),
        "estimate_vx": float(estimate[0]),
        "estimate_vy": float(estimate[1]),
    }


def axial_heading_error_deg(velocity_x, velocity_y, yaw):
    if math.hypot(velocity_x, velocity_y) <= 1.0e-6:
        return 90.0
    error = abs(math.atan2(velocity_y, velocity_x) - yaw)
    error = error % math.pi
    error = min(error, math.pi - error)
    return math.degrees(error)


def linear_fit_residual_m(observations):
    """2D RMS residual from a constant-velocity fit over causal centers."""
    if len(observations) < 3:
        return None
    timestamps = np.asarray(
        [item.timestamp - observations[-1].timestamp for item in observations],
        dtype=np.float64,
    )
    design = np.column_stack((timestamps, np.ones_like(timestamps)))
    x = np.asarray([item.x for item in observations], dtype=np.float64)
    y = np.asarray([item.y for item in observations], dtype=np.float64)
    fit_x = design @ np.linalg.lstsq(design, x, rcond=None)[0]
    fit_y = design @ np.linalg.lstsq(design, y, rcond=None)[0]
    residual = np.hypot(x - fit_x, y - fit_y)
    return float(np.sqrt(np.mean(np.square(residual))))


def velocity_error(velocity_x, velocity_y, target_previous, target_current, dt):
    estimate = np.asarray((velocity_x, velocity_y), dtype=np.float64)
    target = np.asarray(
        (
            (target_current.x - target_previous.x) / dt,
            (target_current.y - target_previous.y) / dt,
        ),
        dtype=np.float64,
    )
    return {
        "dt": dt,
        "vector_error": float(np.linalg.norm(estimate - target)),
        "speed_error": abs(float(np.linalg.norm(estimate) - np.linalg.norm(target))),
        "estimate_speed": float(np.linalg.norm(estimate)),
        "target_speed": float(np.linalg.norm(target)),
    }


def summarize_pair_rows(rows):
    return {
        "pairs": len(rows),
        "baseline_sec": statistics(row["dt"] for row in rows),
        "vector_error_mps": statistics(row["vector_error"] for row in rows),
        "speed_mae_mps": statistics(row["speed_error"] for row in rows),
        "estimated_speed_mps": statistics(row["estimate_speed"] for row in rows),
        "target_speed_mps": statistics(row["target_speed"] for row in rows),
    }


def summarize_quality_gates(rows):
    def report(selected):
        result = summarize_pair_rows(selected)
        result["coverage"] = len(selected) / len(rows) if rows else 0.0
        result["fit_residual_m"] = statistics(
            row["fit_residual_m"]
            for row in selected
            if row.get("fit_residual_m") is not None
        )
        result["heading_error_deg"] = statistics(
            row["heading_error_deg"] for row in selected
        )
        result["velocity_change_mps"] = statistics(
            row["velocity_change_mps"]
            for row in selected
            if row.get("velocity_change_mps") is not None
        )
        return result

    definitions = {
        "all": lambda row: True,
        "fit_residual_le_0p10m": lambda row: (
            row.get("fit_residual_m") is not None
            and row["fit_residual_m"] <= 0.10
        ),
        "fit_residual_le_0p20m": lambda row: (
            row.get("fit_residual_m") is not None
            and row["fit_residual_m"] <= 0.20
        ),
        "heading_le_15deg": lambda row: row["heading_error_deg"] <= 15.0,
        "heading_le_30deg": lambda row: row["heading_error_deg"] <= 30.0,
        "velocity_change_le_1mps": lambda row: (
            row.get("velocity_change_mps") is not None
            and row["velocity_change_mps"] <= 1.0
        ),
        "velocity_change_le_2mps": lambda row: (
            row.get("velocity_change_mps") is not None
            and row["velocity_change_mps"] <= 2.0
        ),
        "combined_loose": lambda row: (
            row.get("fit_residual_m") is not None
            and row["fit_residual_m"] <= 0.20
            and row["heading_error_deg"] <= 30.0
            and row.get("velocity_change_mps") is not None
            and row["velocity_change_mps"] <= 2.0
        ),
    }
    return {
        name: report([row for row in rows if predicate(row)])
        for name, predicate in definitions.items()
    }


def analyze_frames(
    frames,
    *,
    score_threshold,
    gt_match_distance_m,
    track_association_gate_m,
    track_maximum_gap_sec,
    minimum_baseline_sec,
    target_baseline_sec,
    maximum_baseline_sec,
):
    gt_history = defaultdict(list)
    active_tracks = {}
    next_track_id = 1
    gt_pair_rows = []
    association_pair_rows = []
    paired_head_rows = []
    head_errors = []
    association_pairs_with_known_identity = 0
    association_identity_switches = 0
    matched_detections = 0

    for frame in frames:
        detections = tuple(
            item
            for item in frame.predictions
            if item.class_name == "vehicle" and item.score >= score_threshold
        )
        gt_items = tuple(
            item for item in frame.gt if item[0].class_name == "vehicle"
        )
        matches, _, _ = match_estimates(
            gt_items, detections, gt_match_distance_m
        )
        detection_gt = {}
        segment_by_key = {}
        point_by_key = {}
        for gt_index, detection_index, _ in matches:
            segment, point = gt_items[gt_index]
            detection_gt[detection_index] = segment.key
            segment_by_key[segment.key] = segment
            point_by_key[segment.key] = point
            matched_detections += 1

            detection = detections[detection_index]
            current = Observation(
                frame.timestamp, detection.x, detection.y, segment.key, detection.yaw
            )
            previous = choose_past(
                gt_history[segment.key],
                frame.timestamp,
                minimum_baseline_sec,
                target_baseline_sec,
                maximum_baseline_sec,
            )
            if previous is not None:
                target_previous = interpolate_point(
                    segment.points, previous.timestamp, 0.15
                )
                if target_previous is not None:
                    gt_pair_rows.append(
                        pair_error(previous, current, target_previous, point)
                    )
                    if (
                        detection.velocity_x is not None
                        and detection.velocity_y is not None
                    ):
                        paired_head_rows.append(velocity_error(
                            detection.velocity_x,
                            detection.velocity_y,
                            target_previous,
                            point,
                            current.timestamp - previous.timestamp,
                        ))
            gt_history[segment.key].append(current)

            if detection.velocity_x is not None and detection.velocity_y is not None:
                motion = _causal_motion_state(segment, frame.timestamp)
                if motion is not None:
                    cosine = math.cos(motion.yaw)
                    sine = math.sin(motion.yaw)
                    longitudinal = (
                        cosine * detection.velocity_x + sine * detection.velocity_y
                    )
                    head_errors.append(abs(longitudinal - motion.speed))

        expired = [
            track_id
            for track_id, track in active_tracks.items()
            if frame.timestamp - track.observations[-1].timestamp
            > track_maximum_gap_sec
        ]
        for track_id in expired:
            del active_tracks[track_id]

        pairs, _, used_detections = associate_tracks(
            active_tracks,
            detections,
            frame.timestamp,
            track_association_gate_m,
        )
        for track_id, detection_index in pairs:
            track = active_tracks[track_id]
            detection = detections[detection_index]
            gt_key = detection_gt.get(detection_index)
            current = Observation(
                frame.timestamp, detection.x, detection.y, gt_key, detection.yaw
            )
            previous = choose_past(
                track.observations,
                frame.timestamp,
                minimum_baseline_sec,
                target_baseline_sec,
                maximum_baseline_sec,
            )
            if previous is not None and gt_key is not None:
                segment = segment_by_key[gt_key]
                target_previous = interpolate_point(
                    segment.points, previous.timestamp, 0.15
                )
                target_current = point_by_key[gt_key]
                if target_previous is not None:
                    row = pair_error(
                        previous, current, target_previous, target_current
                    )
                    causal_window = [
                        item for item in track.observations
                        if frame.timestamp - item.timestamp <= maximum_baseline_sec
                    ] + [current]
                    row["fit_residual_m"] = linear_fit_residual_m(causal_window)
                    row["heading_error_deg"] = axial_heading_error_deg(
                        row["estimate_vx"], row["estimate_vy"], detection.yaw
                    )
                    row["velocity_change_mps"] = (
                        math.hypot(
                            row["estimate_vx"] - track.last_velocity[0],
                            row["estimate_vy"] - track.last_velocity[1],
                        )
                        if track.last_velocity is not None else None
                    )
                    association_pair_rows.append(row)
                    track.last_velocity = (
                        row["estimate_vx"], row["estimate_vy"]
                    )
                    if previous.gt_key is not None:
                        association_pairs_with_known_identity += 1
                        if previous.gt_key != gt_key:
                            association_identity_switches += 1
            track.observations.append(current)

        for detection_index, detection in enumerate(detections):
            if detection_index in used_detections:
                continue
            active_tracks[next_track_id] = DetectionTrack(
                track_id=next_track_id,
                observations=[Observation(
                    frame.timestamp,
                    detection.x,
                    detection.y,
                    detection_gt.get(detection_index),
                    detection.yaw,
                )],
            )
            next_track_id += 1

    gt_report = summarize_pair_rows(gt_pair_rows)
    association_report = summarize_pair_rows(association_pair_rows)
    association_report.update({
        "pairs_with_known_endpoint_identity": association_pairs_with_known_identity,
        "identity_switches": association_identity_switches,
        "identity_switch_rate": (
            association_identity_switches / association_pairs_with_known_identity
            if association_pairs_with_known_identity else None
        ),
    })
    return {
        "matched_detections": matched_detections,
        "gt_identity_predicted_centers": gt_report,
        "causal_detection_association": association_report,
        "causal_quality_gates": summarize_quality_gates(association_pair_rows),
        "velocity_head_on_identical_pairs": summarize_pair_rows(paired_head_rows),
        "velocity_head_longitudinal_mae_mps": statistics(head_errors),
    }


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
    result = analyze_frames(
        frames,
        score_threshold=args.score_threshold,
        gt_match_distance_m=args.gt_match_distance_m,
        track_association_gate_m=args.track_association_gate_m,
        track_maximum_gap_sec=args.track_maximum_gap_sec,
        minimum_baseline_sec=args.minimum_baseline_sec,
        target_baseline_sec=args.target_baseline_sec,
        maximum_baseline_sec=args.maximum_baseline_sec,
    )
    report = {
        "contract": {
            "class": "vehicle",
            "score_threshold": args.score_threshold,
            "gt_match_distance_m": args.gt_match_distance_m,
            "track_association_gate_m": args.track_association_gate_m,
            "track_maximum_gap_sec": args.track_maximum_gap_sec,
            "baseline_sec": [
                args.minimum_baseline_sec,
                args.maximum_baseline_sec,
            ],
            "target_baseline_sec": args.target_baseline_sec,
            "maximum_range_m": args.maximum_range_m,
        "comparison": (
                "predicted-center displacement versus the current GT actor's "
                "geometric-center displacement over identical timestamps"
            ),
            "gt_center_gt_identity_reference": (
                "The exact GT geometric-center displacement over each pair is "
                "the zero-error reference for all three pairwise estimates."
            ),
        },
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "detection_bag": str(args.detection_bag.resolve()),
            "gt_calibration": calibration,
            **frame_counts,
        },
        **result,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output),
        "matched_detections": result["matched_detections"],
        "gt_identity_speed_mae_mps": result[
            "gt_identity_predicted_centers"]["speed_mae_mps"].get("mean"),
        "causal_association_speed_mae_mps": result[
            "causal_detection_association"]["speed_mae_mps"].get("mean"),
        "association_identity_switch_rate": result[
            "causal_detection_association"]["identity_switch_rate"],
        "velocity_head_longitudinal_mae_mps": result[
            "velocity_head_longitudinal_mae_mps"].get("mean"),
        "velocity_head_pair_speed_mae_mps": result[
            "velocity_head_on_identical_pairs"]["speed_mae_mps"].get("mean"),
    }, indent=2))


if __name__ == "__main__":
    main()
