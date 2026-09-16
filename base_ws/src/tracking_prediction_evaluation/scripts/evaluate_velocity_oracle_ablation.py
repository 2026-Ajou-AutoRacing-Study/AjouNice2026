#!/usr/bin/env python3
"""Measure the upper bound from detector-provided longitudinal velocity.

The recorded PointPainting box pose, yaw, class, confidence and UUID are kept
unchanged.  Only the longitudinal speed attached to a matched tracked object is
replaced with a velocity reconstructed from MORAI GT positions.  No detector or
tracker retraining is involved.

This is an offline oracle ablation, not a production data path.  In particular,
``oracle_initial_speed_hold`` approximates a perfect velocity initialization by
holding the first matched GT speed for the lifetime of a track, whereas
``oracle_velocity_update`` approximates a perfect velocity measurement at every
detection update.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np

from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    MapPredictionConfig,
)
from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.evaluator import estimate_motion_state
from tracking_prediction_evaluation.pipeline_evaluator import (
    ObjectEstimate,
    PredictedPathEstimate,
    active_gt,
    evaluate_pipeline,
    interpolate_ego,
    match_estimates,
    read_ego_points,
    read_output_frames,
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
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument(
        "--tracking-topic",
        default="/perception/object_recognition/tracking/objects",
    )
    parser.add_argument(
        "--prediction-topic",
        default="/perception/object_recognition/objects",
    )
    parser.add_argument(
        "--map-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/lanelet2_map_v69.osm"),
    )
    parser.add_argument(
        "--projector-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/map_projector_info_v1.yaml"),
    )
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-association-distance-m", type=float, default=2.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--maximum-longitudinal-acceleration-mps2", type=float, default=0.5)
    parser.add_argument("--gt-velocity-history-sec", type=float, default=0.5)
    parser.add_argument("--prediction-time-step", type=float, default=0.1)
    parser.add_argument("--prediction-horizon", type=float, default=3.0)
    return parser.parse_args()


def _stats(values):
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(array.mean()),
        "p50": float(np.percentile(array, 50)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
    }


def _horizons(step, horizon):
    return tuple(step * index for index in range(1, int(round(horizon / step)) + 1))


def _causal_gt_speed(segment, origin, timestamp, history_sec):
    history = [
        point
        for point in segment.points
        if timestamp - history_sec <= point.timestamp <= timestamp + 1.0e-9
    ]
    if not history or abs(history[-1].timestamp - timestamp) > 1.0e-9:
        history.append(origin)
    state = estimate_motion_state(history)
    return None if state is None or not math.isfinite(state.speed) else float(state.speed)


def inject_oracle_speeds(
    tracking_frames,
    segments,
    ego_points,
    mode,
    maximum_range_m,
    maximum_association_distance_m,
    maximum_interpolation_bracket_sec,
    history_sec,
):
    """Return copied frames with only matched-object speed changed."""
    if mode not in ("baseline", "oracle_initial_speed_hold", "oracle_velocity_update"):
        raise ValueError(f"unsupported velocity mode: {mode}")

    initial_speed_by_track = {}
    output = {}
    diagnostics = {
        "matched_object_frames": 0,
        "oracle_speed_injections": 0,
        "missing_gt_motion_state": 0,
        "unmatched_estimates": 0,
        "absolute_speed_change_mps": [],
    }
    for timestamp, estimates in sorted(tracking_frames.items()):
        if mode == "baseline":
            output[timestamp] = estimates
            continue
        ego = interpolate_ego(
            ego_points, timestamp, maximum_interpolation_bracket_sec
        )
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        matches, _, matched_estimates = match_estimates(
            gt_items, estimates, maximum_association_distance_m
        )
        replacements = {}
        diagnostics["matched_object_frames"] += len(matches)
        diagnostics["unmatched_estimates"] += len(estimates) - len(matched_estimates)
        for gt_index, estimate_index, _ in matches:
            estimate = estimates[estimate_index]
            segment, origin = gt_items[gt_index]
            gt_speed = _causal_gt_speed(segment, origin, timestamp, history_sec)
            if gt_speed is None:
                diagnostics["missing_gt_motion_state"] += 1
                continue
            if mode == "oracle_initial_speed_hold":
                oracle_speed = initial_speed_by_track.setdefault(
                    estimate.object_id, gt_speed
                )
            else:
                oracle_speed = gt_speed
            replacements[estimate_index] = replace(estimate, speed=oracle_speed)
            diagnostics["oracle_speed_injections"] += 1
            diagnostics["absolute_speed_change_mps"].append(
                abs(oracle_speed - estimate.speed)
            )
        output[timestamp] = tuple(
            replacements.get(index, estimate)
            for index, estimate in enumerate(estimates)
        )
    diagnostics["absolute_speed_change_mps"] = _stats(
        diagnostics["absolute_speed_change_mps"]
    )
    diagnostics["initialized_track_count"] = len(initial_speed_by_track)
    return output, diagnostics


def build_map_predictions(
    tracking_frames,
    runtime_prediction_frames,
    network,
    horizons,
):
    output = {}
    mapped = 0
    fallback = 0
    for timestamp, tracks in sorted(tracking_frames.items()):
        runtime_by_id = {
            item.object_id: item
            for item in runtime_prediction_frames.get(timestamp, ())
        }
        predictions = []
        for track in tracks:
            trajectories = network.predict(
                x=track.x,
                y=track.y,
                z=0.0,
                yaw=track.yaw,
                speed=track.speed,
                horizons=horizons,
            )
            if trajectories:
                mapped += 1
                paths = tuple(
                    PredictedPathEstimate(
                        points=((track.x, track.y),)
                        + tuple((pose[0], pose[1]) for pose in trajectory.poses),
                        step_sec=horizons[0],
                        confidence=trajectory.confidence,
                    )
                    for trajectory in trajectories
                )
                top = max(paths, key=lambda item: item.confidence)
                predictions.append(
                    replace(
                        track,
                        predicted_path=top.points,
                        path_step_sec=top.step_sec,
                        predicted_paths=paths,
                    )
                )
            else:
                fallback += 1
                predictions.append(runtime_by_id.get(track.object_id, track))
        output[timestamp] = tuple(predictions)
    return output, {
        "mapped_object_frames": mapped,
        "fallback_object_frames": fallback,
    }


def _compact_evaluation(summary):
    multimodal = summary["prediction"]["multimodal"]
    horizon_metrics = {}
    for horizon, metrics in multimodal["horizons"].items():
        horizon_metrics[horizon] = {
            "eligible_gt": metrics["eligible_gt"],
            "coverage": metrics["best_of_k_coverage"],
            "min_fde_k_m": {
                key: metrics["min_fde_k_m"].get(key)
                for key in ("mean", "p50", "p90", "p95")
            },
            "min_k_miss_rate_2m": metrics["min_fde_k_miss_rate_2m"],
            "top1_fde_m": {
                key: metrics["top1_fde_m"].get(key)
                for key in ("mean", "p50", "p90", "p95")
            },
        }
    return {
        "tracking_speed_absolute_error_mps": summary["tracking"][
            "absolute_speed_error_mps"
        ],
        "tracking_bbox_center_absolute_distance_m": summary["tracking"][
            "bbox_center_absolute_distance_m"
        ],
        "mean_path_candidates": multimodal["matched_object_path_count"].get("mean"),
        "multi_path_rate": multimodal["multi_path_object_rate"],
        "horizons": horizon_metrics,
    }


def _evaluate(
    segments,
    ego_points,
    tracking_frames,
    prediction_frames,
    args,
    motion_slice,
):
    summary = evaluate_pipeline(
        segments,
        ego_points,
        tracking_frames,
        prediction_frames,
        horizons=(1.0, 2.0, 3.0),
        maximum_range_m=args.maximum_range_m,
        maximum_association_distance_m=args.maximum_association_distance_m,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
        prediction_motion_slice=motion_slice,
        maximum_longitudinal_acceleration_mps2=(
            args.maximum_longitudinal_acceleration_mps2
        ),
        gt_reference="bbox-center-calibrated-v1",
    )
    return _compact_evaluation(summary)


def main():
    args = parse_args()
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
    tracking_frames, runtime_prediction_frames = read_output_frames(
        args.output_bag, args.tracking_topic, args.prediction_topic
    )
    horizons = _horizons(args.prediction_time_step, args.prediction_horizon)

    results = {
        "recorded_runtime": {
            "all": _evaluate(
                segments,
                ego_points,
                tracking_frames,
                runtime_prediction_frames,
                args,
                "all",
            ),
            "cv_stable": _evaluate(
                segments,
                ego_points,
                tracking_frames,
                runtime_prediction_frames,
                args,
                "cv-stable",
            ),
        }
    }
    for mode in (
        "baseline",
        "oracle_initial_speed_hold",
        "oracle_velocity_update",
    ):
        effective_tracks, injection = inject_oracle_speeds(
            tracking_frames,
            segments,
            ego_points,
            mode,
            args.maximum_range_m,
            args.maximum_association_distance_m,
            args.maximum_interpolation_bracket_sec,
            args.gt_velocity_history_sec,
        )
        network = LaneletNetwork.load(
            args.map_file,
            args.projector_file,
            config=MapPredictionConfig(),
        )
        predictions, prediction_diagnostics = build_map_predictions(
            effective_tracks,
            runtime_prediction_frames,
            network,
            horizons,
        )
        results[mode] = {
            "injection": injection,
            "prediction_rebuild": prediction_diagnostics,
            "all": _evaluate(
                segments, ego_points, effective_tracks, predictions, args, "all"
            ),
            "cv_stable": _evaluate(
                segments,
                ego_points,
                effective_tracks,
                predictions,
                args,
                "cv-stable",
            ),
        }

    report = {
        "experiment": "PointPainting box + GT longitudinal velocity oracle",
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "output_bag": str(args.output_bag.resolve()),
            "tracking_frame_count": len(tracking_frames),
            "supported_gt_segments": calibration["supported_segment_count"],
        },
        "method": {
            "gt_velocity": (
                "Causal polynomial fit over GT bbox-center positions; no "
                "ObjectStatus.velocity unit is assumed."
            ),
            "oracle_initial_speed_hold": (
                "First successfully matched GT speed is held per track UUID; "
                "offline approximation of perfect EKF velocity initialization."
            ),
            "oracle_velocity_update": (
                "Matched GT speed replaces tracked longitudinal speed at every "
                "recorded detection frame; upper bound for a velocity head."
            ),
            "unchanged_fields": "position, yaw, dimensions, class, confidence, UUID",
        },
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
