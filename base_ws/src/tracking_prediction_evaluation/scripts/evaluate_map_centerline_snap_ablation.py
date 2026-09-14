#!/usr/bin/env python3
"""Compare gradual and immediate Lanelet-centerline prediction starts.

This is an offline evaluation-only tool.  It rebuilds map-based predictions
from recorded TrackedObjects and never changes the production ROS topics or
the tracker state stored in the source bag.
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
from tracking_prediction_evaluation.pipeline_evaluator import (
    ObjectEstimate,
    PredictedPathEstimate,
    evaluate_pipeline,
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
        default=Path("kcity_map/lanelet_map/lanelet2_map_v64.osm"),
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
    parser.add_argument(
        "--maximum-longitudinal-acceleration-mps2", type=float, default=0.5
    )
    parser.add_argument("--prediction-time-step", type=float, default=0.1)
    parser.add_argument("--prediction-horizon", type=float, default=3.0)
    return parser.parse_args()


def compact_statistics(values):
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


def prediction_horizons(step, horizon):
    count = int(round(horizon / step))
    return tuple(step * index for index in range(1, count + 1))


def snap_estimate_to_best_lane(estimate, network):
    candidates = network.lane_candidates(estimate.x, estimate.y, estimate.yaw)
    if not candidates:
        return estimate, None
    projection = candidates[0].projection
    return (
        replace(
            estimate,
            x=projection.x,
            y=projection.y,
            yaw=projection.yaw,
        ),
        projection.distance,
    )


def build_prediction_frames(
    tracking_frames,
    baseline_prediction_frames,
    network,
    horizons,
    snap_tracking,
):
    output_frames = {}
    effective_tracking_frames = {}
    mapped = 0
    fallback = 0
    snapped = 0
    snap_distances = []
    first_step_lateral = []

    for timestamp, tracks in sorted(tracking_frames.items()):
        baseline_by_id = {
            item.object_id: item
            for item in baseline_prediction_frames.get(timestamp, ())
        }
        effective_tracks = []
        predictions = []
        for original in tracks:
            effective = original
            snap_distance = None
            if snap_tracking:
                effective, snap_distance = snap_estimate_to_best_lane(
                    original, network
                )
                if snap_distance is not None:
                    snapped += 1
                    snap_distances.append(snap_distance)

            trajectories = network.predict(
                x=effective.x,
                y=effective.y,
                z=0.0,
                yaw=effective.yaw,
                speed=effective.speed,
                horizons=horizons,
            )
            if trajectories:
                mapped += 1
                paths = tuple(
                    PredictedPathEstimate(
                        points=((effective.x, effective.y),)
                        + tuple((pose[0], pose[1]) for pose in trajectory.poses),
                        step_sec=horizons[0],
                        confidence=trajectory.confidence,
                    )
                    for trajectory in trajectories
                )
                top = max(paths, key=lambda item: item.confidence)
                if len(top.points) >= 2:
                    dx = top.points[1][0] - top.points[0][0]
                    dy = top.points[1][1] - top.points[0][1]
                    first_step_lateral.append(
                        abs(-math.sin(effective.yaw) * dx + math.cos(effective.yaw) * dy)
                    )
                prediction = replace(
                    effective,
                    predicted_path=top.points,
                    path_step_sec=top.step_sec,
                    predicted_paths=paths,
                )
            else:
                fallback += 1
                # Preserve the runtime fallback exactly when map matching is
                # unavailable.  A tracking snap is only defined when a valid
                # Lanelet candidate exists, so this remains self-consistent.
                prediction = baseline_by_id.get(original.object_id, original)
                effective = original
            effective_tracks.append(effective)
            predictions.append(prediction)
        effective_tracking_frames[timestamp] = tuple(effective_tracks)
        output_frames[timestamp] = tuple(predictions)

    return effective_tracking_frames, output_frames, {
        "mapped_object_frames": mapped,
        "fallback_object_frames": fallback,
        "snapped_object_frames": snapped,
        "snap_distance_m": compact_statistics(snap_distances),
        "first_step_lateral_displacement_m": compact_statistics(first_step_lateral),
    }


def compact_evaluation(summary):
    multimodal = summary["prediction"]["multimodal"]
    horizons = {}
    for horizon, metrics in multimodal["horizons"].items():
        horizons[horizon] = {
            "eligible_gt": metrics["eligible_gt"],
            "coverage": metrics["best_of_k_coverage"],
            "top1_fde_m": {
                key: metrics["top1_fde_m"].get(key)
                for key in ("mean", "p50", "p90", "p95")
            },
            "min_fde_k_m": {
                key: metrics["min_fde_k_m"].get(key)
                for key in ("mean", "p50", "p90", "p95")
            },
            "top1_miss_rate_2m": metrics["top1_fde_miss_rate_2m"],
            "min_k_miss_rate_2m": metrics["min_fde_k_miss_rate_2m"],
            "min_k_components": metrics["min_fde_k_components_gt_heading"],
        }
    return {
        "tracking_center_m": {
            key: summary["tracking"]["bbox_center_absolute_distance_m"].get(key)
            for key in ("mean", "p50", "p90", "p95")
        },
        "tracking_yaw_error_deg": {
            key: summary["tracking"]["absolute_yaw_error_deg"].get(key)
            for key in ("mean", "p50", "p90", "p95")
        },
        "mean_path_candidates": multimodal["matched_object_path_count"].get("mean"),
        "multi_path_rate": multimodal["multi_path_object_rate"],
        "horizons": horizons,
    }


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(args.morai_static_tf)
    object_status_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle",),
    )
    calibrated_segments, calibration = transform_segments_to_calibrated_bbox_center(
        object_status_segments
    )
    segments = transform_segments_planar(
        calibrated_segments, translation_xyz, transform_yaw
    )
    ego_points = read_ego_points(
        args.raw_bag, args.ego_topic, translation_xyz, transform_yaw
    )
    tracking_frames, baseline_prediction_frames = read_output_frames(
        args.output_bag, args.tracking_topic, args.prediction_topic
    )
    horizons = prediction_horizons(
        args.prediction_time_step, args.prediction_horizon
    )

    modes = (
        ("gradual_1p5s", 1.5, False),
        ("prediction_snap_from_0p1s", 0.0, False),
        ("tracking_snap_from_0s", 0.0, True),
    )
    results = {}
    for name, convergence_sec, snap_tracking in modes:
        config = MapPredictionConfig(
            lateral_convergence_time_sec=convergence_sec
        )
        network = LaneletNetwork.load(
            args.map_file, args.projector_file, config=config
        )
        effective_tracks, prediction_frames, diagnostics = build_prediction_frames(
            tracking_frames,
            baseline_prediction_frames,
            network,
            horizons,
            snap_tracking=snap_tracking,
        )
        full = evaluate_pipeline(
            segments,
            ego_points,
            effective_tracks,
            prediction_frames,
            horizons=(1.0, 2.0, 3.0),
            maximum_range_m=args.maximum_range_m,
            maximum_association_distance_m=args.maximum_association_distance_m,
            maximum_interpolation_bracket_sec=(
                args.maximum_interpolation_bracket_sec
            ),
            prediction_motion_slice="cv-stable",
            maximum_longitudinal_acceleration_mps2=(
                args.maximum_longitudinal_acceleration_mps2
            ),
            gt_reference="bbox-center-calibrated-v1",
        )
        results[name] = {
            "configuration": {
                "lateral_convergence_time_sec": convergence_sec,
                "snap_tracking_pose": snap_tracking,
            },
            "diagnostics": diagnostics,
            "evaluation": compact_evaluation(full),
        }

    output = {
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "output_bag": str(args.output_bag.resolve()),
            "map_file": str(args.map_file.resolve()),
            "tracking_frame_count": len(tracking_frames),
            "supported_gt_segments": calibration["supported_segment_count"],
        },
        "notes": {
            "tracking_snap_from_0s": (
                "Offline-only: snap each track to its highest-scoring start "
                "Lanelet before generating all path hypotheses."
            ),
            "prediction_snap_from_0p1s": (
                "Keep the observed track at t=0 and place every map-based "
                "future sample from t=0.1 on its Lanelet centerline."
            ),
        },
        "modes": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
