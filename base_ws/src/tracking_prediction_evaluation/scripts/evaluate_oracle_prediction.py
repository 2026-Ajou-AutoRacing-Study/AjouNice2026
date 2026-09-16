#!/usr/bin/env python3
"""Evaluate CTRV/CTRA using causal MORAI GT history (E0 oracle benchmark)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.evaluator import (
    EvaluationConfig,
    build_summary,
    evaluate_segments,
    write_report,
)
from tracking_prediction_evaluation.models import MapBasedP1Predictor, default_predictors
from tracking_prediction_evaluation.static_transform import (
    load_morai_to_map_transform,
)
from tracking_prediction_evaluation.trajectory import (
    read_morai_trajectories,
    transform_segments_planar,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/Object_topic")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--classes",
        nargs="+",
        choices=("vehicle", "pedestrian", "obstacle"),
        default=("vehicle", "pedestrian"),
    )
    parser.add_argument(
        "--horizons", nargs="+", type=float, default=(0.5, 1.0, 1.5, 2.0, 3.0)
    )
    parser.add_argument("--evaluation-rate-hz", type=float, default=10.0)
    parser.add_argument("--history-sec", type=float, default=0.5)
    parser.add_argument("--trajectory-step-sec", type=float, default=0.1)
    parser.add_argument("--maximum-assignment-distance-m", type=float, default=2.0)
    parser.add_argument("--maximum-gap-sec", type=float, default=0.5)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument("--worst-sample-count", type=int, default=100)
    parser.add_argument(
        "--include-map-based",
        action="store_true",
        help="also evaluate the current Lanelet2 Map P1 top-1 path",
    )
    parser.add_argument(
        "--map-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/lanelet2_map_v72.osm"),
    )
    parser.add_argument(
        "--projector-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/map_projector_info_v1.yaml"),
    )
    parser.add_argument(
        "--object-frame",
        choices=("morai_map", "map"),
        default="morai_map",
        help="coordinate frame used by /Object_topic in the input bag",
    )
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
        help="map->morai_map static TF used to transform raw MORAI GT",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = EvaluationConfig(
        horizons=tuple(sorted(set(args.horizons))),
        trajectory_step_sec=args.trajectory_step_sec,
        evaluation_rate_hz=args.evaluation_rate_hz,
        history_sec=args.history_sec,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
    )
    segments = read_morai_trajectories(
        args.bag,
        topic=args.topic,
        include_classes=args.classes,
        maximum_assignment_distance_m=args.maximum_assignment_distance_m,
        maximum_gap_sec=args.maximum_gap_sec,
    )
    transform_source = None
    if args.object_frame == "morai_map":
        translation_xyz, transform_yaw = load_morai_to_map_transform(
            args.morai_static_tf
        )
        segments = transform_segments_planar(
            segments, translation_xyz, transform_yaw
        )
        transform_source = str(args.morai_static_tf.resolve())
    predictors = default_predictors()
    map_predictor = None
    if args.include_map_based:
        from perception_object_pipeline.map_based_prediction import LaneletNetwork

        network = LaneletNetwork.load(args.map_file, args.projector_file)
        map_predictor = MapBasedP1Predictor(network)
        predictors.append(map_predictor)
    samples = evaluate_segments(segments, predictors, config)
    summary = build_summary(
        samples,
        config,
        source={
            "bag": str(args.bag.resolve()),
            "topic": args.topic,
            "classes": list(args.classes),
            "trajectory_segment_count": len(segments),
            "object_input_frame": args.object_frame,
            "evaluation_frame": "map",
            "morai_static_tf": transform_source,
            "models": [predictor.name for predictor in predictors],
            "map_file": str(args.map_file.resolve()) if map_predictor else None,
            "projector_file": (
                str(args.projector_file.resolve()) if map_predictor else None
            ),
            "map_sha256": map_predictor.network.map_sha256 if map_predictor else None,
            "map_top1_only": bool(map_predictor),
            "map_mapped_origin_count": (
                map_predictor.mapped_origin_count if map_predictor else 0
            ),
            "map_fallback_origin_count": (
                map_predictor.fallback_origin_count if map_predictor else 0
            ),
        },
    )
    summary_path, worst_path = write_report(
        args.output_dir,
        summary,
        samples,
        worst_sample_count=args.worst_sample_count,
    )
    print(json.dumps({
        "evaluation": summary["evaluation"],
        "segments": len(segments),
        "samples": len(samples),
        "summary": str(summary_path),
        "worst_samples": str(worst_path),
        "paired_comparison": summary["paired_comparison"],
    }, indent=2))


if __name__ == "__main__":
    main()
