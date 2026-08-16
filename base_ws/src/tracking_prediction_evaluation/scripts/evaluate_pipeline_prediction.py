#!/usr/bin/env python3
"""Evaluate PointPainting tracking and prediction outputs against MORAI GT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tracking_prediction_evaluation.pipeline_evaluator import (
    evaluate_pipeline,
    evaluate_oracle_predictor_at_timestamps,
    read_ego_points,
    read_output_frames,
)
from tracking_prediction_evaluation.trajectory import (
    read_morai_trajectories,
    transform_segments_planar,
)

from evaluate_oracle_prediction import load_morai_to_map_transform


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
    parser.add_argument("--horizons", nargs="+", type=float, default=(1.0, 2.0, 3.0))
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-association-distance-m", type=float, default=2.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument(
        "--prediction-motion-slice",
        choices=("all", "cv-stable"),
        default="all",
        help="Offline GT motion slice used for prediction metrics only.",
    )
    parser.add_argument(
        "--maximum-longitudinal-acceleration-mps2",
        type=float,
        default=0.5,
        help="CV-stable threshold applied over each complete future horizon.",
    )
    parser.add_argument(
        "--map-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/lanelet2_map_v60.osm"),
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
    return parser.parse_args()


def main():
    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(args.morai_static_tf)
    segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle",),
    )
    segments = transform_segments_planar(segments, translation_xyz, transform_yaw)
    ego_points = read_ego_points(
        args.raw_bag,
        args.ego_topic,
        translation_xyz,
        transform_yaw,
    )
    tracking_frames, prediction_frames = read_output_frames(
        args.output_bag,
        args.tracking_topic,
        args.prediction_topic,
    )
    summary = evaluate_pipeline(
        segments,
        ego_points,
        tracking_frames,
        prediction_frames,
        horizons=tuple(sorted(set(args.horizons))),
        maximum_range_m=args.maximum_range_m,
        maximum_association_distance_m=args.maximum_association_distance_m,
        maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
        prediction_motion_slice=args.prediction_motion_slice,
        maximum_longitudinal_acceleration_mps2=(
            args.maximum_longitudinal_acceleration_mps2
        ),
    )
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    from tracking_prediction_evaluation.models import MapBasedP1Predictor

    oracle_predictor = MapBasedP1Predictor(
        LaneletNetwork.load(args.map_file, args.projector_file)
    )
    summary["object_topic_oracle_map"] = {
        "fde_m": evaluate_oracle_predictor_at_timestamps(
            segments,
            ego_points,
            prediction_frames.keys(),
            oracle_predictor,
            horizons=tuple(sorted(set(args.horizons))),
            maximum_range_m=args.maximum_range_m,
            maximum_interpolation_bracket_sec=args.maximum_interpolation_bracket_sec,
            prediction_motion_slice=args.prediction_motion_slice,
            maximum_longitudinal_acceleration_mps2=(
                args.maximum_longitudinal_acceleration_mps2
            ),
        ),
        "mapped_origin_count": oracle_predictor.mapped_origin_count,
        "fallback_origin_count": oracle_predictor.fallback_origin_count,
    }
    summary["source"] = {
        "raw_bag": str(args.raw_bag.resolve()),
        "output_bag": str(args.output_bag.resolve()),
        "trajectory_segment_count": len(segments),
        "ego_sample_count": len(ego_points),
        "tracking_frame_count": len(tracking_frames),
        "prediction_frame_count": len(prediction_frames),
        "morai_static_tf": str(args.morai_static_tf.resolve()),
        "map_file": str(args.map_file.resolve()),
        "projector_file": str(args.projector_file.resolve()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
