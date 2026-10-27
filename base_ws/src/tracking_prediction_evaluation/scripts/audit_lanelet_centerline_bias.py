#!/usr/bin/env python3
"""Audit fixed-vector bias among MORAI NPC paths, MGeo, and Lanelet2."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    MapPredictionConfig,
)
from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.lanelet_bias import (
    project_samples_to_lanelets,
    read_mgeo_samples,
    select_samples_near_trajectories,
    trajectory_samples,
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
    parser.add_argument("raw_bags", nargs="+", type=Path)
    parser.add_argument(
        "--map-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/lanelet2_map_v97.osm"),
    )
    parser.add_argument(
        "--projector-file",
        type=Path,
        default=Path("kcity_map/lanelet_map/map_projector_info_v1.yaml"),
    )
    parser.add_argument(
        "--mgeo-link-file",
        type=Path,
        default=Path("kcity_map/mgeo_map/link_set.json"),
    )
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    parser.add_argument("--trajectory-stride", type=int, default=10)
    parser.add_argument("--mgeo-stride", type=int, default=4)
    parser.add_argument("--relevant-radius-m", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def runtime_origin_transform(static_tf_path):
    document = yaml.safe_load(static_tf_path.read_text(encoding="utf-8"))
    transform = document["map_to_morai_map"]
    source = transform.get("source") or {}
    xyz = source.get("morai_runtime_origin_delta_xyz")
    if not xyz or len(xyz) != 3:
        raise ValueError("static TF lacks morai_runtime_origin_delta_xyz")
    return tuple(float(value) for value in xyz), 0.0


def evaluate_samples(network, samples):
    _, summary = project_samples_to_lanelets(network, samples)
    return summary


def main():
    args = parse_args()
    configured_xyz, configured_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    runtime_xyz, runtime_yaw = runtime_origin_transform(args.morai_static_tf)
    config = MapPredictionConfig(
        maximum_lane_distance_m=3.0,
        maximum_lane_yaw_error_rad=0.7853981633974483,
        maximum_start_lanelets=1,
    )
    network = LaneletNetwork.load(args.map_file, args.projector_file, config=config)

    scenes = {}
    combined_actor_segments = []
    for raw_bag in args.raw_bags:
        actor_segments_local = read_morai_trajectories(
            raw_bag,
            include_classes=("vehicle",),
        )
        bbox_segments_local, calibration = (
            transform_segments_to_calibrated_bbox_center(actor_segments_local)
        )
        actor_segments = transform_segments_planar(
            actor_segments_local, configured_xyz, configured_yaw
        )
        bbox_segments = transform_segments_planar(
            bbox_segments_local, configured_xyz, configured_yaw
        )
        actor_segments_runtime_origin = transform_segments_planar(
            actor_segments_local, runtime_xyz, runtime_yaw
        )
        bbox_segments_runtime_origin = transform_segments_planar(
            bbox_segments_local, runtime_xyz, runtime_yaw
        )
        combined_actor_segments.extend(actor_segments)
        scene_key = raw_bag.stem
        scenes[scene_key] = {
            "raw_bag": str(raw_bag.resolve()),
            "actor_reference_to_lanelet": evaluate_samples(
                network,
                trajectory_samples(actor_segments, args.trajectory_stride),
            ),
            "bbox_center_to_lanelet": evaluate_samples(
                network,
                trajectory_samples(bbox_segments, args.trajectory_stride),
            ),
            "actor_reference_to_lanelet_runtime_origin_only": evaluate_samples(
                network,
                trajectory_samples(
                    actor_segments_runtime_origin, args.trajectory_stride
                ),
            ),
            "bbox_center_to_lanelet_runtime_origin_only": evaluate_samples(
                network,
                trajectory_samples(
                    bbox_segments_runtime_origin, args.trajectory_stride
                ),
            ),
            "bbox_reference_calibration": calibration,
        }

    configured_mgeo = read_mgeo_samples(
        args.mgeo_link_file,
        configured_xyz,
        configured_yaw,
        args.mgeo_stride,
    )
    runtime_mgeo = read_mgeo_samples(
        args.mgeo_link_file,
        runtime_xyz,
        runtime_yaw,
        args.mgeo_stride,
    )
    configured_relevant = select_samples_near_trajectories(
        configured_mgeo,
        combined_actor_segments,
        args.relevant_radius_m,
    )
    runtime_relevant = select_samples_near_trajectories(
        runtime_mgeo,
        combined_actor_segments,
        args.relevant_radius_m,
    )
    summary = {
        "schema_version": 1,
        "purpose": (
            "Separate a global XY map-frame translation bias from ordinary "
            "lane-relative vehicle offset."
        ),
        "configured_transform": {
            "xyz": list(configured_xyz),
            "yaw_rad": configured_yaw,
        },
        "runtime_origin_only_transform": {
            "xyz": list(runtime_xyz),
            "yaw_rad": runtime_yaw,
        },
        "configured_minus_runtime_origin_xy_m": [
            configured_xyz[0] - runtime_xyz[0],
            configured_xyz[1] - runtime_xyz[1],
        ],
        "scenes": scenes,
        "mgeo_to_lanelet": {
            "configured_transform_all_map": evaluate_samples(
                network, configured_mgeo
            ),
            "configured_transform_bag_region": evaluate_samples(
                network, configured_relevant
            ),
            "runtime_origin_only_bag_region": evaluate_samples(
                network, runtime_relevant
            ),
            "bag_region_radius_m": args.relevant_radius_m,
        },
        "sources": {
            "lanelet_map": str(args.map_file.resolve()),
            "projector": str(args.projector_file.resolve()),
            "mgeo_link_file": str(args.mgeo_link_file.resolve()),
            "static_tf": str(args.morai_static_tf.resolve()),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "configured_minus_runtime_origin_xy_m": summary[
                    "configured_minus_runtime_origin_xy_m"
                ],
                "configured_mgeo_bag_region": summary["mgeo_to_lanelet"][
                    "configured_transform_bag_region"
                ]["global_correction_mean_vector_xy_m"],
                "runtime_origin_only_mgeo_bag_region": summary[
                    "mgeo_to_lanelet"
                ]["runtime_origin_only_bag_region"][
                    "global_correction_mean_vector_xy_m"
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
