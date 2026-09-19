#!/usr/bin/env python3
"""Generate the static Base3-Online highway-only 1K fine-tuning deck."""

import argparse
import copy
import json
import math
from pathlib import Path

import generate_dynamic_object_capture_set as dynamic_generator
import generate_static_classcentric_base3_batch as base3_static
import generate_temporal_sequence_v2_batch as v2


DEFAULT_CLIPS = 334
FRAMES_PER_SCENE = 3
FAMILY_WEIGHTS = {
    "stopped_vehicle": 1.00,
    "static_obstacle": 0.00,
    "sparse_background": 0.00,
}
HIGHWAY_PHASE_WEIGHTS = {
    "highway_ramp_approach": 0.18,
    "highway_merge_lane": 0.22,
    "highway_exit_split": 0.24,
    "highway_four_to_three_taper": 0.16,
    "highway_post_taper_two_usable": 0.20,
}

# Route-relative placement can cross a disconnected highway topology branch
# when an optional rear/context actor is requested near a merge or taper.  The
# metadata then says e.g. "-20 m" while the actual MGeo pose is hundreds of
# metres away.  Such actors are outside the detector range and MORAI may reject
# them at scenario load, so keep only the physically local actor set.
MAX_RUNTIME_ACTOR_DISTANCE_M = 80.0


def apportioned_counts(total, weights):
    exact = {name: total * weight / sum(weights.values()) for name, weight in weights.items()}
    counts = {name: int(value) for name, value in exact.items()}
    for name in sorted(weights, key=lambda key: (-(exact[key] - counts[key]), key))[
        : total - sum(counts.values())
    ]:
        counts[name] += 1
    return counts


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=int, default=DEFAULT_CLIPS)
    parser.add_argument("--campaign-tag", default="onlinehighway1k")
    parser.add_argument("--seed", type=int, default=260919100)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    return parser.parse_args()


def prune_nonlocal_runtime_actors(output_root, group):
    manifest_path = output_root / "manifests/{}.json".format(group)
    manifest = json.loads(manifest_path.read_text())
    removed = []
    list_by_class = {
        "vehicle": "vehicleList",
        "pedestrian": "pedestrianList",
        "obstacle": "obstacleList",
    }

    for scene_entry in manifest["scenes"]:
        scenario_path = Path(scene_entry["scenario"])
        metadata_path = Path(scene_entry["metadata"])
        if not scenario_path.is_absolute():
            scenario_path = v2.PROJECT_ROOT / scenario_path
        if not metadata_path.is_absolute():
            metadata_path = v2.PROJECT_ROOT / metadata_path
        scenario = json.loads(scenario_path.read_text())
        metadata = json.loads(metadata_path.read_text())
        ego = metadata["ego"]["pose"]
        retained = []
        removed_ids = set()
        for actor in metadata.get("runtime_actors", []):
            position = actor["position"]
            distance = math.hypot(position["x"] - ego["x"], position["y"] - ego["y"])
            if distance <= MAX_RUNTIME_ACTOR_DISTANCE_M:
                retained.append(actor)
                continue
            removed_ids.add(int(actor["unique_id"]))
            removed.append(
                {
                    "scene_id": metadata["scene_id"],
                    "unique_id": int(actor["unique_id"]),
                    "class": actor["class"],
                    "geometry_role": actor.get("geometry_role"),
                    "distance_m": distance,
                }
            )

        if not removed_ids:
            continue
        metadata["runtime_actors"] = retained
        metadata["validation"]["unique_id_count"] = 1 + len(retained)
        for class_name, list_name in list_by_class.items():
            scenario[list_name] = [
                actor
                for actor in scenario.get(list_name, [])
                if int(actor.get("UNIQUEID", -999999)) not in removed_ids
            ]
        counts = {
            "npc": len(scenario.get("vehicleList", [])),
            "pedestrian": len(scenario.get("pedestrianList", [])),
            "obstacle": len(scenario.get("obstacleList", [])),
        }
        scene_entry["expected_counts"] = counts
        scene_entry["expected"].update(
            {
                "static_npc": counts["npc"],
                "pedestrian": counts["pedestrian"],
                "obstacle": counts["obstacle"],
            }
        )
        v2.write_json(scenario_path, scenario)
        v2.write_json(metadata_path, metadata)

    v2.write_json(manifest_path, manifest)
    report_path = output_root / "reports/{}_nonlocal_actor_prune.json".format(group)
    v2.write_json(
        report_path,
        {
            "schema_version": 1,
            "maximum_runtime_actor_distance_m": MAX_RUNTIME_ACTOR_DISTANCE_M,
            "removed_actor_count": len(removed),
            "removed": removed,
        },
    )
    return removed, report_path


def main():
    args = parse_args()
    if args.clips < 30:
        raise ValueError("--clips must be at least 30 for highway phase coverage")
    tag = "".join(char for char in args.campaign_tag.lower() if char.isalnum())
    if not tag:
        raise ValueError("--campaign-tag must contain an alphanumeric character")

    family_counts = apportioned_counts(args.clips, FAMILY_WEIGHTS)
    config = base3_static.build_config(
        batch_index=1,
        role="pilot",
        counts=family_counts,
        campaign_tag=tag,
        seed_base=args.seed - 10000,
    )
    # Preserve the Base3 single-frame metadata contract while identifying the
    # new sensor-domain specialization explicitly.
    config["dataset_id"] = "morai_static_classcentric_base3_10k_online_highway_1k"
    config["seed"] = args.seed
    config["scenario_prefix"] = "b3oh1k"
    config["capture"].update(
        {
            "captures_per_scene": FRAMES_PER_SCENE,
            "capture_spacing_m": 3.0,
            "capture_motion_arm_m": 3.0,
            "split_unit": "static_layout_clip",
        }
    )

    for row in config["families"]:
        row["ego_target_speed_kph"] = [30.0, 55.0]
        row["ego_initial_speed_kph"] = [20.0, 45.0]
        row["minimum_ego_speed_ratio"] = 0.65
        row["captures_per_scene"] = FRAMES_PER_SCENE
        row["capture_spacing_m"] = 3.0
        row["capture_motion_arm_m"] = 3.0
        row["route_wide_slot_count"] = int(row["count"])
        row["family_global_variant_offset"] = 0
        row["spatial_strata"] = [
            {"name": name, "weight": weight}
            for name, weight in HIGHWAY_PHASE_WEIGHTS.items()
        ]
        row["global_spatial_stratum_schedule"] = (
            dynamic_generator.weighted_zone_schedule(row, key="spatial_strata")
        )
        row["sampling_role"] = "base3_online_highway_{}".format(row["name"])

        if row["name"] == "static_obstacle":
            # Highway obstacle examples are a small retention slice, not a
            # dense obstacle-course surrogate.  Keep one collision-corridor
            # object and at most two contextual objects.
            row["objects_per_scene"] = [1, 3]
            row["obstacle_geometry_role_templates"] = {
                "1": ["current_lane_front"],
                "2": ["current_lane_front", "adjacent_avoidance_front"],
                "3": [
                    "current_lane_front",
                    "adjacent_avoidance_front",
                    "diagonal_context",
                ],
            }

    group = "base3_online_highway_1k"
    # The lower-level generator recreates metadata/<group>; keep its input
    # config in a distinct directory so cleanup cannot delete the config
    # before it is hashed into per-scene metadata.
    config_root = args.output_root / "metadata/base3_online_highway_1k_config"
    config_path = config_root / "config.json"
    v2.write_json(config_path, config)
    v2.run_generator(
        config_path=config_path,
        output_root=args.output_root,
        group=group,
        id_prefix="b3oh1k_",
        install_dir=None,
    )

    removed, prune_report = prune_nonlocal_runtime_actors(args.output_root, group)

    manifest = args.output_root / "manifests/{}.json".format(group)
    runtime_manifest = None
    if args.install_dir is not None:
        runtime_manifest = base3_static.install_runtime_manifest(
            manifest, args.install_dir, args.network_template
        )

    plan = {
        "schema_version": 1,
        "dataset_id": config["dataset_id"],
        "purpose": (
            "BEVFusion E25 highway Vehicle-positive online sensor-domain fine-tuning"
        ),
        "clips": args.clips,
        "frames": args.clips * FRAMES_PER_SCENE,
        "family_counts": family_counts,
        "highway_phase_weights": HIGHWAY_PHASE_WEIGHTS,
        "portable_dataset_root": "/datasets/base3_online_highway_v1",
        "maximum_runtime_actor_distance_m": MAX_RUNTIME_ACTOR_DISTANCE_M,
        "nonlocal_runtime_actor_count_removed": len(removed),
        "nonlocal_actor_prune_report": str(prune_report),
        "manifest": str(manifest),
        "runtime_manifest": str(runtime_manifest) if runtime_manifest else None,
    }
    v2.write_json(config_root / "campaign_plan.json", plan)
    print("clips/frames: {}/{}".format(args.clips, args.clips * FRAMES_PER_SCENE))
    print("families: {}".format(family_counts))
    print("manifest: {}".format(manifest))
    print("nonlocal actors removed: {} ({})".format(len(removed), prune_report))
    if runtime_manifest:
        print("runtime manifest: {}".format(runtime_manifest))


if __name__ == "__main__":
    main()
