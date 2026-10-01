#!/usr/bin/env python3
"""Generate the static Base3-Online highway-only 1K fine-tuning deck."""

import argparse
import copy
import json
import math
import random
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
    parser.add_argument(
        "--group", default="base3_online_highway_1k",
        help="Repository scenario/metadata/manifest group name.",
    )
    parser.add_argument(
        "--dataset-id",
        default="morai_static_classcentric_base3_10k_online_highway_1k",
    )
    parser.add_argument("--scenario-prefix", default="b3oh1k")
    parser.add_argument(
        "--portable-dataset-root", default="/datasets/base3_online_highway_v1"
    )
    parser.add_argument("--train-ratio", type=float, default=0.70)
    parser.add_argument("--val-ratio", type=float, default=0.15)
    parser.add_argument("--test-ratio", type=float, default=0.15)
    parser.add_argument(
        "--stratify-splits-by-spatial-stratum",
        action="store_true",
        help="Preserve each highway topology ratio independently in every split.",
    )
    parser.add_argument(
        "--shard-size",
        type=int,
        default=0,
        help="Also write smaller source manifests for safe MORAI installation.",
    )
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    return parser.parse_args()


def _apportioned_split_counts(total, ratios):
    names = ("train", "val", "test")
    exact = [total * ratio for ratio in ratios]
    counts = [int(value) for value in exact]
    for index in sorted(
        range(len(names)), key=lambda item: (-(exact[item] - counts[item]), item)
    )[: total - sum(counts)]:
        counts[index] += 1
    return dict(zip(names, counts))


def assign_scene_splits(output_root, group, ratios, seed, stratify=False):
    """Assign whole static clips to deterministic train/val/test splits.

    The shared generator retains its historical 70/15/15 behavior.  Campaigns
    that intentionally omit an internal test set can override that policy here
    without changing any other Base3 deck.
    """
    names = ("train", "val", "test")
    if any(value < 0.0 for value in ratios):
        raise ValueError("split ratios must be non-negative")
    ratio_sum = sum(ratios)
    if ratio_sum <= 0.0:
        raise ValueError("at least one split ratio must be positive")
    normalized = [value / ratio_sum for value in ratios]

    manifest_path = output_root / "manifests/{}.json".format(group)
    manifest = json.loads(manifest_path.read_text())
    scenes = list(manifest["scenes"])
    order = list(range(len(scenes)))
    random.Random(seed).shuffle(order)

    split_by_scene_index = {}
    strata = {"all": order}
    if stratify:
        strata = {}
        for scene_index in order:
            metadata_path = Path(scenes[scene_index]["metadata"])
            if not metadata_path.is_absolute():
                metadata_path = v2.PROJECT_ROOT / metadata_path
            metadata = json.loads(metadata_path.read_text())
            stratum = metadata.get("spatial_sampling", {}).get(
                "spatial_stratum", "unspecified"
            )
            strata.setdefault(stratum, []).append(scene_index)

    counts = dict.fromkeys(names, 0)
    stratum_counts = {}
    for stratum, scene_indices in sorted(strata.items()):
        local_counts = _apportioned_split_counts(len(scene_indices), normalized)
        stratum_counts[stratum] = local_counts
        cursor = 0
        for name in names:
            count = local_counts[name]
            for scene_index in scene_indices[cursor : cursor + count]:
                split_by_scene_index[scene_index] = name
            counts[name] += count
            cursor += count

    for scene_index, scene in enumerate(scenes):
        split = split_by_scene_index[scene_index]
        scene["split"] = split
        metadata_path = Path(scene["metadata"])
        if not metadata_path.is_absolute():
            metadata_path = v2.PROJECT_ROOT / metadata_path
        metadata = json.loads(metadata_path.read_text())
        metadata["split"] = split
        v2.write_json(metadata_path, metadata)

    manifest["scenes"] = scenes
    manifest.setdefault("generation", {})["split_policy"] = {
        "unit": "static_layout_clip",
        "seed": seed,
        "ratios": dict(zip(names, normalized)),
        "counts": counts,
        "stratified_by": "spatial_stratum" if stratify else None,
        "stratum_counts": stratum_counts,
    }
    v2.write_json(manifest_path, manifest)
    return counts


def write_manifest_shards(output_root, group, shard_size):
    if shard_size <= 0:
        return []
    manifest_path = output_root / "manifests/{}.json".format(group)
    manifest = json.loads(manifest_path.read_text())
    scenes = manifest["scenes"]
    paths = []
    for offset in range(0, len(scenes), shard_size):
        shard_number = offset // shard_size + 1
        shard = copy.deepcopy(manifest)
        shard["scenario_group"] = "{}_batch{:03d}".format(group, shard_number)
        shard["scenes"] = scenes[offset : offset + shard_size]
        shard.setdefault("generation", {})["manifest_shard"] = {
            "parent_manifest": str(manifest_path),
            "index": shard_number,
            "offset": offset,
            "scene_count": len(shard["scenes"]),
        }
        path = manifest_path.with_name(
            "{}_batch{:03d}.json".format(group, shard_number)
        )
        v2.write_json(path, shard)
        paths.append(path)
    return paths


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
    group = "".join(
        char for char in args.group.lower() if char.isalnum() or char == "_"
    )
    scenario_prefix = "".join(
        char for char in args.scenario_prefix.lower() if char.isalnum()
    )
    if not group or not scenario_prefix:
        raise ValueError("--group and --scenario-prefix must not be empty")
    if not args.dataset_id.startswith("morai_static_classcentric_base3_10k"):
        raise ValueError(
            "--dataset-id must preserve the Base3 single-frame prefix "
            "'morai_static_classcentric_base3_10k'"
        )

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
    config["dataset_id"] = args.dataset_id
    config["seed"] = args.seed
    config["scenario_prefix"] = scenario_prefix
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

    # The lower-level generator recreates metadata/<group>; keep its input
    # config in a distinct directory so cleanup cannot delete the config
    # before it is hashed into per-scene metadata.
    config_root = args.output_root / "metadata/{}_config".format(group)
    config_path = config_root / "config.json"
    v2.write_json(config_path, config)
    v2.run_generator(
        config_path=config_path,
        output_root=args.output_root,
        group=group,
        id_prefix="{}_".format(scenario_prefix),
        install_dir=None,
    )

    removed, prune_report = prune_nonlocal_runtime_actors(args.output_root, group)
    split_counts = assign_scene_splits(
        args.output_root,
        group,
        (args.train_ratio, args.val_ratio, args.test_ratio),
        args.seed + 7919,
        stratify=args.stratify_splits_by_spatial_stratum,
    )
    shard_manifests = write_manifest_shards(args.output_root, group, args.shard_size)

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
        "portable_dataset_root": args.portable_dataset_root,
        "split_counts": split_counts,
        "manifest_shards": [str(path) for path in shard_manifests],
        "maximum_runtime_actor_distance_m": MAX_RUNTIME_ACTOR_DISTANCE_M,
        "nonlocal_runtime_actor_count_removed": len(removed),
        "nonlocal_actor_prune_report": str(prune_report),
        "manifest": str(manifest),
        "runtime_manifest": str(runtime_manifest) if runtime_manifest else None,
    }
    v2.write_json(config_root / "campaign_plan.json", plan)
    print("clips/frames: {}/{}".format(args.clips, args.clips * FRAMES_PER_SCENE))
    print("families: {}".format(family_counts))
    print("splits: {}".format(split_counts))
    print("manifest: {}".format(manifest))
    if shard_manifests:
        print("manifest shards: {}".format(len(shard_manifests)))
    print("nonlocal actors removed: {} ({})".format(len(removed), prune_report))
    if runtime_manifest:
        print("runtime manifest: {}".format(runtime_manifest))


if __name__ == "__main__":
    main()
