#!/usr/bin/env python3
"""Generate the five representative Base3-Static live smoke scenes."""

import argparse
from pathlib import Path

import generate_static_classcentric_base3_batch as base3_static
import generate_temporal_sequence_v2_batch as v2


SMOKE_COUNTS = {
    "stopped_vehicle": 1,
    "static_obstacle": 1,
    "standing_pedestrian": 1,
    "realistic_mixed_static": 1,
    "sparse_background": 1,
}

SPATIAL_SMOKE_COUNTS = {
    "stopped_vehicle": 7,
    "static_obstacle": 3,
    "realistic_mixed_static": 1,
    "sparse_background": 1,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-tag", default="smoke")
    parser.add_argument("--seed", type=int, default=260828301)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    parser.add_argument("--spatial-smoke", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    tag = "".join(char for char in args.campaign_tag.lower() if char.isalnum())
    if not tag:
        raise ValueError("--campaign-tag must contain an alphanumeric character")

    counts = SPATIAL_SMOKE_COUNTS if args.spatial_smoke else SMOKE_COUNTS
    config = base3_static.build_config(
        batch_index=1,
        role="smoke",
        counts=counts,
        campaign_tag=tag,
        seed_base=args.seed - 15000,
    )
    config["dataset_id"] = "{}_smoke".format(base3_static.DATASET_ID)
    config["seed"] = args.seed
    config["scenario_prefix"] = "b3s{}".format(tag)
    for row in config["families"]:
        row["route_wide_slot_count"] = 1
        if row["name"] == "standing_pedestrian":
            row["crosswalk_scene_count"] = 1

    if args.spatial_smoke:
        config["dataset_id"] += "_spatial"
        config["scenario_prefix"] += "spatial"
        for row in config["families"]:
            row.pop("global_spatial_stratum_schedule", None)
            row["family_global_variant_offset"] = 0
            if row["name"] == "stopped_vehicle":
                row["roundabout_left_vehicle_probability"] = 1.0
                row["highway_left_vehicle_probability"] = 1.0
                row["spatial_strata"] = [
                    {"name": "roundabout", "weight": 4.0},
                    {"name": "highway_three_lane", "weight": 3.0},
                ]
                row["roundabout_phases"] = [
                    {**phase, "weight": 1.0}
                    for phase in base3_static.ROUNDABOUT_PHASES
                ]
            elif row["name"] == "static_obstacle":
                row["spatial_strata"] = [
                    {"name": "highway_three_lane", "weight": 1.0}
                ]
            else:
                row["spatial_strata"] = [
                    {"name": "roundabout", "weight": 1.0}
                ]

    group = (
        "static_classcentric_base3_spatial_smoke"
        if args.spatial_smoke
        else "static_classcentric_base3_smoke"
    )
    # The scenario generator recreates ``metadata/<group>``.  Keep its input
    # config outside that directory so regeneration cannot delete the config
    # before its provenance hash is calculated.
    config_root = args.output_root / "metadata/static_classcentric_base3_smoke_config"
    config_path = config_root / "smoke_config.json"
    v2.write_json(config_path, config)
    v2.run_generator(
        config_path=config_path,
        output_root=args.output_root,
        group=group,
        id_prefix="b3s{}_".format(tag),
        install_dir=None,
    )

    manifest = args.output_root / "manifests/{}.json".format(group)
    runtime_manifest = None
    if args.install_dir is not None:
        runtime_manifest = base3_static.install_runtime_manifest(
            manifest,
            args.install_dir,
            args.network_template,
        )
    print("Base3-Static smoke clips: {}".format(sum(counts.values())))
    print("Base3-Static smoke frames: {}".format(sum(counts.values()) * 3))
    print("manifest: {}".format(manifest))
    if runtime_manifest is not None:
        print("runtime manifest: {}".format(runtime_manifest))


if __name__ == "__main__":
    main()
