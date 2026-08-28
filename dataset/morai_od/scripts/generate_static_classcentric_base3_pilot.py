#!/usr/bin/env python3
"""Generate a 120-scene Base3-Static placement-stability live pilot."""

import argparse
from pathlib import Path

import generate_static_classcentric_base3_batch as base3_static
import generate_temporal_sequence_v2_batch as v2


PILOT_COUNTS = {
    "stopped_vehicle": 108,
    "static_obstacle": 3,
    "standing_pedestrian": 3,
    "realistic_mixed_static": 3,
    "sparse_background": 3,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-tag", default="safety1")
    parser.add_argument("--seed", type=int, default=260829401)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    tag = "".join(char for char in args.campaign_tag.lower() if char.isalnum())
    if not tag:
        raise ValueError("--campaign-tag must contain an alphanumeric character")

    config = base3_static.build_config(
        batch_index=1,
        role="pilot",
        counts=PILOT_COUNTS,
        campaign_tag=tag,
        seed_base=args.seed - 15000,
    )
    config["dataset_id"] = "{}_vehicle_pilot120".format(
        base3_static.DATASET_ID
    )
    config["seed"] = args.seed
    config["scenario_prefix"] = "b3s{}pilot".format(tag)
    for row in config["families"]:
        row.pop("global_spatial_stratum_schedule", None)
        row["family_global_variant_offset"] = 0
        row["route_wide_slot_count"] = int(row["count"])
        if row["name"] == "standing_pedestrian":
            row["crosswalk_scene_count"] = min(2, int(row["count"]))

    group = "static_classcentric_base3_vehicle_pilot120"
    config_root = (
        args.output_root
        / "metadata/static_classcentric_base3_pilot_config"
    )
    config_path = config_root / "pilot120_config.json"
    v2.write_json(config_path, config)
    v2.run_generator(
        config_path=config_path,
        output_root=args.output_root,
        group=group,
        id_prefix="b3s{}pilot_".format(tag),
        install_dir=None,
    )

    manifest = args.output_root / "manifests/{}.json".format(group)
    runtime_manifest = None
    if args.install_dir is not None:
        runtime_manifest = base3_static.install_runtime_manifest(
            manifest, args.install_dir, args.network_template
        )
    print("Base3-Static pilot clips: {}".format(sum(PILOT_COUNTS.values())))
    print("Base3-Static pilot frames: {}".format(sum(PILOT_COUNTS.values()) * 3))
    print("manifest: {}".format(manifest))
    if runtime_manifest is not None:
        print("runtime manifest: {}".format(runtime_manifest))


if __name__ == "__main__":
    main()
