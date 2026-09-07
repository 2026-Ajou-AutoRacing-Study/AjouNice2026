#!/usr/bin/env python3
"""Generate a balanced Base3-Static placement-stability live pilot."""

import argparse
from pathlib import Path

import generate_static_classcentric_base3_batch as base3_static
import generate_temporal_sequence_v2_batch as v2


PILOT_COUNTS = {
    "stopped_vehicle": 48,
    "static_obstacle": 36,
    "standing_pedestrian": 18,
    "sparse_background": 18,
}

PILOT_WEIGHTS = {
    "stopped_vehicle": 0.40,
    "static_obstacle": 0.30,
    "standing_pedestrian": 0.15,
    "sparse_background": 0.15,
}

# Exercise every topology correction before generating the final B5--B7
# campaign.  These are pilot-only distributions; the final batch generator
# keeps its own exact corrective quota table.
PILOT_SPATIAL_WEIGHTS = {
    "stopped_vehicle": {
        "highway_ramp_approach": 2,
        "highway_merge_lane": 4,
        # Eight samples are enough for the low-share exit-branch Ego variant
        # to appear at least once in the 120-clip topology pilot.
        "highway_exit_split": 8,
        "highway_four_to_three_taper": 4,
        "highway_post_taper_two_usable": 6,
        "route_general_two_lane_right": 5,
        "route_general_two_lane_left": 2,
        "roundabout": 8,
        "route_general": 9,
    },
    "static_obstacle": {
        "highway_ramp_approach": 1,
        "highway_merge_lane": 2,
        "highway_exit_split": 3,
        "highway_four_to_three_taper": 3,
        "highway_post_taper_two_usable": 3,
        "route_general_two_lane_right": 4,
        "route_general_two_lane_left": 2,
        "route_general": 18,
    },
    "sparse_background": {
        "highway_ramp_approach": 1,
        "highway_merge_lane": 1,
        "highway_exit_split": 1,
        "highway_four_to_three_taper": 1,
        "highway_post_taper_two_usable": 2,
        "route_general_two_lane_right": 1,
        "route_general_two_lane_left": 1,
        "roundabout": 2,
        "route_general": 8,
    },
}


def balanced_pilot_counts(total_clips):
    if total_clips < len(PILOT_WEIGHTS):
        raise ValueError(
            "--pilot-clips must be at least {}".format(len(PILOT_WEIGHTS))
        )
    raw = {
        name: total_clips * weight
        for name, weight in PILOT_WEIGHTS.items()
    }
    counts = {name: int(value) for name, value in raw.items()}
    remainder = total_clips - sum(counts.values())
    order = list(PILOT_WEIGHTS)
    ranked = sorted(
        PILOT_WEIGHTS,
        key=lambda name: (
            -(raw[name] - counts[name]),
            order.index(name),
        ),
    )
    for name in ranked[:remainder]:
        counts[name] += 1
    return counts


def scaled_spatial_counts(family, total):
    """Scale the 120-clip audit deck while retaining rare topology cases."""

    weights = PILOT_SPATIAL_WEIGHTS.get(family)
    if not weights:
        return None
    total = int(total)
    names = list(weights)
    if total < len(names):
        # Small smoke decks prioritize every missing highway phase first.
        names = names[:total]
        return {name: 1 for name in names}
    weight_sum = sum(weights.values())
    exact = {name: total * value / weight_sum for name, value in weights.items()}
    counts = {name: max(1, int(value)) for name, value in exact.items()}
    while sum(counts.values()) > total:
        candidates = [name for name in names if counts[name] > 1]
        name = max(candidates, key=lambda item: (counts[item] - exact[item], item))
        counts[name] -= 1
    while sum(counts.values()) < total:
        name = max(names, key=lambda item: (exact[item] - counts[item], item))
        counts[name] += 1
    return counts


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-tag", default="safety1")
    parser.add_argument("--seed", type=int, default=260829401)
    parser.add_argument("--pilot-clips", type=int, default=120)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--network-template", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    tag = "".join(char for char in args.campaign_tag.lower() if char.isalnum())
    if not tag:
        raise ValueError("--campaign-tag must contain an alphanumeric character")
    pilot_counts = balanced_pilot_counts(args.pilot_clips)

    config = base3_static.build_config(
        batch_index=1,
        role="pilot",
        counts=pilot_counts,
        campaign_tag=tag,
        seed_base=args.seed - 15000,
    )
    config["dataset_id"] = "{}_balanced_pilot{}".format(
        base3_static.DATASET_ID, args.pilot_clips
    )
    config["seed"] = args.seed
    config["scenario_prefix"] = "b3s{}pilot".format(tag)
    for row in config["families"]:
        row.pop("global_spatial_stratum_schedule", None)
        row["family_global_variant_offset"] = 0
        row["route_wide_slot_count"] = int(row["count"])
        spatial_counts = scaled_spatial_counts(row["name"], row["count"])
        if spatial_counts:
            row["spatial_strata"] = [
                {"name": name, "weight": count}
                for name, count in spatial_counts.items()
            ]
            row["global_spatial_stratum_schedule"] = (
                base3_static.exact_spatial_schedule(spatial_counts)
            )
        if row["name"] == "standing_pedestrian":
            row["crosswalk_scene_count"] = int(
                round(int(row["count"]) * 0.65)
            )

    group = "static_classcentric_base3_balanced_pilot{}".format(
        args.pilot_clips
    )
    config_root = (
        args.output_root
        / "metadata/static_classcentric_base3_pilot_config"
    )
    config_path = config_root / "pilot{}_config.json".format(args.pilot_clips)
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
    print("Base3-Static pilot clips: {}".format(sum(pilot_counts.values())))
    print("Base3-Static pilot counts: {}".format(pilot_counts))
    print("Base3-Static pilot frames: {}".format(sum(pilot_counts.values()) * 3))
    print("manifest: {}".format(manifest))
    if runtime_manifest is not None:
        print("runtime manifest: {}".format(runtime_manifest))


if __name__ == "__main__":
    main()
