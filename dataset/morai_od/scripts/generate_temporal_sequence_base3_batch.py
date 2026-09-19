#!/usr/bin/env python3
"""Generate one Base3-10K route-wide-static temporal batch and reserve."""

import argparse
import copy
from pathlib import Path

import generate_temporal_sequence_v2_batch as v2


PRIMARY_COUNTS = {
    "roundabout_circulation": 14,
    "highway_merge_cruise": 10,
    "highway_cruise_overtake": 26,
    "highway_lane_change_overtake": 4,
    "crosswalk_pedestrian": 25,
    "static_obstacle": 44,
    "static_highway_obstacle": 8,
    "realistic_mixed_static": 8,
    "sparse_background": 12,
    "standing_pedestrian": 4,
    "stopped_vehicle": 4,
    "roundabout_sample_spawn": 8,
    "highway_merge_sample_spawn": 8,
    "gps_shadow_obstacle": 6,
    "gps_shadow_vehicle_interaction": 7,
}

RESERVE_COUNTS = {
    "roundabout_circulation": 2,
    "highway_merge_cruise": 1,
    "highway_cruise_overtake": 3,
    "highway_lane_change_overtake": 1,
    "crosswalk_pedestrian": 10,
    "static_obstacle": 6,
    "static_highway_obstacle": 2,
    "realistic_mixed_static": 1,
    "sparse_background": 2,
    "standing_pedestrian": 2,
    "stopped_vehicle": 1,
    "roundabout_sample_spawn": 1,
    "highway_merge_sample_spawn": 1,
    "gps_shadow_obstacle": 1,
    "gps_shadow_vehicle_interaction": 1,
}

FAMILY_GROUPS = {
    "mission_dynamic": {
        "roundabout_circulation", "highway_merge_cruise",
        "highway_cruise_overtake", "highway_lane_change_overtake",
        "crosswalk_pedestrian",
    },
    "static_temporal": {
        "static_obstacle", "static_highway_obstacle",
        "realistic_mixed_static", "sparse_background",
        "standing_pedestrian", "stopped_vehicle",
    },
    "hard_transition": {
        "roundabout_sample_spawn", "highway_merge_sample_spawn",
        "gps_shadow_obstacle", "gps_shadow_vehicle_interaction",
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument("--campaign-tag", default="base3")
    parser.add_argument("--seed-base", type=int, default=260825000)
    parser.add_argument("--output-root", type=Path, default=v2.DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--no-reserve", action="store_true")
    return parser.parse_args()


def grouped_counts(counts):
    assigned = set().union(*FAMILY_GROUPS.values())
    if set(counts) - assigned:
        raise ValueError("unassigned Base3 families: {}".format(set(counts) - assigned))
    return {
        group: sum(int(counts.get(name, 0)) for name in names)
        for group, names in FAMILY_GROUPS.items()
    }


def base3_defaults():
    base, rows = v2.family_defaults()
    base["dataset_id"] = "morai_temporal_sequence_v2_base3_10k"
    base["use_runtime_pedestrian_pool"] = True
    # The planner path is only a sparse ordered guide.  Route-wide static
    # sampling uses native MGeo directed-link geometry for position, yaw,
    # distance, and the closed-loop connection.
    base["route_wide_route_source"] = "mgeo_guided"
    shared = {
        "spatial_policy": "route_wide_competition_route",
        "route_wide_bin_count": 24,
        "route_wide_margin_m": 0.0,
        "route_wide_closed_loop": True,
        "stable_ego_gate": True,
        # Unlike Base2 mission-local static scenes, route-wide anchors include
        # tight curves where adaptive control intentionally runs below the
        # sampled target speed.  The stability gate still prevents capture
        # during strong acceleration; this lower speed floor only avoids
        # rejecting a valid stable curve traversal.
        "minimum_ego_speed_ratio": 0.65,
        "maximum_motion_m": 90.0,
        "motion_stall_timeout_sec": 15.0,
    }
    for name in (
        "static_obstacle", "realistic_mixed_static", "sparse_background",
        "standing_pedestrian", "stopped_vehicle",
    ):
        rows[name].update(copy.deepcopy(shared))
        rows[name]["sampling_role"] = "base3_route_wide_{}".format(name)

    # Preserve the Base2 obstacle placement budget while spreading it over
    # the complete competition route.
    rows["static_obstacle"].update({
        "objects_per_scene": 6,
        # One slot per clip makes this dominant background/object family an
        # even spatial backbone instead of merely touching each coarse bin.
        "route_wide_slot_count": PRIMARY_COUNTS["static_obstacle"],
    })
    rows["realistic_mixed_static"].update({"objects_per_scene": 2})
    rows["realistic_mixed_static"]["route_wide_slot_count"] = (
        PRIMARY_COUNTS["static_obstacle"]
    )
    rows["sparse_background"].update({
        "objects_per_scene": 1,
        "route_wide_slot_count": PRIMARY_COUNTS["static_obstacle"],
    })
    rows["standing_pedestrian"].update({
        "actors_per_scene": 2,
        "maximum_crosswalk_route_distance_m": 18.0,
    })
    rows["stopped_vehicle"].update({
        "actors_per_scene": 2,
        "route_wide_slot_count": PRIMARY_COUNTS["static_obstacle"],
    })

    # Highway static coverage uses its legal three-lane MGeo paths, but with
    # a flatter lane prior and a wider longitudinal range than mission data.
    rows["static_highway_obstacle"].update({
        "spatial_policy": "route_wide_highway_three_lane",
        "ego_lane_weights": {"6": 0.45, "8": 0.35, "10": 0.20},
        "ego_start_fraction": [0.08, 0.82],
        "objects_per_scene": 5,
        "sampling_role": "base3_route_wide_highway_static_obstacle",
    })
    return base, rows


def build_config(batch_index, role, counts, campaign_tag, seed_base):
    base, defaults = base3_defaults()
    config = copy.deepcopy(base)
    config["dataset_id"] = "morai_temporal_sequence_v2_base3_10k_batch{:03d}_{}".format(
        batch_index, role
    )
    config["seed"] = int(seed_base) + batch_index * 10000 + (
        0 if role == "primary" else 5000
    )
    config["scenario_prefix"] = "t3{}b{:03d}{}".format(
        campaign_tag, batch_index, "p" if role == "primary" else "r"
    )
    config["weighted_obstacle_quota_deck"] = True
    config["families"] = []
    for name, count in counts.items():
        if count <= 0:
            continue
        row = copy.deepcopy(defaults[name])
        row["count"] = int(count)
        if name == "static_highway_obstacle" and int(count) == 8:
            # Eight clips cannot express 45/35/20 exactly.  This seven-batch
            # pattern totals 25/20/11 clips = 44.6/35.7/19.6%.
            lane_quota_patterns = (
                {"6": 3, "8": 3, "10": 2},
                {"6": 4, "8": 3, "10": 1},
                {"6": 3, "8": 3, "10": 2},
                {"6": 4, "8": 3, "10": 1},
                {"6": 4, "8": 2, "10": 2},
                {"6": 4, "8": 3, "10": 1},
                {"6": 3, "8": 3, "10": 2},
            )
            row["ego_lane_weights"] = lane_quota_patterns[
                (int(batch_index) - 1) % len(lane_quota_patterns)
            ]
        if row.get("spatial_policy", "").startswith("route_wide"):
            row["route_wide_phase_offset"] = (int(batch_index) - 1) * 5
        if name == "sparse_background":
            row["empty_scene_count"] = int(round(int(count) * 2.0 / 3.0))
        config["families"].append(row)
    return config


def main():
    args = parse_args()
    if args.batch_index < 1:
        raise ValueError("--batch-index must be positive")
    campaign_tag = "".join(char for char in args.campaign_tag.lower() if char.isalnum())
    roles = [("primary", PRIMARY_COUNTS)]
    if not args.no_reserve:
        roles.append(("reserve", RESERVE_COUNTS))
    primary_groups = grouped_counts(PRIMARY_COUNTS)
    expected = {"mission_dynamic": 79, "static_temporal": 80, "hard_transition": 29}
    if primary_groups != expected:
        raise ValueError("Base3 quota drift: {}".format(primary_groups))

    config_root = args.output_root / "metadata/temporal_base3_batches" / "batch{:03d}".format(args.batch_index)
    plan = {
        "schema_version": 1,
        "dataset_id": "morai_temporal_sequence_v2_base3_10k",
        "human_name": "Base3-10K",
        "batch_index": args.batch_index,
        "campaign_tag": campaign_tag,
        "seed_base": args.seed_base,
        "canonical_frames_per_clip": 8,
        "primary_clip_count": sum(PRIMARY_COUNTS.values()),
        "primary_frame_count": sum(PRIMARY_COUNTS.values()) * 8,
        "roles": [],
    }
    for role, counts in roles:
        config = build_config(args.batch_index, role, counts, campaign_tag, args.seed_base)
        config_path = config_root / "{}_config.json".format(role)
        v2.write_json(config_path, config)
        group = "temporal_base3_batch{:03d}_{}".format(args.batch_index, role)
        prefix = "t3{}b{:03d}{}_".format(campaign_tag, args.batch_index, "p" if role == "primary" else "r")
        v2.run_generator(config_path, args.output_root, group, prefix, args.install_dir)
        plan["roles"].append({
            "role": role,
            "group": group,
            "config": str(config_path),
            "clip_count": sum(counts.values()),
            "frame_count": sum(counts.values()) * 8,
            "family_clip_counts": counts,
            "group_clip_counts": grouped_counts(counts),
        })
    plan_path = config_root / "batch_plan.json"
    v2.write_json(plan_path, plan)
    print("Base3 primary clips: {}".format(plan["primary_clip_count"]))
    print("Base3 primary frames: {}".format(plan["primary_frame_count"]))
    print("plan: {}".format(plan_path))


if __name__ == "__main__":
    main()
