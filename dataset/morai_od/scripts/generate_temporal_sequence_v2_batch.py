#!/usr/bin/env python3
"""Generate one 1,504-frame Temporal V2 baseline batch plus a reserve."""

import argparse
import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
BASE_CONFIG = DATASET_ROOT / "configs/temporal_sequence_capture_v2_pilot304.json"
GENERATOR = DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py"

PRIMARY_COUNTS = {
    # Mission dynamic: 79 clips / 632 frames (42.0%).  Relative-motion roles
    # are consolidated into the live-validated highway family instead of a
    # separate east-corridor-only production quota.
    "roundabout_circulation": 14,
    "highway_merge_cruise": 10,
    "highway_cruise_overtake": 26,
    "highway_lane_change_overtake": 4,
    "crosswalk_pedestrian": 25,
    # Static temporal: 80 clips / 640 frames (42.6%).  Explicit obstacle
    # families are 53/80=66.3%; the remaining clips broaden route background
    # coverage and teach zero-velocity static examples.
    "static_obstacle": 41,
    "static_highway_obstacle": 12,
    "stopped_vehicle": 8,
    "standing_pedestrian": 5,
    "realistic_mixed_static": 7,
    "sparse_background": 7,
    # Hard transitions: 29 clips / 232 frames (15.4%).
    "roundabout_sample_spawn": 8,
    "highway_merge_sample_spawn": 8,
    "gps_shadow_obstacle": 6,
    "gps_shadow_vehicle_interaction": 7,
}

RESERVE_COUNTS = {
    "roundabout_sample_spawn": 1,
    "roundabout_circulation": 1,
    "highway_merge_sample_spawn": 1,
    "highway_merge_cruise": 1,
    "highway_cruise_overtake": 3,
    "highway_lane_change_overtake": 1,
    # Ten reserve clips cover every configured crosswalk ID.  A reserve from
    # another physical location must not replace a failed location-disjoint
    # pedestrian clip.
    "crosswalk_pedestrian": 10,
    "static_obstacle": 5,
    "static_highway_obstacle": 1,
    "stopped_vehicle": 1,
    "standing_pedestrian": 1,
    "realistic_mixed_static": 0,
    "sparse_background": 1,
    "gps_shadow_obstacle": 0,
    "gps_shadow_vehicle_interaction": 1,
}

FAMILY_GROUPS = {
    "mission_dynamic": {
        "roundabout_circulation",
        "highway_merge_cruise",
        "highway_cruise_overtake",
        "highway_lane_change_overtake",
        "crosswalk_pedestrian",
    },
    "static_temporal": {
        "static_obstacle",
        "static_highway_obstacle",
        "stopped_vehicle",
        "standing_pedestrian",
        "realistic_mixed_static",
        "sparse_background",
    },
    "hard_transition": {
        "roundabout_sample_spawn",
        "highway_merge_sample_spawn",
        "gps_shadow_obstacle",
        "gps_shadow_vehicle_interaction",
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument(
        "--campaign-tag",
        default="base1",
        help="Short ID namespace for a clean production regeneration (default: base1).",
    )
    parser.add_argument(
        "--seed-base",
        type=int,
        default=260824000,
        help="Campaign seed base; batch and role offsets are added automatically.",
    )
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    parser.add_argument("--no-reserve", action="store_true")
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def family_defaults():
    base = load_json(BASE_CONFIG)
    rows = {row["name"]: copy.deepcopy(row) for row in base["families"]}
    # These are quota strata, not fixed points. Each scene samples a continuous
    # route-index range inside its assigned phase, then starts Ego far enough
    # upstream to reach the requested speed near that capture anchor.
    roundabout_zones = [
        {"name": "approach", "route_index_range": [1600, 1640], "weight": 0.15},
        {"name": "merge_entry", "route_index_range": [1641, 1690], "weight": 0.35},
        {"name": "circulation", "route_index_range": [1691, 1740], "weight": 0.40},
        # Preserve a small amount of rear-camera/leave-behind coverage without
        # spending four of eighteen mission clips after Ego has already left.
        {"name": "exit", "route_index_range": [1741, 1780], "weight": 0.10},
    ]
    highway_merge_zones = [
        {"name": "approach", "route_index_range": [1940, 1995], "weight": 0.25},
        {"name": "merge", "route_index_range": [1996, 2060], "weight": 0.40},
        {"name": "settled", "route_index_range": [2061, 2130], "weight": 0.35},
    ]
    gps_shadow_zones = [
        {"name": "approach_entry", "route_index_range": [3610, 3690], "weight": 0.20},
        {"name": "early_shadow", "route_index_range": [3691, 3760], "weight": 0.25},
        {"name": "deep_shadow", "route_index_range": [3761, 3840], "weight": 0.30},
        {"name": "exit_recovery", "route_index_range": [3841, 3940], "weight": 0.25},
    ]
    for name in ("roundabout_sample_spawn", "roundabout_circulation"):
        rows[name]["ego_capture_zones"] = copy.deepcopy(roundabout_zones)
        rows[name]["capture_preroll_distance_m"] = [8.0, 12.0]
        rows[name]["maximum_motion_m"] = 110.0
        rows[name]["motion_stall_timeout_sec"] = 16.0
    rows["roundabout_circulation"].update(
        {
            "maximum_spawn_vehicles": 1,
            "spawn_wait_sec": 0.0,
            "spawn_period_sec": [4.0, 5.0],
            "backup_roundabout_spawn": True,
            "backup_roundabout_spawn_unique_id": 104,
            "backup_roundabout_spawn_offset_m": 28.0,
            "maximum_ego_speed_over_npc_kph": 2.0,
            "npc_phase_offset_candidates_m": [-45.0, -40.0, -35.0, -30.0],
            "npc_relative_distance_m": [24.0, 38.0],
            "minimum_npc_spawn_distance_m": 18.0,
            "reposition_roundabout_spawn": True,
            "capture_actor_gate": {
                "class": "vehicle",
                "role": "roundabout_cruising_vehicle",
                "minimum_longitudinal_m": -45.0,
                "maximum_longitudinal_m": 45.0,
                "maximum_abs_lateral_m": 35.0,
                "minimum_distance_m": 5.0,
                "maximum_distance_m": 55.0,
            },
        }
    )
    # Keep the official zero-speed spawn as a separate hard-transition family.
    # It must not inherit the cruising family's relocated, target-speed spawn.
    rows["roundabout_sample_spawn"].update(
        {
            "maximum_spawn_vehicles": 1,
            "reposition_roundabout_spawn": False,
        }
    )
    for name in ("highway_merge_sample_spawn", "highway_merge_cruise"):
        rows[name]["ego_capture_zones"] = copy.deepcopy(highway_merge_zones)
        rows[name]["capture_preroll_distance_m"] = [12.0, 22.0]
        rows[name]["maximum_motion_m"] = 110.0
        rows[name]["motion_stall_timeout_sec"] = 16.0
    rows["gps_shadow_vehicle_interaction"] = {
        "name": "gps_shadow_vehicle_interaction",
        "source_spawn_ids": [4],
        "target_speed_kph": [10.0, 20.0],
        "spawn_period_sec": [8.0, 12.0],
        # ScenarioLoad and Ego acceleration take several seconds. Starting at
        # target speed lets the NPC leave the interaction corridor before
        # frame 0, so use the template's zero spawn speed and accelerate.
        "spawn_velocity_equals_target": False,
        "ego_capture_zones": gps_shadow_zones,
        "capture_preroll_distance_m": [3.0, 6.0],
        "npc_link_path": ["A2256W000126", "A2256W000128"],
        "npc_phase_offset_candidates_m": [6.0, 8.0, 10.0, 12.0],
        "npc_relative_distance_m": [6.0, 12.0],
        "minimum_npc_spawn_distance_m": 6.0,
        "ego_target_speed_kph": [20.0, 35.0],
        "ego_initial_speed_kph": [12.0, 24.0],
        "minimum_ego_speed_ratio": 0.45,
        "maximum_motion_m": 100.0,
        "motion_stall_timeout_sec": 14.0,
        "motion_phase": "gps_shadow_same_lane_interaction",
        "sampling_role": "hard_transition",
        "stable_ego_gate": False,
    }
    rows["stopped_vehicle"] = {
        "name": "stopped_vehicle",
        "actors_per_scene": 2,
        "scale_factor": 1.0,
        "ego_lead_distance_m": [20.0, 30.0],
        "ego_target_speed_kph": [20.0, 40.0],
        "ego_initial_speed_kph": [10.0, 25.0],
        "minimum_ego_speed_ratio": 0.85,
        "motion_phase": "static_temporal",
        "sampling_role": "static_stopped_vehicle",
        "stable_ego_gate": True,
    }
    rows["static_highway_obstacle"] = {
        "name": "static_highway_obstacle",
        "objects_per_scene": 5,
        "ego_start_arc_m": [60.0, 220.0],
        "ego_target_speed_kph": [35.0, 55.0],
        "ego_initial_speed_kph": [25.0, 42.0],
        "minimum_ego_speed_ratio": 0.85,
        "capture_motion_arm_m": 10.0,
        "minimum_capture_progress_m": 0.3,
        "motion_phase": "static_highway_temporal",
        "sampling_role": "static_highway_weighted_lane",
        "stable_ego_gate": True,
    }
    rows["standing_pedestrian"] = {
        "name": "standing_pedestrian",
        "actors_per_scene": 2,
        "scale_factor": 1.0,
        "ego_lead_distance_m": [16.0, 26.0],
        "ego_target_speed_kph": [10.0, 25.0],
        "ego_initial_speed_kph": [6.0, 15.0],
        "minimum_ego_speed_ratio": 0.8,
        "motion_phase": "static_temporal",
        "sampling_role": "static_standing_pedestrian",
        "stable_ego_gate": True,
    }
    rows["realistic_mixed_static"] = {
        "name": "realistic_mixed_static",
        "objects_per_scene": 2,
        "scale_factor": [0.9, 1.1],
        "ego_lead_distance_m": [20.0, 30.0],
        "ego_target_speed_kph": [20.0, 40.0],
        "ego_initial_speed_kph": [10.0, 25.0],
        "minimum_ego_speed_ratio": 0.85,
        "motion_phase": "static_temporal",
        "sampling_role": "static_realistic_mixed",
        "stable_ego_gate": True,
    }
    rows["sparse_background"] = {
        "name": "sparse_background",
        "objects_per_scene": 1,
        "scale_factor": [0.9, 1.1],
        "ego_lead_distance_m": [24.0, 34.0],
        "ego_target_speed_kph": [30.0, 45.0],
        "ego_initial_speed_kph": [20.0, 30.0],
        "minimum_ego_speed_ratio": 0.85,
        "motion_phase": "stable_background",
        "sampling_role": "static_sparse_background",
        "stable_ego_gate": True,
    }
    # These families represent traffic that was already cruising before Ego
    # entered sensor range. The official spawn/acceleration cases stay in the
    # separate highway_merge_sample_spawn hard-transition family.
    rows["highway_cruise_overtake"].update(
        {
            "maximum_spawn_vehicles": 1,
            "start_ego_at_target_speed": True,
            "capture_motion_arm_m": 3.0,
            "interaction_roles": [
                {
                    "name": "front_overtake",
                    "weight": 12,
                    "npc_longitudinal_offset_m": [18.0, 38.0],
                    "gate_longitudinal_m": [8.0, 48.0],
                    "maximum_abs_lateral_m": 6.5,
                },
                {
                    "name": "front_following",
                    "weight": 4,
                    "npc_longitudinal_offset_m": [15.0, 35.0],
                    "gate_longitudinal_m": [6.0, 45.0],
                    "maximum_abs_lateral_m": 2.8,
                    "ego_speed_delta_kph": [-3.0, 3.0],
                },
                {
                    "name": "side_parallel",
                    "weight": 4,
                    "npc_longitudinal_offset_m": [-8.0, 12.0],
                    "gate_longitudinal_m": [-18.0, 22.0],
                    "maximum_abs_lateral_m": 6.5,
                    "npc_speed_delta_kph": [-5.0, 5.0],
                },
                {
                    "name": "rear_receding",
                    "weight": 3,
                    "npc_longitudinal_offset_m": [-35.0, -15.0],
                    "gate_longitudinal_m": [-45.0, -6.0],
                    "maximum_abs_lateral_m": 3.0,
                },
                {
                    "name": "rear_approach",
                    "weight": 3,
                    "npc_longitudinal_offset_m": [-35.0, -15.0],
                    "gate_longitudinal_m": [-45.0, -6.0],
                    "maximum_abs_lateral_m": 3.0,
                    "npc_speed_advantage_kph": [5.0, 10.0],
                },
            ],
        }
    )
    rows["highway_merge_cruise"].update(
        {
            "maximum_spawn_vehicles": 1,
            "spawn_wait_sec": 3.0,
            "start_ego_at_target_speed": False,
            "capture_preroll_distance_m": [8.0, 14.0],
            "minimum_ego_speed_ratio": 0.70,
            "stable_ego_gate": False,
            "maximum_motion_m": 130.0,
            "interaction_roles": [
                {
                    "name": "merge_interacting_traffic",
                    "weight": 1,
                    "npc_longitudinal_offset_m": [-50.0, -35.0],
                    # The three MORAI highway lane paths do not have identical
                    # arc origins.  The same route-relative spawn offset can
                    # therefore reach the capture point slightly ahead or
                    # behind Ego.  Both are safety-relevant merge traffic; the
                    # distance gate keeps the actor close instead of forcing a
                    # brittle front-only sign convention.
                    "gate_longitudinal_m": [-25.0, 60.0],
                    "maximum_abs_lateral_m": 22.0,
                    "minimum_distance_m": 8.0,
                    "maximum_distance_m": 65.0,
                },
            ],
        }
    )
    rows["highway_lane_change_overtake"].update(
        {
            "maximum_spawn_vehicles": 1,
            "spawn_wait_sec": 4.0,
            "pre_length_m": [80.0, 100.0],
            "minimum_ego_speed_ratio": 0.70,
            "minimum_curve_speed_kph": 30.0,
            "route_deviation_limit_m": 2.0,
            # Estimate spawn position from the sampled Ego/NPC speeds and the
            # selected capture phase instead of fixed lane-pair offsets.
            "lane_change_ego_speed_advantage_kph": [2.0, 5.0],
            "interaction_roles": [
                {
                    "name": "front_target_lane",
                    "weight": 2,
                    "npc_longitudinal_offset_m": [-105.0, -75.0],
                    "gate_longitudinal_m": [5.0, 42.0],
                    "desired_capture_longitudinal_m": [16.0, 28.0],
                },
                {
                    "name": "side_target_lane",
                    "weight": 2,
                    "npc_longitudinal_offset_m": [-135.0, -110.0],
                    "gate_longitudinal_m": [-16.0, 20.0],
                    # Do not centre an NPC beside Ego while Ego crosses into
                    # its lane.  The two side-target clips cover safe rear and
                    # front interaction gaps instead.  MORAI starts the NPC
                    # while the loader is still waiting for actor readiness;
                    # the rear target therefore includes roughly 20--30 m of
                    # head-start compensation before Ego control begins.
                    "desired_capture_longitudinal_ranges_m": [
                        [-42.0, -34.0],
                        [12.0, 18.0],
                    ],
                },
            ],
        }
    )
    rows["east_corridor_relative_vehicle"] = {
        "name": "east_corridor_relative_vehicle",
        # Spawn 10 did not activate reliably after arbitrary legal-link
        # relocation in live MORAI tests.  Spawns 4/6/8 retain three vehicle
        # length/type pools and were stable under the same relocation.
        "source_spawn_ids": [4, 6, 8],
        # A relocated MORAI spawn can leave its short generated path before
        # Ego reaches the planned interaction.  Permit one replacement spawn
        # rather than converting the clip into an empty road sample.
        "spawn_period_sec": [4.0, 6.0],
        "spawn_wait_sec": 2.0,
        "maximum_spawn_vehicles": 2,
        "ego_target_speed_kph": [25.0, 45.0],
        # Ego remains stopped during ScenarioLoad verification.  Giving it an
        # initial cruise speed made a slow ROS reconnect move the expected
        # pose by tens of metres before readiness could be checked.
        "start_ego_at_target_speed": False,
        "minimum_ego_speed_ratio": 0.65,
        "capture_preroll_distance_m": [8.0, 14.0],
        "maximum_motion_m": 110.0,
        "npc_remaining_route_m": 120.0,
        "structured_backup_front": True,
        "backup_front_offset_m": 45.0,
        "backup_speed_scale": 1.05,
        "motion_stall_timeout_sec": 16.0,
        "lookahead_m": 4.0,
        "motion_phase": "generic_lane_relative_motion",
        "sampling_role": "structured_random_dynamic_vehicle",
        "stable_ego_gate": False,
        # Arbitrary route-wide NPC relocation is not reliable in MORAI: a
        # geometrically valid MGeo link is not sufficient for the spawn
        # controller to build a live path.  Keep this regularizer on the two
        # route bands that passed repeated live capture, while the mission
        # families provide the rest of the competition-route coverage.
        #
        # Train/val/test intentionally share both bands.  The split atom is a
        # complete scenario clip and each split gets independent scene seeds;
        # making map position itself a split label created an avoidable domain
        # shift and did not reflect the intended benchmark.
        "ego_capture_zones": [
            {"name": "train_stable_a", "route_index_range": [2250, 2500], "weight": 10, "split": "train"},
            {"name": "train_stable_d", "route_index_range": [3045, 3300], "weight": 10, "split": "train"},
            {"name": "val_stable_a", "route_index_range": [2250, 2500], "weight": 2, "split": "val"},
            {"name": "val_stable_d", "route_index_range": [3045, 3300], "weight": 2, "split": "val"},
            {"name": "test_stable_a", "route_index_range": [2250, 2500], "weight": 2, "split": "test"},
            {"name": "test_stable_d", "route_index_range": [3045, 3300], "weight": 2, "split": "test"},
        ],
        "interaction_roles": [
            {"name": "front_slow", "weight": 12, "offset_m": [15.0, 35.0], "speed_scale": [0.65, 0.90], "gate_longitudinal_m": [5.0, 48.0]},
            {"name": "front_matched", "weight": 6, "offset_m": [12.0, 32.0], "speed_scale": [0.90, 1.05], "gate_longitudinal_m": [3.0, 45.0]},
            {"name": "rear_approach", "weight": 6, "offset_m": [-42.0, -28.0], "speed_scale": [1.05, 1.25], "spawn_from_rest": True, "gate_longitudinal_m": [-55.0, -8.0]},
            {"name": "rear_receding", "weight": 4, "offset_m": [-42.0, -28.0], "speed_scale": [0.70, 0.95], "spawn_from_rest": True, "gate_longitudinal_m": [-55.0, -8.0]},
        ],
        # Exact 28-clip primary allocation.  Every split contains front and
        # rear motion instead of accidentally turning map split into a role
        # label.  Reserve configs remove this fixed-size table below.
        "split_role_quotas": {
            "train": {
                "front_slow": 8,
                "front_matched": 4,
                "rear_approach": 4,
                "rear_receding": 4,
            },
            "val": {
                "front_slow": 2,
                "front_matched": 1,
                "rear_approach": 1,
                "rear_receding": 0,
            },
            "test": {
                "front_slow": 2,
                "front_matched": 1,
                "rear_approach": 1,
                "rear_receding": 0,
            },
        },
    }
    return base, rows


def build_config(batch_index, role, counts, campaign_tag="f3", seed_base=260824000):
    base, defaults = family_defaults()
    config = copy.deepcopy(base)
    tag = "batch{:03d}_{}".format(batch_index, role)
    config["dataset_id"] = "morai_temporal_sequence_v2_{}".format(tag)
    config["seed"] = int(seed_base) + batch_index * 10000 + (
        0 if role == "primary" else 5000
    )
    config["scenario_prefix"] = "t2{}b{:03d}{}".format(
        campaign_tag, batch_index, "p" if role == "primary" else "r"
    )
    config["weighted_obstacle_quota_deck"] = True
    config["families"] = []
    for name, count in counts.items():
        if count <= 0:
            continue
        row = copy.deepcopy(defaults[name])
        row["count"] = int(count)
        if name == "east_corridor_relative_vehicle":
            # Rotate the exact role/block pairing between batches.  Otherwise
            # singleton Val/Test blocks would always inherit the same motion
            # role despite having correct global role quotas.
            row["pair_schedule_offset"] = (
                int(batch_index) - 1 + (2 if role == "reserve" else 0)
            ) % len(row["interaction_roles"])
            if int(count) != 28:
                row.pop("split_role_quotas", None)
        config["families"].append(row)
    return config


def grouped_counts(counts):
    assigned = set().union(*FAMILY_GROUPS.values())
    unknown = set(counts) - assigned
    if unknown:
        raise ValueError("unassigned Temporal V2 families: {}".format(sorted(unknown)))
    return {
        group: sum(int(counts.get(name, 0)) for name in names)
        for group, names in FAMILY_GROUPS.items()
    }


def run_generator(
    config_path, output_root, group, id_prefix, install_dir=None
):
    command = [
        sys.executable,
        str(GENERATOR),
        "--config",
        str(config_path),
        "--output-root",
        str(output_root),
        "--group",
        group,
        "--id-prefix",
        id_prefix,
    ]
    if install_dir is not None:
        command.extend(["--install-dir", str(install_dir)])
    subprocess.run(command, check=True)


def main():
    args = parse_args()
    if args.batch_index < 1:
        raise ValueError("--batch-index must be positive")
    campaign_tag = "".join(
        char for char in str(args.campaign_tag).lower() if char.isalnum()
    )
    if not campaign_tag:
        raise ValueError("--campaign-tag must contain at least one alphanumeric character")
    roles = [("primary", PRIMARY_COUNTS)]
    if not args.no_reserve:
        roles.append(("reserve", RESERVE_COUNTS))
    plan = {
        "schema_version": 1,
        "batch_index": args.batch_index,
        "campaign_tag": campaign_tag,
        "seed_base": args.seed_base,
        "canonical_frames_per_clip": 8,
        "requested_chunk_frames": 1500,
        "primary_clip_count": sum(PRIMARY_COUNTS.values()),
        "primary_frame_count": sum(PRIMARY_COUNTS.values()) * 8,
        "rounding_note": "1,500 is not divisible by 8; canonical batch uses 1,504 frames.",
        "roles": [],
    }
    primary_groups = grouped_counts(PRIMARY_COUNTS)
    if primary_groups != {
        "mission_dynamic": 79,
        "static_temporal": 80,
        "hard_transition": 29,
    }:
        raise ValueError("primary Temporal V2 quota drift: {}".format(primary_groups))
    config_root = args.output_root / "metadata/temporal_v2_batches" / "batch{:03d}".format(args.batch_index)
    for role, counts in roles:
        config = build_config(
            args.batch_index,
            role,
            counts,
            campaign_tag=campaign_tag,
            seed_base=args.seed_base,
        )
        config_path = config_root / "{}_config.json".format(role)
        write_json(config_path, config)
        group = "temporal_sequence_v2_batch{:03d}_{}".format(
            args.batch_index, role
        )
        id_prefix = "t2{}b{:03d}{}_".format(
            campaign_tag,
            args.batch_index,
            "p" if role == "primary" else "r",
        )
        run_generator(
            config_path,
            args.output_root,
            group,
            id_prefix,
            args.install_dir,
        )
        clip_count = sum(counts.values())
        plan["roles"].append(
            {
                "role": role,
                "group": group,
                "config": str(config_path),
                "scene_id_prefix": id_prefix,
                "clip_count": clip_count,
                "frame_count": clip_count * 8,
                "family_clip_counts": counts,
                "group_clip_counts": grouped_counts(counts),
            }
        )
    plan_path = config_root / "batch_plan.json"
    write_json(plan_path, plan)
    print("primary clips: {}".format(plan["primary_clip_count"]))
    print("primary frames: {}".format(plan["primary_frame_count"]))
    print("plan: {}".format(plan_path))


if __name__ == "__main__":
    main()
