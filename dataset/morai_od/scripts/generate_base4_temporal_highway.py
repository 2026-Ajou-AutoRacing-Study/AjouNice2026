#!/usr/bin/env python3
"""Generate Base4-Temporal highway merge and high-speed overtake scenes."""

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
GENERATOR = DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py"

sys.path.insert(0, str(DATASET_ROOT / "scripts"))
from generate_temporal_sequence_v2_batch import family_defaults  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-count", type=int, default=30)
    parser.add_argument("--seed", type=int, default=260920401)
    parser.add_argument("--campaign-tag", default="pilot30")
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--install-dir", type=Path)
    return parser.parse_args()


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_config(scene_count, seed, campaign_tag):
    if scene_count < 8:
        raise ValueError("Base4 highway pilot needs at least 8 scenes")
    base, defaults = family_defaults()
    merge_count = max(2, int(round(scene_count * 0.25)))
    cruise_count = scene_count - merge_count

    capture = copy.deepcopy(base["capture"])
    capture.pop("scenario_load_initial_speed_kph", None)
    capture.update(
        {
            "format": "morai_online_rosbag_temporal_gated",
            "captures_per_scene": 16,
            "capture_period_sec": 1.0 / 12.0,
            "capture_spacing_m": 1.5,
            "clip_duration_sec": 1.4,
            "ego_target_speed_kph": 58.0,
            "warmup_sec": 0.0,
            "prime_capture_before_clip": False,
            "maximum_motion_m": 180.0,
            "motion_stall_timeout_sec": 15.0,
            "actor_ready_timeout_sec": 18.0,
            "route_deviation_limit_m": 2.0,
            "heading_error_limit_deg": 35.0,
            "lookahead_m": 8.0,
            "maximum_steering_rad": 0.22,
            "adaptive_curve_control": False,
            "minimum_curve_speed_kph": 45.0,
            "minimum_curve_lookahead_m": 4.0,
            "curve_maximum_steering_rad": 0.30,
        }
    )

    merge = copy.deepcopy(defaults["highway_merge_cruise"])
    merge.update(
        {
            "count": merge_count,
            # Competition rule: the ramp/merge approach must remain strictly
            # below 59 km/h. These are choices, not a continuous range.
            "ego_target_speed_kph": [50.0, 54.0, 58.0],
            "ego_initial_speed_kph": [45.0, 50.0, 54.0],
            "start_ego_at_target_speed": False,
            "minimum_ego_speed_ratio": 0.88,
            "source_spawn_ids": [6, 8, 10],
            "target_speed_scale": [1.0, 1.0],
            "spawn_velocity_equals_target": True,
            "preserve_template_npc_target_speed": True,
            "maximum_spawn_vehicles": 1,
            "spawn_wait_sec": 0.0,
            "spawn_period_sec": [10.0, 12.0],
            "ego_capture_zones": [
                {
                    "name": "ramp_approach",
                    "route_index_range": [1960, 1995],
                    "weight": 0.20,
                },
                {
                    "name": "merge_conflict",
                    "route_index_range": [1996, 2060],
                    "weight": 0.80,
                },
            ],
            "capture_preroll_distance_m": [14.0, 24.0],
            "captures_per_scene": 16,
            "desired_clip_duration_sec": 1.4,
            "maximum_motion_m": 180.0,
            "motion_stall_timeout_sec": 15.0,
            "lookahead_m": 7.0,
            "motion_phase": "highway_merge_below_59kph",
            "sampling_role": "base4_merge_dynamic_vehicle",
        }
    )

    cruise = copy.deepcopy(defaults["highway_cruise_overtake"])
    cruise.update(
        {
            "count": cruise_count,
            "source_spawn_ids": [6, 8, 10],
            # Keep all legal mainline lanes represented.  The schedule varies
            # lanes and relative roles, avoiding a fixed-coordinate shortcut.
            "ego_lane_weights": {"6": 0.25, "8": 0.40, "10": 0.35},
            "ego_lane_speed_kph": {
                "6": [70.0, 100.0],
                "8": [70.0, 120.0],
                "10": [85.0, 120.0],
            },
            # Deterministic repeating deck: 100 km/h dominates, while 115/120
            # are deliberate minority stress cases rather than the dataset's
            # only geometry.
            "ego_speed_schedule_kph": [
                70.0,
                85.0,
                85.0,
                100.0,
                100.0,
                100.0,
                115.0,
                120.0,
            ],
            "npc_target_speed_scale": [1.0, 1.0],
            "preserve_template_npc_target_speed": True,
            "spawn_velocity_equals_target": True,
            "maximum_spawn_vehicles": 1,
            "start_ego_at_target_speed": True,
            "minimum_ego_speed_ratio": 0.92,
            "stable_ego_gate": True,
            "maximum_stable_acceleration_mps2": 1.5,
            "ego_start_arc_m": [70.0, 285.0],
            "ego_speed_advantage_kph": [20.0, 65.0],
            "ego_speed_limit_kph": 120.0,
            "captures_per_scene": 16,
            "desired_clip_duration_sec": 1.4,
            "maximum_motion_m": 190.0,
            "motion_stall_timeout_sec": 14.0,
            "capture_motion_arm_m": 3.0,
            "interaction_roles": [
                {
                    "name": "front_overtake",
                    "weight": 3.0,
                    "npc_longitudinal_offset_m": [20.0, 42.0],
                    "gate_longitudinal_m": [5.0, 50.0],
                    "maximum_abs_lateral_m": 6.5,
                },
                {
                    "name": "side_parallel",
                    "weight": 1.0,
                    "npc_longitudinal_offset_m": [-10.0, 14.0],
                    "gate_longitudinal_m": [-22.0, 25.0],
                    "maximum_abs_lateral_m": 6.5,
                },
            ],
            "motion_phase": "post_merge_high_speed_overtake",
            "sampling_role": "base4_high_speed_relative_vehicle",
        }
    )

    clean_tag = "".join(ch for ch in campaign_tag.lower() if ch.isalnum())
    if not clean_tag:
        raise ValueError("campaign tag must contain an alphanumeric character")
    return {
        "dataset_id": "morai_base4_temporal_highway_{}".format(clean_tag),
        "seed": int(seed),
        "scenario_prefix": "b4th{}".format(clean_tag),
        "use_runtime_obstacle_pool": False,
        "split_ratios": {"train": 0.8, "val": 0.2, "test": 0.0},
        "environment": copy.deepcopy(base["environment"]),
        "capture": capture,
        "families": [merge, cruise],
        "base4_design": {
            "scene_count": scene_count,
            "merge_scene_count": merge_count,
            "post_merge_scene_count": cruise_count,
            "npc_goal_speed_contract_kph": [40.0, 50.0, 60.0],
            "premerge_ego_speed_contract_kph": [50.0, 54.0, 58.0],
            "postmerge_ego_speed_contract_kph": [70.0, 85.0, 100.0, 115.0, 120.0],
            "internal_test_split": False,
        },
    }


def main():
    args = parse_args()
    config = build_config(args.scene_count, args.seed, args.campaign_tag)
    tag = "".join(ch for ch in args.campaign_tag.lower() if ch.isalnum())
    config_path = (
        args.output_root
        / "metadata/base4_temporal_highway"
        / "{}_config.json".format(tag)
    )
    write_json(config_path, config)
    command = [
        sys.executable,
        str(GENERATOR),
        "--config",
        str(config_path),
        "--output-root",
        str(args.output_root),
        "--group",
        "base4_temporal_highway_{}".format(tag),
        "--id-prefix",
        "b4{}d".format(tag),
        "--stem-prefix",
        "b4{}_".format(tag),
    ]
    if args.install_dir is not None:
        command.extend(["--install-dir", str(args.install_dir)])
    subprocess.run(command, check=True)
    print("config: {}".format(config_path))


if __name__ == "__main__":
    main()
