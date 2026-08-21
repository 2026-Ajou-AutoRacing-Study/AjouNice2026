#!/usr/bin/env python3
"""Generate one 1,504-frame temporal V2 batch plus a 160-frame reserve."""

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
    # Mission dynamic: 93 clips / 744 frames (49.5%).
    "roundabout_circulation": 18,
    "highway_merge_cruise": 14,
    "highway_cruise_overtake": 32,
    "highway_lane_change_overtake": 4,
    "crosswalk_pedestrian": 25,
    # Static temporal: 66 clips / 528 frames (35.1%).  The two explicit
    # obstacle families are 45/66=68.2%, matching the planned ~70% obstacle
    # share without contaminating otherwise pure mission scenes.
    "static_obstacle": 35,
    "static_highway_obstacle": 10,
    "stopped_vehicle": 8,
    "standing_pedestrian": 5,
    "realistic_mixed_static": 4,
    "sparse_background": 4,
    # Hard transitions: 29 clips / 232 frames (15.4%).
    "roundabout_sample_spawn": 8,
    "highway_merge_sample_spawn": 8,
    "gps_shadow_obstacle": 13,
}

RESERVE_COUNTS = {
    "roundabout_sample_spawn": 1,
    "roundabout_circulation": 2,
    "highway_merge_sample_spawn": 1,
    "highway_merge_cruise": 2,
    "highway_cruise_overtake": 3,
    "highway_lane_change_overtake": 1,
    "crosswalk_pedestrian": 2,
    "static_obstacle": 4,
    "static_highway_obstacle": 1,
    "stopped_vehicle": 1,
    "standing_pedestrian": 1,
    "realistic_mixed_static": 0,
    "sparse_background": 0,
    "gps_shadow_obstacle": 1,
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
    },
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
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
    return base, rows


def build_config(batch_index, role, counts):
    base, defaults = family_defaults()
    config = copy.deepcopy(base)
    tag = "batch{:03d}_{}".format(batch_index, role)
    config["dataset_id"] = "morai_temporal_sequence_v2_{}".format(tag)
    config["seed"] = 260821000 + batch_index * 10000 + (0 if role == "primary" else 5000)
    config["scenario_prefix"] = "t2b{:03d}{}".format(
        batch_index, "p" if role == "primary" else "r"
    )
    config["weighted_obstacle_quota_deck"] = True
    config["families"] = []
    for name, count in counts.items():
        if count <= 0:
            continue
        row = copy.deepcopy(defaults[name])
        row["count"] = int(count)
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


def run_generator(config_path, output_root, group, install_dir=None):
    command = [
        sys.executable,
        str(GENERATOR),
        "--config",
        str(config_path),
        "--output-root",
        str(output_root),
        "--group",
        group,
    ]
    if install_dir is not None:
        command.extend(["--install-dir", str(install_dir)])
    subprocess.run(command, check=True)


def main():
    args = parse_args()
    if args.batch_index < 1:
        raise ValueError("--batch-index must be positive")
    roles = [("primary", PRIMARY_COUNTS)]
    if not args.no_reserve:
        roles.append(("reserve", RESERVE_COUNTS))
    plan = {
        "schema_version": 1,
        "batch_index": args.batch_index,
        "canonical_frames_per_clip": 8,
        "requested_chunk_frames": 1500,
        "primary_clip_count": sum(PRIMARY_COUNTS.values()),
        "primary_frame_count": sum(PRIMARY_COUNTS.values()) * 8,
        "rounding_note": "1,500 is not divisible by 8; canonical batch uses 1,504 frames.",
        "roles": [],
    }
    primary_groups = grouped_counts(PRIMARY_COUNTS)
    if primary_groups != {
        "mission_dynamic": 93,
        "static_temporal": 66,
        "hard_transition": 29,
    }:
        raise ValueError("primary Temporal V2 quota drift: {}".format(primary_groups))
    config_root = args.output_root / "metadata/temporal_v2_batches" / "batch{:03d}".format(args.batch_index)
    for role, counts in roles:
        config = build_config(args.batch_index, role, counts)
        config_path = config_root / "{}_config.json".format(role)
        write_json(config_path, config)
        group = "temporal_sequence_v2_batch{:03d}_{}".format(
            args.batch_index, role
        )
        run_generator(config_path, args.output_root, group, args.install_dir)
        clip_count = sum(counts.values())
        plan["roles"].append(
            {
                "role": role,
                "group": group,
                "config": str(config_path),
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
