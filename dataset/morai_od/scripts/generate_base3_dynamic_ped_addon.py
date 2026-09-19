#!/usr/bin/env python3
"""Generate the 75-clip Base3 dynamic-pedestrian add-on campaign."""

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
BASE_CONFIG = DATASET_ROOT / "configs/temporal_sequence_capture_v2_pilot304.json"
GENERATOR = DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=75)
    parser.add_argument("--seed", type=int, default=260829801)
    parser.add_argument("--group", default="base3_dynamic_ped_addon_v1")
    parser.add_argument("--id-prefix", default="b3dpv1_")
    parser.add_argument("--stem-prefix", default="b3dpv1_")
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    return parser.parse_args()


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_config(count, seed):
    base = json.loads(BASE_CONFIG.read_text(encoding="utf-8"))
    pedestrian = copy.deepcopy(
        next(row for row in base["families"] if row["name"] == "crosswalk_pedestrian")
    )
    pedestrian.update(
        {
            "count": count,
            "cover_all_crosswalks_when_possible": False,
            "balance_reverse_direction": True,
            "crosswalk_path_margin_m": 4.5,
            "ego_motion_profiles": [
                {
                    "name": "competition_motion",
                    "weight": 0.75,
                    "target_speed_kph": [12.0, 24.0],
                    "initial_speed_kph": [7.0, 15.0],
                },
                {
                    "name": "very_low_speed",
                    "weight": 0.25,
                    "target_speed_kph": [6.0, 10.0],
                    "initial_speed_kph": [4.0, 7.0],
                },
            ],
            # Visibility profiles are audited from the captured camera/LiDAR
            # artifacts.  The generated crosswalk geometry supplies natural
            # structure occlusion; no artificial obstacle is inserted into a
            # pedestrian-only mission clip.
            "motion_profiles": [
                {
                    "name": "walking_clear_candidate",
                    "weight": 0.30,
                    "speed": [4.0, 6.0],
                },
                {
                    "name": "walking_partial_candidate",
                    "weight": 0.15,
                    "speed": [4.0, 6.0],
                    "visibility_audit_required": True,
                },
                {
                    "name": "running_clear_candidate",
                    "weight": 0.30,
                    "speed": [8.0, 10.0],
                },
                {
                    "name": "running_partial_candidate",
                    "weight": 0.15,
                    "speed": [8.0, 10.0],
                    "visibility_audit_required": True,
                },
                {
                    "name": "activation_transition",
                    "weight": 0.10,
                    "speed": [6.0, 8.0],
                    "visibility_audit_required": True,
                },
            ],
            "dynamic_warmup_sec": 0.0,
            "prime_capture_before_clip": False,
            "capture_motion_arm_m": 0.3,
            "minimum_ego_speed_ratio": 0.70,
            "maximum_motion_m": 75.0,
            "motion_stall_timeout_sec": 14.0,
            "sampling_role": "base3_dynamic_pedestrian_addon",
        }
    )
    config = {
        **base,
        "dataset_id": "morai_base3_dynamic_ped_addon_v1",
        "scenario_prefix": "od_b3dp",
        "seed": seed,
        "use_runtime_pedestrian_pool": True,
        "route_wide_route_source": "mgeo_guided",
        "families": [pedestrian],
    }
    config["capture"].update(
        {
            "captures_per_scene": 8,
            "capture_period_sec": 1.0 / 12.0,
            "clip_duration_sec": 7.0 / 12.0,
            "capture_motion_arm_m": 0.3,
            "minimum_capture_progress_m": 0.04,
            "motion_stall_timeout_sec": 14.0,
            "warmup_sec": 0.0,
            "prime_capture_before_clip": False,
        }
    )
    return config


def main():
    args = parse_args()
    if args.count < 6:
        raise ValueError("--count must be at least six to cover every crosswalk")
    config_path = (
        args.output_root / "configs/generated/base3_dynamic_ped_addon_v1.json"
    )
    write_json(config_path, build_config(args.count, args.seed))
    command = [
        sys.executable,
        str(GENERATOR),
        "--config",
        str(config_path),
        "--output-root",
        str(args.output_root),
        "--group",
        args.group,
        "--id-prefix",
        args.id_prefix,
        "--stem-prefix",
        args.stem_prefix,
    ]
    subprocess.run(command, check=True)
    print("config:", config_path)
    print("clips:", args.count)
    print("frames:", args.count * 8)


if __name__ == "__main__":
    main()
