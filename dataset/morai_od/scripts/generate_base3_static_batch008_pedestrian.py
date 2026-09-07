#!/usr/bin/env python3
"""Generate Base3-Static Batch 008: 300 moving-pedestrian clips x 3 frames.

The pedestrian actor moves, but this is intentionally a single-frame OD
campaign.  It uses the same rolling-grouped, 3 m, three-frame capture contract
as Base3-Static Batch 001--007 and is not a temporal-model sequence dataset.
"""

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

CLIP_COUNT = 300
FRAMES_PER_CLIP = 3
GROUP = "static_classcentric_base3_batch008_dynamic_ped"
DATASET_ID = "morai_static_classcentric_base3_10k_batch008_dynamic_ped"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=CLIP_COUNT)
    parser.add_argument("--seed", type=int, default=260830801)
    parser.add_argument("--group", default=GROUP)
    parser.add_argument("--id-prefix", default="b3sdp8_")
    parser.add_argument("--stem-prefix", default="b3sdp8_")
    parser.add_argument("--output-root", type=Path, default=DATASET_ROOT)
    parser.add_argument(
        "--replace-empty-lidar-v1",
        action="store_true",
        help=(
            "Generate the six train/forward clips needed to replace the "
            "deterministic empty-LiDAR failures from the first Batch008 run."
        ),
    )
    parser.add_argument(
        "--replace-collision-v2",
        action="store_true",
        help=(
            "Generate the four residual train/forward clips after the first "
            "replacement deck supplied two valid clips."
        ),
    )
    return parser.parse_args()


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_config(
    count,
    seed,
    replace_empty_lidar_v1=False,
    replace_collision_v2=False,
):
    if replace_empty_lidar_v1 and replace_collision_v2:
        raise ValueError("select only one Batch008 replacement deck")
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
            # Location-disjoint Val/Test remains identical to the 75-clip
            # pilot, while the official sample crossing is capped at 25%.
            "crosswalk_locations": [
                {
                    "crosswalk_id": "CW000000",
                    "weight": 0.25,
                    "role": "official_sample_direct",
                    "split": "train",
                    "use_official_path": True,
                    # The official actor path is short enough that an actor
                    # started at scenario load can finish it while MORAI is
                    # still verifying the scene.  Arm it on Ego approach so
                    # all three distance-spaced frames retain real motion.
                    "activation_policy": "deferred_ego_approach",
                },
                {
                    "crosswalk_id": "CW000002",
                    "weight": 0.20,
                    "role": "route_direct_train",
                    "split": "train",
                    "activation_policy": "deferred_ego_approach",
                },
                {
                    "crosswalk_id": "CW000013",
                    "weight": 0.15,
                    "role": "route_direct_validation",
                    "split": "val",
                    "activation_policy": "deferred_ego_approach",
                },
                {
                    "crosswalk_id": "CW000015",
                    "weight": 0.15,
                    "role": "route_direct_test",
                    "split": "test",
                    "activation_policy": "deferred_ego_approach",
                },
                {
                    "crosswalk_id": "CW000001",
                    "weight": 0.125,
                    "role": "route_adjacent_side",
                    "split": "train",
                    "activation_policy": "deferred_ego_approach",
                },
                {
                    "crosswalk_id": "CW000014",
                    "weight": 0.125,
                    "role": "route_adjacent_side",
                    "split": "train",
                    "activation_policy": "deferred_ego_approach",
                },
            ],
            "ego_motion_profiles": [
                {
                    "name": "competition_motion",
                    "weight": 0.75,
                    "target_speed_kph": [15.0, 28.0],
                    "initial_speed_kph": [9.0, 18.0],
                },
                {
                    "name": "slow_crosswalk_approach",
                    "weight": 0.25,
                    "target_speed_kph": [8.0, 14.0],
                    "initial_speed_kph": [5.0, 10.0],
                },
            ],
            "motion_profiles": [
                {"name": "walking", "weight": 0.45, "speed": [4.0, 6.0]},
                {"name": "running", "weight": 0.45, "speed": [8.0, 10.0]},
                {
                    "name": "activation_transition",
                    "weight": 0.10,
                    "speed": [6.0, 8.0],
                    "visibility_audit_required": True,
                },
            ],
            "dynamic_warmup_sec": 0.0,
            "prime_capture_before_clip": False,
            "capture_motion_arm_m": 3.0,
            "minimum_ego_speed_ratio": 0.65,
            "maximum_motion_m": 75.0,
            "motion_stall_timeout_sec": 15.0,
            "sampling_role": "base3_static_dynamic_pedestrian",
            "motion_phase": "moving_pedestrian_singleframe",
            "capture_actor_gate": {
                "class": "pedestrian",
                "role": "moving_crosswalk_pedestrian",
                "minimum_longitudinal_m": -15.0,
                "maximum_longitudinal_m": 55.0,
                "maximum_abs_lateral_m": 30.0,
                "maximum_distance_m": 60.0,
                "minimum_actor_speed_mps": 0.2,
            },
        }
    )
    if replace_empty_lidar_v1:
        if count != 6:
            raise ValueError("the empty-LiDAR replacement deck requires 6 clips")
        pedestrian.update(
            {
                # Restore the exact location/profile cells removed by the
                # six deterministic empty first-frame LiDAR labels.
                "crosswalk_locations": [
                    {
                        "crosswalk_id": "CW000000",
                        "weight": 4.0 / 6.0,
                        "role": "official_sample_direct",
                        "split": "train",
                        "use_official_path": True,
                        "activation_policy": "deferred_ego_approach",
                    },
                    {
                        "crosswalk_id": "CW000002",
                        "weight": 2.0 / 6.0,
                        "role": "route_direct_train",
                        "split": "train",
                        "activation_policy": "deferred_ego_approach",
                    },
                ],
                "motion_profiles": [
                    {
                        "name": "running",
                        "weight": 4.0 / 6.0,
                        "speed": [8.0, 10.0],
                        "reverse_probability": 0.0,
                    },
                    {
                        "name": "walking",
                        "weight": 2.0 / 6.0,
                        "speed": [4.0, 6.0],
                        "reverse_probability": 0.0,
                    },
                ],
                "balance_reverse_direction": False,
                "reverse_probability": 0.0,
                # Shift the first observation beyond the occluded/far slot
                # that repeatedly produced an empty Instance.txt.
                "ego_lead_distance_m": [10.0, 16.0],
            }
        )
    if replace_collision_v2:
        if count != 4:
            raise ValueError("the collision replacement deck requires 4 clips")
        pedestrian.update(
            {
                "crosswalk_locations": [
                    {
                        "crosswalk_id": "CW000000",
                        "weight": 0.75,
                        "role": "official_sample_direct",
                        "split": "train",
                        "use_official_path": True,
                        "activation_policy": "deferred_ego_approach",
                    },
                    {
                        "crosswalk_id": "CW000002",
                        "weight": 0.25,
                        "role": "route_direct_train",
                        "split": "train",
                        "activation_policy": "deferred_ego_approach",
                    },
                ],
                "motion_profiles": [
                    {
                        "name": "running",
                        "weight": 0.75,
                        "speed": [8.0, 10.0],
                        "reverse_probability": 0.0,
                    },
                    {
                        "name": "walking",
                        "weight": 0.25,
                        "speed": [4.0, 6.0],
                        "reverse_probability": 0.0,
                    },
                ],
                "balance_reverse_direction": False,
                "reverse_probability": 0.0,
                "ego_lead_distance_m": [18.0, 22.0],
            }
        )
    config = {
        **base,
        "dataset_id": DATASET_ID,
        "scenario_prefix": "od_b3sdp8",
        "seed": seed,
        "use_runtime_pedestrian_pool": True,
        "route_wide_route_source": "mgeo_guided",
        "families": [pedestrian],
    }
    config["capture"].update(
        {
            "format": "morai_save_sensor_data_rolling_grouped",
            "captures_per_scene": FRAMES_PER_CLIP,
            # Metadata fallback only. rolling-grouped is distance-triggered.
            "capture_period_sec": 0.35,
            "capture_spacing_m": 3.0,
            "clip_duration_sec": 0.0,
            "capture_motion_arm_m": 3.0,
            "minimum_capture_progress_m": 0.3,
            "motion_stall_timeout_sec": 15.0,
            "maximum_motion_m": 75.0,
            "warmup_sec": 0.0,
            "prime_capture_before_clip": False,
            "split_unit": "static_layout_clip",
        }
    )
    return config


def main():
    args = parse_args()
    if args.count < 6 and not args.replace_collision_v2:
        raise ValueError("at least six clips are required to cover every crosswalk")
    config_path = args.output_root / "configs/generated/{}.json".format(args.group)
    write_json(
        config_path,
        build_config(
            args.count,
            args.seed,
            args.replace_empty_lidar_v1,
            args.replace_collision_v2,
        ),
    )
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
    print("frames:", args.count * FRAMES_PER_CLIP)


if __name__ == "__main__":
    main()
