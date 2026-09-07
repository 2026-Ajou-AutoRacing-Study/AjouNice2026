#!/usr/bin/env python3
"""Generate one clip-safe 1,500-frame MORAI OD production batch.

The batch stays operationally separated by capture mode: mission and dynamic
hard clips use timed-grouped SaveSensorData, while static and static-hard
scenes use rolling-grouped SaveSensorData.  A small reserve set is generated
under different ids and is never counted in the 1,500-frame target.
"""

import argparse
import copy
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
BASE_DYNAMIC_CONFIG = DATASET_ROOT / "configs/dynamic_object_capture_v1.json"
DEFAULT_INSTALL_DIR = (
    Path.home()
    / "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025"
)
DEFAULT_NETWORK_TEMPLATE = (
    DEFAULT_INSTALL_DIR / "EgoNetwork/od_s13p6_186_MN.json"
)

PRIMARY_QUOTAS = {"mission": 744, "static": 525, "hard": 231}
RESERVE_QUOTAS = {"mission": 60, "static": 150, "hard": 24}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-index", type=int, default=1)
    parser.add_argument("--install-dir", type=Path, default=DEFAULT_INSTALL_DIR)
    parser.add_argument("--no-install", action="store_true")
    parser.add_argument("--plan-only", action="store_true")
    return parser.parse_args()


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def family(base, name, count):
    row = copy.deepcopy(next(item for item in base["families"] if item["name"] == name))
    row["count"] = int(count)
    return row


def dynamic_config(base, dataset_id, seed, families):
    value = copy.deepcopy(base)
    value["dataset_id"] = dataset_id
    value["seed"] = int(seed)
    value["families"] = families
    return value


def split_counts(count):
    validation = int(round(count * 0.15))
    test = int(round(count * 0.15))
    return {"train": count - validation - test, "val": validation, "test": test}


def static_spec(batch_tag, role, seed, scene_types):
    count = sum(int(item["count"]) for item in scene_types)
    short_role = {
        "static": "s",
        "static_reserve": "sr",
        "hard_static": "hs",
    }[role]
    group = f"dynamic_dataset_{batch_tag}_{role}"
    return {
        "schema_version": 1,
        "dataset_id": f"morai_dynamic_dataset_{batch_tag}_{role}",
        "scenario_group": group,
        "manifest_stem": group,
        "scene_id_prefix": f"{batch_tag}_{short_role}_",
        "scenario_file_prefix": f"od_{batch_tag}_{short_role}_",
        "seed": int(seed),
        "scenario_count": count,
        "split_counts": split_counts(count),
        "scene_types": scene_types,
    }


def static_scene_types(primary=True):
    # Primary keeps the 35/10/10/40/5 target. Live Batch001 showed a 23%
    # launcher omission rate for dense NPC/pedestrian layouts, so reserve is
    # a 50-scene load-safe pool with 70% obstacle-focused placements.
    counts = (61, 18, 17, 70, 9) if primary else (10, 0, 0, 35, 5)
    return [
        {
            "name": "realistic_mixed",
            "count": counts[0],
            "npc_range": [2, 5], "pedestrian_range": [1, 4], "obstacle_range": [1, 3],
            "topology_preference": ["intersection", "merge_diverge", "curve"],
        },
        {
            "name": "vehicle_centric",
            "count": counts[1],
            "npc_range": [5, 8], "pedestrian_range": [0, 2], "obstacle_range": [0, 2],
            "topology_preference": ["straight", "intersection", "merge_diverge"],
        },
        {
            "name": "pedestrian_centric",
            "count": counts[2],
            "npc_range": [0, 2], "pedestrian_range": [6, 10], "obstacle_range": [0, 2],
            "topology_preference": ["crosswalk", "intersection", "roadside_parking_clutter"],
        },
        {
            "name": "obstacle_centric",
            "count": counts[3],
            "npc_range": [0, 2], "pedestrian_range": [0, 2], "obstacle_range": [5, 7],
            "topology_preference": ["roadside_parking_clutter", "straight", "intersection"],
        },
        {
            "name": "sparse_empty_background",
            "count": counts[4],
            "npc_range": [0, 1], "pedestrian_range": [0, 0], "obstacle_range": [0, 0],
            "topology_preference": ["straight"],
        },
    ]


def hard_static_scene_types():
    return [
        {
            "name": "camera_seam_occlusion",
            "count": 2,
            "npc_range": [2, 4], "pedestrian_range": [2, 4], "obstacle_range": [1, 3],
            "topology_preference": ["curve", "intersection"],
        },
        {
            "name": "dense_obstacle_occlusion",
            "count": 2,
            "npc_range": [0, 2], "pedestrian_range": [1, 1], "obstacle_range": [6, 8],
            "topology_preference": ["roadside_parking_clutter", "intersection"],
        },
        {
            "name": "sparse_hard_negative",
            "count": 1,
            "npc_range": [0, 1], "pedestrian_range": [1, 1], "obstacle_range": [0, 1],
            "topology_preference": ["curve", "straight"],
        },
    ]


def run(command):
    print("+", " ".join(str(item) for item in command), flush=True)
    subprocess.run([str(item) for item in command], cwd=PROJECT_ROOT, check=True)


def main():
    args = parse_args()
    if args.batch_index < 1:
        raise ValueError("batch-index must be >= 1")
    batch_tag = f"batch{args.batch_index:03d}"
    seed_base = 260820000 + args.batch_index * 100000
    generated_dir = DATASET_ROOT / "metadata" / "production_batches" / batch_tag
    install_dir = None if args.no_install else args.install_dir
    base = read_json(BASE_DYNAMIC_CONFIG)

    mission = dynamic_config(
        base,
        f"morai_dynamic_dataset_{batch_tag}_mission",
        seed_base + 1000,
        [
            family(base, "roundabout_circulation", 12),
            family(base, "highway_merge_cruise", 12),
            family(base, "highway_cruise_overtake", 20),
            family(base, "crosswalk_pedestrian", 18),
        ],
    )
    mission_reserve = dynamic_config(
        base,
        f"morai_dynamic_dataset_{batch_tag}_mission_reserve",
        seed_base + 2000,
        [
            family(base, "roundabout_circulation", 1),
            family(base, "highway_merge_cruise", 1),
            family(base, "highway_cruise_overtake", 2),
            family(base, "crosswalk_pedestrian", 1),
        ],
    )
    hard_dynamic = dynamic_config(
        base,
        f"morai_dynamic_dataset_{batch_tag}_hard_dynamic",
        seed_base + 3000,
        [
            # Highway spawn-acceleration variants were rejected by live
            # validation because Ego starts 1.6--15.8 m off the capture
            # route. Keep production hard clips on families whose route and
            # actor activation were validated end to end.
            family(base, "roundabout_sample_spawn", 12),
            family(base, "crosswalk_pedestrian", 6),
        ],
    )
    hard_dynamic_reserve = dynamic_config(
        base,
        f"morai_dynamic_dataset_{batch_tag}_hard_dynamic_reserve",
        seed_base + 4000,
        [
            family(base, "roundabout_sample_spawn", 1),
            family(base, "crosswalk_pedestrian", 1),
        ],
    )
    dynamic_rows = [
        ("mission", mission, "m_", "m_"),
        ("mission_reserve", mission_reserve, "mr_", "mr_"),
        ("hard_dynamic", hard_dynamic, "hd_", "hd_"),
        ("hard_dynamic_reserve", hard_dynamic_reserve, "hdr_", "hdr_"),
    ]
    dynamic_configs = {}
    for role, value, id_suffix, stem_suffix in dynamic_rows:
        path = generated_dir / f"{role}.json"
        write_json(path, value)
        dynamic_configs[role] = path

    static = static_spec(batch_tag, "static", seed_base + 5000, static_scene_types(True))
    static_reserve = static_spec(
        batch_tag, "static_reserve", seed_base + 6000, static_scene_types(False)
    )
    hard_static = static_spec(
        batch_tag, "hard_static", seed_base + 7000, hard_static_scene_types()
    )
    static_specs = {}
    for role, value in (("static", static), ("static_reserve", static_reserve), ("hard_static", hard_static)):
        path = generated_dir / f"{role}.json"
        write_json(path, value)
        static_specs[role] = path

    components = [
        {"role": "mission", "capture_mode": "timed-grouped", "scenes": 62, "frames_per_scene": 12, "frames": 744, "reserve": False},
        {"role": "static", "capture_mode": "rolling-grouped", "scenes": 175, "frames_per_scene": 3, "frames": 525, "reserve": False},
        {"role": "hard_dynamic", "capture_mode": "timed-grouped", "scenes": 18, "frames_per_scene": 12, "frames": 216, "reserve": False},
        {"role": "hard_static", "capture_mode": "rolling-grouped", "scenes": 5, "frames_per_scene": 3, "frames": 15, "reserve": False},
        {"role": "mission_reserve", "capture_mode": "timed-grouped", "scenes": 5, "frames_per_scene": 12, "frames": 60, "reserve": True},
        {"role": "static_reserve", "capture_mode": "rolling-grouped", "scenes": 50, "frames_per_scene": 3, "frames": 150, "reserve": True},
        {"role": "hard_dynamic_reserve", "capture_mode": "timed-grouped", "scenes": 2, "frames_per_scene": 12, "frames": 24, "reserve": True},
    ]
    plan = {
        "schema_version": 1,
        "batch_id": batch_tag,
        "seed_base": seed_base,
        "primary_frame_quotas": PRIMARY_QUOTAS,
        "primary_total_frames": sum(PRIMARY_QUOTAS.values()),
        "reserve_frame_quotas": RESERVE_QUOTAS,
        "reserve_total_frames": sum(RESERVE_QUOTAS.values()),
        "reserve_fraction": sum(RESERVE_QUOTAS.values()) / sum(PRIMARY_QUOTAS.values()),
        "components": components,
        "split_unit": "scenario_clip",
        "environment": {"weather": "Sunny", "time": "13:00"},
        "notes": [
            "Reserve ids are distinct and reserve frames are excluded from the 1,500-frame target unless replacing failures.",
            "Static obstacle placements are counted once per 3-frame scene, never three times.",
            "Hard dynamic spawn acceleration and static geometry hard cases remain separate from mission-family reporting.",
        ],
    }
    plan_path = generated_dir / "batch_plan.json"
    write_json(plan_path, plan)
    if args.plan_only:
        print(f"batch plan: {plan_path}")
        return

    dynamic_script = DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py"
    for role, _value, id_suffix, stem_suffix in dynamic_rows:
        group = f"dynamic_dataset_{batch_tag}_{role}"
        command = [
            sys.executable, dynamic_script,
            "--config", dynamic_configs[role],
            "--group", group,
            "--id-prefix", f"{batch_tag}_{id_suffix}",
            "--stem-prefix", f"{batch_tag}_{stem_suffix}",
        ]
        run(command)
        if install_dir is not None:
            run(
                [
                    sys.executable,
                    DATASET_ROOT / "scripts/install_dynamic_capture_set.py",
                    DATASET_ROOT / "manifests" / f"{group}.json",
                    "--network-template", DEFAULT_NETWORK_TEMPLATE,
                    "--install-dir", install_dir,
                ]
            )

    static_script = DATASET_ROOT / "scripts/generate_static_component_set.py"
    for role in ("static", "static_reserve", "hard_static"):
        command = [sys.executable, static_script, "--component-spec", static_specs[role]]
        if install_dir is not None:
            command.extend(["--install-dir", install_dir])
        run(command)

    report_script = DATASET_ROOT / "scripts/report_static_component_coverage.py"
    run(
        [
            sys.executable,
            report_script,
            DATASET_ROOT / "manifests" / f"dynamic_dataset_{batch_tag}_static_sunny_t13.json",
            DATASET_ROOT / "manifests" / f"dynamic_dataset_{batch_tag}_hard_static_sunny_t13.json",
            "--output",
            DATASET_ROOT / "reports" / f"{batch_tag}_static_obstacle_coverage.json",
        ]
    )
    run(
        [
            sys.executable,
            DATASET_ROOT / "scripts/validate_dynamic_dataset_batch.py",
            "--batch-index",
            str(args.batch_index),
        ]
    )

    print(f"batch plan: {plan_path}")
    print("primary frames: 1500")
    reserve_frames = sum(RESERVE_QUOTAS.values())
    reserve_percent = 100.0 * reserve_frames / sum(PRIMARY_QUOTAS.values())
    print(f"reserve frames: {reserve_frames} ({reserve_percent:.1f}%)")


if __name__ == "__main__":
    main()
