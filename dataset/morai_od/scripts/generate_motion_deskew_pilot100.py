#!/usr/bin/env python3
"""Generate and install a 100-frame motion-focused deskew pilot."""

import copy
import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "dataset/morai_od"
BASE_CONFIG = DATASET_ROOT / "configs/dynamic_object_capture_v1.json"
OUTPUT_ROOT = DATASET_ROOT / "metadata/deskew_pilot100"
GENERATED_CONFIG = OUTPUT_ROOT / "deskew_pilot100.json"
GROUP = "motion_deskew_pilot100"
MANIFEST = DATASET_ROOT / "manifests/{}.json".format(GROUP)
RESERVE_CONFIG = OUTPUT_ROOT / "deskew_pilot100_reserve.json"
RESERVE_GROUP = "motion_deskew_pilot100_reserve"
RESERVE_MANIFEST = DATASET_ROOT / "manifests/{}.json".format(RESERVE_GROUP)
INSTALL_DIR = (
    Path.home()
    / "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/"
    / "R_KR_PR_K-city_2025"
)
NETWORK_TEMPLATE = INSTALL_DIR / "EgoNetwork/od_s13p6_186_MN.json"


FAMILY_COUNTS = {
    # Four 10-frame clips: high translation speed and relative vehicle motion.
    "highway_cruise_overtake": 4,
    # Three clips: sustained Ego yaw rate and curved Lanelet geometry.
    "roundabout_circulation": 3,
    # Two clips: Ego/NPC relative motion near highway merge.
    "highway_merge_cruise": 2,
    # One clip: moving pedestrian label preservation.
    "crosswalk_pedestrian": 1,
}


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def run(command):
    print("+", " ".join(str(item) for item in command), flush=True)
    subprocess.run(
        [str(item) for item in command],
        cwd=PROJECT_ROOT,
        check=True,
    )


def main():
    base = read_json(BASE_CONFIG)
    families = []
    for name, count in FAMILY_COUNTS.items():
        family = copy.deepcopy(
            next(item for item in base["families"] if item["name"] == name)
        )
        family["count"] = count
        families.append(family)

    config = copy.deepcopy(base)
    config["dataset_id"] = "morai_motion_deskew_pilot100"
    config["seed"] = 260821100
    config["capture"]["captures_per_scene"] = 10
    config["capture"]["clip_duration_sec"] = 4.5
    config["families"] = families
    write_json(GENERATED_CONFIG, config)

    run(
        [
            sys.executable,
            DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py",
            "--config",
            GENERATED_CONFIG,
            "--group",
            GROUP,
            "--id-prefix",
            "dp100_",
            "--stem-prefix",
            "od_dp100_",
        ]
    )

    # Two independent 10-frame clips provide replacements for a launcher
    # omission, collision, or route-control failure without duplicating a
    # primary scenario. They are excluded from the 100-frame target unless
    # explicitly captured as a replacement.
    reserve = copy.deepcopy(base)
    reserve["dataset_id"] = "morai_motion_deskew_pilot100_reserve"
    reserve["seed"] = 260821101
    reserve["capture"]["captures_per_scene"] = 10
    reserve["capture"]["clip_duration_sec"] = 4.5
    reserve["families"] = []
    for name in ("highway_cruise_overtake", "roundabout_circulation"):
        family = copy.deepcopy(
            next(item for item in base["families"] if item["name"] == name)
        )
        family["count"] = 1
        reserve["families"].append(family)
    write_json(RESERVE_CONFIG, reserve)
    run(
        [
            sys.executable,
            DATASET_ROOT / "scripts/generate_dynamic_object_capture_set.py",
            "--config",
            RESERVE_CONFIG,
            "--group",
            RESERVE_GROUP,
            "--id-prefix",
            "dp100r_",
            "--stem-prefix",
            "od_dp100r_",
        ]
    )
    run(
        [
            sys.executable,
            DATASET_ROOT / "scripts/install_dynamic_capture_set.py",
            RESERVE_MANIFEST,
            "--network-template",
            NETWORK_TEMPLATE,
            "--install-dir",
            INSTALL_DIR,
        ]
    )
    run(
        [
            sys.executable,
            DATASET_ROOT / "scripts/install_dynamic_capture_set.py",
            MANIFEST,
            "--network-template",
            NETWORK_TEMPLATE,
            "--install-dir",
            INSTALL_DIR,
        ]
    )

    plan = {
        "schema_version": 1,
        "dataset_id": config["dataset_id"],
        "seed": config["seed"],
        "purpose": "raw versus azimuth-estimated Ego-deskew engineering pilot",
        "capture_mode": "timed-grouped SaveSensorData",
        "frames_per_clip": 10,
        "family_clip_counts": FAMILY_COUNTS,
        "target_frames": 100,
        "split_unit": "scenario_clip",
        "environment": config["environment"],
        "manifest": str(MANIFEST.relative_to(PROJECT_ROOT)),
        "reserve_manifest": str(RESERVE_MANIFEST.relative_to(PROJECT_ROOT)),
        "reserve_frames": 20,
    }
    write_json(OUTPUT_ROOT / "pilot_plan.json", plan)
    print("manifest: {}".format(MANIFEST))
    print("target frames: 100")


if __name__ == "__main__":
    main()
