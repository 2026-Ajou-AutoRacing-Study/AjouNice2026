#!/usr/bin/env python3
"""Audit Base3-Static Batch 008 moving-pedestrian generation and capture."""

import argparse
import collections
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_LOCATIONS = {
    "CW000000": 75,
    "CW000002": 60,
    "CW000013": 45,
    "CW000015": 45,
    "CW000001": 38,
    "CW000014": 37,
}
EXPECTED_SPLITS = {"train": 210, "val": 45, "test": 45}
EXPECTED_PROFILES = {"walking": 135, "running": 135, "activation_transition": 30}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--capture-dir", type=Path, action="append", default=[])
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else PROJECT_ROOT / path


def main():
    args = parse_args()
    manifest = load_json(args.manifest)
    errors = []
    locations = collections.Counter()
    splits = collections.Counter()
    assets = collections.Counter()
    profiles = collections.Counter()
    ego_profiles = collections.Counter()
    directions = collections.Counter()

    policy = manifest.get("capture_policy", {})
    if policy.get("format") != "morai_save_sensor_data_rolling_grouped":
        errors.append("capture format is not rolling-grouped")
    if int(policy.get("captures_per_scene", -1)) != 3:
        errors.append("capture count is not three frames per clip")
    if abs(float(policy.get("capture_spacing_m", 0.0)) - 3.0) > 1e-9:
        errors.append("capture spacing is not 3 m")
    if policy.get("split_unit") != "static_layout_clip":
        errors.append("split unit is not static_layout_clip")

    for scene in manifest.get("scenes", []):
        metadata = load_json(resolve(scene["metadata"]))
        scenario = load_json(resolve(scene["scenario"]))
        actors = metadata.get("runtime_actors", [])
        if len(actors) != 1 or actors[0].get("class") != "pedestrian":
            errors.append("{}: expected one pedestrian".format(scene["id"]))
            continue
        actor = actors[0]
        locations[actor.get("crosswalk_id")] += 1
        splits[metadata.get("split")] += 1
        assets[actor.get("name")] += 1
        profiles[actor.get("difficulty_profile")] += 1
        directions["reverse" if actor.get("reversed") else "forward"] += 1
        ego_profiles[
            metadata.get("capture_policy_overrides", {}).get("ego_motion_profile")
        ] += 1
        if len(scenario.get("pedestrianList", [])) != 1:
            errors.append("{}: scenario pedestrian count is not one".format(scene["id"]))
        if len(scenario.get("waypointDataList", [])) != 1:
            errors.append("{}: scenario path count is not one".format(scene["id"]))
        contract = scene.get("sequence_contract", {})
        if int(contract.get("clip_length", -1)) != 3:
            errors.append("{}: manifest clip length is not three".format(scene["id"]))

    checks = {
        "exact_300_clips": len(manifest.get("scenes", [])) == 300,
        "exact_900_frames": len(manifest.get("scenes", [])) * 3 == 900,
        "location_quota": dict(locations) == EXPECTED_LOCATIONS,
        "split_quota": dict(splits) == EXPECTED_SPLITS,
        "all_22_assets": len(assets) == 22 and min(assets.values(), default=0) >= 13,
        "balanced_direction": directions == {"forward": 150, "reverse": 150},
        "ego_motion_quota": ego_profiles == {
            "competition_motion": 225,
            "slow_crosswalk_approach": 75,
        },
        "motion_profile_quota": dict(profiles) == EXPECTED_PROFILES,
    }
    errors.extend("failed check: {}".format(name) for name, ok in checks.items() if not ok)

    capture = None
    if args.capture_dir:
        completed = set()
        failed = set()
        runs = []
        for capture_dir in args.capture_dir:
            summary_path = capture_dir / "summary.json"
            if not summary_path.exists():
                errors.append("capture summary missing: {}".format(capture_dir))
                continue
            summary = load_json(summary_path)
            run_completed = set(summary.get("completed_scene_ids", []))
            run_failed = set(summary.get("failed_scene_ids", []))
            completed.update(run_completed)
            failed.update(run_failed)
            runs.append(
                {
                    "capture_dir": str(capture_dir.resolve()),
                    "completed_clips": len(run_completed),
                    "failed_clips": len(run_failed),
                    "completed_captures": int(summary.get("completed_captures", 0)),
                }
            )
        failed.difference_update(completed)
        expected = {scene["id"] for scene in manifest.get("scenes", [])}
        if completed != expected:
            errors.append(
                "capture scene mismatch: missing={}, extra={}".format(
                    sorted(expected - completed), sorted(completed - expected)
                )
            )
        capture = {
            "runs": runs,
            "completed_clips": len(completed),
            "frames": len(completed) * 3,
            "unresolved_failed_scene_ids": sorted(failed),
        }

    report = {
        "manifest": str(args.manifest.resolve()),
        "checks": checks,
        "locations": dict(locations),
        "splits": dict(splits),
        "assets": dict(assets),
        "profiles": dict(profiles),
        "ego_profiles": dict(ego_profiles),
        "directions": dict(directions),
        "capture": capture,
        "errors": errors,
        "status": "pass" if not errors else "fail",
    }
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    raise SystemExit(0 if not errors else 1)


if __name__ == "__main__":
    main()
