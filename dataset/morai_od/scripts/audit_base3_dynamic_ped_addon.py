#!/usr/bin/env python3
"""Audit the generated or captured Base3 dynamic-pedestrian add-on."""

import argparse
import collections
import json
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
EXPECTED_LOCATIONS = {
    "CW000000": 19,
    "CW000002": 19,
    "CW000013": 11,
    "CW000015": 11,
    "CW000001": 8,
    "CW000014": 7,
}
EXPECTED_SPLITS = {"train": 53, "val": 11, "test": 11}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument(
        "--capture-dir",
        type=Path,
        action="append",
        default=[],
        help="Capture run directory. Repeat to combine primary and replacement runs.",
    )
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def resolve(path):
    value = Path(path)
    return value if value.is_absolute() else PROJECT_ROOT / value


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
    for scene in manifest.get("scenes", []):
        metadata = load_json(resolve(scene["metadata"]))
        scenario = load_json(resolve(scene["scenario"]))
        actors = metadata.get("runtime_actors", [])
        if len(actors) != 1 or actors[0].get("class") != "pedestrian":
            errors.append("{}: expected exactly one pedestrian runtime actor".format(scene["id"]))
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
            errors.append("{}: scenario pedestrian path count is not one".format(scene["id"]))
        contract = scene.get("sequence_contract", {})
        if int(contract.get("clip_length", -1)) != 8:
            errors.append("{}: clip length is not eight".format(scene["id"]))
        if abs(float(contract.get("nominal_period_sec", 0.0)) - 1.0 / 12.0) > 1e-9:
            errors.append("{}: nominal period is not 1/12 sec".format(scene["id"]))

    checks = {
        "exact_75_clips": len(manifest.get("scenes", [])) == 75,
        "exact_600_planned_frames": len(manifest.get("scenes", [])) * 8 == 600,
        "location_quota": dict(locations) == EXPECTED_LOCATIONS,
        "split_quota": dict(splits) == EXPECTED_SPLITS,
        "all_22_assets": len(assets) == 22 and min(assets.values(), default=0) >= 3,
        "balanced_direction": abs(directions["forward"] - directions["reverse"]) <= 1,
        "ego_motion_quota": ego_profiles["competition_motion"] == 56
        and ego_profiles["very_low_speed"] == 19,
        "motion_profile_quota": sum(
            value for key, value in profiles.items() if str(key).startswith("walking_")
        ) == 34
        and sum(
            value for key, value in profiles.items() if str(key).startswith("running_")
        ) == 34
        and profiles["activation_transition"] == 7,
    }
    for name, passed in checks.items():
        if not passed:
            errors.append("failed check: {}".format(name))

    capture = None
    if args.capture_dir:
        completed_ids = set()
        failed_ids = set()
        run_rows = []
        for capture_dir in args.capture_dir:
            summary_path = capture_dir / "summary.json"
            if not summary_path.exists():
                errors.append("capture summary is missing: {}".format(capture_dir))
                continue
            summary = load_json(summary_path)
            run_completed = set(summary.get("completed_scene_ids", []))
            run_failed = set(summary.get("failed_scene_ids", []))
            completed_ids.update(run_completed)
            failed_ids.update(run_failed)
            run_rows.append(
                {
                    "capture_dir": str(capture_dir.resolve()),
                    "completed_clips": len(run_completed),
                    "failed_clips": len(run_failed),
                    "frames": int(summary.get("completed_captures", 0)),
                }
            )
        failed_ids.difference_update(completed_ids)
        capture = {
            "runs": run_rows,
            "completed_clips": len(completed_ids),
            "failed_clips": len(failed_ids),
            "frames": len(completed_ids) * 8,
            "completed_scene_ids": sorted(completed_ids),
            "unresolved_failed_scene_ids": sorted(failed_ids),
        }
        expected_ids = {scene["id"] for scene in manifest.get("scenes", [])}
        if completed_ids != expected_ids:
            errors.append(
                "capture scene set mismatch: missing={}, extra={}".format(
                    sorted(expected_ids - completed_ids),
                    sorted(completed_ids - expected_ids),
                )
            )

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
