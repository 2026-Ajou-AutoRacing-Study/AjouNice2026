#!/usr/bin/env python3
"""Explain MORAI batch failures and rank repeatedly missing actor assets."""

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path


GROUP_TO_ACTUAL = {
    "vehicleList": "npc",
    "pedestrianList": "pedestrian",
    "objectList": "obstacle",
}


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main():
    args = parse_args()
    report = load_json(args.report)
    manifest = load_json(args.manifest)
    manifest_scenes = {item["id"]: item for item in manifest["scenes"]}

    expected_by_asset = Counter()
    missing_by_asset = Counter()
    asset_info = {}
    failure_classes = Counter()
    failure_scene_ids = defaultdict(list)

    for result in report["scenes"]:
        scene = manifest_scenes[result["id"]]
        metadata = load_json(scene["metadata"])
        expected_names = defaultdict(Counter)
        for asset in metadata["assets"]:
            key = (asset["group"], int(asset["data_id"]), asset["name"])
            expected_by_asset[key] += 1
            asset_info[(asset["group"], asset["name"])] = key
            expected_names[asset["group"]][asset["name"]] += 1

        if result["status"] == "completed":
            continue
        error = result.get("error", "")
        total_expected = sum(result.get("expected_counts", {}).values())
        if total_expected == 0 and "relocated capture file is invalid" in error:
            category = "valid_empty_negative_rejected"
        elif "did not become ready" in error:
            category = "scene_actor_or_pose_mismatch"
        elif "rolling-grouped produced" in error:
            category = "sensor_artifact_group_incomplete"
        else:
            category = "other"
        failure_classes[category] += 1
        failure_scene_ids[category].append(result["id"])

        identities = result.get("actual_objects", {}).get("identities", {})
        for group, actual_group in GROUP_TO_ACTUAL.items():
            actual_names = Counter(
                item.get("name", "") for item in identities.get(actual_group, [])
            )
            for name, count in (expected_names[group] - actual_names).items():
                key = asset_info.get((group, name))
                if key is not None:
                    missing_by_asset[key] += count

    rows = []
    for key, expected in expected_by_asset.items():
        missing = missing_by_asset[key]
        if missing:
            rows.append(
                {
                    "group": key[0],
                    "data_id": key[1],
                    "name": key[2],
                    "expected": expected,
                    "missing": missing,
                    "missing_rate": round(missing / expected, 4),
                }
            )
    rows.sort(key=lambda item: (-item["missing"], -item["missing_rate"]))
    output = {
        "schema_version": 1,
        "report": str(args.report),
        "manifest": str(args.manifest),
        "summary": report["summary"],
        "failure_classes": dict(failure_classes),
        "failure_scene_ids": dict(failure_scene_ids),
        "missing_assets": rows,
    }
    rendered = json.dumps(output, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
