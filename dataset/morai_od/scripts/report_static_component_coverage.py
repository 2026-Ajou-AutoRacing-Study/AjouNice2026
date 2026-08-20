#!/usr/bin/env python3
"""Summarize unique static placements by asset, shape proxy, and distance."""

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifests", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def read_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def shape_proxy(asset):
    subtype = str(asset.get("subtype", "unknown"))
    name = str(asset.get("name", "")).lower()
    if subtype in {"road_sign", "fixed_roadside_object"}:
        return "tall_or_vertical"
    if subtype == "barrier" and any(token in name for token in ("concrete", "guard", "fence")):
        return "long_or_wide"
    if subtype == "barrier":
        return "compact_barrier"
    if subtype == "unoccupied_bicycle_or_scooter":
        return "thin_irregular"
    return "other_irregular"


def distance_bucket(distance):
    if distance < 10.0:
        return "0_10m"
    if distance < 20.0:
        return "10_20m"
    if distance < 30.0:
        return "20_30m"
    return "30m_plus"


def main():
    args = parse_args()
    assets = Counter()
    shapes = Counter()
    distances = Counter()
    scene_types = Counter()
    unique_scene_assets = defaultdict(set)
    scene_count = 0
    obstacle_placements = 0

    for manifest_path in args.manifests:
        manifest = read_json(manifest_path)
        for scene in manifest["scenes"]:
            metadata = read_json(PROJECT_ROOT / scene["metadata"])
            scene_count += 1
            scene_types[metadata["scene_type"]] += 1
            for asset in metadata.get("assets", []):
                if asset.get("group") != "objectList":
                    continue
                obstacle_placements += 1
                key = (int(asset["data_id"]), str(asset["name"]))
                assets[key] += 1
                unique_scene_assets[scene["id"]].add(key)
                shapes[shape_proxy(asset)] += 1
                relation = asset.get("capture_layout_relation", {})
                distance = math.hypot(
                    float(relation.get("longitudinal_m", 0.0)),
                    float(relation.get("lateral_m", 0.0)),
                )
                distances[distance_bucket(distance)] += 1

    document = {
        "schema_version": 1,
        "counting_unit": "unique placement per scenario; three captured frames are not triple-counted",
        "scene_count": scene_count,
        "scene_types": dict(sorted(scene_types.items())),
        "obstacle_placements": obstacle_placements,
        "unique_obstacle_assets": len(assets),
        "assets": [
            {"data_id": key[0], "name": key[1], "placements": count}
            for key, count in sorted(assets.items())
        ],
        "shape_proxy": {
            "basis": "MORAI asset taxonomy/name proxy; authoritative GT dimensions are audited after SaveSensorData capture",
            "counts": dict(sorted(shapes.items())),
        },
        "distance_buckets": dict(sorted(distances.items())),
        "asset_compatibility": "generation uses the competition/personal launcher intersection and runtime exclusion list",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"scenes: {scene_count}")
    print(f"obstacle placements: {obstacle_placements}")
    print(f"unique obstacle assets: {len(assets)}")
    print(f"report: {args.output}")


if __name__ == "__main__":
    main()
