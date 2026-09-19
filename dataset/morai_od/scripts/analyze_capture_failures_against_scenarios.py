#!/usr/bin/env python3
"""Correlate streamed MORAI failures with generated scenarios.

The console log is processed one line at a time. Scenario JSON files are opened
one at a time only for actor-count mismatch scenes. The output contains compact
frequency tables rather than raw log messages.
"""

import argparse
import ast
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path


FAILURE_RE = re.compile(
    r"Scene\s+(?P<scene>\S+)\s+failed and will be skipped:\s+(?P<reason>.*)$"
)
IDENTITIES_RE = re.compile(
    r"actual object identities=(?P<identities>\{.*\}); expected objects="
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--console-log", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--asset-catalog", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--top", type=int, default=20)
    return parser.parse_args()


def classify(reason):
    lowered = reason.lower()
    if "object counts" in lowered and "!=" in lowered:
        return "actor_count_mismatch"
    if "collision" in lowered:
        return "collision"
    if "did not become ready" in lowered:
        return "scenario_not_ready_other"
    return "other"


def actor_catalog(path):
    document = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for group in (
        "vehicles",
        "pedestrians_and_related",
        "obstacles_and_related",
    ):
        for actor in document.get(group, []):
            result[int(actor["data_id"])] = actor.get("name", "unknown")
    return result


def scene_number(scene_id):
    match = re.search(r"(\d+)$", scene_id)
    return int(match.group(1)) if match else None


def sequence_block(scene_id, width=100):
    number = scene_number(scene_id)
    if number is None:
        return "unknown"
    start = ((number - 1) // width) * width + 1
    return f"{start:03d}-{start + width - 1:03d}"


def relative_geometry(ego, actor):
    position = actor.get("initPosition", {}).get("pos", {})
    dx = float(position.get("x", 0.0)) - float(ego["x"])
    dy = float(position.get("y", 0.0)) - float(ego["y"])
    yaw = math.radians(float(ego["yaw"]))
    longitudinal = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    distance = math.hypot(dx, dy)
    if longitudinal >= 0 and abs(longitudinal) >= abs(lateral):
        sector = "front"
    elif longitudinal < 0 and abs(longitudinal) >= abs(lateral):
        sector = "rear"
    elif lateral >= 0:
        sector = "left"
    else:
        sector = "right"
    if distance < 5:
        distance_bin = "0-5m"
    elif distance < 10:
        distance_bin = "5-10m"
    elif distance < 20:
        distance_bin = "10-20m"
    elif distance < 30:
        distance_bin = "20-30m"
    else:
        distance_bin = "30m+"
    return sector, distance_bin


def counter_rows(counter, denominator=None, top=20, label="key"):
    rows = []
    for key, count in counter.most_common(top):
        row = {label: key, "count": count}
        if denominator is not None:
            total = denominator.get(key, 0)
            row["expected_placements"] = total
            row["missing_rate"] = round(count / total, 4) if total else None
        rows.append(row)
    return rows


def main():
    args = parse_args()
    manifest_path = args.manifest.expanduser().resolve()
    repository_root = manifest_path.parents[3]
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    scenes = {scene["id"]: scene for scene in manifest["scenes"]}
    catalog = actor_catalog(args.asset_catalog.expanduser().resolve())

    expected_asset_placements = Counter()
    expected_asset_by_block = Counter()
    expected_npc_counts = Counter()
    all_scene_type = Counter()
    all_topology = Counter()
    all_links = Counter()
    for scene in scenes.values():
        all_scene_type[scene.get("scene_type", "unknown")] += 1
        all_topology[scene.get("topology", "unknown")] += 1
        all_links[scene.get("expected_ego", {}).get("link_idx", "unknown")] += 1
        expected_npc_counts[int(scene["expected_counts"]["npc"])] += 1
        scenario_path = repository_root / scene["scenario"]
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        for actor in scenario.get("vehicleList", []):
            data_id = int(actor["DataID"])
            asset_key = (data_id, catalog.get(data_id, "unknown"))
            expected_asset_placements[asset_key] += 1
            expected_asset_by_block[(asset_key, sequence_block(scene["id"]))] += 1

    failure_category = Counter()
    failure_by_block = Counter()
    mismatch_scene_type = Counter()
    mismatch_topology = Counter()
    mismatch_links = Counter()
    mismatch_expected_npc_count = Counter()
    missing_assets = Counter()
    missing_asset_scenes = defaultdict(set)
    missing_asset_by_block = Counter()
    missing_sectors = Counter()
    missing_distances = Counter()
    identity_parse_failures = []
    failure_ids = set()

    source = args.console_log.expanduser().resolve()
    with source.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = FAILURE_RE.search(line)
            if not match:
                continue
            scene_id = match.group("scene")
            reason = match.group("reason").strip()
            category = classify(reason)
            failure_ids.add(scene_id)
            failure_category[category] += 1
            failure_by_block[(sequence_block(scene_id), category)] += 1
            if category != "actor_count_mismatch" or scene_id not in scenes:
                continue

            meta = scenes[scene_id]
            mismatch_scene_type[meta.get("scene_type", "unknown")] += 1
            mismatch_topology[meta.get("topology", "unknown")] += 1
            mismatch_links[meta.get("expected_ego", {}).get("link_idx", "unknown")] += 1
            mismatch_expected_npc_count[int(meta["expected_counts"]["npc"])] += 1
            identities = IDENTITIES_RE.search(reason)
            if not identities:
                identity_parse_failures.append(scene_id)
                continue
            try:
                observed = ast.literal_eval(identities.group("identities"))
            except (SyntaxError, ValueError):
                identity_parse_failures.append(scene_id)
                continue
            observed_ids = {
                int(actor["unique_id"])
                for actor in observed.get("npc", [])
                if "unique_id" in actor
            }
            scenario_path = repository_root / meta["scenario"]
            scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
            ego = meta["expected_ego"]
            for actor in scenario.get("vehicleList", []):
                unique_id = int(actor["UNIQUEID"])
                if unique_id in observed_ids:
                    continue
                data_id = int(actor["DataID"])
                asset_key = (data_id, catalog.get(data_id, "unknown"))
                missing_assets[asset_key] += 1
                missing_asset_scenes[asset_key].add(scene_id)
                missing_asset_by_block[(asset_key, sequence_block(scene_id))] += 1
                sector, distance_bin = relative_geometry(ego, actor)
                missing_sectors[sector] += 1
                missing_distances[distance_bin] += 1

    asset_rows = []
    for asset_key, missing_count in missing_assets.most_common(args.top):
        expected = expected_asset_placements[asset_key]
        asset_rows.append({
            "data_id": asset_key[0],
            "name": asset_key[1],
            "missing_placements": missing_count,
            "affected_scenes": len(missing_asset_scenes[asset_key]),
            "expected_placements": expected,
            "missing_rate": round(missing_count / expected, 4) if expected else None,
            "by_sequence_block": [
                {
                    "block": block,
                    "missing": missing_asset_by_block[(asset_key, block)],
                    "expected": expected_asset_by_block[(asset_key, block)],
                    "missing_rate": round(
                        missing_asset_by_block[(asset_key, block)]
                        / expected_asset_by_block[(asset_key, block)],
                        4,
                    ) if expected_asset_by_block[(asset_key, block)] else None,
                }
                for block in (
                    "001-100", "101-200", "201-300",
                    "301-400", "401-500", "501-600",
                )
            ],
        })

    npc_count_rows = []
    for count in sorted(expected_npc_counts):
        failures = mismatch_expected_npc_count[count]
        total = expected_npc_counts[count]
        npc_count_rows.append({
            "expected_npc_count": count,
            "scenes": total,
            "mismatch_scenes": failures,
            "mismatch_rate": round(failures / total, 4) if total else None,
        })

    result = {
        "schema_version": 1,
        "processing": "console_streamed_and_scenarios_opened_individually",
        "source": {
            "console_log": str(source),
            "console_size_bytes": source.stat().st_size,
            "manifest": str(manifest_path),
            "scene_count": len(scenes),
        },
        "failure_categories": dict(failure_category),
        "failure_by_sequence_block": [
            {"block": block, "category": category, "count": count}
            for (block, category), count in sorted(failure_by_block.items())
        ],
        "actor_count_mismatch": {
            "identity_parse_failure_count": len(identity_parse_failures),
            "identity_parse_failure_samples": identity_parse_failures[:10],
            "missing_vehicle_placements_parsed": sum(missing_assets.values()),
            "missing_vehicle_by_asset": asset_rows,
            "missing_vehicle_by_ego_sector": dict(missing_sectors),
            "missing_vehicle_by_distance": dict(missing_distances),
            "mismatch_rate_by_expected_npc_count": npc_count_rows,
            "mismatch_scene_type_top": counter_rows(
                mismatch_scene_type, all_scene_type, args.top, "scene_type"
            ),
            "mismatch_topology": counter_rows(
                mismatch_topology, all_topology, args.top, "topology"
            ),
            "mismatch_ego_link_top": counter_rows(
                mismatch_links, all_links, args.top, "link_idx"
            ),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("analysis:", args.output)
    print("failed scenes:", len(failure_ids))
    print("categories:", dict(failure_category))
    print("parsed missing vehicle placements:", sum(missing_assets.values()))
    print("identity parse failures:", len(identity_parse_failures))


if __name__ == "__main__":
    main()
