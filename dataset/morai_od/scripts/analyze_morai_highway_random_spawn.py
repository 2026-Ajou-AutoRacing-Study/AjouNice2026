#!/usr/bin/env python3
"""Measure MORAI highway RandomMode vehicle births from an Object_topic bag.

The statistic is event based, not frame-occupancy based.  A vehicle is counted
when its ID first appears, or when the same ID returns after a sufficiently long
absence.  Only births close to one of the requested random highway spawn points
are retained; the fixed roundabout spawn is therefore excluded.
"""

import argparse
import json
import glob
import math
from collections import Counter, defaultdict
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=Path("configs/scenario/2026_molit_comp_sample_scene.json"),
    )
    parser.add_argument(
        "--catalog",
        type=Path,
        default=Path("dataset/morai_od/configs/morai_asset_catalog_260807.json"),
    )
    parser.add_argument("--spawn-ids", default="6,8,10")
    parser.add_argument("--max-spawn-lateral-distance-m", type=float, default=15.0)
    parser.add_argument("--reappearance-gap-sec", type=float, default=2.0)
    parser.add_argument(
        "--reference-glob",
        default=(
            "/home/luv/morai_cap/260920_032434_base3_online_highway_static_v2/"
            "**/dynamic_metadata/*.frames.jsonl"
        ),
        help="controlled metadata used to map ObjectStatus size to asset name",
    )
    parser.add_argument("--size-match-tolerance-m", type=float, default=0.03)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    return parser.parse_args()


def pct(value, total):
    return 100.0 * value / total if total else 0.0


def table(counter, total, key_header):
    rows = ["| {} | Count | Share |".format(key_header), "|---|---:|---:|"]
    for key, value in sorted(counter.items(), key=lambda item: (-item[1], item[0])):
        rows.append("| `{}` | {} | {:.2f}% |".format(key, value, pct(value, total)))
    return rows


def build_size_reference(pattern):
    observations = defaultdict(Counter)
    for path in glob.glob(pattern, recursive=True):
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                frame = json.loads(line)
                for actor in frame.get("objects", {}).get("actors", []):
                    if actor.get("class") != "vehicle":
                        continue
                    size = actor.get("size_m") or {}
                    signature = tuple(round(float(size[key]), 3) for key in ("x", "y", "z"))
                    observations[actor["name"]][signature] += 1
    reference = defaultdict(list)
    for name, counts in observations.items():
        signature = counts.most_common(1)[0][0]
        reference[signature].append(name)
    return reference


def infer_asset(size, reference, tolerance):
    candidates = []
    for signature, names in reference.items():
        error = math.sqrt(sum((size[i] - signature[i]) ** 2 for i in range(3)))
        if error <= tolerance:
            candidates.append((error, signature, names))
    if not candidates:
        return None, [], None
    error, _, names = min(candidates, key=lambda item: item[0])
    return names[0] if len(names) == 1 else None, sorted(names), error


def main():
    args = parse_args()
    import rosbag

    scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
    catalog = json.loads(args.catalog.read_text(encoding="utf-8"))
    requested = {int(value) for value in args.spawn_ids.split(",") if value.strip()}
    spawns = {
        int(row["UNIQUEID"]): (float(row["pos"]["x"]), float(row["pos"]["y"]))
        for row in scenario["spawnPointList"]
        if int(row["UNIQUEID"]) in requested
    }
    if set(spawns) != requested:
        raise ValueError("missing spawn IDs: {}".format(sorted(requested - set(spawns))))

    asset_by_name = {row["name"]: row for row in catalog["vehicles"]}
    size_reference = build_size_reference(args.reference_glob)
    if not size_reference:
        raise RuntimeError("no vehicle size references matched: {}".format(args.reference_glob))

    last_seen = {}
    active_previous = set()
    births = []
    rejected = []
    message_count = 0
    first_stamp = None
    last_stamp = None

    with rosbag.Bag(str(args.bag.resolve()), "r") as bag:
        for _, message, stamp in bag.read_messages(topics=["/Object_topic"]):
            now = stamp.to_sec()
            first_stamp = now if first_stamp is None else first_stamp
            last_stamp = now
            message_count += 1
            active_now = set()
            for actor in message.npc_list:
                uid = int(actor.unique_id)
                active_now.add(uid)
                previous = last_seen.get(uid)
                is_birth = uid not in active_previous and (
                    previous is None or now - previous >= args.reappearance_gap_sec
                )
                last_seen[uid] = now
                if not is_birth:
                    continue

                x = float(actor.position.x)
                y = float(actor.position.y)
                nearest_id, nearest_distance = min(
                    (
                        (spawn_id, abs(x - pos[0]))
                        for spawn_id, pos in spawns.items()
                    ),
                    key=lambda item: item[1],
                )
                reported_name = str(actor.name).strip()
                size = (float(actor.size.x), float(actor.size.y), float(actor.size.z))
                if reported_name in asset_by_name:
                    inferred_name = reported_name
                    candidates = [reported_name]
                    size_error = 0.0
                else:
                    inferred_name, candidates, size_error = infer_asset(
                        size, size_reference, args.size_match_tolerance_m
                    )
                asset = asset_by_name.get(inferred_name) if inferred_name else None
                source_spawn = (
                    nearest_id
                    if abs(y - spawns[nearest_id][1]) <= 50.0
                    else "downstream_unassigned"
                )
                row = {
                    "stamp": now,
                    "unique_id": uid,
                    "reported_name": reported_name or "<empty>",
                    "name": inferred_name or "<ambiguous-or-unmatched>",
                    "candidate_names": candidates,
                    "size_m": size,
                    "size_match_error_m": size_error,
                    "x": x,
                    "y": y,
                    "nearest_spawn_id": source_spawn,
                    "spawn_lateral_distance_m": nearest_distance,
                    "catalog_match": asset is not None,
                    "data_id": asset.get("data_id") if asset else None,
                    "family": asset.get("subtype", "unknown") if asset else "unknown",
                }
                if nearest_distance <= args.max_spawn_lateral_distance_m:
                    births.append(row)
                else:
                    rejected.append(row)
            active_previous = active_now

    name_counts = Counter(row["name"] for row in births)
    family_counts = Counter(row["family"] for row in births)
    spawn_counts = Counter(str(row["nearest_spawn_id"]) for row in births)
    per_spawn_names = defaultdict(Counter)
    for row in births:
        per_spawn_names[str(row["nearest_spawn_id"])][row["name"]] += 1

    result = {
        "schema_version": 1,
        "bag": str(args.bag.resolve()),
        "scenario": str(args.scenario.resolve()),
        "object_topic_messages": message_count,
        "duration_sec": (last_stamp - first_stamp) if first_stamp is not None else 0.0,
        "settings": {
            "spawn_ids": sorted(spawns),
            "spawn_positions": {str(key): value for key, value in spawns.items()},
            "max_spawn_lateral_distance_m": args.max_spawn_lateral_distance_m,
            "reappearance_gap_sec": args.reappearance_gap_sec,
            "reference_glob": args.reference_glob,
            "size_match_tolerance_m": args.size_match_tolerance_m,
            "reference_signatures": len(size_reference),
        },
        "accepted_births": len(births),
        "rejected_births": len(rejected),
        "unique_asset_names": len(name_counts),
        "catalog_unmatched_births": sum(not row["catalog_match"] for row in births),
        "name_counts": dict(name_counts),
        "family_counts": dict(family_counts),
        "spawn_counts": dict(spawn_counts),
        "per_spawn_name_counts": {
            key: dict(value) for key, value in per_spawn_names.items()
        },
        "births": births,
        "rejected": rejected,
    }

    output_json = args.output_json or args.bag.with_suffix(".spawn_stats.json")
    output_md = args.output_md or args.bag.with_suffix(".spawn_stats.md")
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_md.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    lines = [
        "# MORAI Highway Random Spawn Statistics",
        "",
        "- Bag: `{}`".format(result["bag"]),
        "- Duration: `{:.1f} s`".format(result["duration_sec"]),
        "- Accepted unique birth events: `{}`".format(len(births)),
        "- Rejected births outside spawn gate: `{}`".format(len(rejected)),
        "- Unique observed models: `{}`".format(len(name_counts)),
        "- Catalog-unmatched births: `{}`".format(result["catalog_unmatched_births"]),
        "",
        "## Model distribution",
        "",
        *table(name_counts, len(births), "Model"),
        "",
        "## Family distribution",
        "",
        *table(family_counts, len(births), "Family"),
        "",
        "## Source spawn distribution",
        "",
        *table(spawn_counts, len(births), "Spawn ID"),
        "",
    ]
    output_md.write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "duration_sec", "accepted_births", "rejected_births",
        "unique_asset_names", "catalog_unmatched_births")}, indent=2))
    print("json={}".format(output_json))
    print("markdown={}".format(output_md))


if __name__ == "__main__":
    main()
