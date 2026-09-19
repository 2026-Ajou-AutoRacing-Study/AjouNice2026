#!/usr/bin/env python3
"""Audit generated MORAI temporal scenarios before expensive live capture."""

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve(path, root):
    candidate = Path(path)
    return candidate if candidate.is_absolute() else root / candidate


def local_relation(ego, actor):
    yaw = math.radians(float(ego.get("yaw", 0.0)))
    dx = float(actor["x"]) - float(ego["x"])
    dy = float(actor["y"]) - float(ego["y"])
    longitudinal = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if longitudinal >= 5.0:
        sector = "front"
    elif longitudinal <= -5.0:
        sector = "rear"
    elif lateral >= 0.0:
        sector = "left"
    else:
        sector = "right"
    return longitudinal, lateral, sector


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--project-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    manifest = load_json(args.manifest)
    scene_rows = []
    actor_rows = []
    family_counts = Counter()
    split_counts = Counter()
    family_bins = defaultdict(set)
    family_crosswalks = defaultdict(set)
    family_assets = defaultdict(set)
    sector_counts = defaultdict(Counter)

    for scene in manifest["scenes"]:
        metadata = load_json(resolve(scene["metadata"], args.project_root))
        family = metadata["mission_family"]
        split = metadata["split"]
        ego = metadata["ego"]["pose"]
        spatial = metadata.get("spatial_sampling", {})
        family_counts[family] += 1
        split_counts[split] += 1
        if spatial.get("route_wide_bin") is not None:
            family_bins[family].add(int(spatial["route_wide_bin"]))
        if spatial.get("crosswalk_id"):
            family_crosswalks[family].add(spatial["crosswalk_id"])
        scene_rows.append({
            "scene_id": metadata["scene_id"],
            "family": family,
            "split": split,
            "ego_x_m": float(ego["x"]),
            "ego_y_m": float(ego["y"]),
            "ego_z_m": float(ego["z"]),
            "ego_yaw_deg": float(ego.get("yaw", 0.0)),
            "route_arc_m": spatial.get("route_arc_m"),
            "route_wide_bin": spatial.get("route_wide_bin"),
            "crosswalk_id": spatial.get("crosswalk_id"),
            "actor_count": len(metadata.get("runtime_actors", [])),
        })
        for actor in metadata.get("runtime_actors", []):
            position = actor.get("position")
            if not position:
                continue
            longitudinal, lateral, sector = local_relation(ego, position)
            key = (actor.get("class", "unknown"), actor.get("name"), actor.get("data_id"))
            family_assets[family].add(key)
            sector_counts[actor.get("class", "unknown")][sector] += 1
            actor_rows.append({
                "scene_id": metadata["scene_id"],
                "family": family,
                "split": split,
                "class": actor.get("class", "unknown"),
                "name": actor.get("name"),
                "data_id": actor.get("data_id"),
                "x_m": float(position["x"]),
                "y_m": float(position["y"]),
                "z_m": float(position["z"]),
                "longitudinal_m": longitudinal,
                "lateral_m": lateral,
                "sector": sector,
            })

    route_wide_families = {
        row["family"] for row in scene_rows if row["route_wide_bin"] not in (None, "")
    }
    checks = {
        "exact_188_clips": len(scene_rows) == 188,
        "exact_1504_frames": len(scene_rows) * int(manifest["capture_policy"]["captures_per_scene"]) == 1504,
        "static_obstacle_uses_all_24_bins": len(family_bins["static_obstacle"]) == 24,
        "route_wide_static_union_uses_all_24_bins": len(set().union(*(family_bins[name] for name in route_wide_families))) == 24,
        "standing_pedestrian_has_multiple_locations": len(family_crosswalks["standing_pedestrian"]) >= 4,
        "standing_pedestrian_has_multiple_assets": len(family_assets["standing_pedestrian"]) >= 4,
        "empty_background_exists": any(row["family"] == "sparse_background" and row["actor_count"] == 0 for row in scene_rows),
    }
    report = {
        "schema_version": 1,
        "dataset_id": manifest["dataset_id"],
        "manifest": str(args.manifest.resolve()),
        "clip_count": len(scene_rows),
        "frame_count": len(scene_rows) * int(manifest["capture_policy"]["captures_per_scene"]),
        "family_clip_counts": dict(sorted(family_counts.items())),
        "split_clip_counts": dict(sorted(split_counts.items())),
        "family_route_bin_counts": {key: len(value) for key, value in sorted(family_bins.items())},
        "family_crosswalk_counts": {key: len(value) for key, value in sorted(family_crosswalks.items())},
        "family_unique_asset_counts": {key: len(value) for key, value in sorted(family_assets.items())},
        "class_view_sectors": {key: dict(value) for key, value in sorted(sector_counts.items())},
        "checks": checks,
        "route_reconstruction": manifest.get("generation", {}).get(
            "route_reconstruction"
        ),
        "status": "pass" if all(checks.values()) else "fail",
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "generated_scenes.csv", scene_rows)
    write_csv(args.output_dir / "generated_actors.csv", actor_rows)
    route_scene = next(
        (
            scene for scene in manifest["scenes"]
            if scene.get("sampling_role", "").startswith("base3_route_wide_")
            and scene.get("waypoints")
        ),
        None,
    )
    if route_scene is not None:
        route_rows = []
        with resolve(route_scene["waypoints"], args.project_root).open(
            "r", encoding="utf-8"
        ) as stream:
            for line in stream:
                fields = line.split()
                if len(fields) >= 2:
                    route_rows.append({
                        "x_m": float(fields[0]),
                        "y_m": float(fields[1]),
                        "z_m": float(fields[2]) if len(fields) >= 3 else 0.0,
                    })
        write_csv(args.output_dir / "generated_route.csv", route_rows)
    with (args.output_dir / "audit.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("status: {}".format(report["status"]))
    print("clips/frames: {}/{}".format(report["clip_count"], report["frame_count"]))
    print("report: {}".format(args.output_dir / "audit.json"))
    if report["status"] != "pass":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
