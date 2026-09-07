#!/usr/bin/env python3
"""Audit actual LiDAR evidence inside MORAI Instance.txt GT boxes."""

import argparse
import csv
import glob
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


def read_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_jsonl(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def parse_instances(path):
    rows = []
    if not path.is_file():
        return rows
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        fields = line.split()
        if len(fields) < 15:
            continue
        try:
            values = [float(value) for value in fields[1:14]]
            unique_id = int(fields[14])
        except ValueError:
            continue
        rows.append({
            "raw_class": fields[0],
            "unique_id": unique_id,
            "center": np.asarray(values[0:3], dtype=np.float32),
            "rotation_rpy": values[3:6],
            "size": np.asarray(values[6:9], dtype=np.float32),
        })
    return rows


def coarse_class(name):
    lowered = name.lower()
    if "pedestrian" in lowered or "person" in lowered:
        return "pedestrian"
    if lowered in {"vehicle", "sedan", "suv", "truck", "bus", "mpv", "wagon"} or "car" in lowered:
        return "vehicle"
    return "obstacle"


def points_in_box(points, box, tolerance_m=0.0):
    center = box["center"]
    size = box["size"]
    yaw = float(box["rotation_rpy"][2])
    relative = points[:, :3] - center
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * relative[:, 0] + sine * relative[:, 1]
    local_y = -sine * relative[:, 0] + cosine * relative[:, 1]
    return (
        (np.abs(local_x) <= size[0] * 0.5 + tolerance_m)
        & (np.abs(local_y) <= size[1] * 0.5 + tolerance_m)
        & (np.abs(relative[:, 2]) <= size[2] * 0.5 + tolerance_m)
    )


def point_bin(count):
    if count == 0:
        return "0"
    if count <= 4:
        return "1-4"
    if count <= 9:
        return "5-9"
    if count <= 19:
        return "10-19"
    return "20+"


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q)) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--source-metadata-dir", type=Path)
    parser.add_argument("--maximum-center-range-m", type=float, default=30.0)
    parser.add_argument("--minimum-positive-points", type=int, default=5)
    parser.add_argument("--box-tolerance-m", type=float, default=0.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    actor_by_scene_id = {}
    spatial_by_scene_id = {}
    if args.source_metadata_dir:
        for path in args.source_metadata_dir.glob("*.metadata.json"):
            metadata = read_json(path)
            actor_by_scene_id[metadata["scene_id"]] = {
                int(actor["unique_id"]): actor
                for actor in metadata.get("runtime_actors", [])
                if actor.get("unique_id") is not None
            }
            spatial_by_scene_id[metadata["scene_id"]] = (
                metadata.get("spatial_sampling") or {}
            )

    records = []
    for frames_path in sorted((args.run_dir / "dynamic_metadata").glob("*.frames.jsonl")):
        for frame in read_jsonl(frames_path):
            capture_id = str(frame["capture_id"])
            lidar_path = args.run_dir / "LIDAR_5" / (capture_id + "_Intensity.bin")
            instance_path = args.run_dir / "LIDAR_5" / (capture_id + "_Instance.txt")
            if not lidar_path.is_file() or not instance_path.is_file():
                continue
            raw = np.fromfile(lidar_path, dtype=np.float32)
            if raw.size % 4:
                raise ValueError("{} is not XYZI float32".format(lidar_path))
            points = raw.reshape(-1, 4)
            finite = np.isfinite(points).all(axis=1)
            points = points[finite]
            actors = actor_by_scene_id.get(frame["scene_id"], {})
            spatial = spatial_by_scene_id.get(frame["scene_id"], {})
            for box in parse_instances(instance_path):
                distance = float(np.linalg.norm(box["center"][:2]))
                if distance > args.maximum_center_range_m:
                    continue
                count = int(points_in_box(points, box, args.box_tolerance_m).sum())
                if count == 0:
                    disposition = "drop_zero_point"
                elif count < args.minimum_positive_points:
                    disposition = "exclude_sparse_positive"
                else:
                    disposition = "normal_positive"
                actor = actors.get(box["unique_id"], {})
                records.append({
                    "scene_id": frame["scene_id"],
                    "frame_id": frame["frame_id"],
                    "frame_index": int(frame["frame_index"]),
                    "capture_id": capture_id,
                    "mission_family": frame["mission_family"],
                    "class": coarse_class(box["raw_class"]),
                    "raw_class": box["raw_class"],
                    "unique_id": box["unique_id"],
                    "asset_name": actor.get("name") or "unmapped",
                    "asset_subtype": actor.get("subtype") or "unmapped",
                    "geometry_role": actor.get("geometry_role") or "unmapped",
                    "spatial_stratum": spatial.get("spatial_stratum")
                    or "unspecified",
                    "highway_phase_variant": spatial.get(
                        "highway_phase_variant"
                    ) or "unspecified",
                    "center_range_m": distance,
                    "num_lidar_points_in_gt": count,
                    "point_bin": point_bin(count),
                    "training_disposition": disposition,
                })

    def grouped(key):
        values = defaultdict(list)
        for row in records:
            values[row[key]].append(row["num_lidar_points_in_gt"])
        return {
            name: {
                "count": len(counts),
                "zero_count": sum(value == 0 for value in counts),
                "sparse_1_4_count": sum(1 <= value <= 4 for value in counts),
                "normal_positive_count": sum(value >= args.minimum_positive_points for value in counts),
                "p10": percentile(counts, 10),
                "p50": percentile(counts, 50),
                "p90": percentile(counts, 90),
            }
            for name, counts in sorted(values.items())
        }

    counts = [row["num_lidar_points_in_gt"] for row in records]
    report = {
        "schema_version": 1,
        "run_dir": str(args.run_dir.resolve()),
        "maximum_center_range_m": args.maximum_center_range_m,
        "minimum_positive_points": args.minimum_positive_points,
        "box_tolerance_m": args.box_tolerance_m,
        "box_observations": len(records),
        "point_bins": dict(Counter(row["point_bin"] for row in records)),
        "training_dispositions": dict(Counter(row["training_disposition"] for row in records)),
        "all": {
            "zero_rate": sum(value == 0 for value in counts) / len(counts) if counts else None,
            "sparse_1_4_rate": sum(1 <= value <= 4 for value in counts) / len(counts) if counts else None,
            "p10": percentile(counts, 10),
            "p50": percentile(counts, 50),
            "p90": percentile(counts, 90),
        },
        "by_class": grouped("class"),
        "by_asset_name": grouped("asset_name"),
        "by_asset_subtype": grouped("asset_subtype"),
        "by_geometry_role": grouped("geometry_role"),
        "by_spatial_stratum": grouped("spatial_stratum"),
        "by_highway_phase_variant": grouped("highway_phase_variant"),
        "non_normal_examples": [
            row for row in records
            if row["training_disposition"] != "normal_positive"
        ][:100],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    csv_path = args.output.with_suffix(".csv")
    if records:
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
    print("boxes: {}".format(len(records)))
    print("point bins: {}".format(report["point_bins"]))
    print("dispositions: {}".format(report["training_dispositions"]))
    print("report: {}".format(args.output))


if __name__ == "__main__":
    main()
