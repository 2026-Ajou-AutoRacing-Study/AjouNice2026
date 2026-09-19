#!/usr/bin/env python3
"""Recover MORAI ObjectStatus pivot-to-3D-box-center offsets from Base3 captures.

MORAI ObjectStatus.position is an asset reference pivot, not the geometric
centre of the LiDAR annotation box.  This tool pairs the ObjectStatus snapshot
stored in Base3 metadata with MORAI's authoritative ``LIDAR_*/_Instance.txt``
record and estimates an asset-local centre offset.  The resulting catalog is
used when online rosbag ObjectStatus messages are converted to BEVFusion GT.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minimum-observations", type=int, default=2)
    return parser.parse_args()


def read_jsonl(path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def parse_lidar_instances(path):
    records = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        fields = line.split()
        if len(fields) < 15:
            continue
        try:
            values = [float(value) for value in fields[1:14]]
            unique_id = int(fields[14])
        except ValueError:
            continue
        records[unique_id] = {
            "raw_class": fields[0],
            "center_lidar_m": values[0:3],
            "rotation_rpy_rad": values[3:6],
            "size_m": values[6:9],
        }
    return records


def instance_path(frame, capture_root):
    candidates = []
    for key in ("new_files", "morai_source_files"):
        candidates.extend(frame.get(key) or [])
    for value in candidates:
        if value.startswith("LIDAR_") and value.endswith("_Instance.txt"):
            path = capture_root / value
            if path.is_file():
                return path
    return None


def transform_point(transform, xyz):
    vector = np.asarray([xyz[0], xyz[1], xyz[2], 1.0], dtype=np.float64)
    return (np.asarray(transform, dtype=np.float64) @ vector)[:3]


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def main():
    args = arguments()
    root = args.capture_root.expanduser().resolve()
    observations = defaultdict(list)
    skipped = defaultdict(int)

    frame_files = sorted(root.glob("*/dynamic_metadata/*.frames.jsonl"))
    for frames_path in frame_files:
        calibration_path = frames_path.parent / "calibration.json"
        if not calibration_path.is_file():
            skipped["missing_calibration"] += 1
            continue
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        ego_to_lidar = calibration["lidar"]["ego_to_lidar"]
        capture_dir = frames_path.parent.parent
        for frame in read_jsonl(frames_path):
            path = instance_path(frame, capture_dir)
            if path is None:
                skipped["missing_instance_file"] += 1
                continue
            instances = parse_lidar_instances(path)
            actors = (frame.get("objects") or {}).get("actors") or []
            for actor in actors:
                instance = instances.get(int(actor["unique_id"]))
                relative = actor.get("relative_to_ego")
                if instance is None or not relative:
                    skipped["actor_without_lidar_gt"] += 1
                    continue
                position = relative["position_m"]
                pivot_lidar = transform_point(
                    ego_to_lidar,
                    [position["x"], position["y"], position["z"]],
                )
                center = np.asarray(instance["center_lidar_m"], dtype=np.float64)
                delta_lidar = center - pivot_lidar
                yaw = float(instance["rotation_rpy_rad"][2])
                cosine, sine = math.cos(yaw), math.sin(yaw)
                delta_local = np.asarray(
                    [
                        cosine * delta_lidar[0] + sine * delta_lidar[1],
                        -sine * delta_lidar[0] + cosine * delta_lidar[1],
                        delta_lidar[2],
                    ],
                    dtype=np.float64,
                )
                key = (str(actor["class"]), str(actor["name"]))
                observations[key].append(
                    {
                        "offset_local_m": delta_local,
                        "size_m": np.asarray(instance["size_m"], dtype=np.float64),
                        "center_lidar_m": center,
                        "pivot_lidar_m": pivot_lidar,
                        "yaw_rad": yaw,
                        "source": str(path),
                    }
                )

    assets = {}
    residuals_all = []
    for (class_name, asset_name), rows in sorted(observations.items()):
        if len(rows) < args.minimum_observations:
            skipped["insufficient_asset_observations"] += len(rows)
            continue
        offsets = np.stack([row["offset_local_m"] for row in rows])
        sizes = np.stack([row["size_m"] for row in rows])
        median_offset = np.median(offsets, axis=0)
        predicted_centers = []
        residuals = []
        for row in rows:
            yaw = row["yaw_rad"]
            cosine, sine = math.cos(yaw), math.sin(yaw)
            local = median_offset
            delta_lidar = np.asarray(
                [
                    cosine * local[0] - sine * local[1],
                    sine * local[0] + cosine * local[1],
                    local[2],
                ]
            )
            predicted = row["pivot_lidar_m"] + delta_lidar
            predicted_centers.append(predicted)
            residuals.append(float(np.linalg.norm(predicted - row["center_lidar_m"])))
        residuals_all.extend(residuals)
        assets[asset_name] = {
            "class": class_name,
            "pivot_to_bbox_center_local_m": median_offset.tolist(),
            "bbox_size_m": np.median(sizes, axis=0).tolist(),
            "observations": len(rows),
            "offset_mad_m": np.median(np.abs(offsets - median_offset), axis=0).tolist(),
            "center_residual_m": {
                "p50": percentile(residuals, 50),
                "p95": percentile(residuals, 95),
                "max": max(residuals),
            },
        }

    report = {
        "schema_version": 1,
        "source_capture_root": str(root),
        "contract": {
            "object_status_position": "asset pivot in morai_map",
            "offset_axes": "asset local: x forward, y left, z up",
            "application": (
                "bbox_center_lidar = pivot_lidar + "
                "R_z(object_yaw_lidar) @ pivot_to_bbox_center_local_m"
            ),
        },
        "summary": {
            "metadata_files": len(frame_files),
            "assets": len(assets),
            "observations": sum(row["observations"] for row in assets.values()),
            "center_residual_m": {
                "p50": percentile(residuals_all, 50) if residuals_all else None,
                "p95": percentile(residuals_all, 95) if residuals_all else None,
                "max": max(residuals_all) if residuals_all else None,
            },
            "skipped": dict(skipped),
        },
        "assets": assets,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
