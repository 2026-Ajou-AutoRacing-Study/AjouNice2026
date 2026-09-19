#!/usr/bin/env python3
"""Build 3-sweep PointPainting data with optional azimuth Ego deskew."""

import argparse
import json
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from temporal_geometry import (  # noqa: E402
    annotate_simulation_times,
    frame_timestamp,
    temporalize_sweep,
)


SPLITS = ("train", "val", "test")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument("--dynamic-source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--sensor-config", type=Path, default=Path("configs/sensors/Sensor_6.json"))
    parser.add_argument("--num-sweeps", type=int, default=3)
    parser.add_argument("--maximum-adjacent-gap-sec", type=float, default=2.0)
    parser.add_argument("--deskew-mode", choices=("off", "azimuth_ego"), default="off")
    return parser.parse_args()


def read_jsonl(path):
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n")


def load_lidar_config(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    values = data["Lidar3DList"]
    if len(values) != 1:
        raise RuntimeError("exactly one Lidar3D sensor is required")
    sensor = values[0]
    position = tuple(float(sensor["pos"][axis]) for axis in ("x", "y", "z"))
    yaw_rad = np.deg2rad(float(sensor["rot"]["yaw"]))
    period = float(sensor["lc"]["sensorPeriod"])
    return position, float(yaw_rad), period


def load_split_ids(root):
    return {
        split: [
            line.strip()
            for line in (root / "ImageSets" / f"{split}.txt").read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        for split in SPLITS
    }


def eligible_windows(frames, split_by_id, num_sweeps, maximum_gap):
    grouped = defaultdict(list)
    for frame in frames:
        grouped[frame["scene_id"]].append(frame)
    windows = []
    for scene_frames in grouped.values():
        ordered = sorted(scene_frames, key=lambda row: int(row["frame_index"]))
        for end in range(num_sweeps - 1, len(ordered)):
            chronological = ordered[end - num_sweeps + 1 : end + 1]
            if any(
                int(b["frame_index"]) != int(a["frame_index"]) + 1
                for a, b in zip(chronological, chronological[1:])
            ):
                continue
            if any(
                frame_timestamp(b) - frame_timestamp(a) > maximum_gap
                or frame_timestamp(b) <= frame_timestamp(a)
                for a, b in zip(chronological, chronological[1:])
            ):
                continue
            target = chronological[-1]
            split = split_by_id.get(target["frame_id"])
            if split is None or any(split_by_id.get(row["frame_id"]) != split for row in chronological):
                continue
            # Current sweep first, followed by increasingly old sweeps.
            windows.append((split, list(reversed(chronological))))
    return windows


def main():
    args = parse_args()
    if args.num_sweeps < 2:
        raise ValueError("num-sweeps must be at least 2")
    input_root = args.input_root.resolve()
    dynamic_root = args.dynamic_source_root.resolve()
    output_root = args.output_root.resolve()
    lidar_position, lidar_yaw, scan_period = load_lidar_config(args.sensor_config.resolve())

    split_ids = load_split_ids(input_root)
    split_by_id = {
        frame_id: split for split, frame_ids in split_ids.items() for frame_id in frame_ids
    }
    frames = read_jsonl(dynamic_root / "metadata/frames.jsonl")
    timing_diagnostics = annotate_simulation_times(frames)
    windows = eligible_windows(
        frames,
        split_by_id,
        args.num_sweeps,
        args.maximum_adjacent_gap_sec,
    )
    if not windows:
        raise RuntimeError("no eligible temporal windows")
    eligible_scene_ids = {sweeps[0]["scene_id"] for _split, sweeps in windows}
    eligible_timing = [
        row for row in timing_diagnostics if row["scene_id"] in eligible_scene_ids
    ]

    output_ids = defaultdict(list)
    metadata_rows = []
    point_counts = []
    deskew_displacements = []
    role_counts = Counter()
    for index, (split, sweeps) in enumerate(windows, start=1):
        target = sweeps[0]
        frame_id = target["frame_id"]
        temporal_points = []
        sweep_rows = []
        for sweep_index, source in enumerate(sweeps):
            source_path = input_root / "points" / f"{source['frame_id']}.npy"
            points = np.load(source_path)
            raw = temporalize_sweep(
                points,
                source,
                target,
                scan_period,
                lidar_position=lidar_position,
                lidar_yaw_rad=lidar_yaw,
                deskew_ego_motion=False,
            )
            if args.deskew_mode == "azimuth_ego":
                transformed = temporalize_sweep(
                    points,
                    source,
                    target,
                    scan_period,
                    lidar_position=lidar_position,
                    lidar_yaw_rad=lidar_yaw,
                    deskew_ego_motion=True,
                )
                deskew_displacements.extend(
                    np.linalg.norm(transformed[:, :3] - raw[:, :3], axis=1).tolist()
                )
            else:
                transformed = raw
            temporal_points.append(transformed)
            sweep_rows.append(
                {
                    "frame_id": source["frame_id"],
                    "frame_index": int(source["frame_index"]),
                    "reference_time_lag_sec": frame_timestamp(source) - frame_timestamp(target),
                    "points": int(len(points)),
                }
            )

        merged = np.ascontiguousarray(np.concatenate(temporal_points, axis=0), dtype=np.float32)
        point_path = output_root / "points" / f"{frame_id}.npy"
        point_path.parent.mkdir(parents=True, exist_ok=True)
        np.save(point_path, merged)
        label_source = input_root / "labels" / f"{frame_id}.txt"
        label_target = output_root / "labels" / f"{frame_id}.txt"
        label_target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(label_source, label_target)

        output_ids[split].append(frame_id)
        role_counts[str(target.get("role", "unknown"))] += 1
        point_counts.append(len(merged))
        metadata_rows.append(
            {
                "frame_id": frame_id,
                "scene_id": target["scene_id"],
                "split": split,
                "role": target.get("role"),
                "deskew_mode": args.deskew_mode,
                "scan_period_sec": scan_period,
                "sweeps": sweep_rows,
            }
        )
        if index % 100 == 0 or index == len(windows):
            print(f"built {index}/{len(windows)}")

    for split in SPLITS:
        path = output_root / "ImageSets" / f"{split}.txt"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("".join(f"{value}\n" for value in output_ids[split]), encoding="utf-8")
    write_jsonl(output_root / "metadata/frames.jsonl", metadata_rows)

    point_counts_array = np.asarray(point_counts, dtype=np.float64)
    displacement_array = np.asarray(deskew_displacements, dtype=np.float64)
    report = {
        "schema_version": 1,
        "dataset_id": output_root.name,
        "source": str(input_root),
        "dynamic_source": str(dynamic_root),
        "frames": len(metadata_rows),
        "split_frames": {split: len(output_ids[split]) for split in SPLITS},
        "role_frames": dict(sorted(role_counts.items())),
        "num_sweeps": args.num_sweeps,
        "maximum_adjacent_gap_sec": args.maximum_adjacent_gap_sec,
        "deskew_mode": args.deskew_mode,
        "scan_period_sec": scan_period,
        "temporal_time_axis": "position_over_physical_speed_simulation_time",
        "timing_diagnostics": {
            "scenes": len(eligible_timing),
            "fallback_intervals": int(sum(
                row["fallback_intervals"] for row in eligible_timing
            )),
            "median_scene_wall_to_sim_ratio": float(np.median([
                row["wall_to_sim_ratio"] for row in eligible_timing
            ])),
        },
        "lidar_position": list(lidar_position),
        "lidar_yaw_rad": lidar_yaw,
        "point_features": [
            "x", "y", "z", "intensity", "background", "pedestrian",
            "vehicle", "obstacle", "unknown", "time_lag_sec",
        ],
        "points_per_frame": {
            "p50": float(np.percentile(point_counts_array, 50)),
            "p95": float(np.percentile(point_counts_array, 95)),
            "max": int(point_counts_array.max()),
        },
        "deskew_displacement_m": None if not len(displacement_array) else {
            "mean": float(displacement_array.mean()),
            "p50": float(np.percentile(displacement_array, 50)),
            "p95": float(np.percentile(displacement_array, 95)),
            "max": float(displacement_array.max()),
        },
    }
    write_json(output_root / "metadata/dataset.json", report)
    write_json(output_root / "reports/build_summary.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
