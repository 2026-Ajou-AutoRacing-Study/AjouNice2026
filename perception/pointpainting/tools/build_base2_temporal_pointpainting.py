#!/usr/bin/env python3
"""Materialize Base2 D3 samples for 3-sweep PointPainting training.

Each source frame is painted once and cached.  Past painted sweeps are then
aligned into the target LiDAR frame using the audited D3 transform and receive
one constant physical time lag per sweep.  The target label layout is:

    x y z dx dy dz yaw vx vy velocity_loss_weight class

The last numeric field is consumed only by the velocity-weighted anchor head;
it must never scale or mask the seven geometry regression channels.
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import torch


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_sunny13_1500_pointpainting_9feat import (  # noqa: E402
    build_lraspp_4class,
    infer_four_cameras,
    load_lidar_intensity,
    paint_points,
)


CAMERA_MAP = {
    "front": ("CAMERA_1", "Camera_front"),
    "front_left": ("CAMERA_2", "Camera_front_left"),
    "front_right": ("CAMERA_3", "Camera_front_right"),
    "back": ("CAMERA_4", "Camera_back"),
}
def coarse_class(raw_class: str) -> str:
    lowered = raw_class.lower()
    if "pedestrian" in lowered or "person" in lowered:
        return "Pedestrian"
    vehicle_labels = {"vehicle", "sedan", "suv", "truck", "bus", "mpv", "wagon"}
    if lowered in vehicle_labels or "car" in lowered:
        return "Vehicle"
    return "Obstacle"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--d3-index", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--lraspp-checkpoint", type=Path, required=True)
    parser.add_argument("--splits", nargs="+", default=["train", "val"])
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument(
        "--require-moving-target",
        action="store_true",
        help="Pilot mode: retain only samples with a valid non-zero velocity label.",
    )
    parser.add_argument("--short-side", type=int, default=384)
    parser.add_argument(
        "--preprocess-backend", choices=("torchvision", "opencv"), default="opencv"
    )
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--min-lidar-range", type=float, default=0.5)
    parser.add_argument("--min-camera-depth", type=float, default=0.05)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def read_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def read_jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def calibration_for_painter(raw):
    lidar_to_ego = np.asarray(raw["lidar"]["lidar_to_ego"], dtype=np.float64)
    cameras = {}
    for output_name, (_directory, source_name) in CAMERA_MAP.items():
        camera = raw["cameras"][source_name]
        cameras[output_name] = {
            "resolution": list(camera["resolution_px"]),
            "intrinsic": camera["intrinsic"],
            "lidar_to_camera_optical": (
                np.asarray(camera["ego_to_sensor"], dtype=np.float64) @ lidar_to_ego
            ).tolist(),
        }
    return {"cameras": cameras}


def parse_instances(path: Path):
    result = {}
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 15:
                continue
            class_name = coarse_class(fields[0])
            try:
                result[int(fields[14])] = {
                    "class": class_name,
                    "box": [float(value) for value in fields[1:4]]
                    + [float(value) for value in fields[7:10]]
                    + [float(fields[6])],
                }
            except ValueError:
                continue
    return result


def transform_xyz(points: np.ndarray, matrix) -> np.ndarray:
    output = points.copy()
    homogeneous = np.concatenate(
        [points[:, :3].astype(np.float64), np.ones((len(points), 1))], axis=1
    )
    output[:, :3] = (homogeneous @ np.asarray(matrix, dtype=np.float64).T)[:, :3]
    return output


def label_lines(sample, velocity_row, dataset_root: Path):
    instances = parse_instances(
        dataset_root / sample["sweeps_oldest_to_current"][-1]["artifacts"]["lidar"]["instances"]
    )
    velocity_by_id = {int(row["unique_id"]): row for row in velocity_row["labels"]}
    lines = []
    boxes = []
    names = []
    counters = Counter()
    for unique_id, instance in sorted(instances.items()):
        velocity = velocity_by_id.get(unique_id)
        if velocity is None:
            # Preserve the geometry target while masking an unavailable speed.
            vx = vy = weight = 0.0
            counters["missing_velocity"] += 1
        else:
            target = velocity.get("velocity_target_ego_mps") or {}
            vx = float(target.get("x", 0.0)) if velocity.get("velocity_valid") else 0.0
            vy = float(target.get("y", 0.0)) if velocity.get("velocity_valid") else 0.0
            weight = float(velocity.get("velocity_loss_weight", 0.0))
            counters[str(velocity.get("velocity_source", "unknown"))] += 1
        numeric = instance["box"] + [vx, vy, weight]
        lines.append(" ".join("{:.7f}".format(value) for value in numeric) + " " + instance["class"])
        boxes.append(numeric)
        names.append(instance["class"])
    return lines, np.asarray(boxes, dtype=np.float32).reshape(-1, 10), np.asarray(names), counters


def main() -> None:
    args = parse_args()
    dataset_root = args.dataset_root.expanduser().resolve()
    d3_index = args.d3_index.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    selected_splits = set(args.splits)

    samples = [
        row for row in read_jsonl(d3_index / "temporal_samples.jsonl")
        if row["split"] in selected_splits and row["temporal_input_valid"]
    ]
    velocity_rows = {
        row["sample_id"]: row for row in read_jsonl(d3_index / "velocity_labels.jsonl")
    }
    if args.require_moving_target:
        selected = []
        for sample in samples:
            instances = parse_instances(
                dataset_root
                / sample["sweeps_oldest_to_current"][-1]["artifacts"]["lidar"]["instances"]
            )
            moving_ids = {
                int(label["unique_id"])
                for label in velocity_rows[sample["sample_id"]]["labels"]
                if label.get("velocity_valid")
                and np.hypot(
                    float((label.get("velocity_target_ego_mps") or {}).get("x", 0.0)),
                    float((label.get("velocity_target_ego_mps") or {}).get("y", 0.0)),
                ) > 0.1
            }
            if any(
                unique_id in moving_ids
                and abs(instance["box"][0]) <= 29.0
                and abs(instance["box"][1]) <= 29.0
                for unique_id, instance in instances.items()
            ):
                selected.append(sample)
        samples = selected
    if args.max_samples:
        samples = samples[: args.max_samples]
    if not samples:
        raise RuntimeError("no D3 samples selected")
    raw_calibration = read_json(dataset_root / "metadata/calibration.json")
    calibration = calibration_for_painter(raw_calibration)
    model = build_lraspp_4class(args.lraspp_checkpoint, torch.device(args.device))

    cache_dir = output_root / "painted_sweeps"
    points_dir = output_root / "points"
    labels_dir = output_root / "labels"
    for path in (cache_dir, points_dir, labels_dir, output_root / "ImageSets"):
        path.mkdir(parents=True, exist_ok=True)

    split_ids = defaultdict(list)
    infos = defaultdict(list)
    counters = Counter()
    elapsed = []

    def painted_sweep(sweep):
        frame_id = sweep["frame_id"]
        cache_path = cache_dir / (frame_id + ".npy")
        if cache_path.is_file() and not args.overwrite:
            return np.load(cache_path)
        lidar_path = dataset_root / sweep["artifacts"]["lidar"]["intensity"]
        points, _raw_count, _valid_count = load_lidar_intensity(
            lidar_path, args.min_lidar_range
        )
        camera_paths = {
            name: dataset_root / sweep["artifacts"]["cameras"][directory]["intensity"]
            for name, (directory, _source_name) in CAMERA_MAP.items()
        }
        probabilities = infer_four_cameras(
            model=model,
            camera_paths=camera_paths,
            short_side=args.short_side,
            device=torch.device(args.device),
            backend=args.preprocess_backend,
        )
        painted, _seen, _counts = paint_points(
            points, probabilities, calibration, args.min_camera_depth
        )
        np.save(cache_path, np.ascontiguousarray(painted, dtype=np.float32))
        return painted

    for index, sample in enumerate(samples, start=1):
        started = time.perf_counter()
        sample_id = sample["sample_id"]
        output_path = points_dir / (sample_id + ".npy")
        label_path = labels_dir / (sample_id + ".txt")
        temporal = []
        for sweep in sample["sweeps_oldest_to_current"]:
            points = painted_sweep(sweep)
            aligned = transform_xyz(points, sweep["lidar_to_target_lidar"])
            lag = np.full((len(aligned), 1), float(sweep["time_lag_sec"]), dtype=np.float32)
            temporal.append(np.concatenate([aligned, lag], axis=1))
        merged = np.ascontiguousarray(np.concatenate(temporal, axis=0), dtype=np.float32)
        if merged.shape[1] != 10 or not np.isfinite(merged).all():
            raise RuntimeError("{}: invalid temporal points {}".format(sample_id, merged.shape))
        np.save(output_path, merged)

        lines, boxes, names, label_counts = label_lines(
            sample, velocity_rows[sample_id], dataset_root
        )
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        counters.update(label_counts)
        split = sample["split"]
        split_ids[split].append(sample_id)
        infos[split].append({
            "point_cloud": {"num_features": 10, "lidar_idx": sample_id},
            "annos": {"name": names, "gt_boxes_lidar": boxes},
        })
        elapsed.append(time.perf_counter() - started)
        if index <= 3 or index % 100 == 0 or index == len(samples):
            print("built {}/{} {} points={} boxes={}".format(
                index, len(samples), sample_id, len(merged), len(boxes)
            ))

    for split in ("train", "val", "test"):
        (output_root / "ImageSets" / (split + ".txt")).write_text(
            "".join(frame_id + "\n" for frame_id in split_ids[split]), encoding="utf-8"
        )
        if infos[split]:
            with (output_root / ("custom_infos_" + split + ".pkl")).open("wb") as stream:
                pickle.dump(infos[split], stream)

    report = {
        "schema_version": 1,
        "dataset_id": output_root.name,
        "source_dataset": str(dataset_root),
        "d3_index": str(d3_index),
        "samples": len(samples),
        "samples_by_split": {key: len(value) for key, value in split_ids.items()},
        "painted_unique_sweeps": len(list(cache_dir.glob("*.npy"))),
        "point_features": [
            "x", "y", "z", "intensity", "background", "pedestrian",
            "vehicle", "obstacle", "unknown", "time_lag_sec",
        ],
        "gt_layout": [
            "x", "y", "z", "dx", "dy", "dz", "yaw", "vx", "vy",
            "velocity_loss_weight",
        ],
        "velocity_sources": dict(counters),
        "mean_build_sec": float(np.mean(elapsed)),
        "scan_internal_deskew": False,
        "augmentation_contract": "all sweeps are fused before one shared augmentation",
    }
    write_json(output_root / "metadata/dataset.json", report)
    write_json(output_root / "reports/build_summary.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
