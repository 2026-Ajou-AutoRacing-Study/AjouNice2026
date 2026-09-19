#!/usr/bin/env python3
"""Compare raw and azimuth-deskew temporal point-cloud consistency."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path, required=True)
    parser.add_argument("--deskew-root", type=Path, required=True)
    parser.add_argument(
        "--single-sweep-root",
        type=Path,
        help="Optional original dataset used to measure no-alignment geometry.",
    )
    parser.add_argument("--dynamic-source-root", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=200)
    parser.add_argument("--voxel-size", type=float, default=0.25)
    parser.add_argument("--max-points", type=int, default=5000)
    parser.add_argument(
        "--minimum-static-z",
        type=float,
        default=-1.2,
        help="Reject the dense road plane so alignment is judged on structures.",
    )
    return parser.parse_args()


def read_jsonl(path):
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def voxel_sample(points, size, maximum):
    if not len(points):
        return points
    keys = np.floor(points[:, :3] / float(size)).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    indices.sort()
    if len(indices) > maximum:
        select = np.linspace(0, len(indices) - 1, maximum, dtype=np.int64)
        indices = indices[select]
    return points[indices]


def static_points(points, minimum_z):
    # The four-class segmenter's background channel is the safest available
    # static-world mask. Avoid unknown because it can contain unpainted cars.
    mask = (
        (points[:, 4] >= 0.70)
        & (points[:, 0] >= -30.0) & (points[:, 0] <= 30.0)
        & (points[:, 1] >= -30.0) & (points[:, 1] <= 30.0)
        & (points[:, 2] >= float(minimum_z)) & (points[:, 2] <= 2.0)
    )
    return points[mask]


def split_sweeps(points, metadata):
    counts = [int(row["points"]) for row in metadata["sweeps"]]
    boundaries = np.cumsum([0] + counts)
    return [points[boundaries[i] : boundaries[i + 1]] for i in range(len(counts))]


def frame_consistency(points, metadata, voxel_size, max_points, minimum_z):
    sweeps = split_sweeps(points, metadata)
    current = voxel_sample(static_points(sweeps[0], minimum_z), voxel_size, max_points)
    if len(current) < 50:
        return None
    values = []
    for past in sweeps[1:]:
        past = voxel_sample(static_points(past, minimum_z), voxel_size, max_points)
        if len(past) < 50:
            continue
        # Symmetric nearest-neighbour distance avoids one dense sweep winning
        # only because it happens to retain more ground points.
        forward = cKDTree(current[:, :3]).query(past[:, :3], k=1)[0]
        backward = cKDTree(past[:, :3]).query(current[:, :3], k=1)[0]
        values.extend(forward.tolist())
        values.extend(backward.tolist())
    return np.asarray(values, dtype=np.float64) if values else None


def load_unaligned_sweeps(root, metadata):
    values = []
    for sweep in metadata["sweeps"]:
        points = np.load(root / "points" / f"{sweep['frame_id']}.npy", mmap_mode="r")
        values.append(points)
    return np.concatenate(values, axis=0)


def summarize(values):
    values = np.asarray(values, dtype=np.float64)
    return {
        "count": int(len(values)),
        "mean_m": float(values.mean()),
        "p50_m": float(np.percentile(values, 50)),
        "p90_m": float(np.percentile(values, 90)),
        "p95_m": float(np.percentile(values, 95)),
    }


def speed_bin(frame):
    # EgoVehicleStatus speed is the simulator's physical speed. Position
    # finite differences use callback time and shrink when MORAI runs slowly.
    speed = float((frame.get("ego") or {}).get("speed_raw", 0.0))
    if speed < 2.0:
        return "0_2mps"
    if speed < 8.0:
        return "2_8mps"
    return "8mps_plus"


def main():
    args = parse_args()
    raw_root = args.raw_root.resolve()
    deskew_root = args.deskew_root.resolve()
    raw_meta = read_jsonl(raw_root / "metadata/frames.jsonl")
    deskew_meta = {row["frame_id"]: row for row in read_jsonl(deskew_root / "metadata/frames.jsonl")}
    source_meta = {}
    single_sweep_root = args.single_sweep_root.resolve() if args.single_sweep_root else None
    if args.dynamic_source_root:
        source_meta = {
            row["frame_id"]: row
            for row in read_jsonl(args.dynamic_source_root.resolve() / "metadata/frames.jsonl")
        }
    raw_values, deskew_values, unaligned_values = [], [], []
    frame_deltas = []
    grouped = {}
    used = 0
    for row in raw_meta:
        if used >= args.max_frames:
            break
        frame_id = row["frame_id"]
        if frame_id not in deskew_meta:
            continue
        raw = np.load(raw_root / "points" / f"{frame_id}.npy", mmap_mode="r")
        deskew = np.load(deskew_root / "points" / f"{frame_id}.npy", mmap_mode="r")
        raw_distance = frame_consistency(
            raw, row, args.voxel_size, args.max_points, args.minimum_static_z
        )
        deskew_distance = frame_consistency(
            deskew, deskew_meta[frame_id], args.voxel_size, args.max_points,
            args.minimum_static_z,
        )
        unaligned_distance = None
        if single_sweep_root is not None:
            unaligned = load_unaligned_sweeps(single_sweep_root, row)
            unaligned_distance = frame_consistency(
                unaligned, row, args.voxel_size, args.max_points,
                args.minimum_static_z,
            )
        if raw_distance is None or deskew_distance is None:
            continue
        raw_values.extend(raw_distance.tolist())
        deskew_values.extend(deskew_distance.tolist())
        if unaligned_distance is not None:
            unaligned_values.extend(unaligned_distance.tolist())
        delta = float(np.median(deskew_distance) - np.median(raw_distance))
        frame_deltas.append(delta)
        if frame_id in source_meta:
            key = speed_bin(source_meta[frame_id])
            group = grouped.setdefault(key, {"raw": [], "deskew": [], "deltas": []})
            group["raw"].extend(raw_distance.tolist())
            group["deskew"].extend(deskew_distance.tolist())
            group["deltas"].append(delta)
        used += 1

    if not used:
        raise RuntimeError("no comparable frames")
    report = {
        "schema_version": 1,
        "frames": used,
        "metric": "symmetric nearest-neighbour distance on semantic-background points",
        "minimum_static_z_m": args.minimum_static_z,
        "raw": summarize(raw_values),
        "azimuth_ego_deskew": summarize(deskew_values),
        "frame_p50_improved_fraction": float(np.mean(np.asarray(frame_deltas) < 0.0)),
        "frame_p50_delta_m": {
            "mean": float(np.mean(frame_deltas)),
            "p50": float(np.percentile(frame_deltas, 50)),
            "p90": float(np.percentile(frame_deltas, 90)),
        },
        "speed_bins": {
            key: {
                "frames": len(group["deltas"]),
                "raw": summarize(group["raw"]),
                "azimuth_ego_deskew": summarize(group["deskew"]),
                "frame_p50_improved_fraction": float(
                    np.mean(np.asarray(group["deltas"]) < 0.0)
                ),
                "frame_p50_delta_m": float(np.median(group["deltas"])),
            }
            for key, group in sorted(grouped.items())
        },
    }
    if unaligned_values:
        report["unaligned"] = summarize(unaligned_values)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
