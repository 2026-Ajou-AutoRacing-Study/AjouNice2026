#!/usr/bin/env python3
"""Convert Base3 raw metadata to MMDetection3D v2 BEVFusion info files."""

from __future__ import annotations

import argparse
import json
import pickle
from collections import Counter
from pathlib import Path

import numpy as np


CLASSES = ("vehicle", "pedestrian", "obstacle")
CAMERA_ORDER = ("front", "front_left", "front_right", "back")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimum-gt-points", type=int, default=5)
    parser.add_argument("--box-tolerance-m", type=float, default=0.03)
    parser.add_argument("--max-frames-per-split", type=int, default=0)
    parser.add_argument(
        "--capture-root", type=Path,
        help="actual Base3 capture root used while reading files")
    parser.add_argument(
        "--serialized-capture-root", type=Path,
        help="portable capture root written into generated info PKLs")
    return parser.parse_args()


def read_jsonl(path):
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def points_in_box_count(points, box, tolerance):
    center = np.asarray(box[:3], dtype=np.float32)
    dimensions = np.asarray(box[3:6], dtype=np.float32) + 2.0 * tolerance
    yaw = float(box[6])
    translated = points[:, :3] - center
    cosine, sine = np.cos(yaw), np.sin(yaw)
    local_x = cosine * translated[:, 0] + sine * translated[:, 1]
    local_y = -sine * translated[:, 0] + cosine * translated[:, 1]
    inside = (
        (np.abs(local_x) <= dimensions[0] * 0.5)
        & (np.abs(local_y) <= dimensions[1] * 0.5)
        & (np.abs(translated[:, 2]) <= dimensions[2] * 0.5)
    )
    return int(inside.sum())


def select_smoke_frames(frames, annotations, limit):
    """Choose a small split sample that exercises every label and empty frames."""
    if not limit or len(frames) <= limit:
        return frames
    selected, selected_ids = [], set()
    buckets = list(CLASSES) + ["empty"]
    while len(selected) < limit:
        made_progress = False
        for bucket in buckets:
            for frame in frames:
                frame_id = frame["frame_id"]
                if frame_id in selected_ids:
                    continue
                labels = {
                    box["class"]
                    for box in annotations[frame_id]["boxes"]
                    if box["class"] in CLASSES
                }
                if (bucket == "empty" and not labels) or bucket in labels:
                    selected.append(frame)
                    selected_ids.add(frame_id)
                    made_progress = True
                    break
            if len(selected) >= limit:
                break
        if not made_progress:
            break
    if len(selected) < limit:
        selected.extend(
            frame for frame in frames
            if frame["frame_id"] not in selected_ids
        )
    return selected[:limit]


def remap_capture_path(path, recorded_root, actual_root, serialized_root):
    """Return the read path and path stored in an info file."""
    original = Path(path).expanduser()
    try:
        relative = original.relative_to(recorded_root)
    except ValueError as exc:
        raise ValueError(
            f"capture payload is outside metadata capture_root: {original}") from exc
    return actual_root / relative, serialized_root / relative


def main():
    args = arguments()
    root = args.dataset_root.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.minimum_gt_points < 0 or args.box_tolerance_m < 0:
        raise ValueError("point threshold and tolerance must be non-negative")

    frames = read_jsonl(root / "metadata/frames.jsonl")
    annotations = {
        row["frame_id"]: row for row in read_jsonl(root / "annotations/boxes_3d.jsonl")
    }
    calibration = json.loads(
        (root / "metadata/calibrations/sensor6_v1.json").read_text(encoding="utf-8")
    )
    dataset_metadata = json.loads(
        (root / "metadata/dataset.json").read_text(encoding="utf-8")
    )
    recorded_capture_root = Path(dataset_metadata["capture_root"]).expanduser()
    actual_capture_root = (
        args.capture_root.expanduser().resolve()
        if args.capture_root else recorded_capture_root
    )
    serialized_capture_root = (
        args.serialized_capture_root.expanduser()
        if args.serialized_capture_root else actual_capture_root
    )
    label_index = {name: index for index, name in enumerate(CLASSES)}
    split_frames = {name: [] for name in ("train", "val", "test")}
    for frame in frames:
        split_frames[frame["split"]].append(frame)
    if args.max_frames_per_split:
        split_frames = {
            split: select_smoke_frames(rows, annotations, args.max_frames_per_split)
            for split, rows in split_frames.items()
        }
    frames = [frame for split in ("train", "val", "test") for frame in split_frames[split]]

    split_rows = {name: [] for name in ("train", "val", "test")}
    kept, rejected = Counter(), Counter()

    for frame in frames:
        split = frame["split"]
        lidar_path, serialized_lidar_path = remap_capture_path(
            frame["lidar"]["intensity_bin"], recorded_capture_root,
            actual_capture_root, serialized_capture_root)
        values = np.fromfile(lidar_path, dtype=np.float32)
        if values.size % 4:
            raise ValueError(f"{lidar_path}: point payload is not Nx4 float32")
        points = values.reshape(-1, 4)
        instances = []
        for box in annotations[frame["frame_id"]]["boxes"]:
            class_name = box["class"]
            if class_name not in label_index:
                continue
            count = points_in_box_count(points, box["box_lidar"], args.box_tolerance_m)
            if count < args.minimum_gt_points:
                rejected[class_name] += 1
                continue
            kept[class_name] += 1
            instances.append({
                "bbox_3d": [float(value) for value in box["box_lidar"]],
                "bbox_label_3d": label_index[class_name],
                "num_lidar_pts": count,
                "bbox_3d_isvalid": True,
                "instance_id": int(box["instance_id"]),
            })

        images = {}
        for camera_name in CAMERA_ORDER:
            camera = calibration["cameras"][camera_name]
            lidar2cam = np.asarray(camera["lidar_to_camera_optical"], dtype=np.float64)
            intrinsic = np.asarray(camera["intrinsic"], dtype=np.float64)
            images[camera_name] = {
                "img_path": str(remap_capture_path(
                    frame["cameras"][camera_name]["intensity"],
                    recorded_capture_root, actual_capture_root,
                    serialized_capture_root)[1]),
                "height": int(camera["resolution"][1]),
                "width": int(camera["resolution"][0]),
                "cam2img": intrinsic.tolist(),
                "lidar2cam": lidar2cam.tolist(),
                "lidar2img": (intrinsic @ lidar2cam[:3, :]).tolist(),
                "cam2lidar": np.linalg.inv(lidar2cam).tolist(),
            }

        split_rows[split].append({
            "sample_idx": frame["frame_id"],
            "token": frame["frame_id"],
            "scene_id": frame["scene_id"],
            "timestamp": float(frame.get("triggered_unix") or 0.0),
            "lidar_points": {
                "lidar_path": str(serialized_lidar_path),
                "num_pts_feats": 4,
                "lidar2ego": calibration["lidar_to_ego"],
            },
            "images": images,
            "instances": instances,
        })

    metainfo = {
        "dataset": "morai_base3_static_10k",
        "classes": CLASSES,
        "minimum_gt_points": args.minimum_gt_points,
        "box_tolerance_m": args.box_tolerance_m,
    }
    report = {
        "splits": {},
        "kept_gt": dict(kept),
        "rejected_sparse_gt": dict(rejected),
        "path_mapping": {
            "recorded_capture_root": str(recorded_capture_root),
            "actual_capture_root": str(actual_capture_root),
            "serialized_capture_root": str(serialized_capture_root),
        },
    }
    for split, rows in split_rows.items():
        target = output / f"morai_base3_infos_{split}.pkl"
        with target.open("wb") as handle:
            pickle.dump({"metainfo": metainfo, "data_list": rows}, handle)
        report["splits"][split] = {"frames": len(rows), "path": str(target)}
    (output / "conversion_audit.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
