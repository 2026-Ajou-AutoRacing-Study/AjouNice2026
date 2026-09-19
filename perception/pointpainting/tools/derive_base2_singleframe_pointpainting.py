#!/usr/bin/env python3
"""Derive a 9-feature, 7D-box Base2 dataset from the audited D4 cache.

The D4 temporal materialization keeps every painted source sweep as a 9-feature
NumPy array.  This tool hard-links each temporal target frame from that cache,
drops vx/vy/velocity-weight from its labels and infos, and preserves the exact
train/val split.  No LRASPP inference or sensor reprocessing is performed.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import shutil
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def link_or_copy(source: Path, target: Path, overwrite: bool) -> str:
    if target.exists():
        if not overwrite:
            return "existing"
        target.unlink()
    try:
        os.link(source, target)
        return "hardlink"
    except OSError:
        shutil.copy2(source, target)
        return "copy"


def convert_label(source: Path, target: Path) -> int:
    output = []
    for line in source.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 11:
            raise RuntimeError(f"invalid weighted velocity label: {source}: {line}")
        output.append(" ".join(fields[:7] + [fields[-1]]))
    target.write_text("\n".join(output) + ("\n" if output else ""), encoding="utf-8")
    return len(output)


def main() -> None:
    args = parse_args()
    source_root = args.source_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    cache_root = source_root / "painted_sweeps"
    if not cache_root.is_dir():
        raise RuntimeError(f"missing painted sweep cache: {cache_root}")

    for directory in ("points", "labels", "ImageSets", "metadata", "reports"):
        (output_root / directory).mkdir(parents=True, exist_ok=True)

    link_modes = {"hardlink": 0, "copy": 0, "existing": 0}
    split_counts = {}
    total_boxes = 0
    sample_ids = set()

    for split in ("train", "val", "test"):
        source_info = source_root / f"custom_infos_{split}.pkl"
        source_split = source_root / "ImageSets" / f"{split}.txt"
        if not source_info.is_file():
            if source_split.is_file():
                shutil.copy2(source_split, output_root / "ImageSets" / source_split.name)
            continue

        with source_info.open("rb") as stream:
            infos = pickle.load(stream)
        converted = []
        for info in infos:
            sample_id = str(info["point_cloud"]["lidar_idx"])
            sample_ids.add(sample_id)
            source_points = cache_root / f"{sample_id}.npy"
            if not source_points.is_file():
                raise RuntimeError(f"missing target painted sweep: {source_points}")
            points = np.load(source_points, mmap_mode="r")
            if points.ndim != 2 or points.shape[1] != 9:
                raise RuntimeError(f"invalid target sweep shape: {source_points}: {points.shape}")
            mode = link_or_copy(
                source_points, output_root / "points" / source_points.name, args.overwrite
            )
            link_modes[mode] += 1

            total_boxes += convert_label(
                source_root / "labels" / f"{sample_id}.txt",
                output_root / "labels" / f"{sample_id}.txt",
            )
            annos = info["annos"]
            boxes = np.asarray(annos["gt_boxes_lidar"], dtype=np.float32)
            if boxes.ndim != 2 or boxes.shape[1] < 7:
                raise RuntimeError(f"invalid boxes for {sample_id}: {boxes.shape}")
            converted.append(
                {
                    "point_cloud": {"num_features": 9, "lidar_idx": sample_id},
                    "annos": {
                        "name": np.asarray(annos["name"]),
                        "gt_boxes_lidar": np.ascontiguousarray(boxes[:, :7]),
                    },
                }
            )

        with (output_root / f"custom_infos_{split}.pkl").open("wb") as stream:
            pickle.dump(converted, stream)
        shutil.copy2(source_split, output_root / "ImageSets" / source_split.name)
        split_counts[split] = len(converted)

    report = {
        "schema_version": 1,
        "dataset_id": output_root.name,
        "source_dataset": str(source_root),
        "derivation": "target-sweep-only-hardlink-v1",
        "point_features": [
            "x", "y", "z", "intensity", "background", "pedestrian",
            "vehicle", "obstacle", "unknown",
        ],
        "gt_layout": ["x", "y", "z", "dx", "dy", "dz", "yaw"],
        "samples": len(sample_ids),
        "samples_by_split": split_counts,
        "boxes": total_boxes,
        "point_storage": link_modes,
        "notes": [
            "Exact D4 target frames and train/val split are preserved.",
            "Velocity labels and temporal time-lag features are intentionally removed.",
            "Point files normally share inodes with D4 painted_sweeps.",
        ],
    }
    for path in (
        output_root / "metadata" / "dataset.json",
        output_root / "reports" / "derive_summary.json",
    ):
        path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
