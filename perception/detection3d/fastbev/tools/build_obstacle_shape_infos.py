#!/usr/bin/env python3
"""Derive five-class MORAI infos without copying images or changing boxes."""

import argparse
import copy
import hashlib
import json
import pickle
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.detection3d.common.obstacle_shape import (
    SHAPE_CLASSES,
    classify_obstacle_shape,
)


DEFAULT_SOURCE = Path(
    "/home/luv/ajounice_data/morai_od/processed/"
    "fastbev_sunny13_1500_v1/fastbev_infos"
)
DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[4]
    / "artifacts/detection3d/fastbev/data/"
    "fastbev_sunny13_1500_shape5_v1/fastbev_infos"
)
ALL_CLASSES = ("vehicle", "pedestrian") + SHAPE_CLASSES


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_info(info):
    count = len(info["gt_names"])
    for key in ("gt_boxes", "gt_velocity", "num_lidar_pts", "valid_flag"):
        if len(info[key]) != count:
            raise AssertionError(
                f"{info.get('token', '<unknown>')}: {key} has "
                f"{len(info[key])} values for {count} names")
    unknown = set(map(str, info["gt_names"])) - set(ALL_CLASSES)
    if unknown:
        raise AssertionError(f"Unknown derived labels: {sorted(unknown)}")


def transform_payload(payload, ratio_threshold):
    derived = copy.deepcopy(payload)
    counts = Counter()
    sizes = {name: [] for name in SHAPE_CLASSES}
    for info in derived["infos"]:
        names = np.asarray(info["gt_names"]).astype(object, copy=True)
        boxes = np.asarray(info["gt_boxes"])
        if len(names) != len(boxes):
            raise AssertionError("gt_names and gt_boxes are not aligned")
        for index in np.flatnonzero(names == "obstacle"):
            shape_name = classify_obstacle_shape(
                boxes[index, 3:6], ratio_threshold=ratio_threshold)
            names[index] = shape_name
            counts[shape_name] += 1
            sizes[shape_name].append(boxes[index, 3:6].tolist())
        info["gt_names"] = names.astype("<U32")
        validate_info(info)

    metadata = dict(derived.get("metadata", {}))
    metadata.update({
        "version": "fastbev_sunny13_1500_shape5_v1",
        "classes": list(ALL_CLASSES),
        "source_version": payload.get("metadata", {}).get("version"),
        "obstacle_shape_policy": {
            "ratio_threshold": ratio_threshold,
            "precedence": ["tall", "long", "compact"],
        },
    })
    derived["metadata"] = metadata
    return derived, counts, sizes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--ratio-threshold", type=float, default=2.0)
    args = parser.parse_args()
    if args.ratio_threshold <= 1.0:
        raise ValueError("--ratio-threshold must be greater than 1")

    source_dir = args.source_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "schema": "morai_obstacle_shape_infos_v1",
        "source_dir": str(source_dir),
        "output_dir": str(output_dir),
        "ratio_threshold": args.ratio_threshold,
        "classes": list(ALL_CLASSES),
        "splits": {},
    }

    total_counts = Counter()
    total_sizes = {name: [] for name in SHAPE_CLASSES}
    for split in ("train", "val", "test"):
        source = source_dir / f"morai_infos_{split}.pkl"
        target = output_dir / source.name
        with source.open("rb") as stream:
            payload = pickle.load(stream)
        if not isinstance(payload, dict) or "infos" not in payload:
            raise TypeError(f"Unsupported info payload: {source}")
        derived, counts, sizes = transform_payload(
            payload, args.ratio_threshold)
        with target.open("wb") as stream:
            pickle.dump(derived, stream, protocol=pickle.HIGHEST_PROTOCOL)
        total_counts.update(counts)
        for name in SHAPE_CLASSES:
            total_sizes[name].extend(sizes[name])
        report["splits"][split] = {
            "frames": len(derived["infos"]),
            "counts": dict(sorted(counts.items())),
            "source_sha256": sha256(source),
            "output_sha256": sha256(target),
        }

    report["total_counts"] = dict(sorted(total_counts.items()))
    report["representative_median_lwh"] = {
        name: np.median(np.asarray(values), axis=0).round(6).tolist()
        for name, values in total_sizes.items()
        if values
    }
    report_path = output_dir.parent / "shape_derivation_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    print("derived infos:", output_dir)
    print("report:", report_path)


if __name__ == "__main__":
    main()
