#!/usr/bin/env python3
"""Audit a materialized Base2 3-sweep PointPainting dataset."""

import argparse
import json
import pickle
from collections import Counter
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--d3-index", type=Path, required=True)
    parser.add_argument("--point-samples", type=int, default=256)
    args = parser.parse_args()
    root = args.data_root.resolve()
    d3 = args.d3_index.resolve()

    valid_by_split = Counter()
    with (d3 / "temporal_samples.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row["temporal_input_valid"]:
                valid_by_split[row["split"]] += 1

    ids_by_split = {}
    infos_by_split = {}
    failures = []
    weights = []
    box_counts = Counter()
    for split in ("train", "val"):
        ids = [line.strip() for line in (root / "ImageSets" / (split + ".txt")).read_text().splitlines() if line.strip()]
        ids_by_split[split] = ids
        with (root / ("custom_infos_" + split + ".pkl")).open("rb") as stream:
            infos = pickle.load(stream)
        infos_by_split[split] = infos
        if len(ids) != len(infos):
            failures.append("{} ImageSets/info mismatch".format(split))
        if len(ids) != valid_by_split[split]:
            failures.append("{} count differs from valid D3 samples".format(split))
        for frame_id, info in zip(ids, infos):
            if info["point_cloud"]["lidar_idx"] != frame_id:
                failures.append("{} info order mismatch".format(frame_id))
            boxes = np.asarray(info["annos"]["gt_boxes_lidar"])
            if boxes.ndim != 2 or boxes.shape[1] != 10:
                failures.append("{} GT shape {}".format(frame_id, boxes.shape))
                continue
            weights.extend(boxes[:, 9].tolist())
            box_counts.update(info["annos"]["name"].tolist())

    all_ids = ids_by_split["train"] + ids_by_split["val"]
    if len(all_ids) != len(set(all_ids)):
        failures.append("train/val sample overlap")
    sample_positions = np.linspace(
        0, max(0, len(all_ids) - 1), min(args.point_samples, len(all_ids)), dtype=int
    )
    point_counts = []
    lag_sets = []
    for position in sample_positions:
        frame_id = all_ids[int(position)]
        points = np.load(root / "points" / (frame_id + ".npy"), mmap_mode="r")
        if points.ndim != 2 or points.shape[1] != 10:
            failures.append("{} point shape {}".format(frame_id, points.shape))
            continue
        if not np.isfinite(points).all():
            failures.append("{} non-finite points".format(frame_id))
        lags = np.unique(np.asarray(points[:, 9]))
        if len(lags) != 3 or abs(float(lags[-1])) > 1.0e-6:
            failures.append("{} lag contract {}".format(frame_id, lags.tolist()))
        point_counts.append(int(len(points)))
        lag_sets.append(lags.tolist())

    weights_array = np.asarray(weights, dtype=np.float64)
    if len(weights_array) and ((weights_array < 0.0) | (weights_array > 1.0)).any():
        failures.append("velocity weight outside [0,1]")
    report = {
        "status": "pass" if not failures else "fail",
        "data_root": str(root),
        "samples_by_split": {key: len(value) for key, value in ids_by_split.items()},
        "d3_valid_samples_by_split": dict(valid_by_split),
        "sample_overlap": len(all_ids) - len(set(all_ids)),
        "boxes_by_class": dict(box_counts),
        "velocity_weight": {
            "count": int(len(weights_array)),
            "zero": int((weights_array == 0.0).sum()),
            "partial": int(((weights_array > 0.0) & (weights_array < 1.0)).sum()),
            "one": int((weights_array == 1.0).sum()),
            "mean": float(weights_array.mean()) if len(weights_array) else None,
        },
        "audited_point_samples": len(point_counts),
        "point_count_p50": float(np.median(point_counts)) if point_counts else None,
        "example_time_lags_sec": lag_sets[:3],
        "failures": failures,
    }
    output = root / "reports/d4_readiness_audit.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
