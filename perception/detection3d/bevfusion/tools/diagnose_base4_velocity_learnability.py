#!/usr/bin/env python3
"""Measure whether a Base4 info set contains learnable velocity evidence.

This is deliberately an annotation-side diagnostic, not a detector metric.  It
separates four questions that are otherwise easy to conflate:

* are velocity labels consistent with geometric BBox motion;
* how many independent motion scenes exist behind the frame count;
* how much of the target can be guessed from the dataset prior;
* how accurate an associated multi-frame centre displacement can be.

The centre-displacement result is an oracle because it uses instance IDs and
GT BBoxes.  It proves observability in the sequence, but not that a detector
can associate or localize the object that accurately.
"""

from __future__ import annotations

import argparse
import json
import pickle
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


SPLITS = ("train", "val", "test")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--info-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--horizons", type=int, nargs="+", default=(1, 2, 3),
        help="frame-index gaps used by the GT-centre displacement oracle",
    )
    return parser.parse_args()


def summary(values):
    values = np.asarray(list(values), dtype=np.float64)
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "p05": float(np.percentile(values, 5.0)),
        "p50": float(np.percentile(values, 50.0)),
        "p95": float(np.percentile(values, 95.0)),
        "maximum": float(np.max(values)),
    }


def load_rows(info_dir, split):
    path = info_dir / f"morai_base3_infos_{split}.pkl"
    with path.open("rb") as stream:
        return pickle.load(stream).get("data_list", [])


def global_state(row, instance):
    box = np.asarray(instance["bbox_3d"], dtype=np.float64)
    transform = np.asarray(row["lidar2global"], dtype=np.float64)
    center = (transform @ np.r_[box[:3], 1.0])[:2]
    velocity = transform[:2, :2] @ box[7:9]
    return center, velocity


def velocity_records(rows):
    records = []
    for row in rows:
        for instance in row.get("instances", []):
            box = np.asarray(instance["bbox_3d"], dtype=np.float64)
            if box.size < 9:
                continue
            records.append({
                "scene": str(row["scene_id"]),
                "speed": float(np.linalg.norm(box[7:9])),
                "distance": float(np.linalg.norm(box[:2])),
                "points": int(instance.get("num_lidar_pts", 0)),
                "asset": str(instance.get("asset_name", "unknown")),
            })
    return records


def scalar_prior_mae(train_records, evaluation_records):
    if not train_records or not evaluation_records:
        return None
    prediction = float(np.median([record["speed"] for record in train_records]))
    errors = [
        abs(record["speed"] - prediction) for record in evaluation_records
    ]
    return {"prediction_mps": prediction, "speed_mae_mps": float(np.mean(errors))}


def per_scene_oracle(records):
    grouped = defaultdict(list)
    for record in records:
        grouped[record["scene"]].append(record["speed"])
    errors = []
    variations = []
    spans = []
    for speeds in grouped.values():
        prediction = float(np.median(speeds))
        errors.extend(abs(speed - prediction) for speed in speeds)
        variations.append(float(np.std(speeds)))
        spans.append(float(max(speeds) - min(speeds)))
    return {
        "speed_mae_mps": float(np.mean(errors)) if errors else None,
        "within_scene_speed_std_mps": summary(variations),
        "within_scene_speed_span_mps": summary(spans),
        "scenes_with_span_over_0_5_mps": int(sum(
            span > 0.5 for span in spans)),
        "scenes_with_span_over_1_0_mps": int(sum(
            span > 1.0 for span in spans)),
    }


def asset_median_prior(train_records, evaluation_records):
    if not train_records or not evaluation_records:
        return None
    grouped = defaultdict(list)
    for record in train_records:
        grouped[record["asset"]].append(record["speed"])
    fallback = float(np.median(
        [record["speed"] for record in train_records]))
    errors = []
    covered = 0
    for record in evaluation_records:
        if record["asset"] in grouped:
            prediction = float(np.median(grouped[record["asset"]]))
            covered += 1
        else:
            prediction = fallback
        errors.append(abs(record["speed"] - prediction))
    return {
        "speed_mae_mps": float(np.mean(errors)),
        "known_asset_fraction": covered / len(evaluation_records),
    }


def centre_displacement_oracle(rows, horizons):
    grouped = defaultdict(list)
    for row in rows:
        grouped[str(row["scene_id"])].append(row)
    output = {}
    for horizon in horizons:
        vector_errors = []
        speed_errors = []
        elapsed = []
        for scene_rows in grouped.values():
            scene_rows.sort(key=lambda row: float(row["timestamp"]))
            for start, end in zip(scene_rows, scene_rows[horizon:]):
                dt = float(end["timestamp"]) - float(start["timestamp"])
                if dt <= 0.0:
                    continue
                start_instances = {
                    int(instance["instance_id"]): instance
                    for instance in start.get("instances", [])
                    if "instance_id" in instance
                }
                end_instances = {
                    int(instance["instance_id"]): instance
                    for instance in end.get("instances", [])
                    if "instance_id" in instance
                }
                for instance_id in start_instances.keys() & end_instances.keys():
                    center0, velocity0 = global_state(
                        start, start_instances[instance_id])
                    center1, velocity1 = global_state(
                        end, end_instances[instance_id])
                    estimate = (center1 - center0) / dt
                    target = 0.5 * (velocity0 + velocity1)
                    vector_errors.append(float(np.linalg.norm(estimate - target)))
                    speed_errors.append(abs(
                        float(np.linalg.norm(estimate) - np.linalg.norm(target))))
                    elapsed.append(dt)
        output[str(horizon)] = {
            "elapsed_sec": summary(elapsed),
            "vector_error_mps": summary(vector_errors),
            "speed_mae_mps": summary(speed_errors),
        }
    return output


def diagnose(info_dir, horizons=(1, 2, 3)):
    rows = {split: load_rows(info_dir, split) for split in SPLITS}
    records = {split: velocity_records(split_rows) for split, split_rows in rows.items()}
    train_scenes = {record["scene"] for record in records["train"]}
    val_scenes = {record["scene"] for record in records["val"]}
    report = {
        "schema_version": 1,
        "interpretation": {
            "centre_displacement": "GT association/localization oracle, not production performance",
            "per_scene_prior": "dataset repetition/conditional-prior lower bound, not deployable",
        },
        "splits": {},
        "scene_overlap": sorted(train_scenes & val_scenes),
        "global_train_median_prior": {},
        "train_asset_median_prior": {},
        "per_scene_oracle": {},
        "gt_centre_displacement_oracle": {},
    }
    for split in SPLITS:
        split_records = records[split]
        report["splits"][split] = {
            "frames": len(rows[split]),
            "scenes": len({record["scene"] for record in split_records}),
            "instances": len(split_records),
            "assets": len({record["asset"] for record in split_records}),
            "top_assets": Counter(record["asset"] for record in split_records).most_common(10),
            "speed_mps": summary(record["speed"] for record in split_records),
            "distance_m": summary(record["distance"] for record in split_records),
            "lidar_points": summary(record["points"] for record in split_records),
        }
        report["global_train_median_prior"][split] = scalar_prior_mae(
            records["train"], split_records)
        report["train_asset_median_prior"][split] = asset_median_prior(
            records["train"], split_records)
        report["per_scene_oracle"][split] = per_scene_oracle(split_records)
        report["gt_centre_displacement_oracle"][split] = (
            centre_displacement_oracle(rows[split], horizons))
    return report


def main():
    args = arguments()
    report = diagnose(args.info_dir, tuple(args.horizons))
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")


if __name__ == "__main__":
    main()
