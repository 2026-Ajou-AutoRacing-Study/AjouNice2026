#!/usr/bin/env python3
"""Audit Base4 causal-sweep infos beyond the structural validator.

The audit follows the current AjouNice Base4 contract:

* raw XYZI sweeps stay separate and are referenced from each current sample;
* past sweeps use a positive age and a past-LiDAR-to-current-LiDAR transform;
* velocity targets are geometric-BBox-centre velocity in m/s;
* train/val/test isolation is enforced at scene level.

It intentionally does not introduce the alternate pre-merged Nx5 dataset
format.  The checks were selected from the independent three-sweep work on
``perception/base4-temporal-3sweep`` and adapted to this repository's current
schema and velocity convention.
"""

from __future__ import annotations

import argparse
import json
import pickle
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


SPLITS = ("train", "val", "test")
CLASSES = ("vehicle", "pedestrian", "obstacle")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--info-dir", type=Path, required=True)
    parser.add_argument(
        "--velocity-warn-mps", type=float, default=2.0,
        help="finite-difference centre-velocity disagreement warning",
    )
    parser.add_argument(
        "--velocity-reject-mps", type=float, default=5.0,
        help="finite-difference centre-velocity disagreement rejection",
    )
    parser.add_argument(
        "--maximum-pair-gap-sec", type=float, default=0.35,
        help="only compare adjacent object centres up to this time gap",
    )
    parser.add_argument(
        "--maximum-expected-vehicle-speed-mps", type=float, default=35.0,
        help="unit-sanity ceiling; 35 m/s is 126 km/h",
    )
    parser.add_argument(
        "--maximum-state-age-sec", type=float, default=0.10,
        help="maximum nearest Ego/Object sample age in conversion audit",
    )
    parser.add_argument(
        "--maximum-camera-age-sec", type=float, default=0.10,
        help="maximum nearest camera sample age in conversion audit",
    )
    parser.add_argument(
        "--allow-quality-errors", action="store_true",
        help="write the report but return success when quality gates fail",
    )
    return parser.parse_args()


def summary(values):
    values = np.asarray(values, dtype=np.float64)
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


def load_infos(info_dir):
    payloads = {}
    for split in SPLITS:
        path = info_dir / "morai_base3_infos_{}.pkl".format(split)
        if not path.is_file():
            raise FileNotFoundError(path)
        with path.open("rb") as stream:
            payloads[split] = pickle.load(stream)
    return payloads


def transform_quality(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        return None
    rotation = matrix[:3, :3]
    return {
        "orthogonality_error": float(np.linalg.norm(
            rotation.T @ rotation - np.eye(3), ord="fro"
        )),
        "determinant_error": abs(float(np.linalg.det(rotation)) - 1.0),
        "bottom_row_error": float(np.max(np.abs(
            matrix[3] - np.asarray([0.0, 0.0, 0.0, 1.0])
        ))),
    }


def global_center_and_velocity(row, instance):
    box = np.asarray(instance["bbox_3d"], dtype=np.float64)
    lidar2global = np.asarray(row["lidar2global"], dtype=np.float64)
    center = (lidar2global @ np.r_[box[:3], 1.0])[:3]
    velocity = None
    if box.size >= 9:
        velocity = lidar2global[:3, :3] @ np.asarray(
            [box[7], box[8], 0.0], dtype=np.float64
        )
    return center, velocity


def audit(info_dir, velocity_warn_mps, velocity_reject_mps,
          maximum_pair_gap_sec, maximum_expected_vehicle_speed_mps,
          maximum_state_age_sec=0.10, maximum_camera_age_sec=0.10):
    payloads = load_infos(info_dir)
    errors = []
    warnings = []
    split_counts = {}
    scene_splits = defaultdict(set)
    scene_rows = defaultdict(list)
    sweep_ages = []
    sweep_gaps = []
    transform_orthogonality = []
    transform_determinant = []
    transform_bottom_row = []
    velocity_speeds = defaultdict(list)
    velocity_errors = []
    velocity_warn_count = 0
    velocity_reject_count = 0
    excessive_vehicle_speed_count = 0
    excessive_vehicle_speed_examples = []
    velocity_sources = Counter()
    sample_ids = set()

    metainfo_reference = None
    for split, payload in payloads.items():
        metainfo = payload.get("metainfo", {})
        if metainfo_reference is None:
            metainfo_reference = metainfo
        elif tuple(metainfo.get("classes", ())) != tuple(
                metainfo_reference.get("classes", ())):
            errors.append("class metainfo differs across splits")
        rows = payload.get("data_list", [])
        split_counts[split] = len(rows)
        for row in rows:
            sample_ids.add(str(row["sample_idx"]))
            scene_id = str(row.get("scene_id", ""))
            if not scene_id:
                errors.append("{} has no scene_id".format(row.get("sample_idx")))
                continue
            scene_splits[scene_id].add(split)
            scene_rows[(split, scene_id)].append(row)

            target_time = float(row["timestamp"])
            sweeps = row.get("lidar_sweeps", [])
            previous_age = 0.0
            for sweep in sweeps:
                age = float(sweep["time_lag_sec"])
                sweep_ages.append(age)
                if age <= 0.0 or age < previous_age:
                    errors.append(
                        "{} has non-positive or unsorted sweep age".format(
                            row["sample_idx"]
                        )
                    )
                previous_age = age
                timestamp_age = target_time - float(sweep["timestamp"])
                if not np.isclose(timestamp_age, age, atol=1.0e-6):
                    errors.append(
                        "{} sweep timestamp/age mismatch".format(
                            row["sample_idx"]
                        )
                    )
                quality = transform_quality(
                    sweep["lidar_points"]["sweep2lidar"]
                )
                if quality is None:
                    errors.append(
                        "{} has invalid sweep transform".format(
                            row["sample_idx"]
                        )
                    )
                else:
                    transform_orthogonality.append(
                        quality["orthogonality_error"]
                    )
                    transform_determinant.append(
                        quality["determinant_error"]
                    )
                    transform_bottom_row.append(
                        quality["bottom_row_error"]
                    )

            for instance in row.get("instances", []):
                label = int(instance["bbox_label_3d"])
                box = np.asarray(instance["bbox_3d"], dtype=np.float64)
                if box.size < 9:
                    continue
                speed = float(np.linalg.norm(box[7:9]))
                class_name = CLASSES[label]
                velocity_speeds[class_name].append(speed)
                velocity_sources[str(instance.get(
                    "velocity_source", "unspecified"
                ))] += 1
                if (
                    class_name == "vehicle"
                    and speed > maximum_expected_vehicle_speed_mps
                ):
                    excessive_vehicle_speed_count += 1
                    if len(excessive_vehicle_speed_examples) < 10:
                        excessive_vehicle_speed_examples.append({
                            "sample_idx": row["sample_idx"],
                            "speed_mps": speed,
                        })

    leaking_scenes = {
        scene: sorted(splits)
        for scene, splits in scene_splits.items()
        if len(splits) > 1
    }
    if leaking_scenes:
        errors.append(
            "{} scene(s) leak across splits".format(len(leaking_scenes))
        )
    if excessive_vehicle_speed_count:
        errors.append(
            "{} vehicle target(s) exceed the unit-sanity ceiling {:.3f} m/s"
            .format(
                excessive_vehicle_speed_count,
                maximum_expected_vehicle_speed_mps,
            )
        )

    # Compare adjacent geometric BBox centres against the mean endpoint
    # velocity.  This independently catches km/h-as-m/s labels and frame/sign
    # mistakes without trusting ObjectStatus.velocity a second time.
    for (_split, _scene), rows in scene_rows.items():
        rows.sort(key=lambda row: float(row["timestamp"]))
        for previous, current in zip(rows, rows[1:]):
            dt = float(current["timestamp"]) - float(previous["timestamp"])
            if dt <= 0.0:
                errors.append("non-increasing sample timestamp in scene")
                continue
            sweep_gaps.append(dt)
            if dt > maximum_pair_gap_sec:
                continue
            previous_instances = {
                int(instance["instance_id"]): instance
                for instance in previous.get("instances", [])
                if "instance_id" in instance
            }
            current_instances = {
                int(instance["instance_id"]): instance
                for instance in current.get("instances", [])
                if "instance_id" in instance
            }
            for instance_id in previous_instances.keys() & current_instances.keys():
                center0, velocity0 = global_center_and_velocity(
                    previous, previous_instances[instance_id]
                )
                center1, velocity1 = global_center_and_velocity(
                    current, current_instances[instance_id]
                )
                if velocity0 is None or velocity1 is None:
                    continue
                finite_difference = (center1 - center0)[:2] / dt
                expected = 0.5 * (velocity0[:2] + velocity1[:2])
                mismatch = float(np.linalg.norm(finite_difference - expected))
                velocity_errors.append(mismatch)
                if mismatch > velocity_reject_mps:
                    velocity_reject_count += 1
                elif mismatch > velocity_warn_mps:
                    velocity_warn_count += 1

    if velocity_errors:
        mismatch_p95 = float(np.percentile(velocity_errors, 95.0))
        reject_fraction = velocity_reject_count / len(velocity_errors)
        if mismatch_p95 > velocity_reject_mps and reject_fraction > 0.05:
            errors.append(
                "BBox-centre velocity mismatch p95 {:.3f} m/s exceeds "
                "{:.3f} m/s with {:.1%} rejected pairs".format(
                    mismatch_p95, velocity_reject_mps, reject_fraction
                )
            )
        elif velocity_warn_count or velocity_reject_count:
            warnings.append(
                "{} warning and {} rejected velocity pair(s); inspect tails"
                .format(velocity_warn_count, velocity_reject_count)
            )
    else:
        warnings.append("no adjacent instance trajectories available")

    if transform_orthogonality and max(transform_orthogonality) > 1.0e-5:
        errors.append("sweep transform rotation is not orthonormal")
    if transform_determinant and max(transform_determinant) > 1.0e-5:
        errors.append("sweep transform rotation determinant is not +1")
    if transform_bottom_row and max(transform_bottom_row) > 1.0e-8:
        errors.append("sweep transform has an invalid homogeneous bottom row")

    synchronization = {
        "available": False,
        "covered_samples": 0,
        "missing_samples": len(sample_ids),
        "ego_age_sec": {"count": 0},
        "object_age_sec": {"count": 0},
        "camera_age_sec": {"count": 0},
    }
    conversion_audit_path = info_dir / "conversion_audit.json"
    if conversion_audit_path.is_file():
        conversion = json.loads(
            conversion_audit_path.read_text(encoding="utf-8")
        )
        ego_ages = []
        object_ages = []
        camera_ages = []
        covered = set()
        for item in conversion.get("synchronization", []):
            sample_idx = str(item.get("sample_idx", ""))
            if sample_idx not in sample_ids:
                continue
            covered.add(sample_idx)
            if item.get("ego_age_ms") is not None:
                ego_ages.append(float(item["ego_age_ms"]) / 1000.0)
            if item.get("object_age_ms") is not None:
                object_ages.append(float(item["object_age_ms"]) / 1000.0)
            for value in item.get("camera_age_ms", {}).values():
                camera_ages.append(float(value) / 1000.0)
        synchronization = {
            "available": True,
            "covered_samples": len(covered),
            "missing_samples": len(sample_ids - covered),
            "ego_age_sec": summary(ego_ages),
            "object_age_sec": summary(object_ages),
            "camera_age_sec": summary(camera_ages),
        }
        if sample_ids - covered:
            warnings.append(
                "conversion audit misses {} retained sample(s)".format(
                    len(sample_ids - covered)
                )
            )
        if ego_ages and max(ego_ages) > maximum_state_age_sec + 1.0e-9:
            errors.append("Ego synchronization age exceeds its limit")
        if object_ages and max(object_ages) > maximum_state_age_sec + 1.0e-9:
            errors.append("Object synchronization age exceeds its limit")
        if camera_ages and max(camera_ages) > maximum_camera_age_sec + 1.0e-9:
            errors.append("camera synchronization age exceeds its limit")
    else:
        warnings.append("conversion_audit.json is unavailable; sync age not audited")

    report = {
        "schema_version": 1,
        "valid": not errors,
        "contract": {
            "point_storage": "separate_xyzi_with_causal_references",
            "past_time_feature": "positive_age_seconds",
            "velocity_unit": "m/s",
            "velocity_reference": "geometric_bbox_center",
        },
        "splits": split_counts,
        "scenes": {
            split: len({
                scene for (row_split, scene) in scene_rows
                if row_split == split
            })
            for split in SPLITS
        },
        "split_leakage": leaking_scenes,
        "sweep_age_sec": summary(sweep_ages),
        "adjacent_sample_gap_sec": summary(sweep_gaps),
        "sweep_transform": {
            "orthogonality_error": summary(transform_orthogonality),
            "determinant_error": summary(transform_determinant),
            "bottom_row_error": summary(transform_bottom_row),
        },
        "synchronization": synchronization,
        "velocity_speed_mps": {
            class_name: summary(values)
            for class_name, values in velocity_speeds.items()
        },
        "velocity_sources": dict(velocity_sources),
        "excessive_vehicle_speed": {
            "ceiling_mps": maximum_expected_vehicle_speed_mps,
            "count": excessive_vehicle_speed_count,
            "examples": excessive_vehicle_speed_examples,
        },
        "bbox_center_velocity_consistency_mps": {
            **summary(velocity_errors),
            "warning_pairs": velocity_warn_count,
            "rejected_pairs": velocity_reject_count,
        },
        "errors": errors,
        "warnings": warnings,
    }
    return report


def main():
    args = arguments()
    report = audit(
        args.info_dir.expanduser().resolve(),
        args.velocity_warn_mps,
        args.velocity_reject_mps,
        args.maximum_pair_gap_sec,
        args.maximum_expected_vehicle_speed_mps,
        args.maximum_state_age_sec,
        args.maximum_camera_age_sec,
    )
    target = args.info_dir / "temporal_quality_audit.json"
    target.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report["valid"] and not args.allow_quality_errors:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
