#!/usr/bin/env python3
"""Quality gate for MORAI Base4-Temporal-3S datasets.

The audit is intentionally conservative.  A generated dataset is considered a
candidate dataset until this script verifies temporal synchronization,
velocity labels, trajectory consistency and split isolation.

Velocity cross-check:
    previous GT center
        -- past LiDAR -> current LiDAR transform -->
    previous center expressed in current LiDAR frame

    v_fd = (center_current - center_previous_in_current) / dt

This is compared with the MORAI velocity GT expressed in current LiDAR axes.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
import re
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


VEHICLE_LABEL = 0


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "infos",
        nargs="+",
        type=Path,
        help="train/val/test Base4 temporal PKLs",
    )

    parser.add_argument(
        "--report-root",
        type=Path,
        required=True,
    )

    parser.add_argument(
        "--sync-json",
        type=Path,
        help=(
            "base4_temporal_3sweep_sync.json; when omitted it is "
            "looked up beside the first PKL"
        ),
    )

    parser.add_argument(
        "--filtered-root",
        type=Path,
        help="optionally write PKLs with sample-level REJECT rows removed",
    )

    parser.add_argument(
        "--drop-rejected-scenes",
        action="store_true",
        help="also remove every row belonging to a scene rated REJECT",
    )

    parser.add_argument(
        "--require-vehicle",
        action="store_true",
        help="reject samples containing no valid Vehicle GT",
    )

    #
    # Conservative initial thresholds.  These are audit thresholds, not model
    # performance targets.
    #
    parser.add_argument(
        "--velocity-warn-mps",
        type=float,
        default=2.0,
    )
    parser.add_argument(
        "--velocity-reject-mps",
        type=float,
        default=5.0,
    )

    parser.add_argument(
        "--vehicle-speed-reject-mps",
        type=float,
        default=55.0,
        help="~198 km/h",
    )

    parser.add_argument(
        "--accel-warn-mps2",
        type=float,
        default=15.0,
    )
    parser.add_argument(
        "--accel-reject-mps2",
        type=float,
        default=25.0,
    )

    parser.add_argument(
        "--max-yaw-rate-for-fd-deg-s",
        type=float,
        default=30.0,
        help=(
            "skip position/velocity mismatch judgement above this yaw rate "
            "because rotating pivot-to-center offsets can bias center FD"
        ),
    )

    parser.add_argument(
        "--sync-warn-ms",
        type=float,
        default=60.0,
    )
    parser.add_argument(
        "--sync-reject-ms",
        type=float,
        default=100.0,
    )

    parser.add_argument(
        "--scene-reject-ratio",
        type=float,
        default=0.10,
    )
    parser.add_argument(
        "--scene-warn-ratio",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--min-scene-samples",
        type=int,
        default=20,
    )

    return parser.parse_args()


def wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def percentile(values, q):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def summary_stats(values):
    if not values:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p90": None,
            "p95": None,
            "p99": None,
            "max": None,
        }

    array = np.asarray(values, dtype=np.float64)

    return {
        "count": int(len(array)),
        "mean": float(array.mean()),
        "p50": percentile(values, 50),
        "p90": percentile(values, 90),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": float(array.max()),
    }


def split_name(path):
    match = re.search(
        r"_(train|val|test)\.pkl$",
        path.name,
    )
    return match.group(1) if match else path.stem


def severity(current, new):
    order = {
        "PASS": 0,
        "WARN": 1,
        "REJECT": 2,
    }

    return new if order[new] > order[current] else current


def transform_point(point, transform):
    xyz1 = np.asarray(
        [
            float(point[0]),
            float(point[1]),
            float(point[2]),
            1.0,
        ],
        dtype=np.float64,
    )

    return (
        np.asarray(transform, dtype=np.float64)
        @ xyz1
    )[:3]


def transform_velocity(velocity, transform):
    rotation = np.asarray(
        transform,
        dtype=np.float64,
    )[:3, :3]

    vector = np.asarray(
        [
            float(velocity[0]),
            float(velocity[1]),
            0.0,
        ],
        dtype=np.float64,
    )

    return (rotation @ vector)[:2]


def load_payload(path):
    with path.open("rb") as stream:
        payload = pickle.load(stream)

    if "data_list" not in payload:
        raise ValueError(
            "{} has no data_list".format(path)
        )

    return payload


def main():
    args = arguments()

    report_root = args.report_root.expanduser().resolve()
    report_root.mkdir(parents=True, exist_ok=True)

    #
    # Load all splits.
    #
    datasets = {}
    rows_by_split = {}

    for info_path in args.infos:
        info_path = info_path.expanduser().resolve()
        split = split_name(info_path)

        payload = load_payload(info_path)

        datasets[split] = {
            "path": info_path,
            "payload": payload,
        }

        rows_by_split[split] = payload["data_list"]

    #
    # Optional synchronization audit generated by the dataset builder.
    #
    sync_path = args.sync_json

    if sync_path is None:
        candidate = (
            args.infos[0].expanduser().resolve().parent
            / "base4_temporal_3sweep_sync.json"
        )

        if candidate.is_file():
            sync_path = candidate

    sync_by_sample = {}

    if sync_path is not None:
        sync_path = sync_path.expanduser().resolve()

        if sync_path.is_file():
            sync_rows = json.loads(
                sync_path.read_text(encoding="utf-8")
            )

            sync_by_sample = {
                row["sample_idx"]: row
                for row in sync_rows
            }

            print("sync metadata:", sync_path)
        else:
            print("WARNING: sync JSON not found:", sync_path)

    #
    # Check scene leakage before analysing individual samples.
    #
    scene_splits = defaultdict(set)

    for split, rows in rows_by_split.items():
        for row in rows:
            scene_splits[str(row["scene_id"])].add(split)

    leaking_scenes = {
        scene: sorted(splits)
        for scene, splits in scene_splits.items()
        if len(splits) > 1
    }

    #
    # Build timestamp lookups per scene.
    #
    scene_rows = defaultdict(list)

    for split, rows in rows_by_split.items():
        for row in rows:
            scene_rows[str(row["scene_id"])].append(
                (float(row["timestamp"]), split, row)
            )

    for rows in scene_rows.values():
        rows.sort(key=lambda item: item[0])

    def previous_row_at(scene_id, target_stamp):
        rows = scene_rows[str(scene_id)]

        best = None
        best_error = float("inf")

        for stamp, split, row in rows:
            error = abs(float(stamp) - float(target_stamp))

            if error < best_error:
                best_error = error
                best = (stamp, split, row)

        if best is None or best_error > 1e-3:
            return None

        return best[2]

    #
    # Metrics.
    #
    velocity_errors = []
    accelerations = []
    vehicle_speeds = []
    vehicle_ranges = []
    yaw_rates = []
    sweep_gaps_ms = []
    sync_ages_ms = []

    sample_reports = {}
    scene_quality = defaultdict(
        lambda: {
            "samples": 0,
            "pass": 0,
            "warn": 0,
            "reject": 0,
        }
    )

    track_counts = Counter()

    #
    # Evaluate each sample.
    #
    for split, rows in rows_by_split.items():
        for row in rows:
            sample_id = str(row["sample_idx"])
            scene_id = str(row["scene_id"])

            status = "PASS"
            reasons = []

            instances = row.get("instances", [])

            vehicles = [
                instance
                for instance in instances
                if int(instance["bbox_label_3d"]) == VEHICLE_LABEL
            ]

            if args.require_vehicle and not vehicles:
                status = "REJECT"
                reasons.append("no_vehicle_gt")

            #
            # Temporal structure.
            #
            temporal = row.get("temporal")

            if temporal is None:
                status = "REJECT"
                reasons.append("missing_temporal_metadata")
                previous = None
                transform_prev_to_current = None
                dt = None
            else:
                stamps = temporal.get("timestamps", [])
                lags = temporal.get("time_lags_sec", [])
                transforms = temporal.get(
                    "past_to_current_lidar",
                    [],
                )

                if (
                    len(stamps) != 3
                    or len(lags) != 3
                    or len(transforms) != 3
                ):
                    status = "REJECT"
                    reasons.append("invalid_temporal_metadata")
                    previous = None
                    transform_prev_to_current = None
                    dt = None
                else:
                    dt = -float(lags[1])

                    if dt <= 0.0:
                        status = "REJECT"
                        reasons.append("invalid_dt")

                    previous = previous_row_at(
                        scene_id,
                        float(stamps[1]),
                    )

                    transform_prev_to_current = np.asarray(
                        transforms[1],
                        dtype=np.float64,
                    )

            #
            # Synchronization quality.
            #
            sync = sync_by_sample.get(sample_id)

            sample_sync_max = None

            if sync is not None:
                ages = []

                ego_ages = sync.get("ego_age_ms", [])
                if isinstance(ego_ages, list):
                    ages.extend(float(v) for v in ego_ages)

                if sync.get("object_age_ms") is not None:
                    ages.append(float(sync["object_age_ms"]))

                camera_age = sync.get("camera_age_ms", {})
                ages.extend(float(v) for v in camera_age.values())

                if ages:
                    sample_sync_max = max(ages)
                    sync_ages_ms.extend(ages)

                    if sample_sync_max > args.sync_reject_ms:
                        status = severity(status, "REJECT")
                        reasons.append(
                            "sync_age>{:.1f}ms".format(
                                args.sync_reject_ms
                            )
                        )
                    elif sample_sync_max > args.sync_warn_ms:
                        status = severity(status, "WARN")
                        reasons.append(
                            "sync_age>{:.1f}ms".format(
                                args.sync_warn_ms
                            )
                        )

                for value in sync.get("sweep_gap_ms", []):
                    sweep_gaps_ms.append(float(value))

            #
            # Vehicle GT plausibility.
            #
            current_by_id = {}

            max_speed = 0.0
            max_velocity_error = None
            max_acceleration = None
            matched_tracks = 0

            for instance in vehicles:
                instance_id = int(instance["instance_id"])
                current_by_id[instance_id] = instance

                box = np.asarray(
                    instance["bbox_3d"],
                    dtype=np.float64,
                )

                if box.shape != (9,):
                    status = severity(status, "REJECT")
                    reasons.append(
                        "vehicle_box_not_9d:{}".format(instance_id)
                    )
                    continue

                velocity = box[7:9]

                if not np.isfinite(velocity).all():
                    status = severity(status, "REJECT")
                    reasons.append(
                        "nonfinite_velocity:{}".format(instance_id)
                    )
                    continue

                speed = float(np.linalg.norm(velocity))
                distance = float(np.linalg.norm(box[:2]))

                vehicle_speeds.append(speed)
                vehicle_ranges.append(distance)

                max_speed = max(max_speed, speed)
                track_counts[(scene_id, instance_id)] += 1

                if speed > args.vehicle_speed_reject_mps:
                    status = severity(status, "REJECT")
                    reasons.append(
                        "vehicle_speed>{:.1f}:{}".format(
                            args.vehicle_speed_reject_mps,
                            instance_id,
                        )
                    )

            #
            # Velocity cross-check using previous GT position.
            #
            if (
                previous is not None
                and dt is not None
                and dt > 0.0
                and transform_prev_to_current is not None
            ):
                previous_vehicles = {
                    int(instance["instance_id"]): instance
                    for instance in previous.get("instances", [])
                    if int(instance["bbox_label_3d"]) == VEHICLE_LABEL
                }

                for instance_id, current in current_by_id.items():
                    previous_instance = previous_vehicles.get(instance_id)

                    if previous_instance is None:
                        continue

                    current_box = np.asarray(
                        current["bbox_3d"],
                        dtype=np.float64,
                    )

                    previous_box = np.asarray(
                        previous_instance["bbox_3d"],
                        dtype=np.float64,
                    )

                    if (
                        current_box.shape != (9,)
                        or previous_box.shape != (9,)
                    ):
                        continue

                    previous_center_current = transform_point(
                        previous_box[:3],
                        transform_prev_to_current,
                    )

                    displacement = (
                        current_box[:3]
                        - previous_center_current
                    )

                    fd_velocity = displacement[:2] / dt

                    gt_velocity = current_box[7:9]

                    mismatch = float(
                        np.linalg.norm(
                            fd_velocity - gt_velocity
                        )
                    )

                    yaw_delta = wrap_angle(
                        current_box[6] - previous_box[6]
                    )

                    yaw_rate = abs(
                        math.degrees(yaw_delta) / dt
                    )

                    yaw_rates.append(yaw_rate)
                    matched_tracks += 1

                    #
                    # Strong turns can move the geometric box center relative
                    # to the actor pivot.  Report them, but do not judge the
                    # velocity label using center finite differences there.
                    #
                    if (
                        yaw_rate
                        <= args.max_yaw_rate_for_fd_deg_s
                    ):
                        velocity_errors.append(mismatch)

                        if max_velocity_error is None:
                            max_velocity_error = mismatch
                        else:
                            max_velocity_error = max(
                                max_velocity_error,
                                mismatch,
                            )

                        if mismatch > args.velocity_reject_mps:
                            status = severity(status, "REJECT")
                            reasons.append(
                                "velocity_mismatch>{:.1f}:{}".format(
                                    args.velocity_reject_mps,
                                    instance_id,
                                )
                            )
                        elif mismatch > args.velocity_warn_mps:
                            status = severity(status, "WARN")
                            reasons.append(
                                "velocity_mismatch>{:.1f}:{}".format(
                                    args.velocity_warn_mps,
                                    instance_id,
                                )
                            )

                    #
                    # Acceleration sanity check.  Rotate the previous
                    # velocity into current LiDAR axes before differencing.
                    #
                    previous_velocity_current = transform_velocity(
                        previous_box[7:9],
                        transform_prev_to_current,
                    )

                    acceleration = float(
                        np.linalg.norm(
                            current_box[7:9]
                            - previous_velocity_current
                        )
                        / dt
                    )

                    accelerations.append(acceleration)

                    if max_acceleration is None:
                        max_acceleration = acceleration
                    else:
                        max_acceleration = max(
                            max_acceleration,
                            acceleration,
                        )

                    if acceleration > args.accel_reject_mps2:
                        status = severity(status, "REJECT")
                        reasons.append(
                            "accel>{:.1f}:{}".format(
                                args.accel_reject_mps2,
                                instance_id,
                            )
                        )
                    elif acceleration > args.accel_warn_mps2:
                        status = severity(status, "WARN")
                        reasons.append(
                            "accel>{:.1f}:{}".format(
                                args.accel_warn_mps2,
                                instance_id,
                            )
                        )

            sample_reports[sample_id] = {
                "sample_idx": sample_id,
                "split": split,
                "scene_id": scene_id,
                "timestamp": float(row["timestamp"]),
                "status": status,
                "reasons": reasons,
                "vehicle_count": len(vehicles),
                "matched_vehicle_tracks": matched_tracks,
                "max_vehicle_speed_mps": max_speed,
                "max_velocity_mismatch_mps": max_velocity_error,
                "max_acceleration_mps2": max_acceleration,
                "max_sync_age_ms": sample_sync_max,
            }

            scene = scene_quality[scene_id]
            scene["samples"] += 1
            scene[status.lower()] += 1

    #
    # Scene-level classification.
    #
    scene_reports = {}

    for scene_id, counts in scene_quality.items():
        sample_count = int(counts["samples"])

        reject_ratio = (
            float(counts["reject"]) / sample_count
            if sample_count else 1.0
        )

        warn_ratio = (
            float(counts["warn"]) / sample_count
            if sample_count else 0.0
        )

        status = "PASS"
        reasons = []

        if scene_id in leaking_scenes:
            status = "REJECT"
            reasons.append("split_leakage")

        if reject_ratio > args.scene_reject_ratio:
            status = severity(status, "REJECT")
            reasons.append(
                "reject_ratio>{:.2f}".format(
                    args.scene_reject_ratio
                )
            )

        if sample_count < args.min_scene_samples:
            status = severity(status, "WARN")
            reasons.append(
                "samples<{}".format(
                    args.min_scene_samples
                )
            )

        if warn_ratio > args.scene_warn_ratio:
            status = severity(status, "WARN")
            reasons.append(
                "warn_ratio>{:.2f}".format(
                    args.scene_warn_ratio
                )
            )

        scene_reports[scene_id] = {
            **counts,
            "reject_ratio": reject_ratio,
            "warn_ratio": warn_ratio,
            "status": status,
            "reasons": reasons,
        }

    #
    # Global distribution summary.
    #
    status_counts = Counter(
        row["status"]
        for row in sample_reports.values()
    )

    global_status = "PASS"

    if leaking_scenes:
        global_status = "REJECT"

    if status_counts["REJECT"] > 0:
        global_status = severity(
            global_status,
            "WARN",
        )

    if any(
        row["status"] == "REJECT"
        for row in scene_reports.values()
    ):
        global_status = severity(
            global_status,
            "REJECT",
        )

    report = {
        "global_status": global_status,

        "split_leakage": leaking_scenes,

        "sample_status_counts": dict(status_counts),

        "thresholds": {
            "velocity_warn_mps":
                args.velocity_warn_mps,
            "velocity_reject_mps":
                args.velocity_reject_mps,
            "vehicle_speed_reject_mps":
                args.vehicle_speed_reject_mps,
            "accel_warn_mps2":
                args.accel_warn_mps2,
            "accel_reject_mps2":
                args.accel_reject_mps2,
            "max_yaw_rate_for_fd_deg_s":
                args.max_yaw_rate_for_fd_deg_s,
            "sync_warn_ms":
                args.sync_warn_ms,
            "sync_reject_ms":
                args.sync_reject_ms,
        },

        "velocity_mismatch_mps":
            summary_stats(velocity_errors),

        "vehicle_speed_mps":
            summary_stats(vehicle_speeds),

        "vehicle_range_m":
            summary_stats(vehicle_ranges),

        "acceleration_mps2":
            summary_stats(accelerations),

        "yaw_rate_deg_s":
            summary_stats(yaw_rates),

        "sweep_gap_ms":
            summary_stats(sweep_gaps_ms),

        "sync_age_ms":
            summary_stats(sync_ages_ms),

        "scenes": scene_reports,

        "track_lengths_samples": summary_stats(
            list(track_counts.values())
        ),
    }

    report_path = report_root / "quality_audit.json"

    report_path.write_text(
        json.dumps(
            report,
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    #
    # Sample CSV.
    #
    sample_csv = report_root / "sample_quality.csv"

    with sample_csv.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        fieldnames = [
            "sample_idx",
            "split",
            "scene_id",
            "timestamp",
            "status",
            "reasons",
            "vehicle_count",
            "matched_vehicle_tracks",
            "max_vehicle_speed_mps",
            "max_velocity_mismatch_mps",
            "max_acceleration_mps2",
            "max_sync_age_ms",
        ]

        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        for row in sample_reports.values():
            output_row = dict(row)
            output_row["reasons"] = ";".join(
                output_row["reasons"]
            )
            writer.writerow(output_row)

    #
    # Scene CSV.
    #
    scene_csv = report_root / "scene_quality.csv"

    with scene_csv.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        fieldnames = [
            "scene_id",
            "status",
            "samples",
            "pass",
            "warn",
            "reject",
            "warn_ratio",
            "reject_ratio",
            "reasons",
        ]

        writer = csv.DictWriter(
            stream,
            fieldnames=fieldnames,
        )

        writer.writeheader()

        for scene_id, row in scene_reports.items():
            output_row = {
                "scene_id": scene_id,
                **row,
            }
            output_row["reasons"] = ";".join(
                output_row["reasons"]
            )
            writer.writerow(output_row)

    #
    # Optional filtered training PKLs.
    #
    if args.filtered_root is not None:
        filtered_root = (
            args.filtered_root.expanduser().resolve()
        )
        filtered_root.mkdir(
            parents=True,
            exist_ok=True,
        )

        for split, dataset in datasets.items():
            payload = dataset["payload"]

            accepted = []

            for row in payload["data_list"]:
                sample_id = str(row["sample_idx"])
                scene_id = str(row["scene_id"])

                sample_status = sample_reports[
                    sample_id
                ]["status"]

                if sample_status == "REJECT":
                    continue

                if (
                    args.drop_rejected_scenes
                    and scene_reports[scene_id]["status"]
                    == "REJECT"
                ):
                    continue

                accepted.append(row)

            output_payload = dict(payload)
            output_payload["data_list"] = accepted

            metainfo = dict(
                output_payload.get(
                    "metainfo",
                    {}
                )
            )

            metainfo["quality_audit"] = {
                "source":
                    str(report_path),
                "removed_samples":
                    len(payload["data_list"])
                    - len(accepted),
            }

            output_payload["metainfo"] = metainfo

            output_path = (
                filtered_root
                / dataset["path"].name
            )

            with output_path.open("wb") as stream:
                pickle.dump(
                    output_payload,
                    stream,
                    protocol=pickle.HIGHEST_PROTOCOL,
                )

            print(
                "{}: {} -> {} samples".format(
                    split,
                    len(payload["data_list"]),
                    len(accepted),
                )
            )

    print()
    print("=== BASE4 TEMPORAL QUALITY AUDIT ===")
    print("global status:", global_status)
    print("samples:", dict(status_counts))
    print("scene count:", len(scene_reports))
    print("split leakage:", len(leaking_scenes))

    print()
    print(
        "velocity mismatch [m/s]:",
        report["velocity_mismatch_mps"],
    )

    print(
        "vehicle speed [m/s]:",
        report["vehicle_speed_mps"],
    )

    print(
        "acceleration [m/s^2]:",
        report["acceleration_mps2"],
    )

    print(
        "sweep gap [ms]:",
        report["sweep_gap_ms"],
    )

    print()
    print("report:", report_path)
    print("samples:", sample_csv)
    print("scenes:", scene_csv)

    if global_status == "REJECT":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
