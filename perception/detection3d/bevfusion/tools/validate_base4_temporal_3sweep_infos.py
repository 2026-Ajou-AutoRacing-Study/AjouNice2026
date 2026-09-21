#!/usr/bin/env python3
"""Validate MORAI Base4-Temporal-3S infos and Nx5 point payloads."""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("info", type=Path)
    parser.add_argument(
        "--host-root",
        type=Path,
        help="host dataset root used to replace --serialized-root",
    )
    parser.add_argument(
        "--serialized-root",
        type=Path,
        help="dataset root stored inside the PKL",
    )
    return parser.parse_args()


def resolve_path(path, host_root, serialized_root):
    path = Path(path)

    if host_root is None:
        return path

    if serialized_root is None:
        raise ValueError(
            "--host-root requires --serialized-root"
        )

    return host_root / path.relative_to(serialized_root)


def main():
    args = arguments()

    with args.info.open("rb") as stream:
        payload = pickle.load(stream)

    if "data_list" not in payload:
        raise ValueError("PKL has no data_list")

    rows = payload["data_list"]

    errors = []

    total_points = 0
    total_gt = 0

    global_dt_min = float("inf")
    global_dt_max = float("-inf")

    for index, row in enumerate(rows):
        sample_id = row.get(
            "sample_idx",
            "row_{:06d}".format(index),
        )

        lidar_info = row.get("lidar_points", {})

        if lidar_info.get("num_pts_feats") != 5:
            errors.append(
                "{} num_pts_feats != 5".format(sample_id)
            )

        lidar_path = resolve_path(
            lidar_info["lidar_path"],
            args.host_root,
            args.serialized_root,
        )

        if not lidar_path.is_file():
            errors.append(
                "{} missing {}".format(
                    sample_id,
                    lidar_path,
                )
            )
            continue

        raw = np.fromfile(
            lidar_path,
            dtype=np.float32,
        )

        if raw.size % 5 != 0:
            errors.append(
                "{} point payload {} is not divisible by 5".format(
                    sample_id,
                    raw.size,
                )
            )
            continue

        points = raw.reshape(-1, 5)

        if len(points) == 0:
            errors.append(
                "{} empty point cloud".format(sample_id)
            )
            continue

        if not np.isfinite(points).all():
            errors.append(
                "{} non-finite point value".format(sample_id)
            )

        dt = points[:, 4]

        dt_min = float(dt.min())
        dt_max = float(dt.max())

        global_dt_min = min(global_dt_min, dt_min)
        global_dt_max = max(global_dt_max, dt_max)

        if dt_max > 1e-6:
            errors.append(
                "{} positive dt exists: {}".format(
                    sample_id,
                    dt_max,
                )
            )

        if not np.any(np.isclose(dt, 0.0, atol=1e-6)):
            errors.append(
                "{} current sweep dt=0 missing".format(
                    sample_id
                )
            )

        temporal = row.get("temporal")

        if temporal is None:
            errors.append(
                "{} temporal metadata missing".format(
                    sample_id
                )
            )
        else:
            if temporal.get("num_sweeps") != 3:
                errors.append(
                    "{} num_sweeps != 3".format(
                        sample_id
                    )
                )

            lags = temporal.get(
                "time_lags_sec",
                [],
            )

            if len(lags) != 3:
                errors.append(
                    "{} time_lags length != 3".format(
                        sample_id
                    )
                )
            else:
                if not (
                    float(lags[0])
                    < float(lags[1])
                    < 0.0
                ):
                    errors.append(
                        "{} invalid lag ordering {}".format(
                            sample_id,
                            lags,
                        )
                    )

                if abs(float(lags[2])) > 1e-9:
                    errors.append(
                        "{} current temporal lag != 0".format(
                            sample_id
                        )
                    )

            transforms = temporal.get(
                "past_to_current_lidar",
                [],
            )

            if len(transforms) != 3:
                errors.append(
                    "{} temporal transform count != 3".format(
                        sample_id
                    )
                )

        for instance in row.get("instances", []):
            box = np.asarray(
                instance.get("bbox_3d", []),
                dtype=np.float32,
            )

            if box.shape != (9,):
                errors.append(
                    "{} GT box shape {} != (9,)".format(
                        sample_id,
                        box.shape,
                    )
                )
                continue

            if not np.isfinite(box).all():
                errors.append(
                    "{} non-finite GT box".format(
                        sample_id
                    )
                )

            if not np.isfinite(box[7:9]).all():
                errors.append(
                    "{} invalid velocity GT".format(
                        sample_id
                    )
                )

            total_gt += 1

        total_points += len(points)

    print("samples:", len(rows))
    print("GT instances:", total_gt)
    print("merged points:", total_points)

    if rows and total_gt == 0:
        errors.append(
            "dataset contains samples but zero GT instances"
        )

    if rows:
        print(
            "avg points/sample:",
            total_points / len(rows),
        )

    if global_dt_min != float("inf"):
        print(
            "dt range [sec]:",
            global_dt_min,
            global_dt_max,
        )

    if errors:
        print()
        print("VALIDATION_FAILED")
        for error in errors[:100]:
            print(" -", error)

        if len(errors) > 100:
            print(
                "... {} additional errors".format(
                    len(errors) - 100
                )
            )

        raise SystemExit(1)

    print()
    print("BASE4_TEMPORAL_DATASET_OK")


if __name__ == "__main__":
    main()
