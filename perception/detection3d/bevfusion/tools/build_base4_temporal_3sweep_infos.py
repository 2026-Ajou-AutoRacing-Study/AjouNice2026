#!/usr/bin/env python3
"""Build MORAI Base4-Temporal-3S BEVFusion infos from marker-delimited rosbags.

Input at current time t:
    - four cameras at t
    - LiDAR sweeps t-2, t-1, t

Past LiDAR sweeps are ego-motion compensated into the current LiDAR frame.
Stored point format:
    [x, y, z, intensity, time_lag_sec]

Current-frame GT box format:
    [x, y, z, length, width, height, yaw, vx, vy]
"""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag


REPO_ROOT = Path(__file__).resolve().parents[4]

MORAI_SCRIPTS = REPO_ROOT / "dataset/morai_od/scripts"
sys.path.insert(0, str(MORAI_SCRIPTS))

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

from audit_base3_online_bag import build_capture_windows  # noqa: E402
from audit_base3_online_gt_alignment import (  # noqa: E402
    CAMERAS,
    CALIBRATION_CAMERA_NAMES,
    EGO_TOPIC,
    LIDAR_TOPIC,
    MARKER_TOPIC,
    OBJECT_TOPIC,
    actors,
    align_ego_state,
    align_object_state,
    cloud_xyz_i,
    corrected_box,
    nearest,
    points_in_box,
)

from base4_temporal_geometry import (  # noqa: E402
    actor_velocity_in_current_lidar,
    past_lidar_to_current_lidar,
    transform_points,
)


CLASSES = ("vehicle", "pedestrian", "obstacle")
CAMERA_ORDER = ("front", "front_left", "front_right", "back")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("bags", nargs="+", type=Path)

    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--asset-offsets", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)

    parser.add_argument(
        "--serialized-root",
        type=Path,
        help="root stored in PKL paths instead of host output-root",
    )

    parser.add_argument(
        "--frame-stride",
        type=int,
        default=1,
        help="current LiDAR frame stride; temporal history always uses adjacent sweeps",
    )

    parser.add_argument(
        "--max-sweep-gap-sec",
        type=float,
        default=0.20,
        help="maximum allowed gap between consecutive LiDAR sweeps",
    )

    parser.add_argument("--minimum-gt-points", type=int, default=5)
    parser.add_argument("--max-state-age-sec", type=float, default=0.10)
    parser.add_argument("--max-camera-age-sec", type=float, default=0.10)

    parser.add_argument(
        "--state-alignment",
        choices=("interpolate", "nearest"),
        default="interpolate",
    )

    parser.add_argument(
        "--max-extrapolation-sec",
        type=float,
        default=0.05,
    )

    parser.add_argument(
        "--require-positive-class",
        choices=CLASSES,
        help="drop samples without a valid GT instance of this class",
    )

    parser.add_argument(
        "--distance-filter",
        choices=("box-range", "radial-0-30", "radial-30-plus", "all"),
        default="box-range",
    )

    parser.add_argument(
        "--distance-threshold-m",
        type=float,
        default=30.0,
    )

    parser.add_argument(
        "--point-cloud-range",
        type=float,
        nargs=6,
        default=(-30.0, -30.0, -3.0, 30.0, 30.0, 2.0),
        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
    )

    return parser.parse_args()


def serialized_path(host_path, host_root, serialized_root):
    return serialized_root / host_path.relative_to(host_root)


def message_timestamp(message, bag_time):
    """Prefer sensor header timestamp and fall back to rosbag record time."""

    if hasattr(message, "header"):
        stamp = getattr(message.header, "stamp", None)

        if stamp is not None:
            try:
                value = float(stamp.to_sec())
                if value > 0.0:
                    return value
            except Exception:
                pass

    return float(bag_time.to_sec())


def write_camera(path, message):
    payload = bytes(message.data)

    if not payload.startswith(b"\xff\xd8"):
        raise ValueError(
            "expected JPEG CompressedImage for {}".format(path)
        )

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def keep_box_for_distance(
    box,
    mode,
    threshold_m,
    point_cloud_range,
):
    if mode == "all":
        return True

    if mode == "box-range":
        minimum = np.asarray(
            point_cloud_range[:3],
            dtype=np.float64,
        )
        maximum = np.asarray(
            point_cloud_range[3:],
            dtype=np.float64,
        )

        center = np.asarray(
            box[:3],
            dtype=np.float64,
        )

        return bool(
            np.all(center >= minimum)
            and np.all(center <= maximum)
        )

    distance = math.hypot(
        float(box[0]),
        float(box[1]),
    )

    if mode == "radial-0-30":
        return distance < threshold_m

    if mode == "radial-30-plus":
        return distance >= threshold_m

    raise ValueError(mode)


def append_time_lag(points, lag_sec):
    lag = np.full(
        (len(points), 1),
        float(lag_sec),
        dtype=np.float32,
    )

    return np.concatenate(
        [
            np.asarray(points[:, :4], dtype=np.float32),
            lag,
        ],
        axis=1,
    )


def save_infos(output, split_rows):
    metainfo = {
        "classes": CLASSES,
        "dataset": "MORAI Base4-Temporal-3S",
        "point_features": (
            "x",
            "y",
            "z",
            "intensity",
            "time_lag_sec",
        ),
        "box_features": (
            "x",
            "y",
            "z",
            "length",
            "width",
            "height",
            "yaw",
            "vx",
            "vy",
        ),
    }

    for split, rows in split_rows.items():
        path = (
            output
            / "morai_base4_temporal_3sweep_infos_{}.pkl".format(split)
        )

        payload = {
            "metainfo": metainfo,
            "data_list": rows,
        }

        with path.open("wb") as stream:
            pickle.dump(
                payload,
                stream,
                protocol=pickle.HIGHEST_PROTOCOL,
            )

        print(
            "{}: {} samples -> {}".format(
                split,
                len(rows),
                path,
            )
        )


def main():
    args = arguments()

    if args.frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")

    output = args.output_root.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    stored_root = (
        args.serialized_root
        if args.serialized_root is not None
        else output
    )

    calibration = json.loads(
        args.calibration.read_text(encoding="utf-8")
    )

    catalog = json.loads(
        args.asset_offsets.read_text(encoding="utf-8")
    )["assets"]

    label_index = {
        name: index
        for index, name in enumerate(CLASSES)
    }

    lidar_to_ego = np.asarray(
        calibration["lidar"]["lidar_to_ego"],
        dtype=np.float64,
    )

    ego_to_lidar = np.asarray(
        calibration["lidar"]["ego_to_lidar"],
        dtype=np.float64,
    )

    split_rows = {
        "train": [],
        "val": [],
        "test": [],
    }

    counters = Counter()
    kept_classes = Counter()
    rejected_classes = Counter()
    unknown_assets = set()
    synchronization = []

    #
    # Pass 1: capture markers.
    #
    markers = []

    for bag_path in args.bags:
        bag_path = bag_path.expanduser().resolve()

        with rosbag.Bag(str(bag_path), "r") as bag:
            for _topic, message, bag_time in bag.read_messages(
                topics=[MARKER_TOPIC]
            ):
                markers.append(
                    (
                        float(bag_time.to_sec()),
                        json.loads(message.data),
                    )
                )

    windows, marker_errors = build_capture_windows(markers)

    if marker_errors:
        raise ValueError(
            "invalid capture markers: {}".format(marker_errors)
        )

    starts = {
        payload["scene_id"]: payload
        for _stamp, payload in markers
        if payload.get("event") == "capture_start"
    }

    print("capture windows:", len(windows))

    sensor_topics = (
        LIDAR_TOPIC,
        EGO_TOPIC,
        OBJECT_TOPIC,
        *CAMERAS.values(),
    )

    def convert_window(window, topics):
        marker = starts[window["scene_id"]]

        split = str(marker.get("split") or "train")

        if split not in split_rows:
            raise ValueError(
                "unknown split {} for {}".format(
                    split,
                    window["scene_id"],
                )
            )

        counters["scene_" + split] += 1

        missing = [
            topic
            for topic in sensor_topics
            if not topics[topic]
        ]

        if missing:
            counters["window_missing_topics"] += 1
            print(
                "SKIP {} missing {}".format(
                    window["scene_id"],
                    ", ".join(missing),
                )
            )
            return

        lidar_rows = sorted(
            topics[LIDAR_TOPIC],
            key=lambda row: row[0],
        )

        aligned = {
            topic: sorted(
                topics[topic],
                key=lambda row: row[0],
            )
            for topic in (
                EGO_TOPIC,
                OBJECT_TOPIC,
                *CAMERAS.values(),
            )
        }

        if len(lidar_rows) < 3:
            counters["window_less_than_3_lidar"] += 1
            return

        #
        # A sample is current sweep i plus i-1 and i-2.
        #
        for lidar_index in range(
            2,
            len(lidar_rows),
            args.frame_stride,
        ):
            t2, _r2, msg2 = lidar_rows[lidar_index - 2]
            t1, _r1, msg1 = lidar_rows[lidar_index - 1]
            t0, _r0, msg0 = lidar_rows[lidar_index]

            gap_old = float(t1 - t2)
            gap_new = float(t0 - t1)

            if (
                gap_old <= 0.0
                or gap_new <= 0.0
                or gap_old > args.max_sweep_gap_sec
                or gap_new > args.max_sweep_gap_sec
            ):
                counters["invalid_sweep_gap"] += 1
                continue

            #
            # Align ego state independently to all three LiDAR timestamps.
            #
            ego_rows = []

            valid_state_age = True

            for stamp in (t2, t1, t0):
                nearest_row = nearest(
                    stamp,
                    aligned[EGO_TOPIC],
                )

                age = abs(
                    float(nearest_row[0])
                    - float(stamp)
                )

                if age > args.max_state_age_sec:
                    valid_state_age = False
                    break

                try:
                    ego_state, _alignment = align_ego_state(
                        stamp,
                        aligned[EGO_TOPIC],
                        args.state_alignment,
                        args.max_extrapolation_sec,
                    )
                except ValueError:
                    counters["ego_alignment_error"] += 1
                    valid_state_age = False
                    break

                ego_rows.append(
                    (
                        ego_state,
                        age,
                        _alignment,
                    )
                )

            if not valid_state_age:
                counters["ego_state_age"] += 1
                continue

            ego2 = ego_rows[0][0]
            ego1 = ego_rows[1][0]
            ego0 = ego_rows[2][0]

            #
            # Current object GT only.
            #
            object_nearest = nearest(
                t0,
                aligned[OBJECT_TOPIC],
            )

            object_age = abs(
                float(object_nearest[0])
                - float(t0)
            )

            if object_age > args.max_state_age_sec:
                counters["object_state_age"] += 1
                continue

            try:
                object_state, object_alignment = align_object_state(
                    t0,
                    aligned[OBJECT_TOPIC],
                    args.state_alignment,
                    args.max_extrapolation_sec,
                )
            except ValueError:
                counters["object_alignment_error"] += 1
                continue

            #
            # Load all three raw point clouds.
            #
            points2 = cloud_xyz_i(msg2)
            points1 = cloud_xyz_i(msg1)
            points0 = cloud_xyz_i(msg0)

            #
            # Ego-motion compensate past sweeps into current LiDAR frame.
            #
            transform2 = past_lidar_to_current_lidar(
                ego2,
                ego0,
                lidar_to_ego,
                ego_to_lidar,
            )

            transform1 = past_lidar_to_current_lidar(
                ego1,
                ego0,
                lidar_to_ego,
                ego_to_lidar,
            )

            points2_current = transform_points(
                points2,
                transform2,
            )

            points1_current = transform_points(
                points1,
                transform1,
            )

            #
            # Signed time lag is always relative to the current sweep.
            #
            merged_points = np.concatenate(
                [
                    append_time_lag(
                        points2_current,
                        t2 - t0,
                    ),
                    append_time_lag(
                        points1_current,
                        t1 - t0,
                    ),
                    append_time_lag(
                        points0,
                        0.0,
                    ),
                ],
                axis=0,
            ).astype(np.float32)

            #
            # Build current-frame GT.
            #
            instances = []

            for class_name, actor in actors(object_state):
                _naive, box = corrected_box(
                    ego0,
                    class_name,
                    actor,
                    calibration,
                    catalog,
                )

                if box is None:
                    unknown_assets.add(str(actor.name))
                    continue

                if not keep_box_for_distance(
                    box,
                    args.distance_filter,
                    args.distance_threshold_m,
                    args.point_cloud_range,
                ):
                    rejected_classes[class_name] += 1
                    continue

                #
                # Visibility is intentionally measured on CURRENT sweep only.
                # Past sweeps must not rescue an otherwise invisible GT object.
                #
                count, _mask = points_in_box(
                    points0,
                    box,
                )

                if count < args.minimum_gt_points:
                    rejected_classes[class_name] += 1
                    continue

                velocity = actor_velocity_in_current_lidar(
                    actor,
                    ego0,
                    ego_to_lidar,
                )

                box9 = np.concatenate(
                    [
                        np.asarray(box, dtype=np.float32),
                        np.asarray(
                            velocity[:2],
                            dtype=np.float32,
                        ),
                    ]
                )

                instances.append(
                    {
                        "bbox_3d": box9.tolist(),
                        "bbox_label_3d": label_index[class_name],
                        "num_lidar_pts": int(count),
                        "bbox_3d_isvalid": True,
                        "instance_id": int(actor.unique_id),
                        "asset_name": str(actor.name),
                        "velocity_lidar_mps": [
                            float(velocity[0]),
                            float(velocity[1]),
                        ],
                    }
                )

                kept_classes[class_name] += 1

            if args.require_positive_class is not None:
                required_label = label_index[
                    args.require_positive_class
                ]

                if not any(
                    instance["bbox_label_3d"]
                    == required_label
                    for instance in instances
                ):
                    counters["missing_required_class"] += 1
                    continue

            #
            # Current camera set only.
            #
            images = {}
            camera_ages = {}

            camera_valid = True

            for camera_name in CAMERA_ORDER:
                topic = CAMERAS[camera_name]

                camera_row = nearest(
                    t0,
                    aligned[topic],
                )

                age = abs(
                    float(camera_row[0])
                    - float(t0)
                )

                if age > args.max_camera_age_sec:
                    camera_valid = False
                    break

                camera_ages[camera_name] = age

            if not camera_valid:
                counters["camera_age"] += 1
                continue

            sample_id = "{}_f{:06d}".format(
                window["scene_id"],
                lidar_index,
            )

            sample_dir = (
                output
                / "samples"
                / window["scene_id"]
                / sample_id
            )

            sample_dir.mkdir(
                parents=True,
                exist_ok=True,
            )

            lidar_path = sample_dir / "lidar_3sweep.bin"

            merged_points.tofile(
                str(lidar_path)
            )

            for camera_name in CAMERA_ORDER:
                topic = CAMERAS[camera_name]

                camera_row = nearest(
                    t0,
                    aligned[topic],
                )

                image_path = (
                    sample_dir
                    / "{}.jpg".format(camera_name)
                )

                write_camera(
                    image_path,
                    camera_row[2],
                )

                camera = calibration["cameras"][
                    CALIBRATION_CAMERA_NAMES[camera_name]
                ]

                intrinsic = np.asarray(
                    camera["intrinsic"],
                    dtype=np.float64,
                )

                lidar2cam = (
                    np.asarray(
                        camera["ego_to_sensor"],
                        dtype=np.float64,
                    )
                    @ lidar_to_ego
                )

                images[camera_name] = {
                    "img_path": str(
                        serialized_path(
                            image_path,
                            output,
                            stored_root,
                        )
                    ),
                    "height": int(
                        camera["resolution_px"][1]
                    ),
                    "width": int(
                        camera["resolution_px"][0]
                    ),
                    "cam2img": intrinsic.tolist(),
                    "lidar2cam": lidar2cam.tolist(),
                    "lidar2img": (
                        intrinsic
                        @ lidar2cam[:3, :]
                    ).tolist(),
                    "cam2lidar": np.linalg.inv(
                        lidar2cam
                    ).tolist(),
                }

            row = {
                "sample_idx": sample_id,
                "token": sample_id,
                "scene_id": window["scene_id"],
                "timestamp": float(t0),
                "smoke_only": bool(marker.get("smoke_only", False)),
                "do_not_train": bool(marker.get("do_not_train", False)),

                "lidar_points": {
                    "lidar_path": str(
                        serialized_path(
                            lidar_path,
                            output,
                            stored_root,
                        )
                    ),
                    "num_pts_feats": 5,
                    "lidar2ego": calibration[
                        "lidar"
                    ]["lidar_to_ego"],
                },

                "images": images,
                "instances": instances,

                "temporal": {
                    "num_sweeps": 3,
                    "timestamps": [
                        float(t2),
                        float(t1),
                        float(t0),
                    ],
                    "time_lags_sec": [
                        float(t2 - t0),
                        float(t1 - t0),
                        0.0,
                    ],
                    "past_to_current_lidar": [
                        transform2.tolist(),
                        transform1.tolist(),
                        np.eye(
                            4,
                            dtype=np.float64,
                        ).tolist(),
                    ],
                },
            }

            split_rows[split].append(row)

            synchronization.append(
                {
                    "sample_idx": sample_id,
                    "split": split,

                    "sweep_gap_ms": [
                        gap_old * 1000.0,
                        gap_new * 1000.0,
                    ],

                    "time_lags_ms": [
                        (t2 - t0) * 1000.0,
                        (t1 - t0) * 1000.0,
                        0.0,
                    ],

                    "ego_age_ms": [
                        ego_rows[0][1] * 1000.0,
                        ego_rows[1][1] * 1000.0,
                        ego_rows[2][1] * 1000.0,
                    ],

                    "object_age_ms":
                        object_age * 1000.0,

                    "object_alignment":
                        object_alignment,

                    "camera_age_ms": {
                        key: value * 1000.0
                        for key, value
                        in camera_ages.items()
                    },

                    "num_current_points":
                        int(len(points0)),

                    "num_merged_points":
                        int(len(merged_points)),
                }
            )

            counters["samples_" + split] += 1

    #
    # Pass 2: stream sensor payloads window by window.
    #
    topic_rows = {
        topic: []
        for topic in sensor_topics
    }

    window_index = 0

    def reset_topic_rows():
        return {
            topic: []
            for topic in sensor_topics
        }

    def finish_current_window():
        nonlocal topic_rows, window_index

        if window_index >= len(windows):
            return

        if any(topic_rows.values()):
            convert_window(
                windows[window_index],
                topic_rows,
            )

        topic_rows = reset_topic_rows()
        window_index += 1

    for bag_path in args.bags:
        bag_path = bag_path.expanduser().resolve()

        print("reading:", bag_path)

        with rosbag.Bag(str(bag_path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(
                topics=sensor_topics
            ):
                if window_index >= len(windows):
                    break

                record_time = float(
                    bag_time.to_sec()
                )

                while (
                    window_index < len(windows)
                    and record_time
                    > float(
                        windows[window_index]["end"]
                    )
                ):
                    finish_current_window()

                if window_index >= len(windows):
                    break

                window = windows[window_index]

                if record_time < float(
                    window["start"]
                ):
                    continue

                if record_time > float(
                    window["end"]
                ):
                    continue

                stamp = message_timestamp(
                    message,
                    bag_time,
                )

                topic_rows[topic].append(
                    (
                        stamp,
                        record_time,
                        message,
                    )
                )

    while window_index < len(windows):
        finish_current_window()

    save_infos(
        output,
        split_rows,
    )

    summary = {
        "dataset": "MORAI Base4-Temporal-3S",

        "classes": list(CLASSES),

        "samples": {
            split: len(rows)
            for split, rows
            in split_rows.items()
        },

        "kept_gt": dict(
            kept_classes
        ),

        "rejected_gt": dict(
            rejected_classes
        ),

        "counters": dict(
            counters
        ),

        "unknown_assets": sorted(
            unknown_assets
        ),

        "max_sweep_gap_sec":
            args.max_sweep_gap_sec,

        "frame_stride":
            args.frame_stride,

        "point_format": [
            "x",
            "y",
            "z",
            "intensity",
            "time_lag_sec",
        ],

        "box_format": [
            "x",
            "y",
            "z",
            "length",
            "width",
            "height",
            "yaw",
            "vx",
            "vy",
        ],
    }

    (
        output
        / "base4_temporal_3sweep_summary.json"
    ).write_text(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )

    (
        output
        / "base4_temporal_3sweep_sync.json"
    ).write_text(
        json.dumps(
            synchronization,
            indent=2,
        ),
        encoding="utf-8",
    )

    print()
    print(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
