#!/usr/bin/env python3
"""Extract marker-delimited Base3-Online rosbags into BEVFusion v2 infos."""

from __future__ import annotations

import argparse
import json
import math
import pickle
import sys
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag


REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = REPO_ROOT / "dataset/morai_od/scripts"
PIPELINE_SRC = REPO_ROOT / "base_ws/src/perception_object_pipeline/src"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(PIPELINE_SRC))

from audit_base3_online_bag import build_capture_windows, message_stamp  # noqa: E402
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
    selected_indices,
)
from analyze_morai_lidar_deskew import point_field_array  # noqa: E402
from perception_object_pipeline.rolling_lidar import (  # noqa: E402
    deskew_points_to_reference_pose,
    scan_time_contract,
)


CLASSES = ("vehicle", "pedestrian", "obstacle")
CAMERA_ORDER = ("front", "front_left", "front_right", "back")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--asset-offsets", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--samples-per-scene", type=int, default=3)
    parser.add_argument("--minimum-gt-points", type=int, default=5)
    parser.add_argument("--max-state-age-sec", type=float, default=0.10)
    parser.add_argument(
        "--state-alignment", choices=("interpolate", "nearest"),
        default="interpolate",
        help="align Ego/Object poses to each LiDAR timestamp",
    )
    parser.add_argument(
        "--max-extrapolation-sec", type=float, default=0.05,
        help="maximum velocity-based fallback when a timestamp is not bracketed",
    )
    parser.add_argument("--max-camera-age-sec", type=float, default=0.10)
    parser.add_argument(
        "--lidar-deskew", choices=("off", "ego-pose-2d"), default="off",
        help="cache scan-reference Ego-motion-corrected XYZI files",
    )
    parser.add_argument(
        "--lidar-deskew-reference", choices=("end", "median", "header"),
        default="end",
    )
    parser.add_argument(
        "--deskew-max-point-time-span-sec", type=float, default=0.15,
    )
    parser.add_argument(
        "--deskew-invalid-policy", choices=("drop", "error"), default="drop",
        help="never mix invalid raw scans into a deskewed dataset",
    )
    parser.add_argument(
        "--require-positive-class", choices=CLASSES,
        help=(
            "drop a selected frame when no >= minimum-gt-points instance of "
            "this class remains; useful for class-positive specialization decks"
        ),
    )
    parser.add_argument(
        "--drop-scene-asset",
        action="append",
        default=[],
        help=(
            "drop a complete marker window when capture_start reports this "
            "asset name; repeatable and safer than leaving an unsupported "
            "visible asset as unlabeled background"
        ),
    )
    parser.add_argument(
        "--distance-filter",
        choices=("box-range", "radial-0-30", "radial-30-plus", "all"),
        default="box-range",
        help=(
            "GT center selection; box-range preserves the production model's "
            "+/-30 m Cartesian contract"
        ),
    )
    parser.add_argument(
        "--distance-threshold-m", type=float, default=30.0,
        help="radial split used by radial-0-30 and radial-30-plus",
    )
    parser.add_argument(
        "--point-cloud-range", type=float, nargs=6,
        default=(-30.0, -30.0, -3.0, 30.0, 30.0, 2.0),
        metavar=("XMIN", "YMIN", "ZMIN", "XMAX", "YMAX", "ZMAX"),
        help="axis-aligned box used by distance-filter=box-range",
    )
    parser.add_argument(
        "--serialized-root", type=Path,
        help="optional root recorded in PKLs instead of the host output root",
    )
    return parser.parse_args()


def serialized_path(host_path, host_root, serialized_root):
    return serialized_root / host_path.relative_to(host_root)


def write_camera(path, message):
    payload = bytes(message.data)
    if not payload.startswith(b"\xff\xd8"):
        raise ValueError("expected JPEG CompressedImage for {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def keep_box_for_distance(box, mode, threshold_m, point_cloud_range):
    if mode == "all":
        return True
    if mode == "box-range":
        minimum = np.asarray(point_cloud_range[:3], dtype=np.float64)
        maximum = np.asarray(point_cloud_range[3:], dtype=np.float64)
        center = np.asarray(box[:3], dtype=np.float64)
        return bool(np.all(center >= minimum) and np.all(center <= maximum))
    distance = math.hypot(float(box[0]), float(box[1]))
    if mode == "radial-0-30":
        return distance < threshold_m
    if mode == "radial-30-plus":
        return distance >= threshold_m
    raise ValueError("unknown distance filter {}".format(mode))


def main():
    args = arguments()
    output = args.output_root.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    stored_root = args.serialized_root or output
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    catalog = json.loads(args.asset_offsets.read_text(encoding="utf-8"))["assets"]
    label_index = {name: index for index, name in enumerate(CLASSES)}
    split_rows = {name: [] for name in ("train", "val", "test")}
    kept = Counter()
    rejected = Counter()
    rejected_distance = Counter()
    rejected_empty_positive = Counter()
    dropped_scene_assets = Counter()
    dropped_scenes = []
    scene_count = Counter()
    synchronization = []
    unknown_assets = set()
    deskew_rejected = Counter()
    deskew_displacement_p95 = []
    deskew_runtime_ms = []

    # Pass 1 keeps only tiny scene markers.  Sensor payloads are streamed one
    # capture window at a time in pass 2 so 20+ GiB split bags do not exhaust
    # host memory.
    markers = []
    for bag_path in args.bags:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for _topic, message, bag_time in bag.read_messages(topics=[MARKER_TOPIC]):
                markers.append((bag_time.to_sec(), json.loads(message.data)))
    windows, marker_errors = build_capture_windows(markers)
    if marker_errors:
        raise ValueError("invalid capture markers: {}".format(marker_errors))
    starts = {}
    for _stamp, payload in sorted(markers, key=lambda item: item[0]):
        if payload.get("event") in ("capture_start", "sequence_start"):
            # Prefer the later sequence_start: it contains the actor state at
            # the first valid frame rather than the pre-warmup ScenarioLoad
            # state.
            starts[payload["scene_id"]] = payload

    sensor_topics = (LIDAR_TOPIC, EGO_TOPIC, OBJECT_TOPIC, *CAMERAS.values())

    def convert_window(window, topics):
        marker = starts[window["scene_id"]]
        present_assets = {
            str(actor.get("name"))
            for rows in marker.get("actual_objects", {}).get("identities", {}).values()
            for actor in rows
            if actor.get("name") is not None
        }
        blocked_assets = sorted(present_assets.intersection(args.drop_scene_asset))
        if blocked_assets:
            dropped_scenes.append({
                "scene_id": window["scene_id"],
                "assets": blocked_assets,
            })
            dropped_scene_assets.update(blocked_assets)
            return
        split = str(marker.get("split") or "train")
        if split not in split_rows:
            raise ValueError("unknown split {} for {}".format(split, window["scene_id"]))
        scene_count[split] += 1
        missing = [topic for topic in sensor_topics if not topics[topic]]
        if missing:
            raise ValueError(
                "capture window {} has no messages for {}".format(
                    window["scene_id"], ", ".join(missing)
                )
            )
        lidar_rows = topics[LIDAR_TOPIC]
        aligned = {
            topic: sorted(topics[topic], key=lambda row: row[0])
            for topic in (EGO_TOPIC, OBJECT_TOPIC, *CAMERAS.values())
        }
        ego_pose_samples = np.asarray(
            [
                (
                    row[0],
                    float(row[2].position.x),
                    float(row[2].position.y),
                    float(row[2].position.z),
                    math.radians(float(row[2].heading)),
                )
                for row in aligned[EGO_TOPIC]
            ],
            dtype=np.float64,
        )
        ego_pose_samples[:, 4] = np.unwrap(ego_pose_samples[:, 4])
        lidar_to_ego = np.asarray(
            calibration["lidar"]["lidar_to_ego"], dtype=np.float64
        )
        lidar_position = tuple(lidar_to_ego[:3, 3])
        lidar_yaw = math.atan2(lidar_to_ego[1, 0], lidar_to_ego[0, 0])
        for ordinal, lidar_index in enumerate(
            selected_indices(len(lidar_rows), args.samples_per_scene)
        ):
            lidar_stamp, _record_time, lidar_message = lidar_rows[lidar_index]
            points = cloud_xyz_i(lidar_message)
            target_stamp = lidar_stamp
            deskew = {
                "mode": args.lidar_deskew,
                "reference": "header",
                "reference_offset_sec": 0.0,
                "span_sec": 0.0,
                "displacement_p95_m": 0.0,
            }
            if args.lidar_deskew == "ego-pose-2d":
                try:
                    offsets = point_field_array(lidar_message, "time")
                except ValueError:
                    offsets = np.empty(0, dtype=np.float64)
                contract = scan_time_contract(
                    offsets,
                    args.deskew_max_point_time_span_sec,
                    args.lidar_deskew_reference,
                )
                if contract is None:
                    deskew_rejected["invalid_point_time"] += 1
                    if args.deskew_invalid_policy == "error":
                        raise ValueError(
                            "invalid point time at {}".format(lidar_stamp)
                        )
                    continue
                started = time.perf_counter()
                corrected = deskew_points_to_reference_pose(
                    points,
                    offsets,
                    lidar_stamp,
                    contract["reference_offset_sec"],
                    ego_pose_samples,
                    lidar_position=lidar_position,
                    lidar_yaw=lidar_yaw,
                    maximum_extrapolation_sec=args.max_extrapolation_sec,
                )
                deskew_runtime_ms.append((time.perf_counter() - started) * 1000.0)
                if corrected is None:
                    deskew_rejected["pose_coverage"] += 1
                    if args.deskew_invalid_policy == "error":
                        raise ValueError(
                            "deskew pose coverage failed at {}".format(lidar_stamp)
                        )
                    continue
                points, displacement, _reference_pose = corrected
                target_stamp = lidar_stamp + contract["reference_offset_sec"]
                displacement_p95 = float(np.percentile(displacement, 95.0))
                deskew_displacement_p95.append(displacement_p95)
                deskew = {
                    "mode": args.lidar_deskew,
                    "reference": contract["reference"],
                    "reference_offset_sec": contract["reference_offset_sec"],
                    "span_sec": contract["span_sec"],
                    "displacement_p95_m": displacement_p95,
                }

            ego_row = nearest(target_stamp, aligned[EGO_TOPIC])
            object_row = nearest(target_stamp, aligned[OBJECT_TOPIC])
            ego_age = abs(ego_row[0] - target_stamp)
            object_age = abs(object_row[0] - target_stamp)
            if max(ego_age, object_age) > args.max_state_age_sec:
                raise ValueError("state age limit exceeded at {}".format(target_stamp))
            ego_state, ego_alignment = align_ego_state(
                target_stamp, aligned[EGO_TOPIC], args.state_alignment,
                args.max_extrapolation_sec,
            )
            object_state, object_alignment = align_object_state(
                target_stamp, aligned[OBJECT_TOPIC], args.state_alignment,
                args.max_extrapolation_sec,
            )

            sample_id = "{}_f{:03d}".format(window["scene_id"], ordinal)
            sample_dir = output / "samples" / window["scene_id"] / sample_id
            sample_dir.mkdir(parents=True, exist_ok=True)
            lidar_path = sample_dir / "lidar.bin"

            instances = []
            for class_name, actor in actors(object_state):
                _naive, box = corrected_box(
                    ego_state, class_name, actor, calibration, catalog
                )
                if box is None:
                    unknown_assets.add(str(actor.name))
                    continue
                if not keep_box_for_distance(
                    box, args.distance_filter, args.distance_threshold_m,
                    args.point_cloud_range,
                ):
                    rejected_distance[class_name] += 1
                    continue
                count, _mask = points_in_box(points, box)
                if count < args.minimum_gt_points:
                    rejected[class_name] += 1
                    continue
                kept[class_name] += 1
                instances.append(
                    {
                        "bbox_3d": box.tolist(),
                        "bbox_label_3d": label_index[class_name],
                        "num_lidar_pts": count,
                        "bbox_3d_isvalid": True,
                        "instance_id": int(actor.unique_id),
                        "asset_name": str(actor.name),
                    }
                )

            if args.require_positive_class is not None:
                required_label = label_index[args.require_positive_class]
                if not any(
                    instance["bbox_label_3d"] == required_label
                    for instance in instances
                ):
                    rejected_empty_positive[split] += 1
                    try:
                        sample_dir.rmdir()
                    except OSError:
                        pass
                    continue

            points.astype(np.float32).tofile(str(lidar_path))

            images = {}
            camera_ages = {}
            for camera_name in CAMERA_ORDER:
                topic = CAMERAS[camera_name]
                camera_row = nearest(target_stamp, aligned[topic])
                age = abs(camera_row[0] - target_stamp)
                if age > args.max_camera_age_sec:
                    raise ValueError(
                        "camera age limit exceeded at {} {}".format(
                            target_stamp, camera_name
                        )
                    )
                camera_ages[camera_name] = age
                image_path = sample_dir / (camera_name + ".jpg")
                write_camera(image_path, camera_row[2])
                camera = calibration["cameras"][CALIBRATION_CAMERA_NAMES[camera_name]]
                intrinsic = np.asarray(camera["intrinsic"], dtype=np.float64)
                lidar2cam = (
                    np.asarray(camera["ego_to_sensor"], dtype=np.float64)
                    @ lidar_to_ego
                )
                images[camera_name] = {
                    "img_path": str(serialized_path(image_path, output, stored_root)),
                    "height": int(camera["resolution_px"][1]),
                    "width": int(camera["resolution_px"][0]),
                    "cam2img": intrinsic.tolist(),
                    "lidar2cam": lidar2cam.tolist(),
                    "lidar2img": (intrinsic @ lidar2cam[:3, :]).tolist(),
                    "cam2lidar": np.linalg.inv(lidar2cam).tolist(),
                }
            split_rows[split].append(
                {
                    "sample_idx": sample_id,
                    "token": sample_id,
                    "scene_id": window["scene_id"],
                    "timestamp": target_stamp,
                    "source_lidar_header_timestamp": lidar_stamp,
                    "lidar_deskew": deskew,
                    "lidar_points": {
                        "lidar_path": str(
                            serialized_path(lidar_path, output, stored_root)
                        ),
                        "num_pts_feats": 4,
                        "lidar2ego": calibration["lidar"]["lidar_to_ego"],
                    },
                    "images": images,
                    "instances": instances,
                }
            )
            synchronization.append(
                {
                    "sample_idx": sample_id,
                    "ego_age_ms": ego_age * 1000.0,
                    "object_age_ms": object_age * 1000.0,
                    "ego_alignment": ego_alignment,
                    "object_alignment": object_alignment,
                    "camera_age_ms": {
                        key: value * 1000.0 for key, value in camera_ages.items()
                    },
                    "lidar_deskew": deskew,
                }
            )

    topic_rows = {topic: [] for topic in sensor_topics}
    window_index = 0

    def finish_current_window():
        nonlocal topic_rows, window_index
        convert_window(windows[window_index], topic_rows)
        topic_rows = {topic: [] for topic in sensor_topics}
        window_index += 1

    for bag_path in args.bags:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(topics=sensor_topics):
                if window_index >= len(windows):
                    break
                record_time = bag_time.to_sec()
                while (
                    window_index < len(windows)
                    and record_time > windows[window_index]["end"]
                ):
                    finish_current_window()
                if window_index >= len(windows):
                    break
                window = windows[window_index]
                if window["start"] <= record_time <= window["end"]:
                    topic_rows[topic].append(
                        (message_stamp(message, bag_time), record_time, message)
                    )
    while window_index < len(windows):
        finish_current_window()

    if unknown_assets:
        raise ValueError("missing asset offsets: {}".format(sorted(unknown_assets)))
    metainfo = {
        "dataset": "morai_base3_online",
        "classes": CLASSES,
        "minimum_gt_points": args.minimum_gt_points,
        "coordinate_frame": "lidar",
        "state_alignment": args.state_alignment,
        "max_extrapolation_sec": args.max_extrapolation_sec,
        "distance_filter": args.distance_filter,
        "distance_threshold_m": args.distance_threshold_m,
        "point_cloud_range": list(args.point_cloud_range),
        "drop_scene_assets": sorted(set(args.drop_scene_asset)),
        "lidar_deskew": args.lidar_deskew,
        "lidar_deskew_reference": args.lidar_deskew_reference,
        "deskew_max_point_time_span_sec": args.deskew_max_point_time_span_sec,
        "deskew_invalid_policy": args.deskew_invalid_policy,
    }
    for split, rows in split_rows.items():
        with (output / "morai_base3_infos_{}.pkl".format(split)).open("wb") as stream:
            pickle.dump({"metainfo": metainfo, "data_list": rows}, stream, protocol=4)
    report = {
        "schema_version": 2,
        "bags": [str(path.resolve()) for path in args.bags],
        "scenes": dict(scene_count),
        "frames": {key: len(value) for key, value in split_rows.items()},
        "kept_gt": dict(kept),
        "rejected_sparse_gt": dict(rejected),
        "rejected_distance_gt": dict(rejected_distance),
        "rejected_empty_positive_frames": dict(rejected_empty_positive),
        "required_positive_class": args.require_positive_class,
        "dropped_scene_assets": dict(dropped_scene_assets),
        "dropped_scenes": dropped_scenes,
        "unknown_assets": sorted(unknown_assets),
        "deskew_rejected": dict(deskew_rejected),
        "deskew_displacement_p95_m": {
            "mean": float(np.mean(deskew_displacement_p95))
            if deskew_displacement_p95 else None,
            "p95": float(np.percentile(deskew_displacement_p95, 95.0))
            if deskew_displacement_p95 else None,
        },
        "deskew_runtime_ms": {
            "mean": float(np.mean(deskew_runtime_ms)) if deskew_runtime_ms else None,
            "p95": float(np.percentile(deskew_runtime_ms, 95.0))
            if deskew_runtime_ms else None,
        },
        "synchronization": synchronization,
        "host_output_root": str(output),
        "serialized_root": str(stored_root),
    }
    (output / "conversion_audit.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        key: report[key]
        for key in (
            "scenes", "frames", "kept_gt", "rejected_sparse_gt",
            "rejected_distance_gt",
            "rejected_empty_positive_frames", "required_positive_class",
            "dropped_scene_assets", "dropped_scenes", "unknown_assets",
        )
    }, indent=2))


if __name__ == "__main__":
    main()
