#!/usr/bin/env python3
"""Audit rolling-LiDAR point time against PointPainting center errors."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from tracking_prediction_evaluation.bbox_reference import (
    transform_segments_to_calibrated_bbox_center,
)
from tracking_prediction_evaluation.lanelet_bias import signed_statistics
from tracking_prediction_evaluation.pipeline_evaluator import (
    VEHICLE_LABELS,
    _signed_position_error,
    active_gt,
    dominant_label,
    interpolate_ego,
    match_estimates,
    quaternion_yaw,
    read_detection_frames,
    read_ego_points,
    statistics,
    transform_sensor_estimates_to_map,
)
from tracking_prediction_evaluation.static_transform import (
    load_morai_to_map_transform,
)
from tracking_prediction_evaluation.trajectory import (
    interpolate_point,
    read_morai_trajectories,
    transform_segments_planar,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--lidar-topic", default="/sensor/lidar1")
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--maximum-association-distance-m", type=float, default=3.0)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.2)
    parser.add_argument("--sensor-to-ego-x-m", type=float, default=0.855)
    parser.add_argument("--sensor-to-ego-y-m", type=float, default=0.0)
    parser.add_argument("--box-half-extent-scale", type=float, default=0.65)
    parser.add_argument("--minimum-box-points", type=int, default=3)
    return parser.parse_args()


def point_fields(message):
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "time")
    missing = [name for name in required if name not in fields]
    if missing:
        raise RuntimeError(
            "PointCloud2 lacks rolling-scan fields {} (available={})".format(
                missing, sorted(fields)
            )
        )
    endian = ">" if message.is_bigendian else "<"
    formats = []
    offsets = []
    for name in required:
        field = fields[name]
        if field.datatype != field.FLOAT32:
            raise RuntimeError("{} must be FLOAT32".format(name))
        formats.append(endian + "f4")
        offsets.append(int(field.offset))
    dtype = np.dtype(
        {
            "names": list(required),
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(message.point_step),
        }
    )
    return np.frombuffer(
        message.data,
        dtype=dtype,
        count=int(message.width) * int(message.height),
    )


def correlation(left, right):
    if len(left) < 2:
        return None
    return float(np.corrcoef(left, right)[0, 1])


def main():
    import rosbag

    args = parse_args()
    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    actor_segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle",),
    )
    bbox_segments, calibration = transform_segments_to_calibrated_bbox_center(
        actor_segments
    )
    segments = transform_segments_planar(
        bbox_segments, translation_xyz, transform_yaw
    )
    ego_points = read_ego_points(
        args.raw_bag,
        args.ego_topic,
        translation_xyz,
        transform_yaw,
    )
    detection_frames = read_detection_frames(
        args.output_bag, args.detection_topic
    )

    clouds = {}
    lidar_stamps = []
    scan_min_offsets = []
    scan_median_offsets = []
    scan_max_offsets = []
    scan_spans = []
    with rosbag.Bag(str(args.raw_bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[args.lidar_topic]):
            points = point_fields(message)
            valid_time = np.asarray(points["time"], dtype=np.float64)
            valid_time = valid_time[np.isfinite(valid_time)]
            if not valid_time.size:
                continue
            stamp_nsec = message.header.stamp.to_nsec()
            clouds[stamp_nsec] = message
            lidar_stamps.append(message.header.stamp.to_sec())
            scan_min_offsets.append(float(valid_time.min()))
            scan_median_offsets.append(float(np.median(valid_time)))
            scan_max_offsets.append(float(valid_time.max()))
            scan_spans.append(float(valid_time.max() - valid_time.min()))

    current_distance = []
    acquisition_aligned_distance = []
    acquisition_ego_against_header_gt_distance = []
    longitudinal_errors = []
    lateral_errors = []
    object_point_time_offsets = []
    object_point_counts = []
    matched_count = 0
    usable_point_time_count = 0
    with rosbag.Bag(str(args.output_bag), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[args.detection_topic]):
            timestamp = message.header.stamp.to_sec()
            ego = interpolate_ego(
                ego_points,
                timestamp,
                args.maximum_interpolation_bracket_sec,
            )
            local_estimates = detection_frames.get(timestamp)
            cloud = clouds.get(message.header.stamp.to_nsec())
            if ego is None or local_estimates is None or cloud is None:
                continue
            gt_items = active_gt(
                segments,
                timestamp,
                args.maximum_interpolation_bracket_sec,
                ego,
                args.maximum_range_m,
            )
            world_estimates = transform_sensor_estimates_to_map(
                local_estimates,
                ego,
                args.sensor_to_ego_x_m,
                args.sensor_to_ego_y_m,
            )
            matches, _, _ = match_estimates(
                gt_items,
                world_estimates,
                args.maximum_association_distance_m,
            )
            vehicle_objects = [
                obj
                for obj in message.objects
                if dominant_label(obj) in VEHICLE_LABELS
            ]
            points = point_fields(cloud)
            for gt_index, estimate_index, distance in matches:
                matched_count += 1
                if estimate_index >= len(vehicle_objects):
                    continue
                obj = vehicle_objects[estimate_index]
                pose = obj.kinematics.pose_with_covariance.pose
                box_x = float(pose.position.x)
                box_y = float(pose.position.y)
                box_yaw = quaternion_yaw(pose.orientation)
                length = max(0.4, float(obj.shape.dimensions.x))
                width = max(0.4, float(obj.shape.dimensions.y))
                cosine = math.cos(box_yaw)
                sine = math.sin(box_yaw)
                dx = points["x"] - box_x
                dy = points["y"] - box_y
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                mask = (
                    (np.abs(local_x) <= length * args.box_half_extent_scale)
                    & (np.abs(local_y) <= width * args.box_half_extent_scale)
                    & np.isfinite(points["time"])
                )
                point_count = int(mask.sum())
                if point_count < args.minimum_box_points:
                    continue
                usable_point_time_count += 1
                point_offset = float(np.median(points["time"][mask]))
                adjusted_timestamp = timestamp + point_offset
                adjusted_ego = interpolate_ego(
                    ego_points,
                    adjusted_timestamp,
                    args.maximum_interpolation_bracket_sec,
                )
                adjusted_gt = interpolate_point(
                    gt_items[gt_index][0].points,
                    adjusted_timestamp,
                    args.maximum_interpolation_bracket_sec,
                )
                if adjusted_ego is None or adjusted_gt is None:
                    continue
                adjusted_estimate = transform_sensor_estimates_to_map(
                    (local_estimates[estimate_index],),
                    adjusted_ego,
                    args.sensor_to_ego_x_m,
                    args.sensor_to_ego_y_m,
                )[0]
                current_gt = gt_items[gt_index][1]
                longitudinal, lateral = _signed_position_error(
                    world_estimates[estimate_index], current_gt
                )
                current_distance.append(float(distance))
                longitudinal_errors.append(longitudinal)
                lateral_errors.append(lateral)
                object_point_time_offsets.append(point_offset)
                object_point_counts.append(point_count)
                acquisition_aligned_distance.append(
                    math.hypot(
                        adjusted_estimate.x - adjusted_gt.x,
                        adjusted_estimate.y - adjusted_gt.y,
                    )
                )
                acquisition_ego_against_header_gt_distance.append(
                    math.hypot(
                        adjusted_estimate.x - current_gt.x,
                        adjusted_estimate.y - current_gt.y,
                    )
                )

    periods = np.diff(np.asarray(lidar_stamps, dtype=np.float64))
    summary = {
        "schema_version": 1,
        "purpose": (
            "Diagnose whether a rolling PointCloud2 scan is being treated as "
            "one instantaneous detector measurement. Acquisition-aligned "
            "metrics are diagnostic and are not current-time production metrics."
        ),
        "scan": {
            "frame_count": len(lidar_stamps),
            "header_period_sec": statistics(periods),
            "point_time_min_offset_sec": signed_statistics(scan_min_offsets),
            "point_time_median_offset_sec": signed_statistics(
                scan_median_offsets
            ),
            "point_time_max_offset_sec": signed_statistics(scan_max_offsets),
            "point_time_span_sec": statistics(scan_spans),
        },
        "matched_detections": matched_count,
        "detections_with_usable_point_time": usable_point_time_count,
        "object_point_time_offset_sec": signed_statistics(
            object_point_time_offsets
        ),
        "object_point_count": statistics(object_point_counts),
        "current_header_time_center_distance_m": statistics(current_distance),
        "current_header_time_signed_longitudinal_error_m": signed_statistics(
            longitudinal_errors
        ),
        "current_header_time_signed_lateral_error_m": signed_statistics(
            lateral_errors
        ),
        "acquisition_time_aligned_center_distance_m": statistics(
            acquisition_aligned_distance
        ),
        "acquisition_pose_against_header_gt_distance_m": statistics(
            acquisition_ego_against_header_gt_distance
        ),
        "longitudinal_error_vs_point_time_correlation": correlation(
            longitudinal_errors, object_point_time_offsets
        ),
        "bbox_reference_calibration": calibration,
        "sources": {
            "raw_bag": str(args.raw_bag.resolve()),
            "output_bag": str(args.output_bag.resolve()),
            "morai_static_tf": str(args.morai_static_tf.resolve()),
        },
        "limitations": [
            "Points are associated using the predicted oriented box enlarged to 1.3x full extent.",
            "A production fix must propagate each object from acquisition time to the common output time.",
            "A fixed global timestamp shift is not inferred from this audit.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(
        json.dumps(
            {
                "output": str(args.output),
                "matched_detections": matched_count,
                "usable_point_time": usable_point_time_count,
                "current_p50_m": summary[
                    "current_header_time_center_distance_m"
                ].get("p50"),
                "acquisition_aligned_p50_m": summary[
                    "acquisition_time_aligned_center_distance_m"
                ].get("p50"),
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
