#!/usr/bin/env python3
"""Audit ROS timestamp propagation and sweep GT state-reference time."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from tracking_prediction_evaluation.pipeline_evaluator import (
    read_detection_frames,
    read_ego_points,
    read_output_frames,
)
from tracking_prediction_evaluation.timestamp_alignment import (
    build_alignment_samples,
    summarize_alignment_by_actor,
    sweep_alignment_shifts,
)
from tracking_prediction_evaluation.static_transform import (
    load_morai_to_map_transform,
)
from tracking_prediction_evaluation.trajectory import (
    read_morai_trajectories,
    transform_segments_planar,
)

RAW_TOPICS = (
    "/sensor/lidar1",
    "/sensor/camera/front/compressed",
    "/sensor/camera/front_left/compressed",
    "/sensor/camera/front_right/compressed",
    "/sensor/camera/back/compressed",
    "/Object_topic",
    "/Ego_topic",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--detection-topic",
        default="/perception/object_recognition/detection/objects",
    )
    parser.add_argument(
        "--tracking-topic",
        default="/perception/object_recognition/tracking/objects",
    )
    parser.add_argument(
        "--prediction-topic",
        default="/perception/object_recognition/objects",
    )
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument(
        "--association-gate-m",
        type=float,
        default=5.0,
        help="Wide fixed-association gate used before the time sweep.",
    )
    parser.add_argument("--sensor-to-ego-x-m", type=float, default=0.855)
    parser.add_argument("--sensor-to-ego-y-m", type=float, default=0.0)
    parser.add_argument("--shift-min-sec", type=float, default=-0.20)
    parser.add_argument("--shift-max-sec", type=float, default=0.20)
    parser.add_argument("--shift-step-sec", type=float, default=0.005)
    parser.add_argument("--maximum-interpolation-bracket-sec", type=float, default=0.15)
    parser.add_argument(
        "--morai-static-tf",
        type=Path,
        default=Path("configs/tf/morai_lanelet_static_tf.yaml"),
    )
    return parser.parse_args()


def _distribution(values):
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if not len(array):
        return {"count": 0}
    return {
        "count": int(len(array)),
        "mean": float(array.mean()),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "minimum": float(array.min()),
        "maximum": float(array.max()),
    }


def _header_stamp(message, bag_timestamp):
    if hasattr(message, "header"):
        stamp = message.header.stamp
        if stamp.secs != 0 or stamp.nsecs != 0:
            return float(stamp.to_sec())
    return float(bag_timestamp.to_sec())


def read_topic_timing(bag_path, topics):
    import rosbag

    stamps = {topic: [] for topic in topics}
    arrival_minus_header = {topic: [] for topic in topics}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, bag_timestamp in bag.read_messages(topics=list(topics)):
            stamp = _header_stamp(message, bag_timestamp)
            stamps[topic].append(stamp)
            arrival_minus_header[topic].append(bag_timestamp.to_sec() - stamp)
    output = {}
    for topic in topics:
        values = np.asarray(stamps[topic], dtype=np.float64)
        output[topic] = {
            "header_stamp_count": int(len(values)),
            "bag_time_minus_header_sec": _distribution(
                arrival_minus_header[topic]
            ),
            "header_period_sec": _distribution(np.diff(values)),
        }
    return output, stamps


def read_lidar_point_time_offsets(bag_path, topic="/sensor/lidar1", limit=32):
    """Summarize MORAI's per-return time relative to the cloud header.

    Sensor_6 clouds include a float32 ``time`` field.  Negative values ending
    near zero mean the header denotes scan end; this must not be confused with
    detector runtime latency or an Object_topic reference-point offset.
    """
    import rosbag

    minima = []
    medians = []
    maxima = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, _ in bag.read_messages(topics=[topic]):
            field = next(
                (candidate for candidate in message.fields if candidate.name == "time"),
                None,
            )
            if field is None or field.datatype not in (7, 8):
                continue
            endian = ">" if message.is_bigendian else "<"
            scalar = "f4" if field.datatype == 7 else "f8"
            dtype = np.dtype(
                {
                    "names": ["time"],
                    "formats": [endian + scalar],
                    "offsets": [int(field.offset)],
                    "itemsize": int(message.point_step),
                }
            )
            count = int(message.width) * int(message.height)
            values = np.frombuffer(message.data, dtype=dtype, count=count)["time"]
            values = values[np.isfinite(values)]
            if len(values):
                minima.append(float(values.min()))
                medians.append(float(np.median(values)))
                maxima.append(float(values.max()))
            if len(minima) >= limit:
                break
    return {
        "sampled_clouds": len(minima),
        "minimum_offset_sec": _distribution(minima),
        "median_offset_sec": _distribution(medians),
        "maximum_offset_sec": _distribution(maxima),
    }


def nearest_stamp_delta(reference, candidates):
    reference = np.asarray(reference, dtype=np.float64)
    candidates = np.asarray(candidates, dtype=np.float64)
    if not len(reference) or not len(candidates):
        return {"count": 0}
    indexes = np.searchsorted(candidates, reference)
    deltas = []
    for value, index in zip(reference, indexes):
        choices = []
        if index < len(candidates):
            choices.append(candidates[index] - value)
        if index > 0:
            choices.append(candidates[index - 1] - value)
        deltas.append(min(choices, key=abs))
    return _distribution(deltas)


def compact_best(sweep):
    output = {}
    for key, row in sweep["best"].items():
        output[key] = {
            "shift_sec": row["shift_sec"],
            "matched_samples": row["matched_samples"],
            "euclidean_mean_m": row["euclidean_error_m"].get("mean"),
            "euclidean_p50_m": row["euclidean_error_m"].get("p50"),
            "longitudinal_mean_m": row["signed_longitudinal_error_m"].get("mean"),
            "longitudinal_abs_mean_m": row["signed_longitudinal_error_m"].get(
                "absolute_mean"
            ),
            "lateral_mean_m": row["signed_lateral_error_m"].get("mean"),
            "lateral_abs_mean_m": row["signed_lateral_error_m"].get(
                "absolute_mean"
            ),
        }
    return output


def write_sweep_csv(path, stage_sweeps):
    fieldnames = (
        "stage",
        "shift_sec",
        "matched_samples",
        "euclidean_mean_m",
        "euclidean_rmse_m",
        "longitudinal_mean_m",
        "longitudinal_abs_mean_m",
        "lateral_mean_m",
        "lateral_abs_mean_m",
    )
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for stage, result in stage_sweeps.items():
            for row in result["sweep"]:
                writer.writerow(
                    {
                        "stage": stage,
                        "shift_sec": row["shift_sec"],
                        "matched_samples": row["matched_samples"],
                        "euclidean_mean_m": row["euclidean_error_m"].get("mean"),
                        "euclidean_rmse_m": row["euclidean_error_m"].get("rmse"),
                        "longitudinal_mean_m": row[
                            "signed_longitudinal_error_m"
                        ].get("mean"),
                        "longitudinal_abs_mean_m": row[
                            "signed_longitudinal_error_m"
                        ].get("absolute_mean"),
                        "lateral_mean_m": row["signed_lateral_error_m"].get("mean"),
                        "lateral_abs_mean_m": row[
                            "signed_lateral_error_m"
                        ].get("absolute_mean"),
                    }
                )


def main():
    args = parse_args()
    if args.shift_step_sec <= 0.0 or args.shift_max_sec < args.shift_min_sec:
        raise ValueError("invalid time-shift sweep range")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    translation_xyz, transform_yaw = load_morai_to_map_transform(
        args.morai_static_tf
    )
    segments = read_morai_trajectories(
        args.raw_bag,
        topic=args.object_topic,
        include_classes=("vehicle",),
    )
    segments = transform_segments_planar(segments, translation_xyz, transform_yaw)
    ego_points = read_ego_points(
        args.raw_bag,
        args.ego_topic,
        translation_xyz,
        transform_yaw,
    )
    tracking_frames, prediction_frames = read_output_frames(
        args.output_bag,
        args.tracking_topic,
        args.prediction_topic,
    )
    detection_frames = read_detection_frames(args.output_bag, args.detection_topic)

    stage_inputs = {
        "detection": (detection_frames, True),
        "tracking": (tracking_frames, False),
        "prediction_origin": (prediction_frames, False),
    }
    shifts = np.arange(
        args.shift_min_sec,
        args.shift_max_sec + args.shift_step_sec * 0.5,
        args.shift_step_sec,
    )
    stage_sweeps = {}
    stage_counts = {}
    stage_actors = {}
    for stage, (frames, is_sensor_frame) in stage_inputs.items():
        samples, counts = build_alignment_samples(
            segments,
            ego_points,
            frames,
            estimates_are_sensor_frame=is_sensor_frame,
            sensor_to_ego_x_m=args.sensor_to_ego_x_m,
            sensor_to_ego_y_m=args.sensor_to_ego_y_m,
            maximum_range_m=args.maximum_range_m,
            maximum_association_distance_m=args.association_gate_m,
            maximum_interpolation_bracket_sec=(
                args.maximum_interpolation_bracket_sec
            ),
        )
        stage_counts[stage] = counts
        stage_sweeps[stage] = sweep_alignment_shifts(
            samples,
            shifts,
            args.maximum_interpolation_bracket_sec,
            ego_points,
        )
        stage_actors[stage] = summarize_alignment_by_actor(
            samples,
            shifts,
            args.maximum_interpolation_bracket_sec,
            ego_points=ego_points,
        )

    output_topics = (
        args.detection_topic,
        args.tracking_topic,
        args.prediction_topic,
    )
    raw_timing, raw_stamps = read_topic_timing(args.raw_bag, RAW_TOPICS)
    output_timing, output_stamps = read_topic_timing(
        args.output_bag,
        output_topics,
    )
    propagation = {
        "detection_minus_nearest_lidar_header_sec": nearest_stamp_delta(
            output_stamps[args.detection_topic],
            raw_stamps["/sensor/lidar1"],
        ),
        "tracking_minus_nearest_detection_header_sec": nearest_stamp_delta(
            output_stamps[args.tracking_topic],
            output_stamps[args.detection_topic],
        ),
        "prediction_minus_nearest_tracking_header_sec": nearest_stamp_delta(
            output_stamps[args.prediction_topic],
            output_stamps[args.tracking_topic],
        ),
    }

    zero_rows = {}
    for stage, result in stage_sweeps.items():
        zero_rows[stage] = min(
            result["sweep"], key=lambda row: abs(row["shift_sec"])
        )
    summary = {
        "schema_version": 2,
        "source": {
            "raw_bag": str(args.raw_bag.resolve()),
            "output_bag": str(args.output_bag.resolve()),
            "morai_static_tf": str(args.morai_static_tf.resolve()),
            "trajectory_segment_count": len(segments),
            "ego_sample_count": len(ego_points),
        },
        "settings": {
            "shift_min_sec": args.shift_min_sec,
            "shift_max_sec": args.shift_max_sec,
            "shift_step_sec": args.shift_step_sec,
            "maximum_range_m": args.maximum_range_m,
            "association_gate_m": args.association_gate_m,
            "sensor_to_ego_x_m": args.sensor_to_ego_x_m,
            "sensor_to_ego_y_m": args.sensor_to_ego_y_m,
        },
        "timestamp_chain": {
            "raw_topics": raw_timing,
            "output_topics": output_timing,
            "header_propagation": propagation,
            "lidar_point_time_offsets": read_lidar_point_time_offsets(
                args.raw_bag
            ),
        },
        "sample_counts": stage_counts,
        "shift_sweep_sample_counts": {
            stage: {
                "input": result["input_sample_count"],
                "common": result["common_sample_count"],
            }
            for stage, result in stage_sweeps.items()
        },
        "zero_shift": zero_rows,
        "best_shift": {
            stage: compact_best(result) for stage, result in stage_sweeps.items()
        },
        "per_actor": stage_actors,
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_sweep_csv(args.output_dir / "shift_sweep.csv", stage_sweeps)

    compact = {
        "summary": str(summary_path.resolve()),
        "header_propagation": propagation,
        "sample_counts": stage_counts,
        "best_shift": summary["best_shift"],
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
