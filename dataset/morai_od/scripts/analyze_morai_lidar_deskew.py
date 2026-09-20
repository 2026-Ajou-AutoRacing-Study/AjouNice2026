#!/usr/bin/env python3
"""Audit whether MORAI LiDAR point ``time`` improves static-world alignment.

The tool transforms consecutive clouds into the MORAI map frame with four
different timing assumptions and compares non-ground, non-actor structure by
nearest-neighbour distance.  It deliberately uses interpolated Ego poses from
the bag instead of the production constant-twist corrector, so the experiment
tests the point-time contract rather than a motion-model approximation.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
import statistics
import struct

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from scipy.spatial import cKDTree


LIDAR_TOPIC = "/sensor/lidar1"
EGO_TOPIC = "/Ego_topic"
OBJECT_TOPIC = "/Object_topic"
LIDAR_TO_EGO_X_M = 0.855
LIDAR_TO_EGO_Y_M = 0.0


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--plot-dir", type=Path)
    parser.add_argument("--max-bags", type=int, default=32)
    parser.add_argument("--max-scans-per-bag", type=int, default=48)
    parser.add_argument("--max-valid-point-time-sec", type=float, default=0.15)
    parser.add_argument("--voxel-size-m", type=float, default=0.20)
    parser.add_argument("--minimum-range-m", type=float, default=6.0)
    parser.add_argument("--maximum-range-m", type=float, default=50.0)
    parser.add_argument("--minimum-static-z-m", type=float, default=-1.20)
    parser.add_argument("--maximum-static-z-m", type=float, default=4.0)
    parser.add_argument("--maximum-pair-gap-sec", type=float, default=0.20)
    return parser.parse_args()


def message_stamp(message, bag_stamp):
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is not None and not stamp.is_zero():
        return float(stamp.to_sec())
    return float(bag_stamp.to_sec())


def point_field_array(message, name):
    field = next((item for item in message.fields if item.name == name), None)
    if field is None:
        raise ValueError("cloud has no {!r} field".format(name))
    formats = {
        1: "i1", 2: "u1", 3: "<i2", 4: "<u2",
        5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8",
    }
    if field.datatype not in formats or field.count != 1:
        raise ValueError(
            "unsupported field {} datatype={} count={}".format(
                name, field.datatype, field.count
            )
        )
    count = int(message.width) * int(message.height)
    return np.ndarray(
        shape=(count,), dtype=np.dtype(formats[field.datatype]),
        buffer=message.data, offset=int(field.offset),
        strides=(int(message.point_step),),
    ).astype(np.float64, copy=True)


def cloud_points(message):
    return np.column_stack(
        [point_field_array(message, name) for name in ("x", "y", "z", "time")]
    )


def unwrap_degrees(values):
    return np.unwrap(np.radians(np.asarray(values, dtype=np.float64)))


class PoseSeries:
    def __init__(self, rows):
        if len(rows) < 2:
            raise ValueError("at least two Ego samples are required")
        rows = sorted(rows, key=lambda row: row[0])
        self.stamps = np.asarray([row[0] for row in rows], dtype=np.float64)
        self.x = np.asarray([row[1] for row in rows], dtype=np.float64)
        self.y = np.asarray([row[2] for row in rows], dtype=np.float64)
        self.yaw = unwrap_degrees([row[3] for row in rows])

    def at(self, stamps):
        stamps = np.asarray(stamps, dtype=np.float64)
        if stamps.min() < self.stamps[0] or stamps.max() > self.stamps[-1]:
            raise ValueError("point time lies outside Ego pose support")
        return (
            np.interp(stamps, self.stamps, self.x),
            np.interp(stamps, self.stamps, self.y),
            np.interp(stamps, self.stamps, self.yaw),
        )


def nearest_message(stamp, rows):
    stamps = [row[0] for row in rows]
    index = bisect.bisect_left(stamps, stamp)
    choices = []
    if index < len(rows):
        choices.append(rows[index])
    if index:
        choices.append(rows[index - 1])
    return min(choices, key=lambda row: abs(row[0] - stamp))[1]


def actor_iter(message):
    for field in ("npc_list", "pedestrian_list", "obstacle_list"):
        for actor in getattr(message, field, []):
            yield actor


def remove_actor_neighbourhood(points, ego_pose, objects):
    """Remove broad actor discs so moving objects do not decide the A/B."""
    if objects is None:
        return points
    ego_x, ego_y, ego_yaw = ego_pose
    cosine, sine = math.cos(ego_yaw), math.sin(ego_yaw)
    point_x_ego = points[:, 0] + LIDAR_TO_EGO_X_M
    point_y_ego = points[:, 1] + LIDAR_TO_EGO_Y_M
    keep = np.ones(len(points), dtype=bool)
    for actor in actor_iter(objects):
        dx = float(actor.position.x) - ego_x
        dy = float(actor.position.y) - ego_y
        actor_x = cosine * dx + sine * dy
        actor_y = -sine * dx + cosine * dy
        radius = 1.0 + 0.5 * math.hypot(
            max(0.5, float(actor.size.x)), max(0.5, float(actor.size.y))
        )
        keep &= (
            (point_x_ego - actor_x) ** 2 + (point_y_ego - actor_y) ** 2
            > radius ** 2
        )
    return points[keep]


def timing_stamps(header_stamp, point_times, mode):
    span = float(np.max(point_times))
    if mode == "zero_time":
        return np.full(len(point_times), header_stamp)
    if mode == "mid_time":
        return np.full(len(point_times), header_stamp + 0.5 * span)
    if mode == "end_time":
        return np.full(len(point_times), header_stamp + span)
    if mode == "point_time":
        return header_stamp + point_times
    if mode == "reverse_point_time":
        return header_stamp + span - point_times
    raise ValueError(mode)


def transform_to_map(points, header_stamp, poses, mode):
    stamps = timing_stamps(header_stamp, points[:, 3], mode)
    x, y, yaw = poses.at(stamps)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    local_x = points[:, 0] + LIDAR_TO_EGO_X_M
    local_y = points[:, 1] + LIDAR_TO_EGO_Y_M
    return np.column_stack(
        [
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
            points[:, 2],
        ]
    )


def voxel_downsample(points, size):
    cells = np.floor(points / float(size)).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(indices)]


def pair_metric(first, second):
    forward = cKDTree(first).query(second, k=1)[0]
    reverse = cKDTree(second).query(first, k=1)[0]
    distances = np.concatenate([forward, reverse])
    distances = distances[np.isfinite(distances) & (distances <= 2.0)]
    if len(distances) < 100:
        return None
    return {
        "median_m": float(np.median(distances)),
        "p75_m": float(np.percentile(distances, 75)),
        "p90_m": float(np.percentile(distances, 90)),
        "within_0_25m": float(np.mean(distances <= 0.25)),
        "points": int(len(distances)),
    }


def summarize(values):
    if not values:
        return None
    return {
        "count": len(values),
        "mean": float(np.mean(values)),
        "p50": float(np.median(values)),
        "p75": float(np.percentile(values, 75)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def plot_pair(output, clouds_by_mode, title):
    modes = ("zero_time", "end_time", "point_time")
    figure, axes = plt.subplots(1, len(modes), figsize=(18, 6), constrained_layout=True)
    for axis, mode in zip(axes, modes):
        first, second = clouds_by_mode[mode]
        axis.scatter(first[:, 0], first[:, 1], s=0.3, c="#1f77b4", alpha=0.55)
        axis.scatter(second[:, 0], second[:, 1], s=0.3, c="#ff7f0e", alpha=0.55)
        axis.set_title(mode)
        axis.set_aspect("equal")
        axis.set_xlim(np.median(first[:, 0]) - 45, np.median(first[:, 0]) + 45)
        axis.set_ylim(np.median(first[:, 1]) - 45, np.median(first[:, 1]) + 45)
    figure.suptitle(title)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def analyze_bag(path, args, make_plot=False):
    ego_rows = []
    object_rows = []
    cloud_rows = []
    with rosbag.Bag(str(path)) as bag:
        for topic, message, bag_stamp in bag.read_messages(
            topics=[EGO_TOPIC, OBJECT_TOPIC, LIDAR_TOPIC]
        ):
            stamp = message_stamp(message, bag_stamp)
            if topic == EGO_TOPIC:
                ego_rows.append(
                    (
                        stamp, float(message.position.x), float(message.position.y),
                        float(message.heading),
                    )
                )
            elif topic == OBJECT_TOPIC:
                object_rows.append((stamp, message))
            else:
                cloud_rows.append((stamp, message))
    if len(ego_rows) < 2 or len(cloud_rows) < 2:
        return None
    poses = PoseSeries(ego_rows)
    cloud_rows = cloud_rows[: args.max_scans_per_bag]
    spans = []
    scans = []
    invalid_support = 0
    for header_stamp, message in cloud_rows:
        points = cloud_points(message)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if not len(points):
            continue
        span = float(np.max(points[:, 3]))
        spans.append(span)
        if span > args.max_valid_point_time_sec or span <= 0.0:
            continue
        reference_stamp = header_stamp + span
        try:
            ego_x, ego_y, ego_yaw = [float(item[0]) for item in poses.at([reference_stamp])]
        except ValueError:
            invalid_support += 1
            continue
        ranges = np.hypot(points[:, 0], points[:, 1])
        mask = (
            (ranges >= args.minimum_range_m)
            & (ranges <= args.maximum_range_m)
            & (points[:, 2] >= args.minimum_static_z_m)
            & (points[:, 2] <= args.maximum_static_z_m)
        )
        points = points[mask]
        objects = nearest_message(reference_stamp, object_rows) if object_rows else None
        points = remove_actor_neighbourhood(
            points, (ego_x, ego_y, ego_yaw), objects
        )
        if len(points) < 200:
            continue
        scans.append((header_stamp, span, points))

    modes = (
        "zero_time", "mid_time", "end_time", "point_time",
        "reverse_point_time",
    )
    pair_rows = []
    previous = None
    representative = None
    for header_stamp, span, points in scans:
        transformed = {}
        failed = False
        for mode in modes:
            try:
                transformed[mode] = voxel_downsample(
                    transform_to_map(points, header_stamp, poses, mode),
                    args.voxel_size_m,
                )
            except ValueError:
                failed = True
                break
        if failed:
            invalid_support += 1
            continue
        end_pose = poses.at([header_stamp + span])
        current_pose = np.asarray(
            [end_pose[0][0], end_pose[1][0], end_pose[2][0]], dtype=np.float64
        )
        if previous is not None:
            gap = header_stamp - previous["header_stamp"]
            if 0.0 < gap <= args.maximum_pair_gap_sec:
                yaw_delta = math.atan2(
                    math.sin(current_pose[2] - previous["pose"][2]),
                    math.cos(current_pose[2] - previous["pose"][2]),
                )
                yaw_rate_deg_s = abs(math.degrees(yaw_delta) / gap)
                row = {
                    "gap_sec": float(gap),
                    "yaw_rate_deg_s": float(yaw_rate_deg_s),
                    "slice": "turn" if yaw_rate_deg_s >= 3.0 else "straight",
                    "metrics": {},
                }
                for mode in modes:
                    metric = pair_metric(previous["clouds"][mode], transformed[mode])
                    if metric is not None:
                        row["metrics"][mode] = metric
                if len(row["metrics"]) == len(modes):
                    pair_rows.append(row)
                    if representative is None or yaw_rate_deg_s > representative[0]:
                        representative = (
                            yaw_rate_deg_s,
                            {
                                mode: (previous["clouds"][mode], transformed[mode])
                                for mode in modes
                            },
                        )
        previous = {
            "header_stamp": header_stamp,
            "pose": current_pose,
            "clouds": transformed,
        }

    if make_plot and representative is not None and args.plot_dir:
        plot_pair(
            args.plot_dir / (path.stem + "_static_alignment.png"),
            representative[1],
            "{} | yaw rate {:.2f} deg/s".format(path.name, representative[0]),
        )
    return {
        "bag": str(path.resolve()),
        "clouds": len(cloud_rows),
        "usable_scans": len(scans),
        "invalid_pose_support": invalid_support,
        "point_time_spans_sec": spans,
        "pairs": pair_rows,
    }


def main():
    args = arguments()
    bags = [path for path in args.bags if path.exists()][: args.max_bags]
    if not bags:
        raise SystemExit("no input bags")
    results = []
    for index, path in enumerate(bags):
        print("[{}/{}] {}".format(index + 1, len(bags), path), flush=True)
        result = analyze_bag(path, args, make_plot=index < 3)
        if result is not None:
            results.append(result)

    spans = [value for result in results for value in result["point_time_spans_sec"]]
    modes = (
        "zero_time", "mid_time", "end_time", "point_time",
        "reverse_point_time",
    )
    summary = {}
    for slice_name in ("all", "straight", "turn"):
        summary[slice_name] = {}
        rows = [
            row for result in results for row in result["pairs"]
            if slice_name == "all" or row["slice"] == slice_name
        ]
        for mode in modes:
            summary[slice_name][mode] = {
                key: summarize([row["metrics"][mode][key] for row in rows])
                for key in ("median_m", "p75_m", "p90_m", "within_0_25m")
            }

    output = {
        "schema_version": 1,
        "purpose": "MORAI raw versus point-time Ego deskew static-world alignment",
        "bags": len(results),
        "clouds": sum(result["clouds"] for result in results),
        "usable_scans": sum(result["usable_scans"] for result in results),
        "pairs": sum(len(result["pairs"]) for result in results),
        "point_time_span_sec": summarize(spans),
        "point_time_span_over_limit": sum(
            value > args.max_valid_point_time_sec for value in spans
        ),
        "point_time_span_limit_sec": args.max_valid_point_time_sec,
        "static_alignment": summary,
        "config": {
            "voxel_size_m": args.voxel_size_m,
            "range_m": [args.minimum_range_m, args.maximum_range_m],
            "z_m": [args.minimum_static_z_m, args.maximum_static_z_m],
            "actor_neighbourhood_removed": True,
            "ego_pose": "interpolated /Ego_topic ground truth",
        },
        "per_bag": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print("wrote {}".format(args.output))
    print(
        json.dumps(
            {
                "bags": output["bags"], "clouds": output["clouds"],
                "usable_scans": output["usable_scans"], "pairs": output["pairs"],
                "point_time_span_sec": output["point_time_span_sec"],
                "point_time_span_over_limit": output["point_time_span_over_limit"],
                "alignment_median_all": {
                    mode: summary["all"][mode]["median_m"] for mode in modes
                },
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
