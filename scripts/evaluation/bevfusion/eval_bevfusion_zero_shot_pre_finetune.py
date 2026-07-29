#!/usr/bin/env python3
"""Evaluate pre-finetuning BEVFusion MarkerArray detections against MORAI object GT."""

import argparse
import bisect
import csv
import json
import math
import os
import re
import statistics
import sys

import numpy as np


PREDICTION_GROUP = {
    "car": "vehicle",
    "truck": "vehicle",
    "construction_vehicle": "vehicle",
    "bus": "vehicle",
    "trailer": "vehicle",
    "motorcycle": "vehicle",
    "bicycle": "vehicle",
    "pedestrian": "pedestrian",
    "barrier": "obstacle",
    "traffic_cone": "obstacle",
}
GROUPS = ("vehicle", "pedestrian", "obstacle")
SCORE_PATTERN = re.compile(r"^(.*)\s+([-+0-9.eE]+)$")


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_rotation(yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def pose_matrix(position, rotation_degrees):
    roll = math.radians(float(rotation_degrees["roll"]))
    pitch = math.radians(float(rotation_degrees["pitch"]))
    yaw = math.radians(float(rotation_degrees["yaw"]))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = [float(position["x"]), float(position["y"]), float(position["z"])]
    return transform


def load_lidar_calibration(path):
    with open(path, "r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    lidars = config.get("Lidar3DList", [])
    if len(lidars) != 1:
        raise ValueError("expected exactly one Lidar3DList entry")
    lidar = lidars[0]
    return pose_matrix(lidar["pos"], lidar["rot"])


def stamp_seconds(message):
    return message.header.stamp.to_sec()


def marker_stamp(message):
    for marker in message.markers:
        if marker.ns == "bevfusion_boxes" and marker.action == marker.ADD:
            return marker.header.stamp.to_sec()
    for marker in message.markers:
        if not marker.header.stamp.is_zero():
            return marker.header.stamp.to_sec()
    return None


def parse_marker_predictions(message):
    labels = {}
    for marker in message.markers:
        if marker.ns != "bevfusion_labels" or marker.action != marker.ADD:
            continue
        match = SCORE_PATTERN.match(marker.text.strip())
        if match:
            labels[marker.id] = (match.group(1), float(match.group(2)))

    predictions = []
    for marker in message.markers:
        if marker.ns != "bevfusion_boxes" or marker.action != marker.ADD:
            continue
        class_name, score = labels.get(marker.id, ("unknown", float("nan")))
        quaternion = marker.pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        predictions.append(
            {
                "id": marker.id,
                "x": float(marker.pose.position.x),
                "y": float(marker.pose.position.y),
                "z": float(marker.pose.position.z),
                "length": float(marker.scale.x),
                "width": float(marker.scale.y),
                "height": float(marker.scale.z),
                "yaw": yaw,
                "class_name": class_name,
                "group": PREDICTION_GROUP.get(class_name, "unknown"),
                "score": score,
            }
        )
    return predictions


def object_lists(message):
    return (
        [(item, "vehicle") for item in message.npc_list]
        + [(item, "pedestrian") for item in message.pedestrian_list]
        + [(item, "obstacle") for item in message.obstacle_list]
    )


def interpolate_ego(before, after, target):
    before_time = stamp_seconds(before)
    after_time = stamp_seconds(after)
    if after_time <= before_time:
        ratio = 0.0
    else:
        ratio = (target - before_time) / (after_time - before_time)
    ratio = min(1.0, max(0.0, ratio))
    heading_before = math.radians(float(before.heading))
    heading_delta = wrap_angle(math.radians(float(after.heading)) - heading_before)
    return {
        "x": float(before.position.x) + ratio * (float(after.position.x) - float(before.position.x)),
        "y": float(before.position.y) + ratio * (float(after.position.y) - float(before.position.y)),
        "z": float(before.position.z) + ratio * (float(after.position.z) - float(before.position.z)),
        "yaw": wrap_angle(heading_before + ratio * heading_delta),
    }


def ego_at(messages, times, target, max_gap):
    index = bisect.bisect_left(times, target)
    if index == 0:
        candidate = messages[0]
        return interpolate_ego(candidate, candidate, target) if abs(times[0] - target) <= max_gap else None
    if index == len(times):
        candidate = messages[-1]
        return interpolate_ego(candidate, candidate, target) if abs(times[-1] - target) <= max_gap else None
    if target - times[index - 1] > max_gap or times[index] - target > max_gap:
        return None
    return interpolate_ego(messages[index - 1], messages[index], target)


def nearest_message(messages, times, target, max_gap):
    index = bisect.bisect_left(times, target)
    candidates = []
    if index < len(times):
        candidates.append((abs(times[index] - target), messages[index]))
    if index > 0:
        candidates.append((abs(times[index - 1] - target), messages[index - 1]))
    if not candidates:
        return None
    difference, message = min(candidates, key=lambda item: item[0])
    return message if difference <= max_gap else None


def gt_in_lidar(message, ego, base_from_lidar, xy_range):
    lidar_from_base = np.linalg.inv(base_from_lidar)
    world_from_base_rotation = yaw_rotation(ego["yaw"])
    ego_position = np.array([ego["x"], ego["y"], ego["z"]], dtype=np.float64)
    lidar_yaw = math.atan2(base_from_lidar[1, 0], base_from_lidar[0, 0])
    ground_truth = []
    for item, group in object_lists(message):
        length = float(item.size.x)
        width = float(item.size.y)
        height = float(item.size.z)
        # MORAI may publish placeholder objects with a zero-sized box.
        if min(length, width, height) <= 0.05:
            continue
        world_position = np.array(
            [float(item.position.x), float(item.position.y), float(item.position.z)], dtype=np.float64
        )
        base_position = world_from_base_rotation.T @ (world_position - ego_position)
        lidar_position = lidar_from_base[:3, :3] @ base_position + lidar_from_base[:3, 3]
        if not (xy_range[0] <= lidar_position[0] <= xy_range[2]):
            continue
        if not (xy_range[1] <= lidar_position[1] <= xy_range[3]):
            continue
        ground_truth.append(
            {
                "unique_id": int(item.unique_id),
                "group": group,
                "x": float(lidar_position[0]),
                "y": float(lidar_position[1]),
                "z": float(lidar_position[2]),
                "length": length,
                "width": width,
                "height": height,
                "yaw": wrap_angle(math.radians(float(item.heading)) - ego["yaw"] - lidar_yaw),
            }
        )
    return ground_truth


def rectangle_corners(box):
    half_length = max(0.0, box["length"] * 0.5)
    half_width = max(0.0, box["width"] * 0.5)
    local = np.array(
        [[-half_length, -half_width], [half_length, -half_width],
         [half_length, half_width], [-half_length, half_width]],
        dtype=np.float64,
    )
    cosine, sine = math.cos(box["yaw"]), math.sin(box["yaw"])
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return local @ rotation.T + np.array([box["x"], box["y"]])


def polygon_area(points):
    if len(points) < 3:
        return 0.0
    points = np.asarray(points)
    return 0.5 * abs(
        np.dot(points[:, 0], np.roll(points[:, 1], -1))
        - np.dot(points[:, 1], np.roll(points[:, 0], -1))
    )


def cross_2d(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def polygon_clip(subject, clipper):
    output = [np.asarray(point, dtype=np.float64) for point in subject]
    for index in range(len(clipper)):
        edge_start = np.asarray(clipper[index], dtype=np.float64)
        edge_end = np.asarray(clipper[(index + 1) % len(clipper)], dtype=np.float64)
        input_points = output
        output = []
        if not input_points:
            break

        def inside(point):
            return cross_2d(edge_end - edge_start, point - edge_start) >= -1e-9

        def intersection(start, end):
            segment = end - start
            edge = edge_end - edge_start
            denominator = cross_2d(segment, edge)
            if abs(denominator) < 1e-12:
                return end
            ratio = cross_2d(edge_start - start, edge) / denominator
            return start + ratio * segment

        previous = input_points[-1]
        for current in input_points:
            if inside(current):
                if not inside(previous):
                    output.append(intersection(previous, current))
                output.append(current)
            elif inside(previous):
                output.append(intersection(previous, current))
            previous = current
    return output


def bev_iou(first, second):
    first_corners = rectangle_corners(first)
    second_corners = rectangle_corners(second)
    intersection = polygon_area(polygon_clip(first_corners, second_corners))
    union = first["length"] * first["width"] + second["length"] * second["width"] - intersection
    return intersection / union if union > 1e-12 else 0.0


def greedy_frame_match(predictions, ground_truth, threshold, group_aware):
    pairs = []
    available = set(range(len(ground_truth)))
    for prediction_index in sorted(
        range(len(predictions)), key=lambda index: predictions[index]["score"], reverse=True
    ):
        prediction = predictions[prediction_index]
        candidates = []
        for gt_index in available:
            gt = ground_truth[gt_index]
            if group_aware and prediction["group"] != gt["group"]:
                continue
            candidates.append((bev_iou(prediction, gt), gt_index))
        if not candidates:
            continue
        iou, gt_index = max(candidates)
        if iou >= threshold:
            available.remove(gt_index)
            pairs.append((prediction_index, gt_index, iou))
    return pairs


def average_precision(frames, threshold, group=None):
    predictions = []
    gt_by_frame = {}
    gt_count = 0
    for frame_index, frame in enumerate(frames):
        ground_truth = [gt for gt in frame["ground_truth"] if group is None or gt["group"] == group]
        gt_by_frame[frame_index] = ground_truth
        gt_count += len(ground_truth)
        for prediction in frame["predictions"]:
            if group is not None and prediction["group"] != group:
                continue
            predictions.append((prediction["score"], frame_index, prediction))
    if gt_count == 0:
        return None
    matched = {frame_index: set() for frame_index in gt_by_frame}
    true_positive = []
    false_positive = []
    for _score, frame_index, prediction in sorted(predictions, key=lambda item: item[0], reverse=True):
        candidates = [
            (bev_iou(prediction, gt), gt_index)
            for gt_index, gt in enumerate(gt_by_frame[frame_index])
            if gt_index not in matched[frame_index]
        ]
        if candidates:
            iou, gt_index = max(candidates)
        else:
            iou, gt_index = 0.0, -1
        if iou >= threshold:
            matched[frame_index].add(gt_index)
            true_positive.append(1.0)
            false_positive.append(0.0)
        else:
            true_positive.append(0.0)
            false_positive.append(1.0)
    if not true_positive:
        return 0.0
    tp = np.cumsum(true_positive)
    fp = np.cumsum(false_positive)
    recall = tp / float(gt_count)
    precision = tp / np.maximum(tp + fp, 1e-12)
    samples = np.linspace(0.0, 1.0, 101)
    interpolated = [np.max(precision[recall >= value]) if np.any(recall >= value) else 0.0 for value in samples]
    return float(np.mean(interpolated))


def summarize_values(values):
    if not values:
        return "n/a"
    array = np.asarray(values, dtype=np.float64)
    return "mean={:.3f}, median={:.3f}, p95={:.3f}, max={:.3f}".format(
        float(np.mean(array)), float(np.median(array)), float(np.percentile(array, 95)), float(np.max(array))
    )


def write_csv(path, rows, fields):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
    parser = argparse.ArgumentParser(
        description="Evaluate zero-shot, pre-finetuning BEVFusion output against MORAI GT"
    )
    parser.add_argument("raw_bag", help="original Sensor_4 raw bag")
    parser.add_argument("result_bag", help="bag containing GT and BEVFusion output markers")
    parser.add_argument("--out-prefix", required=True, help="output path without extension")
    parser.add_argument(
        "--sensor-config", default=os.path.join(root, "configs/sensors/Sensor_5.json")
    )
    parser.add_argument("--max-sync-gap", type=float, default=0.10)
    parser.add_argument("--iou-thresholds", type=float, nargs="+", default=[0.25, 0.50])
    parser.add_argument("--xy-range", type=float, nargs=4, default=[-54.0, -54.0, 54.0, 54.0])
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        import rosbag
    except ImportError:
        print("rosbag Python module not found; source /opt/ros/noetic/setup.bash first", file=sys.stderr)
        return 2

    for path in (args.raw_bag, args.result_bag, args.sensor_config):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

    base_from_lidar = load_lidar_calibration(args.sensor_config)
    ego_messages, object_messages = [], []
    marker_records, latency_records = [], []
    with rosbag.Bag(args.result_bag, "r") as bag:
        for topic, message, record_time in bag.read_messages(
            topics=["/Ego_topic", "/Object_topic", "/perception/bevfusion/markers", "/perception/bevfusion/latency_ms"]
        ):
            if topic == "/Ego_topic":
                ego_messages.append(message)
            elif topic == "/Object_topic":
                object_messages.append(message)
            elif topic == "/perception/bevfusion/markers":
                stamp = marker_stamp(message)
                if stamp is not None:
                    marker_records.append((stamp, record_time.to_sec(), message))
            elif topic == "/perception/bevfusion/latency_ms":
                latency_records.append((record_time.to_sec(), float(message.data)))

    if not ego_messages or not object_messages or not marker_records:
        raise RuntimeError(
            "result bag needs /Ego_topic, /Object_topic, and /perception/bevfusion/markers"
        )
    ego_messages.sort(key=stamp_seconds)
    object_messages.sort(key=stamp_seconds)
    ego_times = [stamp_seconds(message) for message in ego_messages]
    object_times = [stamp_seconds(message) for message in object_messages]
    latency_times = [item[0] for item in latency_records]

    frames = []
    rejected = 0
    for stamp, record_time, marker_message in marker_records:
        ego = ego_at(ego_messages, ego_times, stamp, args.max_sync_gap)
        objects = nearest_message(object_messages, object_times, stamp, args.max_sync_gap)
        if ego is None or objects is None:
            rejected += 1
            continue
        latency = None
        if latency_records:
            index = bisect.bisect_left(latency_times, record_time)
            candidate_indices = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(latency_records)]
            if candidate_indices:
                nearest_index = min(candidate_indices, key=lambda candidate: abs(latency_times[candidate] - record_time))
                latency = latency_records[nearest_index][1]
        frames.append(
            {
                "stamp": stamp,
                "record_time": record_time,
                "latency_ms": latency,
                "predictions": parse_marker_predictions(marker_message),
                "ground_truth": gt_in_lidar(objects, ego, base_from_lidar, args.xy_range),
            }
        )
    if not frames:
        raise RuntimeError("no synchronized evaluation frames")

    lidar_stamps = []
    with rosbag.Bag(args.raw_bag, "r") as bag:
        for _topic, message, _record_time in bag.read_messages(topics=["/sensor/lidar1"]):
            lidar_stamps.append(stamp_seconds(message))

    frame_rows = []
    detection_rows = []
    primary_threshold = args.iou_thresholds[0]
    center_errors, longitudinal_errors, lateral_errors, yaw_errors, matched_ious = [], [], [], [], []
    total_tp = total_fp = total_fn = 0
    for frame_index, frame in enumerate(frames):
        pairs = greedy_frame_match(frame["predictions"], frame["ground_truth"], primary_threshold, False)
        pair_by_prediction = {prediction: (gt, iou) for prediction, gt, iou in pairs}
        tp = len(pairs)
        fp = len(frame["predictions"]) - tp
        fn = len(frame["ground_truth"]) - tp
        total_tp += tp
        total_fp += fp
        total_fn += fn
        frame_rows.append(
            {
                "stamp": "{:.9f}".format(frame["stamp"]),
                "latency_ms": "" if frame["latency_ms"] is None else "{:.3f}".format(frame["latency_ms"]),
                "gt_count": len(frame["ground_truth"]),
                "prediction_count": len(frame["predictions"]),
                "tp_iou_{:.2f}".format(primary_threshold): tp,
                "fp_iou_{:.2f}".format(primary_threshold): fp,
                "fn_iou_{:.2f}".format(primary_threshold): fn,
            }
        )
        for prediction_index, prediction in enumerate(frame["predictions"]):
            gt_index, iou = pair_by_prediction.get(prediction_index, (None, 0.0))
            gt = frame["ground_truth"][gt_index] if gt_index is not None else None
            if gt is not None:
                delta_x = prediction["x"] - gt["x"]
                delta_y = prediction["y"] - gt["y"]
                center_error = math.hypot(delta_x, delta_y)
                yaw_error = abs(wrap_angle(prediction["yaw"] - gt["yaw"]))
                center_errors.append(center_error)
                longitudinal_errors.append(abs(delta_x))
                lateral_errors.append(abs(delta_y))
                yaw_errors.append(math.degrees(yaw_error))
                matched_ious.append(iou)
            else:
                delta_x = delta_y = center_error = yaw_error = None
            detection_rows.append(
                {
                    "stamp": "{:.9f}".format(frame["stamp"]),
                    "prediction_id": prediction["id"],
                    "class_name": prediction["class_name"],
                    "prediction_group": prediction["group"],
                    "score": "{:.6f}".format(prediction["score"]),
                    "pred_x": "{:.4f}".format(prediction["x"]),
                    "pred_y": "{:.4f}".format(prediction["y"]),
                    "matched": int(gt is not None),
                    "gt_unique_id": "" if gt is None else gt["unique_id"],
                    "gt_group": "" if gt is None else gt["group"],
                    "bev_iou": "{:.6f}".format(iou),
                    "longitudinal_error_m": "" if delta_x is None else "{:.4f}".format(delta_x),
                    "lateral_error_m": "" if delta_y is None else "{:.4f}".format(delta_y),
                    "center_error_m": "" if center_error is None else "{:.4f}".format(center_error),
                    "yaw_error_deg": "" if yaw_error is None else "{:.4f}".format(math.degrees(yaw_error)),
                }
            )

    precision = total_tp / float(total_tp + total_fp) if total_tp + total_fp else 0.0
    recall = total_tp / float(total_tp + total_fn) if total_tp + total_fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    latency_values = [frame["latency_ms"] for frame in frames if frame["latency_ms"] is not None]
    output_duration = frames[-1]["stamp"] - frames[0]["stamp"] if len(frames) > 1 else 0.0
    output_hz = (len(frames) - 1) / output_duration if output_duration > 0.0 else 0.0
    lidar_duration = lidar_stamps[-1] - lidar_stamps[0] if len(lidar_stamps) > 1 else 0.0
    lidar_hz = (len(lidar_stamps) - 1) / lidar_duration if lidar_duration > 0.0 else 0.0

    lines = [
        "======= Pre-finetuning BEVFusion vs MORAI GT Evaluation =======",
        "raw bag: {}".format(args.raw_bag),
        "result bag: {}".format(args.result_bag),
        "sensor config: {}".format(args.sensor_config),
        "metric scope: oriented BEV boxes inside x/y [{:.1f}, {:.1f}] ~ [{:.1f}, {:.1f}] m".format(*args.xy_range),
        "warning: this is MORAI zero-shot AP, not official nuScenes mAP/NDS",
        "------------------------ Coverage ------------------------",
        "raw LiDAR frames: {} ({:.3f} Hz by header stamp)".format(len(lidar_stamps), lidar_hz),
        "evaluated output frames: {} ({:.3f} Hz by source stamp)".format(len(frames), output_hz),
        "sync-rejected output frames: {}".format(rejected),
        "output/raw frame ratio: {:.3f}".format(len(frames) / float(len(lidar_stamps)) if lidar_stamps else 0.0),
        "end-to-end bridge latency ms: {}".format(summarize_values(latency_values)),
        "---------------- Class-agnostic detection ----------------",
    ]
    for threshold in args.iou_thresholds:
        ap = average_precision(frames, threshold)
        lines.append("AP@BEV-IoU {:.2f}: {:.4f}".format(threshold, ap if ap is not None else float("nan")))
    lines.extend(
        [
            "at score>=recorded threshold and IoU {:.2f}: TP={}, FP={}, FN={}".format(
                primary_threshold, total_tp, total_fp, total_fn
            ),
            "precision={:.4f}, recall={:.4f}, F1={:.4f}".format(precision, recall, f1),
            "---------------- Group-aware detection -------------------",
        ]
    )
    for threshold in args.iou_thresholds:
        group_values = []
        for group in GROUPS:
            value = average_precision(frames, threshold, group)
            if value is not None:
                group_values.append(value)
                lines.append("{} AP@{:.2f}: {:.4f}".format(group, threshold, value))
            else:
                lines.append("{} AP@{:.2f}: n/a (no GT)".format(group, threshold))
        lines.append(
            "macro group mAP@{:.2f}: {}".format(
                threshold, "{:.4f}".format(statistics.mean(group_values)) if group_values else "n/a"
            )
        )
    lines.extend(
        [
            "--------------- Matched localization errors --------------",
            "matched BEV IoU: {}".format(summarize_values(matched_ious)),
            "center error m: {}".format(summarize_values(center_errors)),
            "|longitudinal x error| m: {}".format(summarize_values(longitudinal_errors)),
            "|lateral y error| m: {}".format(summarize_values(lateral_errors)),
            "yaw error deg: {}".format(summarize_values(yaw_errors)),
            "===========================================================",
        ]
    )

    output_prefix = os.path.abspath(args.out_prefix)
    os.makedirs(os.path.dirname(output_prefix), exist_ok=True)
    with open(output_prefix + ".txt", "w", encoding="utf-8") as output:
        output.write("\n".join(lines) + "\n")
    frame_fields = [
        "stamp", "latency_ms", "gt_count", "prediction_count",
        "tp_iou_{:.2f}".format(primary_threshold),
        "fp_iou_{:.2f}".format(primary_threshold),
        "fn_iou_{:.2f}".format(primary_threshold),
    ]
    detection_fields = [
        "stamp", "prediction_id", "class_name", "prediction_group", "score",
        "pred_x", "pred_y", "matched", "gt_unique_id", "gt_group", "bev_iou",
        "longitudinal_error_m", "lateral_error_m", "center_error_m", "yaw_error_deg",
    ]
    write_csv(output_prefix + "_frames.csv", frame_rows, frame_fields)
    write_csv(output_prefix + "_detections.csv", detection_rows, detection_fields)
    print("\n".join(lines))
    print("saved: {}.txt".format(output_prefix))
    print("saved: {}_frames.csv".format(output_prefix))
    print("saved: {}_detections.csv".format(output_prefix))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
