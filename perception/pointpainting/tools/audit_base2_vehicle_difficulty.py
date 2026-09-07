#!/usr/bin/env python3
"""Compare Base2 validation and online rosbag vehicle difficulty slices."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--d3-index", type=Path, required=True)
    parser.add_argument("--val-predictions", type=Path, required=True)
    parser.add_argument(
        "--include-train",
        action="store_true",
        help="Also audit the Base2 train GT distribution without detector metrics.",
    )
    parser.add_argument(
        "--online",
        action="append",
        nargs=3,
        metavar=("SCENE", "RAW_BAG", "OUTPUT_BAG"),
        default=[],
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--maximum-range-m", type=float, default=30.0)
    parser.add_argument("--match-distance-m", type=float, default=1.0)
    return parser.parse_args()


def direction_bin(x, y):
    if x >= abs(y):
        return "front"
    if -x >= abs(y):
        return "rear"
    return "left" if y >= 0.0 else "right"


def interval_bin(value, edges, labels):
    for upper, label in zip(edges, labels):
        if value < upper:
            return label
    return labels[-1]


def distance_bin(value):
    return interval_bin(
        value,
        (10.0, 20.0, 30.0, math.inf),
        ("0-10m", "10-20m", "20-30m", "30m+"),
    )


def point_bin(value):
    return interval_bin(
        value,
        (5, 10, 20, 50, math.inf),
        ("0-4", "5-9", "10-19", "20-49", "50+"),
    )


def yaw_rate_bin(value):
    return interval_bin(
        abs(value),
        (2.0, 5.0, 10.0, math.inf),
        ("0-2deg/s", "2-5deg/s", "5-10deg/s", "10deg/s+"),
    )


def points_in_bev_box(points_xy, box):
    x, y, _, length, width, _, yaw = [float(value) for value in box[:7]]
    dx = points_xy[:, 0] - x
    dy = points_xy[:, 1] - y
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    return (np.abs(local_x) <= length * 0.5) & (
        np.abs(local_y) <= width * 0.5
    )


def greedy_center_matches(gt_boxes, pred_boxes, pred_scores, maximum_distance):
    matches = {}
    candidates = []
    for pred_index, (box, score) in enumerate(zip(pred_boxes, pred_scores)):
        for gt_index, gt in enumerate(gt_boxes):
            distance = float(np.linalg.norm(np.asarray(box[:2]) - np.asarray(gt[:2])))
            if distance <= maximum_distance:
                candidates.append((-float(score), distance, pred_index, gt_index))
    used_predictions = set()
    used_gt = set()
    for negative_score, distance, pred_index, gt_index in sorted(candidates):
        if pred_index in used_predictions or gt_index in used_gt:
            continue
        used_predictions.add(pred_index)
        used_gt.add(gt_index)
        matches[gt_index] = {
            "score": -negative_score,
            "center_error_m": distance,
        }
    return matches


def load_d3_rows(path):
    rows = {}
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            rows[row["sample_id"]] = row
    return rows


def target_ego(row):
    return row["sweeps_oldest_to_current"][-1]["ego_pose"]


def audit_dataset_split(
    dataset_root,
    d3_rows,
    split,
    predictions_path,
    match_distance,
    maximum_range,
):
    with (dataset_root / ("custom_infos_{}.pkl".format(split))).open("rb") as stream:
        infos = pickle.load(stream)
    prediction_by_id = None
    if predictions_path is not None:
        with predictions_path.open("rb") as stream:
            predictions = pickle.load(stream)
        prediction_by_id = {item["frame_id"]: item for item in predictions}
    records = []
    for info in infos:
        frame_id = info["point_cloud"]["lidar_idx"]
        row = d3_rows[frame_id]
        names = np.asarray(info["annos"]["name"], dtype=str)
        all_boxes = np.asarray(info["annos"]["gt_boxes_lidar"])
        vehicle_mask = names == "Vehicle"
        vehicle_mask &= all_boxes[:, 0] >= -30.0
        vehicle_mask &= all_boxes[:, 0] < 30.0
        vehicle_mask &= all_boxes[:, 1] >= -30.0
        vehicle_mask &= all_boxes[:, 1] < 30.0
        vehicle_mask &= all_boxes[:, 2] >= -3.0
        vehicle_mask &= all_boxes[:, 2] < 2.0
        vehicle_mask &= np.linalg.norm(all_boxes[:, :2], axis=1) < maximum_range
        vehicle_boxes = all_boxes[vehicle_mask]
        if not len(vehicle_boxes):
            continue
        points = np.load(dataset_root / "points" / (frame_id + ".npy"), mmap_mode="r")
        matches = None
        if prediction_by_id is not None:
            prediction = prediction_by_id.get(frame_id, {})
            pred_names = np.asarray(prediction.get("name", []), dtype=str)
            pred_boxes = np.asarray(
                prediction.get("boxes_lidar", np.empty((0, 7)))
            )
            pred_scores = np.asarray(prediction.get("score", np.empty((0,))))
            vehicle_mask = pred_names == "Vehicle"
            matches = greedy_center_matches(
                vehicle_boxes,
                pred_boxes[vehicle_mask],
                pred_scores[vehicle_mask],
                match_distance,
            )
        ego = target_ego(row)
        yaw_rate = float(ego.get("derived_yaw_rate_dps") or 0.0)
        for gt_index, box in enumerate(vehicle_boxes):
            inside = points_in_bev_box(points[:, :2], box)
            point_count = int(inside.sum())
            vehicle_semantic = (
                float(np.asarray(points[inside, 6]).mean()) if point_count else None
            )
            match = matches.get(gt_index) if matches is not None else None
            x, y = float(box[0]), float(box[1])
            records.append(
                {
                    "source": "base2_{}".format(split),
                    "scene": "base2_{}".format(split),
                    "frame_id": frame_id,
                    "family": row["mission_family"],
                    "distance_m": math.hypot(x, y),
                    "direction": direction_bin(x, y),
                    "point_count": point_count,
                    "vehicle_semantic_mean": vehicle_semantic,
                    "ego_speed_mps": float(ego.get("derived_speed_mps") or 0.0),
                    "ego_yaw_rate_dps": yaw_rate,
                    "matched_1m": (
                        match is not None if matches is not None else None
                    ),
                    "score": (
                        float(match["score"])
                        if match
                        else (0.0 if matches is not None else None)
                    ),
                    "center_error_m": match["center_error_m"] if match else None,
                }
            )
    return records


def configure_evaluator_imports(repo_root):
    source = repo_root / "base_ws/src/tracking_prediction_evaluation/src"
    sys.path.insert(0, str(source))


def read_lidar_frames(path):
    import rosbag
    from sensor_msgs import point_cloud2

    frames = []
    with rosbag.Bag(str(path), "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=["/sensor/lidar1"]):
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = bag_time.to_sec()
            xyz = np.asarray(
                list(
                    point_cloud2.read_points(
                        message,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                    )
                ),
                dtype=np.float32,
            )
            frames.append((float(stamp), xyz[:, :2]))
    return frames


def nearest_lidar(lidar_frames, timestamps, target):
    position = bisect.bisect_left(timestamps, target)
    candidates = []
    for index in (position - 1, position):
        if 0 <= index < len(lidar_frames):
            candidates.append((abs(lidar_frames[index][0] - target), index))
    if not candidates:
        return None, None
    offset, index = min(candidates)
    return lidar_frames[index][1], float(offset)


def ego_motion(ego_points, timestamp):
    times = [item.timestamp for item in ego_points]
    position = bisect.bisect_left(times, timestamp)
    lower = max(0, position - 1)
    upper = min(len(ego_points) - 1, position)
    if lower == upper:
        return 0.0, 0.0
    first = ego_points[lower]
    second = ego_points[upper]
    dt = second.timestamp - first.timestamp
    if dt <= 1.0e-6:
        return 0.0, 0.0
    delta = (second.yaw - first.yaw + math.pi) % (2.0 * math.pi) - math.pi
    speed = math.hypot(second.x - first.x, second.y - first.y) / dt
    return speed, math.degrees(delta / dt)


def audit_online(
    repo_root,
    scene,
    raw_bag,
    output_bag,
    maximum_range,
    match_distance,
):
    configure_evaluator_imports(repo_root)
    from tracking_prediction_evaluation.bbox_reference import (
        transform_segments_to_calibrated_bbox_center,
    )
    from tracking_prediction_evaluation.detection_evaluator import (
        build_evaluation_frames,
        read_detection_frames,
    )
    from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points
    from tracking_prediction_evaluation.static_transform import (
        load_morai_to_map_transform,
    )
    from tracking_prediction_evaluation.trajectory import (
        read_morai_trajectories,
        transform_segments_planar,
    )

    translation_xyz, transform_yaw = load_morai_to_map_transform(
        repo_root / "configs/tf/morai_lanelet_static_tf.yaml"
    )
    segments = read_morai_trajectories(
        raw_bag, topic="/Object_topic", include_classes=("vehicle",)
    )
    segments, _ = transform_segments_to_calibrated_bbox_center(segments)
    segments = transform_segments_planar(segments, translation_xyz, transform_yaw)
    ego_points = tuple(
        sorted(
            read_ego_points(
                raw_bag, "/Ego_topic", translation_xyz, transform_yaw
            ),
            key=lambda item: item.timestamp,
        )
    )
    source_frames = read_detection_frames(
        output_bag, "/perception/object_recognition/detection/objects"
    )
    frames, _ = build_evaluation_frames(
        segments,
        ego_points,
        source_frames,
        maximum_range_m=maximum_range,
    )
    lidar_frames = read_lidar_frames(raw_bag)
    lidar_timestamps = [item[0] for item in lidar_frames]
    records = []
    lidar_offsets = []
    sensor_x = 0.855
    for frame in frames:
        gt_items = [(segment, state) for segment, state in frame.gt if segment.class_name == "vehicle"]
        predictions = [item for item in frame.predictions if item.class_name == "vehicle"]
        gt_local = []
        for _, gt in gt_items:
            dx = gt.x - frame.ego.x
            dy = gt.y - frame.ego.y
            cosine = math.cos(frame.ego.yaw)
            sine = math.sin(frame.ego.yaw)
            local_x = cosine * dx + sine * dy - sensor_x
            local_y = -sine * dx + cosine * dy
            local_yaw = (gt.yaw - frame.ego.yaw + math.pi) % (2.0 * math.pi) - math.pi
            gt_local.append(
                np.asarray(
                    [local_x, local_y, 0.0, gt.length, gt.width, gt.height, local_yaw],
                    dtype=np.float64,
                )
            )
        pred_boxes = [
            np.asarray([item.x, item.y, item.z, item.length, item.width, item.height, item.yaw])
            for item in predictions
        ]
        # Predictions are in map coordinates after build_evaluation_frames, so
        # matching is performed directly in map coordinates here.
        matches = {}
        candidates = []
        for pred_index, prediction in enumerate(predictions):
            for gt_index, (_, gt) in enumerate(gt_items):
                distance = math.hypot(prediction.x - gt.x, prediction.y - gt.y)
                if distance <= match_distance:
                    candidates.append((-prediction.score, distance, pred_index, gt_index))
        used_pred, used_gt = set(), set()
        for negative_score, distance, pred_index, gt_index in sorted(candidates):
            if pred_index in used_pred or gt_index in used_gt:
                continue
            used_pred.add(pred_index)
            used_gt.add(gt_index)
            matches[gt_index] = {"score": -negative_score, "center_error_m": distance}

        lidar_xy, lidar_offset = nearest_lidar(
            lidar_frames, lidar_timestamps, frame.timestamp
        )
        if lidar_offset is not None:
            lidar_offsets.append(lidar_offset)
        ego_speed, yaw_rate = ego_motion(ego_points, frame.timestamp)
        for gt_index, box in enumerate(gt_local):
            inside = (
                points_in_bev_box(lidar_xy, box)
                if lidar_xy is not None
                else np.zeros((0,), dtype=bool)
            )
            match = matches.get(gt_index)
            x, y = float(box[0]), float(box[1])
            records.append(
                {
                    "source": "online",
                    "scene": scene,
                    "frame_id": "{:.9f}".format(frame.timestamp),
                    "family": scene,
                    "distance_m": math.hypot(x, y),
                    "direction": direction_bin(x, y),
                    "point_count": int(inside.sum()),
                    "vehicle_semantic_mean": None,
                    "ego_speed_mps": ego_speed,
                    "ego_yaw_rate_dps": yaw_rate,
                    "matched_1m": match is not None,
                    "score": float(match["score"]) if match else 0.0,
                    "center_error_m": match["center_error_m"] if match else None,
                    "nearest_lidar_offset_sec": lidar_offset,
                }
            )
    return records, lidar_offsets


def summary_stats(records):
    if not records:
        return {"count": 0}
    scored = [row for row in records if row["matched_1m"] is not None]
    scores = np.asarray([row["score"] for row in scored], dtype=np.float64)
    points = np.asarray([row["point_count"] for row in records], dtype=np.float64)
    matched = np.asarray([row["matched_1m"] for row in scored], dtype=bool)
    yaw = np.asarray([abs(row["ego_yaw_rate_dps"]) for row in records], dtype=np.float64)
    semantic = [row["vehicle_semantic_mean"] for row in records if row["vehicle_semantic_mean"] is not None]
    return {
        "count": len(records),
        "recall_1m": float(matched.mean()) if len(matched) else None,
        "score_mean_including_misses": float(scores.mean()) if len(scores) else None,
        "score_p50_including_misses": float(np.median(scores)) if len(scores) else None,
        "point_count_mean": float(points.mean()),
        "point_count_p50": float(np.median(points)),
        "point_count_p10": float(np.percentile(points, 10)),
        "zero_point_rate": float((points == 0).mean()),
        "ego_abs_yaw_rate_mean_dps": float(yaw.mean()),
        "ego_abs_yaw_rate_p50_dps": float(np.median(yaw)),
        "vehicle_semantic_mean": float(np.mean(semantic)) if semantic else None,
    }


def grouped(records, key_function, order=None):
    buckets = defaultdict(list)
    for row in records:
        buckets[key_function(row)].append(row)
    keys = order if order is not None else sorted(buckets)
    return {key: summary_stats(buckets[key]) for key in keys if key in buckets}


def summarize(records):
    return {
        "overall": summary_stats(records),
        "direction": grouped(records, lambda row: row["direction"], ("front", "left", "right", "rear")),
        "distance": grouped(records, lambda row: distance_bin(row["distance_m"]), ("0-10m", "10-20m", "20-30m", "30m+")),
        "point_count": grouped(records, lambda row: point_bin(row["point_count"]), ("0-4", "5-9", "10-19", "20-49", "50+")),
        "ego_yaw_rate": grouped(records, lambda row: yaw_rate_bin(row["ego_yaw_rate_dps"]), ("0-2deg/s", "2-5deg/s", "5-10deg/s", "10deg/s+")),
        "family": grouped(records, lambda row: row["family"]),
    }


def markdown_table(title, mapping):
    lines = ["## " + title, "", "| Slice | GT | Share | Recall@1m | Score mean | Points p50 |", "|---|---:|---:|---:|---:|---:|"]
    total = sum(item["count"] for item in mapping.values())
    for key, item in mapping.items():
        recall = item["recall_1m"]
        score = item["score_mean_including_misses"]
        lines.append(
            "| {} | {} | {:.1%} | {} | {} | {:.1f} |".format(
                key,
                item["count"],
                item["count"] / max(total, 1),
                "{:.1%}".format(recall) if recall is not None else "—",
                "{:.3f}".format(score) if score is not None else "—",
                item["point_count_p50"],
            )
        )
    return lines


def main():
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[3]
    dataset_root = args.dataset_root.resolve()
    d3_rows = load_d3_rows(args.d3_index.resolve() / "temporal_samples.jsonl")
    records = []
    if args.include_train:
        records.extend(
            audit_dataset_split(
                dataset_root,
                d3_rows,
                "train",
                None,
                args.match_distance_m,
                args.maximum_range_m,
            )
        )
    records.extend(audit_dataset_split(
        dataset_root,
        d3_rows,
        "val",
        args.val_predictions.resolve(),
        args.match_distance_m,
        args.maximum_range_m,
    ))
    online_offsets = {}
    for scene, raw_bag, output_bag in args.online:
        scene_records, offsets = audit_online(
            repo_root,
            scene,
            Path(raw_bag).resolve(),
            Path(output_bag).resolve(),
            args.maximum_range_m,
            args.match_distance_m,
        )
        records.extend(scene_records)
        online_offsets[scene] = {
            "count": len(offsets),
            "mean_sec": float(np.mean(offsets)) if offsets else None,
            "p50_sec": float(np.median(offsets)) if offsets else None,
            "p95_sec": float(np.percentile(offsets, 95)) if offsets else None,
        }

    by_source = defaultdict(list)
    for row in records:
        by_source[row["scene"]].append(row)
    report = {
        "protocol": {
            "match_distance_m": args.match_distance_m,
            "direction": "front/rear are +/-45 degrees; remaining sectors are left/right",
            "point_count": "target sweep BEV points inside calibrated GT box",
            "miss_score": 0.0,
            "online_lidar_association": "nearest raw LiDAR header stamp to adjusted detection stamp",
        },
        "sources": {key: summarize(value) for key, value in by_source.items()},
        "online_nearest_lidar_offsets": online_offsets,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "vehicle_difficulty_audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with (args.output_dir / "vehicle_difficulty_records.jsonl").open("w", encoding="utf-8") as stream:
        for row in records:
            stream.write(json.dumps(row, sort_keys=True) + "\n")

    lines = ["# Base2 vehicle-positive difficulty audit", ""]
    for source, summary in report["sources"].items():
        lines.extend(markdown_table(source + " / direction", summary["direction"]))
        lines.append("")
        lines.extend(markdown_table(source + " / distance", summary["distance"]))
        lines.append("")
        lines.extend(markdown_table(source + " / point count", summary["point_count"]))
        lines.append("")
        lines.extend(markdown_table(source + " / Ego yaw rate", summary["ego_yaw_rate"]))
        lines.append("")
    (args.output_dir / "vehicle_difficulty_audit.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
