"""Online 3D detection evaluation from canonical ROS ``DetectedObjects``.

The evaluator deliberately operates on recorded detector messages rather than
model tensors.  It therefore measures the deployed preprocessing, confidence
floor, timestamp, coordinate transform and post-processing together.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .pipeline_evaluator import (
    _causal_motion_state,
    EgoPoint,
    active_gt,
    interpolate_ego,
    message_timestamp,
    quaternion_yaw,
    statistics,
)
from .bbox_reference import oriented_bev_iou


CENTER_THRESHOLDS_M = (0.5, 1.0, 2.0, 4.0)
BEV_IOU_THRESHOLDS = (0.3, 0.5)
DISTANCE_BINS_M = ((0.0, 10.0), (10.0, 20.0), (20.0, 30.0))
CLASS_NAMES = ("vehicle", "pedestrian", "obstacle")


@dataclass(frozen=True)
class DetectionEstimate:
    class_name: str
    score: float
    x: float
    y: float
    z: float
    length: float
    width: float
    height: float
    yaw: float
    velocity_x: Optional[float] = None
    velocity_y: Optional[float] = None


@dataclass(frozen=True)
class EvaluationFrame:
    timestamp: float
    ego: EgoPoint
    gt: tuple
    predictions: tuple[DetectionEstimate, ...]


def _classification_name(label: int) -> str | None:
    # autoware_perception_msgs/ObjectClassification constants.  PointPainting
    # publishes generic obstacles as UNKNOWN and the other two classes as CAR
    # and PEDESTRIAN respectively.
    if label in (1, 2, 3, 4, 5, 6):
        return "vehicle"
    if label == 7:
        return "pedestrian"
    if label == 0:
        return "obstacle"
    return None


def read_detection_frames(bag_path, detection_topic):
    """Read scores and complete boxes in their original LiDAR frame."""
    import rosbag

    frames = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_timestamp in bag.read_messages(topics=[detection_topic]):
            timestamp = message_timestamp(message, bag_timestamp)
            predictions = []
            for obj in message.objects:
                if not obj.classification:
                    continue
                dominant = max(
                    obj.classification, key=lambda item: float(item.probability)
                )
                class_name = _classification_name(int(dominant.label))
                if class_name is None:
                    continue
                pose = obj.kinematics.pose_with_covariance.pose
                dimensions = obj.shape.dimensions
                velocity_x = None
                velocity_y = None
                if obj.kinematics.has_twist:
                    twist = obj.kinematics.twist_with_covariance.twist.linear
                    body_x = float(twist.x)
                    body_y = float(twist.y)
                    local_yaw = quaternion_yaw(pose.orientation)
                    cosine = math.cos(local_yaw)
                    sine = math.sin(local_yaw)
                    velocity_x = cosine * body_x - sine * body_y
                    velocity_y = sine * body_x + cosine * body_y
                predictions.append(
                    DetectionEstimate(
                        class_name=class_name,
                        score=float(
                            max(
                                float(obj.existence_probability),
                                float(dominant.probability),
                            )
                        ),
                        x=float(pose.position.x),
                        y=float(pose.position.y),
                        z=float(pose.position.z),
                        length=float(dimensions.x),
                        width=float(dimensions.y),
                        height=float(dimensions.z),
                        yaw=quaternion_yaw(pose.orientation),
                        velocity_x=velocity_x,
                        velocity_y=velocity_y,
                    )
                )
            # Duplicate header stamps are possible after replay. Keep the most
            # complete deterministic copy, matching the GT reader policy.
            previous = frames.get(timestamp)
            if previous is None or len(predictions) > len(previous):
                frames[timestamp] = tuple(predictions)
    return frames


def transform_detections_to_map(
    detections: Sequence[DetectionEstimate],
    ego: EgoPoint,
    sensor_to_ego_x_m: float,
    sensor_to_ego_y_m: float,
    sensor_to_ego_z_m: float,
):
    cosine = math.cos(ego.yaw)
    sine = math.sin(ego.yaw)
    output = []
    for item in detections:
        local_x = sensor_to_ego_x_m + item.x
        local_y = sensor_to_ego_y_m + item.y
        output.append(
            DetectionEstimate(
                class_name=item.class_name,
                score=item.score,
                x=ego.x + cosine * local_x - sine * local_y,
                y=ego.y + sine * local_x + cosine * local_y,
                z=sensor_to_ego_z_m + item.z,
                length=item.length,
                width=item.width,
                height=item.height,
                yaw=(item.yaw + ego.yaw + math.pi) % (2.0 * math.pi) - math.pi,
                velocity_x=(
                    cosine * item.velocity_x - sine * item.velocity_y
                    if item.velocity_x is not None and item.velocity_y is not None
                    else None
                ),
                velocity_y=(
                    sine * item.velocity_x + cosine * item.velocity_y
                    if item.velocity_x is not None and item.velocity_y is not None
                    else None
                ),
            )
        )
    return tuple(output)


def build_evaluation_frames(
    segments,
    ego_points,
    detection_frames,
    *,
    maximum_range_m=30.0,
    maximum_interpolation_bracket_sec=0.15,
    sensor_to_ego_x_m=0.855,
    sensor_to_ego_y_m=0.0,
    sensor_to_ego_z_m=1.388,
):
    frames = []
    counts = defaultdict(int)
    for timestamp, source_detections in sorted(detection_frames.items()):
        ego = interpolate_ego(
            ego_points, timestamp, maximum_interpolation_bracket_sec
        )
        if ego is None:
            counts["missing_ego_frames"] += 1
            continue
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        predictions = transform_detections_to_map(
            source_detections,
            ego,
            sensor_to_ego_x_m,
            sensor_to_ego_y_m,
            sensor_to_ego_z_m,
        )
        predictions = tuple(
            item
            for item in predictions
            if math.hypot(item.x - ego.x, item.y - ego.y) <= maximum_range_m
        )
        frames.append(EvaluationFrame(timestamp, ego, tuple(gt_items), predictions))
    counts["evaluated_frames"] = len(frames)
    return tuple(frames), dict(counts)


def _scale_error(gt, prediction):
    gt_size = np.maximum(
        np.asarray((gt.length, gt.width, gt.height), dtype=np.float64), 1.0e-8
    )
    pred_size = np.maximum(
        np.asarray(
            (prediction.length, prediction.width, prediction.height),
            dtype=np.float64,
        ),
        1.0e-8,
    )
    intersection = float(np.prod(np.minimum(gt_size, pred_size)))
    union = float(np.prod(gt_size) + np.prod(pred_size) - intersection)
    return 1.0 - intersection / max(union, 1.0e-8)


def _orientation_error(gt, prediction):
    return abs((prediction.yaw - gt.yaw + math.pi) % (2.0 * math.pi) - math.pi)


def _ranked_matches(frames, class_name, distance_threshold_m):
    positives = sum(
        sum(segment.class_name == class_name for segment, _ in frame.gt)
        for frame in frames
    )
    ranked = []
    for frame_index, frame in enumerate(frames):
        ranked.extend(
            (prediction.score, frame_index, prediction_index)
            for prediction_index, prediction in enumerate(frame.predictions)
            if prediction.class_name == class_name
        )
    ranked.sort(key=lambda item: item[0], reverse=True)

    taken = set()
    rows = []
    for score, frame_index, prediction_index in ranked:
        frame = frames[frame_index]
        prediction = frame.predictions[prediction_index]
        candidates = [
            gt_index
            for gt_index, (segment, _) in enumerate(frame.gt)
            if segment.class_name == class_name and (frame_index, gt_index) not in taken
        ]
        best_gt_index = None
        best_distance = math.inf
        for gt_index in candidates:
            gt = frame.gt[gt_index][1]
            distance = math.hypot(prediction.x - gt.x, prediction.y - gt.y)
            if distance < best_distance:
                best_gt_index = gt_index
                best_distance = distance
        matched = (
            best_gt_index is not None and best_distance < distance_threshold_m
        )
        gt = None
        if matched:
            taken.add((frame_index, best_gt_index))
            gt = frame.gt[best_gt_index][1]
        velocity_quality = {}
        if (
            matched
            and prediction.velocity_x is not None
            and prediction.velocity_y is not None
        ):
            motion_state = _causal_motion_state(
                frame.gt[best_gt_index][0], frame.timestamp
            )
            if motion_state is not None:
                gt_cosine = math.cos(motion_state.yaw)
                gt_sine = math.sin(motion_state.yaw)
                gt_velocity_x = motion_state.speed * gt_cosine
                gt_velocity_y = motion_state.speed * gt_sine
                error_x = prediction.velocity_x - gt_velocity_x
                error_y = prediction.velocity_y - gt_velocity_y
                velocity_quality = {
                    "predicted_speed_mps": math.hypot(
                        prediction.velocity_x, prediction.velocity_y
                    ),
                    "gt_speed_mps": abs(motion_state.speed),
                    "velocity_l2_error_mps": math.hypot(error_x, error_y),
                    "speed_error_mps": abs(
                        math.hypot(prediction.velocity_x, prediction.velocity_y)
                        - abs(motion_state.speed)
                    ),
                    "longitudinal_velocity_error_mps": abs(
                        gt_cosine * error_x + gt_sine * error_y
                    ),
                    "lateral_velocity_error_mps": abs(
                        -gt_sine * error_x + gt_cosine * error_y
                    ),
                }
        rows.append(
            {
                "score": float(score),
                "frame_index": frame_index,
                "prediction_index": prediction_index,
                "tp": bool(matched),
                "distance_m": float(best_distance) if matched else None,
                "scale_error": _scale_error(gt, prediction) if matched else None,
                "orientation_error_rad": (
                    _orientation_error(gt, prediction) if matched else None
                ),
                **velocity_quality,
            }
        )
    return positives, rows


def _ranked_bev_iou_matches(frames, class_name, iou_threshold):
    positives = sum(
        sum(segment.class_name == class_name for segment, _ in frame.gt)
        for frame in frames
    )
    ranked = []
    for frame_index, frame in enumerate(frames):
        ranked.extend(
            (prediction.score, frame_index, prediction_index)
            for prediction_index, prediction in enumerate(frame.predictions)
            if prediction.class_name == class_name
        )
    ranked.sort(key=lambda item: item[0], reverse=True)

    taken = set()
    rows = []
    for score, frame_index, prediction_index in ranked:
        frame = frames[frame_index]
        prediction = frame.predictions[prediction_index]
        prediction_box = (
            prediction.x,
            prediction.y,
            prediction.length,
            prediction.width,
            prediction.yaw,
        )
        candidates = [
            gt_index
            for gt_index, (segment, _) in enumerate(frame.gt)
            if segment.class_name == class_name and (frame_index, gt_index) not in taken
        ]
        best_gt_index = None
        best_iou = -math.inf
        for gt_index in candidates:
            gt = frame.gt[gt_index][1]
            value = oriented_bev_iou(
                (gt.x, gt.y, gt.length, gt.width, gt.yaw), prediction_box
            )
            if value > best_iou:
                best_gt_index = gt_index
                best_iou = value
        matched = best_gt_index is not None and best_iou >= iou_threshold
        if matched:
            taken.add((frame_index, best_gt_index))
        rows.append(
            {
                "score": float(score),
                "frame_index": frame_index,
                "prediction_index": prediction_index,
                "tp": bool(matched),
                "bev_iou": float(best_iou) if matched else None,
            }
        )
    return positives, rows


def _precision_recall(positives, rows):
    if not rows:
        return (
            np.zeros((0,), dtype=np.float64),
            np.zeros((0,), dtype=np.float64),
        )
    tp = np.cumsum([float(row["tp"]) for row in rows])
    fp = np.cumsum([float(not row["tp"]) for row in rows])
    recall = tp / max(1, positives)
    precision = tp / np.maximum(tp + fp, 1.0e-12)
    return recall, precision


def _nuscenes_style_ap(positives, rows, min_recall=0.1, min_precision=0.1):
    if positives <= 0:
        return None
    recall, precision = _precision_recall(positives, rows)
    recall_grid = np.linspace(0.0, 1.0, 101)
    if len(recall):
        interpolated = np.interp(recall_grid, recall, precision, right=0.0)
    else:
        interpolated = np.zeros_like(recall_grid)
    first = round(100 * min_recall) + 1
    adjusted = np.maximum(0.0, interpolated[first:] - min_precision)
    return float(np.mean(adjusted) / (1.0 - min_precision))


def _operating_point(positives, rows, score_threshold):
    selected = [row for row in rows if row["score"] >= score_threshold]
    tp = sum(row["tp"] for row in selected)
    fp = len(selected) - tp
    fn = max(0, positives - tp)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / positives if positives else None
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if recall is not None and precision + recall
        else 0.0 if recall is not None else None
    )
    return {
        "score_threshold": float(score_threshold),
        "tp": int(tp),
        "fp": int(fp),
        "fn": int(fn),
        "precision": float(precision),
        "recall": float(recall) if recall is not None else None,
        "f1": float(f1) if f1 is not None else None,
    }


def _best_operating_point(positives, rows):
    if positives <= 0 or not rows:
        return None
    recall, precision = _precision_recall(positives, rows)
    denominator = recall + precision
    f1 = np.divide(
        2.0 * recall * precision,
        denominator,
        out=np.zeros_like(denominator),
        where=denominator > 0.0,
    )
    index = int(np.argmax(f1))
    return {
        "score_threshold": float(rows[index]["score"]),
        "precision": float(precision[index]),
        "recall": float(recall[index]),
        "f1": float(f1[index]),
    }


def _distance_slice(frames, lower_m, upper_m):
    output = []
    for frame in frames:
        gt = tuple(
            item
            for item in frame.gt
            if lower_m
            <= math.hypot(item[1].x - frame.ego.x, item[1].y - frame.ego.y)
            < upper_m
        )
        predictions = tuple(
            item
            for item in frame.predictions
            if lower_m
            <= math.hypot(item.x - frame.ego.x, item.y - frame.ego.y)
            < upper_m
        )
        output.append(
            EvaluationFrame(frame.timestamp, frame.ego, gt, predictions)
        )
    return tuple(output)


def evaluate_detection_frames(
    frames,
    *,
    class_names=CLASS_NAMES,
    center_thresholds_m=CENTER_THRESHOLDS_M,
    operating_distance_threshold_m=1.0,
    production_score_threshold=0.1,
    min_recall=0.1,
    min_precision=0.1,
):
    def evaluate_slice(selected_frames, include_quality):
        class_results = {}
        valid_mean_aps = []
        for class_name in class_names:
            threshold_results = {}
            aps = []
            positives = 0
            prediction_count = 0
            for threshold in center_thresholds_m:
                positives, rows = _ranked_matches(
                    selected_frames, class_name, threshold
                )
                prediction_count = len(rows)
                ap = _nuscenes_style_ap(
                    positives,
                    rows,
                    min_recall=min_recall,
                    min_precision=min_precision,
                )
                threshold_results[f"{threshold:g}"] = {
                    "ap": ap,
                    "gt": int(positives),
                    "predictions": int(prediction_count),
                }
                if ap is not None:
                    aps.append(ap)

            op_positives, op_rows = _ranked_matches(
                selected_frames, class_name, operating_distance_threshold_m
            )
            result = {
                "gt": int(positives),
                "predictions": int(prediction_count),
                "center_ap": threshold_results,
                "mean_ap": float(np.mean(aps)) if aps else None,
                "production_operating_point_1m": _operating_point(
                    op_positives, op_rows, production_score_threshold
                ),
                "best_operating_point_1m": _best_operating_point(
                    op_positives, op_rows
                ),
            }
            bev_iou_ap = {}
            for threshold in BEV_IOU_THRESHOLDS:
                iou_positives, iou_rows = _ranked_bev_iou_matches(
                    selected_frames, class_name, threshold
                )
                bev_iou_ap[f"{threshold:g}"] = {
                    "ap": _nuscenes_style_ap(
                        iou_positives,
                        iou_rows,
                        min_recall=min_recall,
                        min_precision=min_precision,
                    ),
                    "gt": int(iou_positives),
                    "predictions": len(iou_rows),
                }
            result["bev_iou_ap"] = bev_iou_ap
            if result["mean_ap"] is not None:
                valid_mean_aps.append(result["mean_ap"])
            if include_quality:
                quality_positives, quality_rows = _ranked_matches(
                    selected_frames, class_name, 2.0
                )
                matched = [row for row in quality_rows if row["tp"]]
                result["tp_quality_2m"] = {
                    "matched": len(matched),
                    "mATE_m": (
                        float(np.mean([row["distance_m"] for row in matched]))
                        if matched
                        else None
                    ),
                    "mASE": (
                        float(np.mean([row["scale_error"] for row in matched]))
                        if matched
                        else None
                    ),
                    "mAOE_rad": (
                        float(
                            np.mean(
                                [row["orientation_error_rad"] for row in matched]
                            )
                        )
                        if matched
                        else None
                    ),
                    "center_error_m": statistics(
                        [row["distance_m"] for row in matched]
                    ),
                    "velocity_l2_error_mps": statistics(
                        [
                            row["velocity_l2_error_mps"]
                            for row in matched
                            if "velocity_l2_error_mps" in row
                        ]
                    ),
                    "predicted_speed_mps": statistics(
                        [
                            row["predicted_speed_mps"]
                            for row in matched
                            if "predicted_speed_mps" in row
                        ]
                    ),
                    "gt_speed_mps": statistics(
                        [
                            row["gt_speed_mps"]
                            for row in matched
                            if "gt_speed_mps" in row
                        ]
                    ),
                    "speed_error_mps": statistics(
                        [
                            row["speed_error_mps"]
                            for row in matched
                            if "speed_error_mps" in row
                        ]
                    ),
                    "longitudinal_velocity_error_mps": statistics(
                        [
                            row["longitudinal_velocity_error_mps"]
                            for row in matched
                            if "longitudinal_velocity_error_mps" in row
                        ]
                    ),
                    "lateral_velocity_error_mps": statistics(
                        [
                            row["lateral_velocity_error_mps"]
                            for row in matched
                            if "lateral_velocity_error_mps" in row
                        ]
                    ),
                    "gt": int(quality_positives),
                }
            class_results[class_name] = result
        return {
            "classes": class_results,
            "mean_ap": (
                float(np.mean(valid_mean_aps)) if valid_mean_aps else None
            ),
            "classes_with_gt": sum(
                result["gt"] > 0 for result in class_results.values()
            ),
        }

    output = {
        "overall": evaluate_slice(frames, include_quality=True),
        "distance_bins_m": {},
    }
    for lower, upper in DISTANCE_BINS_M:
        output["distance_bins_m"][f"{lower:g}-{upper:g}"] = evaluate_slice(
            _distance_slice(frames, lower, upper), include_quality=False
        )
    return output
