"""End-to-end tracking and prediction evaluation against MORAI ground truth."""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from .trajectory import TrajectoryPoint, TrajectorySegment, interpolate_point


VEHICLE_LABELS = frozenset((1, 2, 3, 4, 5, 6))


@dataclass(frozen=True)
class EgoPoint:
    timestamp: float
    x: float
    y: float


@dataclass(frozen=True)
class ObjectEstimate:
    object_id: bytes
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0
    predicted_path: tuple[tuple[float, float], ...] = ()
    path_step_sec: float = 0.0


def message_timestamp(message, bag_timestamp) -> float:
    stamp = message.header.stamp
    if stamp and (stamp.secs != 0 or stamp.nsecs != 0):
        return stamp.to_sec()
    return bag_timestamp.to_sec()


def transform_xy(x: float, y: float, translation_xyz, yaw: float):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * x - sine * y + translation_xyz[0],
        sine * x + cosine * y + translation_xyz[1],
    )


def read_ego_points(bag_path, topic, translation_xyz, transform_yaw):
    import rosbag

    points = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_timestamp in bag.read_messages(topics=[topic]):
            timestamp = message_timestamp(message, bag_timestamp)
            x, y = transform_xy(
                float(message.position.x),
                float(message.position.y),
                translation_xyz,
                transform_yaw,
            )
            points.append(EgoPoint(timestamp, x, y))
    # Duplicate stamps occur in some MORAI bags. Keep one deterministic sample.
    return tuple({point.timestamp: point for point in points}.values())


def interpolate_ego(points: Sequence[EgoPoint], timestamp: float, maximum_gap: float):
    if not points:
        return None
    times = np.asarray([point.timestamp for point in points], dtype=np.float64)
    right = int(np.searchsorted(times, timestamp))
    if right < len(points) and abs(points[right].timestamp - timestamp) < 1.0e-9:
        return points[right]
    if right == 0 or right >= len(points):
        return None
    before = points[right - 1]
    after = points[right]
    dt = after.timestamp - before.timestamp
    if dt <= 0.0 or dt > maximum_gap:
        return None
    ratio = (timestamp - before.timestamp) / dt
    return EgoPoint(
        timestamp,
        before.x + ratio * (after.x - before.x),
        before.y + ratio * (after.y - before.y),
    )


def dominant_label(obj) -> int:
    if not obj.classification:
        return 0
    return int(max(obj.classification, key=lambda item: item.probability).label)


def uuid_bytes(obj) -> bytes:
    return bytes(bytearray(obj.object_id.uuid))


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _highest_confidence_path(obj):
    paths = obj.kinematics.predicted_paths
    if not paths:
        return (), 0.0
    selected = max(paths, key=lambda path: path.confidence)
    return (
        tuple((float(pose.position.x), float(pose.position.y)) for pose in selected.path),
        float(selected.time_step.to_sec()),
    )


def read_output_frames(bag_path, tracking_topic, prediction_topic):
    import rosbag

    tracking_frames = {}
    prediction_frames = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, message, bag_timestamp in bag.read_messages(
            topics=[tracking_topic, prediction_topic]
        ):
            timestamp = message_timestamp(message, bag_timestamp)
            estimates = []
            for obj in message.objects:
                if dominant_label(obj) not in VEHICLE_LABELS:
                    continue
                if topic == tracking_topic:
                    pose = obj.kinematics.pose_with_covariance.pose
                    estimates.append(
                        ObjectEstimate(
                            object_id=uuid_bytes(obj),
                            x=float(pose.position.x),
                            y=float(pose.position.y),
                            yaw=quaternion_yaw(pose.orientation),
                            speed=float(
                                obj.kinematics.twist_with_covariance.twist.linear.x
                            ),
                        )
                    )
                else:
                    pose = obj.kinematics.initial_pose_with_covariance.pose
                    path, step = _highest_confidence_path(obj)
                    estimates.append(
                        ObjectEstimate(
                            object_id=uuid_bytes(obj),
                            x=float(pose.position.x),
                            y=float(pose.position.y),
                            yaw=quaternion_yaw(pose.orientation),
                            speed=float(
                                obj.kinematics.initial_twist_with_covariance.twist.linear.x
                            ),
                            predicted_path=path,
                            path_step_sec=step,
                        )
                    )
            target = tracking_frames if topic == tracking_topic else prediction_frames
            target[timestamp] = tuple(estimates)
    return tracking_frames, prediction_frames


def active_gt(
    segments: Sequence[TrajectorySegment],
    timestamp: float,
    maximum_bracket_sec: float,
    ego: EgoPoint | None,
    maximum_range_m: float,
):
    output = []
    for segment in segments:
        point = interpolate_point(segment.points, timestamp, maximum_bracket_sec)
        if point is None:
            continue
        if ego is not None and math.hypot(point.x - ego.x, point.y - ego.y) > maximum_range_m:
            continue
        output.append((segment, point))
    return output


def match_estimates(gt_items, estimates, maximum_distance_m):
    if not gt_items or not estimates:
        return [], set(), set()
    distances = np.empty((len(gt_items), len(estimates)), dtype=np.float64)
    for row, (_, point) in enumerate(gt_items):
        for column, estimate in enumerate(estimates):
            distances[row, column] = math.hypot(point.x - estimate.x, point.y - estimate.y)
    rows, columns = linear_sum_assignment(distances)
    matches = []
    matched_gt = set()
    matched_estimates = set()
    for row, column in zip(rows, columns):
        if distances[row, column] > maximum_distance_m:
            continue
        matches.append((int(row), int(column), float(distances[row, column])))
        matched_gt.add(int(row))
        matched_estimates.add(int(column))
    return matches, matched_gt, matched_estimates


def statistics(values: Iterable[float]):
    values = list(values)
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(array.mean()),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "p50": float(np.percentile(array, 50)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(array.max()),
    }


def _path_position(estimate: ObjectEstimate, horizon: float):
    path = estimate.predicted_path
    step = estimate.path_step_sec
    if not path or step <= 0.0:
        return None
    index = horizon / step
    low = int(math.floor(index + 1.0e-9))
    high = int(math.ceil(index - 1.0e-9))
    if low < 0 or high >= len(path):
        return None
    if low == high:
        return path[low]
    ratio = index - low
    return (
        path[low][0] + ratio * (path[high][0] - path[low][0]),
        path[low][1] + ratio * (path[high][1] - path[low][1]),
    )


def is_cv_stable_horizon(
    segment: TrajectorySegment,
    origin_timestamp: float,
    horizon: float,
    maximum_longitudinal_acceleration_mps2: float = 0.5,
    maximum_interpolation_bracket_sec: float = 0.15,
) -> bool:
    """Return whether future GT remains in the constant-velocity slice.

    This is an offline evaluation label, not information available to the
    runtime predictor.  A sample is CV-stable only when the absolute MORAI GT
    longitudinal acceleration remains within the threshold from the
    prediction origin through the requested horizon.  Interior measurements
    and interpolated endpoints are checked independently for each horizon.
    """
    end_timestamp = origin_timestamp + horizon
    origin = interpolate_point(
        segment.points,
        origin_timestamp,
        maximum_interpolation_bracket_sec,
    )
    future = interpolate_point(
        segment.points,
        end_timestamp,
        maximum_interpolation_bracket_sec,
    )
    if origin is None or future is None:
        return False
    points = [origin]
    points.extend(
        point
        for point in segment.points
        if origin_timestamp < point.timestamp < end_timestamp
    )
    points.append(future)
    threshold = maximum_longitudinal_acceleration_mps2 + 1.0e-9
    return all(
        math.isfinite(point.longitudinal_acceleration)
        and abs(point.longitudinal_acceleration) <= threshold
        for point in points
    )


def _prediction_sample_is_eligible(
    segment: TrajectorySegment,
    timestamp: float,
    horizon: float,
    prediction_motion_slice: str,
    maximum_longitudinal_acceleration_mps2: float,
    maximum_interpolation_bracket_sec: float,
) -> bool:
    if prediction_motion_slice == "all":
        return True
    if prediction_motion_slice == "cv-stable":
        return is_cv_stable_horizon(
            segment,
            timestamp,
            horizon,
            maximum_longitudinal_acceleration_mps2,
            maximum_interpolation_bracket_sec,
        )
    raise ValueError(f"unsupported prediction motion slice: {prediction_motion_slice}")


def evaluate_pipeline(
    segments: Sequence[TrajectorySegment],
    ego_points: Sequence[EgoPoint],
    tracking_frames,
    prediction_frames,
    horizons=(1.0, 2.0, 3.0),
    maximum_range_m=30.0,
    maximum_association_distance_m=2.0,
    maximum_interpolation_bracket_sec=0.15,
    prediction_motion_slice="all",
    maximum_longitudinal_acceleration_mps2=0.5,
):
    tracking_counts = defaultdict(int)
    tracking_errors = []
    tracking_speed_errors = []
    tracking_yaw_errors = []
    previous_track_by_gt = {}
    last_seen_by_gt = {}

    for timestamp, estimates in sorted(tracking_frames.items()):
        ego = interpolate_ego(ego_points, timestamp, maximum_interpolation_bracket_sec)
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        visible_estimates = tuple(
            estimate
            for estimate in estimates
            if ego is None
            or math.hypot(estimate.x - ego.x, estimate.y - ego.y) <= maximum_range_m
        )
        matches, matched_gt, matched_estimates = match_estimates(
            gt_items, visible_estimates, maximum_association_distance_m
        )
        tracking_counts["frames"] += 1
        tracking_counts["gt"] += len(gt_items)
        tracking_counts["tp"] += len(matches)
        tracking_counts["fn"] += len(gt_items) - len(matched_gt)
        tracking_counts["fp"] += len(visible_estimates) - len(matched_estimates)
        for gt_index, estimate_index, distance in matches:
            segment, _ = gt_items[gt_index]
            estimate = visible_estimates[estimate_index]
            tracking_errors.append(distance)
            from .evaluator import estimate_motion_state

            history = [
                point
                for point in segment.points
                if timestamp - 0.5 <= point.timestamp <= timestamp + 1.0e-9
            ]
            if history and abs(history[-1].timestamp - timestamp) > 1.0e-9:
                history.append(gt_items[gt_index][1])
            state = estimate_motion_state(history)
            if state is not None:
                tracking_speed_errors.append(abs(estimate.speed - state.speed))
                yaw_delta = (estimate.yaw - state.yaw + math.pi) % (2.0 * math.pi) - math.pi
                tracking_yaw_errors.append(abs(math.degrees(yaw_delta)))
            previous = previous_track_by_gt.get(segment.key)
            last_seen = last_seen_by_gt.get(segment.key, -math.inf)
            if previous is not None and previous != estimate.object_id and timestamp - last_seen <= 0.5:
                tracking_counts["id_switches"] += 1
            previous_track_by_gt[segment.key] = estimate.object_id
            last_seen_by_gt[segment.key] = timestamp

    tp = tracking_counts["tp"]
    fp = tracking_counts["fp"]
    fn = tracking_counts["fn"]
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    f1 = 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0
    mota = 1.0 - (fn + fp + tracking_counts["id_switches"]) / max(1, tracking_counts["gt"])

    horizon_errors = defaultdict(list)
    horizon_eligible = defaultdict(int)
    horizon_matched = defaultdict(int)
    prediction_fp = 0
    prediction_frames_used = 0
    for timestamp, estimates in sorted(prediction_frames.items()):
        ego = interpolate_ego(ego_points, timestamp, maximum_interpolation_bracket_sec)
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        visible_estimates = tuple(
            estimate
            for estimate in estimates
            if ego is None
            or math.hypot(estimate.x - ego.x, estimate.y - ego.y) <= maximum_range_m
        )
        matches, _, matched_estimates = match_estimates(
            gt_items, visible_estimates, maximum_association_distance_m
        )
        prediction_fp += len(visible_estimates) - len(matched_estimates)
        prediction_frames_used += 1
        match_by_gt = {gt_index: estimate_index for gt_index, estimate_index, _ in matches}
        for gt_index, (segment, _) in enumerate(gt_items):
            for horizon in horizons:
                future = interpolate_point(
                    segment.points,
                    timestamp + horizon,
                    maximum_interpolation_bracket_sec,
                )
                if future is None:
                    continue
                if not _prediction_sample_is_eligible(
                    segment,
                    timestamp,
                    horizon,
                    prediction_motion_slice,
                    maximum_longitudinal_acceleration_mps2,
                    maximum_interpolation_bracket_sec,
                ):
                    continue
                horizon_eligible[horizon] += 1
                estimate_index = match_by_gt.get(gt_index)
                if estimate_index is None:
                    continue
                predicted = _path_position(visible_estimates[estimate_index], horizon)
                if predicted is None:
                    continue
                horizon_matched[horizon] += 1
                horizon_errors[horizon].append(
                    math.hypot(predicted[0] - future.x, predicted[1] - future.y)
                )

    prediction = {}
    for horizon in horizons:
        eligible = horizon_eligible[horizon]
        matched = horizon_matched[horizon]
        stats = statistics(horizon_errors[horizon])
        stats.update(
            {
                "eligible_gt": eligible,
                "matched_predictions": matched,
                "coverage": matched / eligible if eligible else 0.0,
                "miss_rate_1m": (
                    float(np.mean(np.asarray(horizon_errors[horizon]) > 1.0))
                    if horizon_errors[horizon]
                    else None
                ),
                "miss_rate_2m": (
                    float(np.mean(np.asarray(horizon_errors[horizon]) > 2.0))
                    if horizon_errors[horizon]
                    else None
                ),
            }
        )
        prediction[f"{horizon:g}"] = stats

    return {
        "evaluation": {
            "class": "vehicle",
            "maximum_range_m": maximum_range_m,
            "maximum_association_distance_m": maximum_association_distance_m,
            "horizons_sec": list(horizons),
            "prediction_motion_slice": prediction_motion_slice,
            "maximum_longitudinal_acceleration_mps2": (
                maximum_longitudinal_acceleration_mps2
                if prediction_motion_slice == "cv-stable"
                else None
            ),
        },
        "tracking": {
            **dict(tracking_counts),
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "mota": mota,
            "motp_center_error_m": statistics(tracking_errors),
            "absolute_speed_error_mps": statistics(tracking_speed_errors),
            "absolute_yaw_error_deg": statistics(tracking_yaw_errors),
            "yaw_flip_rate_over_90deg": (
                float(np.mean(np.asarray(tracking_yaw_errors) > 90.0))
                if tracking_yaw_errors
                else None
            ),
        },
        "prediction": {
            "frames": prediction_frames_used,
            "unmatched_output_objects": prediction_fp,
            "fde_m": prediction,
        },
    }


def evaluate_oracle_predictor_at_timestamps(
    segments: Sequence[TrajectorySegment],
    ego_points: Sequence[EgoPoint],
    timestamps: Iterable[float],
    predictor,
    horizons=(1.0, 2.0, 3.0),
    maximum_range_m=30.0,
    history_sec=0.5,
    maximum_interpolation_bracket_sec=0.15,
    prediction_motion_slice="all",
    maximum_longitudinal_acceleration_mps2=0.5,
):
    """Evaluate a predictor with GT history at the exact pipeline timestamps.

    This creates the paired Object_topic upper bound for an end-to-end run:
    same timestamps, range gate, GT actors and future-availability rules as
    the runtime PointPainting evaluation.
    """
    from .evaluator import estimate_motion_state

    errors = defaultdict(list)
    eligible = defaultdict(int)
    predicted = defaultdict(int)
    for timestamp in sorted(set(timestamps)):
        ego = interpolate_ego(ego_points, timestamp, maximum_interpolation_bracket_sec)
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        for segment, origin in gt_items:
            history = [
                point
                for point in segment.points
                if timestamp - history_sec <= point.timestamp <= timestamp + 1.0e-9
            ]
            if not history or abs(history[-1].timestamp - timestamp) > 1.0e-9:
                history.append(origin)
            state = estimate_motion_state(history)
            for horizon in horizons:
                future = interpolate_point(
                    segment.points,
                    timestamp + horizon,
                    maximum_interpolation_bracket_sec,
                )
                if future is None:
                    continue
                if not _prediction_sample_is_eligible(
                    segment,
                    timestamp,
                    horizon,
                    prediction_motion_slice,
                    maximum_longitudinal_acceleration_mps2,
                    maximum_interpolation_bracket_sec,
                ):
                    continue
                eligible[horizon] += 1
                if state is None:
                    continue
                path = predictor.predict_path(state, (horizon,), segment.class_name)
                if not path:
                    continue
                predicted[horizon] += 1
                x, y, _ = path[0]
                errors[horizon].append(math.hypot(x - future.x, y - future.y))

    output = {}
    for horizon in horizons:
        stats = statistics(errors[horizon])
        stats.update(
            {
                "eligible_gt": eligible[horizon],
                "predicted": predicted[horizon],
                "coverage": predicted[horizon] / eligible[horizon]
                if eligible[horizon]
                else 0.0,
                "miss_rate_1m": float(np.mean(np.asarray(errors[horizon]) > 1.0))
                if errors[horizon]
                else None,
                "miss_rate_2m": float(np.mean(np.asarray(errors[horizon]) > 2.0))
                if errors[horizon]
                else None,
            }
        )
        output[f"{horizon:g}"] = stats
    return output
