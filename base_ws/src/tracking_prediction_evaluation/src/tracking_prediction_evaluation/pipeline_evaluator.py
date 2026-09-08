"""End-to-end tracking and prediction evaluation against MORAI ground truth."""

from __future__ import annotations

import bisect
import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from scipy.optimize import linear_sum_assignment

from .evaluator import estimate_motion_state
from .trajectory import TrajectoryPoint, TrajectorySegment, interpolate_point


VEHICLE_LABELS = frozenset((1, 2, 3, 4, 5, 6))


@dataclass(frozen=True)
class EgoPoint:
    timestamp: float
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class PredictedPathEstimate:
    points: tuple[tuple[float, float], ...]
    step_sec: float
    confidence: float


@dataclass(frozen=True)
class ObjectEstimate:
    object_id: bytes
    x: float
    y: float
    yaw: float = 0.0
    speed: float = 0.0
    predicted_path: tuple[tuple[float, float], ...] = ()
    path_step_sec: float = 0.0
    predicted_paths: tuple[PredictedPathEstimate, ...] = ()


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
            points.append(
                EgoPoint(
                    timestamp,
                    x,
                    y,
                    (math.radians(float(message.heading)) + transform_yaw + math.pi)
                    % (2.0 * math.pi)
                    - math.pi,
                )
            )
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
    yaw_delta = (after.yaw - before.yaw + math.pi) % (2.0 * math.pi) - math.pi
    return EgoPoint(
        timestamp,
        before.x + ratio * (after.x - before.x),
        before.y + ratio * (after.y - before.y),
        before.yaw + ratio * yaw_delta,
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


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _predicted_paths(obj):
    paths = obj.kinematics.predicted_paths
    if not paths:
        return ()
    return tuple(
        PredictedPathEstimate(
            points=tuple(
                (float(pose.position.x), float(pose.position.y))
                for pose in path.path
            ),
            step_sec=duration_seconds(path.time_step),
            confidence=float(path.confidence),
        )
        for path in paths
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
                    paths = _predicted_paths(obj)
                    top_path = (
                        max(paths, key=lambda path: path.confidence)
                        if paths
                        else PredictedPathEstimate((), 0.0, 0.0)
                    )
                    estimates.append(
                        ObjectEstimate(
                            object_id=uuid_bytes(obj),
                            x=float(pose.position.x),
                            y=float(pose.position.y),
                            yaw=quaternion_yaw(pose.orientation),
                            speed=float(
                                obj.kinematics.initial_twist_with_covariance.twist.linear.x
                            ),
                            predicted_path=top_path.points,
                            path_step_sec=top_path.step_sec,
                            predicted_paths=paths,
                        )
                    )
            target = tracking_frames if topic == tracking_topic else prediction_frames
            target[timestamp] = tuple(estimates)
    return tracking_frames, prediction_frames


def read_detection_frames(bag_path, detection_topic):
    """Read vehicle detections in their source sensor frame."""
    import rosbag

    frames = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_timestamp in bag.read_messages(topics=[detection_topic]):
            timestamp = message_timestamp(message, bag_timestamp)
            estimates = []
            for obj in message.objects:
                if dominant_label(obj) not in VEHICLE_LABELS:
                    continue
                pose = obj.kinematics.pose_with_covariance.pose
                estimates.append(
                    ObjectEstimate(
                        object_id=b"",
                        x=float(pose.position.x),
                        y=float(pose.position.y),
                        yaw=quaternion_yaw(pose.orientation),
                    )
                )
            frames[timestamp] = tuple(estimates)
    return frames


def transform_sensor_estimates_to_map(
    estimates: Sequence[ObjectEstimate],
    ego: EgoPoint,
    sensor_to_ego_x_m: float,
    sensor_to_ego_y_m: float,
):
    """Transform detector-frame estimates using the synchronized MORAI Ego pose."""
    cosine = math.cos(ego.yaw)
    sine = math.sin(ego.yaw)
    transformed = []
    for estimate in estimates:
        local_x = sensor_to_ego_x_m + estimate.x
        local_y = sensor_to_ego_y_m + estimate.y
        transformed.append(
            ObjectEstimate(
                object_id=estimate.object_id,
                x=ego.x + cosine * local_x - sine * local_y,
                y=ego.y + sine * local_x + cosine * local_y,
                yaw=(estimate.yaw + ego.yaw + math.pi) % (2.0 * math.pi) - math.pi,
            )
        )
    return tuple(transformed)


def _absolute_yaw_error_deg(estimate_yaw: float, gt_yaw: float) -> float:
    delta = (estimate_yaw - gt_yaw + math.pi) % (2.0 * math.pi) - math.pi
    return abs(math.degrees(delta))


def _signed_position_error(estimate: ObjectEstimate, gt: TrajectoryPoint):
    """Return estimate-minus-GT error in GT longitudinal/lateral axes."""
    dx = estimate.x - gt.x
    dy = estimate.y - gt.y
    cosine = math.cos(gt.yaw)
    sine = math.sin(gt.yaw)
    return (
        dx * cosine + dy * sine,
        -dx * sine + dy * cosine,
    )


def _signed_statistics(values: Iterable[float]):
    values = list(values)
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(array.mean()),
        "p05": float(np.percentile(array, 5)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "minimum": float(array.min()),
        "maximum": float(array.max()),
    }


def _range_bucket(distance_m: float) -> str:
    if distance_m < 10.0:
        return "0_10m"
    if distance_m < 20.0:
        return "10_20m"
    return "20_30m"


def _motion_bucket(speed_mps: float, yaw_rate_rps: float) -> str:
    if speed_mps < 1.0:
        return "slow_under_1mps"
    if abs(yaw_rate_rps) >= 0.05:
        return "turning"
    return "straight"


def _causal_motion_state(segment: TrajectorySegment, timestamp: float):
    times = [point.timestamp for point in segment.points]
    right = bisect.bisect_right(times, timestamp + 1.0e-9)
    left = bisect.bisect_left(times, timestamp - 0.5)
    return estimate_motion_state(segment.points[left:right])


def evaluate_detection_tracking_yaw_flow(
    segments: Sequence[TrajectorySegment],
    ego_points: Sequence[EgoPoint],
    detection_frames,
    tracking_frames,
    maximum_range_m=30.0,
    maximum_detection_association_distance_m=3.0,
    maximum_tracking_association_distance_m=2.0,
    maximum_interpolation_bracket_sec=0.15,
    sensor_to_ego_x_m=0.855,
    sensor_to_ego_y_m=0.0,
):
    """Measure whether pi-yaw ambiguity is inherited or corrected by tracking.

    Detection and tracking estimates are associated independently to the same
    MORAI GT actor at the same header timestamp. This avoids assuming detector
    object order or tracker UUID continuity.
    """
    counts = defaultdict(int)
    detection_errors = []
    tracking_errors = []
    stage_center_errors = {"detection": [], "tracking": []}
    stage_longitudinal_errors = {"detection": [], "tracking": []}
    stage_lateral_errors = {"detection": [], "tracking": []}
    stage_yaw_errors = {"detection": [], "tracking": []}
    stage_range_errors = {
        "detection": defaultdict(list),
        "tracking": defaultdict(list),
    }
    stage_motion_errors = {
        "detection": defaultdict(list),
        "tracking": defaultdict(list),
    }
    stage_motion_longitudinal_errors = {
        "detection": defaultdict(list),
        "tracking": defaultdict(list),
    }
    stage_equivalent_lag_sec = {"detection": [], "tracking": []}
    paired_center_change = []
    timestamps = sorted(set(detection_frames) & set(tracking_frames))
    for timestamp in timestamps:
        ego = interpolate_ego(ego_points, timestamp, maximum_interpolation_bracket_sec)
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
        detections = transform_sensor_estimates_to_map(
            detection_frames[timestamp],
            ego,
            sensor_to_ego_x_m,
            sensor_to_ego_y_m,
        )
        detections = tuple(
            item
            for item in detections
            if math.hypot(item.x - ego.x, item.y - ego.y) <= maximum_range_m
        )
        tracks = tuple(
            item
            for item in tracking_frames[timestamp]
            if math.hypot(item.x - ego.x, item.y - ego.y) <= maximum_range_m
        )
        detection_matches, _, _ = match_estimates(
            gt_items, detections, maximum_detection_association_distance_m
        )
        tracking_matches, _, _ = match_estimates(
            gt_items, tracks, maximum_tracking_association_distance_m
        )
        detection_by_gt = {gt: estimate for gt, estimate, _ in detection_matches}
        tracking_by_gt = {gt: estimate for gt, estimate, _ in tracking_matches}

        for stage, estimates, matches in (
            ("detection", detections, detection_matches),
            ("tracking", tracks, tracking_matches),
        ):
            for gt_index, estimate_index, distance in matches:
                gt_point = gt_items[gt_index][1]
                estimate = estimates[estimate_index]
                longitudinal, lateral = _signed_position_error(estimate, gt_point)
                yaw_error = _absolute_yaw_error_deg(estimate.yaw, gt_point.yaw)
                stage_center_errors[stage].append(distance)
                stage_longitudinal_errors[stage].append(longitudinal)
                stage_lateral_errors[stage].append(lateral)
                stage_yaw_errors[stage].append(yaw_error)
                gt_range = math.hypot(gt_point.x - ego.x, gt_point.y - ego.y)
                stage_range_errors[stage][_range_bucket(gt_range)].append(distance)
                motion_state = _causal_motion_state(gt_items[gt_index][0], timestamp)
                if motion_state is not None:
                    motion_bucket = _motion_bucket(
                        abs(motion_state.speed), motion_state.yaw_rate
                    )
                    stage_motion_errors[stage][motion_bucket].append(distance)
                    stage_motion_longitudinal_errors[stage][motion_bucket].append(
                        longitudinal
                    )
                    if abs(motion_state.speed) >= 1.0:
                        stage_equivalent_lag_sec[stage].append(
                            -longitudinal / abs(motion_state.speed)
                        )

        for gt_index in sorted(set(detection_by_gt) & set(tracking_by_gt)):
            gt_yaw = gt_items[gt_index][1].yaw
            detection_error = _absolute_yaw_error_deg(
                detections[detection_by_gt[gt_index]].yaw, gt_yaw
            )
            tracking_error = _absolute_yaw_error_deg(
                tracks[tracking_by_gt[gt_index]].yaw, gt_yaw
            )
            detection_errors.append(detection_error)
            tracking_errors.append(tracking_error)
            detection_distance = math.hypot(
                detections[detection_by_gt[gt_index]].x - gt_items[gt_index][1].x,
                detections[detection_by_gt[gt_index]].y - gt_items[gt_index][1].y,
            )
            tracking_distance = math.hypot(
                tracks[tracking_by_gt[gt_index]].x - gt_items[gt_index][1].x,
                tracks[tracking_by_gt[gt_index]].y - gt_items[gt_index][1].y,
            )
            paired_center_change.append(tracking_distance - detection_distance)
            detection_flip = detection_error > 90.0
            tracking_flip = tracking_error > 90.0
            if detection_flip and tracking_flip:
                counts["detection_flip_to_tracking_flip"] += 1
            elif detection_flip:
                counts["detection_flip_to_tracking_correct"] += 1
            elif tracking_flip:
                counts["detection_correct_to_tracking_flip"] += 1
            else:
                counts["detection_correct_to_tracking_correct"] += 1

    paired = len(detection_errors)
    counts["paired_objects"] = paired
    improved_count = sum(value < 0.0 for value in paired_center_change)
    return {
        "counts": dict(counts),
        "detection_absolute_yaw_error_deg": statistics(detection_errors),
        "tracking_absolute_yaw_error_deg": statistics(tracking_errors),
        "detection_yaw_flip_rate_over_90deg": (
            float(np.mean(np.asarray(detection_errors) > 90.0))
            if detection_errors
            else None
        ),
        "tracking_yaw_flip_rate_over_90deg": (
            float(np.mean(np.asarray(tracking_errors) > 90.0))
            if tracking_errors
            else None
        ),
        "tracker_correction_rate_given_detection_flip": (
            counts["detection_flip_to_tracking_correct"]
            / max(
                1,
                counts["detection_flip_to_tracking_correct"]
                + counts["detection_flip_to_tracking_flip"],
            )
        ),
        "tracker_regression_rate_given_detection_correct": (
            counts["detection_correct_to_tracking_flip"]
            / max(
                1,
                counts["detection_correct_to_tracking_correct"]
                + counts["detection_correct_to_tracking_flip"],
            )
        ),
        "state_error_by_stage": {
            stage: {
                "center_distance_m": statistics(stage_center_errors[stage]),
                "signed_longitudinal_error_m": _signed_statistics(
                    stage_longitudinal_errors[stage]
                ),
                "signed_lateral_error_m": _signed_statistics(
                    stage_lateral_errors[stage]
                ),
                "absolute_yaw_error_deg": statistics(stage_yaw_errors[stage]),
                "center_distance_by_gt_range_m": {
                    bucket: statistics(values)
                    for bucket, values in sorted(stage_range_errors[stage].items())
                },
                "center_distance_by_gt_motion": {
                    bucket: statistics(values)
                    for bucket, values in sorted(stage_motion_errors[stage].items())
                },
                "signed_longitudinal_error_by_gt_motion_m": {
                    bucket: _signed_statistics(values)
                    for bucket, values in sorted(
                        stage_motion_longitudinal_errors[stage].items()
                    )
                },
                "longitudinal_equivalent_lag_sec": _signed_statistics(
                    stage_equivalent_lag_sec[stage]
                ),
            }
            for stage in ("detection", "tracking")
        },
        "paired_tracking_minus_detection_center_error_m": _signed_statistics(
            paired_center_change
        ),
        "paired_tracking_center_improvement_rate": (
            improved_count / paired if paired else None
        ),
    }


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


def _sample_predicted_path(path, step: float, horizon: float):
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


def _path_position(estimate: ObjectEstimate, horizon: float):
    return _sample_predicted_path(
        estimate.predicted_path,
        estimate.path_step_sec,
        horizon,
    )


def _estimate_paths(estimate: ObjectEstimate):
    if estimate.predicted_paths:
        return estimate.predicted_paths
    if estimate.predicted_path and estimate.path_step_sec > 0.0:
        return (
            PredictedPathEstimate(
                estimate.predicted_path,
                estimate.path_step_sec,
                1.0,
            ),
        )
    return ()


def _path_errors(
    path: PredictedPathEstimate,
    segment: TrajectorySegment,
    timestamp: float,
    horizon: float,
    maximum_interpolation_bracket_sec: float,
    gt_translation_xy=(0.0, 0.0),
):
    """Return (ADE, FDE, maximum displacement) for one trajectory hypothesis."""
    if path.step_sec <= 0.0:
        return None
    sample_count = int(math.floor(horizon / path.step_sec + 1.0e-9))
    sample_times = [path.step_sec * index for index in range(1, sample_count + 1)]
    if not sample_times or sample_times[-1] < horizon - 1.0e-6:
        sample_times.append(horizon)
    errors = []
    for offset in sample_times:
        predicted = _sample_predicted_path(path.points, path.step_sec, offset)
        future = interpolate_point(
            segment.points,
            timestamp + offset,
            maximum_interpolation_bracket_sec,
        )
        if predicted is None or future is None:
            return None
        target_x = future.x + gt_translation_xy[0]
        target_y = future.y + gt_translation_xy[1]
        errors.append(math.hypot(predicted[0] - target_x, predicted[1] - target_y))
    return float(np.mean(errors)), errors[-1], max(errors)


def _path_final_signed_error(
    path: PredictedPathEstimate,
    segment: TrajectorySegment,
    timestamp: float,
    horizon: float,
    maximum_interpolation_bracket_sec: float,
    gt_translation_xy=(0.0, 0.0),
):
    """Project final displacement error into the future GT heading frame.

    Positive longitudinal error means that the prediction is ahead of GT.
    Positive lateral error means that it is left of GT.  Translation is kept
    optional so the same convention can later be applied to origin-aligned
    diagnostics without changing the absolute production metric.
    """
    predicted = _sample_predicted_path(path.points, path.step_sec, horizon)
    future = interpolate_point(
        segment.points,
        timestamp + horizon,
        maximum_interpolation_bracket_sec,
    )
    if predicted is None or future is None:
        return None
    error_x = predicted[0] - (future.x + gt_translation_xy[0])
    error_y = predicted[1] - (future.y + gt_translation_xy[1])
    cosine = math.cos(future.yaw)
    sine = math.sin(future.yaw)
    return (
        error_x * cosine + error_y * sine,
        -error_x * sine + error_y * cosine,
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
    gt_reference="object-status",
):
    tracking_counts = defaultdict(int)
    tracking_errors = []
    tracking_speed_errors = []
    tracking_velocity_l2_errors = []
    tracking_yaw_errors = []
    tracking_longitudinal_speeds = []
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
            tracking_longitudinal_speeds.append(estimate.speed)
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
                estimate_vx = estimate.speed * math.cos(estimate.yaw)
                estimate_vy = estimate.speed * math.sin(estimate.yaw)
                state_vx = state.speed * math.cos(state.yaw)
                state_vy = state.speed * math.sin(state.yaw)
                tracking_velocity_l2_errors.append(
                    math.hypot(estimate_vx - state_vx, estimate_vy - state_vy)
                )
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
    horizon_top1_ade = defaultdict(list)
    horizon_min_fde = defaultdict(list)
    horizon_min_ade = defaultdict(list)
    horizon_min_max_displacement = defaultdict(list)
    horizon_top1_signed_longitudinal_fde = defaultdict(list)
    horizon_top1_signed_lateral_fde = defaultdict(list)
    horizon_min_k_signed_longitudinal_fde = defaultdict(list)
    horizon_min_k_signed_lateral_fde = defaultdict(list)
    horizon_best_confidence = defaultdict(list)
    horizon_brier_min_fde = defaultdict(list)
    horizon_origin_aligned_top1_fde = defaultdict(list)
    horizon_origin_aligned_top1_ade = defaultdict(list)
    horizon_origin_aligned_min_fde = defaultdict(list)
    horizon_origin_aligned_min_ade = defaultdict(list)
    horizon_eligible = defaultdict(int)
    horizon_matched = defaultdict(int)
    horizon_best_of_k_matched = defaultdict(int)
    prediction_fp = 0
    prediction_frames_used = 0
    prediction_path_counts = []
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
        prediction_path_counts.extend(
            len(_estimate_paths(visible_estimates[index]))
            for index in matched_estimates
        )
        match_by_gt = {gt_index: estimate_index for gt_index, estimate_index, _ in matches}
        for gt_index, (segment, origin) in enumerate(gt_items):
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
                estimate = visible_estimates[estimate_index]
                # Object_topic and detector boxes may use different physical
                # reference points (for example vehicle model origin versus
                # geometric box center).  Translation-aligning GT at t=0
                # isolates future-motion quality without pretending that the
                # current-state center error has disappeared.
                gt_translation = (
                    estimate.x - origin.x,
                    estimate.y - origin.y,
                )
                paths = _estimate_paths(estimate)
                if not paths:
                    continue
                ordered_paths = sorted(
                    paths,
                    key=lambda path: path.confidence,
                    reverse=True,
                )
                candidate_errors = []
                for path in ordered_paths:
                    errors = _path_errors(
                        path,
                        segment,
                        timestamp,
                        horizon,
                        maximum_interpolation_bracket_sec,
                    )
                    aligned_errors = _path_errors(
                        path,
                        segment,
                        timestamp,
                        horizon,
                        maximum_interpolation_bracket_sec,
                        gt_translation,
                    )
                    if errors is not None and aligned_errors is not None:
                        candidate_errors.append((path, errors, aligned_errors))
                if not candidate_errors:
                    continue

                top_path, (top_ade, top_fde, _top_max_error), (
                    aligned_top_ade,
                    aligned_top_fde,
                    _aligned_top_max,
                ) = candidate_errors[0]
                top_signed_error = _path_final_signed_error(
                    top_path,
                    segment,
                    timestamp,
                    horizon,
                    maximum_interpolation_bracket_sec,
                )
                if top_signed_error is None:
                    continue
                horizon_matched[horizon] += 1
                horizon_errors[horizon].append(top_fde)
                horizon_top1_ade[horizon].append(top_ade)
                horizon_top1_signed_longitudinal_fde[horizon].append(
                    top_signed_error[0]
                )
                horizon_top1_signed_lateral_fde[horizon].append(
                    top_signed_error[1]
                )
                horizon_origin_aligned_top1_fde[horizon].append(aligned_top_fde)
                horizon_origin_aligned_top1_ade[horizon].append(aligned_top_ade)

                best_fde_path, (_best_fde_ade, best_fde, _best_fde_max), _ = min(
                    candidate_errors,
                    key=lambda item: item[1][1],
                )
                best_fde_signed_error = _path_final_signed_error(
                    best_fde_path,
                    segment,
                    timestamp,
                    horizon,
                    maximum_interpolation_bracket_sec,
                )
                if best_fde_signed_error is None:
                    continue
                _best_ade_path, (best_ade, _best_ade_fde, _best_ade_max), _ = min(
                    candidate_errors,
                    key=lambda item: item[1][0],
                )
                _best_max_path, (
                    _best_max_ade,
                    _best_max_fde,
                    best_max_error,
                ), _ = min(candidate_errors, key=lambda item: item[1][2])
                _aligned_fde_path, _, (
                    _aligned_best_fde_ade,
                    aligned_best_fde,
                    _aligned_best_fde_max,
                ) = min(candidate_errors, key=lambda item: item[2][1])
                _aligned_ade_path, _, (
                    aligned_best_ade,
                    _aligned_best_ade_fde,
                    _aligned_best_ade_max,
                ) = min(candidate_errors, key=lambda item: item[2][0])
                horizon_best_of_k_matched[horizon] += 1
                horizon_min_fde[horizon].append(best_fde)
                horizon_min_k_signed_longitudinal_fde[horizon].append(
                    best_fde_signed_error[0]
                )
                horizon_min_k_signed_lateral_fde[horizon].append(
                    best_fde_signed_error[1]
                )
                horizon_min_ade[horizon].append(best_ade)
                horizon_min_max_displacement[horizon].append(best_max_error)
                horizon_origin_aligned_min_fde[horizon].append(aligned_best_fde)
                horizon_origin_aligned_min_ade[horizon].append(aligned_best_ade)
                best_confidence = max(0.0, min(1.0, best_fde_path.confidence))
                horizon_best_confidence[horizon].append(best_confidence)
                horizon_brier_min_fde[horizon].append(
                    best_fde + (1.0 - best_confidence) ** 2
                )

    prediction = {}
    multimodal_horizons = {}
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
        best_matched = horizon_best_of_k_matched[horizon]
        top1_fde = horizon_errors[horizon]
        min_fde = horizon_min_fde[horizon]
        min_max_displacement = horizon_min_max_displacement[horizon]
        multimodal_horizons[f"{horizon:g}"] = {
            "eligible_gt": eligible,
            "top1_matched_predictions": matched,
            "best_of_k_matched_predictions": best_matched,
            "top1_coverage": matched / eligible if eligible else 0.0,
            "best_of_k_coverage": best_matched / eligible if eligible else 0.0,
            "top1_ade_m": statistics(horizon_top1_ade[horizon]),
            "top1_fde_m": statistics(top1_fde),
            "top1_fde_components_gt_heading": {
                "description": (
                    "Final displacement projected into future GT heading; "
                    "positive longitudinal=ahead, positive lateral=left"
                ),
                "signed_longitudinal_m": _signed_statistics(
                    horizon_top1_signed_longitudinal_fde[horizon]
                ),
                "absolute_longitudinal_m": statistics(
                    abs(value)
                    for value in horizon_top1_signed_longitudinal_fde[horizon]
                ),
                "signed_lateral_m": _signed_statistics(
                    horizon_top1_signed_lateral_fde[horizon]
                ),
                "absolute_lateral_m": statistics(
                    abs(value)
                    for value in horizon_top1_signed_lateral_fde[horizon]
                ),
            },
            "min_ade_k_m": statistics(horizon_min_ade[horizon]),
            "min_fde_k_m": statistics(min_fde),
            "min_fde_k_components_gt_heading": {
                "description": (
                    "GT-closest candidate final displacement projected into "
                    "future GT heading"
                ),
                "signed_longitudinal_m": _signed_statistics(
                    horizon_min_k_signed_longitudinal_fde[horizon]
                ),
                "absolute_longitudinal_m": statistics(
                    abs(value)
                    for value in horizon_min_k_signed_longitudinal_fde[horizon]
                ),
                "signed_lateral_m": _signed_statistics(
                    horizon_min_k_signed_lateral_fde[horizon]
                ),
                "absolute_lateral_m": statistics(
                    abs(value)
                    for value in horizon_min_k_signed_lateral_fde[horizon]
                ),
            },
            "top1_fde_miss_rate_1m": (
                float(np.mean(np.asarray(top1_fde) > 1.0)) if top1_fde else None
            ),
            "top1_fde_miss_rate_2m": (
                float(np.mean(np.asarray(top1_fde) > 2.0)) if top1_fde else None
            ),
            "min_fde_k_miss_rate_1m": (
                float(np.mean(np.asarray(min_fde) > 1.0)) if min_fde else None
            ),
            "min_fde_k_miss_rate_2m": (
                float(np.mean(np.asarray(min_fde) > 2.0)) if min_fde else None
            ),
            "min_k_max_displacement_miss_rate_2m": (
                float(np.mean(np.asarray(min_max_displacement) > 2.0))
                if min_max_displacement
                else None
            ),
            "best_fde_path_confidence": statistics(
                horizon_best_confidence[horizon]
            ),
            "brier_min_fde_m": statistics(horizon_brier_min_fde[horizon]),
            "origin_aligned": {
                "description": (
                    "GT future translated by the matched estimate-minus-GT "
                    "offset at the prediction origin; isolates motion error"
                ),
                "top1_ade_m": statistics(
                    horizon_origin_aligned_top1_ade[horizon]
                ),
                "top1_fde_m": statistics(
                    horizon_origin_aligned_top1_fde[horizon]
                ),
                "min_ade_k_m": statistics(
                    horizon_origin_aligned_min_ade[horizon]
                ),
                "min_fde_k_m": statistics(
                    horizon_origin_aligned_min_fde[horizon]
                ),
            },
        }

    return {
        "evaluation": {
            "class": "vehicle",
            "gt_reference": gt_reference,
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
            "object_status_reference_distance_m": (
                statistics(tracking_errors)
                if gt_reference == "object-status"
                else {
                    "count": 0,
                    "reason": "available in object_status_reference_evaluation",
                }
            ),
            "motp_center_error_m": statistics(tracking_errors),
            "motp_center_error_caveat": (
                "Validated bbox-center distance for the supported calibrated "
                "vehicle subset"
                if gt_reference == "bbox-center-calibrated-v1"
                else "Compatibility alias only: MORAI ObjectStatus.position "
                "and detector bbox center have different empirical "
                "longitudinal reference points, so this is not validated "
                "bbox-center MOTP"
            ),
            "bbox_center_absolute_distance_m": (
                statistics(tracking_errors)
                if gt_reference == "bbox-center-calibrated-v1"
                else {"count": 0, "reason": "bbox-center GT not selected"}
            ),
            "absolute_speed_error_mps": statistics(tracking_speed_errors),
            "velocity_l2_error_mps": statistics(tracking_velocity_l2_errors),
            "absolute_yaw_error_deg": statistics(tracking_yaw_errors),
            "yaw_flip_rate_over_90deg": (
                float(np.mean(np.asarray(tracking_yaw_errors) > 90.0))
                if tracking_yaw_errors
                else None
            ),
            "negative_longitudinal_speed_rate": (
                float(np.mean(np.asarray(tracking_longitudinal_speeds) < -0.1))
                if tracking_longitudinal_speeds
                else None
            ),
        },
        "prediction": {
            "frames": prediction_frames_used,
            "unmatched_output_objects": prediction_fp,
            "fde_m": prediction,
            "multimodal": {
                "matched_object_path_count": statistics(prediction_path_counts),
                "multi_path_object_rate": (
                    float(np.mean(np.asarray(prediction_path_counts) > 1))
                    if prediction_path_counts
                    else None
                ),
                "horizons": multimodal_horizons,
            },
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
