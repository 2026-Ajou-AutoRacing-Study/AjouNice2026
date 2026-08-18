"""Timestamp-alignment diagnostics for detector and tracker object states.

The diagnostic deliberately keeps the estimate/GT association fixed while
the GT reference time is swept.  Re-associating independently at every shift
can make a bad shift appear better merely by matching another nearby actor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from .pipeline_evaluator import (
    EgoPoint,
    ObjectEstimate,
    active_gt,
    interpolate_ego,
    match_estimates,
    statistics,
    transform_sensor_estimates_to_map,
)
from .trajectory import TrajectorySegment, interpolate_point, normalize_angle


@dataclass(frozen=True)
class AlignmentSample:
    timestamp: float
    segment: TrajectorySegment
    estimate: ObjectEstimate
    estimate_is_sensor_frame: bool = False
    sensor_to_ego_x_m: float = 0.0
    sensor_to_ego_y_m: float = 0.0


def _motion_basis(
    segment: TrajectorySegment,
    timestamp: float,
    maximum_interpolation_bracket_sec: float,
    derivative_half_window_sec: float = 0.05,
):
    """Return unit longitudinal/left vectors and centered GT speed."""
    before = interpolate_point(
        segment.points,
        timestamp - derivative_half_window_sec,
        maximum_interpolation_bracket_sec,
    )
    after = interpolate_point(
        segment.points,
        timestamp + derivative_half_window_sec,
        maximum_interpolation_bracket_sec,
    )
    center = interpolate_point(
        segment.points,
        timestamp,
        maximum_interpolation_bracket_sec,
    )
    if center is None:
        return None
    if before is not None and after is not None:
        dt = after.timestamp - before.timestamp
        dx = after.x - before.x
        dy = after.y - before.y
        distance = math.hypot(dx, dy)
        if dt > 1.0e-6 and distance > 1.0e-5:
            longitudinal = (dx / distance, dy / distance)
            return longitudinal, (-longitudinal[1], longitudinal[0]), distance / dt
    longitudinal = (math.cos(center.yaw), math.sin(center.yaw))
    return longitudinal, (-longitudinal[1], longitudinal[0]), 0.0


def build_alignment_samples(
    segments: Sequence[TrajectorySegment],
    ego_points: Sequence[EgoPoint],
    frames: Mapping[float, Sequence[ObjectEstimate]],
    *,
    estimates_are_sensor_frame: bool,
    sensor_to_ego_x_m: float = 0.855,
    sensor_to_ego_y_m: float = 0.0,
    maximum_range_m: float = 30.0,
    maximum_association_distance_m: float = 5.0,
    maximum_interpolation_bracket_sec: float = 0.15,
):
    """Associate estimates to GT once at their unshifted header timestamp."""
    samples = []
    counts = {
        "frames": 0,
        "gt": 0,
        "estimates": 0,
        "matches": 0,
        "missing_ego_frames": 0,
    }
    for timestamp, frame_estimates in sorted(frames.items()):
        ego = interpolate_ego(
            ego_points,
            timestamp,
            maximum_interpolation_bracket_sec,
        )
        if ego is None:
            counts["missing_ego_frames"] += 1
            continue
        source_estimates = tuple(frame_estimates)
        map_estimates = source_estimates
        if estimates_are_sensor_frame:
            map_estimates = transform_sensor_estimates_to_map(
                source_estimates,
                ego,
                sensor_to_ego_x_m,
                sensor_to_ego_y_m,
            )
        visible_pairs = tuple(
            (source, mapped)
            for source, mapped in zip(source_estimates, map_estimates)
            if math.hypot(mapped.x - ego.x, mapped.y - ego.y) <= maximum_range_m
        )
        visible_sources = tuple(source for source, _ in visible_pairs)
        estimates = tuple(mapped for _, mapped in visible_pairs)
        gt_items = active_gt(
            segments,
            timestamp,
            maximum_interpolation_bracket_sec,
            ego,
            maximum_range_m,
        )
        matches, _, _ = match_estimates(
            gt_items,
            estimates,
            maximum_association_distance_m,
        )
        counts["frames"] += 1
        counts["gt"] += len(gt_items)
        counts["estimates"] += len(estimates)
        counts["matches"] += len(matches)
        for gt_index, estimate_index, _ in matches:
            samples.append(
                AlignmentSample(
                    timestamp=timestamp,
                    segment=gt_items[gt_index][0],
                    estimate=(
                        visible_sources[estimate_index]
                        if estimates_are_sensor_frame
                        else estimates[estimate_index]
                    ),
                    estimate_is_sensor_frame=estimates_are_sensor_frame,
                    sensor_to_ego_x_m=sensor_to_ego_x_m,
                    sensor_to_ego_y_m=sensor_to_ego_y_m,
                )
            )
    return tuple(samples), counts


def evaluate_alignment_shift(
    samples: Sequence[AlignmentSample],
    shift_sec: float,
    maximum_interpolation_bracket_sec: float = 0.15,
    ego_points: Sequence[EgoPoint] = (),
):
    euclidean_errors = []
    longitudinal_errors = []
    lateral_errors = []
    speeds = []
    yaw_errors_deg = []
    per_actor = {}
    for sample in samples:
        gt_timestamp = sample.timestamp + shift_sec
        gt = interpolate_point(
            sample.segment.points,
            gt_timestamp,
            maximum_interpolation_bracket_sec,
        )
        basis = _motion_basis(
            sample.segment,
            gt_timestamp,
            maximum_interpolation_bracket_sec,
        )
        if gt is None or basis is None:
            continue
        longitudinal, left, speed = basis
        estimate_x = sample.estimate.x
        estimate_y = sample.estimate.y
        estimate_yaw = sample.estimate.yaw
        if sample.estimate_is_sensor_frame:
            ego = interpolate_ego(
                ego_points,
                gt_timestamp,
                maximum_interpolation_bracket_sec,
            )
            if ego is None:
                continue
            cosine = math.cos(ego.yaw)
            sine = math.sin(ego.yaw)
            local_x = sample.sensor_to_ego_x_m + estimate_x
            local_y = sample.sensor_to_ego_y_m + estimate_y
            estimate_x = ego.x + cosine * local_x - sine * local_y
            estimate_y = ego.y + sine * local_x + cosine * local_y
            estimate_yaw = normalize_angle(estimate_yaw + ego.yaw)
        dx = estimate_x - gt.x
        dy = estimate_y - gt.y
        euclidean = math.hypot(dx, dy)
        longitudinal_error = dx * longitudinal[0] + dy * longitudinal[1]
        lateral_error = dx * left[0] + dy * left[1]
        yaw_delta = (
            estimate_yaw - gt.yaw + math.pi
        ) % (2.0 * math.pi) - math.pi
        euclidean_errors.append(euclidean)
        longitudinal_errors.append(longitudinal_error)
        lateral_errors.append(lateral_error)
        speeds.append(speed)
        yaw_errors_deg.append(abs(math.degrees(yaw_delta)))
        per_actor.setdefault(sample.segment.key, []).append(euclidean)

    longitudinal_array = np.asarray(longitudinal_errors, dtype=np.float64)
    lateral_array = np.asarray(lateral_errors, dtype=np.float64)
    speed_array = np.asarray(speeds, dtype=np.float64)
    regression = {"count": 0}
    if len(speed_array) >= 2 and float(np.std(speed_array)) > 1.0e-6:
        slope, intercept = np.polyfit(speed_array, longitudinal_array, 1)
        predicted = slope * speed_array + intercept
        residual = longitudinal_array - predicted
        total = float(np.sum((longitudinal_array - longitudinal_array.mean()) ** 2))
        regression = {
            "count": int(len(speed_array)),
            "time_slope_sec": float(slope),
            "spatial_intercept_m": float(intercept),
            "r_squared": (
                float(1.0 - np.sum(residual * residual) / total)
                if total > 1.0e-12
                else None
            ),
        }

    actor_means = [float(np.mean(values)) for values in per_actor.values()]
    return {
        "shift_sec": float(shift_sec),
        "matched_samples": len(euclidean_errors),
        "euclidean_error_m": statistics(euclidean_errors),
        "signed_longitudinal_error_m": {
            **statistics(longitudinal_errors),
            "absolute_mean": (
                float(np.mean(np.abs(longitudinal_array)))
                if len(longitudinal_array)
                else None
            ),
        },
        "signed_lateral_error_m": {
            **statistics(lateral_errors),
            "absolute_mean": (
                float(np.mean(np.abs(lateral_array)))
                if len(lateral_array)
                else None
            ),
        },
        "gt_speed_mps": statistics(speeds),
        "absolute_yaw_error_deg": statistics(yaw_errors_deg),
        "per_actor_euclidean_mean_m": statistics(actor_means),
        "longitudinal_error_vs_speed": regression,
    }


def sweep_alignment_shifts(
    samples: Sequence[AlignmentSample],
    shifts_sec: Sequence[float],
    maximum_interpolation_bracket_sec: float = 0.15,
    ego_points: Sequence[EgoPoint] = (),
):
    shifts_sec = tuple(float(value) for value in shifts_sec)
    if not shifts_sec:
        return {
            "input_sample_count": len(samples),
            "common_sample_count": 0,
            "sweep": [],
            "best": {},
        }
    minimum_shift = min(shifts_sec)
    maximum_shift = max(shifts_sec)
    # Use exactly the same estimate/actor pairs at every sweep value.  The
    # extra 50 ms on each side is required by the centered motion derivative.
    common_samples = tuple(
        sample
        for sample in samples
        if interpolate_point(
            sample.segment.points,
            sample.timestamp + minimum_shift - 0.05,
            maximum_interpolation_bracket_sec,
        )
        is not None
        and interpolate_point(
            sample.segment.points,
            sample.timestamp + maximum_shift + 0.05,
            maximum_interpolation_bracket_sec,
        )
        is not None
        and (
            not sample.estimate_is_sensor_frame
            or (
                interpolate_ego(
                    ego_points,
                    sample.timestamp + minimum_shift,
                    maximum_interpolation_bracket_sec,
                )
                is not None
                and interpolate_ego(
                    ego_points,
                    sample.timestamp + maximum_shift,
                    maximum_interpolation_bracket_sec,
                )
                is not None
            )
        )
    )
    rows = [
        evaluate_alignment_shift(
            common_samples,
            shift,
            maximum_interpolation_bracket_sec,
            ego_points,
        )
        for shift in shifts_sec
    ]
    valid = [
        row
        for row in rows
        if row["euclidean_error_m"].get("count", 0) > 0
    ]
    if not valid:
        return {
            "input_sample_count": len(samples),
            "common_sample_count": len(common_samples),
            "sweep": rows,
            "best": {},
        }

    def best_by(path):
        first, second = path
        return min(
            valid,
            key=lambda row: row[first][second]
            if row[first].get(second) is not None
            else math.inf,
        )

    return {
        "input_sample_count": len(samples),
        "common_sample_count": len(common_samples),
        "sweep": rows,
        "best": {
            "euclidean_mean": best_by(("euclidean_error_m", "mean")),
            "euclidean_rmse": best_by(("euclidean_error_m", "rmse")),
            "longitudinal_absolute_mean": best_by(
                ("signed_longitudinal_error_m", "absolute_mean")
            ),
        },
    }


def summarize_alignment_by_actor(
    samples: Sequence[AlignmentSample],
    shifts_sec: Sequence[float],
    maximum_interpolation_bracket_sec: float = 0.15,
    minimum_samples: int = 8,
    ego_points: Sequence[EgoPoint] = (),
):
    """Return compact per-trajectory optima for reference-point diagnosis.

    A global shift can look convincing even when each vehicle merely has a
    different fixed box-origin offset.  Reporting each reconstructed actor
    separately makes that failure mode visible without writing every sample.
    """
    grouped = {}
    for sample in samples:
        grouped.setdefault(sample.segment.key, []).append(sample)

    output = []
    for actor_samples in grouped.values():
        if len(actor_samples) < minimum_samples:
            continue
        result = sweep_alignment_shifts(
            actor_samples,
            shifts_sec,
            maximum_interpolation_bracket_sec,
            ego_points,
        )
        if not result["best"]:
            continue
        zero = min(result["sweep"], key=lambda row: abs(row["shift_sec"]))
        best = result["best"]["euclidean_mean"]
        segment = actor_samples[0].segment
        output.append(
            {
                "segment_key": segment.key,
                "asset_name": segment.asset_name,
                "sample_count": int(best["matched_samples"]),
                "gt_speed_mean_mps": zero["gt_speed_mps"].get("mean"),
                "zero_euclidean_mean_m": zero["euclidean_error_m"].get("mean"),
                "zero_longitudinal_mean_m": zero[
                    "signed_longitudinal_error_m"
                ].get("mean"),
                "zero_lateral_mean_m": zero["signed_lateral_error_m"].get(
                    "mean"
                ),
                "best_shift_sec": best["shift_sec"],
                "best_euclidean_mean_m": best["euclidean_error_m"].get("mean"),
                "best_longitudinal_mean_m": best[
                    "signed_longitudinal_error_m"
                ].get("mean"),
                "best_lateral_mean_m": best["signed_lateral_error_m"].get(
                    "mean"
                ),
            }
        )
    return sorted(output, key=lambda row: row["segment_key"])
