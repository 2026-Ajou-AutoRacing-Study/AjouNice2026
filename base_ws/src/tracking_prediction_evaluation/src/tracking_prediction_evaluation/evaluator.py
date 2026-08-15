"""Causal E0 oracle prediction evaluation."""

from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from .models import MotionState, Predictor
from .trajectory import (
    TrajectoryPoint,
    TrajectorySegment,
    interpolate_point,
    normalize_angle,
    unwrap_angles,
)


@dataclass(frozen=True)
class EvaluationConfig:
    horizons: tuple[float, ...] = (0.5, 1.0, 1.5, 2.0, 3.0)
    trajectory_step_sec: float = 0.1
    evaluation_rate_hz: float = 10.0
    history_sec: float = 0.5
    maximum_interpolation_bracket_sec: float = 0.15
    miss_thresholds_m: tuple[float, ...] = (1.0, 2.0, 3.0)
    stationary_speed_mps: float = 0.25
    turning_yaw_rate_rps: float = math.radians(5.0)


@dataclass(frozen=True)
class SampleError:
    model: str
    segment_key: str
    class_name: str
    asset_name: str
    origin_timestamp: float
    horizon: float
    fde: float
    ade: float
    motion_bucket: str
    speed: float
    yaw_rate: float
    acceleration: float


def _linear_slope(times: np.ndarray, values: np.ndarray) -> float:
    centered = times - times.mean()
    denominator = float(np.dot(centered, centered))
    if denominator < 1.0e-10:
        return 0.0
    return float(np.dot(centered, values - values.mean()) / denominator)


def estimate_motion_state(
    history: Sequence[TrajectoryPoint],
) -> MotionState | None:
    if len(history) < 3:
        return None
    current = history[-1]
    times = np.asarray([point.timestamp for point in history], dtype=np.float64)
    times -= times[-1]
    headings = np.asarray(unwrap_angles([point.yaw for point in history]), dtype=np.float64)
    yaw_rate = _linear_slope(times, headings)

    # Existing bags disagree on the units of ObjectStatus.velocity (some are
    # m/s, others km/h). Fit a causal quadratic to GT positions instead. Its
    # derivative at t=0 gives current map velocity and its second derivative
    # gives acceleration; neither estimate sees a future sample.
    xs = np.asarray([point.x for point in history], dtype=np.float64)
    ys = np.asarray([point.y for point in history], dtype=np.float64)
    if float(times[-1] - times[0]) < 1.0e-3:
        return None
    degree = 2 if len(history) >= 5 else 1
    coefficients_x = np.polyfit(times, xs, degree)
    coefficients_y = np.polyfit(times, ys, degree)
    velocity_x = float(coefficients_x[-2])
    velocity_y = float(coefficients_y[-2])
    acceleration_x = float(2.0 * coefficients_x[0]) if degree == 2 else 0.0
    acceleration_y = float(2.0 * coefficients_y[0]) if degree == 2 else 0.0
    cosine = math.cos(current.yaw)
    sine = math.sin(current.yaw)
    speed = velocity_x * cosine + velocity_y * sine
    acceleration = acceleration_x * cosine + acceleration_y * sine
    return MotionState(
        x=current.x,
        y=current.y,
        yaw=current.yaw,
        speed=speed,
        yaw_rate=yaw_rate,
        acceleration=acceleration,
    )


def motion_bucket(state: MotionState, config: EvaluationConfig) -> str:
    if abs(state.speed) < config.stationary_speed_mps:
        return "stationary"
    if abs(state.yaw_rate) >= config.turning_yaw_rate_rps:
        return "turning"
    return "straight"


def _origin_indices(
    points: Sequence[TrajectoryPoint], evaluation_rate_hz: float
) -> Iterable[int]:
    interval = 1.0 / evaluation_rate_hz
    last_timestamp = -math.inf
    for index, point in enumerate(points):
        if point.timestamp - last_timestamp >= interval - 1.0e-6:
            yield index
            last_timestamp = point.timestamp


def evaluate_segments(
    segments: Sequence[TrajectorySegment],
    predictors: Sequence[Predictor],
    config: EvaluationConfig,
) -> list[SampleError]:
    samples: list[SampleError] = []
    maximum_horizon = max(config.horizons)
    step_count = int(round(maximum_horizon / config.trajectory_step_sec))
    path_offsets = [config.trajectory_step_sec * index for index in range(1, step_count + 1)]

    for segment in segments:
        points = segment.points
        for origin_index in _origin_indices(points, config.evaluation_rate_hz):
            origin = points[origin_index]
            history_start = origin.timestamp - config.history_sec
            history = [
                point
                for point in points[: origin_index + 1]
                if point.timestamp >= history_start
            ]
            state = estimate_motion_state(history)
            if state is None:
                continue

            future_gt: dict[float, TrajectoryPoint] = {}
            for offset in path_offsets:
                point = interpolate_point(
                    points,
                    origin.timestamp + offset,
                    config.maximum_interpolation_bracket_sec,
                )
                if point is not None:
                    future_gt[round(offset, 9)] = point

            for horizon in config.horizons:
                required_offsets = [
                    offset for offset in path_offsets if offset <= horizon + 1.0e-9
                ]
                if not required_offsets or any(
                    round(offset, 9) not in future_gt for offset in required_offsets
                ):
                    continue
                for predictor in predictors:
                    errors = []
                    for offset in required_offsets:
                        predicted_x, predicted_y, _ = predictor.predict(state, offset)
                        gt = future_gt[round(offset, 9)]
                        errors.append(math.hypot(predicted_x - gt.x, predicted_y - gt.y))
                    samples.append(
                        SampleError(
                            model=predictor.name,
                            segment_key=segment.key,
                            class_name=segment.class_name,
                            asset_name=segment.asset_name,
                            origin_timestamp=origin.timestamp,
                            horizon=horizon,
                            fde=errors[-1],
                            ade=float(np.mean(errors)),
                            motion_bucket=motion_bucket(state, config),
                            speed=state.speed,
                            yaw_rate=state.yaw_rate,
                            acceleration=state.acceleration,
                        )
                    )
    return samples


def _statistics(values: Sequence[float], thresholds: Sequence[float]) -> dict:
    array = np.asarray(values, dtype=np.float64)
    if not len(array):
        return {"count": 0}
    output = {
        "count": int(len(array)),
        "mean": float(array.mean()),
        "rmse": float(np.sqrt(np.mean(np.square(array)))),
        "p50": float(np.percentile(array, 50)),
        "p90": float(np.percentile(array, 90)),
        "p95": float(np.percentile(array, 95)),
        "maximum": float(array.max()),
    }
    for threshold in thresholds:
        output[f"miss_rate_{threshold:g}m"] = float(np.mean(array > threshold))
    return output


def build_summary(
    samples: Sequence[SampleError],
    config: EvaluationConfig,
    source: dict,
) -> dict:
    grouped: dict[tuple[str, str, str, float], list[SampleError]] = defaultdict(list)
    for sample in samples:
        for class_name, bucket in (
            (sample.class_name, sample.motion_bucket),
            (sample.class_name, "all"),
            ("all", sample.motion_bucket),
            ("all", "all"),
        ):
            grouped[(sample.model, class_name, bucket, sample.horizon)].append(sample)

    metrics: dict = {}
    for (model, class_name, bucket, horizon), group in sorted(grouped.items()):
        model_metrics = metrics.setdefault(model, {})
        class_metrics = model_metrics.setdefault(class_name, {})
        bucket_metrics = class_metrics.setdefault(bucket, {})
        bucket_metrics[f"{horizon:g}"] = {
            "fde": _statistics(
                [sample.fde for sample in group], config.miss_thresholds_m
            ),
            "ade": _statistics([sample.ade for sample in group], ()),
        }

    paired: dict = {}
    sample_lookup = {
        (
            sample.model,
            sample.segment_key,
            sample.origin_timestamp,
            sample.horizon,
        ): sample
        for sample in samples
    }
    model_names = sorted({sample.model for sample in samples})
    if "ctrv" in model_names:
        for challenger in (name for name in model_names if name != "ctrv"):
            for horizon in config.horizons:
                deltas = []
                wins = 0
                ties = 0
                for key, baseline in sample_lookup.items():
                    model, segment_key, timestamp, sample_horizon = key
                    if model != "ctrv" or sample_horizon != horizon:
                        continue
                    candidate = sample_lookup.get(
                        (challenger, segment_key, timestamp, sample_horizon)
                    )
                    if candidate is None:
                        continue
                    delta = candidate.fde - baseline.fde
                    deltas.append(delta)
                    if delta < -1.0e-9:
                        wins += 1
                    elif abs(delta) <= 1.0e-9:
                        ties += 1
                if deltas:
                    array = np.asarray(deltas)
                    paired.setdefault(f"{challenger}_vs_ctrv", {})[f"{horizon:g}"] = {
                        "count": int(len(array)),
                        "mean_fde_delta_m": float(array.mean()),
                        "p50_fde_delta_m": float(np.percentile(array, 50)),
                        "p90_fde_delta_m": float(np.percentile(array, 90)),
                        "challenger_win_rate": float(wins / len(array)),
                        "tie_rate": float(ties / len(array)),
                    }

    return {
        "schema_version": 1,
        "evaluation": "E0_oracle_prediction",
        "limitations": [
            "GT identity, pose, heading and velocity are used; tracking is not evaluated.",
            "Only causal GT history is used to estimate yaw rate and acceleration.",
            "Detector misses, association errors and runtime latency are excluded.",
        ],
        "source": source,
        "config": asdict(config),
        "sample_count": len(samples),
        "metrics": metrics,
        "paired_comparison": paired,
    }


def write_report(
    output_dir: str | Path,
    summary: dict,
    samples: Sequence[SampleError],
    worst_sample_count: int = 100,
) -> tuple[Path, Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    worst_path = output / "worst_samples.jsonl"
    worst = sorted(samples, key=lambda sample: sample.fde, reverse=True)[
        :worst_sample_count
    ]
    worst_path.write_text(
        "".join(json.dumps(asdict(sample), ensure_ascii=False) + "\n" for sample in worst),
        encoding="utf-8",
    )
    return summary_path, worst_path
