#!/usr/bin/env python3
"""Compare Competition raw yaw-rate, IMU yaw-rate, and heading derivative.

The legacy ``EgoVehicleStatus`` message cannot carry Competition packet angular
velocity. New recordings must include ``/competition/raw_twist``, published by
the native UDP receiver, in addition to ``/Competition_topic`` and
``/sensor/imu``. This tool is diagnostic only and never changes fusion inputs.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag


def stamp(message, bag_time):
    if hasattr(message, "header") and not message.header.stamp.is_zero():
        return message.header.stamp.to_sec()
    return bag_time.to_sec()


def unique_sorted(rows):
    values = np.asarray(rows, dtype=float)
    values = values[np.argsort(values[:, 0])]
    _, indices = np.unique(values[:, 0], return_index=True)
    return values[np.sort(indices)]


def derivative_local_regression(times, angles, window):
    if window < 3 or window % 2 == 0:
        raise ValueError("heading derivative window must be odd and >= 3")
    result = np.full(times.shape, np.nan, dtype=float)
    half = window // 2
    for index in range(half, len(times) - half):
        subset = slice(index - half, index + half + 1)
        relative = times[subset] - times[index]
        if relative[-1] - relative[0] <= 0.0:
            continue
        result[index] = np.polyfit(relative, angles[subset], 1)[0]
    return result


def summary(errors):
    values = np.abs(np.asarray(errors, dtype=float))
    values = values[np.isfinite(values)]
    return {
        "count": int(values.size),
        "mae_radps": float(np.mean(values)),
        "p50_radps": float(np.percentile(values, 50)),
        "p95_radps": float(np.percentile(values, 95)),
        "rmse_radps": float(np.sqrt(np.mean(values**2))),
    }


def compare(reference_times, reference, source, lag_sec=0.0):
    query = reference_times + lag_sec
    valid = (
        np.isfinite(reference)
        & (query >= source[0, 0])
        & (query <= source[-1, 0])
    )
    measured = np.interp(query[valid], source[:, 0], source[:, 1])
    expected = reference[valid]
    moving = (np.abs(measured) + np.abs(expected)) > 0.01
    measured = measured[moving]
    expected = expected[moving]
    correlation = float(np.corrcoef(expected, measured)[0, 1])
    return summary(measured - expected), correlation


def analyze(path, window, max_lag_sec, lag_step_sec, competition_yaw_scale):
    heading = []
    competition_rate = []
    imu_rate = []
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, bag_time in bag.read_messages(
            topics=(
                "/Competition_topic",
                "/competition/raw_twist",
                "/sensor/imu",
            )
        ):
            time_sec = stamp(message, bag_time)
            if topic == "/Competition_topic":
                heading.append((time_sec, math.radians(float(message.heading))))
            elif topic == "/competition/raw_twist":
                competition_rate.append(
                    (
                        time_sec,
                        float(message.twist.angular.z) * competition_yaw_scale,
                    )
                )
            else:
                imu_rate.append((time_sec, float(message.angular_velocity.z)))

    missing = []
    if not heading:
        missing.append("/Competition_topic")
    if not imu_rate:
        missing.append("/sensor/imu")
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(missing))

    heading = unique_sorted(heading)
    imu_rate = unique_sorted(imu_rate)
    heading[:, 1] = np.unwrap(heading[:, 1])
    derivative = derivative_local_regression(heading[:, 0], heading[:, 1], window)

    lags = np.arange(-max_lag_sec, max_lag_sec + 0.5 * lag_step_sec, lag_step_sec)
    imu_trials = []
    competition_trials = []
    if competition_rate:
        competition_rate = unique_sorted(competition_rate)
    for lag in lags:
        imu_error, imu_corr = compare(heading[:, 0], derivative, imu_rate, lag)
        imu_trials.append((imu_error["rmse_radps"], -imu_corr, lag))
        if len(competition_rate):
            comp_error, comp_corr = compare(
                heading[:, 0], derivative, competition_rate, lag
            )
            competition_trials.append((comp_error["rmse_radps"], -comp_corr, lag))

    imu_best = min(imu_trials)
    imu_vs_heading, imu_corr = compare(
        heading[:, 0], derivative, imu_rate, imu_best[2]
    )
    result = {
        "bag": str(path.resolve()),
        "production_changed": False,
        "heading_derivative_window": window,
        "competition_raw_yaw_scale": competition_yaw_scale,
        "competition_raw_twist_available": bool(len(competition_rate)),
        "imu_vs_heading": {
            "best_lag_sec": float(imu_best[2]),
            "correlation": imu_corr,
            "error": imu_vs_heading,
        },
        "decision_rule": (
            "Do not fuse both as independent measurements. Select a replacement "
            "candidate only after repeated MR and LAN bags agree on sign, unit, "
            "lag, and heading-derivative error."
        ),
    }
    if competition_trials:
        competition_best = min(competition_trials)
        comp_vs_heading, comp_corr = compare(
            heading[:, 0], derivative, competition_rate, competition_best[2]
        )
        comp_imu_trials = []
        for lag in lags:
            error, correlation = compare(
                competition_rate[:, 0], competition_rate[:, 1], imu_rate, lag
            )
            comp_imu_trials.append((error["rmse_radps"], -correlation, lag))
        comp_imu_best = min(comp_imu_trials)
        comp_vs_imu, comp_imu_corr = compare(
            competition_rate[:, 0],
            competition_rate[:, 1],
            imu_rate,
            comp_imu_best[2],
        )
        result["competition_vs_heading"] = {
            "best_lag_sec": float(competition_best[2]),
            "correlation": comp_corr,
            "error": comp_vs_heading,
        }
        result["competition_vs_imu"] = {
            "relative_lag_sec": float(comp_imu_best[2]),
            "correlation": comp_imu_corr,
            "error": comp_vs_imu,
        }
    else:
        result["blocked_comparison"] = (
            "Record /competition/raw_twist before comparing Competition and IMU yaw-rate."
        )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--heading-window", type=int, default=7)
    parser.add_argument("--max-lag-sec", type=float, default=0.10)
    parser.add_argument("--lag-step-sec", type=float, default=0.002)
    parser.add_argument(
        "--competition-yaw-scale",
        type=float,
        default=-math.pi / 180.0,
        help="native clockwise deg/s to ROS rad/s; default=-pi/180",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.max_lag_sec < 0.0 or args.lag_step_sec <= 0.0:
        raise ValueError("lag range and step must be positive")
    if not math.isfinite(args.competition_yaw_scale) or args.competition_yaw_scale == 0.0:
        raise ValueError("--competition-yaw-scale must be finite and non-zero")
    result = analyze(
        args.bag,
        args.heading_window,
        args.max_lag_sec,
        args.lag_step_sec,
        args.competition_yaw_scale,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n")
    print(rendered)


if __name__ == "__main__":
    main()
