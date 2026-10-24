#!/usr/bin/env python3
"""Measure estimator drift and correction around recorded GNSS gaps."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

import analyze_localization_stability_bag as stability


def absolute_summary(values):
    return stability.scalar_summary(values, absolute=True)


def pose_errors(state, ego, transform, heading_offset_deg):
    state = state[np.argsort(state[:, 0])]
    ego = ego[np.argsort(ego[:, 0])]
    tx, ty, transform_yaw = transform
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    ego_x = tx + cosine * ego[:, 1] - sine * ego[:, 2]
    ego_y = ty + sine * ego[:, 1] + cosine * ego[:, 2]
    ego_yaw = np.unwrap(
        np.radians(ego[:, 3])
        + math.radians(heading_offset_deg)
        + transform_yaw
    )
    times = state[:, 0]
    gt_x = np.interp(times, ego[:, 0], ego_x)
    gt_y = np.interp(times, ego[:, 0], ego_y)
    gt_yaw = np.interp(times, ego[:, 0], ego_yaw)
    speed = np.interp(times, ego[:, 0], ego[:, 4])
    dx = state[:, 1] - gt_x
    dy = state[:, 2] - gt_y
    return {
        "state": state,
        "times": times,
        "speed": speed,
        "xy": np.hypot(dx, dy),
        "longitudinal": dx * np.cos(gt_yaw) + dy * np.sin(gt_yaw),
        "lateral": -dx * np.sin(gt_yaw) + dy * np.cos(gt_yaw),
    }


def window_summary(errors, start, end, speed_threshold):
    mask = (
        (errors["times"] >= start)
        & (errors["times"] < end)
        & (np.abs(errors["speed"]) > speed_threshold)
    )
    return {
        "start_sec": float(start),
        "end_sec": float(end),
        "moving_sample_count": int(np.count_nonzero(mask)),
        "xy_error_m": absolute_summary(errors["xy"][mask]),
        "longitudinal_error_m": absolute_summary(
            errors["longitudinal"][mask]
        ),
        "lateral_error_m": absolute_summary(errors["lateral"][mask]),
    }


def recovery_step_summary(errors, reacquisition_time, window_sec=1.5):
    times = errors["times"]
    mask = (
        (times >= reacquisition_time - 0.5)
        & (times <= reacquisition_time + window_sec)
    )
    indices = np.flatnonzero(mask)
    if indices.size < 2:
        return {"pair_count": 0}
    consecutive = indices[1:] == indices[:-1] + 1
    indices = indices[:-1][consecutive]
    dt = times[indices + 1] - times[indices]
    valid = (dt > 1.0e-4) & (dt < 0.1)
    indices = indices[valid]
    state = errors["state"]
    position_step = np.hypot(
        state[indices + 1, 1] - state[indices, 1],
        state[indices + 1, 2] - state[indices, 2],
    )
    lateral_error_step = (
        errors["lateral"][indices + 1] - errors["lateral"][indices]
    )
    return {
        "pair_count": int(indices.size),
        "state_position_step_m": absolute_summary(position_step),
        "lateral_error_step_m": absolute_summary(lateral_error_step),
    }


def analyze(path, heading_offset_deg, speed_threshold, gap_threshold):
    series, transform, bag_start, _ = stability.parse_bag(path)
    if transform is None:
        raise RuntimeError("map -> morai_map static TF is missing")
    ego = np.asarray(series["/Ego_topic"], dtype=float)
    gnss = np.asarray(series["/localization/gnss/odom"], dtype=float)
    if not ego.size or not gnss.size:
        raise RuntimeError("Ego oracle or GNSS odometry is missing")

    states = {}
    for label, topic in (
        ("global", "/localization/kinematic_state"),
        ("smooth_control", "/localization/control_kinematic_state"),
    ):
        values = np.asarray(series[topic], dtype=float)
        if values.size:
            states[label] = pose_errors(
                values, ego, transform, heading_offset_deg
            )

    gnss_times = np.sort(gnss[:, 0])
    gaps = np.diff(gnss_times)
    outages = []
    for index in np.flatnonzero(gaps > gap_threshold):
        start = float(gnss_times[index])
        end = float(gnss_times[index + 1])
        result = {
            "start_bag_offset_sec": start - bag_start,
            "end_bag_offset_sec": end - bag_start,
            "duration_sec": end - start,
            "states": {},
        }
        for label, errors in states.items():
            result["states"][label] = {
                "pre_2s": window_summary(
                    errors, start - 2.0, start, speed_threshold
                ),
                "outage": window_summary(
                    errors, start, end, speed_threshold
                ),
                "recovery_0_to_1s": window_summary(
                    errors, end, end + 1.0, speed_threshold
                ),
                "recovery_1_to_4s": window_summary(
                    errors, end + 1.0, end + 4.0, speed_threshold
                ),
                "recovery_steps": recovery_step_summary(errors, end),
            }
        outages.append(result)

    return {
        "bag": str(path.resolve()),
        "configuration": {
            "heading_offset_deg": heading_offset_deg,
            "moving_speed_threshold_mps": speed_threshold,
            "gnss_gap_threshold_sec": gap_threshold,
        },
        "outage_count": len(outages),
        "outages": outages,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347)
    parser.add_argument("--moving-speed-threshold", type=float, default=1.0)
    parser.add_argument("--gnss-gap-threshold", type=float, default=0.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(
        args.bag,
        args.heading_offset_deg,
        args.moving_speed_threshold,
        args.gnss_gap_threshold,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n")
    print(rendered)


if __name__ == "__main__":
    main()
