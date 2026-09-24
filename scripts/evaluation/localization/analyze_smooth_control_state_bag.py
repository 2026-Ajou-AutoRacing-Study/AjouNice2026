#!/usr/bin/env python3
"""Offline A/B for smoothing only the controller-facing map alignment.

This is deliberately a counterfactual replay over recorded global/local EKF
outputs. It does not claim closed-loop improvement; it filters candidates
before a MORAI controller A/B is worth running.
"""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import rosbag


ROOT = Path(__file__).resolve().parents[3]
SMOOTHER_SCRIPT = (
    ROOT
    / "base_ws/src/localization/localization_manager/scripts"
    / "smooth_control_state_publisher.py"
)
SPEC = importlib.util.spec_from_file_location("smooth_control_state", str(SMOOTHER_SCRIPT))
SMOOTHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SMOOTHER)

TOPICS = (
    "/Ego_topic",
    "/localization/kinematic_state",
    "/localization/odometry/local",
    "/tf_static",
)


def message_stamp(message, bag_time):
    if hasattr(message, "header") and not message.header.stamp.is_zero():
        return message.header.stamp.to_sec()
    return bag_time.to_sec()


def percentile_summary(values):
    values = np.abs(np.asarray(values, dtype=float))
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def parse_bag(path):
    ego = []
    global_state = []
    local_state = []
    map_from_morai = None
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, bag_time in bag.read_messages(topics=TOPICS):
            if topic == "/tf_static":
                for transform in message.transforms:
                    if (
                        transform.header.frame_id.lstrip("/") == "map"
                        and transform.child_frame_id.lstrip("/") == "morai_map"
                    ):
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        map_from_morai = (
                            float(translation.x),
                            float(translation.y),
                            SMOOTHER.yaw_from_quaternion(rotation),
                        )
                continue

            stamp = message_stamp(message, bag_time)
            if topic == "/Ego_topic":
                ego.append(
                    (
                        stamp,
                        float(message.position.x),
                        float(message.position.y),
                        float(message.heading),
                        float(message.velocity.x),
                    )
                )
                continue

            pose = message.pose.pose
            row = (
                stamp,
                float(pose.position.x),
                float(pose.position.y),
                SMOOTHER.yaw_from_quaternion(pose.orientation),
            )
            if topic == "/localization/kinematic_state":
                global_state.append(row)
            else:
                local_state.append(row)

    if map_from_morai is None:
        raise RuntimeError("map -> morai_map static TF is missing")
    if not ego or not global_state or not local_state:
        raise RuntimeError("Ego/global/local state is required")
    return (
        np.asarray(ego, dtype=float),
        np.asarray(global_state, dtype=float),
        np.asarray(local_state, dtype=float),
        map_from_morai,
    )


def sorted_unique(series):
    series = series[np.argsort(series[:, 0])]
    _, unique_indices = np.unique(series[:, 0], return_index=True)
    return series[np.sort(unique_indices)]


def interpolate_pose(source, query_times):
    source = sorted_unique(source)
    return np.column_stack(
        (
            np.interp(query_times, source[:, 0], source[:, 1]),
            np.interp(query_times, source[:, 0], source[:, 2]),
            np.interp(query_times, source[:, 0], np.unwrap(source[:, 3])),
        )
    )


def pose_metrics(times, poses, oracle, speed, moving_threshold):
    moving = np.abs(speed) > moving_threshold
    dx = poses[:, 0] - oracle[:, 0]
    dy = poses[:, 1] - oracle[:, 1]
    oracle_yaw = oracle[:, 2]
    longitudinal = dx * np.cos(oracle_yaw) + dy * np.sin(oracle_yaw)
    lateral = -dx * np.sin(oracle_yaw) + dy * np.cos(oracle_yaw)
    yaw_error = np.degrees(
        np.arctan2(
            np.sin(poses[:, 2] - oracle_yaw),
            np.cos(poses[:, 2] - oracle_yaw),
        )
    )
    dt = np.diff(times)
    adjacent = (dt > 1.0e-4) & (dt < 0.1) & moving[1:] & moving[:-1]
    return {
        "moving_samples": int(np.count_nonzero(moving)),
        "lateral_error_m": percentile_summary(lateral[moving]),
        "longitudinal_error_m": percentile_summary(longitudinal[moving]),
        "yaw_error_deg": percentile_summary(yaw_error[moving]),
        "lateral_frame_jump_m": percentile_summary(np.diff(lateral)[adjacent]),
        "longitudinal_frame_jump_m": percentile_summary(
            np.diff(longitudinal)[adjacent]
        ),
    }


def smooth_candidate(times, global_pose, local_pose, tau_sec):
    raw_alignment = np.asarray(
        [
            SMOOTHER.planar_alignment(map_pose, odom_pose)
            for map_pose, odom_pose in zip(global_pose, local_pose)
        ],
        dtype=float,
    )
    smoother = SMOOTHER.PlanarAlignmentSmoother(
        translation_tau_sec=tau_sec,
        yaw_tau_sec=tau_sec,
        reset_gap_sec=0.5,
    )
    filtered_alignment = []
    for stamp, target in zip(times, raw_alignment):
        filtered_alignment.append(smoother.update(target, stamp))
    filtered_alignment = np.asarray(filtered_alignment, dtype=float)
    poses = np.asarray(
        [
            SMOOTHER.compose_planar(alignment, odom_pose)
            for alignment, odom_pose in zip(filtered_alignment, local_pose)
        ],
        dtype=float,
    )
    raw_step = np.hypot(np.diff(raw_alignment[:, 0]), np.diff(raw_alignment[:, 1]))
    filtered_step = np.hypot(
        np.diff(filtered_alignment[:, 0]), np.diff(filtered_alignment[:, 1])
    )
    return poses, {
        "raw_alignment_translation_step_m": percentile_summary(raw_step),
        "filtered_alignment_translation_step_m": percentile_summary(filtered_step),
    }


def analyze(path, taus, heading_offset_deg, moving_threshold):
    ego, global_state, local_state, transform = parse_bag(path)
    ego = sorted_unique(ego)
    global_state = sorted_unique(global_state)
    local_state = sorted_unique(local_state)
    start = max(ego[0, 0], global_state[0, 0], local_state[0, 0])
    end = min(ego[-1, 0], global_state[-1, 0], local_state[-1, 0])
    global_state = global_state[
        (global_state[:, 0] >= start) & (global_state[:, 0] <= end)
    ]
    times = global_state[:, 0]
    local_pose = interpolate_pose(local_state, times)

    tx, ty, transform_yaw = transform
    cosine = math.cos(transform_yaw)
    sine = math.sin(transform_yaw)
    ego_map = np.column_stack(
        (
            tx + cosine * ego[:, 1] - sine * ego[:, 2],
            ty + sine * ego[:, 1] + cosine * ego[:, 2],
            np.unwrap(
                np.radians(ego[:, 3] + heading_offset_deg) + transform_yaw
            ),
        )
    )
    oracle = np.column_stack(
        (
            np.interp(times, ego[:, 0], ego_map[:, 0]),
            np.interp(times, ego[:, 0], ego_map[:, 1]),
            np.interp(times, ego[:, 0], ego_map[:, 2]),
        )
    )
    speed = np.interp(times, ego[:, 0], ego[:, 4])
    global_pose = global_state[:, 1:4].copy()
    global_pose[:, 2] = np.unwrap(global_pose[:, 2])

    result = {
        "bag": str(path.resolve()),
        "contract": {
            "acquisition": "recorded closed-loop; verify catalog provenance",
            "estimator_heading_offset_deg": heading_offset_deg,
            "evaluator_heading_offset_deg": heading_offset_deg,
            "candidate_kind": "counterfactual output-only smoothing",
            "production_changed": False,
        },
        "baseline": pose_metrics(
            times, global_pose, oracle, speed, moving_threshold
        ),
        "candidates": {},
    }
    for tau_sec in taus:
        poses, alignment = smooth_candidate(
            times, global_pose, local_pose, tau_sec
        )
        result["candidates"]["tau_{:.3f}s".format(tau_sec)] = {
            "tau_sec": tau_sec,
            "metrics": pose_metrics(
                times, poses, oracle, speed, moving_threshold
            ),
            "alignment": alignment,
        }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--taus", default="0.1,0.2,0.5,1.0,2.0,5.0")
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347)
    parser.add_argument("--moving-speed-threshold", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    taus = [float(value) for value in args.taus.split(",")]
    if not taus or not all(math.isfinite(value) and value >= 0.0 for value in taus):
        raise ValueError("--taus must contain finite non-negative seconds")
    result = analyze(
        args.bag,
        taus,
        args.heading_offset_deg,
        args.moving_speed_threshold,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n")
    print(rendered)


if __name__ == "__main__":
    main()
