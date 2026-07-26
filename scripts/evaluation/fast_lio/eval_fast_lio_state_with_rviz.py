#!/usr/bin/env python3
"""Evaluate FAST-LIO pose and twist against MORAI Ego ground truth.

The evaluator accepts either the raw FAST-LIO odometry contract
(`camera_init -> body`) or the Planning contract (`map -> base_link`).  Raw
body pose and twist are converted to the base_link reference point using the
IMU mounting pose in a MORAI Sensor JSON before comparison.
"""

import argparse
import bisect
import contextlib
import csv
import io
import math
import os
from dataclasses import dataclass

import numpy as np
import rosbag

import eval_fast_lio_base_link_with_rviz as base_eval
import eval_fast_lio_with_rviz as core


# /Ego_topic.position is expressed in MORAI's local map coordinates.  The
# evaluator aligns every state trajectory to those raw Ego coordinates, so the
# generated RViz paths and markers must carry this frame.  Merely changing the
# frame_id to "map" would relabel the coordinates without applying the recorded
# map -> morai_map transform and would offset them from /cloud_registered.
EVALUATION_FRAME = "morai_map"
EGO_VELOCITY_SCALES_TO_MPS = {
    "mps": 1.0,
    "kmph": 1.0 / 3.6,
}


@dataclass
class StateSample:
    stamp: float
    bag_time: float
    x: float
    y: float
    z: float
    yaw: float
    linear: np.ndarray
    angular: np.ndarray
    child_frame: str


def normalized_frame(frame):
    return str(frame).lstrip("/")


def finite_sample(sample):
    values = np.array(
        [
            sample.x,
            sample.y,
            sample.z,
            sample.yaw,
            *sample.linear,
            *sample.angular,
        ],
        dtype=float,
    )
    return bool(np.all(np.isfinite(values)))


def read_bag_states(bag_path, ego_topic, state_topic, ego_velocity_unit="mps"):
    ego_samples = []
    state_samples = []
    ego_velocity_scale = EGO_VELOCITY_SCALES_TO_MPS[ego_velocity_unit]

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, bag_time in bag.read_messages(
            topics=[ego_topic, state_topic]
        ):
            stamp = core.message_time(message, bag_time)
            if topic == ego_topic:
                sample = StateSample(
                    stamp=stamp,
                    bag_time=bag_time.to_sec(),
                    x=float(message.position.x),
                    y=float(message.position.y),
                    z=float(message.position.z),
                    yaw=math.radians(float(message.heading)),
                    linear=ego_velocity_scale
                    * np.array(
                        [
                            message.velocity.x,
                            message.velocity.y,
                            message.velocity.z,
                        ],
                        dtype=float,
                    ),
                    # GT yaw rate is estimated consistently from the heading
                    # time series below. beta_drive EgoVehicleStatus no longer
                    # contains angular_velocity, so do not depend on that
                    # legacy-only field while loading samples.
                    angular=np.zeros(3, dtype=float),
                    child_frame="base_link",
                )
                if finite_sample(sample):
                    ego_samples.append(sample)
            elif topic == state_topic:
                pose = message.pose.pose
                twist = message.twist.twist
                sample = StateSample(
                    stamp=stamp,
                    bag_time=bag_time.to_sec(),
                    x=float(pose.position.x),
                    y=float(pose.position.y),
                    z=float(pose.position.z),
                    yaw=core.yaw_from_quat(pose.orientation),
                    linear=np.array(
                        [twist.linear.x, twist.linear.y, twist.linear.z], dtype=float
                    ),
                    angular=np.array(
                        [twist.angular.x, twist.angular.y, twist.angular.z],
                        dtype=float,
                    ),
                    child_frame=normalized_frame(message.child_frame_id),
                )
                if finite_sample(sample):
                    state_samples.append(sample)

    ego_samples.sort(key=lambda sample: sample.stamp)
    state_samples.sort(key=lambda sample: sample.stamp)
    return ego_samples, state_samples


def estimate_heading_yaw_rates(ego_samples, window_size):
    """Estimate MORAI GT yaw rate from unwrapped heading with local regression."""
    if window_size < 3 or window_size % 2 == 0:
        raise ValueError("--ego-yaw-rate-window must be an odd integer >= 3")

    stamps = np.array([sample.stamp for sample in ego_samples], dtype=float)
    headings = np.unwrap(np.array([sample.yaw for sample in ego_samples], dtype=float))
    yaw_rates = np.zeros(len(ego_samples), dtype=float)
    half_window = window_size // 2

    for index in range(len(ego_samples)):
        start = max(0, index - half_window)
        end = min(len(ego_samples), index + half_window + 1)
        local_time = stamps[start:end]
        local_heading = headings[start:end]
        centered_time = local_time - local_time.mean()
        denominator = float(np.dot(centered_time, centered_time))
        if denominator > 1e-12:
            yaw_rates[index] = float(
                np.dot(centered_time, local_heading - local_heading.mean())
                / denominator
            )

    return stamps, headings, yaw_rates


def unique_samples_by_stamp(samples):
    """Drop repeated messages with identical header timestamps."""
    unique = []
    duplicate_count = 0
    for sample in samples:
        if unique and abs(sample.stamp - unique[-1].stamp) <= 1e-12:
            duplicate_count += 1
            continue
        unique.append(sample)
    return unique, duplicate_count


def interpolate_ego_to_state_times(
    ego_samples, state_samples, max_dt, yaw_rate_window
):
    """Interpolate Ego GT at each estimator timestamp.

    A state is accepted only when it lies on an Ego sample or is bracketed by
    two Ego samples that are both within max_dt.  The third tuple item is the
    distance to the nearest source Ego sample, not a residual synchronization
    error; the interpolated Ego sample itself has exactly the state timestamp.
    """
    unique_ego_samples, duplicate_count = unique_samples_by_stamp(ego_samples)
    stamps, unwrapped_heading, yaw_rates = estimate_heading_yaw_rates(
        unique_ego_samples, yaw_rate_window
    )
    stamp_list = stamps.tolist()
    matched = []
    exact_count = 0
    interpolated_count = 0
    rejected_outside = 0
    rejected_gap = 0

    for state in state_samples:
        right = bisect.bisect_left(stamp_list, state.stamp)

        if right < len(unique_ego_samples) and abs(stamps[right] - state.stamp) <= 1e-9:
            source = unique_ego_samples[right]
            ego = StateSample(
                stamp=state.stamp,
                bag_time=source.bag_time,
                x=source.x,
                y=source.y,
                z=source.z,
                yaw=source.yaw,
                linear=source.linear.copy(),
                angular=np.array([np.nan, np.nan, yaw_rates[right]], dtype=float),
                child_frame="base_link",
            )
            matched.append((ego, state, 0.0))
            exact_count += 1
            continue

        if right == 0 or right >= len(unique_ego_samples):
            rejected_outside += 1
            continue

        left = right - 1
        left_dt = state.stamp - stamps[left]
        right_dt = stamps[right] - state.stamp
        interval = stamps[right] - stamps[left]
        if interval <= 1e-12 or max(left_dt, right_dt) > max_dt:
            rejected_gap += 1
            continue

        ratio = left_dt / interval
        left_sample = unique_ego_samples[left]
        right_sample = unique_ego_samples[right]
        xyz = (1.0 - ratio) * np.array(
            [left_sample.x, left_sample.y, left_sample.z], dtype=float
        ) + ratio * np.array(
            [right_sample.x, right_sample.y, right_sample.z], dtype=float
        )
        linear = (1.0 - ratio) * left_sample.linear + ratio * right_sample.linear
        yaw = core.wrap_angle(
            (1.0 - ratio) * unwrapped_heading[left]
            + ratio * unwrapped_heading[right]
        )
        yaw_rate = (1.0 - ratio) * yaw_rates[left] + ratio * yaw_rates[right]
        bag_time = (
            (1.0 - ratio) * left_sample.bag_time + ratio * right_sample.bag_time
        )
        ego = StateSample(
            stamp=state.stamp,
            bag_time=float(bag_time),
            x=float(xyz[0]),
            y=float(xyz[1]),
            z=float(xyz[2]),
            yaw=float(yaw),
            linear=linear,
            angular=np.array([np.nan, np.nan, yaw_rate], dtype=float),
            child_frame="base_link",
        )
        matched.append((ego, state, min(left_dt, right_dt)))
        interpolated_count += 1

    statistics = {
        "exact": exact_count,
        "interpolated": interpolated_count,
        "rejected_outside": rejected_outside,
        "rejected_gap": rejected_gap,
        "yaw_rate_window": yaw_rate_window,
        "duplicate_stamps": duplicate_count,
    }
    return matched, statistics


def rotation_from_yaw(yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )


def body_state_to_base(sample, translation_body_base, yaw_body_base):
    rotation_camera_body = rotation_from_yaw(sample.yaw)
    rotation_body_base = rotation_from_yaw(yaw_body_base)
    rotation_base_body = rotation_body_base.transpose()

    base_position = (
        np.array([sample.x, sample.y, sample.z], dtype=float)
        + rotation_camera_body @ translation_body_base
    )
    velocity_base_origin_in_body = sample.linear + np.cross(
        sample.angular, translation_body_base
    )

    return StateSample(
        stamp=sample.stamp,
        bag_time=sample.bag_time,
        x=float(base_position[0]),
        y=float(base_position[1]),
        z=float(base_position[2]),
        yaw=core.wrap_angle(sample.yaw + yaw_body_base),
        linear=rotation_base_body @ velocity_base_origin_in_body,
        angular=rotation_base_body @ sample.angular,
        child_frame="base_link",
    )


def normalize_state_reference(state_samples, sensor_config, imu_name):
    child_frames = {sample.child_frame for sample in state_samples}
    if len(child_frames) != 1:
        raise RuntimeError(
            f"State topic has inconsistent child_frame_id values: {sorted(child_frames)}"
        )

    child_frame = next(iter(child_frames))
    if child_frame == "base_link":
        return state_samples, None
    if child_frame != "body":
        raise RuntimeError(
            "Expected state child_frame_id to be 'body' or 'base_link'; "
            f"received '{child_frame}'"
        )

    sensor_name, base_to_body_xyz, base_to_body_rpy_deg = base_eval.read_imu_pose(
        sensor_config, imu_name
    )
    translation_body_base, yaw_body_base = base_eval.inverse_planar_mount(
        base_to_body_xyz, base_to_body_rpy_deg
    )
    corrected = [
        body_state_to_base(sample, translation_body_base, yaw_body_base)
        for sample in state_samples
    ]
    mount = {
        "sensor_name": sensor_name,
        "base_to_body_xyz": base_to_body_xyz,
        "base_to_body_rpy_deg": base_to_body_rpy_deg,
        "translation_body_base": translation_body_base,
        "yaw_body_base": yaw_body_base,
    }
    return corrected, mount


def component_statistics(error):
    return {
        "bias": float(np.mean(error)),
        "mae": float(np.mean(np.abs(error))),
        "rmse": math.sqrt(float(np.mean(error**2))),
        "max_abs": float(np.max(np.abs(error))),
    }


def project_position_error_to_ego_frame(position_error_xyz, ego_yaw):
    """Project map-frame state-minus-GT error into Ego forward/left axes."""
    cosine = np.cos(ego_yaw)
    sine = np.sin(ego_yaw)
    longitudinal = cosine * position_error_xyz[:, 0] + sine * position_error_xyz[:, 1]
    lateral = -sine * position_error_xyz[:, 0] + cosine * position_error_xyz[:, 1]
    return longitudinal, lateral


def align_from_reference_pose(matched, reference_time_sec):
    """Rigidly align state and Ego at a selected elapsed timestamp.

    The default reference time of zero preserves the historical evaluator
    behavior. A positive time is useful when a recorded estimator or simulated
    IMU publishes a transient orientation during startup; the transient is
    still evaluated, but it no longer rotates the entire trajectory.
    """
    first_stamp = matched[0][0].stamp
    reference_index = len(matched) - 1
    for index, (ego_sample, _, _) in enumerate(matched):
        if ego_sample.stamp - first_stamp >= reference_time_sec:
            reference_index = index
            break

    ego_reference, state_reference, _ = matched[reference_index]
    alignment_yaw = core.wrap_angle(
        ego_reference.yaw - state_reference.yaw
    )
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]], dtype=float)

    ego_xyz = np.array(
        [[match[0].x, match[0].y, match[0].z] for match in matched],
        dtype=float,
    )
    state_xyz = np.array(
        [[match[1].x, match[1].y, match[1].z] for match in matched],
        dtype=float,
    )
    state_xy_relative = (
        state_xyz[:, :2] - state_xyz[reference_index, :2]
    )
    aligned_xy = (
        state_xy_relative @ rotation.T
        + ego_xyz[reference_index, :2]
    )
    aligned_z = (
        state_xyz[:, 2]
        - state_xyz[reference_index, 2]
        + ego_xyz[reference_index, 2]
    )
    aligned_xyz = np.column_stack((aligned_xy, aligned_z))
    aligned_yaw = np.array(
        [
            core.wrap_angle(match[1].yaw + alignment_yaw)
            for match in matched
        ],
        dtype=float,
    )
    reference_elapsed_sec = (
        matched[reference_index][0].stamp - first_stamp
    )
    return (
        ego_xyz,
        aligned_xyz,
        aligned_yaw,
        alignment_yaw,
        reference_elapsed_sec,
    )


def mask_segments(mask, relative_times, minimum_duration):
    segments = []
    start = None
    for index, active in enumerate(mask):
        if active and start is None:
            start = index
        if start is not None and (not active or index == len(mask) - 1):
            end = index if active and index == len(mask) - 1 else index - 1
            if relative_times[end] - relative_times[start] >= minimum_duration:
                segments.append((start, end))
            start = None
    return segments


def print_error_segments(title, segments, relative_times, unit):
    print(f"{title}: {len(segments)}")
    for number, segment in enumerate(segments, start=1):
        print(
            f"  [{number}] {segment['start_time']:.3f}s ~ "
            f"{segment['end_time']:.3f}s, max={segment['max_error']:.3f}{unit} "
            f"at {relative_times[segment['max_index']]:.3f}s"
        )


def print_report(
    bag_path,
    state_topic,
    matched,
    relative_times,
    ego_xyz,
    aligned_xyz,
    position_errors,
    longitudinal_errors,
    lateral_errors,
    yaw_errors,
    alignment_yaw,
    ego_linear,
    state_linear,
    ego_angular,
    state_angular,
    position_segments,
    speed_segments,
    yaw_rate_segments,
    stationary_segments,
    mount,
    interpolation_statistics,
    ego_velocity_unit,
    alignment_reference_sec,
):
    ego_distance = float(np.linalg.norm(np.diff(ego_xyz[:, :2], axis=0), axis=1).sum())
    state_distance = float(
        np.linalg.norm(np.diff(aligned_xyz[:, :2], axis=0), axis=1).sum()
    )
    path_ratio = state_distance / ego_distance if ego_distance > 1e-9 else float("nan")
    final_error = float(position_errors[-1])
    drift = final_error / ego_distance * 100.0 if ego_distance > 1e-9 else float("nan")
    source_dt = np.array([match[2] for match in matched], dtype=float)

    ego_speed = np.linalg.norm(ego_linear[:, :2], axis=1)
    state_speed = np.linalg.norm(state_linear[:, :2], axis=1)
    linear_error = state_linear - ego_linear
    yaw_rate_error = state_angular[:, 2] - ego_angular[:, 2]

    print("======= FAST-LIO Pose + Twist vs MORAI Ego Evaluation =======")
    print(f"bag: {bag_path}")
    print(f"state topic: {state_topic}")
    if ego_velocity_unit == "kmph":
        print("Ego velocity input: km/h (converted to m/s with scale 1/3.6)")
    else:
        print("Ego velocity input: m/s")
    print(f"matched samples: {len(matched)}")
    print(f"duration: {relative_times[-1]:.3f} sec")
    print("comparison time: state timestamp (Ego GT interpolated)")
    print(
        "Ego interpolation | "
        f"exact: {interpolation_statistics['exact']}, "
        f"interpolated: {interpolation_statistics['interpolated']}, "
        f"rejected outside: {interpolation_statistics['rejected_outside']}, "
        f"rejected gap: {interpolation_statistics['rejected_gap']}"
    )
    print(
        "Ego duplicate timestamps discarded: "
        f"{interpolation_statistics['duplicate_stamps']}"
    )
    print(
        f"nearest Ego source distance | mean: {source_dt.mean():.4f} sec, "
        f"max: {source_dt.max():.4f} sec"
    )
    if mount is None:
        print("state reference: base_link (no mounting correction applied)")
    else:
        xyz = mount["translation_body_base"]
        print(
            "state reference: body -> base_link corrected, "
            f"T_body_base=[{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}] m"
        )

    print("------------------------- Pose -------------------------")
    print(f"ego path length:       {ego_distance:.3f} m")
    print(f"FAST-LIO path length:  {state_distance:.3f} m")
    print(f"path length ratio:     {path_ratio:.5f}")
    print(f"path length error:     {state_distance - ego_distance:+.3f} m")
    print(f"initial alignment yaw: {math.degrees(alignment_yaw):.3f} deg")
    print(
        f"alignment reference:    {alignment_reference_sec:.3f} sec "
        "from first matched state"
    )
    print(f"XY mean error:         {position_errors.mean():.3f} m")
    print(f"XY RMSE:               {math.sqrt(float(np.mean(position_errors**2))):.3f} m")
    print(f"XY max error:          {position_errors.max():.3f} m")
    print(f"XY final error:        {final_error:.3f} m")
    print(f"drift:                 {drift:.3f} %")
    print(f"Yaw mean abs:          {np.degrees(np.abs(yaw_errors)).mean():.3f} deg")
    print(f"Yaw max abs:           {np.degrees(np.abs(yaw_errors)).max():.3f} deg")

    print("--------------- Ego-frame position error --------------")
    print("convention: error=state-GT, longitudinal +=ahead, lateral +=left")
    for name, errors in (
        ("longitudinal", longitudinal_errors),
        ("lateral", lateral_errors),
    ):
        stats = component_statistics(errors)
        print(
            f"{name:12s} | bias={stats['bias']:+.3f}, MAE={stats['mae']:.3f}, "
            f"RMSE={stats['rmse']:.3f}, max_abs={stats['max_abs']:.3f} m"
        )
    squared_error_sum = float(
        np.sum(longitudinal_errors**2 + lateral_errors**2)
    )
    if squared_error_sum > 1e-12:
        longitudinal_contribution = (
            float(np.sum(longitudinal_errors**2)) / squared_error_sum * 100.0
        )
        lateral_contribution = (
            float(np.sum(lateral_errors**2)) / squared_error_sum * 100.0
        )
        print(
            "squared error contribution | "
            f"longitudinal={longitudinal_contribution:.1f}%, "
            f"lateral={lateral_contribution:.1f}%"
        )
    else:
        print("squared error contribution | unavailable (zero position error)")
    print(
        "final error | "
        f"longitudinal={longitudinal_errors[-1]:+.3f} m, "
        f"lateral={lateral_errors[-1]:+.3f} m, XY={final_error:.3f} m"
    )

    print("--------------------- Linear twist ---------------------")
    for index, axis in enumerate(("x", "y", "z")):
        stats = component_statistics(linear_error[:, index])
        print(
            f"linear.{axis} | bias={stats['bias']:+.3f}, MAE={stats['mae']:.3f}, "
            f"RMSE={stats['rmse']:.3f}, max_abs={stats['max_abs']:.3f} m/s"
        )
    speed_stats = component_statistics(state_speed - ego_speed)
    print(
        f"speed    | bias={speed_stats['bias']:+.3f}, MAE={speed_stats['mae']:.3f}, "
        f"RMSE={speed_stats['rmse']:.3f}, max_abs={speed_stats['max_abs']:.3f} m/s"
    )

    print("-------------------- Angular twist ---------------------")
    print(
        "GT yaw rate source: derivative of unwrapped Ego heading "
        f"(local regression window={interpolation_statistics['yaw_rate_window']})"
    )
    stats = component_statistics(yaw_rate_error)
    print(
        f"angular.z | bias={stats['bias']:+.4f}, MAE={stats['mae']:.4f}, "
        f"RMSE={stats['rmse']:.4f}, max_abs={stats['max_abs']:.4f} rad/s"
    )
    print("angular.x/y: not evaluated (MORAI Ego heading provides yaw only)")

    print("-------------------- Stationary state ------------------")
    print(f"stationary segments: {len(stationary_segments)}")
    for number, (start, end) in enumerate(stationary_segments, start=1):
        segment_xyz = aligned_xyz[start : end + 1, :2]
        segment_speed = state_speed[start : end + 1]
        segment_yaw_rate = np.abs(state_angular[start : end + 1, 2])
        net_displacement = float(np.linalg.norm(segment_xyz[-1] - segment_xyz[0]))
        accumulated_distance = float(
            np.linalg.norm(np.diff(segment_xyz, axis=0), axis=1).sum()
        )
        print(
            f"  [{number}] {relative_times[start]:.3f}s ~ {relative_times[end]:.3f}s, "
            f"net={net_displacement:.3f}m, accumulated={accumulated_distance:.3f}m, "
            f"speed_mean={segment_speed.mean():.3f}m/s, speed_max={segment_speed.max():.3f}m/s, "
            f"|wz|_mean={segment_yaw_rate.mean():.4f}rad/s"
        )

    print("--------------------- Error segments -------------------")
    print_error_segments(
        "position error segments above threshold",
        position_segments,
        relative_times,
        "m",
    )
    print_error_segments(
        "speed error segments above threshold",
        speed_segments,
        relative_times,
        "m/s",
    )
    print_error_segments(
        "yaw-rate error segments above threshold",
        yaw_rate_segments,
        relative_times,
        "rad/s",
    )
    print("=========================================================")


def output_paths(bag_path, requested_path):
    if requested_path:
        csv_path = os.path.abspath(requested_path)
        if not csv_path.lower().endswith(".csv"):
            csv_path += ".csv"
    else:
        bag_absolute = os.path.abspath(bag_path)
        stem = os.path.splitext(os.path.basename(bag_absolute))[0]
        csv_path = os.path.join(os.path.dirname(bag_absolute), stem + "_state_eval.csv")
    return csv_path, os.path.splitext(csv_path)[0] + ".txt"


def write_csv(
    csv_path,
    matched,
    relative_times,
    ego_xyz,
    aligned_xyz,
    aligned_yaw,
    position_errors,
    longitudinal_errors,
    lateral_errors,
    yaw_errors,
    ego_linear,
    state_linear,
    ego_angular,
    state_angular,
    stationary_mask,
    ego_velocity_unit,
):
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    position_error_xyz = aligned_xyz - ego_xyz
    linear_error = state_linear - ego_linear
    angular_error = state_angular - ego_angular
    ego_speed = np.linalg.norm(ego_linear[:, :2], axis=1)
    state_speed = np.linalg.norm(state_linear[:, :2], axis=1)

    header = [
        "t_rel", "ego_stamp", "ego_bag_time", "state_stamp", "state_bag_time",
        "ego_nearest_source_dt", "ego_velocity_source_unit",
        "ego_velocity_scale_to_mps",
        "ego_x", "ego_y", "ego_z", "state_aligned_x", "state_aligned_y", "state_aligned_z",
        "err_x", "err_y", "err_z", "err_xy", "err_longitudinal", "err_lateral",
        "ego_yaw_deg", "state_yaw_deg", "yaw_err_deg",
        "ego_vx", "ego_vy", "ego_vz", "state_vx", "state_vy", "state_vz",
        "velocity_err_x", "velocity_err_y", "velocity_err_z", "ego_speed", "state_speed", "speed_err",
        "ego_wx", "ego_wy", "ego_wz", "state_wx", "state_wy", "state_wz",
        "angular_err_x", "angular_err_y", "angular_err_z", "is_stationary",
    ]
    with open(csv_path, "w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(header)
        for index, (ego_sample, state_sample, nearest_source_dt) in enumerate(matched):
            writer.writerow(
                [
                    relative_times[index], ego_sample.stamp, ego_sample.bag_time,
                    state_sample.stamp, state_sample.bag_time, nearest_source_dt,
                    ego_velocity_unit,
                    EGO_VELOCITY_SCALES_TO_MPS[ego_velocity_unit],
                    *ego_xyz[index], *aligned_xyz[index], *position_error_xyz[index],
                    position_errors[index], longitudinal_errors[index], lateral_errors[index],
                    math.degrees(ego_sample.yaw),
                    math.degrees(aligned_yaw[index]), math.degrees(yaw_errors[index]),
                    *ego_linear[index], *state_linear[index], *linear_error[index],
                    ego_speed[index], state_speed[index], state_speed[index] - ego_speed[index],
                    *ego_angular[index], *state_angular[index], *angular_error[index],
                    int(stationary_mask[index]),
                ]
            )


def publish_rviz(
    frame_id,
    matched,
    ego_xyz,
    aligned_xyz,
    aligned_yaw,
    alignment_yaw,
    position_errors,
    speed_errors,
    yaw_rate_errors,
    position_threshold,
    speed_threshold,
    yaw_rate_threshold,
    marker_stride,
    publish_alignment_tf,
):
    import rospy
    import tf2_ros
    from geometry_msgs.msg import Point, PoseStamped, TransformStamped
    from nav_msgs.msg import Path
    from visualization_msgs.msg import Marker, MarkerArray

    rospy.init_node("fast_lio_state_eval_visualizer", anonymous=True)
    ego_publisher = rospy.Publisher(
        "/fast_lio_state_eval/ego_path", Path, queue_size=1, latch=True
    )
    state_publisher = rospy.Publisher(
        "/fast_lio_state_eval/lio_aligned_path", Path, queue_size=1, latch=True
    )
    marker_publisher = rospy.Publisher(
        "/fast_lio_state_eval/error_markers", MarkerArray, queue_size=1, latch=True
    )

    stamp = rospy.Time.now()
    ego_path = Path()
    ego_path.header.frame_id = frame_id
    ego_path.header.stamp = stamp
    state_path = Path()
    state_path.header = ego_path.header

    for index, (ego_sample, _, _) in enumerate(matched):
        ego_pose = PoseStamped()
        ego_pose.header = ego_path.header
        ego_pose.pose.position.x = float(ego_xyz[index, 0])
        ego_pose.pose.position.y = float(ego_xyz[index, 1])
        ego_pose.pose.position.z = float(ego_xyz[index, 2])
        core.quaternion_from_yaw(ego_sample.yaw, ego_pose.pose.orientation)
        ego_path.poses.append(ego_pose)

        state_pose = PoseStamped()
        state_pose.header = state_path.header
        state_pose.pose.position.x = float(aligned_xyz[index, 0])
        state_pose.pose.position.y = float(aligned_xyz[index, 1])
        state_pose.pose.position.z = float(aligned_xyz[index, 2])
        core.quaternion_from_yaw(aligned_yaw[index], state_pose.pose.orientation)
        state_path.poses.append(state_pose)

    def points_marker(namespace, marker_id, red, green, blue):
        marker = Marker()
        marker.header = ego_path.header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.55
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.9
        return marker

    position_marker = points_marker("position_error", 0, 1.0, 0.15, 0.05)
    speed_marker = points_marker("speed_error", 1, 1.0, 0.0, 0.8)
    yaw_rate_marker = points_marker("yaw_rate_error", 2, 0.0, 0.8, 1.0)
    stride = max(1, marker_stride)
    for index in range(0, len(matched), stride):
        point = Point(
            x=float(aligned_xyz[index, 0]),
            y=float(aligned_xyz[index, 1]),
            z=float(aligned_xyz[index, 2]),
        )
        if position_errors[index] >= position_threshold:
            position_marker.points.append(point)
        if speed_errors[index] >= speed_threshold:
            speed_marker.points.append(point)
        if yaw_rate_errors[index] >= yaw_rate_threshold:
            yaw_rate_marker.points.append(point)

    ego_publisher.publish(ego_path)
    state_publisher.publish(state_path)
    marker_publisher.publish(
        MarkerArray(markers=[position_marker, speed_marker, yaw_rate_marker])
    )

    static_broadcaster = None
    if publish_alignment_tf:
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        ego0, state0, _ = matched[0]
        cosine = math.cos(alignment_yaw)
        sine = math.sin(alignment_yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = frame_id
        transform.child_frame_id = "camera_init"
        transform.transform.translation.x = ego0.x - (
            cosine * state0.x - sine * state0.y
        )
        transform.transform.translation.y = ego0.y - (
            sine * state0.x + cosine * state0.y
        )
        transform.transform.translation.z = ego0.z - state0.z
        core.quaternion_from_yaw(alignment_yaw, transform.transform.rotation)
        static_broadcaster.sendTransform(transform)

    rospy.loginfo(
        "Published FAST-LIO state evaluation paths and markers in frame '%s'",
        frame_id,
    )
    rospy.loginfo(
        "RViz topics: /fast_lio_state_eval/ego_path, "
        "/fast_lio_state_eval/lio_aligned_path, "
        "/fast_lio_state_eval/error_markers"
    )
    rospy.spin()


def main():
    repository_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..")
    )
    default_sensor_config = os.path.join(
        repository_root, "configs", "sensors", "Sensor_2.json"
    )

    parser = argparse.ArgumentParser(
        description="Evaluate FAST-LIO pose and twist at the base_link reference point."
    )
    parser.add_argument("bag", help="Bag containing /Ego_topic and the state topic")
    parser.add_argument(
        "--out",
        default="",
        help=(
            "Output CSV path; a matching .txt is saved. If omitted, files are "
            "created beside the input bag with suffix _state_eval."
        ),
    )
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument(
        "--ego-velocity-unit",
        choices=tuple(EGO_VELOCITY_SCALES_TO_MPS),
        default="mps",
        help=(
            "Unit stored in /Ego_topic.velocity. Use 'kmph' for legacy bags "
            "recorded from the AjouNice MORAI UDP receiver before SI-unit "
            "conversion was added; new receiver output and native ROS "
            "EgoVehicleStatus use 'mps' (default)."
        ),
    )
    parser.add_argument(
        "--state-topic",
        default="/localization/odom",
        help="Raw FAST-LIO odom or base_link /localization/kinematic_state",
    )
    parser.add_argument("--sensor-config", default=default_sensor_config)
    parser.add_argument("--imu-name", default="imu")
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument(
        "--alignment-reference-sec",
        type=float,
        default=0.0,
        help=(
            "Elapsed time of the pose pair used for one-time trajectory "
            "alignment. Keep 0 for the historical first-sample behavior; "
            "use a positive value only when a documented startup transient "
            "must not define the whole-run alignment."
        ),
    )
    parser.add_argument(
        "--ego-yaw-rate-window",
        type=int,
        default=7,
        help=(
            "Odd number of raw Ego heading samples used by the local linear "
            "regression that estimates GT yaw rate (default: 7)"
        ),
    )
    parser.add_argument("--position-error-threshold", type=float, default=1.0)
    parser.add_argument("--speed-error-threshold", type=float, default=0.5)
    parser.add_argument("--yaw-rate-error-threshold", type=float, default=0.1)
    parser.add_argument("--stationary-speed-threshold", type=float, default=0.1)
    parser.add_argument("--stationary-min-duration", type=float, default=1.0)
    parser.add_argument("--publish-rviz", action="store_true")
    parser.add_argument("--marker-stride", type=int, default=1)
    args = parser.parse_args()

    ego_samples, raw_state_samples = read_bag_states(
        args.bag,
        args.ego_topic,
        args.state_topic,
        args.ego_velocity_unit,
    )
    if not ego_samples:
        raise RuntimeError(f"No valid messages found on {args.ego_topic}")
    if not raw_state_samples:
        raise RuntimeError(f"No valid messages found on {args.state_topic}")

    original_child_frame = raw_state_samples[0].child_frame
    state_samples, mount = normalize_state_reference(
        raw_state_samples, os.path.abspath(args.sensor_config), args.imu_name
    )
    matched, interpolation_statistics = interpolate_ego_to_state_times(
        ego_samples,
        state_samples,
        args.max_dt,
        args.ego_yaw_rate_window,
    )
    if len(matched) < 10:
        raise RuntimeError(
            f"Too few unique matched samples: {len(matched)}. "
            "Increase --max-dt or check timestamps."
        )
    if args.alignment_reference_sec < 0.0:
        raise ValueError("--alignment-reference-sec must be non-negative")
    available_duration = matched[-1][0].stamp - matched[0][0].stamp
    if args.alignment_reference_sec > available_duration:
        raise ValueError(
            "--alignment-reference-sec exceeds the matched trajectory "
            f"duration ({available_duration:.3f} sec)"
        )

    (
        ego_xyz,
        aligned_xyz,
        aligned_yaw,
        alignment_yaw,
        alignment_reference_sec,
    ) = align_from_reference_pose(
        matched, args.alignment_reference_sec
    )
    position_error_xyz = aligned_xyz - ego_xyz
    position_errors = np.linalg.norm(position_error_xyz[:, :2], axis=1)
    ego_yaw = np.array([match[0].yaw for match in matched], dtype=float)
    longitudinal_errors, lateral_errors = project_position_error_to_ego_frame(
        position_error_xyz, ego_yaw
    )
    yaw_errors = np.array(
        [
            core.wrap_angle(aligned_yaw[index] - matched[index][0].yaw)
            for index in range(len(matched))
        ],
        dtype=float,
    )
    relative_times = np.array(
        [match[0].stamp - matched[0][0].stamp for match in matched], dtype=float
    )
    ego_linear = np.array([match[0].linear for match in matched], dtype=float)
    state_linear = np.array([match[1].linear for match in matched], dtype=float)
    ego_angular = np.array([match[0].angular for match in matched], dtype=float)
    state_angular = np.array([match[1].angular for match in matched], dtype=float)
    ego_speed = np.linalg.norm(ego_linear[:, :2], axis=1)
    state_speed = np.linalg.norm(state_linear[:, :2], axis=1)
    speed_errors = np.abs(state_speed - ego_speed)
    yaw_rate_errors = np.abs(state_angular[:, 2] - ego_angular[:, 2])

    position_segments = core.threshold_segments(
        position_errors, relative_times, args.position_error_threshold
    )
    speed_segments = core.threshold_segments(
        speed_errors, relative_times, args.speed_error_threshold
    )
    yaw_rate_segments = core.threshold_segments(
        yaw_rate_errors, relative_times, args.yaw_rate_error_threshold
    )
    stationary_mask = ego_speed < args.stationary_speed_threshold
    stationary_segments = mask_segments(
        stationary_mask, relative_times, args.stationary_min_duration
    )

    csv_path, text_path = output_paths(args.bag, args.out)
    report_buffer = io.StringIO()
    with contextlib.redirect_stdout(report_buffer):
        print_report(
            args.bag,
            args.state_topic,
            matched,
            relative_times,
            ego_xyz,
            aligned_xyz,
            position_errors,
            longitudinal_errors,
            lateral_errors,
            yaw_errors,
            alignment_yaw,
            ego_linear,
            state_linear,
            ego_angular,
            state_angular,
            position_segments,
            speed_segments,
            yaw_rate_segments,
            stationary_segments,
            mount,
            interpolation_statistics,
            args.ego_velocity_unit,
            alignment_reference_sec,
        )
    report = report_buffer.getvalue()
    print(report, end="")

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    with open(text_path, "w") as output:
        output.write(report)
    write_csv(
        csv_path,
        matched,
        relative_times,
        ego_xyz,
        aligned_xyz,
        aligned_yaw,
        position_errors,
        longitudinal_errors,
        lateral_errors,
        yaw_errors,
        ego_linear,
        state_linear,
        ego_angular,
        state_angular,
        stationary_mask,
        args.ego_velocity_unit,
    )
    print(f"CSV saved: {csv_path}")
    print(f"Text report saved: {text_path}")

    if args.publish_rviz:
        publish_rviz(
            EVALUATION_FRAME,
            matched,
            ego_xyz,
            aligned_xyz,
            aligned_yaw,
            alignment_yaw,
            position_errors,
            speed_errors,
            yaw_rate_errors,
            args.position_error_threshold,
            args.speed_error_threshold,
            args.yaw_rate_error_threshold,
            args.marker_stride,
            original_child_frame == "body",
        )


if __name__ == "__main__":
    main()
