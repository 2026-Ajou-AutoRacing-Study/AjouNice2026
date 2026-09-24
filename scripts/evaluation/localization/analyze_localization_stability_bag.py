#!/usr/bin/env python3
"""Quantify localization error and steering chatter from a live-drive bag.

The MORAI Ego pose is used only as an offline oracle.  Production localization
continues to consume /Competition_topic, GPS, and IMU.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rosbag


TOPICS = [
    "/Ego_topic",
    "/Competition_topic",
    "/sensor/gps",
    "/sensor/imu",
    "/localization/gnss/odom",
    "/localization/odometry/local",
    "/localization/kinematic_state",
    "/localization/control_kinematic_state",
    "/planning/trajectory",
    "/control/autoware/controller_valid",
    "/control/command/steering",
    "/purepursuit_autoware/debug/lateral_error",
    "/ctrl_cmd",
    "/tf",
    "/tf_static",
]


def stamp_or_bag(message, bag_time):
    header = getattr(message, "header", None)
    if header is not None and not header.stamp.is_zero():
        return header.stamp.to_sec()
    return bag_time.to_sec()


def yaw_from_quaternion(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
    return math.atan2(siny, cosy)


def wrap_angle(values):
    return (values + np.pi) % (2.0 * np.pi) - np.pi


def percentile(values, q):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return None
    return float(np.percentile(values, q))


def scalar_summary(values, absolute=False):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {"count": 0}
    reported = np.abs(values) if absolute else values
    return {
        "count": int(values.size),
        "mean": float(np.mean(values)),
        "absolute_mean": float(np.mean(np.abs(values))),
        "std": float(np.std(values)),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "p50": float(np.percentile(reported, 50)),
        "p95": float(np.percentile(reported, 95)),
        "p99": float(np.percentile(reported, 99)),
        "max": float(np.max(reported)),
    }


def rate_summary(times):
    times = np.asarray(times, dtype=float)
    if times.size < 2:
        return {"count": int(times.size)}
    times = np.sort(times)
    gaps = np.diff(times)
    positive = gaps[gaps > 1.0e-6]
    duration = float(times[-1] - times[0])
    return {
        "count": int(times.size),
        "duration_sec": duration,
        "mean_hz": float((times.size - 1) / duration) if duration > 0 else None,
        "gap_p50_sec": percentile(positive, 50),
        "gap_p95_sec": percentile(positive, 95),
        "gap_max_sec": float(np.max(positive)) if positive.size else None,
    }


def nearest_values(source_times, source_values, query_times, default=np.nan):
    source_times = np.asarray(source_times, dtype=float)
    source_values = np.asarray(source_values)
    query_times = np.asarray(query_times, dtype=float)
    if source_times.size == 0:
        return np.full(query_times.shape, default)
    indices = np.searchsorted(source_times, query_times)
    indices = np.clip(indices, 0, source_times.size - 1)
    previous = np.clip(indices - 1, 0, source_times.size - 1)
    choose_previous = np.abs(query_times - source_times[previous]) <= np.abs(
        source_times[indices] - query_times
    )
    indices[choose_previous] = previous[choose_previous]
    return source_values[indices]


def contiguous_intervals(times, mask, max_gap=0.25, min_duration=1.0):
    times = np.asarray(times, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    selected = times[mask]
    if not selected.size:
        return []
    intervals = []
    start = selected[0]
    previous = selected[0]
    for current in selected[1:]:
        if current - previous > max_gap:
            if previous - start >= min_duration:
                intervals.append([float(start), float(previous)])
            start = current
        previous = current
    if previous - start >= min_duration:
        intervals.append([float(start), float(previous)])
    return intervals


def parse_bag(path):
    series = {topic: [] for topic in TOPICS}
    map_from_morai = None
    bag_start = None
    bag_end = None

    with rosbag.Bag(str(path), "r") as bag:
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, bag_time in bag.read_messages(topics=TOPICS):
            record_time = bag_time.to_sec()
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
                            yaw_from_quaternion(rotation),
                        )
                continue

            if topic == "/tf":
                for transform in message.transforms:
                    if (
                        transform.header.frame_id.lstrip("/") == "map"
                        and transform.child_frame_id.lstrip("/") == "odom"
                    ):
                        translation = transform.transform.translation
                        series[topic].append(
                            (
                                transform.header.stamp.to_sec(),
                                float(translation.x),
                                float(translation.y),
                                yaw_from_quaternion(transform.transform.rotation),
                                record_time,
                            )
                        )
                continue

            time = stamp_or_bag(message, bag_time)
            if topic == "/Ego_topic":
                series[topic].append(
                    (
                        time,
                        float(message.position.x),
                        float(message.position.y),
                        float(message.heading),
                        float(message.velocity.x),
                        record_time,
                    )
                )
            elif topic in (
                "/localization/kinematic_state",
                "/localization/control_kinematic_state",
            ):
                position = message.pose.pose.position
                series[topic].append(
                    (
                        time,
                        float(position.x),
                        float(position.y),
                        yaw_from_quaternion(message.pose.pose.orientation),
                        float(message.twist.twist.linear.x),
                        record_time,
                    )
                )
            elif topic == "/localization/odometry/local":
                position = message.pose.pose.position
                series[topic].append(
                    (
                        time,
                        float(position.x),
                        float(position.y),
                        yaw_from_quaternion(message.pose.pose.orientation),
                        record_time,
                    )
                )
            elif topic == "/localization/gnss/odom":
                position = message.pose.pose.position
                series[topic].append((time, float(position.x), float(position.y), record_time))
            elif topic in (
                "/control/command/steering",
                "/purepursuit_autoware/debug/lateral_error",
            ):
                series[topic].append((record_time, float(message.data)))
            elif topic == "/control/autoware/controller_valid":
                series[topic].append((record_time, bool(message.data)))
            elif topic == "/ctrl_cmd":
                series[topic].append(
                    (
                        record_time,
                        float(message.steering),
                        float(message.accel),
                        float(message.brake),
                    )
                )
            else:
                series[topic].append(record_time)

    return series, map_from_morai, bag_start, bag_end


def analyze(path, moving_speed_threshold=1.0, heading_offset_deg=-1.347):
    series, transform, bag_start, bag_end = parse_bag(path)
    if transform is None:
        raise RuntimeError("map -> morai_map static TF is missing")
    if not series["/Ego_topic"]:
        raise RuntimeError("/Ego_topic oracle is missing")
    if not series["/localization/kinematic_state"]:
        raise RuntimeError("/localization/kinematic_state is missing")

    ego = np.asarray(series["/Ego_topic"], dtype=float)
    localization = np.asarray(series["/localization/kinematic_state"], dtype=float)
    ego_order = np.argsort(ego[:, 0])
    ego = ego[ego_order]
    loc_order = np.argsort(localization[:, 0])
    localization = localization[loc_order]

    tx, ty, transform_yaw = transform
    cos_tf = math.cos(transform_yaw)
    sin_tf = math.sin(transform_yaw)
    ego_map_x = tx + cos_tf * ego[:, 1] - sin_tf * ego[:, 2]
    ego_map_y = ty + sin_tf * ego[:, 1] + cos_tf * ego[:, 2]
    ego_yaw = np.unwrap(
        np.radians(ego[:, 3]) + math.radians(heading_offset_deg) + transform_yaw
    )

    # MORAI pose and velocity agree over a complete drive, but the pose update
    # cadence is not perfectly uniform.  Quantify that oracle-side timing
    # residual so localization frame-jump metrics are not interpreted as if
    # the reference itself were kinematically smooth at every sample.
    ego_dt = np.diff(ego[:, 0])
    ego_mid_yaw = 0.5 * (ego_yaw[:-1] + ego_yaw[1:])
    ego_mid_speed = 0.5 * (ego[:-1, 4] + ego[1:, 4])
    expected_dx = ego_mid_speed * ego_dt * np.cos(ego_mid_yaw)
    expected_dy = ego_mid_speed * ego_dt * np.sin(ego_mid_yaw)
    ego_residual_x = np.diff(ego_map_x) - expected_dx
    ego_residual_y = np.diff(ego_map_y) - expected_dy
    ego_longitudinal_residual = (
        ego_residual_x * np.cos(ego_mid_yaw)
        + ego_residual_y * np.sin(ego_mid_yaw)
    )
    ego_lateral_residual = (
        -ego_residual_x * np.sin(ego_mid_yaw)
        + ego_residual_y * np.cos(ego_mid_yaw)
    )
    ego_oracle_valid = (
        (ego_dt > 1.0e-4)
        & (ego_dt < 0.1)
        & (np.abs(ego_mid_speed) > moving_speed_threshold)
    )

    loc_times = localization[:, 0]
    valid_time = (loc_times >= ego[0, 0]) & (loc_times <= ego[-1, 0])
    loc_times = loc_times[valid_time]
    localization = localization[valid_time]
    gt_x = np.interp(loc_times, ego[:, 0], ego_map_x)
    gt_y = np.interp(loc_times, ego[:, 0], ego_map_y)
    gt_yaw = np.interp(loc_times, ego[:, 0], ego_yaw)
    gt_speed = np.interp(loc_times, ego[:, 0], ego[:, 4])

    dx = localization[:, 1] - gt_x
    dy = localization[:, 2] - gt_y
    longitudinal = dx * np.cos(gt_yaw) + dy * np.sin(gt_yaw)
    lateral = -dx * np.sin(gt_yaw) + dy * np.cos(gt_yaw)
    yaw_error_deg = np.degrees(wrap_angle(localization[:, 3] - gt_yaw))
    moving = np.abs(gt_speed) > moving_speed_threshold

    loc_dt = np.diff(loc_times)
    adjacent = (loc_dt > 1.0e-4) & (loc_dt < 0.1) & moving[1:] & moving[:-1]
    lateral_jump = np.diff(lateral)[adjacent]
    longitudinal_jump = np.diff(longitudinal)[adjacent]

    gnss_odom = np.asarray(series["/localization/gnss/odom"], dtype=float)
    gnss_outages = []
    gnss_outage_windows = []
    normal_gnss = np.ones(loc_times.shape, dtype=bool)
    if gnss_odom.size:
        gnss_times = np.sort(gnss_odom[:, 0])
        gnss_gaps = np.diff(gnss_times)
        for index in np.where(gnss_gaps > 0.5)[0]:
            outage_start = float(gnss_times[index])
            outage_end = float(gnss_times[index + 1])
            # Keep the first second after reacquisition out of normal metrics;
            # this is where an abrupt global re-anchor reaches the controller.
            recovery_end = outage_end + 1.0
            normal_gnss &= ~(
                (loc_times >= outage_start) & (loc_times <= recovery_end)
            )
            gnss_outage_windows.append((outage_start, recovery_end))
            gnss_outages.append(
                {
                    "start_bag_offset_sec": outage_start - bag_start,
                    "end_bag_offset_sec": outage_end - bag_start,
                    "duration_sec": outage_end - outage_start,
                    "recovery_exclusion_end_bag_offset_sec": recovery_end - bag_start,
                }
            )

    normal_moving = moving & normal_gnss
    normal_adjacent = (
        (loc_dt > 1.0e-4)
        & (loc_dt < 0.1)
        & normal_moving[1:]
        & normal_moving[:-1]
    )

    steering = np.asarray(series["/control/command/steering"], dtype=float)
    lateral_control = np.asarray(
        series["/purepursuit_autoware/debug/lateral_error"], dtype=float
    )
    valid_series = series["/control/autoware/controller_valid"]
    if valid_series:
        valid_array = np.asarray(valid_series, dtype=float)
    else:
        valid_array = np.empty((0, 2), dtype=float)

    control_metrics = {"available": bool(steering.size)}
    moving_intervals = contiguous_intervals(ego[:, 5], np.abs(ego[:, 4]) > moving_speed_threshold)
    if steering.size:
        steering = steering[np.argsort(steering[:, 0])]
        steering_speed = np.interp(steering[:, 0], ego[:, 5], ego[:, 4])
        if valid_array.size:
            valid_at_steering = nearest_values(
                valid_array[:, 0], valid_array[:, 1].astype(bool), steering[:, 0], False
            ).astype(bool)
        else:
            valid_at_steering = np.ones(steering.shape[0], dtype=bool)
        steering_mask = (np.abs(steering_speed) > moving_speed_threshold) & valid_at_steering
        selected_steering = steering[:, 1][steering_mask]
        selected_times = steering[:, 0][steering_mask]
        dt = np.diff(selected_times)
        consecutive = (dt > 1.0e-4) & (dt < 0.1)
        steering_rate = np.diff(selected_steering)[consecutive] / dt[consecutive]
        sign_threshold = 0.01
        active = np.abs(selected_steering) > sign_threshold
        signs = np.sign(selected_steering[active])
        sign_times = selected_times[active]
        flips = int(np.count_nonzero(signs[1:] * signs[:-1] < 0)) if signs.size > 1 else 0
        active_duration = float(sign_times[-1] - sign_times[0]) if sign_times.size > 1 else 0.0
        control_metrics.update(
            {
                "valid_moving_samples": int(selected_steering.size),
                "steering_rad": scalar_summary(selected_steering, absolute=True),
                "steering_rate_radps": scalar_summary(steering_rate, absolute=True),
                "steering_sign_flips_threshold_rad": sign_threshold,
                "steering_sign_flip_count": flips,
                "steering_sign_flips_per_sec": flips / active_duration
                if active_duration > 0
                else None,
            }
        )

    if lateral_control.size:
        lateral_control = lateral_control[np.argsort(lateral_control[:, 0])]
        control_speed = np.interp(lateral_control[:, 0], ego[:, 5], ego[:, 4])
        if valid_array.size:
            valid_at_control = nearest_values(
                valid_array[:, 0],
                valid_array[:, 1].astype(bool),
                lateral_control[:, 0],
                False,
            ).astype(bool)
        else:
            valid_at_control = np.ones(lateral_control.shape[0], dtype=bool)
        control_mask = (np.abs(control_speed) > moving_speed_threshold) & valid_at_control
        control_metrics["controller_lateral_error_m"] = scalar_summary(
            lateral_control[:, 1][control_mask], absolute=True
        )

    rate_topics = [
        "/Ego_topic",
        "/Competition_topic",
        "/sensor/gps",
        "/sensor/imu",
        "/localization/gnss/odom",
        "/localization/kinematic_state",
        "/localization/control_kinematic_state",
        "/planning/trajectory",
    ]
    rates = {}
    for topic in rate_topics:
        values = series[topic]
        if topic == "/Ego_topic":
            times = [row[5] for row in values]
        elif topic in (
            "/localization/gnss/odom",
            "/localization/kinematic_state",
            "/localization/control_kinematic_state",
        ):
            times = [row[-1] for row in values]
        else:
            times = values
        rates[topic] = rate_summary(times)

    control_state_result = {"available": False}
    if series["/localization/control_kinematic_state"]:
        control_state = np.asarray(
            series["/localization/control_kinematic_state"], dtype=float
        )
        control_state = control_state[np.argsort(control_state[:, 0])]
        valid = (
            (control_state[:, 0] >= ego[0, 0])
            & (control_state[:, 0] <= ego[-1, 0])
        )
        control_state = control_state[valid]
        control_times = control_state[:, 0]
        control_gt_x = np.interp(control_times, ego[:, 0], ego_map_x)
        control_gt_y = np.interp(control_times, ego[:, 0], ego_map_y)
        control_gt_yaw = np.interp(control_times, ego[:, 0], ego_yaw)
        control_gt_speed = np.interp(control_times, ego[:, 0], ego[:, 4])
        control_dx = control_state[:, 1] - control_gt_x
        control_dy = control_state[:, 2] - control_gt_y
        control_longitudinal = (
            control_dx * np.cos(control_gt_yaw)
            + control_dy * np.sin(control_gt_yaw)
        )
        control_lateral = (
            -control_dx * np.sin(control_gt_yaw)
            + control_dy * np.cos(control_gt_yaw)
        )
        control_yaw_error = np.degrees(
            wrap_angle(control_state[:, 3] - control_gt_yaw)
        )
        control_moving = np.abs(control_gt_speed) > moving_speed_threshold
        control_normal = control_moving.copy()
        for outage_start, recovery_end in gnss_outage_windows:
            control_normal &= ~(
                (control_times >= outage_start)
                & (control_times <= recovery_end)
            )
        control_dt = np.diff(control_times)
        control_adjacent = (
            (control_dt > 1.0e-4)
            & (control_dt < 0.1)
            & control_moving[1:]
            & control_moving[:-1]
        )
        control_normal_adjacent = (
            (control_dt > 1.0e-4)
            & (control_dt < 0.1)
            & control_normal[1:]
            & control_normal[:-1]
        )

        def control_pose_summary(mask, adjacent_mask):
            return {
                "sample_count": int(np.count_nonzero(mask)),
                "longitudinal_error_m": scalar_summary(
                    control_longitudinal[mask], absolute=True
                ),
                "lateral_error_m": scalar_summary(
                    control_lateral[mask], absolute=True
                ),
                "yaw_error_deg": scalar_summary(
                    control_yaw_error[mask], absolute=True
                ),
                "longitudinal_frame_jump_m": scalar_summary(
                    np.diff(control_longitudinal)[adjacent_mask], absolute=True
                ),
                "lateral_frame_jump_m": scalar_summary(
                    np.diff(control_lateral)[adjacent_mask], absolute=True
                ),
            }

        control_state_result = {
            "available": True,
            "moving": control_pose_summary(control_moving, control_adjacent),
            "normal_gnss_moving": control_pose_summary(
                control_normal, control_normal_adjacent
            ),
        }

    tf_contract = {"available": False}
    local_odom = np.asarray(series["/localization/odometry/local"], dtype=float)
    map_to_odom = np.asarray(series["/tf"], dtype=float)
    if local_odom.size and map_to_odom.size:
        local_odom = local_odom[np.argsort(local_odom[:, 0])]
        map_to_odom = map_to_odom[np.argsort(map_to_odom[:, 0])]
        tf_times = map_to_odom[:, 0]
        valid = (
            (tf_times >= local_odom[0, 0])
            & (tf_times <= local_odom[-1, 0])
            & (tf_times >= localization[0, 0])
            & (tf_times <= localization[-1, 0])
        )
        map_to_odom = map_to_odom[valid]
        tf_times = map_to_odom[:, 0]
        local_x = np.interp(tf_times, local_odom[:, 0], local_odom[:, 1])
        local_y = np.interp(tf_times, local_odom[:, 0], local_odom[:, 2])
        local_yaw = np.interp(
            tf_times, local_odom[:, 0], np.unwrap(local_odom[:, 3])
        )
        global_x = np.interp(tf_times, localization[:, 0], localization[:, 1])
        global_y = np.interp(tf_times, localization[:, 0], localization[:, 2])
        global_yaw = np.interp(
            tf_times, localization[:, 0], np.unwrap(localization[:, 3])
        )
        tf_cosine = np.cos(map_to_odom[:, 3])
        tf_sine = np.sin(map_to_odom[:, 3])
        composed_x = (
            map_to_odom[:, 1]
            + tf_cosine * local_x
            - tf_sine * local_y
        )
        composed_y = (
            map_to_odom[:, 2]
            + tf_sine * local_x
            + tf_cosine * local_y
        )
        closure_position = np.hypot(
            composed_x - global_x, composed_y - global_y
        )
        closure_yaw = np.degrees(
            wrap_angle(map_to_odom[:, 3] + local_yaw - global_yaw)
        )
        tf_contract = {
            "available": True,
            "sample_count": int(tf_times.size),
            "description": (
                "Closure residual of recorded map->odom composed with co-timed "
                "local odometry against localization/kinematic_state."
            ),
            "position_closure_error_m": scalar_summary(
                closure_position, absolute=True
            ),
            "yaw_closure_error_deg": scalar_summary(
                closure_yaw, absolute=True
            ),
        }

    result = {
        "bag": str(path.resolve()),
        "duration_sec": float(bag_end - bag_start),
        "map_from_morai": {"x": tx, "y": ty, "yaw_rad": transform_yaw},
        "configuration": {
            "moving_speed_threshold_mps": moving_speed_threshold,
            "ego_heading_offset_deg": heading_offset_deg,
        },
        "rates": rates,
        "motion": {
            "moving_intervals_bag_time": moving_intervals,
            "moving_interval_count": len(moving_intervals),
            "moving_duration_sec": float(sum(end - start for start, end in moving_intervals)),
        },
        "ego_oracle_quality": {
            "description": (
                "Per-frame MORAI pose increment minus the increment predicted "
                "by its own velocity and heading. Large values form a lower "
                "bound on interpretable localization frame-jump metrics."
            ),
            "valid_moving_pair_count": int(np.count_nonzero(ego_oracle_valid)),
            "longitudinal_kinematic_residual_m": scalar_summary(
                ego_longitudinal_residual[ego_oracle_valid], absolute=True
            ),
            "lateral_kinematic_residual_m": scalar_summary(
                ego_lateral_residual[ego_oracle_valid], absolute=True
            ),
        },
        "localization_moving": {
            "sample_count": int(np.count_nonzero(moving)),
            "longitudinal_error_m": scalar_summary(longitudinal[moving], absolute=True),
            "lateral_error_m": scalar_summary(lateral[moving], absolute=True),
            "yaw_error_deg": scalar_summary(yaw_error_deg[moving], absolute=True),
            "longitudinal_frame_jump_m": scalar_summary(longitudinal_jump, absolute=True),
            "lateral_frame_jump_m": scalar_summary(lateral_jump, absolute=True),
        },
        "gnss_outages": gnss_outages,
        "localization_normal_gnss_moving": {
            "sample_count": int(np.count_nonzero(normal_moving)),
            "longitudinal_error_m": scalar_summary(
                longitudinal[normal_moving], absolute=True
            ),
            "lateral_error_m": scalar_summary(lateral[normal_moving], absolute=True),
            "yaw_error_deg": scalar_summary(yaw_error_deg[normal_moving], absolute=True),
            "longitudinal_frame_jump_m": scalar_summary(
                np.diff(longitudinal)[normal_adjacent], absolute=True
            ),
            "lateral_frame_jump_m": scalar_summary(
                np.diff(lateral)[normal_adjacent], absolute=True
            ),
        },
        "control_moving_valid": control_metrics,
        "control_state": control_state_result,
        "tf_contract": tf_contract,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--moving-speed-threshold", type=float, default=1.0)
    parser.add_argument("--heading-offset-deg", type=float, default=-1.347)
    args = parser.parse_args()

    result = analyze(
        args.bag,
        moving_speed_threshold=args.moving_speed_threshold,
        heading_offset_deg=args.heading_offset_deg,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()
