#!/usr/bin/env python3
"""Geometry helpers for MORAI temporal PointPainting datasets."""

import math
from collections import defaultdict

import numpy as np


def normalize_angle(value):
    return np.arctan2(np.sin(value), np.cos(value))


def estimate_clockwise_point_offsets(
    points,
    scan_period_sec,
    start_azimuth_rad=0.0,
):
    """Estimate acquisition offsets from MORAI's clockwise LiDAR azimuth.

    SaveSensorData stores 1,200 azimuth groups of 16 channels.  The first
    group is at zero degrees and subsequent groups rotate clockwise by about
    0.3 degrees.  The estimator uses coordinates instead of point indices so
    it remains valid after range/ego filtering and semantic painting.
    """
    values = np.asarray(points)
    if values.ndim != 2 or values.shape[1] < 2:
        raise ValueError("points must have shape [N, >=2]")
    azimuth = np.mod(np.arctan2(values[:, 1], values[:, 0]), 2.0 * np.pi)
    phase = np.mod(float(start_azimuth_rad) - azimuth, 2.0 * np.pi)
    return phase / (2.0 * np.pi) * float(scan_period_sec)


def pose_from_frame(frame):
    ego = frame["ego"]
    position = ego["position_m"]
    return np.asarray(
        [
            float(position["x"]),
            float(position["y"]),
            float(position["z"]),
            math.radians(float(ego["heading_deg"])),
        ],
        dtype=np.float64,
    )


def frame_timestamp(frame):
    if frame.get("simulation_time_sec") is not None:
        return float(frame["simulation_time_sec"])
    ego = frame.get("ego") or {}
    if ego.get("timestamp") is not None:
        return float(ego["timestamp"])
    return float(frame["triggered_unix"])


def _position_xy(state):
    position = state.get("position_m") or {}
    return float(position.get("x", 0.0)), float(position.get("y", 0.0))


def _interval_motion_time(left, right):
    """Estimate simulator motion time between two captured snapshots.

    The personal launcher can run below real time while ROS header stamps keep
    advancing in wall time. EgoVehicleStatus velocity is m/s in this capture,
    whereas the launcher's ObjectStatus velocity is km/h. Position/speed gives
    the physical simulator interval without depending on the real-time factor.
    """
    candidates = []
    left_ego, right_ego = left.get("ego"), right.get("ego")
    if left_ego and right_ego:
        x0, y0 = _position_xy(left_ego)
        x1, y1 = _position_xy(right_ego)
        speed = 0.5 * (
            float(left_ego.get("speed_raw", 0.0))
            + float(right_ego.get("speed_raw", 0.0))
        )
        if speed > 0.5:
            candidates.append(math.hypot(x1 - x0, y1 - y0) / speed)

    left_actors = {
        int(actor["unique_id"]): actor
        for actor in ((left.get("objects") or {}).get("actors") or [])
    }
    right_actors = {
        int(actor["unique_id"]): actor
        for actor in ((right.get("objects") or {}).get("actors") or [])
    }
    for unique_id in left_actors.keys() & right_actors.keys():
        actor0, actor1 = left_actors[unique_id], right_actors[unique_id]
        x0, y0 = _position_xy(actor0)
        x1, y1 = _position_xy(actor1)
        # MORAI personal-launcher ObjectStatus uses km/h here.
        speed = 0.5 * (
            float(actor0.get("morai_speed_raw", 0.0))
            + float(actor1.get("morai_speed_raw", 0.0))
        ) / 3.6
        if speed > 0.5:
            candidates.append(math.hypot(x1 - x0, y1 - y0) / speed)
    candidates = [value for value in candidates if 0.005 <= value <= 1.0]
    return float(np.median(candidates)) if candidates else None


def annotate_simulation_times(frames):
    """Attach a per-scene physical simulator time axis to capture metadata."""
    grouped = defaultdict(list)
    for frame in frames:
        grouped[frame["scene_id"]].append(frame)
    diagnostics = []
    for scene_id, scene_frames in grouped.items():
        ordered = sorted(scene_frames, key=lambda row: int(row["frame_index"]))
        intervals = [_interval_motion_time(a, b) for a, b in zip(ordered, ordered[1:])]
        observed = [value for value in intervals if value is not None]
        wall_intervals = [
            max(1.0e-3, float(b["triggered_unix"]) - float(a["triggered_unix"]))
            for a, b in zip(ordered, ordered[1:])
        ]
        ratios = [
            value / wall
            for value, wall in zip(intervals, wall_intervals)
            if value is not None
        ]
        fallback_ratio = float(np.median(ratios)) if ratios else 1.0
        current = 0.0
        if ordered:
            ordered[0]["simulation_time_sec"] = current
        for index, frame in enumerate(ordered[1:]):
            dt = intervals[index]
            if dt is None:
                dt = wall_intervals[index] * fallback_ratio
            current += float(dt)
            frame["simulation_time_sec"] = current
        diagnostics.append(
            {
                "scene_id": scene_id,
                "frames": len(ordered),
                "observed_intervals": len(observed),
                "fallback_intervals": len(intervals) - len(observed),
                "median_interval_sec": None if not intervals else float(
                    np.median([
                        value if value is not None else wall * fallback_ratio
                        for value, wall in zip(intervals, wall_intervals)
                    ])
                ),
                "wall_to_sim_ratio": fallback_ratio,
            }
        )
    return diagnostics


def derive_actor_velocities(frames):
    """Derive absolute actor velocity from positions on simulator time."""
    annotate_simulation_times(frames)
    grouped = defaultdict(list)
    frame_by_key = {}
    for frame in frames:
        for actor in ((frame.get("objects") or {}).get("actors") or []):
            key = (frame["scene_id"], int(actor["unique_id"]))
            grouped[key].append((frame_timestamp(frame), actor, frame))
            frame_by_key[(frame["frame_id"], int(actor["unique_id"]))] = actor
    for track in grouped.values():
        track.sort(key=lambda item: item[0])
        for index, (_timestamp, actor, frame) in enumerate(track):
            left = track[max(0, index - 1)]
            right = track[min(len(track) - 1, index + 1)]
            dt = right[0] - left[0]
            if dt <= 1.0e-4:
                continue
            x0, y0 = _position_xy(left[1])
            x1, y1 = _position_xy(right[1])
            map_vx, map_vy = (x1 - x0) / dt, (y1 - y0) / dt
            ego = frame.get("ego") or {}
            yaw = math.radians(float(ego.get("heading_deg", 0.0)))
            cosine, sine = math.cos(yaw), math.sin(yaw)
            actor["simulation_velocity_lidar_mps"] = {
                "x": cosine * map_vx + sine * map_vy,
                "y": -sine * map_vx + cosine * map_vy,
            }
    return frame_by_key


def propagate_ego_pose(frame, offsets_sec):
    """Propagate the frame pose over one physical LiDAR scan.

    Capture callback timestamps advance in wall/ROS time while a loaded MORAI
    simulation can run below real time.  ``derived_velocity_mps`` is therefore
    correct for aligning two saved snapshots but underestimates motion inside
    the simulator's fixed 66.7 ms LiDAR scan. Prefer EgoVehicleStatus' local
    physical velocity for intra-scan motion and use the measured path curvature
    to scale yaw rate to that physical speed.
    """
    offsets = np.asarray(offsets_sec, dtype=np.float64)
    reference = pose_from_frame(frame)
    ego = frame["ego"]
    raw_velocity = ego.get("velocity_raw") or {}
    local_vx = float(raw_velocity.get("x", 0.0))
    local_vy = float(raw_velocity.get("y", 0.0))
    raw_speed = math.hypot(local_vx, local_vy)
    if raw_speed > 1.0e-3:
        cosine = math.cos(reference[3])
        sine = math.sin(reference[3])
        vx = cosine * local_vx - sine * local_vy
        vy = sine * local_vx + cosine * local_vy
        vz = 0.0
    else:
        velocity = ego.get("derived_velocity_mps") or {}
        vx = float(velocity.get("x", 0.0))
        vy = float(velocity.get("y", 0.0))
        vz = float(velocity.get("z", 0.0))
        raw_speed = math.hypot(vx, vy)

    derived_speed = float(ego.get("derived_speed_mps", 0.0))
    derived_yaw_rate = math.radians(float(ego.get("derived_yaw_rate_dps", 0.0)))
    yaw_rate = derived_yaw_rate
    if derived_speed > 0.1 and raw_speed > 0.1:
        yaw_rate *= raw_speed / derived_speed
    result = np.empty((len(offsets), 4), dtype=np.float64)
    result[:, 0] = reference[0] + vx * offsets
    result[:, 1] = reference[1] + vy * offsets
    result[:, 2] = reference[2] + vz * offsets
    result[:, 3] = normalize_angle(reference[3] + yaw_rate * offsets)
    return result


def lidar_world_pose(ego_poses, lidar_position, lidar_yaw_rad=0.0):
    poses = np.asarray(ego_poses, dtype=np.float64).reshape(-1, 4)
    lidar_position = np.asarray(lidar_position, dtype=np.float64)
    cosine = np.cos(poses[:, 3])
    sine = np.sin(poses[:, 3])
    return np.column_stack(
        (
            poses[:, 0]
            + cosine * lidar_position[0]
            - sine * lidar_position[1],
            poses[:, 1]
            + sine * lidar_position[0]
            + cosine * lidar_position[1],
            poses[:, 2] + lidar_position[2],
            normalize_angle(poses[:, 3] + float(lidar_yaw_rad)),
        )
    )


def transform_points_to_target(
    points,
    source_frame,
    target_frame,
    point_offsets_sec,
    lidar_position=(0.855, 0.0, 1.388),
    lidar_yaw_rad=0.0,
    deskew_ego_motion=False,
):
    """Express one rolling sweep in the target frame's reference LiDAR pose.

    With deskew disabled, every source point uses the source reference pose.
    With deskew enabled, the source pose is propagated to each estimated point
    time.  Object motion is intentionally not corrected.
    """
    values = np.asarray(points)
    offsets = np.asarray(point_offsets_sec, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 3 or len(values) != len(offsets):
        raise ValueError("point/offset shape mismatch")

    if deskew_ego_motion:
        source_ego = propagate_ego_pose(source_frame, offsets)
    else:
        source_ego = np.repeat(pose_from_frame(source_frame)[None, :], len(values), axis=0)
    source_lidar = lidar_world_pose(source_ego, lidar_position, lidar_yaw_rad)
    target_lidar = lidar_world_pose(
        pose_from_frame(target_frame)[None, :], lidar_position, lidar_yaw_rad
    )[0]

    source_cos = np.cos(source_lidar[:, 3])
    source_sin = np.sin(source_lidar[:, 3])
    world_x = source_lidar[:, 0] + source_cos * values[:, 0] - source_sin * values[:, 1]
    world_y = source_lidar[:, 1] + source_sin * values[:, 0] + source_cos * values[:, 1]
    world_z = source_lidar[:, 2] + values[:, 2]

    dx = world_x - target_lidar[0]
    dy = world_y - target_lidar[1]
    target_cos = math.cos(float(target_lidar[3]))
    target_sin = math.sin(float(target_lidar[3]))
    output = np.array(values, copy=True)
    output[:, 0] = target_cos * dx + target_sin * dy
    output[:, 1] = -target_sin * dx + target_cos * dy
    output[:, 2] = world_z - target_lidar[2]
    return output


def temporalize_sweep(
    points,
    source_frame,
    target_frame,
    scan_period_sec,
    lidar_position=(0.855, 0.0, 1.388),
    lidar_yaw_rad=0.0,
    deskew_ego_motion=False,
):
    offsets = estimate_clockwise_point_offsets(points, scan_period_sec)
    transformed = transform_points_to_target(
        points,
        source_frame,
        target_frame,
        offsets,
        lidar_position=lidar_position,
        lidar_yaw_rad=lidar_yaw_rad,
        deskew_ego_motion=deskew_ego_motion,
    )
    time_lag = (
        frame_timestamp(source_frame)
        + offsets
        - frame_timestamp(target_frame)
    ).astype(np.float32)
    return np.concatenate(
        (transformed.astype(np.float32, copy=False), time_lag[:, None]), axis=1
    )
