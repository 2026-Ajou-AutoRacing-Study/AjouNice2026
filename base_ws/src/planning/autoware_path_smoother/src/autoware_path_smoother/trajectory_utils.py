#!/usr/bin/env python3

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.interpolate import CubicSpline


EPSILON = 1.0e-6


@dataclass
class TrajectorySample:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    longitudinal_velocity_mps: float = 0.0
    lateral_velocity_mps: float = 0.0
    acceleration_mps2: float = 0.0
    heading_rate_rps: float = 0.0
    front_wheel_angle_rad: float = 0.0
    rear_wheel_angle_rad: float = 0.0
    is_final: bool = False


def distance_2d(first, second):
    return math.hypot(second.x - first.x, second.y - first.y)


def cumulative_lengths(points):
    if not points:
        return np.asarray([], dtype=float)
    values = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        values.append(values[-1] + distance_2d(first, second))
    return np.asarray(values, dtype=float)


def _unique_points(points):
    output = []
    for point in points:
        if output and distance_2d(output[-1], point) < EPSILON:
            output[-1] = point
        else:
            output.append(point)
    return output


def _target_stations(total, interval, extra_stations=None):
    interval = max(float(interval), 0.01)
    values = list(np.arange(0.0, max(total, 0.0) + EPSILON, interval))
    values.extend(float(value) for value in (extra_stations or []))
    values.append(float(total))
    return np.asarray(sorted(set(round(max(0.0, min(total, value)), 9) for value in values)))


def _yaw_from_tangent(xs, ys, stations, source_points=None, source_stations=None):
    if len(stations) < 2:
        if source_points and source_stations is not None:
            source_yaws = np.unwrap(
                np.asarray([point.yaw for point in source_points], dtype=float)
            )
            return np.interp(stations, source_stations, source_yaws)
        return np.zeros(len(stations), dtype=float)
    dx = np.gradient(xs, stations, edge_order=1)
    dy = np.gradient(ys, stations, edge_order=1)
    return np.arctan2(dy, dx)


def interpolate_samples(points, target_stations):
    points = _unique_points(points)
    if len(points) < 2:
        return list(points)

    stations = cumulative_lengths(points)
    total = float(stations[-1])
    targets = np.asarray(target_stations, dtype=float)
    targets = np.clip(targets, 0.0, total)
    x_values = np.asarray([point.x for point in points], dtype=float)
    y_values = np.asarray([point.y for point in points], dtype=float)
    z_values = np.asarray([point.z for point in points], dtype=float)

    if len(points) >= 3:
        xs = CubicSpline(stations, x_values)(targets)
        ys = CubicSpline(stations, y_values)(targets)
    else:
        xs = np.interp(targets, stations, x_values)
        ys = np.interp(targets, stations, y_values)
    zs = np.interp(targets, stations, z_values)
    yaws = _yaw_from_tangent(xs, ys, targets, points, stations)

    output = []
    for index, target in enumerate(targets):
        source_index = min(int(np.searchsorted(stations, target, side="right") - 1), len(points) - 1)
        source_index = max(source_index, 0)
        source = points[source_index]
        output.append(
            replace(
                source,
                x=float(xs[index]),
                y=float(ys[index]),
                z=float(zs[index]),
                yaw=float(yaws[index]),
            )
        )
    return output


def resample(points, interval, preserve_stop=True):
    points = _unique_points(points)
    if len(points) < 2:
        return list(points)
    stations = cumulative_lengths(points)
    stop_stations = []
    if preserve_stop:
        stop_stations.extend(
            float(stations[index])
            for index, point in enumerate(points)
            if abs(point.longitudinal_velocity_mps) < 1.0e-4
            and (index == 0 or abs(points[index - 1].longitudinal_velocity_mps) >= 1.0e-4)
        )
    targets = _target_stations(stations[-1], interval)
    for station in stop_stations:
        nearest_index = int(np.argmin(np.abs(targets - station)))
        # Autoware's trajectory resampler coalesces a stop with a very close
        # regular sample; otherwise the stop station is inserted explicitly.
        if abs(targets[nearest_index] - station) <= max(float(interval) * 0.1, 1.0e-3):
            targets[nearest_index] = station
        else:
            targets = np.append(targets, station)
        targets = np.asarray(sorted(set(round(float(value), 9) for value in targets)))
    output = interpolate_samples(points, targets)
    for station in stop_stations:
        index = int(np.argmin(np.abs(targets - station)))
        output[index] = replace(output[index], longitudinal_velocity_mps=0.0)
    return output


def project_point(points, x, y):
    if len(points) < 2:
        return 0, 0.0, math.hypot(points[0].x - x, points[0].y - y) if points else math.inf
    stations = cumulative_lengths(points)
    best = None
    for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
        dx = second.x - first.x
        dy = second.y - first.y
        length_sq = dx * dx + dy * dy
        if length_sq < EPSILON:
            continue
        ratio = max(0.0, min(1.0, ((x - first.x) * dx + (y - first.y) * dy) / length_sq))
        px = first.x + ratio * dx
        py = first.y + ratio * dy
        distance = math.hypot(x - px, y - py)
        value = (distance, index, float(stations[index] + math.sqrt(length_sq) * ratio))
        if best is None or value < best:
            best = value
    return (best[1], best[2], best[0]) if best is not None else (0, 0.0, math.inf)


def crop(points, ego_x, ego_y, forward_length, backward_length):
    if len(points) < 2:
        return list(points)
    _, ego_station, _ = project_point(points, ego_x, ego_y)
    stations = cumulative_lengths(points)
    start = max(0.0, ego_station - max(float(backward_length), 0.0))
    end = min(float(stations[-1]), ego_station + max(float(forward_length), 0.0))
    # motion_utils::cropPoints keeps original samples inside the requested
    # window. It does not insert interpolated points at the crop boundaries.
    start_index = int(np.searchsorted(stations, start, side="left"))
    end_index = int(np.searchsorted(stations, end, side="right"))
    start_index = min(start_index, len(points) - 2)
    end_index = max(end_index, start_index + 2)
    return list(points[start_index:min(end_index, len(points))])


def nearest_segment(points, target):
    return project_point(points, target.x, target.y)[0]


def insert_stop_point(points, input_stop):
    if len(points) < 2:
        return list(points)
    segment_index, station, _ = project_point(points, input_stop.x, input_stop.y)
    stations = cumulative_lengths(points)
    if station < EPSILON or station > stations[-1] + EPSILON:
        return list(points)
    output = list(points)
    for index in (segment_index, min(segment_index + 1, len(output) - 1)):
        if distance_2d(output[index], input_stop) < 1.0e-3:
            output[index] = replace(output[index], longitudinal_velocity_mps=0.0)
            return output
    stop = interpolate_samples(points, np.asarray([station], dtype=float))[0]
    stop = replace(stop, longitudinal_velocity_mps=0.0)
    output.insert(segment_index + 1, stop)
    return output


def set_zero_velocity_after_stop(points):
    output = list(points)
    stop_index = next(
        (index for index, point in enumerate(output) if abs(point.longitudinal_velocity_mps) < 1.0e-4),
        None,
    )
    if stop_index is None:
        return output
    return [
        replace(point, longitudinal_velocity_mps=0.0) if index >= stop_index else point
        for index, point in enumerate(output)
    ]


def apply_input_velocity(output, source):
    if not output or not source:
        return list(output)
    result = []
    for point in output:
        segment_index, _, _ = project_point(source, point.x, point.y)
        segment_index = min(segment_index, len(source) - 1)
        result.append(
            replace(
                point,
                longitudinal_velocity_mps=source[segment_index].longitudinal_velocity_mps,
                lateral_velocity_mps=source[segment_index].lateral_velocity_mps,
                acceleration_mps2=source[segment_index].acceleration_mps2,
                heading_rate_rps=source[segment_index].heading_rate_rps,
            )
        )
    stop = next((point for point in source if abs(point.longitudinal_velocity_mps) < 1.0e-4), None)
    return insert_stop_point(result, stop) if stop is not None else result


def extend_trajectory(source, optimized, output_interval):
    if not source or not optimized:
        return list(optimized)
    _, joint_station, _ = project_point(source, optimized[-1].x, optimized[-1].y)
    source_stations = cumulative_lengths(source)
    total = float(source_stations[-1])
    if total - joint_station < 5.0:
        return resample(optimized, output_interval, preserve_stop=True)

    extension_start = min(total, joint_station + 15.0)
    start_index = int(np.searchsorted(source_stations, extension_start, side="left"))
    start_index = min(max(start_index, 0), len(source) - 1)
    extended = list(optimized) + list(source[start_index:])
    output = resample(extended, output_interval, preserve_stop=True)
    stop = next((point for point in source if abs(point.longitudinal_velocity_mps) < 1.0e-4), None)
    if stop is not None:
        output = insert_stop_point(output, stop)
    return output
