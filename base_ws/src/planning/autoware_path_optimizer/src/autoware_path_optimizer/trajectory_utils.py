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


def unique_points(points):
    output = []
    for point in points:
        if output and distance_2d(output[-1], point) < EPSILON:
            output[-1] = point
        else:
            output.append(point)
    return output


def interpolate_samples(points, targets):
    points = unique_points(points)
    if len(points) < 2:
        return list(points)
    stations = cumulative_lengths(points)
    targets = np.clip(np.asarray(targets, dtype=float), 0.0, float(stations[-1]))
    xs = np.asarray([point.x for point in points], dtype=float)
    ys = np.asarray([point.y for point in points], dtype=float)
    zs = np.asarray([point.z for point in points], dtype=float)
    if len(points) >= 3:
        output_x = CubicSpline(stations, xs)(targets)
        output_y = CubicSpline(stations, ys)(targets)
    else:
        output_x = np.interp(targets, stations, xs)
        output_y = np.interp(targets, stations, ys)
    output_z = np.interp(targets, stations, zs)
    if len(targets) > 1:
        yaw = np.arctan2(
            np.gradient(output_y, targets, edge_order=1),
            np.gradient(output_x, targets, edge_order=1),
        )
    else:
        yaw = np.asarray([points[0].yaw], dtype=float)
    output = []
    for index, station in enumerate(targets):
        source_index = int(np.searchsorted(stations, station, side="right") - 1)
        source_index = min(max(source_index, 0), len(points) - 1)
        output.append(
            replace(
                points[source_index],
                x=float(output_x[index]),
                y=float(output_y[index]),
                z=float(output_z[index]),
                yaw=float(yaw[index]),
            )
        )
    return output


def resample(points, interval, preserve_stop=True):
    points = unique_points(points)
    if len(points) < 2:
        return list(points)
    stations = cumulative_lengths(points)
    interval = max(float(interval), 0.05)
    targets = list(np.arange(0.0, float(stations[-1]) + EPSILON, interval))
    targets.append(float(stations[-1]))
    stop_stations = []
    if preserve_stop:
        stop_stations = [
            float(stations[index])
            for index, point in enumerate(points)
            if abs(point.longitudinal_velocity_mps) < 1.0e-4
            and (index == 0 or abs(points[index - 1].longitudinal_velocity_mps) >= 1.0e-4)
        ]
        targets.extend(stop_stations)
    targets = np.asarray(sorted(set(round(value, 9) for value in targets)), dtype=float)
    output = interpolate_samples(points, targets)
    for station in stop_stations:
        index = int(np.argmin(np.abs(targets - station)))
        output[index] = replace(output[index], longitudinal_velocity_mps=0.0)
    return output


def project_point(points, x, y):
    if len(points) < 2:
        distance = math.hypot(points[0].x - x, points[0].y - y) if points else math.inf
        return 0, 0.0, distance, (points[0].x, points[0].y) if points else (x, y)
    stations = cumulative_lengths(points)
    best = None
    for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
        dx = second.x - first.x
        dy = second.y - first.y
        length_sq = dx * dx + dy * dy
        if length_sq < EPSILON:
            continue
        ratio = max(0.0, min(1.0, ((x - first.x) * dx + (y - first.y) * dy) / length_sq))
        projected_x = first.x + ratio * dx
        projected_y = first.y + ratio * dy
        distance = math.hypot(x - projected_x, y - projected_y)
        value = (
            distance,
            index,
            float(stations[index] + math.sqrt(length_sq) * ratio),
            (projected_x, projected_y),
        )
        if best is None or value[:3] < best[:3]:
            best = value
    if best is None:
        return 0, 0.0, math.inf, (x, y)
    return best[1], best[2], best[0], best[3]


def crop(points, ego_x, ego_y, forward_length, backward_length):
    if len(points) < 2:
        return list(points)
    _, ego_station, _, _ = project_point(points, ego_x, ego_y)
    stations = cumulative_lengths(points)
    start = max(0.0, ego_station - max(float(backward_length), 0.0))
    end = min(float(stations[-1]), ego_station + max(float(forward_length), 0.0))
    start_index = min(int(np.searchsorted(stations, start, side="left")), len(points) - 2)
    end_index = max(int(np.searchsorted(stations, end, side="right")), start_index + 2)
    return list(points[start_index:min(end_index, len(points))])


def nearest_point_on_polyline(polyline, x, y):
    samples = [TrajectorySample(point[0], point[1]) for point in polyline]
    return project_point(samples, x, y)[3] if samples else (x, y)


def apply_input_velocity(output, source):
    result = []
    if not output or not source:
        return list(output)
    for point in output:
        segment, _, _, _ = project_point(source, point.x, point.y)
        segment = min(max(segment, 0), len(source) - 1)
        value = source[segment]
        result.append(
            replace(
                point,
                longitudinal_velocity_mps=value.longitudinal_velocity_mps,
                lateral_velocity_mps=value.lateral_velocity_mps,
                acceleration_mps2=value.acceleration_mps2,
            )
        )
    stop = next((point for point in source if abs(point.longitudinal_velocity_mps) < 1.0e-4), None)
    if stop is None:
        return result
    segment, station, _, _ = project_point(result, stop.x, stop.y)
    result_stations = cumulative_lengths(result)
    if not len(result_stations):
        return result
    if abs(float(result_stations[min(segment, len(result_stations) - 1)]) - station) < 1.0e-3:
        result[segment] = replace(result[segment], longitudinal_velocity_mps=0.0)
    else:
        inserted = interpolate_samples(result, [station])[0]
        result.insert(segment + 1, replace(inserted, longitudinal_velocity_mps=0.0))
    return result


def set_zero_velocity_after(points, start_index):
    return [
        replace(point, longitudinal_velocity_mps=0.0) if index >= start_index else point
        for index, point in enumerate(points)
    ]


def set_zero_velocity_after_first_stop(points):
    stop_index = next(
        (index for index, point in enumerate(points) if abs(point.longitudinal_velocity_mps) < 1.0e-4),
        None,
    )
    return list(points) if stop_index is None else set_zero_velocity_after(points, stop_index)


def join_with_remaining_path(source, optimized, output_interval):
    if not source or not optimized:
        return list(optimized)
    _, joint_station, _, _ = project_point(source, optimized[-1].x, optimized[-1].y)
    source_stations = cumulative_lengths(source)
    total = float(source_stations[-1])
    if total - joint_station < 3.0:
        return resample(optimized, output_interval, preserve_stop=True)
    extension_station = min(total, joint_station + 10.0)
    extension_index = int(np.searchsorted(source_stations, extension_station, side="left"))
    extension_index = min(max(extension_index, 0), len(source) - 1)
    return resample(list(optimized) + list(source[extension_index:]), output_interval, True)


def curvature(points):
    if len(points) < 3:
        return [0.0] * len(points)
    values = [0.0]
    for first, middle, last in zip(points[:-2], points[1:-1], points[2:]):
        a = distance_2d(first, middle)
        b = distance_2d(middle, last)
        c = distance_2d(first, last)
        denominator = max(a * b * c, EPSILON)
        cross = (middle.x - first.x) * (last.y - first.y) - (middle.y - first.y) * (last.x - first.x)
        values.append(2.0 * cross / denominator)
    values.append(values[-1])
    values[0] = values[1]
    return values
