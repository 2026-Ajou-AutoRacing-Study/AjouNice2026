#!/usr/bin/env python3
"""Strict centerline stitching and arc-length resampling utilities."""

import math
from competition_route_loader import RouteLoadError


def distance(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def stitch(rows, gap_limit):
    result = []
    for row in rows:
        points = list(reversed(row["centerline"])) if row.get("reverse") else list(row["centerline"])
        if result:
            gap = distance(result[-1][0], points[0])
            if gap > gap_limit:
                raise RouteLoadError(
                    "stitch gap %.3f m exceeds %.3f m between lanelet %s and %s" %
                    (gap, gap_limit, result[-1][1]["lanelet_id"], row["lanelet_id"])
                )
            if gap < 1e-5:
                points = points[1:]
        for point in points:
            result.append((point, row))
    if len(result) < 2:
        raise RouteLoadError("stitched route has fewer than two points")
    return result


def resample(stitched, ds):
    if ds <= 0.0:
        raise RouteLoadError("resample_ds_m must be positive")
    cumulative = [0.0]
    for i in range(1, len(stitched)):
        cumulative.append(cumulative[-1] + distance(stitched[i - 1][0], stitched[i][0]))
    total = cumulative[-1]
    if total <= 0.0:
        raise RouteLoadError("route has zero length")
    targets = [0.0]
    s = ds
    while s < total:
        targets.append(s)
        s += ds
    if total - targets[-1] > 1e-6:
        targets.append(total)
    result, segment = [], 0
    for target in targets:
        while segment + 1 < len(cumulative) - 1 and cumulative[segment + 1] < target:
            segment += 1
        start_s, end_s = cumulative[segment], cumulative[segment + 1]
        ratio = 0.0 if end_s <= start_s else (target - start_s) / (end_s - start_s)
        a, b = stitched[segment][0], stitched[segment + 1][0]
        xyz = tuple(a[i] + ratio * (b[i] - a[i]) for i in range(3))
        result.append({"s_m": target, "xyz": xyz, "lanelet": stitched[segment][1]})
    return result


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def annotate_geometry(points, wheelbase):
    if len(points) < 2:
        raise RouteLoadError("cannot annotate one-point trajectory")
    yaws = []
    for i in range(len(points)):
        a = points[max(0, i - 1)]["xyz"]
        b = points[min(len(points) - 1, i + 1)]["xyz"]
        yaws.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    for i, point in enumerate(points):
        if i == 0:
            ds = max(points[1]["s_m"] - point["s_m"], 1e-6)
            curvature = wrap(yaws[1] - yaws[0]) / ds
        elif i == len(points) - 1:
            ds = max(point["s_m"] - points[i - 1]["s_m"], 1e-6)
            curvature = wrap(yaws[-1] - yaws[-2]) / ds
        else:
            ds = max(points[i + 1]["s_m"] - points[i - 1]["s_m"], 1e-6)
            curvature = wrap(yaws[i + 1] - yaws[i - 1]) / ds
        point["yaw_rad"] = yaws[i]
        point["curvature"] = curvature
        point["front_wheel_angle_rad"] = math.atan(wheelbase * curvature)
    return points
