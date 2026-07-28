#!/usr/bin/env python3
"""Generate a high-speed constant-curvature lane change between adjacent lanes."""

import argparse
import bisect
import json
import math
from pathlib import Path

import generate_constant_curvature_return as curvature
import generate_trajectory as base


DEFAULT_START_LANELET_ID = 200137
DEFAULT_END_LANELET_ID = 200188
DEFAULT_SPEEDS_KPH = tuple(range(60, 171, 10))
SAMPLE_INTERVAL_M = 0.1


def cumulative_lengths(points):
    stations = [0.0]
    for first, second in zip(points, points[1:]):
        distance = math.dist(first, second)
        if distance <= 1.0e-9:
            raise ValueError("centerline contains duplicate consecutive points")
        stations.append(stations[-1] + distance)
    return stations


def interpolate(points, stations, station):
    station = min(max(float(station), 0.0), stations[-1])
    right = bisect.bisect_right(stations, station)
    segment = min(max(right - 1, 0), len(points) - 2)
    start_station = stations[segment]
    end_station = stations[segment + 1]
    ratio = (station - start_station) / (end_station - start_station)
    first = points[segment]
    second = points[segment + 1]
    return tuple(
        first[axis] + ratio * (second[axis] - first[axis])
        for axis in range(3)
    )


def tangent_yaw(points, stations, station):
    delta = min(0.5, 0.25 * stations[-1])
    before = interpolate(points, stations, max(0.0, station - delta))
    after = interpolate(points, stations, min(stations[-1], station + delta))
    if math.hypot(after[0] - before[0], after[1] - before[1]) <= 1.0e-9:
        raise ValueError("cannot determine centerline tangent")
    return math.atan2(after[1] - before[1], after[0] - before[0])


def sample_polyline_prefix(
    points,
    stations,
    length_m,
    interval_m,
    velocity_mps,
    lanelet_id,
    route_order,
):
    sample_stations = curvature.sample_stations(length_m, interval_m)
    output = []
    for station in sample_stations:
        point = interpolate(points, stations, station)
        output.append(
            {
                "index": len(output),
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "yaw": tangent_yaw(points, stations, station),
                "velocity_mps": velocity_mps,
                "acceleration_mps2": 0.0,
                "lanelet_id": lanelet_id,
                "route_order": route_order,
            }
        )
    return output


def sample_polyline_suffix(
    points,
    stations,
    start_station_m,
    interval_m,
    velocity_mps,
    lanelet_id,
    route_order,
):
    remaining_length_m = stations[-1] - start_station_m
    sample_stations = curvature.sample_stations(
        remaining_length_m, interval_m
    )
    output = []
    for offset_m in sample_stations:
        station = start_station_m + offset_m
        point = interpolate(points, stations, station)
        output.append(
            {
                "index": len(output),
                "x": point[0],
                "y": point[1],
                "z": point[2],
                "yaw": tangent_yaw(points, stations, station),
                "velocity_mps": velocity_mps,
                "acceleration_mps2": 0.0,
                "lanelet_id": lanelet_id,
                "route_order": route_order,
            }
        )
    return output


def lane_change_width(kappa_mag, segment_length_m):
    if kappa_mag < curvature.CURVATURE_EPSILON:
        return 0.0
    angle = kappa_mag * segment_length_m
    return 2.0 * (1.0 - math.cos(angle)) / kappa_mag


def solve_length_for_width(target_width_m, kappa_mag):
    target_width_m = base.positive_finite(
        target_width_m, "target_width_m"
    )
    kappa_mag = base.positive_finite(kappa_mag, "kappa_mag")
    argument = 1.0 - 0.5 * kappa_mag * target_width_m
    if argument < -1.0 or argument > 1.0:
        raise ValueError("target width cannot be reached with this curvature")
    length_m = math.acos(argument) / kappa_mag
    if kappa_mag * length_m >= math.pi / 2.0:
        raise ValueError(
            "target width requires delta_theta >= 90 deg for this curvature"
        )
    return length_m


def relative_lateral_offset(origin, yaw, target):
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    return -math.sin(yaw) * dx + math.cos(yaw) * dy


def kappa_filename_token(kappa_mag):
    text = "{:.6f}".format(kappa_mag).rstrip("0").rstrip(".")
    return text.replace(".", "p")


def build_lane_change_geometry(
    source_points,
    target_points,
    kappa_mag,
):
    source_stations = cumulative_lengths(source_points)
    target_stations = cumulative_lengths(target_points)
    start = source_points[-1]
    start_yaw = tangent_yaw(
        source_points, source_stations, source_stations[-1]
    )
    target_at_start = target_points[0]
    signed_width = relative_lateral_offset(
        start, start_yaw, target_at_start
    )
    direction_sign = 1 if signed_width > 0.0 else -1
    direction = "left" if direction_sign > 0 else "right"
    target_width = abs(signed_width)

    kappa_mag = base.positive_finite(kappa_mag, "kappa_mag")
    segment_length_m = solve_length_for_width(
        target_width, kappa_mag
    )

    angle = kappa_mag * segment_length_m
    longitudinal_displacement_m = 2.0 * math.sin(angle) / kappa_mag
    if longitudinal_displacement_m > target_stations[-1]:
        raise ValueError(
            "lane-change longitudinal displacement exceeds target lanelet: "
            "{:.3f} > {:.3f} m".format(
                longitudinal_displacement_m, target_stations[-1]
            )
        )
    target_station = longitudinal_displacement_m
    target = interpolate(target_points, target_stations, target_station)
    if angle >= math.pi / 2.0:
        raise ValueError("kappa_mag * segment_length_m must be below 90 deg")

    segments = (
        curvature.CurvatureSegment(
            direction_sign * kappa_mag, segment_length_m
        ),
        curvature.CurvatureSegment(
            -direction_sign * kappa_mag, segment_length_m
        ),
    )
    return {
        "source_stations": source_stations,
        "start": start,
        "start_yaw": start_yaw,
        "target": target,
        "target_station": target_station,
        "target_width_m": target_width,
        "direction": direction,
        "direction_sign": direction_sign,
        "kappa_mag": kappa_mag,
        "segments": segments,
    }


def generate_points(
    geometry,
    source_points,
    target_points,
    sample_interval_m,
    velocity_mps,
    start_lanelet_id,
    end_lanelet_id,
    route_order,
):
    approach = sample_polyline_prefix(
        source_points,
        geometry["source_stations"],
        geometry["source_stations"][-1],
        sample_interval_m,
        velocity_mps,
        start_lanelet_id,
        route_order,
    )
    lane_change = curvature.generate_from_curvature_segments(
        geometry["start"][0],
        geometry["start"][1],
        geometry["start"][2],
        geometry["start_yaw"],
        geometry["segments"],
        sample_interval_m,
        velocity_mps,
        end_lanelet_id,
        route_order,
    )
    target_stations = cumulative_lengths(target_points)
    target_suffix = sample_polyline_suffix(
        target_points,
        target_stations,
        geometry["target_station"],
        sample_interval_m,
        velocity_mps,
        end_lanelet_id,
        route_order,
    )
    points = approach + lane_change.points[1:] + target_suffix[1:]
    for index, point in enumerate(points):
        point["index"] = index
    return points, lane_change.segment_ranges, lane_change.points[-1]


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a constant-curvature high-speed lane-change trajectory"
        )
    )
    parser.add_argument(
        "--config", required=True, help="repository-relative generator YAML"
    )
    parser.add_argument(
        "--start-lanelet-id", type=int, default=DEFAULT_START_LANELET_ID
    )
    parser.add_argument(
        "--end-lanelet-id", type=int, default=DEFAULT_END_LANELET_ID
    )
    parser.add_argument(
        "--kappa-mag",
        type=float,
        required=True,
        help="fixed curvature in 1/m; L is solved from lane spacing",
    )
    parser.add_argument(
        "--speeds-kph",
        nargs="+",
        type=float,
        default=DEFAULT_SPEEDS_KPH,
    )
    parser.add_argument("--output-directory")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_args()
    root = base.repository_root()
    config = base.load_config(
        base.resolve_repo_path(root, arguments.config, "config")
    )
    map_config = base.require(config, "map", "config")
    osm_path = base.resolve_repo_path(
        root,
        base.require(map_config, "lanelet_map_file", "map"),
        "lanelet_map_file",
    )
    projector_path = base.resolve_repo_path(
        root,
        base.require(map_config, "projector_file", "map"),
        "projector_file",
    )
    speeds_kph = [
        base.positive_finite(speed, "speeds_kph") for speed in arguments.speeds_kph
    ]

    projector = base.parse_projector(projector_path)
    centerlines, unused_speed_limits = base.load_lanelet_centerlines(
        osm_path, projector
    )
    del unused_speed_limits
    missing = [
        lanelet_id
        for lanelet_id in (
            arguments.start_lanelet_id,
            arguments.end_lanelet_id,
        )
        if lanelet_id not in centerlines
    ]
    if missing:
        raise ValueError("missing lanelet centerlines: {}".format(missing))

    geometry = build_lane_change_geometry(
        centerlines[arguments.start_lanelet_id],
        centerlines[arguments.end_lanelet_id],
        arguments.kappa_mag,
    )
    approach_length_m = geometry["source_stations"][-1]
    segment_length_m = geometry["segments"][0].length_m
    designed_width = lane_change_width(
        geometry["kappa_mag"], segment_length_m
    )

    if arguments.output_directory:
        output_root = (root / arguments.output_directory).resolve()
        try:
            output_root.relative_to(root)
        except ValueError as error:
            raise ValueError(
                "output_directory must stay inside repository"
            ) from error
    else:
        output_root = (
            root
            / "base_ws/src/control/trajectory_generator/traj_sample"
        )

    for speed_kph in speeds_kph:
        points, segment_ranges, lane_change_end = generate_points(
            geometry,
            centerlines[arguments.start_lanelet_id],
            centerlines[arguments.end_lanelet_id],
            SAMPLE_INTERVAL_M,
            speed_kph / 3.6,
            arguments.start_lanelet_id,
            arguments.end_lanelet_id,
            0,
        )
        name = "{}_{}_k{}_lc".format(
            arguments.start_lanelet_id,
            arguments.end_lanelet_id,
            kappa_filename_token(geometry["kappa_mag"]),
        )
        directory = output_root / "{}kph".format(
            curvature.filename_number(speed_kph)
        )
        csv_path = directory / (name + ".csv")
        json_path = directory / (name + ".json")
        base.write_csv(csv_path, points, arguments.overwrite)

        end = points[-1]
        endpoint_gap = math.dist(
            (
                lane_change_end["x"],
                lane_change_end["y"],
                lane_change_end["z"],
            ),
            geometry["target"],
        )
        target_continuation_length_m = (
            cumulative_lengths(
                centerlines[arguments.end_lanelet_id]
            )[-1] - geometry["target_station"]
        )
        manifest = {
            "generator_name": "constant_curvature_lane_change",
            "generator_version": "1.0.0",
            "maneuver_type": "lane_change",
            "direction": geometry["direction"],
            "direction_sign": geometry["direction_sign"],
            "sign_sequence": [
                geometry["direction_sign"],
                -geometry["direction_sign"],
            ],
            "start_lanelet_id": arguments.start_lanelet_id,
            "end_lanelet_id": arguments.end_lanelet_id,
            "lanelet_map_file": str(osm_path.relative_to(root)),
            "approach_length_m": approach_length_m,
            "lane_change_start_reference": "start_lanelet_end",
            "design_kappa_mag": geometry["kappa_mag"],
            "parameter_mode": "fixed_kappa_solved_length",
            "segment_length_m": segment_length_m,
            "lane_change_length_m": 2.0 * segment_length_m,
            "target_lane_continuation_length_m":
            target_continuation_length_m,
            "total_length_m": approach_length_m
            + 2.0 * segment_length_m
            + target_continuation_length_m,
            "sample_interval_m": SAMPLE_INTERVAL_M,
            "speed_kph": speed_kph,
            "radius_m": 1.0 / geometry["kappa_mag"],
            "delta_theta_rad": geometry["kappa_mag"]
            * segment_length_m,
            "delta_theta_deg": math.degrees(
                geometry["kappa_mag"] * segment_length_m
            ),
            "target_lane_width_m": geometry["target_width_m"],
            "design_lateral_displacement_m": designed_width,
            "target_endpoint_gap_m": endpoint_gap,
            "start_pose": {
                "x": geometry["start"][0],
                "y": geometry["start"][1],
                "z": geometry["start"][2],
                "yaw": geometry["start_yaw"],
            },
            "lane_change_end_pose": {
                "x": lane_change_end["x"],
                "y": lane_change_end["y"],
                "z": lane_change_end["z"],
                "yaw": lane_change_end["yaw"],
            },
            "end_pose": {
                "x": end["x"],
                "y": end["y"],
                "z": end["z"],
                "yaw": end["yaw"],
            },
            "segment_ranges_without_approach": [
                list(value) for value in segment_ranges
            ],
        }
        curvature.write_json(json_path, manifest, arguments.overwrite)
        print(
            "{}: speed={:.0f}kph kappa={:.9f} 1/m "
            "L={:.3f}m width={:.3f}m endpoint_gap={:.3f}m points={}".format(
                csv_path,
                speed_kph,
                geometry["kappa_mag"],
                segment_length_m,
                designed_width,
                endpoint_gap,
                len(points),
            )
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("generate_constant_curvature_lc: {}".format(error))
