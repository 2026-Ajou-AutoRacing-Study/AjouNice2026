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
DEFAULT_SPEEDS_KPH = tuple(range(60, 171, 10))
SAMPLE_INTERVAL_M = 0.1
DEFAULT_LANE_WIDTH_M = 3.5


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


def build_repeated_lane_change_segments(
    kappa_mag,
    lane_width_m,
    direction,
    lane_change_count,
    initial_straight_length_m,
    between_lane_change_length_m,
    final_straight_length_m,
):
    kappa_mag = base.positive_finite(kappa_mag, "kappa_mag")
    lane_width_m = base.positive_finite(lane_width_m, "lane_width_m")
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    if lane_change_count < 1:
        raise ValueError("lane_change_count must be at least one")
    initial = curvature.nonnegative_finite(
        initial_straight_length_m, "initial_straight_length_m"
    )
    between = curvature.nonnegative_finite(
        between_lane_change_length_m, "between_lane_change_length_m"
    )
    final = curvature.nonnegative_finite(
        final_straight_length_m, "final_straight_length_m"
    )
    direction_sign = 1 if direction == "left" else -1
    arc_length_m = solve_length_for_width(lane_width_m, kappa_mag)
    specs = []
    if initial > 0.0:
        specs.append(
            ("initial_straight", curvature.CurvatureSegment(0.0, initial))
        )
    for index in range(lane_change_count):
        number = index + 1
        specs.extend(
            (
                (
                    "lane_change_{}_entry_arc".format(number),
                    curvature.CurvatureSegment(
                        direction_sign * kappa_mag, arc_length_m
                    ),
                ),
                (
                    "lane_change_{}_exit_arc".format(number),
                    curvature.CurvatureSegment(
                        -direction_sign * kappa_mag, arc_length_m
                    ),
                ),
            )
        )
        if index < lane_change_count - 1 and between > 0.0:
            specs.append(
                (
                    "between_lane_change_{}_straight".format(number),
                    curvature.CurvatureSegment(0.0, between),
                )
            )
    if final > 0.0:
        specs.append(
            ("final_straight", curvature.CurvatureSegment(0.0, final))
        )
    return {
        "segments": [segment for _, segment in specs],
        "segment_roles": [role for role, _ in specs],
        "direction": direction,
        "direction_sign": direction_sign,
        "kappa_mag": kappa_mag,
        "lane_width_m": lane_width_m,
        "lane_change_count": lane_change_count,
        "arc_length_m": arc_length_m,
        "initial_straight_length_m": initial,
        "between_lane_change_length_m": between,
        "final_straight_length_m": final,
    }


def resolve_final_straight_length(
    total_length_m,
    kappa_mag,
    lane_width_m,
    lane_change_count,
    initial_straight_length_m,
    between_lane_change_length_m,
):
    total = base.positive_finite(total_length_m, "total_length_m")
    initial = curvature.nonnegative_finite(
        initial_straight_length_m, "initial_straight_length_m"
    )
    between = curvature.nonnegative_finite(
        between_lane_change_length_m, "between_lane_change_length_m"
    )
    if lane_change_count < 1:
        raise ValueError("lane_change_count must be at least one")
    arc_length = solve_length_for_width(lane_width_m, kappa_mag)
    minimum_length = (
        initial
        + 2.0 * lane_change_count * arc_length
        + max(lane_change_count - 1, 0) * between
    )
    remaining = total - minimum_length
    if remaining < -1.0e-9:
        raise ValueError(
            "total_length_m {:.3f} m is too short; fixed lane-change "
            "sections require at least {:.3f} m".format(total, minimum_length)
        )
    return max(remaining, 0.0), minimum_length


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate repeated synthetic lane changes from a lanelet start pose"
    )
    parser.add_argument(
        "--config", required=True, help="repository-relative generator YAML"
    )
    parser.add_argument(
        "--start-lanelet-id", type=int, default=DEFAULT_START_LANELET_ID
    )
    parser.add_argument("--lane-width-m", type=float, default=DEFAULT_LANE_WIDTH_M)
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--lane-change-count", type=int, default=1)
    parser.add_argument("--initial-straight-length-m", type=float, default=0.0)
    parser.add_argument(
        "--between-lane-change-length-m", type=float, default=0.0
    )
    parser.add_argument("--final-straight-length-m", type=float, default=0.0)
    parser.add_argument(
        "--total-length-m",
        type=float,
        help="exact synthetic path length; remaining distance becomes final straight",
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
    reference_config = base.require(config, "reference", "config")
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
    route_path = base.resolve_repo_path(
        root,
        base.require(map_config, "route_lanelets_file", "map"),
        "route_lanelets_file",
    )
    global_path_path = base.resolve_repo_path(
        root,
        base.require(reference_config, "global_path_file", "reference"),
        "global_path_file",
    )
    speeds_kph = [
        base.positive_finite(speed, "speeds_kph") for speed in arguments.speeds_kph
    ]

    projector = base.parse_projector(projector_path)
    centerlines, unused_speed_limits = base.load_lanelet_centerlines(
        osm_path, projector
    )
    del unused_speed_limits
    if arguments.start_lanelet_id not in centerlines:
        raise ValueError(
            "missing lanelet centerline: {}".format(arguments.start_lanelet_id)
        )
    route_rows = base.load_route(route_path)
    matching_rows = [
        row
        for row in route_rows
        if row["lanelet_id"] == arguments.start_lanelet_id
    ]
    if len(matching_rows) != 1:
        raise ValueError(
            "start lanelet {} must appear exactly once in preferred route; "
            "found {}".format(arguments.start_lanelet_id, len(matching_rows))
        )
    start_row = matching_rows[0]
    global_path = base.load_global_path(global_path_path)
    start_pose = curvature.start_lanelet_start_pose(
        [start_row], centerlines, global_path
    )
    if (
        arguments.total_length_m is not None
        and arguments.final_straight_length_m != 0.0
    ):
        raise ValueError(
            "--total-length-m and --final-straight-length-m cannot be used together"
        )
    minimum_length_m = None
    final_straight_length_m = arguments.final_straight_length_m
    if arguments.total_length_m is not None:
        final_straight_length_m, minimum_length_m = (
            resolve_final_straight_length(
                arguments.total_length_m,
                arguments.kappa_mag,
                arguments.lane_width_m,
                arguments.lane_change_count,
                arguments.initial_straight_length_m,
                arguments.between_lane_change_length_m,
            )
        )
    geometry = build_repeated_lane_change_segments(
        arguments.kappa_mag,
        arguments.lane_width_m,
        arguments.direction,
        arguments.lane_change_count,
        arguments.initial_straight_length_m,
        arguments.between_lane_change_length_m,
        final_straight_length_m,
    )
    segment_length_m = geometry["arc_length_m"]
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
        generated = curvature.generate_from_curvature_segments(
            start_pose[0],
            start_pose[1],
            start_pose[2],
            start_pose[3],
            geometry["segments"],
            SAMPLE_INTERVAL_M,
            speed_kph / 3.6,
            arguments.start_lanelet_id,
            start_row["order"],
        )
        for point in generated.points:
            point["segment_role"] = geometry["segment_roles"][
                point["segment_id"]
            ]
        points = generated.points
        length_suffix = (
            "t{}".format(curvature.filename_token(arguments.total_length_m))
            if arguments.total_length_m is not None
            else "f{}".format(
                curvature.filename_token(
                    geometry["final_straight_length_m"]
                )
            )
        )
        name = "{}_k{}_w{}_n{}_i{}_m{}_{}_lc".format(
            arguments.start_lanelet_id,
            kappa_filename_token(geometry["kappa_mag"]),
            curvature.filename_token(geometry["lane_width_m"]),
            geometry["lane_change_count"],
            curvature.filename_token(
                geometry["initial_straight_length_m"]
            ),
            curvature.filename_token(
                geometry["between_lane_change_length_m"]
            ),
            length_suffix,
        )
        directory = output_root / "{}kph".format(
            curvature.filename_number(speed_kph)
        )
        csv_path = directory / (name + ".csv")
        json_path = directory / (name + ".json")
        base.write_csv(csv_path, points, arguments.overwrite)

        end = points[-1]
        total_length_m = (
            geometry["initial_straight_length_m"]
            + geometry["final_straight_length_m"]
            + geometry["lane_change_count"] * 2.0 * segment_length_m
            + max(geometry["lane_change_count"] - 1, 0)
            * geometry["between_lane_change_length_m"]
        )
        if arguments.total_length_m is not None and not math.isclose(
            total_length_m,
            arguments.total_length_m,
            rel_tol=0.0,
            abs_tol=1.0e-8,
        ):
            raise ValueError(
                "generated design length does not match total_length_m"
            )
        curvature.validate_generated_path(
            generated, total_length_m, SAMPLE_INTERVAL_M
        )
        manifest = {
            "generator_name": "constant_curvature_lane_change",
            "generator_version": "2.0.0",
            "maneuver_type": "lane_change",
            "direction": geometry["direction"],
            "direction_sign": geometry["direction_sign"],
            "lane_change_count": geometry["lane_change_count"],
            "start_lanelet_id": arguments.start_lanelet_id,
            "lanelet_map_file": str(osm_path.relative_to(root)),
            "route_lanelets_file": str(route_path.relative_to(root)),
            "lane_change_start_reference": "start_lanelet_start",
            "design_kappa_mag": geometry["kappa_mag"],
            "parameter_mode": "fixed_kappa_solved_length",
            "segment_length_m": segment_length_m,
            "lane_change_length_m_each": 2.0 * segment_length_m,
            "initial_straight_length_m":
            geometry["initial_straight_length_m"],
            "between_lane_change_length_m":
            geometry["between_lane_change_length_m"],
            "final_straight_length_m":
            geometry["final_straight_length_m"],
            "total_length_m": total_length_m,
            "requested_total_length_m": arguments.total_length_m,
            "minimum_maneuver_length_m": minimum_length_m,
            "sample_interval_m": SAMPLE_INTERVAL_M,
            "speed_kph": speed_kph,
            "radius_m": 1.0 / geometry["kappa_mag"],
            "delta_theta_rad": geometry["kappa_mag"]
            * segment_length_m,
            "delta_theta_deg": math.degrees(
                geometry["kappa_mag"] * segment_length_m
            ),
            "target_lane_width_m": geometry["lane_width_m"],
            "design_lateral_displacement_m_each": designed_width,
            "design_lateral_displacement_m_total":
            geometry["direction_sign"]
            * geometry["lane_change_count"]
            * designed_width,
            "start_pose": {
                "x": start_pose[0],
                "y": start_pose[1],
                "z": start_pose[2],
                "yaw": start_pose[3],
            },
            "end_pose": {
                "x": end["x"],
                "y": end["y"],
                "z": end["z"],
                "yaw": end["yaw"],
            },
            "segment_roles": geometry["segment_roles"],
            "segment_ranges": [
                list(value) for value in generated.segment_ranges
            ],
        }
        curvature.write_json(json_path, manifest, arguments.overwrite)
        print(
            "{}: speed={:.0f}kph kappa={:.9f} 1/m "
            "L={:.3f}m width={:.3f}m count={} points={}".format(
                csv_path,
                speed_kph,
                geometry["kappa_mag"],
                segment_length_m,
                designed_width,
                geometry["lane_change_count"],
                len(points),
            )
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("generate_constant_curvature_lc: {}".format(error))
