#!/usr/bin/env python3
"""Generate a constant-curvature sign-reversal lane-return trajectory."""

import argparse
import json
import math
import os
import tempfile
import warnings
from dataclasses import dataclass
from pathlib import Path

import generate_trajectory as base


GENERATOR_NAME = "constant_curvature_sign_reversal"
GENERATOR_VERSION = "2.0.0"
MANEUVER_TYPE = "avoidance"
SIGN_SEQUENCE = (1, -1, -1, 1)
CURVATURE_EPSILON = 1.0e-9
YAW_TOLERANCE_RAD = 1.0e-9
POSITION_TOLERANCE_M = 1.0e-8


@dataclass(frozen=True)
class CurvatureSegment:
    curvature_1pm: float
    length_m: float


@dataclass
class GeneratedPath:
    points: list
    metadata: dict
    segment_ranges: list


def finite(value, label):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite".format(label))
    return number


def nonnegative_finite(value, label):
    number = finite(value, label)
    if number < 0.0:
        raise ValueError("{} must be nonnegative".format(label))
    return number


def normalize_yaw(yaw):
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def filename_number(value):
    number = finite(value, "filename value")
    text = "{:.12g}".format(number)
    return "0" if text == "-0" else text


def filename_token(value):
    return (
        filename_number(value)
        .replace("-", "m")
        .replace("+", "")
        .replace(".", "p")
    )


def local_pose(start_x, start_y, start_yaw, x, y, yaw):
    dx = x - start_x
    dy = y - start_y
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        yaw - start_yaw,
    )


def sample_stations(length_m, interval_m):
    count = int(math.floor(length_m / interval_m))
    stations = [index * interval_m for index in range(count + 1)]
    if length_m - stations[-1] > 1.0e-12:
        stations.append(length_m)
    else:
        stations[-1] = length_m
    return stations


def sample_constant_curvature_segment(
    start_x_m,
    start_y_m,
    start_yaw_rad,
    curvature_1pm,
    segment_length_m,
    sample_interval_m,
    include_start,
):
    """Sample one constant-curvature arc using its closed-form solution."""
    segment_length_m = base.positive_finite(segment_length_m, "segment_length_m")
    sample_interval_m = base.positive_finite(sample_interval_m, "sample_interval_m")
    curvature_1pm = finite(curvature_1pm, "curvature_1pm")
    stations = sample_stations(segment_length_m, sample_interval_m)
    if not include_start:
        stations = stations[1:]

    points = []
    for station in stations:
        heading = start_yaw_rad + curvature_1pm * station
        if abs(curvature_1pm) < CURVATURE_EPSILON:
            x = start_x_m + station * math.cos(start_yaw_rad)
            y = start_y_m + station * math.sin(start_yaw_rad)
        else:
            x = start_x_m + (
                math.sin(heading) - math.sin(start_yaw_rad)
            ) / curvature_1pm
            y = start_y_m - (
                math.cos(heading) - math.cos(start_yaw_rad)
            ) / curvature_1pm
        points.append(
            {
                "x": x,
                "y": y,
                "yaw": heading,
                "s_segment_m": station,
            }
        )
    return points


def generate_from_curvature_segments(
    start_x_m,
    start_y_m,
    start_z_m,
    start_yaw_rad,
    segments,
    sample_interval_m,
    desired_velocity_mps,
    lanelet_id,
    route_order,
    normalize_output_yaw=False,
):
    if not segments:
        raise ValueError("segments must not be empty")
    start_x_m = finite(start_x_m, "start_x_m")
    start_y_m = finite(start_y_m, "start_y_m")
    start_z_m = finite(start_z_m, "start_z_m")
    start_yaw_rad = finite(start_yaw_rad, "start_yaw_rad")
    sample_interval_m = base.positive_finite(
        sample_interval_m, "sample_interval_m"
    )
    desired_velocity_mps = nonnegative_finite(
        desired_velocity_mps, "desired_velocity_mps"
    )

    points = []
    segment_ranges = []
    current_x = start_x_m
    current_y = start_y_m
    current_yaw = start_yaw_rad
    global_station = 0.0

    for segment_id, segment in enumerate(segments):
        curvature = finite(segment.curvature_1pm, "segment.curvature_1pm")
        length = base.positive_finite(segment.length_m, "segment.length_m")
        sampled = sample_constant_curvature_segment(
            current_x,
            current_y,
            current_yaw,
            curvature,
            length,
            sample_interval_m,
            include_start=segment_id == 0,
        )
        start_index = len(points)
        for point in sampled:
            point.update(
                {
                    "index": len(points),
                    "z": start_z_m,
                    "velocity_mps": desired_velocity_mps,
                    "acceleration_mps2": 0.0,
                    "lanelet_id": int(lanelet_id),
                    "route_order": int(route_order),
                    "reference_kappa": curvature,
                    "segment_id": segment_id,
                    "segment_sign": (
                        0
                        if abs(curvature) < CURVATURE_EPSILON
                        else (1 if curvature > 0.0 else -1)
                    ),
                    "s_global_m": global_station + point["s_segment_m"],
                }
            )
            if normalize_output_yaw:
                point["yaw"] = normalize_yaw(point["yaw"])
            points.append(point)
        segment_ranges.append((start_index, len(points) - 1))

        endpoint = sample_constant_curvature_segment(
            current_x,
            current_y,
            current_yaw,
            curvature,
            length,
            length,
            include_start=False,
        )[-1]
        current_x = endpoint["x"]
        current_y = endpoint["y"]
        current_yaw = endpoint["yaw"]
        global_station += length

    return GeneratedPath(points, {}, segment_ranges)


def validate_generated_path(result, expected_length_m, sample_interval_m):
    if len(result.points) < 2:
        raise ValueError("generated path has fewer than two points")
    if result.points[0]["index"] != 0:
        raise ValueError("generated path index does not start at zero")

    measured_length = 0.0
    for index, (first, second) in enumerate(
        zip(result.points, result.points[1:])
    ):
        if second["index"] != index + 1:
            raise ValueError("generated path indices are not contiguous")
        distance = math.hypot(
            second["x"] - first["x"], second["y"] - first["y"]
        )
        if distance <= 0.0:
            raise ValueError("generated path contains duplicate consecutive points")
        if (
            first["segment_id"] == second["segment_id"]
        ):
            arc_step = second["s_segment_m"] - first["s_segment_m"]
            curvature = abs(first["reference_kappa"])
            expected_chord = (
                arc_step
                if curvature < CURVATURE_EPSILON
                else 2.0
                * math.sin(0.5 * curvature * arc_step)
                / curvature
            )
            if not math.isclose(
                distance,
                expected_chord,
                rel_tol=0.0,
                abs_tol=1.0e-9,
            ):
                raise ValueError(
                    "unexpected waypoint spacing at index {}: "
                    "{:.9f} m (expected chord {:.9f} m)".format(
                        index, distance, expected_chord
                    )
                )
        measured_length += distance

    # Chord lengths are slightly shorter than their corresponding arc lengths.
    maximum_chord_error = (
        len(result.points)
        * sample_interval_m ** 3
        * max(
            abs(point["reference_kappa"]) for point in result.points
        )
        ** 2
        / 24.0
    )
    if abs(measured_length - expected_length_m) > max(
        maximum_chord_error * 1.01, 1.0e-8
    ):
        raise ValueError(
            "measured chord length {:.9f} differs from design arc length {:.9f}".format(
                measured_length, expected_length_m
            )
        )
    return measured_length


def generate_lane_return(
    start_x_m,
    start_y_m,
    start_z_m,
    start_yaw_rad,
    kappa_mag,
    segment_length_m,
    sample_interval_m=0.1,
    desired_velocity_mps=0.0,
    lanelet_id=-1,
    route_order=0,
    normalize_output_yaw=False,
    start_straight_length_m=0.0,
    middle_straight_length_m=0.0,
):
    """Generate straight + (+k,-k) + straight + (-k,+k) lane return."""
    kappa_mag = nonnegative_finite(kappa_mag, "kappa_mag")
    start_straight_length_m = nonnegative_finite(
        start_straight_length_m, "start_straight_length_m"
    )
    middle_straight_length_m = nonnegative_finite(
        middle_straight_length_m, "middle_straight_length_m"
    )
    segment_length_m = base.positive_finite(
        segment_length_m, "segment_length_m"
    )
    sample_interval_m = base.positive_finite(
        sample_interval_m, "sample_interval_m"
    )
    delta_theta = kappa_mag * segment_length_m
    if delta_theta >= math.pi / 2.0:
        raise ValueError(
            "delta_theta >= 90 deg: path is not monotonic along the initial "
            "forward axis and is invalid for this benchmark"
        )

    warning_messages = []
    if delta_theta >= math.radians(60.0):
        message = (
            "delta_theta >= 60 deg: forward-progress efficiency is degraded; "
            "consider a smaller kappa_mag or smaller segment_length_m"
        )
        warnings.warn(message)
        warning_messages.append(message)

    segment_specs = []
    if start_straight_length_m > 0.0:
        segment_specs.append(
            ("start_straight", CurvatureSegment(0.0, start_straight_length_m))
        )
    segment_specs.extend(
        (
            ("outbound_positive_arc", CurvatureSegment(kappa_mag, segment_length_m)),
            ("outbound_negative_arc", CurvatureSegment(-kappa_mag, segment_length_m)),
        )
    )
    if middle_straight_length_m > 0.0:
        segment_specs.append(
            ("middle_straight", CurvatureSegment(0.0, middle_straight_length_m))
        )
    segment_specs.extend(
        (
            ("return_negative_arc", CurvatureSegment(-kappa_mag, segment_length_m)),
            ("return_positive_arc", CurvatureSegment(kappa_mag, segment_length_m)),
        )
    )
    segment_roles = [role for role, _ in segment_specs]
    segments = [segment for _, segment in segment_specs]
    result = generate_from_curvature_segments(
        start_x_m,
        start_y_m,
        start_z_m,
        start_yaw_rad,
        segments,
        sample_interval_m,
        desired_velocity_mps,
        lanelet_id,
        route_order,
        normalize_output_yaw,
    )
    role_signs = {
        "start_straight": 0,
        "outbound_positive_arc": 1,
        "outbound_negative_arc": -1,
        "middle_straight": 0,
        "return_negative_arc": -1,
        "return_positive_arc": 1,
    }
    for point in result.points:
        segment_role = segment_roles[point["segment_id"]]
        point["segment_role"] = segment_role
        point["segment_sign"] = role_signs[segment_role]

    middle_straight_index = (
        segment_roles.index("middle_straight")
        if "middle_straight" in segment_roles
        else None
    )
    if middle_straight_index is not None:
        middle_start_index = result.segment_ranges[middle_straight_index][0]
        middle_start_yaw_error = normalize_yaw(
            result.points[middle_start_index]["yaw"] - start_yaw_rad
        )
        if abs(middle_start_yaw_error) > YAW_TOLERANCE_RAD:
            raise ValueError(
                "middle straight does not start at zero heading error: "
                "{:.3e} rad".format(middle_start_yaw_error)
            )

    end = result.points[-1]
    local_end = local_pose(
        start_x_m,
        start_y_m,
        start_yaw_rad,
        end["x"],
        end["y"],
        (
            start_yaw_rad
            + sum(segment.curvature_1pm * segment.length_m for segment in segments)
        ),
    )
    if abs(local_end[2]) > YAW_TOLERANCE_RAD:
        raise ValueError(
            "lane-return end yaw error exceeds tolerance: {:.3e} rad".format(
                local_end[2]
            )
        )
    if abs(local_end[1]) > POSITION_TOLERANCE_M:
        raise ValueError(
            "lane-return lateral return error exceeds tolerance: {:.3e} m".format(
                local_end[1]
            )
        )

    total_length = (
        4.0 * segment_length_m
        + start_straight_length_m
        + middle_straight_length_m
    )
    measured_chord_length = validate_generated_path(
        result, total_length, sample_interval_m
    )
    if kappa_mag < CURVATURE_EPSILON:
        radius_m = None
        x_displacement = total_length
        peak_offset = 0.0
    else:
        radius_m = 1.0 / kappa_mag
        x_displacement = (
            start_straight_length_m
            + middle_straight_length_m
            + 4.0 * math.sin(delta_theta) / kappa_mag
        )
        peak_offset = 2.0 * (1.0 - math.cos(delta_theta)) / kappa_mag

    result.metadata = {
        "generator_name": GENERATOR_NAME,
        "generator_version": GENERATOR_VERSION,
        "maneuver_type": MANEUVER_TYPE,
        "sign_sequence": list(SIGN_SEQUENCE),
        "design_kappa_mag": kappa_mag,
        "segment_length_m": segment_length_m,
        "start_straight_length_m": start_straight_length_m,
        "middle_straight_length_m": middle_straight_length_m,
        "sample_interval_m": sample_interval_m,
        "segment_count": len(segments),
        "segment_roles": segment_roles,
        "segment_ranges": [list(value) for value in result.segment_ranges],
        "total_length_m": total_length,
        "measured_chord_length_m": measured_chord_length,
        "delta_theta_rad": delta_theta,
        "delta_theta_deg": math.degrees(delta_theta),
        "radius_m": radius_m,
        "start_pose": {
            "x": start_x_m,
            "y": start_y_m,
            "z": start_z_m,
            "yaw": start_yaw_rad,
        },
        "end_pose": {
            "x": end["x"],
            "y": end["y"],
            "z": end["z"],
            "yaw": normalize_yaw(end["yaw"])
            if normalize_output_yaw
            else end["yaw"],
        },
        "design_x_displacement_m": x_displacement,
        "design_y_displacement_m": 0.0,
        "design_w_peak_m": peak_offset,
        "warnings": warning_messages,
    }
    return result


def write_json(path, data, overwrite):
    if path.exists() and not overwrite:
        raise FileExistsError(
            "output exists; pass --overwrite to replace it: {}".format(path)
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def start_lanelet_start_pose(chain, centerlines, global_path):
    first_row = chain[0]
    lanelet_id = first_row["lanelet_id"]
    if lanelet_id not in centerlines:
        raise ValueError("start lanelet {} has no usable centerline".format(lanelet_id))
    points = base.orient_centerline(
        centerlines[lanelet_id], first_row, None, global_path
    )
    if len(points) < 2:
        raise ValueError("start lanelet centerline has fewer than two points")
    start = points[0]
    following = next(
        (
            point
            for point in points[1:]
            if math.hypot(point[0] - start[0], point[1] - start[1]) > 1.0e-9
        ),
        None,
    )
    if following is None:
        raise ValueError("start lanelet centerline has no usable heading")
    return (
        start[0],
        start[1],
        start[2],
        math.atan2(following[1] - start[1], following[0] - start[0]),
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Generate a straight,+k,-k,straight,-k,+k lane-return trajectory"
        )
    )
    parser.add_argument(
        "--config", required=True, help="repository-relative generator YAML"
    )
    parser.add_argument("--start-lanelet-id", type=int)
    parser.add_argument("--end-lanelet-id", type=int)
    parser.add_argument("--start-route-order", type=int)
    parser.add_argument("--end-route-order", type=int)
    parser.add_argument("--kappa-mag", required=True, type=float, help="1/m")
    parser.add_argument("--segment-length-m", required=True, type=float)
    parser.add_argument(
        "--start-straight-length-m",
        type=float,
        default=0.0,
        help="straight approach length before the first curvature arc (m)",
    )
    parser.add_argument(
        "--middle-straight-length-m",
        type=float,
        default=0.0,
        help="straight length at the first zero-heading-error point (m)",
    )
    parser.add_argument("--sample-interval-m", type=float, default=0.1)
    parser.add_argument("--desired-velocity-kph", type=float)
    parser.add_argument("--normalize-output-yaw", action="store_true")
    parser.add_argument("--output-csv")
    parser.add_argument("--output-manifest")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    root = base.repository_root()
    config_path = base.resolve_repo_path(root, arguments.config, "config")
    config = base.load_config(config_path)
    map_config = base.require(config, "map", "config")
    reference_config = base.require(config, "reference", "config")
    anchors = base.require(config, "anchors", "config")
    trajectory_config = base.require(config, "trajectory", "config")

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
    start_id = (
        arguments.start_lanelet_id
        if arguments.start_lanelet_id is not None
        else int(base.require(anchors, "start_lanelet_id", "anchors"))
    )
    end_id = (
        arguments.end_lanelet_id
        if arguments.end_lanelet_id is not None
        else int(base.require(anchors, "end_lanelet_id", "anchors"))
    )
    desired_velocity_kph = (
        arguments.desired_velocity_kph
        if arguments.desired_velocity_kph is not None
        else float(
            base.require(
                trajectory_config, "desired_velocity_kph", "trajectory"
            )
        )
    )
    desired_velocity_kph = nonnegative_finite(
        desired_velocity_kph, "desired_velocity_kph"
    )

    route_rows = base.load_route(route_path)
    chain = base.find_unique_chain(
        route_rows,
        start_id,
        end_id,
        arguments.start_route_order
        if arguments.start_route_order is not None
        else anchors.get("start_route_order"),
        arguments.end_route_order
        if arguments.end_route_order is not None
        else anchors.get("end_route_order"),
    )
    projector = base.parse_projector(projector_path)
    centerlines, unused_speed_limits = base.load_lanelet_centerlines(
        osm_path, projector
    )
    del unused_speed_limits
    global_path = base.load_global_path(global_path_path)
    start_pose = start_lanelet_start_pose(chain, centerlines, global_path)

    result = generate_lane_return(
        start_x_m=start_pose[0],
        start_y_m=start_pose[1],
        start_z_m=start_pose[2],
        start_yaw_rad=start_pose[3],
        kappa_mag=arguments.kappa_mag,
        segment_length_m=arguments.segment_length_m,
        sample_interval_m=arguments.sample_interval_m,
        desired_velocity_mps=desired_velocity_kph / 3.6,
        lanelet_id=start_id,
        route_order=chain[0]["order"],
        normalize_output_yaw=arguments.normalize_output_yaw,
        start_straight_length_m=arguments.start_straight_length_m,
        middle_straight_length_m=arguments.middle_straight_length_m,
    )
    result.metadata.update(
        {
            "start_lanelet_id": start_id,
            "start_anchor": "start_lanelet_start",
            "end_lanelet_id": end_id,
            "start_route_order": chain[0]["order"],
            "end_route_order": chain[-1]["order"],
            "selected_route_lanelet_count": len(chain),
            "lanelet_map_file": str(osm_path.relative_to(root)),
            "route_lanelets_file": str(route_path.relative_to(root)),
            "desired_velocity_kph": desired_velocity_kph,
            "end_lanelet_attachment": False,
        }
    )

    if arguments.output_csv:
        output_csv = base.resolve_repo_output_path(
            root, arguments.output_csv, "output_csv"
        )
    else:
        speed_directory = "{}kph".format(
            filename_number(desired_velocity_kph)
        )
        output_name = "{}_{}_k{}_l{}_s{}_m{}.csv".format(
            start_id,
            end_id,
            filename_token(arguments.kappa_mag),
            filename_token(arguments.segment_length_m),
            filename_token(arguments.start_straight_length_m),
            filename_token(arguments.middle_straight_length_m),
        )
        output_csv = (
            root
            / "base_ws/src/control/trajectory_generator/traj_sample"
            / speed_directory
            / output_name
        )
    if arguments.output_manifest:
        output_manifest = (
            root / arguments.output_manifest
        ).resolve()
        try:
            output_manifest.relative_to(root)
        except ValueError as error:
            raise ValueError(
                "output_manifest must stay inside repository: {}".format(
                    arguments.output_manifest
                )
            ) from error
        if output_manifest.suffix.lower() != ".json":
            raise ValueError("output_manifest must use a .json filename")
    else:
        output_manifest = output_csv.with_suffix(".json")

    if output_csv.exists() and not arguments.overwrite:
        raise FileExistsError(
            "output exists; pass --overwrite to replace it: {}".format(
                output_csv
            )
        )
    if output_manifest.exists() and not arguments.overwrite:
        raise FileExistsError(
            "output exists; pass --overwrite to replace it: {}".format(
                output_manifest
            )
        )
    base.write_csv(output_csv, result.points, arguments.overwrite)
    write_json(output_manifest, result.metadata, arguments.overwrite)

    print("lane-return trajectory generated: {}".format(output_csv))
    print("manifest generated: {}".format(output_manifest))
    print(
        "start_lanelet={} end_lanelet={} kappa_1pm={:.6f} "
        "segment_length_m={:.3f} start_straight_m={:.3f} "
        "middle_straight_m={:.3f} total_length_m={:.3f} points={}".format(
            start_id,
            end_id,
            arguments.kappa_mag,
            arguments.segment_length_m,
            arguments.start_straight_length_m,
            arguments.middle_straight_length_m,
            result.metadata["total_length_m"],
            len(result.points),
        )
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit(
            "generate_constant_curvature_return: {}".format(error)
        )
