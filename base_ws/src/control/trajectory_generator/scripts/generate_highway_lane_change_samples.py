#!/usr/bin/env python3
"""Generate the 200101 high-speed one- and two-lane-change samples."""

import argparse
import bisect
import math
from pathlib import Path

import generate_trajectory as base
import numpy as np


MAP_PATH = "kcity_map/lanelet_map/lanelet2_map_v63.osm"
PROJECTOR_PATH = "kcity_map/lanelet_map/map_projector_info_v1.yaml"
SAMPLE_ROOT = "base_ws/src/control/trajectory_generator/traj_sample"

VARIANTS = {
    "lc1": {
        "start_lanelet_id": 200101,
        "end_lanelet_id": 200130,
        "lane_sequences": (
            (200101, 201438, 200128, 200102, 201442, 200134, 201446),
            (201434, 200099, 200137, 200187, 201443, 200130, 201441),
        ),
        "route_orders": (26, 26, 27, 28, 28, 29, 29),
        "transitions": ((175.0, 275.0, 1.0),),
    },
    "lc2": {
        "start_lanelet_id": 200102,
        "end_lanelet_id": 201401,
        "label_source_lane_only": True,
        "lane_sequences": (
            (
                200102,
                201442,
                200134,
                201446,
                200360,
                201454,
                201447,
                201399,
                201451,
                200088,
                201401,
            ),
        ),
        "route_orders": (28, 28, 29, 29, 30, 30, 30, 31, 31, 32, 33),
        "transitions": (),
    },
}

DENSE_INTERVAL_M = 0.05
OUTPUT_INTERVAL_M = 0.1
JUNCTION_SMOOTHING_RADIUS_M = 15.0
PATH_SMOOTHING_SIGMA_M = 2.0
PATH_SMOOTHING_RADIUS_M = 6.0
DEFAULT_SPEEDS_KPH = tuple(range(60, 171, 10))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate fixed-geometry K-City high-speed lane-change samples"
    )
    parser.add_argument(
        "--speeds-kph",
        nargs="+",
        type=float,
        default=DEFAULT_SPEEDS_KPH,
        help="trajectory speeds in kph (default: 60 through 170 by 10)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace existing matching sample CSVs",
    )
    arguments = parser.parse_args()
    for speed in arguments.speeds_kph:
        if not math.isfinite(speed) or speed <= 0.0:
            parser.error("all --speeds-kph values must be finite and positive")
        if not float(speed).is_integer():
            parser.error("sample filenames require integer --speeds-kph values")
    return arguments


def cumulative_lengths(points):
    stations = [0.0]
    for first, second in zip(points, points[1:]):
        distance = math.dist(first, second)
        if distance <= 1e-9:
            raise ValueError("centerline contains duplicate consecutive points")
        stations.append(stations[-1] + distance)
    return stations


def interpolate(points, stations, station):
    station = min(max(station, 0.0), stations[-1])
    right = bisect.bisect_right(stations, station)
    segment = min(max(right - 1, 0), len(points) - 2)
    start_station = stations[segment]
    end_station = stations[segment + 1]
    ratio = (station - start_station) / (end_station - start_station)
    first = points[segment]
    second = points[segment + 1]
    return tuple(
        first[index] + ratio * (second[index] - first[index])
        for index in range(3)
    )


def smoothstep5(value):
    value = min(max(value, 0.0), 1.0)
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def transition_progress(station, start, end):
    if station <= start:
        return 0.0
    if station >= end:
        return 1.0
    return smoothstep5((station - start) / (end - start))


def lateral_lane_position(station, transitions):
    return sum(
        direction * transition_progress(station, start, end)
        for start, end, direction in transitions
    )


def blend(first, second, ratio):
    return tuple(
        first[index] + ratio * (second[index] - first[index])
        for index in range(3)
    )


def cubic_hermite(first, second, first_tangent, second_tangent, ratio):
    ratio2 = ratio * ratio
    ratio3 = ratio2 * ratio
    h00 = 2.0 * ratio3 - 3.0 * ratio2 + 1.0
    h10 = ratio3 - 2.0 * ratio2 + ratio
    h01 = -2.0 * ratio3 + 3.0 * ratio2
    h11 = ratio3 - ratio2
    return tuple(
        h00 * first[index]
        + h10 * first_tangent[index]
        + h01 * second[index]
        + h11 * second_tangent[index]
        for index in range(3)
    )


def smooth_junctions(geometry, dense_stations, section_starts):
    smoothed = list(geometry)
    for junction in section_starts[1:-1]:
        start_station = junction - JUNCTION_SMOOTHING_RADIUS_M
        end_station = junction + JUNCTION_SMOOTHING_RADIUS_M
        start_index = bisect.bisect_left(dense_stations, start_station)
        end_index = bisect.bisect_right(dense_stations, end_station) - 1
        if start_index < 1 or end_index >= len(geometry) - 1:
            raise ValueError("junction smoothing window exceeds trajectory bounds")

        first = geometry[start_index][:3]
        second = geometry[end_index][:3]
        duration = dense_stations[end_index] - dense_stations[start_index]
        first_step = dense_stations[start_index] - dense_stations[start_index - 1]
        second_step = dense_stations[end_index + 1] - dense_stations[end_index]
        first_tangent = tuple(
            (geometry[start_index][axis] - geometry[start_index - 1][axis])
            / first_step
            * duration
            for axis in range(3)
        )
        second_tangent = tuple(
            (geometry[end_index + 1][axis] - geometry[end_index][axis])
            / second_step
            * duration
            for axis in range(3)
        )

        for index in range(start_index, end_index + 1):
            ratio = (
                dense_stations[index] - dense_stations[start_index]
            ) / duration
            point = cubic_hermite(
                first,
                second,
                first_tangent,
                second_tangent,
                ratio,
            )
            smoothed[index] = (
                point[0],
                point[1],
                point[2],
                geometry[index][3],
                geometry[index][4],
            )
    return smoothed


def smooth_path(geometry, dense_stations):
    coordinates = np.asarray([point[:3] for point in geometry], dtype=float)
    sigma_samples = PATH_SMOOTHING_SIGMA_M / DENSE_INTERVAL_M
    radius_samples = int(
        math.ceil(PATH_SMOOTHING_RADIUS_M / DENSE_INTERVAL_M)
    )
    offsets = np.arange(-radius_samples, radius_samples + 1, dtype=float)
    kernel = np.exp(-0.5 * (offsets / sigma_samples) ** 2)
    kernel /= np.sum(kernel)

    start_step = coordinates[1] - coordinates[0]
    end_step = coordinates[-1] - coordinates[-2]
    start_padding = np.asarray(
        [
            coordinates[0] - start_step * index
            for index in range(radius_samples, 0, -1)
        ]
    )
    end_padding = np.asarray(
        [
            coordinates[-1] + end_step * index
            for index in range(1, radius_samples + 1)
        ]
    )
    padded = np.vstack((start_padding, coordinates, end_padding))
    filtered = np.column_stack(
        [
            np.convolve(padded[:, axis], kernel, mode="same")[
                radius_samples : radius_samples + len(coordinates)
            ]
            for axis in range(3)
        ]
    )

    # Preserve the exact first 0.5 m so every speed and both variants share the
    # existing 200101 spawn pose and tangent. A Hermite correction then decays
    # into the globally smoothed geometry without reintroducing a kink.
    start_anchor_index = bisect.bisect_left(dense_stations, 0.5)
    start_blend_length = 15.0
    start_anchor_station = dense_stations[start_anchor_index]
    correction_start = (
        coordinates[start_anchor_index] - filtered[start_anchor_index]
    )
    raw_tangent = (
        coordinates[start_anchor_index] - coordinates[start_anchor_index - 1]
    ) / (
        dense_stations[start_anchor_index]
        - dense_stations[start_anchor_index - 1]
    )
    filtered_tangent = (
        filtered[start_anchor_index + 1] - filtered[start_anchor_index - 1]
    ) / (
        dense_stations[start_anchor_index + 1]
        - dense_stations[start_anchor_index - 1]
    )
    correction_tangent = (
        raw_tangent - filtered_tangent
    ) * start_blend_length
    zero = (0.0, 0.0, 0.0)
    for index, station in enumerate(dense_stations):
        if index <= start_anchor_index:
            filtered[index] = coordinates[index]
        elif station < start_anchor_station + start_blend_length:
            ratio = (
                station - start_anchor_station
            ) / start_blend_length
            correction = cubic_hermite(
                correction_start,
                zero,
                correction_tangent,
                zero,
                ratio,
            )
            filtered[index] += correction

    # Return smoothly to the exact requested lanelet endpoint.
    total_length = dense_stations[-1]
    endpoint_correction = coordinates[-1] - filtered[-1]
    for index, station in enumerate(dense_stations):
        if station >= total_length - 20.0:
            end_weight = smoothstep5(
                (station - (total_length - 20.0)) / 20.0
            )
            filtered[index] += end_weight * endpoint_correction

    filtered[-1] = coordinates[-1]
    return [
        (
            filtered[index, 0],
            filtered[index, 1],
            filtered[index, 2],
            geometry[index][3],
            geometry[index][4],
        )
        for index in range(len(geometry))
    ]


def build_geometry(centerlines, variant):
    lane_sequences = variant["lane_sequences"]
    route_orders = variant["route_orders"]
    lane_data = []
    for sequence in lane_sequences:
        sequence_data = []
        for lanelet_id in sequence:
            points = centerlines[lanelet_id]
            stations = cumulative_lengths(points)
            sequence_data.append((points, stations))
        lane_data.append(sequence_data)

    source_lengths = [
        lane_data[0][index][1][-1]
        for index in range(len(lane_sequences[0]))
    ]
    section_starts = [0.0]
    for length in source_lengths:
        section_starts.append(section_starts[-1] + length)
    total_source_length = section_starts[-1]

    dense_stations = [
        index * DENSE_INTERVAL_M
        for index in range(int(total_source_length // DENSE_INTERVAL_M) + 1)
    ]
    if total_source_length - dense_stations[-1] > 1e-9:
        dense_stations.append(total_source_length)
    else:
        dense_stations[-1] = total_source_length

    geometry = []
    for station in dense_stations:
        section = min(
            bisect.bisect_right(section_starts, station) - 1,
            len(source_lengths) - 1,
        )
        section_station = station - section_starts[section]
        section_ratio = section_station / source_lengths[section]
        lane_points = []
        for lane_index in range(len(lane_sequences)):
            points, stations = lane_data[lane_index][section]
            lane_points.append(
                interpolate(points, stations, section_ratio * stations[-1])
            )

        lane_position = lateral_lane_position(station, variant["transitions"])
        lane_position = min(max(lane_position, 0.0), len(lane_sequences) - 1)
        lower_lane = int(math.floor(lane_position))
        upper_lane = min(lower_lane + 1, len(lane_sequences) - 1)
        point = blend(
            lane_points[lower_lane],
            lane_points[upper_lane],
            lane_position - lower_lane,
        )

        lane_index = (
            0
            if variant.get("label_source_lane_only", False)
            else min(
                int(math.floor(lane_position + 0.5)),
                len(lane_sequences) - 1,
            )
        )
        geometry.append(
            (
                point[0],
                point[1],
                point[2],
                lane_sequences[lane_index][section],
                route_orders[section],
            )
        )
    junction_smoothed = smooth_junctions(
        geometry, dense_stations, section_starts
    )
    return smooth_path(junction_smoothed, dense_stations)


def trajectory_points(geometry, speed_kph):
    speed_mps = speed_kph / 3.6
    vertices = [
        (x, y, z, lanelet_id, route_order, speed_mps)
        for x, y, z, lanelet_id, route_order in geometry
    ]
    points, length_m = base.resample(vertices, OUTPUT_INTERVAL_M)
    return points, length_m


def output_path(root, speed_kph, start_lanelet_id, end_lanelet_id):
    speed_label = str(int(speed_kph))
    return (
        root
        / SAMPLE_ROOT
        / "{}kph".format(speed_label)
        / "{}_{}_{}.csv".format(
            start_lanelet_id,
            end_lanelet_id,
            speed_label,
        )
    )


def main():
    arguments = parse_args()
    root = base.repository_root()
    projector = base.parse_projector(root / PROJECTOR_PATH)
    centerlines, _ = base.load_lanelet_centerlines(root / MAP_PATH, projector)

    required_lanelets = {
        lanelet_id
        for variant in VARIANTS.values()
        for sequence in variant["lane_sequences"]
        for lanelet_id in sequence
    }
    missing = sorted(required_lanelets.difference(centerlines))
    if missing:
        raise ValueError("missing lanelet centerlines: {}".format(missing))

    for variant_name, variant in VARIANTS.items():
        geometry = build_geometry(centerlines, variant)
        for speed_kph in arguments.speeds_kph:
            points, length_m = trajectory_points(geometry, speed_kph)
            path = output_path(
                root,
                speed_kph,
                variant["start_lanelet_id"],
                variant["end_lanelet_id"],
            )
            base.write_csv(path, points, arguments.overwrite)
            print(
                "{}: {} speed_kph={:.0f} length_m={:.3f} points={}".format(
                    variant_name,
                    path,
                    speed_kph,
                    length_m,
                    len(points),
                )
            )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("generate_highway_lane_change_samples: {}".format(error))
