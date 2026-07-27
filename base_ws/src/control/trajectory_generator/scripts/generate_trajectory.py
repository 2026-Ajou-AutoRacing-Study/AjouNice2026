#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from pyproj import Transformer


CSV_FIELDS = [
    "index",
    "x",
    "y",
    "z",
    "yaw",
    "velocity_mps",
    "acceleration_mps2",
    "lanelet_id",
    "route_order",
]


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "base_ws" / "src").is_dir() and (parent / "kcity_map").is_dir():
            return parent
    raise RuntimeError("cannot locate AjouNice2026 repository root")


def resolve_repo_path(root, value, label):
    path = (root / str(value)).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("{} must stay inside repository: {}".format(label, value)) from error
    if not path.is_file():
        raise FileNotFoundError("{} does not exist: {}".format(label, path))
    return path


def resolve_repo_output_path(root, value, label):
    path = (root / str(value)).resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("{} must stay inside repository: {}".format(label, value)) from error
    if path.suffix.lower() != ".csv":
        raise ValueError("{} must use a .csv filename: {}".format(label, value))
    return path


def require(mapping, key, label):
    if not isinstance(mapping, dict) or key not in mapping:
        raise ValueError("missing config value {}.{}".format(label, key))
    return mapping[key]


def positive_finite(value, label):
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("{} must be finite and positive".format(label))
    return number


def load_config(path):
    with path.open() as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError("generator config must be a YAML mapping")
    return config


def load_speed_profile(trajectory_config):
    raw = trajectory_config.get("speed_profile")
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ValueError("trajectory.speed_profile must be a mapping")

    default_velocity_kph = positive_finite(
        require(raw, "default_velocity_kph", "trajectory.speed_profile"),
        "trajectory.speed_profile.default_velocity_kph",
    )
    raw_overrides = raw.get("lanelet_velocity_kph", {})
    if not isinstance(raw_overrides, dict):
        raise ValueError(
            "trajectory.speed_profile.lanelet_velocity_kph must be a mapping"
        )
    lanelet_velocity_kph = {}
    for raw_lanelet_id, raw_velocity in raw_overrides.items():
        try:
            lanelet_id = int(raw_lanelet_id)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "speed-profile lanelet ID must be an integer: {}".format(
                    raw_lanelet_id
                )
            ) from error
        if lanelet_id in lanelet_velocity_kph:
            raise ValueError(
                "duplicate speed-profile lanelet ID: {}".format(lanelet_id)
            )
        lanelet_velocity_kph[lanelet_id] = positive_finite(
            raw_velocity,
            "trajectory.speed_profile.lanelet_velocity_kph.{}".format(
                lanelet_id
            ),
        )

    respect_speed_limits = raw.get("respect_speed_limits", True)
    if not isinstance(respect_speed_limits, bool):
        raise ValueError(
            "trajectory.speed_profile.respect_speed_limits must be true or false"
        )

    return {
        "default_velocity_kph": default_velocity_kph,
        "lanelet_velocity_kph": lanelet_velocity_kph,
        "respect_speed_limits": respect_speed_limits,
        "max_acceleration_mps2": positive_finite(
            require(
                raw,
                "max_acceleration_mps2",
                "trajectory.speed_profile",
            ),
            "trajectory.speed_profile.max_acceleration_mps2",
        ),
        "max_deceleration_mps2": positive_finite(
            require(
                raw,
                "max_deceleration_mps2",
                "trajectory.speed_profile",
            ),
            "trajectory.speed_profile.max_deceleration_mps2",
        ),
    }


def load_route(path):
    rows = []
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "order",
            "lanelet_id",
            "successor_lanelet_id",
            "speed_limit_kph",
            "start_x",
            "start_y",
            "end_x",
            "end_y",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError("route CSV missing columns: {}".format(sorted(missing)))
        for raw in reader:
            if raw.get("role") != "preferred_course":
                continue
            row = dict(raw)
            try:
                row["order"] = int(raw["order"])
            except ValueError as error:
                raise ValueError(
                    "preferred_course row has non-integer order: {}".format(raw["order"])
                ) from error
            row["lanelet_id"] = int(raw["lanelet_id"])
            successor = raw["successor_lanelet_id"].strip()
            row["successor_lanelet_id"] = int(successor) if successor else None
            rows.append(row)
    if not rows:
        raise ValueError("route CSV is empty")
    return rows


def find_unique_chain(rows, start_id, end_id, start_order=None, end_order=None):
    by_lanelet = {}
    for row in rows:
        by_lanelet.setdefault(row["lanelet_id"], []).append(row)
    for candidates in by_lanelet.values():
        candidates.sort(key=lambda item: item["order"])

    chains = []

    def visit(row, chain, visited_orders):
        if row["order"] in visited_orders:
            return
        next_chain = chain + [row]
        if row["lanelet_id"] == end_id and (
            end_order is None or row["order"] == end_order
        ):
            chains.append(next_chain)
            return
        successor = row["successor_lanelet_id"]
        if successor is None:
            return
        next_candidates = [
            candidate
            for candidate in by_lanelet.get(successor, [])
            if candidate["order"] > row["order"]
        ]
        for candidate in next_candidates:
            visit(candidate, next_chain, visited_orders | {row["order"]})

    for start in by_lanelet.get(start_id, []):
        if start_order is not None and start["order"] != start_order:
            continue
        visit(start, [], set())

    unique = {}
    for chain in chains:
        unique[tuple(row["order"] for row in chain)] = chain
    if not unique:
        raise ValueError(
            "no successor chain reaches lanelet {} from {}".format(end_id, start_id)
        )
    if len(unique) != 1:
        candidates = [list(key) for key in sorted(unique)]
        raise ValueError(
            "start/end anchors are ambiguous; reachable route orders={}".format(candidates)
        )
    return next(iter(unique.values()))


def parse_projector(path):
    with path.open() as stream:
        data = yaml.safe_load(stream)
    if data.get("projector_type") != "LocalCartesianUTM":
        raise ValueError("only LocalCartesianUTM projector is supported")
    origin = require(data, "map_origin", "projector")
    latitude = float(require(origin, "latitude", "projector.map_origin"))
    longitude = float(require(origin, "longitude", "projector.map_origin"))
    altitude = float(origin.get("altitude", 0.0))
    zone = int((longitude + 180.0) // 6.0) + 1
    epsg = 32600 + zone if latitude >= 0.0 else 32700 + zone
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:{}".format(epsg), always_xy=True)
    origin_easting, origin_northing = transformer.transform(longitude, latitude)
    return transformer, origin_easting, origin_northing, altitude


def load_lanelet_centerlines(osm_path, projector):
    root = ET.parse(str(osm_path)).getroot()
    nodes = {}
    for node in root.findall("node"):
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in node.findall("tag")}
        nodes[int(node.attrib["id"])] = (
            float(node.attrib["lon"]),
            float(node.attrib["lat"]),
            float(tags.get("ele", 0.0)),
        )
    ways = {
        int(way.attrib["id"]): [int(nd.attrib["ref"]) for nd in way.findall("nd")]
        for way in root.findall("way")
    }
    transformer, origin_easting, origin_northing, origin_altitude = projector
    centerlines = {}
    speed_limits = {}
    for relation in root.findall("relation"):
        tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        lanelet_id = int(relation.attrib["id"])
        centerline_members = [
            member
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
            and member.attrib.get("role") == "centerline"
        ]
        if len(centerline_members) != 1:
            continue
        way_id = int(centerline_members[0].attrib["ref"])
        points = []
        for node_id in ways.get(way_id, []):
            if node_id not in nodes:
                raise ValueError("centerline way {} references missing node {}".format(way_id, node_id))
            longitude, latitude, altitude = nodes[node_id]
            easting, northing = transformer.transform(longitude, latitude)
            points.append(
                (
                    easting - origin_easting,
                    northing - origin_northing,
                    altitude - origin_altitude,
                )
            )
        if len(points) >= 2:
            centerlines[lanelet_id] = points
        if tags.get("speed_limit"):
            try:
                speed_limits[lanelet_id] = float(tags["speed_limit"])
            except ValueError:
                pass
    return centerlines, speed_limits


def load_global_path(path):
    points = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) < 2:
                raise ValueError("global path line {} has fewer than 2 fields".format(line_number))
            points.append((float(fields[0]), float(fields[1])))
    if len(points) < 2:
        raise ValueError("global path must contain at least two points")
    return points


def squared_distance_xy(first, second):
    return (first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2


def nearest_reference_index(point, global_path):
    return min(
        range(len(global_path)),
        key=lambda index: squared_distance_xy(point, global_path[index]),
    )


def route_endpoint(row, prefix):
    return float(row[prefix + "_x"]), float(row[prefix + "_y"])


def orient_centerline(points, row, previous_end, global_path):
    route_start = route_endpoint(row, "start")
    route_end = route_endpoint(row, "end")

    def score(candidate):
        value = squared_distance_xy(candidate[0], route_start)
        value += squared_distance_xy(candidate[-1], route_end)
        if previous_end is not None:
            value += 4.0 * squared_distance_xy(candidate[0], previous_end)
        start_index = nearest_reference_index(candidate[0], global_path)
        end_index = nearest_reference_index(candidate[-1], global_path)
        if end_index < start_index:
            value += 1.0
        return value

    forward = list(points)
    reverse = list(reversed(points))
    return forward if score(forward) <= score(reverse) else reverse


def row_speed_limit_kph(row, lanelet_speed_kph, default_speed_limit_kph):
    raw = row.get("speed_limit_kph", "").strip()
    speed_kph = float(raw) if raw else lanelet_speed_kph
    if speed_kph is None:
        speed_kph = default_speed_limit_kph
    if not math.isfinite(speed_kph) or speed_kph <= 0.0:
        raise ValueError("invalid speed limit for lanelet {}".format(row["lanelet_id"]))
    return speed_kph


def target_speed_mps(
    row,
    lanelet_speed_kph,
    desired_velocity_kph,
    default_speed_limit_kph,
    speed_profile=None,
):
    requested_velocity_kph = desired_velocity_kph
    if speed_profile is not None:
        requested_velocity_kph = speed_profile["lanelet_velocity_kph"].get(
            row["lanelet_id"],
            speed_profile["default_velocity_kph"],
        )
        if not speed_profile["respect_speed_limits"]:
            return requested_velocity_kph / 3.6

    speed_limit_kph = row_speed_limit_kph(
        row, lanelet_speed_kph, default_speed_limit_kph
    )
    return min(requested_velocity_kph, speed_limit_kph) / 3.6


def stitch_chain(
    chain,
    centerlines,
    speed_limits,
    global_path,
    desired_velocity_kph,
    default_speed_limit_kph,
    speed_profile=None,
):
    vertices = []
    previous_end = None
    for row in chain:
        lanelet_id = row["lanelet_id"]
        if lanelet_id not in centerlines:
            raise ValueError("lanelet {} has no usable centerline".format(lanelet_id))
        points = orient_centerline(centerlines[lanelet_id], row, previous_end, global_path)
        speed = target_speed_mps(
            row,
            speed_limits.get(lanelet_id),
            desired_velocity_kph,
            default_speed_limit_kph,
            speed_profile,
        )
        if previous_end is not None:
            gap = math.sqrt(squared_distance_xy(previous_end, points[0]))
            if gap > 5.0:
                raise ValueError(
                    "lanelet {} starts {:.3f} m from previous centerline".format(lanelet_id, gap)
                )
            if gap < 1e-3:
                points = points[1:]
        for point in points:
            if vertices and math.dist(vertices[-1][:3], point) < 1e-6:
                continue
            vertices.append(
                (point[0], point[1], point[2], lanelet_id, row["order"], speed)
            )
        if not vertices:
            raise ValueError("empty stitched trajectory")
        previous_end = vertices[-1][:3]
    if len(vertices) < 2:
        raise ValueError("stitched trajectory has fewer than two points")
    return vertices


def resample(vertices, interval):
    stations = [0.0]
    for index in range(1, len(vertices)):
        distance = math.dist(vertices[index - 1][:3], vertices[index][:3])
        if distance <= 1e-9:
            continue
        stations.append(stations[-1] + distance)
    if len(stations) != len(vertices):
        raise ValueError("trajectory contains duplicate consecutive vertices")
    total_length = stations[-1]
    if total_length < interval:
        raise ValueError("trajectory length is shorter than resample interval")
    sample_stations = [index * interval for index in range(int(total_length // interval) + 1)]
    if total_length - sample_stations[-1] > 1e-6:
        sample_stations.append(total_length)
    else:
        sample_stations[-1] = total_length

    output = []
    for station in sample_stations:
        right = bisect.bisect_right(stations, station)
        segment = min(max(right - 1, 0), len(vertices) - 2)
        start_station = stations[segment]
        end_station = stations[segment + 1]
        ratio = (station - start_station) / (end_station - start_station)
        first = vertices[segment]
        second = vertices[segment + 1]
        output.append(
            {
                "x": first[0] + ratio * (second[0] - first[0]),
                "y": first[1] + ratio * (second[1] - first[1]),
                "z": first[2] + ratio * (second[2] - first[2]),
                "velocity_mps": first[5],
                "acceleration_mps2": 0.0,
                "lanelet_id": first[3],
                "route_order": first[4],
            }
        )
    for index, point in enumerate(output):
        previous = output[max(0, index - 1)]
        following = output[min(len(output) - 1, index + 1)]
        point["yaw"] = math.atan2(
            following["y"] - previous["y"], following["x"] - previous["x"]
        )
        point["index"] = index
    return output, total_length


def apply_speed_envelope(
    points,
    max_acceleration_mps2,
    max_deceleration_mps2,
):
    if len(points) < 2:
        raise ValueError("speed envelope requires at least two trajectory points")
    max_acceleration_mps2 = positive_finite(
        max_acceleration_mps2, "max_acceleration_mps2"
    )
    max_deceleration_mps2 = positive_finite(
        max_deceleration_mps2, "max_deceleration_mps2"
    )
    # velocity_mps is serialized to six decimal places. Keep a small margin so
    # the profile reconstructed from the CSV still respects the requested
    # limits after that quantization.
    quantization_margin_mps2 = 1.0e-3
    envelope_acceleration_mps2 = max(
        max_acceleration_mps2 - quantization_margin_mps2, 1.0e-9
    )
    envelope_deceleration_mps2 = max(
        max_deceleration_mps2 - quantization_margin_mps2, 1.0e-9
    )

    segment_lengths = []
    for first, second in zip(points, points[1:]):
        distance = math.dist(
            (first["x"], first["y"], first["z"]),
            (second["x"], second["y"], second["z"]),
        )
        if distance <= 1e-9:
            raise ValueError("trajectory contains duplicate consecutive points")
        segment_lengths.append(distance)

    velocities = [point["velocity_mps"] for point in points]
    for index in range(1, len(velocities)):
        reachable = math.sqrt(
            velocities[index - 1] ** 2
            + 2.0
            * envelope_acceleration_mps2
            * segment_lengths[index - 1]
        )
        velocities[index] = min(velocities[index], reachable)

    for index in range(len(velocities) - 2, -1, -1):
        reachable = math.sqrt(
            velocities[index + 1] ** 2
            + 2.0
            * envelope_deceleration_mps2
            * segment_lengths[index]
        )
        velocities[index] = min(velocities[index], reachable)

    velocities = [round(velocity, 6) for velocity in velocities]
    for index, point in enumerate(points):
        point["velocity_mps"] = velocities[index]
        if index + 1 < len(points):
            point["acceleration_mps2"] = (
                velocities[index + 1] ** 2 - velocities[index] ** 2
            ) / (2.0 * segment_lengths[index])
        else:
            point["acceleration_mps2"] = 0.0


def write_csv(output_path, points, overwrite):
    if output_path.exists() and not overwrite:
        raise FileExistsError(
            "output exists; pass --overwrite to replace it: {}".format(output_path)
        )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=output_path.name + ".", suffix=".tmp", dir=str(output_path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", newline="") as stream:
            writer = csv.DictWriter(
                stream, fieldnames=CSV_FIELDS, lineterminator="\n"
            )
            writer.writeheader()
            for point in points:
                writer.writerow(
                    {
                        "index": point["index"],
                        "x": "{:.9f}".format(point["x"]),
                        "y": "{:.9f}".format(point["y"]),
                        "z": "{:.9f}".format(point["z"]),
                        "yaw": "{:.12f}".format(point["yaw"]),
                        "velocity_mps": "{:.6f}".format(point["velocity_mps"]),
                        "acceleration_mps2": "{:.6f}".format(point["acceleration_mps2"]),
                        "lanelet_id": point["lanelet_id"],
                        "route_order": point["route_order"],
                    }
                )
        os.replace(temporary_name, output_path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main():
    parser = argparse.ArgumentParser(description="Generate a controller trajectory from Lanelet2")
    parser.add_argument("--config", required=True, help="repository-relative generator YAML")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing output CSV")
    parser.add_argument(
        "--start-lanelet-id",
        type=int,
        help="override anchors.start_lanelet_id from the config",
    )
    parser.add_argument(
        "--end-lanelet-id",
        type=int,
        help="override anchors.end_lanelet_id from the config",
    )
    parser.add_argument(
        "--start-route-order",
        type=int,
        help="select the start lanelet occurrence when its ID appears more than once",
    )
    parser.add_argument(
        "--end-route-order",
        type=int,
        help="select the end lanelet occurrence when its ID appears more than once",
    )
    parser.add_argument(
        "--desired-velocity-kph",
        type=float,
        help="override trajectory.desired_velocity_kph from the config",
    )
    parser.add_argument(
        "--output-csv",
        help="repository-relative CSV output path (default: traj_sample/<start>_<end>.csv)",
    )
    arguments = parser.parse_args()

    root = repository_root()
    config_path = resolve_repo_path(root, arguments.config, "config")
    config = load_config(config_path)
    map_config = require(config, "map", "config")
    reference_config = require(config, "reference", "config")
    anchors = require(config, "anchors", "config")
    trajectory_config = require(config, "trajectory", "config")

    osm_path = resolve_repo_path(
        root, require(map_config, "lanelet_map_file", "map"), "lanelet_map_file"
    )
    projector_path = resolve_repo_path(
        root, require(map_config, "projector_file", "map"), "projector_file"
    )
    route_path = resolve_repo_path(
        root, require(map_config, "route_lanelets_file", "map"), "route_lanelets_file"
    )
    global_path = resolve_repo_path(
        root,
        require(reference_config, "global_path_file", "reference"),
        "global_path_file",
    )
    start_id = (
        arguments.start_lanelet_id
        if arguments.start_lanelet_id is not None
        else int(require(anchors, "start_lanelet_id", "anchors"))
    )
    end_id = (
        arguments.end_lanelet_id
        if arguments.end_lanelet_id is not None
        else int(require(anchors, "end_lanelet_id", "anchors"))
    )
    interval = float(require(trajectory_config, "resample_interval_m", "trajectory"))
    desired_velocity_kph = (
        arguments.desired_velocity_kph
        if arguments.desired_velocity_kph is not None
        else float(require(trajectory_config, "desired_velocity_kph", "trajectory"))
    )
    default_speed_limit_kph = float(
        require(trajectory_config, "default_speed_limit_kph", "trajectory")
    )
    speed_profile = load_speed_profile(trajectory_config)
    if not math.isclose(interval, 0.1, rel_tol=0.0, abs_tol=1e-9):
        raise ValueError("resample_interval_m must be 0.1 for the MVP contract")
    if not math.isfinite(desired_velocity_kph) or desired_velocity_kph <= 0.0:
        raise ValueError("desired_velocity_kph must be finite and positive")
    if not math.isfinite(default_speed_limit_kph) or default_speed_limit_kph <= 0.0:
        raise ValueError("default_speed_limit_kph must be finite and positive")

    route_rows = load_route(route_path)
    chain = find_unique_chain(
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
    if speed_profile is not None:
        chain_lanelet_ids = {row["lanelet_id"] for row in chain}
        unused_overrides = sorted(
            set(speed_profile["lanelet_velocity_kph"]).difference(
                chain_lanelet_ids
            )
        )
        if unused_overrides:
            raise ValueError(
                "speed-profile lanelets are not in the selected chain: {}".format(
                    unused_overrides
                )
            )
    projector = parse_projector(projector_path)
    centerlines, speed_limits = load_lanelet_centerlines(osm_path, projector)
    reference_points = load_global_path(global_path)
    vertices = stitch_chain(
        chain,
        centerlines,
        speed_limits,
        reference_points,
        desired_velocity_kph,
        default_speed_limit_kph,
        speed_profile,
    )
    points, length_m = resample(vertices, interval)
    if speed_profile is not None:
        apply_speed_envelope(
            points,
            speed_profile["max_acceleration_mps2"],
            speed_profile["max_deceleration_mps2"],
        )

    configured_output = trajectory_config.get("output_csv")
    if arguments.output_csv:
        output_path = resolve_repo_output_path(
            root, arguments.output_csv, "output_csv"
        )
    elif configured_output:
        output_path = resolve_repo_output_path(
            root, configured_output, "trajectory.output_csv"
        )
    else:
        output_path = (
            root
            / "base_ws/src/control/trajectory_generator/traj_sample"
            / "{}_{}.csv".format(start_id, end_id)
        )
    write_csv(output_path, points, arguments.overwrite)
    print("trajectory generated: {}".format(output_path))
    print(
        "lanelets={} route_orders={}..{} length_m={:.3f} points={} interval_m={:.3f}".format(
            len(chain), chain[0]["order"], chain[-1]["order"], length_m, len(points), interval
        )
    )
    print(
        "desired_velocity_kph={:.3f} default_speed_limit_kph={:.3f} "
        "final_velocity_range_kph={:.3f}..{:.3f}".format(
            desired_velocity_kph,
            default_speed_limit_kph,
            min(point["velocity_mps"] for point in points) * 3.6,
            max(point["velocity_mps"] for point in points) * 3.6,
        )
    )
    if speed_profile is not None:
        print(
            "speed_profile default_kph={:.3f} overrides={} "
            "respect_speed_limits={} accel_mps2=+{:.3f}/-{:.3f}".format(
                speed_profile["default_velocity_kph"],
                len(speed_profile["lanelet_velocity_kph"]),
                speed_profile["respect_speed_limits"],
                speed_profile["max_acceleration_mps2"],
                speed_profile["max_deceleration_mps2"],
            )
        )


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        raise SystemExit("generate_trajectory: {}".format(error))
