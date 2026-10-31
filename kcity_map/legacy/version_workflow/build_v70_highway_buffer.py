#!/usr/bin/env python3
"""Build v70 with one-metre inner buffers and a 150 km/h highway limit."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer

from build_v69_speed_policy import HIGHWAY_EXCEPTION_LANELETS
from split_tollgate_v38_to_v39 import (
    build_road_lanelets,
    make_local_coord,
    next_numeric_id,
    segment_line_intersection,
    set_tag,
    split_lanelet,
    tags,
)


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v69.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v70.osm"
MGEO = ROOT / "mgeo_map/link_set.json"
START_LINK = "A2256W000411"
END_LINK = "A2256W000153"
BUFFER_M = 1.0
START_LANELETS = ("200101", "201434", "200152", "200142")
END_LANELETS = ("201451", "201452", "201450")
# 200136 shares boundary 102236 with 200142. It is not part of the
# competition high-speed corridor, but it must be split at the same section so
# the shared boundary and lateral topology remain valid.
AUXILIARY_START_LANELETS = ("200136",)
REFERENCE_START_PREDECESSOR = "201086"
REFERENCE_END_SUCCESSOR = "200088"


def way_refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def centerline_members(root):
    ways = {item.get("id"): item for item in root.findall("way")}
    result = {}
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet":
            continue
        center = next((m.get("ref") for m in relation.findall("member") if m.get("role") == "centerline"), None)
        if center:
            result[relation.get("id")] = way_refs(ways[center])
    return result


def polyline_point(points, distance_m):
    remaining = distance_m
    for first, second in zip(points, points[1:]):
        length = math.dist(first[:2], second[:2])
        if remaining <= length:
            ratio = remaining / length
            point = tuple(first[index] + ratio * (second[index] - first[index]) for index in range(3))
            tangent = (second[0] - first[0], second[1] - first[1])
            return point, tangent
        remaining -= length
    raise ValueError(f"distance {distance_m} exceeds polyline length")


def cut_line(point, tangent, half_width=30.0):
    norm = math.hypot(*tangent)
    nx, ny = -tangent[1] / norm, tangent[0] / norm
    return (
        (point[0] - nx * half_width, point[1] - ny * half_width),
        (point[0] + nx * half_width, point[1] + ny * half_width),
    )


def length(refs, coord):
    return sum(math.dist(coord[a], coord[b]) for a, b in zip(refs, refs[1:]))


def intersection_station(refs, coord, line):
    _, _, point = segment_line_intersection(refs, coord, line)
    station = 0.0
    for first, second in zip(refs, refs[1:]):
        segment_length = math.dist(coord[first], coord[second])
        first_to_point = math.dist(coord[first], point)
        point_to_second = math.dist(point, coord[second])
        if abs(first_to_point + point_to_second - segment_length) < 1e-5:
            return station + first_to_point
        station += segment_length
    raise AssertionError("split point is not on centerline")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    links = {item["idx"]: item for item in json.loads(MGEO.read_text())}
    start_points = [tuple(point) for point in links[START_LINK]["points"]]
    end_points = [tuple(point) for point in links[END_LINK]["points"]]
    end_length = sum(math.dist(a[:2], b[:2]) for a, b in zip(end_points, end_points[1:]))

    tree = etree.parse(str(INPUT), etree.XMLParser(remove_blank_text=False, huge_tree=True))
    root = tree.getroot()
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    coord = make_local_coord(nodes, to_utm)
    centers = centerline_members(root)

    predecessor_end = coord[centers[REFERENCE_START_PREDECESSOR][-1]]
    successor_start = coord[centers[REFERENCE_END_SUCCESSOR][0]]
    start_offset = (predecessor_end[0] - start_points[0][0], predecessor_end[1] - start_points[0][1])
    end_offset = (successor_start[0] - end_points[-1][0], successor_start[1] - end_points[-1][1])
    assert math.dist(start_offset, end_offset) < 1e-3, (start_offset, end_offset)
    offset = ((start_offset[0] + end_offset[0]) / 2.0, (start_offset[1] + end_offset[1]) / 2.0)
    def start_cross_section(reference_distance):
        point, tangent = polyline_point(start_points, reference_distance)
        local = (point[0] + offset[0], point[1] + offset[1], point[2])
        return point, cut_line(local, tangent)

    def end_cross_section(reference_buffer):
        point, tangent = polyline_point(end_points, end_length - reference_buffer)
        local = (point[0] + offset[0], point[1] + offset[1], point[2])
        return point, cut_line(local, tangent)

    # A single perpendicular cross-section keeps all parallel lanelets aligned.
    # Increase the reference-link offset only enough that the shortest lane-wise
    # buffer is exactly the requested one metre.
    low, high = BUFFER_M, BUFFER_M + 0.5
    for _ in range(50):
        middle = (low + high) / 2.0
        _, candidate = start_cross_section(middle)
        minimum = min(intersection_station(centers[item], coord, candidate) for item in START_LANELETS)
        if minimum < BUFFER_M:
            low = middle
        else:
            high = middle
    start_reference_buffer = high
    start_mgeo, start_cut = start_cross_section(start_reference_buffer)

    low, high = BUFFER_M, BUFFER_M + 0.5
    for _ in range(50):
        middle = (low + high) / 2.0
        _, candidate = end_cross_section(middle)
        minimum = min(
            length(centers[item], coord) - intersection_station(centers[item], coord, candidate)
            for item in END_LANELETS
        )
        if minimum < BUFFER_M:
            low = middle
        else:
            high = middle
    end_reference_buffer = high
    end_mgeo, end_cut = end_cross_section(end_reference_buffer)

    lanelets = build_road_lanelets(root)
    next_ids = {
        "node": next_numeric_id(root, "node"),
        "way": next_numeric_id(root, "way"),
        "relation": next_numeric_id(root, "relation"),
    }
    cache = {}
    split_ids = {}
    split_points = {}
    for lanelet_id in START_LANELETS:
        new_id, results = split_lanelet(
            root, ways, nodes, coord, to_wgs, next_ids, cache, lanelets,
            lanelet_id, "highway_start_plus_1m", start_cut, "150",
        )
        split_ids[lanelet_id] = new_id
        split_points[lanelet_id] = results["centerline"].local_point

    lanelets = build_road_lanelets(root)
    for lanelet_id in END_LANELETS:
        new_id, results = split_lanelet(
            root, ways, nodes, coord, to_wgs, next_ids, cache, lanelets,
            lanelet_id, "highway_end_minus_1m", end_cut, "58",
        )
        split_ids[lanelet_id] = new_id
        split_points[lanelet_id] = results["centerline"].local_point

    lanelets = build_road_lanelets(root)
    for lanelet_id in AUXILIARY_START_LANELETS:
        new_id, results = split_lanelet(
            root, ways, nodes, coord, to_wgs, next_ids, cache, lanelets,
            lanelet_id, "highway_start_plus_1m", start_cut, "58",
        )
        split_ids[lanelet_id] = new_id
        split_points[lanelet_id] = results["centerline"].local_point

    relations = {item.get("id"): item for item in root.findall("relation")}
    for lanelet_id in HIGHWAY_EXCEPTION_LANELETS:
        set_tag(relations[lanelet_id], "speed_limit", "150")
    for lanelet_id in START_LANELETS:
        set_tag(relations[lanelet_id], "speed_limit", "58")
        set_tag(relations[split_ids[lanelet_id]], "speed_limit", "150")
    for lanelet_id in END_LANELETS:
        set_tag(relations[lanelet_id], "speed_limit", "150")
        set_tag(relations[split_ids[lanelet_id]], "speed_limit", "58")
    for lanelet_id in AUXILIARY_START_LANELETS:
        set_tag(relations[lanelet_id], "speed_limit", "58")
        set_tag(relations[split_ids[lanelet_id]], "speed_limit", "58")
    root.find("MetaInfo").set("map_version", "70")

    # Recompute exact segment lengths from the final split ways.
    ways = {item.get("id"): item for item in root.findall("way")}
    final_lanelets = build_road_lanelets(root)
    segment_lengths = {}
    for old_id, new_id in split_ids.items():
        old_length = length(way_refs(ways[final_lanelets[old_id]["centerline"]]), coord)
        new_length = length(way_refs(ways[final_lanelets[new_id]["centerline"]]), coord)
        segment_lengths[old_id] = {"new_lanelet": new_id, "old_segment_m": old_length, "new_segment_m": new_length}

    road_speeds = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") == "lanelet" and relation_tags.get("subtype") == "road":
            road_speeds[relation_tags["speed_limit"]] += 1
    assert road_speeds == Counter({"58": 1133, "150": 41, "45": 34, "30": 14}), road_speeds
    start_buffers = [segment_lengths[item]["old_segment_m"] for item in START_LANELETS]
    end_buffers = [segment_lengths[item]["new_segment_m"] for item in END_LANELETS]
    assert min(start_buffers) >= BUFFER_M - 1e-6
    assert min(end_buffers) >= BUFFER_M - 1e-6
    assert min(start_buffers) < BUFFER_M + 1e-4
    assert min(end_buffers) < BUFFER_M + 1e-4

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    report = {
        "version": 70,
        "source_version": 69,
        "policy": {
            "unrestricted_corridor_map_limit_kph": 150,
            "outside_limit_kph": 58,
            "inner_buffer_m": BUFFER_M,
            "start_reference_link_offset_m": start_reference_buffer,
            "end_reference_link_offset_m": end_reference_buffer,
            "minimum_start_lane_buffer_m": min(start_buffers),
            "minimum_end_lane_buffer_m": min(end_buffers),
        },
        "mgeo_boundary": {
            "start_link": START_LINK,
            "start_xyz": start_points[0],
            "effective_start_xyz": start_mgeo,
            "end_link": END_LINK,
            "end_xyz": end_points[-1],
            "effective_end_xyz": end_mgeo,
        },
        "split_lanelets": segment_lengths,
        "split_center_points_local": split_points,
        "speed_distribution": dict(sorted(road_speeds.items(), key=lambda item: float(item[0]))),
        "sha256": sha256(OUTPUT),
    }
    report_path = ROOT / "docs/kcity_v70_highway_buffer_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
