#!/usr/bin/env python3
"""Build v82 by extending intersection-area polygon 104158 by 5 m at both road ends."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v81.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v82.osm"
REPORT = ROOT / "docs/kcity_v82_intersection_104158_build.json"
EXPECTED_INPUT_SHA256 = "4ece7b1b549cd6920018195f8f3ec8ff6c4925ee3e290ca92d86ae17a2cac892"

POLYGON_WAY_ID = "104158"
EXTENSION_M = 5.0
NORTH_LANELETS = ("200292", "200871")
SOUTH_LANELETS = ("201062", "200346")
NORTH_CAP_NODES = ("231577", "231578", "231579")
SOUTH_CAP_NODES = ("231570", "231571", "231572")


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    input_sha = sha256(INPUT)
    if input_sha != EXPECTED_INPUT_SHA256:
        raise RuntimeError(f"unexpected v81 input SHA: {input_sha}")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    meta_info = root.find("MetaInfo")
    if meta_info is None or meta_info.get("map_version") != "81":
        raise RuntimeError("unexpected v81 MetaInfo")
    meta_info.set("map_version", "82")

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    polygon_way = ways[POLYGON_WAY_ID]
    polygon_refs = refs(polygon_way)
    if tags(polygon_way) != {"type": "intersection_area", "area": "yes"}:
        raise RuntimeError(f"unexpected tags on polygon {POLYGON_WAY_ID}")
    if polygon_refs[0] != polygon_refs[-1]:
        raise RuntimeError(f"polygon {POLYGON_WAY_ID} is not closed")

    moved_nodes = set(NORTH_CAP_NODES + SOUTH_CAP_NODES)
    all_way_refs = [node_id for way in ways.values() for node_id in refs(way)]
    for node_id in moved_nodes:
        if all_way_refs.count(node_id) != 1 or node_id not in polygon_refs:
            raise RuntimeError(f"polygon cap node {node_id} is unexpectedly shared")

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)

    def xy(node_id):
        node = nodes[node_id]
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    def way_points(way_id):
        return [xy(node_id) for node_id in refs(ways[way_id])]

    polygon_before = Polygon(way_points(POLYGON_WAY_ID))
    if not polygon_before.is_valid:
        raise RuntimeError(f"polygon {POLYGON_WAY_ID} is invalid before extension")

    def centerline_id(lanelet_id):
        relation = relations[lanelet_id]
        return next(
            member.get("ref")
            for member in relation.findall("member")
            if member.get("role") == "centerline"
        )

    def outward_displacement(lanelet_id):
        line = LineString(way_points(centerline_id(lanelet_id)))
        first = Point(line.coords[0])
        last = Point(line.coords[-1])
        if first.distance(polygon_before.centroid) < last.distance(polygon_before.centroid):
            inner = line.interpolate(0.0)
            outer = line.interpolate(min(EXTENSION_M, line.length))
        else:
            inner = line.interpolate(line.length)
            outer = line.interpolate(max(0.0, line.length - EXTENSION_M))
        dx, dy = outer.x - inner.x, outer.y - inner.y
        norm = math.hypot(dx, dy)
        if norm < EXTENSION_M - 0.01:
            raise RuntimeError(f"lanelet {lanelet_id} has insufficient centerline length")
        return dx / norm, dy / norm

    def averaged_extension(lanelet_ids):
        directions = [outward_displacement(lanelet_id) for lanelet_id in lanelet_ids]
        dx = sum(item[0] for item in directions)
        dy = sum(item[1] for item in directions)
        norm = math.hypot(dx, dy)
        return EXTENSION_M * dx / norm, EXTENSION_M * dy / norm

    north_extension = averaged_extension(NORTH_LANELETS)
    south_extension = averaged_extension(SOUTH_LANELETS)
    original_positions = {}
    updated_positions = {}

    def move(node_ids, displacement):
        dx, dy = displacement
        for node_id in node_ids:
            old_x, old_y = xy(node_id)
            new_x, new_y = old_x + dx, old_y + dy
            lon, lat = to_wgs.transform(new_x, new_y)
            original_positions[node_id] = [old_x, old_y]
            updated_positions[node_id] = [new_x, new_y]
            nodes[node_id].set("lat", f"{lat:.14f}")
            nodes[node_id].set("lon", f"{lon:.14f}")

    move(NORTH_CAP_NODES, north_extension)
    move(SOUTH_CAP_NODES, south_extension)

    polygon_after = Polygon(way_points(POLYGON_WAY_ID))
    if not polygon_after.is_valid or polygon_after.area <= polygon_before.area:
        raise RuntimeError(f"invalid extended polygon {POLYGON_WAY_ID}")

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=False)
    output_sha = sha256(OUTPUT)
    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": input_sha,
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": output_sha,
        "polygon_id": int(POLYGON_WAY_ID),
        "extension_m": EXTENSION_M,
        "north": {
            "lanelet_ids": [int(item) for item in NORTH_LANELETS],
            "cap_node_ids": [int(item) for item in NORTH_CAP_NODES],
            "translation_xy_m": list(north_extension),
        },
        "south": {
            "lanelet_ids": [int(item) for item in SOUTH_LANELETS],
            "cap_node_ids": [int(item) for item in SOUTH_CAP_NODES],
            "translation_xy_m": list(south_extension),
        },
        "area_before_m2": polygon_before.area,
        "area_after_m2": polygon_after.area,
        "area_increase_m2": polygon_after.area - polygon_before.area,
        "original_positions_utm": original_positions,
        "updated_positions_utm": updated_positions,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
