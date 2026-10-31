#!/usr/bin/env python3
"""Validate the v85 crosswalk 201190 regulatory-element release."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import sys

from lxml import etree
from shapely.geometry import LineString, Polygon


ROOT = Path(__file__).resolve().parents[2]
EXTENSION_LIBRARY = (
    ROOT.parent / "planning_ws/install_isolated/lib/libautoware_lanelet2_extension_lib.so"
)
if EXTENSION_LIBRARY.exists() and str(EXTENSION_LIBRARY) not in os.environ.get("LD_PRELOAD", ""):
    environment = os.environ.copy()
    environment["LD_PRELOAD"] = ":".join(
        item for item in [str(EXTENSION_LIBRARY), environment.get("LD_PRELOAD", "")] if item
    )
    os.execve(sys.executable, [sys.executable, *sys.argv], environment)

import lanelet2  # noqa: E402  # custom regulatory-element factory must be preloaded first


MAP = ROOT / "lanelet_map/lanelet2_map_v85.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v84.osm"
BUILD_REPORT = ROOT / "docs/kcity_v85_crosswalk_201190_regulatory_build.json"
REPORT = ROOT / "docs/kcity_v85_crosswalk_201190_regulatory_validation.json"

CROSSWALK_LANELET_ID = 201190
CROSSWALK_POLYGON_ID = 105624
CROSSWALK_REGULATORY_ID = 201480
CROSSWALK_NODE_IDS = [92376, 92377, 92379, 92378, 92376]
ROAD_LANELET_IDS = {200309, 200383, 200433, 200950}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def member_tuples(relation):
    return [
        (item.get("type"), item.get("role"), int(item.get("ref")))
        for item in relation.findall("member")
    ]


def main():
    build = json.loads(BUILD_REPORT.read_text())
    assert sha256(MAP) == build["output_sha256"]
    assert sha256(BASELINE) == build["input_sha256"]

    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(MAP), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    assert root.find("MetaInfo").get("map_version") == "85"
    assert old_root.find("MetaInfo").get("map_version") == "84"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    old_nodes = {int(item.get("id")): item for item in old_root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    old_ways = {int(item.get("id")): item for item in old_root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}

    assert set(nodes) == set(old_nodes)
    assert set(ways) == set(old_ways) | {CROSSWALK_POLYGON_ID}
    assert set(relations) == set(old_relations) | {CROSSWALK_REGULATORY_ID}
    for node_id in nodes:
        assert etree.tostring(nodes[node_id]) == etree.tostring(old_nodes[node_id]), node_id
    for way_id in old_ways:
        assert etree.tostring(ways[way_id]) == etree.tostring(old_ways[way_id]), way_id
    for relation_id in old_relations:
        if relation_id in ROAD_LANELET_IDS:
            current = relations[relation_id]
            previous = old_relations[relation_id]
            new_member = ("relation", "regulatory_element", CROSSWALK_REGULATORY_ID)
            assert current.attrib == previous.attrib, relation_id
            assert [item for item in member_tuples(current) if item != new_member] == member_tuples(
                previous
            ), relation_id
            assert tags(current) == tags(previous), relation_id
        else:
            assert etree.tostring(relations[relation_id]) == etree.tostring(
                old_relations[relation_id]
            ), relation_id

    polygon_way = ways[CROSSWALK_POLYGON_ID]
    assert [int(item.get("ref")) for item in polygon_way.findall("nd")] == CROSSWALK_NODE_IDS
    assert tags(polygon_way) == {"type": "crosswalk_polygon", "area": "yes"}

    regulatory = relations[CROSSWALK_REGULATORY_ID]
    assert member_tuples(regulatory) == [
        ("way", "crosswalk_polygon", CROSSWALK_POLYGON_ID),
        ("relation", "refers", CROSSWALK_LANELET_ID),
    ]
    assert tags(regulatory) == {"type": "regulatory_element", "subtype": "crosswalk"}

    for lanelet_id in ROAD_LANELET_IDS:
        assert member_tuples(relations[lanelet_id]).count(
            ("relation", "regulatory_element", CROSSWALK_REGULATORY_ID)
        ) == 1

    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(37.2380038082, 126.7730819386)
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    crosswalk_lanelet = lanelet_map.laneletLayer[CROSSWALK_LANELET_ID]
    assert str(crosswalk_lanelet.attributes["subtype"]) == "crosswalk"
    polygon = lanelet_map.polygonLayer[CROSSWALK_POLYGON_ID]
    assert [point.id for point in polygon] == CROSSWALK_NODE_IDS
    shape = Polygon([(point.x, point.y) for point in polygon])
    assert shape.is_valid and shape.area > 0.0

    attached_lanelets = set()
    for lanelet in lanelet_map.laneletLayer:
        if any(reg.id == CROSSWALK_REGULATORY_ID for reg in lanelet.regulatoryElements):
            attached_lanelets.add(lanelet.id)
    assert attached_lanelets == ROAD_LANELET_IDS, attached_lanelets

    intersecting_roads = set()
    invalid_lanelets = []
    for lanelet in lanelet_map.laneletLayer:
        lane_shape = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        if not lane_shape.is_valid or lane_shape.area <= 0.0:
            invalid_lanelets.append(lanelet.id)
        if (
            str(lanelet.attributes["subtype"]) == "road"
            and shape.intersection(lane_shape).area > 1e-6
        ):
            intersecting_roads.add(lanelet.id)
    assert intersecting_roads == ROAD_LANELET_IDS, intersecting_roads
    assert not invalid_lanelets, invalid_lanelets

    nearby_stop_lines = []
    for line in lanelet_map.lineStringLayer:
        if "type" in line.attributes and str(line.attributes["type"]) == "stop_line":
            distance = shape.distance(LineString([(point.x, point.y) for point in line]))
            if distance < 50.0:
                nearby_stop_lines.append({"id": line.id, "distance_m": distance})
    assert not nearby_stop_lines, nearby_stop_lines

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "map_load": "pass",
        "crosswalk_lanelet_id": CROSSWALK_LANELET_ID,
        "crosswalk_polygon_id": CROSSWALK_POLYGON_ID,
        "crosswalk_regulatory_element_id": CROSSWALK_REGULATORY_ID,
        "crosswalk_polygon_area_m2": shape.area,
        "intersecting_road_lanelets": sorted(intersecting_roads),
        "regulatory_element_attached_lanelets": sorted(attached_lanelets),
        "nearby_stop_lines_within_50m": nearby_stop_lines,
        "invalid_lanelets": invalid_lanelets,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
