#!/usr/bin/env python3
"""Validate v83 topology and intersection-area attachment for lanelet 200291."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import lanelet2
from lxml import etree
from shapely.geometry import LineString, Point, Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v83.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v82.osm"
BUILD_REPORT = ROOT / "docs/kcity_v83_lanelet_200291_intersection_build.json"
REPORT = ROOT / "docs/kcity_v83_lanelet_200291_intersection_validation.json"
POLYGON_ID = 104158
ENDPOINTS = {2447: 101881, 6120: 101881, 3050: 100396, 6030: 100396}
REMOVED_NODES = {6035, 6036, 6038}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def members(relation):
    return {(item.get("type"), item.get("role")): int(item.get("ref")) for item in relation.findall("member")}


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def main():
    build = json.loads(BUILD_REPORT.read_text())
    assert sha256(MAP) == build["output_sha256"]
    assert sha256(BASELINE) == build["input_sha256"]

    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(MAP), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    assert root.find("MetaInfo").get("map_version") == "83"
    assert old_root.find("MetaInfo").get("map_version") == "82"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_nodes = {int(item.get("id")): item for item in old_root.findall("node")}
    old_ways = {int(item.get("id")): item for item in old_root.findall("way")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}

    assert set(nodes) == set(old_nodes) - REMOVED_NODES
    assert set(ways) == set(old_ways) - {100394}
    assert set(relations) == set(old_relations)
    for node_id in nodes:
        assert etree.tostring(nodes[node_id]) == etree.tostring(old_nodes[node_id]), node_id
    changed_ways = {
        way_id for way_id in set(ways) & set(old_ways)
        if etree.tostring(ways[way_id]) != etree.tostring(old_ways[way_id])
    }
    changed_relations = {
        relation_id for relation_id in relations
        if etree.tostring(relations[relation_id]) != etree.tostring(old_relations[relation_id])
    }
    assert changed_ways == {100395, 104158}, changed_ways
    assert changed_relations == {200289}, changed_relations

    assert members(relations[200289])[("way", "left")] == 100395
    assert members(relations[200291])[("way", "left")] == 100395
    assert members(relations[200289])[("way", "right")] == 101881
    assert members(relations[200291])[("way", "right")] == 100396
    assert tags(ways[100395]) == {"type": "line_thin", "subtype": "dashed"}
    assert tags(ways[104158]) == {"type": "intersection_area", "area": "yes"}
    for lanelet_id in (200289, 200291):
        assert tags(relations[lanelet_id])["intersection_area"] == "104158"

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082, 126.7730819386))
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    lane_200289 = lanelet_map.laneletLayer[200289]
    lane_200291 = lanelet_map.laneletLayer[200291]

    assert lane_200291.leftBound.id == 100395 and not lane_200291.leftBound.inverted()
    assert lane_200289.leftBound.id == 100395 and lane_200289.leftBound.inverted()
    left_opposite = lanelet_map.laneletLayer.findUsages(lane_200291.leftBound.invert())
    left_opposite_ids = sorted(lane.id for lane in left_opposite if lane.rightBound.id != 100395)
    assert left_opposite_ids == [200289], left_opposite_ids
    # SOA getShiftableRatio uses getMostRight(opposite).invert(); its left bound is
    # therefore the opposite lane's original right/outer bound.
    soa_outer_left_bound = lane_200289.invert().leftBound.id
    assert soa_outer_left_bound == 101881

    polygon = lanelet_map.polygonLayer[POLYGON_ID]
    polygon_ids = [point.id for point in polygon]
    assert polygon_ids[0] == polygon_ids[-1]
    shape = Polygon([(point.x, point.y) for point in polygon])
    assert shape.is_valid and shape.area > 0.0

    endpoint_distances = {}
    for node_id, bound_id in ENDPOINTS.items():
        assert node_id in polygon_ids
        node = lanelet_map.pointLayer[node_id]
        polygon_boundary = LineString([(point.x, point.y) for point in polygon])
        distance = Point(node.x, node.y).distance(polygon_boundary)
        assert distance < 1e-9, (node_id, distance)
        assert node_id in [point.id for point in lanelet_map.lineStringLayer[bound_id]]
        endpoint_distances[str(node_id)] = distance

    invalid_lanelets = []
    for lanelet in lanelet_map.laneletLayer:
        lane_shape = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        if not lane_shape.is_valid or lane_shape.area <= 0.0:
            invalid_lanelets.append(lanelet.id)
    assert not invalid_lanelets, invalid_lanelets

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "map_load": "pass",
        "left_opposite_lanelets_of_200291": left_opposite_ids,
        "shared_center_bound": 100395,
        "shared_center_bound_tags": tags(ways[100395]),
        "soa_outer_left_bound": soa_outer_left_bound,
        "intersection_area": POLYGON_ID,
        "intersection_area_tags": tags(ways[104158]),
        "polygon_valid": shape.is_valid,
        "polygon_area_m2": shape.area,
        "v82_cap_extension_coordinates_preserved": True,
        "endpoint_distances_m": endpoint_distances,
        "invalid_lanelets": invalid_lanelets,
        "static_clearance_requirement_m": 2.09,
        "runtime_obstacle_clearance": "requires live obstacle pose and candidate path",
        "runtime_candidate_topic": "requires live planning scenario",
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
