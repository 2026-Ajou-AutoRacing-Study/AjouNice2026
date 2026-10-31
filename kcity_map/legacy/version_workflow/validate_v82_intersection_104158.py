#!/usr/bin/env python3
"""Validate the v81 -> v82 extension of intersection-area polygon 104158."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import lanelet2
from lxml import etree
from shapely.geometry import LineString, Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v82.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v81.osm"
BUILD_REPORT = ROOT / "docs/kcity_v82_intersection_104158_build.json"
REPORT = ROOT / "docs/kcity_v82_intersection_104158_validation.json"
POLYGON_WAY_ID = 104158
MOVED_NODES = {231570, 231571, 231572, 231577, 231578, 231579}
TARGET_LANELETS = {200292, 200871, 201062, 200346}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def refs(way):
    return [int(item.get("ref")) for item in way.findall("nd")]


def signature(graph, lanelet):
    def one(method):
        item = method(lanelet)
        return item.id if item else None

    return {
        "previous": sorted(item.id for item in graph.previous(lanelet)),
        "following": sorted(item.id for item in graph.following(lanelet)),
        "left": one(graph.left),
        "right": one(graph.right),
        "adjacent_left": one(graph.adjacentLeft),
        "adjacent_right": one(graph.adjacentRight),
    }


def main():
    build = json.loads(BUILD_REPORT.read_text())
    assert sha256(MAP) == build["output_sha256"]
    assert sha256(BASELINE) == build["input_sha256"]

    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(MAP), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    assert root.find("MetaInfo").get("map_version") == "82"
    assert old_root.find("MetaInfo").get("map_version") == "81"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_nodes = {int(item.get("id")): item for item in old_root.findall("node")}
    old_ways = {int(item.get("id")): item for item in old_root.findall("way")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}

    assert set(nodes) == set(old_nodes)
    assert set(ways) == set(old_ways)
    assert set(relations) == set(old_relations)
    changed_nodes = {
        node_id
        for node_id in nodes
        if nodes[node_id].attrib != old_nodes[node_id].attrib
        or [item.attrib for item in nodes[node_id].findall("tag")]
        != [item.attrib for item in old_nodes[node_id].findall("tag")]
    }
    assert changed_nodes == MOVED_NODES, changed_nodes
    for way_id in ways:
        assert etree.tostring(ways[way_id]) == etree.tostring(old_ways[way_id]), way_id
    for relation_id in relations:
        assert etree.tostring(relations[relation_id]) == etree.tostring(old_relations[relation_id]), relation_id

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082, 126.7730819386))
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    old_map = lanelet2.io.load(str(BASELINE), projector)
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    old_graph = lanelet2.routing.RoutingGraph(old_map, rules)
    graph_changes = {
        lanelet.id
        for lanelet in old_map.laneletLayer
        if signature(old_graph, lanelet) != signature(graph, lanelet_map.laneletLayer[lanelet.id])
    }
    assert not graph_changes, graph_changes

    polygon = lanelet_map.polygonLayer[POLYGON_WAY_ID]
    old_polygon = old_map.polygonLayer[POLYGON_WAY_ID]
    polygon_shape = Polygon([(point.x, point.y) for point in polygon])
    old_polygon_shape = Polygon([(point.x, point.y) for point in old_polygon])
    assert polygon_shape.is_valid
    assert polygon_shape.area > old_polygon_shape.area
    assert math.isclose(build["area_before_m2"], old_polygon_shape.area, abs_tol=0.01)
    assert math.isclose(build["area_after_m2"], polygon_shape.area, abs_tol=0.01)
    for side in ("north", "south"):
        dx, dy = build[side]["translation_xy_m"]
        assert math.isclose(math.hypot(dx, dy), 5.0, abs_tol=1e-9)

    centerline_overlap = {}
    for lanelet_id in TARGET_LANELETS:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        old_lanelet = old_map.laneletLayer[lanelet_id]
        line = LineString([(point.x, point.y) for point in lanelet.centerline])
        old_line = LineString([(point.x, point.y) for point in old_lanelet.centerline])
        old_overlap = old_line.intersection(old_polygon_shape).length
        new_overlap = line.intersection(polygon_shape).length
        increase = new_overlap - old_overlap
        assert math.isclose(increase, 5.0, abs_tol=0.01), (lanelet_id, increase)
        centerline_overlap[str(lanelet_id)] = {
            "before_m": old_overlap,
            "after_m": new_overlap,
            "increase_m": increase,
        }

    invalid_lanelets = []
    for lanelet in lanelet_map.laneletLayer:
        shape = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        if not shape.is_valid or shape.area <= 0.0:
            invalid_lanelets.append(lanelet.id)
    assert not invalid_lanelets, invalid_lanelets

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "polygon_id": POLYGON_WAY_ID,
        "moved_nodes": sorted(changed_nodes),
        "north_extension_m": math.hypot(*build["north"]["translation_xy_m"]),
        "south_extension_m": math.hypot(*build["south"]["translation_xy_m"]),
        "area_before_m2": old_polygon_shape.area,
        "area_after_m2": polygon_shape.area,
        "area_increase_m2": polygon_shape.area - old_polygon_shape.area,
        "routing_graph_changes": sorted(graph_changes),
        "lanelets": len(lanelet_map.laneletLayer),
        "invalid_lanelets": invalid_lanelets,
        "target_lanelet_centerline_overlap": centerline_overlap,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
