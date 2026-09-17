#!/usr/bin/env python3
"""Validate the focused v79 -> v80 highway bottleneck cleanup."""

from __future__ import annotations

import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import lanelet2
import numpy as np
from lxml import etree
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v80.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v79.osm"
BUILD_REPORT = ROOT / "docs/kcity_v80_bottleneck_geometry_build.json"
LC_REPORT = ROOT / "docs/kcity_v76_highway_lc_build.json"
REPORT = ROOT / "docs/kcity_v80_bottleneck_geometry_validation.json"
TARGET_LANELETS = (200360, 201453, 200185, 201476, 201477)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def graph_signature(graph, lanelet):
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


def main() -> None:
    build = json.loads(BUILD_REPORT.read_text())
    assert sha256(MAP) == build["output_sha256"]
    assert sha256(BASELINE) == build["input_sha256"]

    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(MAP), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    assert root.find("MetaInfo").get("map_version") == "80"
    assert old_root.find("MetaInfo").get("map_version") == "79"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_nodes = {int(item.get("id")): item for item in old_root.findall("node")}
    old_ways = {int(item.get("id")): item for item in old_root.findall("way")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}

    removed_nodes = set(build["removed_orphan_nodes"])
    changed_nodes = {int(item) for item in build["changed_nodes"]}
    changed_ways = {int(item) for item in build["changed_ways"]}
    assert set(nodes) == set(old_nodes) - removed_nodes
    assert set(ways) == set(old_ways)
    assert set(relations) == set(old_relations)

    for relation_id in relations:
        assert etree.tostring(relations[relation_id]) == etree.tostring(
            old_relations[relation_id]
        )
    for way_id in set(ways) - changed_ways:
        assert etree.tostring(ways[way_id]) == etree.tostring(old_ways[way_id])
    for node_id in set(nodes) - changed_nodes:
        assert etree.tostring(nodes[node_id]) == etree.tostring(old_nodes[node_id])

    duplicate_positions = []
    used_nodes = set()
    for way_id, way in ways.items():
        refs = [int(item.get("ref")) for item in way.findall("nd")]
        assert set(refs) <= set(nodes), way_id
        used_nodes.update(refs)
        for first, second in zip(refs, refs[1:]):
            first_node, second_node = nodes[first], nodes[second]
            if first == second or (
                first_node.get("lat"), first_node.get("lon"), tags(first_node).get("ele")
            ) == (
                second_node.get("lat"), second_node.get("lon"), tags(second_node).get("ele")
            ):
                duplicate_positions.append((way_id, first, second))
    assert not duplicate_positions, duplicate_positions
    for relation in relations.values():
        for member in relation.findall("member"):
            layer = {"node": nodes, "way": ways, "relation": relations}[member.get("type")]
            assert int(member.get("ref")) in layer, (relation.get("id"), member.attrib)

    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(37.2380038082, 126.7730819386)
    )
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    old_map = lanelet2.io.load(str(BASELINE), projector)
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    old_graph = lanelet2.routing.RoutingGraph(old_map, rules)

    graph_changes = []
    for old_lanelet in old_map.laneletLayer:
        lanelet = lanelet_map.laneletLayer[old_lanelet.id]
        if graph_signature(old_graph, old_lanelet) != graph_signature(graph, lanelet):
            graph_changes.append(old_lanelet.id)
    assert not graph_changes, graph_changes

    speed_counts = Counter()
    invalid_polygons = []
    for lanelet in lanelet_map.laneletLayer:
        attributes = dict(lanelet.attributes)
        if attributes.get("subtype") == "road":
            speed_counts[attributes.get("speed_limit")] += 1
        polygon = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        if not polygon.is_valid or polygon.area <= 0.0:
            invalid_polygons.append(lanelet.id)
    assert not invalid_polygons, invalid_polygons
    assert speed_counts == Counter({"59": 1127, "150": 49, "45": 34, "30": 14})

    target_rows = []
    for lanelet_id in TARGET_LANELETS:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        assert len(lanelet.leftBound) == len(lanelet.rightBound) == len(lanelet.centerline) == 2
        corners = {
            (round(point.x, 7), round(point.y, 7))
            for point in list(lanelet.leftBound) + list(lanelet.rightBound)
        }
        assert len(corners) == 4, (lanelet_id, corners)
        left = np.array([(point.x, point.y) for point in lanelet.leftBound])
        right = np.array([(point.x, point.y) for point in lanelet.rightBound])
        center = np.array([(point.x, point.y) for point in lanelet.centerline])
        widths = [
            float(np.linalg.norm((1.0 - ratio) * left[0] + ratio * left[1]
                                 - ((1.0 - ratio) * right[0] + ratio * right[1])))
            for ratio in np.linspace(0.0, 1.0, 101)
        ]
        assert min(widths) >= 3.0, (lanelet_id, min(widths))
        assert max(widths) <= 3.6, (lanelet_id, max(widths))
        assert np.max(np.linalg.norm(center - (left + right) / 2.0, axis=1)) < 1e-3
        target_rows.append({
            "lanelet": lanelet_id,
            "width_min_m": min(widths),
            "width_max_m": max(widths),
            **graph_signature(graph, lanelet),
        })

    # 105449 and 105599 must be one collinear edge ending at the same split
    # point as 105603. This explicitly prevents the old X-shaped seam.
    def refs(way_id):
        return [int(item.get("ref")) for item in ways[way_id].findall("nd")]

    assert refs(105449)[-1] == refs(105599)[0]
    assert refs(105599)[-1] == refs(105603)[-1]
    points = {}
    for node_id, element in nodes.items():
        points[node_id] = np.array([float(element.get("lon")), float(element.get("lat"))])
    first = points[refs(105449)[0]]
    joint = points[refs(105449)[-1]]
    finish = points[refs(105599)[-1]]
    vector_a, vector_b = joint - first, finish - joint
    angle = math.degrees(
        math.acos(np.clip(np.dot(vector_a, vector_b)
                          / np.linalg.norm(vector_a) / np.linalg.norm(vector_b), -1.0, 1.0))
    )
    assert angle < 0.05, angle

    lc = json.loads(LC_REPORT.read_text())["bidirectional_pairs"]
    missing_lc = []
    for left_id, right_id in lc:
        left, right = lanelet_map.laneletLayer[left_id], lanelet_map.laneletLayer[right_id]
        if not graph.right(left) or graph.right(left).id != right_id:
            missing_lc.append((left_id, right_id, "right"))
        if not graph.left(right) or graph.left(right).id != left_id:
            missing_lc.append((right_id, left_id, "left"))
    assert not missing_lc, missing_lc

    missing_longitudinal = []
    for lanelet in lanelet_map.laneletLayer:
        if dict(lanelet.attributes).get("speed_limit") == "150":
            if not graph.previous(lanelet) or not graph.following(lanelet):
                missing_longitudinal.append(lanelet.id)
    assert not missing_longitudinal, missing_longitudinal

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "lanelets": len(lanelet_map.laneletLayer),
        "target_lanelets": target_rows,
        "removed_orphan_nodes": len(removed_nodes),
        "invalid_polygons": 0,
        "graph_changes_from_v79": 0,
        "bidirectional_lc_pairs": len(lc),
        "highway_150_missing_previous_or_following": 0,
        "speed_counts": dict(speed_counts),
        "105449_105599_joint_angle_deg": angle,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
