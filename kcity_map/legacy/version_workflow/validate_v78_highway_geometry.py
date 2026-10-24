#!/usr/bin/env python3
"""Validate v78 highway geometry, routing topology, LC coverage, and cleanup."""

from __future__ import annotations

import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

import lanelet2
import numpy as np
from lxml import etree
from shapely.geometry import LineString, Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v78.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v77.osm"
BUILD_REPORT = ROOT / "docs/kcity_v77_highway_geometry_build.json"
V76_LC_REPORT = ROOT / "docs/kcity_v76_highway_lc_build.json"
REPORT = ROOT / "docs/kcity_v78_highway_geometry_validation.json"

MERGE_ONLY_PAIRS = (
    (201467, 201468),
    (201474, 201475),
    (201439, 201441),
    (201441, 201446),
    (200185, 200360),
)


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def graph_signature(graph, lanelet):
    def one(method):
        value = method(lanelet)
        return value.id if value else None

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
    v76_lc = json.loads(V76_LC_REPORT.read_text())
    target = set(build["target_lanelets"])
    cleanup = json.loads((ROOT / "docs/kcity_v78_obsolete_geometry_build.json").read_text())
    assert sha256(MAP) == cleanup["output_sha256"]

    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(MAP), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    assert root.find("MetaInfo").get("map_version") == "78"
    assert old_root.find("MetaInfo").get("map_version") == "77"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}
    assert set(relations) == set(old_relations)

    lanelet_relations = {
        key: value for key, value in relations.items()
        if tags(value).get("type") == "lanelet"
    }
    assert len(lanelet_relations) == 1313

    def role(relation, name):
        values = [
            int(item.get("ref")) for item in relation.findall("member")
            if item.get("type") == "way" and item.get("role") == name
        ]
        assert len(values) == 1, (relation.get("id"), name, values)
        return values[0]

    # Every primitive reference resolves, and no linestring contains duplicate
    # adjacent point IDs or duplicate adjacent positions.
    duplicate_points = []
    for way_id, way in ways.items():
        refs = [int(item.get("ref")) for item in way.findall("nd")]
        assert set(refs) <= set(nodes), way_id
        for first, second in zip(refs, refs[1:]):
            first_node, second_node = nodes[first], nodes[second]
            if first == second or (
                first_node.get("lat"), first_node.get("lon"), tags(first_node).get("ele")
            ) == (
                second_node.get("lat"), second_node.get("lon"), tags(second_node).get("ele")
            ):
                duplicate_points.append((way_id, first, second))
    assert not duplicate_points, duplicate_points
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

    # This release is a geometry cleanup. All longitudinal and lateral routing
    # relations from v76 must remain bit-for-bit equivalent at the graph level.
    graph_changes = []
    for old_lanelet in old_map.laneletLayer:
        lanelet = lanelet_map.laneletLayer[old_lanelet.id]
        old_signature = graph_signature(old_graph, old_lanelet)
        signature = graph_signature(graph, lanelet)
        if signature != old_signature:
            graph_changes.append((lanelet.id, old_signature, signature))
    assert not graph_changes, graph_changes

    speed_counts = Counter()
    invalid_polygons = []
    for lanelet in lanelet_map.laneletLayer:
        attrs = dict(lanelet.attributes)
        if attrs.get("subtype") == "road":
            speed_counts[attrs.get("speed_limit")] += 1
        polygon = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        if not polygon.is_valid or polygon.area <= 0.0:
            invalid_polygons.append(lanelet.id)
    assert not invalid_polygons, invalid_polygons
    assert speed_counts == Counter({"59": 1127, "150": 49, "45": 34, "30": 14}), speed_counts

    boundary_uses = defaultdict(list)
    for lanelet_id, relation in lanelet_relations.items():
        for name in ("left", "right"):
            boundary_uses[role(relation, name)].append((lanelet_id, name))

    target_rows = []
    target_boundaries = set()
    for lanelet_id in sorted(target):
        relation = lanelet_relations[lanelet_id]
        left_id, right_id = role(relation, "left"), role(relation, "right")
        center_id = role(relation, "centerline")
        target_boundaries.update((left_id, right_id))
        lanelet = lanelet_map.laneletLayer[lanelet_id]
        assert len(lanelet.leftBound) == len(lanelet.rightBound) == len(lanelet.centerline) == 2
        corners = {
            (round(point.x, 7), round(point.y, 7))
            for point in list(lanelet.leftBound) + list(lanelet.rightBound)
        }
        assert len(corners) == 4, (lanelet_id, corners)
        left = np.array([(point.x, point.y) for point in lanelet.leftBound])
        right = np.array([(point.x, point.y) for point in lanelet.rightBound])
        widths = [
            float(np.linalg.norm((1.0 - ratio) * left[0] + ratio * left[1]
                                 - ((1.0 - ratio) * right[0] + ratio * right[1])))
            for ratio in np.linspace(0.0, 1.0, 101)
        ]
        polygon = Polygon(list(map(tuple, left)) + list(map(tuple, right[::-1])))
        centerline = LineString([(point.x, point.y) for point in lanelet.centerline])
        assert min(widths) >= 2.9, (lanelet_id, min(widths))
        assert max(widths) <= 4.0, (lanelet_id, max(widths))
        assert centerline.difference(polygon.buffer(0.01)).length <= 0.01, lanelet_id
        target_rows.append({
            "lanelet": lanelet_id,
            "left_way": left_id,
            "right_way": right_id,
            "centerline_way": center_id,
            "width_min_m": min(widths),
            "width_max_m": max(widths),
            "area_m2": polygon.area,
            "previous": [item.id for item in graph.previous(lanelet)],
            "following": [item.id for item in graph.following(lanelet)],
            "left": graph.left(lanelet).id if graph.left(lanelet) else None,
            "right": graph.right(lanelet).id if graph.right(lanelet) else None,
        })

    # All changed boundaries are wholly contained in the cleanup scope.
    outside_users = {
        boundary_id: uses for boundary_id, uses in boundary_uses.items()
        if boundary_id in target_boundaries and any(item[0] not in target for item in uses)
    }
    assert not outside_users, outside_users

    # A passable separator must be one canonical dashed way referenced by two
    # lanelets. Every v76 LC pair remains reciprocal in v77.
    malformed_dashed = []
    for boundary_id in sorted(target_boundaries):
        if tags(ways[boundary_id]).get("subtype") == "dashed":
            if len(boundary_uses[boundary_id]) != 2:
                malformed_dashed.append((boundary_id, boundary_uses[boundary_id]))
    assert not malformed_dashed, malformed_dashed

    missing_lc = []
    for left_id, right_id in v76_lc["bidirectional_pairs"]:
        left_lanelet = lanelet_map.laneletLayer[left_id]
        right_lanelet = lanelet_map.laneletLayer[right_id]
        if not graph.right(left_lanelet) or graph.right(left_lanelet).id != right_id:
            missing_lc.append((left_id, right_id, "right"))
        if not graph.left(right_lanelet) or graph.left(right_lanelet).id != left_id:
            missing_lc.append((right_id, left_id, "left"))
    assert not missing_lc, missing_lc

    # Overlapping merge corridors cannot use one shared edge. They retain
    # separate quadrilateral geometry and valid forward successors instead.
    merge_rows = []
    for first_id, second_id in MERGE_ONLY_PAIRS:
        first, second = lanelet_map.laneletLayer[first_id], lanelet_map.laneletLayer[second_id]
        first_poly = Polygon(
            [(point.x, point.y) for point in first.leftBound]
            + [(point.x, point.y) for point in reversed(first.rightBound)]
        )
        second_poly = Polygon(
            [(point.x, point.y) for point in second.leftBound]
            + [(point.x, point.y) for point in reversed(second.rightBound)]
        )
        merge_rows.append({
            "lanelets": [first_id, second_id],
            "overlap_m2": first_poly.intersection(second_poly).area,
            "first_following": [item.id for item in graph.following(first)],
            "second_following": [item.id for item in graph.following(second)],
        })
        assert graph.following(first) and graph.following(second), (first_id, second_id)

    highway_rows = []
    for lanelet in sorted(lanelet_map.laneletLayer, key=lambda item: item.id):
        if dict(lanelet.attributes).get("speed_limit") != "150":
            continue
        signature = graph_signature(graph, lanelet)
        assert signature["previous"], ("no_previous", lanelet.id)
        assert signature["following"], ("no_following", lanelet.id)
        highway_rows.append({"lanelet": lanelet.id, **signature})

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "lanelets": len(lanelet_map.laneletLayer),
        "target_lanelets": len(target),
        "quadrilateral_targets": len(target_rows),
        "invalid_polygons": 0,
        "graph_changes_from_v77": 0,
        "bidirectional_lc_pairs": len(v76_lc["bidirectional_pairs"]),
        "malformed_dashed_boundaries": 0,
        "speed_counts": dict(speed_counts),
        "target_geometry": target_rows,
        "merge_only_corridors": merge_rows,
        "highway_150_topology": highway_rows,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("target_geometry", "merge_only_corridors", "highway_150_topology")}, indent=2))


if __name__ == "__main__":
    main()
