#!/usr/bin/env python3
"""Validate the v80 -> v81 midpoint split and its routing topology."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path

import lanelet2
from lxml import etree
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v81.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v80.osm"
BUILD_REPORT = ROOT / "docs/kcity_v81_mid_bottleneck_lc_build.json"
LC_REPORT = ROOT / "docs/kcity_v76_highway_lc_build.json"
REPORT = ROOT / "docs/kcity_v81_mid_bottleneck_lc_validation.json"
EXPECTED_GRAPH_CHANGES = {201477, 201448, 201447, 201400}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


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
    assert root.find("MetaInfo").get("map_version") == "81"
    assert old_root.find("MetaInfo").get("map_version") == "80"

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    old_nodes = {int(item.get("id")): item for item in old_root.findall("node")}
    old_ways = {int(item.get("id")): item for item in old_root.findall("way")}
    old_relations = {int(item.get("id")): item for item in old_root.findall("relation")}
    new_nodes = set(nodes) - set(old_nodes)
    new_ways = set(ways) - set(old_ways)
    new_relations = set(relations) - set(old_relations)
    assert new_nodes == set(range(233909, 233914)), new_nodes
    assert new_ways == set(range(105619, 105624)), new_ways
    assert new_relations == {201478, 201479}, new_relations
    assert set(old_nodes) <= set(nodes)
    assert set(old_ways) <= set(ways)
    assert set(old_relations) <= set(relations)

    duplicate_positions = []
    for way_id, way in ways.items():
        node_refs = refs(way)
        assert set(node_refs) <= set(nodes), way_id
        for first, second in zip(node_refs, node_refs[1:]):
            a, b = nodes[first], nodes[second]
            if first == second or (a.get("lat"), a.get("lon"), tags(a).get("ele")) == (
                b.get("lat"), b.get("lon"), tags(b).get("ele")
            ):
                duplicate_positions.append((way_id, first, second))
    assert not duplicate_positions, duplicate_positions
    for relation in relations.values():
        for member in relation.findall("member"):
            layer = {"node": nodes, "way": ways, "relation": relations}[member.get("type")]
            assert int(member.get("ref")) in layer, (relation.get("id"), member.attrib)

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082, 126.7730819386))
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    old_map = lanelet2.io.load(str(BASELINE), projector)
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    old_graph = lanelet2.routing.RoutingGraph(old_map, rules)
    changed = {
        lanelet.id
        for lanelet in old_map.laneletLayer
        if signature(old_graph, lanelet) != signature(graph, lanelet_map.laneletLayer[lanelet.id])
    }
    assert changed == EXPECTED_GRAPH_CHANGES, changed

    expected = {
        201448: {"previous": [201476], "following": [201478], "right": 201479},
        201479: {"previous": [201477], "following": [201447], "left": 201448},
        201478: {"previous": [201448], "following": [201400], "right": 201447},
        201447: {"previous": [201479], "following": [201399], "left": 201478},
    }
    target_rows = {}
    for lanelet_id, required in expected.items():
        actual = signature(graph, lanelet_map.laneletLayer[lanelet_id])
        for key, value in required.items():
            assert actual[key] == value, (lanelet_id, key, actual[key], value)
        target_rows[str(lanelet_id)] = actual

    for left_id, right_id in ((201448, 201479), (201478, 201447)):
        left = lanelet_map.laneletLayer[left_id]
        right = lanelet_map.laneletLayer[right_id]
        assert left.rightBound.id == right.leftBound.id
        assert tags(ways[left.rightBound.id]).get("subtype") == "dashed"
        assert graph.right(left).id == right_id
        assert graph.left(right).id == left_id

    lc_pairs = json.loads(LC_REPORT.read_text())["bidirectional_pairs"]
    checked_pairs = []
    for left_id, right_id in lc_pairs:
        pairs = ((201448, 201479), (201478, 201447)) if (left_id, right_id) == (201448, 201447) else ((left_id, right_id),)
        for new_left, new_right in pairs:
            left = lanelet_map.laneletLayer[new_left]
            right = lanelet_map.laneletLayer[new_right]
            assert graph.right(left) and graph.right(left).id == new_right
            assert graph.left(right) and graph.left(right).id == new_left
            checked_pairs.append((new_left, new_right))

    speed_counts = Counter()
    invalid_polygons = []
    missing_longitudinal = []
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
        if attributes.get("speed_limit") == "150" and (
            not graph.previous(lanelet) or not graph.following(lanelet)
        ):
            missing_longitudinal.append(lanelet.id)
    assert not invalid_polygons, invalid_polygons
    assert not missing_longitudinal, missing_longitudinal
    assert speed_counts == Counter({"59": 1127, "150": 51, "45": 34, "30": 14}), speed_counts

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "sha256": sha256(MAP),
        "lanelets": len(lanelet_map.laneletLayer),
        "new_lanelets": sorted(new_relations),
        "new_ways": sorted(new_ways),
        "new_nodes": sorted(new_nodes),
        "graph_changes_from_v80": sorted(changed),
        "split_lanelet_graph": target_rows,
        "bidirectional_lc_pairs_checked": len(checked_pairs),
        "invalid_polygons": 0,
        "highway_150_missing_previous_or_following": 0,
        "speed_counts": dict(speed_counts),
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
