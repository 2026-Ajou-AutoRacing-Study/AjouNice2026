#!/usr/bin/env python3
"""Validate v73 geometry, Lanelet2 routing, and competition-route contracts."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path

import lanelet2
import yaml
from lxml import etree
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v73.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v72.osm"
SPLIT_REFERENCE = ROOT / "lanelet_map/lanelet2_map_v60.osm"
REPORT = ROOT / "docs/kcity_v73_highway_lane_change_report.json"
ROUTE = ROOT.parent / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v73_competition_route.yaml"

PASSABLE_PAIRS = (
    (201459, 201458), (201458, 201457), (201457, 201456),
    (201436, 201435), (201435, 200099),
    (200186, 200138), (200138, 200137), (200137, 200128),
    (200141, 201444), (201444, 200187), (200187, 200102),
    (200148, 200127), (200127, 200130), (200130, 200134),
    (200147, 200185), (201448, 201447), (201400, 201399),
)

MERGE_CHAINS = (
    (200148, 201440, 200147),
    (200127, 201439, 200185),
    (200130, 201441, 200360),
    (200134, 201446, 200360),
)


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def role(relation, name):
    values = [item.get("ref") for item in relation.findall("member") if item.get("role") == name]
    assert len(values) == 1, (relation.get("id"), name, values)
    return values[0]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def semantic(element):
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        tuple(semantic(child) for child in element),
    )


def main():
    report = json.loads(REPORT.read_text())
    assert sha256(MAP) == report["output_sha256"]

    root = etree.parse(str(MAP), etree.XMLParser(huge_tree=True)).getroot()
    assert root.find("MetaInfo").get("map_version") == "73"
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}

    baseline_root = etree.parse(str(BASELINE), etree.XMLParser(huge_tree=True)).getroot()
    reference_root = etree.parse(str(SPLIT_REFERENCE), etree.XMLParser(huge_tree=True)).getroot()
    baseline_nodes = {item.get("id"): item for item in baseline_root.findall("node")}
    baseline_ways = {item.get("id"): item for item in baseline_root.findall("way")}
    baseline_relations = {item.get("id"): item for item in baseline_root.findall("relation")}
    reference_relations = {item.get("id"): item for item in reference_root.findall("relation")}
    reference_ways = {item.get("id"): item for item in reference_root.findall("way")}
    restored_ids = {str(item) for item in report["restored_lanelets"]}
    restored_way_ids = {
        item.get("ref")
        for lanelet_id in restored_ids
        for item in reference_relations[lanelet_id].findall("member")
        if item.get("type") == "way"
    }
    restored_node_ids = {
        item.get("ref")
        for way_id in restored_way_ids
        for item in reference_ways[way_id].findall("nd")
    }
    changed_nodes = {
        key for key in set(nodes) & set(baseline_nodes)
        if semantic(nodes[key]) != semantic(baseline_nodes[key])
    }
    assert changed_nodes <= restored_node_ids, sorted(changed_nodes - restored_node_ids)
    assert set(nodes) - set(baseline_nodes) <= restored_node_ids
    changed_ways = {
        key for key in set(ways) & set(baseline_ways)
        if semantic(ways[key]) != semantic(baseline_ways[key])
    }
    allowed_way_changes = restored_way_ids | {str(item) for item in report["passable_boundaries"]}
    assert changed_ways <= allowed_way_changes, sorted(changed_ways - allowed_way_changes)
    assert set(ways) - set(baseline_ways) <= restored_way_ids
    assert set(baseline_ways) - set(ways) == {str(item) for item in report["removed_superseded_ways"]}
    changed_relations = {
        key for key in set(relations) & set(baseline_relations)
        if semantic(relations[key]) != semantic(baseline_relations[key])
    }
    assert changed_relations <= restored_ids, sorted(changed_relations - restored_ids)
    assert set(relations) - set(baseline_relations) == restored_ids - set(baseline_relations)
    assert not (set(baseline_relations) - set(relations))
    road = {
        key: value for key, value in relations.items()
        if tags(value).get("type") == "lanelet" and tags(value).get("subtype") == "road"
    }
    speeds = Counter(tags(value).get("speed_limit") for value in road.values())
    assert speeds == Counter({"59": 1126, "150": 36, "45": 34, "30": 14}), speeds

    for left_id, right_id in PASSABLE_PAIRS:
        left = relations[str(left_id)]
        right = relations[str(right_id)]
        boundary = role(left, "right")
        assert boundary == role(right, "left"), (left_id, right_id, boundary, role(right, "left"))
        assert tags(ways[boundary]).get("type") == "line_thin", boundary
        assert tags(ways[boundary]).get("subtype") == "dashed", boundary

    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(37.2380038082, 126.7730819386)
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    lanelets = {int(item.id): item for item in lanelet_map.laneletLayer}

    for left_id, right_id in PASSABLE_PAIRS:
        assert graph.right(lanelets[left_id]).id == right_id, (left_id, right_id, "right")
        assert graph.left(lanelets[right_id]).id == left_id, (right_id, left_id, "left")

    for first, middle, last in MERGE_CHAINS:
        assert middle in {int(item.id) for item in graph.following(lanelets[first])}, (first, middle)
        assert last in {int(item.id) for item in graph.following(lanelets[middle])}, (middle, last)

    # Shared-boundary lanelets must meet without a positive-area overlap.
    polygons = {}
    for lanelet_id in {item for pair in PASSABLE_PAIRS for item in pair}:
        lanelet = lanelets[lanelet_id]
        polygons[lanelet_id] = Polygon(
            [(point.x, point.y) for point in lanelet.leftBound]
            + [(point.x, point.y) for point in reversed(lanelet.rightBound)]
        )
        assert polygons[lanelet_id].is_valid, lanelet_id
    for left_id, right_id in PASSABLE_PAIRS:
        overlap = polygons[left_id].intersection(polygons[right_id]).area
        assert overlap < 1e-6, (left_id, right_id, overlap)

    route = yaml.safe_load(ROUTE.read_text())
    assert route["map"] == {
        "version": 73,
        "file": "kcity_map/lanelet_map/lanelet2_map_v73.osm",
        "projector_file": "kcity_map/lanelet_map/map_projector_info_v1.yaml",
        "sha256": sha256(MAP),
    }
    for key in ("route_a", "route_b"):
        sequence = route[key]["expected_path_lanelet_ids"]
        assert sequence[sequence.index(200130):sequence.index(200360) + 1] == [200130, 201441, 200360]
        assert all(int(item) in lanelets for item in sequence)
    assert route["fallback_route"]["expected_path_lanelet_ids"][:2] == [201441, 200360]
    assert route["poses"]["fallback_cutoff_201441"]["lanelet_id"] == 201441
    assert route["poses"]["fallback_cutoff_201441"]["lanelet_s_m"] == 9.142493

    print(f"sha256={sha256(MAP)}")
    print(f"lanelets={len(lanelets)} road_lanelets={len(road)} speed_distribution={dict(speeds)}")
    print(f"passable_parallel_pairs={len(PASSABLE_PAIRS)} reciprocal_graph_checks={2 * len(PASSABLE_PAIRS)}")
    print(f"merge_chain_checks={len(MERGE_CHAINS)} shared_pair_overlap_max_m2=0")
    print("route_a_b_tollgate_sequence=200130,201441,200360 fallback_start=201441")


if __name__ == "__main__":
    main()
