#!/usr/bin/env python3
"""Exhaustively validate the v70 buffered 150 km/h highway corridor."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Point, Polygon
import yaml

from build_v69_speed_policy import HIGHWAY_EXCEPTION_LANELETS
from build_v70_highway_buffer import (
    AUXILIARY_START_LANELETS,
    END_LANELETS,
    START_LANELETS,
    tags,
)


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v70.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v69.osm"
REPORT = ROOT / "docs/kcity_v70_highway_buffer_report.json"
ROUTE = ROOT.parent / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v70_competition_route.yaml"
SPLITS = {
    "200101": "201456", "201434": "201457", "200152": "201458", "200142": "201459",
    "201451": "201460", "201452": "201461", "201450": "201462",
    "200136": "201463",
}


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def members(relation):
    return [(item.get("type"), item.get("ref"), item.get("role")) for item in relation.findall("member")]


def role(relation, name):
    values = [item.get("ref") for item in relation.findall("member") if item.get("role") == name]
    assert len(values) == 1, (relation.get("id"), name, values)
    return values[0]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root = etree.parse(str(INPUT), etree.XMLParser(huge_tree=True)).getroot()
    old_root = etree.parse(str(BASELINE), etree.XMLParser(huge_tree=True)).getroot()
    assert root.find("MetaInfo").get("map_version") == "70"
    report = json.loads(REPORT.read_text())
    assert sha256(INPUT) == report["sha256"]

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    old_nodes = {item.get("id"): item for item in old_root.findall("node")}
    old_ways = {item.get("id"): item for item in old_root.findall("way")}
    old_relations = {item.get("id"): item for item in old_root.findall("relation")}
    assert len(nodes) == len(set(nodes)) and len(ways) == len(set(ways)) and len(relations) == len(set(relations))
    assert set(old_nodes) <= set(nodes) and set(old_ways) <= set(ways) and set(old_relations) <= set(relations)
    assert set(relations) - set(old_relations) == set(SPLITS.values())
    assert all(etree.tostring(nodes[key]) == etree.tostring(old_nodes[key]) for key in old_nodes)

    missing_refs = []
    adjacent_duplicates = []
    for way_id, way in ways.items():
        values = refs(way)
        for node_id in values:
            if node_id not in nodes:
                missing_refs.append((way_id, node_id))
        adjacent_duplicates.extend((way_id, a, b) for a, b in zip(values, values[1:]) if a == b)
    for relation_id, relation in relations.items():
        for member in relation.findall("member"):
            layer = {"node": nodes, "way": ways, "relation": relations}[member.get("type")]
            if member.get("ref") not in layer:
                missing_refs.append((relation_id, member.get("ref")))
    assert not missing_refs and not adjacent_duplicates

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    xy = {
        key: to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        for key, node in nodes.items()
    }
    lanelets = {key: value for key, value in relations.items() if tags(value).get("type") == "lanelet"}
    road = {key: value for key, value in lanelets.items() if tags(value).get("subtype") == "road"}
    assert len(lanelets) == 1311 and len(road) == 1222

    outside_centerlines = []
    for lanelet_id, lanelet in road.items():
        polygon = Polygon(
            [xy[item] for item in refs(ways[role(lanelet, "left")])]
            + [xy[item] for item in reversed(refs(ways[role(lanelet, "right")]))]
        ).buffer(0.001)
        center = refs(ways[role(lanelet, "centerline")])
        if not polygon.is_valid or any(not polygon.covers(Point(xy[item])) for item in center[1:-1]):
            outside_centerlines.append(lanelet_id)
    assert not outside_centerlines, outside_centerlines

    # Every pre-existing relation keeps identical membership. Only the highway
    # speed tags and the eight split source ways may change. Lanelet 200136 is
    # an auxiliary 58 km/h split needed to preserve its shared boundary.
    for relation_id, old_relation in old_relations.items():
        assert members(relations[relation_id]) == members(old_relation), relation_id
        current_tags = tags(relations[relation_id])
        previous_tags = tags(old_relation)
        if relation_id in HIGHWAY_EXCEPTION_LANELETS:
            expected = dict(previous_tags)
            expected["speed_limit"] = "58" if relation_id in START_LANELETS else "150"
            assert current_tags == expected, relation_id
        else:
            assert current_tags == previous_tags, relation_id

    target_way_ids = {
        role(old_relations[lanelet_id], role_name)
        for lanelet_id in tuple(START_LANELETS) + tuple(END_LANELETS) + tuple(AUXILIARY_START_LANELETS)
        for role_name in ("left", "right", "centerline")
    }
    assert all(
        etree.tostring(ways[key]) == etree.tostring(old_ways[key])
        for key in old_ways if key not in target_way_ids
    )

    expected_fast = (set(HIGHWAY_EXCEPTION_LANELETS) - set(START_LANELETS)) | {
        SPLITS[item] for item in START_LANELETS
    }
    actual_fast = {key for key, value in road.items() if tags(value).get("speed_limit") == "150"}
    assert actual_fast == expected_fast
    speeds = Counter(tags(value).get("speed_limit") for value in road.values())
    assert speeds == Counter({"58": 1133, "150": 41, "45": 34, "30": 14}), speeds

    for old_id, new_id in SPLITS.items():
        old_center = refs(ways[role(relations[old_id], "centerline")])
        new_center = refs(ways[role(relations[new_id], "centerline")])
        assert old_center[-1] == new_center[0]
        assert tags(relations[new_id]).get("speed_limit") == ("150" if old_id in START_LANELETS else "58")
    assert report["policy"]["minimum_start_lane_buffer_m"] >= 1.0 - 1e-6
    assert report["policy"]["minimum_end_lane_buffer_m"] >= 1.0 - 1e-6

    route = yaml.safe_load(ROUTE.read_text())
    assert route["map"] == {
        "version": 70,
        "file": "kcity_map/lanelet_map/lanelet2_map_v70.osm",
        "projector_file": "kcity_map/lanelet_map/map_projector_info_v1.yaml",
        "sha256": sha256(INPUT),
    }
    for key in ("route_a", "route_b", "fallback_route"):
        sequence = route[key]["expected_path_lanelet_ids"]
        if 201434 in sequence:
            assert sequence[sequence.index(201434) + 1] == 201457
        if 201451 in sequence:
            assert sequence[sequence.index(201451) + 1] == 201460
    assert route["poses"]["wp_201456"]["lanelet_id"] == 201456
    assert route["poses"]["wp_201457"]["lanelet_id"] == 201457

    print(f"sha256={sha256(INPUT)}")
    print(f"lanelets={len(lanelets)} road_lanelets={len(road)}")
    print("speed_distribution=" + ",".join(f"{key}:{value}" for key, value in sorted(speeds.items(), key=lambda item: float(item[0]))))
    print("split_lanelets=" + ",".join(f"{key}->{value}" for key, value in SPLITS.items()))
    print(f"minimum_start_buffer_m={report['policy']['minimum_start_lane_buffer_m']:.9f}")
    print(f"minimum_end_buffer_m={report['policy']['minimum_end_lane_buffer_m']:.9f}")
    print("missing_references=0 duplicate_ids=0 unexpected_semantic_changes=0")


if __name__ == "__main__":
    main()
