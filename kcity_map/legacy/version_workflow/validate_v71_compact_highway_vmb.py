#!/usr/bin/env python3
"""Exhaustively validate v71 compaction, topology, and reported VMB findings."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon
import yaml

from build_v71_compact_highway_vmb import (
    BACKWARD_MERGES,
    CANONICAL_NODE,
    DUPLICATE_NODE,
    FORWARD_MERGES,
    INTERSECTION_BUFFER_M,
    TRAFFIC_LIGHTS_TO_REVERSE,
    tags,
)


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v71.osm"
BASELINE = ROOT / "lanelet_map/lanelet2_map_v70.osm"
REPORT = ROOT / "docs/kcity_v71_compaction_report.json"
ROUTE = ROOT.parent / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v71_competition_route.yaml"
REMOVED_TO_RETAINED = {second: first for first, second in FORWARD_MERGES}
REMOVED_TO_RETAINED.update({first: second for first, second in BACKWARD_MERGES})


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def members(relation):
    return [(item.get("type"), item.get("ref"), item.get("role")) for item in relation.findall("member")]


def role(relation, name):
    values = [item.get("ref") for item in relation.findall("member") if item.get("role") == name]
    if len(values) != 1:
        raise AssertionError((relation.get("id"), name, values))
    return values[0]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = etree.XMLParser(huge_tree=True)
    root = etree.parse(str(INPUT), parser).getroot()
    old_root = etree.parse(str(BASELINE), parser).getroot()
    report = json.loads(REPORT.read_text())
    assert root.find("MetaInfo").get("map_version") == "71"
    assert sha256(INPUT) == report["sha256"]

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    old_nodes = {item.get("id"): item for item in old_root.findall("node")}
    old_ways = {item.get("id"): item for item in old_root.findall("way")}
    old_relations = {item.get("id"): item for item in old_root.findall("relation")}
    assert len(nodes) == len(set(nodes))
    assert len(ways) == len(set(ways))
    assert len(relations) == len(set(relations))
    assert set(old_relations) - set(relations) == set(REMOVED_TO_RETAINED)
    assert not (set(relations) - set(old_relations))
    assert DUPLICATE_NODE not in nodes and CANONICAL_NODE in nodes

    missing_refs = []
    for way_id, way in ways.items():
        missing_refs.extend((way_id, node_id) for node_id in refs(way) if node_id not in nodes)
    for relation_id, relation in relations.items():
        for item in relation.findall("member"):
            layer = {"node": nodes, "way": ways, "relation": relations}[item.get("type")]
            if item.get("ref") not in layer:
                missing_refs.append((relation_id, item.get("ref")))
    assert not missing_refs, missing_refs

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    xy = {
        key: to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        for key, node in nodes.items()
    }

    # VMB error: no adjacent node IDs or coordinates may coincide.
    adjacent_duplicates = []
    for way_id, way in ways.items():
        node_refs = refs(way)
        for first, second in zip(node_refs, node_refs[1:]):
            distance = math.dist(xy[first], xy[second])
            if first == second or distance < 1e-6:
                adjacent_duplicates.append((way_id, first, second, distance))
    assert not adjacent_duplicates, adjacent_duplicates

    # VMB steep point: after canonicalization, every interior occurrence of
    # point 148210 must form an ordinary road angle rather than a zero segment.
    canonical_angles = []
    for way_id, way in ways.items():
        node_refs = refs(way)
        for index in range(1, len(node_refs) - 1):
            if node_refs[index] != CANONICAL_NODE:
                continue
            a, b, c = xy[node_refs[index - 1]], xy[node_refs[index]], xy[node_refs[index + 1]]
            u, v = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
            cosine = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))))
            canonical_angles.append((way_id, math.degrees(math.acos(cosine))))
    if not canonical_angles:
        ending = [refs(way)[-2] for way in ways.values() if refs(way)[-1] == CANONICAL_NODE]
        starting = [refs(way)[1] for way in ways.values() if refs(way)[0] == CANONICAL_NODE]
        assert ending and starting
        for previous in ending:
            for following in starting:
                a, b, c = xy[previous], xy[CANONICAL_NODE], xy[following]
                u, v = (a[0] - b[0], a[1] - b[1]), (c[0] - b[0], c[1] - b[1])
                cosine = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (math.hypot(*u) * math.hypot(*v))))
                canonical_angles.append((f"{previous}->{following}", math.degrees(math.acos(cosine))))
    assert min(value for _, value in canonical_angles) > 90.0, canonical_angles

    lanelets = {key: value for key, value in relations.items() if tags(value).get("type") == "lanelet"}
    road = {key: value for key, value in lanelets.items() if tags(value).get("subtype") == "road"}
    assert len(lanelets) == 1295 and len(road) == 1206
    speeds = Counter(tags(value).get("speed_limit") for value in road.values())
    assert speeds == Counter({"58": 1126, "150": 32, "45": 34, "30": 14}), speeds

    # Every explicit centerline stays inside its compacted lanelet polygon.
    outside_centerlines = []
    invalid_polygons = []
    for lanelet_id, lanelet in road.items():
        polygon = Polygon(
            [xy[item] for item in refs(ways[role(lanelet, "left")])]
            + [xy[item] for item in reversed(refs(ways[role(lanelet, "right")]))]
        ).buffer(0)
        if not polygon.is_valid or polygon.area <= 0.0:
            invalid_polygons.append(lanelet_id)
            continue
        center = refs(ways[role(lanelet, "centerline")])
        if any(not polygon.buffer(0.001).covers(Point(xy[item])) for item in center[1:-1]):
            outside_centerlines.append(lanelet_id)
    assert not invalid_polygons, invalid_polygons
    assert not outside_centerlines, outside_centerlines

    traffic_light_stopline_misses = []
    traffic_light_stopline_checks = 0
    for lanelet_id, lanelet in road.items():
        lanelet_polygon = Polygon(
            [xy[item] for item in refs(ways[role(lanelet, "left")])]
            + [xy[item] for item in reversed(refs(ways[role(lanelet, "right")]))]
        ).buffer(0)
        for item in lanelet.findall("member"):
            if item.get("role") != "regulatory_element":
                continue
            regulatory = relations[item.get("ref")]
            if tags(regulatory).get("subtype") != "traffic_light":
                continue
            for ref_line in regulatory.findall("member[@role='ref_line']"):
                traffic_light_stopline_checks += 1
                line = LineString([xy[node_id] for node_id in refs(ways[ref_line.get("ref")])])
                if not lanelet_polygon.intersects(line):
                    traffic_light_stopline_misses.append((lanelet_id, ref_line.get("ref")))
    assert traffic_light_stopline_checks == 68
    assert not traffic_light_stopline_misses, traffic_light_stopline_misses

    # VMB intersection-area warnings: all edge and interior points belonging
    # to an associated lanelet must be covered by its polygon.
    area_ways = {key: value for key, value in ways.items() if tags(value).get("type") == "intersection_area"}
    assert len(area_ways) == 7
    area_polygons = {
        key: Polygon([xy[item] for item in refs(value)]).buffer(0)
        for key, value in area_ways.items()
    }
    area_misses = []
    area_members = 0
    for lanelet_id, lanelet in road.items():
        area_id = tags(lanelet).get("intersection_area")
        if not area_id:
            continue
        area_members += 1
        point_ids = []
        for name in ("left", "right", "centerline"):
            point_ids.extend(refs(ways[role(lanelet, name)]))
        for node_id in point_ids:
            if not area_polygons[area_id].covers(Point(xy[node_id])):
                area_misses.append((lanelet_id, area_id, node_id))
    assert area_members == 60
    assert not area_misses, area_misses

    # VMB traffic-light ordering findings: reverse the three reported lines,
    # while keeping the unreported 103905 order unchanged.
    for way_id in TRAFFIC_LIGHTS_TO_REVERSE:
        assert refs(ways[way_id]) == list(reversed(refs(old_ways[way_id])))
    assert refs(ways["103905"]) == refs(old_ways["103905"])
    assert members(relations["201455"]) == members(old_relations["201455"])

    # Validate exact merge geometry and tag preservation.
    for removed_id, retained_id in REMOVED_TO_RETAINED.items():
        assert tags(relations[retained_id]) == tags(old_relations[retained_id])
        assert tags(old_relations[removed_id]) == tags(old_relations[retained_id])
        merged_center = refs(ways[role(relations[retained_id], "centerline")])
        first_id, second_id = (
            (retained_id, removed_id) if (retained_id, removed_id) in FORWARD_MERGES
            else (removed_id, retained_id)
        )
        old_first = refs(old_ways[role(old_relations[first_id], "centerline")])
        old_second = refs(old_ways[role(old_relations[second_id], "centerline")])
        old_first = [CANONICAL_NODE if item == DUPLICATE_NODE else item for item in old_first]
        old_second = [CANONICAL_NODE if item == DUPLICATE_NODE else item for item in old_second]
        expected = old_first + old_second[1:]
        expected = [item for index, item in enumerate(expected) if index == 0 or item != expected[index - 1]]
        assert merged_center == expected, (removed_id, retained_id)

    old_fast = {key for key, value in old_relations.items() if tags(value).get("speed_limit") == "150"}
    expected_fast = old_fast - set(REMOVED_TO_RETAINED)
    actual_fast = {key for key, value in road.items() if tags(value).get("speed_limit") == "150"}
    assert actual_fast == expected_fast

    route = yaml.safe_load(ROUTE.read_text())
    assert route["map"] == {
        "version": 71,
        "file": "kcity_map/lanelet_map/lanelet2_map_v71.osm",
        "projector_file": "kcity_map/lanelet_map/map_projector_info_v1.yaml",
        "sha256": sha256(INPUT),
    }
    for key in ("route_a", "route_b", "fallback_route"):
        sequence = route[key]["expected_path_lanelet_ids"]
        assert not (set(sequence) & set(REMOVED_TO_RETAINED)), (key, set(sequence) & set(REMOVED_TO_RETAINED))
        assert all(str(item) in lanelets for item in sequence)
    assert route["poses"]["wp_200187_merged"]["lanelet_id"] == 200187
    assert route["poses"]["fallback_cutoff_200130"]["lanelet_id"] == 200130

    print(f"sha256={sha256(INPUT)}")
    print(f"lanelets={len(lanelets)} road_lanelets={len(road)} removed_lanelets={len(REMOVED_TO_RETAINED)}")
    print("speed_distribution=" + ",".join(f"{key}:{value}" for key, value in sorted(speeds.items(), key=lambda item: float(item[0]))))
    print(f"intentional_speed_warning_lanelets={len(actual_fast)}")
    print(f"intersection_area_members={area_members} misses=0 buffer_m={INTERSECTION_BUFFER_M}")
    print("adjacent_duplicate_points=0 steep_point_148210=0 traffic_light_order_errors=0")
    print(f"traffic_light_stopline_checks={traffic_light_stopline_checks} misses=0 roundabout_unchanged=1")
    print("missing_references=0 invalid_lanelet_polygons=0 outside_centerlines=0")


if __name__ == "__main__":
    main()
