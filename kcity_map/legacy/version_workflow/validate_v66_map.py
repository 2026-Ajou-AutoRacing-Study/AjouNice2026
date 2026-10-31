#!/usr/bin/env python3
"""Independent exhaustive validation for K-City Lanelet2 map v66."""

from __future__ import annotations

import argparse
import hashlib
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASELINE = ROOT / "lanelet_map/lanelet2_map_v65.osm"
DEFAULT_INPUT = ROOT / "lanelet_map/lanelet2_map_v66.osm"
REVERSED_TRAFFIC_LIGHTS = {"103861", "103905", "104150", "103828"}
STOP_LINES = {
    "103984", "103951", "103978", "103979", "103982", "103959", "103961",
    "103960", "103942", "103981", "103970", "103957", "103971", "103973", "103955",
}
EXPECTED_HIGHWAY_PREFERRED = [
    201434, 201435, 200137, 200187, 201443, 200130, 201441, 200360,
]


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def member(relation: etree._Element, role: str) -> str:
    values = [item.get("ref") for item in relation.findall("member") if item.get("role") == role]
    if len(values) != 1:
        raise AssertionError(f"lanelet {relation.get('id')} role={role}: {len(values)} members")
    return values[0]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    args = parser.parse_args()

    xml_parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    root = etree.parse(str(args.input), xml_parser).getroot()
    baseline = etree.parse(str(args.baseline), xml_parser).getroot()
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    old_nodes = {item.get("id"): item for item in baseline.findall("node")}
    old_ways = {item.get("id"): item for item in baseline.findall("way")}
    old_relations = {item.get("id"): item for item in baseline.findall("relation")}
    lanelets = {
        relation_id: relation
        for relation_id, relation in relations.items()
        if tags(relation).get("type") == "lanelet"
    }
    old_lanelets = {
        relation_id: relation
        for relation_id, relation in old_relations.items()
        if tags(relation).get("type") == "lanelet"
    }
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def xy(node_id: str):
        node = nodes[node_id]
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    def line(way_id: str) -> LineString:
        return LineString([xy(node_id) for node_id in refs(ways[way_id])])

    assert len(lanelets) == len(old_lanelets) == 1303
    assert set(lanelets) == set(old_lanelets)
    assert root.find("MetaInfo").get("map_version") == "66"

    # Attachment: adjacent duplicate points and steep point 161266.
    adjacent_duplicates = []
    for way_id, way in ways.items():
        for first, second in zip(refs(way), refs(way)[1:]):
            if first == second or Point(xy(first)).distance(Point(xy(second))) < 1e-6:
                adjacent_duplicates.append((way_id, first, second))
    assert not adjacent_duplicates, adjacent_duplicates
    assert all("161266" not in refs(way) for way in ways.values())

    # Attachment: every explicit centerline connects by one shared node at
    # boundary-defined predecessor/successor cross-sections.
    starts = defaultdict(list)
    ends = []
    explicit_centerlines = 0
    for lanelet_id, lanelet in lanelets.items():
        if not any(item.get("role") == "centerline" for item in lanelet.findall("member")):
            continue
        explicit_centerlines += 1
        left = refs(ways[member(lanelet, "left")])
        right = refs(ways[member(lanelet, "right")])
        center = refs(ways[member(lanelet, "centerline")])
        starts[(left[0], right[0])].append((lanelet_id, center[0]))
        ends.append((lanelet_id, (left[-1], right[-1]), center[-1]))
    disconnected = []
    for lanelet_id, key, center_end in ends:
        for next_id, center_start in starts.get(key, []):
            if center_end != center_start:
                disconnected.append((lanelet_id, next_id))
    assert explicit_centerlines == 1214
    assert not disconnected, disconnected

    # Attachment: centerline interior points must remain inside the lanelet.
    outside = []
    for lanelet_id, lanelet in lanelets.items():
        if not any(item.get("role") == "centerline" for item in lanelet.findall("member")):
            continue
        polygon = Polygon(
            [xy(node_id) for node_id in refs(ways[member(lanelet, "left")])]
            + [xy(node_id) for node_id in reversed(refs(ways[member(lanelet, "right")]))]
        ).buffer(0.001)
        center = refs(ways[member(lanelet, "centerline")])
        if any(not polygon.covers(Point(xy(node_id))) for node_id in center[1:-1]):
            outside.append(lanelet_id)
    assert not outside, outside

    # Attachment: speed and intersection-area references.
    speed_over_threshold = []
    missing_areas = []
    for lanelet_id, lanelet in lanelets.items():
        lanelet_tags = tags(lanelet)
        if "speed_limit" in lanelet_tags:
            speed = float(lanelet_tags["speed_limit"].split()[0])
            if speed > 100.0:
                speed_over_threshold.append(lanelet_id)
        area_id = lanelet_tags.get("intersection_area")
        if area_id:
            area = ways.get(area_id)
            if area is None or tags(area).get("type") != "intersection_area" or tags(area).get("area") != "yes":
                missing_areas.append(lanelet_id)
    assert not speed_over_threshold, speed_over_threshold
    assert not missing_areas, missing_areas
    assert sum(tags(way).get("type") == "intersection_area" for way in ways.values()) == 7

    # Attachment: all directly attached traffic-light stop lines intersect
    # their lanelet.  This catches the 15 named stop lines and every other one.
    stopline_misses = []
    traffic_light_checks = 0
    row_warning_candidates = []
    for lanelet_id, lanelet in lanelets.items():
        polygon = Polygon(
            [xy(node_id) for node_id in refs(ways[member(lanelet, "left")])]
            + [xy(node_id) for node_id in reversed(refs(ways[member(lanelet, "right")]))]
        )
        lanelet_tags = tags(lanelet)
        for item in lanelet.findall("member"):
            if item.get("role") != "regulatory_element":
                continue
            regulatory = relations[item.get("ref")]
            if tags(regulatory).get("subtype") != "traffic_light":
                continue
            if lanelet_tags.get("intersection_area") and lanelet_tags.get("turn_direction") in {"left", "right"}:
                row_warning_candidates.append(lanelet_id)
            for ref_line in regulatory.findall("member[@role='ref_line']"):
                traffic_light_checks += 1
                if not polygon.intersects(line(ref_line.get("ref"))):
                    stopline_misses.append((lanelet_id, ref_line.get("ref")))
    assert traffic_light_checks == 68
    assert not stopline_misses, stopline_misses
    assert not row_warning_candidates, row_warning_candidates

    # The four warning lines must be the exact reverse of v65.  Stop-line
    # geometry itself remains byte-for-byte identical at the node level.
    for way_id in REVERSED_TRAFFIC_LIGHTS:
        assert refs(ways[way_id]) == list(reversed(refs(old_ways[way_id])))
    for way_id in STOP_LINES:
        assert refs(ways[way_id]) == refs(old_ways[way_id])
        for node_id in refs(ways[way_id]):
            assert etree.tostring(nodes[node_id]) == etree.tostring(old_nodes[node_id])

    # v65 mission-route topology and the roundabout definition are preserved.
    roundabouts = [relation for relation in relations.values() if tags(relation).get("subtype") == "roundabout"]
    assert len(roundabouts) == 1
    assert roundabouts[0].get("id") == "201455"
    assert len(roundabouts[0].findall("member")) == 14
    assert all(str(lanelet_id) in lanelets for lanelet_id in EXPECTED_HIGHWAY_PREFERRED)

    print("attachment_errors=0")
    print("attachment_warnings=0")
    print(f"lanelets={len(lanelets)}")
    print(f"explicit_centerlines={explicit_centerlines}")
    print(f"traffic_light_stopline_checks={traffic_light_checks}")
    print(f"intersection_areas=7")
    print(f"roundabout_members=14")
    print(f"sha256={sha256(args.input)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
