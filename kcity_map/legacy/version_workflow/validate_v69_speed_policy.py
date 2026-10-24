#!/usr/bin/env python3
"""Validate the v69 competition speed policy against v68 and MORAI MGeo."""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path

from lxml import etree

from build_v69_speed_policy import (
    HIGHWAY_EXCEPTION_LANELETS,
    INTERSECTION_SPEEDS,
    MGEO_BOUNDARY,
)


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = ROOT / "lanelet_map/lanelet2_map_v69.osm"
DEFAULT_BASELINE = ROOT / "lanelet_map/lanelet2_map_v68.osm"
DEFAULT_MGEO = Path("/home/jihun/Autoware/kcity_map/mgeo_map/link_set.json")
UNCHANGED_INTERSECTIONS = {"104016": "30", "104158": "30"}
EXPECTED_DEMOTED = {"200088", "200150", "201086"}


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--mgeo", type=Path, default=DEFAULT_MGEO)
    args = parser.parse_args()

    root = etree.parse(str(args.input)).getroot()
    old_root = etree.parse(str(args.baseline)).getroot()
    assert root.find("MetaInfo").get("map_version") == "69"
    assert old_root.find("MetaInfo").get("map_version") == "68"

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    old_nodes = {item.get("id"): item for item in old_root.findall("node")}
    old_ways = {item.get("id"): item for item in old_root.findall("way")}
    old_relations = {item.get("id"): item for item in old_root.findall("relation")}
    lanelets = {key: value for key, value in relations.items() if tags(value).get("type") == "lanelet"}

    assert len(lanelets) == 1303
    assert set(nodes) == set(old_nodes) and set(ways) == set(old_ways) and set(relations) == set(old_relations)
    assert all(etree.tostring(nodes[key]) == etree.tostring(old_nodes[key]) for key in nodes)
    assert all(etree.tostring(ways[key]) == etree.tostring(old_ways[key]) for key in ways)

    allowed = set(EXPECTED_DEMOTED)
    area_members = {}
    for area_id, speed in INTERSECTION_SPEEDS.items():
        members = sorted(
            [key for key, value in lanelets.items() if tags(value).get("intersection_area") == area_id],
            key=int,
        )
        assert members and all(tags(lanelets[key]).get("speed_limit") == speed for key in members)
        area_members[area_id] = members
        allowed.update(members)
    for area_id, speed in UNCHANGED_INTERSECTIONS.items():
        members = [key for key, value in lanelets.items() if tags(value).get("intersection_area") == area_id]
        assert members and all(tags(lanelets[key]).get("speed_limit") == speed for key in members)

    for relation_id, relation in relations.items():
        old_relation = old_relations[relation_id]
        assert [(m.get("type"), m.get("ref"), m.get("role")) for m in relation.findall("member")] == [
            (m.get("type"), m.get("ref"), m.get("role")) for m in old_relation.findall("member")
        ]
        current_tags = tags(relation)
        previous_tags = tags(old_relation)
        if relation_id not in allowed:
            assert current_tags == previous_tags, relation_id
        else:
            normalized = dict(current_tags)
            normalized["speed_limit"] = previous_tags["speed_limit"]
            assert normalized == previous_tags, relation_id

    road = {key: value for key, value in lanelets.items() if tags(value).get("subtype") == "road"}
    speeds = Counter(tags(value).get("speed_limit") for value in road.values())
    assert speeds == Counter({"58": 1125, "100": 41, "45": 34, "30": 14}), speeds
    speed_100 = {key for key, value in road.items() if tags(value).get("speed_limit") == "100"}
    assert speed_100 == HIGHWAY_EXCEPTION_LANELETS
    assert all(float(tags(value).get("speed_limit", "0")) <= 58 for key, value in road.items() if key not in HIGHWAY_EXCEPTION_LANELETS)
    assert {key for key in EXPECTED_DEMOTED if tags(lanelets[key]).get("speed_limit") == "58"} == EXPECTED_DEMOTED

    links = {item["idx"]: item for item in json.loads(args.mgeo.read_text())}
    start = links[MGEO_BOUNDARY["start_link"]]["points"][0]
    end = links[MGEO_BOUNDARY["end_link"]]["points"][-1]
    assert start == MGEO_BOUNDARY["start_xyz"]
    assert end == MGEO_BOUNDARY["end_xyz"]

    print(f"sha256={sha256(args.input)}")
    print(f"lanelets={len(lanelets)} road_lanelets={len(road)}")
    print("speed_distribution=" + ",".join(f"{speed}:{count}" for speed, count in sorted(speeds.items(), key=lambda item: float(item[0]))))
    print(f"highway_exception_lanelets={len(HIGHWAY_EXCEPTION_LANELETS)}")
    print("demoted_outside_exception=" + ",".join(sorted(EXPECTED_DEMOTED, key=int)))
    for area_id, members in area_members.items():
        print(f"polygon_{area_id}={INTERSECTION_SPEEDS[area_id]}kph:{len(members)}")
    print(f"mgeo_start={MGEO_BOUNDARY['start_link']}:{start}")
    print(f"mgeo_end={MGEO_BOUNDARY['end_link']}:{end}")
    print("geometry_changes=0 unexpected_semantic_changes=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
