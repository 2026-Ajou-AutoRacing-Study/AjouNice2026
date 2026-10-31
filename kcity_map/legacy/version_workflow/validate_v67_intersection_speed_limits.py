#!/usr/bin/env python3
"""Validate K-City v67 intersection coverage, semantics, and 30 km/h limits."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from typing import Dict, List

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Polygon

from build_v67_intersection_speed_limits import INTERSECTION_AREAS, MISSING_MOVEMENTS


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASELINE = ROOT / "lanelet_map/lanelet2_map_v66.osm"
DEFAULT_INPUT = ROOT / "lanelet_map/lanelet2_map_v67.osm"


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    args = parser.parse_args()

    xml_parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    root = etree.parse(str(args.input), xml_parser).getroot()
    old_root = etree.parse(str(args.baseline), xml_parser).getroot()
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    old_nodes = {item.get("id"): item for item in old_root.findall("node")}
    old_ways = {item.get("id"): item for item in old_root.findall("way")}
    old_relations = {item.get("id"): item for item in old_root.findall("relation")}
    lanelets = {item_id: item for item_id, item in relations.items() if tags(item).get("type") == "lanelet"}
    old_lanelets = {
        item_id: item for item_id, item in old_relations.items() if tags(item).get("type") == "lanelet"
    }
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def xy(node_id: str):
        node = nodes[node_id]
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    def polygon_for_lanelet(lanelet: etree._Element) -> Polygon:
        return Polygon(
            [xy(node_id) for node_id in refs(ways[member(lanelet, "left")])]
            + [xy(node_id) for node_id in reversed(refs(ways[member(lanelet, "right")]))]
        ).buffer(0)

    assert root.find("MetaInfo").get("map_version") == "67"
    assert set(nodes) == set(old_nodes)
    assert set(ways) == set(old_ways)
    assert set(relations) == set(old_relations)
    assert len(lanelets) == len(old_lanelets) == 1303

    # v67 must not alter any point, line, polygon, or relation membership.
    assert all(etree.tostring(nodes[item_id]) == etree.tostring(old_nodes[item_id]) for item_id in nodes)
    assert all(etree.tostring(ways[item_id]) == etree.tostring(old_ways[item_id]) for item_id in ways)
    for relation_id, relation in relations.items():
        current_members = [(m.get("type"), m.get("ref"), m.get("role")) for m in relation.findall("member")]
        old_members = [
            (m.get("type"), m.get("ref"), m.get("role")) for m in old_relations[relation_id].findall("member")
        ]
        assert current_members == old_members, relation_id

    road_inside = {}
    pedestrian_inside = {}
    semantic_members = {}
    for area_id, specification in INTERSECTION_AREAS.items():
        area_way = ways[area_id]
        area_tags = tags(area_way)
        assert area_tags.get("type") == "intersection_area"
        assert area_tags.get("area") == "yes"
        area_refs = refs(area_way)
        assert len(area_refs) >= 4 and area_refs[0] == area_refs[-1]
        area = Polygon([xy(node_id) for node_id in area_refs]).buffer(0)
        assert area.is_valid and area.area > 200.0

        road_ids = []
        pedestrian_ids = []
        for lanelet_id, lanelet in lanelets.items():
            lanelet_polygon = polygon_for_lanelet(lanelet)
            if lanelet_polygon.area <= 0.0:
                continue
            coverage = lanelet_polygon.intersection(area).area / lanelet_polygon.area
            if coverage < 0.95:
                continue
            lanelet_tags = tags(lanelet)
            if lanelet_tags.get("subtype") == "road" and lanelet_tags.get("participant:vehicle") == "yes":
                road_ids.append(lanelet_id)
            elif lanelet_tags.get("subtype") == "crosswalk":
                pedestrian_ids.append(lanelet_id)

        linked = sorted(
            [lanelet_id for lanelet_id, lanelet in lanelets.items() if tags(lanelet).get("intersection_area") == area_id],
            key=int,
        )
        road_ids = sorted(road_ids, key=int)
        assert linked == road_ids, (area_id, linked, road_ids)
        assert specification["route_lanelet"] in linked
        assert all(tags(lanelets[lanelet_id]).get("turn_direction") in {"straight", "left", "right"} for lanelet_id in linked)
        assert all(tags(lanelets[lanelet_id]).get("speed_limit") == "30" for lanelet_id in linked)
        assert all("speed_limit" not in tags(lanelets[lanelet_id]) for lanelet_id in pedestrian_ids)
        road_inside[area_id] = road_ids
        pedestrian_inside[area_id] = sorted(pedestrian_ids, key=int)
        semantic_members[area_id] = linked

    speed_members = sorted(
        [lanelet_id for lanelet_id, lanelet in lanelets.items() if tags(lanelet).get("intersection_area") in INTERSECTION_AREAS],
        key=int,
    )
    assert len(speed_members) == 60
    assert sum(len(values) for values in road_inside.values()) == 60

    # Only the intended semantic tags and speed values may differ from v66.
    changed_lanelets = []
    for lanelet_id in lanelets:
        current = tags(lanelets[lanelet_id])
        old = tags(old_lanelets[lanelet_id])
        if current == old:
            continue
        changed_lanelets.append(lanelet_id)
        expected = dict(old)
        if lanelet_id in MISSING_MOVEMENTS:
            expected["turn_direction"] = MISSING_MOVEMENTS[lanelet_id]
            expected["intersection_area"] = "104158"
        expected["speed_limit"] = "30"
        assert current == expected, (lanelet_id, old, current)
    assert sorted(changed_lanelets, key=int) == speed_members

    print(f"sha256={sha256(args.input)}")
    print(f"lanelets={len(lanelets)}")
    print(f"intersection_polygons={len(INTERSECTION_AREAS)}")
    print(f"vehicle_lanelets_at_30_kph={len(speed_members)}")
    print(f"completed_missing_movements={','.join(sorted(MISSING_MOVEMENTS, key=int))}")
    for area_id in INTERSECTION_AREAS:
        print(f"area_{area_id}_road={','.join(road_inside[area_id])}")
        print(f"area_{area_id}_crosswalk_excluded={','.join(pedestrian_inside[area_id]) or '-'}")
    print("geometry_changes=0")
    print("unexpected_semantic_changes=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
