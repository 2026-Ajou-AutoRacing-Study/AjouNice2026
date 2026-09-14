#!/usr/bin/env python3
"""Build K-City v66 by repairing every issue reported for v65.

The builder keeps the v65 road, stop-line, intersection, roundabout, and route
geometry.  It only repairs invalid regulatory attachments, centerlines, map
validator metadata, traffic-light point order, and speed values above the
configured 100 km/h validation threshold.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = ROOT / "lanelet_map/lanelet2_map_v65.osm"
DEFAULT_BASELINE = ROOT / "lanelet_map/lanelet2_map_v64.osm"
DEFAULT_OUTPUT = ROOT / "lanelet_map/lanelet2_map_v66.osm"

OUTSIDE_CENTERLINES = {
    "200172", "200259", "200269", "200402", "200426", "200532",
    "200668", "200761", "200822", "201019", "201175", "201176",
    "201177", "201179", "201180", "201181", "201182", "201183",
    "201184", "201185", "201443", "201451",
}
REVERSED_TRAFFIC_LIGHTS = {"103861", "103905", "104150", "103828"}
STEEP_DUPLICATE_POINT = "161266"

Point3 = Tuple[float, float, float]


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    element.append(etree.Element("tag", k=key, v=value))


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    children = list(way)
    positions = [index for index, child in enumerate(children) if child.tag == "nd"]
    insert_at = positions[0] if positions else 0
    for child in list(way.findall("nd")):
        way.remove(child)
    for offset, ref in enumerate(refs):
        way.insert(insert_at + offset, etree.Element("nd", ref=str(ref)))


def relation_member(relation: etree._Element, role: str) -> etree._Element:
    members = [member for member in relation.findall("member") if member.get("role") == role]
    if len(members) != 1:
        raise RuntimeError(
            f"lanelet {relation.get('id')} must have exactly one {role}, got {len(members)}"
        )
    return members[0]


def interpolate(points: Sequence[Point3], count: int) -> List[Point3]:
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(
            cumulative[-1] + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    if cumulative[-1] <= 1e-9:
        return [points[0]] * count
    output: List[Point3] = []
    segment = 0
    for index in range(count):
        target = cumulative[-1] * index / max(1, count - 1)
        while segment + 1 < len(cumulative) - 1 and cumulative[segment + 1] < target:
            segment += 1
        span = cumulative[segment + 1] - cumulative[segment]
        ratio = 0.0 if span <= 1e-9 else (target - cumulative[segment]) / span
        first, second = points[segment], points[segment + 1]
        output.append(tuple(first[axis] + ratio * (second[axis] - first[axis]) for axis in range(3)))
    return output


def baseline_regulatory_refs(root: etree._Element) -> Dict[str, set[str]]:
    output: Dict[str, set[str]] = {}
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet":
            continue
        output[relation.get("id")] = {
            member.get("ref")
            for member in relation.findall("member")
            if member.get("role") == "regulatory_element"
        }
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    xml_parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(args.input), xml_parser)
    root = tree.getroot()
    baseline = etree.parse(str(args.baseline), xml_parser).getroot()

    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    relations = {relation.get("id"): relation for relation in root.findall("relation")}
    lanelets = {
        relation_id: relation
        for relation_id, relation in relations.items()
        if tags(relation).get("type") == "lanelet"
    }
    baseline_refs = baseline_regulatory_refs(baseline)
    subtype = {relation_id: tags(relation).get("subtype") for relation_id, relation in relations.items()}

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", "66")

    removed_traffic_light_refs = 0
    for lanelet_id, lanelet in lanelets.items():
        allowed = baseline_refs.get(lanelet_id, set())
        for member in list(lanelet.findall("member")):
            if (
                member.get("role") == "regulatory_element"
                and subtype.get(member.get("ref")) == "traffic_light"
                and member.get("ref") not in allowed
            ):
                lanelet.remove(member)
                removed_traffic_light_refs += 1

    intersection_areas = 0
    for way in ways.values():
        if tags(way).get("type") == "intersection_area":
            set_tag(way, "area", "yes")
            intersection_areas += 1

    speed_caps = 0
    for lanelet in lanelets.values():
        for tag in lanelet.findall("tag"):
            if tag.get("k") != "speed_limit":
                continue
            raw = tag.get("v", "")
            try:
                speed = float(raw.split()[0])
            except (ValueError, IndexError):
                continue
            if speed > 100.0:
                tag.set("v", "100")
                speed_caps += 1

    reversed_lights = 0
    for way_id in REVERSED_TRAFFIC_LIGHTS:
        way = ways[way_id]
        replace_way_refs(way, list(reversed(way_refs(way))))
        reversed_lights += 1

    # Remove the duplicate/steep centerline vertex in lanelet 200309.  The
    # second duplicate pair belongs to an outside-centerline lanelet and is
    # replaced completely below.
    center_200309 = ways[relation_member(lanelets["200309"], "centerline").get("ref")]
    old_refs = way_refs(center_200309)
    new_refs = [ref for ref in old_refs if ref != STEEP_DUPLICATE_POINT]
    if len(old_refs) - len(new_refs) != 1:
        raise RuntimeError("expected exactly one steep duplicate point in lanelet 200309")
    replace_way_refs(center_200309, new_refs)

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)
    next_node_id = max(int(node_id) for node_id in nodes) + 1
    first_way_index = next(index for index, child in enumerate(root) if child.tag == "way")
    added_nodes: List[etree._Element] = []

    def point(node_id: str) -> Point3:
        node = nodes[node_id]
        x, y = to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        elevation = float(tags(node).get("ele", "0"))
        return x, y, elevation

    def new_node(value: Point3) -> str:
        nonlocal next_node_id
        node_id = str(next_node_id)
        next_node_id += 1
        lon, lat = to_wgs.transform(value[0], value[1])
        node = etree.Element("node", id=node_id, visible="true", version="1")
        node.set("lat", f"{lat:.11f}")
        node.set("lon", f"{lon:.11f}")
        node.append(etree.Element("tag", k="ele", v=f"{value[2]:.11f}"))
        nodes[node_id] = node
        added_nodes.append(node)
        return node_id

    regenerated_centerlines = 0
    for lanelet_id in sorted(OUTSIDE_CENTERLINES, key=int):
        lanelet = lanelets[lanelet_id]
        left = [point(ref) for ref in way_refs(ways[relation_member(lanelet, "left").get("ref")])]
        right = [point(ref) for ref in way_refs(ways[relation_member(lanelet, "right").get("ref")])]
        center_way = ways[relation_member(lanelet, "centerline").get("ref")]
        count = max(2, len(way_refs(center_way)))
        left_i, right_i = interpolate(left, count), interpolate(right, count)
        midpoint = [
            tuple((left_i[index][axis] + right_i[index][axis]) * 0.5 for axis in range(3))
            for index in range(count)
        ]
        replace_way_refs(center_way, [new_node(value) for value in midpoint])
        regenerated_centerlines += 1

    # Lanelet2's centerline connectivity validator requires adjacent lanelets
    # to share the same node ID at a common boundary cross-section.  v61 made
    # geometrically coincident but independent nodes.  Canonicalize every
    # centerline endpoint from its left/right boundary endpoint pair.
    endpoint_nodes: Dict[Tuple[str, str], str] = {}
    canonicalized_endpoints = 0
    for lanelet_id in sorted(lanelets, key=int):
        lanelet = lanelets[lanelet_id]
        if not any(member.get("role") == "centerline" for member in lanelet.findall("member")):
            continue
        left_refs = way_refs(ways[relation_member(lanelet, "left").get("ref")])
        right_refs = way_refs(ways[relation_member(lanelet, "right").get("ref")])
        center_way = ways[relation_member(lanelet, "centerline").get("ref")]
        center_refs = way_refs(center_way)
        if len(center_refs) < 2:
            raise RuntimeError(f"lanelet {lanelet_id} has a short centerline")
        for position, key in ((0, (left_refs[0], right_refs[0])), (-1, (left_refs[-1], right_refs[-1]))):
            if key not in endpoint_nodes:
                # Reuse a current centerline endpoint.  This avoids adding a
                # second coincident point while still making consecutive
                # lanelets share the exact same OSM node ID.
                endpoint_nodes[key] = center_refs[position]
            if center_refs[position] != endpoint_nodes[key]:
                center_refs[position] = endpoint_nodes[key]
                canonicalized_endpoints += 1
        replace_way_refs(center_way, center_refs)

    root[first_way_index:first_way_index] = added_nodes

    referenced_nodes = {nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")}
    children = list(root)
    retained = [
        child
        for child in children
        if child.tag != "node" or child.get("id") in referenced_nodes
    ]
    removed_orphan_nodes = len(children) - len(retained)
    root[:] = retained

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    print(f"removed_traffic_light_refs={removed_traffic_light_refs}")
    print(f"intersection_areas={intersection_areas}")
    print(f"speed_caps={speed_caps}")
    print(f"reversed_traffic_lights={reversed_lights}")
    print(f"regenerated_centerlines={regenerated_centerlines}")
    print(f"canonicalized_centerline_endpoints={canonicalized_endpoints}")
    print(f"added_nodes={len(added_nodes)}")
    print(f"removed_orphan_nodes={removed_orphan_nodes}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
