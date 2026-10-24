#!/usr/bin/env python3
"""Build v71 by compacting safe highway chains and repairing v70 VMB findings."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v70.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v71.osm"
REPORT = ROOT / "docs/kcity_v71_compaction_report.json"

# Retain the first relation and append the second relation's geometry.
FORWARD_MERGES = (
    # Absorb the four one-metre 58 km/h start buffers upstream.
    ("201086", "200101"),
    ("200145", "201434"),
    ("200355", "200152"),
    ("200461", "200142"),
    # Three parallel 150 km/h lanes around the former 200187/201443 seam.
    ("201444", "200188"),
    ("200187", "201443"),
    ("200102", "201442"),
    # Merge the complete tollgate cross-section together.  The outer pair is
    # included so route-marker drivable primitives stay longitudinally aligned.
    ("200148", "201440"),
    ("200127", "201439"),
    ("200130", "201441"),
    ("200134", "201446"),
    # Isolated straight chains with no lateral or regulatory transition.
    ("200360", "201454"),
    ("201449", "201450"),
)

# Retain the second relation and prepend the one-metre 58 km/h end buffer.
BACKWARD_MERGES = (
    ("201460", "200088"),
    ("201461", "200150"),
    ("201462", "200149"),
)

TRAFFIC_LIGHTS_TO_REVERSE = ("103861", "104150", "103828")
INTERSECTION_BUFFER_M = 0.05
DUPLICATE_NODE = "231231"
CANONICAL_NODE = "148210"


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def replace_refs(way, node_refs):
    children = list(way)
    positions = [index for index, child in enumerate(children) if child.tag == "nd"]
    insert_at = positions[0] if positions else 0
    for child in list(way.findall("nd")):
        way.remove(child)
    for offset, node_id in enumerate(node_refs):
        way.insert(insert_at + offset, etree.Element("nd", ref=str(node_id)))


def member(relation, role):
    values = [item for item in relation.findall("member") if item.get("role") == role]
    if len(values) != 1:
        raise RuntimeError(f"relation {relation.get('id')} role={role}: {len(values)}")
    return values[0]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    tree = etree.parse(str(INPUT), etree.XMLParser(remove_blank_text=False, huge_tree=True))
    root = tree.getroot()
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}

    # The v70 cut landed 8e-9 degrees from existing point 148210. Reuse the
    # existing node everywhere so VMB no longer sees a zero-length segment or
    # a steep turn at the split.
    for way in ways.values():
        node_refs = [CANONICAL_NODE if item == DUPLICATE_NODE else item for item in refs(way)]
        compact_refs = [node_refs[0]]
        for node_id in node_refs[1:]:
            if node_id != compact_refs[-1]:
                compact_refs.append(node_id)
        if len(compact_refs) < 2:
            raise RuntimeError(f"duplicate-node repair collapsed way {way.get('id')}")
        replace_refs(way, compact_refs)
    root.remove(nodes[DUPLICATE_NODE])
    del nodes[DUPLICATE_NODE]

    merged_way_pairs = {}
    candidate_removed_ways = set()
    merge_log = []

    def concatenate(first_way_id, second_way_id, retain_first):
        cache_key = (first_way_id, second_way_id, retain_first)
        if cache_key in merged_way_pairs:
            return merged_way_pairs[cache_key]
        first, second = ways[first_way_id], ways[second_way_id]
        first_refs, second_refs = refs(first), refs(second)
        if first_refs[-1] != second_refs[0]:
            raise RuntimeError(
                f"disconnected merge ways {first_way_id}->{second_way_id}: "
                f"{first_refs[-1]} != {second_refs[0]}"
            )
        combined = first_refs + second_refs[1:]
        retained_id = first_way_id if retain_first else second_way_id
        removed_id = second_way_id if retain_first else first_way_id
        replace_refs(ways[retained_id], combined)
        merged_way_pairs[cache_key] = retained_id
        candidate_removed_ways.add(removed_id)
        return retained_id

    def replace_relation_references(old_id, new_id):
        for relation in relations.values():
            for item in relation.findall("member"):
                if item.get("type") == "relation" and item.get("ref") == old_id:
                    item.set("ref", new_id)

    def merge_lanelets(first_id, second_id, retain_first):
        first, second = relations[first_id], relations[second_id]
        if tags(first) != tags(second):
            raise RuntimeError(f"semantic mismatch {first_id}->{second_id}")
        retained = first if retain_first else second
        removed = second if retain_first else first
        retained_id = retained.get("id")
        removed_id = removed.get("id")
        for role in ("left", "right", "centerline"):
            first_member = member(first, role)
            second_member = member(second, role)
            merged_way = concatenate(
                first_member.get("ref"), second_member.get("ref"), retain_first
            )
            member(retained, role).set("ref", merged_way)
        replace_relation_references(removed_id, retained_id)
        root.remove(removed)
        del relations[removed_id]
        merge_log.append(
            {"first": first_id, "second": second_id, "retained": retained_id, "removed": removed_id}
        )

    for first_id, second_id in FORWARD_MERGES:
        merge_lanelets(first_id, second_id, True)
    for first_id, second_id in BACKWARD_MERGES:
        merge_lanelets(first_id, second_id, False)

    # Delete only geometry made redundant by the explicit lanelet merges.
    referenced_ways = {
        item.get("ref")
        for relation in relations.values()
        for item in relation.findall("member")
        if item.get("type") == "way"
    }
    removed_ways = []
    for way_id in sorted(candidate_removed_ways, key=int):
        if way_id not in referenced_ways:
            root.remove(ways[way_id])
            del ways[way_id]
            removed_ways.append(way_id)

    # Current VMB flags three traffic-light linestrings in the v66-reversed
    # order. Restore the reported left-to-right order while leaving 103905,
    # which is not reported, unchanged.
    for way_id in TRAFFIC_LIGHTS_TO_REVERSE:
        replace_refs(ways[way_id], list(reversed(refs(ways[way_id]))))

    # All intersection-area misses are numerical edge mismatches: the maximum
    # v70 point distance is 3.72 cm and polygon coverage is at least 99.977%.
    # A 5 cm mitred buffer preserves the physical footprint while making every
    # associated lanelet point strictly covered for VMB.
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)
    next_node_id = max(map(int, nodes)) + 1
    new_nodes = []
    expanded_areas = {}
    for way_id, way in sorted(ways.items(), key=lambda item: int(item[0])):
        if tags(way).get("type") != "intersection_area":
            continue
        old_refs = refs(way)
        points = [
            to_utm.transform(float(nodes[node_id].get("lon")), float(nodes[node_id].get("lat")))
            for node_id in old_refs
        ]
        polygon = Polygon(points).buffer(0)
        expanded = polygon.buffer(INTERSECTION_BUFFER_M, resolution=1, join_style=2)
        if expanded.geom_type != "Polygon":
            raise RuntimeError(f"intersection area {way_id} became {expanded.geom_type}")
        elevation_values = [float(tags(nodes[node_id]).get("ele", "0")) for node_id in old_refs]
        elevation = sum(elevation_values) / len(elevation_values)
        area_refs = []
        for x, y in list(expanded.exterior.coords)[:-1]:
            node_id = str(next_node_id)
            next_node_id += 1
            lon, lat = to_wgs.transform(x, y)
            node = etree.Element(
                "node", id=node_id, visible="true", version="1",
                lat=f"{lat:.14f}", lon=f"{lon:.14f}",
            )
            node.append(etree.Element("tag", k="ele", v=f"{elevation:.9f}"))
            nodes[node_id] = node
            new_nodes.append(node)
            area_refs.append(node_id)
        area_refs.append(area_refs[0])
        replace_refs(way, area_refs)
        expanded_areas[way_id] = {
            "old_area_m2": polygon.area,
            "new_area_m2": expanded.area,
            "added_area_m2": expanded.area - polygon.area,
        }

    first_way_index = next(index for index, child in enumerate(root) if child.tag == "way")
    root[first_way_index:first_way_index] = new_nodes
    root.find("MetaInfo").set("map_version", "71")

    lanelets = {
        relation_id: relation
        for relation_id, relation in relations.items()
        if tags(relation).get("type") == "lanelet"
    }
    road = [item for item in lanelets.values() if tags(item).get("subtype") == "road"]
    speeds = Counter(tags(item).get("speed_limit") for item in road)
    expected_speeds = Counter({"58": 1126, "150": 32, "45": 34, "30": 14})
    if speeds != expected_speeds:
        raise RuntimeError(f"unexpected road speed distribution: {speeds}")
    if len(merge_log) != 16:
        raise RuntimeError(f"expected 16 merges, got {len(merge_log)}")

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    report = {
        "version": 71,
        "source_version": 70,
        "lanelet_merges": merge_log,
        "removed_lanelet_count": len(merge_log),
        "removed_way_count": len(removed_ways),
        "duplicate_node_replacement": {DUPLICATE_NODE: CANONICAL_NODE},
        "traffic_lights_reversed": list(TRAFFIC_LIGHTS_TO_REVERSE),
        "intersection_area_buffer_m": INTERSECTION_BUFFER_M,
        "expanded_intersection_areas": expanded_areas,
        "speed_distribution": dict(sorted(speeds.items(), key=lambda item: float(item[0]))),
        "intentional_vmb_warning": "32 lanelets retain 150 km/h above the 100 km/h validator threshold",
        "sha256": sha256(OUTPUT),
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
