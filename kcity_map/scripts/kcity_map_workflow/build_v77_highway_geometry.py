#!/usr/bin/env python3
"""Build v77 with clean quadrilateral geometry on the straight highway.

The v76 lane-change reconstruction resampled and fitted lane boundaries.  That
made straight lanelets visually wavy and left several one-sided dashed ways in
overlapping merge corridors.  v77 preserves every longitudinal endpoint and
shared boundary identity, while reducing each straight boundary to its two
cross-section endpoints.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v76.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v77.osm"
REPORT = ROOT / "docs/kcity_v77_highway_geometry_build.json"

# Straight and merging highway lanelets from the conservative speed-release
# boundary through the end of the bottleneck. Curved exit lanelets downstream
# of y=328 m are intentionally outside this geometry-only rewrite.
TARGET_LANELETS = {
    201463,
    201456, 201457, 201458, 201459,
    201437, 201435, 201436, 201438, 200099,
    201464,
    201465, 201466, 201467, 201468,
    200128, 200137, 200138, 200186,
    200102, 200187, 201444, 200141, 201445,
    201469, 201470, 201471,
    200148, 200127, 200130, 200134,
    201472, 201473, 201474, 201475,
    201440, 201439, 201441, 201446,
}


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def set_tag(element, key, value):
    tag = next((tag for tag in element.findall("tag") if tag.get("k") == key), None)
    if tag is None:
        tag = etree.SubElement(element, "tag", k=key)
    tag.set("v", value)


def remove_tag(element, key):
    for tag in list(element.findall("tag")):
        if tag.get("k") == key:
            element.remove(tag)


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    parser = etree.XMLParser(huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}

    forward = Transformer.from_crs(4326, 32652, always_xy=True)
    inverse = Transformer.from_crs(32652, 4326, always_xy=True)
    origin_x, origin_y = forward.transform(126.7730819386, 37.2380038082)

    def member(lanelet_id, role):
        values = [
            item for item in relations[lanelet_id].findall("member")
            if item.get("type") == "way" and item.get("role") == role
        ]
        if len(values) != 1:
            raise RuntimeError((lanelet_id, role, len(values)))
        return values[0]

    def way_id(lanelet_id, role):
        return int(member(lanelet_id, role).get("ref"))

    def refs(way_id_value):
        return [int(item.get("ref")) for item in ways[way_id_value].findall("nd")]

    def xyz(node_id):
        node = nodes[node_id]
        x, y = forward.transform(float(node.get("lon")), float(node.get("lat")))
        node_tags = tags(node)
        return x - origin_x, y - origin_y, float(node_tags.get("ele", 0.0))

    next_node_id = max(nodes) + 1

    def add_node(point):
        nonlocal next_node_id
        node_id = next_node_id
        next_node_id += 1
        lon, lat = inverse.transform(point[0] + origin_x, point[1] + origin_y)
        node = etree.Element(
            "node", id=str(node_id), lat=f"{lat:.14f}", lon=f"{lon:.14f}",
            version="1", visible="true",
        )
        etree.SubElement(node, "tag", k="ele", v=f"{point[2]:.9f}")
        nodes[node_id] = node
        root.append(node)
        return node_id

    def replace_nodes(way, node_ids):
        for item in list(way.findall("nd")):
            way.remove(item)
        first_tag = way.find("tag")
        insert_at = len(way) if first_tag is None else way.index(first_tag)
        for offset, node_id in enumerate(node_ids):
            way.insert(insert_at + offset, etree.Element("nd", ref=str(node_id)))

    boundary_way_ids = {
        way_id(lanelet_id, role)
        for lanelet_id in TARGET_LANELETS
        for role in ("left", "right")
    }
    boundary_before = {way: len(refs(way)) for way in boundary_way_ids}

    # Preserve the endpoint node IDs. They are the exact longitudinal joints
    # used by predecessor/successor lanelets and by shared lateral boundaries.
    for boundary_id in boundary_way_ids:
        node_ids = refs(boundary_id)
        replace_nodes(ways[boundary_id], [node_ids[0], node_ids[-1]])

    # Rebuild each target centerline as the midpoint of its two straight edges.
    # Centerline ways are not shared, so fresh nodes cannot disturb neighbors.
    for lanelet_id in TARGET_LANELETS:
        left = refs(way_id(lanelet_id, "left"))
        right = refs(way_id(lanelet_id, "right"))
        left_points = [xyz(left[0]), xyz(left[-1])]
        right_points = [xyz(right[0]), xyz(right[-1])]
        center_points = [
            tuple((left_points[index][axis] + right_points[index][axis]) / 2.0 for axis in range(3))
            for index in (0, 1)
        ]
        center_id = way_id(lanelet_id, "centerline")
        replace_nodes(ways[center_id], [add_node(point) for point in center_points])

    # A dashed line is a passable separator only when two lanelets reference
    # that exact way. Split merge-corridor edges copied from dashed source ways
    # were one-sided in v76 and must be virtual boundaries instead.
    all_lanelet_uses = {}
    for lanelet_id, relation in relations.items():
        if tags(relation).get("type") != "lanelet":
            continue
        for role in ("left", "right"):
            boundary_id = way_id(lanelet_id, role)
            all_lanelet_uses.setdefault(boundary_id, []).append((lanelet_id, role))

    one_sided_dashed = []
    for boundary_id in boundary_way_ids:
        boundary = ways[boundary_id]
        if tags(boundary).get("subtype") != "dashed":
            continue
        if len(all_lanelet_uses.get(boundary_id, [])) == 1:
            one_sided_dashed.append(boundary_id)
            set_tag(boundary, "type", "virtual")
            remove_tag(boundary, "subtype")

    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "76":
        raise RuntimeError("input MetaInfo map_version is not 76")
    meta.set("map_version", "77")

    root[:] = sorted(
        root,
        key=lambda element: {"MetaInfo": 0, "node": 1, "way": 2, "relation": 3}.get(
            element.tag, 0
        ),
    )
    etree.indent(tree, space="  ")
    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True)

    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": hashlib.sha256(INPUT.read_bytes()).hexdigest(),
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),
        "target_lanelets": sorted(TARGET_LANELETS),
        "straightened_boundary_ways": sorted(boundary_way_ids),
        "boundary_node_counts_before": boundary_before,
        "boundary_node_count_after": 2,
        "one_sided_dashed_converted_to_virtual": sorted(one_sided_dashed),
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
