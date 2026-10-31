#!/usr/bin/env python3
"""Build v81 by splitting 201448/201447 at their common longitudinal midpoint."""

from __future__ import annotations

import hashlib
import json
import math
from copy import deepcopy
from pathlib import Path

from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v80.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v81.osm"
REPORT = ROOT / "docs/kcity_v81_mid_bottleneck_lc_build.json"
EXPECTED_INPUT_SHA256 = "1f3ff082de3dde845f59c50abbdf07418a196fc07a9c965029e3b46005649174"

LEFT_UPSTREAM = "201448"
RIGHT_DOWNSTREAM = "201447"
LEFT_DOWNSTREAM = "201478"
RIGHT_UPSTREAM = "201479"

TARGET_WAYS = ("105458", "105461", "105453", "105491", "105486")


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def replace_refs(way, node_ids):
    children = list(way)
    nd_positions = [index for index, child in enumerate(children) if child.tag == "nd"]
    insert_at = nd_positions[0] if nd_positions else 0
    for child in list(way.findall("nd")):
        way.remove(child)
    for offset, node_id in enumerate(node_ids):
        way.insert(insert_at + offset, etree.Element("nd", ref=str(node_id)))


def node_ele(node):
    return float(tags(node).get("ele", "0"))


def append_before_relations(root, element):
    relations = root.findall("relation")
    if relations:
        root.insert(root.index(relations[0]), element)
    else:
        root.append(element)


def append_relation(root, relation):
    root.append(relation)


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    input_sha = hashlib.sha256(INPUT.read_bytes()).hexdigest()
    if input_sha != EXPECTED_INPUT_SHA256:
        raise RuntimeError(f"unexpected v80 input SHA: {input_sha}")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    meta_info = root.find("MetaInfo")
    if meta_info is None or meta_info.get("map_version") != "80":
        raise RuntimeError("unexpected v80 MetaInfo")
    meta_info.set("map_version", "81")
    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}

    if LEFT_DOWNSTREAM in relations or RIGHT_UPSTREAM in relations:
        raise RuntimeError("v81 lanelet IDs already exist in v80")
    if max(map(int, relations)) != 201477 or max(map(int, ways)) != 105618:
        raise RuntimeError("unexpected v80 ID allocation")

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32652", "EPSG:4326", always_xy=True)
    next_node = max(map(int, nodes)) + 1
    next_way = max(map(int, ways)) + 1
    split_log = []
    downstream_way = {}

    def xyz(node_id):
        node = nodes[node_id]
        x, y = to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        return x, y, node_ele(node)

    def split_way(way_id):
        nonlocal next_node, next_way
        way = ways[way_id]
        node_ids = refs(way)
        points = [xyz(node_id) for node_id in node_ids]
        lengths = [
            math.dist(points[index][:2], points[index + 1][:2])
            for index in range(len(points) - 1)
        ]
        total = sum(lengths)
        target = total * 0.5
        accumulated = 0.0
        segment = None
        ratio = None
        for index, length in enumerate(lengths):
            if accumulated + length >= target:
                segment = index
                ratio = (target - accumulated) / length
                break
            accumulated += length
        if segment is None or ratio is None:
            raise RuntimeError(f"failed to split way {way_id}")
        first, second = points[segment], points[segment + 1]
        cut = tuple(first[axis] + ratio * (second[axis] - first[axis]) for axis in range(3))
        lon, lat = to_wgs.transform(cut[0], cut[1])
        node_id = str(next_node)
        next_node += 1
        node = etree.Element("node", id=node_id, lat=f"{lat:.14f}", lon=f"{lon:.14f}")
        etree.SubElement(node, "tag", k="ele", v=f"{cut[2]:.9f}")
        append_before_relations(root, node)
        nodes[node_id] = node

        upstream_refs = node_ids[: segment + 1] + [node_id]
        downstream_refs = [node_id] + node_ids[segment + 1 :]
        if len(upstream_refs) < 2 or len(downstream_refs) < 2:
            raise RuntimeError(f"degenerate split for way {way_id}")
        replace_refs(way, upstream_refs)

        new_way_id = str(next_way)
        next_way += 1
        new_way = deepcopy(way)
        new_way.set("id", new_way_id)
        new_way.attrib.pop("version", None)
        new_way.attrib.pop("visible", None)
        replace_refs(new_way, downstream_refs)
        append_before_relations(root, new_way)
        ways[new_way_id] = new_way
        downstream_way[way_id] = new_way_id
        split_log.append(
            {
                "source_way": int(way_id),
                "downstream_way": int(new_way_id),
                "split_node": int(node_id),
                "source_point_count": len(node_ids),
                "upstream_point_count": len(upstream_refs),
                "downstream_point_count": len(downstream_refs),
                "source_length_m": total,
                "upstream_length_m": target,
                "downstream_length_m": total - target,
                "split_lat": lat,
                "split_lon": lon,
                "split_ele": cut[2],
            }
        )

    for way_id in TARGET_WAYS:
        split_way(way_id)

    def set_member(relation, role, way_id):
        member = next(item for item in relation.findall("member") if item.get("role") == role)
        member.set("ref", way_id)

    left_upstream = relations[LEFT_UPSTREAM]
    right_original = relations[RIGHT_DOWNSTREAM]

    left_downstream = deepcopy(left_upstream)
    left_downstream.set("id", LEFT_DOWNSTREAM)
    left_downstream.attrib.pop("version", None)
    left_downstream.attrib.pop("visible", None)
    set_member(left_downstream, "left", downstream_way["105458"])
    set_member(left_downstream, "right", downstream_way["105461"])
    set_member(left_downstream, "centerline", downstream_way["105491"])

    right_upstream = deepcopy(right_original)
    right_upstream.set("id", RIGHT_UPSTREAM)
    right_upstream.attrib.pop("version", None)
    right_upstream.attrib.pop("visible", None)

    set_member(right_original, "left", downstream_way["105461"])
    set_member(right_original, "right", downstream_way["105453"])
    set_member(right_original, "centerline", downstream_way["105486"])

    append_relation(root, left_downstream)
    append_relation(root, right_upstream)

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=False)
    output_sha = hashlib.sha256(OUTPUT.read_bytes()).hexdigest()
    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": input_sha,
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": output_sha,
        "split_ratio": 0.5,
        "lanelets": {
            "left_upstream_retained": int(LEFT_UPSTREAM),
            "left_downstream_new": int(LEFT_DOWNSTREAM),
            "right_upstream_new": int(RIGHT_UPSTREAM),
            "right_downstream_retained": int(RIGHT_DOWNSTREAM),
        },
        "ways": split_log,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
