#!/usr/bin/env python3
"""Build K-City v73 with consistent high-speed lane-change topology."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v72.osm"
SPLIT_REFERENCE = ROOT / "lanelet_map/lanelet2_map_v60.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v73.osm"
REPORT = ROOT / "docs/kcity_v73_highway_lane_change_report.json"

# v71 merged across a topology transition where four parallel lanes begin to
# converge.  Restore the cross-section boundary so lateral routing applies only
# while the lanes are actually parallel.
RESTORED_LANELETS = (
    "200148", "200127", "200130", "200134",
    "201440", "201439", "201441", "201446",
)

# Every internal boundary in a physically parallel 150 km/h cross-section is
# passable in v73.  Boundaries in converging/merging polygons are intentionally
# excluded and remain longitudinal routing transitions.
PASSABLE_BOUNDARIES = (
    "102241",  # 200186 <-> 200138
    "102237",  # 200141 <-> 201444
    "101992",  # 200148 <-> 200127
    "101993",  # 200127 <-> 200130
    "101991",  # 200130 <-> 200134
    "105456",  # 200147 <-> 200185 (downstream v72 geometry)
    "105462",  # 201400 <-> 201399
)


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def replace_tag(element, key, value):
    tag = next((tag for tag in element.findall("tag") if tag.get("k") == key), None)
    if tag is None:
        element.append(etree.Element("tag", k=key, v=value))
    else:
        tag.set("v", value)


def member(relation, role):
    values = [item for item in relation.findall("member") if item.get("role") == role]
    if len(values) != 1:
        raise RuntimeError(f"relation {relation.get('id')} role={role}: {len(values)}")
    return values[0]


def replace_or_append(root, index, element):
    element_id = element.get("id")
    old = index.get(element_id)
    copied = copy.deepcopy(element)
    if old is None:
        first_relation = root.find("relation")
        root.insert(root.index(first_relation), copied)
    else:
        root.replace(old, copied)
    index[element_id] = copied


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    reference_root = etree.parse(str(SPLIT_REFERENCE), parser).getroot()

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}
    ref_nodes = {item.get("id"): item for item in reference_root.findall("node")}
    ref_ways = {item.get("id"): item for item in reference_root.findall("way")}
    ref_relations = {item.get("id"): item for item in reference_root.findall("relation")}

    copied_way_ids = set()
    copied_node_ids = set()
    for lanelet_id in RESTORED_LANELETS:
        relation = ref_relations[lanelet_id]
        for item in relation.findall("member"):
            if item.get("type") != "way":
                continue
            way_id = item.get("ref")
            way = ref_ways[way_id]
            for nd in way.findall("nd"):
                node_id = nd.get("ref")
                replace_or_append(root, nodes, ref_nodes[node_id])
                copied_node_ids.add(node_id)
            replace_or_append(root, ways, way)
            copied_way_ids.add(way_id)
        if lanelet_id in relations:
            # Keep all v72 semantic tags/regulatory members and replace only the
            # three geometry members with the last known-good v60 topology.
            current = relations[lanelet_id]
            for role in ("left", "right", "centerline"):
                member(current, role).set("ref", member(relation, role).get("ref"))
        else:
            copied = copy.deepcopy(relation)
            replace_tag(copied, "speed_limit", "150")
            replace_or_append(root, relations, copied)

    # The downstream map was resampled in v61.  Snap the restored merge ways to
    # the existing v72 successor nodes so following relations remain exact.
    def first_node(lanelet_id, role):
        way = ways[member(relations[lanelet_id], role).get("ref")]
        return way.find("nd").get("ref")

    downstream_nodes = {
        "outer": first_node("200147", "left"),
        "middle": first_node("200147", "right"),
        "inner": first_node("200185", "right"),
    }
    merge_end_roles = {
        "201440": ("outer", "middle"),
        "201439": ("middle", "inner"),
        "201441": ("middle", "inner"),
        "201446": ("middle", "inner"),
    }
    for lanelet_id, (left_key, right_key) in merge_end_roles.items():
        for role, key in (("left", left_key), ("right", right_key)):
            way = ways[member(relations[lanelet_id], role).get("ref")]
            way.findall("nd")[-1].set("ref", downstream_nodes[key])

    for way_id in PASSABLE_BOUNDARIES:
        way = ways[way_id]
        replace_tag(way, "type", "line_thin")
        replace_tag(way, "subtype", "dashed")

    # Remove superseded virtual ways from the broken v61 layout when no other
    # relation uses them.
    referenced_way_ids = {
        item.get("ref")
        for relation in relations.values()
        for item in relation.findall("member")
        if item.get("type") == "way"
    }
    removed_ways = []
    for way_id in ("105443", "105442", "105444", "105434", "105440", "105438"):
        if way_id in ways and way_id not in referenced_way_ids:
            root.remove(ways[way_id])
            removed_ways.append(way_id)

    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "72":
        raise RuntimeError("input MetaInfo map_version is not 72")
    meta.set("map_version", "73")

    etree.indent(tree, space="  ")
    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": sha256(INPUT),
        "split_reference": str(SPLIT_REFERENCE.relative_to(ROOT)),
        "split_reference_sha256": sha256(SPLIT_REFERENCE),
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": sha256(OUTPUT),
        "restored_lanelets": [int(item) for item in RESTORED_LANELETS],
        "copied_way_count": len(copied_way_ids),
        "copied_node_count": len(copied_node_ids),
        "passable_boundaries": [int(item) for item in PASSABLE_BOUNDARIES],
        "removed_superseded_ways": [int(item) for item in removed_ways],
    }
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
