#!/usr/bin/env python3
"""Build v78 by removing obsolete highway geometry left by the v76/v77 rebuilds.

v76 replaced lanelet boundary/centerline ways while constructing split and shared
boundaries, but retained the superseded ways as standalone map geometry.  v77
then simplified the active highway ways but likewise retained their old interior
nodes.  These objects are not referenced by any relation and produce duplicate
or dangling lines in map editors (notably way 104165 over way 105553 at nodes
131840--45533).

This cleanup removes only ways proven to have been superseded by the v76 build
and nodes orphaned by those replacements.  Active relation membership and all
active way node sequences are preserved byte-for-byte apart from XML ordering.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
V75 = ROOT / "lanelet_map/lanelet2_map_v75.osm"
V76 = ROOT / "lanelet_map/lanelet2_map_v76.osm"
INPUT = ROOT / "lanelet_map/lanelet2_map_v77.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v78.osm"
REPORT = ROOT / "docs/kcity_v78_obsolete_geometry_build.json"


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def lanelet_way_refs(root):
    result = set()
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet":
            continue
        result.update(
            int(member.get("ref"))
            for member in relation.findall("member")
            if member.get("type") == "way"
        )
    return result


def relation_way_refs(root):
    return {
        int(member.get("ref"))
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "way"
    }


def relation_node_refs(root):
    return {
        int(member.get("ref"))
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "node"
    }


def way_nodes(way):
    return [int(node.get("ref")) for node in way.findall("nd")]


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    parser = etree.XMLParser(huge_tree=True)
    tree75 = etree.parse(str(V75), parser)
    tree76 = etree.parse(str(V76), etree.XMLParser(huge_tree=True))
    tree = etree.parse(str(INPUT), etree.XMLParser(huge_tree=True))
    root75 = tree75.getroot()
    root76 = tree76.getroot()
    root = tree.getroot()

    ways75 = {int(item.get("id")): item for item in root75.findall("way")}
    ways76 = {int(item.get("id")): item for item in root76.findall("way")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    nodes = {int(item.get("id")): item for item in root.findall("node")}

    active_way_refs = relation_way_refs(root)
    max_v75_way_id = max(ways75)

    # Original v75 lanelet members replaced by v76 split/shared ways.
    superseded_original = (lanelet_way_refs(root75) & set(ways)) - active_way_refs
    # Temporary split ways created during v76 and subsequently superseded while
    # assigning canonical shared boundaries.
    superseded_generated = {
        way_id for way_id in ways
        if way_id > max_v75_way_id and way_id not in active_way_refs
    }
    removed_way_ids = sorted(superseded_original | superseded_generated)

    candidate_nodes = {
        node_id
        for way_id in removed_way_ids
        for node_id in way_nodes(ways[way_id])
    }

    # v77 shortened active straight boundaries and centerlines to their two
    # section endpoints. Collect their removed v76 interior nodes for pruning.
    for way_id in active_way_refs & set(ways76) & set(ways):
        candidate_nodes.update(set(way_nodes(ways76[way_id])) - set(way_nodes(ways[way_id])))

    for way_id in removed_way_ids:
        root.remove(ways[way_id])

    remaining_way_nodes = {
        node_id
        for way in root.findall("way")
        for node_id in way_nodes(way)
    }
    protected_nodes = remaining_way_nodes | relation_node_refs(root)
    coordinate_only_tags = {"ele", "local_x", "local_y"}
    removed_node_ids = []
    retained_semantic_nodes = []
    for node_id in sorted(candidate_nodes - protected_nodes):
        node = nodes[node_id]
        semantic_tags = set(tags(node)) - coordinate_only_tags
        if semantic_tags:
            retained_semantic_nodes.append(node_id)
            continue
        root.remove(node)
        removed_node_ids.append(node_id)

    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "77":
        raise RuntimeError("input MetaInfo map_version is not 77")
    meta.set("map_version", "78")

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
        "removed_superseded_original_ways": sorted(superseded_original),
        "removed_superseded_generated_ways": sorted(superseded_generated),
        "removed_nodes": removed_node_ids,
        "retained_semantic_nodes": retained_semantic_nodes,
        "target_way_104165_removed": 104165 not in {
            int(item.get("id")) for item in root.findall("way")
        },
        "target_point_45533_removed": 45533 not in {
            int(item.get("id")) for item in root.findall("node")
        },
        "junction_point_131840_retained": 131840 in {
            int(item.get("id")) for item in root.findall("node")
        },
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
