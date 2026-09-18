#!/usr/bin/env python3
"""Build v79 by removing superseded standalone geometry in the highway bottleneck."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v78.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v79.osm"
REPORT = ROOT / "docs/kcity_v79_bottleneck_cleanup_build.json"

# Each way was formerly a lanelet boundary/centerline, was replaced between
# v68 and v75, and has remained as unreferenced display geometry ever since.
OBSOLETE_WAYS = {
    100690,
    101993,
    104167,
    104201,
    105436,
    105476,
    105478,
    105480,
    105482,
}


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")

    tree = etree.parse(str(INPUT), etree.XMLParser(huge_tree=True))
    root = tree.getroot()
    ways = {int(item.get("id")): item for item in root.findall("way")}
    nodes = {int(item.get("id")): item for item in root.findall("node")}
    relation_way_refs = {
        int(member.get("ref"))
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "way"
    }
    relation_node_refs = {
        int(member.get("ref"))
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "node"
    }

    missing = OBSOLETE_WAYS - set(ways)
    referenced = OBSOLETE_WAYS & relation_way_refs
    if missing or referenced:
        raise RuntimeError({"missing": sorted(missing), "referenced": sorted(referenced)})

    candidate_nodes = {
        int(node.get("ref"))
        for way_id in OBSOLETE_WAYS
        for node in ways[way_id].findall("nd")
    }
    for way_id in OBSOLETE_WAYS:
        root.remove(ways[way_id])

    remaining_way_nodes = {
        int(node.get("ref"))
        for way in root.findall("way")
        for node in way.findall("nd")
    }
    protected_nodes = remaining_way_nodes | relation_node_refs
    coordinate_only_tags = {"ele", "local_x", "local_y"}
    removed_nodes = []
    for node_id in sorted(candidate_nodes - protected_nodes):
        if set(tags(nodes[node_id])) - coordinate_only_tags:
            raise RuntimeError(f"obsolete geometry node {node_id} has semantic tags")
        root.remove(nodes[node_id])
        removed_nodes.append(node_id)

    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "78":
        raise RuntimeError("input MetaInfo map_version is not 78")
    meta.set("map_version", "79")

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
        "removed_obsolete_ways": sorted(OBSOLETE_WAYS),
        "removed_orphan_nodes": removed_nodes,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({**report, "removed_orphan_nodes": len(removed_nodes)}, indent=2))


if __name__ == "__main__":
    main()
