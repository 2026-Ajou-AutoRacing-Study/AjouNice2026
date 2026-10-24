#!/usr/bin/env python3
"""Build v84 by restoring polygon 104158 cap nodes to their pre-v82 positions."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v83.osm"
REFERENCE = ROOT / "lanelet_map/lanelet2_map_v81.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v84.osm"
REPORT = ROOT / "docs/kcity_v84_restore_intersection_104158_caps_build.json"
EXPECTED_INPUT_SHA256 = "46cbdd08545b88fa313f2a9a6e81fdf1066d65c403cc70c57757bc2ca8d32863"
EXPECTED_REFERENCE_SHA256 = "4ece7b1b549cd6920018195f8f3ec8ff6c4925ee3e290ca92d86ae17a2cac892"
POLYGON_ID = "104158"
RESTORED_NODES = {"231570", "231571", "231572", "231577", "231578", "231579"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    if sha256(INPUT) != EXPECTED_INPUT_SHA256:
        raise RuntimeError("unexpected v83 input SHA")
    if sha256(REFERENCE) != EXPECTED_REFERENCE_SHA256:
        raise RuntimeError("unexpected v81 reference SHA")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    reference_tree = etree.parse(str(REFERENCE), parser)
    root = tree.getroot()
    reference_root = reference_tree.getroot()
    if root.find("MetaInfo").get("map_version") != "83":
        raise RuntimeError("unexpected v83 MetaInfo")
    root.find("MetaInfo").set("map_version", "84")

    nodes = {item.get("id"): item for item in root.findall("node")}
    reference_nodes = {item.get("id"): item for item in reference_root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    polygon = ways[POLYGON_ID]
    polygon_refs = refs(polygon)
    if tags(polygon) != {"type": "intersection_area", "area": "yes"}:
        raise RuntimeError("unexpected polygon 104158 tags")
    if polygon_refs[0] != polygon_refs[-1]:
        raise RuntimeError("polygon 104158 is not closed")
    if not RESTORED_NODES.issubset(polygon_refs):
        raise RuntimeError("polygon 104158 is missing a cap node")

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def xy(node):
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    restored = {}
    for node_id in sorted(RESTORED_NODES, key=int):
        current = nodes[node_id]
        reference = reference_nodes[node_id]
        current_xy = xy(current)
        reference_xy = xy(reference)
        distance = math.hypot(reference_xy[0] - current_xy[0], reference_xy[1] - current_xy[1])
        if not math.isclose(distance, 5.0, abs_tol=0.01):
            raise RuntimeError(f"node {node_id} was not displaced by 5 m: {distance}")
        current.set("lat", reference.get("lat"))
        current.set("lon", reference.get("lon"))
        restored[node_id] = {
            "from_utm": list(current_xy),
            "to_utm": list(reference_xy),
            "restored_distance_m": distance,
        }

    shape = Polygon([xy(nodes[node_id]) for node_id in polygon_refs])
    if not shape.is_valid or shape.area <= 0.0:
        raise RuntimeError("restored polygon 104158 is invalid")

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=False)
    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": sha256(INPUT),
        "reference": str(REFERENCE.relative_to(ROOT)),
        "reference_sha256": sha256(REFERENCE),
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": sha256(OUTPUT),
        "map_version": 84,
        "polygon_id": int(POLYGON_ID),
        "restored_nodes": restored,
        "polygon_area_m2": shape.area,
        "preserved_v83_topology_and_endpoint_links": True,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
