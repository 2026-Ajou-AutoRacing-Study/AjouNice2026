#!/usr/bin/env python3
"""Build v83: connect the 200291 opposite lane and polygon 104158 to road bounds."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Polygon


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v82.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v83.osm"
REPORT = ROOT / "docs/kcity_v83_lanelet_200291_intersection_build.json"
EXPECTED_INPUT_SHA256 = "c7632c255eec8e55b18550f7b52b8eb9ec986bd76e8654d26ca555b6d0599cb6"

SHARED_BOUND = "100395"
DUPLICATE_BOUND = "100394"
POLYGON_ID = "104158"
ENDPOINT_INSERTIONS = {
    "231569": "3050",  # 200291 / 100396 south end
    "231572": "6120",  # 200289 / 101881 south end
    "231576": "2447",  # 200289 / 101881 north end
    "231579": "6030",  # 200291 / 100396 north end
}
REMOVABLE_ORPHAN_NODES = {"6035", "6036", "6038"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def refs(way):
    return [item.get("ref") for item in way.findall("nd")]


def relation_member(relation, role):
    return next(item for item in relation.findall("member") if item.get("role") == role)


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    input_sha = sha256(INPUT)
    if input_sha != EXPECTED_INPUT_SHA256:
        raise RuntimeError(f"unexpected v82 input SHA: {input_sha}")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "82":
        raise RuntimeError("unexpected v82 MetaInfo")
    meta.set("map_version", "83")

    nodes = {item.get("id"): item for item in root.findall("node")}
    ways = {item.get("id"): item for item in root.findall("way")}
    relations = {item.get("id"): item for item in root.findall("relation")}

    lane_200289 = relations["200289"]
    lane_200291 = relations["200291"]
    if relation_member(lane_200289, "left").get("ref") != DUPLICATE_BOUND:
        raise RuntimeError("unexpected 200289 left bound")
    if relation_member(lane_200291, "left").get("ref") != SHARED_BOUND:
        raise RuntimeError("unexpected 200291 left bound")
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def point(node_id):
        node = nodes[node_id]
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    duplicate_shape = LineString([point(item) for item in refs(ways[DUPLICATE_BOUND])])
    shared_shape = LineString([point(item) for item in refs(ways[SHARED_BOUND])])
    duplicate_hausdorff_m = duplicate_shape.hausdorff_distance(shared_shape)
    if duplicate_hausdorff_m > 0.05:
        raise RuntimeError(
            f"100394 and 100395 geometry differs by {duplicate_hausdorff_m:.3f} m"
        )
    if point(refs(ways[DUPLICATE_BOUND])[0]) != point(refs(ways[SHARED_BOUND])[-1]):
        raise RuntimeError("100394/100395 first shared endpoint does not match")
    if point(refs(ways[DUPLICATE_BOUND])[-1]) != point(refs(ways[SHARED_BOUND])[0]):
        raise RuntimeError("100394/100395 second shared endpoint does not match")

    # Lanelet2 infers inversion from the lanelet polygon orientation. Reusing the
    # same primitive gives 200289 an inverted 100395 while 200291 remains direct.
    relation_member(lane_200289, "left").set("ref", SHARED_BOUND)

    # The physical center marking is traversable by the configured SOA opposite-
    # lane policy. Leaving it as road_border would make candidate validation reject it.
    shared = ways[SHARED_BOUND]
    for tag in list(shared.findall("tag")):
        shared.remove(tag)
    etree.SubElement(shared, "tag", k="type", v="line_thin")
    etree.SubElement(shared, "tag", k="subtype", v="dashed")

    polygon = ways[POLYGON_ID]
    if tags(polygon) != {"type": "intersection_area", "area": "yes"}:
        raise RuntimeError("unexpected polygon 104158 tags")
    polygon_refs_before = refs(polygon)
    if polygon_refs_before[0] != polygon_refs_before[-1]:
        raise RuntimeError("polygon 104158 is not closed")
    if any(node_id in polygon_refs_before for node_id in ENDPOINT_INSERTIONS.values()):
        raise RuntimeError("an endpoint node is already present in polygon 104158")

    for nd in list(polygon.findall("nd")):
        endpoint = ENDPOINT_INSERTIONS.get(nd.get("ref"))
        if endpoint is not None:
            nd.addnext(etree.Element("nd", ref=endpoint))

    polygon_refs_after = refs(polygon)
    for anchor, endpoint in ENDPOINT_INSERTIONS.items():
        index = polygon_refs_after.index(anchor)
        if polygon_refs_after[index + 1] != endpoint:
            raise RuntimeError(f"failed to insert endpoint {endpoint} after {anchor}")

    # Remove the duplicate way. Only its now-unreferenced private nodes are removed;
    # node 6037 remains because another way also uses it.
    root.remove(ways[DUPLICATE_BOUND])
    ways.pop(DUPLICATE_BOUND)
    usage = Counter(node_id for way in ways.values() for node_id in refs(way))
    removed_nodes = []
    for node_id in sorted(REMOVABLE_ORPHAN_NODES, key=int):
        if usage[node_id] != 0:
            raise RuntimeError(f"expected node {node_id} to be orphaned")
        root.remove(nodes[node_id])
        removed_nodes.append(int(node_id))
    if usage["6037"] == 0:
        raise RuntimeError("shared node 6037 must remain in use")

    shape = Polygon([point(node_id) for node_id in polygon_refs_after])
    if not shape.is_valid or shape.area <= 0.0:
        raise RuntimeError("polygon 104158 became invalid")

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=False)
    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": input_sha,
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": sha256(OUTPUT),
        "map_version": 83,
        "topology": {
            "lanelet_200291_left": int(SHARED_BOUND),
            "lanelet_200289_left": int(SHARED_BOUND),
            "removed_duplicate_way": int(DUPLICATE_BOUND),
            "removed_orphan_nodes": removed_nodes,
            "duplicate_geometry_hausdorff_m": duplicate_hausdorff_m,
            "shared_bound_tags": tags(shared),
        },
        "intersection_area": {
            "id": int(POLYGON_ID),
            "tags": tags(polygon),
            "inserted_endpoint_nodes": [int(item) for item in ENDPOINT_INSERTIONS.values()],
            "area_m2": shape.area,
        },
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
