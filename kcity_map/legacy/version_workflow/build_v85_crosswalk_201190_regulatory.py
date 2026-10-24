#!/usr/bin/env python3
"""Build v85 by attaching crosswalk 201190 to every intersecting road lanelet."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Polygon


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v84.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v85.osm"
REPORT = ROOT / "docs/kcity_v85_crosswalk_201190_regulatory_build.json"
EXPECTED_INPUT_SHA256 = "36369766a0fade01cbd64a49d4d9c33248b15770eb1433ce334d6becaddf3e4d"

CROSSWALK_LANELET_ID = 201190
CROSSWALK_POLYGON_ID = 105624
CROSSWALK_REGULATORY_ID = 201480
CROSSWALK_NODE_IDS = [92376, 92377, 92379, 92378, 92376]
ROAD_LANELET_IDS = [200309, 200383, 200433, 200950]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tags(element):
    return {item.get("k"): item.get("v") for item in element.findall("tag")}


def insert_member_before_tags(relation, member):
    first_tag = relation.find("tag")
    if first_tag is None:
        relation.append(member)
    else:
        relation.insert(relation.index(first_tag), member)


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    if sha256(INPUT) != EXPECTED_INPUT_SHA256:
        raise RuntimeError("unexpected v84 input SHA")

    parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is None or meta.get("map_version") != "84":
        raise RuntimeError("unexpected v84 MetaInfo")

    nodes = {int(item.get("id")): item for item in root.findall("node")}
    ways = {int(item.get("id")): item for item in root.findall("way")}
    relations = {int(item.get("id")): item for item in root.findall("relation")}
    if CROSSWALK_POLYGON_ID in ways or CROSSWALK_REGULATORY_ID in relations:
        raise RuntimeError("reserved v85 primitive ID already exists")
    if not set(CROSSWALK_NODE_IDS).issubset(nodes):
        raise RuntimeError("crosswalk corner node is missing")
    if CROSSWALK_LANELET_ID not in relations:
        raise RuntimeError("crosswalk lanelet 201190 is missing")
    if tags(relations[CROSSWALK_LANELET_ID]).get("subtype") != "crosswalk":
        raise RuntimeError("lanelet 201190 is not a crosswalk")
    if not set(ROAD_LANELET_IDS).issubset(relations):
        raise RuntimeError("intersecting road lanelet is missing")

    meta.set("map_version", "85")

    polygon = etree.Element("way", id=str(CROSSWALK_POLYGON_ID))
    for node_id in CROSSWALK_NODE_IDS:
        etree.SubElement(polygon, "nd", ref=str(node_id))
    etree.SubElement(polygon, "tag", k="type", v="crosswalk_polygon")
    etree.SubElement(polygon, "tag", k="area", v="yes")
    first_relation_index = next(
        index for index, item in enumerate(root) if item.tag == "relation"
    )
    root.insert(first_relation_index, polygon)

    regulatory = etree.Element("relation", id=str(CROSSWALK_REGULATORY_ID))
    etree.SubElement(
        regulatory,
        "member",
        type="way",
        role="crosswalk_polygon",
        ref=str(CROSSWALK_POLYGON_ID),
    )
    etree.SubElement(
        regulatory,
        "member",
        type="relation",
        role="refers",
        ref=str(CROSSWALK_LANELET_ID),
    )
    etree.SubElement(regulatory, "tag", k="type", v="regulatory_element")
    etree.SubElement(regulatory, "tag", k="subtype", v="crosswalk")
    root.append(regulatory)

    for lanelet_id in ROAD_LANELET_IDS:
        road = relations[lanelet_id]
        if tags(road).get("subtype") != "road":
            raise RuntimeError(f"lanelet {lanelet_id} is not a road")
        duplicate = [
            member
            for member in road.findall("member")
            if member.get("type") == "relation"
            and member.get("role") == "regulatory_element"
            and member.get("ref") == str(CROSSWALK_REGULATORY_ID)
        ]
        if duplicate:
            raise RuntimeError(f"lanelet {lanelet_id} already references the new regulatory element")
        insert_member_before_tags(
            road,
            etree.Element(
                "member",
                type="relation",
                role="regulatory_element",
                ref=str(CROSSWALK_REGULATORY_ID),
            ),
        )

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)

    def xy(node_id):
        node = nodes[node_id]
        return to_utm.transform(float(node.get("lon")), float(node.get("lat")))

    shape = Polygon([xy(node_id) for node_id in CROSSWALK_NODE_IDS])
    if not shape.is_valid or shape.area <= 0.0:
        raise RuntimeError("new crosswalk polygon is invalid")

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=False)
    report = {
        "input": str(INPUT.relative_to(ROOT)),
        "input_sha256": sha256(INPUT),
        "output": str(OUTPUT.relative_to(ROOT)),
        "output_sha256": sha256(OUTPUT),
        "map_version": 85,
        "crosswalk_lanelet_id": CROSSWALK_LANELET_ID,
        "crosswalk_polygon_id": CROSSWALK_POLYGON_ID,
        "crosswalk_regulatory_element_id": CROSSWALK_REGULATORY_ID,
        "crosswalk_polygon_node_ids": CROSSWALK_NODE_IDS,
        "road_lanelet_ids": ROAD_LANELET_IDS,
        "crosswalk_polygon_area_m2": shape.area,
        "explicit_stop_line": False,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
