#!/usr/bin/env python3
"""Build K-City v67 with complete competition-intersection 30 km/h zones.

The seven existing physical intersection polygons are preserved.  Every
vehicle road lanelet contained by one of them is attached to that polygon and
receives speed_limit=30.  Pedestrian-only crosswalk lanelets are deliberately
excluded because a vehicle speed limit is not a valid crosswalk attribute.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = ROOT / "lanelet_map/lanelet2_map_v66.osm"
DEFAULT_OUTPUT = ROOT / "lanelet_map/lanelet2_map_v67.osm"

INTERSECTION_AREAS = {
    "104013": {"route_lanelet": "200337", "name": "Route A/B intersection 1"},
    "104014": {"route_lanelet": "200348", "name": "Route A/B intersection 2"},
    "104158": {"route_lanelet": "200291", "name": "Route A/B intersection 3"},
    "104016": {"route_lanelet": "200950", "name": "Route A/B intersection 4"},
    "104017": {"route_lanelet": "200261", "name": "Route A/B intersection 5"},
    "104018": {"route_lanelet": "200445", "name": "Route A/B intersection 6"},
    "104012": {"route_lanelet": "200220", "name": "Route B intersection 7"},
}

# v66 already associated 57 vehicle movements.  Geometry audit found these
# three additional vehicle movements fully contained by polygon 104158.
MISSING_MOVEMENTS = {
    "200289": "straight",
    "200328": "right",
    "200416": "left",
}


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    element.append(etree.Element("tag", k=key, v=value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    xml_parser = etree.XMLParser(remove_blank_text=False, huge_tree=True)
    tree = etree.parse(str(args.input), xml_parser)
    root = tree.getroot()
    ways = {way.get("id"): way for way in root.findall("way")}
    lanelets = {
        relation.get("id"): relation
        for relation in root.findall("relation")
        if tags(relation).get("type") == "lanelet"
    }

    meta = root.find("MetaInfo")
    if meta is None:
        raise RuntimeError("MetaInfo is missing")
    meta.set("map_version", "67")

    for area_id in INTERSECTION_AREAS:
        area = ways.get(area_id)
        if area is None or tags(area).get("type") != "intersection_area":
            raise RuntimeError(f"missing intersection polygon {area_id}")
        set_tag(area, "area", "yes")

    for lanelet_id, direction in MISSING_MOVEMENTS.items():
        lanelet = lanelets[lanelet_id]
        set_tag(lanelet, "turn_direction", direction)
        set_tag(lanelet, "intersection_area", "104158")

    changed = []
    for lanelet_id, lanelet in lanelets.items():
        lanelet_tags = tags(lanelet)
        if lanelet_tags.get("intersection_area") not in INTERSECTION_AREAS:
            continue
        if lanelet_tags.get("subtype") != "road" or lanelet_tags.get("participant:vehicle") != "yes":
            raise RuntimeError(f"intersection member {lanelet_id} is not a vehicle road lanelet")
        old_speed = lanelet_tags.get("speed_limit")
        set_tag(lanelet, "speed_limit", "30")
        changed.append((lanelet_id, lanelet_tags["intersection_area"], old_speed))

    if len(changed) != 60:
        raise RuntimeError(f"expected 60 speed-limited vehicle movements, got {len(changed)}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    print(f"map_version=67")
    print(f"intersection_polygons={len(INTERSECTION_AREAS)}")
    print(f"completed_missing_movements={len(MISSING_MOVEMENTS)}")
    print(f"speed_limit_30_lanelets={len(changed)}")
    for area_id in INTERSECTION_AREAS:
        members = sorted(int(item[0]) for item in changed if item[1] == area_id)
        print(f"area_{area_id}={','.join(map(str, members))}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
