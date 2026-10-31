#!/usr/bin/env python3
"""Release v69 with competition intersection and highway speed policy."""

from copy import deepcopy
from pathlib import Path
import hashlib
import json

from lxml import etree as E


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v68.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v69.osm"

INTERSECTION_SPEEDS = {
    "104012": "45",
    "104013": "58",
    "104014": "58",
    "104017": "45",
    "104018": "45",
}

# MGeo competition exception: A2256W000411 start to A2256W000153 end.
# These are every v68 lanelet wholly contained in that longitudinal corridor,
# including its parallel lanes, merge connectors, and tollgate fan-out.
HIGHWAY_EXCEPTION_LANELETS = {
    "200099", "200101", "200102", "200127", "200128", "200130", "200134",
    "200137", "200138", "200141", "200142", "200147", "200148", "200152",
    "200185", "200186", "200187", "200188", "200360", "201399", "201400",
    "201434", "201435", "201436", "201438", "201439", "201440", "201441",
    "201442", "201443", "201444", "201445", "201446", "201447", "201448",
    "201449", "201450", "201451", "201452", "201453", "201454",
}

MGEO_BOUNDARY = {
    "start_link": "A2256W000411",
    "start_xyz": [62.073913682601415, 256.4949275725521, 28.268789437406806],
    "end_link": "A2256W000153",
    "end_xyz": [70.21414883446414, -365.74847655789927, 28.301585314776467],
}


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def set_tag(element, key, value):
    match = element.find(f"tag[@k='{key}']")
    if match is None:
        element.append(E.Element("tag", k=key, v=value))
    else:
        match.set("v", value)


def main():
    if OUTPUT.exists():
        raise RuntimeError(f"released map output already exists: {OUTPUT}")
    tree = E.parse(str(INPUT))
    root = tree.getroot()
    before = deepcopy(root)
    lanelets = {
        relation.get("id"): relation
        for relation in root.findall("relation")
        if tags(relation).get("type") == "lanelet"
    }

    intersection_members = {}
    for area_id, speed in INTERSECTION_SPEEDS.items():
        members = []
        for lanelet_id, lanelet in lanelets.items():
            lanelet_tags = tags(lanelet)
            if lanelet_tags.get("intersection_area") == area_id:
                assert lanelet_tags.get("subtype") == "road"
                set_tag(lanelet, "speed_limit", speed)
                members.append(lanelet_id)
        assert members, area_id
        intersection_members[area_id] = sorted(members, key=int)

    demoted = []
    for lanelet_id, lanelet in lanelets.items():
        lanelet_tags = tags(lanelet)
        if lanelet_tags.get("subtype") != "road":
            continue
        speed = float(lanelet_tags.get("speed_limit", "0"))
        if lanelet_id in HIGHWAY_EXCEPTION_LANELETS:
            set_tag(lanelet, "speed_limit", "100")
        elif speed > 58:
            set_tag(lanelet, "speed_limit", "58")
            demoted.append(lanelet_id)

    assert sorted(demoted, key=int) == ["200088", "200150", "201086"]
    root.find("MetaInfo").set("map_version", "69")

    # Only version and the intended speed_limit tags may differ from v68.
    allowed = set(demoted)
    for members in intersection_members.values():
        allowed.update(members)
    normalized = deepcopy(root)
    normalized.find("MetaInfo").set("map_version", "68")
    old_relations = {r.get("id"): r for r in before.findall("relation")}
    new_relations = {r.get("id"): r for r in normalized.findall("relation")}
    for relation_id in allowed:
        old_speed = old_relations[relation_id].find("tag[@k='speed_limit']")
        new_speed = new_relations[relation_id].find("tag[@k='speed_limit']")
        new_speed.set("v", old_speed.get("v"))
    for document in (before, normalized):
        for element in document.iter():
            element.text = (element.text or "").strip() or None
            element.tail = None
    assert E.tostring(before) == E.tostring(normalized)

    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    report = {
        "version": 69,
        "source_version": 68,
        "mgeo_boundary": MGEO_BOUNDARY,
        "highway_exception_lanelets": sorted(HIGHWAY_EXCEPTION_LANELETS, key=int),
        "demoted_outside_exception": sorted(demoted, key=int),
        "intersection_speeds_kph": INTERSECTION_SPEEDS,
        "intersection_members": intersection_members,
        "geometry_and_other_semantics_unchanged": True,
        "sha256": hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),
    }
    report_path = ROOT / "docs/kcity_v69_speed_policy_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
