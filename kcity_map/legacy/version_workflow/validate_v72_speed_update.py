#!/usr/bin/env python3
"""Verify that v72 differs from v71 only by the requested speed update."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
OLD = ROOT / "lanelet_map/lanelet2_map_v71.osm"
NEW = ROOT / "lanelet_map/lanelet2_map_v72.osm"
REPORT = ROOT / "docs/kcity_v72_speed_update_report.json"


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def members(element):
    return [(m.get("type"), m.get("ref"), m.get("role")) for m in element.findall("member")]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = etree.XMLParser(huge_tree=True)
    old_root = etree.parse(str(OLD), parser).getroot()
    new_root = etree.parse(str(NEW), parser).getroot()
    report = json.loads(REPORT.read_text())
    assert old_root.find("MetaInfo").get("map_version") == "71"
    assert new_root.find("MetaInfo").get("map_version") == "72"
    assert sha256(OLD) == report["input_sha256"]
    assert sha256(NEW) == report["output_sha256"]

    for kind in ("node", "way"):
        old = {item.get("id"): etree.tostring(item) for item in old_root.findall(kind)}
        new = {item.get("id"): etree.tostring(item) for item in new_root.findall(kind)}
        assert old == new, f"{kind} geometry changed"

    old_relations = {item.get("id"): item for item in old_root.findall("relation")}
    new_relations = {item.get("id"): item for item in new_root.findall("relation")}
    assert set(old_relations) == set(new_relations)
    changed = []
    speeds = Counter()
    for relation_id, old_relation in old_relations.items():
        new_relation = new_relations[relation_id]
        assert members(old_relation) == members(new_relation), relation_id
        old_tags = tags(old_relation)
        new_tags = tags(new_relation)
        if old_tags.get("type") == "lanelet" and old_tags.get("subtype") == "road":
            speeds[new_tags.get("speed_limit")] += 1
        if old_tags == new_tags:
            continue
        expected = dict(old_tags)
        assert old_tags.get("type") == "lanelet" and old_tags.get("subtype") == "road", relation_id
        assert old_tags.get("speed_limit") == "58", relation_id
        expected["speed_limit"] = "59"
        assert new_tags == expected, relation_id
        changed.append(int(relation_id))

    assert len(changed) == 1126
    assert changed == report["changed_lanelet_ids"]
    assert speeds == Counter({"59": 1126, "150": 32, "45": 34, "30": 14}), speeds
    assert not any(tags(r).get("speed_limit") == "58" for r in new_relations.values())
    print(f"sha256={sha256(NEW)}")
    print(f"changed_58_to_59={len(changed)} remaining_58=0")
    print("speed_distribution=30:14,45:34,59:1126,150:32")
    print("nodes_unchanged=1 ways_unchanged=1 relation_members_unchanged=1 other_tags_unchanged=1")


if __name__ == "__main__":
    main()
