#!/usr/bin/env python3
"""Build v72 by raising only 58 km/h road lanelets to 59 km/h."""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
INPUT = ROOT / "lanelet_map/lanelet2_map_v71.osm"
OUTPUT = ROOT / "lanelet_map/lanelet2_map_v72.osm"
REPORT = ROOT / "docs/kcity_v72_speed_update_report.json"


def tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = etree.XMLParser(huge_tree=True, remove_blank_text=False)
    tree = etree.parse(str(INPUT), parser)
    root = tree.getroot()
    meta = root.find("MetaInfo")
    assert meta is not None and meta.get("map_version") == "71"

    before = Counter()
    changed = []
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        speed = relation_tags.get("speed_limit")
        before[speed] += 1
        if speed != "58":
            continue
        speed_tag = next(tag for tag in relation.findall("tag") if tag.get("k") == "speed_limit")
        speed_tag.set("v", "59")
        changed.append(int(relation.get("id")))

    assert before == Counter({"58": 1126, "150": 32, "45": 34, "30": 14}), before
    assert len(changed) == 1126
    meta.set("map_version", "72")
    tree.write(str(OUTPUT), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    report = {
        "input": str(INPUT.relative_to(ROOT.parent)),
        "input_sha256": sha256(INPUT),
        "output": str(OUTPUT.relative_to(ROOT.parent)),
        "output_sha256": sha256(OUTPUT),
        "changed_lanelet_count": len(changed),
        "changed_lanelet_ids": changed,
        "before": dict(sorted(before.items(), key=lambda item: float(item[0]))),
        "after": {"30": 14, "45": 34, "59": 1126, "150": 32},
    }
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "changed_lanelet_ids"}, indent=2))


if __name__ == "__main__":
    main()
