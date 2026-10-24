#!/usr/bin/env python3
"""Create v40 from v39 with VMB-friendly numeric speed_limit tags."""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import shutil
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, Sequence

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
SPEED_RE = re.compile(r"^\s*([0-9]+(?:\.[0-9]+)?)\s*(?:km/h)?\s*$")


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    element.append(etree.Element("tag", k=key, v=value))


def normalize_speed(value: str) -> str:
    match = SPEED_RE.match(value or "")
    if not match:
        raise ValueError(f"unsupported speed_limit value: {value!r}")
    number = float(match.group(1))
    if number.is_integer():
        return str(int(number))
    return f"{number:g}"


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def reference_errors(root: etree._Element) -> int:
    node_ids = {node.get("id") for node in root.findall("node")}
    way_ids = {way.get("id") for way in root.findall("way")}
    relation_ids = {relation.get("id") for relation in root.findall("relation")}
    errors = 0
    for way in root.findall("way"):
        for nd in way.findall("nd"):
            errors += int(nd.get("ref") not in node_ids)
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            ref = member.get("ref")
            member_type = member.get("type")
            if member_type == "node":
                errors += int(ref not in node_ids)
            elif member_type == "way":
                errors += int(ref not in way_ids)
            elif member_type == "relation":
                errors += int(ref not in relation_ids)
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v39.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v40.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v39")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v40")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", default="40")
    args = parser.parse_args()

    tree = etree.parse(str(args.input), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    rows = []
    counts = Counter()
    changed = 0
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        before = relation_tags.get("speed_limit", "")
        after = normalize_speed(before)
        set_tag(relation, "speed_limit", after)
        counts[after] += 1
        if before != after:
            changed += 1
            rows.append({"lanelet_id": relation.get("id"), "before": before, "after": after})

    missing_refs = reference_errors(root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    write_csv(args.report_dir / "v40_vmb_speed_format.csv", ["lanelet_id", "before", "after"], rows)
    write_csv(
        args.report_dir / "final_validation_summary_v40.csv",
        ["check", "status", "detail"],
        [
            {
                "check": "final_map",
                "status": "PASS",
                "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}",
            },
            {
                "check": "packaged_map",
                "status": "PASS" if sha256(args.output) == sha256(args.package_dir / "lanelet2_map.osm") else "FAIL",
                "detail": f"{(args.package_dir / 'lanelet2_map.osm').relative_to(ROOT)} byte-identical",
            },
            {"check": "osm_references", "status": "PASS" if missing_refs == 0 else "FAIL", "detail": f"missing_reference_count={missing_refs}"},
            {"check": "speed_tag_format", "status": "PASS", "detail": f"numeric_speed_limit_tags_changed={changed}"},
            {"check": "speed_counts", "status": "PASS", "detail": "; ".join(f"{speed}={count}" for speed, count in sorted(counts.items()))},
        ],
    )

    print(f"wrote={args.output.relative_to(ROOT)}")
    print(f"package={args.package_dir.relative_to(ROOT)}")
    print(f"sha256={sha256(args.output)}")
    print(f"changed={changed}")
    print("speed_counts=" + "; ".join(f"{speed}:{count}" for speed, count in sorted(counts.items())))
    print(f"missing_references={missing_refs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
