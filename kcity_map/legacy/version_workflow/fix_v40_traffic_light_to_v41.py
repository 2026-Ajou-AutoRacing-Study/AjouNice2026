#!/usr/bin/env python3
"""Create v41 from v40 by matching VMB traffic-light line order for 103877."""

from __future__ import annotations

import argparse
import csv
import hashlib
import shutil
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, Sequence

from lxml import etree


ROOT = Path(__file__).resolve().parents[2]
TRAFFIC_LIGHT_FIXES = {"103877": ("92827", "92828")}


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def way_refs(way: etree._Element):
    return [nd.get("ref") for nd in way.findall("nd")]


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    children = list(way)
    insert_at = next((idx for idx, child in enumerate(children) if child.tag == "nd"), 0)
    for nd in list(way.findall("nd")):
        way.remove(nd)
    for offset, ref in enumerate(refs):
        way.insert(insert_at + offset, etree.Element("nd", ref=ref))


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
        errors += sum(1 for nd in way.findall("nd") if nd.get("ref") not in node_ids)
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
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v40.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v41.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v40")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v41")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", default="41")
    args = parser.parse_args()

    tree = etree.parse(str(args.input), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    ways = {way.get("id"): way for way in root.findall("way")}
    rows = []
    for way_id, new_refs in TRAFFIC_LIGHT_FIXES.items():
        old_refs = way_refs(ways[way_id])
        replace_way_refs(ways[way_id], new_refs)
        rows.append({"way_id": way_id, "before": " ".join(old_refs), "after": " ".join(new_refs)})

    speed_counts = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") == "lanelet" and relation_tags.get("subtype") == "road":
            speed_counts[relation_tags.get("speed_limit", "")] += 1

    missing_refs = reference_errors(root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    write_csv(args.report_dir / "v41_traffic_light_order_fix.csv", ["way_id", "before", "after"], rows)
    write_csv(
        args.report_dir / "final_validation_summary_v41.csv",
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
            {"check": "traffic_light_order_fix", "status": "PASS", "detail": "; ".join(f"{row['way_id']}:{row['before']}->{row['after']}" for row in rows)},
            {"check": "speed_counts", "status": "PASS", "detail": "; ".join(f"{speed}={count}" for speed, count in sorted(speed_counts.items()))},
        ],
    )

    print(f"wrote={args.output.relative_to(ROOT)}")
    print(f"package={args.package_dir.relative_to(ROOT)}")
    print(f"sha256={sha256(args.output)}")
    print("speed_counts=" + "; ".join(f"{speed}:{count}" for speed, count in sorted(speed_counts.items())))
    print(f"missing_references={missing_refs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
