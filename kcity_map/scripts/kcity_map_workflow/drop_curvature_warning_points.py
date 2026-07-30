#!/usr/bin/env python3
"""Remove non-endpoint way vertices named by lanelet2 curvature warnings."""

from __future__ import annotations

import argparse
import re
from pathlib import Path
from typing import Set

from lxml import etree


def warning_points(log_path: Path) -> Set[str]:
    points: Set[str] = set()
    pattern = re.compile(r"Curvature at point (\d+) is bigger")
    for line in log_path.read_text(errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            points.add(match.group(1))
    return points


def replace_way_refs(way: etree._Element, refs) -> None:
    children = list(way)
    insert_at = 0
    for idx, child in enumerate(children):
        if child.tag == "nd":
            insert_at = idx
            break
    for child in list(way.findall("nd")):
        way.remove(child)
    for offset, ref in enumerate(refs):
        nd = etree.Element("nd")
        nd.set("ref", ref)
        way.insert(insert_at + offset, nd)


def collapse_consecutive(refs):
    collapsed = []
    for ref in refs:
        if not collapsed or collapsed[-1] != ref:
            collapsed.append(ref)
    return collapsed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--map-version")
    args = parser.parse_args()

    points = warning_points(args.log)
    tree = etree.parse(str(args.input), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    if args.map_version:
        meta = root.find("MetaInfo")
        if meta is not None:
            meta.set("map_version", str(args.map_version))

    changed_ways = 0
    removed_refs = 0
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        if len(refs) <= 2:
            continue
        filtered = []
        for idx, ref in enumerate(refs):
            if 0 < idx < len(refs) - 1 and ref in points:
                removed_refs += 1
                continue
            filtered.append(ref)
        filtered = collapse_consecutive(filtered)
        if len(filtered) >= 2 and filtered != refs:
            changed_ways += 1
            replace_way_refs(way, filtered)

    referenced = {nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")}
    removed_nodes = 0
    for node in list(root.findall("node")):
        if node.get("id") not in referenced:
            root.remove(node)
            removed_nodes += 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    print(f"warning_points={len(points)}")
    print(f"changed_ways={changed_ways}")
    print(f"removed_refs={removed_refs}")
    print(f"removed_unreferenced_nodes={removed_nodes}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
