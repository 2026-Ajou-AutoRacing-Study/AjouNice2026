#!/usr/bin/env python3
"""Transplant selected way node sequences and required nodes from a donor OSM."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Set

from lxml import etree


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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--donor", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--way-ids", required=True, help="Comma-separated way IDs")
    args = parser.parse_args()

    target_tree = etree.parse(str(args.input), etree.XMLParser(remove_blank_text=False))
    donor_tree = etree.parse(str(args.donor), etree.XMLParser(remove_blank_text=False))
    target_root = target_tree.getroot()
    donor_root = donor_tree.getroot()

    target_ways = {way.get("id"): way for way in target_root.findall("way")}
    donor_ways = {way.get("id"): way for way in donor_root.findall("way")}
    target_nodes = {node.get("id"): node for node in target_root.findall("node")}
    donor_nodes = {node.get("id"): node for node in donor_root.findall("node")}

    inserted_nodes = 0
    transplanted = 0
    needed_nodes: Set[str] = set()
    for way_id in [item.strip() for item in args.way_ids.split(",") if item.strip()]:
        donor_way = donor_ways[way_id]
        target_way = target_ways[way_id]
        refs = [nd.get("ref") for nd in donor_way.findall("nd")]
        replace_way_refs(target_way, refs)
        needed_nodes.update(refs)
        transplanted += 1

    first_way = target_root.find("way")
    insert_at = list(target_root).index(first_way) if first_way is not None else len(target_root)
    for node_id in sorted(needed_nodes, key=int):
        if node_id in target_nodes:
            continue
        target_root.insert(insert_at, etree.fromstring(etree.tostring(donor_nodes[node_id])))
        insert_at += 1
        inserted_nodes += 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    target_tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    print(f"transplanted_ways={transplanted}")
    print(f"inserted_nodes={inserted_nodes}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
