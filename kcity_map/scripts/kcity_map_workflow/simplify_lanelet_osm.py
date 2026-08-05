#!/usr/bin/env python3
"""Simplify selected Lanelet2 OSM ways using existing node vertices only."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from lxml import etree
from pyproj import Transformer
from scipy.spatial import cKDTree


def parse_tags(elem: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in elem.findall("tag") if tag.get("k")}


def point_line_distance(point: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def rdp_indexes(points: Sequence[Tuple[float, float]], tolerance: float) -> List[int]:
    if len(points) <= 2:
        return list(range(len(points)))

    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        max_dist = -1.0
        max_idx = None
        a = points[start]
        b = points[end]
        for idx in range(start + 1, end):
            dist = point_line_distance(points[idx], a, b)
            if dist > max_dist:
                max_dist = dist
                max_idx = idx
        if max_idx is not None and max_dist > tolerance:
            keep.add(max_idx)
            stack.append((start, max_idx))
            stack.append((max_idx, end))
    return sorted(keep)


def remove_tiny_segments(
    refs: Sequence[str], points: Sequence[Tuple[float, float]], min_segment: float
) -> Tuple[List[str], List[Tuple[float, float]]]:
    if len(refs) <= 2 or min_segment <= 0.0:
        return list(refs), list(points)
    kept_refs = [refs[0]]
    kept_points = [points[0]]
    for ref, point in zip(refs[1:-1], points[1:-1]):
        if math.hypot(point[0] - kept_points[-1][0], point[1] - kept_points[-1][1]) >= min_segment:
            kept_refs.append(ref)
            kept_points.append(point)
    kept_refs.append(refs[-1])
    kept_points.append(points[-1])
    return kept_refs, kept_points


def replace_way_refs(way: etree._Element, refs: Iterable[str]) -> None:
    insert_at = 0
    children = list(way)
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


class UnionFind:
    def __init__(self, ids: Sequence[str]) -> None:
        self.parent = {node_id: node_id for node_id in ids}

    def find(self, node_id: str) -> str:
        parent = self.parent[node_id]
        if parent != node_id:
            self.parent[node_id] = self.find(parent)
        return self.parent[node_id]

    def union(self, a: str, b: str) -> None:
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return
        if int(ra) <= int(rb):
            self.parent[rb] = ra
        else:
            self.parent[ra] = rb


def merge_close_nodes(
    root: etree._Element, node_xy: Dict[str, Tuple[float, float]], threshold_m: float
) -> Tuple[Dict[str, str], int]:
    if threshold_m <= 0.0:
        return {node_id: node_id for node_id in node_xy}, 0

    ids = list(node_xy)
    coords = [node_xy[node_id] for node_id in ids]
    tree = cKDTree(coords)
    pairs = tree.query_pairs(threshold_m)
    uf = UnionFind(ids)
    for ia, ib in pairs:
        uf.union(ids[ia], ids[ib])

    ref_map = {node_id: uf.find(node_id) for node_id in ids}
    merged = sum(1 for node_id, canonical in ref_map.items() if node_id != canonical)
    return ref_map, merged


def add_missing_tag(elem: etree._Element, key: str, value: str) -> bool:
    if elem.xpath(f'tag[@k="{key}"]'):
        return False
    tag = etree.Element("tag")
    tag.set("k", key)
    tag.set("v", value)
    elem.append(tag)
    return True


def add_road_lanelet_tags(root: etree._Element) -> int:
    added = 0
    for relation in root.findall("relation"):
        tags = parse_tags(relation)
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        if add_missing_tag(relation, "one_way", "yes"):
            added += 1
        if add_missing_tag(relation, "participant:vehicle", "yes"):
            added += 1
        if add_missing_tag(relation, "location", "urban"):
            added += 1
    return added


def rewrite_and_prune_nodes(root: etree._Element, ref_map: Dict[str, str]) -> Tuple[int, int, int]:
    rewritten_refs = 0
    collapsed_refs = 0
    referenced = set()
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        if not refs:
            continue
        mapped = [ref_map.get(ref, ref) for ref in refs]
        rewritten_refs += sum(1 for old, new in zip(refs, mapped) if old != new)
        collapsed = []
        for ref in mapped:
            if not collapsed or collapsed[-1] != ref:
                collapsed.append(ref)
        if len(collapsed) < 2 and len(mapped) >= 2:
            collapsed = [mapped[0], mapped[-1]]
        collapsed_refs += len(mapped) - len(collapsed)
        replace_way_refs(way, collapsed)
        referenced.update(collapsed)

    removed_nodes = 0
    for node in list(root.findall("node")):
        if node.get("id") not in referenced:
            root.remove(node)
            removed_nodes += 1
    return rewritten_refs, collapsed_refs, removed_nodes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--summary-csv", required=True, type=Path)
    parser.add_argument("--map-version", default="24")
    parser.add_argument("--tolerance-m", type=float, default=0.05)
    parser.add_argument("--min-segment-m", type=float, default=0.03)
    parser.add_argument("--merge-close-nodes-m", type=float, default=0.0)
    parser.add_argument("--add-road-tags", action="store_true")
    parser.add_argument(
        "--way-types",
        default="road_border,virtual,line_thin",
        help="Comma-separated OSM way type values to simplify.",
    )
    args = parser.parse_args()

    tree = etree.parse(str(args.input), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    transformer = Transformer.from_crs(
        "EPSG:4326", "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs", always_xy=True
    )

    for meta in root.findall("MetaInfo"):
        meta.set("map_version", args.map_version)

    node_xy: Dict[str, Tuple[float, float]] = {}
    for node in root.findall("node"):
        lon = float(node.get("lon"))
        lat = float(node.get("lat"))
        node_xy[node.get("id")] = transformer.transform(lon, lat)

    added_tags = add_road_lanelet_tags(root) if args.add_road_tags else 0
    ref_map: Optional[Dict[str, str]] = None
    merged_nodes = 0
    rewritten_refs = 0
    collapsed_refs = 0
    removed_nodes = 0
    if args.merge_close_nodes_m > 0.0:
        ref_map, merged_nodes = merge_close_nodes(root, node_xy, args.merge_close_nodes_m)
        rewritten_refs, collapsed_refs, removed_nodes = rewrite_and_prune_nodes(root, ref_map)
        node_xy = {node_id: xy for node_id, xy in node_xy.items() if ref_map.get(node_id, node_id) == node_id}

    target_types = {value.strip() for value in args.way_types.split(",") if value.strip()}
    rows = []
    total_original = 0
    total_final = 0
    changed = 0

    for way in root.findall("way"):
        tags = parse_tags(way)
        if tags.get("type") not in target_types:
            continue
        refs = [nd.get("ref") for nd in way.findall("nd")]
        if len(refs) <= 2:
            continue
        points = [node_xy[ref] for ref in refs if ref in node_xy]
        if len(points) != len(refs):
            continue
        filtered_refs, filtered_points = remove_tiny_segments(refs, points, args.min_segment_m)
        keep = rdp_indexes(filtered_points, args.tolerance_m)
        simplified_refs = [filtered_refs[idx] for idx in keep]
        if len(simplified_refs) < 2:
            simplified_refs = [refs[0], refs[-1]]
        total_original += len(refs)
        total_final += len(simplified_refs)
        if simplified_refs != refs:
            changed += 1
            replace_way_refs(way, simplified_refs)
            rows.append(
                {
                    "way_id": way.get("id"),
                    "type": tags.get("type", ""),
                    "subtype": tags.get("subtype", ""),
                    "original_points": len(refs),
                    "filtered_points": len(filtered_refs),
                    "simplified_points": len(simplified_refs),
                    "removed_points": len(refs) - len(simplified_refs),
                }
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.summary_csv.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    with args.summary_csv.open("w", newline="") as fp:
        writer = csv.DictWriter(
            fp,
            fieldnames=[
                "way_id",
                "type",
                "subtype",
                "original_points",
                "filtered_points",
                "simplified_points",
                "removed_points",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"changed_ways={changed}")
    print(f"added_road_tags={added_tags}")
    print(f"merged_nodes={merged_nodes}")
    print(f"rewritten_node_refs={rewritten_refs}")
    print(f"collapsed_duplicate_refs={collapsed_refs}")
    print(f"removed_unreferenced_nodes={removed_nodes}")
    print(f"target_points_before={total_original}")
    print(f"target_points_after={total_final}")
    print(f"removed_points={total_original - total_final}")
    print(f"output={args.output}")
    print(f"summary_csv={args.summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
