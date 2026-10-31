#!/usr/bin/env python3
"""Audit Lanelet2 lanelet polygons for visualization/triangulation issues."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Polygon
from shapely.validation import explain_validity


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs"
DEFAULT_ORIGIN_LAT = 37.2380038082
DEFAULT_ORIGIN_LON = 126.7730819386


def tags(element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def point_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def min_edge(points: List[Tuple[float, float]]) -> float:
    edges = [point_distance(a, b) for a, b in zip(points, points[1:])]
    if len(points) > 2:
        edges.append(point_distance(points[-1], points[0]))
    return min(edges) if edges else 0.0


def duplicate_edges(points: List[Tuple[float, float]], tolerance: float) -> int:
    count = 0
    for a, b in zip(points, points[1:]):
        if point_distance(a, b) <= tolerance:
            count += 1
    return count


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--osm", default=str(ROOT / "lanelet_map/lanelet2_map_v87.osm"))
    parser.add_argument("--output", default=str(ROOT / "reports/polygon_geometry_audit.csv"))
    parser.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    parser.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    parser.add_argument("--duplicate-tolerance-m", type=float, default=0.02)
    parser.add_argument("--small-area-m2", type=float, default=1.0)
    args = parser.parse_args()

    osm_path = Path(args.osm)
    root = etree.parse(str(osm_path)).getroot()
    transformer = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    origin_x, origin_y = transformer.transform(args.origin_lon, args.origin_lat)

    nodes = {}
    for node in root.findall("node"):
        x, y = transformer.transform(float(node.get("lon")), float(node.get("lat")))
        nodes[node.get("id")] = (x - origin_x, y - origin_y)

    ways = {}
    for way in root.findall("way"):
        ways[way.get("id")] = [nodes[nd.get("ref")] for nd in way.findall("nd") if nd.get("ref") in nodes]

    rows = []
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        left_id = ""
        right_id = ""
        for member in relation.findall("member"):
            if member.get("role") == "left":
                left_id = member.get("ref")
            elif member.get("role") == "right":
                right_id = member.get("ref")
        left = ways.get(left_id, [])
        right = ways.get(right_id, [])
        polygon_points = left + list(reversed(right))
        unique_count = len({(round(x, 4), round(y, 4)) for x, y in polygon_points})
        if len(polygon_points) >= 3:
            polygon = Polygon(polygon_points)
            valid = bool(polygon.is_valid)
            area = abs(float(polygon.area))
            reason = "Valid Geometry" if valid else explain_validity(polygon)
        else:
            valid = False
            area = 0.0
            reason = "fewer than 3 polygon points"
        flags = []
        if not valid:
            flags.append("invalid")
        if area < args.small_area_m2:
            flags.append("small_area")
        dup_count = duplicate_edges(polygon_points, args.duplicate_tolerance_m)
        if dup_count:
            flags.append("duplicate_or_tiny_edges")
        rows.append(
            {
                "relation_id": relation.get("id"),
                "subtype": relation_tags.get("subtype", ""),
                "left_way": left_id,
                "right_way": right_id,
                "left_points": len(left),
                "right_points": len(right),
                "polygon_points": len(polygon_points),
                "unique_xy_rounded_0_1mm": unique_count,
                "area_m2": f"{area:.6f}",
                "min_edge_m": f"{min_edge(polygon_points):.6f}",
                "tiny_or_duplicate_edges": dup_count,
                "valid": valid,
                "validity_reason": reason,
                "flags": ";".join(flags),
            }
        )

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as fp:
        fieldnames = [
            "relation_id",
            "subtype",
            "left_way",
            "right_way",
            "left_points",
            "right_points",
            "polygon_points",
            "unique_xy_rounded_0_1mm",
            "area_m2",
            "min_edge_m",
            "tiny_or_duplicate_edges",
            "valid",
            "validity_reason",
            "flags",
        ]
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    invalid = sum(1 for row in rows if row["valid"] is False)
    small = sum(1 for row in rows if "small_area" in row["flags"])
    tiny = sum(1 for row in rows if "duplicate_or_tiny_edges" in row["flags"])
    print(f"wrote {output}")
    print(f"lanelets={len(rows)} invalid={invalid} small_area={small} duplicate_or_tiny_edges={tiny}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
