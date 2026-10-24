#!/usr/bin/env python3
"""Repair self-intersecting lanelet polygons by sorting bounds along centerlines.

The script does not invent new coordinates. For each invalid lanelet polygon it
orders existing left/right boundary node references by projection onto the
lanelet centerline. Reordered ways are cloned and referenced only by the repaired
relation, so other lanelets that share the original way are left untouched.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon
from shapely.validation import explain_validity


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs"
DEFAULT_ORIGIN_LAT = 37.2380038082
DEFAULT_ORIGIN_LON = 126.7730819386


def element_tags(element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def polygon_status(points: List[Tuple[float, float]]) -> Tuple[bool, float, str]:
    if len(points) < 3:
        return False, 0.0, "fewer than 3 polygon points"
    polygon = Polygon(points)
    return bool(polygon.is_valid), abs(float(polygon.area)), "Valid Geometry" if polygon.is_valid else explain_validity(polygon)


def point_line_distance(point: Tuple[float, float], start: Tuple[float, float], end: Tuple[float, float]) -> float:
    if start == end:
        return ((point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2) ** 0.5
    numerator = abs(
        (end[1] - start[1]) * point[0]
        - (end[0] - start[0]) * point[1]
        + end[0] * start[1]
        - end[1] * start[0]
    )
    denominator = ((end[1] - start[1]) ** 2 + (end[0] - start[0]) ** 2) ** 0.5
    return numerator / denominator


def clone_way(original, new_id: int, refs: List[str]):
    clone = etree.Element("way", dict(original.attrib))
    clone.set("id", str(new_id))
    for ref in refs:
        etree.SubElement(clone, "nd", ref=ref)
    for tag in original.findall("tag"):
        clone.append(etree.Element("tag", dict(tag.attrib)))
    return clone


def insert_way(root, way_element) -> None:
    children = list(root)
    insert_at = len(children)
    for idx, child in enumerate(children):
        if child.tag == "relation":
            insert_at = idx
            break
    root.insert(insert_at, way_element)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report", default=str(ROOT / "reports/bound_sort_repair.csv"))
    parser.add_argument("--map-version", required=True)
    parser.add_argument("--origin-lat", type=float, default=DEFAULT_ORIGIN_LAT)
    parser.add_argument("--origin-lon", type=float, default=DEFAULT_ORIGIN_LON)
    parser.add_argument("--min-valid-area-m2", type=float, default=1.0)
    parser.add_argument("--fallback-rdp-tolerances", default="0.02,0.05,0.10,0.20,0.50")
    args = parser.parse_args()
    fallback_tolerances = [float(value) for value in args.fallback_rdp_tolerances.split(",") if value.strip()]

    tree = etree.parse(str(Path(args.input)))
    root = tree.getroot()
    transformer = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    origin_x, origin_y = transformer.transform(args.origin_lon, args.origin_lat)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", str(args.map_version))

    node_xy: Dict[str, Tuple[float, float]] = {}
    for node in root.findall("node"):
        x, y = transformer.transform(float(node.get("lon")), float(node.get("lat")))
        node_xy[node.get("id")] = (x - origin_x, y - origin_y)

    way_elements = {way.get("id"): way for way in root.findall("way")}
    way_refs = {way_id: [nd.get("ref") for nd in way.findall("nd")] for way_id, way in way_elements.items()}
    next_way_id = max(int(way_id) for way_id in way_elements) + 1

    def refs_to_points(refs: List[str]) -> List[Tuple[float, float]]:
        return [node_xy[ref] for ref in refs if ref in node_xy]

    def sorted_refs_by_centerline(refs: List[str], center_refs: List[str]) -> List[str]:
        center_points = refs_to_points(center_refs)
        if len(center_points) < 2:
            return refs
        line = LineString(center_points)
        keyed = []
        for idx, ref in enumerate(refs):
            if ref not in node_xy:
                continue
            keyed.append((line.project(Point(node_xy[ref])), idx, ref))
        keyed.sort()
        output = []
        for _, _, ref in keyed:
            if not output or output[-1] != ref:
                output.append(ref)
        return output

    def rdp_refs(refs: List[str], tolerance: float) -> List[str]:
        if len(refs) <= 2:
            return refs
        points = refs_to_points(refs)

        def recurse(lo: int, hi: int) -> List[str]:
            best_distance = -1.0
            best_index = None
            for idx in range(lo + 1, hi):
                dist = point_line_distance(points[idx], points[lo], points[hi])
                if dist > best_distance:
                    best_distance = dist
                    best_index = idx
            if best_index is not None and best_distance > tolerance:
                return recurse(lo, best_index)[:-1] + recurse(best_index, hi)
            return [refs[lo], refs[hi]]

        output = recurse(0, len(refs) - 1)
        deduped = []
        for ref in output:
            if not deduped or deduped[-1] != ref:
                deduped.append(ref)
        return deduped

    rows = []
    for relation in root.findall("relation"):
        relation_tags = element_tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue

        members = {member.get("role"): member for member in relation.findall("member")}
        left_member = members.get("left")
        right_member = members.get("right")
        center_member = members.get("centerline")
        if left_member is None or right_member is None or center_member is None:
            continue

        left_id = left_member.get("ref")
        right_id = right_member.get("ref")
        center_id = center_member.get("ref")
        left_refs = way_refs.get(left_id, [])
        right_refs = way_refs.get(right_id, [])
        center_refs = way_refs.get(center_id, [])
        current_points = refs_to_points(left_refs) + list(reversed(refs_to_points(right_refs)))
        current_valid, current_area, current_reason = polygon_status(current_points)
        if current_valid:
            continue

        sorted_left = sorted_refs_by_centerline(left_refs, center_refs)
        sorted_right = sorted_refs_by_centerline(right_refs, center_refs)
        final_left = sorted_left
        final_right = sorted_right
        repair_method = "centerline_sort"
        repair_tolerance = ""
        repaired_points = refs_to_points(final_left) + list(reversed(refs_to_points(final_right)))
        repaired_valid, repaired_area, repaired_reason = polygon_status(repaired_points)
        repaired = repaired_valid and repaired_area >= args.min_valid_area_m2
        if not repaired:
            for tolerance in fallback_tolerances:
                candidate_left = rdp_refs(sorted_left, tolerance)
                candidate_right = rdp_refs(sorted_right, tolerance)
                candidate_points = refs_to_points(candidate_left) + list(reversed(refs_to_points(candidate_right)))
                candidate_valid, candidate_area, candidate_reason = polygon_status(candidate_points)
                if candidate_valid and candidate_area >= args.min_valid_area_m2:
                    final_left = candidate_left
                    final_right = candidate_right
                    repair_method = "centerline_sort_rdp"
                    repair_tolerance = f"{tolerance:.3f}"
                    repaired_valid = candidate_valid
                    repaired_area = candidate_area
                    repaired_reason = candidate_reason
                    repaired = True
                    break
        new_left_id = ""
        new_right_id = ""
        if repaired:
            if final_left != left_refs:
                new_left_id = str(next_way_id)
                next_way_id += 1
                insert_way(root, clone_way(way_elements[left_id], int(new_left_id), final_left))
                left_member.set("ref", new_left_id)
            if final_right != right_refs:
                new_right_id = str(next_way_id)
                next_way_id += 1
                insert_way(root, clone_way(way_elements[right_id], int(new_right_id), final_right))
                right_member.set("ref", new_right_id)
        rows.append(
            {
                "relation_id": relation.get("id"),
                "subtype": relation_tags.get("subtype", ""),
                "left_way": left_id,
                "right_way": right_id,
                "new_left_way": new_left_id,
                "new_right_way": new_right_id,
                "current_area_m2": f"{current_area:.6f}",
                "current_reason": current_reason,
                "repaired": repaired,
                "repair_method": repair_method if repaired else "",
                "repair_tolerance_m": repair_tolerance,
                "repaired_area_m2": f"{repaired_area:.6f}",
                "repaired_reason": repaired_reason,
                "left_points": len(left_refs),
                "right_points": len(right_refs),
                "sorted_left_points": len(sorted_left),
                "sorted_right_points": len(sorted_right),
                "final_left_points": len(final_left),
                "final_right_points": len(final_right),
            }
        )

    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(Path(args.output)), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    report = Path(args.report)
    report.parent.mkdir(parents=True, exist_ok=True)
    with report.open("w", newline="") as fp:
        fieldnames = [
            "relation_id",
            "subtype",
            "left_way",
            "right_way",
            "new_left_way",
            "new_right_way",
            "current_area_m2",
            "current_reason",
            "repaired",
            "repair_method",
            "repair_tolerance_m",
            "repaired_area_m2",
            "repaired_reason",
            "left_points",
            "right_points",
            "sorted_left_points",
            "sorted_right_points",
            "final_left_points",
            "final_right_points",
        ]
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    repaired_count = sum(1 for row in rows if row["repaired"])
    print(f"wrote {Path(args.output)}")
    print(f"wrote {report}")
    print(f"invalid_lanelets_seen={len(rows)} repaired={repaired_count} unrepaired={len(rows) - repaired_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
