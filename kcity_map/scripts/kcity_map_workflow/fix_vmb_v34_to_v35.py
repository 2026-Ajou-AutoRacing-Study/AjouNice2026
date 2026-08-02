#!/usr/bin/env python3
"""Create v35 from v34 by fixing VMB centerline and traffic-light warnings."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

from lxml import etree
from pyproj import Transformer
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]

# VMB reports these lanelets as having centerline connection warnings.
# The fix makes the flagged successor/predecessor centerline endpoints share
# the same node as the adjacent centerline endpoint.
CENTERLINE_CONNECTION_FIXES = (
    ("201185", "start", "99014", "connect_after_200195"),
    ("201180", "start", "99391", "connect_after_200224"),
    ("201181", "start", "99391", "connect_after_200224"),
    ("201174", "start", "130909", "connect_after_200225"),
    ("201182", "start", "99410", "connect_after_200226"),
    ("201175", "start", "100198", "connect_after_200300"),
    ("201176", "start", "100213", "connect_after_200304"),
    ("201177", "end", "100020", "connect_before_200293"),
    ("201179", "end", "99417", "connect_before_200228"),
    ("201183", "end", "101180", "connect_before_200343"),
    ("201184", "end", "99667", "connect_before_200279"),
)

# VMB reports these lanelets as having internal centerline points outside the
# lanelet polygon. Each offending internal node is cloned for that centerline
# way only, then moved just inside the polygon.
CENTERLINE_INSIDE_TARGET_LANELETS = (
    "200225",
    "200269",
    "200304",
    "201177",
    "201179",
    "201180",
    "201181",
    "201182",
    "201184",
    "201185",
)

# 103817 and 103839 are already rightward relative to their referenced lanelet
# centerlines in v34. 103877 is still leftward, so only it is reversed for v35.
TRAFFIC_LIGHT_REF_FIXES = {
    "103877": ("92828", "92827"),
}


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    nd_elements = way.findall("nd")
    insert_at = list(way).index(nd_elements[0]) if nd_elements else 0
    for nd in nd_elements:
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


def node_ele(node: etree._Element) -> float:
    return float(tags(node).get("ele", "0"))


def set_node_ele(node: etree._Element, ele: float) -> None:
    ele_tag = None
    for tag in node.findall("tag"):
        if tag.get("k") == "ele":
            ele_tag = tag
            break
    if ele_tag is None:
        node.append(etree.Element("tag", k="ele", v=f"{ele:.9f}"))
    else:
        ele_tag.set("v", f"{ele:.9f}")


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    dz = (a[2] - b[2]) if len(a) >= 3 and len(b) >= 3 else 0.0
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + dz * dz)


def remove_duplicate_nodes(root: etree._Element) -> List[Dict[str, object]]:
    seen: Dict[str, etree._Element] = {}
    rows = []
    for node in list(root.findall("node")):
        node_id = node.get("id")
        previous = seen.get(node_id)
        if previous is None:
            seen[node_id] = node
            continue
        same_lat_lon = previous.get("lat") == node.get("lat") and previous.get("lon") == node.get("lon")
        same_tags = tags(previous) == tags(node)
        if not same_lat_lon or not same_tags:
            raise ValueError(f"duplicate node id {node_id} has different coordinates or tags")
        root.remove(node)
        rows.append(
            {
                "action": "remove_duplicate_node",
                "target": node_id,
                "before": f"{node.get('lat')},{node.get('lon')}",
                "after": "removed duplicate copy",
                "detail": "identical duplicate node element",
            }
        )
    return rows


def remove_unreferenced_nodes(root: etree._Element) -> int:
    referenced = {nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")}
    referenced.update(
        member.get("ref")
        for relation in root.findall("relation")
        for member in relation.findall("member")
        if member.get("type") == "node"
    )
    removed = 0
    for node in list(root.findall("node")):
        if node.get("id") not in referenced:
            root.remove(node)
            removed += 1
    return removed


def append_node(
    root: etree._Element,
    insert_at: int,
    node_id: str,
    point: Point3,
    transformer_to_wgs: Transformer,
) -> etree._Element:
    lon, lat = transformer_to_wgs.transform(point[0], point[1])
    node = etree.Element("node", id=node_id, visible="true", version="1", lat=f"{lat:.14f}", lon=f"{lon:.14f}")
    set_node_ele(node, point[2])
    root.insert(insert_at, node)
    return node


def build_lanelets(root: etree._Element) -> Dict[str, Dict[str, str]]:
    lanelets: Dict[str, Dict[str, str]] = {}
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet" or tags(relation).get("subtype") != "road":
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way"
        }
        if {"left", "right", "centerline"}.issubset(members):
            lanelets[relation.get("id")] = members
    return lanelets


def make_coord(nodes: Dict[str, etree._Element], transformer_to_utm: Transformer) -> Dict[str, Point3]:
    coord: Dict[str, Point3] = {}
    for node_id, node in nodes.items():
        x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        coord[node_id] = (x, y, node_ele(node))
    return coord


def polygon_for_lanelet(
    lanelet_id: str,
    lanelets: Dict[str, Dict[str, str]],
    ways: Dict[str, etree._Element],
    coord: Dict[str, Point3],
) -> Polygon:
    lanelet = lanelets[lanelet_id]
    left = [coord[ref][:2] for ref in way_refs(ways[lanelet["left"]])]
    right = [coord[ref][:2] for ref in way_refs(ways[lanelet["right"]])]
    return Polygon(left + list(reversed(right)))


def apply_connection_fixes(
    lanelets: Dict[str, Dict[str, str]],
    ways: Dict[str, etree._Element],
) -> List[Dict[str, object]]:
    rows = []
    for lanelet_id, end_name, shared_node_id, detail in CENTERLINE_CONNECTION_FIXES:
        center_way_id = lanelets[lanelet_id]["centerline"]
        center_refs = way_refs(ways[center_way_id])
        old_refs = list(center_refs)
        if end_name == "start":
            center_refs[0] = shared_node_id
        elif end_name == "end":
            center_refs[-1] = shared_node_id
        else:
            raise ValueError(f"unknown end name {end_name}")
        replace_way_refs(ways[center_way_id], center_refs)
        rows.append(
            {
                "action": "connect_centerline_endpoint",
                "target": lanelet_id,
                "before": " ".join(old_refs),
                "after": " ".join(center_refs),
                "detail": f"{detail}; center_way_id={center_way_id}; changed_{end_name}={shared_node_id}",
            }
        )
    return rows


def apply_traffic_light_fixes(ways: Dict[str, etree._Element]) -> List[Dict[str, object]]:
    rows = []
    for way_id, new_refs in TRAFFIC_LIGHT_REF_FIXES.items():
        old_refs = way_refs(ways[way_id])
        replace_way_refs(ways[way_id], new_refs)
        rows.append(
            {
                "action": "set_traffic_light_left_to_right",
                "target": way_id,
                "before": " ".join(old_refs),
                "after": " ".join(new_refs),
                "detail": "VMB left-to-right order relative to lanelet travel direction",
            }
        )
    return rows


def clone_outside_centerline_points(
    root: etree._Element,
    lanelets: Dict[str, Dict[str, str]],
    ways: Dict[str, etree._Element],
    nodes: Dict[str, etree._Element],
    coord: Dict[str, Point3],
    transformer_to_wgs: Transformer,
    inward_offset_m: float,
) -> List[Dict[str, object]]:
    rows = []
    next_node_id = max(int(node_id) for node_id in nodes) + 1
    first_way = root.find("way")
    insert_at = list(root).index(first_way) if first_way is not None else len(root)

    for lanelet_id in CENTERLINE_INSIDE_TARGET_LANELETS:
        center_way_id = lanelets[lanelet_id]["centerline"]
        center_refs = way_refs(ways[center_way_id])
        polygon = polygon_for_lanelet(lanelet_id, lanelets, ways, coord)
        representative = polygon.representative_point()

        for idx, old_ref in list(enumerate(center_refs[1:-1], start=1)):
            old_point = Point(coord[old_ref][:2])
            if polygon.covers(old_point):
                continue

            nearest_on_boundary = nearest_points(polygon.boundary, old_point)[0]
            vx = representative.x - nearest_on_boundary.x
            vy = representative.y - nearest_on_boundary.y
            norm = math.hypot(vx, vy) or 1.0
            offset = inward_offset_m
            candidate = None
            for _ in range(16):
                candidate_point = (
                    nearest_on_boundary.x + vx / norm * offset,
                    nearest_on_boundary.y + vy / norm * offset,
                    coord[old_ref][2],
                )
                if polygon.covers(Point(candidate_point[:2])):
                    candidate = candidate_point
                    break
                offset *= 0.5
            if candidate is None:
                raise ValueError(f"could not move node {old_ref} inside lanelet {lanelet_id}")

            new_ref = str(next_node_id)
            next_node_id += 1
            append_node(root, insert_at, new_ref, candidate, transformer_to_wgs)
            insert_at += 1
            nodes[new_ref] = root.find(f"node[@id='{new_ref}']")
            coord[new_ref] = candidate
            center_refs[idx] = new_ref

            rows.append(
                {
                    "action": "clone_centerline_point_inside_polygon",
                    "target": lanelet_id,
                    "before": f"center_way_id={center_way_id}; idx={idx}; node={old_ref}",
                    "after": f"node={new_ref}",
                    "detail": f"outside_before_m={old_point.distance(polygon):.9f}; inward_offset_m={offset:.9f}",
                }
            )
        replace_way_refs(ways[center_way_id], center_refs)
    return rows


def find_close_pairs(nodes_xyz: Dict[str, Point3], threshold_m: float) -> List[Tuple[str, str, float]]:
    cell_size = threshold_m
    grid: Dict[Tuple[int, int], List[str]] = defaultdict(list)
    pairs = []
    for node_id, point in nodes_xyz.items():
        cell = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other_id in grid[(cell[0] + dx, cell[1] + dy)]:
                    dist = distance(point, nodes_xyz[other_id])
                    if dist < threshold_m:
                        first, second = sorted((node_id, other_id), key=int)
                        pairs.append((first, second, dist))
        grid[cell].append(node_id)
    unique = {}
    for first, second, dist in pairs:
        unique[(first, second)] = min(dist, unique.get((first, second), dist))
    return [(first, second, dist) for (first, second), dist in sorted(unique.items(), key=lambda item: (int(item[0][0]), int(item[0][1])))]


def audit_centerlines(
    lanelets: Dict[str, Dict[str, str]],
    ways: Dict[str, etree._Element],
    coord: Dict[str, Point3],
    gap_threshold_m: float,
) -> Tuple[List[Dict[str, object]], int, int]:
    starts: Dict[Tuple[str, str], List[str]] = defaultdict(list)
    ends: Dict[Tuple[str, str], List[str]] = defaultdict(list)
    for lanelet_id, lanelet in lanelets.items():
        left_refs = way_refs(ways[lanelet["left"]])
        right_refs = way_refs(ways[lanelet["right"]])
        starts[tuple(sorted((left_refs[0], right_refs[0]), key=int))].append(lanelet_id)
        ends[tuple(sorted((left_refs[-1], right_refs[-1]), key=int))].append(lanelet_id)

    rows = []
    lanelets_with_outside = 0
    bad_successor_edges = 0
    for lanelet_id in sorted(lanelets, key=int):
        lanelet = lanelets[lanelet_id]
        center_refs = way_refs(ways[lanelet["centerline"]])
        polygon = polygon_for_lanelet(lanelet_id, lanelets, ways, coord)

        outside_nodes = []
        for idx, ref in enumerate(center_refs[1:-1], start=1):
            point = Point(coord[ref][:2])
            if not polygon.covers(point):
                outside_nodes.append(f"{idx}:{ref}:{point.distance(polygon):.6f}")
        if outside_nodes:
            lanelets_with_outside += 1

        left_refs = way_refs(ways[lanelet["left"]])
        right_refs = way_refs(ways[lanelet["right"]])
        successor_key = tuple(sorted((left_refs[-1], right_refs[-1]), key=int))
        bad_successors = []
        for successor_id in starts.get(successor_key, []):
            successor_refs = way_refs(ways[lanelets[successor_id]["centerline"]])
            gap = distance(coord[center_refs[-1]], coord[successor_refs[0]])
            if gap > gap_threshold_m:
                bad_successors.append(f"{successor_id}:{gap:.6f}")
        bad_successor_edges += len(bad_successors)

        rows.append(
            {
                "lanelet_id": lanelet_id,
                "center_way_id": lanelet["centerline"],
                "outside_internal_center_nodes": " ".join(outside_nodes),
                "bad_successor_centerline_gaps": " ".join(bad_successors),
            }
        )

    return rows, lanelets_with_outside, bad_successor_edges


def audit_traffic_light_order(
    root: etree._Element,
    lanelets: Dict[str, Dict[str, str]],
    ways: Dict[str, etree._Element],
    coord: Dict[str, Point3],
) -> Tuple[List[Dict[str, object]], int]:
    regulatory_by_traffic_light = defaultdict(list)
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "regulatory_element" or relation_tags.get("subtype") != "traffic_light":
            continue
        for member in relation.findall("member"):
            if member.get("type") == "way" and member.get("role") == "refers":
                regulatory_by_traffic_light[member.get("ref")].append(relation.get("id"))

    lanelet_by_reg = defaultdict(list)
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet":
            continue
        for member in relation.findall("member"):
            if member.get("type") == "relation" and member.get("role") == "regulatory_element":
                lanelet_by_reg[member.get("ref")].append(relation.get("id"))

    rows = []
    bad_count = 0
    for traffic_light_way_id in ("103817", "103839", "103877"):
        tl_refs = way_refs(ways[traffic_light_way_id])
        tl_start = coord[tl_refs[0]]
        tl_end = coord[tl_refs[-1]]
        tl_vector = (tl_end[0] - tl_start[0], tl_end[1] - tl_start[1])
        for reg_id in regulatory_by_traffic_light[traffic_light_way_id]:
            for lanelet_id in lanelet_by_reg[reg_id]:
                lanelet = lanelets.get(lanelet_id)
                if lanelet is None:
                    continue
                center_refs = way_refs(ways[lanelet["centerline"]])
                start = coord[center_refs[0]]
                end = coord[center_refs[-1]]
                travel = (end[0] - start[0], end[1] - start[1])
                left = (-travel[1], travel[0])
                dot_left = tl_vector[0] * left[0] + tl_vector[1] * left[1]
                is_rightward = dot_left < 0
                if not is_rightward:
                    bad_count += 1
                rows.append(
                    {
                        "traffic_light_way_id": traffic_light_way_id,
                        "regulatory_element_id": reg_id,
                        "lanelet_id": lanelet_id,
                        "refs": " ".join(tl_refs),
                        "dot_left": f"{dot_left:.9f}",
                        "order": "rightward" if is_rightward else "leftward",
                    }
                )
    return rows, bad_count


def count_duplicate_ids(root: etree._Element) -> Dict[str, List[Tuple[str, int]]]:
    duplicates = {}
    for kind in ("node", "way", "relation"):
        counts = Counter(element.get("id") for element in root.findall(kind))
        duplicates[kind] = [(item_id, count) for item_id, count in counts.items() if count > 1]
    return duplicates


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v34.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v35.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v34")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v35")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", type=str, default="35")
    parser.add_argument("--inward-offset-m", type=float, default=0.05)
    parser.add_argument("--gap-threshold-m", type=float, default=0.001)
    parser.add_argument("--close-threshold-m", type=float, default=0.05)
    parser.add_argument(
        "--lanelet2-validate-log",
        type=Path,
        default=ROOT / "reports/validation_logs/lanelet2_map_v35.log",
    )
    args = parser.parse_args()

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.input), parser_xml)
    root = tree.getroot()
    transformer_to_utm = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    transformer_to_wgs = Transformer.from_crs(UTM52, "EPSG:4326", always_xy=True)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    action_rows = remove_duplicate_nodes(root)
    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    lanelets = build_lanelets(root)
    coord = make_coord(nodes, transformer_to_utm)

    action_rows.extend(apply_connection_fixes(lanelets, ways))
    action_rows.extend(apply_traffic_light_fixes(ways))
    action_rows.extend(
        clone_outside_centerline_points(
            root,
            lanelets,
            ways,
            nodes,
            coord,
            transformer_to_wgs,
            args.inward_offset_m,
        )
    )

    removed_unreferenced = remove_unreferenced_nodes(root)

    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    lanelets = build_lanelets(root)
    coord = make_coord(nodes, transformer_to_utm)

    duplicate_ids = count_duplicate_ids(root)
    close_pairs = find_close_pairs(coord, args.close_threshold_m)
    centerline_rows, lanelets_with_outside, bad_successor_edges = audit_centerlines(
        lanelets,
        ways,
        coord,
        args.gap_threshold_m,
    )
    traffic_light_rows, bad_traffic_light_orders = audit_traffic_light_order(root, lanelets, ways, coord)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    write_csv(
        args.report_dir / "v35_vmb_warning_fix.csv",
        ["action", "target", "before", "after", "detail"],
        action_rows
        + [
            {
                "action": "remove_unreferenced_nodes",
                "target": "all",
                "before": "",
                "after": "",
                "detail": f"removed_unreferenced={removed_unreferenced}",
            }
        ],
    )
    write_csv(
        args.report_dir / "v35_centerline_vmb_audit.csv",
        ["lanelet_id", "center_way_id", "outside_internal_center_nodes", "bad_successor_centerline_gaps"],
        centerline_rows,
    )
    write_csv(
        args.report_dir / "v35_traffic_light_order_audit.csv",
        ["traffic_light_way_id", "regulatory_element_id", "lanelet_id", "refs", "dot_left", "order"],
        traffic_light_rows,
    )

    relation_counts = Counter()
    way_counts = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        relation_counts[f"{relation_tags.get('type', '')}/{relation_tags.get('subtype', '')}"] += 1
    for way in root.findall("way"):
        way_tags = tags(way)
        way_counts[f"{way_tags.get('type', '')}/{way_tags.get('subtype', '')}"] += 1

    duplicate_detail = "; ".join(
        f"{kind}:{','.join(f'{item_id}x{count}' for item_id, count in values)}"
        for kind, values in duplicate_ids.items()
        if values
    )
    if not duplicate_detail:
        duplicate_detail = "no duplicate ids"

    summary_rows = [
        {"check": "final_map", "status": "PASS", "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}"},
        {
            "check": "packaged_map",
            "status": "PASS" if sha256(args.output) == sha256(args.package_dir / "lanelet2_map.osm") else "FAIL",
            "detail": f"{(args.package_dir / 'lanelet2_map.osm').relative_to(ROOT)} byte-identical",
        },
        {
            "check": "vmb_centerline_connectivity",
            "status": "PASS" if bad_successor_edges == 0 else "FAIL",
            "detail": f"bad_successor_edges_gt_{args.gap_threshold_m:.3f}m={bad_successor_edges}",
        },
        {
            "check": "vmb_centerline_inside_lanelet",
            "status": "PASS" if lanelets_with_outside == 0 else "FAIL",
            "detail": f"lanelets_with_internal_center_points_outside={lanelets_with_outside}",
        },
        {
            "check": "vmb_traffic_light_left_to_right",
            "status": "PASS" if bad_traffic_light_orders == 0 else "FAIL",
            "detail": f"bad_checked_orders={bad_traffic_light_orders}",
        },
        {
            "check": "duplicate_osm_ids",
            "status": "PASS" if not any(duplicate_ids.values()) else "FAIL",
            "detail": duplicate_detail,
        },
        {
            "check": "points_too_close_internal",
            "status": "PASS" if not close_pairs else "FAIL",
            "detail": f"close_pairs_3d_lt_{args.close_threshold_m:.3f}m={len(close_pairs)}",
        },
        {
            "check": "osm_counts",
            "status": "PASS",
            "detail": (
                f"nodes={len(root.findall('node'))} ways={len(root.findall('way'))} relations={len(root.findall('relation'))} "
                f"road_lanelets={relation_counts.get('lanelet/road', 0)} "
                f"crosswalk_lanelets={relation_counts.get('lanelet/crosswalk', 0)} "
                f"traffic_light_regs={relation_counts.get('regulatory_element/traffic_light', 0)} "
                f"stop_line_ways={way_counts.get('line_thin/stop_line', 0)} "
                f"removed_unreferenced={removed_unreferenced}"
            ),
        },
    ]
    if args.lanelet2_validate_log.exists():
        validate_text = args.lanelet2_validate_log.read_text(errors="replace")
        validate_line = next((line.strip() for line in validate_text.splitlines() if line.strip()), "")
        validate_status = "PASS" if "0 issues found" in validate_text else "FAIL"
        summary_rows.append(
            {
                "check": "static_lanelet2_validate",
                "status": validate_status,
                "detail": f"{validate_line}; validation_logs/{args.lanelet2_validate_log.name}",
            }
        )
    write_csv(args.report_dir / "final_validation_summary_v35.csv", ["check", "status", "detail"], summary_rows)

    print(f"wrote={args.output.relative_to(ROOT)}")
    print(f"package={args.package_dir.relative_to(ROOT)}")
    print(f"sha256={sha256(args.output)}")
    print(f"bad_successor_edges={bad_successor_edges}")
    print(f"lanelets_with_internal_center_points_outside={lanelets_with_outside}")
    print(f"bad_traffic_light_orders={bad_traffic_light_orders}")
    print(f"close_pairs={len(close_pairs)}")
    print(f"removed_unreferenced={removed_unreferenced}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
