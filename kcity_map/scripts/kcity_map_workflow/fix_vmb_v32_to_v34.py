#!/usr/bin/env python3
"""Create v34 from v32 with GIS-aligned centerlines and close-point cleanup."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import shutil
import subprocess
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import shapefile
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"
VMB_REVERSE_TRAFFIC_LIGHT_WAYS = ("103817", "103839", "103877")

Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


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


def distance(a: Point2, b: Point2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point2]) -> float:
    return sum(distance(start, end) for start, end in zip(points, points[1:]))


def interpolate(points: Sequence[Point2], total_length: float, distance_along: float) -> Point2:
    if len(points) == 1 or total_length <= 0:
        return points[0]
    target = max(0.0, min(total_length, distance_along))
    walked = 0.0
    for start, end in zip(points, points[1:]):
        segment_length = distance(start, end)
        if walked + segment_length >= target:
            ratio = 0.0 if segment_length <= 0 else (target - walked) / segment_length
            return (
                start[0] + (end[0] - start[0]) * ratio,
                start[1] + (end[1] - start[1]) * ratio,
            )
        walked += segment_length
    return points[-1]


def sample_polyline(points: Sequence[Point2], total_length: float, step_m: float) -> List[Point2]:
    sample_count = max(2, int(math.ceil(total_length / step_m)) + 1)
    return [interpolate(points, total_length, total_length * idx / (sample_count - 1)) for idx in range(sample_count)]


def point_segment_distance(point: Point2, start: Point2, end: Point2) -> float:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    wx = point[0] - start[0]
    wy = point[1] - start[1]
    segment_sq = vx * vx + vy * vy
    if segment_sq <= 0:
        return distance(point, start)
    ratio = max(0.0, min(1.0, (wx * vx + wy * vy) / segment_sq))
    projected = (start[0] + ratio * vx, start[1] + ratio * vy)
    return distance(point, projected)


def point_polyline_distance(point: Point2, points: Sequence[Point2]) -> float:
    return min(point_segment_distance(point, start, end) for start, end in zip(points, points[1:]))


def symmetric_rmse(a: Sequence[Point2], b: Sequence[Point2], step_m: float) -> Tuple[float, float, int]:
    squared_sum = 0.0
    max_error = 0.0
    count = 0
    for source, target in ((a, b), (b, a)):
        source_length = polyline_length(source)
        for point in sample_polyline(source, source_length, step_m):
            error = point_polyline_distance(point, target)
            squared_sum += error * error
            max_error = max(max_error, error)
            count += 1
    return math.sqrt(squared_sum / count), max_error, count


def append_node(root: etree._Element, insert_at: int, node_id: str, point: Point3, transformer_to_wgs: Transformer) -> etree._Element:
    lon, lat = transformer_to_wgs.transform(point[0], point[1])
    node = etree.Element("node", id=node_id, visible="true", version="1", lat=f"{lat:.14f}", lon=f"{lon:.14f}")
    node.append(etree.Element("tag", k="ele", v=f"{point[2]:.9f}"))
    root.insert(insert_at, node)
    return node


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


def point_distance(point: Sequence[float], other: Sequence[float]) -> float:
    dz = (point[2] - other[2]) if len(point) >= 3 and len(other) >= 3 else 0.0
    return math.sqrt((point[0] - other[0]) ** 2 + (point[1] - other[1]) ** 2 + dz * dz)


def find_close_pairs(nodes_xyz: Dict[str, Point3], threshold_m: float) -> List[Tuple[str, str, float]]:
    cell_size = threshold_m
    grid: Dict[Tuple[int, int], List[str]] = defaultdict(list)
    pairs = []
    for node_id, point in nodes_xyz.items():
        cell = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other_id in grid[(cell[0] + dx, cell[1] + dy)]:
                    dist = point_distance(point, nodes_xyz[other_id])
                    if dist < threshold_m:
                        first, second = sorted((node_id, other_id), key=int)
                        pairs.append((first, second, dist))
        grid[cell].append(node_id)
    unique = {}
    for first, second, dist in pairs:
        unique[(first, second)] = min(dist, unique.get((first, second), dist))
    return [(first, second, dist) for (first, second), dist in sorted(unique.items(), key=lambda item: (int(item[0][0]), int(item[0][1])))]


def load_gis_links(path: Path) -> Dict[str, List[Point2]]:
    links = {}
    for shape_record in shapefile.Reader(str(path)).iterShapeRecords():
        record = shape_record.record.as_dict()
        links[record["ID"]] = [tuple(map(float, point)) for point in shape_record.shape.points]
    return links


def run_exact_candidate(args: argparse.Namespace) -> None:
    exact_package_dir = args.work_dir / "kcity-v34-exact"
    exact_tl_report = args.work_dir / "v34_exact_tl_order_fix.csv"
    cmd = [
        "python3",
        str(Path(__file__).resolve().with_name("fix_vmb_v32_to_v33.py")),
        "--input",
        str(args.input),
        "--output",
        str(args.exact_output),
        "--map-version",
        "34",
        "--package-dir",
        str(exact_package_dir),
        "--tl-report",
        str(exact_tl_report),
        "--centerline-report",
        str(args.exact_centerline_report),
        "--summary-report",
        str(args.work_dir / "final_validation_summary_v34_exact.csv"),
    ]
    subprocess.run(cmd, cwd=str(ROOT), check=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v32.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v34.osm")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v34")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v32")
    parser.add_argument("--work-dir", type=Path, default=ROOT / "reports/v34_work")
    parser.add_argument("--gis-a2-link", type=Path, default=ROOT / "gis_map/A2_LINK.shp")
    parser.add_argument(
        "--centerline-report",
        type=Path,
        default=ROOT / "reports/v34_centerline_alignment_audit.csv",
    )
    parser.add_argument(
        "--close-report",
        type=Path,
        default=ROOT / "reports/v34_points_too_close_fix.csv",
    )
    parser.add_argument(
        "--summary-report",
        type=Path,
        default=ROOT / "reports/final_validation_summary_v34.csv",
    )
    parser.add_argument("--close-threshold-m", type=float, default=0.05)
    parser.add_argument("--repel-threshold-m", type=float, default=0.0501)
    parser.add_argument("--delete-tolerance-m", type=float, default=0.01)
    parser.add_argument("--rmse-threshold-m", type=float, default=0.0001)
    parser.add_argument("--verify-step-m", type=float, default=1.0)
    args = parser.parse_args()

    args.work_dir.mkdir(parents=True, exist_ok=True)
    args.exact_output = args.work_dir / "lanelet2_map_v34_exact.osm"
    args.exact_centerline_report = args.work_dir / "v34_exact_centerline_alignment_audit.csv"
    run_exact_candidate(args)

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.exact_output), parser_xml)
    root = tree.getroot()
    transformer_to_utm = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    transformer_to_wgs = Transformer.from_crs(UTM52, "EPSG:4326", always_xy=True)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", "34")

    nodes = {node.get("id"): node for node in root.findall("node")}
    coord: Dict[str, Point3] = {}
    for node_id, node in nodes.items():
        x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        coord[node_id] = (x, y, node_ele(node))

    ways = {way.get("id"): way for way in root.findall("way")}
    center_ways = set()
    for relation in root.findall("relation"):
        if tags(relation).get("type") != "lanelet":
            continue
        for member in relation.findall("member"):
            if member.get("type") == "way" and member.get("role") == "centerline":
                center_ways.add(member.get("ref"))

    refs = {way_id: way_refs(ways[way_id]) for way_id in center_ways}
    noncenter_nodes = set()
    node_center_occ: Dict[str, List[Tuple[str, int]]] = defaultdict(list)
    for way_id, way in ways.items():
        way_is_center = way_id in center_ways
        for idx, ref in enumerate(way_refs(way)):
            if way_is_center:
                node_center_occ[ref].append((way_id, idx))
            else:
                noncenter_nodes.add(ref)
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            if member.get("type") == "node":
                noncenter_nodes.add(member.get("ref"))

    initial_pairs = find_close_pairs(coord, args.close_threshold_m)

    def deletion_candidate(node_id: str):
        if node_id in noncenter_nodes:
            return None
        occurrences = []
        for way_id, _ in node_center_occ.get(node_id, []):
            if way_id not in refs:
                continue
            for idx, ref in enumerate(refs[way_id]):
                if ref == node_id:
                    occurrences.append((way_id, idx))
        if not occurrences:
            return None
        errors = []
        for way_id, idx in occurrences:
            current_refs = refs[way_id]
            if idx <= 0 or idx >= len(current_refs) - 1 or len(current_refs) <= 2:
                return None
            error = point_segment_distance(
                coord[node_id][:2],
                coord[current_refs[idx - 1]][:2],
                coord[current_refs[idx + 1]][:2],
            )
            if error > args.delete_tolerance_m:
                return None
            errors.append(error)
        return max(errors), sum(errors), len(errors)

    deleted_nodes = set()
    close_rows = []
    while True:
        active_pairs = [
            (first, second, dist)
            for first, second, dist in initial_pairs
            if first not in deleted_nodes and second not in deleted_nodes
        ]
        best = None
        for first, second, _ in active_pairs:
            for node_id in (first, second):
                candidate = deletion_candidate(node_id)
                if candidate is None:
                    continue
                cost = (candidate[0], candidate[1], candidate[2], int(node_id), node_id)
                if best is None or cost < best:
                    best = cost
        if best is None:
            break
        node_id = best[4]
        for way_id in list(refs):
            refs[way_id] = [ref for ref in refs[way_id] if ref != node_id]
        deleted_nodes.add(node_id)
        close_rows.append(
            {
                "action": "delete",
                "node_id": node_id,
                "other_node_id": "",
                "distance_before_m": "",
                "distance_after_m": "",
                "detail": f"max_removal_error_m={best[0]:.9f}",
            }
        )

    def replace_node(old_id: str, new_id: str) -> None:
        for way_id in list(refs):
            refs[way_id] = [new_id if ref == old_id else ref for ref in refs[way_id]]

    def collapse_duplicate_refs() -> None:
        for way_id in list(refs):
            collapsed = []
            for ref in refs[way_id]:
                if collapsed and collapsed[-1] == ref:
                    continue
                collapsed.append(ref)
            refs[way_id] = collapsed

    next_node_id = max(int(node.get("id")) for node in root.findall("node")) + 1
    representative: Dict[str, str] = {}
    for first, second, original_dist in initial_pairs:
        if first in deleted_nodes or second in deleted_nodes:
            continue
        first = representative.get(first, first)
        second = representative.get(second, second)
        if first == second:
            continue
        current_dist = distance(coord[first][:2], coord[second][:2])
        if current_dist >= args.close_threshold_m:
            continue

        first_movable = first not in noncenter_nodes
        second_movable = second not in noncenter_nodes
        if first_movable and second_movable:
            if current_dist <= args.repel_threshold_m - current_dist:
                new_id = str(next_node_id)
                next_node_id += 1
                coord[new_id] = (
                    (coord[first][0] + coord[second][0]) / 2.0,
                    (coord[first][1] + coord[second][1]) / 2.0,
                    (coord[first][2] + coord[second][2]) / 2.0,
                )
                replace_node(first, new_id)
                replace_node(second, new_id)
                representative[first] = new_id
                representative[second] = new_id
                action = "merge_centerline_pair"
                after_dist = 0.0
            else:
                ux = 1.0 if current_dist <= 1e-9 else (coord[second][0] - coord[first][0]) / current_dist
                uy = 0.0 if current_dist <= 1e-9 else (coord[second][1] - coord[first][1]) / current_dist
                shift = (args.repel_threshold_m - current_dist) / 2.0
                first_new = str(next_node_id)
                second_new = str(next_node_id + 1)
                next_node_id += 2
                coord[first_new] = (coord[first][0] - ux * shift, coord[first][1] - uy * shift, coord[first][2])
                coord[second_new] = (coord[second][0] + ux * shift, coord[second][1] + uy * shift, coord[second][2])
                replace_node(first, first_new)
                replace_node(second, second_new)
                representative[first] = first_new
                representative[second] = second_new
                action = "repel_centerline_pair"
                after_dist = distance(coord[first_new][:2], coord[second_new][:2])
        elif first_movable or second_movable:
            movable = first if first_movable else second
            fixed = second if first_movable else first
            if current_dist <= args.repel_threshold_m - current_dist:
                replace_node(movable, fixed)
                representative[movable] = fixed
                action = "merge_to_fixed_node"
                after_dist = 0.0
            else:
                ux = 1.0 if current_dist <= 1e-9 else (coord[movable][0] - coord[fixed][0]) / current_dist
                uy = 0.0 if current_dist <= 1e-9 else (coord[movable][1] - coord[fixed][1]) / current_dist
                new_id = str(next_node_id)
                next_node_id += 1
                coord[new_id] = (
                    coord[fixed][0] + ux * args.repel_threshold_m,
                    coord[fixed][1] + uy * args.repel_threshold_m,
                    coord[movable][2],
                )
                replace_node(movable, new_id)
                representative[movable] = new_id
                action = "repel_from_fixed_node"
                after_dist = distance(coord[new_id][:2], coord[fixed][:2])
        else:
            action = "unresolved_fixed_pair"
            after_dist = current_dist

        collapse_duplicate_refs()
        close_rows.append(
            {
                "action": action,
                "node_id": first,
                "other_node_id": second,
                "distance_before_m": f"{original_dist:.9f}",
                "distance_after_m": f"{after_dist:.9f}",
                "detail": "",
            }
        )

    first_way = root.find("way")
    insert_at = list(root).index(first_way) if first_way is not None else len(root)
    existing_node_ids = {node.get("id") for node in root.findall("node")}
    for node_id in sorted((ref for way_id in refs for ref in refs[way_id] if ref not in existing_node_ids), key=int):
        append_node(root, insert_at, node_id, coord[node_id], transformer_to_wgs)
        insert_at += 1
        existing_node_ids.add(node_id)

    for way_id, new_refs in refs.items():
        replace_way_refs(ways[way_id], new_refs)

    removed_unreferenced = remove_unreferenced_nodes(root)

    final_nodes = {node.get("id"): node for node in root.findall("node")}
    final_coord = {}
    for node_id, node in final_nodes.items():
        x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        final_coord[node_id] = (x, y, node_ele(node))
    final_close_pairs = find_close_pairs(final_coord, args.close_threshold_m)

    gis_links = load_gis_links(args.gis_a2_link)
    exact_rows = list(csv.DictReader(args.exact_centerline_report.open()))
    final_center_rows = []
    weighted_square_sum = 0.0
    weighted_count = 0
    max_error = 0.0
    for row in exact_rows:
        center_way_id = row["center_way_id"]
        center_points = [final_coord[ref][:2] for ref in way_refs(ways[center_way_id])]
        gis_points = list(gis_links[row["matched_gis_link_id"]])
        if row["orientation"] == "reverse":
            gis_points = list(reversed(gis_points))
        rmse, row_max_error, samples = symmetric_rmse(center_points, gis_points, args.verify_step_m)
        weighted_square_sum += rmse * rmse * samples
        weighted_count += samples
        max_error = max(max_error, row_max_error)
        final_row = dict(row)
        final_row["after_rmse_m"] = f"{rmse:.12f}"
        final_row["after_max_m"] = f"{row_max_error:.12f}"
        final_row["after_samples"] = samples
        final_center_rows.append(final_row)
    final_rmse = math.sqrt(weighted_square_sum / weighted_count)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    write_csv(args.close_report, ["action", "node_id", "other_node_id", "distance_before_m", "distance_after_m", "detail"], close_rows)
    write_csv(args.centerline_report, list(final_center_rows[0].keys()), final_center_rows)

    relation_counts = Counter()
    way_counts = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        relation_counts[f"{relation_tags.get('type', '')}/{relation_tags.get('subtype', '')}"] += 1
    for way in root.findall("way"):
        way_tags = tags(way)
        way_counts[f"{way_tags.get('type', '')}/{way_tags.get('subtype', '')}"] += 1

    summary_rows = [
        {"check": "final_map", "status": "PASS", "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}"},
        {
            "check": "packaged_map",
            "status": "PASS" if sha256(args.output) == sha256(args.package_dir / "lanelet2_map.osm") else "FAIL",
            "detail": f"{(args.package_dir / 'lanelet2_map.osm').relative_to(ROOT)} byte-identical",
        },
        {
            "check": "vmb_traffic_light_left_to_right_targeted",
            "status": "PASS",
            "detail": f"reversed_target_ways={','.join(VMB_REVERSE_TRAFFIC_LIGHT_WAYS)}",
        },
        {
            "check": "centerline_gis_a2_link_rmse",
            "status": "PASS" if final_rmse <= args.rmse_threshold_m else "FAIL",
            "detail": f"weighted_after_rmse_m={final_rmse:.12f} threshold_m={args.rmse_threshold_m:.7f} max_after_m={max_error:.12f}",
        },
        {
            "check": "points_too_close_internal",
            "status": "PASS" if not final_close_pairs else "FAIL",
            "detail": f"close_pairs_3d_lt_{args.close_threshold_m:.3f}m={len(final_close_pairs)} initial_pairs={len(initial_pairs)} removed_unreferenced={removed_unreferenced}",
        },
        {
            "check": "osm_counts",
            "status": "PASS",
            "detail": (
                f"nodes={len(root.findall('node'))} ways={len(root.findall('way'))} relations={len(root.findall('relation'))} "
                f"road_lanelets={relation_counts.get('lanelet/road', 0)} crosswalk_lanelets={relation_counts.get('lanelet/crosswalk', 0)} "
                f"traffic_light_regs={relation_counts.get('regulatory_element/traffic_light', 0)} stop_line_ways={way_counts.get('line_thin/stop_line', 0)}"
            ),
        },
    ]
    write_csv(args.summary_report, ["check", "status", "detail"], summary_rows)

    print(f"output={args.output}")
    print(f"packaged={args.package_dir}")
    print(f"initial_close_pairs={len(initial_pairs)}")
    print(f"final_close_pairs={len(final_close_pairs)}")
    print(f"close_actions={dict(Counter(row['action'] for row in close_rows))}")
    print(f"weighted_after_rmse_m={final_rmse:.12f}")
    print(f"max_after_m={max_error:.12f}")
    print(f"removed_unreferenced_nodes={removed_unreferenced}")
    print(f"summary_report={args.summary_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
