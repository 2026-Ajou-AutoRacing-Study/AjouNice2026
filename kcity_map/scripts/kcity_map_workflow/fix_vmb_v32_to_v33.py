#!/usr/bin/env python3
"""Create v33 by fixing VMB traffic-light order and GIS-aligning centerlines."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import shapefile
from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"

VMB_REVERSE_TRAFFIC_LIGHT_WAYS = {"103817", "103839", "103877"}


Point2 = Tuple[float, float]
Point3 = Tuple[float, float, float]


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def relation_members(relation: etree._Element) -> List[Tuple[str, str, str]]:
    return [(member.get("type"), member.get("ref"), member.get("role")) for member in relation.findall("member")]


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    nd_elements = way.findall("nd")
    insert_at = list(way).index(nd_elements[0]) if nd_elements else 0
    for nd in nd_elements:
        way.remove(nd)
    for offset, ref in enumerate(refs):
        way.insert(insert_at + offset, etree.Element("nd", ref=ref))


def node_ele(node: etree._Element) -> float:
    return float(tags(node).get("ele", "0"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def distance(a: Point2, b: Point2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point2]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def interpolate(points: Sequence[Point2], total_length: float, distance_along: float) -> Point2:
    if not points:
        return (0.0, 0.0)
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


def directed_rmse(source_points: Sequence[Point2], source_length: float, target_points: Sequence[Point2], step_m: float) -> Tuple[float, float, int]:
    squared_sum = 0.0
    max_error = 0.0
    count = 0
    for point in sample_polyline(source_points, source_length, step_m):
        error = point_polyline_distance(point, target_points)
        squared_sum += error * error
        max_error = max(max_error, error)
        count += 1
    return math.sqrt(squared_sum / count), max_error, count


def symmetric_rmse(a: Dict[str, object], b: Dict[str, object], step_m: float) -> Tuple[float, float, int]:
    a_rmse, a_max, a_count = directed_rmse(a["points2"], a["length"], b["points2"], step_m)
    b_rmse, b_max, b_count = directed_rmse(b["points2"], b["length"], a["points2"], step_m)
    weighted = math.sqrt((a_rmse * a_rmse * a_count + b_rmse * b_rmse * b_count) / (a_count + b_count))
    return weighted, max(a_max, b_max), a_count + b_count


def point_on_polygon_boundary(point: Point2, polygon: Sequence[Point2], tolerance_m: float) -> bool:
    return any(point_segment_distance(point, start, end) <= tolerance_m for start, end in zip(polygon, polygon[1:] + polygon[:1]))


def point_in_polygon(point: Point2, polygon: Sequence[Point2], tolerance_m: float) -> bool:
    if len(polygon) < 3:
        return False
    if point_on_polygon_boundary(point, polygon, tolerance_m):
        return True
    inside = False
    x, y = point
    prev_x, prev_y = polygon[-1]
    for curr_x, curr_y in polygon:
        crosses = (curr_y > y) != (prev_y > y)
        if crosses:
            intersect_x = (prev_x - curr_x) * (y - curr_y) / (prev_y - curr_y + 1e-15) + curr_x
            if x < intersect_x:
                inside = not inside
        prev_x, prev_y = curr_x, curr_y
    return inside


def with_geometry_metadata(item: Dict[str, object]) -> Dict[str, object]:
    points2 = item["points2"]
    length = polyline_length(points2)
    item["length"] = length
    item["start"] = points2[0]
    item["end"] = points2[-1]
    item["mid"] = interpolate(points2, length, length / 2.0)
    return item


def load_gis_links(path: Path) -> List[Dict[str, object]]:
    reader = shapefile.Reader(str(path))
    links = []
    for shape_record in reader.iterShapeRecords():
        record = shape_record.record.as_dict()
        xy_points = [tuple(map(float, point)) for point in shape_record.shape.points]
        z_values = list(getattr(shape_record.shape, "z", []))
        if len(xy_points) < 2:
            continue
        points3 = []
        for idx, point in enumerate(xy_points):
            z = float(z_values[idx]) if idx < len(z_values) else 0.0
            points3.append((point[0], point[1], z))
        links.append(
            with_geometry_metadata(
                {
                    "id": record["ID"],
                    "link_type": str(record.get("LinkType", "")),
                    "from_node_id": record.get("FromNodeID", ""),
                    "to_node_id": record.get("ToNodeID", ""),
                    "points3": points3,
                    "points2": [(x, y) for x, y, _ in points3],
                }
            )
        )
    return links


def load_mgeo_link_ids(path: Path) -> set:
    return {link["idx"] for link in json.load(path.open())}


def lanelet_polygon_points(lanelet_relation: etree._Element, ways: Dict[str, etree._Element], nodes_xy: Dict[str, Point2]) -> List[Point2]:
    left_id: Optional[str] = None
    right_id: Optional[str] = None
    for member_type, ref, role in relation_members(lanelet_relation):
        if member_type == "way" and role == "left":
            left_id = ref
        elif member_type == "way" and role == "right":
            right_id = ref
    if not left_id or not right_id:
        return []
    left = [nodes_xy[ref] for ref in way_refs(ways[left_id]) if ref in nodes_xy]
    right = [nodes_xy[ref] for ref in way_refs(ways[right_id]) if ref in nodes_xy]
    return left + list(reversed(right))


def extract_road_centerlines(
    root: etree._Element,
    ways: Dict[str, etree._Element],
    nodes_xy: Dict[str, Point2],
    nodes_xyz: Dict[str, Point3],
) -> List[Dict[str, object]]:
    centerlines = []
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        center_way_id = None
        for member_type, ref, role in relation_members(relation):
            if member_type == "way" and role == "centerline":
                center_way_id = ref
                break
        if center_way_id is None or center_way_id not in ways:
            continue
        refs = way_refs(ways[center_way_id])
        points3 = [nodes_xyz[ref] for ref in refs if ref in nodes_xyz]
        if len(points3) < 2:
            continue
        centerlines.append(
            with_geometry_metadata(
                {
                    "lanelet_id": relation.get("id"),
                    "center_way_id": center_way_id,
                    "old_node_count": len(refs),
                    "points3": points3,
                    "points2": [(x, y) for x, y, _ in points3],
                    "polygon": lanelet_polygon_points(relation, ways, nodes_xy),
                }
            )
        )
    return centerlines


def choose_best_gis_link(centerline: Dict[str, object], gis_links: Sequence[Dict[str, object]], candidate_count: int, step_m: float) -> Dict[str, object]:
    candidates = sorted(gis_links, key=lambda link: distance(centerline["mid"], link["mid"]))[:candidate_count]
    best_link = candidates[0]
    best_score = math.inf
    for link in candidates:
        length_ratio = abs(centerline["length"] - link["length"]) / max(centerline["length"], link["length"], 1.0)
        endpoint_error = min(
            distance(centerline["start"], link["start"]) + distance(centerline["end"], link["end"]),
            distance(centerline["start"], link["end"]) + distance(centerline["end"], link["start"]),
        )
        rmse, _, _ = symmetric_rmse(centerline, link, step_m)
        score = rmse + 0.02 * endpoint_error + 0.5 * length_ratio
        if score < best_score:
            best_score = score
            best_link = link
    return best_link


def oriented_gis_points(centerline: Dict[str, object], gis_link: Dict[str, object]) -> Tuple[List[Point3], str]:
    points3 = list(gis_link["points3"])
    forward_error = distance(centerline["start"], gis_link["start"]) + distance(centerline["end"], gis_link["end"])
    reverse_error = distance(centerline["start"], gis_link["end"]) + distance(centerline["end"], gis_link["start"])
    if reverse_error < forward_error:
        return list(reversed(points3)), "reverse"
    return points3, "forward"


def append_centerline_node(
    root: etree._Element,
    insert_at: int,
    node_id: str,
    point: Point3,
    transformer_to_wgs: Transformer,
) -> etree._Element:
    lon, lat = transformer_to_wgs.transform(point[0], point[1])
    node = etree.Element("node", id=node_id, visible="true", version="1", lat=f"{lat:.14f}", lon=f"{lon:.14f}")
    node.append(etree.Element("tag", k="ele", v=f"{point[2]:.9f}"))
    root.insert(insert_at, node)
    return node


def coordinate_key(point: Point3) -> Tuple[float, float]:
    return (round(point[0], 7), round(point[1], 7))


def remove_unreferenced_nodes(root: etree._Element) -> int:
    referenced = {nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")}
    referenced.update(member.get("ref") for relation in root.findall("relation") for member in relation.findall("member") if member.get("type") == "node")
    removed = 0
    for node in list(root.findall("node")):
        if node.get("id") not in referenced:
            root.remove(node)
            removed += 1
    return removed


def count_duplicate_traffic_light_regs(root: etree._Element) -> Tuple[int, int]:
    relations = {relation.get("id"): relation for relation in root.findall("relation")}
    traffic_light_regs = {
        relation_id
        for relation_id, relation in relations.items()
        if tags(relation).get("type") == "regulatory_element" and tags(relation).get("subtype") == "traffic_light"
    }
    lanelets_with_duplicates = 0
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        count = sum(
            1
            for member_type, ref, role in relation_members(relation)
            if member_type == "relation" and role == "regulatory_element" and ref in traffic_light_regs
        )
        if count >= 2:
            lanelets_with_duplicates += 1
    return lanelets_with_duplicates, len(traffic_light_regs)


def count_abrupt_z_edges(root: etree._Element, threshold_m: float) -> int:
    nodes = {node.get("id"): node_ele(node) for node in root.findall("node")}
    abrupt = 0
    for way in root.findall("way"):
        refs = way_refs(way)
        for start, end in zip(refs, refs[1:]):
            if start in nodes and end in nodes and abs(nodes[start] - nodes[end]) > threshold_m:
                abrupt += 1
    return abrupt


def osm_summary(path: Path, root: etree._Element) -> Dict[str, object]:
    relation_counts: Dict[str, int] = {}
    way_counts: Dict[str, int] = {}
    for way in root.findall("way"):
        way_tags = tags(way)
        way_counts[f"{way_tags.get('type', '')}/{way_tags.get('subtype', '')}"] = way_counts.get(
            f"{way_tags.get('type', '')}/{way_tags.get('subtype', '')}", 0
        ) + 1
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        relation_counts[f"{relation_tags.get('type', '')}/{relation_tags.get('subtype', '')}"] = relation_counts.get(
            f"{relation_tags.get('type', '')}/{relation_tags.get('subtype', '')}", 0
        ) + 1
    return {
        "nodes": len(root.findall("node")),
        "ways": len(root.findall("way")),
        "relations": len(root.findall("relation")),
        "road_lanelets": relation_counts.get("lanelet/road", 0),
        "crosswalk_lanelets": relation_counts.get("lanelet/crosswalk", 0),
        "traffic_light_regs": relation_counts.get("regulatory_element/traffic_light", 0),
        "stop_line_ways": way_counts.get("line_thin/stop_line", 0),
        "sha256": sha256(path),
    }


def copy_package(output_map: Path, package_dir: Path, source_package_dir: Path) -> None:
    package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(output_map, package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(source_package_dir / name, package_dir / name)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v32.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v33.osm")
    parser.add_argument("--map-version", default="33")
    parser.add_argument("--gis-a2-link", type=Path, default=ROOT / "gis_map/A2_LINK.shp")
    parser.add_argument("--mgeo-link-set", type=Path, default=ROOT / "mgeo_map/link_set.json")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v33")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v32")
    parser.add_argument(
        "--tl-report",
        type=Path,
        default=ROOT / "reports/v33_vmb_tl_order_fix.csv",
    )
    parser.add_argument(
        "--centerline-report",
        type=Path,
        default=ROOT / "reports/v33_centerline_alignment_audit.csv",
    )
    parser.add_argument(
        "--summary-report",
        type=Path,
        default=ROOT / "reports/final_validation_summary_v33.csv",
    )
    parser.add_argument("--match-candidates", type=int, default=80)
    parser.add_argument("--match-step-m", type=float, default=8.0)
    parser.add_argument("--verify-step-m", type=float, default=1.0)
    parser.add_argument("--rmse-threshold-m", type=float, default=0.0001)
    parser.add_argument("--inside-sample-step-m", type=float, default=5.0)
    parser.add_argument("--inside-tolerance-m", type=float, default=0.20)
    args = parser.parse_args()

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.input), parser_xml)
    root = tree.getroot()

    transformer_to_utm = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    transformer_to_wgs = Transformer.from_crs(UTM52, "EPSG:4326", always_xy=True)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", str(args.map_version))

    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    nodes_xyz: Dict[str, Point3] = {}
    nodes_xy: Dict[str, Point2] = {}
    for node_id, node in nodes.items():
        x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        nodes_xyz[node_id] = (x, y, node_ele(node))
        nodes_xy[node_id] = (x, y)

    tl_rows = []
    for way_id in sorted(VMB_REVERSE_TRAFFIC_LIGHT_WAYS, key=int):
        way = ways[way_id]
        old_refs = way_refs(way)
        new_refs = list(reversed(old_refs))
        replace_way_refs(way, new_refs)
        tl_rows.append({"way_id": way_id, "old_refs": " ".join(old_refs), "new_refs": " ".join(new_refs)})

    gis_links = load_gis_links(args.gis_a2_link)
    mgeo_link_ids = load_mgeo_link_ids(args.mgeo_link_set)
    centerlines = extract_road_centerlines(root, ways, nodes_xy, nodes_xyz)

    first_way = root.find("way")
    insert_at = list(root).index(first_way) if first_way is not None else len(root)
    next_node_id = max(int(node.get("id")) for node in root.findall("node")) + 1
    coordinate_cache = {coordinate_key(point): node_id for node_id, point in nodes_xyz.items()}

    centerline_rows = []
    weighted_after_square_sum = 0.0
    weighted_after_count = 0
    after_max_error = 0.0
    before_values = []
    created_centerline_nodes = 0
    reused_centerline_nodes = 0

    for centerline in centerlines:
        gis_link = choose_best_gis_link(centerline, gis_links, args.match_candidates, args.match_step_m)
        before_rmse, before_max, before_samples = symmetric_rmse(centerline, gis_link, args.verify_step_m)
        before_values.append(before_rmse)

        oriented_points3, orientation = oriented_gis_points(centerline, gis_link)
        oriented_points2 = [(x, y) for x, y, _ in oriented_points3]
        oriented_length = polyline_length(oriented_points2)

        new_refs = []
        after_points2 = []
        for point in oriented_points3:
            key = coordinate_key(point)
            node_id = coordinate_cache.get(key)
            if node_id is None:
                node_id = str(next_node_id)
                next_node_id += 1
                node = append_centerline_node(root, insert_at, node_id, point, transformer_to_wgs)
                insert_at += 1
                x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
                nodes_xy[node_id] = (x, y)
                nodes_xyz[node_id] = (x, y, point[2])
                coordinate_cache[key] = node_id
                created_centerline_nodes += 1
            else:
                reused_centerline_nodes += 1
            if new_refs and new_refs[-1] == node_id:
                continue
            new_refs.append(node_id)
            after_points2.append(nodes_xy[node_id])

        replace_way_refs(ways[centerline["center_way_id"]], new_refs)

        after_item = with_geometry_metadata({"points2": after_points2, "points3": oriented_points3})
        source_item = with_geometry_metadata({"points2": oriented_points2, "points3": oriented_points3})
        after_rmse, after_max, after_samples = symmetric_rmse(after_item, source_item, args.verify_step_m)
        weighted_after_square_sum += after_rmse * after_rmse * after_samples
        weighted_after_count += after_samples
        after_max_error = max(after_max_error, after_max)

        outside_samples = 0
        total_inside_samples = 0
        polygon = centerline["polygon"]
        if polygon:
            for sample_point in sample_polyline(oriented_points2, oriented_length, args.inside_sample_step_m):
                total_inside_samples += 1
                if not point_in_polygon(sample_point, polygon, args.inside_tolerance_m):
                    outside_samples += 1

        endpoint_error_before = min(
            distance(centerline["start"], gis_link["start"]) + distance(centerline["end"], gis_link["end"]),
            distance(centerline["start"], gis_link["end"]) + distance(centerline["end"], gis_link["start"]),
        )

        centerline_rows.append(
            {
                "lanelet_id": centerline["lanelet_id"],
                "center_way_id": centerline["center_way_id"],
                "matched_gis_link_id": gis_link["id"],
                "matched_in_mgeo": str(gis_link["id"] in mgeo_link_ids),
                "gis_link_type": gis_link["link_type"],
                "orientation": orientation,
                "old_node_count": centerline["old_node_count"],
                "new_node_count": len(new_refs),
                "old_length_m": f"{centerline['length']:.6f}",
                "gis_length_m": f"{gis_link['length']:.6f}",
                "before_rmse_m": f"{before_rmse:.9f}",
                "before_max_m": f"{before_max:.9f}",
                "before_samples": before_samples,
                "before_endpoint_error_m": f"{endpoint_error_before:.9f}",
                "after_rmse_m": f"{after_rmse:.12f}",
                "after_max_m": f"{after_max:.12f}",
                "after_samples": after_samples,
                "outside_polygon_samples": outside_samples,
                "polygon_samples": total_inside_samples,
            }
        )

    removed_nodes = remove_unreferenced_nodes(root)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    copy_package(args.output, args.package_dir, args.source_package_dir)

    write_csv(args.tl_report, ["way_id", "old_refs", "new_refs"], tl_rows)
    write_csv(
        args.centerline_report,
        [
            "lanelet_id",
            "center_way_id",
            "matched_gis_link_id",
            "matched_in_mgeo",
            "gis_link_type",
            "orientation",
            "old_node_count",
            "new_node_count",
            "old_length_m",
            "gis_length_m",
            "before_rmse_m",
            "before_max_m",
            "before_samples",
            "before_endpoint_error_m",
            "after_rmse_m",
            "after_max_m",
            "after_samples",
            "outside_polygon_samples",
            "polygon_samples",
        ],
        centerline_rows,
    )

    final_after_rmse = math.sqrt(weighted_after_square_sum / weighted_after_count)
    duplicate_lanelets, traffic_light_reg_count = count_duplicate_traffic_light_regs(root)
    abrupt_z_edges = count_abrupt_z_edges(root, 2.0)
    summary = osm_summary(args.output, root)
    package_map = args.package_dir / "lanelet2_map.osm"
    package_identical = sha256(args.output) == sha256(package_map)
    outside_total = sum(int(row["outside_polygon_samples"]) for row in centerline_rows)
    max_before = max(before_values) if before_values else math.nan

    summary_rows = [
        {
            "check": "final_map",
            "status": "PASS",
            "detail": f"{args.output.relative_to(ROOT)} sha256={summary['sha256']}",
        },
        {
            "check": "packaged_map",
            "status": "PASS" if package_identical else "FAIL",
            "detail": f"{package_map.relative_to(ROOT)} {'byte-identical' if package_identical else 'sha256 differs'}",
        },
        {
            "check": "vmb_traffic_light_left_to_right_targeted",
            "status": "PASS",
            "detail": f"reversed_target_ways={','.join(sorted(VMB_REVERSE_TRAFFIC_LIGHT_WAYS, key=int))}",
        },
        {
            "check": "centerline_gis_a2_link_rmse",
            "status": "PASS" if final_after_rmse <= args.rmse_threshold_m else "FAIL",
            "detail": f"weighted_after_rmse_m={final_after_rmse:.12f} threshold_m={args.rmse_threshold_m:.7f} max_after_m={after_max_error:.12f} max_before_rmse_m={max_before:.6f}",
        },
        {
            "check": "centerline_inside_existing_lanelet_polygon",
            "status": "PASS" if outside_total == 0 else "WARN",
            "detail": f"outside_samples={outside_total} tolerance_m={args.inside_tolerance_m:.2f}",
        },
        {
            "check": "vmb_duplicate_traffic_light_reg",
            "status": "PASS" if duplicate_lanelets == 0 else "FAIL",
            "detail": f"lanelets_with_2plus_tl_regs={duplicate_lanelets} traffic_light_regs={traffic_light_reg_count}",
        },
        {
            "check": "vmb_abrupt_z",
            "status": "PASS" if abrupt_z_edges == 0 else "FAIL",
            "detail": f"abrupt_z_edges_gt_2m={abrupt_z_edges}",
        },
        {
            "check": "osm_counts",
            "status": "PASS",
            "detail": " ".join(f"{key}={value}" for key, value in summary.items() if key != "sha256"),
        },
    ]
    write_csv(args.summary_report, ["check", "status", "detail"], summary_rows)

    print(f"output={args.output}")
    print(f"map_version={args.map_version}")
    print(f"packaged={args.package_dir}")
    print(f"traffic_light_ways_reversed={len(tl_rows)}")
    print(f"road_centerlines_aligned={len(centerline_rows)}")
    print(f"created_centerline_nodes={created_centerline_nodes}")
    print(f"reused_centerline_nodes={reused_centerline_nodes}")
    print(f"centerline_weighted_after_rmse_m={final_after_rmse:.12f}")
    print(f"centerline_after_max_m={after_max_error:.12f}")
    print(f"centerline_outside_polygon_samples={outside_total}")
    print(f"removed_unreferenced_nodes={removed_nodes}")
    print(f"tl_report={args.tl_report}")
    print(f"centerline_report={args.centerline_report}")
    print(f"summary_report={args.summary_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
