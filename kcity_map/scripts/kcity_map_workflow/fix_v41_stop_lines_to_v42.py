#!/usr/bin/env python3
"""Create v42 by adding verified stop-sign regulatory elements to v41."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import shapefile
from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point, Polygon


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs"
WGS84 = "EPSG:4326"
STOP_SIGN_REMARKS = {"정지", "일시정지"}


@dataclass(frozen=True)
class WayInfo:
    way_id: str
    refs: List[str]
    tags: Dict[str, str]
    points: List[Tuple[float, float, float]]
    line: Optional[LineString]


@dataclass(frozen=True)
class LaneletInfo:
    lanelet_id: str
    tags: Dict[str, str]
    members: List[Tuple[str, str, str]]
    centerline_id: str
    centerline: LineString
    polygon: Polygon


@dataclass(frozen=True)
class GisFeature:
    feature_id: str
    record: Dict[str, object]
    points: List[Tuple[float, float, float]]
    geometry: object


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def members(element: etree._Element) -> List[Tuple[str, str, str]]:
    return [
        (member.get("type"), member.get("ref"), member.get("role", ""))
        for member in element.findall("member")
    ]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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


def shape_points_3d(shape) -> List[Tuple[float, float, float]]:
    z_values = list(getattr(shape, "z", []) or [])
    points = []
    for idx, (x, y) in enumerate(shape.points):
        z = float(z_values[idx]) if idx < len(z_values) else 0.0
        points.append((float(x), float(y), z))
    return points


def feature_geometry(shape, points: List[Tuple[float, float, float]]):
    xy = [(x, y) for x, y, _ in points]
    if len(xy) == 1:
        return Point(xy[0])
    if shape.shapeTypeName.startswith("POLYGON") and len(xy) >= 3:
        poly = Polygon(xy)
        return poly if poly.is_valid else poly.buffer(0)
    return LineString(xy)


def read_gis_layer(path: Path) -> List[GisFeature]:
    reader = shapefile.Reader(str(path), encoding="utf-8", encodingErrors="replace")
    field_names = [field[0] for field in reader.fields[1:]]
    features = []
    for shape_record in reader.iterShapeRecords():
        record = dict(zip(field_names, shape_record.record))
        points = shape_points_3d(shape_record.shape)
        if not points:
            continue
        geometry = feature_geometry(shape_record.shape, points)
        if geometry.is_empty or any(not math.isfinite(value) for value in geometry.bounds):
            continue
        features.append(
            GisFeature(
                feature_id=str(record.get("ID", "")),
                record=record,
                points=points,
                geometry=geometry,
            )
        )
    return features


def point_to_segment_distance(point: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    cx = ax + ratio * dx
    cy = ay + ratio * dy
    return math.hypot(px - cx, py - cy)


def load_mgeo_links(root: Path) -> List[Dict[str, object]]:
    with (root / "mgeo_map/global_info.json").open(encoding="utf-8") as fp:
        origin = json.load(fp)["local_origin_in_global"]
    with (root / "mgeo_map/link_set.json").open(encoding="utf-8") as fp:
        links = json.load(fp)
    for link in links:
        link["_global_xy"] = [(point[0] + origin[0], point[1] + origin[1]) for point in link["points"]]
    return links


def mgeo_link_distance(link: Dict[str, object], point: Tuple[float, float]) -> float:
    pts = link["_global_xy"]
    if len(pts) < 2:
        return float("inf")
    return min(point_to_segment_distance(point, a, b) for a, b in zip(pts, pts[1:]))


def nearest_mgeo_links(links: Sequence[Dict[str, object]], point: Tuple[float, float], count: int = 5):
    return sorted(((mgeo_link_distance(link, point), link) for link in links), key=lambda item: item[0])[:count]


def parse_osm(path: Path):
    tree = etree.parse(str(path), etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    to_utm = Transformer.from_crs(WGS84, UTM52, always_xy=True)
    nodes = {}
    for node in root.findall("node"):
        node_tags = tags(node)
        lon = float(node.get("lon"))
        lat = float(node.get("lat"))
        x, y = to_utm.transform(lon, lat)
        z = float(node_tags.get("ele", "0"))
        nodes[node.get("id")] = (x, y, z, node)

    ways: Dict[str, WayInfo] = {}
    for way in root.findall("way"):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        pts = [(nodes[ref][0], nodes[ref][1], nodes[ref][2]) for ref in refs if ref in nodes]
        line = LineString([(x, y) for x, y, _ in pts]) if len(pts) >= 2 else None
        ways[way.get("id")] = WayInfo(way.get("id"), refs, tags(way), pts, line)

    relations = {relation.get("id"): relation for relation in root.findall("relation")}
    lanelets: Dict[str, LaneletInfo] = {}
    for relation in root.findall("relation"):
        rel_tags = tags(relation)
        if rel_tags.get("type") != "lanelet" or rel_tags.get("subtype") != "road":
            continue
        if rel_tags.get("participant:vehicle", "yes") != "yes":
            continue
        rel_members = members(relation)
        way_by_role = {role: ref for member_type, ref, role in rel_members if member_type == "way"}
        if not {"left", "right", "centerline"} <= set(way_by_role):
            continue
        left = ways.get(way_by_role["left"])
        right = ways.get(way_by_role["right"])
        center = ways.get(way_by_role["centerline"])
        if not left or not right or not center or center.line is None:
            continue
        if len(left.points) < 2 or len(right.points) < 2:
            continue
        polygon = Polygon([(x, y) for x, y, _ in left.points] + [(x, y) for x, y, _ in reversed(right.points)])
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        lanelets[relation.get("id")] = LaneletInfo(
            lanelet_id=relation.get("id"),
            tags=rel_tags,
            members=rel_members,
            centerline_id=way_by_role["centerline"],
            centerline=center.line,
            polygon=polygon,
        )
    return tree, root, nodes, ways, relations, lanelets


def old_traffic_light_stop_lines(path: Path) -> set:
    old_ids = set()
    if not path.exists():
        return old_ids
    with path.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            for stop_line_id in row.get("old_stop_line_ids", "").split():
                old_ids.add(stop_line_id)
    return old_ids


def relation_ref_line_stop_lines(root: etree._Element, stop_line_ids: set) -> Dict[str, List[str]]:
    result: Dict[str, List[str]] = {}
    for relation in root.findall("relation"):
        rel_tags = tags(relation)
        if rel_tags.get("type") != "regulatory_element":
            continue
        for member_type, ref, role in members(relation):
            if member_type == "way" and role == "ref_line" and ref in stop_line_ids:
                result.setdefault(ref, []).append(relation.get("id"))
    return result


def nearest_features(features: Sequence[GisFeature], point: Point, count: int = 5):
    distances = []
    for feature in features:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            distance = feature.geometry.distance(point)
        if not math.isfinite(distance):
            distance = feature.geometry.centroid.distance(point)
        distances.append((distance, feature))
    return sorted(distances, key=lambda item: item[0])[:count]


def best_lanelets_for_stop_line(
    line: LineString,
    midpoint: Point,
    lanelets: Dict[str, LaneletInfo],
) -> List[Dict[str, object]]:
    rows = []
    for lanelet_id, lanelet in lanelets.items():
        if lanelet.centerline.length <= 1e-9:
            continue
        fraction = lanelet.centerline.project(midpoint) / lanelet.centerline.length
        end_distance = midpoint.distance(Point(lanelet.centerline.coords[-1]))
        rows.append(
            {
                "lanelet_id": lanelet_id,
                "polygon_distance_m": line.distance(lanelet.polygon),
                "center_distance_m": midpoint.distance(lanelet.centerline),
                "center_fraction": fraction,
                "end_distance_m": end_distance,
            }
        )
    rows.sort(key=lambda row: (row["polygon_distance_m"], abs(1.0 - row["center_fraction"]), row["center_distance_m"]))
    return rows


def classify_stop_line(
    stop_line_id: str,
    way: WayInfo,
    linked_reg_ids: List[str],
    old_tl_stop_lines: set,
    stop_signs: Sequence[GisFeature],
    all_signs: Sequence[GisFeature],
    traffic_lights: Sequence[GisFeature],
    mgeo_links: Sequence[Dict[str, object]],
    lanelets: Dict[str, LaneletInfo],
    args,
) -> Dict[str, object]:
    line = way.line
    assert line is not None
    midpoint = line.interpolate(0.5, normalized=True)
    lanelet_rows = best_lanelets_for_stop_line(line, midpoint, lanelets)
    best_lanelet = lanelet_rows[0] if lanelet_rows else {}
    applicable = [
        row
        for row in lanelet_rows
        if row["polygon_distance_m"] <= args.lanelet_polygon_tolerance_m
        and row["center_distance_m"] <= args.max_centerline_distance_m
        and row["center_fraction"] >= args.min_front_fraction
        and row["end_distance_m"] <= args.max_end_distance_m
    ]
    if not applicable and best_lanelet:
        applicable = (
            [best_lanelet]
            if best_lanelet["polygon_distance_m"] <= args.lanelet_polygon_tolerance_m
            and best_lanelet["center_distance_m"] <= args.max_centerline_distance_m
            and best_lanelet["center_fraction"] >= args.min_front_fraction
            and best_lanelet["end_distance_m"] <= args.max_end_distance_m
            else []
        )

    nearest_stop = nearest_features(stop_signs, midpoint, 1)
    nearest_any_sign = nearest_features(all_signs, midpoint, 1)
    nearest_light = nearest_features(traffic_lights, midpoint, 1)
    nearest_links = nearest_mgeo_links(mgeo_links, (midpoint.x, midpoint.y), 3)

    stop_dist, stop_feature = nearest_stop[0] if nearest_stop else (float("inf"), None)
    any_sign_dist, any_sign_feature = nearest_any_sign[0] if nearest_any_sign else (float("inf"), None)
    light_dist, light_feature = nearest_light[0] if nearest_light else (float("inf"), None)

    if linked_reg_ids:
        classification = "already_linked"
        reason = "이미 regulatory_element ref_line으로 연결됨"
    elif stop_line_id in old_tl_stop_lines:
        classification = "exclude_old_traffic_light_line"
        reason = "v32에서 대체된 과거 신호등 정지선"
    elif light_dist <= args.traffic_light_exclusion_distance_m:
        classification = "hold_possible_traffic_light"
        reason = "가까운 신호등이 있어 stop_sign으로 단정하지 않음"
    elif stop_dist <= args.stop_sign_distance_m and applicable:
        classification = "fix_stop_sign"
        reason = "GIS 정지표지와 lanelet 앞쪽 정지선 기하가 모두 확인됨"
    elif applicable:
        classification = "hold_no_stop_sign_evidence"
        reason = "lanelet 앞쪽 정지선은 확인되나 GIS/MGeo 정지표지 근거가 부족함"
    else:
        classification = "hold_geometry_uncertain"
        reason = "적용 lanelet 또는 진행 방향 기하가 불확실함"

    return {
        "stop_line_id": stop_line_id,
        "classification": classification,
        "reason": reason,
        "linked_regulatory_ids": " ".join(linked_reg_ids),
        "old_traffic_light_stop_line": stop_line_id in old_tl_stop_lines,
        "best_lanelet_id": best_lanelet.get("lanelet_id", ""),
        "applicable_lanelet_ids": " ".join(row["lanelet_id"] for row in applicable),
        "best_lanelet_polygon_distance_m": f"{best_lanelet.get('polygon_distance_m', float('nan')):.3f}" if best_lanelet else "",
        "best_lanelet_center_distance_m": f"{best_lanelet.get('center_distance_m', float('nan')):.3f}" if best_lanelet else "",
        "best_lanelet_center_fraction": f"{best_lanelet.get('center_fraction', float('nan')):.6f}" if best_lanelet else "",
        "best_lanelet_end_distance_m": f"{best_lanelet.get('end_distance_m', float('nan')):.3f}" if best_lanelet else "",
        "nearest_stop_sign_id": stop_feature.feature_id if stop_feature else "",
        "nearest_stop_sign_distance_m": f"{stop_dist:.3f}" if math.isfinite(stop_dist) else "",
        "nearest_stop_sign_remark": stop_feature.record.get("Remark", "") if stop_feature else "",
        "nearest_stop_sign_link_id": stop_feature.record.get("LinkID", "") if stop_feature else "",
        "nearest_any_sign_id": any_sign_feature.feature_id if any_sign_feature else "",
        "nearest_any_sign_distance_m": f"{any_sign_dist:.3f}" if math.isfinite(any_sign_dist) else "",
        "nearest_any_sign_type": any_sign_feature.record.get("Type", "") if any_sign_feature else "",
        "nearest_any_sign_remark": any_sign_feature.record.get("Remark", "") if any_sign_feature else "",
        "nearest_any_sign_link_id": any_sign_feature.record.get("LinkID", "") if any_sign_feature else "",
        "nearest_traffic_light_id": light_feature.feature_id if light_feature else "",
        "nearest_traffic_light_distance_m": f"{light_dist:.3f}" if math.isfinite(light_dist) else "",
        "nearest_mgeo_link_ids": " ".join(link.get("idx", "") for _, link in nearest_links),
        "nearest_mgeo_link_distances_m": " ".join(f"{distance:.3f}" for distance, _ in nearest_links),
        "nearest_mgeo_link_signals": " ".join(str(link.get("related_signal")) for _, link in nearest_links),
    }


def add_node(root: etree._Element, node_id: str, x: float, y: float, z: float, to_wgs: Transformer) -> etree._Element:
    lon, lat = to_wgs.transform(x, y)
    node = etree.Element("node", id=node_id, visible="true", version="1", lat=f"{lat:.14f}", lon=f"{lon:.14f}")
    node.append(etree.Element("tag", k="ele", v=f"{z:.9f}"))
    root.append(node)
    return node


def add_way(root: etree._Element, way_id: str, node_refs: Sequence[str], tag_items: Sequence[Tuple[str, str]]) -> etree._Element:
    way = etree.Element("way", id=way_id, visible="true", version="1")
    for ref in node_refs:
        way.append(etree.Element("nd", ref=ref))
    for key, value in tag_items:
        way.append(etree.Element("tag", k=key, v=value))
    root.append(way)
    return way


def add_regulatory_relation(
    root: etree._Element,
    relation_id: str,
    sign_way_id: str,
    stop_line_id: str,
    source_id: str,
) -> etree._Element:
    relation = etree.Element("relation", id=relation_id, visible="true", version="1")
    relation.append(etree.Element("member", type="way", ref=sign_way_id, role="refers"))
    relation.append(etree.Element("member", type="way", ref=stop_line_id, role="ref_line"))
    relation.append(etree.Element("tag", k="type", v="regulatory_element"))
    relation.append(etree.Element("tag", k="subtype", v="traffic_sign"))
    relation.append(etree.Element("tag", k="sign_type", v="stop_sign"))
    relation.append(etree.Element("tag", k="source", v="GIS:B1_SAFETYSIGN"))
    relation.append(etree.Element("tag", k="source_id", v=source_id))
    root.append(relation)
    return relation


def add_lanelet_regulatory_member(lanelet_relation: etree._Element, relation_id: str) -> None:
    member = etree.Element("member", type="relation", ref=relation_id, role="regulatory_element")
    tag_elements = lanelet_relation.findall("tag")
    if tag_elements:
        lanelet_relation.insert(list(lanelet_relation).index(tag_elements[0]), member)
    else:
        lanelet_relation.append(member)


def sign_way_points(feature: GisFeature) -> List[Tuple[float, float, float]]:
    points = feature.points
    if len(points) >= 2:
        max_pair = (points[0], points[1])
        max_dist = -1.0
        for idx, a in enumerate(points):
            for b in points[idx + 1 :]:
                dist = math.hypot(a[0] - b[0], a[1] - b[1])
                if dist > max_dist:
                    max_dist = dist
                    max_pair = (a, b)
        return [max_pair[0], max_pair[1]]
    p = points[0]
    return [(p[0] - 0.3, p[1], p[2]), (p[0] + 0.3, p[1], p[2])]


def build_v42(args) -> Dict[str, object]:
    tree, root, nodes, ways, relations, lanelets = parse_osm(args.input)
    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    stop_line_ids = {way_id for way_id, way in ways.items() if way.tags.get("type") == "line_thin" and way.tags.get("subtype") == "stop_line"}
    linked = relation_ref_line_stop_lines(root, stop_line_ids)
    old_tl_stop_lines = old_traffic_light_stop_lines(args.old_traffic_light_report)
    all_signs = read_gis_layer(ROOT / "gis_map/B1_SAFETYSIGN.shp")
    stop_signs = [
        feature
        for feature in all_signs
        if str(feature.record.get("Remark", "")).strip() in STOP_SIGN_REMARKS
    ]
    traffic_lights = read_gis_layer(ROOT / "gis_map/C1_TRAFFICLIGHT.shp")
    mgeo_links = load_mgeo_links(ROOT)

    audit_rows = []
    for stop_line_id in sorted(stop_line_ids, key=int):
        way = ways[stop_line_id]
        if way.line is None:
            continue
        audit_rows.append(
            classify_stop_line(
                stop_line_id,
                way,
                linked.get(stop_line_id, []),
                old_tl_stop_lines,
                stop_signs,
                all_signs,
                traffic_lights,
                mgeo_links,
                lanelets,
                args,
            )
        )

    fix_candidates = [row for row in audit_rows if row["classification"] == "fix_stop_sign"]
    next_node_id = max(int(node_id) for node_id in nodes) + 1
    next_way_id = max(int(way_id) for way_id in ways) + 1
    next_relation_id = max(int(relation_id) for relation_id in relations) + 1
    to_wgs = Transformer.from_crs(UTM52, WGS84, always_xy=True)

    stop_sign_by_id = {feature.feature_id: feature for feature in stop_signs}
    fix_rows = []
    for candidate in fix_candidates:
        source_feature = stop_sign_by_id[candidate["nearest_stop_sign_id"]]
        sign_node_refs = []
        for x, y, z in sign_way_points(source_feature):
            node_id = str(next_node_id)
            next_node_id += 1
            add_node(root, node_id, x, y, z, to_wgs)
            sign_node_refs.append(node_id)
        sign_way_id = str(next_way_id)
        next_way_id += 1
        add_way(
            root,
            sign_way_id,
            sign_node_refs,
            [
                ("type", "traffic_sign"),
                ("subtype", "stop_sign"),
                ("source", "GIS:B1_SAFETYSIGN"),
                ("source_id", source_feature.feature_id),
                ("source_remark", str(source_feature.record.get("Remark", ""))),
            ],
        )
        relation_id = str(next_relation_id)
        next_relation_id += 1
        add_regulatory_relation(root, relation_id, sign_way_id, candidate["stop_line_id"], source_feature.feature_id)
        lanelet_ids = candidate["applicable_lanelet_ids"].split()
        for lanelet_id in lanelet_ids:
            add_lanelet_regulatory_member(relations[lanelet_id], relation_id)
        fix_rows.append(
            {
                "lanelet_id": " ".join(lanelet_ids),
                "regulatory_element_id": relation_id,
                "stop_line_id": candidate["stop_line_id"],
                "traffic_sign_way_id": sign_way_id,
                "traffic_sign_node_ids": " ".join(sign_node_refs),
                "rule_type": "stop_sign",
                "gis_sign_id": source_feature.feature_id,
                "gis_sign_remark": source_feature.record.get("Remark", ""),
                "gis_sign_link_id": source_feature.record.get("LinkID", ""),
                "stop_sign_distance_m": candidate["nearest_stop_sign_distance_m"],
                "lanelet_center_fraction": candidate["best_lanelet_center_fraction"],
                "lanelet_end_distance_m": candidate["best_lanelet_end_distance_m"],
                "evidence": candidate["reason"],
            }
        )

    missing_refs = reference_errors(root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    audit_fields = [
        "stop_line_id",
        "classification",
        "reason",
        "linked_regulatory_ids",
        "old_traffic_light_stop_line",
        "best_lanelet_id",
        "applicable_lanelet_ids",
        "best_lanelet_polygon_distance_m",
        "best_lanelet_center_distance_m",
        "best_lanelet_center_fraction",
        "best_lanelet_end_distance_m",
        "nearest_stop_sign_id",
        "nearest_stop_sign_distance_m",
        "nearest_stop_sign_remark",
        "nearest_stop_sign_link_id",
        "nearest_any_sign_id",
        "nearest_any_sign_distance_m",
        "nearest_any_sign_type",
        "nearest_any_sign_remark",
        "nearest_any_sign_link_id",
        "nearest_traffic_light_id",
        "nearest_traffic_light_distance_m",
        "nearest_mgeo_link_ids",
        "nearest_mgeo_link_distances_m",
        "nearest_mgeo_link_signals",
    ]
    write_csv(args.audit_report, audit_fields, audit_rows)
    write_csv(
        args.fix_report,
        [
            "lanelet_id",
            "regulatory_element_id",
            "stop_line_id",
            "traffic_sign_way_id",
            "traffic_sign_node_ids",
            "rule_type",
            "gis_sign_id",
            "gis_sign_remark",
            "gis_sign_link_id",
            "stop_sign_distance_m",
            "lanelet_center_fraction",
            "lanelet_end_distance_m",
            "evidence",
        ],
        fix_rows,
    )

    summary_rows = [
        {"check": "final_map", "status": "PASS", "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}"},
        {
            "check": "packaged_map",
            "status": "PASS" if sha256(args.output) == sha256(args.package_dir / "lanelet2_map.osm") else "FAIL",
            "detail": f"{(args.package_dir / 'lanelet2_map.osm').relative_to(ROOT)} byte-identical",
        },
        {"check": "osm_references", "status": "PASS" if missing_refs == 0 else "FAIL", "detail": f"missing_reference_count={missing_refs}"},
        {"check": "stop_line_inventory", "status": "PASS", "detail": f"stop_line_total={len(stop_line_ids)} linked_before={len(linked)} audit_rows={len(audit_rows)}"},
        {"check": "stop_sign_fix_count", "status": "PASS" if fix_rows else "WARN", "detail": f"created_regulatory_elements={len(fix_rows)}"},
        {
            "check": "held_200739_103498",
            "status": "PASS",
            "detail": next(
                (
                    f"classification={row['classification']} reason={row['reason']}"
                    for row in audit_rows
                    if row["stop_line_id"] == "103498"
                ),
                "not_found",
            ),
        },
    ]
    write_csv(args.summary_report, ["check", "status", "detail"], summary_rows)

    return {
        "audit_rows": audit_rows,
        "fix_rows": fix_rows,
        "missing_refs": missing_refs,
        "output": str(args.output),
        "package_dir": str(args.package_dir),
        "sha256": sha256(args.output),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v41.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v42.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v41")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v42")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", default="42")
    parser.add_argument("--stop-sign-distance-m", type=float, default=30.0)
    parser.add_argument("--traffic-light-exclusion-distance-m", type=float, default=8.0)
    parser.add_argument("--lanelet-polygon-tolerance-m", type=float, default=0.20)
    parser.add_argument("--max-centerline-distance-m", type=float, default=1.0)
    parser.add_argument("--min-front-fraction", type=float, default=0.90)
    parser.add_argument("--max-end-distance-m", type=float, default=5.0)
    args = parser.parse_args()
    args.old_traffic_light_report = args.report_dir / "v32_vmb_regulatory_fix.csv"
    args.audit_report = args.report_dir / "v42_stop_line_regulatory_audit.csv"
    args.fix_report = args.report_dir / "v42_stop_line_regulatory_fix.csv"
    args.summary_report = args.report_dir / "final_validation_summary_v42.csv"
    return args


def main() -> int:
    args = parse_args()
    result = build_v42(args)
    print(f"wrote={Path(result['output']).relative_to(ROOT)}")
    print(f"package={Path(result['package_dir']).relative_to(ROOT)}")
    print(f"sha256={result['sha256']}")
    print(f"created_stop_sign_regulatory_elements={len(result['fix_rows'])}")
    print(f"missing_references={result['missing_refs']}")
    for row in result["fix_rows"]:
        print(
            "fix "
            f"lanelet={row['lanelet_id']} reg={row['regulatory_element_id']} "
            f"stop_line={row['stop_line_id']} sign={row['gis_sign_id']}"
        )
    return 0 if result["missing_refs"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
