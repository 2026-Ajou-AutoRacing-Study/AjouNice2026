#!/usr/bin/env python3
"""Create v43 by adding direction-verified traffic-light stop-line rules.

The source layers do not share IDs with Lanelet2, so proximity alone is not
enough.  A fix is accepted only when all of these agree:

* an MGeo node is marked as a stop line and references a GIS traffic light;
* an incoming MGeo link and a Lanelet2 centerline have the same direction;
* the GIS stop line intersects the candidate lanelet polygon and centerline;
* the GIS stop line and traffic light both have exact Lanelet2 way matches;
* the candidate lanelet does not already have a traffic-light regulation.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point

from fix_v41_stop_lines_to_v42 import (
    GisFeature,
    LaneletInfo,
    WayInfo,
    members,
    parse_osm,
    read_gis_layer,
    reference_errors,
    tags,
)


ROOT = Path(__file__).resolve().parents[2]
GIS_STOP_LINE_KIND = "530"
NONSTANDARD_PROVENANCE_TAGS = {"source", "source_id", "source_remark"}


@dataclass(frozen=True)
class MGeoNode:
    node_id: str
    point: Point
    on_stop_line: bool
    traffic_light_id: Optional[str]


@dataclass(frozen=True)
class MGeoLink:
    link_id: str
    from_node_id: str
    to_node_id: str
    line: LineString
    related_signal: Optional[str]


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


def load_mgeo() -> Tuple[Dict[str, MGeoNode], List[MGeoLink]]:
    with (ROOT / "mgeo_map/global_info.json").open(encoding="utf-8") as fp:
        origin = json.load(fp)["local_origin_in_global"]
    with (ROOT / "mgeo_map/node_set.json").open(encoding="utf-8") as fp:
        raw_nodes = json.load(fp)
    with (ROOT / "mgeo_map/link_set.json").open(encoding="utf-8") as fp:
        raw_links = json.load(fp)

    nodes = {}
    for item in raw_nodes:
        point = item["point"]
        nodes[item["idx"]] = MGeoNode(
            node_id=item["idx"],
            point=Point(float(point[0]) + origin[0], float(point[1]) + origin[1]),
            on_stop_line=bool(item.get("on_stop_line")),
            traffic_light_id=item.get("traffic_light_id"),
        )

    links = []
    for item in raw_links:
        xy = [(float(point[0]) + origin[0], float(point[1]) + origin[1]) for point in item["points"]]
        if len(xy) < 2:
            continue
        links.append(
            MGeoLink(
                link_id=item["idx"],
                from_node_id=item["from_node_idx"],
                to_node_id=item["to_node_idx"],
                line=LineString(xy),
                related_signal=item.get("related_signal"),
            )
        )
    return nodes, links


def unit_vector(a: Tuple[float, float], b: Tuple[float, float]) -> Optional[Tuple[float, float]]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return None
    return dx / length, dy / length


def tail_vector(line: LineString, length_m: float = 5.0) -> Optional[Tuple[float, float]]:
    if line.length <= 1e-9:
        return None
    end = line.interpolate(line.length)
    start = line.interpolate(max(0.0, line.length - min(length_m, line.length)))
    return unit_vector((start.x, start.y), (end.x, end.y))


def local_lanelet_vector(lanelet: LaneletInfo, point: Point, length_m: float = 5.0) -> Optional[Tuple[float, float]]:
    line = lanelet.centerline
    if line.length <= 1e-9:
        return None
    distance = line.project(point)
    half = min(length_m / 2.0, line.length / 2.0)
    start = line.interpolate(max(0.0, distance - half))
    end = line.interpolate(min(line.length, distance + half))
    return unit_vector((start.x, start.y), (end.x, end.y))


def sample_tail_distances(link: MGeoLink, lanelet: LaneletInfo, tail_m: float = 12.0) -> Tuple[float, float]:
    start = max(0.0, link.line.length - min(tail_m, link.line.length))
    count = 7
    distances = []
    for idx in range(count):
        distance = start + (link.line.length - start) * idx / (count - 1)
        distances.append(link.line.interpolate(distance).distance(lanelet.centerline))
    return sum(distances) / len(distances), max(distances)


def best_lanelet_for_incoming_link(
    link: MGeoLink,
    stop_node: MGeoNode,
    lanelets: Dict[str, LaneletInfo],
    args: argparse.Namespace,
) -> Tuple[Optional[Dict[str, object]], List[Dict[str, object]]]:
    link_vector = tail_vector(link.line)
    candidates = []
    for lanelet_id, lanelet in lanelets.items():
        stop_distance = stop_node.point.distance(lanelet.centerline)
        if stop_distance > args.max_stop_node_centerline_distance_m:
            continue
        lanelet_vector = local_lanelet_vector(lanelet, stop_node.point)
        if link_vector is None or lanelet_vector is None:
            continue
        direction_dot = link_vector[0] * lanelet_vector[0] + link_vector[1] * lanelet_vector[1]
        mean_distance, max_distance = sample_tail_distances(link, lanelet)
        fraction = lanelet.centerline.project(stop_node.point) / lanelet.centerline.length
        score = mean_distance + 0.25 * max_distance + 2.0 * (1.0 - direction_dot) + stop_distance
        candidates.append(
            {
                "lanelet_id": lanelet_id,
                "stop_node_centerline_distance_m": stop_distance,
                "tail_mean_distance_m": mean_distance,
                "tail_max_distance_m": max_distance,
                "direction_dot": direction_dot,
                "centerline_fraction": fraction,
                "score": score,
            }
        )
    candidates.sort(key=lambda row: (row["score"], int(row["lanelet_id"])))
    accepted = [
        row
        for row in candidates
        if row["direction_dot"] >= args.min_direction_dot
        and row["tail_mean_distance_m"] <= args.max_tail_mean_distance_m
        and row["tail_max_distance_m"] <= args.max_tail_distance_m
    ]
    return (accepted[0] if accepted else None), candidates[:5]


def nearest_feature(features: Sequence[GisFeature], point: Point) -> Tuple[float, Optional[GisFeature]]:
    if not features:
        return float("inf"), None
    distance, feature = min((feature.geometry.distance(point), feature) for feature in features)
    return float(distance), feature


def nearest_way(ways: Sequence[WayInfo], geometry) -> Tuple[float, Optional[WayInfo]]:
    valid = [way for way in ways if way.line is not None]
    if not valid:
        return float("inf"), None
    distance, way = min((way.line.distance(geometry), way) for way in valid)
    return float(distance), way


def matched_traffic_light_ways(
    feature: GisFeature,
    traffic_light_ways: Sequence[WayInfo],
    tolerance_m: float,
) -> Tuple[List[WayInfo], float]:
    distances = sorted(
        ((way.line.distance(feature.geometry), way) for way in traffic_light_ways if way.line is not None),
        key=lambda item: (item[0], int(item[1].way_id)),
    )
    if not distances:
        return [], float("inf")
    nearest = float(distances[0][0])
    matched = [way for distance, way in distances if distance <= tolerance_m]
    return matched, nearest


def regulatory_maps(root: etree._Element):
    relations = {relation.get("id"): relation for relation in root.findall("relation")}
    traffic_light_regs = {
        relation_id: relation
        for relation_id, relation in relations.items()
        if tags(relation).get("type") == "regulatory_element" and tags(relation).get("subtype") == "traffic_light"
    }
    lanelet_regs: Dict[str, List[str]] = defaultdict(list)
    reg_lanelets: Dict[str, List[str]] = defaultdict(list)
    for relation_id, relation in relations.items():
        if tags(relation).get("type") != "lanelet":
            continue
        for member_type, ref, role in members(relation):
            if member_type == "relation" and role == "regulatory_element" and ref in traffic_light_regs:
                lanelet_regs[relation_id].append(ref)
                reg_lanelets[ref].append(relation_id)
    return relations, traffic_light_regs, lanelet_regs, reg_lanelets


def append_lanelet_regulatory_member(lanelet_relation: etree._Element, relation_id: str) -> None:
    member = etree.Element("member", type="relation", ref=relation_id, role="regulatory_element")
    first_tag = lanelet_relation.find("tag")
    if first_tag is None:
        lanelet_relation.append(member)
    else:
        lanelet_relation.insert(list(lanelet_relation).index(first_tag), member)


def append_traffic_light_regulatory_relation(
    root: etree._Element,
    relation_id: str,
    stop_line_id: str,
    traffic_light_way_ids: Sequence[str],
) -> etree._Element:
    relation = etree.Element("relation", id=relation_id, visible="true", version="1")
    relation.append(etree.Element("member", type="way", ref=stop_line_id, role="ref_line"))
    for way_id in sorted(set(traffic_light_way_ids), key=int):
        relation.append(etree.Element("member", type="way", ref=way_id, role="refers"))
    relation.append(etree.Element("tag", k="type", v="regulatory_element"))
    relation.append(etree.Element("tag", k="subtype", v="traffic_light"))
    root.append(relation)
    return relation


def remove_nonstandard_provenance_tags(root: etree._Element) -> int:
    removed = 0
    for element in list(root.findall("way")) + list(root.findall("relation")):
        for tag in list(element.findall("tag")):
            if tag.get("k") in NONSTANDARD_PROVENANCE_TAGS:
                element.remove(tag)
                removed += 1
    return removed


def deduplicate_stop_sign_facilities(root: etree._Element) -> Tuple[int, int, int]:
    node_elements = {node.get("id"): node for node in root.findall("node")}
    sign_ways = []
    for way in root.findall("way"):
        way_tags = tags(way)
        if way_tags.get("type") == "traffic_sign" and way_tags.get("subtype") == "stop_sign":
            sign_ways.append(way)

    by_geometry: Dict[Tuple[Tuple[str, str, str], ...], List[etree._Element]] = defaultdict(list)
    for way in sign_ways:
        signature = []
        for nd in way.findall("nd"):
            node = node_elements[nd.get("ref")]
            signature.append(
                (
                    node.get("lat", ""),
                    node.get("lon", ""),
                    tags(node).get("ele", ""),
                )
            )
        by_geometry[tuple(signature)].append(way)

    duplicate_to_canonical = {}
    duplicate_node_ids = set()
    for grouped_ways in by_geometry.values():
        if len(grouped_ways) < 2:
            continue
        grouped_ways.sort(key=lambda way: int(way.get("id")))
        canonical = grouped_ways[0]
        for duplicate in grouped_ways[1:]:
            duplicate_to_canonical[duplicate.get("id")] = canonical.get("id")
            duplicate_node_ids.update(nd.get("ref") for nd in duplicate.findall("nd"))

    replaced_members = 0
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            canonical_id = duplicate_to_canonical.get(member.get("ref"))
            if member.get("type") == "way" and canonical_id:
                member.set("ref", canonical_id)
                replaced_members += 1

    way_elements = {way.get("id"): way for way in root.findall("way")}
    for duplicate_id in duplicate_to_canonical:
        root.remove(way_elements[duplicate_id])

    used_node_ids = {nd.get("ref") for way in root.findall("way") for nd in way.findall("nd")}
    removed_nodes = 0
    for node_id in duplicate_node_ids - used_node_ids:
        root.remove(node_elements[node_id])
        removed_nodes += 1
    return len(duplicate_to_canonical), removed_nodes, replaced_members


def remove_single_use_consecutive_close_nodes(root: etree._Element, tolerance_m: float = 0.05) -> int:
    node_elements = {node.get("id"): node for node in root.findall("node")}
    use_count: Dict[str, int] = defaultdict(int)
    for way in root.findall("way"):
        for nd in way.findall("nd"):
            use_count[nd.get("ref")] += 1

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    xy = {
        node_id: to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        for node_id, node in node_elements.items()
    }
    removed_node_ids = set()
    for way in root.findall("way"):
        nd_elements = list(way.findall("nd"))
        if len(nd_elements) <= 2:
            continue
        for index in range(len(nd_elements) - 1):
            first_id = nd_elements[index].get("ref")
            second_id = nd_elements[index + 1].get("ref")
            if first_id in removed_node_ids or second_id in removed_node_ids:
                continue
            distance = math.hypot(xy[first_id][0] - xy[second_id][0], xy[first_id][1] - xy[second_id][1])
            if distance >= tolerance_m:
                continue
            if use_count[second_id] == 1 and index + 1 < len(nd_elements) - 1:
                way.remove(nd_elements[index + 1])
                removed_node_ids.add(second_id)

    for node_id in removed_node_ids:
        root.remove(node_elements[node_id])
    return len(removed_node_ids)


def candidate_audit(args: argparse.Namespace):
    tree, root, nodes, ways, relations, lanelets = parse_osm(args.input)
    mgeo_nodes, mgeo_links = load_mgeo()

    gis_stop_lines = [
        feature
        for feature in read_gis_layer(ROOT / "gis_map/B2_SURFACELINEMARK.shp")
        if str(feature.record.get("Kind", "")).strip() == GIS_STOP_LINE_KIND
    ]
    gis_traffic_lights = {
        feature.feature_id: feature
        for feature in read_gis_layer(ROOT / "gis_map/C1_TRAFFICLIGHT.shp")
    }
    physical_stop_line_ways = [
        way
        for way in ways.values()
        if way.tags.get("type") == "line_thin" and way.tags.get("subtype") == "stop_line" and way.line is not None
    ]
    traffic_light_ways = [
        way for way in ways.values() if way.tags.get("type") == "traffic_light" and way.line is not None
    ]

    _, traffic_light_regs, lanelet_regs, _ = regulatory_maps(root)
    incoming_by_node: Dict[str, List[MGeoLink]] = defaultdict(list)
    for link in mgeo_links:
        incoming_by_node[link.to_node_id].append(link)

    rows = []
    accepted_parts: Dict[Tuple[str, str], Dict[str, object]] = {}
    for node in sorted(mgeo_nodes.values(), key=lambda item: item.node_id):
        if not node.on_stop_line or not node.traffic_light_id:
            continue
        c1_feature = gis_traffic_lights.get(node.traffic_light_id)
        stop_distance, gis_stop_line = nearest_feature(gis_stop_lines, node.point)
        stop_way_distance = float("inf")
        stop_way = None
        if gis_stop_line is not None:
            stop_way_distance, stop_way = nearest_way(physical_stop_line_ways, gis_stop_line.geometry)
        light_way_matches = []
        light_way_distance = float("inf")
        if c1_feature is not None:
            light_way_matches, light_way_distance = matched_traffic_light_ways(
                c1_feature,
                traffic_light_ways,
                args.gis_way_match_tolerance_m,
            )

        incoming = incoming_by_node.get(node.node_id, [])
        if not incoming:
            rows.append(
                {
                    "mgeo_stop_node_id": node.node_id,
                    "mgeo_link_id": "",
                    "traffic_light_id": node.traffic_light_id,
                    "lanelet_id": "",
                    "status": "hold_no_incoming_link",
                    "reason": "MGeo 정지 노드로 들어오는 링크가 없음",
                }
            )
            continue

        for link in sorted(incoming, key=lambda item: item.link_id):
            best, alternatives = best_lanelet_for_incoming_link(link, node, lanelets, args)
            lanelet_id = best["lanelet_id"] if best else ""
            lanelet = lanelets.get(lanelet_id) if lanelet_id else None
            existing_reg_ids = lanelet_regs.get(lanelet_id, [])
            stop_line_intersects = bool(
                lanelet is not None
                and stop_way is not None
                and stop_way.line is not None
                and stop_way.line.distance(lanelet.polygon) <= args.lanelet_stop_line_tolerance_m
            )
            stop_line_centerline_distance = (
                stop_way.line.distance(lanelet.centerline)
                if lanelet is not None and stop_way is not None and stop_way.line is not None
                else float("inf")
            )
            stop_line_crosses_centerline = (
                stop_line_centerline_distance <= args.lanelet_stop_line_tolerance_m
            )

            failures = []
            if best is None:
                failures.append("동일 방향 lanelet 중심선 매칭 실패")
            if c1_feature is None:
                failures.append("MGeo traffic_light_id가 GIS C1에 없음")
            if gis_stop_line is None or stop_distance > args.max_mgeo_stop_line_distance_m:
                failures.append("MGeo 정지 노드와 GIS 정지선 거리 초과")
            if stop_way is None or stop_way_distance > args.gis_way_match_tolerance_m:
                failures.append("GIS 정지선과 Lanelet2 way 매칭 실패")
            if not stop_line_intersects:
                failures.append("정지선이 후보 lanelet 면적을 가로지르지 않음")
            if not stop_line_crosses_centerline:
                failures.append("정지선이 후보 lanelet 중심선을 가로지르지 않음")
            if not light_way_matches:
                failures.append("GIS 신호등과 Lanelet2 traffic_light way 매칭 실패")

            if existing_reg_ids:
                status = "covered_existing_traffic_light_regulatory"
                reason = "lanelet에 기존 traffic_light regulatory_element가 있음"
            elif failures:
                status = "hold"
                reason = "; ".join(failures)
            else:
                status = "fix_traffic_light"
                if stop_distance <= args.strict_mgeo_stop_line_distance_m:
                    reason = "MGeo 방향, GIS 정지선, GIS 신호등, lanelet 교차가 모두 일치"
                else:
                    reason = (
                        "MGeo 정지 노드와 GIS 정지선은 2 m 이내이며, 진행 방향, "
                        "lanelet 교차, GIS 신호등, Lanelet2 신호등이 모두 일치"
                    )
                key = (lanelet_id, stop_way.way_id)
                part = accepted_parts.setdefault(
                    key,
                    {
                        "lanelet_id": lanelet_id,
                        "stop_line_id": stop_way.way_id,
                        "mgeo_stop_node_ids": set(),
                        "mgeo_link_ids": set(),
                        "gis_stop_line_ids": set(),
                        "gis_traffic_light_ids": set(),
                        "traffic_light_way_ids": set(),
                        "max_mgeo_stop_line_distance_m": 0.0,
                    },
                )
                part["mgeo_stop_node_ids"].add(node.node_id)
                part["mgeo_link_ids"].add(link.link_id)
                part["gis_stop_line_ids"].add(gis_stop_line.feature_id)
                part["gis_traffic_light_ids"].add(node.traffic_light_id)
                part["traffic_light_way_ids"].update(way.way_id for way in light_way_matches)
                part["max_mgeo_stop_line_distance_m"] = max(
                    float(part["max_mgeo_stop_line_distance_m"]), stop_distance
                )

            rows.append(
                {
                    "mgeo_stop_node_id": node.node_id,
                    "mgeo_link_id": link.link_id,
                    "traffic_light_id": node.traffic_light_id,
                    "lanelet_id": lanelet_id,
                    "status": status,
                    "reason": reason,
                    "existing_regulatory_ids": " ".join(sorted(existing_reg_ids, key=int)),
                    "gis_stop_line_id": gis_stop_line.feature_id if gis_stop_line else "",
                    "stop_line_id": stop_way.way_id if stop_way else "",
                    "mgeo_to_gis_stop_line_distance_m": f"{stop_distance:.3f}" if math.isfinite(stop_distance) else "",
                    "gis_to_osm_stop_line_distance_m": f"{stop_way_distance:.3f}" if math.isfinite(stop_way_distance) else "",
                    "stop_line_intersects_lanelet": stop_line_intersects,
                    "stop_line_centerline_distance_m": (
                        f"{stop_line_centerline_distance:.3f}"
                        if math.isfinite(stop_line_centerline_distance)
                        else ""
                    ),
                    "stop_line_crosses_centerline": stop_line_crosses_centerline,
                    "traffic_light_way_ids": " ".join(sorted((way.way_id for way in light_way_matches), key=int)),
                    "gis_to_osm_traffic_light_distance_m": f"{light_way_distance:.3f}" if math.isfinite(light_way_distance) else "",
                    "stop_node_centerline_distance_m": f"{best['stop_node_centerline_distance_m']:.3f}" if best else "",
                    "tail_mean_distance_m": f"{best['tail_mean_distance_m']:.3f}" if best else "",
                    "tail_max_distance_m": f"{best['tail_max_distance_m']:.3f}" if best else "",
                    "direction_dot": f"{best['direction_dot']:.6f}" if best else "",
                    "centerline_fraction": f"{best['centerline_fraction']:.6f}" if best else "",
                    "lanelet_alternatives": " ".join(
                        f"{item['lanelet_id']}:{item['score']:.2f}:{item['direction_dot']:.3f}"
                        for item in alternatives
                    ),
                }
            )

    accepted = []
    for key in sorted(
        accepted_parts,
        key=lambda item: (
            accepted_parts[item]["max_mgeo_stop_line_distance_m"]
            > args.strict_mgeo_stop_line_distance_m,
            int(item[0]),
            int(item[1]),
        ),
    ):
        item = accepted_parts[key]
        accepted.append(
            {
                field: " ".join(sorted(value, key=lambda x: int(x) if str(x).isdigit() else str(x)))
                if isinstance(value, set)
                else value
                for field, value in item.items()
            }
        )
    return tree, root, relations, traffic_light_regs, rows, accepted


def build_v43(args: argparse.Namespace) -> Dict[str, object]:
    tree, root, relations, traffic_light_regs, audit_rows, accepted = candidate_audit(args)
    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    next_relation_id = max(int(relation.get("id")) for relation in root.findall("relation")) + 1
    fix_rows = []
    if not args.dry_run:
        for item in accepted:
            relation_id = str(next_relation_id)
            next_relation_id += 1
            append_traffic_light_regulatory_relation(
                root,
                relation_id,
                item["stop_line_id"],
                item["traffic_light_way_ids"].split(),
            )
            append_lanelet_regulatory_member(relations[item["lanelet_id"]], relation_id)
            fix_rows.append(
                {
                    "lanelet_id": item["lanelet_id"],
                    "regulatory_element_id": relation_id,
                    "stop_line_id": item["stop_line_id"],
                    "rule_type": "traffic_light",
                    "traffic_light_way_ids": item["traffic_light_way_ids"],
                    "gis_traffic_light_ids": item["gis_traffic_light_ids"],
                    "gis_stop_line_ids": item["gis_stop_line_ids"],
                    "mgeo_stop_node_ids": item["mgeo_stop_node_ids"],
                    "mgeo_link_ids": item["mgeo_link_ids"],
                    "mgeo_stop_line_max_distance_m": (
                        f"{float(item['max_mgeo_stop_line_distance_m']):.3f}"
                    ),
                    "evidence": (
                        "MGeo 진입 방향, GIS 정지선, GIS 신호등, lanelet 교차 일치"
                        if float(item["max_mgeo_stop_line_distance_m"])
                        <= args.strict_mgeo_stop_line_distance_m
                        else (
                            "MGeo 정지 노드-GIS 정지선 2 m 이내, 진입 방향, "
                            "lanelet 교차, GIS 신호등, Lanelet2 신호등 일치"
                        )
                    ),
                }
            )

        removed_tags = remove_nonstandard_provenance_tags(root)
        duplicate_sign_ways, duplicate_sign_nodes, replaced_sign_members = deduplicate_stop_sign_facilities(root)
        removed_close_nodes = remove_single_use_consecutive_close_nodes(root)
        missing_refs = reference_errors(root)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

        args.package_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
        for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
            shutil.copy2(args.source_package_dir / name, args.package_dir / name)
    else:
        removed_tags = 0
        duplicate_sign_ways = 0
        duplicate_sign_nodes = 0
        replaced_sign_members = 0
        removed_close_nodes = 0
        missing_refs = reference_errors(root)

    audit_fields = [
        "mgeo_stop_node_id",
        "mgeo_link_id",
        "traffic_light_id",
        "lanelet_id",
        "status",
        "reason",
        "existing_regulatory_ids",
        "gis_stop_line_id",
        "stop_line_id",
        "mgeo_to_gis_stop_line_distance_m",
        "gis_to_osm_stop_line_distance_m",
        "stop_line_intersects_lanelet",
        "stop_line_centerline_distance_m",
        "stop_line_crosses_centerline",
        "traffic_light_way_ids",
        "gis_to_osm_traffic_light_distance_m",
        "stop_node_centerline_distance_m",
        "tail_mean_distance_m",
        "tail_max_distance_m",
        "direction_dot",
        "centerline_fraction",
        "lanelet_alternatives",
    ]
    write_csv(args.audit_report, audit_fields, audit_rows)
    write_csv(
        args.fix_report,
        [
            "lanelet_id",
            "regulatory_element_id",
            "stop_line_id",
            "rule_type",
            "traffic_light_way_ids",
            "gis_traffic_light_ids",
            "gis_stop_line_ids",
            "mgeo_stop_node_ids",
            "mgeo_link_ids",
            "mgeo_stop_line_max_distance_m",
            "evidence",
        ],
        fix_rows if not args.dry_run else [
            {
                "lanelet_id": item["lanelet_id"],
                "regulatory_element_id": "DRY_RUN",
                "stop_line_id": item["stop_line_id"],
                "rule_type": "traffic_light",
                "traffic_light_way_ids": item["traffic_light_way_ids"],
                "gis_traffic_light_ids": item["gis_traffic_light_ids"],
                "gis_stop_line_ids": item["gis_stop_line_ids"],
                "mgeo_stop_node_ids": item["mgeo_stop_node_ids"],
                "mgeo_link_ids": item["mgeo_link_ids"],
                "mgeo_stop_line_max_distance_m": (
                    f"{float(item['max_mgeo_stop_line_distance_m']):.3f}"
                ),
                "evidence": "DRY_RUN",
            }
            for item in accepted
        ],
    )

    if not args.dry_run:
        package_map = args.package_dir / "lanelet2_map.osm"
        summary_rows = [
            {"check": "final_map", "status": "PASS", "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}"},
            {
                "check": "packaged_map",
                "status": "PASS" if sha256(args.output) == sha256(package_map) else "FAIL",
                "detail": f"{package_map.relative_to(ROOT)} byte-identical",
            },
            {"check": "osm_references", "status": "PASS" if missing_refs == 0 else "FAIL", "detail": f"missing_reference_count={missing_refs}"},
            {"check": "traffic_light_fix_count", "status": "PASS", "detail": f"created_regulatory_elements={len(fix_rows)}"},
            {"check": "removed_nonstandard_provenance_tags", "status": "PASS", "detail": f"removed_tag_count={removed_tags}"},
            {
                "check": "deduplicated_stop_sign_facilities",
                "status": "PASS",
                "detail": (
                    f"removed_ways={duplicate_sign_ways} removed_nodes={duplicate_sign_nodes} "
                    f"replaced_members={replaced_sign_members}"
                ),
            },
            {
                "check": "removed_redundant_close_nodes",
                "status": "PASS",
                "detail": f"removed_node_count={removed_close_nodes}",
            },
        ]
        write_csv(args.summary_report, ["check", "status", "detail"], summary_rows)

    return {
        "audit_rows": audit_rows,
        "accepted": accepted,
        "fix_rows": fix_rows,
        "removed_tags": removed_tags,
        "duplicate_sign_ways": duplicate_sign_ways,
        "duplicate_sign_nodes": duplicate_sign_nodes,
        "replaced_sign_members": replaced_sign_members,
        "removed_close_nodes": removed_close_nodes,
        "missing_refs": missing_refs,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v42.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v43.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v42")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v43")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", default="43")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-stop-node-centerline-distance-m", type=float, default=1.0)
    parser.add_argument("--max-tail-mean-distance-m", type=float, default=0.75)
    parser.add_argument("--max-tail-distance-m", type=float, default=1.5)
    parser.add_argument("--min-direction-dot", type=float, default=0.85)
    parser.add_argument("--strict-mgeo-stop-line-distance-m", type=float, default=1.0)
    parser.add_argument("--max-mgeo-stop-line-distance-m", type=float, default=2.0)
    parser.add_argument("--gis-way-match-tolerance-m", type=float, default=0.15)
    parser.add_argument("--lanelet-stop-line-tolerance-m", type=float, default=0.15)
    args = parser.parse_args()
    args.audit_report = args.report_dir / "v43_stop_line_regulatory_audit.csv"
    args.fix_report = args.report_dir / "v43_stop_line_regulatory_fix.csv"
    args.summary_report = args.report_dir / "final_validation_summary_v43.csv"
    return args


def main() -> int:
    args = parse_args()
    result = build_v43(args)
    print(f"dry_run={args.dry_run}")
    print(f"audit_rows={len(result['audit_rows'])}")
    print(f"accepted_fixes={len(result['accepted'])}")
    print(f"missing_references={result['missing_refs']}")
    for item in result["accepted"]:
        print(
            "candidate "
            f"lanelet={item['lanelet_id']} stop_line={item['stop_line_id']} "
            f"traffic_lights={item['gis_traffic_light_ids']} mgeo_links={item['mgeo_link_ids']}"
        )
    if not args.dry_run:
        print(f"wrote={args.output.relative_to(ROOT)}")
        print(f"package={args.package_dir.relative_to(ROOT)}")
        print(f"removed_nonstandard_provenance_tags={result['removed_tags']}")
        print(
            "deduplicated_stop_sign_facilities="
            f"ways:{result['duplicate_sign_ways']} nodes:{result['duplicate_sign_nodes']} "
            f"members:{result['replaced_sign_members']}"
        )
        print(f"removed_redundant_close_nodes={result['removed_close_nodes']}")
    return 0 if result["missing_refs"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
