#!/usr/bin/env python3
"""Create v39 from v38 by splitting tollgate lanelets and normalizing speeds."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import math
import shutil
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from lxml import etree
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"
LOCAL_X_OFFSET = 300000.0
LOCAL_Y_OFFSET = 4100000.0

Point3 = Tuple[float, float, float]
CutLine = Tuple[Tuple[float, float], Tuple[float, float]]

TOLLGATE_START: CutLine = ((2665.39, 23818.27), (2669.48, 23818.27))
TOLLGATE_END: CutLine = ((2665.58, 23759.58), (2669.70, 23759.25))

# High-speed course corridor up to the tollgate.  The list follows
# docs/report/kcity_lanelet_workflow/course_autoware_v35_highway_corridor_lanelets.csv.
HIGHWAY_PRE_TOLL_LANELETS = {
    "201086",
    "200101",
    "200099",
    "200152",
    "200142",
    "200128",
    "200137",
    "200138",
    "200186",
    "200102",
    "200187",
    "200188",
    "200141",
    "200134",
    "200130",
    "200127",
    "200148",
    "200147",
    "200360",
    "200185",
}


@dataclass
class SplitResult:
    way_id: str
    downstream_way_id: str
    split_node_id: str
    local_point: Point3
    cut_name: str


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag") if tag.get("k")}


def set_tag(element: etree._Element, key: str, value: str) -> None:
    for tag in element.findall("tag"):
        if tag.get("k") == key:
            tag.set("v", value)
            return
    element.append(etree.Element("tag", k=key, v=value))


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    children = list(way)
    insert_at = 0
    for idx, child in enumerate(children):
        if child.tag == "nd":
            insert_at = idx
            break
    for nd in list(way.findall("nd")):
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


def make_local_coord(nodes: Dict[str, etree._Element], transformer_to_utm: Transformer) -> Dict[str, Point3]:
    coord: Dict[str, Point3] = {}
    for node_id, node in nodes.items():
        x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
        coord[node_id] = (x - LOCAL_X_OFFSET, y - LOCAL_Y_OFFSET, node_ele(node))
    return coord


def line_side(point: Point3, cut_line: CutLine) -> float:
    (ax, ay), (bx, by) = cut_line
    return (bx - ax) * (point[1] - ay) - (by - ay) * (point[0] - ax)


def distance(a: Point3, b: Point3) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def interpolate(a: Point3, b: Point3, t: float) -> Point3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def segment_line_intersection(
    refs: Sequence[str],
    coord: Dict[str, Point3],
    cut_line: CutLine,
    eps: float = 1e-8,
) -> Tuple[int, Optional[str], Point3]:
    candidates: List[Tuple[float, int, Optional[str], Point3]] = []
    accumulated = 0.0
    for idx in range(len(refs) - 1):
        a = coord[refs[idx]]
        b = coord[refs[idx + 1]]
        sa = line_side(a, cut_line)
        sb = line_side(b, cut_line)
        seg_len = distance(a, b)
        if abs(sa) < eps:
            candidates.append((accumulated, idx, refs[idx], a))
        if abs(sb) < eps:
            candidates.append((accumulated + seg_len, idx + 1, refs[idx + 1], b))
        if sa * sb < 0.0:
            t = sa / (sa - sb)
            point = interpolate(a, b, t)
            candidates.append((accumulated + seg_len * t, idx, None, point))
        accumulated += seg_len

    if not candidates:
        raise ValueError(f"cut line does not intersect way refs {refs[0]}..{refs[-1]}")

    _, split_index, existing_node, point = sorted(candidates, key=lambda item: item[0])[0]
    return split_index, existing_node, point


def next_numeric_id(root: etree._Element, element_name: str) -> int:
    return max(int(element.get("id")) for element in root.findall(element_name)) + 1


def append_node(
    root: etree._Element,
    node_id: str,
    local_point: Point3,
    transformer_to_wgs: Transformer,
) -> etree._Element:
    lon, lat = transformer_to_wgs.transform(
        local_point[0] + LOCAL_X_OFFSET,
        local_point[1] + LOCAL_Y_OFFSET,
    )
    node = etree.Element(
        "node",
        id=node_id,
        visible="true",
        version="1",
        lat=f"{lat:.14f}",
        lon=f"{lon:.14f}",
    )
    node.append(etree.Element("tag", k="ele", v=f"{local_point[2]:.9f}"))
    root.append(node)
    return node


def append_way(root: etree._Element, source_way: etree._Element, way_id: str, refs: Sequence[str]) -> etree._Element:
    way = etree.Element(
        "way",
        id=way_id,
        visible=source_way.get("visible", "true"),
        version=source_way.get("version", "1"),
    )
    for ref in refs:
        way.append(etree.Element("nd", ref=ref))
    for tag in source_way.findall("tag"):
        way.append(copy.deepcopy(tag))
    root.append(way)
    return way


def split_way(
    root: etree._Element,
    ways: Dict[str, etree._Element],
    nodes: Dict[str, etree._Element],
    coord: Dict[str, Point3],
    transformer_to_wgs: Transformer,
    next_ids: Dict[str, int],
    split_cache: Dict[Tuple[str, str], SplitResult],
    way_id: str,
    cut_name: str,
    cut_line: CutLine,
) -> SplitResult:
    cache_key = (way_id, cut_name)
    if cache_key in split_cache:
        return split_cache[cache_key]

    way = ways[way_id]
    refs = way_refs(way)
    split_index, existing_node_id, point = segment_line_intersection(refs, coord, cut_line)
    if existing_node_id is None:
        split_node_id = str(next_ids["node"])
        next_ids["node"] += 1
        node = append_node(root, split_node_id, point, transformer_to_wgs)
        nodes[split_node_id] = node
        coord[split_node_id] = point
        upstream_refs = refs[: split_index + 1] + [split_node_id]
        downstream_refs = [split_node_id] + refs[split_index + 1 :]
    else:
        split_node_id = existing_node_id
        point = coord[split_node_id]
        upstream_refs = refs[: split_index + 1]
        downstream_refs = refs[split_index:]

    if len(upstream_refs) < 2 or len(downstream_refs) < 2:
        raise ValueError(f"split would create a one-point way: way={way_id} cut={cut_name}")

    downstream_way_id = str(next_ids["way"])
    next_ids["way"] += 1
    replace_way_refs(way, upstream_refs)
    downstream_way = append_way(root, way, downstream_way_id, downstream_refs)
    ways[downstream_way_id] = downstream_way

    result = SplitResult(way_id, downstream_way_id, split_node_id, point, cut_name)
    split_cache[cache_key] = result
    return result


def build_road_lanelets(root: etree._Element) -> Dict[str, Dict[str, str]]:
    lanelets: Dict[str, Dict[str, str]] = {}
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way" and member.get("role") in {"left", "right", "centerline"}
        }
        if {"left", "right", "centerline"}.issubset(members):
            lanelets[relation.get("id")] = members
    return lanelets


def relation_by_id(root: etree._Element, relation_id: str) -> etree._Element:
    matches = root.xpath(f'//relation[@id="{relation_id}"]')
    if not matches:
        raise KeyError(f"relation {relation_id} not found")
    return matches[0]


def set_relation_way_members(relation: etree._Element, members: Dict[str, str]) -> None:
    for member in relation.findall("member"):
        role = member.get("role")
        if member.get("type") == "way" and role in members:
            member.set("ref", members[role])


def append_lanelet_relation(
    root: etree._Element,
    source_relation: etree._Element,
    relation_id: str,
    members: Dict[str, str],
    speed_limit: str,
) -> etree._Element:
    relation = copy.deepcopy(source_relation)
    relation.set("id", relation_id)
    set_relation_way_members(relation, members)
    set_tag(relation, "speed_limit", speed_limit)
    root.append(relation)
    return relation


def split_lanelet(
    root: etree._Element,
    ways: Dict[str, etree._Element],
    nodes: Dict[str, etree._Element],
    coord: Dict[str, Point3],
    transformer_to_wgs: Transformer,
    next_ids: Dict[str, int],
    split_cache: Dict[Tuple[str, str], SplitResult],
    lanelets: Dict[str, Dict[str, str]],
    lanelet_id: str,
    cut_name: str,
    cut_line: CutLine,
    downstream_speed: str,
) -> Tuple[str, Dict[str, SplitResult]]:
    source_relation = relation_by_id(root, lanelet_id)
    original_members = lanelets[lanelet_id]
    split_results = {
        role: split_way(
            root,
            ways,
            nodes,
            coord,
            transformer_to_wgs,
            next_ids,
            split_cache,
            way_id,
            cut_name,
            cut_line,
        )
        for role, way_id in original_members.items()
    }
    upstream_members = {role: result.way_id for role, result in split_results.items()}
    downstream_members = {role: result.downstream_way_id for role, result in split_results.items()}

    set_relation_way_members(source_relation, upstream_members)
    new_lanelet_id = str(next_ids["relation"])
    next_ids["relation"] += 1
    append_lanelet_relation(root, source_relation, new_lanelet_id, downstream_members, downstream_speed)
    return new_lanelet_id, split_results


def normalize_speeds(
    root: etree._Element,
    toll_lanelets: Iterable[str],
    highway_lanelets: Iterable[str],
) -> Counter:
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") == "lanelet" and relation_tags.get("subtype") == "road":
            set_tag(relation, "speed_limit", "60 km/h")

    for lanelet_id in highway_lanelets:
        matches = root.xpath(f'//relation[@id="{lanelet_id}"]')
        if matches:
            set_tag(matches[0], "speed_limit", "100 km/h")

    for lanelet_id in toll_lanelets:
        set_tag(relation_by_id(root, lanelet_id), "speed_limit", "30 km/h")

    counts = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") == "lanelet" and relation_tags.get("subtype") == "road":
            counts[relation_tags.get("speed_limit", "")] += 1
    return counts


def count_duplicate_ids(root: etree._Element) -> Dict[str, List[Tuple[str, int]]]:
    duplicates = {}
    for kind in ("node", "way", "relation"):
        counts = Counter(element.get("id") for element in root.findall(kind))
        duplicates[kind] = [(item_id, count) for item_id, count in counts.items() if count > 1]
    return duplicates


def reference_errors(root: etree._Element) -> List[str]:
    node_ids = {node.get("id") for node in root.findall("node")}
    way_ids = {way.get("id") for way in root.findall("way")}
    relation_ids = {relation.get("id") for relation in root.findall("relation")}
    errors: List[str] = []
    for way in root.findall("way"):
        for nd in way.findall("nd"):
            if nd.get("ref") not in node_ids:
                errors.append(f"way {way.get('id')} references missing node {nd.get('ref')}")
    for relation in root.findall("relation"):
        for member in relation.findall("member"):
            member_type = member.get("type")
            ref = member.get("ref")
            if member_type == "node" and ref not in node_ids:
                errors.append(f"relation {relation.get('id')} references missing node {ref}")
            elif member_type == "way" and ref not in way_ids:
                errors.append(f"relation {relation.get('id')} references missing way {ref}")
            elif member_type == "relation" and ref not in relation_ids:
                errors.append(f"relation {relation.get('id')} references missing relation {ref}")
    return errors


def signed_area(points: Sequence[Tuple[float, float]]) -> float:
    area = 0.0
    for idx, point in enumerate(points):
        nxt = points[(idx + 1) % len(points)]
        area += point[0] * nxt[1] - nxt[0] * point[1]
    return area / 2.0


def point_in_triangle(point, a, b, c) -> bool:
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

    d1 = sign(point, a, b)
    d2 = sign(point, b, c)
    d3 = sign(point, c, a)
    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (has_neg and has_pos)


def is_convex(prev, curr, nxt, polygon_ccw: bool) -> bool:
    cross = (curr[0] - prev[0]) * (nxt[1] - curr[1]) - (curr[1] - prev[1]) * (nxt[0] - curr[0])
    return cross > 0 if polygon_ccw else cross < 0


def triangulatable(points: Sequence[Tuple[float, float]]) -> bool:
    if len(points) < 3 or abs(signed_area(points)) < 1e-8:
        return False
    remaining = list(range(len(points)))
    polygon_ccw = signed_area(points) > 0
    guard = 0
    while len(remaining) > 3 and guard < len(points) * len(points):
        guard += 1
        clipped = False
        for pos, idx in enumerate(remaining):
            prev_idx = remaining[pos - 1]
            next_idx = remaining[(pos + 1) % len(remaining)]
            prev_point = points[prev_idx]
            curr_point = points[idx]
            next_point = points[next_idx]
            if not is_convex(prev_point, curr_point, next_point, polygon_ccw):
                continue
            if any(
                point_in_triangle(points[other_idx], prev_point, curr_point, next_point)
                for other_idx in remaining
                if other_idx not in {prev_idx, idx, next_idx}
            ):
                continue
            del remaining[pos]
            clipped = True
            break
        if not clipped:
            return False
    return True


def audit_target_lanelet_polygons(
    root: etree._Element,
    ways: Dict[str, etree._Element],
    coord: Dict[str, Point3],
    lanelet_ids: Iterable[str],
) -> List[Dict[str, object]]:
    rows = []
    lanelets = build_road_lanelets(root)
    for lanelet_id in lanelet_ids:
        members = lanelets[lanelet_id]
        left = [coord[ref][:2] for ref in way_refs(ways[members["left"]])]
        right = [coord[ref][:2] for ref in way_refs(ways[members["right"]])]
        polygon = left + list(reversed(right))
        area = abs(signed_area(polygon))
        rows.append(
            {
                "lanelet_id": lanelet_id,
                "left_way": members["left"],
                "right_way": members["right"],
                "center_way": members["centerline"],
                "left_points": len(left),
                "right_points": len(right),
                "area_m2": f"{area:.6f}",
                "triangulatable": triangulatable(polygon),
            }
        )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v38.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v39.osm")
    parser.add_argument("--source-package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v38")
    parser.add_argument("--package-dir", type=Path, default=ROOT.parent / "autoware_data/maps/kcity-v39")
    parser.add_argument("--report-dir", type=Path, default=ROOT / "reports")
    parser.add_argument("--map-version", type=str, default="39")
    args = parser.parse_args()

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.input), parser_xml)
    root = tree.getroot()
    transformer_to_utm = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    transformer_to_wgs = Transformer.from_crs(UTM52, "EPSG:4326", always_xy=True)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", args.map_version)

    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    coord = make_local_coord(nodes, transformer_to_utm)
    lanelets = build_road_lanelets(root)
    next_ids = {
        "node": next_numeric_id(root, "node"),
        "way": next_numeric_id(root, "way"),
        "relation": next_numeric_id(root, "relation"),
    }
    split_cache: Dict[Tuple[str, str], SplitResult] = {}
    split_rows: List[Dict[str, object]] = []
    new_lanelet_ids: Dict[str, str] = {}
    split_results_by_lanelet: Dict[str, Dict[str, SplitResult]] = {}

    for lanelet_id in ("200360", "200185"):
        new_lanelet_id, split_results = split_lanelet(
            root,
            ways,
            nodes,
            coord,
            transformer_to_wgs,
            next_ids,
            split_cache,
            lanelets,
            lanelet_id,
            "tollgate_start",
            TOLLGATE_START,
            "30 km/h",
        )
        new_lanelet_ids[f"{lanelet_id}_toll"] = new_lanelet_id
        split_results_by_lanelet[lanelet_id] = split_results

    lanelets = build_road_lanelets(root)
    for lanelet_id, downstream_speed in (("200088", "60 km/h"), ("200150", "60 km/h"), ("200149", "60 km/h")):
        new_lanelet_id, split_results = split_lanelet(
            root,
            ways,
            nodes,
            coord,
            transformer_to_wgs,
            next_ids,
            split_cache,
            lanelets,
            lanelet_id,
            "tollgate_end",
            TOLLGATE_END,
            downstream_speed,
        )
        new_lanelet_ids[f"{lanelet_id}_after"] = new_lanelet_id
        split_results_by_lanelet[lanelet_id] = split_results

    toll_lanelets = {
        new_lanelet_ids["200360_toll"],
        new_lanelet_ids["200185_toll"],
        "200088",
        "200150",
    }
    speed_counts = normalize_speeds(root, toll_lanelets, HIGHWAY_PRE_TOLL_LANELETS)

    for source_lanelet, role_results in split_results_by_lanelet.items():
        for role, result in role_results.items():
            split_rows.append(
                {
                    "source_lanelet": source_lanelet,
                    "role": role,
                    "cut": result.cut_name,
                    "source_way": result.way_id,
                    "downstream_way": result.downstream_way_id,
                    "split_node": result.split_node_id,
                    "split_x_m": f"{result.local_point[0]:.6f}",
                    "split_y_m": f"{result.local_point[1]:.6f}",
                    "split_z_m": f"{result.local_point[2]:.6f}",
                    "new_lanelet": (
                        new_lanelet_ids.get(f"{source_lanelet}_toll")
                        or new_lanelet_ids.get(f"{source_lanelet}_after")
                    ),
                }
            )

    # Rebuild maps after all appended elements and relation updates.
    nodes = {node.get("id"): node for node in root.findall("node")}
    ways = {way.get("id"): way for way in root.findall("way")}
    coord = make_local_coord(nodes, transformer_to_utm)
    duplicate_ids = count_duplicate_ids(root)
    ref_errors = reference_errors(root)
    target_lanelets = [
        "200360",
        new_lanelet_ids["200360_toll"],
        "200185",
        new_lanelet_ids["200185_toll"],
        "200088",
        new_lanelet_ids["200088_after"],
        "200150",
        new_lanelet_ids["200150_after"],
        "200149",
        new_lanelet_ids["200149_after"],
    ]
    polygon_rows = audit_target_lanelet_polygons(root, ways, coord, target_lanelets)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.package_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.output, args.package_dir / "lanelet2_map.osm")
    for name in ("map_config.yaml", "map_projector_info.yaml", "pointcloud_map.pcd"):
        shutil.copy2(args.source_package_dir / name, args.package_dir / name)

    write_csv(
        args.report_dir / "v39_tollgate_speed_split.csv",
        [
            "source_lanelet",
            "role",
            "cut",
            "source_way",
            "downstream_way",
            "split_node",
            "split_x_m",
            "split_y_m",
            "split_z_m",
            "new_lanelet",
        ],
        split_rows,
    )
    write_csv(
        args.report_dir / "v39_tollgate_target_polygon_audit.csv",
        [
            "lanelet_id",
            "left_way",
            "right_way",
            "center_way",
            "left_points",
            "right_points",
            "area_m2",
            "triangulatable",
        ],
        polygon_rows,
    )

    duplicate_detail = "; ".join(
        f"{kind}:{','.join(f'{item_id}x{count}' for item_id, count in values)}"
        for kind, values in duplicate_ids.items()
        if values
    )
    if not duplicate_detail:
        duplicate_detail = "no duplicate ids"

    relation_counts = Counter()
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        relation_counts[f"{relation_tags.get('type', '')}/{relation_tags.get('subtype', '')}"] += 1

    summary_rows = [
        {"check": "final_map", "status": "PASS", "detail": f"{args.output.relative_to(ROOT)} sha256={sha256(args.output)}"},
        {
            "check": "packaged_map",
            "status": "PASS" if sha256(args.output) == sha256(args.package_dir / "lanelet2_map.osm") else "FAIL",
            "detail": f"{(args.package_dir / 'lanelet2_map.osm').relative_to(ROOT)} byte-identical",
        },
        {
            "check": "duplicate_osm_ids",
            "status": "PASS" if not any(duplicate_ids.values()) else "FAIL",
            "detail": duplicate_detail,
        },
        {
            "check": "osm_references",
            "status": "PASS" if not ref_errors else "FAIL",
            "detail": f"missing_reference_count={len(ref_errors)}",
        },
        {
            "check": "target_lanelet_polygons",
            "status": "PASS" if all(row["triangulatable"] for row in polygon_rows) else "FAIL",
            "detail": f"target_lanelets={','.join(target_lanelets)}",
        },
        {
            "check": "speed_counts",
            "status": "PASS",
            "detail": "; ".join(f"{speed}={count}" for speed, count in sorted(speed_counts.items())),
        },
        {
            "check": "osm_counts",
            "status": "PASS",
            "detail": (
                f"nodes={len(root.findall('node'))} ways={len(root.findall('way'))} "
                f"relations={len(root.findall('relation'))} "
                f"road_lanelets={relation_counts.get('lanelet/road', 0)}"
            ),
        },
    ]
    write_csv(args.report_dir / "final_validation_summary_v39.csv", ["check", "status", "detail"], summary_rows)

    print(f"wrote={args.output.relative_to(ROOT)}")
    print(f"package={args.package_dir.relative_to(ROOT)}")
    print(f"sha256={sha256(args.output)}")
    print(f"new_lanelets={new_lanelet_ids}")
    print("speed_counts=" + "; ".join(f"{speed}:{count}" for speed, count in sorted(speed_counts.items())))
    print(f"duplicate_id_groups={sum(len(values) for values in duplicate_ids.values())}")
    print(f"missing_references={len(ref_errors)}")
    print(f"target_polygon_failures={sum(1 for row in polygon_rows if not row['triangulatable'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
