#!/usr/bin/env python3
"""Create a v32 Lanelet2 map that clears VMB traffic-light checks.

The script keeps v31 intact and makes a new versioned OSM. It fixes three VMB
classes:

* one traffic-light regulatory element per controlled lanelet;
* generated stop lines that cross the controlled lanelet geometry;
* abrupt z jumps caused by shared ground nodes in traffic-light/virtual ways.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Sequence, Tuple

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point


ROOT = Path(__file__).resolve().parents[2]
UTM52 = "EPSG:32652"

VMB_REVERSE_TRAFFIC_LIGHT_WAYS = {
    "103815",
    "103816",
    "103818",
    "103819",
    "103822",
    "103823",
    "103824",
    "103825",
    "103826",
    "103827",
    "103828",
    "103829",
    "103830",
    "103831",
    "103832",
    "103833",
    "103834",
    "103835",
    "103836",
    "103837",
    "103838",
    "103840",
    "103841",
    "103842",
    "103843",
    "103844",
    "103845",
    "103846",
    "103847",
    "103848",
    "103849",
    "103850",
    "103851",
    "103852",
    "103853",
    "103854",
    "103855",
    "103856",
    "103858",
    "103859",
    "103860",
    "103862",
    "103863",
    "103864",
    "103865",
    "103866",
    "103867",
    "103868",
    "103869",
    "103871",
    "103872",
    "103873",
    "103874",
    "103875",
    "103877",
    "103878",
    "103879",
    "103880",
    "103881",
    "103882",
    "103883",
    "103884",
    "103885",
    "103886",
    "103887",
    "103888",
    "103889",
    "103890",
    "103891",
    "103892",
    "103893",
    "103894",
    "103895",
    "103896",
    "103897",
    "103898",
    "103899",
    "103900",
    "103901",
    "103903",
    "103904",
    "103906",
    "103907",
    "103909",
    "103910",
    "103911",
    "103912",
    "103913",
    "103915",
    "103916",
    "103917",
    "103918",
    "103921",
    "103922",
    "103923",
    "103924",
    "103925",
    "103926",
    "103928",
    "103929",
    "103930",
    "103931",
    "103932",
    "103933",
    "103934",
    "103936",
    "103937",
}


def tags(element: etree._Element) -> Dict[str, str]:
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def replace_way_refs(way: etree._Element, refs: Sequence[str]) -> None:
    nd_elements = way.findall("nd")
    if not nd_elements:
        return
    insert_at = list(way).index(nd_elements[0])
    for nd in nd_elements:
        way.remove(nd)
    for offset, ref in enumerate(refs):
        way.insert(insert_at + offset, etree.Element("nd", ref=ref))


def replace_relation_members(relation: etree._Element, members: Sequence[Tuple[str, str, str]]) -> None:
    member_elements = relation.findall("member")
    if member_elements:
        insert_at = list(relation).index(member_elements[0])
    else:
        tag_elements = relation.findall("tag")
        insert_at = list(relation).index(tag_elements[0]) if tag_elements else len(relation)
    for member in member_elements:
        relation.remove(member)
    for offset, (member_type, ref, role) in enumerate(members):
        relation.insert(insert_at + offset, etree.Element("member", type=member_type, ref=ref, role=role))


def append_node(root: etree._Element, node_id: str, lat: float, lon: float, ele: float) -> etree._Element:
    node = etree.Element("node", id=node_id, visible="true", version="1", lat=f"{lat:.11f}", lon=f"{lon:.11f}")
    node.append(etree.Element("tag", k="ele", v=f"{ele:.3f}"))
    root.append(node)
    return node


def append_stop_line_way(root: etree._Element, way_id: str, node_refs: Sequence[str]) -> etree._Element:
    way = etree.Element("way", id=way_id, visible="true", version="1")
    for ref in node_refs:
        way.append(etree.Element("nd", ref=ref))
    way.append(etree.Element("tag", k="type", v="line_thin"))
    way.append(etree.Element("tag", k="subtype", v="stop_line"))
    root.append(way)
    return way


def node_ele(node: etree._Element) -> float:
    node_tags = tags(node)
    return float(node_tags.get("ele", "0"))


def way_refs(way: etree._Element) -> List[str]:
    return [nd.get("ref") for nd in way.findall("nd")]


def point3_for_ref(
    ref: str,
    node_elements: Dict[str, etree._Element],
    transformer_to_utm: Transformer,
) -> Tuple[float, float, float]:
    node = node_elements[ref]
    x, y = transformer_to_utm.transform(float(node.get("lon")), float(node.get("lat")))
    return x, y, node_ele(node)


def line2_for_way(
    way: etree._Element,
    node_elements: Dict[str, etree._Element],
    transformer_to_utm: Transformer,
) -> LineString:
    return LineString(
        [
            point3_for_ref(ref, node_elements, transformer_to_utm)[:2]
            for ref in way_refs(way)
            if ref in node_elements
        ]
    )


def interpolate_way_3d(
    way: etree._Element,
    fraction: float,
    node_elements: Dict[str, etree._Element],
    transformer_to_utm: Transformer,
) -> Tuple[float, float, float]:
    points = [point3_for_ref(ref, node_elements, transformer_to_utm) for ref in way_refs(way) if ref in node_elements]
    if not points:
        raise ValueError(f"way {way.get('id')} has no usable points")
    if len(points) == 1:
        return points[0]

    segment_lengths = []
    total = 0.0
    for start, end in zip(points, points[1:]):
        length = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
        segment_lengths.append(length)
        total += length
    if total <= 0:
        return points[0]

    target = max(0.0, min(1.0, fraction)) * total
    walked = 0.0
    for idx, length in enumerate(segment_lengths):
        if walked + length >= target:
            local = 0.0 if length <= 0 else (target - walked) / length
            start = points[idx]
            end = points[idx + 1]
            return (
                start[0] + (end[0] - start[0]) * local,
                start[1] + (end[1] - start[1]) * local,
                start[2] + (end[2] - start[2]) * local,
            )
        walked += length
    return points[-1]


def relation_members(relation: etree._Element) -> List[Tuple[str, str, str]]:
    return [(member.get("type"), member.get("ref"), member.get("role")) for member in relation.findall("member")]


def clone_node_with_ele(
    root: etree._Element,
    original_ref: str,
    new_id: str,
    ele: float,
    node_elements: Dict[str, etree._Element],
) -> etree._Element:
    original = node_elements[original_ref]
    return append_node(root, new_id, float(original.get("lat")), float(original.get("lon")), ele)


def next_numeric_id(elements: Iterable[etree._Element]) -> int:
    return max(int(element.get("id")) for element in elements) + 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v31.osm")
    parser.add_argument("--output", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v32.osm")
    parser.add_argument(
        "--reg-report",
        type=Path,
        default=ROOT / "reports/v32_vmb_regulatory_fix.csv",
    )
    parser.add_argument(
        "--z-report",
        type=Path,
        default=ROOT / "reports/v32_vmb_z_fix.csv",
    )
    parser.add_argument("--map-version", default="32")
    parser.add_argument("--stopline-margin-m", type=float, default=0.75)
    parser.add_argument("--stopline-end-inset-m", type=float, default=0.20)
    args = parser.parse_args()

    parser_xml = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(args.input), parser_xml)
    root = tree.getroot()

    transformer_to_utm = Transformer.from_crs("EPSG:4326", UTM52, always_xy=True)
    transformer_to_wgs = Transformer.from_crs(UTM52, "EPSG:4326", always_xy=True)

    meta = root.find("MetaInfo")
    if meta is not None:
        meta.set("map_version", str(args.map_version))

    node_elements = {node.get("id"): node for node in root.findall("node")}
    way_elements = {way.get("id"): way for way in root.findall("way")}
    relation_elements = {relation.get("id"): relation for relation in root.findall("relation")}

    next_node_id = next_numeric_id(root.findall("node"))
    next_way_id = next_numeric_id(root.findall("way"))

    z_rows = []

    def add_cloned_node(original_ref: str, ele: float, reason: str, way_id: str) -> str:
        nonlocal next_node_id
        new_ref = str(next_node_id)
        next_node_id += 1
        clone = clone_node_with_ele(root, original_ref, new_ref, ele, node_elements)
        node_elements[new_ref] = clone
        z_rows.append(
            {
                "way_id": way_id,
                "old_node_id": original_ref,
                "new_node_id": new_ref,
                "old_ele": f"{node_ele(node_elements[original_ref]):.3f}",
                "new_ele": f"{ele:.3f}",
                "reason": reason,
            }
        )
        return new_ref

    # The node is correct for road geometry but incorrect for this virtual way.
    virtual_way = way_elements["103306"]
    virtual_refs = way_refs(virtual_way)
    idx_38944 = virtual_refs.index("38944")
    before_ele = node_ele(node_elements[virtual_refs[idx_38944 - 1]])
    after_ele = node_ele(node_elements[virtual_refs[idx_38944 + 1]])
    virtual_clone = add_cloned_node("38944", (before_ele + after_ele) / 2.0, "virtual_way_shared_ground_node", "103306")
    virtual_refs[idx_38944] = virtual_clone
    replace_way_refs(virtual_way, virtual_refs)

    # These traffic-light lines mixed a shared ground node with a high light node.
    for way_id in ("103817", "103828", "103839", "103861"):
        way = way_elements[way_id]
        refs = way_refs(way)
        if len(refs) != 2:
            continue
        first_ele = node_ele(node_elements[refs[0]])
        second_ele = node_ele(node_elements[refs[1]])
        if abs(first_ele - second_ele) <= 2.0:
            continue
        if first_ele < second_ele:
            low_idx, high_idx = 0, 1
        else:
            low_idx, high_idx = 1, 0
        clone_ref = add_cloned_node(
            refs[low_idx],
            node_ele(node_elements[refs[high_idx]]),
            "traffic_light_shared_ground_node",
            way_id,
        )
        refs[low_idx] = clone_ref
        replace_way_refs(way, refs)

    reversed_tl_count = 0
    for way_id in sorted(VMB_REVERSE_TRAFFIC_LIGHT_WAYS, key=int):
        way = way_elements.get(way_id)
        if way is None:
            continue
        refs = way_refs(way)
        if len(refs) >= 2:
            replace_way_refs(way, list(reversed(refs)))
            reversed_tl_count += 1

    traffic_light_regs = {
        relation_id
        for relation_id, relation in relation_elements.items()
        if tags(relation).get("type") == "regulatory_element" and tags(relation).get("subtype") == "traffic_light"
    }

    regs_by_lanelet: Dict[str, List[str]] = {}
    reg_to_lanelet: Dict[str, str] = {}
    for relation_id, relation in relation_elements.items():
        if tags(relation).get("type") != "lanelet":
            continue
        lanelet_regs = []
        for member_type, ref, role in relation_members(relation):
            if member_type == "relation" and role == "regulatory_element" and ref in traffic_light_regs:
                lanelet_regs.append(ref)
                reg_to_lanelet[ref] = relation_id
        if lanelet_regs:
            regs_by_lanelet[relation_id] = lanelet_regs

    reg_rows = []
    obsolete_regs = set()

    for lanelet_id, reg_ids in sorted(regs_by_lanelet.items(), key=lambda item: int(item[0])):
        lanelet = relation_elements[lanelet_id]
        lanelet_member_map = relation_members(lanelet)
        left_way_id = next(ref for member_type, ref, role in lanelet_member_map if member_type == "way" and role == "left")
        right_way_id = next(ref for member_type, ref, role in lanelet_member_map if member_type == "way" and role == "right")
        center_way_id = next(
            (ref for member_type, ref, role in lanelet_member_map if member_type == "way" and role == "centerline"),
            None,
        )

        center_way = way_elements.get(center_way_id) if center_way_id else None
        left_way = way_elements[left_way_id]
        right_way = way_elements[right_way_id]
        center_line = line2_for_way(center_way, node_elements, transformer_to_utm) if center_way is not None else None

        old_stop_ids = []
        traffic_light_ids = []
        projection_distances = []
        for reg_id in reg_ids:
            reg = relation_elements[reg_id]
            for member_type, ref, role in relation_members(reg):
                if member_type == "way" and role == "ref_line":
                    old_stop_ids.append(ref)
                    if center_line is not None and center_line.length > 0 and ref in way_elements:
                        projection_distances.append(center_line.project(line2_for_way(way_elements[ref], node_elements, transformer_to_utm).centroid))
                elif member_type == "way" and role == "refers":
                    traffic_light_ids.append(ref)
                    if center_line is not None and center_line.length > 0 and ref in way_elements:
                        projection_distances.append(center_line.project(line2_for_way(way_elements[ref], node_elements, transformer_to_utm).centroid))

        traffic_light_ids = sorted(set(traffic_light_ids), key=int)
        old_stop_ids = sorted(set(old_stop_ids), key=int)

        if center_line is not None and center_line.length > 0 and projection_distances:
            raw_distance = median(projection_distances)
            margin = min(args.stopline_margin_m, center_line.length * 0.25)
            distance = min(max(raw_distance, margin), max(margin, center_line.length - margin))
            fraction = 0.5 if center_line.length <= 0 else distance / center_line.length
        else:
            fraction = 0.5

        left_point = interpolate_way_3d(left_way, fraction, node_elements, transformer_to_utm)
        right_point = interpolate_way_3d(right_way, fraction, node_elements, transformer_to_utm)
        dx = right_point[0] - left_point[0]
        dy = right_point[1] - left_point[1]
        dz = right_point[2] - left_point[2]
        width = (dx**2 + dy**2) ** 0.5
        if width > 0:
            inset = min(args.stopline_end_inset_m, width * 0.25)
            ux = dx / width
            uy = dy / width
            z_step = dz / width
            left_point = (
                left_point[0] + ux * inset,
                left_point[1] + uy * inset,
                left_point[2] + z_step * inset,
            )
            right_point = (
                right_point[0] - ux * inset,
                right_point[1] - uy * inset,
                right_point[2] - z_step * inset,
            )
        left_lon, left_lat = transformer_to_wgs.transform(left_point[0], left_point[1])
        right_lon, right_lat = transformer_to_wgs.transform(right_point[0], right_point[1])

        left_node_id = str(next_node_id)
        next_node_id += 1
        right_node_id = str(next_node_id)
        next_node_id += 1
        left_node = append_node(root, left_node_id, left_lat, left_lon, left_point[2])
        right_node = append_node(root, right_node_id, right_lat, right_lon, right_point[2])
        node_elements[left_node_id] = left_node
        node_elements[right_node_id] = right_node

        new_stop_way_id = str(next_way_id)
        next_way_id += 1
        new_stop_way = append_stop_line_way(root, new_stop_way_id, (left_node_id, right_node_id))
        way_elements[new_stop_way_id] = new_stop_way

        primary_reg_id = sorted(reg_ids, key=int)[0]
        primary_reg = relation_elements[primary_reg_id]
        new_members = [("way", new_stop_way_id, "ref_line")]
        new_members.extend(("way", traffic_light_id, "refers") for traffic_light_id in traffic_light_ids)
        replace_relation_members(primary_reg, new_members)
        for reg_id in reg_ids:
            if reg_id != primary_reg_id:
                obsolete_regs.add(reg_id)

        kept_lanelet_members = [
            member
            for member in lanelet_member_map
            if not (member[0] == "relation" and member[2] == "regulatory_element" and member[1] in traffic_light_regs)
        ]
        kept_lanelet_members.append(("relation", primary_reg_id, "regulatory_element"))
        replace_relation_members(lanelet, kept_lanelet_members)

        reg_rows.append(
            {
                "lanelet_id": lanelet_id,
                "primary_reg_id": primary_reg_id,
                "old_reg_ids": " ".join(sorted(reg_ids, key=int)),
                "old_reg_count": str(len(reg_ids)),
                "old_stop_line_ids": " ".join(old_stop_ids),
                "new_stop_line_id": new_stop_way_id,
                "new_stop_node_ids": f"{left_node_id} {right_node_id}",
                "traffic_light_way_ids": " ".join(traffic_light_ids),
                "traffic_light_count": str(len(traffic_light_ids)),
                "centerline_fraction": f"{fraction:.6f}",
            }
        )

    for reg_id in sorted(obsolete_regs, key=int):
        relation = relation_elements[reg_id]
        root.remove(relation)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(args.output), encoding="UTF-8", xml_declaration=True, pretty_print=True)

    args.reg_report.parent.mkdir(parents=True, exist_ok=True)
    with args.reg_report.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "lanelet_id",
                "primary_reg_id",
                "old_reg_ids",
                "old_reg_count",
                "old_stop_line_ids",
                "new_stop_line_id",
                "new_stop_node_ids",
                "traffic_light_way_ids",
                "traffic_light_count",
                "centerline_fraction",
            ],
        )
        writer.writeheader()
        writer.writerows(reg_rows)

    with args.z_report.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["way_id", "old_node_id", "new_node_id", "old_ele", "new_ele", "reason"])
        writer.writeheader()
        writer.writerows(z_rows)

    print(f"output={args.output}")
    print(f"map_version={args.map_version}")
    print(f"controlled_lanelets={len(reg_rows)}")
    print(f"traffic_light_regs_before={len(traffic_light_regs)}")
    print(f"traffic_light_regs_removed={len(obsolete_regs)}")
    print(f"traffic_light_regs_after={len(traffic_light_regs) - len(obsolete_regs)}")
    print(f"generated_stop_lines={len(reg_rows)}")
    print(f"reversed_traffic_light_ways={reversed_tl_count}")
    print(f"z_cloned_nodes={len(z_rows)}")
    print(f"reg_report={args.reg_report}")
    print(f"z_report={args.z_report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
