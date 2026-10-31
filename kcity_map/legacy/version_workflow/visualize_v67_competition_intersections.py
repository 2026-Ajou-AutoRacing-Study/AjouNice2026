#!/usr/bin/env python3
"""Render the seven v67 competition-route intersection speed zones as SVG."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from pyproj import Transformer

from build_v67_intersection_speed_limits import INTERSECTION_AREAS


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP = ROOT / "lanelet_map/lanelet2_map_v67.osm"
DEFAULT_ROUTE = (
    ROOT.parent
    / "base_ws/src/planning_addons/competition_route_requester/config/kcity_v67_competition_route.yaml"
)
DEFAULT_OUTPUT = ROOT / "docs/assets/kcity_v67_competition_intersections.svg"
COLORS = ["#e63946", "#f28c28", "#8a5cf6", "#159f8c", "#3a86ff", "#d33682", "#6a994e"]


def tags(element: ET.Element) -> dict[str, str]:
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    root = ET.parse(args.map).getroot()
    config = yaml.safe_load(args.route.read_text(encoding="utf-8"))
    nodes_raw = {
        item.attrib["id"]: (float(item.attrib["lon"]), float(item.attrib["lat"]))
        for item in root.findall("node")
    }
    ways_raw = {
        item.attrib["id"]: [node.attrib["ref"] for node in item.findall("nd")]
        for item in root.findall("way")
    }
    relations = {item.attrib["id"]: item for item in root.findall("relation")}
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    nodes = {node_id: transformer.transform(*position) for node_id, position in nodes_raw.items()}

    def way_points(way_id: str):
        return [nodes[node_id] for node_id in ways_raw[way_id]]

    centerlines = {}
    lanelet_polygons = {}
    area_members = {area_id: [] for area_id in INTERSECTION_AREAS}
    for relation_id, relation in relations.items():
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        members = {member.attrib.get("role"): member.attrib.get("ref") for member in relation.findall("member")}
        if "left" in members and "right" in members:
            lanelet_polygons[int(relation_id)] = (
                way_points(members["left"]) + list(reversed(way_points(members["right"])))
            )
        for member in relation.findall("member"):
            if member.attrib.get("role") == "centerline":
                centerlines[int(relation_id)] = way_points(member.attrib["ref"])
        area_id = relation_tags.get("intersection_area")
        if area_id in area_members:
            area_members[area_id].append(int(relation_id))

    route_ids = config["route_b"]["expected_path_lanelet_ids"]
    route_points = [point for lanelet_id in route_ids for point in centerlines.get(lanelet_id, [])]
    area_polygons = {area_id: way_points(area_id) for area_id in INTERSECTION_AREAS}
    focus_points = route_points + [point for polygon in area_polygons.values() for point in polygon]
    min_x = min(point[0] for point in focus_points) - 20.0
    max_x = max(point[0] for point in focus_points) + 20.0
    min_y = min(point[1] for point in focus_points) - 20.0
    max_y = max(point[1] for point in focus_points) + 20.0
    panel_x, panel_y, panel_w, panel_h = 35.0, 125.0, 990.0, 1030.0
    scale = min(panel_w / (max_x - min_x), panel_h / (max_y - min_y))
    used_w, used_h = (max_x - min_x) * scale, (max_y - min_y) * scale
    x0 = panel_x + (panel_w - used_w) / 2.0 - min_x * scale
    y0 = panel_y + (panel_h - used_h) / 2.0 + max_y * scale

    def screen(point):
        return x0 + point[0] * scale, y0 - point[1] * scale

    def path(points, close=False):
        rendered = "M " + " L ".join(f"{x:.1f},{y:.1f}" for x, y in map(screen, points))
        return rendered + (" Z" if close else "")

    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1200" viewBox="0 0 1600 1200" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v67 competition intersection 30 km/h zones</title>',
        '<desc id="desc">Seven physical intersection polygons on competition Routes A and B and every vehicle lanelet limited to 30 km/h inside them.</desc>',
        '<defs><style>',
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill:#172033; }",
        '.title{font-size:31px;font-weight:700}.subtitle{font-size:16px;fill:#526079}.panel{fill:#fff;stroke:#dce3ed;stroke-width:1.5}',
        '.zone{font-size:14px;font-weight:700}.ids{font-size:11px;fill:#526079}.mapid{font-size:13px;font-weight:700;paint-order:stroke;stroke:white;stroke-width:4px}',
        '</style></defs>',
        '<rect width="1600" height="1200" fill="#f4f7fb"/>',
        '<text x="40" y="50" class="title">K-City v67 대회 경로 교차로 30 km/h 구간</text>',
        '<text x="40" y="82" class="subtitle">실제 intersection_area 경계 · 파란 선은 Route B · 색칠된 영역 안 차량용 road lanelet 전체에 speed_limit=30</text>',
        '<rect x="25" y="105" width="1020" height="1070" rx="22" class="panel"/>',
        '<rect x="1070" y="105" width="505" height="1070" rx="22" class="panel"/>',
    ]
    for points in centerlines.values():
        if max(p[0] for p in points) < min_x or min(p[0] for p in points) > max_x:
            continue
        if max(p[1] for p in points) < min_y or min(p[1] for p in points) > max_y:
            continue
        svg.append(f'<path d="{path(points)}" fill="none" stroke="#d5dce6" stroke-width="1.2"/>')
    for lanelet_id in route_ids:
        if lanelet_id in centerlines:
            svg.append(f'<path d="{path(centerlines[lanelet_id])}" fill="none" stroke="#2563eb" stroke-width="4" stroke-linecap="round"/>')

    for index, (area_id, specification) in enumerate(INTERSECTION_AREAS.items(), start=1):
        color = COLORS[index - 1]
        polygon = area_polygons[area_id]
        svg.append(f'<path d="{path(polygon, True)}" fill="{color}" fill-opacity="0.30" stroke="{color}" stroke-width="3"/>')
        cx = sum(p[0] for p in polygon[:-1]) / (len(polygon) - 1)
        cy = sum(p[1] for p in polygon[:-1]) / (len(polygon) - 1)
        sx, sy = screen((cx, cy))
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="17" fill="white" stroke="{color}" stroke-width="4"/>')
        svg.append(f'<text x="{sx:.1f}" y="{sy+5:.1f}" text-anchor="middle" class="zone">{index}</text>')
        svg.append(f'<text x="{sx+23:.1f}" y="{sy-8:.1f}" class="mapid">area {area_id}</text>')
        svg.append(f'<text x="{sx+23:.1f}" y="{sy+11:.1f}" class="mapid">route {specification["route_lanelet"]}</text>')

    svg.append('<text x="1100" y="150" style="font-size:21px;font-weight:700">적용 범위</text>')
    y = 190
    for index, (area_id, specification) in enumerate(INTERSECTION_AREAS.items(), start=1):
        color = COLORS[index - 1]
        members = sorted(area_members[area_id])
        svg.append(f'<circle cx="1112" cy="{y-5}" r="10" fill="{color}"/><text x="1112" y="{y-1}" text-anchor="middle" style="font-size:11px;font-weight:700;fill:white">{index}</text>')
        svg.append(f'<text x="1132" y="{y}" class="zone">area {area_id} · route {specification["route_lanelet"]}</text>')
        rows = [members[offset:offset+6] for offset in range(0, len(members), 6)]
        for row_index, row in enumerate(rows):
            value = " · ".join(map(str, row))
            svg.append(f'<text x="1100" y="{y+24+row_index*18}" class="ids">{value}</text>')
        y += 58 + max(0, len(rows)-1) * 18
    svg.extend([
        '<line x1="1100" y1="970" x2="1545" y2="970" stroke="#dce3ed"/>',
        '<text x="1100" y="1005" class="zone">정의 원칙</text>',
        '<text x="1100" y="1033" class="ids">• 연석/차로 경계 안의 실제 차량 충돌 공간을 polygon으로 사용</text>',
        '<text x="1100" y="1055" class="ids">• polygon에 95% 이상 포함되는 vehicle road lanelet 전부 연결</text>',
        '<text x="1100" y="1077" class="ids">• pedestrian-only crosswalk에는 차량 speed_limit를 넣지 않음</text>',
        f'<text x="1100" y="1114" class="ids">Source: {args.map.name} · SHA-256 {digest(args.map)[:16]}…</text>',
        '<text x="1100" y="1136" class="ids">7 polygons · 60 vehicle lanelets · speed_limit=30 km/h</text>',
        '</svg>',
    ])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(svg) + "\n", encoding="utf-8")
    print(f"wrote {args.output}")

    # A second sheet provides a readable, local-scale view of every polygon.
    detail = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1400" viewBox="0 0 1600 1400" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v67 intersection polygon details</title>',
        '<desc id="desc">Seven local-scale intersection polygon views with all 30 km/h vehicle lanelets.</desc>',
        '<defs><style>',
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill:#172033; }",
        '.title{font-size:31px;font-weight:700}.subtitle{font-size:16px;fill:#526079}.panel{fill:#fff;stroke:#dce3ed;stroke-width:1.5}',
        '.zone{font-size:17px;font-weight:700}.ids{font-size:12px;fill:#526079}.lane{font-size:11px;font-weight:700;paint-order:stroke;stroke:white;stroke-width:3px}',
        '</style></defs>',
        '<rect width="1600" height="1400" fill="#f4f7fb"/>',
        '<text x="40" y="50" class="title">K-City v67 교차로 polygon 상세</text>',
        '<text x="40" y="82" class="subtitle">색 면은 실제 intersection_area · 굵은 선과 ID는 polygon 안에서 30 km/h가 적용되는 차량 lanelet</text>',
    ]
    for index, (area_id, specification) in enumerate(INTERSECTION_AREAS.items(), start=1):
        col = (index - 1) % 2
        row = (index - 1) // 2
        px, py = 25 + col * 787, 110 + row * 312
        pw, ph = 762, 288
        color = COLORS[index - 1]
        polygon = area_polygons[area_id]
        member_ids = sorted(area_members[area_id])
        local_points = polygon + [point for lanelet_id in member_ids for point in lanelet_polygons[lanelet_id]]
        local_min_x = min(point[0] for point in local_points) - 7.0
        local_max_x = max(point[0] for point in local_points) + 7.0
        local_min_y = min(point[1] for point in local_points) - 7.0
        local_max_y = max(point[1] for point in local_points) + 7.0
        map_x, map_y, map_w, map_h = px + 15, py + 48, 455, ph - 65
        local_scale = min(map_w / (local_max_x-local_min_x), map_h / (local_max_y-local_min_y))
        local_used_w = (local_max_x-local_min_x)*local_scale
        local_used_h = (local_max_y-local_min_y)*local_scale
        local_x0 = map_x + (map_w-local_used_w)/2 - local_min_x*local_scale
        local_y0 = map_y + (map_h-local_used_h)/2 + local_max_y*local_scale

        def local_screen(point):
            return local_x0 + point[0]*local_scale, local_y0 - point[1]*local_scale

        def local_path(points, close=False):
            value = "M " + " L ".join(f"{x:.1f},{y:.1f}" for x, y in map(local_screen, points))
            return value + (" Z" if close else "")

        detail.append(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" rx="18" class="panel"/>')
        detail.append(f'<circle cx="{px+25}" cy="{py+27}" r="11" fill="{color}"/><text x="{px+25}" y="{py+32}" text-anchor="middle" style="font-size:12px;font-weight:700;fill:white">{index}</text>')
        detail.append(f'<text x="{px+46}" y="{py+33}" class="zone">area {area_id} · Route lanelet {specification["route_lanelet"]}</text>')
        for lanelet_id, points in centerlines.items():
            if max(p[0] for p in points) < local_min_x or min(p[0] for p in points) > local_max_x:
                continue
            if max(p[1] for p in points) < local_min_y or min(p[1] for p in points) > local_max_y:
                continue
            detail.append(f'<path d="{local_path(points)}" fill="none" stroke="#d5dce6" stroke-width="1.2"/>')
        detail.append(f'<path d="{local_path(polygon, True)}" fill="{color}" fill-opacity="0.28" stroke="{color}" stroke-width="3"/>')
        for lanelet_id in member_ids:
            points = centerlines[lanelet_id]
            detail.append(f'<path d="{local_path(points)}" fill="none" stroke="{color}" stroke-width="4" stroke-linecap="round"/>')
            mid = points[len(points)//2]
            lx, ly = local_screen(mid)
            detail.append(f'<text x="{lx+3:.1f}" y="{ly-3:.1f}" class="lane">{lanelet_id}</text>')
        detail.append(f'<text x="{px+495}" y="{py+75}" class="zone">30 km/h ({len(member_ids)}개)</text>')
        for line_index, offset in enumerate(range(0, len(member_ids), 4)):
            value = " · ".join(map(str, member_ids[offset:offset+4]))
            detail.append(f'<text x="{px+495}" y="{py+105+line_index*21}" class="ids">{value}</text>')
    detail.extend([
        '<text x="40" y="1370" class="ids">회전교차로는 별도 Roundabout regulatory element 구간이며 이 7개 일반 교차로 속도 영역에 포함하지 않음</text>',
        '</svg>',
    ])
    detail_output = args.output.with_name(args.output.stem + "_detail.svg")
    detail_output.write_text("\n".join(detail) + "\n", encoding="utf-8")
    print(f"wrote {detail_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
