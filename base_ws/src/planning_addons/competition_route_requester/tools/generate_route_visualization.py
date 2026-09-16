#!/usr/bin/env python3
"""Generate the K-City v74 competition route documentation SVG set.

The overview, Route A detail, Route B detail, and reroute-process figures are
derived from the package YAML files and Lanelet2 centerlines.  This is a
documentation-only tool; it does not import ROS or change a route at runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
from pyproj import Transformer


SVG_WIDTH = 1400
SVG_HEIGHT = 1200


def parse_args() -> argparse.Namespace:
    package_dir = Path(__file__).resolve().parents[1]
    workspace_dir = Path(__file__).resolve().parents[5]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=package_dir / "config/kcity_v74_competition_route.yaml",
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=workspace_dir,
        help="Repository root used to resolve map.file from the route YAML.",
    )
    parser.add_argument(
        "--param-config",
        type=Path,
        default=package_dir / "config/competition_route_requester.param.yaml",
        help="Policy draft used only for the reroute-process figure.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=package_dir / "docs/assets/kcity_v74_route_a_b.svg",
        help="Overview SVG path; the three detail SVGs are written beside it.",
    )
    return parser.parse_args()


def tags(element: ET.Element) -> dict[str, str]:
    return {tag.attrib["k"]: tag.attrib["v"] for tag in element.findall("tag")}


def load_osm(map_path: Path, origin_lat: float, origin_lon: float):
    root = ET.parse(str(map_path)).getroot()
    raw_nodes = {
        int(node.attrib["id"]): (float(node.attrib["lat"]), float(node.attrib["lon"]))
        for node in root.findall("node")
    }
    ways = {
        int(way.attrib["id"]): [int(nd.attrib["ref"]) for nd in way.findall("nd")]
        for way in root.findall("way")
    }

    zone = int((origin_lon + 180.0) // 6.0) + 1
    epsg = (32600 if origin_lat >= 0.0 else 32700) + zone
    transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
    origin_e, origin_n = transformer.transform(origin_lon, origin_lat)

    nodes = {}
    for node_id, (lat, lon) in raw_nodes.items():
        easting, northing = transformer.transform(lon, lat)
        nodes[node_id] = (easting - origin_e, northing - origin_n)

    lanelets = {}
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet":
            continue
        members = {
            member.attrib.get("role", ""): int(member.attrib["ref"])
            for member in relation.findall("member")
            if member.attrib.get("type") == "way"
        }
        if "centerline" not in members:
            continue
        lanelet_id = int(relation.attrib["id"])
        lanelets[lanelet_id] = {
            "centerline": [nodes[node_id] for node_id in ways[members["centerline"]]],
            "left": [nodes[node_id] for node_id in ways[members["left"]]]
            if "left" in members
            else [],
            "right": [nodes[node_id] for node_id in ways[members["right"]]]
            if "right" in members
            else [],
            "tags": relation_tags,
        }
    return lanelets, epsg


def points_at_distance(points, distance_m: float):
    remaining = distance_m
    for point_a, point_b in zip(points, points[1:]):
        segment_length = math.dist(point_a, point_b)
        if remaining <= segment_length:
            ratio = remaining / segment_length
            return (
                point_a[0] + ratio * (point_b[0] - point_a[0]),
                point_a[1] + ratio * (point_b[1] - point_a[1]),
            )
        remaining -= segment_length
    return points[-1]


def polyline_length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def joined_centerlines(lanelets, lanelet_ids, lateral_tolerance=0.10):
    points = []
    for lanelet_id in lanelet_ids:
        centerline = lanelets[lanelet_id]["centerline"]
        if points and math.dist(points[-1], centerline[0]) > lateral_tolerance:
            raise ValueError(
                f"Disconnected route: {lanelet_ids[lanelet_ids.index(lanelet_id)-1]}"
                f" -> {lanelet_id}"
            )
        points.extend(centerline if not points else centerline[1:])
    return points


def build_highway_lane_strategy_svg(config, lanelets, map_path, actual_hash, epsg):
    """Show the v74 highway lane groups without treating lane changes as serial links."""
    current = [201456, 201438, 200128, 200102, 200134, 201446, 200360]
    selected = [201457, 201435, 200137, 200187, 200130, 201441, 200360]
    upper = [201459, 201436, 200138, 201444, 200127, 201439, 200360]
    highway_ids = list(dict.fromkeys(current + selected + upper + [201458, 200099]))
    missing = [lanelet_id for lanelet_id in highway_ids if lanelet_id not in lanelets]
    if missing:
        raise ValueError(f"Highway visualization lanelets missing from map: {missing}")
    source_points = [point for lanelet_id in highway_ids for point in lanelets[lanelet_id]["centerline"]]
    center_x = sum(point[0] for point in source_points) / len(source_points)
    display_lines = {
        lanelet_id: [(center_x + (x - center_x) * 20.0, y)
                     for x, y in lanelets[lanelet_id]['centerline']]
        for lanelet_id in highway_ids
    }
    points = [point for lanelet_id in highway_ids for point in display_lines[lanelet_id]]
    view = Viewport(bounds_for(points, 10.0), (45, 125, 830, 720), padding=18)
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="900" viewBox="0 0 1400 900" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v74 highway preferred-lane strategy</title>',
        '<desc id="desc">Former right lane, selected middle lane, and upper lane alternatives using actual v74 lanelet centerlines.</desc>',
        f'<!-- source_map={map_path.name} sha256={actual_hash} projection=EPSG:{epsg} -->',
        '<defs><style>',
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        '.title { font-size: 32px; font-weight: 700; }.subtitle { font-size: 16px; fill: #526079; }',
        '.panel { fill: #fff; stroke: #dce3ed; stroke-width: 1.5; }.label { font-size: 14px; font-weight: 700; }',
        '.small { font-size: 13px; fill: #526079; }.lane-id { font-size: 12px; font-weight: 700; paint-order: stroke; stroke: white; stroke-width: 4px; }',
        '</style><marker id="lc-arrow" markerWidth="9" markerHeight="9" refX="8" refY="4.5" orient="auto"><path d="M0,0 L9,4.5 L0,9 Z" fill="#d33682"/></marker></defs>',
        '<rect width="1400" height="900" fill="#f4f7fb"/>',
        '<text x="42" y="52" class="title">K-City v74 고주로 선호 차선 검토</text>',
        '<text x="42" y="82" class="subtitle">실제 Lanelet2 중심선 · 진행 방향은 위에서 아래 · 차선 간격은 가독성을 위해 가로 20배 확대</text>',
        '<rect x="25" y="105" width="875" height="760" rx="22" class="panel"/>',
        '<rect x="925" y="105" width="450" height="760" rx="22" class="panel"/>',
        '<text x="955" y="150" style="font-size:21px;font-weight:700">판단 결과</text>',
    ]
    for lanelet_id in highway_ids:
        svg.append(f'<path d="{view.path(display_lines[lanelet_id])}" fill="none" stroke="#d7dde5" stroke-width="2"/>')
    for route, color, width, dash in ((current, '#7a879b', 5, '8 7'),
                                      (upper, '#e59a23', 5, '4 7'),
                                      (selected, '#2563eb', 9, None)):
        for lanelet_id in route:
            line = display_lines[lanelet_id]
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ''
            svg.append(f'<path d="{view.path(line)}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linecap="round"{dash_attr}/>')
    for lanelet_id in highway_ids:
        line = display_lines[lanelet_id]
        point = points_at_distance(line, polyline_length(line) * 0.5)
        x, y = view.point(point)
        color = '#174ea6' if lanelet_id in selected else '#56657a'
        svg.append(f'<text x="{x+5:.1f}" y="{y-4:.1f}" class="lane-id" fill="{color}">{lanelet_id}</text>')
    a = points_at_distance(display_lines[201456], 16.0)
    b = points_at_distance(display_lines[201457], 16.0)
    ax, ay = view.point(a); bx, by = view.point(b)
    svg.extend([
        f'<path d="M {ax:.1f},{ay:.1f} L {bx:.1f},{by:.1f}" stroke="#d33682" stroke-width="4" marker-end="url(#lc-arrow)"/>',
        f'<text x="{(ax+bx)/2+8:.1f}" y="{(ay+by)/2-10:.1f}" class="label" fill="#b42367">진입 직후 LC</text>',
    ])
    y = 205
    for heading, color, route in (('기존 우측', '#7a879b', current),
                                  ('적용 중간', '#2563eb', selected),
                                  ('최상위 비교', '#e59a23', upper)):
        svg.append(f'<path d="M 955 {y-6} L 1010 {y-6}" stroke="{color}" stroke-width="7" stroke-linecap="round"/>')
        svg.append(f'<text x="1025" y="{y}" class="label">{heading}</text>')
        for start in range(0, len(route), 4):
            value = ' → '.join(str(item) for item in route[start:start+4])
            svg.append(f'<text x="955" y="{y+32+(start//4)*24}" class="small">{value}</text>')
        y += 118
    add_multiline(svg, 955, 585, [
        '• 200137은 200128과 나란한 상위 차선이며 누락 구간이 아니다.',
        '• 병합된 200187은 중간 차선이다. 같은 위치의 최상위는 201444이다.',
        '• 진입 직후 평행 후보 201456~201459 중에서',
        '  SetPreferredPrimitive로 중간 차선 201457을 선택한다.',
        '• 이후 201435→200137→200187 중간 차선을 유지한다.',
    ], 'small', 29)
    svg.append(f'<text x="955" y="812" class="small">v{config["map"]["version"]} · SHA-256 {actual_hash[:16]}…</text>')
    svg.append('</svg>')
    return '\n'.join(svg) + '\n'


class Viewport:
    def __init__(self, data_bounds, panel_bounds, padding=18.0):
        min_x, max_x, min_y, max_y = data_bounds
        px, py, pw, ph = panel_bounds
        scale = min((pw - 2 * padding) / (max_x - min_x), (ph - 2 * padding) / (max_y - min_y))
        used_w = (max_x - min_x) * scale
        used_h = (max_y - min_y) * scale
        self.scale = scale
        self.x0 = px + (pw - used_w) / 2.0 - min_x * scale
        self.y0 = py + (ph - used_h) / 2.0 + max_y * scale

    def point(self, point):
        return self.x0 + point[0] * self.scale, self.y0 - point[1] * self.scale

    def path(self, points):
        screen = [self.point(point) for point in points]
        return "M " + " L ".join(f"{x:.1f},{y:.1f}" for x, y in screen)


def bounds_for(points, margin):
    return (
        min(point[0] for point in points) - margin,
        max(point[0] for point in points) + margin,
        min(point[1] for point in points) - margin,
        max(point[1] for point in points) + margin,
    )


def intersects(points, bounds):
    min_x, max_x, min_y, max_y = bounds
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return not (max(xs) < min_x or min(xs) > max_x or max(ys) < min_y or min(ys) > max_y)


def marker(svg, viewport, point, color, number, radius=10):
    x, y = viewport.point(point)
    svg.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius}" fill="white" stroke="{color}" stroke-width="4"/>')
    svg.append(
        f'<text x="{x:.1f}" y="{y + 5:.1f}" class="marker-number" text-anchor="middle">{number}</text>'
    )


def arrow(svg, viewport, points, fraction, color):
    target = polyline_length(points) * fraction
    remaining = target
    for point_a, point_b in zip(points, points[1:]):
        length = math.dist(point_a, point_b)
        if remaining <= length:
            ratio = remaining / length
            point = (
                point_a[0] + ratio * (point_b[0] - point_a[0]),
                point_a[1] + ratio * (point_b[1] - point_a[1]),
            )
            angle = math.atan2(point_b[1] - point_a[1], point_b[0] - point_a[0])
            x, y = viewport.point(point)
            screen_angle = -math.degrees(angle)
            svg.append(
                f'<path d="M -9,-7 L 9,0 L -9,7 Z" fill="{color}" stroke="white" stroke-width="1.5" '
                f'transform="translate({x:.1f} {y:.1f}) rotate({screen_angle:.1f})"/>'
            )
            return
        remaining -= length


def add_multiline(svg, x, y, lines, css_class, line_height=24, anchor="start"):
    svg.append(f'<text x="{x}" y="{y}" class="{css_class}" text-anchor="{anchor}">')
    for index, line in enumerate(lines):
        dy = 0 if index == 0 else line_height
        svg.append(f'<tspan x="{x}" dy="{dy}">{line}</tspan>')
    svg.append("</text>")


def detail_svg_start(width, height, title, subtitle, description, source_comment):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        f'<title id="title">{title}</title>',
        f'<desc id="desc">{description}</desc>',
        f'<!-- {source_comment} -->',
        "<defs>",
        '<clipPath id="map-clip"><rect x="50" y="155" width="455" height="1170" rx="18"/></clipPath>',
        "<style>",
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        ".title { font-size: 34px; font-weight: 700; }",
        ".subtitle { font-size: 16px; fill: #526079; }",
        ".panel-title { font-size: 21px; font-weight: 700; }",
        ".label { font-size: 15px; font-weight: 650; }",
        ".small { font-size: 13px; fill: #526079; }",
        ".tiny { font-size: 11px; fill: #67748a; }",
        ".mono { font-family: 'DejaVu Sans Mono', monospace; font-size: 13px; }",
        ".marker-number { font-size: 11px; font-weight: 800; }",
        ".card { fill: #ffffff; stroke: #dce3ed; stroke-width: 1.5; }",
        ".road { fill: none; stroke: #d7dde5; stroke-width: 1.1; stroke-linecap: round; }",
        "</style>",
        "</defs>",
        f'<rect width="{width}" height="{height}" fill="#f4f7fb"/>',
        f'<text x="42" y="54" class="title">{title}</text>',
        f'<text x="42" y="84" class="subtitle">{subtitle}</text>',
    ]


def add_map_background(svg, lanelets, viewport, data_bounds, clip_id="map-clip"):
    svg.append(f'<g clip-path="url(#{clip_id})">')
    for lanelet in lanelets.values():
        centerline = lanelet["centerline"]
        if lanelet["tags"].get("subtype") == "road" and intersects(centerline, data_bounds):
            svg.append(f'<path d="{viewport.path(centerline)}" class="road"/>')
    svg.append("</g>")


def add_waypoint_table(svg, config, route_key, y_start):
    route = config[route_key]
    poses = config["poses"]
    svg.append('<text x="575" y="145" class="panel-title">① 실제 SetWaypointRoute 요청 구성</text>')
    if route_key == "route_a":
        svg.append('<rect x="575" y="165" width="960" height="60" rx="12" fill="#eaf1ff" stroke="#b7cdfd"/>')
        svg.append('<text x="595" y="190" class="label">START = 현재 odometry · 공식 200892→200889 경계 (허용 반경 6 m)</text>')
        svg.append('<text x="595" y="213" class="small">start_pose를 요청에 넣지 않는다. 지도 S는 공식 START/END 좌표이며 두 lanelet 판정을 허용한다.</text>')
    else:
        svg.append('<rect x="575" y="165" width="960" height="60" rx="12" fill="#e6f7f4" stroke="#8ad7cd"/>')
        svg.append('<text x="595" y="190" class="label">START = 요청 시점의 현재 pose · lanelet 200178에서 s ≥ 8 m</text>')
        svg.append('<text x="595" y="213" class="small">wp_200178은 현재 start이므로 waypoint_refs에서 의도적으로 제외한다.</text>')

    svg.append('<text x="590" y="258" class="small">순서</text>')
    svg.append('<text x="670" y="258" class="small">waypoint_ref</text>')
    svg.append('<text x="930" y="258" class="small">lanelet_id</text>')
    svg.append('<text x="1090" y="258" class="small">lanelet s</text>')
    svg.append('<text x="1235" y="258" class="small">요청 역할</text>')
    svg.append('<path d="M 580 270 L 1530 270" stroke="#dce3ed"/>')
    for index, pose_ref in enumerate(route["waypoint_refs"], start=1):
        pose = poses[pose_ref]
        y = y_start + (index - 1) * 30
        svg.append(f'<circle cx="610" cy="{y-5}" r="11" fill="white" stroke="#d99100" stroke-width="3"/>')
        svg.append(f'<text x="610" y="{y-1}" class="marker-number" text-anchor="middle">{index}</text>')
        svg.append(f'<text x="670" y="{y}" class="mono">{pose_ref}</text>')
        svg.append(f'<text x="930" y="{y}" class="mono">{pose["lanelet_id"]}</text>')
        svg.append(f'<text x="1090" y="{y}" class="mono">{pose["lanelet_s_m"]:.1f} m</text>')
        svg.append('<text x="1235" y="{}" class="small">sparse 경유 pose</text>'.format(y))

    goal_ref = route["goal_ref"]
    goal = poses[goal_ref]
    goal_y = y_start + len(route["waypoint_refs"]) * 30 + 10
    svg.append(f'<path d="M 580 {goal_y-23} L 1530 {goal_y-23}" stroke="#dce3ed"/>')
    svg.append(f'<circle cx="610" cy="{goal_y-5}" r="12" fill="white" stroke="#6d28d9" stroke-width="3"/>')
    svg.append(f'<text x="610" y="{goal_y-1}" class="marker-number" text-anchor="middle">G</text>')
    svg.append(f'<text x="670" y="{goal_y}" class="mono">{goal_ref}</text>')
    svg.append(f'<text x="930" y="{goal_y}" class="mono">{goal["lanelet_id"]}</text>')
    position = f'{goal["lanelet_s_m"]:.1f} m' if "lanelet_s_m" in goal else f'{goal["lanelet_ratio"]:.2f} ratio'
    svg.append(f'<text x="1090" y="{goal_y}" class="mono">{position}</text>')
    goal_role = "Route A 임시 정지 goal" if route_key == "route_a" else "Route B 최종 goal"
    svg.append(f'<text x="1235" y="{goal_y}" class="small">{goal_role}</text>')


def add_lanelet_verification_grid(svg, route_ids, waypoint_ids, start_id, goal_id):
    svg.append('<text x="575" y="710" class="panel-title">② expected_path_lanelet_ids · 수신 경로 검증 순서</text>')
    svg.append('<text x="575" y="738" class="small">아래 ID는 waypoint 요청 목록이 아니다. Mission Planner 결과가 이 전체 순서와 일치하는지 검증한다.</text>')
    columns = 4
    rows = math.ceil(len(route_ids) / columns)
    column_width = 238
    for index, lanelet_id in enumerate(route_ids, start=1):
        column = (index - 1) // rows
        row = (index - 1) % rows
        x = 575 + column * column_width
        y = 775 + row * 27
        fill = "#ffffff"
        stroke = "#dce3ed"
        if lanelet_id in waypoint_ids:
            fill, stroke = "#fff6dc", "#d99100"
        if lanelet_id == start_id:
            fill, stroke = "#eaf1ff", "#2563eb"
        if lanelet_id == goal_id:
            fill, stroke = "#f2eafe", "#6d28d9"
        svg.append(f'<rect x="{x}" y="{y-18}" width="215" height="23" rx="6" fill="{fill}" stroke="{stroke}"/>')
        svg.append(f'<text x="{x+9}" y="{y-2}" class="tiny">{index:02d}</text>')
        svg.append(f'<text x="{x+42}" y="{y-2}" class="mono">{lanelet_id}</text>')
    legend_y = 1187
    legends = [
        ("#eaf1ff", "#2563eb", "canonical outgoing start"),
        ("#fff6dc", "#d99100", "configured waypoint lanelet"),
        ("#f2eafe", "#6d28d9", "goal lanelet"),
        ("#ffffff", "#dce3ed", "검증 경로 중간 lanelet"),
    ]
    for index, (fill, stroke, text_value) in enumerate(legends):
        x = 575 + index * 238
        svg.append(f'<rect x="{x}" y="{legend_y-14}" width="20" height="14" rx="3" fill="{fill}" stroke="{stroke}"/>')
        svg.append(f'<text x="{x+28}" y="{legend_y-2}" class="tiny">{text_value}</text>')


def build_route_detail_svg(config, lanelets, map_path, actual_hash, epsg, origin, route_key):
    route = config[route_key]
    route_ids = route["expected_path_lanelet_ids"]
    route_points = joined_centerlines(lanelets, route_ids, lateral_tolerance=5.0)
    route_name = "Route A" if route_key == "route_a" else "Route B 재경로"
    subtitle = (
        "공식 START/END 경계 → 11개 sparse waypoint → 비순환 임시 goal 200181"
        if route_key == "route_a"
        else "A→B 전환 시 200178의 현재 pose → 10개 sparse waypoint → 최종 goal 200210"
    )
    svg = detail_svg_start(
        1600,
        1400,
        f"K-City v74 · {route_name} 상세",
        f"{subtitle} · 요청 데이터와 검증용 lanelet 순서를 분리 표시",
        f"Detailed {route_name} request waypoints and expected verification lanelets.",
        f"source_map={map_path.name} sha256={actual_hash} projection=EPSG:{epsg} route={route_key}",
    )
    svg.extend(
        [
            '<rect x="30" y="110" width="500" height="1260" rx="22" class="card"/>',
            f'<text x="50" y="142" class="panel-title">{route_name} 지도 · 선 = expected path</text>',
            '<rect x="50" y="155" width="455" height="1170" rx="18" fill="#fbfcfe"/>',
            '<rect x="550" y="110" width="1020" height="550" rx="22" class="card"/>',
            '<rect x="550" y="680" width="1020" height="530" rx="22" class="card"/>',
            '<rect x="550" y="1230" width="1020" height="140" rx="22" class="card"/>',
        ]
    )
    route_bounds = bounds_for(route_points, 24.0)
    viewport = Viewport(route_bounds, (50, 155, 455, 1170), padding=14)
    add_map_background(svg, lanelets, viewport, route_bounds)
    svg.append('<g clip-path="url(#map-clip)">')
    if route_key == "route_a":
        svg.append(f'<path d="{viewport.path(route_points)}" fill="none" stroke="#2563eb" stroke-width="11" stroke-linecap="round" stroke-linejoin="round"/>')
    else:
        trigger_id = config["poses"][config["reroute_trigger"]["pose_ref"]]["lanelet_id"]
        a_ids = config["route_a"]["expected_path_lanelet_ids"]
        shared = a_ids[a_ids.index(trigger_id) :]
        shared_points = joined_centerlines(lanelets, shared, lateral_tolerance=5.0)
        continuation_points = joined_centerlines(lanelets, route_ids[len(shared) :], lateral_tolerance=5.0)
        svg.append(f'<path d="{viewport.path(shared_points)}" fill="none" stroke="#0f9f8f" stroke-width="11" stroke-linecap="round" stroke-linejoin="round"/>')
        svg.append(f'<path d="{viewport.path(continuation_points)}" fill="none" stroke="#f28c28" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append("</g>")
    for fraction in (0.18, 0.47, 0.76):
        arrow(svg, viewport, route_points, fraction, "#174ea6" if route_key == "route_a" else "#0b776c")

    if route_key == "route_a":
        start_pose = config["start"]["map_pose"]
        start_point = (float(start_pose["x"]), float(start_pose["y"]))
        start_id = config["start"]["outgoing_lanelet_id"]
    else:
        start_pose = config["poses"][config["reroute_trigger"]["pose_ref"]]
        start_point = (float(start_pose["x"]), float(start_pose["y"]))
        start_id = start_pose["lanelet_id"]
    marker(svg, viewport, start_point, "#2563eb", "S", radius=12)
    for index, pose_ref in enumerate(route["waypoint_refs"], start=1):
        pose = config["poses"][pose_ref]
        marker(svg, viewport, (float(pose["x"]), float(pose["y"])), "#d99100", str(index), radius=11)
    goal = config["poses"][route["goal_ref"]]
    marker(svg, viewport, (float(goal["x"]), float(goal["y"])), "#6d28d9", "G", radius=12)
    svg.append('<text x="70" y="190" class="small">N</text><path d="M 78 226 L 78 197 M 78 197 L 72 208 M 78 197 L 84 208" stroke="#172033" stroke-width="2" fill="none"/>')
    svg.append('<rect x="65" y="1272" width="16" height="16" rx="8" fill="white" stroke="#d99100" stroke-width="3"/>')
    svg.append('<text x="90" y="1285" class="tiny">숫자 표식 = waypoint_refs 순번 · S = 현재 pose · G = goal pose</text>')

    add_waypoint_table(svg, config, route_key, 295)
    waypoint_ids = {config["poses"][ref]["lanelet_id"] for ref in route["waypoint_refs"]}
    add_lanelet_verification_grid(svg, route_ids, waypoint_ids, start_id, goal["lanelet_id"])
    if route_key == "route_a":
        lines = [
            "Route A 요청: start는 공식 경계의 최신 odometry이며 200892/200889 판정을 모두 허용한다.",
            f"11개 waypoint와 goal_route_a_200181만 요청 pose다. {len(route_ids)}개 canonical ID는 결과 검증 기준이다.",
        ]
    else:
        lines = [
            "Route B 재경로 요청: 차량은 이미 200178 위에 있으므로 wp_200178을 waypoint_refs에 넣지 않는다.",
            f"route_b_only도 200178 s≥8m 시작 조건을 만족해야 하며, 200210까지 {len(route_ids)}개 ID를 확인한다.",
        ]
    add_multiline(svg, 575, 1267, lines, "label", 28)
    svg.append(f'<text x="575" y="1345" class="tiny">Map {map_path.name} · SHA-256 {actual_hash[:16]}… · LocalCartesianUTM EPSG:{epsg} · origin ({origin["latitude"]}, {origin["longitude"]})</text>')
    svg.append("</svg>")
    return "\n".join(svg) + "\n"


def build_reroute_process_svg(config, policy, lanelets, map_path, actual_hash, epsg):
    trigger = config["poses"][config["reroute_trigger"]["pose_ref"]]
    deadline = config["poses"][config["reroute_trigger"]["deadline_pose_ref"]]
    trigger_point = (float(trigger["x"]), float(trigger["y"]))
    deadline_anchor = (float(deadline["x"]), float(deadline["y"]))
    inset_ids = [200178, 200337, 200287, 200348, 200346]
    inset_route = joined_centerlines(lanelets, inset_ids)
    inset_bounds = bounds_for(inset_route, 20.0)
    viewport = Viewport(inset_bounds, (50, 170, 500, 820), padding=20)
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1100" viewBox="0 0 1600 1100" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v74 reroute trigger and process</title>',
        '<desc id="desc">Reroute trigger, freshness and hold checks, request and verification while Route A remains active, and deadline before lanelet 200346.</desc>',
        f'<!-- source_map={map_path.name} sha256={actual_hash} projection=EPSG:{epsg} -->',
        "<defs>",
        '<clipPath id="reroute-map-clip"><rect x="50" y="170" width="500" height="820" rx="18"/></clipPath>',
        "<style>",
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        ".title { font-size: 34px; font-weight: 700; } .subtitle { font-size: 16px; fill: #526079; }",
        ".panel-title { font-size: 21px; font-weight: 700; } .label { font-size: 15px; font-weight: 650; }",
        ".small { font-size: 13px; fill: #526079; } .tiny { font-size: 11px; fill: #67748a; }",
        ".marker-number { font-size: 12px; font-weight: 800; } .card { fill: #fff; stroke: #dce3ed; stroke-width: 1.5; }",
        ".road { fill: none; stroke: #d7dde5; stroke-width: 1.2; stroke-linecap: round; }",
        "</style></defs>",
        '<rect width="1600" height="1100" fill="#f4f7fb"/>',
        '<text x="42" y="54" class="title">K-City v74 · 정상 Route A → Route B 재경로 절차</text>',
        '<text x="42" y="84" class="subtitle">route_a_to_b 계열의 trigger·deadline + Route A를 유지한 채 수행하는 요청·검증 순서</text>',
        '<rect x="30" y="110" width="540" height="930" rx="22" class="card"/>',
        '<text x="50" y="145" class="panel-title">공간 경계</text>',
        '<rect x="50" y="170" width="500" height="820" rx="18" fill="#fbfcfe"/>',
        '<rect x="590" y="110" width="980" height="695" rx="22" class="card"/>',
        '<text x="615" y="145" class="panel-title">시간 순서 · Route A는 성공 확인 전까지 계속 활성</text>',
        '<rect x="590" y="825" width="980" height="215" rx="22" class="card"/>',
        '<text x="615" y="862" class="panel-title">200346 deadline과 YAML anchor는 다르다</text>',
    ]
    svg.append('<g clip-path="url(#reroute-map-clip)">')
    for lanelet in lanelets.values():
        centerline = lanelet["centerline"]
        if lanelet["tags"].get("subtype") == "road" and intersects(centerline, inset_bounds):
            svg.append(f'<path d="{viewport.path(centerline)}" class="road"/>')
    svg.append(f'<path d="{viewport.path(inset_route)}" fill="none" stroke="#0f9f8f" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append("</g>")
    trigger_line = lanelets[trigger["lanelet_id"]]["centerline"]
    nearest = min(range(len(trigger_line)), key=lambda i: math.dist(trigger_line[i], trigger_point))
    request_window = [trigger_point] + trigger_line[nearest + 1 :]
    for lanelet_id in inset_ids[1:-1]:
        line = lanelets[lanelet_id]["centerline"]
        request_window.extend(line[1:] if math.dist(request_window[-1], line[0]) < 0.1 else line)
    deadline_line = lanelets[deadline["lanelet_id"]]["centerline"]
    deadline_entry = deadline_line[0]
    request_window.append(deadline_entry)
    svg.append(f'<path d="{viewport.path(request_window)}" fill="none" stroke="#ffd7d2" stroke-width="25" stroke-linecap="round" stroke-linejoin="round" opacity="0.85" clip-path="url(#reroute-map-clip)"/>')
    svg.append(f'<path d="{viewport.path(inset_route)}" fill="none" stroke="#0f9f8f" stroke-width="9" stroke-linecap="round" stroke-linejoin="round" clip-path="url(#reroute-map-clip)"/>')
    direction = (deadline_line[1][0] - deadline_line[0][0], deadline_line[1][1] - deadline_line[0][1])
    norm = math.hypot(*direction)
    normal = (-direction[1] / norm, direction[0] / norm)
    bar = [
        (deadline_entry[0] + normal[0] * 9, deadline_entry[1] + normal[1] * 9),
        (deadline_entry[0] - normal[0] * 9, deadline_entry[1] - normal[1] * 9),
    ]
    svg.append(f'<path d="{viewport.path(bar)}" stroke="#b42318" stroke-width="6" stroke-linecap="round"/>')
    marker(svg, viewport, trigger_point, "#0f9f8f", "T", radius=13)
    marker(svg, viewport, deadline_anchor, "#b42318", "A", radius=13)
    ex, ey = viewport.point(deadline_entry)
    svg.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="6" fill="#b42318"/>')
    tx, ty = viewport.point(trigger_point)
    add_multiline(svg, tx + 28, ty + 4, ["TRIGGER T", "200178 · s ≥ 8 m"], "label", 22)
    ax, ay = viewport.point(deadline_anchor)
    add_multiline(svg, ax + 30, ay, ["ANCHOR A", "wp_200346 · s = 8 m"], "small", 21)
    add_multiline(svg, ex - 18, ey - 32, ["DEADLINE", "200346 진입 전"], "label", 22, anchor="end")
    svg.append('<rect x="75" y="946" width="18" height="11" rx="5" fill="#ffd7d2"/>')
    svg.append('<text x="104" y="957" class="small">Route B 요청·검증 가능 창</text>')

    step_data = [
        ("1", "#2563eb", "Route A ACTIVE", "기존 route를 지우지 않는다 · never_clear_route=true"),
        ("2", "#0f9f8f", "Trigger 통과", "lanelet 200178에 진입하고 s ≥ 8 m"),
        ("3", "#8b5cf6", "Guard 통과", f'odometry ≤ {policy["odometry_max_age"]:.1f}s · availability sample ≤ {policy["reroute_availability_max_age"]:.1f}s · true hold ≥ {policy["reroute_available_hold_time"]:.1f}s'),
        ("4", "#f59e0b", "Route B 요청", "현재 pose를 start로 사용 · wp_200178은 waypoint 목록에서 제외"),
        ("5", "#6d28d9", "Route B 검증", "UUID + goal 200210 + ordered lanelets + 새 trajectory 확인"),
        ("6", "#15803d", "Route B ACTIVE", "200346 진입 전에 모든 검증이 끝난 경우에만 전환 완료"),
    ]
    for index, (number, color, heading, detail) in enumerate(step_data):
        y = 185 + index * 91
        svg.append(f'<circle cx="650" cy="{y}" r="18" fill="white" stroke="{color}" stroke-width="4"/>')
        svg.append(f'<text x="650" y="{y+5}" class="marker-number" text-anchor="middle">{number}</text>')
        svg.append(f'<rect x="690" y="{y-31}" width="830" height="63" rx="12" fill="#fbfcfe" stroke="#dce3ed"/>')
        svg.append(f'<text x="712" y="{y-6}" class="label">{heading}</text>')
        svg.append(f'<text x="712" y="{y+18}" class="small">{detail}</text>')
        if index < len(step_data) - 1:
            svg.append(f'<path d="M 650 {y+20} L 650 {y+70}" stroke="#aebbd0" stroke-width="2" stroke-dasharray="5 4"/>')
    svg.append('<rect x="690" y="735" width="830" height="48" rx="10" fill="#fff3f0" stroke="#f3b5ad"/>')
    svg.append(f'<text x="712" y="756" class="label">실패/timeout: Route A 유지 → {policy["retry_reroute_interval"]:.1f}s 후 guard 재확인 → deadline 전 재시도</text>')
    svg.append('<text x="712" y="776" class="tiny">route_a_to_b는 실패 상태, route_a_to_b_with_fallback은 병합된 200130 cutoff에서 fallback 요청으로 승격한다.</text>')

    add_multiline(
        svg,
        620,
        900,
        [
            "DEADLINE = lanelet 200346의 진입 경계: 이 선을 넘기기 전에 Route B 적용·검증이 끝나야 한다.",
            "ANCHOR A = config의 deadline_pose_ref인 wp_200346(s=8 m): deadline 상태를 감지하는 설정 pose다.",
            "따라서 anchor가 deadline 자체라는 뜻이 아니다. 그림의 빨간 막대와 A 표식을 분리해서 읽는다.",
        ],
        "label",
        34,
    )
    svg.append(f'<text x="620" y="1013" class="tiny">Policy values are from competition_route_requester.param.yaml draft · request/verify timeouts {policy["service_timeout"]:.1f}s/{policy["route_verification_timeout"]:.1f}s/{policy["trajectory_verification_timeout"]:.1f}s · runtime ROS1 contract is not defined by this figure</text>')
    svg.append("</svg>")
    return "\n".join(svg) + "\n"


def build_operation_cases_svg(config, policy, map_path, actual_hash):
    selected_case = policy["route_operation_case"]
    fallback = config["fallback_route"]
    cutoff = config["poses"][fallback["hard_cutoff_pose_ref"]]
    cases = [
        ("route_a_only", "A", "사용 안 함", "사용 안 함", "SetWaypointRoute(A)"),
        ("route_b_only", "B@200178", "사용 안 함", "사용 안 함", "SetWaypointRoute(B)"),
        ("route_a_to_b", "A", "정상 A→B", "사용 안 함", "Waypoint A → Waypoint B"),
        ("route_a_to_fallback", "A", "사용 안 함", "cutoff에서 사용", "Waypoint A → Lanelet fallback"),
        (
            "route_a_to_b_with_fallback",
            "A",
            "정상 A→B 우선",
            "B 미확인 시 사용",
            "Waypoint A/B → Lanelet fallback",
        ),
    ]
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="980" viewBox="0 0 1500 980" role="img" aria-labelledby="title desc">',
        '<title id="title">Competition route operation cases and input-manager architecture</title>',
        '<desc id="desc">Five allowed YAML operation cases and the addon request path into RouteSelector main and MissionPlanner, without an output route mux.</desc>',
        f'<!-- source_map={map_path.name} sha256={actual_hash} selected_case={selected_case} -->',
        '<defs><style>',
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        '.title { font-size: 34px; font-weight: 700; } .subtitle { font-size: 16px; fill: #526079; }',
        '.panel-title { font-size: 22px; font-weight: 700; } .label { font-size: 15px; font-weight: 650; }',
        '.small { font-size: 13px; fill: #526079; } .tiny { font-size: 11px; fill: #67748a; }',
        '.mono { font-family: "DejaVu Sans Mono", monospace; font-size: 13px; }',
        '.card { fill: #fff; stroke: #dce3ed; stroke-width: 1.5; }',
        '</style></defs>',
        '<rect width="1500" height="980" fill="#f4f7fb"/>',
        '<text x="42" y="54" class="title">Competition route input manager · 5개 operation case</text>',
        '<text x="42" y="84" class="subtitle">YAML case 하나만 선택 · RouteSelector main 입력 관리 · Mission Planner 출력 뒤 별도 mux 없음</text>',
        '<rect x="30" y="110" width="1440" height="300" rx="22" class="card"/>',
        '<text x="55" y="148" class="panel-title">요청 구조</text>',
        '<rect x="55" y="175" width="245" height="78" rx="13" fill="#eaf1ff" stroke="#8eb2ff"/>',
        '<text x="177" y="205" class="label" text-anchor="middle">route_supervisor</text>',
        '<text x="177" y="229" class="small" text-anchor="middle">case·진행도·정상 A/B 정책</text>',
        '<rect x="55" y="285" width="245" height="78" rx="13" fill="#fff3f0" stroke="#f3b5ad"/>',
        '<text x="177" y="315" class="label" text-anchor="middle">finish_guard + fallback builder</text>',
        '<text x="177" y="339" class="small" text-anchor="middle">fallback case에서만 요청 생성</text>',
        '<rect x="375" y="230" width="230" height="78" rx="13" fill="#fff6dc" stroke="#e5bc54"/>',
        '<text x="490" y="260" class="label" text-anchor="middle">route_requester</text>',
        '<text x="490" y="284" class="small" text-anchor="middle">요청 1개 직렬화·우선순위</text>',
        '<rect x="680" y="230" width="245" height="78" rx="13" fill="#e6f7f4" stroke="#8ad7cd"/>',
        '<text x="802" y="260" class="label" text-anchor="middle">RouteSelector main 입력</text>',
        '<text x="802" y="284" class="small" text-anchor="middle">SetWaypointRoute / SetLaneletRoute</text>',
        '<rect x="1000" y="230" width="205" height="78" rx="13" fill="#f2eafe" stroke="#c5a0f5"/>',
        '<text x="1102" y="260" class="label" text-anchor="middle">MissionPlanner</text>',
        '<text x="1102" y="284" class="small" text-anchor="middle">LaneletRoute 생성·안전 검사</text>',
        '<rect x="1280" y="230" width="145" height="78" rx="13" fill="#eaf7ed" stroke="#8dcc9a"/>',
        '<text x="1352" y="260" class="label" text-anchor="middle">BPP</text>',
        '<text x="1352" y="284" class="small" text-anchor="middle">기존 입력 유지</text>',
        '<path d="M 300 214 C 335 214 340 248 375 262" fill="none" stroke="#718096" stroke-width="2.5"/>',
        '<path d="M 300 324 C 335 324 340 290 375 278" fill="none" stroke="#718096" stroke-width="2.5"/>',
        '<path d="M 605 269 L 680 269 M 925 269 L 1000 269 M 1205 269 L 1280 269" stroke="#718096" stroke-width="2.5"/>',
        '<text x="642" y="254" class="tiny" text-anchor="middle">요청</text>',
        '<text x="962" y="254" class="tiny" text-anchor="middle">전달</text>',
        '<text x="1242" y="254" class="tiny" text-anchor="middle">실제 route</text>',
        '<rect x="30" y="430" width="1440" height="455" rx="22" class="card"/>',
        '<text x="55" y="470" class="panel-title">YAML에서 허용하는 5개 case</text>',
        f'<rect x="1080" y="447" width="350" height="38" rx="19" fill="#172033"/><text x="1255" y="472" fill="white" style="fill:white;font-size:13px;font-weight:700" text-anchor="middle">현재 설정: {selected_case}</text>',
        '<text x="70" y="520" class="small">route_operation_case</text>',
        '<text x="515" y="520" class="small">초기</text>',
        '<text x="620" y="520" class="small">정상 재경로</text>',
        '<text x="830" y="520" class="small">결승선 fallback</text>',
        '<text x="1080" y="520" class="small">RouteSelector main 요청</text>',
        '<path d="M 55 535 L 1445 535" stroke="#dce3ed"/>',
    ]
    for index, (case_name, initial, reroute, fallback_text, request) in enumerate(cases):
        y = 575 + index * 58
        selected = case_name == selected_case
        fill = "#eaf1ff" if selected else "#fbfcfe"
        stroke = "#2563eb" if selected else "#dce3ed"
        svg.append(f'<rect x="55" y="{y-31}" width="1390" height="48" rx="10" fill="{fill}" stroke="{stroke}"/>')
        svg.append(f'<text x="70" y="{y}" class="mono">{case_name}</text>')
        svg.append(f'<text x="535" y="{y}" class="label" text-anchor="middle">{initial}</text>')
        svg.append(f'<text x="620" y="{y}" class="small">{reroute}</text>')
        svg.append(f'<text x="830" y="{y}" class="small">{fallback_text}</text>')
        svg.append(f'<text x="1080" y="{y}" class="small">{request}</text>')
    svg.extend(
        [
            '<rect x="30" y="905" width="1440" height="50" rx="16" fill="#fff6dc" stroke="#e5bc54"/>',
            f'<text x="55" y="936" class="label">fallback cutoff = {fallback["hard_cutoff_pose_ref"]} · lanelet {cutoff["lanelet_id"]} · s={cutoff["lanelet_s_m"]:.1f}m · goal={fallback["goal_ref"]} · UUID={fallback["uuid"]}</text>',
            '<text x="1445" y="936" class="tiny" text-anchor="end">잘못된 case는 route service 호출 전에 시작 거부</text>',
            '</svg>',
        ]
    )
    return "\n".join(svg) + "\n"


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    param_config_path = args.param_config.resolve()
    workspace_dir = args.workspace.resolve()
    output_path = args.output.resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    policy = yaml.safe_load(param_config_path.read_text(encoding="utf-8"))

    allowed_operation_cases = {
        "route_a_only",
        "route_b_only",
        "route_a_to_b",
        "route_a_to_fallback",
        "route_a_to_b_with_fallback",
    }
    operation_case = policy.get("route_operation_case")
    if operation_case not in allowed_operation_cases:
        raise ValueError(
            f"Invalid route_operation_case {operation_case!r}; "
            f"allowed values are {sorted(allowed_operation_cases)}"
        )
    obsolete_switches = [key for key in policy if key.startswith("enable_")]
    if obsolete_switches:
        raise ValueError(f"Obsolete independent enable switches remain: {obsolete_switches}")

    map_path = (workspace_dir / config["map"]["file"]).resolve()
    projector_path = (map_path.parent / Path(config["map"]["projector_file"]).name).resolve()
    actual_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
    expected_hash = config["map"]["sha256"]
    if actual_hash != expected_hash:
        raise ValueError(f"Map SHA-256 mismatch: expected {expected_hash}, got {actual_hash}")

    projector = yaml.safe_load(projector_path.read_text(encoding="utf-8"))
    origin = projector["map_origin"]
    lanelets, epsg = load_osm(map_path, float(origin["latitude"]), float(origin["longitude"]))

    route_a_ids = config["route_a"]["expected_path_lanelet_ids"]
    route_b_ids = config["route_b"]["expected_path_lanelet_ids"]
    incoming_start_id = config["start"]["incoming_lanelet_id"]
    outgoing_start_id = config["start"]["outgoing_lanelet_id"]
    expected_start_prefixes = [[outgoing_start_id], [incoming_start_id, outgoing_start_id]]
    if config["route_a"]["accepted_start_prefixes"] != expected_start_prefixes:
        raise ValueError("Route A accepted start prefixes do not match the official lanelet boundary")
    if route_a_ids[0] != outgoing_start_id:
        raise ValueError("Route A canonical path must begin on the outgoing start lanelet")
    route_a_goal_id = config["poses"][config["route_a"]["goal_ref"]]["lanelet_id"]
    if route_a_ids[-1] != route_a_goal_id:
        raise ValueError("Route A canonical path does not end on its configured goal lanelet")
    if incoming_start_id in route_a_ids:
        raise ValueError("Route A must end before re-entering the incoming START/END lanelet")
    trigger_id = config["poses"][config["reroute_trigger"]["pose_ref"]]["lanelet_id"]
    trigger_index_a = route_a_ids.index(trigger_id)
    shared_ids = route_a_ids[trigger_index_a:]
    if route_b_ids[: len(shared_ids)] != shared_ids:
        raise ValueError("Route B prefix is not the Route A suffix beginning at the trigger lanelet")

    fallback = config["fallback_route"]
    if fallback["request_type"] != "set_lanelet_route":
        raise ValueError("Fallback must use SetLaneletRoute through RouteSelector main")
    fallback_cutoff = config["poses"][fallback["hard_cutoff_pose_ref"]]
    fallback_cutoff_id = fallback_cutoff["lanelet_id"]
    fallback_ids = fallback["expected_path_lanelet_ids"]
    expected_fallback_ids = route_b_ids[route_b_ids.index(fallback_cutoff_id) :]
    if fallback_ids != expected_fallback_ids:
        raise ValueError("Fallback path must be the Route B suffix beginning at the hard cutoff")
    if fallback_ids[-1] != config["poses"][fallback["goal_ref"]]["lanelet_id"]:
        raise ValueError("Fallback path does not end on its configured goal lanelet")
    if len(fallback_ids) != len(set(fallback_ids)):
        raise ValueError("Fallback path must not contain a lanelet loop")

    approach_ids = route_a_ids[:trigger_index_a]
    continuation_ids = route_b_ids[len(shared_ids) :]
    all_route_ids = list(dict.fromkeys(route_a_ids + route_b_ids))
    missing_ids = [lanelet_id for lanelet_id in all_route_ids if lanelet_id not in lanelets]
    if missing_ids:
        raise ValueError(f"Route lanelets missing from map: {missing_ids}")

    approach = joined_centerlines(lanelets, approach_ids, lateral_tolerance=5.0)
    shared = joined_centerlines(lanelets, shared_ids, lateral_tolerance=5.0)
    continuation = joined_centerlines(lanelets, continuation_ids, lateral_tolerance=5.0)
    full_route_points = [point for lanelet_id in all_route_ids for point in lanelets[lanelet_id]["centerline"]]

    pose_values = config["poses"]
    trigger_pose = pose_values[config["reroute_trigger"]["pose_ref"]]
    deadline_pose = pose_values[config["reroute_trigger"]["deadline_pose_ref"]]
    route_a_goal = pose_values[config["route_a"]["goal_ref"]]
    route_b_goal = pose_values[config["route_b"]["goal_ref"]]

    named_poses = [trigger_pose, deadline_pose, fallback_cutoff, route_a_goal, route_b_goal]
    for pose in named_poses:
        centerline = lanelets[pose["lanelet_id"]]["centerline"]
        if "lanelet_s_m" in pose:
            calculated = points_at_distance(centerline, float(pose["lanelet_s_m"]))
        else:
            calculated = points_at_distance(centerline, polyline_length(centerline) * float(pose["lanelet_ratio"]))
        configured = (float(pose["x"]), float(pose["y"]))
        if math.dist(calculated, configured) > 0.01:
            raise ValueError(f"Configured pose is not on lanelet {pose['lanelet_id']}: {configured} vs {calculated}")

    trigger_point = (float(trigger_pose["x"]), float(trigger_pose["y"]))
    deadline_anchor = (float(deadline_pose["x"]), float(deadline_pose["y"]))
    goal_a_point = (float(route_a_goal["x"]), float(route_a_goal["y"]))
    goal_b_point = (float(route_b_goal["x"]), float(route_b_goal["y"]))
    start_pose = config["start"]["map_pose"]
    start_anchor = (float(start_pose["x"]), float(start_pose["y"]))

    incoming_start_line = lanelets[config["start"]["incoming_lanelet_id"]]["centerline"]
    outgoing_start_line = lanelets[config["start"]["outgoing_lanelet_id"]]["centerline"]
    if math.dist(start_anchor, incoming_start_line[-1]) > 0.01:
        raise ValueError("Official start is not at the end of the incoming start lanelet")
    if math.dist(start_anchor, outgoing_start_line[0]) > 0.01:
        raise ValueError("Official start is not at the beginning of the outgoing start lanelet")

    main_bounds = bounds_for(full_route_points, 24.0)
    inset_ids = [200178, 200337, 200287, 200348, 200346]
    inset_points = [point for lanelet_id in inset_ids for point in lanelets[lanelet_id]["centerline"]]
    inset_bounds = bounds_for(inset_points, 18.0)
    main_view = Viewport(main_bounds, (55, 145, 370, 990), padding=8)
    inset_view = Viewport(inset_bounds, (475, 170, 415, 555), padding=16)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{SVG_WIDTH}" height="{SVG_HEIGHT}" viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v74 competition Route A and Route B</title>',
        '<desc id="desc">North-up map of Route A ending on lanelet 200181 before the start boundary, Route B continuation through the start boundary, reroute trigger at lanelet 200178 s 8 metres, deadline before lanelet 200346, and final goal 200210.</desc>',
        f'<!-- source_map={map_path.name} sha256={actual_hash} projection=EPSG:{epsg} -->',
        "<defs>",
        '<clipPath id="main-clip"><rect x="55" y="145" width="370" height="990" rx="18"/></clipPath>',
        '<clipPath id="inset-clip"><rect x="475" y="170" width="415" height="555" rx="18"/></clipPath>',
        "<style>",
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        ".title { font-size: 34px; font-weight: 700; }",
        ".subtitle { font-size: 16px; fill: #526079; }",
        ".panel-title { font-size: 21px; font-weight: 700; }",
        ".label { font-size: 15px; font-weight: 600; }",
        ".small { font-size: 13px; fill: #526079; }",
        ".tiny { font-size: 11px; fill: #67748a; }",
        ".marker-number { font-size: 12px; font-weight: 800; }",
        ".card { fill: #ffffff; stroke: #dce3ed; stroke-width: 1.5; }",
        ".road { fill: none; stroke: #d7dde5; stroke-width: 1.2; stroke-linecap: round; }",
        "</style>",
        "</defs>",
        '<rect width="1400" height="1200" fill="#f4f7fb"/>',
        '<text x="42" y="54" class="title">K-City v74 경쟁 경로 · Route A → Route B</text>',
        '<text x="42" y="84" class="subtitle">Lanelet2 중심선 기반 · 북쪽이 위 · 좌표 단위 m · 설정된 경로 순서 그대로 표시</text>',
        '<rect x="35" y="110" width="405" height="1045" rx="22" class="card"/>',
        '<text x="55" y="140" class="panel-title">전체 경로</text>',
        '<rect x="55" y="145" width="370" height="990" rx="18" fill="#fbfcfe"/>',
        '<rect x="460" y="110" width="445" height="665" rx="22" class="card"/>',
        '<text x="480" y="144" class="panel-title">재경로 구간 확대</text>',
        '<rect x="475" y="170" width="415" height="555" rx="18" fill="#fbfcfe"/>',
        '<rect x="925" y="110" width="440" height="665" rx="22" class="card"/>',
        '<text x="950" y="150" class="panel-title">적용 순서와 안전 경계</text>',
        '<rect x="460" y="795" width="905" height="360" rx="22" class="card"/>',
        '<text x="485" y="836" class="panel-title">선과 표식 읽는 법</text>',
    ]

    # Background road network, clipped to each geographic panel.
    for clip_id, viewport, data_bounds in (
        ("main-clip", main_view, main_bounds),
        ("inset-clip", inset_view, inset_bounds),
    ):
        svg.append(f'<g clip-path="url(#{clip_id})">')
        for lanelet in lanelets.values():
            centerline = lanelet["centerline"]
            if lanelet["tags"].get("subtype") == "road" and intersects(centerline, data_bounds):
                svg.append(f'<path d="{viewport.path(centerline)}" class="road"/>')
        svg.append("</g>")

    # Main route categories: blue approach, teal shared route, orange continuation.
    svg.append('<g clip-path="url(#main-clip)">')
    svg.append(f'<path d="{main_view.path(approach)}" fill="none" stroke="#2563eb" stroke-width="12" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append(f'<path d="{main_view.path(shared)}" fill="none" stroke="#0f9f8f" stroke-width="12" stroke-linecap="round" stroke-linejoin="round"/>')
    # B continuation may reuse 200889 spatially; a narrower orange overlay preserves both readings.
    svg.append(f'<path d="{main_view.path(continuation)}" fill="none" stroke="#f28c28" stroke-width="8" stroke-linecap="round" stroke-linejoin="round"/>')
    inset_min_x, inset_max_x, inset_min_y, inset_max_y = inset_bounds
    top_left = main_view.point((inset_min_x, inset_max_y))
    bottom_right = main_view.point((inset_max_x, inset_min_y))
    svg.append(
        f'<rect x="{top_left[0]:.1f}" y="{top_left[1]:.1f}" width="{bottom_right[0]-top_left[0]:.1f}" '
        f'height="{bottom_right[1]-top_left[1]:.1f}" fill="none" stroke="#b42318" stroke-width="2" stroke-dasharray="6 5"/>'
    )
    svg.append("</g>")

    for fraction in (0.30, 0.72):
        arrow(svg, main_view, shared, fraction, "#0b776c")
    arrow(svg, main_view, continuation, 0.68, "#c66012")
    arrow(svg, main_view, approach, 0.55, "#174ea6")

    marker(svg, main_view, start_anchor, "#2563eb", "1")
    marker(svg, main_view, trigger_point, "#0f9f8f", "2")
    marker(svg, main_view, deadline_anchor, "#b42318", "3")
    marker(svg, main_view, goal_a_point, "#6d28d9", "4")
    marker(svg, main_view, goal_b_point, "#f28c28", "5")
    svg.append('<text x="75" y="178" class="small">N</text><path d="M 82 214 L 82 185 M 82 185 L 76 196 M 82 185 L 88 196" stroke="#172033" stroke-width="2" fill="none"/>')

    # Inset route and request window.
    inset_route = joined_centerlines(lanelets, inset_ids)
    deadline_entry = lanelets[deadline_pose["lanelet_id"]]["centerline"][0]
    trigger_line = lanelets[trigger_pose["lanelet_id"]]["centerline"]
    nearest_trigger_index = min(range(len(trigger_line)), key=lambda i: math.dist(trigger_line[i], trigger_point))
    request_window = [trigger_point] + trigger_line[nearest_trigger_index + 1 :]
    for lanelet_id in inset_ids[1:-1]:
        next_line = lanelets[lanelet_id]["centerline"]
        request_window.extend(next_line[1:] if math.dist(request_window[-1], next_line[0]) < 0.1 else next_line)
    request_window.append(deadline_entry)

    svg.append('<g clip-path="url(#inset-clip)">')
    svg.append(f'<path d="{inset_view.path(request_window)}" fill="none" stroke="#ffd7d2" stroke-width="24" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append(f'<path d="{inset_view.path(inset_route)}" fill="none" stroke="#0f9f8f" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>')
    for lanelet_id, color in ((200178, "#0f9f8f"), (200346, "#b42318")):
        lanelet = lanelets[lanelet_id]
        polygon = lanelet["left"] + list(reversed(lanelet["right"]))
        svg.append(f'<path d="{inset_view.path(polygon)} Z" fill="none" stroke="{color}" stroke-width="2.5" stroke-dasharray="7 5"/>')
    svg.append("</g>")
    for fraction in (0.20, 0.55, 0.83):
        arrow(svg, inset_view, inset_route, fraction, "#0b776c")

    # Deadline bar at lanelet 200346 entry.
    deadline_line = lanelets[deadline_pose["lanelet_id"]]["centerline"]
    direction = (deadline_line[1][0] - deadline_line[0][0], deadline_line[1][1] - deadline_line[0][1])
    direction_norm = math.hypot(*direction)
    normal = (-direction[1] / direction_norm, direction[0] / direction_norm)
    bar_a = (deadline_entry[0] + normal[0] * 8.0, deadline_entry[1] + normal[1] * 8.0)
    bar_b = (deadline_entry[0] - normal[0] * 8.0, deadline_entry[1] - normal[1] * 8.0)
    svg.append(f'<path d="{inset_view.path([bar_a, bar_b])}" stroke="#b42318" stroke-width="5" stroke-linecap="round"/>')

    marker(svg, inset_view, trigger_point, "#0f9f8f", "2", radius=12)
    marker(svg, inset_view, deadline_anchor, "#b42318", "3", radius=12)
    entry_x, entry_y = inset_view.point(deadline_entry)
    svg.append(f'<circle cx="{entry_x:.1f}" cy="{entry_y:.1f}" r="5" fill="#b42318"/>')
    tx, ty = inset_view.point(trigger_point)
    add_multiline(svg, tx + 28, ty - 18, ["TRIGGER", "200178 · s ≥ 8 m"], "label", 21)
    dx, dy = inset_view.point(deadline_anchor)
    add_multiline(svg, dx + 28, dy - 5, ["설정 anchor", "200346 · s = 8 m"], "small", 20)
    add_multiline(svg, entry_x - 20, entry_y - 36, ["DEADLINE", "200346 진입 전 적용"], "label", 21, anchor="end")
    svg.append('<rect x="493" y="682" width="17" height="10" rx="5" fill="#ffd7d2"/>')
    svg.append('<text x="520" y="692" class="small">Route B 요청·검증을 끝내야 하는 구간</text>')

    # Right-side sequence.
    steps = [
        ("1", "#2563eb", "Route A 시작", "공식 200892→200889 경계의 현재 odometry 확인"),
        ("2", "#0f9f8f", "Route B 요청 준비", "lanelet 200178에 s ≥ 8 m 진입"),
        ("3", "#b42318", "Route B 적용 deadline", "lanelet 200346 진입 전에 교체·검증 완료"),
        ("4", "#6d28d9", "Route A 임시 goal", "200892 재진입 전 · lanelet 200181의 0.75 지점"),
        ("5", "#f28c28", "Route B 최종 goal", "200892→200889 경계를 통과해 200210에서 종료"),
    ]
    for index, (number, color, heading, detail) in enumerate(steps):
        cy = 205 + index * 103
        svg.append(f'<circle cx="970" cy="{cy}" r="18" fill="white" stroke="{color}" stroke-width="4"/>')
        svg.append(f'<text x="970" y="{cy+5}" class="marker-number" text-anchor="middle">{number}</text>')
        svg.append(f'<text x="1005" y="{cy-5}" class="label">{heading}</text>')
        svg.append(f'<text x="1005" y="{cy+22}" class="small">{detail}</text>')
        if index < len(steps) - 1:
            svg.append(f'<path d="M 970 {cy+22} L 970 {cy+78}" stroke="#c9d2df" stroke-width="2" stroke-dasharray="5 5"/>')

    # Legend and key caveats.
    legend_y = 888
    legend = [
        ("#2563eb", 12, "Route A 출발→trigger 전용 구간"),
        ("#0f9f8f", 12, "Route A/B 공통 구간: 200178→200181"),
        ("#f28c28", 8, "Route B 연장 구간: 200892→200210"),
    ]
    for index, (color, width, text_value) in enumerate(legend):
        y = legend_y + index * 46
        svg.append(f'<path d="M 495 {y} L 565 {y}" stroke="{color}" stroke-width="{width}" stroke-linecap="round"/>')
        svg.append(f'<text x="585" y="{y+5}" class="label">{text_value}</text>')

    add_multiline(
        svg,
        920,
        887,
        [
            "• 공식 START/END는 200892 끝과 200889 시작의 동일 경계점이다.",
            "• 시작 표식 ①은 공식 좌표이며 실제 요청 start pose는 현재 odometry다.",
            "• 200346의 빨간 막대는 ‘진입 전 적용’ 경계, ③은 YAML의 s=8 anchor다.",
            "• Route A를 지우지 않은 상태에서 Route B 성공을 확인하는 운영 명세다.",
        ],
        "small",
        34,
    )
    add_multiline(
        svg,
        495,
        1050,
        [
            f"Source: {config_path.name} + {map_path.name} (SHA-256 {actual_hash[:12]}…)",
            f"Projection: LocalCartesianUTM / EPSG:{epsg}, origin ({origin['latitude']}, {origin['longitude']})",
            "Generated by tools/generate_route_visualization.py · geometry is documentation evidence, not a runtime route command",
        ],
        "tiny",
        24,
    )
    svg.append("</svg>")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(svg) + "\n", encoding="utf-8")
    detail_outputs = {
        output_path.parent / "kcity_v74_route_a_detail.svg": build_route_detail_svg(
            config, lanelets, map_path, actual_hash, epsg, origin, "route_a"
        ),
        output_path.parent / "kcity_v74_route_b_detail.svg": build_route_detail_svg(
            config, lanelets, map_path, actual_hash, epsg, origin, "route_b"
        ),
        output_path.parent / "kcity_v74_reroute_process.svg": build_reroute_process_svg(
            config, policy, lanelets, map_path, actual_hash, epsg
        ),
        output_path.parent / "kcity_v74_operation_cases.svg": build_operation_cases_svg(
            config, policy, map_path, actual_hash
        ),
        output_path.parent / "kcity_v74_highway_lane_strategy.svg": build_highway_lane_strategy_svg(
            config, lanelets, map_path, actual_hash, epsg
        ),
    }
    for detail_path, content in detail_outputs.items():
        detail_path.write_text(content, encoding="utf-8")
    print(f"wrote {output_path}")
    for detail_path in detail_outputs:
        print(f"wrote {detail_path}")
    print(f"validated map sha256: {actual_hash}")
    print(f"validated route A lanelets: {len(route_a_ids)}")
    print(f"validated route B lanelets: {len(route_b_ids)}")
    print(f"validated shared lanelets: {len(shared_ids)}")
    print(f"validated fallback lanelets: {len(fallback_ids)}")
    print(f"validated operation case: {operation_case}")


if __name__ == "__main__":
    main()
