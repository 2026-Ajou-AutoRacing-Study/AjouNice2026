#!/usr/bin/env python3
"""Generate the v81 initial-route windows and active A-to-B trigger diagram."""

import hashlib
from pathlib import Path

import yaml

from generate_route_visualization import (
    Viewport,
    add_map_background,
    add_multiline,
    arrow,
    bounds_for,
    joined_centerlines,
    load_osm,
    marker,
    points_at_distance,
)


def main():
    package = Path(__file__).resolve().parents[1]
    repo = Path(__file__).resolve().parents[5]
    config_path = package / "config/kcity_v81_competition_route.yaml"
    output_path = package / "docs/assets/kcity_v81_route_activation_windows.svg"
    config = yaml.safe_load(config_path.read_text())
    map_path = repo / config["map"]["file"]
    actual_hash = hashlib.sha256(map_path.read_bytes()).hexdigest()
    if actual_hash != config["map"]["sha256"]:
        raise ValueError("v81 map SHA mismatch")
    projector = yaml.safe_load((repo / config["map"]["projector_file"]).read_text())
    origin = projector["map_origin"]
    lanelets, epsg = load_osm(map_path, origin["latitude"], origin["longitude"])

    route_a = config["route_a"]["expected_path_lanelet_ids"]
    route_b = config["route_b"]["waypoint_path_lanelet_ids"]
    trigger_pose = config["poses"][config["reroute_trigger"]["pose_ref"]]
    trigger_lane = trigger_pose["lanelet_id"]
    end_lane = config["startup_route_b_trigger"]["end_lanelet_id"]
    a_ids = route_a[: route_a.index(trigger_lane)]
    b_ids = route_b[route_b.index(trigger_lane) : route_b.index(end_lane) + 1]
    a_line = joined_centerlines(lanelets, a_ids, lateral_tolerance=5.0)
    b_line = joined_centerlines(lanelets, b_ids, lateral_tolerance=12.0)

    all_points = a_line + b_line
    main_bounds = bounds_for(all_points, 25.0)
    main_view = Viewport(main_bounds, (45, 145, 600, 820), padding=18)

    inset_ids = [200308, 200258, 200261]
    inset_line = joined_centerlines(lanelets, inset_ids, lateral_tolerance=5.0)
    inset_points = [p for lane in inset_ids for p in lanelets[lane]["centerline"]]
    inset_bounds = bounds_for(inset_points, 13.0)
    inset_view = Viewport(inset_bounds, (710, 145, 735, 390), padding=24)

    start = config["start"]["map_pose"]
    start_point = (float(start["x"]), float(start["y"]))
    trigger_point = (float(trigger_pose["x"]), float(trigger_pose["y"]))
    b_entry = lanelets[trigger_lane]["centerline"][0]
    svg = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1500" height="1020" viewBox="0 0 1500 1020" role="img" aria-labelledby="title desc">',
        '<title id="title">K-City v81 Route A/B activation windows</title>',
        '<desc id="desc">Initial Route A and Route B request windows and the active Route A to B transition point on lanelet 200258.</desc>',
        f'<!-- source={config_path.name} map={map_path.name} sha256={actual_hash} projection=EPSG:{epsg} -->',
        '<defs><style>',
        "text { font-family: 'Noto Sans CJK KR', 'Noto Sans KR', sans-serif; fill: #172033; }",
        '.title { font-size: 34px; font-weight: 750; }.subtitle { font-size: 16px; fill: #526079; }',
        '.panel { fill: #fff; stroke: #dce3ed; stroke-width: 1.5; }.panel-title { font-size: 21px; font-weight: 700; }',
        '.label { font-size: 15px; font-weight: 700; }.small { font-size: 13px; fill: #526079; }',
        '.tiny { font-size: 11px; fill: #67748a; }.marker-number { font-size: 11px; font-weight: 800; }',
        '.road { fill: none; stroke: #d7dde5; stroke-width: 1.1; stroke-linecap: round; }',
        '</style>',
        '<clipPath id="main-clip"><rect x="45" y="145" width="600" height="820" rx="18"/></clipPath>',
        '<clipPath id="inset-clip"><rect x="710" y="145" width="735" height="390" rx="18"/></clipPath>',
        '</defs>',
        '<rect width="1500" height="1020" fill="#f4f7fb"/>',
        '<text x="42" y="54" class="title">K-City v81 · Route 요청 구간 구분</text>',
        '<text x="42" y="84" class="subtitle">활성 경로가 없을 때의 A/B 선택과 Route A 활성 상태의 A→B 전환점을 분리해 표시</text>',
        '<rect x="25" y="110" width="640" height="885" rx="22" class="panel"/>',
        '<text x="50" y="138" class="panel-title">전체 경로 · 북쪽이 위</text>',
        '<rect x="690" y="110" width="780" height="450" rx="22" class="panel"/>',
        '<text x="715" y="138" class="panel-title">lanelet 200258 확대</text>',
        '<rect x="690" y="585" width="780" height="410" rx="22" class="panel"/>',
        '<text x="720" y="625" class="panel-title">판정 규칙</text>',
    ]

    add_map_background(svg, lanelets, main_view, main_bounds, "main-clip")
    svg.append('<g clip-path="url(#main-clip)">')
    svg.append(f'<path d="{main_view.path(a_line)}" fill="none" stroke="#2563eb" stroke-width="12" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append(f'<path d="{main_view.path(b_line)}" fill="none" stroke="#f28c28" stroke-width="10" stroke-linecap="round" stroke-linejoin="round"/>')
    svg.append('</g>')
    for fraction in (0.35, 0.75):
        arrow(svg, main_view, a_line, fraction, "#174ea6")
    for fraction in (0.18, 0.52, 0.84):
        arrow(svg, main_view, b_line, fraction, "#c66012")
    marker(svg, main_view, start_point, "#6d28d9", "S/E", radius=15)
    marker(svg, main_view, trigger_point, "#d12f53", "T", radius=12)

    add_map_background(svg, lanelets, inset_view, inset_bounds, "inset-clip")
    svg.append('<g clip-path="url(#inset-clip)">')
    svg.append(f'<path d="{inset_view.path(inset_line)}" fill="none" stroke="#94a3b8" stroke-width="14" stroke-linecap="round"/>')
    lane_200258 = lanelets[200258]["centerline"]
    svg.append(f'<path d="{inset_view.path(lane_200258)}" fill="none" stroke="#f28c28" stroke-width="10" stroke-linecap="round"/>')
    transition_tail = [trigger_point] + [p for p in lane_200258 if p[1] > trigger_point[1]]
    svg.append(f'<path d="{inset_view.path(transition_tail)}" fill="none" stroke="#d12f53" stroke-width="4" stroke-dasharray="8 6"/>')
    svg.append('</g>')
    marker(svg, inset_view, b_entry, "#f28c28", "B", radius=13)
    marker(svg, inset_view, trigger_point, "#d12f53", "T", radius=13)
    bx, by = inset_view.point(b_entry)
    tx, ty = inset_view.point(trigger_point)
    add_multiline(svg, bx - 30, by + 2, ["활성 경로 없음", "200258 진입(s=0)부터 Route B"], "label", 22, anchor="end")
    add_multiline(svg, tx + 32, ty - 16, ["Route A 활성 상태", "200258 s=8 m에서 A→B"], "label", 22)
    svg.append('<text x="730" y="510" class="small">회색: 인접 중심선 · 주황: 최초 Route B 구간 · 빨간 점선: A 활성 상태에서 전환 후 구간</text>')

    rules = [
        ("①", "#2563eb", "활성 경로 없음 → Route A", "공식 출발점부터 200258 진입 직전까지"),
        ("②", "#f28c28", "활성 경로 없음 → Route B", "200258 진입(s=0)부터 200892 내부까지"),
        ("③", "#d12f53", "Route A 활성 → Route B 전환", "lanelet 200258의 local s가 8 m 이상일 때만"),
        ("④", "#6d28d9", "200892 끝점", "공식 출발 경계와 동일하므로 정확한 끝점은 Route A"),
    ]
    for i, (number, color, heading, detail) in enumerate(rules):
        y = 680 + i * 72
        svg.append(f'<circle cx="742" cy="{y-5}" r="17" fill="white" stroke="{color}" stroke-width="4"/>')
        svg.append(f'<text x="742" y="{y}" class="marker-number" text-anchor="middle">{number}</text>')
        svg.append(f'<text x="780" y="{y-7}" class="label">{heading}</text>')
        svg.append(f'<text x="780" y="{y+18}" class="small">{detail}</text>')
    add_multiline(svg, 1120, 680, [
        "경계 요약",
        "START ── Route A ── 200258(s=0)",
        "200258(s=0) ── Route B ── 200892 내부",
        "A가 이미 활성: 200258(s=8)에서 B 전환",
    ], "label", 31)
    add_multiline(svg, 1120, 835, [
        "주의",
        "200892 끝과 200889 시작은 같은 공식 출발점이다.",
        "현재 pose가 정확한 경계점이면 Route A로 판정한다.",
        "200892 이후 200220부터는 최초 B 요청 범위 밖이다.",
    ], "small", 27)
    svg.append(f'<text x="720" y="970" class="tiny">Generated from {config_path.name} + {map_path.name} · SHA-256 {actual_hash[:16]}…</text>')
    svg.append('</svg>')
    output_path.write_text("\n".join(svg) + "\n", encoding="utf-8")
    print(output_path)


if __name__ == "__main__":
    main()
