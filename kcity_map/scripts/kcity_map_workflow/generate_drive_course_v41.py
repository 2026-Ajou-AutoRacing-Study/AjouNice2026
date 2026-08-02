#!/usr/bin/env python3
"""Generate v41 drive-course CSV and a thin-line presentation map."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml
from PIL import Image
from pyproj import Transformer


Point3 = Tuple[float, float, float]

PREFERRED_ROUTE = [
    200889,
    200392,
    200178,
    200337,
    200287,
    200348,
    200346,
    200291,
    200292,
    200305,
    200841,
    200950,
    200310,
    200308,
    200258,
    200261,
    200315,
    200445,
    200311,
    200968,
    201095,
    201098,
    201096,
    201087,
    201086,
    200101,
    200128,
    200102,
    200134,
    200360,
    201399,
    200088,
    201401,
    200183,
    200037,
    200471,
    201071,
    200182,
    200361,
    200181,
    200892,
    200889,
]

CORRIDOR_CANDIDATES = [
    200099,
    200152,
    200142,
    200137,
    200138,
    200186,
    200187,
    200188,
    200141,
    200130,
    200127,
    200148,
    200185,
    200147,
    200150,
    200149,
    201402,
]

EXCLUDED_LANELETS = [200136]

MANEUVER_KO = {
    "straight": "\uc9c1\uc9c4",
    "left_curve": "\uc88c\ucee4\ube0c",
    "right_curve": "\uc6b0\ucee4\ube0c",
    "left_turn": "\uc88c\ud68c\uc804",
    "right_turn": "\uc6b0\ud68c\uc804",
    "u_turn": "\uc720\ud134",
    "roundabout": "\ud68c\uc804\uad50\ucc28\ub85c",
}

ROUNDABOUT_HINT_IDS = {201095, 201098, 201096, 201087, 201086, 200183}


@dataclass
class WayData:
    way_id: int
    refs: List[str]
    tags: Dict[str, str]
    points: List[Point3]


@dataclass
class LaneletData:
    lanelet_id: int
    left_way_id: Optional[int]
    right_way_id: Optional[int]
    center_way_id: Optional[int]
    left: List[Point3]
    right: List[Point3]
    center: List[Point3]
    tags: Dict[str, str]

    @property
    def speed_kph(self) -> Optional[float]:
        value = self.tags.get("speed_limit") or self.tags.get("maxspeed") or self.tags.get("max_speed")
        if value is None:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    @property
    def length_m(self) -> float:
        return polyline_length(self.center)


class LocalMapProjector:
    def __init__(self, projector_yaml: Path, utm_projection: str) -> None:
        config = yaml.safe_load(projector_yaml.read_text(encoding="utf-8"))
        origin = config["map_origin"]
        self.origin_altitude = float(origin.get("altitude", 0.0))
        self.to_utm = Transformer.from_crs("EPSG:4326", utm_projection, always_xy=True)
        self.origin_x, self.origin_y = self.to_utm.transform(
            float(origin["longitude"]),
            float(origin["latitude"]),
        )

    def from_latlon(self, lat: float, lon: float, z: float) -> Point3:
        x, y = self.to_utm.transform(float(lon), float(lat))
        return (x - self.origin_x, y - self.origin_y, float(z) - self.origin_altitude)


def tags(element) -> Dict[str, str]:
    return {tag.attrib.get("k"): tag.attrib.get("v") for tag in element.findall("tag") if tag.attrib.get("k")}


def load_lanelet_osm(map_path: Path, projector: LocalMapProjector) -> Tuple[Dict[int, LaneletData], Dict[int, WayData]]:
    root = ET.parse(map_path).getroot()
    nodes: Dict[str, Point3] = {}
    for node in root.findall("node"):
        node_tags = tags(node)
        ele = float(node_tags.get("ele", 0.0))
        nodes[node.attrib["id"]] = projector.from_latlon(float(node.attrib["lat"]), float(node.attrib["lon"]), ele)

    ways: Dict[int, WayData] = {}
    for way in root.findall("way"):
        way_id = int(way.attrib["id"])
        refs = [nd.attrib["ref"] for nd in way.findall("nd")]
        ways[way_id] = WayData(way_id, refs, tags(way), [nodes[ref] for ref in refs if ref in nodes])

    lanelets: Dict[int, LaneletData] = {}
    for relation in root.findall("relation"):
        rel_tags = tags(relation)
        if rel_tags.get("type") != "lanelet" or rel_tags.get("subtype") != "road":
            continue
        member_by_role: Dict[str, int] = {}
        for member in relation.findall("member"):
            if member.attrib.get("type") == "way" and member.attrib.get("role") in ("left", "right", "centerline"):
                member_by_role[member.attrib["role"]] = int(member.attrib["ref"])
        left_way = ways.get(member_by_role.get("left", -1))
        right_way = ways.get(member_by_role.get("right", -1))
        center_way = ways.get(member_by_role.get("centerline", -1))
        lanelet_id = int(relation.attrib["id"])
        lanelets[lanelet_id] = LaneletData(
            lanelet_id=lanelet_id,
            left_way_id=member_by_role.get("left"),
            right_way_id=member_by_role.get("right"),
            center_way_id=member_by_role.get("centerline"),
            left=list(left_way.points if left_way else []),
            right=list(right_way.points if right_way else []),
            center=list(center_way.points if center_way else []),
            tags=rel_tags,
        )
    return lanelets, ways


def distance(a: Point3, b: Point3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polyline_length(points: Sequence[Point3]) -> float:
    return sum(distance(a, b) for a, b in zip(points, points[1:]))


def interpolate_polyline(points: Sequence[Point3], fraction: float) -> Tuple[Point3, float]:
    if not points:
        return (0.0, 0.0, 0.0), 0.0
    if len(points) == 1:
        return points[0], 0.0
    lengths = [distance(a, b) for a, b in zip(points, points[1:])]
    total = sum(lengths)
    target = max(0.0, min(1.0, fraction)) * total
    walked = 0.0
    for idx, seg_len in enumerate(lengths):
        if walked + seg_len >= target or idx == len(lengths) - 1:
            a = points[idx]
            b = points[idx + 1]
            ratio = 0.0 if seg_len <= 1e-9 else (target - walked) / seg_len
            point = (
                a[0] + (b[0] - a[0]) * ratio,
                a[1] + (b[1] - a[1]) * ratio,
                a[2] + (b[2] - a[2]) * ratio,
            )
            angle = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            if angle > 90.0:
                angle -= 180.0
            elif angle < -90.0:
                angle += 180.0
            return point, angle
        walked += seg_len
    a, b = points[-2], points[-1]
    return points[-1], math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def heading_deg(points: Sequence[Point3]) -> float:
    if len(points) < 2:
        return 0.0
    a, b = points[0], points[-1]
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def normalize_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def classify_maneuver(lanelet_id: int, delta_deg: Optional[float]) -> str:
    if lanelet_id in ROUNDABOUT_HINT_IDS:
        return "roundabout"
    if delta_deg is None:
        return "straight"
    delta = normalize_angle(delta_deg)
    abs_delta = abs(delta)
    if abs_delta >= 135.0:
        return "u_turn"
    if abs_delta >= 40.0:
        return "left_turn" if delta > 0.0 else "right_turn"
    if abs_delta >= 12.0:
        return "left_curve" if delta > 0.0 else "right_curve"
    return "straight"


def section_name(order: int) -> str:
    if order <= 15:
        return "section_01_start_to_200258"
    if order <= 21:
        return "section_02_200261_to_roundabout1"
    if order <= 36:
        return "section_03_roundabout1_to_200183"
    return "section_04_return_to_start"


def point_string(points: Sequence[Point3]) -> str:
    return ";".join(f"{x:.3f}:{y:.3f}:{z:.3f}" for x, y, z in points)


def cap_segments(lanelet: LaneletData) -> List[Tuple[Point3, Point3]]:
    if not lanelet.left or not lanelet.right:
        return []
    starts = [(lanelet.left[0], lanelet.right[0]), (lanelet.left[0], lanelet.right[-1])]
    ends = [(lanelet.left[-1], lanelet.right[-1]), (lanelet.left[-1], lanelet.right[0])]
    start = min(starts, key=lambda pair: distance(pair[0], pair[1]))
    end = min(ends, key=lambda pair: distance(pair[0], pair[1]))
    return [start, end]


def lanelet_bounds(lanelets: Iterable[LaneletData]) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for lanelet in lanelets:
        for point in lanelet.left + lanelet.right + lanelet.center:
            xs.append(point[0])
            ys.append(point[1])
    return min(xs), max(xs), min(ys), max(ys)


def draw_line(ax, points: Sequence[Point3], color, linewidth: float, alpha: float = 1.0, linestyle="-", zorder: int = 1) -> None:
    if len(points) < 2:
        return
    ax.plot(
        [point[0] for point in points],
        [point[1] for point in points],
        color=color,
        linewidth=linewidth,
        alpha=alpha,
        linestyle=linestyle,
        solid_capstyle="butt",
        dash_capstyle="butt",
        zorder=zorder,
    )


def way_linestyle(way: Optional[WayData]) -> str:
    if way is None:
        return "-"
    way_type = way.tags.get("type", "")
    subtype = way.tags.get("subtype", "")
    if way_type == "virtual":
        return (0, (0.4, 1.2))
    if way_type == "line_thin" and subtype == "dashed":
        return (0, (1.2, 1.0))
    if way_type == "line_thin" and subtype == "solid_dashed":
        return (0, (2.0, 0.8, 0.4, 0.8))
    return "-"


def add_label(
    ax,
    used_boxes: List[Tuple[float, float, float, float]],
    lanelet: LaneletData,
    label: str,
    fig_width_px: int,
    data_width_m: float,
    dpi: int,
) -> Dict[str, object]:
    font_size = 0.85
    px_per_m = fig_width_px / data_width_m
    font_px = font_size * dpi / 72.0
    text_width_m = max(2.6, len(label) * font_px * 0.56 / px_per_m)
    text_height_m = max(0.75, font_px * 1.05 / px_per_m)
    candidates = []
    for fraction in (0.50, 0.34, 0.66, 0.20, 0.80, 0.10, 0.90, 0.42, 0.58, 0.28, 0.72, 0.04, 0.96):
        point, angle = interpolate_polyline(lanelet.center, fraction)
        rad = math.radians(angle + 90.0)
        for offset in (0.0, 0.85, -0.85, 1.55, -1.55):
            candidates.append(((point[0] + math.cos(rad) * offset, point[1] + math.sin(rad) * offset, point[2]), angle))

    chosen_point, chosen_angle = candidates[0]
    chosen_box = None
    best_overlap_score = float("inf")
    best_candidate_box = None
    best_candidate = candidates[0]
    for point, angle in candidates:
        theta = math.radians(angle)
        half_x = abs(math.cos(theta)) * text_width_m / 2.0 + abs(math.sin(theta)) * text_height_m / 2.0 + 0.25
        half_y = abs(math.sin(theta)) * text_width_m / 2.0 + abs(math.cos(theta)) * text_height_m / 2.0 + 0.25
        box = (
            point[0] - half_x,
            point[0] + half_x,
            point[1] - half_y,
            point[1] + half_y,
        )
        overlap_score = 0.0
        for old in used_boxes:
            overlap_x = max(0.0, min(box[1], old[1]) - max(box[0], old[0]))
            overlap_y = max(0.0, min(box[3], old[3]) - max(box[2], old[2]))
            overlap_score += overlap_x * overlap_y
        if overlap_score < best_overlap_score:
            best_overlap_score = overlap_score
            best_candidate = (point, angle)
            best_candidate_box = box
        if overlap_score <= 1e-9:
            chosen_point, chosen_angle = point, angle
            chosen_box = box
            break
    if chosen_box is None:
        chosen_point, chosen_angle = best_candidate
        chosen_box = best_candidate_box
    used_boxes.append(chosen_box)
    ax.text(
        chosen_point[0],
        chosen_point[1],
        label,
        fontsize=font_size,
        rotation=chosen_angle,
        rotation_mode="anchor",
        ha="center",
        va="center",
        color="#111111",
        clip_on=True,
        zorder=8,
    )
    return {"label": label, "x": chosen_point[0], "y": chosen_point[1], "angle_deg": chosen_angle, "box": chosen_box}


def write_course_csv(path: Path, lanelets: Dict[int, LaneletData], preferred: Sequence[int], candidates: Sequence[int]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    headings = {lanelet_id: heading_deg(lanelets[lanelet_id].center) for lanelet_id in set(preferred) if lanelet_id in lanelets}
    for idx, lanelet_id in enumerate(preferred, start=1):
        lanelet = lanelets[lanelet_id]
        next_id = preferred[idx] if idx < len(preferred) else None
        delta = normalize_angle(headings[next_id] - headings[lanelet_id]) if next_id in headings else None
        maneuver = classify_maneuver(lanelet_id, delta)
        start = lanelet.center[0] if lanelet.center else (0.0, 0.0, 0.0)
        mid, _ = interpolate_polyline(lanelet.center, 0.5)
        end = lanelet.center[-1] if lanelet.center else (0.0, 0.0, 0.0)
        rows.append(
            {
                "order": idx,
                "lanelet_id": lanelet_id,
                "role": "preferred_course",
                "section": section_name(idx),
                "successor_lanelet_id": next_id or "",
                "speed_limit_kph": "" if lanelet.speed_kph is None else f"{lanelet.speed_kph:.0f}",
                "maneuver": maneuver,
                "maneuver_ko": MANEUVER_KO[maneuver],
                "heading_deg": f"{heading_deg(lanelet.center):.2f}",
                "next_heading_delta_deg": "" if delta is None else f"{delta:.2f}",
                "length_m": f"{lanelet.length_m:.3f}",
                "start_x": f"{start[0]:.3f}",
                "start_y": f"{start[1]:.3f}",
                "mid_x": f"{mid[0]:.3f}",
                "mid_y": f"{mid[1]:.3f}",
                "end_x": f"{end[0]:.3f}",
                "end_y": f"{end[1]:.3f}",
                "centerline_xy": point_string(lanelet.center),
                "note": "closed-loop return to start" if idx == len(preferred) else "v41 preferred route lanelet",
            }
        )

    preferred_unique = set(preferred)
    candidate_order = 1
    for lanelet_id in candidates:
        if lanelet_id in preferred_unique or lanelet_id not in lanelets:
            continue
        lanelet = lanelets[lanelet_id]
        start = lanelet.center[0] if lanelet.center else (0.0, 0.0, 0.0)
        mid, _ = interpolate_polyline(lanelet.center, 0.5)
        end = lanelet.center[-1] if lanelet.center else (0.0, 0.0, 0.0)
        rows.append(
            {
                "order": f"C{candidate_order:02d}",
                "lanelet_id": lanelet_id,
                "role": "candidate_corridor",
                "section": "candidate_post_201086_high_speed_corridor",
                "successor_lanelet_id": "",
                "speed_limit_kph": "" if lanelet.speed_kph is None else f"{lanelet.speed_kph:.0f}",
                "maneuver": "candidate",
                "maneuver_ko": "\ud6c4\ubcf4",
                "heading_deg": f"{heading_deg(lanelet.center):.2f}",
                "next_heading_delta_deg": "",
                "length_m": f"{lanelet.length_m:.3f}",
                "start_x": f"{start[0]:.3f}",
                "start_y": f"{start[1]:.3f}",
                "mid_x": f"{mid[0]:.3f}",
                "mid_y": f"{mid[1]:.3f}",
                "end_x": f"{end[0]:.3f}",
                "end_y": f"{end[1]:.3f}",
                "centerline_xy": point_string(lanelet.center),
                "note": "usable candidate lanelet from the v35 full-course corridor set",
            }
        )
        candidate_order += 1

    fieldnames = [
        "order",
        "lanelet_id",
        "role",
        "section",
        "successor_lanelet_id",
        "speed_limit_kph",
        "maneuver",
        "maneuver_ko",
        "heading_deg",
        "next_heading_delta_deg",
        "length_m",
        "start_x",
        "start_y",
        "mid_x",
        "mid_y",
        "end_x",
        "end_y",
        "centerline_xy",
        "note",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def render_map(
    output_png: Path,
    lanelets: Dict[int, LaneletData],
    ways: Dict[int, WayData],
    preferred: Sequence[int],
    candidates: Sequence[int],
) -> Dict[str, object]:
    xmin, xmax, ymin, ymax = lanelet_bounds(lanelets.values())
    pad = 35.0
    xmin -= pad
    xmax += pad
    ymin -= pad
    ymax += pad
    data_width = xmax - xmin
    data_height = ymax - ymin
    width_in = 10.0
    height_in = width_in * data_height / data_width
    dpi = 620
    fig_width_px = int(width_in * dpi)

    fig, ax = plt.subplots(figsize=(width_in, height_in), dpi=dpi)
    ax.set_facecolor("white")

    for lanelet in lanelets.values():
        for points, way_id in ((lanelet.left, lanelet.left_way_id), (lanelet.right, lanelet.right_way_id)):
            draw_line(
                ax,
                points,
                color="#b8b8b8",
                linewidth=0.075,
                alpha=0.55,
                linestyle=way_linestyle(ways.get(way_id) if way_id is not None else None),
                zorder=1,
            )

    for way in ways.values():
        if way.tags.get("type") != "line_thin":
            continue
        subtype = way.tags.get("subtype", "")
        if subtype == "dashed":
            linestyle = (0, (1.15, 1.05))
            color = "#737373"
        elif subtype == "solid_dashed":
            linestyle = (0, (2.0, 0.8, 0.4, 0.8))
            color = "#606060"
        else:
            linestyle = "-"
            color = "#666666"
        draw_line(ax, way.points, color=color, linewidth=0.085, alpha=0.45, linestyle=linestyle, zorder=2)

    preferred_unique = list(dict.fromkeys(preferred))
    candidate_unique = [lanelet_id for lanelet_id in candidates if lanelet_id not in set(preferred_unique)]
    course_ids = preferred_unique + [lanelet_id for lanelet_id in candidate_unique if lanelet_id in lanelets]

    for lanelet_id in course_ids:
        lanelet = lanelets[lanelet_id]
        boundary_color = "#000000"
        for points, way_id in ((lanelet.left, lanelet.left_way_id), (lanelet.right, lanelet.right_way_id)):
            draw_line(
                ax,
                points,
                color=boundary_color,
                linewidth=0.085,
                alpha=0.98,
                linestyle=way_linestyle(ways.get(way_id) if way_id is not None else None),
                zorder=5,
            )
        for a, b in cap_segments(lanelet):
            draw_line(ax, [a, b], color=boundary_color, linewidth=0.085, alpha=0.98, linestyle="-", zorder=5)

    for lanelet_id in preferred_unique:
        if lanelet_id in lanelets:
            draw_line(ax, lanelets[lanelet_id].center, color="#d7191c", linewidth=0.13, alpha=0.95, linestyle="-", zorder=6)

    used_boxes: List[Tuple[float, float, float, float]] = []
    labels: List[Dict[str, object]] = []
    for lanelet_id in course_ids:
        lanelet = lanelets[lanelet_id]
        speed = "?" if lanelet.speed_kph is None else f"{lanelet.speed_kph:.0f}"
        labels.append(add_label(ax, used_boxes, lanelet, f"{lanelet_id} {speed}", fig_width_px, data_width, dpi))

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.axis("off")
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, bbox_inches="tight", pad_inches=0.02, facecolor="white")
    plt.close(fig)

    image = Image.open(output_png).convert("RGBA")
    pixels = image.getdata()
    red_pixels = 0
    dark_pixels = 0
    nonwhite_pixels = 0
    for r, g, b, a in pixels:
        if a == 0:
            continue
        if (r, g, b) != (255, 255, 255):
            if max(r, g, b) < 250:
                nonwhite_pixels += 1
        if r > 150 and g < 80 and b < 80:
            red_pixels += 1
        if r < 70 and g < 70 and b < 70:
            dark_pixels += 1

    overlap_count = 0
    for idx, label in enumerate(labels):
        box = label["box"]
        for other in labels[idx + 1 :]:
            other_box = other["box"]
            if not (
                box[1] < other_box[0]
                or box[0] > other_box[1]
                or box[3] < other_box[2]
                or box[2] > other_box[3]
            ):
                overlap_count += 1

    return {
        "image": str(output_png),
        "size_px": list(image.size),
        "preferred_lanelet_count": len(preferred_unique),
        "candidate_lanelet_count": len(candidate_unique),
        "course_label_count": len(labels),
        "red_pixel_count": red_pixels,
        "dark_pixel_count": dark_pixels,
        "nonwhite_pixel_count": nonwhite_pixels,
        "estimated_label_box_overlap_count": overlap_count,
        "label_positions": labels,
    }


def write_summary(path: Path, csv_path: Path, png_path: Path, validation: Dict[str, object], preferred: Sequence[int], candidates: Sequence[int]) -> None:
    preferred_unique = list(dict.fromkeys(preferred))
    candidate_unique = [lanelet_id for lanelet_id in candidates if lanelet_id not in set(preferred_unique)]
    summary = {
        "map_version": "v41",
        "map_path": "lanelet_map/lanelet2_map_v41.osm",
        "csv_path": str(csv_path),
        "image_path": str(png_path),
        "preferred_closed_loop_order": preferred,
        "preferred_unique_lanelets": preferred_unique,
        "candidate_lanelets_drawn": candidate_unique,
        "excluded_lanelets": EXCLUDED_LANELETS,
        "validation": validation,
    }
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--map", type=Path, default=Path("lanelet_map/lanelet2_map_v41.osm"))
    parser.add_argument("--projector", type=Path, default=Path("lanelet_map/map_projector_info_v1.yaml"))
    parser.add_argument("--out-dir", type=Path, default=Path("competition_route/v41"))
    args = parser.parse_args()

    root = args.root.expanduser()
    map_path = args.map if args.map.is_absolute() else root / args.map
    projector_path = args.projector if args.projector.is_absolute() else root / args.projector
    out_dir = args.out_dir if args.out_dir.is_absolute() else root / args.out_dir
    csv_path = out_dir / "drive_course_v41_lanelets.csv"
    png_path = out_dir / "drive_course_v41_full_map.png"
    summary_path = out_dir / "drive_course_v41_summary.json"

    projector = LocalMapProjector(projector_path, "+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs")
    lanelets, ways = load_lanelet_osm(map_path, projector)

    missing = [lanelet_id for lanelet_id in set(PREFERRED_ROUTE + CORRIDOR_CANDIDATES) if lanelet_id not in lanelets]
    if missing:
        raise RuntimeError(f"missing lanelets in v41 map: {missing}")

    rows = write_course_csv(csv_path, lanelets, PREFERRED_ROUTE, CORRIDOR_CANDIDATES)
    validation = render_map(png_path, lanelets, ways, PREFERRED_ROUTE, CORRIDOR_CANDIDATES)
    validation["csv_row_count"] = len(rows)
    write_summary(summary_path, csv_path, png_path, validation, PREFERRED_ROUTE, CORRIDOR_CANDIDATES)

    print(f"wrote {csv_path}")
    print(f"wrote {png_path}")
    print(f"wrote {summary_path}")
    print(json.dumps(validation, indent=2)[:2000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
