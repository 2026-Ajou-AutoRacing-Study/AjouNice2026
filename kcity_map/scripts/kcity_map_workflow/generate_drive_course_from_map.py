#!/usr/bin/env python3
"""Generate drive-course CSV plus vector map from any compatible Lanelet OSM.

The course definition comes from a reference CSV with `centerline_xy` points.
Those coordinates are treated as the stable drive-course path; lanelet IDs are
resolved again from the input OSM by nearest centerline geometry.
"""

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

MANEUVER_KO = {
    "straight": "\uc9c1\uc9c4",
    "left_curve": "\uc88c\ucee4\ube0c",
    "right_curve": "\uc6b0\ucee4\ube0c",
    "left_turn": "\uc88c\ud68c\uc804",
    "right_turn": "\uc6b0\ud68c\uc804",
    "u_turn": "\uc720\ud134",
    "roundabout": "\ud68c\uc804\uad50\ucc28\ub85c",
    "candidate": "\ud6c4\ubcf4",
}


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
    def vehicle_allowed(self) -> bool:
        return self.tags.get("participant:vehicle", "yes") != "no"

    @property
    def length_m(self) -> float:
        return polyline_length(self.center)


@dataclass
class CourseReference:
    source_order: str
    source_lanelet_id: int
    role: str
    section: str
    maneuver: str
    maneuver_ko: str
    points: List[Point3]


@dataclass
class MatchResult:
    reference: CourseReference
    lanelet: LaneletData
    mean_distance_m: float
    max_distance_m: float
    progress_m: float
    match_method: str


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


def as_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else root / path


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


def point_string(points: Sequence[Point3]) -> str:
    return ";".join(f"{x:.3f}:{y:.3f}:{z:.3f}" for x, y, z in points)


def parse_point_string(value: str) -> List[Point3]:
    points: List[Point3] = []
    for chunk in (value or "").split(";"):
        if not chunk.strip():
            continue
        pieces = chunk.split(":")
        if len(pieces) < 2:
            continue
        x = float(pieces[0])
        y = float(pieces[1])
        z = float(pieces[2]) if len(pieces) > 2 else 0.0
        points.append((x, y, z))
    return points


def load_reference_csv(path: Path) -> List[CourseReference]:
    refs: List[CourseReference] = []
    with path.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            role = row.get("role", "")
            if role not in ("preferred_course", "candidate_corridor"):
                continue
            points = parse_point_string(row.get("centerline_xy", ""))
            if not points:
                points = [
                    (float(row["start_x"]), float(row["start_y"]), 0.0),
                    (float(row["mid_x"]), float(row["mid_y"]), 0.0),
                    (float(row["end_x"]), float(row["end_y"]), 0.0),
                ]
            refs.append(
                CourseReference(
                    source_order=str(row.get("order", "")),
                    source_lanelet_id=int(row["lanelet_id"]),
                    role=role,
                    section=row.get("section", ""),
                    maneuver=row.get("maneuver", ""),
                    maneuver_ko=row.get("maneuver_ko", ""),
                    points=points,
                )
            )
    return refs


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


def sample_polyline(points: Sequence[Point3], count: int) -> List[Point3]:
    if not points:
        return []
    if len(points) <= count:
        return list(points)
    return [interpolate_polyline(points, idx / max(1, count - 1))[0] for idx in range(count)]


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


def classify_maneuver(delta_deg: Optional[float], source_maneuver: str = "") -> str:
    if source_maneuver == "roundabout":
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


def dist2_point_to_segment(point: Point3, a: Point3, b: Point3) -> Tuple[float, float]:
    px, py, _ = point
    ax, ay, _ = a
    bx, by, _ = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return (px - ax) ** 2 + (py - ay) ** 2, 0.0
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    cx = ax + ratio * dx
    cy = ay + ratio * dy
    return (px - cx) ** 2 + (py - cy) ** 2, ratio


def project_point_to_polyline(point: Point3, polyline: Sequence[Point3]) -> Tuple[float, float]:
    if not polyline:
        return float("inf"), 0.0
    if len(polyline) == 1:
        return distance(point, polyline[0]), 0.0
    best_dist2 = float("inf")
    best_progress = 0.0
    walked = 0.0
    for a, b in zip(polyline, polyline[1:]):
        seg_len = distance(a, b)
        dist2, ratio = dist2_point_to_segment(point, a, b)
        if dist2 < best_dist2:
            best_dist2 = dist2
            best_progress = walked + ratio * seg_len
        walked += seg_len
    return math.sqrt(best_dist2), best_progress


def lanelet_reference_score(lanelet: LaneletData, reference: CourseReference) -> Tuple[float, float, float]:
    samples = sample_polyline(lanelet.center, 9)
    if not samples:
        return float("inf"), float("inf"), 0.0
    distances = []
    progresses = []
    for point in samples:
        dist, progress = project_point_to_polyline(point, reference.points)
        distances.append(dist)
        progresses.append(progress)
    return sum(distances) / len(distances), max(distances), sum(progresses) / len(progresses)


def nearest_reference_score(lanelet: LaneletData, reference: CourseReference) -> Tuple[float, float, float]:
    samples = sample_polyline(reference.points, 11)
    distances = []
    progresses = []
    for point in samples:
        dist, progress = project_point_to_polyline(point, lanelet.center)
        distances.append(dist)
        progresses.append(progress)
    return sum(distances) / len(distances), max(distances), sum(progresses) / len(progresses)


def resolve_reference(
    reference: CourseReference,
    lanelets: Dict[int, LaneletData],
    match_threshold_m: float,
    max_distance_m: float,
    prefer_source_id: bool,
) -> List[MatchResult]:
    road_lanelets = [lanelet for lanelet in lanelets.values() if lanelet.center and lanelet.vehicle_allowed]

    if prefer_source_id and reference.source_lanelet_id in lanelets:
        lanelet = lanelets[reference.source_lanelet_id]
        if lanelet.center and lanelet.vehicle_allowed:
            mean_dist, max_dist, progress = lanelet_reference_score(lanelet, reference)
            if mean_dist <= match_threshold_m and max_dist <= max_distance_m:
                return [MatchResult(reference, lanelet, mean_dist, max_dist, progress, "source_id_exact")]

    matches: List[MatchResult] = []
    for lanelet in road_lanelets:
        mean_dist, max_dist, progress = lanelet_reference_score(lanelet, reference)
        if mean_dist <= match_threshold_m and max_dist <= max_distance_m:
            matches.append(MatchResult(reference, lanelet, mean_dist, max_dist, progress, "centerline_overlap"))

    if matches:
        return sorted(matches, key=lambda item: (item.progress_m, item.mean_distance_m, item.lanelet.lanelet_id))

    fallback: Optional[MatchResult] = None
    for lanelet in road_lanelets:
        mean_dist, max_dist, progress = nearest_reference_score(lanelet, reference)
        candidate = MatchResult(reference, lanelet, mean_dist, max_dist, progress, "nearest_fallback")
        if fallback is None or (candidate.mean_distance_m, candidate.max_distance_m) < (
            fallback.mean_distance_m,
            fallback.max_distance_m,
        ):
            fallback = candidate
    return [fallback] if fallback is not None else []


def resolve_course(
    references: Sequence[CourseReference],
    lanelets: Dict[int, LaneletData],
    match_threshold_m: float,
    max_distance_m: float,
    prefer_source_id: bool,
) -> List[MatchResult]:
    resolved: List[MatchResult] = []
    preferred_ids = set()
    for reference in references:
        matches = resolve_reference(reference, lanelets, match_threshold_m, max_distance_m, prefer_source_id)
        if reference.role == "preferred_course":
            for match in matches:
                if resolved and resolved[-1].lanelet.lanelet_id == match.lanelet.lanelet_id:
                    continue
                resolved.append(match)
                preferred_ids.add(match.lanelet.lanelet_id)

    candidate_seen = set()
    for reference in references:
        if reference.role != "candidate_corridor":
            continue
        matches = resolve_reference(reference, lanelets, match_threshold_m, max_distance_m, prefer_source_id)
        for match in matches:
            lanelet_id = match.lanelet.lanelet_id
            if lanelet_id in preferred_ids or lanelet_id in candidate_seen:
                continue
            resolved.append(match)
            candidate_seen.add(lanelet_id)
    return resolved


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


def cap_segments(lanelet: LaneletData) -> List[Tuple[Point3, Point3]]:
    if not lanelet.left or not lanelet.right:
        return []
    starts = [(lanelet.left[0], lanelet.right[0]), (lanelet.left[0], lanelet.right[-1])]
    ends = [(lanelet.left[-1], lanelet.right[-1]), (lanelet.left[-1], lanelet.right[0])]
    return [min(starts, key=lambda pair: distance(pair[0], pair[1])), min(ends, key=lambda pair: distance(pair[0], pair[1]))]


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

    best_point, best_angle = candidates[0]
    best_box = None
    best_overlap = float("inf")
    for point, angle in candidates:
        theta = math.radians(angle)
        half_x = abs(math.cos(theta)) * text_width_m / 2.0 + abs(math.sin(theta)) * text_height_m / 2.0 + 0.25
        half_y = abs(math.sin(theta)) * text_width_m / 2.0 + abs(math.cos(theta)) * text_height_m / 2.0 + 0.25
        box = (point[0] - half_x, point[0] + half_x, point[1] - half_y, point[1] + half_y)
        overlap = 0.0
        for old in used_boxes:
            overlap_x = max(0.0, min(box[1], old[1]) - max(box[0], old[0]))
            overlap_y = max(0.0, min(box[3], old[3]) - max(box[2], old[2]))
            overlap += overlap_x * overlap_y
        if overlap < best_overlap:
            best_overlap = overlap
            best_point, best_angle, best_box = point, angle, box
        if overlap <= 1e-9:
            break
    used_boxes.append(best_box)
    ax.text(
        best_point[0],
        best_point[1],
        label,
        fontsize=font_size,
        rotation=best_angle,
        rotation_mode="anchor",
        ha="center",
        va="center",
        color="#111111",
        clip_on=True,
        zorder=8,
    )
    return {"label": label, "x": best_point[0], "y": best_point[1], "angle_deg": best_angle, "box": best_box}


def label_overlap_count(labels: Sequence[Dict[str, object]]) -> int:
    count = 0
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
                count += 1
    return count


def write_course_csv(path: Path, matches: Sequence[MatchResult]) -> List[Dict[str, object]]:
    preferred = [match for match in matches if match.reference.role == "preferred_course"]
    candidates = [match for match in matches if match.reference.role == "candidate_corridor"]
    headings = [heading_deg(match.lanelet.center) for match in preferred]
    rows: List[Dict[str, object]] = []

    for idx, match in enumerate(preferred, start=1):
        lanelet = match.lanelet
        next_match = preferred[idx] if idx < len(preferred) else None
        delta = normalize_angle(headings[idx] - headings[idx - 1]) if idx < len(preferred) else None
        maneuver = classify_maneuver(delta, match.reference.maneuver)
        start = lanelet.center[0] if lanelet.center else (0.0, 0.0, 0.0)
        mid, _ = interpolate_polyline(lanelet.center, 0.5)
        end = lanelet.center[-1] if lanelet.center else (0.0, 0.0, 0.0)
        rows.append(
            {
                "order": idx,
                "lanelet_id": lanelet.lanelet_id,
                "role": "preferred_course",
                "section": match.reference.section,
                "successor_lanelet_id": next_match.lanelet.lanelet_id if next_match else "",
                "speed_limit_kph": "" if lanelet.speed_kph is None else f"{lanelet.speed_kph:.0f}",
                "maneuver": maneuver,
                "maneuver_ko": MANEUVER_KO.get(maneuver, maneuver),
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
                "source_order": match.reference.source_order,
                "source_lanelet_id": match.reference.source_lanelet_id,
                "match_mean_distance_m": f"{match.mean_distance_m:.3f}",
                "match_max_distance_m": f"{match.max_distance_m:.3f}",
                "match_method": match.match_method,
                "note": "coordinate-resolved preferred drive-course lanelet",
            }
        )

    for idx, match in enumerate(candidates, start=1):
        lanelet = match.lanelet
        start = lanelet.center[0] if lanelet.center else (0.0, 0.0, 0.0)
        mid, _ = interpolate_polyline(lanelet.center, 0.5)
        end = lanelet.center[-1] if lanelet.center else (0.0, 0.0, 0.0)
        rows.append(
            {
                "order": f"C{idx:02d}",
                "lanelet_id": lanelet.lanelet_id,
                "role": "candidate_corridor",
                "section": match.reference.section,
                "successor_lanelet_id": "",
                "speed_limit_kph": "" if lanelet.speed_kph is None else f"{lanelet.speed_kph:.0f}",
                "maneuver": "candidate",
                "maneuver_ko": MANEUVER_KO["candidate"],
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
                "source_order": match.reference.source_order,
                "source_lanelet_id": match.reference.source_lanelet_id,
                "match_mean_distance_m": f"{match.mean_distance_m:.3f}",
                "match_max_distance_m": f"{match.max_distance_m:.3f}",
                "match_method": match.match_method,
                "note": "coordinate-resolved candidate corridor lanelet",
            }
        )

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
        "source_order",
        "source_lanelet_id",
        "match_mean_distance_m",
        "match_max_distance_m",
        "match_method",
        "note",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def write_match_audit(path: Path, matches: Sequence[MatchResult]) -> None:
    fieldnames = [
        "source_order",
        "source_lanelet_id",
        "role",
        "resolved_lanelet_id",
        "match_mean_distance_m",
        "match_max_distance_m",
        "match_progress_m",
        "match_method",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for match in matches:
            writer.writerow(
                {
                    "source_order": match.reference.source_order,
                    "source_lanelet_id": match.reference.source_lanelet_id,
                    "role": match.reference.role,
                    "resolved_lanelet_id": match.lanelet.lanelet_id,
                    "match_mean_distance_m": f"{match.mean_distance_m:.3f}",
                    "match_max_distance_m": f"{match.max_distance_m:.3f}",
                    "match_progress_m": f"{match.progress_m:.3f}",
                    "match_method": match.match_method,
                }
            )


def render_map(
    output_prefix: Path,
    lanelets: Dict[int, LaneletData],
    ways: Dict[int, WayData],
    matches: Sequence[MatchResult],
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

    preferred_ids = list(dict.fromkeys(match.lanelet.lanelet_id for match in matches if match.reference.role == "preferred_course"))
    candidate_ids = [
        lanelet_id
        for lanelet_id in dict.fromkeys(match.lanelet.lanelet_id for match in matches if match.reference.role == "candidate_corridor")
        if lanelet_id not in set(preferred_ids)
    ]
    course_ids = preferred_ids + candidate_ids

    for lanelet_id in course_ids:
        lanelet = lanelets[lanelet_id]
        for points, way_id in ((lanelet.left, lanelet.left_way_id), (lanelet.right, lanelet.right_way_id)):
            draw_line(
                ax,
                points,
                color="#000000",
                linewidth=0.085,
                alpha=0.98,
                linestyle=way_linestyle(ways.get(way_id) if way_id is not None else None),
                zorder=5,
            )
        for a, b in cap_segments(lanelet):
            draw_line(ax, [a, b], color="#000000", linewidth=0.085, alpha=0.98, linestyle="-", zorder=5)

    for lanelet_id in preferred_ids:
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
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    png_path = output_prefix.with_suffix(".png")
    svg_path = output_prefix.with_suffix(".svg")
    pdf_path = output_prefix.with_suffix(".pdf")
    fig.savefig(svg_path, bbox_inches="tight", pad_inches=0.02, facecolor="white")
    fig.savefig(pdf_path, bbox_inches="tight", pad_inches=0.02, facecolor="white")
    fig.savefig(png_path, bbox_inches="tight", pad_inches=0.02, facecolor="white", dpi=dpi)
    plt.close(fig)

    image = Image.open(png_path).convert("RGBA")
    red_pixels = 0
    dark_pixels = 0
    nonwhite_pixels = 0
    for r, g, b, a in image.getdata():
        if a == 0:
            continue
        if max(r, g, b) < 250:
            nonwhite_pixels += 1
        if r > 150 and g < 80 and b < 80:
            red_pixels += 1
        if r < 70 and g < 70 and b < 70:
            dark_pixels += 1

    return {
        "png_path": str(png_path),
        "svg_path": str(svg_path),
        "pdf_path": str(pdf_path),
        "png_size_px": list(image.size),
        "preferred_lanelet_count": len(preferred_ids),
        "candidate_lanelet_count": len(candidate_ids),
        "course_label_count": len(labels),
        "red_pixel_count": red_pixels,
        "dark_pixel_count": dark_pixels,
        "nonwhite_pixel_count": nonwhite_pixels,
        "estimated_label_box_overlap_count": label_overlap_count(labels),
    }


def endpoint_gaps(rows: Sequence[Dict[str, object]]) -> List[float]:
    preferred = [row for row in rows if row["role"] == "preferred_course"]

    def point(row, prefix: str) -> Tuple[float, float]:
        return float(row[f"{prefix}_x"]), float(row[f"{prefix}_y"])

    gaps = []
    for a, b in zip(preferred, preferred[1:]):
        ax, ay = point(a, "end")
        bx, by = point(b, "start")
        gaps.append(math.hypot(ax - bx, ay - by))
    return gaps


def write_summary(
    path: Path,
    map_path: Path,
    reference_csv: Path,
    csv_path: Path,
    audit_path: Path,
    render_validation: Dict[str, object],
    rows: Sequence[Dict[str, object]],
    matches: Sequence[MatchResult],
    match_threshold_m: float,
    max_distance_m: float,
) -> None:
    gaps = endpoint_gaps(rows)
    preferred = [match for match in matches if match.reference.role == "preferred_course"]
    candidates = [match for match in matches if match.reference.role == "candidate_corridor"]
    match_distances = [match.mean_distance_m for match in matches]
    summary = {
        "map_path": str(map_path),
        "reference_csv": str(reference_csv),
        "csv_path": str(csv_path),
        "match_audit_csv": str(audit_path),
        "match_threshold_m": match_threshold_m,
        "max_distance_m": max_distance_m,
        "preferred_rows": len([row for row in rows if row["role"] == "preferred_course"]),
        "candidate_rows": len([row for row in rows if row["role"] == "candidate_corridor"]),
        "preferred_unique_lanelets": len(set(match.lanelet.lanelet_id for match in preferred)),
        "candidate_unique_lanelets": len(set(match.lanelet.lanelet_id for match in candidates)),
        "max_preferred_endpoint_gap_m": max(gaps) if gaps else 0.0,
        "max_match_mean_distance_m": max(match_distances) if match_distances else None,
        "render_validation": render_validation,
    }
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--map", type=str, required=True, help="Lanelet2 OSM path")
    parser.add_argument("--projector", type=str, default="lanelet_map/map_projector_info_v1.yaml")
    parser.add_argument("--reference-csv", type=str, required=True)
    parser.add_argument("--out-dir", type=str, required=True)
    parser.add_argument("--output-prefix", type=str, default="drive_course")
    parser.add_argument("--utm-projection", type=str, default="+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs")
    parser.add_argument("--match-threshold-m", type=float, default=2.4)
    parser.add_argument("--max-distance-m", type=float, default=4.2)
    parser.add_argument(
        "--no-prefer-source-id",
        action="store_true",
        help="Ignore reference lanelet IDs and resolve only by geometry.",
    )
    args = parser.parse_args()

    root = args.root.expanduser()
    map_path = as_path(root, args.map)
    projector_path = as_path(root, args.projector)
    reference_csv = as_path(root, args.reference_csv)
    out_dir = as_path(root, args.out_dir)
    output_prefix = out_dir / args.output_prefix
    csv_path = output_prefix.with_name(f"{args.output_prefix}_lanelets.csv")
    audit_path = output_prefix.with_name(f"{args.output_prefix}_match_audit.csv")
    summary_path = output_prefix.with_name(f"{args.output_prefix}_summary.json")
    image_prefix = output_prefix.with_name(f"{args.output_prefix}_full_map")

    projector = LocalMapProjector(projector_path, args.utm_projection)
    lanelets, ways = load_lanelet_osm(map_path, projector)
    references = load_reference_csv(reference_csv)
    matches = resolve_course(
        references,
        lanelets,
        args.match_threshold_m,
        args.max_distance_m,
        not args.no_prefer_source_id,
    )

    rows = write_course_csv(csv_path, matches)
    write_match_audit(audit_path, matches)
    render_validation = render_map(image_prefix, lanelets, ways, matches)
    write_summary(
        summary_path,
        map_path,
        reference_csv,
        csv_path,
        audit_path,
        render_validation,
        rows,
        matches,
        args.match_threshold_m,
        args.max_distance_m,
    )

    print(f"wrote {csv_path}")
    print(f"wrote {audit_path}")
    print(f"wrote {image_prefix.with_suffix('.svg')}")
    print(f"wrote {image_prefix.with_suffix('.pdf')}")
    print(f"wrote {image_prefix.with_suffix('.png')}")
    print(f"wrote {summary_path}")
    print(json.dumps(render_validation, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
