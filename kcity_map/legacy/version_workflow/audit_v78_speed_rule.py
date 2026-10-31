#!/usr/bin/env python3
"""Audit v78 speed limits against the competition highway exception bounds."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import lanelet2
from pyproj import Transformer


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "lanelet_map/lanelet2_map_v78.osm"
REPORT = ROOT / "docs/kcity_v78_speed_rule_audit.json"
START_LINK = "A2256W000411"
END_LINK = "A2256W000153"
BUFFER_M = 1.0
MAP_ORIGIN_LAT = 37.2380038082
MAP_ORIGIN_LON = 126.7730819386


def unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    norm = math.hypot(dx, dy)
    if norm <= 1.0e-9:
        raise RuntimeError("MGeo endpoint tangent is degenerate")
    return dx / norm, dy / norm


def main():
    global_info = json.loads((ROOT / "mgeo_map/global_info.json").read_text())
    mgeo_origin = global_info["local_origin_in_global"]
    links = {
        item["idx"]: item
        for item in json.loads((ROOT / "mgeo_map/link_set.json").read_text())
    }

    transform = Transformer.from_crs(4326, 32652, always_xy=True)
    origin_x, origin_y = transform.transform(MAP_ORIGIN_LON, MAP_ORIGIN_LAT)

    def map_xy(point):
        return (
            point[0] + mgeo_origin[0] - origin_x,
            point[1] + mgeo_origin[1] - origin_y,
        )

    start_points = links[START_LINK]["points"]
    end_points = links[END_LINK]["points"]
    start_mgeo = start_points[0][:2]
    end_mgeo = end_points[-1][:2]
    start = map_xy(start_points[0])
    end = map_xy(end_points[-1])
    start_tangent = unit(map_xy(start_points[0]), map_xy(start_points[1]))
    end_tangent = unit(map_xy(end_points[-2]), map_xy(end_points[-1]))

    safe_start_map = [
        start[0] + BUFFER_M * start_tangent[0],
        start[1] + BUFFER_M * start_tangent[1],
    ]
    safe_end_map = [
        end[0] - BUFFER_M * end_tangent[0],
        end[1] - BUFFER_M * end_tangent[1],
    ]
    safe_start_mgeo = [
        start_mgeo[0] + BUFFER_M * start_tangent[0],
        start_mgeo[1] + BUFFER_M * start_tangent[1],
    ]
    safe_end_mgeo = [
        end_mgeo[0] - BUFFER_M * end_tangent[0],
        end_mgeo[1] - BUFFER_M * end_tangent[1],
    ]

    projector = lanelet2.projection.UtmProjector(
        lanelet2.io.Origin(MAP_ORIGIN_LAT, MAP_ORIGIN_LON)
    )
    lanelet_map = lanelet2.io.load(str(MAP), projector)

    speed_counts = {}
    missing_speed = []
    over_60 = []
    violations = []
    for lanelet in lanelet_map.laneletLayer:
        attrs = dict(lanelet.attributes)
        if attrs.get("subtype") != "road":
            continue
        speed_text = attrs.get("speed_limit")
        speed_counts[speed_text] = speed_counts.get(speed_text, 0) + 1
        if speed_text is None:
            missing_speed.append(lanelet.id)
            continue
        speed = float(speed_text)
        if speed <= 60.0:
            continue

        # Check the complete lanelet geometry. This is stricter than checking
        # only the centerline followed by the ego vehicle.
        points = list(lanelet.leftBound) + list(lanelet.rightBound) + list(lanelet.centerline)
        start_margins = [
            (point.x - start[0]) * start_tangent[0]
            + (point.y - start[1]) * start_tangent[1]
            for point in points
        ]
        end_margins = [
            (end[0] - point.x) * end_tangent[0]
            + (end[1] - point.y) * end_tangent[1]
            for point in points
        ]
        row = {
            "lanelet": lanelet.id,
            "speed_limit_kph": speed,
            "distance_after_exception_start_m": min(start_margins),
            "distance_before_exception_end_m": min(end_margins),
        }
        over_60.append(row)
        if min(start_margins) + 1.0e-4 < BUFFER_M or min(end_margins) + 1.0e-4 < BUFFER_M:
            violations.append(row)

    if missing_speed:
        raise RuntimeError(f"road lanelets without speed_limit: {missing_speed}")

    report = {
        "map": str(MAP.relative_to(ROOT)),
        "map_sha256": hashlib.sha256(MAP.read_bytes()).hexdigest(),
        "rule": {
            "normal_limit_kph": 60,
            "exception_start_link": START_LINK,
            "exception_end_link": END_LINK,
            "conservative_buffer_m": BUFFER_M,
        },
        "mgeo_coordinates": {
            "exception_start": start_mgeo,
            "safe_start": safe_start_mgeo,
            "safe_end": safe_end_mgeo,
            "exception_end": end_mgeo,
        },
        "map_coordinates": {
            "exception_start": list(start),
            "safe_start": safe_start_map,
            "safe_end": safe_end_map,
            "exception_end": list(end),
        },
        "road_speed_limit_counts": speed_counts,
        "over_60_lanelet_count": len(over_60),
        "minimum_start_margin_m": min(
            item["distance_after_exception_start_m"] for item in over_60
        ),
        "minimum_end_margin_m": min(
            item["distance_before_exception_end_m"] for item in over_60
        ),
        "over_60_lanelets": sorted(over_60, key=lambda item: item["lanelet"]),
        "violations": violations,
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "over_60_lanelets"}, indent=2))
    if violations:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
