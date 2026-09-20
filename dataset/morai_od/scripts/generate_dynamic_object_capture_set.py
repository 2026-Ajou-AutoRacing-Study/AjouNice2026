#!/usr/bin/env python3
"""Generate mission-conditioned MORAI dynamic-object capture scenarios.

The official competition sample is used as the serialization template.  This
is intentional: MORAI pedestrian waypoints and NPC spawn points contain
launcher-specific fields that are safer to clone than to recreate from an
unverified schema.
"""

import argparse
import bisect
import copy
import hashlib
import heapq
import json
import math
import random
import shutil
from collections import Counter, defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG = (
    PROJECT_ROOT
    / "dataset/morai_od/configs/dynamic_object_capture_v1.json"
)
DEFAULT_TEMPLATE = (
    PROJECT_ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
)
DEFAULT_ROUTE = (
    PROJECT_ROOT / "configs/global_path/2026_molit_comp_global_path.txt"
)
DEFAULT_MGEO = PROJECT_ROOT / "kcity_map/mgeo_map_scenario_runner_2025"
DEFAULT_OUTPUT = PROJECT_ROOT / "dataset/morai_od"
DEFAULT_ASSET_CATALOG = (
    PROJECT_ROOT / "dataset/morai_od/configs/morai_asset_catalog_personal_260807.json"
)
DEFAULT_ASSET_COMPATIBILITY = (
    PROJECT_ROOT
    / "dataset/morai_od/configs/morai_asset_launcher_compatibility_260807.json"
)
DEFAULT_RUNTIME_ASSET_DIR = Path.home() / (
    "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/"
    "Scenario/R_KR_PR_K-city_2025"
)


FAMILY_CODES = {
    "roundabout_sample_spawn": "rs",
    "roundabout_circulation": "rc",
    "highway_merge_sample_spawn": "hm",
    "highway_merge_cruise": "mc",
    "highway_cruise_overtake": "hc",
    "highway_lane_change_overtake": "lc",
    "east_corridor_relative_vehicle": "ecv",
    "crosswalk_pedestrian": "pd",
    "static_obstacle": "so",
    "gps_shadow_obstacle": "go",
    "gps_shadow_vehicle_interaction": "gv",
    "stopped_vehicle": "sv",
    "standing_pedestrian": "sp",
    "realistic_mixed_static": "mx",
    "sparse_background": "bg",
    "static_highway_obstacle": "sh",
}

ROUTE_WIDE_STATIC_FAMILIES = {
    "static_obstacle",
    "stopped_vehicle",
    "standing_pedestrian",
    "realistic_mixed_static",
    "sparse_background",
}


SIMPLE_SPAWN_VEHICLE_FAMILIES = {
    "roundabout_sample_spawn",
    "roundabout_circulation",
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
    "gps_shadow_vehicle_interaction",
}

SINGLE_LANE_SPAWN_FAMILIES = {
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
    "gps_shadow_vehicle_interaction",
}


# Three parallel high-speed lanes and their smooth MGeo continuations. The
# spawn IDs are the official 40/50/60 km/h lanes from the competition sample.
HIGHWAY_LANE_LINK_PATHS = {
    6: (
        "A2256W000406",
        "A2256W000409",
        "A2256W000420",
        "A2256W000408",
        "A2256W000445",
    ),
    8: (
        "A2256W000175",
        "A2256W000410",
        "A2256W000430",
        "A2256W000434",
        "A2256W000422",
    ),
    10: (
        "A2256W000029",
        "A2256W000418",
        "A2256W000431",
        "A2256W000435",
        "A2256W000423",
    ),
}


# Base3-Static must model the competition highway by topology rather than by
# one persistent lane number.  MGeo calls both the entering ramp A411 and the
# departing branch A179 ``ego_lane=5``, even though they have opposite roles.
# The left-most links are intentionally absent: the competition sample never
# uses that solid-separated lane for Ego or NPC traffic.
HIGHWAY_RESTRICTED_LINK_IDS = frozenset({
    "A2256W000010", "A2256W000429", "A2256W000433",
    "A2256W000421", "A2256W000425", "A2256W000424",
})

HIGHWAY_STATIC_PHASE_VARIANTS = {
    "highway_ramp_approach": (
        {
            "name": "ramp_approach",
            "weight": 1.0,
            "route": ("A2256W000846", "A2256W000411", "A2256W000420"),
            "target_link": "A2256W000846",
            "side_routes": (),
        },
    ),
    "highway_merge_lane": (
        {
            "name": "merge_lane_ego",
            "weight": 0.70,
            "route": ("A2256W000411", "A2256W000420"),
            "target_link": "A2256W000411",
            "side_routes": (("A2256W000409", "A2256W000420"),),
        },
        {
            "name": "merge_mainline_ego",
            "weight": 0.30,
            "route": ("A2256W000409", "A2256W000420"),
            "target_link": "A2256W000409",
            "side_routes": (("A2256W000411", "A2256W000420"),),
        },
    ),
    "highway_exit_split": (
        {
            "name": "exit_mainline_lane2",
            "weight": 0.20,
            "route": ("A2256W000435", "A2256W000423", "A2256W000432"),
            "target_link": "A2256W000435",
            "side_routes": (("A2256W000434", "A2256W000422", "A2256W000153"),),
        },
        {
            "name": "exit_mainline_lane3",
            "weight": 0.30,
            "route": ("A2256W000434", "A2256W000422", "A2256W000153"),
            "target_link": "A2256W000434",
            "side_routes": (
                ("A2256W000435", "A2256W000423", "A2256W000432"),
                ("A2256W000408", "A2256W000445", "A2256W000153"),
            ),
        },
        {
            "name": "exit_mainline_lane4",
            "weight": 0.45,
            "route": ("A2256W000408", "A2256W000445", "A2256W000153"),
            "target_link": "A2256W000408",
            "side_routes": (
                ("A2256W000434", "A2256W000422", "A2256W000153"),
                ("A2256W000179", "A2256W001105"),
            ),
        },
        {
            "name": "exit_branch_ego",
            "weight": 0.05,
            "route": ("A2256W000179", "A2256W001105"),
            "target_link": "A2256W000179",
            "side_routes": (("A2256W000408", "A2256W000445", "A2256W000153"),),
        },
    ),
    "highway_four_to_three_taper": (
        {
            "name": "taper_lane2",
            "weight": 0.20,
            "route": ("A2256W000423", "A2256W000432"),
            "target_link": "A2256W000423",
            "side_routes": (("A2256W000422", "A2256W000153"),),
        },
        {
            "name": "taper_lane3",
            "weight": 0.30,
            "route": ("A2256W000422", "A2256W000153"),
            "target_link": "A2256W000422",
            "side_routes": (
                ("A2256W000423", "A2256W000432"),
                ("A2256W000445", "A2256W000153"),
            ),
        },
        {
            "name": "taper_lane4",
            "weight": 0.50,
            "route": ("A2256W000445", "A2256W000153"),
            "target_link": "A2256W000445",
            "side_routes": (("A2256W000422", "A2256W000153"),),
        },
    ),
    "highway_post_taper_two_usable": (
        {
            "name": "post_taper_left",
            "weight": 0.40,
            "route": ("A2256W000432", "A2256W000427", "A2256W000444"),
            "target_link": "A2256W000432",
            "side_routes": (("A2256W000153", "A2256W000451", "A2256W000446"),),
        },
        {
            "name": "post_taper_right",
            "weight": 0.60,
            "route": ("A2256W000153", "A2256W000451", "A2256W000446"),
            "target_link": "A2256W000153",
            "side_routes": (("A2256W000432", "A2256W000427", "A2256W000444"),),
        },
    ),
}


# Five topology-complete, same-direction two-lane portions intersect the
# competition route.  Both sides carry an execution path so moving Ego can be
# captured on the right lane without steering back to the canonical left
# route.  These are ordinary-road links; the highway A408/A179 pair is kept in
# the phase model above.
ROUTE_GENERAL_TWO_LANE_VARIANTS = (
    {
        "name": "two_lane_202_009",
        "left_route": ("A2256W000220", "A2256W000202", "A2256W000728"),
        "left_target_link": "A2256W000202",
        "right_route": ("A2256W000220", "A2256W000009", "A2256W000201"),
        "right_target_link": "A2256W000009",
    },
    {
        "name": "two_lane_205_314",
        "left_route": ("A2256W000207", "A2256W000205", "A2256W000315"),
        "left_target_link": "A2256W000205",
        "right_route": ("A2256W000208", "A2256W000314", "A2256W000310"),
        "right_target_link": "A2256W000314",
    },
    {
        "name": "two_lane_315_310",
        "left_route": ("A2256W000205", "A2256W000315", "A2256W000318"),
        "left_target_link": "A2256W000315",
        "right_route": ("A2256W000314", "A2256W000310", "A2256W000319"),
        "right_target_link": "A2256W000310",
    },
    {
        "name": "two_lane_308_309",
        "left_route": ("A2256W000318", "A2256W000308", "A2256W000297"),
        "left_target_link": "A2256W000308",
        "right_route": ("A2256W000319", "A2256W000309", "A2256W000299"),
        "right_target_link": "A2256W000309",
    },
    {
        "name": "two_lane_304_307",
        "left_route": ("A2256W000300", "A2256W000304", "A2256W000148"),
        "left_target_link": "A2256W000304",
        "right_route": ("A2256W000303", "A2256W000307", "A2256W000739"),
        "right_target_link": "A2256W000307",
    },
)


# Ordered MGeo path used by the official roundabout NPC spawn. Temporal V2
# moves the start point along this same legal path so spatially distributed
# Ego capture anchors do not lose the actor.
ROUNDABOUT_NPC_LINK_PATH = (
    "A2256W000144",
    "A2256W000133",
    "A2256W000134",
    "A2256W000135",
    "A2256W000102",
    # Keep a cruising actor alive after it leaves the roundabout.  Ending at
    # A2256W000102 left only a few seconds of route for late-circulation
    # spawns, so MORAI deleted the actor before frame zero in some clips.
    "A2256W000848",
    "A2256W000845",
    "A2256W000843",
)


# The competition route traverses the GPS-shadow tunnel on these two
# consecutive links.  A vehicle spawned here stays on the legal route instead
# of borrowing a roundabout/highway spawn whose destination is unrelated.
GPS_SHADOW_NPC_LINK_PATH = (
    "A2256W000126",
    "A2256W000128",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--mgeo-dir", type=Path, default=DEFAULT_MGEO)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--asset-catalog", type=Path, default=DEFAULT_ASSET_CATALOG)
    parser.add_argument(
        "--asset-compatibility",
        type=Path,
        default=DEFAULT_ASSET_COMPATIBILITY,
    )
    parser.add_argument(
        "--runtime-asset-scenario-dir",
        type=Path,
        default=DEFAULT_RUNTIME_ASSET_DIR,
    )
    parser.add_argument("--group", default="dynamic_object_v1_smoke")
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--id-prefix",
        default="",
        help="Prefix every public scene id so independently generated batches cannot collide.",
    )
    parser.add_argument(
        "--stem-prefix",
        default="",
        help="Prefix every MORAI scenario stem so installed batch files cannot overwrite one another.",
    )
    parser.add_argument(
        "--install-dir",
        type=Path,
        help="Optionally install generated scenario JSON into MORAI SaveFile/Scenario.",
    )
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_crosswalk_catalog(mgeo_dir):
    single_rows = load_json(mgeo_dir / "singlecrosswalk_set.json")
    group_rows = load_json(mgeo_dir / "crosswalk_set.json")
    link_rows = load_json(mgeo_dir / "link_set.json")
    singles = {row["idx"]: row for row in single_rows}
    links = {row["idx"]: row for row in link_rows}
    catalog = {}
    for group in group_rows:
        points = []
        link_ids = []
        for single_id in group.get("single_crosswalk_list", []):
            single = singles.get(single_id, {})
            polygon = single.get("points", [])
            link_ids.extend(single.get("link_id_list", []))
            points.extend(
                {
                    "x": float(point[0]),
                    "y": float(point[1]),
                    "z": float(point[2]) if len(point) >= 3 else 0.0,
                }
                for point in polygon
                if len(point) >= 2
            )
        if points:
            center_x = sum(point["x"] for point in points) / len(points)
            center_y = sum(point["y"] for point in points) / len(points)
            road_vectors = []
            for link_id in sorted(set(link_ids)):
                link_points = links.get(link_id, {}).get("points", [])
                if len(link_points) < 2:
                    continue
                nearest = min(
                    range(len(link_points)),
                    key=lambda index: math.hypot(
                        float(link_points[index][0]) - center_x,
                        float(link_points[index][1]) - center_y,
                    ),
                )
                before = link_points[max(0, nearest - 1)]
                after = link_points[min(len(link_points) - 1, nearest + 1)]
                dx = float(after[0]) - float(before[0])
                dy = float(after[1]) - float(before[1])
                length = math.hypot(dx, dy)
                if length <= 1e-9:
                    continue
                vector = (dx / length, dy / length)
                if road_vectors and (
                    vector[0] * road_vectors[0][0]
                    + vector[1] * road_vectors[0][1]
                ) < 0.0:
                    vector = (-vector[0], -vector[1])
                road_vectors.append(vector)
            pedestrian_axis = None
            if road_vectors:
                road_x = sum(vector[0] for vector in road_vectors)
                road_y = sum(vector[1] for vector in road_vectors)
                length = math.hypot(road_x, road_y)
                if length > 1e-9:
                    pedestrian_axis = (-road_y / length, road_x / length)
            catalog[group["idx"]] = {
                "crosswalk_id": group["idx"],
                "single_crosswalk_ids": list(
                    group.get("single_crosswalk_list", [])
                ),
                "traffic_light_ids": list(
                    group.get("ref_traffic_light_list", [])
                ),
                "link_ids": sorted(set(link_ids)),
                "pedestrian_axis": pedestrian_axis,
                "points": points,
            }
    return catalog


def weighted_variant_choice(rows, variant_index, family_count):
    if not rows:
        raise ValueError("weighted variant rows are empty")
    weights = [float(row.get("weight", 1.0)) for row in rows]
    total = sum(weights)
    if total <= 0.0:
        raise ValueError("weighted variant rows must have positive total weight")
    quantile = (int(variant_index) + 0.5) / max(1, int(family_count))
    cumulative = 0.0
    for row, weight in zip(rows, weights):
        cumulative += weight / total
        if quantile <= cumulative:
            return row
    return rows[-1]


def point_to_route_distance(point, route, maximum_segment_m=10.0):
    """Return distance to valid local route segments, ignoring route jumps."""

    best = float("inf")
    for before, after in zip(route, route[1:]):
        dx = after["x"] - before["x"]
        dy = after["y"] - before["y"]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        if math.sqrt(length_squared) > maximum_segment_m:
            continue
        ratio = (
            (point["x"] - before["x"]) * dx
            + (point["y"] - before["y"]) * dy
        ) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        projected_x = before["x"] + ratio * dx
        projected_y = before["y"] + ratio * dy
        best = min(
            best,
            math.hypot(point["x"] - projected_x, point["y"] - projected_y),
        )
    if not math.isfinite(best):
        raise ValueError("route has no valid local segment")
    return best


def local_route_window(route, point, behind_points=160, ahead_points=240):
    """Return the contiguous route portion Ego can actually sweep nearby."""

    nearest_index = min(
        range(len(route)),
        key=lambda index: math.hypot(
            route[index]["x"] - point["x"],
            route[index]["y"] - point["y"],
        ),
    )
    begin = max(0, nearest_index - int(behind_points))
    end = min(len(route), nearest_index + int(ahead_points) + 1)
    return route[begin:end]


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_route(path):
    route = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 2:
                continue
            route.append(
                {
                    "x": float(fields[0]),
                    "y": float(fields[1]),
                    "z": float(fields[2]) if len(fields) >= 3 else 0.0,
                }
            )
    if len(route) < 2:
        raise ValueError("route must contain at least two points")
    arc = [0.0]
    for before, after in zip(route, route[1:]):
        arc.append(
            arc[-1]
            + math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        )
    return route, arc


def densify_route(route, maximum_segment_m=0.5):
    """Fill long waypoint gaps without modifying the official route file."""

    dense = [dict(route[0])]
    for before, after in zip(route, route[1:]):
        distance = math.hypot(
            after["x"] - before["x"], after["y"] - before["y"]
        )
        steps = max(1, int(math.ceil(distance / maximum_segment_m)))
        for step in range(1, steps + 1):
            ratio = step / steps
            dense.append({
                "x": before["x"] + (after["x"] - before["x"]) * ratio,
                "y": before["y"] + (after["y"] - before["y"]) * ratio,
                "z": before["z"] + (after["z"] - before["z"]) * ratio,
            })
    return dense


def _wrapped_radians(value):
    return math.atan2(math.sin(value), math.cos(value))


def _link_length(link):
    return sum(
        math.hypot(
            float(after[0]) - float(before[0]),
            float(after[1]) - float(before[1]),
        )
        for before, after in zip(link.get("points", []), link.get("points", [])[1:])
    )


def _guide_link_anchors(guide_route, links, maximum_guide_segment_m=2.0):
    """Identify stable MGeo links from short, directed guide segments.

    Point matching is ambiguous at MGeo link boundaries because multiple
    links intentionally share the same endpoint.  A run of at least two
    matching guide segments supplies both position and direction and is a
    much safer route anchor.  One-segment connector links are recovered later
    from the directed graph instead of being guessed here.
    """

    cell_size = 2.0
    segment_grid = defaultdict(list)
    for link in links:
        points = link.get("points", [])
        for point_index, (before, after) in enumerate(zip(points, points[1:])):
            dx = float(after[0]) - float(before[0])
            dy = float(after[1]) - float(before[1])
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                continue
            midpoint_x = (float(before[0]) + float(after[0])) * 0.5
            midpoint_y = (float(before[1]) + float(after[1])) * 0.5
            cell = (
                int(math.floor(midpoint_x / cell_size)),
                int(math.floor(midpoint_y / cell_size)),
            )
            segment_grid[cell].append({
                "link_idx": link["idx"],
                "point_index": point_index,
                "x": midpoint_x,
                "y": midpoint_y,
                "yaw": math.atan2(dy, dx),
                "length": length,
            })

    matched_links = []
    unmatched_long_segments = []
    for segment_index, (before, after) in enumerate(
        zip(guide_route, guide_route[1:])
    ):
        dx = float(after["x"]) - float(before["x"])
        dy = float(after["y"]) - float(before["y"])
        length = math.hypot(dx, dy)
        if length > float(maximum_guide_segment_m):
            matched_links.append(None)
            unmatched_long_segments.append({
                "segment_index": segment_index,
                "length_m": length,
                "from": dict(before),
                "to": dict(after),
            })
            continue
        if length <= 1e-9:
            matched_links.append(None)
            continue
        midpoint_x = (float(before["x"]) + float(after["x"])) * 0.5
        midpoint_y = (float(before["y"]) + float(after["y"])) * 0.5
        yaw = math.atan2(dy, dx)
        cell = (
            int(math.floor(midpoint_x / cell_size)),
            int(math.floor(midpoint_y / cell_size)),
        )
        candidates = []
        for cell_dx in (-1, 0, 1):
            for cell_dy in (-1, 0, 1):
                candidates.extend(
                    segment_grid.get((cell[0] + cell_dx, cell[1] + cell_dy), [])
                )
        if not candidates:
            raise ValueError(
                "guide segment {} has no nearby MGeo segment".format(segment_index)
            )
        best = min(
            candidates,
            key=lambda item: (
                math.hypot(item["x"] - midpoint_x, item["y"] - midpoint_y)
                + 2.0 * abs(_wrapped_radians(item["yaw"] - yaw))
                + 0.1 * abs(item["length"] - length)
            ),
        )
        distance = math.hypot(best["x"] - midpoint_x, best["y"] - midpoint_y)
        heading_error = abs(
            math.degrees(_wrapped_radians(best["yaw"] - yaw))
        )
        if distance > 1.5 or heading_error > 45.0:
            raise ValueError(
                "guide segment {} cannot be aligned to MGeo: distance={:.2f}m "
                "heading={:.1f}deg".format(segment_index, distance, heading_error)
            )
        matched_links.append(best["link_idx"])

    runs = []
    for segment_index, link_id in enumerate(matched_links):
        if link_id is None:
            continue
        if runs and runs[-1]["link_idx"] == link_id:
            runs[-1]["last_segment"] = segment_index
            runs[-1]["segment_count"] += 1
        else:
            runs.append({
                "link_idx": link_id,
                "first_segment": segment_index,
                "last_segment": segment_index,
                "segment_count": 1,
            })
    anchors = [run for run in runs if run["segment_count"] >= 2]
    if len(anchors) < 2:
        raise ValueError("guide route does not provide enough stable MGeo link anchors")
    return anchors, unmatched_long_segments


def _shortest_directed_link_path(links_by_id, successors, start_link, goal_link):
    """Return a deterministic shortest directed MGeo link path."""

    if start_link == goal_link:
        return [start_link]
    queue = [(0.0, (start_link,), start_link)]
    best_cost = {}
    while queue:
        cost, path_key, link_id = heapq.heappop(queue)
        if link_id in best_cost:
            continue
        best_cost[link_id] = cost
        if link_id == goal_link:
            return list(path_key)
        node_id = links_by_id[link_id]["to_node_idx"]
        for successor in sorted(successors.get(node_id, [])):
            if successor in best_cost or successor in path_key:
                continue
            successor_cost = max(0.01, _link_length(links_by_id[successor]))
            heapq.heappush(
                queue,
                (cost + successor_cost, path_key + (successor,), successor),
            )
    raise ValueError(
        "MGeo has no directed path from {} to {}".format(start_link, goal_link)
    )


def _sampled_route_point_distances(points, route, cell_size=2.0):
    """Measure guide-to-route distance with a small spatial hash."""

    grid = defaultdict(list)
    for row in route:
        cell = (
            int(math.floor(float(row["x"]) / cell_size)),
            int(math.floor(float(row["y"]) / cell_size)),
        )
        grid[cell].append(row)
    distances = []
    for point in points:
        cell = (
            int(math.floor(float(point["x"]) / cell_size)),
            int(math.floor(float(point["y"]) / cell_size)),
        )
        candidates = []
        for cell_dx in (-1, 0, 1):
            for cell_dy in (-1, 0, 1):
                candidates.extend(
                    grid.get((cell[0] + cell_dx, cell[1] + cell_dy), [])
                )
        if not candidates:
            raise ValueError("guide point has no nearby reconstructed route sample")
        distances.append(min(
            math.hypot(
                float(candidate["x"]) - float(point["x"]),
                float(candidate["y"]) - float(point["y"]),
            )
            for candidate in candidates
        ))
    return distances


def build_mgeo_guided_competition_route(guide_route, links):
    """Reconstruct the closed competition route from native MGeo geometry.

    The planner guide selects the directed link sequence; generated Ego poses,
    headings, and arc distances come exclusively from MGeo link geometry.
    """

    links_by_id = {link["idx"]: link for link in links}
    successors = defaultdict(list)
    for link in links:
        # MGeo materializes lane-change shortcuts as lazy two-point links.
        # They are valid graph edges for planning, but not native road
        # centerline geometry for spatial dataset sampling.  A normal
        # directed lane path is always preferred here; explicit multi-lane
        # dataset families already build their own verified MGeo routes.
        if link.get("lazy_init") or link.get("lane_ch_link_path"):
            continue
        successors[link["from_node_idx"]].append(link["idx"])
    anchors, guide_gaps = _guide_link_anchors(guide_route, links)
    anchor_ids = [row["link_idx"] for row in anchors]
    link_path = [anchor_ids[0]]
    inserted_paths = []
    for pair_index in range(len(anchor_ids)):
        start = anchor_ids[pair_index]
        goal = anchor_ids[(pair_index + 1) % len(anchor_ids)]
        path = _shortest_directed_link_path(
            links_by_id, successors, start, goal
        )
        inserted = path[1:-1]
        if inserted:
            inserted_paths.append({
                "from_link": start,
                "to_link": goal,
                "inserted_links": inserted,
            })
        if pair_index < len(anchor_ids) - 1:
            link_path.extend(path[1:])
        else:
            # The final path closes the loop back to the first anchor.  Keep
            # connector links but do not duplicate the first link itself.
            link_path.extend(path[1:-1])

    if len(link_path) != len(set(link_path)):
        duplicates = [
            link_id for link_id, count in Counter(link_path).items() if count > 1
        ]
        raise ValueError(
            "MGeo competition route repeats links: {}".format(duplicates)
        )
    samples, _ = link_path_samples(links_by_id, link_path)
    if len(samples) < 2:
        raise ValueError("MGeo competition route contains fewer than two samples")
    closing_gap = math.hypot(
        samples[-1]["x"] - samples[0]["x"],
        samples[-1]["y"] - samples[0]["y"],
    )
    if closing_gap > 0.75:
        raise ValueError(
            "MGeo competition route is not closed: gap={:.3f}m".format(closing_gap)
        )
    maximum_segment = max(
        math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        for before, after in zip(samples, samples[1:])
    )
    if maximum_segment > 0.75:
        raise ValueError(
            "MGeo competition route sampling gap exceeds 0.75m: {:.3f}m".format(
                maximum_segment
            )
        )
    guide_distances = _sampled_route_point_distances(guide_route, samples)
    audit = {
        "source": "mgeo_directed_links_guided_by_sparse_global_path",
        "guide_point_count": len(guide_route),
        "stable_anchor_link_count": len(anchor_ids),
        "link_count": len(link_path),
        "link_path": link_path,
        "inserted_paths": inserted_paths,
        "guide_gaps": guide_gaps,
        "point_count": len(samples),
        "length_m": route_arc_coordinates(samples)[-1],
        "maximum_segment_m": maximum_segment,
        "closing_gap_m": closing_gap,
        "guide_distance_mean_m": sum(guide_distances) / len(guide_distances),
        "guide_distance_max_m": max(guide_distances),
    }
    return samples, audit


def nearest_index(points, x, y):
    return min(
        range(len(points)),
        key=lambda index: math.hypot(points[index]["x"] - x, points[index]["y"] - y),
    )


def continuous_route_pose_at_arc(route, arc, distance_m, wrap=False):
    """Interpolate a pose on an open or closed route arc.

    Legacy guide routes retain their route indices.  Native MGeo routes also
    retain link identity so a pose near the closed-loop seam cannot later be
    snapped to the spatially identical point on the wrong directed link.
    """

    distance_m = float(distance_m)
    if wrap and arc[-1] > 0.0:
        distance_m %= arc[-1]
    else:
        distance_m = max(0.0, min(float(arc[-1]), distance_m))
    upper = bisect.bisect_left(arc, distance_m)
    upper = max(1, min(len(route) - 1, upper))
    lower = upper - 1
    span = max(1e-9, float(arc[upper]) - float(arc[lower]))
    ratio = (distance_m - float(arc[lower])) / span
    before = route[lower]
    after = route[upper]
    pose = {
        "x": before["x"] + (after["x"] - before["x"]) * ratio,
        "y": before["y"] + (after["y"] - before["y"]) * ratio,
        "z": before["z"] + (after["z"] - before["z"]) * ratio,
        "yaw": math.degrees(
            math.atan2(after["y"] - before["y"], after["x"] - before["x"])
        ),
        "route_index": lower,
        "route_arc_m": distance_m,
    }
    # Native MGeo routes carry an unambiguous link identity.  Preserve it so
    # a closed-loop seam shared by the final and first link cannot be snapped
    # to the wrong occurrence by a later nearest-point query.
    selected = before if ratio < 0.5 else after
    if selected.get("link_idx") is not None:
        pose["link_idx"] = selected["link_idx"]
        pose["point_idx"] = int(selected.get("point_idx", 0))
        pose["max_speed"] = float(selected.get("max_speed", 30.0))
        if before.get("link_idx") == after.get("link_idx"):
            pose["point_ratio"] = (
                float(before.get("point_ratio", 0.0))
                + (
                    float(after.get("point_ratio", 0.0))
                    - float(before.get("point_ratio", 0.0))
                )
                * ratio
            )
        else:
            pose["point_ratio"] = float(selected.get("point_ratio", 0.0))
    return pose


def route_pose_before(
    route, arc, target, lead_distance, wrap_closed_route=False
):
    target_index = nearest_index(route, target["x"], target["y"])
    target_arc = float(target.get("route_arc_m", arc[target_index]))
    if not wrap_closed_route:
        target_arc = max(0.0, target_arc - lead_distance)
        # Do not quantize Ego to a sparse MGeo vertex.  Highway link points
        # can be more than 10 m apart, which previously turned an intended
        # 11 m lead into 22+ m and shifted every same-lane target accordingly.
        return continuous_route_pose_at_arc(route, arc, target_arc, wrap=False)
    return continuous_route_pose_at_arc(
        route,
        arc,
        target_arc - lead_distance,
        wrap=wrap_closed_route,
    )


def rotate_closed_route_for_ego(
    route,
    arc,
    ego_pose,
    backtrack_m=12.0,
):
    """Move a closed waypoint array's seam behind the scene's Ego pose.

    The online controller intentionally does not wrap an array index from the
    final waypoint back to zero.  A geometrically closed route can therefore
    still fail when Ego starts close to the file's final waypoint: lookahead
    clamps at the end and eventually points behind the vehicle.  Rotating the
    same directed MGeo route per scene keeps its geometry unchanged while
    placing the file seam safely behind Ego.
    """

    if len(route) < 3 or len(route) != len(arc):
        raise ValueError("closed route rotation requires matching route/arc")
    closing_gap = math.hypot(
        route[-1]["x"] - route[0]["x"],
        route[-1]["y"] - route[0]["y"],
    )
    if closing_gap > 0.75:
        raise ValueError(
            "cannot rotate an open route: closing gap={:.3f}m".format(
                closing_gap
            )
        )

    # The native MGeo reconstruction ends at the same coordinate at which it
    # starts.  Keep only one copy while rotating, then append a closing copy.
    core = list(route[:-1]) if closing_gap < 1e-4 else list(route)
    total_length = float(arc[-1])
    ego_arc = float(
        ego_pose.get(
            "route_arc_m",
            arc[nearest_index(route, ego_pose["x"], ego_pose["y"])],
        )
    )
    start_arc = (ego_arc - float(backtrack_m)) % total_length
    start_index = route_index_at_arc(arc, start_arc)
    start_index = min(start_index, len(core) - 1)
    rotated = core[start_index:] + core[:start_index]
    rotated.append(dict(rotated[0]))
    anchor_index = nearest_index(
        rotated,
        ego_pose["x"],
        ego_pose["y"],
    )
    execution_arc = route_arc_coordinates(rotated)
    if anchor_index <= 0 or execution_arc[anchor_index] < 8.0:
        raise ValueError(
            "rotated route did not retain enough controller backtrack: "
            "anchor={} arc={:.3f}m".format(
                anchor_index,
                execution_arc[anchor_index],
            )
        )
    maximum_segment = max(
        math.hypot(
            after["x"] - before["x"],
            after["y"] - before["y"],
        )
        for before, after in zip(rotated, rotated[1:])
    )
    if maximum_segment > 0.75:
        raise ValueError(
            "rotated route sampling gap exceeds 0.75m: {:.3f}m".format(
                maximum_segment
            )
        )
    return rotated, anchor_index, {
        "canonical_ego_arc_m": ego_arc,
        "execution_start_arc_m": start_arc,
        "execution_anchor_index": anchor_index,
        "execution_anchor_arc_m": execution_arc[anchor_index],
        "execution_length_m": execution_arc[-1],
        "maximum_segment_m": maximum_segment,
        "closing_gap_m": math.hypot(
            rotated[-1]["x"] - rotated[0]["x"],
            rotated[-1]["y"] - rotated[0]["y"],
        ),
    }


def mgeo_samples(links):
    samples = []
    for link in links:
        points = link.get("points", [])
        for point_index, point in enumerate(points):
            if len(point) < 2:
                continue
            before = points[max(0, point_index - 1)]
            after = points[min(len(points) - 1, point_index + 1)]
            samples.append(
                {
                    "x": float(point[0]),
                    "y": float(point[1]),
                    "z": float(point[2]) if len(point) >= 3 else 0.0,
                    "yaw": math.degrees(
                        math.atan2(after[1] - before[1], after[0] - before[0])
                    ),
                    "link_idx": link["idx"],
                    "point_idx": point_index,
                    "point_ratio": point_index / max(1, len(points) - 1),
                    "max_speed": float(link.get("max_speed", 30.0)),
                }
            )
    return samples


def nearest_sample(samples, pose):
    return min(
        samples,
        key=lambda item: math.hypot(item["x"] - pose["x"], item["y"] - pose["y"]),
    )


_M_GEO_SAMPLE_GRID_CACHE = {}


def nearby_mgeo_samples(samples, pose, radius_m, cell_size_m=5.0):
    """Return local MGeo samples without rescanning the complete map."""

    cache_key = id(samples)
    cached = _M_GEO_SAMPLE_GRID_CACHE.get(cache_key)
    if cached is None or cached[0] is not samples:
        grid = {}
        for item in samples:
            cell = (
                int(math.floor(float(item["x"]) / cell_size_m)),
                int(math.floor(float(item["y"]) / cell_size_m)),
            )
            grid.setdefault(cell, []).append(item)
        cached = (samples, grid)
        _M_GEO_SAMPLE_GRID_CACHE[cache_key] = cached
    grid = cached[1]
    center_x = int(math.floor(float(pose["x"]) / cell_size_m))
    center_y = int(math.floor(float(pose["y"]) / cell_size_m))
    reach = int(math.ceil(float(radius_m) / cell_size_m))
    nearby = []
    for cell_x in range(center_x - reach, center_x + reach + 1):
        for cell_y in range(center_y - reach, center_y + reach + 1):
            nearby.extend(grid.get((cell_x, cell_y), ()))
    return nearby


def wrapped_degrees(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def nearest_aligned_sample(samples, pose, search_radius_m=6.0):
    nearby = []
    for item in nearby_mgeo_samples(samples, pose, search_radius_m):
        distance = math.hypot(item["x"] - pose["x"], item["y"] - pose["y"])
        if distance <= search_radius_m:
            heading_error = abs(wrapped_degrees(item["yaw"] - pose["yaw"]))
            nearby.append((heading_error > 90.0, heading_error, distance, item))
    if not nearby:
        return nearest_sample(samples, pose)
    nearby.sort(key=lambda row: (row[0], row[2] + row[1] * 0.02))
    return dict(nearby[0][3])


def project_relative(path):
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path.resolve())


def formatted_vector(x, y, z):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "_x": f"{float(x):.3f}",
        "_y": f"{float(y):.3f}",
        "_z": f"{float(z):.3f}",
    }


def formatted_rotation(yaw):
    return {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw:.3f}"}


def zero_vector(vector):
    if not isinstance(vector, dict):
        return
    vector.update(formatted_vector(0.0, 0.0, 0.0))


def set_ego_pose(ego, sample, initial_speed_kph=0.0):
    initial = ego["initPosition"]
    initial["initPositionMode"] = "Absolute"
    initial["pos"] = formatted_vector(sample["x"], sample["y"], sample["z"] + 0.34)
    initial["rot"] = formatted_rotation(sample["yaw"])
    initial["initLink"] = sample["link_idx"]
    initial["initLinkRatio"] = sample["point_ratio"]
    initial["gear"] = 4
    ego["currentVelocity"] = float(initial_speed_kph)
    ego["customValue"] = float(initial_speed_kph)
    ego["linkValue"] = sample["max_speed"]
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = float(initial_speed_kph)
    zero_vector(saved.get("lastVelocity"))
    zero_vector(saved.get("vehicle", {}).get("lastVelocity"))
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 4


def world_to_morai_waypoint(position):
    # Verified against PedPath - 3 in the official competition sample.
    return formatted_vector(-position["x"], position["z"], -position["y"])


def waypoint_to_world(position):
    return {
        "x": -float(position["x"]),
        "y": -float(position["z"]),
        "z": float(position["y"]),
    }


def set_actor_world_position(actor, position, yaw=0.0):
    actor["pos"] = formatted_vector(position["x"], position["y"], position["z"])
    actor["rot"] = formatted_rotation(yaw)
    internal = world_to_morai_waypoint(position)
    if "initPos" in actor:
        actor["initPos"] = copy.deepcopy(internal)
    if "standardPos" in actor:
        actor["standardPos"] = copy.deepcopy(internal)


def clear_actor_lists(scenario):
    scenario["vehicleList"] = []
    scenario["pedestrianList"] = []
    scenario["objectList"] = []
    scenario["spawnPointList"] = []
    scenario["pedspawnPointList"] = []
    scenario["aiPedstrianList"] = []
    scenario["waypointDataList"] = []


def uniform_or_choice(rng, value):
    if isinstance(value, list) and len(value) == 2:
        return rng.uniform(float(value[0]), float(value[1]))
    if isinstance(value, list):
        return rng.choice(value)
    return value


def integer_count(rng, value):
    """Choose an inclusive integer count from either N or [minimum, maximum]."""

    if isinstance(value, list):
        if len(value) != 2:
            return int(rng.choice(value))
        lower, upper = (int(value[0]), int(value[1]))
        if lower > upper:
            raise ValueError("integer count range is reversed")
        return rng.randint(lower, upper)
    return int(value)


def choose_lead(rng, family):
    return uniform_or_choice(rng, family.get("ego_lead_distance_m", [18.0, 25.0]))


def target_position_for_family(template, family_name, selected_spawns=None):
    if family_name in SIMPLE_SPAWN_VEHICLE_FAMILIES:
        spawns = selected_spawns or template["spawnPointList"]
        x = sum(float(item["pos"]["x"]) for item in spawns) / len(spawns)
        y = sum(float(item["pos"]["y"]) for item in spawns) / len(spawns)
        z = sum(float(item["pos"]["z"]) for item in spawns) / len(spawns)
        return {"x": x, "y": y, "z": z}
    if family_name == "crosswalk_pedestrian":
        return dict(template["pedestrianList"][0]["pos"])
    if family_name == "static_obstacle":
        return dict(template["objectList"][0]["pos"])
    if family_name == "gps_shadow_obstacle":
        return dict(template["shadedAreaList"][0]["pos"])
    if family_name == "stopped_vehicle":
        return dict(template["vehicleList"][0]["initPosition"]["pos"])
    if family_name == "standing_pedestrian":
        return dict(template["pedestrianList"][0]["pos"])
    if family_name in {"realistic_mixed_static", "sparse_background"}:
        return dict(template["objectList"][0]["pos"])
    raise ValueError(f"unsupported family: {family_name}")


def configure_spawn(spawn, rng, family, base_target_speed=None):
    period = uniform_or_choice(rng, family["spawn_period_sec"])
    spawn["minSpawnPeriod"] = period
    spawn["maxSpawnPeriod"] = period
    spawn["maximumSpawnVehicle"] = int(
        family.get("maximum_spawn_vehicles", 20)
    )
    spawn["spawnedVehiclesCount"] = 0
    spawn["waitSpawnTime"] = float(family.get("spawn_wait_sec", 0.0))
    spawn["isSpawning"] = True
    if "target_speed_kph" in family:
        target = float(uniform_or_choice(rng, family["target_speed_kph"]))
    else:
        scale = uniform_or_choice(rng, family.get("target_speed_scale", [1.0, 1.0]))
        target = float(base_target_speed) * float(scale)
    spawn["MinDesiredVelocity_Custom"] = target
    spawn["MaxDesiredVelocity_Custom"] = target
    if family.get("spawn_velocity_equals_target", False):
        spawn["SpawnVelocityType"] = 2
        spawn["MinSpawnVelocity_Custom"] = target
        spawn["MaxSpawnVelocity_Custom"] = target
    return target, period


def set_spawn_target_speed(spawn, target, match_spawn_velocity=False):
    target = float(target)
    spawn["MinDesiredVelocity_Custom"] = target
    spawn["MaxDesiredVelocity_Custom"] = target
    if match_spawn_velocity:
        spawn["SpawnVelocityType"] = 2
        spawn["MinSpawnVelocity_Custom"] = target
        spawn["MaxSpawnVelocity_Custom"] = target


def weighted_lane_schedule(family):
    """Return an interleaved, exact-count Ego lane schedule for one family."""

    total = int(family.get("count", 1))
    raw_weights = family.get("ego_lane_weights")
    if not raw_weights:
        source_ids = [int(item) for item in family["source_spawn_ids"]]
        return [source_ids[index % len(source_ids)] for index in range(total)]
    weights = {int(key): float(value) for key, value in raw_weights.items()}
    weight_sum = sum(weights.values())
    if total <= 0 or weight_sum <= 0.0:
        raise ValueError("invalid Ego lane weight configuration")
    exact = {
        lane: total * weight / weight_sum for lane, weight in weights.items()
    }
    quotas = {lane: int(math.floor(value)) for lane, value in exact.items()}
    remaining = total - sum(quotas.values())
    for lane in sorted(
        weights,
        key=lambda item: (-(exact[item] - quotas[item]), item),
    )[:remaining]:
        quotas[lane] += 1
    selected = {lane: 0 for lane in quotas}
    schedule = []
    while len(schedule) < total:
        candidates = [lane for lane in quotas if selected[lane] < quotas[lane]]
        lane = min(
            candidates,
            key=lambda item: (
                (selected[item] + 1.0) / quotas[item],
                item,
            ),
        )
        schedule.append(lane)
        selected[lane] += 1
    return schedule


def weighted_zone_schedule(family, key="ego_capture_zones"):
    """Return an interleaved exact-count spatial-zone schedule."""

    zones = list(family.get(key, []))
    total = int(family.get("count", 1))
    if not zones:
        return []
    weights = {
        str(zone["name"]): float(zone.get("weight", 1.0)) for zone in zones
    }
    if len(weights) != len(zones) or any(
        value <= 0.0 for value in weights.values()
    ):
        raise ValueError("invalid or duplicate capture-zone configuration")
    weight_sum = sum(weights.values())
    exact = {name: total * weight / weight_sum for name, weight in weights.items()}
    quotas = {name: int(math.floor(value)) for name, value in exact.items()}
    remaining = total - sum(quotas.values())
    for name in sorted(
        weights,
        key=lambda item: (-(exact[item] - quotas[item]), item),
    )[:remaining]:
        quotas[name] += 1
    selected = {name: 0 for name in quotas}
    schedule = []
    while len(schedule) < total:
        candidates = [name for name in quotas if selected[name] < quotas[name]]
        name = min(
            candidates,
            key=lambda item: ((selected[item] + 1.0) / quotas[item], item),
        )
        schedule.append(name)
        selected[name] += 1
    return schedule


def weighted_pair_schedule(
    family,
    zone_key="ego_capture_zones",
    role_key="interaction_roles",
):
    """Pair exact zone/role quotas without making either a split cue.

    Building the two weighted schedules independently can repeatedly bind one
    road block to one motion role.  Greedy normalized pair counts retain both
    marginal quotas while spreading every role across as many blocks as the
    small campaign permits.
    """

    zones = weighted_zone_schedule(family, key=zone_key)
    role_rows = {
        str(row["name"]): row for row in family.get(role_key, [])
    }
    role_names = list(role_rows)
    offset = int(family.get("pair_schedule_offset", 0)) % len(role_names)
    rotated_names = role_names[offset:] + role_names[:offset]
    role_priority = {
        name: index for index, name in enumerate(rotated_names)
    }
    role_schedule = weighted_zone_schedule(family, key=role_key)
    zone_quota = Counter(zones)
    role_quota = Counter(role_schedule)
    zone_rows = {
        str(row["name"]): row for row in family.get(zone_key, [])
    }
    split_role_quotas = family.get("split_role_quotas")
    split_remaining = None
    if split_role_quotas:
        split_remaining = {
            str(split): Counter(
                {str(role): int(count) for role, count in quotas.items()}
            )
            for split, quotas in split_role_quotas.items()
        }
        actual_split_quota = Counter(
            str(zone_rows[zone].get("split", "train")) for zone in zones
        )
        if {
            split: sum(counts.values())
            for split, counts in split_remaining.items()
        } != dict(actual_split_quota):
            raise ValueError("split_role_quotas do not match spatial quotas")
        assigned_roles = Counter()
        for counts in split_remaining.values():
            assigned_roles.update(counts)
        if assigned_roles != role_quota:
            raise ValueError("split_role_quotas do not match role quotas")
    role_remaining = Counter(role_schedule)
    pair_counts = Counter()
    pairs = []
    total = max(1, len(zones))
    for zone in zones:
        split = str(zone_rows[zone].get("split", "train"))
        candidates = [role for role in role_rows if role_remaining[role] > 0]
        if split_remaining is not None:
            candidates = [
                role
                for role in candidates
                if split_remaining[split][role] > 0
            ]
        role = min(
            candidates,
            key=lambda name: (
                (pair_counts[(zone, name)] + 1.0)
                / max(
                    1e-9,
                    zone_quota[zone] * role_quota[name] / total,
                ),
                (role_quota[name] - role_remaining[name])
                / role_quota[name],
                role_priority[name],
            ),
        )
        pairs.append((zone, role))
        pair_counts[(zone, role)] += 1
        role_remaining[role] -= 1
        if split_remaining is not None:
            split_remaining[split][role] -= 1
    if any(role_remaining.values()):
        raise AssertionError("zone/role pair schedule did not exhaust quotas")
    if split_remaining is not None and any(
        count
        for counts in split_remaining.values()
        for count in counts.values()
    ):
        raise AssertionError("zone/role pair schedule did not exhaust split quotas")
    return pairs


def route_pose_at_index(route, index):
    index = max(0, min(len(route) - 1, int(index)))
    before = route[max(0, index - 1)]
    after = route[min(len(route) - 1, index + 1)]
    return {
        **route[index],
        "yaw": math.degrees(
            math.atan2(after["y"] - before["y"], after["x"] - before["x"])
        ),
        "route_index": index,
    }


def route_index_at_arc(arc, distance_m):
    insertion = bisect.bisect_left(arc, float(distance_m))
    candidates = [max(0, min(len(arc) - 1, insertion))]
    if insertion:
        candidates.append(insertion - 1)
    return min(candidates, key=lambda item: abs(arc[item] - float(distance_m)))


def route_wide_anchor_for_variant(route, arc, family, variant_index, rng):
    """Sample a deterministic, stratified anchor over the competition route."""

    bin_count = int(family.get("route_wide_bin_count", 24))
    margin_m = float(family.get("route_wide_margin_m", 35.0))
    if family.get("route_wide_closed_loop", False):
        margin_m = 0.0
    usable_start = margin_m
    usable_end = max(usable_start + 1.0, float(arc[-1]) - margin_m)
    slot_count = int(family.get("route_wide_slot_count", bin_count))
    family_offset = (
        sum(ord(char) for char in family["name"])
        + int(family.get("route_wide_phase_offset", 0))
    ) % slot_count
    slot_index = (int(variant_index) * 7 + family_offset) % slot_count
    slot_width = (usable_end - usable_start) / slot_count
    lower = usable_start + slot_index * slot_width
    # Keep jitter inside the middle 40% of each slot.  The old 76% jitter
    # could align adjacent samples at opposite bin edges and leave a visible
    # 100+ m hole despite nominal 24-bin coverage.
    target_arc = rng.uniform(
        lower + 0.30 * slot_width,
        lower + 0.70 * slot_width,
    )
    bin_index = min(
        bin_count - 1,
        int((target_arc - usable_start) / (usable_end - usable_start) * bin_count),
    )
    pose = continuous_route_pose_at_arc(
        route,
        arc,
        target_arc,
        wrap=family.get("route_wide_closed_loop", False),
    )
    index = int(pose["route_index"])
    pose.update({
        "route_wide_bin": bin_index,
        "route_wide_bin_count": bin_count,
        "route_arc_m": float(target_arc),
    })
    return pose


def route_anchor_in_intervals(route, arc, intervals, variant_index, total, rng):
    """Stratify samples by metric distance over one or more route intervals."""

    valid = [(float(a), float(b)) for a, b in intervals if float(b) > float(a)]
    length = sum(b - a for a, b in valid)
    if not valid or length <= 0.0:
        raise ValueError("route interval sampling requires a positive interval")
    slot = (int(variant_index) + rng.uniform(0.30, 0.70)) / max(1, int(total))
    distance = (slot % 1.0) * length
    for lower, upper in valid:
        span = upper - lower
        if distance <= span:
            pose = continuous_route_pose_at_arc(route, arc, lower + distance, wrap=True)
            pose["route_arc_m"] = float(lower + distance)
            return pose
        distance -= span
    return continuous_route_pose_at_arc(route, arc, valid[-1][1], wrap=True)


def weighted_row_schedule(rows, total, minimum_each=0):
    """Return an exact interleaved schedule of named weighted dictionaries."""

    total = int(total)
    if total <= 0 or not rows:
        return []
    by_name = {str(row["name"]): row for row in rows}
    if len(by_name) != len(rows):
        raise ValueError("weighted row names must be unique")
    weights = {name: float(row.get("weight", 1.0)) for name, row in by_name.items()}
    if any(value <= 0.0 for value in weights.values()):
        raise ValueError("weighted row values must be positive")
    minimum_each = int(minimum_each)
    if minimum_each < 0 or minimum_each * len(rows) > total:
        raise ValueError("weighted row minimum exceeds schedule size")
    weighted_total = total - minimum_each * len(rows)
    exact = {
        name: minimum_each + weighted_total * value / sum(weights.values())
        for name, value in weights.items()
    }
    quotas = {name: int(math.floor(value)) for name, value in exact.items()}
    remainder = total - sum(quotas.values())
    for name in sorted(
        weights,
        key=lambda item: (-(exact[item] - quotas[item]), item),
    )[:remainder]:
        quotas[name] += 1
    selected = {name: 0 for name in quotas}
    schedule = []
    while len(schedule) < total:
        candidates = [name for name in quotas if selected[name] < quotas[name]]
        name = min(
            candidates,
            key=lambda item: ((selected[item] + 1.0) / quotas[item], item),
        )
        schedule.append(by_name[name])
        selected[name] += 1
    return schedule


def link_arc_interval(route, arc, link_id, start_margin_m=30.0, end_margin_m=15.0):
    """Return a safe target interval inside one link of a flattened route."""

    indices = [index for index, row in enumerate(route) if row.get("link_idx") == link_id]
    if not indices:
        raise ValueError("target link {} is absent from route".format(link_id))
    lower = float(arc[min(indices)])
    upper = float(arc[max(indices)])
    safe_lower = lower + float(start_margin_m)
    safe_upper = upper - float(end_margin_m)
    if safe_upper <= safe_lower:
        midpoint = (lower + upper) * 0.5
        half = max(0.5, min((upper - lower) * 0.2, 5.0))
        safe_lower, safe_upper = midpoint - half, midpoint + half
    return safe_lower, safe_upper


def static_highway_phase_target(
    stratum,
    phase_within,
    phase_total,
    rng,
    links_by_id,
):
    """Resolve one topology-aware Base3 highway route and target anchor."""

    variants = HIGHWAY_STATIC_PHASE_VARIANTS[stratum]
    schedule = weighted_row_schedule(
        variants,
        phase_total,
        minimum_each=1 if phase_total >= len(variants) else 0,
    )
    variant = schedule[int(phase_within)]
    variant_within = sum(
        row["name"] == variant["name"] for row in schedule[:int(phase_within)]
    )
    variant_total = sum(row["name"] == variant["name"] for row in schedule)
    route_ids = tuple(variant["route"])
    restricted = HIGHWAY_RESTRICTED_LINK_IDS.intersection(route_ids)
    if restricted:
        raise ValueError(
            "restricted highway links in phase route: {}".format(sorted(restricted))
        )
    route, arc = route_from_link_path(links_by_id, route_ids)
    interval = link_arc_interval(route, arc, variant["target_link"])
    target = route_anchor_in_intervals(
        route,
        arc,
        [interval],
        variant_within,
        variant_total,
        rng,
    )
    metadata = {
        "spatial_stratum": stratum,
        "highway_phase": stratum,
        "highway_phase_variant": variant["name"],
        "highway_phase_within": int(phase_within),
        "highway_phase_total": int(phase_total),
        "highway_phase_variant_within": int(variant_within),
        "highway_phase_variant_total": int(variant_total),
        "highway_target_link": variant["target_link"],
        "highway_route_links": list(route_ids),
        "highway_side_routes": [list(row) for row in variant.get("side_routes", ())],
    }
    return route, arc, target, metadata


def static_two_lane_target(stratum, within, total, rng, links_by_id):
    """Return an explicit left/right execution route on an ordinary 2-lane road."""

    side = "right" if stratum.endswith("_right") else "left"
    variants = [
        {**row, "weight": 1.0} for row in ROUTE_GENERAL_TWO_LANE_VARIANTS
    ]
    schedule = weighted_row_schedule(variants, total)
    variant = schedule[int(within)]
    variant_within = sum(
        row["name"] == variant["name"] for row in schedule[:int(within)]
    )
    variant_total = sum(row["name"] == variant["name"] for row in schedule)
    route_ids = tuple(variant[side + "_route"])
    target_link = variant[side + "_target_link"]
    route, arc = route_from_link_path(links_by_id, route_ids)
    target = route_anchor_in_intervals(
        route,
        arc,
        [link_arc_interval(route, arc, target_link)],
        variant_within,
        variant_total,
        rng,
    )
    opposite = "left" if side == "right" else "right"
    metadata = {
        "spatial_stratum": stratum,
        "ordinary_two_lane_side": side,
        "ordinary_two_lane_variant": variant["name"],
        "ordinary_two_lane_within": int(within),
        "ordinary_two_lane_total": int(total),
        "ordinary_two_lane_route_links": list(route_ids),
        "ordinary_two_lane_target_link": target_link,
        "ordinary_two_lane_parallel_route": list(
            variant[opposite + "_route"]
        ),
    }
    return route, arc, target, metadata


def static_spatial_target(
    family,
    variant_index,
    rng,
    route,
    arc,
    links_by_id,
):
    """Resolve Base3 static's route stratum, roundabout phase, and highway lane."""

    global_strata = family.get("global_spatial_stratum_schedule")
    global_variant_index = int(variant_index) + int(
        family.get("family_global_variant_offset", 0)
    )
    if global_strata:
        strata = list(global_strata)
        schedule_index = global_variant_index
    else:
        strata = weighted_zone_schedule(family, key="spatial_strata")
        schedule_index = int(variant_index)
    stratum = strata[schedule_index] if strata else "route_general"
    metadata = {"spatial_stratum": stratum}
    if stratum in HIGHWAY_STATIC_PHASE_VARIANTS:
        count = strata.count(stratum)
        within = sum(item == stratum for item in strata[:schedule_index])
        return static_highway_phase_target(
            stratum, within, count, rng, links_by_id
        )
    if stratum in {
        "route_general_two_lane_left", "route_general_two_lane_right"
    }:
        count = strata.count(stratum)
        within = sum(item == stratum for item in strata[:schedule_index])
        return static_two_lane_target(
            stratum, within, count, rng, links_by_id
        )
    if stratum == "roundabout":
        count = strata.count(stratum)
        within = sum(item == stratum for item in strata[:schedule_index])
        phase_family = {
            "count": count,
            "roundabout_phases": family["roundabout_phases"],
        }
        phase_schedule = weighted_zone_schedule(
            phase_family, key="roundabout_phases"
        )
        phase = phase_schedule[within]
        phase_within = sum(item == phase for item in phase_schedule[:within])
        phase_total = phase_schedule.count(phase)
        row = next(item for item in family["roundabout_phases"] if item["name"] == phase)
        target = route_anchor_in_intervals(
            route, arc, [row["arc_m"]], phase_within, phase_total, rng
        )
        metadata["roundabout_phase"] = phase
        metadata["roundabout_phase_within"] = phase_within
        metadata["roundabout_phase_total"] = phase_total
        return route, arc, target, metadata
    if stratum == "highway_three_lane":
        count = strata.count(stratum)
        within = sum(item == stratum for item in strata[:schedule_index])
        lane_family = {
            "count": count,
            "source_spawn_ids": list(HIGHWAY_LANE_LINK_PATHS),
            "ego_lane_weights": {"6": 1.0, "8": 1.0, "10": 1.0},
        }
        lane_schedule = weighted_lane_schedule(lane_family)
        lane_id = lane_schedule[within]
        lane_within = sum(item == lane_id for item in lane_schedule[:within])
        lane_total = lane_schedule.count(lane_id)
        lane_route, lane_arc = route_from_link_path(
            links_by_id, HIGHWAY_LANE_LINK_PATHS[lane_id]
        )
        margin = 35.0
        target = route_anchor_in_intervals(
            lane_route,
            lane_arc,
            [(margin, max(margin + 1.0, lane_arc[-1] - margin))],
            lane_within,
            lane_total,
            rng,
        )
        metadata["highway_lane_id"] = lane_id
        return lane_route, lane_arc, target, metadata
    intervals = family.get(
        "route_general_arc_intervals_m", [[0.0, float(arc[-1])]]
    )
    count = strata.count(stratum) if strata else int(family["count"])
    within = (
        sum(item == stratum for item in strata[:schedule_index])
        if strata else int(variant_index)
    )
    target = route_anchor_in_intervals(
        route, arc, intervals, within, count, rng
    )
    return route, arc, target, metadata


def route_wide_crosswalk_target(crosswalk_catalog, route, family, variant_index):
    """Choose a route-adjacent crosswalk without making map position a split."""

    candidates = []
    for crosswalk in crosswalk_catalog.values():
        center = {
            axis: sum(float(point[axis]) for point in crosswalk["points"])
            / len(crosswalk["points"])
            for axis in ("x", "y", "z")
        }
        distance = point_to_route_distance(center, route)
        if distance <= float(family.get("maximum_crosswalk_route_distance_m", 18.0)):
            candidates.append((crosswalk["crosswalk_id"], center, distance))
    if not candidates:
        raise ValueError("no route-adjacent crosswalk is available")
    candidates.sort(key=lambda row: row[0])
    selected = candidates[
        (
            int(variant_index) * 5
            + 1
            + int(family.get("route_wide_phase_offset", 0))
        )
        % len(candidates)
    ]
    return {
        **selected[1],
        "crosswalk_id": selected[0],
        "route_distance_m": selected[2],
    }


def link_path_samples(links_by_id, link_ids):
    """Flatten a directed MGeo link path while retaining link/point identity."""

    samples = []
    for link_id in link_ids:
        link = links_by_id[link_id]
        points = link["points"]
        for point_index, point in enumerate(points):
            row = {
                "x": float(point[0]),
                "y": float(point[1]),
                "z": float(point[2]) if len(point) >= 3 else 0.0,
                "link_idx": link_id,
                "point_idx": point_index,
                "point_ratio": point_index / max(1, len(points) - 1),
                "max_speed": float(link.get("max_speed", 30.0)),
            }
            if samples and math.hypot(
                samples[-1]["x"] - row["x"], samples[-1]["y"] - row["y"]
            ) < 1e-4:
                continue
            samples.append(row)
    arc = [0.0]
    for before, after in zip(samples, samples[1:]):
        arc.append(
            arc[-1]
            + math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        )
    return samples, arc


def reposition_spawn_on_link_path(
    spawn, links_by_id, link_ids, ego_anchor, offset_m
):
    """Move a cloned MORAI spawn along its existing legal route."""

    samples, arc = link_path_samples(links_by_id, link_ids)
    nearest = min(
        range(len(samples)),
        key=lambda index: math.hypot(
            samples[index]["x"] - ego_anchor["x"],
            samples[index]["y"] - ego_anchor["y"],
        ),
    )
    # Leave enough route after the spawn for the complete temporal clip.
    desired_arc = max(0.0, min(arc[-1] - 8.0, arc[nearest] + float(offset_m)))
    index = route_index_at_arc(arc, desired_arc)
    before = samples[max(0, index - 1)]
    after = samples[min(len(samples) - 1, index + 1)]
    pose = dict(samples[index])
    pose["yaw"] = math.degrees(
        math.atan2(after["y"] - before["y"], after["x"] - before["x"])
    )
    spawn["pos"] = formatted_vector(pose["x"], pose["y"], pose["z"])
    spawn["rot"] = formatted_rotation(pose["yaw"])
    spawn["startLinkInfo"] = {
        "linkIdx": pose["link_idx"],
        "pointIdx": int(pose["point_idx"]),
    }
    final_link = links_by_id[link_ids[-1]]
    spawn["endLinkInfo"] = {
        "linkIdx": link_ids[-1],
        "pointIdx": len(final_link["points"]) - 1,
    }
    return pose


def build_route_anchored_vehicle_scene(
    scenario,
    template,
    family,
    rng,
    variant_index,
    route,
    route_arc,
    samples,
    links_by_id,
):
    """Build a spawn-vehicle scene around an explicit desired capture anchor."""

    _target, runtime = build_vehicle_scene(
        scenario, template, family, rng, variant_index
    )
    zones = {str(item["name"]): item for item in family["ego_capture_zones"]}
    zone_name = weighted_zone_schedule(family)[variant_index]
    zone = zones[zone_name]
    lower, upper = [int(value) for value in zone["route_index_range"]]
    if lower < 1 or upper >= len(route) or upper < lower:
        raise ValueError("capture zone is outside the competition route")
    capture_index = rng.randint(lower, upper)
    capture_anchor = route_pose_at_index(route, capture_index)
    preroll = float(
        uniform_or_choice(
            rng,
            zone.get(
                "capture_preroll_distance_m",
                family["capture_preroll_distance_m"],
            ),
        )
    )
    start_arc = max(0.0, route_arc[capture_index] - preroll)
    start_index = route_index_at_arc(route_arc, start_arc)
    route_pose = route_pose_at_index(route, start_index)
    ego_sample = nearest_aligned_sample(samples, route_pose)
    ego_sample["route_index"] = start_index
    ego_speed = float(uniform_or_choice(rng, family["ego_target_speed_kph"]))
    if family.get("name") == "roundabout_circulation":
        npc_speed = float(runtime[0].get("target_speed_kph", ego_speed))
        ego_speed = min(
            ego_speed,
            npc_speed
            + float(family.get("maximum_ego_speed_over_npc_kph", 2.0)),
        )
    ego_initial_speed = min(
        ego_speed,
        float(uniform_or_choice(rng, family.get("ego_initial_speed_kph", 0.0))),
    )
    if family.get("start_ego_at_target_speed", False):
        ego_initial_speed = ego_speed
    set_ego_pose(scenario["egoVehicle"], ego_sample, ego_initial_speed)

    capture_overrides = {}
    npc_link_path = family.get("npc_link_path")
    if family.get("reposition_roundabout_spawn", False):
        npc_link_path = ROUNDABOUT_NPC_LINK_PATH
    if family.get("name") == "highway_merge_cruise":
        role_name = weighted_zone_schedule(
            family, key="interaction_roles"
        )[variant_index]
        roles = {
            str(item["name"]): item
            for item in family.get("interaction_roles", [])
        }
        role = roles[role_name]
        if family.get("preserve_template_npc_target_speed", False):
            npc_target_speed = float(runtime[0]["target_speed_kph"])
        elif role_name == "side_merge_traffic":
            npc_speed_factor = rng.uniform(0.90, 1.05)
            npc_target_speed = max(10.0, ego_speed * npc_speed_factor)
        else:
            npc_speed_factor = rng.uniform(0.75, 0.95)
            npc_target_speed = max(10.0, ego_speed * npc_speed_factor)
        spawn = scenario["spawnPointList"][0]
        set_spawn_target_speed(
            spawn, npc_target_speed, match_spawn_velocity=True
        )
        runtime[0]["target_speed_kph"] = npc_target_speed
        runtime[0]["spawn_speed_kph"] = npc_target_speed
        npc_link_path = HIGHWAY_LANE_LINK_PATHS[
            int(runtime[0]["spawn_unique_id"])
        ]
        offset_range = role["npc_longitudinal_offset_m"]
        selected_offset = rng.uniform(
            float(offset_range[0]), float(offset_range[1])
        )
        npc_pose = reposition_spawn_on_link_path(
            spawn,
            links_by_id,
            npc_link_path,
            capture_anchor,
            selected_offset,
        )
        npc_distance = math.hypot(
            npc_pose["x"] - capture_anchor["x"],
            npc_pose["y"] - capture_anchor["y"],
        )
        runtime[0].update(
            {
                "start_link": npc_pose["link_idx"],
                "end_link": npc_link_path[-1],
                "spawn_position": {
                    axis: float(npc_pose[axis]) for axis in ("x", "y", "z")
                },
                "spatial_phase": zone_name,
                "interaction_role": role_name,
                "planned_spawn_distance_m": float(npc_distance),
                "planned_spawn_offset_m": float(selected_offset),
            }
        )
        gate_range = role["gate_longitudinal_m"]
        capture_overrides = {
            "capture_actor_gate": {
                "class": "vehicle",
                "role": role_name,
                "minimum_longitudinal_m": float(gate_range[0]),
                "maximum_longitudinal_m": float(gate_range[1]),
                "maximum_abs_lateral_m": float(
                    role.get("maximum_abs_lateral_m", 15.0)
                ),
                "minimum_distance_m": float(
                    role.get("minimum_distance_m", 0.0)
                ),
                "maximum_distance_m": float(
                    role.get("maximum_distance_m", 55.0)
                ),
            },
            "prime_capture_before_clip": False,
            "preserve_ego_initial_speed_on_load": bool(
                family.get("start_ego_at_target_speed", False)
            ),
        }
        npc_link_path = None
    if npc_link_path:
        spawn = scenario["spawnPointList"][0]
        desired_distance = float(
            uniform_or_choice(
                rng, family.get("npc_relative_distance_m", [14.0, 28.0])
            )
        )
        offsets = family.get(
            "npc_phase_offset_candidates_m",
            [-28.0, -22.0, -16.0, 16.0, 22.0, 28.0],
        )
        minimum_spawn_distance = float(
            family.get("minimum_npc_spawn_distance_m", 10.0)
        )
        candidates = []
        for offset in offsets:
            candidate_spawn = copy.deepcopy(spawn)
            candidate_pose = reposition_spawn_on_link_path(
                candidate_spawn,
                links_by_id,
                npc_link_path,
                capture_anchor,
                float(offset),
            )
            distance = math.hypot(
                candidate_pose["x"] - capture_anchor["x"],
                candidate_pose["y"] - capture_anchor["y"],
            )
            candidates.append(
                (
                    distance < minimum_spawn_distance,
                    abs(distance - desired_distance),
                    rng.random(),
                    candidate_spawn,
                    candidate_pose,
                    distance,
                    float(offset),
                )
            )
        _, _, _, selected_spawn, npc_pose, npc_distance, selected_offset = min(
            candidates, key=lambda row: row[:3]
        )
        spawn.clear()
        spawn.update(selected_spawn)
        runtime[0].update(
            {
                "start_link": npc_pose["link_idx"],
                "end_link": npc_link_path[-1],
                "spawn_position": {
                    axis: float(npc_pose[axis]) for axis in ("x", "y", "z")
                },
                "spatial_phase": zone_name,
                "planned_ego_distance_m": float(desired_distance),
                "planned_spawn_distance_m": float(npc_distance),
                "planned_spawn_offset_m": float(selected_offset),
            }
        )

        if family.get("backup_roundabout_spawn", False):
            # Some relocated roundabout spawn positions are accepted by the
            # JSON loader but never produce a moving actor in MORAI.  Keep a
            # second legal spawn on a different phase of the same verified
            # lane path, rather than weakening the live actor requirement.
            spawn["maximumSpawnVehicle"] = 1
            backup_spawn = copy.deepcopy(spawn)
            backup_spawn["UNIQUEID"] = int(
                family.get("backup_roundabout_spawn_unique_id", 104)
            )
            backup_spawn["spawnedVehiclesCount"] = 0
            backup_spawn["maximumSpawnVehicle"] = 1
            backup_offset = float(
                family.get("backup_roundabout_spawn_offset_m", 28.0)
            )
            backup_pose = reposition_spawn_on_link_path(
                backup_spawn,
                links_by_id,
                npc_link_path,
                capture_anchor,
                backup_offset,
            )
            scenario["spawnPointList"].append(backup_spawn)
            backup_runtime = copy.deepcopy(runtime[0])
            backup_runtime.update(
                {
                    "spawn_unique_id": int(backup_spawn["UNIQUEID"]),
                    "start_link": backup_pose["link_idx"],
                    "spawn_position": {
                        axis: float(backup_pose[axis])
                        for axis in ("x", "y", "z")
                    },
                    "interaction_role": "roundabout_backup_cruising_vehicle",
                    "planned_spawn_offset_m": backup_offset,
                }
            )
            runtime.append(backup_runtime)

        configured_gate = family.get("capture_actor_gate")
        if configured_gate:
            capture_overrides.update(
                {
                    "capture_actor_gate": copy.deepcopy(configured_gate),
                    "prime_capture_before_clip": False,
                }
            )

    capture_overrides.update({
        "ego_target_speed_kph": ego_speed,
        "ego_initial_speed_kph": ego_initial_speed,
        "minimum_ego_speed_kph": ego_speed
        * float(family.get("minimum_ego_speed_ratio", 0.65)),
        "capture_motion_arm_m": preroll,
        "planned_capture_anchor_route_index": capture_index,
        "planned_capture_anchor": {
            axis: float(capture_anchor[axis]) for axis in ("x", "y", "z")
        },
        "spatial_phase": zone_name,
        "maximum_motion_m": float(family.get("maximum_motion_m", 90.0)),
        "motion_stall_timeout_sec": float(
            family.get("motion_stall_timeout_sec", 14.0)
        ),
        "lookahead_m": float(family.get("lookahead_m", 4.0)),
    })
    temporal_capture_count = int(
        family.get("captures_per_scene", 0)
    )
    desired_duration_sec = float(
        family.get("desired_clip_duration_sec", 0.0)
    )
    if temporal_capture_count > 1 and desired_duration_sec > 0.0:
        capture_overrides.update(
            {
                "captures_per_scene": temporal_capture_count,
                "capture_spacing_m": max(
                    0.5,
                    ego_speed
                    / 3.6
                    * desired_duration_sec
                    / (temporal_capture_count - 1),
                ),
                "desired_clip_duration_sec": desired_duration_sec,
            }
        )
    for policy_key in (
        "minimum_curve_speed_kph",
        "route_deviation_limit_m",
        "heading_error_limit_deg",
        "maximum_steering_rad",
        "load_settle_sec",
    ):
        if policy_key in family:
            capture_overrides[policy_key] = float(family[policy_key])
    return ego_sample, runtime, capture_overrides, capture_anchor


def ego_lane_speed_range(family, lane_id):
    configured = family.get("ego_lane_speed_kph", {})
    value = configured.get(str(lane_id), configured.get(lane_id))
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError("ego_lane_speed_kph values must be [min, max]")
    lower, upper = float(value[0]), float(value[1])
    if lower <= 0.0 or upper < lower:
        raise ValueError("invalid Ego lane speed range for {}".format(lane_id))
    return lower, upper


def build_vehicle_scene(scenario, template, family, rng, variant_index):
    spawn_by_id = {item["UNIQUEID"]: item for item in template["spawnPointList"]}
    source_ids = list(family["source_spawn_ids"])
    if family["name"] in SINGLE_LANE_SPAWN_FAMILIES:
        # Single-lane clips make velocity phases and failures attributable.
        source_ids = [source_ids[variant_index % len(source_ids)]]
    selected = [copy.deepcopy(spawn_by_id[item]) for item in source_ids]
    runtime = []
    for spawn in selected:
        base = float(spawn["MinDesiredVelocity_Custom"])
        target, period = configure_spawn(spawn, rng, family, base)
        runtime.append(
            {
                "spawn_unique_id": spawn["UNIQUEID"],
                "start_link": spawn["startLinkInfo"]["linkIdx"],
                "end_link": spawn["endLinkInfo"]["linkIdx"],
                "target_speed_kph": target,
                "spawn_speed_kph": float(
                    spawn.get("MinSpawnVelocity_Custom", 0.0)
                ),
                "spawn_period_sec": period,
                "motion_phase": family["motion_phase"],
            }
        )
    scenario["spawnPointList"] = selected
    return target_position_for_family(template, family["name"], selected), runtime


def route_from_link_path(links_by_id, link_ids):
    for link_id in link_ids:
        if link_id not in links_by_id:
            raise ValueError(f"unknown highway route link {link_id}")
    # Preserve link/point identity.  Phase-aware placement needs this metadata
    # to sample inside a specific entry/exit/taper link and to write the exact
    # MORAI initLink instead of re-projecting onto a nearby parallel lane.
    return link_path_samples(links_by_id, link_ids)


def route_arc_coordinates(route):
    arc = [0.0]
    for before, after in zip(route, route[1:]):
        arc.append(
            arc[-1]
            + math.hypot(
                after["x"] - before["x"],
                after["y"] - before["y"],
            )
        )
    return arc


def route_pose_at(route, arc, distance_m, link_id, links_by_id):
    index = min(
        range(len(arc)), key=lambda item: abs(arc[item] - float(distance_m))
    )
    before = route[max(0, index - 1)]
    after = route[min(len(route) - 1, index + 1)]
    yaw = math.degrees(
        math.atan2(after["y"] - before["y"], after["x"] - before["x"])
    )
    first_link = links_by_id[link_id]
    point_index = min(
        range(len(first_link["points"])),
        key=lambda item: math.hypot(
            float(first_link["points"][item][0]) - route[index]["x"],
            float(first_link["points"][item][1]) - route[index]["y"],
        ),
    )
    return {
        **route[index],
        "yaw": yaw,
        "route_index": index,
        "link_idx": link_id,
        "point_idx": point_index,
        "point_ratio": point_index / max(1, len(first_link["points"]) - 1),
        "max_speed": float(first_link.get("max_speed", 120.0)),
    }


def sample_route_at_arc(route, arc, distance_m):
    """Linearly sample a route at an arc-length coordinate."""

    distance_m = max(0.0, min(float(distance_m), float(arc[-1])))
    after_index = bisect.bisect_left(arc, distance_m)
    if after_index <= 0:
        before_index, after_index = 0, 1
    elif after_index >= len(route):
        before_index, after_index = len(route) - 2, len(route) - 1
    else:
        before_index = after_index - 1
    before = route[before_index]
    after = route[after_index]
    span = max(1e-9, arc[after_index] - arc[before_index])
    ratio = (distance_m - arc[before_index]) / span
    return {
        axis: float(before[axis]) + ratio * (
            float(after[axis]) - float(before[axis])
        )
        for axis in ("x", "y", "z")
    }


def smoothstep_quintic(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def estimated_accelerating_travel_time(
    distance_m, target_speed_kph, acceleration_mps2=1.7
):
    """Estimate travel time for MORAI Ego loaded stationary then accelerated."""

    distance_m = max(0.0, float(distance_m))
    target_speed_mps = max(0.1, float(target_speed_kph) / 3.6)
    acceleration_mps2 = max(0.1, float(acceleration_mps2))
    acceleration_time = target_speed_mps / acceleration_mps2
    acceleration_distance = 0.5 * acceleration_mps2 * acceleration_time ** 2
    if distance_m <= acceleration_distance:
        return math.sqrt(2.0 * distance_m / acceleration_mps2)
    return acceleration_time + (
        distance_m - acceleration_distance
    ) / target_speed_mps


def build_lane_change_route(
    source_route,
    source_arc,
    target_route,
    target_arc,
    start_arc_m,
    transition_length_m,
    pre_length_m=25.0,
    post_length_m=30.0,
    sample_spacing_m=0.5,
):
    """Blend adjacent MGeo centerlines into one smooth lane-change route."""

    source_start = max(0.0, float(start_arc_m) - float(pre_length_m))
    source_change_start = float(start_arc_m)
    source_change_end = source_change_start + float(transition_length_m)
    source_end = source_change_end + float(post_length_m)
    if source_end > source_arc[-1]:
        raise ValueError("source lane does not contain the requested maneuver")

    source_anchor = sample_route_at_arc(
        source_route, source_arc, source_change_start
    )
    target_change_start = min(
        target_arc,
        key=lambda value: math.hypot(
            sample_route_at_arc(target_route, target_arc, value)["x"]
            - source_anchor["x"],
            sample_route_at_arc(target_route, target_arc, value)["y"]
            - source_anchor["y"],
        ),
    )
    target_end = (
        float(target_change_start)
        + float(transition_length_m)
        + float(post_length_m)
    )
    if target_end > target_arc[-1]:
        raise ValueError("target lane does not contain the requested maneuver")

    total_length = pre_length_m + transition_length_m + post_length_m
    sample_count = int(math.ceil(total_length / sample_spacing_m)) + 1
    output = []
    for index in range(sample_count):
        local_arc = min(total_length, index * sample_spacing_m)
        source_distance = source_start + local_arc
        source = sample_route_at_arc(source_route, source_arc, source_distance)
        if local_arc <= pre_length_m:
            point = source
        elif local_arc < pre_length_m + transition_length_m:
            progress = (local_arc - pre_length_m) / transition_length_m
            target_distance = float(target_change_start) + (
                local_arc - pre_length_m
            )
            target = sample_route_at_arc(
                target_route, target_arc, target_distance
            )
            blend = smoothstep_quintic(progress)
            point = {
                axis: source[axis] + blend * (target[axis] - source[axis])
                for axis in ("x", "y", "z")
            }
        else:
            target_distance = float(target_change_start) + (
                local_arc - pre_length_m
            )
            point = sample_route_at_arc(
                target_route, target_arc, target_distance
            )
        if not output or math.hypot(
            point["x"] - output[-1]["x"],
            point["y"] - output[-1]["y"],
        ) > 1e-4:
            output.append(point)
    return output


def write_waypoint_route(path, route):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(
            "{:.9f} {:.9f} {:.9f}\n".format(
                point["x"], point["y"], point["z"]
            )
            for point in route
        ),
        encoding="utf-8",
    )


def build_highway_cruise_scene(
    scenario, template, family, rng, variant_index, links_by_id
):
    source_ids = list(family["source_spawn_ids"])
    ego_lane_id = weighted_lane_schedule(family)[variant_index]
    roles = {
        str(item["name"]): item
        for item in family.get("interaction_roles", [])
    }
    role_schedule = weighted_zone_schedule(
        family, key="interaction_roles"
    )
    role_name = role_schedule[
        (
            variant_index
            + int(family.get("interaction_role_schedule_offset", 0))
        )
        % len(role_schedule)
    ]
    role = roles[role_name]
    if role_name == "front_overtake" and ego_lane_id == min(source_ids):
        # The rightmost Ego lane has no slower lane on its right.  Keeping the
        # NPC in the same lane while Ego is deliberately faster turns the
        # capture controller into an unavoidable rear-end collision.  Move
        # these overtaking clips to the middle lane instead.
        overtaking_lanes = sorted(source_ids)[1:]
        ego_lane_id = overtaking_lanes[variant_index % len(overtaking_lanes)]
    if role_name == "side_parallel":
        adjacent = [lane for lane in source_ids if abs(lane - ego_lane_id) == 2]
        npc_spawn_id = adjacent[variant_index % len(adjacent)]
    elif role_name == "front_overtake":
        right_lanes = [lane for lane in source_ids if lane < ego_lane_id]
        npc_spawn_id = right_lanes[variant_index % len(right_lanes)]
    else:
        npc_spawn_id = ego_lane_id
    spawn = copy.deepcopy(
        next(
            item
            for item in template["spawnPointList"]
            if item["UNIQUEID"] == npc_spawn_id
        )
    )
    base_target = float(spawn["MinDesiredVelocity_Custom"])
    scale = rng.uniform(*family["npc_target_speed_scale"])
    target, period = configure_spawn(
        spawn,
        rng,
        {**family, "target_speed_kph": base_target * scale},
        base_target,
    )
    route_link_ids = HIGHWAY_LANE_LINK_PATHS[ego_lane_id]
    ego_route, ego_route_arc = route_from_link_path(links_by_id, route_link_ids)
    start_arc = rng.uniform(*family["ego_start_arc_m"])
    ego_sample = route_pose_at(
        ego_route, ego_route_arc, start_arc, route_link_ids[0], links_by_id
    )
    same_lane = ego_lane_id == npc_spawn_id
    lane_speed_range = ego_lane_speed_range(family, ego_lane_id)
    if role_name == "front_following":
        ego_speed = max(
            10.0,
            target
            + rng.uniform(*role.get("ego_speed_delta_kph", [-3.0, 3.0])),
        )
        relative_lane_intent = "same_lane_following"
    elif role_name in {"front_overtake", "rear_receding"}:
        desired = target + rng.uniform(*family["ego_speed_advantage_kph"])
        if lane_speed_range is not None:
            ego_speed = max(
                lane_speed_range[0], min(lane_speed_range[1], desired)
            )
        else:
            ego_speed = min(float(family["ego_speed_limit_kph"]), desired)
        relative_lane_intent = role_name
    elif role_name == "rear_approach":
        target += rng.uniform(
            *role.get("npc_speed_advantage_kph", [5.0, 10.0])
        )
        set_spawn_target_speed(spawn, target, match_spawn_velocity=True)
        if lane_speed_range is not None:
            ego_speed = rng.uniform(*lane_speed_range)
        else:
            ego_speed = max(10.0, target - 7.0)
        relative_lane_intent = "rear_approach"
    elif role_name == "side_parallel":
        if lane_speed_range is not None:
            ego_speed = rng.uniform(*lane_speed_range)
        else:
            ego_speed = target
        if family.get("preserve_template_npc_target_speed", False):
            target = base_target
        else:
            target = max(
                10.0,
                ego_speed
                + rng.uniform(*role.get("npc_speed_delta_kph", [-5.0, 5.0])),
            )
        set_spawn_target_speed(spawn, target, match_spawn_velocity=True)
        relative_lane_intent = "side_parallel"
    elif same_lane:
        if lane_speed_range is not None:
            target = max(
                lane_speed_range[0], min(lane_speed_range[1], target)
            )
            set_spawn_target_speed(
                spawn,
                target,
                match_spawn_velocity=family.get(
                    "spawn_velocity_equals_target", False
                ),
            )
        ego_speed = target
        relative_lane_intent = "same_lane_following"
    elif ego_lane_id > npc_spawn_id:
        # Higher lane IDs are farther left in the southbound competition
        # highway.  Only a left-side Ego is assigned an overtaking speed
        # advantage; this prevents systematic right-side passing scenes.
        desired = target + rng.uniform(*family["ego_speed_advantage_kph"])
        if lane_speed_range is not None:
            ego_speed = max(
                lane_speed_range[0], min(lane_speed_range[1], desired)
            )
        else:
            ego_speed = min(float(family["ego_speed_limit_kph"]), desired)
        relative_lane_intent = "left_side_overtake"
    else:
        if lane_speed_range is not None:
            sampled = rng.uniform(*lane_speed_range)
            ego_speed = max(lane_speed_range[0], min(sampled, target))
        else:
            ego_speed = min(float(family["ego_speed_limit_kph"]), target)
        relative_lane_intent = "right_side_parallel_or_following"
    speed_schedule = family.get("ego_speed_schedule_kph")
    if speed_schedule:
        ego_speed = float(speed_schedule[variant_index % len(speed_schedule)])
    if family.get("start_ego_at_target_speed", False):
        initial_speed = ego_speed
    else:
        initial_speed = float(
            uniform_or_choice(
                rng,
                family.get("ego_initial_speed_kph", min(ego_speed, 15.0)),
            )
        )
        initial_speed = min(initial_speed, ego_speed)
    if role_name in {"rear_approach", "rear_receding"}:
        # A single behind-Ego spawn can leave its short route during
        # ScenarioLoad verification before capture begins.  Permit one
        # replacement only for these rear roles.
        spawn["maximumSpawnVehicle"] = 2
        spawn["minSpawnPeriod"] = 4.0
        spawn["maxSpawnPeriod"] = 5.0
    set_ego_pose(scenario["egoVehicle"], ego_sample, initial_speed)
    capture_arm_m = float(family.get("capture_motion_arm_m", 3.0))
    # NPC motion begins at ScenarioLoad, while Ego deliberately starts from
    # rest so external control can take over safely. Back-project the NPC by
    # its expected pre-gate travel; otherwise it is already 100+ m too far
    # ahead when a zero-speed Ego finally reaches the capture gate.
    compensate_npc_motion = bool(
        family.get("compensate_npc_motion_until_capture_gate", False)
    )
    if compensate_npc_motion:
        expected_accel = max(
            0.5, float(family.get("expected_ego_acceleration_mps2", 1.8))
        )
        minimum_capture_speed_mps = (
            ego_speed * float(family["minimum_ego_speed_ratio"]) / 3.6
        )
        acceleration_time_sec = minimum_capture_speed_mps / expected_accel
        acceleration_distance_m = (
            0.5 * expected_accel * acceleration_time_sec * acceleration_time_sec
        )
        expected_gate_distance_m = max(capture_arm_m, acceleration_distance_m)
        stable_gate_sec = (
            float(family.get("stable_ego_window_sec", 0.5))
            if family.get("stable_ego_gate", False)
            else 0.0
        )
        expected_gate_time_sec = max(
            math.sqrt(2.0 * capture_arm_m / expected_accel),
            acceleration_time_sec,
        ) + stable_gate_sec
    else:
        expected_gate_distance_m = capture_arm_m
        expected_gate_time_sec = 0.0
    capture_anchor = sample_route_at_arc(
        ego_route, ego_route_arc, start_arc + expected_gate_distance_m
    )
    offset_range = role.get("npc_longitudinal_offset_m", [15.0, 30.0])
    offset_m = rng.uniform(float(offset_range[0]), float(offset_range[1]))
    npc_pre_gate_motion_m = target / 3.6 * expected_gate_time_sec
    load_time_offset_m = offset_m - npc_pre_gate_motion_m
    npc_pose = reposition_spawn_on_link_path(
        spawn,
        links_by_id,
        HIGHWAY_LANE_LINK_PATHS[npc_spawn_id],
        capture_anchor,
        load_time_offset_m,
    )
    scenario["spawnPointList"] = [spawn]
    gate_range = role.get("gate_longitudinal_m", [-40.0, 40.0])
    actor_gate = {
        "class": "vehicle",
        # The exact planned relative role remains metadata.  MORAI can advance
        # or recycle a relocated spawn during ScenarioLoad, so the live gate
        # accepts any nearby moving vehicle instead of waiting 130 m for an
        # exact sign/range that may no longer exist.
        "role": "any_moving_vehicle",
        "planned_role": role_name,
        "minimum_longitudinal_m": -65.0,
        "maximum_longitudinal_m": 65.0,
        "maximum_abs_lateral_m": float(
            max(8.0, role.get("maximum_abs_lateral_m", 6.5))
        ),
        "maximum_distance_m": 65.0,
        "minimum_actor_speed_mps": 0.5,
    }
    runtime = [
        {
            "spawn_unique_id": npc_spawn_id,
            "start_link": spawn["startLinkInfo"]["linkIdx"],
            "end_link": spawn["endLinkInfo"]["linkIdx"],
            "spawn_speed_kph": target,
            "target_speed_kph": target,
            "spawn_period_sec": period,
            "ego_lane_spawn_id": ego_lane_id,
            "ego_lane_link": route_link_ids[0],
            "ego_target_speed_kph": ego_speed,
            "ego_initial_speed_kph": initial_speed,
            "same_lane": same_lane,
            "relative_lane_intent": relative_lane_intent,
            "interaction_role": role_name,
            "planned_longitudinal_offset_m": offset_m,
            "load_time_longitudinal_offset_m": load_time_offset_m,
            "expected_capture_gate_distance_m": expected_gate_distance_m,
            "expected_capture_gate_time_sec": expected_gate_time_sec,
            "npc_pre_gate_motion_compensation_m": npc_pre_gate_motion_m,
            "npc_motion_compensation_enabled": compensate_npc_motion,
            "spawn_position": {
                axis: float(npc_pose[axis]) for axis in ("x", "y", "z")
            },
        }
    ]
    capture_overrides = {
        "ego_target_speed_kph": ego_speed,
        "minimum_ego_speed_kph": (
            ego_speed * float(family["minimum_ego_speed_ratio"])
        ),
        "maximum_motion_m": float(family.get("maximum_motion_m", 130.0)),
        "motion_stall_timeout_sec": float(
            family.get("motion_stall_timeout_sec", 12.0)
        ),
        "lookahead_m": 6.0,
        "adaptive_curve_control": False,
        "capture_motion_arm_m": capture_arm_m,
        "prime_capture_before_clip": False,
        "preserve_ego_initial_speed_on_load": bool(
            family.get("start_ego_at_target_speed", False)
        ),
        "capture_actor_gate": actor_gate,
    }
    temporal_capture_count = int(
        family.get("captures_per_scene", 0)
    )
    desired_duration_sec = float(
        family.get("desired_clip_duration_sec", 0.0)
    )
    if temporal_capture_count > 1 and desired_duration_sec > 0.0:
        capture_overrides.update(
            {
                "captures_per_scene": temporal_capture_count,
                "capture_spacing_m": max(
                    0.5,
                    ego_speed
                    / 3.6
                    * desired_duration_sec
                    / (temporal_capture_count - 1),
                ),
                "desired_clip_duration_sec": desired_duration_sec,
            }
        )
    if ego_speed > 80.0:
        capture_overrides["allow_high_speed_capture"] = True
    if "load_settle_sec" in family:
        capture_overrides["load_settle_sec"] = float(
            family["load_settle_sec"]
        )
    return ego_route, ego_sample, runtime, capture_overrides


def place_spawn_on_competition_route(
    spawn,
    route,
    route_arc,
    samples,
    anchor_index,
    offset_m,
    remaining_route_m=120.0,
):
    """Place a MORAI spawn on a route-aligned MGeo link without a fixed mission anchor."""

    desired_arc = max(
        0.0,
        min(route_arc[-1] - 12.0, route_arc[anchor_index] + float(offset_m)),
    )
    route_index = route_index_at_arc(route_arc, desired_arc)
    pose = route_pose_at_index(route, route_index)
    aligned = nearest_aligned_sample(samples, pose)
    end_index = route_index_at_arc(
        route_arc,
        min(route_arc[-1], desired_arc + float(remaining_route_m)),
    )
    end_pose = route_pose_at_index(route, end_index)
    end_sample = nearest_aligned_sample(samples, end_pose)
    spawn["pos"] = formatted_vector(
        aligned["x"], aligned["y"], aligned["z"]
    )
    spawn["rot"] = formatted_rotation(aligned["yaw"])
    spawn["startLinkInfo"] = {
        "linkIdx": aligned["link_idx"],
        "pointIdx": int(aligned["point_idx"]),
    }
    spawn["endLinkInfo"] = {
        "linkIdx": end_sample["link_idx"],
        "pointIdx": int(end_sample["point_idx"]),
    }
    return {
        **aligned,
        "route_index": route_index,
        "end_route_index": end_index,
        "end_link_idx": end_sample["link_idx"],
        "end_point_idx": int(end_sample["point_idx"]),
    }


def build_east_corridor_relative_vehicle_scene(
    scenario,
    template,
    family,
    rng,
    variant_index,
    route,
    route_arc,
    samples,
):
    """Build one legal lane-following vehicle interaction outside mission anchors."""

    zones = {str(item["name"]): item for item in family["ego_capture_zones"]}
    # Select space and motion jointly.  Independent schedules can make a
    # spatial split an unintended proxy for one interaction role.
    zone_name, role_name = weighted_pair_schedule(family)[variant_index]
    zone = zones[zone_name]
    lower, upper = [int(value) for value in zone["route_index_range"]]
    capture_index = rng.randint(lower, upper)
    capture_anchor = route_pose_at_index(route, capture_index)
    preroll = float(uniform_or_choice(rng, family["capture_preroll_distance_m"]))
    start_index = route_index_at_arc(
        route_arc, max(0.0, route_arc[capture_index] - preroll)
    )
    ego_pose = route_pose_at_index(route, start_index)
    ego_sample = nearest_aligned_sample(samples, ego_pose)
    ego_sample["route_index"] = start_index
    ego_speed = float(uniform_or_choice(rng, family["ego_target_speed_kph"]))
    ego_initial_speed = (
        ego_speed
        if family.get("start_ego_at_target_speed", False)
        else 0.0
    )
    set_ego_pose(scenario["egoVehicle"], ego_sample, ego_initial_speed)

    roles = {
        str(item["name"]): item for item in family["interaction_roles"]
    }
    role = roles[role_name]
    offset_m = rng.uniform(*[float(value) for value in role["offset_m"]])
    speed_scale = rng.uniform(*[float(value) for value in role["speed_scale"]])
    npc_speed = max(8.0, min(60.0, ego_speed * speed_scale))

    source_ids = list(family.get("source_spawn_ids", [4, 6, 8, 10]))
    source_id = source_ids[variant_index % len(source_ids)]
    spawn = copy.deepcopy(
        next(
            item
            for item in template["spawnPointList"]
            if int(item["UNIQUEID"]) == int(source_id)
        )
    )
    configure_spawn(
        spawn,
        rng,
        {
            **family,
            "target_speed_kph": npc_speed,
        },
        npc_speed,
    )
    set_spawn_target_speed(
        spawn,
        npc_speed,
        match_spawn_velocity=not bool(role.get("spawn_from_rest", False)),
    )
    npc_pose = place_spawn_on_competition_route(
        spawn,
        route,
        route_arc,
        samples,
        capture_index,
        offset_m,
        remaining_route_m=float(family.get("npc_remaining_route_m", 120.0)),
    )
    backup_spawn = None
    backup_pose = None
    backup_speed = None
    if family.get("structured_backup_front", False):
        backup_source_id = source_ids[(variant_index + 1) % len(source_ids)]
        backup_spawn = copy.deepcopy(
            next(
                item
                for item in template["spawnPointList"]
                if int(item["UNIQUEID"]) == int(backup_source_id)
            )
        )
        backup_speed = max(
            npc_speed,
            ego_speed
            * float(family.get("backup_speed_scale", 1.05)),
        )
        configure_spawn(
            backup_spawn,
            rng,
            {
                **family,
                "target_speed_kph": backup_speed,
            },
            backup_speed,
        )
        set_spawn_target_speed(
            backup_spawn, backup_speed, match_spawn_velocity=True
        )
        backup_pose = place_spawn_on_competition_route(
            backup_spawn,
            route,
            route_arc,
            samples,
            capture_index,
            float(family.get("backup_front_offset_m", 45.0)),
            remaining_route_m=float(
                family.get("npc_remaining_route_m", 120.0)
            ),
        )
    scenario["spawnPointList"] = [spawn] + (
        [backup_spawn] if backup_spawn is not None else []
    )

    gate = role["gate_longitudinal_m"]
    if family.get("structured_backup_front", False):
        # Planned roles remain useful generation metadata, but a relocated
        # MORAI spawn can advance or recycle before Ego reaches it.  With the
        # redundant spawn enabled, accept any genuinely moving nearby vehicle
        # and record its observed relative state in every frame.  Requiring
        # only the originally planned sign made otherwise valid clips fail.
        gate = family.get("structured_observed_gate_longitudinal_m", [-55.0, 55.0])
        gate_role = "any_moving_vehicle"
    else:
        gate_role = role_name
    capture_overrides = {
        "ego_target_speed_kph": ego_speed,
        "ego_initial_speed_kph": ego_initial_speed,
        "minimum_ego_speed_kph": ego_speed
        * float(family.get("minimum_ego_speed_ratio", 0.70)),
        "capture_motion_arm_m": preroll,
        "maximum_motion_m": float(family.get("maximum_motion_m", 100.0)),
        "motion_stall_timeout_sec": float(
            family.get("motion_stall_timeout_sec", 14.0)
        ),
        "lookahead_m": float(family.get("lookahead_m", 4.0)),
        "prime_capture_before_clip": False,
        "preserve_ego_initial_speed_on_load": bool(
            family.get("start_ego_at_target_speed", False)
        ),
        "planned_capture_anchor_route_index": capture_index,
        "planned_capture_anchor": {
            axis: float(capture_anchor[axis]) for axis in ("x", "y", "z")
        },
        "spatial_phase": zone_name,
        "spatial_split": zone.get("split"),
        "dynamic_actor_ready_requires_motion": not bool(
            role.get("spawn_from_rest", False)
        ),
        "capture_actor_gate": {
            "class": "vehicle",
            "role": gate_role,
            "planned_role": role_name,
            "minimum_longitudinal_m": float(gate[0]),
            "maximum_longitudinal_m": float(gate[1]),
            "maximum_abs_lateral_m": float(
                role.get("maximum_abs_lateral_m", 6.0)
            ),
            "minimum_distance_m": float(role.get("minimum_distance_m", 6.0)),
            "maximum_distance_m": float(role.get("maximum_distance_m", 55.0)),
            # Do not silently turn the dynamic quota into another stopped-car
            # clip when MORAI halts a spawned NPC at a signal or bad branch.
            "minimum_actor_speed_mps": float(
                role.get("minimum_actor_speed_mps", 0.5)
            ),
        },
    }
    runtime = [
        {
            "spawn_unique_id": int(spawn["UNIQUEID"]),
            "unique_id": None,
            "data_id": None,
            "actor_placement_mode": "spawn_point",
            "start_link": npc_pose["link_idx"],
            "end_link": npc_pose["end_link_idx"],
            "spawn_speed_kph": npc_speed,
            "target_speed_kph": npc_speed,
            "spawn_period_sec": float(spawn["minSpawnPeriod"]),
            "motion_phase": family["motion_phase"],
            "spatial_phase": zone_name,
            "interaction_role": role_name,
            "planned_longitudinal_offset_m": offset_m,
            "ego_target_speed_kph": ego_speed,
            "spawn_position": {
                axis: float(npc_pose[axis]) for axis in ("x", "y", "z")
            },
        }
    ]
    if backup_spawn is not None:
        runtime.append(
            {
                "spawn_unique_id": int(backup_spawn["UNIQUEID"]),
                "unique_id": None,
                "data_id": None,
                "actor_placement_mode": "spawn_point",
                "start_link": backup_pose["link_idx"],
                "end_link": backup_pose["end_link_idx"],
                "spawn_speed_kph": backup_speed,
                "target_speed_kph": backup_speed,
                "spawn_period_sec": float(backup_spawn["minSpawnPeriod"]),
                "motion_phase": family["motion_phase"],
                "spatial_phase": zone_name,
                "interaction_role": "backup_front_cruise",
                "planned_longitudinal_offset_m": float(
                    family.get("backup_front_offset_m", 45.0)
                ),
                "ego_target_speed_kph": ego_speed,
                "spawn_position": {
                    axis: float(backup_pose[axis])
                    for axis in ("x", "y", "z")
                },
            }
        )
    return ego_sample, runtime, capture_overrides, capture_anchor


def build_highway_lane_change_scene(
    scenario, template, family, rng, variant_index, links_by_id
):
    lane_pairs = [tuple(item) for item in family["lane_change_pairs"]]
    source_lane, target_lane = lane_pairs[variant_index % len(lane_pairs)]
    source_ids = HIGHWAY_LANE_LINK_PATHS[source_lane]
    target_ids = HIGHWAY_LANE_LINK_PATHS[target_lane]
    source_route, source_arc = route_from_link_path(links_by_id, source_ids)
    target_route, target_arc = route_from_link_path(links_by_id, target_ids)

    target_lane_speed_range = ego_lane_speed_range(family, target_lane)
    if target_lane_speed_range is None:
        ego_speed = float(uniform_or_choice(rng, family["ego_target_speed_kph"]))
    else:
        ego_speed = rng.uniform(*target_lane_speed_range)
    transition_length = max(
        float(family.get("minimum_transition_length_m", 35.0)),
        min(
            float(family.get("maximum_transition_length_m", 75.0)),
            ego_speed / 3.6 * float(family.get("transition_duration_sec", 4.0)),
        ),
    )
    start_arc = float(uniform_or_choice(rng, family["lane_change_start_arc_m"]))
    pre_length = float(uniform_or_choice(rng, family["pre_length_m"]))
    post_length = float(uniform_or_choice(rng, family["post_length_m"]))
    ego_route = build_lane_change_route(
        source_route,
        source_arc,
        target_route,
        target_arc,
        start_arc,
        transition_length,
        pre_length,
        post_length,
    )
    ego_sample = route_pose_at(
        ego_route,
        route_arc_coordinates(ego_route),
        0.0,
        source_ids[0],
        links_by_id,
    )
    initial_speed = min(
        ego_speed,
        float(uniform_or_choice(rng, family["ego_initial_speed_kph"])),
    )
    set_ego_pose(scenario["egoVehicle"], ego_sample, initial_speed)
    phase_names = ("lane_change_entry", "lane_change_middle", "lane_change_exit")
    phase_progress = (0.0, 0.45, 0.85)
    phase_index = variant_index % len(phase_names)

    spawn_by_id = {
        item["UNIQUEID"]: item for item in template["spawnPointList"]
    }
    roles = {
        str(item["name"]): item
        for item in family.get("interaction_roles", [])
    }
    role_name = weighted_zone_schedule(
        family, key="interaction_roles"
    )[variant_index]
    role = roles[role_name]
    npc_lane = target_lane if role_name != "rear_source_lane" else source_lane
    spawn = copy.deepcopy(spawn_by_id[npc_lane])
    target_speed = max(
        10.0,
        ego_speed
        - float(
            uniform_or_choice(
                rng,
                family.get(
                    "lane_change_ego_speed_advantage_kph", [2.0, 5.0]
                ),
            )
        ),
    )
    target_speed, period = configure_spawn(
        spawn,
        rng,
        {**family, "target_speed_kph": target_speed},
        target_speed,
    )
    capture_arc = pre_length + transition_length * phase_progress[phase_index]
    capture_anchor = sample_route_at_arc(
        ego_route, route_arc_coordinates(ego_route), capture_arc
    )
    desired_capture_ranges = role.get("desired_capture_longitudinal_ranges_m")
    if desired_capture_ranges:
        # A vehicle centred beside Ego while Ego crosses into its lane is a
        # deterministic collision, not a useful lane-change observation.
        # Alternate the side-target clips between a safe rear and front gap.
        role_schedule = weighted_zone_schedule(
            family, key="interaction_roles"
        )
        role_occurrence = sum(
            scheduled_role == role_name
            for scheduled_role in role_schedule[:variant_index]
        )
        desired_capture_range = desired_capture_ranges[
            role_occurrence % len(desired_capture_ranges)
        ]
    else:
        desired_capture_range = role.get(
            "desired_capture_longitudinal_m",
            role.get("gate_longitudinal_m", [10.0, 25.0]),
        )
    desired_capture_longitudinal = rng.uniform(
        float(desired_capture_range[0]),
        float(desired_capture_range[1]),
    )
    # Place the NPC so that constant-speed motion brings it to the desired
    # relative location when Ego reaches this clip's lane-change phase.  The
    # old fixed per-lane offsets overcompensated by up to ~100 m and caused
    # both missing actors and deterministic collisions.
    estimated_capture_time_sec = estimated_accelerating_travel_time(
        capture_arc,
        ego_speed,
        acceleration_mps2=float(
            family.get("estimated_ego_acceleration_mps2", 1.7)
        ),
    )
    offset_m = desired_capture_longitudinal - (
        target_speed / 3.6 * estimated_capture_time_sec
    )
    npc_pose = reposition_spawn_on_link_path(
        spawn,
        links_by_id,
        HIGHWAY_LANE_LINK_PATHS[npc_lane],
        capture_anchor,
        offset_m,
    )
    scenario["spawnPointList"] = [spawn]
    runtime = [{
        "spawn_unique_id": npc_lane,
        "target_speed_kph": target_speed,
        "spawn_period_sec": period,
        "source_ego_lane_spawn_id": source_lane,
        "target_ego_lane_spawn_id": target_lane,
        "lane_change_direction": "left" if target_lane > source_lane else "right",
        "lane_change_start_arc_m": start_arc,
        "lane_change_transition_length_m": transition_length,
        "pre_length_m": pre_length,
        "post_length_m": post_length,
        "capture_phase": phase_names[phase_index],
        "capture_phase_progress": phase_progress[phase_index],
        "interaction_role": role_name,
        "planned_longitudinal_offset_m": offset_m,
        "desired_capture_longitudinal_m": desired_capture_longitudinal,
        "estimated_capture_time_sec": estimated_capture_time_sec,
        "spawn_position": {
            axis: float(npc_pose[axis]) for axis in ("x", "y", "z")
        },
    }]
    capture_overrides = {
        "ego_target_speed_kph": ego_speed,
        "minimum_ego_speed_kph": ego_speed
        * float(family.get("minimum_ego_speed_ratio", 0.85)),
        "capture_motion_arm_m": pre_length
        + transition_length * phase_progress[phase_index],
        "minimum_capture_progress_m": float(
            family.get("minimum_capture_progress_m", 0.3)
        ),
        "maximum_motion_m": pre_length + transition_length + post_length + 10.0,
        "motion_stall_timeout_sec": 15.0,
        "route_deviation_limit_m": float(
            family.get("route_deviation_limit_m", 1.2)
        ),
        "lookahead_m": 6.0,
        "adaptive_curve_control": True,
        "minimum_curve_speed_kph": float(
            family.get("minimum_curve_speed_kph", 30.0)
        ),
        "clip_motion_phase": phase_names[phase_index],
        "prime_capture_before_clip": False,
        "capture_actor_gate": {
            "class": "vehicle",
            "role": "any_moving_vehicle",
            "planned_role": role_name,
            "minimum_longitudinal_m": -55.0,
            "maximum_longitudinal_m": 55.0,
            "maximum_abs_lateral_m": float(
                role.get("maximum_abs_lateral_m", 6.5)
            ),
            "maximum_distance_m": 60.0,
            "minimum_actor_speed_mps": 0.5,
        },
    }
    return ego_route, ego_sample, runtime, capture_overrides


def build_static_highway_scene(
    scenario,
    family,
    rng,
    variant_index,
    links_by_id,
    obstacle_pool,
    obstacle_pool_offset=0,
):
    if family.get("ego_lane_weights"):
        lane_schedule = weighted_lane_schedule({
            **family,
            "source_spawn_ids": list(HIGHWAY_LANE_LINK_PATHS),
        })
        lane_id = lane_schedule[int(variant_index) % len(lane_schedule)]
    else:
        lane_id = static_highway_lane_for_variant(
            variant_index, int(family.get("count", 1))
        )
    route_ids = HIGHWAY_LANE_LINK_PATHS[lane_id]
    ego_route, ego_arc = route_from_link_path(links_by_id, route_ids)
    if "ego_start_fraction" in family:
        lower, upper = family["ego_start_fraction"]
        count = max(1, int(family.get("count", 1)))
        quantile = (int(variant_index) + rng.uniform(0.15, 0.85)) / count
        fraction = float(lower) + (float(upper) - float(lower)) * quantile
        start_arc = max(20.0, min(ego_arc[-1] - 70.0, ego_arc[-1] * fraction))
    else:
        start_arc = float(uniform_or_choice(rng, family["ego_start_arc_m"]))
    ego_sample = route_pose_at(
        ego_route, ego_arc, start_arc, route_ids[0], links_by_id
    )
    ego_speed = float(uniform_or_choice(rng, family["ego_target_speed_kph"]))
    initial_speed = min(
        ego_speed,
        float(uniform_or_choice(rng, family["ego_initial_speed_kph"])),
    )
    set_ego_pose(scenario["egoVehicle"], ego_sample, initial_speed)
    scenario["objectList"] = []
    runtime = []
    count = int(family.get("objects_per_scene", 5))
    for index in range(count):
        row = obstacle_pool[(obstacle_pool_offset + index) % len(obstacle_pool)]
        actor = copy.deepcopy(row["template"])
        route_point = sample_route_at_arc(
            ego_route,
            ego_arc,
            start_arc + 18.0 + index * 7.0,
        )
        next_point = sample_route_at_arc(
            ego_route,
            ego_arc,
            start_arc + 19.0 + index * 7.0,
        )
        yaw = math.degrees(
            math.atan2(
                next_point["y"] - route_point["y"],
                next_point["x"] - route_point["x"],
            )
        )
        lateral = (-1.0 if index % 2 == 0 else 1.0) * rng.uniform(4.0, 6.0)
        position = obstacle_local_to_world(route_point, yaw, 0.0, lateral)
        actor["UNIQUEID"] = 3001 + index
        set_actor_world_position(actor, position, yaw + rng.uniform(-15.0, 15.0))
        actor["moveSpeed"] = 0.0
        actor["isWaypointMode"] = False
        actor["activeDistance"] = 0.0
        scenario["objectList"].append(actor)
        runtime.append({
            "unique_id": 3001 + index,
            "data_id": int(row["data_id"]),
            "name": row.get("name"),
            "subtype": row.get("subtype"),
            "motion": "static",
            "class": "obstacle",
            "position": position,
            "yaw_deg": yaw,
            "ego_lane_spawn_id": lane_id,
        })
    capture_overrides = {
        "ego_target_speed_kph": ego_speed,
        "minimum_ego_speed_kph": ego_speed
        * float(family.get("minimum_ego_speed_ratio", 0.85)),
        "capture_motion_arm_m": float(family.get("capture_motion_arm_m", 10.0)),
        "minimum_capture_progress_m": float(
            family.get("minimum_capture_progress_m", 0.3)
        ),
        "maximum_motion_m": 110.0,
        "motion_stall_timeout_sec": 15.0,
        "lookahead_m": 6.0,
        "adaptive_curve_control": False,
    }
    return ego_route, ego_sample, runtime, capture_overrides


def static_highway_lane_for_variant(variant_index, family_count):
    family_count = max(1, int(family_count))
    quantile = (int(variant_index) + 0.5) / family_count
    return 6 if quantile < 0.60 else (8 if quantile < 0.85 else 10)


def crosswalk_path_endpoints(crosswalk, margin_m=4.0):
    points = crosswalk["points"]
    center = {
        axis: sum(point[axis] for point in points) / len(points)
        for axis in ("x", "y", "z")
    }
    xx = yy = xy = 0.0
    for point in points:
        dx = point["x"] - center["x"]
        dy = point["y"] - center["y"]
        xx += dx * dx
        yy += dy * dy
        xy += dx * dy
    axis = crosswalk.get("pedestrian_axis")
    if axis is None:
        angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
        axis = (math.cos(angle), math.sin(angle))
    projections = [
        (point["x"] - center["x"]) * axis[0]
        + (point["y"] - center["y"]) * axis[1]
        for point in points
    ]
    polygon_low = min(projections)
    polygon_high = max(projections)
    path_center_offset = (polygon_low + polygon_high) / 2.0
    center = {
        "x": center["x"] + axis[0] * path_center_offset,
        "y": center["y"] + axis[1] * path_center_offset,
        "z": center["z"],
    }
    low = (polygon_low - path_center_offset) - float(margin_m)
    high = (polygon_high - path_center_offset) + float(margin_m)
    start = {
        "x": center["x"] + axis[0] * low,
        "y": center["y"] + axis[1] * low,
        "z": center["z"],
    }
    end = {
        "x": center["x"] + axis[0] * high,
        "y": center["y"] + axis[1] * high,
        "z": center["z"],
    }
    return center, start, end


def set_pedestrian_waypoint_path(path, start, end):
    prototype = copy.deepcopy(path["waypointData"][0])
    path["waypointData"] = []
    for position in (start, end):
        waypoint = copy.deepcopy(prototype)
        waypoint["pos"] = world_to_morai_waypoint(position)
        path["waypointData"].append(waypoint)


def build_pedestrian_scene(
    scenario,
    template,
    family,
    rng,
    variant_index,
    crosswalk_catalog,
    pedestrian_pool=None,
    pedestrian_pool_offset=0,
):
    asset = (
        pedestrian_pool[pedestrian_pool_offset % len(pedestrian_pool)]
        if pedestrian_pool
        else None
    )
    official_actor = template["pedestrianList"][0]
    actor = copy.deepcopy(official_actor)
    if asset:
        # PedPath behavior in MORAI is tied to the verified official actor
        # structure (notably UNIQUEID and waypointEventList).  Replacing the
        # complete catalog actor silently loads a visible but stationary
        # pedestrian.  Change only the visual/animation identity while
        # preserving the proven waypoint controller fields.
        asset_actor = asset["template"]
        actor["DataID"] = asset_actor["DataID"]
        for key in (
            "anim",
            "stateNameHash",
            "stateNormalizedTime",
            "pedestrianType",
        ):
            if key in asset_actor:
                actor[key] = copy.deepcopy(asset_actor[key])
    path = copy.deepcopy(
        next(
            item
            for item in template["waypointDataList"]
            if item.get("pathName") == "PedPath - 3"
        )
    )
    family_count = int(family.get("count", 1))
    locations = family.get("crosswalk_locations") or [
        {
            "crosswalk_id": "CW000000",
            "weight": 1.0,
            "role": "official_sample_direct",
            "use_official_path": True,
        }
    ]
    if (
        family.get("cover_all_crosswalks_when_possible", False)
        and family_count <= len(locations)
    ):
        # A small smoke set is for geometry/visibility coverage rather than
        # production-frequency matching. Exercise each configured location
        # once before the weighted production quota is trusted.
        location = locations[variant_index % len(locations)]
    else:
        location = weighted_variant_choice(
            locations,
            variant_index,
            family_count,
        )
    crosswalk_id = location["crosswalk_id"]
    if crosswalk_id not in crosswalk_catalog:
        raise ValueError("unknown MGeo crosswalk {}".format(crosswalk_id))
    crosswalk = crosswalk_catalog[crosswalk_id]
    if location.get("use_official_path", False):
        start = waypoint_to_world(path["waypointData"][0]["pos"])
        end = waypoint_to_world(path["waypointData"][-1]["pos"])
        target = {
            axis: (start[axis] + end[axis]) / 2.0
            for axis in ("x", "y", "z")
        }
    else:
        target, start, end = crosswalk_path_endpoints(
            crosswalk,
            family.get("crosswalk_path_margin_m", 4.0),
        )
        set_pedestrian_waypoint_path(path, start, end)

    profile_index = (variant_index * 7 + 1) % max(1, family_count)
    profile = weighted_variant_choice(
        family.get(
            "motion_profiles",
            [{"name": "normal_visible", "weight": 1.0}],
        ),
        profile_index,
        family_count,
    )
    reverse_probability = float(
        profile.get(
            "reverse_probability",
            family.get("reverse_probability", 0.5),
        )
    )
    if family.get("balance_reverse_direction", False):
        reverse = bool(variant_index % 2)
    else:
        reverse = rng.random() < reverse_probability
    if profile.get("official_occluded_start", False) and location.get(
        "use_official_path", False
    ):
        reverse = False
    if reverse:
        path["waypointData"].reverse()
        start, end = end, start
    speed = float(
        uniform_or_choice(rng, profile.get("speed", family["speed"]))
    )
    active_distance = float(uniform_or_choice(rng, family["active_distance_m"]))
    actor["speed"] = speed
    actor["activeDistance"] = active_distance
    actor["active"] = True
    actor["loop"] = False
    actor["isWaypointMode"] = True
    yaw = math.degrees(math.atan2(end["y"] - start["y"], end["x"] - start["x"]))
    set_actor_world_position(actor, start, yaw)
    scenario["pedestrianList"] = [actor]
    scenario["waypointDataList"] = [path]
    runtime = [{
        "class": "pedestrian",
        "unique_id": actor["UNIQUEID"],
        "data_id": actor["DataID"],
        "name": asset.get("name") if asset else "official_template_pedestrian",
        "asset_subtype": asset.get("subtype") if asset else "person",
        "speed_config_value": speed,
        "speed_unit": "morai_pedestrian_config_value_unverified",
        "active_distance_m": active_distance,
        "reversed": reverse,
        "crosswalk_id": crosswalk_id,
        "crosswalk_role": location.get("role"),
        "crosswalk_split": location.get("split"),
        "difficulty_profile": profile.get("name"),
        "visibility_audit_required": bool(
            profile.get("visibility_audit_required", False)
        ),
        "activation_policy": location.get(
            "activation_policy",
            (
                "active_at_load"
                if location.get("use_official_path", False)
                else "deferred_ego_approach"
            ),
        ),
        "traffic_light_ids": crosswalk.get("traffic_light_ids", []),
        "start": start,
        "end": end,
    }]
    return target, runtime


def arm_pedestrian_before_capture(scenario, runtime, activation_margin_m):
    """Ensure the primary pedestrian is active before Ego capture motion.

    MORAI only starts waypoint behavior once Ego enters ``activeDistance``.
    Randomizing that value independently from the generated Ego pose can
    therefore produce a perfectly valid but stationary pedestrian clip.
    """

    ego = scenario["egoVehicle"]["initPosition"]["pos"]
    actor = scenario["pedestrianList"][0]
    distance = math.hypot(
        float(actor["pos"]["x"]) - float(ego["x"]),
        float(actor["pos"]["y"]) - float(ego["y"]),
    )
    required = distance + float(activation_margin_m)
    actor["activeDistance"] = max(float(actor["activeDistance"]), required)
    runtime[0]["active_distance_m"] = actor["activeDistance"]
    runtime[0]["initial_ego_distance_m"] = distance
    runtime[0]["activation_margin_m"] = (
        actor["activeDistance"] - distance
    )
    return distance


def configure_pedestrian_activation(
    scenario,
    runtime,
    ego_route,
    activation_margin_m=5.0,
    deferred_preroll_m=1.5,
    route_clearance_margin_m=0.5,
):
    """Keep the verified official scene or defer generated crossings safely."""

    if runtime[0].get("activation_policy") != "deferred_ego_approach":
        return arm_pedestrian_before_capture(
            scenario, runtime, activation_margin_m
        )

    ego = scenario["egoVehicle"]["initPosition"]["pos"]
    actor = scenario["pedestrianList"][0]
    actor_position = {
        "x": float(actor["pos"]["x"]),
        "y": float(actor["pos"]["y"]),
    }
    initial_distance = math.hypot(
        actor_position["x"] - float(ego["x"]),
        actor_position["y"] - float(ego["y"]),
    )
    route_clearance = point_to_route_distance(actor_position, ego_route)
    minimum_active_distance = route_clearance + float(
        route_clearance_margin_m
    )
    maximum_active_distance = initial_distance - float(deferred_preroll_m)
    if maximum_active_distance <= minimum_active_distance:
        raise ValueError(
            "generated pedestrian lacks safe activation window: "
            "initial={:.2f}m, route_clearance={:.2f}m".format(
                initial_distance, route_clearance
            )
        )
    actor["activeDistance"] = min(
        maximum_active_distance,
        max(float(actor["activeDistance"]), minimum_active_distance),
    )
    runtime[0].update(
        {
            "active_distance_m": float(actor["activeDistance"]),
            "initial_ego_distance_m": initial_distance,
            "activation_preroll_m": initial_distance
            - float(actor["activeDistance"]),
            "route_clearance_to_actor_m": route_clearance,
        }
    )
    return initial_distance


def shade_local_to_world(shade, longitudinal, lateral):
    # MORAI's shaded-area rectangle uses the vehicle-style local axis: its
    # long ``size.x`` corridor is yaw-90 degrees in world XY.  Treating the
    # raw yaw as the long axis placed GPS-shadow obstacles perpendicular to
    # the competition road by about 93 degrees.
    yaw = math.radians(float(shade["rot"]["yaw"]) - 90.0)
    return {
        "x": float(shade["pos"]["x"]) + math.cos(yaw) * longitudinal - math.sin(yaw) * lateral,
        "y": float(shade["pos"]["y"]) + math.sin(yaw) * longitudinal + math.cos(yaw) * lateral,
        "z": float(shade["pos"]["z"]),
    }


def load_runtime_obstacle_pool(catalog_path, compatibility_path, runtime_dir):
    catalog = load_json(catalog_path)
    compatibility = load_json(compatibility_path)
    source_name = compatibility["personal"]["generated_from"]["obstacle"][
        "file_name"
    ]
    source_path = runtime_dir.expanduser().resolve() / source_name
    source = load_json(source_path)
    actors = {
        int(actor["DataID"]): actor for actor in source.get("objectList", [])
    }
    eligible = set(compatibility["automatic_eligible"]["obstacle"])
    rows = []
    for row in catalog["obstacles_and_related"]:
        data_id = int(row["data_id"])
        if data_id not in eligible or data_id not in actors:
            continue
        if not str(row.get("generation_policy", "")).startswith("enabled"):
            continue
        rows.append({**row, "template": actors[data_id]})
    if not rows:
        raise ValueError("runtime obstacle pool is empty")
    rows.sort(key=lambda row: (row.get("subtype", ""), row["name"]))
    return rows


def load_runtime_pedestrian_pool(catalog_path, compatibility_path, runtime_dir):
    catalog = load_json(catalog_path)
    compatibility = load_json(compatibility_path)
    source_name = compatibility["personal"]["generated_from"]["pedestrian"][
        "file_name"
    ]
    source = load_json(runtime_dir.expanduser().resolve() / source_name)
    actors = {
        int(actor["DataID"]): actor for actor in source.get("pedestrianList", [])
    }
    eligible = set(compatibility["automatic_eligible"]["pedestrian"])
    rows = []
    for row in catalog["pedestrians_and_related"]:
        data_id = int(row["data_id"])
        if data_id not in eligible or data_id not in actors:
            continue
        if row.get("ontology_class") != "Pedestrian":
            continue
        # NCAP test dummies are catalogued as Pedestrian positives, but some
        # MRP builds accept their DataID in a scenario and then omit the actor
        # from ObjectStatus.  Route-wide standing/mixed scenes need a stable
        # human actor, so keep the runtime pool to real person assets.
        if row.get("subtype") != "person":
            continue
        if row.get("generation_policy") not in {"enabled", "limited_weight"}:
            continue
        rows.append({**row, "template": actors[data_id]})
    if not rows:
        raise ValueError("runtime pedestrian pool is empty")
    rows.sort(key=lambda row: (row.get("subtype", ""), row["name"]))
    return rows


def load_runtime_vehicle_pool(catalog_path, compatibility_path, runtime_dir):
    """Load the personal-launcher vehicle assets proven safe for automation."""

    catalog = load_json(catalog_path)
    compatibility = load_json(compatibility_path)
    source_name = compatibility["personal"]["generated_from"]["vehicle"][
        "file_name"
    ]
    source = load_json(runtime_dir.expanduser().resolve() / source_name)
    actors = {
        int(actor["DataID"]): actor for actor in source.get("vehicleList", [])
    }
    eligible = set(compatibility["automatic_eligible"]["vehicle"])
    rows = []
    for row in catalog["vehicles"]:
        data_id = int(row["data_id"])
        if data_id not in eligible or data_id not in actors:
            continue
        if row.get("ontology_class") != "Vehicle":
            continue
        if row.get("generation_policy") not in {"enabled", "limited_weight"}:
            continue
        rows.append({**row, "template": actors[data_id]})
    if not rows:
        raise ValueError("runtime vehicle pool is empty")
    rows.sort(key=lambda row: (row.get("subtype", ""), row["name"]))
    return rows


def obstacle_unique_quota(row):
    subtype = row.get("subtype")
    if subtype in {"box", "barrel_or_drum"}:
        return 72
    if subtype == "barrier":
        return 62
    if subtype == "unoccupied_bicycle_or_scooter":
        return 45
    if subtype == "road_sign":
        return 25
    if subtype in {"fixed_roadside_object", "road_surface"}:
        return 20
    raise ValueError("no V2 obstacle quota for {}".format(row.get("name")))


def build_weighted_obstacle_deck(rows, asset_weights=None, deck_size=None):
    """Interleave an obstacle quota deck without random asset starvation.

    V2 keeps its historical subtype quotas.  Base3 can instead provide an
    explicit, competition-relevant asset mix and a fixed deck size.
    """

    if asset_weights:
        weights = {
            int(row["data_id"]): float(asset_weights[row["name"]])
            for row in rows
            if row["name"] in asset_weights
        }
        if not weights:
            raise ValueError("configured obstacle asset weights match no runtime asset")
        total = int(deck_size or round(sum(weights.values())))
        exact = {
            data_id: total * weight / sum(weights.values())
            for data_id, weight in weights.items()
        }
        quotas = {data_id: int(math.floor(value)) for data_id, value in exact.items()}
        remainder = total - sum(quotas.values())
        for data_id in sorted(
            weights,
            key=lambda item: (-(exact[item] - quotas[item]), item),
        )[:remainder]:
            quotas[data_id] += 1
        rows = [row for row in rows if int(row["data_id"]) in quotas]
    else:
        quotas = {int(row["data_id"]): obstacle_unique_quota(row) for row in rows}
    selected = {data_id: 0 for data_id in quotas}
    row_by_id = {int(row["data_id"]): row for row in rows}
    deck = []
    while len(deck) < sum(quotas.values()):
        candidates = [
            data_id
            for data_id, quota in quotas.items()
            if selected[data_id] < quota
        ]
        data_id = min(
            candidates,
            key=lambda item: (
                (selected[item] + 1.0) / quotas[item],
                selected[item],
                item,
            ),
        )
        deck.append(row_by_id[data_id])
        selected[data_id] += 1
    return deck


def obstacle_local_to_world(center, yaw_deg, longitudinal, lateral):
    yaw = math.radians(float(yaw_deg))
    return {
        "x": center["x"] + math.cos(yaw) * longitudinal - math.sin(yaw) * lateral,
        "y": center["y"] + math.sin(yaw) * longitudinal + math.cos(yaw) * lateral,
        "z": center["z"] + 0.03,
    }


def static_obstacle_geometry_role(family, placement_index):
    rows = family.get("obstacle_geometry_roles")
    if not rows:
        return "legacy_roadside"
    schedule = weighted_zone_schedule(
        {"count": 100, "obstacle_geometry_roles": rows},
        key="obstacle_geometry_roles",
    )
    return schedule[int(placement_index) % len(schedule)]


def static_obstacle_scene_roles(family, count, scene_index):
    templates = family.get("obstacle_geometry_role_templates", {})
    configured = templates.get(str(int(count)))
    if not configured:
        return [
            static_obstacle_geometry_role(family, scene_index + index)
            for index in range(int(count))
        ]
    roles = list(configured)
    # Rotate which asset receives the primary collision-corridor role, while
    # preserving exactly one such role in every scene.
    rotation = int(scene_index) % len(roles)
    return roles[rotation:] + roles[:rotation]


def static_obstacle_heading(row, road_yaw, placement_index, rng):
    """Return diverse but mission-plausible obstacle headings."""

    name = row.get("name")
    subtype = row.get("subtype")
    long_barriers = {
        "concrete-barrier", "Concrete_Barricade01", "Concrete_Barricade02",
        "Steel_Barricade", "PE_Firewall_Orange", "PE_Firewall_White",
        "Lane_Separation_Bar",
    }
    if name in long_barriers:
        # 40% perpendicular, 35% diagonal and 25% road-parallel.  Both signs
        # are alternated so an elongated GT box is not trained to one yaw.
        modes = (
            [90.0, -90.0] * 4
            + [45.0, -45.0, 135.0, -135.0, 45.0, -45.0, 135.0]
            + [0.0, 180.0, 0.0, 180.0, 0.0]
        )
        delta = modes[int(placement_index) % len(modes)]
        return float(road_yaw) + delta + rng.uniform(-6.0, 6.0), (
            "perpendicular" if abs(delta) == 90.0 else
            "diagonal" if abs(delta) in {45.0, 135.0} else "parallel"
        )
    if subtype in {"box", "barrel_or_drum", "barrier"}:
        return float(road_yaw) + rng.uniform(-180.0, 180.0), "free_rotation"
    if subtype == "unoccupied_bicycle_or_scooter":
        delta = (0.0, 25.0, -25.0, 180.0)[int(placement_index) % 4]
        return float(road_yaw) + delta + rng.uniform(-8.0, 8.0), "road_or_oblique"
    return float(road_yaw) + rng.uniform(-20.0, 20.0), "road_aligned"


def static_obstacle_first_capture_distance(row, value):
    """Limit small obstacle stress range while retaining long barriers."""

    long_barriers = {
        "concrete-barrier", "Concrete_Barricade01", "Concrete_Barricade02",
        "Steel_Barricade", "PE_Firewall_Orange", "PE_Firewall_White",
    }
    value = float(value)
    if row.get("subtype") == "barrel_or_drum" and value > 14.0:
        return 14.0
    if row.get("subtype") == "box" and value > 16.0:
        return 16.0
    if row.get("name") not in long_barriers and value > 18.0:
        return 18.0
    return value


def build_obstacle_scene(
    scenario,
    template,
    family,
    rng,
    samples,
    ego_route,
    obstacle_pool=None,
    obstacle_pool_offset=0,
    target_override=None,
    ego_lead_distance_m=None,
    capture_motion_arm_m=0.0,
    capture_spacing_m=3.0,
    capture_count=3,
):
    base_actor = copy.deepcopy(template["objectList"][0])
    scale = float(uniform_or_choice(rng, family["scale_factor"]))
    if family["name"] == "gps_shadow_obstacle":
        shade = template["shadedAreaList"][0]
        lateral_range = family["route_lateral_offset_m"]
        # SaveSensorData takes substantially longer in wall time than the
        # nominal 12 Hz sequence clock.  Ego also travels 11--21 m while it
        # reaches the capture speed.  The shaded-area centre is about 26 m
        # ahead of the generated Ego, so place the cluster another 18--30 m
        # down the corridor.  The initial 42--57 m gap becomes roughly
        # 20--40 m at the first captured frame and remains collision-free
        # throughout the eight-frame approach.
        raw = shade_local_to_world(
            shade,
            rng.uniform(18.0, 30.0),
            rng.uniform(float(lateral_range[0]), float(lateral_range[1])),
        )
        # Keep the placement in the official shaded-area OBB. A global
        # nearest-link query can snap this narrow zone to a nearby parallel
        # road and silently move the actor tens of metres away.
        position = raw
        yaw = float(shade["rot"]["yaw"]) - 90.0
        target = dict(shade["pos"])
    elif target_override is not None:
        position = dict(target_override)
        yaw = float(target_override.get("yaw", 0.0))
        target = dict(target_override)
    else:
        position = dict(base_actor["pos"])
        yaw = float(base_actor["rot"]["yaw"])
        target = target_position_for_family(template, family["name"])

    count = integer_count(rng, family.get("objects_per_scene", 1))
    selected = []
    if obstacle_pool:
        for index in range(count):
            selected.append(
                obstacle_pool[(obstacle_pool_offset + index) % len(obstacle_pool)]
            )
    else:
        selected = [{
            "data_id": int(base_actor["DataID"]),
            "name": "template_obstacle",
            "subtype": "unknown",
            "template": base_actor,
        }]

    swept_route = local_route_window(ego_route, target)
    # Always use the actual Ego capture route tangent.  In the GPS shaded
    # area this also prevents the OBB's display rotation from becoming the
    # obstacle/road heading.
    road = route_pose_at_index(
        swept_route,
        min(
            range(len(swept_route)),
            key=lambda route_index: math.hypot(
                swept_route[route_index]["x"] - position["x"],
                swept_route[route_index]["y"] - position["y"],
            ),
        ),
    )
    scenario["objectList"] = []
    runtime = []
    scene_roles = static_obstacle_scene_roles(
        family,
        len(selected),
        int(obstacle_pool_offset) // max(1, len(selected)),
    )
    if family["name"] == "gps_shadow_obstacle":
        longitudinal_slots = [-2.0, 0.0, 2.0]
    else:
        longitudinal_slots = [-7.0, -2.0, 3.0, 8.0, 13.0, 18.0, 23.0]
    rng.shuffle(longitudinal_slots)
    for index, row in enumerate(selected):
        actor = copy.deepcopy(row["template"])
        unique_id = 3001 + index
        actor["UNIQUEID"] = unique_id
        lateral_sign = -1.0 if index % 2 == 0 else 1.0
        subtype = row.get("subtype")
        placement_index = int(obstacle_pool_offset) + index
        geometry_role = scene_roles[index]
        longitudinal = longitudinal_slots[index % len(longitudinal_slots)]
        minimum_clearance = 2.8
        if family["name"] == "gps_shadow_obstacle":
            lateral = lateral_sign * rng.uniform(0.2, 0.8)
            longitudinal = longitudinal_slots[index % len(longitudinal_slots)]
            minimum_clearance = 0.0
        elif geometry_role == "current_lane_front":
            first_capture_relative = (18.0, 22.0, 26.0, 30.0)[placement_index % 4]
            first_capture_relative = static_obstacle_first_capture_distance(
                row, first_capture_relative
            )
            longitudinal = (
                first_capture_relative
                - float(ego_lead_distance_m or 0.0)
                + float(capture_motion_arm_m)
            )
            lateral = rng.uniform(-0.25, 0.25)
            minimum_clearance = 0.0
        elif geometry_role == "adjacent_avoidance_front":
            first_capture_relative = (12.0, 18.0, 24.0)[placement_index % 3]
            first_capture_relative = static_obstacle_first_capture_distance(
                row, first_capture_relative
            )
            longitudinal = (
                first_capture_relative
                - float(ego_lead_distance_m or 0.0)
                + float(capture_motion_arm_m)
            )
            lateral = lateral_sign * rng.uniform(3.2, 4.4)
            minimum_clearance = 2.7
        elif geometry_role == "diagonal_context":
            first_capture_relative = (6.0, 12.0, 18.0, 24.0)[placement_index % 4]
            first_capture_relative = static_obstacle_first_capture_distance(
                row, first_capture_relative
            )
            longitudinal = (
                first_capture_relative
                - float(ego_lead_distance_m or 0.0)
                + float(capture_motion_arm_m)
            )
            lateral = lateral_sign * rng.uniform(3.8, 5.2)
            minimum_clearance = 3.0
        elif geometry_role == "near_side_or_rear":
            first_capture_relative = (-8.0, -3.0, 3.0, 8.0)[placement_index % 4]
            if subtype in {"box", "barrel_or_drum"} and abs(first_capture_relative) < 10.0:
                first_capture_relative = 12.0 if first_capture_relative >= 0.0 else -6.0
            longitudinal = (
                first_capture_relative
                - float(ego_lead_distance_m or 0.0)
                + float(capture_motion_arm_m)
            )
            lateral = lateral_sign * rng.uniform(3.0, 4.8)
            minimum_clearance = 2.7
        elif geometry_role == "roadside_context":
            first_capture_relative = (-10.0, 0.0, 10.0, 20.0, 30.0)[placement_index % 5]
            first_capture_relative = static_obstacle_first_capture_distance(
                row, first_capture_relative
            )
            longitudinal = (
                first_capture_relative
                - float(ego_lead_distance_m or 0.0)
                + float(capture_motion_arm_m)
            )
            lateral = lateral_sign * rng.uniform(5.0, 7.0)
            minimum_clearance = 4.0
        elif subtype == "road_surface":
            # Even nominally flat road-surface assets can own a collider in
            # MORAI.  Keep their centre beyond the Ego swept corridor.
            lateral = lateral_sign * rng.uniform(3.0, 4.2)
        elif subtype in {
            "barrier",
            "road_sign",
            "fixed_roadside_object",
        }:
            lateral = lateral_sign * rng.uniform(5.0, 7.0)
        elif subtype == "unoccupied_bicycle_or_scooter":
            lateral = lateral_sign * rng.uniform(2.5, 4.0)
        else:
            lateral = lateral_sign * rng.uniform(3.0, 6.0)
        if geometry_role == "legacy_roadside" and subtype in {
            "barrier", "road_sign", "fixed_roadside_object"
        }:
            minimum_clearance = 4.0
        if family["name"] == "gps_shadow_obstacle":
            # This family intentionally places the mission obstacle inside
            # the official shaded-area corridor.  It is audited separately;
            # moving it outside the corridor would change the mission.
            minimum_clearance = 0.0
        # A nearby return/crossing segment can make only one roadside safe.
        # Search both sides, preferring the originally scheduled side and the
        # smallest offset that clears every local Ego-route segment.
        clearance_candidates = []
        preferred_sign = math.copysign(1.0, lateral)
        longitudinal_shifts = (
            (0.0, 6.0, 12.0, 18.0)
            if geometry_role == "current_lane_front"
            else (0.0, 6.0, -6.0, 12.0, -12.0)
        )
        for longitudinal_shift in longitudinal_shifts:
            for step in range(21):
                magnitude = abs(lateral) + step * 0.5
                for sign_penalty, sign in enumerate(
                    (preferred_sign, -preferred_sign)
                ):
                    candidate_lateral = sign * magnitude
                    candidate_longitudinal = longitudinal + longitudinal_shift
                    candidate_position = obstacle_local_to_world(
                        road, road["yaw"], candidate_longitudinal, candidate_lateral
                    )
                    candidate_clearance = point_to_route_distance(
                        candidate_position, swept_route
                    )
                    footprint_radius = static_actor_footprint_radius({
                        "class": "obstacle", "subtype": subtype
                    })
                    if (
                        candidate_clearance >= minimum_clearance
                        and static_actor_candidate_is_clear(
                            candidate_position,
                            footprint_radius,
                            runtime,
                            margin_m=0.6,
                        )
                    ):
                        clearance_candidates.append(
                            (
                                abs(longitudinal_shift),
                                magnitude,
                                sign_penalty,
                                candidate_longitudinal,
                                candidate_lateral,
                                candidate_position,
                                candidate_clearance,
                            )
                        )
                if clearance_candidates:
                    break
            if clearance_candidates:
                break
        if clearance_candidates:
            (
                _,
                _,
                _,
                longitudinal,
                lateral,
                actor_position,
                route_clearance,
            ) = min(clearance_candidates, key=lambda item: item[:3])
        else:
            if geometry_role != "legacy_roadside" and family["name"] != "gps_shadow_obstacle":
                raise ValueError(
                    "could not place mission-geometry obstacle without overlap: "
                    "asset={} role={}".format(row.get("name"), geometry_role)
                )
            actor_position = obstacle_local_to_world(
                road, road["yaw"], longitudinal, lateral
            )
            route_clearance = point_to_route_distance(
                actor_position, swept_route
            )
        if route_clearance < minimum_clearance:
            raise ValueError(
                "static obstacle violates Ego swept-path clearance: "
                "asset={} subtype={} clearance={:.2f}/{:.2f}m".format(
                    row.get("name"), subtype, route_clearance, minimum_clearance
                )
            )
        actor_yaw, heading_mode = static_obstacle_heading(
            row, road["yaw"], placement_index, rng
        )
        set_actor_world_position(actor, actor_position, actor_yaw)
        local_scale = scale * rng.uniform(0.95, 1.05)
        actor["scale"] = formatted_vector(
            float(actor["scale"]["x"]) * local_scale,
            float(actor["scale"]["y"]) * local_scale,
            float(actor["scale"]["z"]) * local_scale,
        )
        actor["moveSpeed"] = 0.0
        actor["isWaypointMode"] = False
        actor["activeDistance"] = 0.0
        scenario["objectList"].append(actor)
        runtime.append({
            "unique_id": unique_id,
            "data_id": int(row["data_id"]),
            "name": row.get("name"),
            "subtype": row.get("subtype"),
            "motion": "static",
            "class": "obstacle",
            "position": actor_position,
            "yaw_deg": actor_yaw,
            "scale_factor": local_scale,
            "distance_bin_target": family.get("distance_bin_target"),
            "visibility_target": family.get("visibility_target", "clear"),
            "ego_route_clearance_m": route_clearance,
            "minimum_ego_route_clearance_m": minimum_clearance,
            "placement_footprint_radius_m": static_actor_footprint_radius({
                "class": "obstacle", "subtype": row.get("subtype")
            }),
            "route_relative_lateral_offset_m": lateral,
            "route_relative_longitudinal_offset_m": longitudinal,
            "planned_capture_relative_longitudinal_m": [
                float(longitudinal)
                + float(ego_lead_distance_m or 0.0)
                - float(capture_motion_arm_m)
                - float(capture_spacing_m) * frame_index
                for frame_index in range(int(capture_count))
            ],
            "geometry_role": geometry_role,
            "heading_mode": heading_mode,
            "base_road_yaw_deg": float(road["yaw"]),
        })
    return target, runtime


def set_static_vehicle_actor(
    actor, sample, position, unique_id, yaw_override=None
):
    actor["UNIQUEID"] = unique_id
    initial = actor["initPosition"]
    initial["initPositionMode"] = "Absolute"
    # ``sample`` supplies the legal MGeo link metadata only.  The actor must
    # retain its deliberately offset world position; snapping to sample x/y
    # puts it back on Ego's centreline and caused deterministic collisions.
    initial["pos"] = formatted_vector(
        position["x"], position["y"], position["z"] + 0.36
    )
    actor_yaw = (
        float(sample["yaw"])
        if yaw_override is None
        else float(yaw_override)
    )
    initial["rot"] = formatted_rotation(actor_yaw)
    initial["initLink"] = sample["link_idx"]
    initial["initLinkRatio"] = sample["point_ratio"]
    initial["gear"] = 4
    for key in (
        "linkVelocity",
        "constantVelocity",
        "desiredVelocity",
        "currentVelocity",
        "viewCurrentVelocity",
    ):
        if key in actor:
            actor[key] = 0.0
    actor["velocityType"] = 1
    actor["isCloseLoop"] = False
    actor["isLaneChange"] = False
    actor["isEgo"] = False
    actor["spawnPointUniqueID"] = -1
    actor["destinationUniqueID"] = -1
    actor["isWaypointMode"] = False
    actor["wayPointInfos"] = []
    actor["activeDistance"] = 0.0


def static_actor_footprint_radius(actor):
    """Return a conservative BEV clearance radius for generated actors.

    MORAI's asset catalog does not expose physical dimensions.  The values
    below are deliberately conservative placement envelopes, not GT box
    dimensions.  They keep large buses and long barriers from overlapping a
    different generated actor while preserving the class/asset diversity.
    """

    actor_class = actor.get("class")
    subtype = actor.get("subtype")
    if actor_class == "vehicle":
        if subtype in {"bus_van", "truck"}:
            return 5.5
        return 3.0
    if actor_class == "obstacle":
        if subtype in {"barrier", "fixed_roadside_object"}:
            return 3.0
        if subtype in {"unoccupied_bicycle_or_scooter", "road_sign"}:
            return 1.2
        return 1.7
    if actor_class == "pedestrian":
        return 0.8
    return 1.5


def vehicle_subtype_policy(family, key, subtype, default):
    """Resolve one Base3 static policy value for a vehicle subtype."""

    configured = family.get(key, {})
    if isinstance(configured, dict) and subtype in configured:
        return configured[subtype]
    return default


def static_vehicle_heading_jitter_deg(family, subtype, geometry_role=None):
    if geometry_role and (
        geometry_role == "roundabout_left_legal_path"
        or geometry_role.startswith("highway_parallel_lane_")
        or geometry_role.startswith("route_general_")
    ):
        return float(family.get("legal_path_heading_jitter_deg", 1.0))
    return float(
        vehicle_subtype_policy(
            family,
            "actor_heading_jitter_deg_by_subtype",
            subtype,
            family.get("actor_heading_jitter_deg", 0.0),
        )
    )


def same_lane_capture_slots(family, subtype):
    default = family.get(
        "same_lane_fallback_capture_slots_m",
        [-18.0, 15.0, 24.0, 33.0, 42.0],
    )
    return [
        float(value)
        for value in vehicle_subtype_policy(
            family,
            "same_lane_fallback_capture_slots_by_subtype",
            subtype,
            default,
        )
    ]


def controlled_same_lane_capture_slots(family, subtype, role):
    """Return role-specific capture-relative slots for Base3 static cars."""

    if role == "front":
        key = "same_lane_front_capture_slots_by_subtype"
        default = family.get("same_lane_front_capture_slots_m", [18.0, 21.0, 24.0])
    elif role == "rear":
        key = "same_lane_rear_capture_slots_by_subtype"
        default = family.get("same_lane_rear_capture_slots_m", [-18.0, -21.0, -24.0])
    else:
        raise ValueError("unknown controlled same-lane role: {}".format(role))
    return [
        float(value)
        for value in vehicle_subtype_policy(family, key, subtype, default)
    ]


def same_lane_rear_count(family, variant_index):
    """Choose an interleaved rear count without random quota drift."""

    rows = family.get("same_lane_rear_count_weights")
    if not rows:
        return 0
    schedule = weighted_zone_schedule(
        {"count": 100, "same_lane_rear_counts": rows},
        key="same_lane_rear_counts",
    )
    return int(schedule[int(variant_index) % len(schedule)])


def minimum_final_capture_separation_m(family, subtype):
    return float(
        vehicle_subtype_policy(
            family,
            "minimum_final_capture_separation_m_by_subtype",
            subtype,
            8.0,
        )
    )


def static_actor_candidate_is_clear(position, radius, occupied_runtime, margin_m=0.8):
    for occupied in occupied_runtime or []:
        other = occupied.get("position")
        if not other:
            continue
        required = (
            float(radius)
            + static_actor_footprint_radius(occupied)
            + float(margin_m)
        )
        if math.hypot(
            float(position["x"]) - float(other["x"]),
            float(position["y"]) - float(other["y"]),
        ) < required:
            return False
    return True


def static_vehicle_candidate_is_clear(
    position, radius, subtype, occupied_runtime
):
    """Use body-class margins for static Vehicle-to-Vehicle placement."""

    pair_margins = {
        "passenger_car": 1.0,
        "electric_passenger_car": 1.0,
        "wagon": 1.2,
        "suv": 2.0,
        "mpv": 2.0,
        "bus_van": 4.0,
        "truck": 4.0,
    }
    current_margin = float(pair_margins.get(subtype, 1.0))
    for occupied in occupied_runtime or []:
        other = occupied.get("position")
        if not other:
            continue
        if occupied.get("class") == "vehicle":
            margin = max(
                current_margin,
                float(pair_margins.get(occupied.get("subtype"), 1.0)),
            )
        else:
            margin = 0.8
        required = (
            float(radius)
            + static_actor_footprint_radius(occupied)
            + margin
        )
        if math.hypot(
            float(position["x"]) - float(other["x"]),
            float(position["y"]) - float(other["y"]),
        ) < required:
            return False
    return True


def link_sample_pose(samples, index):
    """Return one legal MGeo-link sample with a local tangent yaw."""

    index = max(0, min(len(samples) - 1, int(index)))
    before = samples[max(0, index - 1)]
    after = samples[min(len(samples) - 1, index + 1)]
    pose = dict(samples[index])
    pose["yaw"] = math.degrees(
        math.atan2(after["y"] - before["y"], after["x"] - before["x"])
    )
    return pose


def legal_path_pose_near_target(links_by_id, link_ids, target, offset_m):
    """Pick a legal lane-centre pose near a target plus pathwise offset."""

    samples, arc = link_path_samples(links_by_id, link_ids)
    nearest = min(
        range(len(samples)),
        key=lambda index: math.hypot(
            samples[index]["x"] - float(target["x"]),
            samples[index]["y"] - float(target["y"]),
        ),
    )
    desired_arc = max(0.0, min(arc[-1], arc[nearest] + float(offset_m)))
    return link_sample_pose(samples, route_index_at_arc(arc, desired_arc))


def roundabout_left_vehicle_poses(
    links_by_id,
    ego_route,
    target,
    count,
    phase_index,
):
    """Select stable left-side snapshots on the official roundabout NPC path.

    The candidates stay on a legal MGeo path and outside Ego's swept route.
    This represents the static geometry of a circulating/merging vehicle; it
    intentionally does not pretend to reproduce dynamic sensor timing.
    """

    samples, _ = link_path_samples(links_by_id, ROUNDABOUT_NPC_LINK_PATH)
    target_yaw = math.radians(float(target.get("yaw", 0.0)))
    swept_route = local_route_window(ego_route, target)
    # Two-actor layouts must remain valid even when the runtime asset deck
    # assigns buses or trucks to both forced slots.  Keep their desired
    # distances roughly 18 m apart instead of packing them into adjacent
    # 8 m rings.
    desired_distance_sets = (
        (14.0, 32.0),
        (18.0, 36.0),
        (12.0, 30.0),
    )
    desired_distances = desired_distance_sets[
        int(phase_index) % len(desired_distance_sets)
    ]
    selected = []
    for order in range(max(0, int(count))):
        desired = desired_distances[order % len(desired_distances)]
        candidates = []
        for index in range(len(samples)):
            pose = link_sample_pose(samples, index)
            dx = pose["x"] - float(target["x"])
            dy = pose["y"] - float(target["y"])
            longitudinal = math.cos(target_yaw) * dx + math.sin(target_yaw) * dy
            lateral = -math.sin(target_yaw) * dx + math.cos(target_yaw) * dy
            distance = math.hypot(dx, dy)
            route_clearance = point_to_route_distance(pose, swept_route)
            if not (
                lateral >= 2.5
                and 7.0 <= distance <= 42.0
                # The online controller can cut an entry curve by more than
                # the source centreline suggests.  The old 3.15 m gate caused
                # one real SUV collision at 5.12 m clearance in queue4.
                and route_clearance >= 7.0
            ):
                continue
            # Use the largest vehicle envelope here because the asset subtype
            # is assigned later by build_stopped_vehicle_scene.
            if not static_actor_candidate_is_clear(
                pose, 5.5, selected, margin_m=4.0
            ):
                continue
            candidates.append((abs(distance - desired), -lateral, pose, route_clearance))
        if not candidates:
            break
        _, _, pose, route_clearance = min(candidates, key=lambda row: row[:2])
        selected.append({
            "position": {axis: float(pose[axis]) for axis in ("x", "y", "z")},
            "sample": pose,
            "yaw_deg": float(pose["yaw"]),
            "ego_route_clearance_m": float(route_clearance),
            "geometry_role": "roundabout_left_legal_path",
            # Conservative placeholder used only while choosing the next
            # forced pose.  The actual asset metadata replaces it later.
            "class": "vehicle",
            "subtype": "bus_van",
        })
    return selected


def roundabout_left_vehicle_count(family, spatial_details, rng):
    """Return an exact-quota 0/1/2 actor count for one roundabout phase."""

    rows = family.get("roundabout_left_actor_count_weights")
    phase_total = spatial_details.get("roundabout_phase_total")
    phase_within = spatial_details.get("roundabout_phase_within")
    if rows and phase_total is not None and phase_within is not None:
        schedule = weighted_zone_schedule(
            {"count": int(phase_total), "actor_counts": rows},
            key="actor_counts",
        )
        return int(schedule[int(phase_within)])
    if rng.random() >= float(
        family.get("roundabout_left_vehicle_probability", 0.0)
    ):
        return 0
    return 1 + int(
        rng.random() < float(
            family.get("roundabout_second_left_vehicle_probability", 0.0)
        )
    )


def highway_other_lane_order(ego_lane_id, count, variant_index):
    """Return deterministic near/far target lanes for one highway scene.

    The three competition highway lanes share one direction.  Outer-lane Ego
    scenes prefer the immediately adjacent lane but retain a 30% far-lane
    sample.  Middle-lane scenes balance both adjacent lanes.  When two side
    actors are requested, both non-Ego lanes are represented.
    """

    ego_lane_id = int(ego_lane_id)
    count = max(0, min(2, int(count)))
    if ego_lane_id not in HIGHWAY_LANE_LINK_PATHS or count == 0:
        return []
    if ego_lane_id == 6:
        near, far = 8, 10
    elif ego_lane_id == 10:
        near, far = 8, 6
    else:
        near, far = ((6, 10) if int(variant_index) % 2 == 0 else (10, 6))
    if count >= 2:
        return [near, far]
    if ego_lane_id == 8:
        return [near]
    # Seven slots out of ten prefer the adjacent lane, while three preserve
    # the two-lanes-away geometry that the old left-only helper never emitted.
    return [near if int(variant_index) % 10 < 7 else far]


def highway_other_lane_vehicle_poses(
    links_by_id,
    ego_lane_id,
    target,
    count,
    variant_index,
):
    """Place static snapshots in all valid same-direction highway lanes."""

    target_lanes = highway_other_lane_order(
        ego_lane_id, count, variant_index
    )
    # Alternate useful front/rear diagonals.  All anchors stay inside the
    # 30 m training/evaluation range, unlike the old +38 m tail.
    single_offsets = (12.0, 20.0, 28.0, -12.0, -20.0)
    double_offsets = ((14.0, -14.0), (-14.0, 14.0))
    poses = []
    ego_lane_position = {6: 0, 8: 1, 10: 2}[int(ego_lane_id)]
    for index, target_lane in enumerate(target_lanes):
        if len(target_lanes) >= 2:
            offset = double_offsets[int(variant_index) % 2][index]
        else:
            offset = single_offsets[int(variant_index) % len(single_offsets)]
        pose = legal_path_pose_near_target(
            links_by_id, HIGHWAY_LANE_LINK_PATHS[target_lane], target, offset
        )
        lane_gap = abs(
            {6: 0, 8: 1, 10: 2}[int(target_lane)] - ego_lane_position
        )
        poses.append({
            "position": {axis: float(pose[axis]) for axis in ("x", "y", "z")},
            "sample": pose,
            "yaw_deg": float(pose["yaw"]),
            "ego_route_clearance_m": 3.15,
            "geometry_role": "highway_parallel_lane_{}_gap_{}".format(
                target_lane, lane_gap
            ),
        })
    return poses


def highway_phase_side_vehicle_poses(
    links_by_id,
    spatial_details,
    target,
    count,
    variant_index,
):
    """Place side Vehicles only on phase-authorized usable highway paths."""

    side_routes = [
        tuple(row) for row in spatial_details.get("highway_side_routes", [])
    ]
    count = min(max(0, int(count)), len(side_routes))
    if count == 0:
        return []
    ego_route_ids = tuple(spatial_details.get("highway_route_links", ()))
    ego_route, _ego_arc = route_from_link_path(links_by_id, ego_route_ids)
    # Near-side observations are intentionally interleaved with front/rear
    # diagonals.  They supplement, rather than replace, the mandatory lead.
    single_offsets = (0.0, 8.0, -8.0, 16.0, -16.0)
    double_offsets = ((8.0, -8.0), (-8.0, 8.0))
    poses = []
    for index, route_ids in enumerate(side_routes[:count]):
        restricted = HIGHWAY_RESTRICTED_LINK_IDS.intersection(route_ids)
        if restricted:
            raise ValueError(
                "restricted highway side route: {}".format(sorted(restricted))
            )
        offset = (
            double_offsets[int(variant_index) % 2][index]
            if count >= 2
            else single_offsets[int(variant_index) % len(single_offsets)]
        )
        pose = legal_path_pose_near_target(
            links_by_id, route_ids, target, offset
        )
        poses.append({
            "position": {axis: float(pose[axis]) for axis in ("x", "y", "z")},
            "sample": pose,
            "yaw_deg": float(pose["yaw"]),
            "ego_route_clearance_m": point_to_route_distance(
                pose, local_route_window(ego_route, target)
            ),
            "geometry_role": "{}_{}_side_path_{}".format(
                spatial_details.get("highway_phase"),
                spatial_details.get("highway_phase_variant"),
                index,
            ),
        })
    return poses


def route_general_parallel_vehicle_poses(
    links_by_id,
    ego_route,
    target,
    count,
    variant_index,
):
    """Use only explicit MGeo lane-change neighbours in ordinary roads.

    A nearby centreline alone is not enough: it may be opposing, turning, or
    disconnected.  This helper requires an explicit MGeo lane-change edge,
    compatible tangent heading, ordinary lane width, and Ego-route clearance.
    It therefore improves the few topology-complete route-general locations
    without guessing at the majority of single-lane/ambiguous locations.
    """

    if int(count) <= 0:
        return []
    source = links_by_id.get(str(target.get("link_idx")))
    if not source:
        return []
    neighbours = []
    for side in ("left", "right"):
        if not source.get("can_move_{}_lane".format(side)):
            continue
        destination_id = source.get("{}_lane_change_dst_link_idx".format(side))
        destination = links_by_id.get(str(destination_id))
        if not destination or len(destination.get("points", [])) < 3:
            continue
        pose = legal_path_pose_near_target(
            links_by_id,
            (str(destination_id),),
            target,
            (-12.0, 10.0, 18.0, 26.0)[int(variant_index) % 4],
        )
        heading_error = abs(
            wrapped_degrees(float(pose["yaw"]) - float(target["yaw"]))
        )
        distance = math.hypot(
            float(pose["x"]) - float(target["x"]),
            float(pose["y"]) - float(target["y"]),
        )
        clearance = point_to_route_distance(
            pose, local_route_window(ego_route, target)
        )
        if heading_error > 15.0 or not 2.5 <= distance <= 36.0 or clearance < 3.15:
            continue
        neighbours.append((side, str(destination_id), pose, clearance))
    if not neighbours:
        return []
    side, destination_id, pose, clearance = neighbours[
        int(variant_index) % len(neighbours)
    ]
    return [{
        "position": {axis: float(pose[axis]) for axis in ("x", "y", "z")},
        "sample": pose,
        "yaw_deg": float(pose["yaw"]),
        "ego_route_clearance_m": float(clearance),
        "geometry_role": "route_general_{}_parallel_link_{}".format(
            side, destination_id
        ),
    }]


def build_stopped_vehicle_scene(
    scenario,
    template,
    family,
    rng,
    samples,
    ego_route,
    target_override=None,
    vehicle_pool=None,
    vehicle_pool_offset=0,
    occupied_runtime=None,
    forced_actor_poses=None,
    ego_lead_distance_m=None,
    capture_motion_arm_m=0.0,
    capture_spacing_m=3.0,
    capture_count=3,
):
    target = (
        dict(target_override)
        if target_override is not None
        else target_position_for_family(template, family["name"])
    )
    swept_route = local_route_window(ego_route, target)
    ego_route_arc = route_arc_coordinates(ego_route)
    target_route_index = min(
        range(len(ego_route)),
        key=lambda route_index: math.hypot(
            float(ego_route[route_index]["x"]) - float(target["x"]),
            float(ego_route[route_index]["y"]) - float(target["y"]),
        ),
    )
    # Route-wide samplers carry the exact directed arc.  Prefer it over a
    # nearest-coordinate lookup because roundabout entry/circulation links can
    # be spatially close while representing different route occurrences.
    target_route_arc = float(
        target.get("route_arc_m", ego_route_arc[target_route_index])
    )
    # ``route_arc_m`` belongs to the reconstructed competition loop.  A
    # highway scene can replace that loop segment with lane 6/8/10, whose
    # cumulative arc origin is different.  Reusing the global scalar then
    # moves a nominal +24 m actor tens of metres away.  Validate the supplied
    # arc in the actual scene route and fall back to its nearest directed arc.
    supplied_arc_pose = continuous_route_pose_at_arc(
        ego_route,
        ego_route_arc,
        target_route_arc,
        wrap=bool(family.get("route_wide_closed_loop", False)),
    )
    if math.hypot(
        float(supplied_arc_pose["x"]) - float(target["x"]),
        float(supplied_arc_pose["y"]) - float(target["y"]),
    ) > 5.0:
        target_route_arc = float(ego_route_arc[target_route_index])
    road = route_pose_at_index(
        ego_route,
        min(
            range(len(ego_route)),
            key=lambda route_index: math.hypot(
                ego_route[route_index]["x"] - target["x"],
                ego_route[route_index]["y"] - target["y"],
            ),
        ),
    )
    count = integer_count(rng, family.get("actors_per_scene", 2))
    scenario["vehicleList"] = []
    runtime = []
    longitudinal_slots = list(
        family.get(
            "actor_longitudinal_slots_m",
            [4.0 + index * 9.0 for index in range(max(1, count))],
        )
    )
    if family.get("shuffle_actor_longitudinal_slots", False):
        rng.shuffle(longitudinal_slots)
    forced_actor_poses = list(forced_actor_poses or [])
    for index in range(count):
        asset = (
            vehicle_pool[(vehicle_pool_offset + index) % len(vehicle_pool)]
            if vehicle_pool
            else None
        )
        actor = copy.deepcopy(
            asset["template"]
            if asset
            else template["vehicleList"][index % len(template["vehicleList"])]
        )
        footprint_radius = static_actor_footprint_radius({
            "class": "vehicle",
            "subtype": asset.get("subtype") if asset else None,
        })
        subtype = asset.get("subtype") if asset else None
        forced = forced_actor_poses[index] if index < len(forced_actor_poses) else None
        nonforced_index = index - len(forced_actor_poses)
        controlled_same_lane = bool(
            family.get("controlled_same_lane_front_rear", False) and not forced
        )
        controlled_role = None
        if controlled_same_lane:
            if nonforced_index == 0:
                controlled_role = "front"
            elif nonforced_index == 1:
                controlled_role = "rear"
            else:
                raise ValueError(
                    "controlled static Vehicle layout permits only one front "
                    "and one optional rear actor"
                )
            fallback_capture_slots = controlled_same_lane_capture_slots(
                family, subtype, controlled_role
            )
        else:
            fallback_capture_slots = same_lane_capture_slots(family, subtype)
        if family.get("shuffle_actor_longitudinal_slots", False):
            rng.shuffle(fallback_capture_slots)
        longitudinal = float(longitudinal_slots[index % len(longitudinal_slots)])
        preferred_sign = float(
            family.get(
                "actor_lateral_sign",
                -1.0 if index % 2 == 0 else 1.0,
            )
        )
        same_lane_ahead = family.get("actor_placement_mode") == "same_lane_ahead"
        base_lateral = (
            rng.uniform(0.0, 0.45)
            if same_lane_ahead
            else rng.uniform(3.5, 5.0)
        )
        minimum_route_clearance = 0.0 if same_lane_ahead else max(
            3.0, footprint_radius * 0.55 + 1.5
        )
        clearance_candidates = []
        planned_ego_minimum_distance_m = None
        fallback_only = bool(
            (family.get("same_lane_fallback_only", False) or controlled_same_lane)
            and not forced
        )
        snap_to_mgeo = bool(
            family.get("snap_static_vehicle_to_mgeo", False)
            and not same_lane_ahead
            and not fallback_only
        )
        longitudinal_shifts = (
            ()
            if fallback_only
            else (
                (0.0,)
                if forced
                else (
                    (0.0, 5.0, -5.0)
                    if snap_to_mgeo
                    else (0.0, 5.0, -5.0, 10.0, -10.0)
                )
            )
        )
        lateral_steps = 1 if forced else (6 if snap_to_mgeo else 29)
        for longitudinal_shift in longitudinal_shifts:
            for step in range(lateral_steps):
                if forced and step > 0:
                    break
                magnitude = base_lateral + step * 0.5
                for sign_penalty, sign in enumerate(
                    (preferred_sign, -preferred_sign)
                ):
                    candidate_lateral = sign * magnitude
                    candidate_longitudinal = longitudinal + longitudinal_shift
                    forced_sample = None
                    geometry_role = None
                    if forced:
                        candidate = dict(forced["position"])
                        candidate_yaw = float(forced["yaw_deg"])
                        forced_sample = dict(forced["sample"])
                        candidate_longitudinal = longitudinal
                        candidate_lateral = 0.0
                        geometry_role = forced.get("geometry_role")
                        if geometry_role and (
                            geometry_role == "roundabout_left_legal_path"
                        ):
                            minimum_route_clearance = 7.0
                        elif geometry_role and (
                            geometry_role.startswith("highway_parallel_lane_")
                            or geometry_role.startswith("highway_")
                            or geometry_role.startswith("route_general_")
                        ):
                            # The generic footprint radius includes vehicle
                            # length and is intentionally conservative for
                            # arbitrary placements.  It must not reject a bus
                            # that is legally parallel in an adjacent lane.
                            # MGeo establishes the lane relationship; body-to-
                            # body spacing is still checked separately below.
                            minimum_route_clearance = 3.15
                    elif same_lane_ahead:
                        candidate_road = continuous_route_pose_at_arc(
                            ego_route,
                            ego_route_arc,
                            target_route_arc
                            + candidate_longitudinal,
                            wrap=bool(family.get("route_wide_closed_loop", False)),
                        )
                        candidate = obstacle_local_to_world(
                            candidate_road,
                            candidate_road["yaw"],
                            0.0,
                            candidate_lateral,
                        )
                        candidate_yaw = float(candidate_road["yaw"])
                    else:
                        candidate = obstacle_local_to_world(
                            road,
                            road["yaw"],
                            candidate_longitudinal,
                            candidate_lateral,
                        )
                        candidate_yaw = float(road["yaw"])
                    clearance = point_to_route_distance(candidate, swept_route)
                    candidate_sample = forced_sample
                    if (
                        not forced
                        and not same_lane_ahead
                        and snap_to_mgeo
                    ):
                        candidate_sample = nearest_aligned_sample(
                            samples, {**candidate, "yaw": candidate_yaw}, search_radius_m=8.0
                        )
                        snap_distance = math.hypot(
                            float(candidate_sample["x"]) - float(candidate["x"]),
                            float(candidate_sample["y"]) - float(candidate["y"]),
                        )
                        if snap_distance > float(
                            family.get("maximum_vehicle_lane_snap_distance_m", 2.75)
                        ):
                            continue
                        candidate = {
                            axis: float(candidate_sample[axis]) for axis in ("x", "y", "z")
                        }
                        candidate_yaw = float(candidate_sample["yaw"])
                        clearance = point_to_route_distance(candidate, swept_route)
                    # Route clearance protects Ego's swept corridor.  The
                    # mutual-clearance check protects mixed-class layouts.
                    if (
                        clearance >= minimum_route_clearance
                        and static_vehicle_candidate_is_clear(
                            candidate,
                            footprint_radius,
                            subtype,
                            list(occupied_runtime or []) + runtime,
                        )
                    ):
                        clearance_candidates.append((
                            abs(longitudinal_shift),
                            magnitude,
                            sign_penalty,
                            candidate_longitudinal,
                            candidate_lateral,
                            candidate,
                            clearance,
                            candidate_yaw,
                            candidate_sample,
                            geometry_role,
                        ))
                if clearance_candidates:
                    break
            if clearance_candidates:
                break
        if not clearance_candidates:
            if forced:
                # Side-path context is optional.  Never convert a failed side
                # placement into an additional same-lane front queue.
                continue
            # A route-general location can have no legal neighbouring lane.
            # Fall back to a same-lane queue well beyond the three capture
            # frames instead of placing a vehicle on unverified shoulder.
            target_arc = target_route_arc
            preferred_capture_relative = float(
                fallback_capture_slots[index % len(fallback_capture_slots)]
            )
            if controlled_role == "front":
                supplemental_slots = [36.0, 42.0, 50.0, 62.0]
            elif controlled_role == "rear":
                supplemental_slots = [-26.0, -30.0, -36.0, -42.0]
            else:
                supplemental_slots = [-30.0, 50.0, 62.0]
            fallback_options = [preferred_capture_relative] + [
                float(value)
                for value in fallback_capture_slots + supplemental_slots
                if not math.isclose(float(value), preferred_capture_relative)
            ]
            fallback_choice = None
            for capture_relative in fallback_options:
                final_relative = (
                    capture_relative
                    - float(capture_spacing_m) * (int(capture_count) - 1)
                    if capture_relative > 0.0
                    else capture_relative
                )
                minimum_separation = minimum_final_capture_separation_m(
                    family, subtype
                )
                if (
                    capture_relative > 0.0
                    and final_relative < minimum_separation
                ) or (
                    capture_relative < 0.0
                    and abs(capture_relative) < minimum_separation
                ):
                    continue
                fallback_arc = (
                    target_arc
                    - float(ego_lead_distance_m or 0.0)
                    + float(capture_motion_arm_m)
                    + capture_relative
                )
                fallback = continuous_route_pose_at_arc(
                    ego_route,
                    ego_route_arc,
                    fallback_arc,
                    wrap=bool(family.get("route_wide_closed_loop", False)),
                )
                candidate_sample = nearest_aligned_sample(samples, fallback)
                candidate_position = {
                    axis: float(candidate_sample[axis]) for axis in ("x", "y", "z")
                }
                ego_start_arc = target_arc - float(ego_lead_distance_m or 0.0)
                planned_ego_arcs = [ego_start_arc] + [
                    ego_start_arc
                    + float(capture_motion_arm_m)
                    + float(capture_spacing_m) * frame_index
                    for frame_index in range(int(capture_count))
                ]
                planned_ego_poses = [
                    continuous_route_pose_at_arc(
                        ego_route,
                        ego_route_arc,
                        arc,
                        wrap=bool(family.get("route_wide_closed_loop", False)),
                    )
                    for arc in planned_ego_arcs
                ]
                physical_distances = [
                    math.hypot(
                        float(candidate_position["x"]) - float(ego_pose["x"]),
                        float(candidate_position["y"]) - float(ego_pose["y"]),
                    )
                    for ego_pose in planned_ego_poses
                ]
                physical_minimum = min(physical_distances)
                # Arc distance alone is unsafe on roundabouts and other
                # self-near route geometry.  Enforce physical body clearance
                # against the initial Ego pose and every planned capture pose.
                if physical_minimum < minimum_separation:
                    continue
                if static_vehicle_candidate_is_clear(
                    candidate_position,
                    footprint_radius,
                    subtype,
                    list(occupied_runtime or []) + runtime,
                ):
                    fallback_choice = (
                        capture_relative,
                        candidate_sample,
                        candidate_position,
                        physical_minimum,
                    )
                    break
            if fallback_choice is None:
                if controlled_role == "rear":
                    # Rear context is optional; the primary front actor is not.
                    break
                raise ValueError("no collision-free same-lane Vehicle fallback")
            (
                capture_relative,
                sample,
                position,
                planned_ego_minimum_distance_m,
            ) = fallback_choice
            route_clearance = 0.0
            lateral = 0.0
            longitudinal = capture_relative
            placement_yaw = float(sample["yaw"])
            geometry_role = (
                "same_lane_primary_front"
                if controlled_role == "front"
                else "same_lane_optional_rear"
                if controlled_role == "rear"
                else "same_lane_safe_fallback"
            )
            minimum_route_clearance = 0.0
        else:
            (
                _, _, _, longitudinal, lateral, position, route_clearance,
                placement_yaw, sample, geometry_role,
            ) = min(clearance_candidates, key=lambda item: item[:3])
            if sample is None:
                sample = nearest_aligned_sample(
                    samples, {**position, "yaw": placement_yaw}, search_radius_m=8.0
                )
        if route_clearance < minimum_route_clearance:
            raise ValueError(
                "stopped vehicle violates Ego swept-path clearance: "
                "clearance={:.2f}/{:.2f}m".format(
                    route_clearance, minimum_route_clearance
                )
            )
        # Keep the body aligned with the route used to construct the offset.
        # Using the yaw of a nearby parallel/opposing link can rotate a long
        # bus across Ego's lane even when its centre has enough clearance.
        heading_jitter_limit = static_vehicle_heading_jitter_deg(
            family, subtype, geometry_role
        )
        heading_jitter = rng.uniform(
            -heading_jitter_limit, heading_jitter_limit
        )
        actor_yaw = float(placement_yaw) + heading_jitter
        opposite_heading_applied = False
        if rng.random() < float(
            family.get("opposite_heading_probability", 0.0)
        ):
            actor_yaw += 180.0
            opposite_heading_applied = True
        set_static_vehicle_actor(
            actor,
            sample,
            position,
            1001 + index,
            yaw_override=actor_yaw,
        )
        scenario["vehicleList"].append(actor)
        runtime.append({
            "unique_id": 1001 + index,
            "data_id": int(actor["DataID"]),
            "name": asset.get("name") if asset else None,
            "subtype": asset.get("subtype") if asset else None,
            "motion": "static",
            "class": "vehicle",
            "position": position,
            "yaw_deg": actor_yaw,
            "base_lane_yaw_deg": float(placement_yaw),
            "heading_jitter_deg": heading_jitter,
            "opposite_heading_applied": opposite_heading_applied,
            "ego_route_clearance_m": route_clearance,
            "minimum_ego_route_clearance_m": minimum_route_clearance,
            "placement_footprint_radius_m": footprint_radius,
            "route_relative_lateral_offset_m": lateral,
            "route_relative_longitudinal_offset_m": longitudinal,
            "planned_capture_relative_longitudinal_m": [
                float(longitudinal) - float(capture_spacing_m) * frame_index
                for frame_index in range(int(capture_count))
            ],
            "geometry_role": geometry_role or (
                "same_lane_queue" if same_lane_ahead else "mgeo_lane_snapped"
            ),
            "capture_relative_longitudinal_target_m": (
                longitudinal
                if geometry_role in {
                    "same_lane_safe_fallback",
                    "same_lane_primary_front",
                    "same_lane_optional_rear",
                }
                else None
            ),
            "planned_ego_minimum_physical_distance_m": (
                planned_ego_minimum_distance_m
            ),
        })
    return target, runtime


def build_standing_pedestrian_scene(
    scenario,
    template,
    family,
    rng,
    samples,
    ego_route=None,
    target_override=None,
    pedestrian_pool=None,
    pedestrian_pool_offset=0,
    occupied_runtime=None,
):
    target = (
        dict(target_override)
        if target_override is not None
        else target_position_for_family(template, family["name"])
    )
    if ego_route is None:
        road = nearest_sample(samples, target)
    else:
        local_route = local_route_window(ego_route, target)
        road = route_pose_at_index(
            local_route,
            min(
                range(len(local_route)),
                key=lambda index: math.hypot(
                    local_route[index]["x"] - target["x"],
                    local_route[index]["y"] - target["y"],
                ),
            ),
        )
    count = integer_count(rng, family.get("actors_per_scene", 2))
    scenario["pedestrianList"] = []
    runtime = []
    longitudinal_slots = list(
        family.get(
            "actor_longitudinal_slots_m",
            [3.0 + index * 7.0 for index in range(max(1, count))],
        )
    )
    if family.get("shuffle_actor_longitudinal_slots", False):
        rng.shuffle(longitudinal_slots)
    for index in range(count):
        asset = (
            pedestrian_pool[(pedestrian_pool_offset + index) % len(pedestrian_pool)]
            if pedestrian_pool
            else None
        )
        actor = copy.deepcopy(
            asset["template"] if asset else template["pedestrianList"][0]
        )
        longitudinal = float(longitudinal_slots[index % len(longitudinal_slots)])
        preferred_sign = -1.0 if index % 2 == 0 else 1.0
        position = None
        for longitudinal_shift in (
            0.0, 3.0, -3.0, 6.0, -6.0, 9.0, -9.0, 12.0, -12.0,
            15.0, -15.0,
        ):
            for step in range(18):
                magnitude = rng.uniform(2.5, 4.0) + step * 0.45
                for sign in (preferred_sign, -preferred_sign):
                    lateral = sign * magnitude
                    candidate = obstacle_local_to_world(
                        road,
                        road["yaw"],
                        longitudinal + longitudinal_shift,
                        lateral,
                    )
                    if static_actor_candidate_is_clear(
                        candidate, 0.8, list(occupied_runtime or []) + runtime
                    ):
                        position = candidate
                        break
                if position is not None:
                    break
            if position is not None:
                break
        if position is None:
            raise ValueError("could not place standing pedestrian without overlap")
        is_crosswalk_scene = bool(family.get("is_crosswalk_scene", False))
        mode_key = (
            "pedestrian_crosswalk_heading_modes"
            if is_crosswalk_scene
            else "pedestrian_route_heading_modes"
        )
        modes = list(family.get(mode_key, ["with_traffic"]))
        variant_index = int(family.get("scene_variant_index", 0))
        heading_mode = modes[(variant_index * 7 + index) % len(modes)]
        heading_delta = {
            "with_traffic": 0.0,
            "against_traffic": 180.0,
            "cross_left_to_right": -90.0,
            "cross_right_to_left": 90.0,
        }.get(heading_mode)
        if heading_delta is None:
            raise ValueError("unknown pedestrian heading mode: {}".format(heading_mode))
        heading_jitter = rng.uniform(
            -float(family.get("pedestrian_heading_jitter_deg", 0.0)),
            float(family.get("pedestrian_heading_jitter_deg", 0.0)),
        )
        actor_yaw = float(road["yaw"]) + heading_delta + heading_jitter
        set_actor_world_position(actor, position, actor_yaw)
        actor["UNIQUEID"] = 2001 + index
        actor["speed"] = 0.0
        actor["active"] = True
        actor["loop"] = False
        actor["movingDistance"] = 0.0
        actor["movingDistanceAmount"] = 0.0
        actor["animSpeed"] = 0.0
        actor["isWaypointMode"] = False
        actor["waypointEventList"] = []
        scenario["pedestrianList"].append(actor)
        runtime.append({
            "unique_id": 2001 + index,
            "data_id": int(actor["DataID"]),
            "name": asset.get("name") if asset else actor.get("name", "Man1"),
            "motion": "static",
            "class": "pedestrian",
            "position": position,
            "yaw_deg": actor_yaw,
            "base_road_yaw_deg": float(road["yaw"]),
            "heading_mode": heading_mode,
            "heading_delta_deg": heading_delta,
            "heading_jitter_deg": heading_jitter,
            "placement_footprint_radius_m": 0.8,
        })
    return target, runtime


def build_realistic_mixed_scene(
    scenario,
    template,
    family,
    rng,
    samples,
    ego_route,
    obstacle_pool,
    obstacle_pool_offset,
    target_override=None,
    pedestrian_pool=None,
    pedestrian_pool_offset=0,
    vehicle_pool=None,
    vehicle_pool_offset=0,
    ego_lead_distance_m=None,
    capture_motion_arm_m=3.0,
    capture_spacing_m=3.0,
    capture_count=3,
):
    target, runtime = build_obstacle_scene(
        scenario,
        template,
        {**family, "name": "static_obstacle"},
        rng,
        samples,
        ego_route,
        obstacle_pool=obstacle_pool,
        obstacle_pool_offset=obstacle_pool_offset,
        target_override=target_override,
    )
    _, vehicles = build_stopped_vehicle_scene(
        scenario,
        template,
        {
            **family,
            "name": "stopped_vehicle",
            "actors_per_scene": family.get("vehicle_actors_per_scene", 1),
        },
        rng,
        samples,
        ego_route,
        target_override=target,
        vehicle_pool=vehicle_pool,
        vehicle_pool_offset=vehicle_pool_offset,
        occupied_runtime=runtime,
        ego_lead_distance_m=ego_lead_distance_m,
        capture_motion_arm_m=float(capture_motion_arm_m),
        capture_spacing_m=float(capture_spacing_m),
        capture_count=int(capture_count),
    )
    _, pedestrians = build_standing_pedestrian_scene(
        scenario,
        template,
        {
            **family,
            "name": "standing_pedestrian",
            "actors_per_scene": family.get(
                "pedestrian_actors_per_scene", 1
            ),
        },
        rng,
        samples,
        ego_route=ego_route,
        target_override=target,
        pedestrian_pool=pedestrian_pool,
        pedestrian_pool_offset=pedestrian_pool_offset,
        occupied_runtime=runtime + vehicles,
    )
    return target, runtime + vehicles + pedestrians


def scenario_unique_ids(scenario):
    values = []
    for key in ("vehicleList", "pedestrianList", "objectList", "spawnPointList", "shadedAreaList"):
        values.extend(item.get("UNIQUEID") for item in scenario.get(key, []))
    return [value for value in values if value is not None]


def validate_scene(scenario, family, links_by_id, route, expected_runtime):
    identifiers = scenario_unique_ids(scenario)
    if len(identifiers) != len(set(identifiers)):
        raise ValueError(f"duplicate UNIQUEID in {family}")
    ego = scenario["egoVehicle"]["initPosition"]
    if ego.get("gear") != 4:
        raise ValueError("Ego must be in D gear")
    ego_pos = ego["pos"]
    ego_link = str(ego.get("initLink", ""))
    if ego_link in HIGHWAY_RESTRICTED_LINK_IDS:
        raise ValueError("Ego occupies restricted highway link {}".format(ego_link))
    for actor in scenario.get("vehicleList", []):
        actor_link = str(actor.get("initPosition", {}).get("initLink", ""))
        if actor_link in HIGHWAY_RESTRICTED_LINK_IDS:
            raise ValueError(
                "NPC occupies restricted highway link {}".format(actor_link)
            )
    for actor in expected_runtime:
        actor_link = str(actor.get("sample", {}).get("link_idx", ""))
        if actor.get("class") == "vehicle" and actor_link in HIGHWAY_RESTRICTED_LINK_IDS:
            raise ValueError(
                "runtime NPC occupies restricted highway link {}".format(actor_link)
            )
    route_distance = min(
        math.hypot(float(ego_pos["x"]) - item["x"], float(ego_pos["y"]) - item["y"])
        for item in route
    )
    if route_distance > 2.5:
        raise ValueError(
            "Ego is {:.2f} m from competition route: family={} "
            "position=({:.3f}, {:.3f})".format(
                route_distance,
                family,
                float(ego_pos["x"]),
                float(ego_pos["y"]),
            )
        )
    for spawn in scenario["spawnPointList"]:
        for key in ("startLinkInfo", "endLinkInfo"):
            link = spawn[key]["linkIdx"]
            if link not in links_by_id:
                raise ValueError(f"unknown MGeo link {link}")
        if spawn["MaxDesiredVelocity_Custom"] <= 0.0:
            raise ValueError("dynamic vehicle target speed must be positive")
    if family == "crosswalk_pedestrian":
        if len(scenario["pedestrianList"]) != 1 or len(scenario["waypointDataList"]) != 1:
            raise ValueError("pedestrian scene must retain one actor and one path")
        if scenario["pedestrianList"][0]["speed"] <= 0.0:
            raise ValueError("dynamic pedestrian speed must be positive")
    if family.endswith("obstacle"):
        if not scenario["objectList"] or any(
            actor["moveSpeed"] != 0.0
            for actor in scenario["objectList"]
        ):
            raise ValueError("obstacle family must contain static obstacles")
    if not expected_runtime and family != "sparse_background":
        raise ValueError("runtime actor specification is empty")
    return {"ego_route_distance_m": route_distance, "unique_id_count": len(identifiers)}


def split_for_variant(index, family_count, split_ratios=None):
    if split_ratios is not None:
        ratios = {
            name: float(split_ratios.get(name, 0.0))
            for name in ("train", "val", "test")
        }
        if any(value < 0.0 for value in ratios.values()):
            raise ValueError("split ratios must be non-negative")
        total = sum(ratios.values())
        if total <= 0.0:
            raise ValueError("at least one split ratio must be positive")
        normalized = {name: value / total for name, value in ratios.items()}
        val_count = int(round(family_count * normalized["val"]))
        test_count = int(round(family_count * normalized["test"]))
        train_count = family_count - val_count - test_count
        if train_count < 0:
            raise ValueError("rounded split counts exceed family count")
        if index < train_count:
            return "train"
        if index < train_count + val_count:
            return "val"
        return "test"
    if family_count < 3:
        return "train"
    validation_count = max(1, int(round(family_count * 0.15)))
    test_count = max(1, int(round(family_count * 0.15)))
    train_count = family_count - validation_count - test_count
    if index < train_count:
        return "train"
    if index < train_count + validation_count:
        return "val"
    return "test"


def main():
    args = parse_args()
    config = load_json(args.config)
    template = load_json(args.template)
    route, route_arc = load_route(args.route)
    links = load_json(args.mgeo_dir / "link_set.json")
    links_by_id = {item["idx"]: item for item in links}
    samples = mgeo_samples(links)
    crosswalk_catalog = load_crosswalk_catalog(args.mgeo_dir)
    route_reconstruction = None
    if config.get("route_wide_route_source") == "mgeo_guided":
        dense_route, route_reconstruction = build_mgeo_guided_competition_route(
            route, links
        )
    else:
        dense_route = densify_route(route)
    dense_route_arc = route_arc_coordinates(dense_route)
    obstacle_pool = None
    if config.get("use_runtime_obstacle_pool", False):
        obstacle_pool = load_runtime_obstacle_pool(
            args.asset_catalog,
            args.asset_compatibility,
            args.runtime_asset_scenario_dir,
        )
        allowlist = config.get("runtime_obstacle_asset_allowlist")
        if allowlist:
            allowed = set(allowlist)
            obstacle_pool = [
                row for row in obstacle_pool if row.get("name") in allowed
            ]
            missing = allowed - {row.get("name") for row in obstacle_pool}
            if missing:
                raise ValueError(
                    "configured obstacle assets are unavailable: {}".format(
                        sorted(missing)
                    )
                )
        if config.get("weighted_obstacle_quota_deck", False):
            obstacle_pool = build_weighted_obstacle_deck(
                obstacle_pool,
                asset_weights=config.get("obstacle_asset_quota_weights"),
                deck_size=config.get("obstacle_asset_quota_deck_size"),
            )
    pedestrian_pool = None
    if config.get("use_runtime_pedestrian_pool", False):
        pedestrian_pool = load_runtime_pedestrian_pool(
            args.asset_catalog,
            args.asset_compatibility,
            args.runtime_asset_scenario_dir,
        )
    vehicle_pool = None
    if config.get("use_runtime_vehicle_pool", False):
        vehicle_pool = load_runtime_vehicle_pool(
            args.asset_catalog,
            args.asset_compatibility,
            args.runtime_asset_scenario_dir,
        )
    seed = int(args.seed if args.seed is not None else config["seed"])
    scenario_dir = args.output_root / "scenarios" / args.group
    metadata_dir = args.output_root / "metadata" / args.group
    # A group is a deterministic generator-owned output.  Remove stale files
    # before regeneration so lowering a family quota cannot leave orphaned
    # scenarios/metadata that disagree with the new manifest.
    for generated_dir in (scenario_dir, metadata_dir):
        if generated_dir.exists():
            shutil.rmtree(generated_dir)
    route_dir = metadata_dir / "routes"
    manifest_path = args.output_root / "manifests" / f"{args.group}.json"
    scenes = []
    global_index = 0
    obstacle_pool_cursor = 0
    pedestrian_pool_cursor = 0
    vehicle_pool_cursor = 0

    for family in config["families"]:
        name = family["name"]
        if name not in FAMILY_CODES:
            raise ValueError(f"unsupported family in config: {name}")
        for variant_index in range(int(family["count"])):
            global_index += 1
            scene_seed = seed + global_index * 1009
            rng = random.Random(scene_seed)
            scenario = copy.deepcopy(template)
            clear_actor_lists(scenario)
            route_override = None
            capture_overrides = {}
            ego_sample = None
            route_wide_target = None
            spatial_details = {}
            preselected_ego_lead = None
            scene_route = route
            scene_route_arc = route_arc
            scene_route_closed_loop = bool(
                family.get("route_wide_closed_loop", False)
            )
            if (
                name == "crosswalk_pedestrian"
                and config.get("route_wide_route_source") == "mgeo_guided"
            ):
                scene_route = dense_route
                scene_route_arc = dense_route_arc
                route_override = scene_route
                scene_route_closed_loop = True
            if name in {"stopped_vehicle", "realistic_mixed_static", "static_obstacle"} and (
                family.get("same_lane_fallback_capture_slots_m")
                or family.get("controlled_same_lane_front_rear", False)
                or family.get("obstacle_geometry_roles")
                or family.get("obstacle_geometry_role_templates")
            ):
                preselected_ego_lead = choose_lead(rng, family)
            if family.get("spatial_policy") == "route_wide_competition_route":
                scene_route = dense_route
                scene_route_arc = dense_route_arc
                route_override = scene_route
                crosswalk_scene_count = int(
                    family.get("crosswalk_scene_count", family["count"])
                )
                family_scene_count = int(family["count"])
                use_crosswalk_target = (
                    ((variant_index + 1) * crosswalk_scene_count)
                    // family_scene_count
                    > (variant_index * crosswalk_scene_count)
                    // family_scene_count
                )
                if (
                    name == "standing_pedestrian"
                    and use_crosswalk_target
                ):
                    route_wide_target = route_wide_crosswalk_target(
                        crosswalk_catalog, scene_route, family, variant_index
                    )
                    spatial_details["spatial_stratum"] = "crosswalk"
                elif family.get("spatial_strata"):
                    (
                        scene_route,
                        scene_route_arc,
                        route_wide_target,
                        spatial_details,
                    ) = static_spatial_target(
                        family,
                        variant_index,
                        rng,
                        dense_route,
                        dense_route_arc,
                        links_by_id,
                    )
                    route_override = scene_route
                    scene_route_closed_loop = (
                        spatial_details.get("spatial_stratum")
                        != "highway_three_lane"
                        and spatial_details.get("highway_phase") is None
                        and spatial_details.get("ordinary_two_lane_side") is None
                    )
                    if route_wide_target is not None:
                        bin_count = int(family.get("route_wide_bin_count", 24))
                        route_wide_target.setdefault(
                            "route_wide_bin_count", bin_count
                        )
                        route_wide_target.setdefault(
                            "route_wide_bin",
                            min(
                                bin_count - 1,
                                int(
                                    float(route_wide_target["route_arc_m"])
                                    / max(1e-9, float(scene_route_arc[-1]))
                                    * bin_count
                                ),
                            ),
                        )
                elif name in ROUTE_WIDE_STATIC_FAMILIES:
                    route_wide_target = route_wide_anchor_for_variant(
                        scene_route,
                        scene_route_arc,
                        family,
                        variant_index,
                        rng,
                    )

            if name in SIMPLE_SPAWN_VEHICLE_FAMILIES and family.get(
                "ego_capture_zones"
            ):
                (
                    ego_sample,
                    runtime,
                    capture_overrides,
                    target,
                ) = build_route_anchored_vehicle_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    variant_index,
                    route,
                    route_arc,
                    samples,
                    links_by_id,
                )
            elif name in SIMPLE_SPAWN_VEHICLE_FAMILIES:
                target, runtime = build_vehicle_scene(
                    scenario, template, family, rng, variant_index
                )
            elif name == "highway_cruise_overtake":
                (
                    route_override,
                    ego_sample,
                    runtime,
                    capture_overrides,
                ) = build_highway_cruise_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    variant_index,
                    links_by_id,
                )
                spawn = scenario["spawnPointList"][0]
                target = dict(spawn["pos"])
            elif name == "highway_lane_change_overtake":
                (
                    route_override,
                    ego_sample,
                    runtime,
                    capture_overrides,
                ) = build_highway_lane_change_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    variant_index,
                    links_by_id,
                )
                spawn = scenario["spawnPointList"][0]
                target = dict(spawn["pos"])
            elif name == "east_corridor_relative_vehicle":
                (
                    ego_sample,
                    runtime,
                    capture_overrides,
                    target,
                ) = build_east_corridor_relative_vehicle_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    variant_index,
                    route,
                    route_arc,
                    samples,
                )
            elif name == "static_highway_obstacle":
                (
                    route_override,
                    ego_sample,
                    runtime,
                    capture_overrides,
                ) = build_static_highway_scene(
                    scenario,
                    family,
                    rng,
                    variant_index,
                    links_by_id,
                    obstacle_pool,
                    obstacle_pool_cursor,
                )
                target = dict(runtime[0]["position"])
            elif name == "crosswalk_pedestrian":
                target, runtime = build_pedestrian_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    variant_index,
                    crosswalk_catalog,
                    pedestrian_pool=pedestrian_pool,
                    pedestrian_pool_offset=pedestrian_pool_cursor,
                )
            elif name == "stopped_vehicle":
                placement_family = family
                roundabout_phase = spatial_details.get("roundabout_phase")
                forced_vehicle_poses = []
                if roundabout_phase in {
                    "approach", "entry_merge", "circulation", "exit_egress"
                }:
                    placement_family = {
                        **family,
                        # Keep non-forced actors in the same class-aware queue
                        # for every roundabout phase. Legal circulating actors
                        # remain optional and never become queue fallbacks.
                        "same_lane_fallback_only": True,
                    }
                if roundabout_phase in {
                    "approach", "entry_merge", "circulation"
                }:
                    roundabout_vehicle_count = roundabout_left_vehicle_count(
                        family, spatial_details, rng
                    )
                    if roundabout_vehicle_count:
                        forced_vehicle_poses = roundabout_left_vehicle_poses(
                            links_by_id,
                            scene_route,
                            route_wide_target,
                            roundabout_vehicle_count,
                            {"approach": 0, "entry_merge": 1, "circulation": 2}[
                                roundabout_phase
                            ],
                        )
                highway_lane_id = spatial_details.get("highway_lane_id")
                highway_phase = spatial_details.get("highway_phase")
                if highway_phase in HIGHWAY_STATIC_PHASE_VARIANTS:
                    available_side_routes = len(
                        spatial_details.get("highway_side_routes", [])
                    )
                    highway_vehicle_count = min(
                        available_side_routes,
                        1 + int(
                            available_side_routes > 1
                            and rng.random() < float(
                                family.get(
                                    "highway_second_side_vehicle_probability",
                                    0.0,
                                )
                            )
                        ),
                    )
                    forced_vehicle_poses = highway_phase_side_vehicle_poses(
                        links_by_id,
                        spatial_details,
                        route_wide_target,
                        highway_vehicle_count,
                        variant_index,
                    )
                elif (
                    highway_lane_id in HIGHWAY_LANE_LINK_PATHS
                    and rng.random() < float(
                        family.get(
                            "highway_side_vehicle_probability",
                            family.get("highway_left_vehicle_probability", 0.0),
                        )
                    )
                ):
                    highway_vehicle_count = 1 + int(
                        rng.random() < float(
                            family.get(
                                "highway_second_side_vehicle_probability",
                                0.0,
                            )
                        )
                    )
                    forced_vehicle_poses = highway_other_lane_vehicle_poses(
                        links_by_id,
                        highway_lane_id,
                        route_wide_target,
                        highway_vehicle_count,
                        variant_index,
                    )
                elif (
                    str(spatial_details.get("spatial_stratum", "")).startswith(
                        "route_general"
                    )
                    and rng.random() < float(
                        family.get(
                            "route_general_parallel_vehicle_probability", 0.0
                        )
                    )
                ):
                    forced_vehicle_poses = route_general_parallel_vehicle_poses(
                        links_by_id,
                        scene_route,
                        route_wide_target,
                        1,
                        variant_index,
                    )
                placement_family = {
                    **placement_family,
                    "actors_per_scene": (
                        len(forced_vehicle_poses)
                        + 1
                        + same_lane_rear_count(family, variant_index)
                    ),
                }
                target, runtime = build_stopped_vehicle_scene(
                    scenario,
                    template,
                    placement_family,
                    rng,
                    samples,
                    scene_route,
                    target_override=route_wide_target,
                    vehicle_pool=vehicle_pool,
                    vehicle_pool_offset=vehicle_pool_cursor,
                    forced_actor_poses=forced_vehicle_poses,
                    ego_lead_distance_m=preselected_ego_lead,
                    capture_motion_arm_m=float(
                        family.get(
                            "capture_motion_arm_m",
                            config["capture"].get("capture_motion_arm_m", 0.0),
                        )
                    ),
                    capture_spacing_m=float(
                        config["capture"].get("capture_spacing_m", 3.0)
                    ),
                    capture_count=int(
                        config["capture"].get("captures_per_scene", 3)
                    ),
                )
            elif name == "standing_pedestrian":
                target, runtime = build_standing_pedestrian_scene(
                    scenario,
                    template,
                    {
                        **family,
                        "scene_variant_index": variant_index,
                        "is_crosswalk_scene": (
                            spatial_details.get("spatial_stratum") == "crosswalk"
                        ),
                    },
                    rng,
                    samples,
                    ego_route=scene_route,
                    target_override=route_wide_target,
                    pedestrian_pool=pedestrian_pool,
                    pedestrian_pool_offset=pedestrian_pool_cursor,
                )
            elif name == "realistic_mixed_static":
                placement_family = family
                if spatial_details.get("roundabout_phase") == "exit_egress":
                    placement_family = {
                        **family,
                        "actor_longitudinal_slots_m": [-10.0, -3.0, 5.0, 10.0],
                    }
                target, runtime = build_realistic_mixed_scene(
                    scenario,
                    template,
                    placement_family,
                    rng,
                    samples,
                    scene_route,
                    obstacle_pool,
                    obstacle_pool_cursor,
                    target_override=route_wide_target,
                    pedestrian_pool=pedestrian_pool,
                    pedestrian_pool_offset=pedestrian_pool_cursor,
                    vehicle_pool=vehicle_pool,
                    vehicle_pool_offset=vehicle_pool_cursor,
                    ego_lead_distance_m=preselected_ego_lead,
                    capture_motion_arm_m=float(
                        family.get(
                            "capture_motion_arm_m",
                            config["capture"].get("capture_motion_arm_m", 0.0),
                        )
                    ),
                    capture_spacing_m=float(
                        config["capture"].get("capture_spacing_m", 3.0)
                    ),
                    capture_count=int(
                        config["capture"].get("captures_per_scene", 3)
                    ),
                )
            elif (
                name == "sparse_background"
                and (
                    family.get("empty_scene", False)
                    or int(variant_index)
                    < int(family.get("empty_scene_count", 0))
                )
            ):
                target = route_wide_target
                runtime = []
            else:
                target, runtime = build_obstacle_scene(
                    scenario,
                    template,
                    family,
                    rng,
                    samples,
                    scene_route,
                    obstacle_pool=obstacle_pool,
                    obstacle_pool_offset=obstacle_pool_cursor,
                    target_override=route_wide_target,
                    ego_lead_distance_m=preselected_ego_lead,
                    capture_motion_arm_m=float(
                        family.get(
                            "capture_motion_arm_m",
                            config["capture"].get("capture_motion_arm_m", 0.0),
                        )
                    ),
                    capture_spacing_m=float(
                        config["capture"].get("capture_spacing_m", 3.0)
                    ),
                    capture_count=int(
                        config["capture"].get("captures_per_scene", 3)
                    ),
                )

            if name in {
                "static_obstacle",
                "static_highway_obstacle",
                "realistic_mixed_static",
                "sparse_background",
                "gps_shadow_obstacle",
            }:
                obstacle_pool_cursor += sum(
                    actor.get("motion") == "static"
                    and actor.get("name") is not None
                    for actor in runtime
                )
            pedestrian_pool_cursor += sum(
                actor.get("class") == "pedestrian" for actor in runtime
            )
            vehicle_pool_cursor += sum(
                actor.get("class") == "vehicle" for actor in runtime
            )

            if ego_sample is None:
                lead = (
                    preselected_ego_lead
                    if preselected_ego_lead is not None
                    else choose_lead(rng, family)
                )
                route_pose = route_pose_before(
                    scene_route,
                    scene_route_arc,
                    target,
                    lead,
                    wrap_closed_route=bool(
                        scene_route_closed_loop
                    ),
                )
                # The competition guide route is already the authoritative
                # Ego path for crosswalk capture.  Snapping it to the nearest
                # heading-compatible MGeo sample can select a parallel road
                # near CW000002/CW000013 and move Ego over ten metres away.
                if name == "crosswalk_pedestrian":
                    crosswalk_link_sample = nearest_aligned_sample(
                        samples, route_pose
                    )
                    ego_sample = dict(crosswalk_link_sample)
                    ego_sample.update(
                        {
                            key: route_pose[key]
                            for key in ("x", "y", "z", "yaw")
                            if key in route_pose
                        }
                    )
                elif route_pose.get("link_idx") in links_by_id:
                    ego_sample = dict(route_pose)
                else:
                    ego_sample = nearest_aligned_sample(samples, route_pose)
                ego_motion_profiles = family.get("ego_motion_profiles")
                ego_motion_profile = None
                if ego_motion_profiles:
                    ego_motion_profile = weighted_variant_choice(
                        ego_motion_profiles,
                        variant_index,
                        int(family["count"]),
                    )
                ego_speed = float(
                    uniform_or_choice(
                        rng,
                        (
                            ego_motion_profile.get("target_speed_kph")
                            if ego_motion_profile
                            else family.get("ego_target_speed_kph", 0.0)
                        ),
                    )
                )
                ego_initial_speed = float(
                    uniform_or_choice(
                        rng,
                        (
                            ego_motion_profile.get(
                                "initial_speed_kph", ego_speed
                            )
                            if ego_motion_profile
                            else family.get("ego_initial_speed_kph", ego_speed)
                        ),
                    )
                )
                ego_initial_speed = min(ego_initial_speed, ego_speed)
                set_ego_pose(
                    scenario["egoVehicle"], ego_sample, ego_initial_speed
                )
                if ego_speed > 0.0:
                    capture_overrides.update(
                        {
                            "ego_target_speed_kph": ego_speed,
                            "ego_initial_speed_kph": ego_initial_speed,
                            "minimum_ego_speed_kph": ego_speed
                            * float(family.get("minimum_ego_speed_ratio", 0.8)),
                            "maximum_motion_m": float(
                                family.get("maximum_motion_m", 60.0)
                            ),
                            "motion_stall_timeout_sec": float(
                                family.get("motion_stall_timeout_sec", 10.0)
                            ),
                            "lookahead_m": 4.0,
                        }
                    )
                    if ego_motion_profile:
                        capture_overrides["ego_motion_profile"] = (
                            ego_motion_profile.get("name")
                        )
                    if "minimum_curve_speed_kph" in family:
                        capture_overrides["minimum_curve_speed_kph"] = float(
                            family["minimum_curve_speed_kph"]
                        )
                validation_route = scene_route
            else:
                lead = None
                route_pose = ego_sample
                validation_route = route_override if route_override is not None else route
            if family.get("stable_ego_gate", False):
                capture_overrides.update(
                    {
                        "stable_ego_window_sec": float(
                            family.get("stable_ego_window_sec", 0.5)
                        ),
                        "maximum_stable_acceleration_mps2": float(
                            family.get(
                                "maximum_stable_acceleration_mps2", 1.0
                            )
                        ),
                    }
                )
            if name == "crosswalk_pedestrian":
                configure_pedestrian_activation(
                    scenario,
                    runtime,
                    route,
                    activation_margin_m=family.get(
                        "activation_margin_m", 5.0
                    ),
                    deferred_preroll_m=family.get(
                        "deferred_activation_preroll_m", 1.5
                    ),
                )
                # Start the short crossing clip immediately after MORAI first
                # reports motion.  Priming the heavyweight SaveSensorData
                # writers after the pedestrian starts can consume most of the
                # 13.8 m sample path before frame zero is recorded.
                capture_overrides.update(
                    {
                        "warmup_sec": float(
                            family.get("dynamic_warmup_sec", 0.0)
                        ),
                        "prime_capture_before_clip": bool(
                            family.get("prime_capture_before_clip", False)
                        ),
                        "capture_motion_arm_m": float(
                            family.get("capture_motion_arm_m", 0.3)
                        ),
                    }
                )
                configured_gate = family.get("capture_actor_gate")
                if configured_gate:
                    capture_overrides["capture_actor_gate"] = copy.deepcopy(
                        configured_gate
                    )
            scenario_load_initial_speed = config["capture"].get(
                "scenario_load_initial_speed_kph"
            )
            if (
                scenario_load_initial_speed is not None
                and not capture_overrides.get(
                    "preserve_ego_initial_speed_on_load", False
                )
            ):
                nominal_initial_speed = float(
                    scenario["egoVehicle"].get("currentVelocity", 0.0)
                )
                set_ego_pose(
                    scenario["egoVehicle"],
                    ego_sample,
                    float(scenario_load_initial_speed),
                )
                capture_overrides["nominal_ego_initial_speed_kph"] = (
                    nominal_initial_speed
                )
                capture_overrides["ego_initial_speed_kph"] = float(
                    scenario_load_initial_speed
                )
            execution_route = route_override
            execution_anchor_index = int(route_pose["route_index"])
            execution_route_rotation = None
            if (
                route_override is not None
                and scene_route_closed_loop
            ):
                (
                    execution_route,
                    execution_anchor_index,
                    execution_route_rotation,
                ) = rotate_closed_route_for_ego(
                    route_override,
                    scene_route_arc,
                    route_pose,
                )
            validation = validate_scene(
                scenario, name, links_by_id, validation_route, runtime
            )
            effective_motion_phase = capture_overrides.get(
                "clip_motion_phase", family.get("motion_phase")
            )
            scene_split = split_for_variant(
                variant_index,
                int(family["count"]),
                config.get("split_ratios"),
            )
            if name == "crosswalk_pedestrian":
                scene_split = runtime[0].get("crosswalk_split") or scene_split
            if name == "east_corridor_relative_vehicle":
                scene_split = (
                    capture_overrides.get("spatial_split") or scene_split
                )

            code = FAMILY_CODES[name]
            scene_id = (
                f"{args.id_prefix}d{code}{variant_index + 1:03d}"
            )
            stem = (
                f"{config['scenario_prefix']}_{args.stem_prefix}"
                f"{code}{variant_index + 1:03d}"
            )
            scenario_path = scenario_dir / f"{stem}.json"
            metadata_path = metadata_dir / f"{stem}.metadata.json"
            route_path = None
            if execution_route is not None:
                route_path = route_dir / f"{stem}.waypoints.txt"
                write_waypoint_route(route_path, execution_route)
            write_json(scenario_path, scenario)
            is_singleframe_static = str(config["dataset_id"]).startswith(
                "morai_static_classcentric_base3_10k"
            )
            metadata = {
                "schema_version": 1,
                "dataset_id": config["dataset_id"],
                "scene_id": scene_id,
                "scenario_stem": stem,
                "mission_family": name,
                "motion_phase": effective_motion_phase,
                "sampling_role": family.get("sampling_role"),
                "recommended_phase_share_within_segment": family.get(
                    "recommended_phase_share_within_segment"
                ),
                "seed": scene_seed,
                "split": scene_split,
                "environment": config["environment"],
                "capture_policy": config["capture"],
                "capture_policy_overrides": capture_overrides,
                "ego": {
                    "lead_distance_m": lead,
                    "route_index": route_pose["route_index"],
                    "execution_route": execution_route_rotation,
                    "pose": ego_sample,
                    "initial_speed_kph": float(
                        scenario["egoVehicle"].get("currentVelocity", 0.0)
                    ),
                    "target_speed_kph": float(
                        capture_overrides.get("ego_target_speed_kph", 0.0)
                    ),
                },
                "runtime_actors": runtime,
                "target_position": target,
                "spatial_sampling": {
                    "policy": family.get("spatial_policy", "mission_conditioned"),
                    "route_wide_bin": (
                        route_wide_target.get("route_wide_bin")
                        if route_wide_target else None
                    ),
                    "route_wide_bin_count": (
                        route_wide_target.get("route_wide_bin_count")
                        if route_wide_target else None
                    ),
                    "route_arc_m": (
                        route_wide_target.get("route_arc_m")
                        if route_wide_target else None
                    ),
                    "crosswalk_id": (
                        route_wide_target.get("crosswalk_id")
                        if route_wide_target else None
                    ),
                    "spatial_stratum": spatial_details.get("spatial_stratum"),
                    "roundabout_phase": spatial_details.get("roundabout_phase"),
                    "roundabout_phase_within": spatial_details.get(
                        "roundabout_phase_within"
                    ),
                    "roundabout_phase_total": spatial_details.get(
                        "roundabout_phase_total"
                    ),
                    "highway_lane_id": spatial_details.get("highway_lane_id"),
                    "highway_phase": spatial_details.get("highway_phase"),
                    "highway_phase_variant": spatial_details.get(
                        "highway_phase_variant"
                    ),
                    "highway_phase_within": spatial_details.get(
                        "highway_phase_within"
                    ),
                    "highway_phase_total": spatial_details.get(
                        "highway_phase_total"
                    ),
                    "highway_target_link": spatial_details.get(
                        "highway_target_link"
                    ),
                    "highway_route_links": spatial_details.get(
                        "highway_route_links"
                    ),
                    "ordinary_two_lane_side": spatial_details.get(
                        "ordinary_two_lane_side"
                    ),
                    "ordinary_two_lane_variant": spatial_details.get(
                        "ordinary_two_lane_variant"
                    ),
                    "ordinary_two_lane_route_links": spatial_details.get(
                        "ordinary_two_lane_route_links"
                    ),
                    "ordinary_two_lane_target_link": spatial_details.get(
                        "ordinary_two_lane_target_link"
                    ),
                },
                "validation": validation,
                "sources": {
                    "official_template": str(args.template.resolve()),
                    "official_template_sha256": sha256(args.template),
                    "route": str(
                        route_path.resolve()
                        if route_path is not None
                        else args.route.resolve()
                    ),
                    "route_sha256": sha256(
                        route_path if route_path is not None else args.route
                    ),
                    "mgeo_link_set": str((args.mgeo_dir / "link_set.json").resolve()),
                    "mgeo_link_set_sha256": sha256(args.mgeo_dir / "link_set.json"),
                    "config": str(args.config.resolve()),
                    "config_sha256": sha256(args.config),
                },
                "notes": {
                    "velocity_ground_truth": (
                        "Raw and finite-difference motion are diagnostics only; "
                        "Base3 does not train a velocity target."
                        if is_singleframe_static
                        else "Record /Object_topic raw velocity and acceleration plus finite-difference derivatives."
                    ),
                    "split_contract": (
                        "Keep the three independent single-frame captures from "
                        "this layout clip in one split."
                        if is_singleframe_static
                        else "Keep every frame from this temporal clip in the same split."
                    ),
                },
            }
            write_json(metadata_path, metadata)
            if args.install_dir:
                args.install_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(scenario_path, args.install_dir / scenario_path.name)
            scenes.append(
                {
                    "id": scene_id,
                    "file": stem,
                    "file_name": stem,
                    "scenario": project_relative(scenario_path),
                    "metadata": project_relative(metadata_path),
                    "mission_family": name,
                    "motion_phase": effective_motion_phase,
                    "sampling_role": family.get("sampling_role"),
                    "sequence_contract": (
                        {
                            "clip_length": int(config["capture"]["captures_per_scene"]),
                            "capture_spacing_m": float(
                                config["capture"].get("capture_spacing_m", 3.0)
                            ),
                            "split_group": scene_id,
                            "training_mode": "independent_single_frame",
                            "temporal_training": False,
                            "pointpainting_sweep_count": 1,
                            "pointpainting_target_lags_sec": [0.0],
                            "exact_point_time_available": False,
                        }
                        if is_singleframe_static
                        else {
                            "clip_length": int(config["capture"]["captures_per_scene"]),
                            "nominal_period_sec": float(
                                config["capture"]["capture_period_sec"]
                            ),
                            "split_group": scene_id,
                            "pointpainting_sweep_count": 3,
                            "pointpainting_target_lags_sec": [
                                -index
                                * float(config["capture"]["capture_period_sec"])
                                for index in range(3)
                            ],
                            "exact_point_time_available": False,
                        }
                    ),
                    "split": metadata["split"],
                    "anchor_index": execution_anchor_index,
                    "waypoints": (
                        project_relative(route_path)
                        if route_path is not None
                        else project_relative(args.route)
                    ),
                    "capture_policy_overrides": capture_overrides,
                    "expected_ego": {
                        "x": float(ego_sample["x"]),
                        "y": float(ego_sample["y"]),
                        "z": float(ego_sample["z"]) + 0.34,
                    },
                    "expected_counts": {
                        "npc": len(scenario["vehicleList"]),
                        "pedestrian": len(scenario["pedestrianList"]),
                        "obstacle": len(scenario["objectList"]),
                    },
                    "dynamic_actor_policy": (
                        "observe_after_warmup"
                        if any(
                            actor.get("motion") != "static" for actor in runtime
                        ) or bool(scenario["spawnPointList"])
                        else "fixed_static"
                    ),
                    "dynamic_actor_ready_requires_motion": bool(
                        capture_overrides.get(
                            "dynamic_actor_ready_requires_motion",
                            any(
                                actor.get("motion") != "static"
                                for actor in runtime
                            ) and not bool(
                                name == "crosswalk_pedestrian"
                                and runtime[0].get("activation_policy")
                                == "deferred_ego_approach"
                            ),
                        )
                    ),
                    "capture_arm_requires_actor_motion": bool(
                        name == "crosswalk_pedestrian"
                        and runtime[0].get("activation_policy")
                        == "deferred_ego_approach"
                    ),
                    "ego_readiness_policy": (
                        "route_aligned"
                        if capture_overrides.get("minimum_ego_speed_kph", 0.0)
                        > 0.0
                        else "fixed_pose"
                    ),
                    "expected": {
                        "static_npc": len(scenario["vehicleList"]),
                        "pedestrian": len(scenario["pedestrianList"]),
                        "obstacle": len(scenario["objectList"]),
                        "spawn_points": len(scenario["spawnPointList"]),
                        "dynamic_actor_policy": (
                            "observe_after_warmup"
                            if any(
                                actor.get("motion") != "static"
                                for actor in runtime
                            ) or bool(scenario["spawnPointList"])
                            else "fixed_static"
                        ),
                    },
                }
            )

    manifest = {
        "schema_version": 1,
        "dataset_id": config["dataset_id"],
        "scenario_group": args.group,
        "environment": config["environment"],
        "sensor_config": "configs/sensors/Sensor_6.json",
        "sensor_config_sha256": sha256(
            PROJECT_ROOT / "configs/sensors/Sensor_6.json"
        ),
        "waypoints": str(args.route.relative_to(PROJECT_ROOT)),
        "capture_policy": config["capture"],
        "scenes": scenes,
                "generation": {
            "seed": seed,
            "scenario_count": len(scenes),
            "id_prefix": args.id_prefix,
            "stem_prefix": args.stem_prefix,
            "config": project_relative(args.config),
            "template": project_relative(args.template),
            "route_wide_route_source": config.get(
                "route_wide_route_source", "densified_guide"
            ),
            "route_reconstruction": route_reconstruction,
        },
    }
    write_json(manifest_path, manifest)
    print(f"generated scenarios: {len(scenes)}")
    print(f"scenario dir: {scenario_dir}")
    print(f"metadata dir: {metadata_dir}")
    print(f"manifest: {manifest_path}")
    if args.install_dir:
        print(f"installed: {args.install_dir}")


if __name__ == "__main__":
    main()
