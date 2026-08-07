#!/usr/bin/env python3
"""Generate 30 paired MORAI OD base scenarios and six environment manifests."""

import argparse
import copy
import hashlib
import json
import math
import random
import shutil
from collections import Counter
from pathlib import Path

import yaml

from generate_morai_od_prototype import (
    build_link_samples,
    heading_degrees,
    is_clear,
    load_json,
    load_waypoints,
    make_pose_dict,
    nearest_sample,
    set_obstacle_pose,
    set_pedestrian_pose,
    set_vehicle_pose,
    validate_scenario,
    wrap_degrees,
)
from generate_ros_capture_poc_set import (
    EGO_SIDECAR_REQUIRED_ROS_TOPICS,
    build_ego_only_network_template,
)


DATASET_ID = "morai_od180_v1"
SCENARIO_GROUP = "od180_base"
MANIFEST_STEM = "od180"
SCENE_ID_PREFIX = "base"
SCENARIO_FILE_PREFIX = "od180_b"
BASE_SCENARIO_COUNT = 30
SPLIT_COUNTS = {"train": 20, "val": 5, "test": 5}
MINIMUM_ANCHOR_ARC_DISTANCE_M = 20.0
ANCHOR_CANDIDATE_SPACING_M = 4.0
MASTER_SEED = 26080730
ENVIRONMENTS = (
    {"id": "sunny_t11", "weather": "sunny", "time_hour": 11},
    {"id": "sunny_t13", "weather": "sunny", "time_hour": 13},
    {"id": "sunny_t15", "weather": "sunny", "time_hour": 15},
    {
        "id": "foggy_t11",
        "weather": "foggy",
        "time_hour": 11,
        "foggy_density": 1,
        "foggy_distance": 0,
    },
    {
        "id": "foggy_t13",
        "weather": "foggy",
        "time_hour": 13,
        "foggy_density": 1,
        "foggy_distance": 0,
    },
    {
        "id": "foggy_t15",
        "weather": "foggy",
        "time_hour": 15,
        "foggy_density": 1,
        "foggy_distance": 0,
    },
)


SCENE_TYPE_SPECS = (
    {
        "name": "pedestrian_centric",
        "count": 8,
        "splits": ("train",) * 5 + ("val",) * 2 + ("test",),
        "npc_range": (0, 2),
        "pedestrian_range": (5, 8),
        "obstacle_range": (0, 2),
        "topology_preference": (
            "crosswalk",
            "roadside_parking_clutter",
            "intersection",
        ),
    },
    {
        "name": "vehicle_centric",
        "count": 6,
        "splits": ("train",) * 4 + ("val",) + ("test",),
        "npc_range": (5, 8),
        "pedestrian_range": (0, 1),
        "obstacle_range": (0, 2),
        "topology_preference": ("straight", "intersection", "merge_diverge"),
    },
    {
        "name": "mixed_vehicle_pedestrian",
        "count": 9,
        "splits": ("train",) * 6 + ("val",) + ("test",) * 2,
        "npc_range": (3, 5),
        "pedestrian_range": (3, 6),
        "obstacle_range": (1, 3),
        "topology_preference": ("intersection", "merge_diverge", "curve"),
    },
    {
        "name": "obstacle_context",
        "count": 3,
        "splits": ("train",) * 2 + ("val",),
        "npc_range": (0, 2),
        "pedestrian_range": (0, 0),
        "obstacle_range": (3, 4),
        "topology_preference": ("roadside_parking_clutter", "straight"),
    },
    {
        "name": "camera_seam_occlusion",
        "count": 3,
        "splits": ("train",) * 2 + ("test",),
        "npc_range": (2, 4),
        "pedestrian_range": (3, 5),
        "obstacle_range": (0, 2),
        "topology_preference": ("curve", "intersection"),
    },
    {
        "name": "sparse_empty_background",
        "count": 1,
        "splits": ("train",),
        "npc_range": (0, 0),
        "pedestrian_range": (0, 0),
        "obstacle_range": (0, 0),
        "topology_preference": ("straight",),
    },
)


TOPOLOGY_TARGETS = (
    ("crosswalk", 5),
    ("intersection", 5),
    ("merge_diverge", 4),
    ("roadside_parking_clutter", 4),
    ("curve", 5),
    ("straight", 7),
)


VEHICLE_Z_LIFT = {
    "passenger_car": 0.34,
    "electric_passenger_car": 0.35,
    "suv": 0.37,
    "wagon": 0.35,
    "mpv": 0.38,
    "bus_van": 0.48,
    "truck": 0.55,
}


VEHICLE_RADIUS = {
    "passenger_car": 2.7,
    "electric_passenger_car": 2.7,
    "suv": 2.9,
    "wagon": 2.8,
    "mpv": 3.0,
    "bus_van": 4.5,
    "truck": 5.0,
}


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    personal_root = Path.home() / "MoraiLauncher_Lin_per"
    parser = argparse.ArgumentParser(
        description=(
            "Generate 30 base OD scenarios, split files, and six paired "
            "weather/time manifests."
        )
    )
    parser.add_argument("--project-root", type=Path, default=project_root)
    parser.add_argument(
        "--config",
        type=Path,
        default=project_root
        / "dataset/morai_od/configs/od_180_pilot.yaml",
    )
    parser.add_argument(
        "--asset-scenario-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/"
            "Scenario/R_KR_PR_K-city_2025"
        ),
    )
    parser.add_argument(
        "--runtime-asset-scenario-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/"
            "Scenario/R_KR_PR_K-city_2025"
        ),
        help=(
            "Scenario directory containing the runtime launcher's curated "
            "asset templates. Common competition assets are instantiated "
            "from these personal-launcher templates."
        ),
    )
    parser.add_argument(
        "--asset-compatibility",
        type=Path,
        default=project_root
        / (
            "dataset/morai_od/configs/"
            "morai_asset_launcher_compatibility_260807.json"
        ),
    )
    parser.add_argument(
        "--install-dir",
        type=Path,
        default=None,
        help=(
            "Optional MORAI current-map Scenario directory. When supplied, "
            "the 30 JSON files and Ego-only ROS network sidecars are installed."
        ),
    )
    parser.add_argument(
        "--network-template",
        type=Path,
        default=personal_root
        / (
            "MoraiLauncher_Lin_Data/SaveFile/Scenario/"
            "R_KR_PR_K-city_2025/EgoNetwork/ros_path_probe_MN.json"
        ),
    )
    parser.add_argument("--seed", type=int, default=MASTER_SEED)
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def distance_xy(left, right):
    return math.hypot(left[0] - right[0], left[1] - right[1])


def path_centroid(points):
    values = [point for point in points if len(point) >= 2]
    return (
        sum(point[0] for point in values) / len(values),
        sum(point[1] for point in values) / len(values),
    )


def nearest_distance(point, candidates):
    if not candidates:
        return float("inf")
    return min(distance_xy(point, candidate) for candidate in candidates)


def route_arc_lengths(waypoints):
    values = [0.0]
    for before, after in zip(waypoints, waypoints[1:]):
        values.append(values[-1] + distance_xy(before, after))
    return values


def index_near_arc(arc_lengths, target):
    return min(
        range(len(arc_lengths)),
        key=lambda index: abs(arc_lengths[index] - target),
    )


def route_curvature_deg(waypoints, arc_lengths, index, window_m=8.0):
    center = arc_lengths[index]
    before = index_near_arc(arc_lengths, max(0.0, center - window_m))
    after = index_near_arc(
        arc_lengths, min(arc_lengths[-1], center + window_m)
    )
    if before == index or after == index:
        return 0.0
    heading_before = heading_degrees(waypoints[before], waypoints[index])
    heading_after = heading_degrees(waypoints[index], waypoints[after])
    return abs(wrap_degrees(heading_after - heading_before))


def candidate_route_indices(arc_lengths, spacing_m=4.0):
    indices = []
    distance = 12.0
    stop = arc_lengths[-1] - 12.0
    while distance <= stop:
        index = index_near_arc(arc_lengths, distance)
        if not indices or index != indices[-1]:
            indices.append(index)
        distance += spacing_m
    return indices


def mgeo_feature_points(mgeo_dir):
    crosswalks = load_json(mgeo_dir / "singlecrosswalk_set.json")
    parking = load_json(mgeo_dir / "parking_space_set.json")
    nodes = load_json(mgeo_dir / "node_set.json")
    links = load_json(mgeo_dir / "link_set.json")

    crosswalk_centers = [
        path_centroid(item["points"])
        for item in crosswalks
        if item.get("points")
    ]
    parking_centers = [
        tuple(item.get("center_point", path_centroid(item["points"]))[:2])
        for item in parking
        if item.get("points")
    ]
    stopline_points = [
        tuple(item["point"][:2])
        for item in nodes
        if item.get("on_stop_line")
    ]
    degree = Counter()
    node_points = {item["idx"]: tuple(item["point"][:2]) for item in nodes}
    for link in links:
        degree[link.get("from_node_idx")] += 1
        degree[link.get("to_node_idx")] += 1
    branch_points = [
        node_points[node_id]
        for node_id, value in degree.items()
        if value >= 3 and node_id in node_points
    ]
    return {
        "crosswalk_centers": crosswalk_centers,
        "parking_centers": parking_centers,
        "stopline_points": stopline_points,
        "branch_points": branch_points,
        "crosswalk_polygons": [
            item["points"] for item in crosswalks if item.get("points")
        ],
    }


def build_anchor_candidates(
    waypoints,
    arc_lengths,
    features,
    spacing_m=ANCHOR_CANDIDATE_SPACING_M,
):
    candidates = []
    segment_lengths = [
        distance_xy(before, after)
        for before, after in zip(waypoints, waypoints[1:])
    ]
    for index in candidate_route_indices(arc_lengths, spacing_m=spacing_m):
        # The official path currently contains one large discontinuity.
        # Exclude a generous neighborhood so route-follow never aims across it.
        check_start = max(0, index - 20)
        check_stop = min(len(segment_lengths), index + 30)
        if any(
            length > 2.0
            for length in segment_lengths[check_start:check_stop]
        ):
            continue
        point = waypoints[index]
        candidates.append(
            {
                "index": index,
                "arc_m": arc_lengths[index],
                "point": point,
                "curvature_deg": route_curvature_deg(
                    waypoints, arc_lengths, index
                ),
                "crosswalk_distance_m": nearest_distance(
                    point, features["crosswalk_centers"]
                ),
                "parking_distance_m": nearest_distance(
                    point, features["parking_centers"]
                ),
                "stopline_distance_m": nearest_distance(
                    point, features["stopline_points"]
                ),
                "branch_distance_m": nearest_distance(
                    point, features["branch_points"]
                ),
            }
        )
    return candidates


def topology_sort_key(name, candidate):
    if name == "crosswalk":
        return (
            candidate["crosswalk_distance_m"],
            -candidate["curvature_deg"],
        )
    if name == "intersection":
        return (
            min(
                candidate["stopline_distance_m"],
                candidate["branch_distance_m"],
            ),
            candidate["crosswalk_distance_m"],
        )
    if name == "merge_diverge":
        return (
            candidate["branch_distance_m"],
            -candidate["curvature_deg"],
        )
    if name == "roadside_parking_clutter":
        return (
            candidate["parking_distance_m"],
            candidate["curvature_deg"],
        )
    if name == "curve":
        return (-candidate["curvature_deg"],)
    return (
        candidate["curvature_deg"],
        -min(
            candidate["crosswalk_distance_m"],
            candidate["stopline_distance_m"],
            candidate["branch_distance_m"],
        ),
    )


def select_anchors(candidates, minimum_arc_distance_m=20.0):
    selected = []
    used_indices = set()
    for topology, count in TOPOLOGY_TARGETS:
        ordered = sorted(
            candidates, key=lambda item: topology_sort_key(topology, item)
        )
        added = 0
        for candidate in ordered:
            if candidate["index"] in used_indices:
                continue
            if any(
                abs(candidate["arc_m"] - item["arc_m"])
                < minimum_arc_distance_m
                for item in selected
            ):
                continue
            selected.append({**candidate, "topology": topology})
            used_indices.add(candidate["index"])
            added += 1
            if added == count:
                break
        if added != count:
            raise RuntimeError(
                "Could select only {}/{} anchors for {}".format(
                    added, count, topology
                )
            )
    if len(selected) != BASE_SCENARIO_COUNT:
        raise AssertionError(
            "anchor selection did not produce {} scenes".format(
                BASE_SCENARIO_COUNT
            )
        )
    return selected


def make_scene_specs(rng, anchors):
    remaining = list(anchors)
    pending = []
    for type_spec in SCENE_TYPE_SPECS:
        for local_index in range(type_spec["count"]):
            pending.append((type_spec, local_index))

    # Allocate the most restrictive topology preferences first. Otherwise
    # broad mixed scenes can consume every straight anchor before the one
    # sparse/empty control scene is assigned.
    pending.sort(
        key=lambda item: (
            len(item[0]["topology_preference"]),
            item[0]["name"],
            item[1],
        )
    )
    specs = []
    for type_spec, local_index in pending:
        anchor = None
        for preferred in type_spec["topology_preference"]:
            anchor = next(
                (
                    item
                    for item in remaining
                    if item["topology"] == preferred
                ),
                None,
            )
            if anchor is not None:
                break
        if anchor is None:
            anchor = remaining[0]
        remaining.remove(anchor)

        npc = rng.randint(*type_spec["npc_range"])
        pedestrian = rng.randint(*type_spec["pedestrian_range"])
        obstacle = rng.randint(*type_spec["obstacle_range"])
        while npc + pedestrian + obstacle > 14:
            if pedestrian > type_spec["pedestrian_range"][0]:
                pedestrian -= 1
            elif npc > type_spec["npc_range"][0]:
                npc -= 1
            else:
                obstacle -= 1
        specs.append(
            {
                "scene_type": type_spec["name"],
                "split": type_spec["splits"][local_index],
                "anchor": anchor,
                "npc_count": npc,
                "pedestrian_count": pedestrian,
                "obstacle_count": obstacle,
            }
        )
    if remaining or len(specs) != BASE_SCENARIO_COUNT:
        raise AssertionError(
            "scene specification did not consume {} anchors".format(
                BASE_SCENARIO_COUNT
            )
        )
    # Do not let category order become a hidden split or route-order cue.
    rng.shuffle(specs)
    pedestrian_specs = [
        item for item in specs if item["pedestrian_count"] > 0
    ]
    ncap_specs = [
        next(item for item in pedestrian_specs if item["split"] == "train"),
        next(item for item in pedestrian_specs if item["split"] != "train"),
    ]
    for item in specs:
        item["use_ncap"] = any(item is selected for selected in ncap_specs)
    return specs


class AssetDeck:
    def __init__(self, rows, source_by_id, seed):
        self.rows = list(rows)
        self.source_by_id = source_by_id
        self.rng = random.Random(seed)
        self.deck = []
        self.cursor = 0
        self._reshuffle()

    def _reshuffle(self):
        self.deck = list(self.rows)
        self.rng.shuffle(self.deck)
        self.cursor = 0

    def take(self, count):
        chosen = []
        chosen_ids = set()
        while len(chosen) < count:
            if self.cursor >= len(self.deck):
                self._reshuffle()
            row = self.deck[self.cursor]
            self.cursor += 1
            if row["data_id"] in chosen_ids and len(self.rows) >= count:
                continue
            chosen.append((row, copy.deepcopy(self.source_by_id[row["data_id"]])))
            chosen_ids.add(row["data_id"])
        return chosen


def source_actor_map(document, source_list):
    result = {}
    for actor in document.get(source_list, []):
        result.setdefault(int(actor["DataID"]), actor)
    return result


def local_coordinates(ego, point):
    yaw = math.radians(ego["yaw"])
    dx = point["x"] - ego["x"]
    dy = point["y"] - ego["y"]
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
    )


def in_ego_swept_corridor(ego, point, extra_margin=0.0):
    longitudinal, lateral = local_coordinates(ego, point)
    return (
        -2.0 - extra_margin
        <= longitudinal
        <= 9.0 + extra_margin
        and abs(lateral) <= 1.7 + extra_margin
    )


def weighted_distance(rng):
    value = rng.random()
    if value < 0.25:
        return rng.uniform(3.5, 8.0)
    if value < 0.60:
        return rng.uniform(8.0, 15.0)
    if value < 0.90:
        return rng.uniform(15.0, 22.0)
    return rng.uniform(22.0, 28.0)


def choose_vehicle_samples(rng, samples, ego, assets, occupied):
    candidates = [
        item
        for item in samples
        if 5.0 <= math.hypot(item["x"] - ego["x"], item["y"] - ego["y"]) <= 28.0
        and not in_ego_swept_corridor(ego, item, extra_margin=1.0)
    ]
    selected = []
    for row, actor in assets:
        radius = VEHICLE_RADIUS.get(row["subtype"], 3.0)
        target_distance = weighted_distance(rng)
        ordered = list(candidates)
        rng.shuffle(ordered)
        ordered.sort(
            key=lambda item: abs(
                math.hypot(item["x"] - ego["x"], item["y"] - ego["y"])
                - target_distance
            )
        )
        sample = next(
            (
                item
                for item in ordered
                if is_clear(item, occupied, radius + 0.8)
            ),
            None,
        )
        if sample is None:
            raise RuntimeError("No collision-free MGeo sample for vehicle")
        occupied.append({"x": sample["x"], "y": sample["y"], "radius": radius})
        selected.append((row, actor, sample))
    return selected


def point_in_polygon(x, y, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous[0], previous[1]
        x2, y2 = current[0], current[1]
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def random_point_in_polygon(rng, polygon):
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    for _ in range(100):
        x = rng.uniform(min(xs), max(xs))
        y = rng.uniform(min(ys), max(ys))
        if point_in_polygon(x, y, polygon):
            return x, y
    return path_centroid(polygon)


def nearby_crosswalks(features, ego, radius=28.0):
    result = []
    for polygon in features["crosswalk_polygons"]:
        center = path_centroid(polygon)
        if distance_xy((ego["x"], ego["y"]), center) <= radius:
            result.append(polygon)
    return result


def choose_pedestrian_poses(
    rng,
    samples,
    features,
    ego,
    count,
    occupied,
    scene_type,
    topology,
):
    selected = []
    crosswalks = nearby_crosswalks(features, ego)
    seam_angles = (45.0, -45.0, 135.0, -135.0)
    for index in range(count):
        pose = None
        for _ in range(300):
            use_crosswalk = (
                topology == "crosswalk"
                and crosswalks
                and rng.random() < 0.70
            )
            if use_crosswalk:
                x, y = random_point_in_polygon(rng, rng.choice(crosswalks))
            else:
                radius = weighted_distance(rng)
                if scene_type == "camera_seam_occlusion":
                    angle = seam_angles[index % len(seam_angles)]
                    angle += rng.uniform(-6.0, 6.0)
                else:
                    lateral_sign = rng.choice((-1.0, 1.0))
                    lateral = lateral_sign * rng.uniform(4.0, min(7.0, radius))
                    longitudinal = math.sqrt(
                        max(0.0, radius * radius - lateral * lateral)
                    )
                    longitudinal *= rng.choice((-1.0, 1.0))
                    angle = math.degrees(math.atan2(lateral, longitudinal))
                global_angle = math.radians(ego["yaw"] + angle)
                x = ego["x"] + math.cos(global_angle) * radius
                y = ego["y"] + math.sin(global_angle) * radius
            raw = {"x": x, "y": y}
            if in_ego_swept_corridor(ego, raw, extra_margin=0.4):
                continue
            road = nearest_sample(samples, x, y)
            candidate = {"x": x, "y": y, "z": road["z"] + 0.05}
            if not is_clear(candidate, occupied, required_clearance=1.0):
                continue
            pose = {
                **candidate,
                "yaw": wrap_degrees(ego["yaw"] + rng.uniform(-180.0, 180.0)),
            }
            break
        if pose is None:
            raise RuntimeError("Could not place pedestrian")
        occupied.append({"x": pose["x"], "y": pose["y"], "radius": 0.45})
        selected.append(pose)
    return selected


def choose_obstacle_poses(rng, samples, ego, assets, occupied):
    selected = []
    for row, actor in assets:
        pose = None
        subtype = row["subtype"]
        for _ in range(300):
            distance = weighted_distance(rng)
            if subtype == "road_surface":
                longitudinal = rng.choice((-1.0, 1.0)) * max(10.0, distance)
                lateral = rng.choice((-1.0, 1.0)) * rng.uniform(2.5, 4.0)
            else:
                longitudinal = rng.uniform(-min(20.0, distance), min(20.0, distance))
                lateral_mag = math.sqrt(
                    max(0.0, distance * distance - longitudinal * longitudinal)
                )
                lateral = rng.choice((-1.0, 1.0)) * max(4.0, lateral_mag)
            yaw = math.radians(ego["yaw"])
            x = (
                ego["x"]
                + math.cos(yaw) * longitudinal
                - math.sin(yaw) * lateral
            )
            y = (
                ego["y"]
                + math.sin(yaw) * longitudinal
                + math.cos(yaw) * lateral
            )
            raw = {"x": x, "y": y}
            if math.hypot(x - ego["x"], y - ego["y"]) > 28.0:
                continue
            if in_ego_swept_corridor(ego, raw, extra_margin=0.5):
                continue
            road = nearest_sample(samples, x, y)
            candidate = {
                "x": x,
                "y": y,
                "z": road["z"],
                "yaw": (
                    road["yaw"]
                    if subtype == "road_surface"
                    else wrap_degrees(ego["yaw"] + rng.uniform(-20.0, 20.0))
                ),
            }
            if not is_clear(candidate, occupied, required_clearance=1.7):
                continue
            pose = candidate
            break
        if pose is None:
            raise RuntimeError("Could not place obstacle {}".format(row["name"]))
        occupied.append({"x": pose["x"], "y": pose["y"], "radius": 1.0})
        selected.append((row, actor, pose))
    return selected


def choose_ego_sample(waypoints, samples, anchor_index):
    anchor = waypoints[anchor_index]
    nearest = nearest_sample(samples, anchor[0], anchor[1])
    before = waypoints[max(0, anchor_index - 1)]
    after = waypoints[min(len(waypoints) - 1, anchor_index + 1)]
    return {
        **nearest,
        "x": anchor[0],
        "y": anchor[1],
        "z": anchor[2],
        "yaw": heading_degrees(before, after),
    }


def clear_template_lists(scenario):
    for key in (
        "vehicleList",
        "pedestrianList",
        "objectList",
        "spawnPointList",
        "pedspawnPointList",
        "aiPedstrianList",
        "shadedAreaList",
        "frictionChangeMaterialList",
        "waypointDataList",
    ):
        scenario[key] = []


def actor_record(row, unique_id, group):
    return {
        "unique_id": unique_id,
        "group": group,
        "data_id": row["data_id"],
        "name": row["name"],
        "ontology_class": row["ontology_class"],
        "subtype": row["subtype"],
        "generation_policy": row["generation_policy"],
    }


def actor_record_with_pose(row, unique_id, group, position, yaw):
    record = actor_record(row, unique_id, group)
    record["pose"] = {
        "x": float(position["x"]),
        "y": float(position["y"]),
        "z": float(position["z"]),
        "yaw": float(yaw),
    }
    return record


def generate_scene(
    template,
    spec,
    scene_index,
    seed,
    waypoints,
    samples,
    link_ids,
    features,
    vehicle_deck,
    pedestrian_deck,
    ncap_deck,
    obstacle_deck,
):
    rng = random.Random(seed)
    ego = choose_ego_sample(
        waypoints, samples, spec["anchor"]["index"]
    )
    scenario = copy.deepcopy(template)
    clear_template_lists(scenario)
    set_vehicle_pose(
        scenario["egoVehicle"], ego, unique_id=-1, z_lift=0.34, ego=True
    )
    scenario["egoVehicle"]["customValue"] = 0.0
    scenario["egoVehicle"]["linkValue"] = ego["max_speed"]

    occupied = [{"x": ego["x"], "y": ego["y"], "radius": 3.2}]
    assets = []

    vehicle_assets = vehicle_deck.take(spec["npc_count"])
    vehicle_placements = choose_vehicle_samples(
        rng, samples, ego, vehicle_assets, occupied
    )
    for actor_index, (row, actor, sample) in enumerate(vehicle_placements):
        unique_id = 1001 + actor_index
        set_vehicle_pose(
            actor,
            sample,
            unique_id=unique_id,
            z_lift=VEHICLE_Z_LIFT.get(row["subtype"], 0.36),
            ego=False,
        )
        scenario["vehicleList"].append(actor)
        assets.append(
            actor_record_with_pose(
                row, unique_id, "vehicleList", actor["initPosition"]["pos"],
                sample["yaw"]
            )
        )

    pedestrian_count = spec["pedestrian_count"]
    pedestrian_assets = pedestrian_deck.take(pedestrian_count)
    # Two pedestrian-bearing scenes receive one NCAP dummy, remaining below
    # the configured 10% scene fraction.
    if pedestrian_count and spec["use_ncap"]:
        pedestrian_assets[-1] = ncap_deck.take(1)[0]
    pedestrian_poses = choose_pedestrian_poses(
        rng,
        samples,
        features,
        ego,
        pedestrian_count,
        occupied,
        spec["scene_type"],
        spec["anchor"]["topology"],
    )
    for actor_index, ((row, actor), pose) in enumerate(
        zip(pedestrian_assets, pedestrian_poses)
    ):
        unique_id = 2001 + actor_index
        set_pedestrian_pose(actor, pose, pose["yaw"], unique_id)
        scenario["pedestrianList"].append(actor)
        assets.append(
            actor_record_with_pose(
                row, unique_id, "pedestrianList", actor["pos"], pose["yaw"]
            )
        )

    obstacle_assets = obstacle_deck.take(spec["obstacle_count"])
    obstacle_placements = choose_obstacle_poses(
        rng, samples, ego, obstacle_assets, occupied
    )
    for actor_index, (row, actor, pose) in enumerate(obstacle_placements):
        unique_id = 3001 + actor_index
        set_obstacle_pose(
            actor,
            pose,
            unique_id=unique_id,
            scale=1.0,
        )
        actor["rot"]["yaw"] = "{:.3f}".format(pose["yaw"])
        scenario["objectList"].append(actor)
        assets.append(
            actor_record_with_pose(
                row, unique_id, "objectList", actor["pos"], pose["yaw"]
            )
        )

    expected = {
        "vehicleList": spec["npc_count"],
        "pedestrianList": spec["pedestrian_count"],
        "objectList": spec["obstacle_count"],
    }
    validate_scenario(
        scenario, ego, radius=28.0, link_ids=link_ids, expected_counts=expected
    )
    return scenario, ego, assets, expected


def capture_policy(config):
    policy = config["capture"]
    return {
        "captures_per_scene": int(policy["frames_per_run"]),
        "capture_spacing_m": float(policy["spacing_m"]),
        "ego_target_speed_kph": float(policy["ego_speed_kph"]),
        "maximum_motion_m": float(policy["maximum_motion_m"]),
        "recommended_motion_mode": "rolling-grouped",
        "load_settle_sec": float(policy["load_settle_sec"]),
        "capture_stable_sec": float(policy["capture_stable_sec"]),
        "continue_on_scene_error": bool(policy["continue_on_scene_error"]),
        "note": (
            "Use rolling-grouped for training data. It sends captures while "
            "moving, then groups and verifies Camera/LiDAR/GT artifacts by "
            "their shared MORAI timestamp."
        ),
    }


def write_json(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_manifest(
    scenes,
    network_sha,
    policy,
    environment=None,
):
    return {
        "schema_version": 3,
        "dataset_id": DATASET_ID,
        "purpose": (
            "{} MORAI base scenarios for PointPainting and future "
            "multi-sensor BEV fine-tuning".format(BASE_SCENARIO_COUNT)
        ),
        "map_name": "R_KR_PR_K-city_2025",
        "base_scenario_count": BASE_SCENARIO_COUNT,
        "environment": environment or {
            "id": "base_only",
            "note": "Use an environment batch manifest to capture.",
        },
        "network_policy": {
            "transport": "rosbridge",
            "load_network_connection_data": True,
            "sidecar_scope": "ego_only",
            "required_unique_ids": [0],
            "forbidden_unique_ids": [-100],
            "required_topics": list(EGO_SIDECAR_REQUIRED_ROS_TOPICS),
            "template_sha256": network_sha,
            "note": (
                "Load the Simulator Network once. Each ScenarioLoad restores "
                "only the Ego Network from EgoNetwork/<stem>_MN.json."
            ),
        },
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": policy,
        "scenes": scenes,
    }


def main():
    args = parse_args()
    project_root = args.project_root.resolve()
    workspace = project_root / "dataset/morai_od"
    config_path = args.config.expanduser().resolve()
    catalog_path = workspace / "configs/morai_asset_catalog_260807.json"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    catalog = load_json(catalog_path)
    compatibility_path = args.asset_compatibility.expanduser().resolve()
    compatibility = load_json(compatibility_path)

    if config["capture"]["unique_geometry_layouts"] != BASE_SCENARIO_COUNT:
        raise ValueError(
            "{} must request {} base layouts".format(
                config_path, BASE_SCENARIO_COUNT
            )
        )
    if (
        config["capture"]["environment_variants_per_layout"]
        != len(ENVIRONMENTS)
    ):
        raise ValueError(
            "{} must request {} environments".format(
                config_path, len(ENVIRONMENTS)
            )
        )

    asset_dir = args.asset_scenario_dir.expanduser().resolve()
    vehicle_path = asset_dir / catalog["generated_from"]["vehicle"]["file_name"]
    pedestrian_path = (
        asset_dir / catalog["generated_from"]["pedestrian"]["file_name"]
    )
    obstacle_path = (
        asset_dir / catalog["generated_from"]["obstacle"]["file_name"]
    )
    for path, key in (
        (vehicle_path, "vehicle"),
        (pedestrian_path, "pedestrian"),
        (obstacle_path, "obstacle"),
    ):
        if not path.is_file():
            raise FileNotFoundError(path)
        if sha256(path) != catalog["generated_from"][key]["sha256"]:
            raise ValueError(
                "asset scenario changed since catalog extraction: {}".format(
                    path
                )
            )

    runtime_asset_dir = (
        args.runtime_asset_scenario_dir.expanduser().resolve()
    )
    runtime_sources = compatibility["personal"]["generated_from"]
    runtime_paths = {
        key: runtime_asset_dir / runtime_sources[key]["file_name"]
        for key in ("vehicle", "pedestrian", "obstacle")
    }
    for key, path in runtime_paths.items():
        if not path.is_file():
            raise FileNotFoundError(path)
        if sha256(path) != runtime_sources[key]["sha256"]:
            raise ValueError(
                "runtime asset scenario changed since compatibility "
                "extraction: {}".format(path)
            )

    vehicle_source = source_actor_map(
        load_json(runtime_paths["vehicle"]), "vehicleList"
    )
    pedestrian_source = source_actor_map(
        load_json(runtime_paths["pedestrian"]), "pedestrianList"
    )
    obstacle_source = source_actor_map(
        load_json(runtime_paths["obstacle"]), "objectList"
    )
    automatic_ids = {
        key: set(values)
        for key, values in compatibility["automatic_eligible"].items()
    }

    vehicle_rows = [
        row
        for row in catalog["vehicles"]
        if row["generation_policy"] == "enabled"
        and row["data_id"] in automatic_ids["vehicle"]
    ]
    pedestrian_rows = [
        row
        for row in catalog["pedestrians_and_related"]
        if row["source_list"] == "pedestrianList"
        and row["generation_policy"] == "enabled"
        and row["data_id"] in automatic_ids["pedestrian"]
    ]
    ncap_rows = [
        row
        for row in catalog["pedestrians_and_related"]
        if row["source_list"] == "pedestrianList"
        and row["generation_policy"] == "limited_weight"
        and row["data_id"] in automatic_ids["pedestrian"]
    ]
    obstacle_rows = [
        row
        for row in catalog["obstacles_and_related"]
        if row["generation_policy"].startswith("enabled")
        and row["data_id"] in automatic_ids["obstacle"]
    ]
    for rows, source, name in (
        (vehicle_rows, vehicle_source, "vehicle"),
        (pedestrian_rows + ncap_rows, pedestrian_source, "pedestrian"),
        (obstacle_rows, obstacle_source, "obstacle"),
    ):
        missing = sorted(row["data_id"] for row in rows if row["data_id"] not in source)
        if missing:
            raise ValueError("{} source actors missing: {}".format(name, missing))

    template_path = (
        project_root / "configs/scenario/2026_molit_comp_sample_scene.json"
    )
    waypoints_path = (
        project_root
        / "configs/global_path/2026_molit_comp_global_path.txt"
    )
    mgeo_dir = project_root / "kcity_map/mgeo_map_scenario_runner_2025"
    template = load_json(template_path)
    waypoints = load_waypoints(waypoints_path)
    links = load_json(mgeo_dir / "link_set.json")
    samples = build_link_samples(links)
    link_ids = {str(link["idx"]) for link in links}
    arc_lengths = route_arc_lengths(waypoints)
    features = mgeo_feature_points(mgeo_dir)
    candidates = build_anchor_candidates(
        waypoints,
        arc_lengths,
        features,
        spacing_m=ANCHOR_CANDIDATE_SPACING_M,
    )
    anchors = select_anchors(
        candidates,
        minimum_arc_distance_m=MINIMUM_ANCHOR_ARC_DISTANCE_M,
    )

    rng = random.Random(args.seed)
    specs = make_scene_specs(rng, anchors)
    vehicle_deck = AssetDeck(vehicle_rows, vehicle_source, args.seed + 1)
    pedestrian_deck = AssetDeck(
        pedestrian_rows, pedestrian_source, args.seed + 2
    )
    ncap_deck = AssetDeck(ncap_rows, pedestrian_source, args.seed + 3)
    obstacle_deck = AssetDeck(obstacle_rows, obstacle_source, args.seed + 4)

    network_path = args.network_template.expanduser().resolve()
    _network_document, network_payload = build_ego_only_network_template(
        network_path
    )
    network_sha = hashlib.sha256(network_payload).hexdigest()

    scenario_dir = workspace / "scenarios" / SCENARIO_GROUP
    metadata_dir = workspace / "metadata" / SCENARIO_GROUP
    manifest_dir = workspace / "manifests"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    manifest_dir.mkdir(parents=True, exist_ok=True)

    scenes = []
    split_ids = {"train": [], "val": [], "test": []}
    coverage = Counter()
    for scene_index, spec in enumerate(specs, start=1):
        base_id = "{}{:03d}".format(SCENE_ID_PREFIX, scene_index)
        stem = "{}{:03d}".format(SCENARIO_FILE_PREFIX, scene_index)
        initial_scene_seed = args.seed * 100 + scene_index
        generation_error = None
        for generation_attempt in range(20):
            scene_seed = initial_scene_seed + generation_attempt * 1000003
            try:
                scenario, ego, assets, expected = generate_scene(
                    template,
                    spec,
                    scene_index,
                    scene_seed,
                    waypoints,
                    samples,
                    link_ids,
                    features,
                    vehicle_deck,
                    pedestrian_deck,
                    ncap_deck,
                    obstacle_deck,
                )
                generation_error = None
                break
            except (RuntimeError, ValueError) as error:
                generation_error = error
        if generation_error is not None:
            raise RuntimeError(
                "Could not generate scene {} after 20 attempts: {}".format(
                    base_id, generation_error
                )
            )
        scenario_path = scenario_dir / (stem + ".json")
        metadata_path = metadata_dir / (stem + ".metadata.json")
        write_json(scenario_path, scenario)

        for asset in assets:
            coverage[(asset["group"], asset["data_id"], asset["name"])] += 1
        metadata = {
            "schema_version": 1,
            "dataset_id": DATASET_ID,
            "base_scenario_id": base_id,
            "scenario_stem": stem,
            "split": spec["split"],
            "scene_type": spec["scene_type"],
            "topology": spec["anchor"]["topology"],
            "seed": scene_seed,
            "generation_attempt": generation_attempt + 1,
            "anchor": {
                "index": spec["anchor"]["index"],
                "arc_m": spec["anchor"]["arc_m"],
                "curvature_deg": spec["anchor"]["curvature_deg"],
                "crosswalk_distance_m": spec["anchor"][
                    "crosswalk_distance_m"
                ],
                "parking_distance_m": spec["anchor"]["parking_distance_m"],
                "stopline_distance_m": spec["anchor"][
                    "stopline_distance_m"
                ],
                "branch_distance_m": spec["anchor"]["branch_distance_m"],
            },
            "ego": {
                "x": ego["x"],
                "y": ego["y"],
                "z": ego["z"] + 0.34,
                "yaw": ego["yaw"],
                "link_idx": ego["link_idx"],
                "point_idx": ego["point_idx"],
            },
            "counts": {
                "npc": expected["vehicleList"],
                "pedestrian": expected["pedestrianList"],
                "obstacle": expected["objectList"],
            },
            "assets": assets,
            "environment_policy": (
                "Weather/time are not embedded in this JSON. Capture it in "
                "the environment variants declared by the manifest."
            ),
            "source_hashes": {
                "template": sha256(template_path),
                "waypoints": sha256(waypoints_path),
                "sensor_config": sha256(
                    project_root / "configs/sensors/Sensor_6.json"
                ),
                "asset_catalog": sha256(catalog_path),
                "asset_compatibility": sha256(compatibility_path),
                "generation_config": sha256(config_path),
            },
        }
        write_json(metadata_path, metadata)
        # Re-open serialized output and validate the actual file.
        validate_scenario(
            load_json(scenario_path),
            ego,
            radius=28.0,
            link_ids=link_ids,
            expected_counts=expected,
        )

        scene = {
            "id": base_id,
            "base_scenario_id": base_id,
            "file_name": stem,
            "scenario": str(scenario_path.relative_to(project_root)),
            "metadata": str(metadata_path.relative_to(project_root)),
            "network_sidecar": "EgoNetwork/{}_MN.json".format(stem),
            "split": spec["split"],
            "scene_type": spec["scene_type"],
            "topology": spec["anchor"]["topology"],
            "anchor_index": spec["anchor"]["index"],
            "generation_seed": scene_seed,
            "expected_ego": metadata["ego"],
            "expected_counts": metadata["counts"],
            "run_by_default": True,
        }
        scenes.append(scene)
        split_ids[spec["split"]].append(base_id)

    if {
        key: len(value) for key, value in split_ids.items()
    } != SPLIT_COUNTS:
        raise AssertionError(
            "base split counts are not {}".format(SPLIT_COUNTS)
        )

    policy = capture_policy(config)
    base_manifest = build_manifest(scenes, network_sha, policy)
    write_json(
        manifest_dir / "{}_base.json".format(MANIFEST_STEM),
        base_manifest,
    )
    for environment in ENVIRONMENTS:
        manifest = build_manifest(
            scenes, network_sha, policy, environment=environment
        )
        write_json(
            manifest_dir / "{}_{}.json".format(
                MANIFEST_STEM, environment["id"]
            ),
            manifest,
        )

    for split, ids in split_ids.items():
        (
            manifest_dir
            / "{}_{}.txt".format(MANIFEST_STEM, split)
        ).write_text(
            "".join(item + "\n" for item in ids),
            encoding="utf-8",
        )

    runs_path = manifest_dir / "{}_runs.jsonl".format(MANIFEST_STEM)
    with runs_path.open("w", encoding="utf-8") as stream:
        for environment in ENVIRONMENTS:
            for scene in scenes:
                stream.write(
                    json.dumps(
                        {
                            "run_id": "{}_{}".format(
                                scene["id"], environment["id"]
                            ),
                            "base_scenario_id": scene["id"],
                            "scenario_stem": scene["file_name"],
                            "split": scene["split"],
                            "environment": environment,
                            "expected_frames": policy[
                                "captures_per_scene"
                            ],
                        },
                        ensure_ascii=False,
                    )
                    + "\n"
                )

    coverage_path = (
        workspace / "reports/{}_asset_coverage.json".format(MANIFEST_STEM)
    )
    coverage_rows = [
        {
            "group": group,
            "data_id": data_id,
            "name": name,
            "placements": count,
        }
        for (group, data_id, name), count in sorted(coverage.items())
    ]
    write_json(
        coverage_path,
        {
            "schema_version": 1,
            "dataset_id": DATASET_ID,
            "base_scenario_count": len(scenes),
            "unique_assets_placed": len(coverage_rows),
            "placements": coverage_rows,
        },
    )
    generation_summary_path = (
        workspace
        / "reports/{}_generation_summary.json".format(MANIFEST_STEM)
    )
    write_json(
        generation_summary_path,
        {
            "schema_version": 1,
            "dataset_id": DATASET_ID,
            "quality_status": "static_validated_simulator_visual_check_pending",
            "base_scenarios": len(scenes),
            "environment_variants": len(ENVIRONMENTS),
            "capture_runs": len(scenes) * len(ENVIRONMENTS),
            "expected_multisensor_frames": (
                len(scenes)
                * len(ENVIRONMENTS)
                * policy["captures_per_scene"]
            ),
            "split_base_scenarios": {
                key: len(value) for key, value in sorted(split_ids.items())
            },
            "scene_types": dict(
                sorted(Counter(item["scene_type"] for item in scenes).items())
            ),
            "topologies": dict(
                sorted(Counter(item["topology"] for item in scenes).items())
            ),
            "actor_instances_per_base_pass": {
                group: sum(
                    item["placements"]
                    for item in coverage_rows
                    if item["group"] == group
                )
                for group in ("vehicleList", "pedestrianList", "objectList")
            },
            "unique_assets_placed": len(coverage_rows),
            "route_constraints": {
                "minimum_anchor_arc_distance_m": (
                    MINIMUM_ANCHOR_ARC_DISTANCE_M
                ),
                "path_discontinuity_rejection_threshold_m": 2.0,
                "ego_swept_corridor_clear": True,
            },
            "capture_policy": policy,
        },
    )

    install_dir = (
        args.install_dir.expanduser().resolve()
        if args.install_dir is not None
        else None
    )
    if install_dir is not None:
        install_dir.mkdir(parents=True, exist_ok=True)
        (install_dir / "EgoNetwork").mkdir(parents=True, exist_ok=True)
        for scene in scenes:
            source = project_root / scene["scenario"]
            shutil.copy2(source, install_dir / (scene["file_name"] + ".json"))
            sidecar = (
                install_dir
                / "EgoNetwork"
                / (scene["file_name"] + "_MN.json")
            )
            sidecar.write_bytes(network_payload)

    print("MORAI OD 180-run pilot generated")
    print("base scenarios:", len(scenes))
    print("environment manifests:", len(ENVIRONMENTS))
    print("capture runs:", len(scenes) * len(ENVIRONMENTS))
    print("split:", {key: len(value) for key, value in split_ids.items()})
    print("scenario directory:", scenario_dir)
    print(
        "base manifest:",
        manifest_dir / "{}_base.json".format(MANIFEST_STEM),
    )
    print("runs:", runs_path)
    print("asset coverage:", coverage_path)
    print("generation summary:", generation_summary_path)
    print("MORAI install:", install_dir or "not requested")


if __name__ == "__main__":
    main()
