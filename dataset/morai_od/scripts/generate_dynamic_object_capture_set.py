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
import json
import math
import random
import shutil
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


FAMILY_CODES = {
    "roundabout_sample_spawn": "rs",
    "roundabout_circulation": "rc",
    "highway_merge_sample_spawn": "hm",
    "highway_merge_cruise": "mc",
    "highway_cruise_overtake": "hc",
    "crosswalk_pedestrian": "pd",
    "static_obstacle": "so",
    "gps_shadow_obstacle": "go",
}


SIMPLE_SPAWN_VEHICLE_FAMILIES = {
    "roundabout_sample_spawn",
    "roundabout_circulation",
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
}

SINGLE_LANE_SPAWN_FAMILIES = {
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
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


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--mgeo-dir", type=Path, default=DEFAULT_MGEO)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--group", default="dynamic_object_v1_smoke")
    parser.add_argument("--seed", type=int)
    parser.add_argument(
        "--install-dir",
        type=Path,
        help="Optionally install generated scenario JSON into MORAI SaveFile/Scenario.",
    )
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


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


def nearest_index(points, x, y):
    return min(
        range(len(points)),
        key=lambda index: math.hypot(points[index]["x"] - x, points[index]["y"] - y),
    )


def route_pose_before(route, arc, target, lead_distance):
    target_index = nearest_index(route, target["x"], target["y"])
    target_arc = max(0.0, arc[target_index] - lead_distance)
    insertion = bisect.bisect_left(arc, target_arc)
    candidates = [min(len(arc) - 1, insertion)]
    if insertion:
        candidates.append(insertion - 1)
    index = min(candidates, key=lambda item: abs(arc[item] - target_arc))
    after = min(len(route) - 1, index + 1)
    before = max(0, index - 1)
    heading = math.degrees(
        math.atan2(
            route[after]["y"] - route[before]["y"],
            route[after]["x"] - route[before]["x"],
        )
    )
    return {**route[index], "yaw": heading, "route_index": index}


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


def wrapped_degrees(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def nearest_aligned_sample(samples, pose, search_radius_m=6.0):
    nearby = []
    for item in samples:
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
    raise ValueError(f"unsupported family: {family_name}")


def configure_spawn(spawn, rng, family, base_target_speed=None):
    period = uniform_or_choice(rng, family["spawn_period_sec"])
    spawn["minSpawnPeriod"] = period
    spawn["maxSpawnPeriod"] = period
    spawn["maximumSpawnVehicle"] = 20
    spawn["spawnedVehiclesCount"] = 0
    spawn["waitSpawnTime"] = 0.0
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
    route = []
    for link_id in link_ids:
        if link_id not in links_by_id:
            raise ValueError(f"unknown highway route link {link_id}")
        for point in links_by_id[link_id]["points"]:
            row = {
                "x": float(point[0]),
                "y": float(point[1]),
                "z": float(point[2]) if len(point) >= 3 else 0.0,
            }
            if route and math.hypot(
                route[-1]["x"] - row["x"], route[-1]["y"] - row["y"]
            ) < 1e-4:
                continue
            route.append(row)
    arc = [0.0]
    for before, after in zip(route, route[1:]):
        arc.append(
            arc[-1]
            + math.hypot(
                after["x"] - before["x"], after["y"] - before["y"]
            )
        )
    return route, arc


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
    npc_spawn_id = source_ids[variant_index % len(source_ids)]
    ego_lane_id = source_ids[(variant_index // len(source_ids)) % len(source_ids)]
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
    scenario["spawnPointList"] = [spawn]

    route_link_ids = HIGHWAY_LANE_LINK_PATHS[ego_lane_id]
    ego_route, ego_route_arc = route_from_link_path(links_by_id, route_link_ids)
    start_arc = rng.uniform(*family["ego_start_arc_m"])
    ego_sample = route_pose_at(
        ego_route, ego_route_arc, start_arc, route_link_ids[0], links_by_id
    )
    same_lane = ego_lane_id == npc_spawn_id
    if same_lane:
        ego_speed = target
    else:
        ego_speed = min(
            float(family["ego_speed_limit_kph"]),
            target + rng.uniform(*family["ego_speed_advantage_kph"]),
        )
    set_ego_pose(scenario["egoVehicle"], ego_sample, ego_speed)
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
            "same_lane": same_lane,
        }
    ]
    capture_overrides = {
        "ego_target_speed_kph": ego_speed,
        "minimum_ego_speed_kph": (
            ego_speed * float(family["minimum_ego_speed_ratio"])
        ),
        "maximum_motion_m": 130.0,
        "motion_stall_timeout_sec": 12.0,
        "lookahead_m": 6.0,
        "adaptive_curve_control": False,
    }
    return ego_route, ego_sample, runtime, capture_overrides


def build_pedestrian_scene(scenario, template, family, rng):
    actor = copy.deepcopy(template["pedestrianList"][0])
    path = copy.deepcopy(
        next(
            item
            for item in template["waypointDataList"]
            if item.get("pathName") == "PedPath - 3"
        )
    )
    reverse = rng.random() < float(family.get("reverse_probability", 0.5))
    if reverse:
        path["waypointData"].reverse()
    speed = float(uniform_or_choice(rng, family["speed"]))
    active_distance = float(uniform_or_choice(rng, family["active_distance_m"]))
    actor["speed"] = speed
    actor["activeDistance"] = active_distance
    actor["active"] = True
    actor["loop"] = False
    actor["isWaypointMode"] = True
    start = waypoint_to_world(path["waypointData"][0]["pos"])
    end = waypoint_to_world(path["waypointData"][-1]["pos"])
    yaw = math.degrees(math.atan2(end["y"] - start["y"], end["x"] - start["x"]))
    set_actor_world_position(actor, start, yaw)
    scenario["pedestrianList"] = [actor]
    scenario["waypointDataList"] = [path]
    runtime = [{
        "unique_id": actor["UNIQUEID"],
        "data_id": actor["DataID"],
        "speed_config_value": speed,
        "speed_unit": "morai_pedestrian_config_value_unverified",
        "active_distance_m": active_distance,
        "reversed": reverse,
        "start": start,
        "end": end,
    }]
    return target_position_for_family(template, family["name"]), runtime


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


def shade_local_to_world(shade, longitudinal, lateral):
    yaw = math.radians(float(shade["rot"]["yaw"]))
    return {
        "x": float(shade["pos"]["x"]) + math.cos(yaw) * longitudinal - math.sin(yaw) * lateral,
        "y": float(shade["pos"]["y"]) + math.sin(yaw) * longitudinal + math.cos(yaw) * lateral,
        "z": float(shade["pos"]["z"]),
    }


def build_obstacle_scene(scenario, template, family, rng, samples):
    actor = copy.deepcopy(template["objectList"][0])
    scale = float(uniform_or_choice(rng, family["scale_factor"]))
    if family["name"] == "gps_shadow_obstacle":
        shade = template["shadedAreaList"][0]
        half_length = max(4.0, float(shade["size"]["x"]) * 0.35)
        lateral_range = family["route_lateral_offset_m"]
        raw = shade_local_to_world(
            shade,
            rng.uniform(-half_length, half_length),
            rng.uniform(float(lateral_range[0]), float(lateral_range[1])),
        )
        road = nearest_sample(samples, raw)
        position = {"x": road["x"], "y": road["y"], "z": road["z"] + 0.03}
        yaw = road["yaw"]
        target = dict(shade["pos"])
    else:
        position = dict(actor["pos"])
        yaw = float(actor["rot"]["yaw"])
        target = target_position_for_family(template, family["name"])
    set_actor_world_position(actor, position, yaw)
    actor["scale"] = formatted_vector(
        float(actor["scale"]["x"]) * scale,
        float(actor["scale"]["y"]) * scale,
        float(actor["scale"]["z"]) * scale,
    )
    actor["moveSpeed"] = 0.0
    actor["isWaypointMode"] = False
    actor["activeDistance"] = 0.0
    scenario["objectList"] = [actor]
    runtime = [{
        "unique_id": actor["UNIQUEID"],
        "data_id": actor["DataID"],
        "motion": "static",
        "position": position,
        "yaw_deg": yaw,
        "scale_factor": scale,
    }]
    return target, runtime


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
    route_distance = min(
        math.hypot(float(ego_pos["x"]) - item["x"], float(ego_pos["y"]) - item["y"])
        for item in route
    )
    if route_distance > 2.5:
        raise ValueError(f"Ego is {route_distance:.2f} m from competition route")
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
        if len(scenario["objectList"]) != 1 or scenario["objectList"][0]["moveSpeed"] != 0.0:
            raise ValueError("obstacle family must contain one static obstacle")
    if not expected_runtime:
        raise ValueError("runtime actor specification is empty")
    return {"ego_route_distance_m": route_distance, "unique_id_count": len(identifiers)}


def split_for_variant(index, family_count):
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
    seed = int(args.seed if args.seed is not None else config["seed"])
    scenario_dir = args.output_root / "scenarios" / args.group
    metadata_dir = args.output_root / "metadata" / args.group
    route_dir = metadata_dir / "routes"
    manifest_path = args.output_root / "manifests" / f"{args.group}.json"
    scenes = []
    global_index = 0

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

            if name in SIMPLE_SPAWN_VEHICLE_FAMILIES:
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
            elif name == "crosswalk_pedestrian":
                target, runtime = build_pedestrian_scene(scenario, template, family, rng)
            else:
                target, runtime = build_obstacle_scene(
                    scenario, template, family, rng, samples
                )

            if route_override is None:
                lead = choose_lead(rng, family)
                route_pose = route_pose_before(route, route_arc, target, lead)
                ego_sample = nearest_aligned_sample(samples, route_pose)
                ego_speed = float(
                    uniform_or_choice(
                        rng, family.get("ego_target_speed_kph", 0.0)
                    )
                )
                set_ego_pose(scenario["egoVehicle"], ego_sample, ego_speed)
                if ego_speed > 0.0:
                    capture_overrides.update(
                        {
                            "ego_target_speed_kph": ego_speed,
                            "minimum_ego_speed_kph": ego_speed
                            * float(family.get("minimum_ego_speed_ratio", 0.8)),
                            "maximum_motion_m": 60.0,
                            "motion_stall_timeout_sec": 10.0,
                            "lookahead_m": 4.0,
                        }
                    )
                validation_route = route
            else:
                lead = None
                route_pose = ego_sample
                validation_route = route_override
            if name == "crosswalk_pedestrian":
                arm_pedestrian_before_capture(
                    scenario,
                    runtime,
                    family.get("activation_margin_m", 5.0),
                )
            validation = validate_scene(
                scenario, name, links_by_id, validation_route, runtime
            )

            code = FAMILY_CODES[name]
            scene_id = f"d{code}{variant_index + 1:03d}"
            stem = f"{config['scenario_prefix']}_{code}{variant_index + 1:03d}"
            scenario_path = scenario_dir / f"{stem}.json"
            metadata_path = metadata_dir / f"{stem}.metadata.json"
            route_path = None
            if route_override is not None:
                route_path = route_dir / f"{stem}.waypoints.txt"
                write_waypoint_route(route_path, route_override)
            write_json(scenario_path, scenario)
            metadata = {
                "schema_version": 1,
                "dataset_id": config["dataset_id"],
                "scene_id": scene_id,
                "scenario_stem": stem,
                "mission_family": name,
                "motion_phase": family.get("motion_phase"),
                "sampling_role": family.get("sampling_role"),
                "recommended_phase_share_within_segment": family.get(
                    "recommended_phase_share_within_segment"
                ),
                "seed": scene_seed,
                "split": split_for_variant(variant_index, int(family["count"])),
                "environment": config["environment"],
                "capture_policy": config["capture"],
                "capture_policy_overrides": capture_overrides,
                "ego": {
                    "lead_distance_m": lead,
                    "route_index": route_pose["route_index"],
                    "pose": ego_sample,
                },
                "runtime_actors": runtime,
                "target_position": target,
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
                    "velocity_ground_truth": "Record /Object_topic raw velocity and acceleration plus finite-difference derivatives.",
                    "temporal_split": "Keep every frame from this scene/clip in the same split.",
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
                    "split": metadata["split"],
                    "anchor_index": route_pose["route_index"],
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
                    "dynamic_actor_policy": "observe_after_warmup",
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
                        "dynamic_actor_policy": "observe_after_warmup",
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
            "config": project_relative(args.config),
            "template": project_relative(args.template),
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
