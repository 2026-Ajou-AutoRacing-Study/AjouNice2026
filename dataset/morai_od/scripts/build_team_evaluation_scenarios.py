#!/usr/bin/env python3
"""Build human-driven MORAI evaluation baselines from the official sample.

Two mission variants are produced.  The first clones the official static
obstacle asset into the GPS-shadow corridor.  The second adds one deterministic
NPC spawn point that starts near the entrance of that same GPS-shadow corridor
at 0 km/h, targets 50 km/h, and destroys the NPC near the corridor exit.  Each
variant is emitted at four parked Ego poses for a human-driven evaluation run.
The official sample itself is never modified.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path

from generate_dynamic_object_capture_set import (
    build_mgeo_guided_competition_route,
    continuous_route_pose_at_arc,
    formatted_rotation,
    formatted_vector,
    load_json,
    load_route,
    nearest_index,
    route_arc_coordinates,
    route_pose_at_index,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TEMPLATE = REPO_ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
DEFAULT_ROUTE = REPO_ROOT / "configs/global_path/2026_molit_comp_global_path.txt"
DEFAULT_MGEO = REPO_ROOT / "kcity_map/mgeo_map_scenario_runner_2025"
DEFAULT_OUTPUT = REPO_ROOT / "configs/scenario/team_evaluation_baselines"

OFFICIAL_OBSTACLE_DATA_ID = 40100001
EXTRA_OBSTACLE_UNIQUE_ID = 1001
EXTRA_SPAWN_UNIQUE_ID = 1002
EXTRA_DESTINATION_UNIQUE_ID = 1003

EGO_ARCS_M = {
    "start": None,
    "signal_to_roundabout": 650.0,
    "roundabout_to_highway": 990.0,
    "surround_to_gps_shadow": 1670.0,
}

OUTPUT_STEMS = {
    "gps_shadow_obstacle": "sample_gps_shadow_obstacle",
    "gps_shadow_vehicle": "sample_gps_shadow_vehicle",
}


def portable_path(path):
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(resolved)


def zero_vector(value):
    if isinstance(value, dict):
        value.update(formatted_vector(0.0, 0.0, 0.0))


def configure_parked_ego(scenario, pose, preserve_default=False):
    ego = scenario["egoVehicle"]
    ego["beforeControlMode"] = 0
    ego["currentControlMode"] = 0
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    initial = ego["initPosition"]
    initial["initPositionMode"] = "Absolute"
    if not preserve_default:
        initial["pos"] = formatted_vector(
            pose["x"], pose["y"], pose["z"] + 0.34
        )
        initial["rot"] = formatted_rotation(pose["yaw"])
        initial["initLink"] = pose["link_idx"]
        initial["initLinkRatio"] = float(pose["point_ratio"])
        ego["linkValue"] = float(pose.get("max_speed", ego.get("linkValue", 85.0)))
    initial["gear"] = 1
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    zero_vector(saved.get("lastVelocity"))
    zero_vector(saved.get("vehicle", {}).get("lastVelocity"))
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 1


def lateral_offset(pose, offset_m):
    yaw = math.radians(float(pose["yaw"]))
    return {
        "x": float(pose["x"]) - math.sin(yaw) * float(offset_m),
        "y": float(pose["y"]) + math.cos(yaw) * float(offset_m),
        "z": float(pose["z"]),
    }


def add_gps_shadow_obstacle(scenario, route):
    source = next(
        actor
        for actor in scenario["objectList"]
        if int(actor["DataID"]) == OFFICIAL_OBSTACLE_DATA_ID
    )
    shade = scenario["shadedAreaList"][0]
    route_index = nearest_index(route, float(shade["pos"]["x"]), float(shade["pos"]["y"]))
    road = route_pose_at_index(route, route_index)
    # Occupy the mission lane without rotating the long obstacle across the
    # tunnel.  A small lateral offset leaves a deliberate avoidance side.
    position = lateral_offset(road, 0.8)
    actor = copy.deepcopy(source)
    actor["UNIQUEID"] = EXTRA_OBSTACLE_UNIQUE_ID
    actor["pos"] = formatted_vector(position["x"], position["y"], position["z"])
    actor["rot"] = formatted_rotation(road["yaw"])
    actor["isWaypointMode"] = False
    actor["moveSpeed"] = 0.0
    actor["activeDistance"] = 0.0
    actor["waypointEventList"] = []
    scenario["objectList"].append(actor)
    return actor, road


def gps_shadow_route_bounds(scenario, route, route_arc):
    """Return the longest competition-route interval inside the shade volume.

    MORAI's serialized shade dimensions use an asset-local convention whose
    long axis may appear as either x or y.  Evaluate both conventions and keep
    the longest contiguous route intersection rather than hard-coding one.
    """
    shade = scenario["shadedAreaList"][0]
    center_x = float(shade["pos"]["x"])
    center_y = float(shade["pos"]["y"])
    yaw = math.radians(float(shade["rot"]["yaw"]))
    size_x = float(shade["size"]["x"])
    size_y = float(shade["size"]["y"])

    best_run = None
    for half_x, half_y in ((size_x / 2.0, size_y / 2.0),
                           (size_y / 2.0, size_x / 2.0)):
        indices = []
        for index, point in enumerate(route):
            dx = float(point["x"]) - center_x
            dy = float(point["y"]) - center_y
            local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                indices.append(index)
        runs = []
        for index in indices:
            if not runs or index != runs[-1][-1] + 1:
                runs.append([index])
            else:
                runs[-1].append(index)
        for run in runs:
            span = float(route_arc[run[-1]] - route_arc[run[0]])
            if best_run is None or span > best_run[0]:
                best_run = (span, run[0], run[-1])
    if best_run is None or best_run[0] < 20.0:
        raise RuntimeError("GPS-shadow volume does not intersect the route")
    return best_run


def add_gps_shadow_vehicle_spawn(scenario, route, route_arc):
    shade_span, first_index, last_index = gps_shadow_route_bounds(
        scenario, route, route_arc
    )
    margin_m = min(5.0, shade_span * 0.1)
    start_arc = float(route_arc[first_index]) + margin_m
    end_arc = float(route_arc[last_index]) - margin_m
    start = continuous_route_pose_at_arc(route, route_arc, start_arc, wrap=False)
    end = continuous_route_pose_at_arc(route, route_arc, end_arc, wrap=False)
    source = next(
        spawn
        for spawn in scenario["spawnPointList"]
        if int(spawn["UNIQUEID"]) == 8
    )
    spawn = copy.deepcopy(source)
    spawn["UNIQUEID"] = EXTRA_SPAWN_UNIQUE_ID
    spawn["pos"] = formatted_vector(start["x"], start["y"], start["z"])
    spawn["rot"] = formatted_rotation(start["yaw"])
    spawn["spawnVehicleTypeID"] = 30100005
    spawn["maximumSpawnVehicle"] = 1
    spawn["minSpawnPeriod"] = 1.0
    spawn["maxSpawnPeriod"] = 1.0
    spawn["SpawnVelocityType"] = 2
    spawn["MinSpawnVelocity_Custom"] = 0.0
    spawn["MaxSpawnVelocity_Custom"] = 0.0
    spawn["MinSpawnVelocity_Link"] = 0.0
    spawn["MaxSpawnVelocity_Link"] = 0.0
    spawn["DesiredVelocityType"] = 2
    spawn["MinDesiredVelocity_Custom"] = 50.0
    spawn["MaxDesiredVelocity_Custom"] = 50.0
    spawn["spawnedVehiclesCount"] = 0
    spawn["isSpawning"] = True
    spawn["waitSpawnTime"] = 0.0
    spawn["destinationObjUniqueID"] = EXTRA_DESTINATION_UNIQUE_ID
    spawn["startLinkInfo"] = {
        "linkIdx": start["link_idx"],
        "pointIdx": int(start["point_idx"]),
    }
    spawn["endLinkInfo"] = {
        "linkIdx": end["link_idx"],
        "pointIdx": int(end["point_idx"]),
    }
    spawn["isCloseLoop"] = False
    spawn["isLaneChange"] = False
    spawn["DestinationMode"] = 1
    spawn["NPCSpawnType"] = 1
    spawn["rANDOMPOINT_LIFE_MODE"] = 0
    scenario["spawnPointList"].append(spawn)
    return spawn, start, end, end_arc - start_arc


def validate_base_template(template):
    ego = template["egoVehicle"]
    initial = ego["initPosition"]
    if (
        int(ego.get("beforeControlMode", -1)) != 0
        or int(ego.get("currentControlMode", -1)) != 0
        or int(initial.get("gear", -1)) != 1
        or abs(float(ego.get("currentVelocity", 1.0))) > 1e-6
    ):
        raise RuntimeError(
            "official sample must be Keyboard mode, stationary, and in Park"
        )
    if not any(
        int(actor.get("DataID", -1)) == OFFICIAL_OBSTACLE_DATA_ID
        for actor in template.get("objectList", [])
    ):
        raise RuntimeError("official static-obstacle asset is missing")


def validate_output(scenario, mission, ego_key):
    ego = scenario["egoVehicle"]
    initial = ego["initPosition"]
    if (
        int(ego["beforeControlMode"]) != 0
        or int(ego["currentControlMode"]) != 0
        or int(initial["gear"]) != 1
        or float(ego["currentVelocity"]) != 0.0
    ):
        raise RuntimeError("generated Ego is not parked in Keyboard mode")
    if ego_key != "start" and not initial.get("initLink"):
        raise RuntimeError("relocated Ego lacks MGeo initLink")
    obstacle_ids = [
        int(actor["UNIQUEID"])
        for actor in scenario.get("objectList", [])
        if int(actor.get("DataID", -1)) == OFFICIAL_OBSTACLE_DATA_ID
    ]
    spawn_ids = {
        int(spawn["UNIQUEID"]) for spawn in scenario.get("spawnPointList", [])
    }
    if mission == "gps_shadow_obstacle":
        if EXTRA_OBSTACLE_UNIQUE_ID not in obstacle_ids:
            raise RuntimeError("GPS-shadow obstacle was not added")
        if EXTRA_SPAWN_UNIQUE_ID in spawn_ids:
            raise RuntimeError("vehicle spawn leaked into obstacle baseline")
    else:
        if EXTRA_OBSTACLE_UNIQUE_ID in obstacle_ids:
            raise RuntimeError("GPS obstacle leaked into vehicle baseline")
        if EXTRA_SPAWN_UNIQUE_ID not in spawn_ids:
            raise RuntimeError("GPS-shadow vehicle spawn was not added")
        spawn = next(
            item
            for item in scenario["spawnPointList"]
            if int(item["UNIQUEID"]) == EXTRA_SPAWN_UNIQUE_ID
        )
        if (
            int(spawn["maximumSpawnVehicle"]) != 1
            or float(spawn["MinSpawnVelocity_Custom"]) != 0.0
            or float(spawn["MaxSpawnVelocity_Custom"]) != 0.0
            or float(spawn["MinDesiredVelocity_Custom"]) != 50.0
            or float(spawn["MaxDesiredVelocity_Custom"]) != 50.0
            or not spawn["isSpawning"]
        ):
            raise RuntimeError("GPS-shadow vehicle spawn contract is invalid")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--mgeo-dir", type=Path, default=DEFAULT_MGEO)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    template = load_json(args.template.resolve())
    validate_base_template(template)
    guide, _ = load_route(args.route.resolve())
    links = load_json(args.mgeo_dir.resolve() / "link_set.json")
    route, route_audit = build_mgeo_guided_competition_route(guide, links)
    route_arc = route_arc_coordinates(route)

    ego_poses = {}
    for key, distance_m in EGO_ARCS_M.items():
        if distance_m is None:
            ego_poses[key] = None
        else:
            ego_poses[key] = continuous_route_pose_at_arc(
                route, route_arc, distance_m, wrap=True
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    for mission in ("gps_shadow_obstacle", "gps_shadow_vehicle"):
        mission_template = copy.deepcopy(template)
        if mission == "gps_shadow_obstacle":
            actor, road = add_gps_shadow_obstacle(mission_template, route)
            mission_description = {
                "extra_actor": {
                    "kind": "static_obstacle",
                    "data_id": int(actor["DataID"]),
                    "unique_id": int(actor["UNIQUEID"]),
                    "position": actor["pos"],
                    "route_heading_deg": float(road["yaw"]),
                }
            }
        else:
            spawn, start, end, route_span_m = add_gps_shadow_vehicle_spawn(
                mission_template, route, route_arc
            )
            mission_description = {
                "extra_actor": {
                    "kind": "gps_shadow_vehicle_spawn",
                    "data_id": int(spawn["spawnVehicleTypeID"]),
                    "unique_id": int(spawn["UNIQUEID"]),
                    "start_speed_kph": 0.0,
                    "target_speed_kph": 50.0,
                    "start": start,
                    "end": end,
                    "route_span_m": float(route_span_m),
                }
            }
        for ego_key, pose in ego_poses.items():
            scenario = copy.deepcopy(mission_template)
            configure_parked_ego(
                scenario, pose, preserve_default=(ego_key == "start")
            )
            validate_output(scenario, mission, ego_key)
            output = args.output_dir / "{}_ego_{}.json".format(
                OUTPUT_STEMS[mission], ego_key
            )
            output.write_text(
                json.dumps(scenario, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            outputs.append({
                "file": output.name,
                "mission": mission,
                "ego_anchor": ego_key,
                "ego_route_arc_m": EGO_ARCS_M[ego_key],
                "ego_position": scenario["egoVehicle"]["initPosition"]["pos"],
                "ego_yaw_deg": float(
                    scenario["egoVehicle"]["initPosition"]["rot"]["yaw"]
                ),
                **mission_description,
            })

    manifest = {
        "source_template": portable_path(args.template),
        "source_template_control": "Keyboard",
        "source_template_gear": "P",
        "route": portable_path(args.route),
        "mgeo_dir": portable_path(args.mgeo_dir),
        "route_reconstruction": route_audit,
        "outputs": outputs,
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("generated {} scenarios in {}".format(len(outputs), args.output_dir))
    for row in outputs:
        print(
            "{}: mission={} ego={} arc={}".format(
                row["file"], row["mission"], row["ego_anchor"],
                row["ego_route_arc_m"],
            )
        )


if __name__ == "__main__":
    main()
