#!/usr/bin/env python3
"""Build matched actor-free / single-obstacle MR GPS-shadow replay scenes."""

import copy
import json
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "dataset/morai_od/scripts"))

from build_team_evaluation_scenarios import configure_parked_ego, lateral_offset  # noqa: E402
from generate_dynamic_object_capture_set import (  # noqa: E402
    build_mgeo_guided_competition_route,
    formatted_rotation,
    formatted_vector,
    load_json,
    load_route,
    nearest_index,
    route_pose_at_index,
)


BASE = ROOT / "configs/scenario/localization_evaluation/localization_full_route_no_actors.json"
SAMPLE = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
OUT = ROOT / "configs/scenario/localization_evaluation"
ROUTE = ROOT / "configs/global_path/2026_molit_comp_global_path.txt"
MGEO = ROOT / "kcity_map/mgeo_map_scenario_runner_2025/link_set.json"

# Raw /Ego_topic positions from planning_rviz_20260929_032704.bag. Both poses
# are snapped to the same directed competition MGeo route for valid initLink.
EGO_REFERENCE = (53.080, -466.312)
OBSTACLE_REFERENCE = (16.200, -496.800)
MAX_EGO_SNAP_M = 0.5
MAX_OBSTACLE_SNAP_M = 1.0
OBSTACLE_RIGHT_OFFSET_M = -0.5
OBSTACLE_UNIQUE_ID = 9001


def snapped_pose(route, reference, tolerance_m):
    index = nearest_index(route, *reference)
    pose = route_pose_at_index(route, index)
    distance = math.hypot(pose["x"] - reference[0], pose["y"] - reference[1])
    if distance > tolerance_m:
        raise ValueError("reference is {:.3f} m from route: {}".format(distance, reference))
    return pose


def main():
    guide, _ = load_route(ROUTE)
    links = load_json(MGEO)
    route, _ = build_mgeo_guided_competition_route(guide, links)
    ego_pose = snapped_pose(route, EGO_REFERENCE, MAX_EGO_SNAP_M)
    obstacle_pose = snapped_pose(route, OBSTACLE_REFERENCE, MAX_OBSTACLE_SNAP_M)

    clear = load_json(BASE)
    if any(clear[key] for key in ("vehicleList", "pedestrianList", "objectList", "spawnPointList")):
        raise ValueError("base scene is not actor-free")
    if len(clear["shadedAreaList"]) != 1:
        raise ValueError("expected the sample GNSS shadow")
    configure_parked_ego(clear, ego_pose)

    obstacle = copy.deepcopy(clear)
    source = load_json(SAMPLE)
    cargo = next(item for item in source["objectList"] if int(item["DataID"]) == 40100001)
    cargo = copy.deepcopy(cargo)
    position = lateral_offset(obstacle_pose, OBSTACLE_RIGHT_OFFSET_M)
    cargo["UNIQUEID"] = OBSTACLE_UNIQUE_ID
    cargo["pos"] = formatted_vector(position["x"], position["y"], position["z"])
    cargo["rot"] = formatted_rotation(obstacle_pose["yaw"])
    cargo["isWaypointMode"] = False
    cargo["moveSpeed"] = 0.0
    cargo["activeDistance"] = 0.0
    cargo["waypointEventList"] = []
    obstacle["objectList"] = [cargo]

    # Keep the exact same obstacle but enter the area through the original
    # full-route start. This separates a mid-route Route-B cold start from
    # obstacle-asset behavior without changing both variables together.
    full_start_obstacle = copy.deepcopy(clear)
    full_start_obstacle["egoVehicle"] = copy.deepcopy(load_json(BASE)["egoVehicle"])
    full_start_obstacle["objectList"] = [copy.deepcopy(cargo)]

    for name, scene in (
        ("localization_shadow_entry_clear.json", clear),
        ("localization_shadow_entry_cargobox.json", obstacle),
        ("localization_shadow_full_start_cargobox.json", full_start_obstacle),
    ):
        target = OUT / name
        target.write_text(json.dumps(scene, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(target)
    print("ego pose:", ego_pose)
    print("CargoBox:", cargo["pos"], "scale:", cargo["scale"])


if __name__ == "__main__":
    main()
