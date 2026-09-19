#!/usr/bin/env python3
"""Build a clean, human-driven static-obstacle OD benchmark for MORAI MR."""

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
    route_arc_coordinates,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TEMPLATE = REPO_ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
DEFAULT_ROUTE = REPO_ROOT / "configs/global_path/2026_molit_comp_global_path.txt"
DEFAULT_MGEO = REPO_ROOT / "kcity_map/mgeo_map_scenario_runner_2025"
DEFAULT_ASSET_TEMPLATE = (
    Path.home()
    / "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/"
    / "R_KR_PR_K-city_2025/obstacle_list_without_vehicle_and_ped_like_objects.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "configs/scenario/perception_benchmark/bench_static_obstacle_base3_assets_raw.json"
)

# These are all members of the final Base3-Static obstacle allowlist.  Small
# road markings, covers, poles, and thin sign plates are intentionally absent.
PLACEMENTS = (
    (40100001, "CargoBox", 276.0, 0.85),
    (40100027, "PE_Drum", 294.0, -0.85),
    (40100005, "TrafficBarrel", 312.0, 0.90),
    (40200162, "concrete-barrier", 330.0, -0.90),
    (40100047, "Concrete_Barricade01", 348.0, 0.85),
    (40100028, "PE_Firewall_Orange", 366.0, -0.85),
)
EGO_ARC_M = 235.0


def lateral_offset(pose, offset_m):
    yaw = math.radians(float(pose["yaw"]))
    return {
        "x": float(pose["x"]) - math.sin(yaw) * float(offset_m),
        "y": float(pose["y"]) + math.cos(yaw) * float(offset_m),
        "z": float(pose["z"]),
    }


def configure_parked_manual_ego(scenario, pose):
    ego = scenario["egoVehicle"]
    ego["beforeControlMode"] = 0
    ego["currentControlMode"] = 0
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    initial = ego["initPosition"]
    initial["initPositionMode"] = "Absolute"
    initial["pos"] = formatted_vector(pose["x"], pose["y"], pose["z"] + 0.34)
    initial["rot"] = formatted_rotation(pose["yaw"])
    initial["initLink"] = pose["link_idx"]
    initial["initLinkRatio"] = float(pose["point_ratio"])
    initial["gear"] = 1
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    for vector in (saved.get("lastVelocity"), saved.get("vehicle", {}).get("lastVelocity")):
        if isinstance(vector, dict):
            vector.update(formatted_vector(0.0, 0.0, 0.0))
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 1


def validate(scenario):
    ego = scenario["egoVehicle"]
    assert int(ego["beforeControlMode"]) == 0
    assert int(ego["currentControlMode"]) == 0
    assert int(ego["initPosition"]["gear"]) == 1
    assert float(ego["currentVelocity"]) == 0.0
    assert not scenario["vehicleList"]
    assert not scenario["pedestrianList"]
    assert not scenario["spawnPointList"]
    assert len(scenario["objectList"]) == len(PLACEMENTS)
    assert [int(row["DataID"]) for row in scenario["objectList"]] == [
        row[0] for row in PLACEMENTS
    ]
    assert len({int(row["UNIQUEID"]) for row in scenario["objectList"]}) == len(PLACEMENTS)
    for actor in scenario["objectList"]:
        assert int(actor["m_eobstacleObjType"]) == 4
        assert float(actor["moveSpeed"]) == 0.0
        assert not actor["isWaypointMode"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--route", type=Path, default=DEFAULT_ROUTE)
    parser.add_argument("--mgeo-dir", type=Path, default=DEFAULT_MGEO)
    parser.add_argument("--asset-template", type=Path, default=DEFAULT_ASSET_TEMPLATE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    scenario = load_json(args.template.resolve())
    asset_source = load_json(args.asset_template.resolve())
    guide, _ = load_route(args.route.resolve())
    links = load_json(args.mgeo_dir.resolve() / "link_set.json")
    route, route_audit = build_mgeo_guided_competition_route(guide, links)
    route_arc = route_arc_coordinates(route)
    templates = {int(row["DataID"]): row for row in asset_source["objectList"]}

    configure_parked_manual_ego(
        scenario,
        continuous_route_pose_at_arc(route, route_arc, EGO_ARC_M, wrap=False),
    )
    # Keep map, traffic-light, and course settings, but isolate obstacle OD from
    # the official sample's moving actors and procedural traffic.
    scenario["vehicleList"] = []
    scenario["pedestrianList"] = []
    scenario["spawnPointList"] = []
    scenario["pedspawnPointList"] = []
    scenario["aiPedstrianList"] = []
    scenario["objectList"] = []

    audit_rows = []
    for index, (data_id, name, arc_m, offset_m) in enumerate(PLACEMENTS):
        if data_id not in templates:
            raise RuntimeError("MR asset template lacks {} ({})".format(name, data_id))
        pose = continuous_route_pose_at_arc(route, route_arc, arc_m, wrap=False)
        position = lateral_offset(pose, offset_m)
        actor = copy.deepcopy(templates[data_id])
        actor["UNIQUEID"] = 4101 + index
        actor["pos"] = formatted_vector(position["x"], position["y"], position["z"])
        actor["rot"] = formatted_rotation(pose["yaw"])
        actor["scale"] = formatted_vector(1.0, 1.0, 1.0)
        actor["initPos"] = formatted_vector(0.0, 0.0, 0.0)
        actor["initRot"] = formatted_rotation(0.0)
        actor["isWaypointMode"] = False
        actor["moveSpeed"] = 0.0
        actor["activeDistance"] = 0.0
        actor["waypointEventList"] = []
        scenario["objectList"].append(actor)
        audit_rows.append(
            {
                "name": name,
                "data_id": data_id,
                "route_arc_m": arc_m,
                "lateral_offset_m": offset_m,
                "position": actor["pos"],
                "yaw_deg": float(pose["yaw"]),
                "link_idx": pose["link_idx"],
            }
        )

    validate(scenario)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(scenario, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    audit = {
        "scenario": args.output.name,
        "purpose": "human-driven static-obstacle OD rosbag benchmark",
        "ego_route_arc_m": EGO_ARC_M,
        "obstacle_spacing_m": 18.0,
        "objects": audit_rows,
        "route_source": route_audit,
    }
    audit_path = args.output.with_suffix(".audit.json")
    audit_path.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(args.output)
    print(audit_path)
    print("objects={}".format(len(audit_rows)))


if __name__ == "__main__":
    main()
