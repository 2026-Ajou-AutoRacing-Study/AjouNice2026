#!/usr/bin/env python3
"""Generate interleaved control/probe scenes for quarantined MORAI assets."""

import copy
import json
from pathlib import Path

from generate_morai_od_prototype import (
    make_pose_dict,
    set_obstacle_pose,
    set_pedestrian_pose,
    set_vehicle_pose,
)


ROOT = Path(__file__).resolve().parents[3]
DATASET = ROOT / "dataset/morai_od"
PERSONAL = Path.home() / (
    "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/Scenario/"
    "R_KR_PR_K-city_2025"
)
COMPETITION = Path.home() / (
    "MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/"
    "R_KR_PR_K-city_2025"
)

GROUP_INFO = {
    "vehicle": {
        "list_key": "vehicleList",
        "personal_file": "vehicle_list_per.json",
        "competition_file": "vehicle_list.json",
        "control_id": 30100004,
    },
    "pedestrian": {
        "list_key": "pedestrianList",
        "personal_file": "pedestrian_list_per.json",
        "competition_file": "pedestrian_list.json",
        "control_id": 50100001,
    },
    "obstacle": {
        "list_key": "objectList",
        "personal_file": "obstacle_list_per_without_vehicle_and_ped_like_objects.json",
        "competition_file": "obstacle_list_without_vehicle_and_ped_like_objects.json",
        "control_id": 40100001,
    },
}

CANDIDATES = {
    "vehicle": [
        30100001,
        30400001,
        30200011,
        30300003,
        30700004,
        30400005,
        30100009,
        30800004,
    ],
    "pedestrian": [50100005, 50200004, 50100055, 50200003],
    "obstacle": [40100062],
}


def load(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write(path, document):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def actor_map(path, list_key):
    return {int(row["DataID"]): row for row in load(path)[list_key]}


def catalog_names():
    names = {}
    for path in (
        DATASET / "configs/morai_asset_catalog_personal_260807.json",
        DATASET / "configs/morai_asset_catalog_260807.json",
    ):
        document = load(path)
        for key in ("vehicles", "pedestrians_and_related", "obstacles_and_related"):
            for row in document[key]:
                names[int(row["data_id"])] = row["name"]
    return names


def clear_scene(scene):
    for key in (
        "vehicleList",
        "pedestrianList",
        "objectList",
        "spawnPointList",
        "pedspawnPointList",
        "aiPedstrianList",
        "waypointDataList",
    ):
        scene[key] = []
    ego = scene["egoVehicle"]
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    saved = ego["initPosition"].get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    return scene


def vehicle_sample(slot):
    pos = slot["initPosition"]["pos"]
    rot = slot["initPosition"]["rot"]
    link = slot.get("currentLinkInfo") or slot.get("startPointLinkInfo") or {}
    return {
        "x": float(pos["x"]),
        "y": float(pos["y"]),
        "z": float(pos["z"]),
        "yaw": float(rot["yaw"]),
        "link_idx": slot["initPosition"].get("initLink") or link.get("linkIdx"),
        "point_idx": int(link.get("pointIdx", 0)),
        "point_ratio": float(slot["initPosition"].get("initLinkRatio", 0.0)),
        "max_speed": float(slot.get("linkVelocity", 30.0)),
    }


def place_actor(actor, group, slot, unique_id):
    actor = copy.deepcopy(actor)
    if group == "vehicle":
        set_vehicle_pose(actor, vehicle_sample(slot), unique_id, 0.0)
    elif group == "pedestrian":
        pose = slot["pos"]
        set_pedestrian_pose(
            actor,
            {axis: float(pose[axis]) for axis in ("x", "y", "z")},
            float(slot["rot"]["yaw"]),
            unique_id,
        )
    else:
        pose = slot["pos"]
        sample = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "z": float(pose["z"]),
            "yaw": float(slot["rot"]["yaw"]),
        }
        set_obstacle_pose(actor, sample, unique_id, 1.0)
    return actor


def expected_ego(scene):
    pos = scene["egoVehicle"]["initPosition"]["pos"]
    return {axis: float(pos[axis]) for axis in ("x", "y", "z")}


def main():
    group = "asset_revalidation_260822"
    scenario_dir = DATASET / "scenarios" / group
    manifest_path = DATASET / "manifests" / f"{group}.json"
    bases = [
        load(DATASET / "scenarios/geometry_trial20_v3/od_geo3_001.json"),
        load(DATASET / "scenarios/geometry_trial20_v3/od_geo3_003.json"),
    ]
    sources = {}
    names = catalog_names()
    for name, info in GROUP_INFO.items():
        personal = actor_map(PERSONAL / info["personal_file"], info["list_key"])
        competition = actor_map(
            COMPETITION / info["competition_file"], info["list_key"]
        )
        sources[name] = (personal, competition)

    scenes = []
    sequence = 0
    for group_name, ids in CANDIDATES.items():
        info = GROUP_INFO[group_name]
        personal, competition = sources[group_name]
        control = personal[info["control_id"]]
        for data_id in ids:
            target_source = "personal" if data_id in personal else "competition"
            target = (personal if data_id in personal else competition)[data_id]
            for trial_index, base in enumerate(bases, 1):
                slots = base[info["list_key"]]
                if len(slots) < 2:
                    raise RuntimeError(f"{group_name} base has fewer than two slots")
                for role in ("control", "probe"):
                    sequence += 1
                    scene = clear_scene(copy.deepcopy(base))
                    actors = [place_actor(control, group_name, slots[1], 9101)]
                    if role == "probe":
                        actors.insert(
                            0,
                            place_actor(target, group_name, slots[0], 9100),
                        )
                    scene[info["list_key"]] = actors
                    scene_id = f"arv{sequence:03d}"
                    stem = f"od_arv_{sequence:03d}"
                    scenario_path = scenario_dir / f"{stem}.json"
                    write(scenario_path, scene)
                    counts = {"npc": 0, "pedestrian": 0, "obstacle": 0}
                    counts[
                        {"vehicle": "npc", "pedestrian": "pedestrian", "obstacle": "obstacle"}[group_name]
                    ] = len(actors)
                    scenes.append(
                        {
                            "id": scene_id,
                            "file": stem,
                            "file_name": stem,
                            "scenario": str(scenario_path.relative_to(ROOT)),
                            "expected_ego": expected_ego(scene),
                            "expected_counts": counts,
                            "expected": {
                                "static_npc": counts["npc"],
                                "pedestrian": counts["pedestrian"],
                                "obstacle": counts["obstacle"],
                                "spawn_points": 0,
                            },
                            "asset_probe": {
                                "role": role,
                                "group": group_name,
                                "trial_index": trial_index,
                                "candidate_data_id": data_id,
                                "candidate_name": names[data_id],
                                "candidate_template_source": target_source,
                                "control_data_id": info["control_id"],
                                "control_name": names[info["control_id"]],
                            },
                        }
                    )

    manifest = {
        "schema_version": 1,
        "dataset_id": "morai_asset_revalidation_260822",
        "scenario_group": group,
        "environment": {"weather": "Sunny", "time": "13:00"},
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": {
            "format": "morai_save_sensor_data_stationary_probe",
            "captures_per_scene": 1,
            "capture_spacing_m": 1.5,
            "ego_target_speed_kph": 0.5,
            "maximum_motion_m": 1.0,
            "minimum_curve_speed_kph": 0.5,
            "required_sensor_dirs": [
                "CAMERA_1",
                "CAMERA_2",
                "CAMERA_3",
                "CAMERA_4",
                "LIDAR_5",
            ],
        },
        "scenes": scenes,
        "generation": {
            "candidate_count": sum(len(rows) for rows in CANDIDATES.values()),
            "trial_count_per_candidate": 2,
            "interleaving": "control-only scene immediately before every probe scene",
            "excluded_policy_only_asset": {"data_id": 40100032, "name": "Speed_Bump"},
        },
    }
    write(manifest_path, manifest)
    print(f"candidates: {manifest['generation']['candidate_count']}")
    print(f"scenes: {len(scenes)}")
    print(f"manifest: {manifest_path}")


if __name__ == "__main__":
    main()
