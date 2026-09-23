#!/usr/bin/env python3
"""Build deterministic, human-driven MR evaluation scenarios.

The generated files deliberately keep Ego in Keyboard mode at zero speed.
Highway/roundabout traffic profiles are fixed per file so a recorded run can
be reproduced later; this is preferable to an unrecorded runtime RNG state.
"""

import copy
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SAMPLE = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
BENCHMARK = ROOT / "configs/scenario/perception_benchmark"
OUTPUT = ROOT / "configs/scenario/human_driving_evaluation"

HIGHWAY_PROFILES = {
    # spawn point IDs 6/8/10 correspond to the three official highway routes.
    "balanced": {6: 45.0, 8: 60.0, 10: 75.0},
    "slow_lead": {6: 35.0, 8: 50.0, 10: 65.0},
    "mixed_overtake": {6: 65.0, 8: 40.0, 10: 80.0},
    "fast_surround": {6: 55.0, 8: 75.0, 10: 85.0},
}

ROUNDABOUT_PROFILES = {
    "slow": 15.0,
    "nominal": 20.0,
    "fast": 25.0,
}

PEDESTRIAN_EGO = {
    "x": -60.04467620967861,
    "y": -56.50993470607794,
    "z": 28.383777555866246,
    "yaw": 90.53239128788123,
    "link": "A2256W000202",
    "link_ratio": 0.7843488758243736,
}


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write(path, data):
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def vector(x, y, z):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "_x": "{:.3f}".format(float(x)),
        "_y": "{:.3f}".format(float(y)),
        "_z": "{:.3f}".format(float(z)),
    }


def set_ego_parked(scenario, pose=None):
    ego = scenario["egoVehicle"]
    initial = ego["initPosition"]
    ego["beforeControlMode"] = 0
    ego["currentControlMode"] = 0
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    initial["gear"] = 1
    if pose:
        initial["initPositionMode"] = "Absolute"
        initial["pos"] = vector(pose["x"], pose["y"], pose["z"])
        initial["rot"] = {
            "roll": "0.000",
            "pitch": "0.000",
            "yaw": "{:.3f}".format(pose["yaw"]),
        }
        initial["initLink"] = pose["link"]
        initial["initLinkRatio"] = float(pose["link_ratio"])
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    for container in (saved, saved.get("vehicle", {})):
        velocity = container.get("lastVelocity")
        if isinstance(velocity, dict):
            velocity.update({"x": 0.0, "y": 0.0, "z": 0.0})
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 1


def set_spawn_speed(spawn, speed_kph):
    speed = float(speed_kph)
    # Spawn at the target velocity to avoid a long artificial acceleration
    # transient before the human-driven Ego reaches the highway interaction.
    spawn["SpawnVelocityType"] = 2
    spawn["MinSpawnVelocity_Custom"] = speed
    spawn["MaxSpawnVelocity_Custom"] = speed
    spawn["DesiredVelocityType"] = 2
    spawn["MinDesiredVelocity_Custom"] = speed
    spawn["MaxDesiredVelocity_Custom"] = speed
    spawn["spawnedVehiclesCount"] = 0
    spawn["waitSpawnTime"] = 0.0
    spawn["isSpawning"] = True


def build_highway():
    base = read(BENCHMARK / "bench_highway_merge_raw.json")
    outputs = []
    for name, speeds in HIGHWAY_PROFILES.items():
        scenario = copy.deepcopy(base)
        set_ego_parked(scenario)
        actual = {}
        for spawn in scenario["spawnPointList"]:
            unique_id = int(spawn["UNIQUEID"])
            set_spawn_speed(spawn, speeds[unique_id])
            actual[unique_id] = speeds[unique_id]
        if actual != speeds:
            raise RuntimeError("highway spawn IDs do not match the official template")
        path = OUTPUT / "ajounice_eval_highway_{}.json".format(name)
        write(path, scenario)
        outputs.append(path)
    return outputs


def build_roundabout():
    base = read(BENCHMARK / "bench_roundabout_raw.json")
    outputs = []
    for name, speed in ROUNDABOUT_PROFILES.items():
        scenario = copy.deepcopy(base)
        set_ego_parked(scenario)
        if len(scenario["spawnPointList"]) != 1:
            raise RuntimeError("roundabout template must contain one spawn point")
        set_spawn_speed(scenario["spawnPointList"][0], speed)
        path = OUTPUT / "ajounice_eval_roundabout_{}.json".format(name)
        write(path, scenario)
        outputs.append(path)
    return outputs


def build_pedestrian():
    scenario = read(SAMPLE)
    scenario["vehicleList"] = []
    scenario["objectList"] = []
    scenario["spawnPointList"] = []
    scenario["pedspawnPointList"] = []
    scenario["aiPedstrianList"] = []
    scenario["waypointDataList"] = [
        item
        for item in scenario.get("waypointDataList", [])
        if str(item.get("pathName", "")).startswith("PedPath")
    ]
    if len(scenario.get("pedestrianList", [])) != 1:
        raise RuntimeError("official pedestrian actor is missing")
    if not scenario["waypointDataList"]:
        raise RuntimeError("official pedestrian path is missing")
    set_ego_parked(scenario, PEDESTRIAN_EGO)
    path = OUTPUT / "ajounice_eval_pedestrian_crossing.json"
    write(path, scenario)
    return [path]


def build_obstacle():
    scenario = read(BENCHMARK / "bench_static_obstacle_base3_assets_raw.json")
    set_ego_parked(scenario)
    for actor in scenario.get("objectList", []):
        scale = actor.get("scale", {})
        if any(abs(float(scale.get(axis, 1.0)) - 1.0) > 1e-9 for axis in "xyz"):
            raise RuntimeError("obstacle evaluation requires scale=1")
    path = OUTPUT / "ajounice_eval_obstacle_scale1_six_assets.json"
    write(path, scenario)
    return [path]


def validate(outputs):
    for path in outputs:
        scenario = read(path)
        ego = scenario["egoVehicle"]
        initial = ego["initPosition"]
        if (
            int(ego["beforeControlMode"]) != 0
            or int(ego["currentControlMode"]) != 0
            or float(ego["currentVelocity"]) != 0.0
            or int(initial["gear"]) != 1
        ):
            raise RuntimeError("{} does not start parked in Keyboard mode".format(path))


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    outputs = build_highway() + build_roundabout() + build_pedestrian() + build_obstacle()
    validate(outputs)
    print("generated {} human evaluation scenarios".format(len(outputs)))
    for path in outputs:
        print(path.relative_to(ROOT))


if __name__ == "__main__":
    main()
