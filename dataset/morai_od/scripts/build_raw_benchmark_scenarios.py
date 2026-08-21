#!/usr/bin/env python3
"""Build clean roundabout/highway raw-sensor recording scenarios.

The official competition sample remains the source of NPC spawn routes and
speeds.  This script only removes unrelated actors and relocates Ego to the
mission approach in Park, ready for a human-driven rosbag recording.
"""

import argparse
import copy
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TEMPLATE = REPO_ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
DEFAULT_OUTPUT = REPO_ROOT / "configs/scenario/perception_benchmark"

EGO_POSES = {
    "roundabout": {
        "x": -98.95323590016368,
        "y": 319.2550292775465,
        "z": 28.690865271205574,
        "yaw": 74.91993199467063,
        "link": "A2256W000146",
        "link_ratio": 0.07894736842105263,
    },
    "highway_merge": {
        "x": 44.676904138191006,
        "y": 324.05794253100623,
        "z": 28.675191521049697,
        "yaw": -53.67606992160243,
        "link": "A2256W000846",
        "link_ratio": 0.4684014869888476,
    },
}

RETAINED_SPAWN_IDS = {
    "roundabout": {4},
    "highway_merge": {6, 8, 10},
}

OUTPUT_NAMES = {
    "roundabout": "bench_roundabout_raw.json",
    "highway_merge": "bench_highway_merge_raw.json",
}


def formatted_vector(x, y, z):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "_x": "{:.3f}".format(float(x)),
        "_y": "{:.3f}".format(float(y)),
        "_z": "{:.3f}".format(float(z)),
    }


def configure_parked_ego(scenario, pose):
    ego = scenario["egoVehicle"]
    # Scenario JSON uses MORAI's internal enum here: 0=Keyboard,
    # 1=AV-MORAI Built-In, 2=AV-ExternalCtrl.  Keep both fields at Keyboard
    # so loading the benchmark never restores the sample's built-in driver.
    ego["beforeControlMode"] = 0
    ego["currentControlMode"] = 0
    initial = ego["initPosition"]
    initial["initPositionMode"] = "Absolute"
    initial["pos"] = formatted_vector(pose["x"], pose["y"], pose["z"])
    initial["rot"] = {
        "roll": "0.000",
        "pitch": "0.000",
        "yaw": "{:.3f}".format(pose["yaw"]),
    }
    initial["initLink"] = pose["link"]
    initial["initLinkRatio"] = float(pose["link_ratio"])
    initial["gear"] = 1
    ego["currentVelocity"] = 0.0
    ego["customValue"] = 0.0
    saved = initial.get("vehicleSaveData", {})
    saved["velocity"] = 0.0
    for container in (saved, saved.get("vehicle", {})):
        velocity = container.get("lastVelocity")
        if isinstance(velocity, dict):
            velocity.update({"x": 0.0, "y": 0.0, "z": 0.0})
    inputs = saved.get("inputData")
    if isinstance(inputs, list) and len(inputs) > 6:
        inputs[6] = 1


def build_scenario(template, mission):
    scenario = copy.deepcopy(template)
    retained_ids = RETAINED_SPAWN_IDS[mission]
    scenario["spawnPointList"] = [
        spawn
        for spawn in scenario.get("spawnPointList", [])
        if int(spawn.get("UNIQUEID", -1)) in retained_ids
    ]
    for spawn in scenario["spawnPointList"]:
        # The official file was saved from an already-running simulation.
        # Reset runtime counters so every benchmark starts from the same phase.
        spawn["spawnedVehiclesCount"] = 0
        spawn["waitSpawnTime"] = 0.0
        spawn["isSpawning"] = True
    for key in (
        "vehicleList",
        "pedestrianList",
        "objectList",
        "pedspawnPointList",
        "aiPedstrianList",
        "waypointDataList",
    ):
        scenario[key] = []
    configure_parked_ego(scenario, EGO_POSES[mission])
    return scenario


def validate_scenario(scenario, mission):
    actual_ids = {int(item["UNIQUEID"]) for item in scenario["spawnPointList"]}
    if actual_ids != RETAINED_SPAWN_IDS[mission]:
        raise RuntimeError(
            "{} spawn IDs {} != {}".format(
                mission, sorted(actual_ids), sorted(RETAINED_SPAWN_IDS[mission])
            )
        )
    for spawn in scenario["spawnPointList"]:
        if (
            int(spawn.get("spawnedVehiclesCount", -1)) != 0
            or float(spawn.get("waitSpawnTime", -1.0)) != 0.0
            or not spawn.get("isSpawning")
        ):
            raise RuntimeError("spawn runtime state was not reset")
    for key in (
        "vehicleList",
        "pedestrianList",
        "objectList",
        "pedspawnPointList",
        "aiPedstrianList",
        "waypointDataList",
    ):
        if scenario.get(key):
            raise RuntimeError("{} is not empty in {}".format(key, mission))
    initial = scenario["egoVehicle"]["initPosition"]
    ego = scenario["egoVehicle"]
    if (
        int(ego.get("beforeControlMode", -1)) != 0
        or int(ego.get("currentControlMode", -1)) != 0
        or int(initial["gear"]) != 1
        or float(ego["currentVelocity"]) != 0.0
    ):
        raise RuntimeError("Ego must start in Keyboard mode, stationary in Park")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    template = json.loads(args.template.resolve().read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for mission in ("roundabout", "highway_merge"):
        scenario = build_scenario(template, mission)
        validate_scenario(scenario, mission)
        output = args.output_dir / OUTPUT_NAMES[mission]
        output.write_text(
            json.dumps(scenario, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            "{}: Ego=({:.3f}, {:.3f}), mode=Keyboard, gear=P, spawn_ids={}".format(
                output,
                EGO_POSES[mission]["x"],
                EGO_POSES[mission]["y"],
                sorted(RETAINED_SPAWN_IDS[mission]),
            )
        )


if __name__ == "__main__":
    main()
