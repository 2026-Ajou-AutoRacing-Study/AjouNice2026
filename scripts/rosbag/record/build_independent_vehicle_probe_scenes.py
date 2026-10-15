#!/usr/bin/env python3
"""Prepare bounded, independently parameterized MORAI vehicle QA scenes.

This is a pure file-preparation step: no ROS, simulator mutation, installation,
GPU inference, upload, or production policy changes. Authored road geometry
and Ego's benchmark starting pose remain exact. NPC appearance, initial/goal
speed and spawn period differ from the sample and the earlier failure bag.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BENCHMARK = ROOT / "configs/scenario/perception_benchmark"
SAMPLE = ROOT / "configs/scenario/2026_molit_comp_sample_scene.json"
CATALOG = ROOT / "dataset/morai_od/configs/morai_asset_catalog_mr_base5_261005.json"
MISSION_BASE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/missions"
EMPTY_ACTOR_GROUPS = (
    "vehicleList", "pedestrianList", "objectList", "pedspawnPointList",
    "aiPedstrianList", "waypointDataList",
)

# Speed fields use MORAI's stored custom-speed km/h convention. The three
# IDs in each highway profile are keyed by spawn ID, not by file list order.
# Existing no-lane-change MGeo start/end links and clearance settings survive.
PROFILES = (
    {
        "name": "ind_highway_launch",
        "mission": "highway",
        "source": "bench_highway_merge_raw.json",
        "intent": "Young tracks during NPC launch; mixed body shapes and accelerations.",
        "spawns": {
            6: {"data_id": 30100007, "initial_kmh": 0., "goal_kmh": 36., "period_sec": 13.},
            8: {"data_id": 30200003, "initial_kmh": 0., "goal_kmh": 48., "period_sec": 13.},
            10: {"data_id": 30100014, "initial_kmh": 0., "goal_kmh": 62., "period_sec": 13.},
        },
    },
    {
        "name": "ind_highway_stream",
        "mission": "highway",
        "source": "bench_highway_merge_raw.json",
        "intent": "Moving-entry tracks, relative-speed variation and natural occlusion/reappearance.",
        "spawns": {
            6: {"data_id": 30100010, "initial_kmh": 24., "goal_kmh": 50., "period_sec": 15.},
            8: {"data_id": 30100003, "initial_kmh": 34., "goal_kmh": 62., "period_sec": 15.},
            10: {"data_id": 30300002, "initial_kmh": 42., "goal_kmh": 74., "period_sec": 15.},
        },
    },
    {
        "name": "ind_roundabout_curve",
        "mission": "roundabout",
        "source": "bench_roundabout_raw.json",
        "intent": "Normal turning-direction control and raw LiDAR boundary behavior during yaw change.",
        "spawns": {
            4: {"data_id": 30100007, "initial_kmh": 7., "goal_kmh": 24., "period_sec": 7.5},
        },
    },
)

MUTABLE_SPAWN_FIELDS = frozenset((
    "spawnVehicleTypeID", "minSpawnPeriod", "maxSpawnPeriod", "SpawnVelocityType",
    "MinSpawnVelocity_Custom", "MaxSpawnVelocity_Custom", "DesiredVelocityType",
    "MinDesiredVelocity_Custom", "MaxDesiredVelocity_Custom", "maximumSpawnVehicle",
    "spawnedVehiclesCount", "isSpawning", "waitSpawnTime",
))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def asset_names(catalog):
    return {
        int(asset["data_id"]): asset["name"]
        for asset in catalog["assets"]
        if asset.get("source_list") == "vehicleList"
        and asset.get("generation_policy") == "enabled"
    }


def validate_settings(settings, assets):
    if int(settings["data_id"]) not in assets:
        raise ValueError("Requested model is not an enabled MORAI vehicle asset")
    initial, goal, period = (float(settings[key]) for key in ("initial_kmh", "goal_kmh", "period_sec"))
    if not all(math.isfinite(value) for value in (initial, goal, period)):
        raise ValueError("Finite speeds and period required")
    if not 0. <= initial <= goal <= 80. or not 7. <= period <= 20.:
        raise ValueError("Vehicle probe speed/spacing envelope exceeded")


def build_scene(source, profile, assets):
    """Change only a declared traffic parameter subset; no guessed terrain Z."""
    scene = copy.deepcopy(source)
    spawns = scene.get("spawnPointList", [])
    source_ids = [int(spawn["UNIQUEID"]) for spawn in spawns]
    if len(set(source_ids)) != len(source_ids) or set(source_ids) != set(profile["spawns"]):
        raise ValueError("Profile must bind every retained spawn once, with no extra IDs")
    if any(scene.get(group) for group in EMPTY_ACTOR_GROUPS):
        raise ValueError("Expected the empty-actor raw benchmark, not a saved running scene")
    for spawn in spawns:
        if spawn.get("isLaneChange") or spawn.get("isCloseLoop"):
            raise ValueError("Expected authored forward no-lane-change MGeo spawn routes")
        settings = profile["spawns"][int(spawn["UNIQUEID"])]
        validate_settings(settings, assets)
        spawn.update(
            spawnVehicleTypeID=int(settings["data_id"]),
            SpawnVelocityType=2,
            MinSpawnVelocity_Custom=float(settings["initial_kmh"]),
            MaxSpawnVelocity_Custom=float(settings["initial_kmh"]),
            DesiredVelocityType=2,
            MinDesiredVelocity_Custom=float(settings["goal_kmh"]),
            MaxDesiredVelocity_Custom=float(settings["goal_kmh"]),
            minSpawnPeriod=float(settings["period_sec"]),
            maxSpawnPeriod=float(settings["period_sec"]),
            maximumSpawnVehicle=12,
            spawnedVehiclesCount=0,
            isSpawning=True,
            waitSpawnTime=0.,
        )
    validate_geometry_preserved(source, scene)
    return scene


def validate_geometry_preserved(source, scene):
    """Fail closed for any edit not in the small declared spawn-field allowlist."""
    if set(source) != set(scene):
        raise ValueError("Scenario top-level keys changed")
    for key, value in source.items():
        if key != "spawnPointList" and scene[key] != value:
            raise ValueError("Non-spawn scenario data changed: " + key)
    if len(source["spawnPointList"]) != len(scene["spawnPointList"]):
        raise ValueError("Spawn count changed")
    for original, modified in zip(source["spawnPointList"], scene["spawnPointList"]):
        if set(original) != set(modified):
            raise ValueError("Spawn schema changed")
        for key, value in original.items():
            if key not in MUTABLE_SPAWN_FIELDS and modified[key] != value:
                raise ValueError("Spawn geometry/route/safety data changed: " + key)
    ego = scene["egoVehicle"]
    if ego["beforeControlMode"] != 0 or ego["currentControlMode"] != 0:
        raise ValueError("Preparation must retain Keyboard mode; runtime owns ExternalCtrl")
    if ego["initPosition"]["gear"] != 1 or float(ego["currentVelocity"]) != 0.:
        raise ValueError("Prepared Ego must remain stationary in Park")


def prepare(output, selected=None):
    """Write scenes and provenance to a fresh directory; never install or run."""
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError("Fresh scene output required; existing evidence is never overwritten")
    names = {profile["name"] for profile in PROFILES}
    if selected is not None and (not selected or set(selected) - names):
        raise ValueError("Select one or more known scene names")
    assets = asset_names(json.loads(CATALOG.read_text()))
    rows, built, sources = [], {}, {str(SAMPLE): sha256(SAMPLE), str(CATALOG): sha256(CATALOG)}
    # Validate every requested scene before creating any output files.
    for profile in PROFILES:
        name, mission = profile["name"], profile["mission"]
        if selected is not None and name not in selected:
            continue
        source = BENCHMARK / profile["source"]
        mission_cfg = MISSION_BASE / mission / "base/example.yaml"
        sources.update({str(path): sha256(path) for path in (source, mission_cfg)})
        built[name] = build_scene(json.loads(source.read_text()), profile, assets)
        rows.append(dict(
            name=name, mission=mission, source=str(source), source_sha256=sha256(source),
            path=str(output / "scenes" / (name + ".json")),
            mission_configuration_source=str(mission_cfg),
            mission_configuration_sha256=sha256(mission_cfg),
            intent=profile["intent"],
            traffic=[dict(spawn_id=identity, asset=assets[settings["data_id"]], **settings)
                     for identity, settings in sorted(profile["spawns"].items())],
            independence="New fixed traffic parameters; same surveyed map route and benchmark Ego pose.",
            actual_reverse_npc=False,
            live_observed=False,
            event_occurrence_guaranteed=False,
            safety_certified=False,
        ))
    output.mkdir(parents=True, exist_ok=False)
    for name, scene in built.items():
        write_json(output / "scenes" / (name + ".json"), scene)
    for row in rows:
        row["sha256"] = sha256(row["path"])
    plan = dict(
        schema_version=1,
        source_sha256=sources,
        scenes=rows,
        preparation_only=True,
        production_changed=False,
        bulk_dataset=False,
        geometry_policy="Exact authored spawn XYZ/rot/start/end links, lateral bias, safety distance and Ego pose retained.",
        evaluation_contract={
            "same_input_ab": "Replay the exact saved raw packets/camera and Object GT for every tracking candidate.",
            "lidar_policy": "Runtime chooses 50/51 only; this scene generator changes neither RPM nor timestamps.",
            "live_required": ["Actual requested NPC models/speeds", "Object GT and raw sensor completeness",
                              "Collision/stream/route/stop checks", "Natural occlusion/reappearance coverage"],
            "no_guarantee": "Changed traffic is not proof of independent detection errors or controlled reacquisition events.",
            "reverse_scope": "MGeo forward-route scenes do not certify legitimate reversing; keep synthetic reverse/stop/U-turn tests separate.",
            "roundabout_reason": "One normal turning control tests direction-guard release and scan boundary behavior, not only straight-line failures.",
            "no_dataset_collection": "Three bounded QA scenes only; no 5K generation/capture.",
        },
    )
    write_json(output / "plan.json", plan)
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", action="append", choices=[profile["name"] for profile in PROFILES])
    args = parser.parse_args()
    plan = prepare(args.output, args.scene)
    print(json.dumps(dict(output=str(args.output.resolve()), scenes=[row["name"] for row in plan["scenes"]],
                          preparation_only=True), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
