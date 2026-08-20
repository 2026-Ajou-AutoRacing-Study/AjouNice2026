#!/usr/bin/env python3
"""Validate generated MORAI dynamic-object scenarios without launching MORAI."""

import argparse
import json
import math
from collections import Counter
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]

SPAWN_VEHICLE_FAMILIES = {
    "roundabout_sample_spawn",
    "roundabout_circulation",
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
    "highway_cruise_overtake",
}

TARGET_SPEED_START_FAMILIES = {
    "roundabout_circulation",
    "highway_merge_cruise",
    "highway_cruise_overtake",
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--report", type=Path)
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def resolve(value):
    path = Path(value)
    return path if path.is_absolute() else PROJECT_ROOT / path


def waypoint_to_world(position):
    return (-float(position["x"]), -float(position["z"]), float(position["y"]))


def shade_coordinates(shade, position):
    yaw = math.radians(float(shade["rot"]["yaw"]))
    dx = float(position["x"]) - float(shade["pos"]["x"])
    dy = float(position["y"]) - float(shade["pos"]["y"])
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
    )


def validate_scene(entry):
    scenario_path = resolve(entry["scenario"])
    metadata_path = resolve(entry["metadata"])
    scenario = load_json(scenario_path)
    metadata = load_json(metadata_path)
    family = entry["mission_family"]
    errors = []
    warnings = []

    if metadata["scene_id"] != entry["id"]:
        errors.append("scene id differs between manifest and metadata")
    if metadata["mission_family"] != family:
        errors.append("mission family differs between manifest and metadata")
    if len(entry["file"].encode("utf-8")) > 30:
        errors.append("scenario stem exceeds MORAI legacy 30-byte limit")
    ids = []
    for key in ("vehicleList", "pedestrianList", "objectList", "spawnPointList", "shadedAreaList"):
        ids.extend(item.get("UNIQUEID") for item in scenario.get(key, []))
    ids = [item for item in ids if item is not None]
    if len(ids) != len(set(ids)):
        errors.append("duplicate UNIQUEID")

    if family in SPAWN_VEHICLE_FAMILIES:
        motion_phase = metadata.get("motion_phase")
        sampling_role = metadata.get("sampling_role")
        if not motion_phase:
            errors.append("vehicle family metadata has no motion_phase")
        if sampling_role not in {
            "minority_hard_case", "production_representative"
        }:
            errors.append("vehicle family metadata has invalid sampling_role")
        if scenario["vehicleList"]:
            errors.append("spawn-driven vehicle scenario contains stale saved vehicles")
        if not scenario["spawnPointList"]:
            errors.append("vehicle scenario has no spawn point")
        for spawn in scenario["spawnPointList"]:
            if spawn["MinDesiredVelocity_Custom"] <= 0.0:
                errors.append("vehicle target speed is not positive")
            if spawn["minSpawnPeriod"] <= 0.0:
                errors.append("vehicle spawn period is not positive")
            if family in TARGET_SPEED_START_FAMILIES:
                if (
                    spawn["MinSpawnVelocity_Custom"]
                    != spawn["MinDesiredVelocity_Custom"]
                    or spawn["MaxSpawnVelocity_Custom"]
                    != spawn["MaxDesiredVelocity_Custom"]
                ):
                    errors.append(
                        "target-speed-start spawn and desired velocities differ"
                    )
            elif motion_phase == "spawn_acceleration" and (
                float(spawn.get("MinSpawnVelocity_Custom", 0.0))
                >= float(spawn["MinDesiredVelocity_Custom"])
            ):
                errors.append("spawn-acceleration family does not start below target speed")
        if family in {
            "highway_merge_sample_spawn",
            "highway_merge_cruise",
            "highway_cruise_overtake",
        }:
            waypoint_path = resolve(entry.get("waypoints", ""))
            if not waypoint_path.is_file():
                errors.append("highway vehicle route is missing")
            overrides = entry.get("capture_policy_overrides", {})
            if float(overrides.get("ego_target_speed_kph", 0.0)) <= 0.0:
                errors.append("highway Ego target speed is missing")
            if float(overrides.get("minimum_ego_speed_kph", 0.0)) <= 0.0:
                errors.append("highway Ego speed gate is missing")
            if entry.get("ego_readiness_policy") != "route_aligned":
                errors.append("moving highway Ego must use route-aligned readiness")
    elif family == "crosswalk_pedestrian":
        if len(scenario["pedestrianList"]) != 1:
            errors.append("pedestrian scene must contain exactly one primary pedestrian")
        if len(scenario["waypointDataList"]) != 1:
            errors.append("pedestrian scene must contain exactly one path")
        else:
            points = scenario["waypointDataList"][0].get("waypointData", [])
            if len(points) < 2:
                errors.append("pedestrian path has fewer than two points")
            else:
                start = waypoint_to_world(points[0]["pos"])
                actor = scenario["pedestrianList"][0]["pos"]
                gap = math.hypot(start[0] - actor["x"], start[1] - actor["y"])
                if gap > 0.05:
                    errors.append(f"pedestrian actor/path start gap is {gap:.3f} m")
    elif family.endswith("obstacle"):
        if len(scenario["objectList"]) != 1:
            errors.append("obstacle scene must contain exactly one obstacle")
        elif scenario["objectList"][0].get("moveSpeed") != 0.0:
            errors.append("competition obstacle must remain static")

    if family == "gps_shadow_obstacle" and scenario["objectList"]:
        shade = scenario["shadedAreaList"][0]
        longitudinal, lateral = shade_coordinates(shade, scenario["objectList"][0]["pos"])
        # The obstacle is map-matched to the road after sampling within the
        # OBB. Allow a small lateral margin for MGeo/Lanelet center mismatch.
        if abs(longitudinal) > float(shade["size"]["x"]) / 2 + 2.0:
            errors.append("GPS obstacle is outside shaded area longitudinally")
        if abs(lateral) > float(shade["size"]["y"]) / 2 + 2.0:
            errors.append("GPS obstacle is outside shaded area laterally")

    if not metadata.get("runtime_actors"):
        errors.append("runtime actor metadata is empty")
    policy = metadata.get("capture_policy", {})
    if policy.get("format") != "morai_save_sensor_data_timed_grouped":
        errors.append("dynamic scene must use timed grouped SaveSensorData capture")
    if policy.get("split_unit") != "scenario_clip":
        errors.append("temporal split unit must be scenario_clip")
    if int(policy.get("captures_per_scene", 0)) < 2:
        errors.append("temporal clip must contain at least two SaveSensorData frames")
    if float(policy.get("capture_period_sec", 0.0)) <= 0.0:
        errors.append("capture_period_sec must be positive")
    required_sensor_dirs = set(policy.get("required_sensor_dirs", []))
    expected_sensor_dirs = {
        "CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4", "LIDAR_5"
    }
    if required_sensor_dirs != expected_sensor_dirs:
        errors.append("required SaveSensorData sensor directories are incomplete")
    if family == "crosswalk_pedestrian":
        units = metadata["runtime_actors"][0].get("speed_unit")
        if units and units.endswith("unverified"):
            warnings.append("pedestrian speed config unit requires MORAI displacement audit")
        runtime = metadata["runtime_actors"][0]
        initial_distance = float(runtime.get("initial_ego_distance_m", 0.0))
        active_distance = float(runtime.get("active_distance_m", 0.0))
        if active_distance <= initial_distance:
            errors.append(
                "pedestrian activeDistance does not include initial Ego pose"
            )

    return {
        "id": entry["id"],
        "family": family,
        "scenario": str(scenario_path),
        "errors": errors,
        "warnings": warnings,
    }


def main():
    args = parse_args()
    manifest = load_json(args.manifest)
    results = [validate_scene(entry) for entry in manifest["scenes"]]
    errors = [message for item in results for message in item["errors"]]
    warnings = [message for item in results for message in item["warnings"]]
    report = {
        "schema_version": 1,
        "manifest": str(args.manifest.resolve()),
        "scenario_count": len(results),
        "family_counts": dict(Counter(item["family"] for item in results)),
        "error_count": len(errors),
        "warning_count": len(warnings),
        "status": "pass" if not errors else "fail",
        "scenes": results,
    }
    report_path = args.report or (
        PROJECT_ROOT
        / "dataset/morai_od/reports"
        / f"{args.manifest.stem}_validation.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"scenarios: {len(results)}")
    print(f"errors: {len(errors)}")
    print(f"warnings: {len(warnings)}")
    print(f"status: {report['status']}")
    print(f"report: {report_path}")
    if errors:
        for item in results:
            for error in item["errors"]:
                print(f"ERROR {item['id']}: {error}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
