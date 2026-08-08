#!/usr/bin/env python3
"""Generate the MORAI NPC-vehicle intersection validation matrix.

The three trigger scenes hold the saved ego poses for the three trigger
positions.  The left/right scenes hold the target NPC vehicle trajectories.
For every output scene, the trigger scene is used as the base and only its
target NPC vehicle is replaced with the left or right NPC trajectory.

The default mapping is:

* ``1_trigger.json``: before the stop line
* ``2_trigger.json``: arrow tip
* ``3_trigger.json``: first dashed line

Target and ego speeds are recorded in the filename.  The target NPC is created
by a dedicated Spawn Point every three seconds; its vehicle type and speed are
written into the Spawn Point fields.  Ego speed remains a planner launch
setting, so the validation runner must be launched at the matching speed.
"""

import argparse
import copy
import json
import math
import os
import re
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


DEFAULT_SCENARIO_DIR = (
    Path.home()
    / "Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario"
    / "R_KR_PR_K-city_2025"
)
DEFAULT_TRIGGER_TEMPLATES = (
    ("trigger01", DEFAULT_SCENARIO_DIR / "1_trigger.json"),
    ("trigger02", DEFAULT_SCENARIO_DIR / "2_trigger.json"),
    ("trigger03", DEFAULT_SCENARIO_DIR / "3_trigger.json"),
)
DEFAULT_VEHICLE_TEMPLATES = (
    ("left", DEFAULT_SCENARIO_DIR / "left_spawnpoint.json"),
    ("right", DEFAULT_SCENARIO_DIR / "right_spawnpoint.json"),
)
DEFAULT_TRIGGER_DESCRIPTIONS = {
    "trigger01": "before_stopline",
    "trigger02": "arrow_tip",
    "trigger03": "first_dashed_line",
}
DEFAULT_TARGET_SPEEDS_KPH = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
DEFAULT_EGO_SPEEDS_KPH = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
DEFAULT_TARGET_UNIQUE_ID = 14
DEFAULT_TARGET_SPAWN_UNIQUE_ID = 18
TARGET_SPAWN_PERIOD_SECONDS = 3.0
TARGET_MAXIMUM_SPAWN = 50


class GenerationError(RuntimeError):
    """Raised for invalid source scenes or unsafe output settings."""


def parse_named_path(raw: str) -> Tuple[str, Path]:
    name, delimiter, location = raw.partition("=")
    name = name.strip().lower()
    if not delimiter or not name or not location.strip():
        raise argparse.ArgumentTypeError(
            "template must use NAME=PATH, for example left=car_left_2trigger.json"
        )
    if any(character not in "abcdefghijklmnopqrstuvwxyz0123456789_-" for character in name):
        raise argparse.ArgumentTypeError(
            "template name may contain only lowercase letters, numbers, '_' and '-'"
        )
    return name, Path(location.strip()).expanduser()


def parse_number_list(raw: str, label: str) -> Tuple[float, ...]:
    values: List[float] = []
    for text in raw.split(","):
        try:
            value = float(text.strip())
        except ValueError as error:
            raise GenerationError("{} contains a non-numeric value: {!r}".format(label, text)) from error
        if value <= 0.0:
            raise GenerationError("{} values must be greater than zero".format(label))
        values.append(value)
    if not values:
        raise GenerationError("{} must contain at least one value".format(label))
    return tuple(values)


def compact_number(value: float) -> str:
    if value.is_integer():
        return str(int(value))
    return ("{:.3f}".format(value)).rstrip("0").rstrip(".").replace(".", "p")


def load_scene(path: Path, role: str) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            scene = json.load(stream)
    except FileNotFoundError as error:
        raise GenerationError("{} template does not exist: {}".format(role, path)) from error
    except json.JSONDecodeError as error:
        raise GenerationError("{} template is not valid JSON: {}".format(role, path)) from error

    if not isinstance(scene.get("egoVehicle"), dict):
        raise GenerationError("{} template does not contain egoVehicle: {}".format(role, path))
    if not isinstance(scene.get("vehicleList"), list):
        raise GenerationError("{} template does not contain vehicleList: {}".format(role, path))
    return scene


def unique_named_paths(
    values: Iterable[Tuple[str, Path]], argument_name: str
) -> List[Tuple[str, Path]]:
    result: List[Tuple[str, Path]] = []
    used = set()
    for name, path in values:
        if name in used:
            raise GenerationError("{} contains duplicate name: {}".format(argument_name, name))
        used.add(name)
        result.append((name, path))
    if not result:
        raise GenerationError("at least one {} is required".format(argument_name))
    return result


def target_vehicle(scene: Dict[str, Any], unique_id: int, role: str) -> Dict[str, Any]:
    matches = [
        vehicle
        for vehicle in scene["vehicleList"]
        if isinstance(vehicle, dict) and int(vehicle.get("UNIQUEID", -1)) == unique_id
    ]
    if len(matches) != 1:
        raise GenerationError(
            "{} template must contain exactly one vehicle UNIQUEID {} (found {})".format(
                role, unique_id, len(matches)
            )
        )
    return matches[0]


def target_spawn_point(
    scene: Dict[str, Any], unique_id: int, role: str
) -> Dict[str, Any]:
    matches = [
        spawn_point
        for spawn_point in scene.get("spawnPointList", [])
        if isinstance(spawn_point, dict)
        and int(spawn_point.get("UNIQUEID", -1)) == unique_id
    ]
    if len(matches) != 1:
        raise GenerationError(
            "{} template must contain exactly one Spawn Point UNIQUEID {} "
            "(found {})".format(role, unique_id, len(matches))
        )
    spawn_point = matches[0]
    start_link = spawn_point.get("startLinkInfo", {}).get("linkIdx", "")
    end_link = spawn_point.get("endLinkInfo", {}).get("linkIdx", "")
    if not start_link or not end_link:
        raise GenerationError(
            "{} Spawn Point UNIQUEID {} needs both start and destination links".format(
                role, unique_id
            )
        )
    return spawn_point


def scale_velocity(vector: Any, target_speed_kph: float) -> None:
    """Keep serialized physics state consistent with the requested speed."""
    if not isinstance(vector, dict):
        return
    try:
        x = float(vector.get("x", 0.0))
        y = float(vector.get("y", 0.0))
        z = float(vector.get("z", 0.0))
    except (TypeError, ValueError):
        return
    magnitude = math.sqrt(x * x + y * y + z * z)
    if magnitude <= 1.0e-8:
        return
    scale = (target_speed_kph / 3.6) / magnitude
    vector["x"] = x * scale
    vector["y"] = y * scale
    vector["z"] = z * scale


def set_vehicle_speed(vehicle: Dict[str, Any], speed_kph: float) -> None:
    """Set every MORAI velocity field that can otherwise retain template speed."""
    vehicle["velocityType"] = 2
    for field in (
        "constantVelocity",
        "desiredVelocity",
        "currentVelocity",
        "viewCurrentVelocity",
    ):
        vehicle[field] = speed_kph

    save_data = vehicle.get("initPosition", {}).get("vehicleSaveData")
    if not isinstance(save_data, dict):
        return
    save_data["velocity"] = speed_kph
    scale_velocity(save_data.get("lastVelocity"), speed_kph)
    saved_vehicle = save_data.get("vehicle")
    if isinstance(saved_vehicle, dict):
        scale_velocity(saved_vehicle.get("lastVelocity"), speed_kph)


def configure_target_spawn_point(
    spawn_point: Dict[str, Any], vehicle_model_id: int, target_speed_kph: float
) -> Dict[str, Any]:
    """Pin a Spawn Point to the validation vehicle and a three-second period."""
    configured = copy.deepcopy(spawn_point)
    configured["spawnVehicleTypeID"] = vehicle_model_id
    configured["ParameterType"] = 2
    configured["maximumSpawnVehicle"] = TARGET_MAXIMUM_SPAWN
    configured["minSpawnPeriod"] = TARGET_SPAWN_PERIOD_SECONDS
    configured["maxSpawnPeriod"] = TARGET_SPAWN_PERIOD_SECONDS
    configured["SpawnVelocityType"] = 2
    configured["MinSpawnVelocity_Custom"] = target_speed_kph
    configured["MaxSpawnVelocity_Custom"] = target_speed_kph
    configured["DesiredVelocityType"] = 2
    configured["MinDesiredVelocity_Custom"] = target_speed_kph
    configured["MaxDesiredVelocity_Custom"] = target_speed_kph
    configured["spawnedVehiclesCount"] = 0
    configured["isSpawning"] = False
    configured["waitSpawnTime"] = TARGET_SPAWN_PERIOD_SECONDS
    return configured


def is_target_path_entry(entry: Any, unique_id: int) -> bool:
    if not isinstance(entry, dict):
        return False
    return entry.get("pathName") in {
        "VehiclePath - {}".format(unique_id),
        "ObstaclePath - {}".format(unique_id),
    }


def configure_target_spawn(
    trigger_scene: Dict[str, Any], vehicle_scene: Dict[str, Any], unique_id: int,
    spawn_unique_id: int, target_speed_kph: float,
) -> Dict[str, Any]:
    """Preserve trigger ego state and add the dedicated repeating vehicle Spawn Point."""
    output = copy.deepcopy(trigger_scene)
    source_target = target_vehicle(vehicle_scene, unique_id, "vehicle")
    source_spawn = target_spawn_point(vehicle_scene, spawn_unique_id, "vehicle")
    configured_spawn = configure_target_spawn_point(
        source_spawn, int(source_target["DataID"]), target_speed_kph
    )

    # The original trigger scene contains a single NPC with this ID.  Removing
    # it avoids a one-off vehicle in addition to the repeating Spawn Point.
    target_vehicle(output, unique_id, "trigger")
    output["vehicleList"] = [
        vehicle
        for vehicle in output["vehicleList"]
        if not (
            isinstance(vehicle, dict) and int(vehicle.get("UNIQUEID", -1)) == unique_id
        )
    ]
    output["waypointDataList"] = [
        entry
        for entry in output.get("waypointDataList", [])
        if not is_target_path_entry(entry, unique_id)
    ]
    output["spawnPointList"] = [
        spawn_point
        for spawn_point in output.get("spawnPointList", [])
        if not (
            isinstance(spawn_point, dict)
            and int(spawn_point.get("UNIQUEID", -1)) == spawn_unique_id
        )
    ] + [configured_spawn]
    return output


def output_name(
    side: str, target_speed_kph: float, trigger_location: str, ego_speed_kph: float
) -> str:
    return "dynamic_car_{}_obj{}_{}_ego{}_pos001.json".format(
        side,
        compact_number(target_speed_kph).zfill(2),
        trigger_location,
        compact_number(ego_speed_kph).zfill(2),
    )


def generated_scenario_pattern(
    sides: Sequence[Tuple[str, Path]], triggers: Sequence[Tuple[str, Path]]
) -> re.Pattern[str]:
    side_names = "|".join(re.escape(name) for name, _ in sides)
    trigger_names = "|".join(
        re.escape(label)
        for label in {
            name for name, _ in triggers
        }
        | {
            DEFAULT_TRIGGER_DESCRIPTIONS.get(name, name) for name, _ in triggers
        }
    )
    return re.compile(
        r"^dynamic_car_(?:{})_obj[0-9p]+_(?:{})_ego[0-9p]+_pos001\.json$".format(
            side_names, trigger_names
        )
    )


def atomic_json(path: Path, document: Dict[str, Any]) -> None:
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trigger-template",
        action="append",
        type=parse_named_path,
        default=None,
        metavar="NAME=PATH",
        help="trigger/ego-pose scene template; may be repeated",
    )
    parser.add_argument(
        "--vehicle-template",
        action="append",
        type=parse_named_path,
        default=None,
        metavar="NAME=PATH",
        help="target vehicle trajectory template; may be repeated",
    )
    parser.add_argument("--target-unique-id", type=int, default=DEFAULT_TARGET_UNIQUE_ID)
    parser.add_argument(
        "--target-spawn-unique-id", type=int, default=DEFAULT_TARGET_SPAWN_UNIQUE_ID
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_SCENARIO_DIR)
    parser.add_argument(
        "--target-speeds-kph",
        default=",".join(compact_number(value) for value in DEFAULT_TARGET_SPEEDS_KPH),
    )
    parser.add_argument(
        "--ego-speeds-kph",
        default=",".join(compact_number(value) for value in DEFAULT_EGO_SPEEDS_KPH),
    )
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--sync-template-spawns",
        action="store_true",
        help=(
            "write the fixed vehicle model, 3-second period and maximum spawn "
            "settings back to the left/right Spawn Point templates"
        ),
    )
    parser.add_argument(
        "--prune",
        action="store_true",
        help="remove previously generated vehicle scenes outside this requested matrix",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        triggers = unique_named_paths(
            args.trigger_template or DEFAULT_TRIGGER_TEMPLATES, "trigger-template"
        )
        sides = unique_named_paths(
            args.vehicle_template or DEFAULT_VEHICLE_TEMPLATES, "vehicle-template"
        )
        target_speeds = parse_number_list(args.target_speeds_kph, "target speeds")
        ego_speeds = parse_number_list(args.ego_speeds_kph, "ego speeds")
        output_dir = args.output_dir.expanduser().resolve()

        trigger_scenes = [
            (name, path, load_scene(path, "trigger")) for name, path in triggers
        ]
        vehicle_scenes = [
            (name, path, load_scene(path, "vehicle")) for name, path in sides
        ]
        for _, _, scene in trigger_scenes:
            target_vehicle(scene, args.target_unique_id, "trigger")
        for _, _, scene in vehicle_scenes:
            target_vehicle(scene, args.target_unique_id, "vehicle")
            target_spawn_point(scene, args.target_spawn_unique_id, "vehicle")

        if args.sync_template_spawns:
            for _, path, scene in vehicle_scenes:
                vehicle = target_vehicle(scene, args.target_unique_id, "vehicle")
                replacement = configure_target_spawn_point(
                    target_spawn_point(
                        scene, args.target_spawn_unique_id, "vehicle"
                    ),
                    int(vehicle["DataID"]),
                    float(vehicle.get("desiredVelocity", 0.0)),
                )
                scene["spawnPointList"] = [
                    replacement
                    if isinstance(spawn_point, dict)
                    and int(spawn_point.get("UNIQUEID", -1))
                    == args.target_spawn_unique_id
                    else spawn_point
                    for spawn_point in scene["spawnPointList"]
                ]
                atomic_json(path, scene)

        plans: List[Tuple[Path, Dict[str, Any], Dict[str, Any]]] = []
        for side_name, side_path, vehicle_scene in vehicle_scenes:
            for trigger_name, trigger_path, trigger_scene in trigger_scenes:
                trigger_location = DEFAULT_TRIGGER_DESCRIPTIONS.get(
                    trigger_name, trigger_name
                )
                for target_speed_kph in target_speeds:
                    for ego_speed_kph in ego_speeds:
                        filename = output_name(
                            side_name,
                            target_speed_kph,
                            trigger_location,
                            ego_speed_kph,
                        )
                        scene = configure_target_spawn(
                            trigger_scene,
                            vehicle_scene,
                            args.target_unique_id,
                            args.target_spawn_unique_id,
                            target_speed_kph,
                        )
                        plans.append(
                            (
                                output_dir / filename,
                                scene,
                                {
                                    "file": filename,
                                    "side": side_name,
                                    "vehicle_template": side_path.name,
                                    "trigger": trigger_name,
                                    "trigger_description": trigger_location,
                                    "trigger_template": trigger_path.name,
                                    "target_speed_kph": target_speed_kph,
                                    "ego_speed_kph": ego_speed_kph,
                                    "target_spawn_point_unique_id": args.target_spawn_unique_id,
                                    "target_spawn_period_s": TARGET_SPAWN_PERIOD_SECONDS,
                                },
                            )
                        )

        index_path = output_dir / "dynamic_car_intersection_matrix_index.json"
        target_paths = [path for path, _, _ in plans] + [index_path]
        if not args.overwrite and not args.dry_run:
            existing = [path for path in target_paths if path.exists()]
            if existing:
                raise GenerationError(
                    "refusing to overwrite existing output(s); pass --overwrite: {}".format(
                        ", ".join(str(path) for path in existing[:5])
                    )
                )

        planned_names = {path.name for path, _, _ in plans}
        stale_paths: List[Path] = []
        if args.prune and output_dir.is_dir():
            pattern = generated_scenario_pattern(sides, triggers)
            stale_paths = sorted(
                (
                    path
                    for path in output_dir.iterdir()
                    if path.is_file()
                    and pattern.fullmatch(path.name)
                    and path.name not in planned_names
                ),
                key=lambda path: path.name,
            )

        print("Validated {} vehicle scenario file(s).".format(len(plans)))
        print("Output directory: {}".format(output_dir))
        if args.prune:
            print("Stale generated scene file(s) to remove: {}".format(len(stale_paths)))
        if args.dry_run:
            return 0

        output_dir.mkdir(parents=True, exist_ok=True)
        for output_path, scene, _ in plans:
            atomic_json(output_path, scene)
        index = {
            "schema_version": 1,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "description": (
                "MORAI NPC vehicle intersection matrix. Trigger scenes preserve the "
                "ego pose and the vehicle templates provide the target trajectory."
            ),
            "target_unique_id": args.target_unique_id,
            "target_speeds_kph": list(target_speeds),
            "ego_speeds_kph": list(ego_speeds),
            "scene_count": len(plans),
            "scenarios": [metadata for _, _, metadata in plans],
        }
        atomic_json(index_path, index)
        for stale_path in stale_paths:
            stale_path.unlink()
        if stale_paths:
            print("Removed stale generated scene file(s): {}".format(len(stale_paths)))
        print("Generated matrix index: {}".format(index_path))
        return 0
    except GenerationError as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
