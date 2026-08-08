#!/usr/bin/env python3
"""Generate repeatable MORAI pedestrian-crossing scenario variants.

The generator keeps the position, heading and waypoint data saved in the two
MORAI scene templates.  It changes only the pedestrian model identity, its
walk speed and its activation distance.  Ego speed is a planner launch
parameter rather than a MORAI scene field, so it is recorded in the generated
matrix index for the validation runner.

The default templates are the two scenes currently saved in the local MORAI
K-City scenario directory:

* ``Dynamic_kid_10.json``: front crosswalk
* ``dynamic_test_1.json``: opposite crosswalk

Use ``--site-template`` and ``--actor-template`` to point at any other MORAI
scene templates.  A template must contain exactly one pedestrian.  This is
intentional: a group such as person+dog needs a dedicated group generator so
that the second actor's relative path is not accidentally misplaced.
"""

import argparse
import copy
import json
import os
import re
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple


DEFAULT_SCENARIO_DIR = (
    Path.home()
    / "Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario"
    / "R_KR_PR_K-city_2025"
)
DEFAULT_KID_TEMPLATE = DEFAULT_SCENARIO_DIR / "Dynamic_kid_10.json"
DEFAULT_ADULT_TEMPLATE = DEFAULT_SCENARIO_DIR / "dynamic_test_1.json"
DEFAULT_BICYCLE_TEMPLATE = DEFAULT_SCENARIO_DIR / "bicycle.json"
DEFAULT_DEER_TEMPLATE = DEFAULT_SCENARIO_DIR / "deer.json"
DEFAULT_DOG_TEMPLATE = DEFAULT_SCENARIO_DIR / "dog.json"
DEFAULT_HUMAN_DOG_TEMPLATE = DEFAULT_SCENARIO_DIR / "dogandpedestrian.json"
DEFAULT_OBJECT_SPEEDS_KPH = (2.0, 3.0, 5.0, 10.0, 15.0)
DEFAULT_ACTIVATION_DISTANCES_M = (7.0, 10.0, 15.0)
DEFAULT_EGO_SPEEDS_KPH = (5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0)

# These are model/animation fields.  Route and position fields deliberately
# stay from the site template, including waypointDataList at scene level.
PEDESTRIAN_IDENTITY_FIELDS = (
    "DataID",
    "m_eobstacleObjType",
    "pedestrianType",
    "anim",
    "stateNameHash",
    "stateNormalizedTime",
    "animSpeed",
)


class GenerationError(RuntimeError):
    """Raised for invalid templates or unsafe generation settings."""


def parse_named_path(raw: str) -> Tuple[str, Path]:
    """Parse ``name=/absolute/or/relative/path.json`` arguments."""
    name, delimiter, location = raw.partition("=")
    name = name.strip().lower()
    if not delimiter or not name or not location.strip():
        raise argparse.ArgumentTypeError(
            "template must use NAME=PATH, for example kid=Dynamic_kid_10.json"
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
    """Use stable, file-name-safe whole-number labels where possible."""
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

    pedestrians = scene.get("pedestrianList")
    if not isinstance(pedestrians, list) or len(pedestrians) != 1:
        count = len(pedestrians) if isinstance(pedestrians, list) else "missing"
        raise GenerationError(
            "{} template must contain exactly one pedestrian (found {}) in {}".format(
                role, count, path
            )
        )
    if not isinstance(pedestrians[0], dict):
        raise GenerationError("{} pedestrian entry is not an object: {}".format(role, path))
    return scene


def atomic_json(path: Path, document: Dict[str, Any]) -> None:
    """Write a complete JSON file or leave an existing output untouched."""
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


def build_pedestrian(
    site_pedestrian: Dict[str, Any], actor_pedestrian: Dict[str, Any], speed_kph: float,
    activation_distance_m: float,
) -> Dict[str, Any]:
    """Put an actor model on a site while retaining its saved crossing route."""
    pedestrian = copy.deepcopy(site_pedestrian)
    for field in PEDESTRIAN_IDENTITY_FIELDS:
        if field in actor_pedestrian:
            pedestrian[field] = copy.deepcopy(actor_pedestrian[field])

    # MORAI's Scenario Editor stores these UI values directly in km/h and m.
    pedestrian["speed"] = speed_kph
    pedestrian["activeDistance"] = activation_distance_m
    pedestrian["active"] = True
    return pedestrian


def scenario_name(
    actor_name: str,
    place_name: str,
    object_speed_kph: float,
    crosswalk_name: str,
    ego_speed_kph: float,
    activation_distance_m: float,
) -> str:
    """Create one stable name per full validation condition.

    The requested field order is dynamic / object / place / object speed /
    front-or-opposite crosswalk / ego speed.  ``act`` is appended because
    MORAI activation distance is another varied input and omitting it would
    overwrite four otherwise distinct cases.
    """
    return "dynamic_{}_{}_obj{}_{}_crosswalk_ego{}_act{}_pos001.json".format(
        actor_name,
        place_name,
        compact_number(object_speed_kph).zfill(2),
        crosswalk_name,
        compact_number(ego_speed_kph).zfill(2),
        compact_number(activation_distance_m).zfill(2),
    )


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


def generated_scenario_pattern(
    actors: Sequence[Tuple[str, Path]], sites: Sequence[Tuple[str, Path]]
) -> re.Pattern[str]:
    """Match only scenario files whose names this invocation owns."""
    actor_names = "|".join(re.escape(name) for name, _ in actors)
    site_names = "|".join(re.escape(name) for name, _ in sites)
    return re.compile(
        r"^dynamic_(?:{})_loc\d{{2}}_obj[0-9p]+_(?:{})_crosswalk_ego[0-9p]+_"
        r"act[0-9p]+_pos001\.json$".format(actor_names, site_names)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--site-template",
        action="append",
        type=parse_named_path,
        default=None,
        metavar="NAME=PATH",
        help="crosswalk location template; may be repeated",
    )
    parser.add_argument(
        "--actor-template",
        action="append",
        type=parse_named_path,
        default=None,
        metavar="NAME=PATH",
        help="single-pedestrian actor template; may be repeated",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help=(
            "MORAI scenario directory receiving JSON files; defaults to the common "
            "parent of the site templates"
        ),
    )
    parser.add_argument(
        "--object-speeds-kph",
        default=",".join(compact_number(value) for value in DEFAULT_OBJECT_SPEEDS_KPH),
        help="comma-separated pedestrian speeds in km/h",
    )
    parser.add_argument(
        "--activation-distances-m",
        default=",".join(
            compact_number(value) for value in DEFAULT_ACTIVATION_DISTANCES_M
        ),
        help="comma-separated MORAI activation distances in metres",
    )
    parser.add_argument(
        "--ego-speeds-kph",
        default=",".join(compact_number(value) for value in DEFAULT_EGO_SPEEDS_KPH),
        help="comma-separated planner speed cases recorded in the matrix index",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace only files this generator would create",
    )
    parser.add_argument(
        "--prune",
        action="store_true",
        help=(
            "remove previously generated files for these actors/sites that are not "
            "in the requested matrix"
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate templates and print the plan without writing files",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        site_values = args.site_template or [
            ("front", DEFAULT_KID_TEMPLATE),
            ("opposite", DEFAULT_ADULT_TEMPLATE),
        ]
        actor_values = args.actor_template or [
            ("kid", DEFAULT_KID_TEMPLATE),
            ("adult", DEFAULT_ADULT_TEMPLATE),
            ("bicycle", DEFAULT_BICYCLE_TEMPLATE),
            ("deer", DEFAULT_DEER_TEMPLATE),
            ("dog", DEFAULT_DOG_TEMPLATE),
            ("human_dog", DEFAULT_HUMAN_DOG_TEMPLATE),
        ]
        sites = unique_named_paths(site_values, "site-template")
        actors = unique_named_paths(actor_values, "actor-template")
        object_speeds = parse_number_list(args.object_speeds_kph, "object speeds")
        activation_distances = parse_number_list(
            args.activation_distances_m, "activation distances"
        )
        ego_speeds = parse_number_list(args.ego_speeds_kph, "ego speeds")

        site_scenes = [(name, path, load_scene(path, "site")) for name, path in sites]
        actor_scenes = [(name, path, load_scene(path, "actor")) for name, path in actors]

        if args.output_dir is None:
            parents = {path.resolve().parent for _, path in sites}
            if len(parents) != 1:
                raise GenerationError(
                    "--output-dir is required when site templates have different parents"
                )
            output_dir = parents.pop()
        else:
            output_dir = args.output_dir.expanduser().resolve()

        plans: List[Tuple[Path, Dict[str, Any], Dict[str, Any]]] = []
        for actor_name, actor_path, actor_scene in actor_scenes:
            actor_pedestrian = actor_scene["pedestrianList"][0]
            for site_number, (site_name, site_path, site_scene) in enumerate(
                site_scenes, start=1
            ):
                site_pedestrian = site_scene["pedestrianList"][0]
                place_name = "loc{:02d}".format(site_number)
                for speed_kph in object_speeds:
                    for activation_distance_m in activation_distances:
                        for ego_speed_kph in ego_speeds:
                            filename = scenario_name(
                                actor_name,
                                place_name,
                                speed_kph,
                                site_name,
                                ego_speed_kph,
                                activation_distance_m,
                            )
                            scene = copy.deepcopy(site_scene)
                            scene["pedestrianList"] = [
                                build_pedestrian(
                                    site_pedestrian,
                                    actor_pedestrian,
                                    speed_kph,
                                    activation_distance_m,
                                )
                            ]
                            metadata = {
                            "file": filename,
                            "actor": actor_name,
                            "actor_template": actor_path.name,
                            "place": place_name,
                            "site": site_name,
                            "site_template": site_path.name,
                                "object_speed_kph": speed_kph,
                                "activation_distance_m": activation_distance_m,
                                "ego_speed_kph": ego_speed_kph,
                            }
                            plans.append((output_dir / filename, scene, metadata))

        index_path = output_dir / "dynamic_crosswalk_matrix_index.json"
        targets = [path for path, _, _ in plans] + [index_path]
        if not args.overwrite and not args.dry_run:
            existing = [path for path in targets if path.exists()]
            if existing:
                raise GenerationError(
                    "refusing to overwrite existing output(s); pass --overwrite: {}".format(
                        ", ".join(str(path) for path in existing[:5])
                    )
                )

        planned_names = {path.name for path, _, _ in plans}
        stale_paths: List[Path] = []
        if args.prune and output_dir.is_dir():
            pattern = generated_scenario_pattern(actors, sites)
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

        total_cases = len(plans)
        print(
            "Validated {} scene file(s), each representing one planner-speed case.".format(
                len(plans), total_cases
            )
        )
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
                "MORAI crossing scenes. activation_distance_m is MORAI's "
                "Activation Distance setting; ego speeds are planner launch cases."
            ),
            "object_speeds_kph": list(object_speeds),
            "activation_distances_m": list(activation_distances),
            "ego_speeds_kph": list(ego_speeds),
            "scene_count": len(plans),
            "planner_speed_case_count": total_cases,
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
        print("error: {}".format(error), file=os.sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
