#!/usr/bin/env python3
"""Build the actor-free full-route scenario used for localization A/B tests."""

import argparse
import json
from copy import deepcopy
from pathlib import Path


ACTOR_FIELDS = (
    "vehicleList",
    "pedestrianList",
    "objectList",
    "spawnPointList",
    "pedspawnPointList",
    "aiPedstrianList",
    "waypointDataList",
)


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=root / "configs/scenario/2026_molit_comp_sample_scene.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            root
            / "configs/scenario/localization_evaluation/"
            "localization_full_route_no_actors.json"
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    source = json.loads(args.source.read_text(encoding="utf-8"))
    scenario = deepcopy(source)

    for field in ACTOR_FIELDS:
        if field not in scenario or not isinstance(scenario[field], list):
            raise ValueError("missing or invalid scenario field: {}".format(field))
        scenario[field] = []

    # These define the course and GNSS-shadow test, not perception actors.
    for field in ("egoVehicle", "mapInfo", "tlList", "shadedAreaList"):
        if scenario.get(field) != source.get(field):
            raise AssertionError("course field changed unexpectedly: {}".format(field))

    if not scenario["shadedAreaList"]:
        raise ValueError("source has no GNSS shaded area")
    if not scenario["tlList"]:
        raise ValueError("source has no traffic-light state list")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(scenario, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(args.output)


if __name__ == "__main__":
    main()
