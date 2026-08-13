#!/usr/bin/env python3
"""Audit generated MORAI actor sectors and mission-aware placement policy."""

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path


GROUP_NAMES = {
    "vehicleList": "vehicle",
    "pedestrianList": "pedestrian",
    "objectList": "obstacle",
}


def percentage(counter, key):
    total = sum(counter.values())
    return 0.0 if not total else 100.0 * counter[key] / total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metadata_dir", type=Path)
    parser.add_argument("--expected-scenes", type=int, default=180)
    args = parser.parse_args()

    paths = sorted(args.metadata_dir.glob("*.metadata.json"))
    if len(paths) != args.expected_scenes:
        raise RuntimeError(
            "metadata count {} != {}".format(len(paths), args.expected_scenes)
        )

    sectors = defaultdict(Counter)
    relations = Counter()
    splits = Counter()
    tangent_changes = []
    excluded_pose_hits = []
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        scenario_path = (
            args.metadata_dir.parent.parent
            / "scenarios"
            / args.metadata_dir.name
            / (document["scenario_stem"] + ".json")
        )
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        vehicle_links = {
            int(actor["UNIQUEID"]): actor["initPosition"]["initLink"]
            for actor in scenario.get("vehicleList", [])
        }
        exclusion_zones = document.get("geometry_policy", {}).get(
            "vehicle_link_exclusion_zones", []
        )
        splits[document["split"]] += 1
        ego = document["ego"]
        reference = document["capture_layout_reference"]
        delta = (reference["yaw"] - ego["yaw"] + 180.0) % 360.0 - 180.0
        tangent_changes.append((abs(delta), document["base_scenario_id"]))
        for actor in document["assets"]:
            group = GROUP_NAMES[actor["group"]]
            relation = actor["capture_layout_relation"]
            sectors[group][relation["directional_sector"]] += 1
            if group == "vehicle":
                relations[relation["traffic_direction_relation"]] += 1
                pose = actor["pose"]
                for zone in exclusion_zones:
                    if str(vehicle_links.get(actor["unique_id"])) != str(
                        zone["link_idx"]
                    ):
                        continue
                    if math.hypot(
                        pose["x"] - float(zone["x"]),
                        pose["y"] - float(zone["y"]),
                    ) <= float(zone["radius_m"]):
                        excluded_pose_hits.append(
                            (document["base_scenario_id"], actor["name"])
                        )

    expected_splits = Counter({"train": 126, "val": 27, "test": 27})
    if args.expected_scenes == 180 and splits != expected_splits:
        raise RuntimeError("unexpected split counts: {}".format(dict(splits)))
    if excluded_pose_hits:
        raise RuntimeError(
            "vehicle found in repeated-failure exclusion: {}".format(
                excluded_pose_hits
            )
        )

    checks = {
        "vehicle_front_min_45": percentage(sectors["vehicle"], "front") >= 45,
        "vehicle_rear_15_to_30": 15
        <= percentage(sectors["vehicle"], "rear")
        <= 30,
        "pedestrian_front_min_60": (
            percentage(sectors["pedestrian"], "front") >= 60
        ),
        "pedestrian_rear_max_10": (
            percentage(sectors["pedestrian"], "rear") <= 10
        ),
        "obstacle_front_min_60": (
            percentage(sectors["obstacle"], "front") >= 60
        ),
        "obstacle_rear_max_5": percentage(sectors["obstacle"], "rear") <= 5,
        "opposing_vehicle_10_to_30": 10
        <= percentage(relations, "opposing_direction")
        <= 30,
        "curved_reference_rotates": max(tangent_changes)[0] >= 10,
    }

    print("scenes:", len(paths), "split:", dict(sorted(splits.items())))
    for group in ("vehicle", "pedestrian", "obstacle"):
        values = sectors[group]
        print(
            "{} sectors: {}".format(
                group,
                {
                    key: {"count": values[key], "percent": round(percentage(values, key), 1)}
                    for key in ("front", "left", "right", "rear")
                },
            )
        )
    print(
        "vehicle relations:",
        {
            key: {"count": relations[key], "percent": round(percentage(relations, key), 1)}
            for key in (
                "same_direction",
                "opposing_direction",
                "crossing_or_merging",
            )
        },
    )
    print("maximum 4.5m-reference yaw change:", round(max(tangent_changes)[0], 2), "deg")
    print("checks:", checks)
    if not all(checks.values()):
        raise RuntimeError("one or more distribution checks failed")


if __name__ == "__main__":
    main()
