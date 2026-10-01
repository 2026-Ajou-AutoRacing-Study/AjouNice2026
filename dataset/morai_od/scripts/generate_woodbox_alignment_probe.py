#!/usr/bin/env python3
"""Generate paired CargoBox/WoodBox geometry audit scenes."""

import argparse
import copy
import json
import math
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-scenario",
        type=Path,
        default=Path(
            "dataset/morai_od/scenarios/temporal_sequence_v2_batch005_primary/"
            "t2base2b005p_so018.json"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("dataset/morai_od/scenarios/woodbox_alignment_260920"),
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("dataset/morai_od/manifests/woodbox_alignment_260920.json"),
    )
    parser.add_argument("--distances-m", default="5,10,15,20")
    parser.add_argument("--relative-yaws-deg", default="0,30,45,60,90")
    parser.add_argument(
        "--rolling-placements",
        help=(
            "Optional semicolon-separated longitudinal:lateral:yaw triples in "
            "the Ego start frame. When set, replaces the distance/yaw grid; "
            "for example '14:2.5:0;14:-2.5:30'."
        ),
    )
    parser.add_argument(
        "--anchor-index",
        type=int,
        help="Route anchor required by the rolling-grouped scenario runner.",
    )
    parser.add_argument("--scene-prefix", default="wba")
    return parser.parse_args()


def xyz(position, x, y, z):
    position.update({"x": x, "y": y, "z": z, "_x": f"{x:.3f}", "_y": f"{y:.3f}", "_z": f"{z:.3f}"})


def main():
    args = parse_args()
    distances = [float(value) for value in args.distances_m.split(",")]
    relative_yaws = [float(value) for value in args.relative_yaws_deg.split(",")]
    if args.rolling_placements:
        placements = []
        for raw in args.rolling_placements.split(";"):
            longitudinal, lateral, relative_yaw = (
                float(value) for value in raw.split(":")
            )
            placements.append((longitudinal, lateral, relative_yaw))
    else:
        placements = [
            (distance, 0.0, relative_yaw)
            for distance in distances
            for relative_yaw in relative_yaws
        ]
    base = json.loads(args.base_scenario.read_text(encoding="utf-8"))
    if not base.get("objectList"):
        raise ValueError("base scenario has no object template")
    object_template = copy.deepcopy(base["objectList"][0])
    ego_position = base["egoVehicle"]["initPosition"]["pos"]
    ego_yaw = float(base["egoVehicle"]["initPosition"]["rot"]["yaw"])
    ground_z = float(object_template["pos"]["z"])
    forward = (math.cos(math.radians(ego_yaw)), math.sin(math.radians(ego_yaw)))
    left = (-forward[1], forward[0])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    scenes = []
    index = 0
    for longitudinal, lateral, relative_yaw in placements:
        for asset_name, data_id, role in (
            ("CargoBox", 40100001, "control"),
            ("WoodBox", 40100002, "probe"),
        ):
            index += 1
            scene_id = f"{args.scene_prefix}{index:03d}"
            scene = copy.deepcopy(base)
            scene["vehicleList"] = []
            scene["pedestrianList"] = []
            scene["spawnPointList"] = []
            scene["pedspawnPointList"] = []
            scene["aiPedstrianList"] = []
            # Geometry probes do not exercise traffic lights, GPS shadow,
            # friction changes, or stored waypoints.  Strip this unrelated
            # MRP-derived state before installing the scene in MR.  Note that
            # MR can still emit IntTL log lines with an empty tlList; those
            # messages are map/runtime generated and are not a tlList test.
            scene["tlList"] = []
            scene["shadedAreaList"] = []
            scene["frictionChangeMaterialList"] = []
            scene["waypointDataList"] = []
            actor = copy.deepcopy(object_template)
            actor["DataID"] = data_id
            actor["UNIQUEID"] = 3001
            actor["scale"] = {
                "x": 1.0, "y": 1.0, "z": 1.0,
                "_x": "1.000", "_y": "1.000", "_z": "1.000",
            }
            x = (
                float(ego_position["x"])
                + longitudinal * forward[0]
                + lateral * left[0]
            )
            y = (
                float(ego_position["y"])
                + longitudinal * forward[1]
                + lateral * left[1]
            )
            xyz(actor["pos"], x, y, ground_z)
            actor["rot"].update(
                {
                    "roll": "0.000",
                    "pitch": "0.000",
                    "yaw": f"{ego_yaw + relative_yaw:.3f}",
                }
            )
            # MORAI serializes initPos with an internal axis convention.
            xyz(actor["initPos"], -x, ground_z, -y)
            scene["objectList"] = [actor]

            scenario_path = args.output_dir / f"{scene_id}.json"
            scenario_path.write_text(
                json.dumps(scene, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            scene_record = {
                "id": scene_id,
                "file": scene_id,
                "file_name": scene_id,
                "scenario": str(scenario_path),
                "expected_ego": {
                    key: float(ego_position[key]) for key in ("x", "y", "z")
                },
                "expected_counts": {
                    "npc": 0, "pedestrian": 0, "obstacle": 1
                },
                "expected": {
                    "static_npc": 0,
                    "pedestrian": 0,
                    "obstacle": 1,
                    "spawn_points": 0,
                },
                "asset_probe": {
                    "role": role,
                    "group": "woodbox_alignment",
                    "asset_name": asset_name,
                    "data_id": data_id,
                    "distance_m": math.hypot(longitudinal, lateral),
                    "longitudinal_m": longitudinal,
                    "lateral_m": lateral,
                    "bearing_deg": math.degrees(
                        math.atan2(lateral, longitudinal)
                    ),
                    "relative_yaw_deg": relative_yaw,
                },
            }
            if args.anchor_index is not None:
                scene_record["anchor_index"] = args.anchor_index
            scenes.append(scene_record)

    manifest = {
        "schema_version": 1,
        "dataset_id": "morai_woodbox_alignment_260920",
        "scenario_group": "woodbox_alignment_260920",
        "environment": {"weather": "Sunny", "time": "13:00"},
        "waypoints": "configs/global_path/2026_molit_comp_global_path.txt",
        "capture_policy": {
            "format": (
                "rosbag_rolling_geometry_probe"
                if args.anchor_index is not None
                else "rosbag_stationary_geometry_probe"
            ),
            "required_topics": [
                "/Ego_topic", "/Object_topic", "/sensor/lidar1",
                "/sensor/camera/front/compressed", "/dataset/scenario_marker",
            ],
        },
        "scenes": scenes,
        "generation": {
            "base_scenario": str(args.base_scenario),
            "distances_m": distances,
            "relative_yaws_deg": relative_yaws,
            "rolling_placements": [
                {
                    "longitudinal_m": longitudinal,
                    "lateral_m": lateral,
                    "relative_yaw_deg": relative_yaw,
                }
                for longitudinal, lateral, relative_yaw in placements
            ],
            "anchor_index": args.anchor_index,
            "sanitized_fields": [
                "tlList",
                "shadedAreaList",
                "frictionChangeMaterialList",
                "waypointDataList",
            ],
            "pair_order": ["CargoBox", "WoodBox"],
        },
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("generated {} paired scenes".format(len(scenes)))
    print("manifest={}".format(args.manifest))
    print("scenario_dir={}".format(args.output_dir))


if __name__ == "__main__":
    main()
