#!/usr/bin/env python3
"""Load one installed dynamic scene and verify actor motion through MORAI GT."""

import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]

VEHICLE_FAMILIES = {
    "roundabout_sample_spawn",
    "roundabout_circulation",
    "highway_merge_sample_spawn",
    "highway_merge_cruise",
    "highway_cruise_overtake",
}

TRACK_SEGMENT_GAP_SEC = 0.5


def split_track_segments(values, maximum_gap_sec=TRACK_SEGMENT_GAP_SEC):
    """Separate recycled MORAI actor IDs after an observation gap."""

    if not values:
        return []
    segments = [[values[0]]]
    for value in values[1:]:
        if value[0] - segments[-1][-1][0] > maximum_gap_sec:
            segments.append([value])
        else:
            segments[-1].append(value)
    return segments


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--observe-sec", type=float, default=12.0)
    parser.add_argument("--load-timeout-sec", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--skip-scenario-load", action="store_true")
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def actor_values(message):
    for class_name, values in (
        ("vehicle", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    ):
        for actor in values:
            yield class_name, actor


def main():
    args = parse_args()
    try:
        import rospy
        from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList, ScenarioLoad
    except ImportError as error:
        raise SystemExit(f"source ROS Noetic and base_ws before running: {error}")

    manifest = load_json(args.manifest)
    scene = next((item for item in manifest["scenes"] if item["id"] == args.scene), None)
    if scene is None:
        raise SystemExit(f"unknown scene: {args.scene}")
    metadata_path = Path(scene["metadata"])
    if not metadata_path.is_absolute():
        metadata_path = PROJECT_ROOT / metadata_path
    scene_metadata = load_json(metadata_path)
    latest = {"ego": None, "objects": None, "object_wall": 0.0}
    tracks = defaultdict(list)

    def ego_callback(message):
        latest["ego"] = message

    def object_callback(message):
        latest["objects"] = message
        latest["object_wall"] = time.monotonic()
        now = message.header.stamp.to_sec() or rospy.Time.now().to_sec()
        for class_name, actor in actor_values(message):
            tracks[(class_name, int(actor.unique_id))].append(
                (
                    now,
                    float(actor.position.x),
                    float(actor.position.y),
                    math.sqrt(
                        actor.velocity.x * actor.velocity.x
                        + actor.velocity.y * actor.velocity.y
                        + actor.velocity.z * actor.velocity.z
                    ),
                )
            )

    rospy.init_node("verify_dynamic_scene_live", anonymous=False)
    rospy.Subscriber("/Ego_topic", EgoVehicleStatus, ego_callback, queue_size=20)
    rospy.Subscriber("/Object_topic", ObjectStatusList, object_callback, queue_size=50)
    publisher = rospy.Publisher("/ScenarioLoad", ScenarioLoad, queue_size=1)
    if not args.skip_scenario_load:
        deadline = time.monotonic() + args.load_timeout_sec
        while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.1)
        if publisher.get_num_connections() == 0:
            raise SystemExit("MORAI is not subscribed to /ScenarioLoad")
        request = ScenarioLoad()
        request.file_name = scene["file"]
        request.delete_all = False
        request.load_network_connection_data = True
        request.load_ego_vehicle_data = True
        request.load_surrounding_vehicle_data = True
        request.load_pedestrian_data = True
        if hasattr(request, "load_obstacle_data"):
            request.load_obstacle_data = True
        else:
            request.load_object_data = True
        request.set_pause = False
        issued = time.monotonic()
        publisher.publish(request)
        time.sleep(0.2)
        deadline = time.monotonic() + args.load_timeout_sec
        while time.monotonic() < deadline:
            if latest["object_wall"] > issued and latest["ego"] is not None:
                break
            time.sleep(0.1)
        else:
            raise SystemExit("fresh Ego/Object messages did not arrive after ScenarioLoad")

    start = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - start < args.observe_sec:
        time.sleep(0.1)

    actors = []
    for (class_name, unique_id), values in sorted(tracks.items()):
        for segment_index, segment in enumerate(split_track_segments(values)):
            first = segment[0]
            last = segment[-1]
            displacement = math.hypot(
                last[1] - first[1], last[2] - first[2]
            )
            actors.append(
                {
                    "class": class_name,
                    "unique_id": unique_id,
                    "segment_index": segment_index,
                    "sample_count": len(segment),
                    "duration_sec": last[0] - first[0],
                    "displacement_m": displacement,
                    "initial_morai_raw_speed": first[3],
                    "final_morai_raw_speed": last[3],
                    "minimum_morai_raw_speed": min(
                        item[3] for item in segment
                    ),
                    "maximum_morai_raw_speed": max(
                        item[3] for item in segment
                    ),
                    "endpoint_speed_change_raw": last[3] - first[3],
                }
            )

    family = scene["mission_family"]
    moving = [item for item in actors if item["displacement_m"] >= 0.5]
    if family in VEHICLE_FAMILIES:
        moving_vehicles = [
            item for item in moving if item["class"] == "vehicle"
        ]
        motion_phase = scene_metadata.get("motion_phase")
        if motion_phase == "spawn_acceleration":
            passed = any(
                item["maximum_morai_raw_speed"]
                >= item["initial_morai_raw_speed"] + 0.5
                for item in moving_vehicles
            )
            reason = "a spawned vehicle moved and accelerated by >= 0.5 raw speed units"
        elif motion_phase == "target_speed_start":
            passed = any(
                item["initial_morai_raw_speed"] >= 1.0
                for item in moving_vehicles
            )
            reason = "a spawned vehicle was already moving at first observation"
        else:
            passed = bool(moving_vehicles)
            reason = "at least one spawned vehicle moved >= 0.5 m"
    elif family == "crosswalk_pedestrian":
        passed = any(item["class"] == "pedestrian" for item in moving)
        reason = "the pedestrian moved >= 0.5 m"
    else:
        obstacles = [item for item in actors if item["class"] == "obstacle"]
        passed = bool(obstacles) and all(item["displacement_m"] < 0.2 for item in obstacles)
        reason = "the static obstacle remained within 0.2 m"
    report = {
        "scene": scene["id"],
        "scenario_file": scene["file"],
        "mission_family": family,
        "motion_phase": scene_metadata.get("motion_phase"),
        "sampling_role": scene_metadata.get("sampling_role"),
        "observe_sec": args.observe_sec,
        "status": "pass" if passed else "fail",
        "criterion": reason,
        "actors": actors,
    }
    output = args.output or (
        PROJECT_ROOT / "dataset/morai_od/reports" / f"{scene['id']}_live_motion.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"status: {report['status']}")
    print(f"criterion: {reason}")
    for actor in actors:
        print(
            "{class} id={unique_id} segment={segment_index}: "
            "displacement={displacement_m:.3f}m, "
            "raw_speed={initial_morai_raw_speed:.3f}->"
            "{final_morai_raw_speed:.3f}, "
            "max={maximum_morai_raw_speed:.3f}".format(**actor)
        )
    print(f"report: {output}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
