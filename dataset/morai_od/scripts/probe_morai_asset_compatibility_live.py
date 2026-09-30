#!/usr/bin/env python3
"""Reload interleaved MORAI asset scenes and verify Object_topic contents."""

import argparse
import json
import time
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--load-timeout-sec", type=float, default=5.0)
    parser.add_argument("--settle-sec", type=float, default=0.6)
    parser.add_argument("--attempts", type=int, default=2)
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=0.0,
        help="after a successful load, repeatedly publish a scene marker for rosbag capture",
    )
    parser.add_argument(
        "--scene",
        action="append",
        default=[],
        help="scene id to probe; repeatable, all scenes when omitted",
    )
    return parser.parse_args()


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def actor_rows(message):
    rows = {"npc": [], "pedestrian": [], "obstacle": []}
    for key, values in (
        ("npc", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    ):
        for actor in values:
            rows[key].append(
                {
                    "unique_id": int(actor.unique_id),
                    "name": str(actor.name),
                    "x": float(actor.position.x),
                    "y": float(actor.position.y),
                    "z": float(actor.position.z),
                }
            )
    return rows


def counts(rows):
    return {key: len(values) for key, values in rows.items()}


def main():
    args = parse_args()
    if args.attempts < 1:
        raise ValueError("--attempts must be positive")

    import rospy
    from morai_msgs.msg import ObjectStatusList, ScenarioLoad
    from std_msgs.msg import String

    manifest = load_json(args.manifest.resolve())
    scenes = manifest["scenes"]
    if args.scene:
        selected = set(args.scene)
        scenes = [scene for scene in scenes if scene["id"] in selected]
        missing = selected - {scene["id"] for scene in scenes}
        if missing:
            raise ValueError("unknown scene ids: {}".format(sorted(missing)))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    latest = {"message": None, "wall": 0.0}

    def callback(message):
        latest["message"] = message
        latest["wall"] = time.monotonic()

    rospy.init_node("probe_morai_asset_compatibility_live", anonymous=True)
    rospy.Subscriber("/Object_topic", ObjectStatusList, callback, queue_size=20)
    publisher = rospy.Publisher("/ScenarioLoad", ScenarioLoad, queue_size=1)
    marker_publisher = rospy.Publisher("/dataset/scenario_marker", String, queue_size=10)
    deadline = time.monotonic() + 10.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.1)
    if publisher.get_num_connections() == 0:
        raise RuntimeError("MORAI is not subscribed to /ScenarioLoad")

    completed = []
    failed = []
    observations = []
    failure_rows = []
    started = time.time()

    for index, scene in enumerate(scenes, 1):
        expected = {
            key: int(scene["expected_counts"].get(key, 0))
            for key in ("npc", "pedestrian", "obstacle")
        }
        best_rows = {"npc": [], "pedestrian": [], "obstacle": []}
        passed = False
        attempt_records = []

        for attempt in range(1, args.attempts + 1):
            request = ScenarioLoad()
            request.file_name = scene["file_name"]
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
            deadline = issued + args.load_timeout_sec
            matched_since = None
            last_rows = best_rows
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if latest["wall"] <= issued or latest["message"] is None:
                    time.sleep(0.05)
                    continue
                last_rows = actor_rows(latest["message"])
                if counts(last_rows) == expected:
                    if matched_since is None:
                        matched_since = time.monotonic()
                    if time.monotonic() - matched_since >= args.settle_sec:
                        passed = True
                        break
                else:
                    matched_since = None
                time.sleep(0.05)
            best_rows = last_rows
            attempt_records.append(
                {
                    "attempt": attempt,
                    "expected_counts": expected,
                    "observed_counts": counts(best_rows),
                    "actors": best_rows,
                    "passed": passed,
                }
            )
            if passed:
                break

        scene_id = scene["id"]
        row = {
            "id": scene_id,
            "file_name": scene["file_name"],
            "asset_probe": scene["asset_probe"],
            "expected_counts": expected,
            "observed_counts": counts(best_rows),
            "actors": best_rows,
            "attempts": attempt_records,
            "status": "completed" if passed else "failed",
        }
        if passed and args.hold_sec > 0:
            marker_base = {
                "scene_id": scene_id,
                "file_name": scene["file_name"],
                "asset_probe": scene["asset_probe"],
                "mission_family": "woodbox_alignment",
            }
            marker = String()
            marker.data = json.dumps(
                {**marker_base, "event": "capture_start"}, ensure_ascii=False
            )
            marker_publisher.publish(marker)
            hold_deadline = time.monotonic() + args.hold_sec
            while not rospy.is_shutdown() and time.monotonic() < hold_deadline:
                time.sleep(0.1)
            marker.data = json.dumps(
                {**marker_base, "event": "capture_end", "status": "completed"},
                ensure_ascii=False,
            )
            marker_publisher.publish(marker)
        observations.append(row)
        if passed:
            completed.append(scene_id)
        else:
            failed.append(scene_id)
            failure_rows.append(
                {
                    "id": scene_id,
                    "error": "expected counts {} but observed {}".format(
                        expected, counts(best_rows)
                    ),
                }
            )
        print(
            "[{}/{}] {} {} expected={} observed={}".format(
                index,
                len(scenes),
                scene_id,
                "PASS" if passed else "FAIL",
                expected,
                counts(best_rows),
            ),
            flush=True,
        )

    summary = {
        "schema_version": 1,
        "manifest": str(args.manifest.resolve()),
        "started_unix": started,
        "finished_unix": time.time(),
        "requested_scenes": len(scenes),
        "attempted_scenes": len(observations),
        "completed_scenes": len(completed),
        "failed_scenes": len(failed),
        "completed_scene_ids": completed,
        "failed_scene_ids": failed,
        "finished": True,
        "interrupted": False,
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    with (args.output_dir / "scenes.jsonl").open("w", encoding="utf-8") as stream:
        for row in observations:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")
    with (args.output_dir / "failures.jsonl").open("w", encoding="utf-8") as stream:
        for row in failure_rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
