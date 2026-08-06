#!/usr/bin/env python3
"""Load MORAI scenarios and trigger verified multi-sensor captures over ROS.

The first validation mode keeps Ego stationary and proves ScenarioLoad plus
SaveSensorData.  The route-follow mode then moves Ego only a few metres along
the official competition waypoint path with a bounded low-speed controller.
Wall time is used internally because loading a scenario can reset simulation
time.
"""

import argparse
import json
import math
import os
import threading
import time
from pathlib import Path

import rospy
from morai_msgs.msg import (
    CollisionData,
    CtrlCmd,
    EgoVehicleStatus,
    ObjectStatusList,
    SaveSensorData,
    ScenarioLoad,
)
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest


def parse_args():
    project_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="MORAI ROS scenario load and sensor capture proof of concept."
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=project_root
        / "dataset/morai_od/manifests/ros_capture_poc.json",
    )
    parser.add_argument(
        "--capture-dir",
        type=Path,
        required=True,
        help=(
            "Absolute directory on the MORAI simulator host. Keep it shorter "
            "than 60 bytes for compatibility with the legacy transport."
        ),
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Scene id or file_name to run. Repeatable; omitted means all scenes.",
    )
    parser.add_argument(
        "--motion-mode",
        choices=("stationary", "route-follow"),
        default="stationary",
    )
    parser.add_argument("--captures-per-scene", type=int, default=None)
    parser.add_argument("--capture-spacing-m", type=float, default=None)
    parser.add_argument("--speed-kph", type=float, default=None)
    parser.add_argument("--maximum-motion-m", type=float, default=None)
    parser.add_argument("--load-timeout-sec", type=float, default=15.0)
    parser.add_argument("--load-settle-sec", type=float, default=2.0)
    parser.add_argument("--capture-timeout-sec", type=float, default=8.0)
    parser.add_argument("--motion-timeout-sec", type=float, default=15.0)
    parser.add_argument("--minimum-files-per-capture", type=int, default=2)
    parser.add_argument("--ego-position-tolerance-m", type=float, default=5.0)
    parser.add_argument("--route-deviation-limit-m", type=float, default=2.5)
    parser.add_argument("--heading-error-limit-deg", type=float, default=40.0)
    parser.add_argument("--lookahead-m", type=float, default=3.0)
    parser.add_argument("--wheelbase-m", type=float, default=3.0)
    parser.add_argument("--maximum-steering-rad", type=float, default=0.20)
    parser.add_argument("--scenario-topic", default="/ScenarioLoad")
    parser.add_argument("--capture-topic", default="/SaveSensorData")
    parser.add_argument("--control-topic", default="/ctrl_cmd")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--collision-topic", default="/CollisionData")
    parser.add_argument(
        "--control-mode-service",
        default="/Service_MoraiEventCmd",
        help="MORAI service used to select AV-ExternalCtrl after every load.",
    )
    parser.add_argument(
        "--skip-control-mode-service",
        action="store_true",
        help=(
            "Assume AV-ExternalCtrl was selected manually. This weakens "
            "multi-scene automation because ScenarioLoad can restore the "
            "scenario's saved control mode."
        ),
    )
    parser.add_argument(
        "--allow-unverified-capture",
        action="store_true",
        help=(
            "Do not fail when no new file appears under --capture-dir. "
            "Use only to diagnose MORAI versions that ignore file_dir."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate manifest, paths, naming, and policy without ROS publishing.",
    )
    return parser.parse_args()


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def load_waypoints(path):
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            stripped = line.strip()
            if stripped:
                values = stripped.split()
                if len(values) != 3:
                    raise ValueError("waypoint lines must contain x y z")
                points.append(tuple(float(value) for value in values))
    if len(points) < 3:
        raise ValueError("waypoint path must contain at least three points")
    return points


def distance_xy(left, right):
    return math.hypot(left[0] - right[0], left[1] - right[1])


def wrapped_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def recursive_files(root):
    return {
        path.resolve()
        for path in root.rglob("*")
        if path.is_file()
    }


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ego = None
        self.ego_wall_time = 0.0
        self.objects = None
        self.objects_wall_time = 0.0
        self.collision = None
        self.collision_wall_time = 0.0

    def ego_callback(self, message):
        with self.lock:
            self.ego = message
            self.ego_wall_time = time.monotonic()

    def object_callback(self, message):
        with self.lock:
            self.objects = message
            self.objects_wall_time = time.monotonic()

    def collision_callback(self, message):
        with self.lock:
            self.collision = message
            self.collision_wall_time = time.monotonic()

    def snapshot(self):
        with self.lock:
            return {
                "ego": self.ego,
                "ego_wall_time": self.ego_wall_time,
                "objects": self.objects,
                "objects_wall_time": self.objects_wall_time,
                "collision": self.collision,
                "collision_wall_time": self.collision_wall_time,
            }


class AutoCapture:
    def __init__(self, args, manifest, project_root):
        self.args = args
        self.manifest = manifest
        self.project_root = project_root
        waypoint_path = Path(manifest["waypoints"])
        if not waypoint_path.is_absolute():
            waypoint_path = project_root / waypoint_path
        self.waypoints = load_waypoints(waypoint_path)
        self.state = SharedState()
        self.run_id = time.strftime("r%m%d%H%M%S", time.localtime())

        rospy.init_node("morai_ros_scenario_autocapture_poc", anonymous=False)
        self.scenario_pub = rospy.Publisher(
            args.scenario_topic, ScenarioLoad, queue_size=1
        )
        self.capture_pub = rospy.Publisher(
            args.capture_topic, SaveSensorData, queue_size=10
        )
        self.control_pub = rospy.Publisher(
            args.control_topic, CtrlCmd, queue_size=10
        )
        self.control_mode_client = rospy.ServiceProxy(
            args.control_mode_service, MoraiEventCmdSrv
        )
        self.subscribers = [
            rospy.Subscriber(
                args.ego_topic,
                EgoVehicleStatus,
                self.state.ego_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                args.object_topic,
                ObjectStatusList,
                self.state.object_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                args.collision_topic,
                CollisionData,
                self.state.collision_callback,
                queue_size=10,
            ),
        ]

    def wait_for_connections(self):
        required = (
            ("ScenarioLoad", self.scenario_pub),
            ("SaveSensorData", self.capture_pub),
        )
        if self.args.motion_mode == "route-follow":
            required += (("CtrlCmd", self.control_pub),)
        deadline = time.monotonic() + 15.0
        missing = []
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            missing = [
                name
                for name, publisher in required
                if publisher.get_num_connections() == 0
            ]
            if not missing:
                return
            time.sleep(0.1)
        raise RuntimeError(
            "MORAI subscriber connection missing: " + ", ".join(missing)
        )

    def publish_scenario_load(self, scene, file_name):
        message = ScenarioLoad()
        message.file_name = file_name
        message.delete_all = True
        message.load_network_connection_data = False
        message.load_ego_vehicle_data = True
        message.load_surrounding_vehicle_data = True
        message.load_pedestrian_data = True
        if hasattr(message, "load_obstacle_data"):
            message.load_obstacle_data = True
        elif hasattr(message, "load_object_data"):
            message.load_object_data = True
        else:
            raise RuntimeError(
                "ScenarioLoad has neither load_obstacle_data nor load_object_data"
            )
        message.set_pause = False
        issued = time.monotonic()
        self.scenario_pub.publish(message)
        rospy.loginfo("ScenarioLoad published: %s", file_name)
        return issued

    def scene_is_ready(self, scene, issued):
        snapshot = self.state.snapshot()
        if (
            snapshot["ego"] is None
            or snapshot["objects"] is None
            or snapshot["ego_wall_time"] <= issued
            or snapshot["objects_wall_time"] <= issued
        ):
            return False, "waiting for fresh Ego/Object messages"

        ego = snapshot["ego"]
        expected_ego = scene["expected_ego"]
        ego_error = math.hypot(
            ego.position.x - expected_ego["x"],
            ego.position.y - expected_ego["y"],
        )
        if ego_error > self.args.ego_position_tolerance_m:
            return False, "Ego position error {:.2f} m".format(ego_error)

        objects = snapshot["objects"]
        expected = scene["expected_counts"]
        actual = {
            "npc": objects.num_of_npcs,
            "pedestrian": objects.num_of_pedestrian,
            "obstacle": objects.num_of_obstacle,
        }
        if actual != expected:
            return False, "object counts {} != {}".format(actual, expected)
        return True, "ready"

    def wait_for_scene(self, scene, issued):
        deadline = time.monotonic() + self.args.load_timeout_sec
        reason = "no data"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            ready, reason = self.scene_is_ready(scene, issued)
            if ready:
                time.sleep(self.args.load_settle_sec)
                rospy.loginfo(
                    "Scenario verified: %s (npc=%d, pedestrian=%d, obstacle=%d)",
                    scene["file_name"],
                    scene["expected_counts"]["npc"],
                    scene["expected_counts"]["pedestrian"],
                    scene["expected_counts"]["obstacle"],
                )
                return
            time.sleep(0.1)
        raise RuntimeError(
            "Scenario {} did not become ready: {}. {}".format(
                scene["file_name"],
                reason,
                self.scene_diagnostics(scene),
            )
        )

    def scene_diagnostics(self, scene):
        snapshot = self.state.snapshot()
        parts = []
        expected_ego = scene["expected_ego"]
        parts.append(
            "expected Ego=({:.3f}, {:.3f})".format(
                expected_ego["x"], expected_ego["y"]
            )
        )
        if snapshot["ego"] is None:
            parts.append("actual Ego=<not received>")
        else:
            ego = snapshot["ego"]
            parts.append(
                "actual Ego=({:.3f}, {:.3f})".format(
                    ego.position.x, ego.position.y
                )
            )
        if snapshot["objects"] is None:
            parts.append("actual objects=<not received>")
        else:
            objects = snapshot["objects"]
            parts.append(
                "actual objects=(npc={}, pedestrian={}, obstacle={})".format(
                    objects.num_of_npcs,
                    objects.num_of_pedestrian,
                    objects.num_of_obstacle,
                )
            )
        parts.append("expected objects={}".format(scene["expected_counts"]))
        return "; ".join(parts)

    def load_and_verify_scene(self, scene):
        candidates = [scene["file_name"]]
        if not scene["file_name"].endswith(".json"):
            candidates.append(scene["file_name"] + ".json")
        failures = []
        for candidate in candidates:
            issued = self.publish_scenario_load(scene, candidate)
            try:
                self.wait_for_scene(scene, issued)
                return
            except RuntimeError as error:
                failures.append("{}: {}".format(candidate, error))
                rospy.logwarn(
                    "Scenario load attempt failed for %s; trying next naming form",
                    candidate,
                )
        raise RuntimeError(
            "All ScenarioLoad filename forms failed:\n- "
            + "\n- ".join(failures)
        )

    def select_external_control_mode(self):
        if self.args.skip_control_mode_service:
            rospy.logwarn(
                "Control-mode service skipped; verify AV-ExternalCtrl in MORAI UI"
            )
            return
        try:
            rospy.wait_for_service(
                self.args.control_mode_service, timeout=5.0
            )
        except rospy.ROSException as error:
            raise RuntimeError(
                "{} is unavailable. Select AV-ExternalCtrl manually and "
                "rerun with --skip-control-mode-service, or enable the MORAI "
                "event service.".format(self.args.control_mode_service)
            ) from error

        request = MoraiEventCmdSrvRequest()
        request.request.option = 1
        request.request.ctrl_mode = 3
        request.request.gear = -1
        request.request.lamps.turnSignal = 3
        request.request.lamps.emergencySignal = 0
        request.request.set_pause = False
        response = self.control_mode_client(request)
        rospy.loginfo(
            "MORAI control mode response: ctrl_mode=%d",
            response.response.ctrl_mode,
        )
        if response.response.ctrl_mode not in (3, 4, 5):
            raise RuntimeError(
                "MORAI did not enter an external control mode: {}".format(
                    response.response.ctrl_mode
                )
            )

    def trigger_capture(self, scene, frame_index):
        capture_id = "{}_{}_f{:02d}".format(
            self.run_id, scene["id"], frame_index
        )
        if len(capture_id.encode("ascii")) > 29:
            raise ValueError("capture id exceeds MORAI legacy filename limit")

        before = recursive_files(self.args.capture_dir)
        message = SaveSensorData()
        message.is_custom_file_name = True
        message.custom_file_name = capture_id
        message.file_dir = str(self.args.capture_dir)
        self.capture_pub.publish(message)

        deadline = time.monotonic() + self.args.capture_timeout_sec
        new_files = set()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            new_files = recursive_files(self.args.capture_dir) - before
            if len(new_files) >= self.args.minimum_files_per_capture:
                time.sleep(0.3)
                new_files = recursive_files(self.args.capture_dir) - before
                break
            time.sleep(0.1)

        verified = len(new_files) >= self.args.minimum_files_per_capture
        if not verified and not self.args.allow_unverified_capture:
            raise RuntimeError(
                "Capture {} produced only {} new files under {}. Check "
                "Capture Mode, file_dir support, and simulator permissions.".format(
                    capture_id, len(new_files), self.args.capture_dir
                )
            )
        rospy.loginfo(
            "Capture %s: files=%d, verified=%s",
            capture_id,
            len(new_files),
            verified,
        )
        return {
            "capture_id": capture_id,
            "verified": verified,
            "new_file_count": len(new_files),
            "new_files": sorted(
                str(path.relative_to(self.args.capture_dir))
                for path in new_files
            ),
        }

    def publish_stop(self):
        message = CtrlCmd()
        message.longlCmdType = 1
        message.accel = 0.0
        message.brake = 1.0
        message.steering = 0.0
        message.velocity = 0.0
        message.acceleration = 0.0
        for _ in range(10):
            self.control_pub.publish(message)
            time.sleep(0.05)

    def nearest_waypoint_index(self, position, hint=None, window=120):
        if hint is None:
            candidates = range(len(self.waypoints))
        else:
            start = max(0, hint - 20)
            stop = min(len(self.waypoints), hint + window)
            candidates = range(start, stop)
        return min(
            candidates,
            key=lambda index: (
                self.waypoints[index][0] - position[0]
            )
            ** 2
            + (self.waypoints[index][1] - position[1]) ** 2,
        )

    def lookahead_index(self, nearest, distance):
        accumulated = 0.0
        index = nearest
        while index + 1 < len(self.waypoints) and accumulated < distance:
            accumulated += distance_xy(
                self.waypoints[index], self.waypoints[index + 1]
            )
            index += 1
        return index

    def real_collision_after(self, issued):
        snapshot = self.state.snapshot()
        if (
            snapshot["collision"] is None
            or snapshot["collision_wall_time"] <= issued
        ):
            return False
        candidates = list(snapshot["collision"].collision_object)
        candidates += list(snapshot["collision"].collision_objecta)
        return any(item.type != -1 for item in candidates)

    def drive_and_capture(self, scene, capture_count):
        captures = [self.trigger_capture(scene, 0)]
        if capture_count == 1:
            return captures

        snapshot = self.state.snapshot()
        ego = snapshot["ego"]
        start = (ego.position.x, ego.position.y)
        last_capture = start
        nearest_hint = scene["anchor_index"]
        issued = time.monotonic()
        deadline = issued + self.args.motion_timeout_sec

        try:
            while (
                not rospy.is_shutdown()
                and len(captures) < capture_count
                and time.monotonic() < deadline
            ):
                snapshot = self.state.snapshot()
                if snapshot["ego"] is None:
                    raise RuntimeError("Ego telemetry disappeared during motion")
                ego = snapshot["ego"]
                position = (ego.position.x, ego.position.y)
                travelled = distance_xy(start, position)
                if travelled > self.args.maximum_motion_m:
                    raise RuntimeError(
                        "maximum motion reached before all captures: {:.2f} m".format(
                            travelled
                        )
                    )
                if self.real_collision_after(issued):
                    raise RuntimeError("collision reported during PoC motion")

                nearest_hint = self.nearest_waypoint_index(
                    position, hint=nearest_hint
                )
                route_point = self.waypoints[nearest_hint]
                route_error = distance_xy(position, route_point)
                if route_error > self.args.route_deviation_limit_m:
                    raise RuntimeError(
                        "route deviation {:.2f} m exceeds safety limit".format(
                            route_error
                        )
                    )

                target_index = self.lookahead_index(
                    nearest_hint, self.args.lookahead_m
                )
                target = self.waypoints[target_index]
                target_heading = math.atan2(
                    target[1] - position[1], target[0] - position[0]
                )
                ego_heading = math.radians(ego.heading)
                heading_error = wrapped_angle(target_heading - ego_heading)
                if (
                    abs(math.degrees(heading_error))
                    > self.args.heading_error_limit_deg
                ):
                    raise RuntimeError(
                        "heading error {:.1f} deg exceeds safety limit".format(
                            math.degrees(heading_error)
                        )
                    )

                target_distance = max(0.5, distance_xy(position, target))
                curvature = 2.0 * math.sin(heading_error) / target_distance
                steering = math.atan(self.args.wheelbase_m * curvature)
                steering = max(
                    -self.args.maximum_steering_rad,
                    min(self.args.maximum_steering_rad, steering),
                )

                command = CtrlCmd()
                command.longlCmdType = 2
                command.velocity = self.args.speed_kph
                command.steering = steering
                command.accel = 0.0
                command.brake = 0.0
                command.acceleration = 0.0
                self.control_pub.publish(command)

                if (
                    distance_xy(last_capture, position)
                    >= self.args.capture_spacing_m
                ):
                    captures.append(
                        self.trigger_capture(scene, len(captures))
                    )
                    last_capture = position
                time.sleep(0.05)
        finally:
            self.publish_stop()

        if len(captures) != capture_count:
            raise RuntimeError(
                "motion ended with {}/{} captures".format(
                    len(captures), capture_count
                )
            )
        return captures

    def run(self, scenes):
        self.wait_for_connections()
        report = {
            "schema_version": 1,
            "started_unix": time.time(),
            "motion_mode": self.args.motion_mode,
            "capture_dir": str(self.args.capture_dir),
            "scenes": [],
        }
        for scene in scenes:
            self.load_and_verify_scene(scene)
            if self.args.motion_mode == "stationary":
                captures = [
                    self.trigger_capture(scene, index)
                    for index in range(self.args.captures_per_scene)
                ]
            else:
                self.select_external_control_mode()
                captures = self.drive_and_capture(
                    scene, self.args.captures_per_scene
                )
            report["scenes"].append(
                {
                    "id": scene["id"],
                    "file_name": scene["file_name"],
                    "captures": captures,
                }
            )
        report["finished_unix"] = time.time()
        return report


def select_scenes(manifest, requested):
    scenes = manifest["scenes"]
    if not requested:
        return scenes
    requested_set = set(requested)
    selected = [
        scene
        for scene in scenes
        if scene["id"] in requested_set
        or scene["file_name"] in requested_set
    ]
    found = {scene["id"] for scene in selected}
    found.update(scene["file_name"] for scene in selected)
    missing = requested_set - found
    if missing:
        raise ValueError("unknown scenario selectors: " + ", ".join(sorted(missing)))
    return selected


def validate_policy(args, manifest):
    policy = manifest["capture_policy"]
    if args.captures_per_scene is None:
        args.captures_per_scene = int(policy["captures_per_scene"])
    if args.capture_spacing_m is None:
        args.capture_spacing_m = float(policy["capture_spacing_m"])
    if args.speed_kph is None:
        args.speed_kph = float(policy["ego_target_speed_kph"])
    if args.maximum_motion_m is None:
        args.maximum_motion_m = float(policy["maximum_motion_m"])

    if args.captures_per_scene < 1:
        raise ValueError("--captures-per-scene must be >= 1")
    if args.capture_spacing_m <= 0.0:
        raise ValueError("--capture-spacing-m must be positive")
    if not 0.5 <= args.speed_kph <= 7.2:
        raise ValueError("--speed-kph must be within the PoC safety range 0.5..7.2")
    required_motion = (
        args.capture_spacing_m * max(0, args.captures_per_scene - 1)
    )
    if args.motion_mode == "route-follow" and required_motion > args.maximum_motion_m:
        raise ValueError(
            "capture policy requires {:.2f} m but maximum motion is {:.2f} m".format(
                required_motion, args.maximum_motion_m
            )
        )
    capture_dir_bytes = len(str(args.capture_dir).encode("utf-8"))
    if capture_dir_bytes > 59:
        raise ValueError(
            "--capture-dir is {} bytes; MORAI legacy transport allows 59".format(
                capture_dir_bytes
            )
        )


def main():
    args = parse_args()
    project_root = Path(__file__).resolve().parents[3]
    args.manifest = args.manifest.expanduser().resolve()
    args.capture_dir = args.capture_dir.expanduser().resolve()
    manifest = load_json(args.manifest)
    validate_policy(args, manifest)
    scenes = select_scenes(manifest, args.scenario)

    for scene in scenes:
        scenario_path = Path(scene["scenario"])
        if not scenario_path.is_absolute():
            scenario_path = project_root / scenario_path
        if not scenario_path.is_file():
            raise FileNotFoundError(scenario_path)
        if len(scene["file_name"].encode("ascii")) > 29:
            raise ValueError(
                "scenario filename stem exceeds MORAI legacy transport limit"
            )

    print("MORAI ROS auto-capture PoC")
    print("scenes:", ", ".join(scene["id"] for scene in scenes))
    print("motion mode:", args.motion_mode)
    print("captures per scene:", args.captures_per_scene)
    print("capture spacing m:", args.capture_spacing_m)
    print("capture dir:", args.capture_dir)
    if args.dry_run:
        print("dry-run validation: OK")
        return

    args.capture_dir.mkdir(parents=True, exist_ok=True)
    runner = AutoCapture(args, manifest, project_root)
    report = runner.run(scenes)
    report_path = args.capture_dir / "ros_capture_poc_report.json"
    temporary = report_path.with_suffix(".json.tmp")
    temporary.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(report_path))
    print("auto-capture complete")
    print("report:", report_path)


if __name__ == "__main__":
    main()
