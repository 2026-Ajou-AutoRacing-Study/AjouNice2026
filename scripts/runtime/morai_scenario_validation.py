#!/usr/bin/env python3
"""Run generated MORAI scenarios through the AjouNice ROS1 stack.

The default mode is a dry run.  Pass ``--execute`` only after MORAI is open,
its ROS ScenarioLoad provider is connected, and no other AJOUNICE stack is
using the host ROS master/UDP ports.
"""

import argparse
import json
import math
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple
from urllib.parse import urlparse


DEFAULT_SCENARIO_DIR = Path("/morai_scenarios")
DEFAULT_OUTPUT_DIR = Path(
    "/home/ajou/AjouNice2026/artifacts/morai_scenario_validation"
)
DEFAULT_HISTORY_DIR = Path(
    "/home/ajou/AjouNice2026/base_ws/src/control/trajectory_validator/history"
)
DEFAULT_TRAJECTORY_REFERENCE = "traj_sample/variable/200889_200889_v.csv"
DRIVABLE_PATH_TOPIC = (
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id"
)
DEFAULT_TOPICS = (
    "/Ego_topic",
    "/Object_topic",
    "/CollisionData",
    "/localization/kinematic_state",
    "/planning/mission_planning/route",
    DRIVABLE_PATH_TOPIC,
    "/control/trajectory",
    "/ctrl_cmd",
    "/purepursuit_autoware/debug/controller_state",
    "/trajectory_validator/debug",
    "/tf",
    "/tf_static",
)

SAFE_COMPONENT = re.compile(r"[^0-9A-Za-z._-]+")
POSITION_SUFFIX = re.compile(r"_pos(\d+)$", re.IGNORECASE)


class ValidationError(RuntimeError):
    """Raised when the validation cannot continue safely."""


def shell_join(command: Sequence[str]) -> str:
    return " ".join(shlex.quote(str(item)) for item in command)


def safe_component(value: str) -> str:
    cleaned = SAFE_COMPONENT.sub("_", value).strip("._")
    if not cleaned:
        raise ValidationError("value cannot be represented as a safe path component")
    return cleaned


def natural_scenario_key(path: Path) -> Tuple[str, int, str]:
    match = POSITION_SUFFIX.search(path.stem)
    if match:
        return (path.stem[: match.start()].casefold(), int(match.group(1)), path.name)
    return (path.stem.casefold(), sys.maxsize, path.name)


def point_in_polygon(
    point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]
) -> bool:
    """Return whether a 2D point is inside or on a polygon boundary."""
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy
        if length_sq > 1.0e-12:
            ratio = ((x - x1) * dx + (y - y1) * dy) / length_sq
            if -1.0e-9 <= ratio <= 1.0 + 1.0e-9:
                projected_x = x1 + ratio * dx
                projected_y = y1 + ratio * dy
                if math.hypot(x - projected_x, y - projected_y) <= 1.0e-7:
                    return True
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def atomic_json(path: Path, document: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def discover_scenarios(
    scenario_dir: Path, pattern: str, requested: Sequence[str]
) -> List[Path]:
    scenario_dir = scenario_dir.resolve()
    if not scenario_dir.is_dir():
        raise ValidationError("scenario directory does not exist: {}".format(scenario_dir))

    if requested:
        scenarios = []
        for name in requested:
            candidate = (scenario_dir / name).resolve()
            try:
                candidate.relative_to(scenario_dir)
            except ValueError as error:
                raise ValidationError(
                    "scenario must stay inside scenario directory: {}".format(name)
                ) from error
            scenarios.append(candidate)
    else:
        scenarios = sorted(scenario_dir.glob(pattern), key=natural_scenario_key)

    if not scenarios:
        raise ValidationError(
            "no scenarios matched {!r} in {}".format(pattern, scenario_dir)
        )

    validated = []
    for scenario in scenarios:
        if not scenario.is_file():
            raise ValidationError("scenario file does not exist: {}".format(scenario))
        try:
            with scenario.open("r", encoding="utf-8-sig") as stream:
                document = json.load(stream)
        except (OSError, json.JSONDecodeError) as error:
            raise ValidationError(
                "invalid scenario JSON {}: {}".format(scenario, error)
            ) from error
        if not isinstance(document, dict) or not isinstance(
            document.get("objectList"), list
        ):
            raise ValidationError(
                "scenario has no objectList: {}".format(scenario)
            )
        validated.append(scenario)
    return validated


def master_endpoint(uri: str) -> Tuple[str, int]:
    parsed = urlparse(uri)
    return parsed.hostname or "127.0.0.1", parsed.port or 11311


def tcp_port_open(host: str, port: int) -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.5):
            return True
    except OSError:
        return False


def require_commands(names: Iterable[str]) -> None:
    missing = [name for name in names if shutil.which(name) is None]
    if missing:
        raise ValidationError(
            "required ROS commands are unavailable: {}".format(", ".join(missing))
        )


@dataclass
class ManagedProcess:
    name: str
    command: List[str]
    log_path: Path
    process: Optional[subprocess.Popen] = None
    _log_stream: Any = field(default=None, init=False, repr=False)

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = self.log_path.open("ab", buffering=0)
        print("[start] {}: {}".format(self.name, shell_join(self.command)))
        try:
            self.process = subprocess.Popen(
                self.command,
                stdin=subprocess.DEVNULL,
                stdout=self._log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            self._close_log()
            raise

    def poll(self) -> Optional[int]:
        if self.process is None:
            return None
        return self.process.poll()

    def assert_running(self) -> None:
        if self.process is None:
            raise ValidationError("process was not started: {}".format(self.name))
        return_code = self.process.poll()
        if return_code is not None:
            raise ValidationError(
                "{} exited unexpectedly with code {}; see {}".format(
                    self.name, return_code, self.log_path
                )
            )

    def stop(self, interrupt_timeout: float = 10.0) -> None:
        process = self.process
        if process is None:
            self._close_log()
            return
        if process.poll() is None:
            print("[stop] {}".format(self.name))
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=interrupt_timeout)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5.0)
        self._close_log()

    def _close_log(self) -> None:
        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None


def stop_processes(processes: Iterable[ManagedProcess]) -> None:
    for process in reversed(list(processes)):
        try:
            process.stop()
        except Exception as error:  # Cleanup must continue for the remaining nodes.
            print("[cleanup warning] {}: {}".format(process.name, error), file=sys.stderr)


def run_checked(command: Sequence[str], timeout: float = 15.0) -> str:
    completed = subprocess.run(
        list(command),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )
    if completed.returncode != 0:
        raise ValidationError(
            "command failed ({}): {}\n{}".format(
                completed.returncode, shell_join(command), completed.stdout.strip()
            )
        )
    return completed.stdout


def wait_for_master(process: ManagedProcess, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.assert_running()
        try:
            run_checked(("rosnode", "list"), timeout=2.0)
            return
        except (ValidationError, subprocess.TimeoutExpired):
            time.sleep(0.25)
    raise ValidationError("ROS master did not become ready within {:.1f}s".format(timeout))


def wait_for_node(node_name: str, process: ManagedProcess, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.assert_running()
        try:
            nodes = run_checked(("rosnode", "list"), timeout=2.0).splitlines()
            if node_name in nodes:
                return
        except (ValidationError, subprocess.TimeoutExpired):
            pass
        time.sleep(0.25)
    raise ValidationError(
        "ROS node {} did not become ready within {:.1f}s".format(node_name, timeout)
    )


def metadata_snapshot(history_root: Path) -> Set[Path]:
    if not history_root.is_dir():
        return set()
    return {path.resolve() for path in history_root.glob("*/*/metadata.json")}


def newest_new_metadata(history_root: Path, before: Set[Path]) -> Optional[Path]:
    candidates = metadata_snapshot(history_root) - before
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def load_validator_result(metadata_path: Optional[Path]) -> Dict[str, Any]:
    if metadata_path is None:
        return {}
    try:
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        return {"metadata_error": str(error)}
    return {
        "metadata_file": str(metadata_path),
        "status": metadata.get("status"),
        "recording": metadata.get("recording"),
        "result": metadata.get("result"),
    }


def process_commands(args: argparse.Namespace) -> Dict[str, List[str]]:
    return {
        "roscore": ["roscore"],
        "udp": ["roslaunch", "sensor_interface", "morai_udp_interface.launch"],
        "sensing": [
            "roslaunch",
            "bringup",
            "sensing.launch",
            "start_rviz:=false",
            "enable_vehicle_model:=false",
        ],
        "localization": [
            "roslaunch",
            "bringup",
            "localization_morai_gt.launch",
            "start_lanelet_map:=true",
        ],
        "planner": [
            "roslaunch",
            "bringup",
            "kcity_morai_competition_planner_rviz.launch",
            "launch_rviz:=false",
            "start_route_publisher:=false",
            "start_map_visualizer:=false",
            "enable_planner_csv_logger:=false",
            "enable_planner_bag_recorder:=false",
            "default_velocity_kph:={}".format(args.velocity_kph),
        ],
        # kroute is deliberately separate.  The planner's built-in route
        # publisher is disabled above to avoid a duplicate node/service call.
        "kroute": [
            "roslaunch",
            "bringup",
            "kcity_competition_route.launch",
        ],
        "validator": [
            "roslaunch",
            "trajectory_validator",
            "record.launch",
        ],
        "follower": [
            "roslaunch",
            "trajectory_follower",
            "pp_pi_autoware.launch",
        ],
    }


def print_dry_run(
    args: argparse.Namespace, scenarios: Sequence[Path], commands: Dict[str, List[str]]
) -> None:
    print("DRY RUN: no MORAI/ROS process will be started.")
    print("Selected scenarios ({}):".format(len(scenarios)))
    for scenario in scenarios:
        print("  - {}".format(scenario.name))
    print("\nShared stack:")
    for name in ("roscore", "udp", "sensing", "localization", "planner"):
        print("  {}: {}".format(name, shell_join(commands[name])))
    print("\nPer scenario:")
    print("  1. call {} with the scenario filename".format(args.scenario_service))
    print("  2. {}".format(shell_join(commands["kroute"])))
    print("  3. rosbag record -> <session>/<scenario>/topics.bag")
    print("  4. {}".format(shell_join(commands["validator"])))
    print("  5. {}  # starts vehicle control last".format(shell_join(commands["follower"])))
    print(
        "  6. stop within {:.2f}m of MORAI ENU ({:.2f}, {:.2f}), "
        "collision={}, or {:.1f}s timeout; write summary.json".format(
            args.goal_radius,
            args.goal_x,
            args.goal_y,
            args.stop_on_collision,
            args.scenario_timeout,
        )
    )
    print(
        "     fail if speed <= {:.2f}m/s for {:.1f}s, or outside BPP "
        "drivable bounds for {:.1f}s".format(
            args.stopped_speed_threshold,
            args.stopped_duration,
            args.lane_departure_duration,
        )
    )
    print("\nPass --execute to perform this plan.")


def publish_stop_command() -> None:
    payload = (
        "{longlCmdType: 1, accel: 0.0, brake: 1.0, steering: 0.0, "
        "velocity: 0.0, acceleration: 0.0}"
    )
    try:
        run_checked(
            ("rostopic", "pub", "-1", "/ctrl_cmd", "morai_msgs/CtrlCmd", payload),
            timeout=5.0,
        )
    except (ValidationError, subprocess.TimeoutExpired) as error:
        print("[warning] failed to publish final brake command: {}".format(error), file=sys.stderr)


def start_process(
    name: str, command: List[str], log_dir: Path, collection: List[ManagedProcess]
) -> ManagedProcess:
    process = ManagedProcess(name, command, log_dir / (name + ".log"))
    process.start()
    collection.append(process)
    return process


def assert_critical_processes(processes: Iterable[ManagedProcess]) -> None:
    for process in processes:
        process.assert_running()


def execute(args: argparse.Namespace, scenarios: Sequence[Path]) -> int:
    require_commands(
        ("roscore", "roslaunch", "rosnode", "rostopic", "rosbag", "rosparam")
    )
    master_uri = os.environ.get("ROS_MASTER_URI", "http://127.0.0.1:11311")
    master_host, master_port = master_endpoint(master_uri)
    if tcp_port_open(master_host, master_port):
        raise ValidationError(
            "ROS master {}:{} is already active. Stop the existing AJOUNICE "
            "containers/nodes before an automated run.".format(master_host, master_port)
        )

    session_id = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    session_dir = args.output_dir.resolve() / session_id
    session_dir.mkdir(parents=True, exist_ok=False)
    shared_logs = session_dir / "shared_logs"
    commands = process_commands(args)
    shared: List[ManagedProcess] = []
    session_results: List[Dict[str, Any]] = []
    interrupted = threading.Event()

    def handle_signal(signum: int, _frame: Any) -> None:
        print("received signal {}; stopping after cleanup".format(signum), file=sys.stderr)
        interrupted.set()

    previous_sigint = signal.signal(signal.SIGINT, handle_signal)
    previous_sigterm = signal.signal(signal.SIGTERM, handle_signal)

    rospy = None
    try:
        core = start_process("roscore", commands["roscore"], shared_logs, shared)
        wait_for_master(core, args.startup_timeout)

        # ROS imports are intentionally delayed so dry-run works on the Ubuntu
        # 22.04 host where only ROS2 is installed.
        import rospy as rospy_module
        from autoware_internal_planning_msgs.msg import PathWithLaneId
        from autoware_map_msgs.msg import LaneletMapBin
        from autoware_planning_msgs.msg import LaneletRoute, Trajectory
        from morai_msgs.msg import (
            CollisionData,
            CtrlCmd,
            EgoVehicleStatus,
            ObjectStatusList,
        )
        from morai_msgs.srv import MoraiScenarioLoadSrv, MoraiScenarioLoadSrvRequest
        from nav_msgs.msg import Odometry

        rospy = rospy_module
        rospy.init_node(
            "morai_scenario_validation_runner", anonymous=True, disable_signals=True
        )

        collision_event = threading.Event()
        goal_event = threading.Event()
        stopped_event = threading.Event()
        lane_departure_event = threading.Event()
        monitor_lock = threading.Lock()
        goal_state: Dict[str, Any] = {
            "active": False,
            "last_x": None,
            "last_y": None,
            "last_distance_m": None,
            "minimum_distance_m": None,
            "reached_at": None,
        }
        stopped_state: Dict[str, Any] = {
            "active": False,
            "last_speed_mps": None,
            "stopped_since_monotonic": None,
            "continuous_stopped_s": 0.0,
            "maximum_continuous_stopped_s": 0.0,
            "failed_at": None,
        }
        lane_state: Dict[str, Any] = {
            "active": False,
            "polygon": None,
            "last_x": None,
            "last_y": None,
            "last_inside": None,
            "outside_since_monotonic": None,
            "continuous_outside_s": 0.0,
            "maximum_continuous_outside_s": 0.0,
            "failed_at": None,
        }

        def collision_callback(message: Any) -> None:
            objects = list(message.collision_object) + list(message.collision_objecta)
            if any(int(getattr(item, "type", -1)) >= 0 for item in objects):
                collision_event.set()

        def ego_goal_callback(message: Any) -> None:
            x = float(message.position.x)
            y = float(message.position.y)
            distance = math.hypot(x - args.goal_x, y - args.goal_y)
            speed = math.sqrt(
                float(message.velocity.x) ** 2
                + float(message.velocity.y) ** 2
                + float(message.velocity.z) ** 2
            )
            now_monotonic = time.monotonic()
            now_iso = datetime.now().astimezone().isoformat()
            with monitor_lock:
                if goal_state["active"]:
                    goal_state["last_x"] = x
                    goal_state["last_y"] = y
                    goal_state["last_distance_m"] = distance
                    minimum = goal_state["minimum_distance_m"]
                    if minimum is None or distance < minimum:
                        goal_state["minimum_distance_m"] = distance
                    if distance <= args.goal_radius:
                        if goal_state["reached_at"] is None:
                            goal_state["reached_at"] = now_iso
                        goal_event.set()

                stopped_state["last_speed_mps"] = speed
                if stopped_state["active"]:
                    if speed <= args.stopped_speed_threshold:
                        if stopped_state["stopped_since_monotonic"] is None:
                            stopped_state["stopped_since_monotonic"] = now_monotonic
                        duration = (
                            now_monotonic
                            - stopped_state["stopped_since_monotonic"]
                        )
                        stopped_state["continuous_stopped_s"] = duration
                        stopped_state["maximum_continuous_stopped_s"] = max(
                            stopped_state["maximum_continuous_stopped_s"], duration
                        )
                        if duration >= args.stopped_duration:
                            if stopped_state["failed_at"] is None:
                                stopped_state["failed_at"] = now_iso
                            stopped_event.set()
                    else:
                        stopped_state["stopped_since_monotonic"] = None
                        stopped_state["continuous_stopped_s"] = 0.0

        def drivable_path_callback(message: Any) -> None:
            left = [(float(item.x), float(item.y)) for item in message.left_bound]
            right = [(float(item.x), float(item.y)) for item in message.right_bound]
            polygon = (
                left + list(reversed(right))
                if len(left) >= 2 and len(right) >= 2
                else None
            )
            with monitor_lock:
                lane_state["polygon"] = polygon

        def lane_odometry_callback(message: Any) -> None:
            x = float(message.pose.pose.position.x)
            y = float(message.pose.pose.position.y)
            now_monotonic = time.monotonic()
            now_iso = datetime.now().astimezone().isoformat()
            with monitor_lock:
                polygon = lane_state["polygon"]
                if not lane_state["active"] or not polygon:
                    return
                inside = point_in_polygon((x, y), polygon)
                lane_state["last_x"] = x
                lane_state["last_y"] = y
                lane_state["last_inside"] = inside
                if inside:
                    lane_state["outside_since_monotonic"] = None
                    lane_state["continuous_outside_s"] = 0.0
                    return
                if lane_state["outside_since_monotonic"] is None:
                    lane_state["outside_since_monotonic"] = now_monotonic
                duration = now_monotonic - lane_state["outside_since_monotonic"]
                lane_state["continuous_outside_s"] = duration
                lane_state["maximum_continuous_outside_s"] = max(
                    lane_state["maximum_continuous_outside_s"], duration
                )
                if duration >= args.lane_departure_duration:
                    if lane_state["failed_at"] is None:
                        lane_state["failed_at"] = now_iso
                    lane_departure_event.set()

        collision_subscriber = rospy.Subscriber(
            "/CollisionData", CollisionData, collision_callback, queue_size=10
        )
        ego_goal_subscriber = rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, ego_goal_callback, queue_size=20
        )
        drivable_path_subscriber = rospy.Subscriber(
            DRIVABLE_PATH_TOPIC,
            PathWithLaneId,
            drivable_path_callback,
            queue_size=2,
        )
        lane_odometry_subscriber = rospy.Subscriber(
            "/localization/kinematic_state",
            Odometry,
            lane_odometry_callback,
            queue_size=20,
        )

        udp = start_process("udp", commands["udp"], shared_logs, shared)
        sensing = start_process("sensing", commands["sensing"], shared_logs, shared)
        localization = start_process(
            "localization", commands["localization"], shared_logs, shared
        )

        rospy.wait_for_message(
            "/map/vector_map", LaneletMapBin, timeout=args.startup_timeout
        )
        assert_critical_processes((core, udp, sensing, localization))

        planner = start_process("planner", commands["planner"], shared_logs, shared)
        rospy.wait_for_service(
            "/planning/mission_planning/mission_planner/set_lanelet_route",
            timeout=args.startup_timeout,
        )
        rospy.wait_for_service(args.scenario_service, timeout=args.startup_timeout)
        scenario_loader = rospy.ServiceProxy(
            args.scenario_service, MoraiScenarioLoadSrv, persistent=False
        )

        critical_shared = (core, udp, sensing, localization, planner)
        rospy.set_param(
            "/trajectory_publisher/trajectory_csv", args.trajectory_reference
        )

        for index, scenario in enumerate(scenarios, start=1):
            if interrupted.is_set():
                break
            print(
                "\n=== Scenario {}/{}: {} ===".format(index, len(scenarios), scenario.name)
            )
            scenario_started = datetime.now().astimezone()
            scenario_dir = session_dir / safe_component(scenario.stem)
            scenario_dir.mkdir(parents=True, exist_ok=False)
            logs = scenario_dir / "logs"
            per_scenario: List[ManagedProcess] = []
            history_before = metadata_snapshot(args.history_dir)
            end_reason = "error"
            error_message = None
            validator_finished_before_goal = False
            validator_return_code = None
            collision_event.clear()
            goal_event.clear()
            stopped_event.clear()
            lane_departure_event.clear()
            with monitor_lock:
                goal_state.update(
                    {
                        "active": False,
                        "last_x": None,
                        "last_y": None,
                        "last_distance_m": None,
                        "minimum_distance_m": None,
                        "reached_at": None,
                    }
                )
                stopped_state.update(
                    {
                        "active": False,
                        "last_speed_mps": None,
                        "stopped_since_monotonic": None,
                        "continuous_stopped_s": 0.0,
                        "maximum_continuous_stopped_s": 0.0,
                        "failed_at": None,
                    }
                )
                lane_state.update(
                    {
                        "active": False,
                        "polygon": None,
                        "last_x": None,
                        "last_y": None,
                        "last_inside": None,
                        "outside_since_monotonic": None,
                        "continuous_outside_s": 0.0,
                        "maximum_continuous_outside_s": 0.0,
                        "failed_at": None,
                    }
                )

            try:
                assert_critical_processes(critical_shared)

                request = MoraiScenarioLoadSrvRequest()
                request.request.file_name = scenario.name
                request.request.load_network_connection_data = False
                request.request.delete_all = True
                request.request.load_ego_vehicle_data = True
                request.request.load_surrounding_vehicle_data = True
                request.request.load_pedestrian_data = True
                request.request.load_obstacle_data = True
                request.request.set_pause = False
                response = scenario_loader(request)
                if not bool(response.response.result):
                    raise ValidationError(
                        "MORAI rejected scenario load: {}".format(scenario.name)
                    )

                with monitor_lock:
                    goal_state["active"] = True
                first_ego_message = rospy.wait_for_message(
                    "/Ego_topic", EgoVehicleStatus, timeout=args.startup_timeout
                )
                ego_goal_callback(first_ego_message)
                object_message = rospy.wait_for_message(
                    "/Object_topic", ObjectStatusList, timeout=args.startup_timeout
                )
                if int(object_message.num_of_obstacle) <= 0:
                    raise ValidationError(
                        "loaded scenario reports no MORAI obstacles: {}".format(
                            scenario.name
                        )
                    )
                rospy.wait_for_message(
                    "/localization/kinematic_state",
                    Odometry,
                    timeout=args.startup_timeout,
                )
                time.sleep(args.settle_seconds)

                bag_prefix = scenario_dir / "topics"
                if not args.no_rosbag:
                    bag_command = [
                        "rosbag",
                        "record",
                        "--lz4",
                        "-O",
                        str(bag_prefix),
                    ] + list(args.topic)
                    start_process("rosbag", bag_command, logs, per_scenario)

                kroute = start_process(
                    "kroute", commands["kroute"], logs, per_scenario
                )
                wait_for_node(
                    "/kcity_competition_live_route_publisher",
                    kroute,
                    args.startup_timeout,
                )
                rospy.wait_for_message(
                    "/planning/mission_planning/route",
                    LaneletRoute,
                    timeout=args.startup_timeout,
                )
                time.sleep(args.route_settle_seconds)
                kroute.assert_running()
                rospy.wait_for_message(
                    "/control/trajectory", Trajectory, timeout=args.startup_timeout
                )
                first_drivable_path = rospy.wait_for_message(
                    DRIVABLE_PATH_TOPIC,
                    PathWithLaneId,
                    timeout=args.startup_timeout,
                )
                drivable_path_callback(first_drivable_path)

                validator = start_process(
                    "validator", commands["validator"], logs, per_scenario
                )
                wait_for_node(
                    "/trajectory_validator_recorder",
                    validator,
                    args.startup_timeout,
                )

                follower = start_process(
                    "follower", commands["follower"], logs, per_scenario
                )
                wait_for_node(
                    "/purepursuit_autoware", follower, args.startup_timeout
                )
                rospy.wait_for_message(
                    "/ctrl_cmd", CtrlCmd, timeout=args.startup_timeout
                )
                with monitor_lock:
                    stopped_state["active"] = True
                    if (
                        stopped_state["last_speed_mps"] is not None
                        and stopped_state["last_speed_mps"]
                        <= args.stopped_speed_threshold
                    ):
                        stopped_state["stopped_since_monotonic"] = time.monotonic()
                    lane_state["active"] = True

                deadline = time.monotonic() + args.scenario_timeout
                while time.monotonic() < deadline:
                    if interrupted.is_set():
                        end_reason = "interrupted"
                        break
                    assert_critical_processes(critical_shared)
                    follower.assert_running()
                    kroute.assert_running()
                    if collision_event.is_set() and args.stop_on_collision:
                        end_reason = "collision"
                        if args.post_collision_seconds > 0.0:
                            time.sleep(args.post_collision_seconds)
                        break
                    if lane_departure_event.is_set():
                        end_reason = "lane_departure"
                        break
                    if stopped_event.is_set():
                        end_reason = "stopped_too_long"
                        break
                    if goal_event.is_set():
                        end_reason = "goal_reached"
                        break
                    current_validator_code = validator.poll()
                    if (
                        current_validator_code is not None
                        and not validator_finished_before_goal
                    ):
                        validator_return_code = current_validator_code
                        if current_validator_code != 0:
                            raise ValidationError(
                                "trajectory validator exited with code {} before "
                                "the fixed MORAI ENU goal".format(
                                    current_validator_code
                                )
                            )
                        validator_finished_before_goal = True
                        print(
                            "[warning] trajectory validator finished before the "
                            "fixed MORAI ENU goal; rosbag recording and vehicle "
                            "control will continue"
                        )
                    time.sleep(0.25)
                else:
                    end_reason = "timeout"

            except Exception as error:
                error_message = "{}: {}".format(type(error).__name__, error)
                print("[scenario error] {}".format(error_message), file=sys.stderr)
            finally:
                with monitor_lock:
                    goal_state["active"] = False
                    stopped_state["active"] = False
                    lane_state["active"] = False
                follower_process = next(
                    (item for item in per_scenario if item.name == "follower"), None
                )
                if follower_process is not None:
                    follower_process.stop()
                publish_stop_command()
                remaining = [
                    item for item in per_scenario if item is not follower_process
                ]
                stop_processes(remaining)
                time.sleep(0.5)

            metadata_path = newest_new_metadata(args.history_dir, history_before)
            validator_data = load_validator_result(metadata_path)
            result_data = validator_data.get("result") or {}
            collision_detected = bool(
                collision_event.is_set() or result_data.get("collision_detected", False)
            )
            reached_goal = goal_event.is_set()
            stopped_too_long = stopped_event.is_set()
            lane_departure = lane_departure_event.is_set()
            with monitor_lock:
                fixed_goal_data = {
                    "frame": "morai_enu",
                    "x": args.goal_x,
                    "y": args.goal_y,
                    "radius_m": args.goal_radius,
                    "reached": reached_goal,
                    "reached_at": goal_state["reached_at"],
                    "minimum_distance_m": goal_state["minimum_distance_m"],
                    "last_position": {
                        "x": goal_state["last_x"],
                        "y": goal_state["last_y"],
                    },
                    "last_distance_m": goal_state["last_distance_m"],
                }
                stopped_data = {
                    "speed_threshold_mps": args.stopped_speed_threshold,
                    "duration_threshold_s": args.stopped_duration,
                    "failed": stopped_too_long,
                    "failed_at": stopped_state["failed_at"],
                    "last_speed_mps": stopped_state["last_speed_mps"],
                    "maximum_continuous_stopped_s": stopped_state[
                        "maximum_continuous_stopped_s"
                    ],
                }
                lane_departure_data = {
                    "frame": "map",
                    "path_topic": DRIVABLE_PATH_TOPIC,
                    "duration_threshold_s": args.lane_departure_duration,
                    "failed": lane_departure,
                    "failed_at": lane_state["failed_at"],
                    "last_inside_drivable_bounds": lane_state["last_inside"],
                    "last_position": {
                        "x": lane_state["last_x"],
                        "y": lane_state["last_y"],
                    },
                    "maximum_continuous_outside_s": lane_state[
                        "maximum_continuous_outside_s"
                    ],
                }
            summary: Dict[str, Any] = {
                "scenario": scenario.name,
                "scenario_index": index,
                "started_at": scenario_started.isoformat(),
                "finished_at": datetime.now().astimezone().isoformat(),
                "end_reason": end_reason,
                "error": error_message,
                "collision_detected": collision_detected,
                "reached_goal": reached_goal,
                "stopped_too_long": stopped_too_long,
                "lane_departure": lane_departure,
                "passed": (
                    error_message is None
                    and reached_goal
                    and not collision_detected
                    and not stopped_too_long
                    and not lane_departure
                ),
                "fixed_goal": fixed_goal_data,
                "stopped_failure": stopped_data,
                "lane_departure_failure": lane_departure_data,
                "validator_finished_before_fixed_goal": (
                    validator_finished_before_goal
                ),
                "validator_return_code": validator_return_code,
                "rosbag_file": str(scenario_dir / "topics.bag")
                if not args.no_rosbag
                else None,
                "validator": validator_data,
            }
            atomic_json(scenario_dir / "summary.json", summary)
            session_results.append(summary)

            if not summary["passed"] and args.fail_fast:
                break

        aggregate = {
            "session_id": session_id,
            "started_scenarios": len(session_results),
            "requested_scenarios": len(scenarios),
            "passed": sum(1 for item in session_results if item["passed"]),
            "failed": sum(1 for item in session_results if not item["passed"]),
            "collisions": sum(
                1 for item in session_results if item["collision_detected"]
            ),
            "stopped_too_long": sum(
                1 for item in session_results if item["stopped_too_long"]
            ),
            "lane_departures": sum(
                1 for item in session_results if item["lane_departure"]
            ),
            "errors": sum(1 for item in session_results if item["error"]),
            "results": session_results,
        }
        atomic_json(session_dir / "summary.json", aggregate)
        print("\nSession summary: {}".format(session_dir / "summary.json"))
        return 1 if aggregate["failed"] else 0

    finally:
        if rospy is not None:
            try:
                rospy.signal_shutdown("validation cleanup")
            except Exception:
                pass
        stop_processes(shared)
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Load generated MORAI scenarios and validate them with AJOUNICE "
            "UDP/localization/planner/kroute/controller nodes."
        )
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="actually start ROS/MORAI validation (default: print dry-run plan)",
    )
    parser.add_argument(
        "--scenario-dir", type=Path, default=DEFAULT_SCENARIO_DIR
    )
    parser.add_argument(
        "--scenario-pattern",
        default="*_pos[0-9]*.json",
        help="glob used when --scenario is not supplied",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="specific scenario filename; may be repeated",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR
    )
    parser.add_argument(
        "--history-dir", type=Path, default=DEFAULT_HISTORY_DIR
    )
    parser.add_argument("--scenario-service", default="/ScenarioLoad")
    parser.add_argument("--velocity-kph", type=float, default=10.0)
    parser.add_argument(
        "--goal-x",
        type=float,
        default=-60.29,
        help="fixed goal X in MORAI ENU coordinates",
    )
    parser.add_argument(
        "--goal-y",
        type=float,
        default=-117.32,
        help="fixed goal Y in MORAI ENU coordinates",
    )
    parser.add_argument(
        "--goal-radius",
        type=float,
        default=1.0,
        help="goal acceptance radius in metres",
    )
    parser.add_argument(
        "--stopped-duration",
        type=float,
        default=10.0,
        help="fail after this many continuous stopped seconds",
    )
    parser.add_argument(
        "--stopped-speed-threshold",
        type=float,
        default=0.1,
        help="MORAI speed at or below this value is stopped (m/s)",
    )
    parser.add_argument(
        "--lane-departure-duration",
        type=float,
        default=0.5,
        help="fail after this many continuous seconds outside BPP drivable bounds",
    )
    parser.add_argument("--startup-timeout", type=float, default=45.0)
    parser.add_argument("--scenario-timeout", type=float, default=120.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--route-settle-seconds", type=float, default=2.0)
    parser.add_argument("--post-collision-seconds", type=float, default=2.0)
    collision_group = parser.add_mutually_exclusive_group()
    collision_group.add_argument(
        "--stop-on-collision",
        dest="stop_on_collision",
        action="store_true",
        help="stop the current scenario after a real collision (default)",
    )
    collision_group.add_argument(
        "--no-stop-on-collision",
        dest="stop_on_collision",
        action="store_false",
        help="keep running until goal or timeout after a collision",
    )
    parser.set_defaults(stop_on_collision=True)
    parser.add_argument("--fail-fast", action="store_true")
    parser.add_argument("--no-rosbag", action="store_true")
    parser.add_argument(
        "--trajectory-reference", default=DEFAULT_TRAJECTORY_REFERENCE
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=list(DEFAULT_TOPICS),
        help="additional rosbag topic; may be repeated",
    )
    args = parser.parse_args()

    minimum_values = {
        "velocity_kph": 0.001,
        "goal_radius": 0.001,
        "stopped_duration": 0.001,
        "stopped_speed_threshold": 0.0,
        "lane_departure_duration": 0.001,
        "startup_timeout": 0.001,
        "scenario_timeout": 0.001,
        "settle_seconds": 0.0,
        "route_settle_seconds": 0.0,
        "post_collision_seconds": 0.0,
    }
    for name, minimum in minimum_values.items():
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < minimum:
            parser.error("--{} must be >= {}".format(name.replace("_", "-"), minimum))
    if args.startup_timeout <= 0.0 or args.scenario_timeout <= 0.0:
        parser.error("timeouts must be positive")
    if args.goal_radius <= 0.0:
        parser.error("--goal-radius must be positive")
    if args.stopped_duration <= 0.0:
        parser.error("--stopped-duration must be positive")
    if args.stopped_speed_threshold < 0.0:
        parser.error("--stopped-speed-threshold must be non-negative")
    if args.lane_departure_duration <= 0.0:
        parser.error("--lane-departure-duration must be positive")
    if not math.isfinite(args.goal_x) or not math.isfinite(args.goal_y):
        parser.error("--goal-x and --goal-y must be finite")
    return args


def main() -> int:
    args = parse_args()
    try:
        scenarios = discover_scenarios(
            args.scenario_dir, args.scenario_pattern, args.scenario
        )
        commands = process_commands(args)
        if not args.execute:
            print_dry_run(args, scenarios, commands)
            return 0
        return execute(args, scenarios)
    except Exception as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
