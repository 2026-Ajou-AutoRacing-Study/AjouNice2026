#!/usr/bin/env python3
"""Run generated MORAI scenarios through the AjouNice ROS1 stack.

The default mode is a dry run.  Pass ``--execute`` only after MORAI is open,
its ROS ScenarioLoad provider is connected, and no other AJOUNICE stack is
using the host ROS master/UDP ports.
"""

import argparse
import csv
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


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENARIO_DIR = Path("/morai_scenarios")
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "artifacts/morai_scenario_validation"
DEFAULT_HISTORY_DIR = (
    REPOSITORY_ROOT / "base_ws/src/control/trajectory_validator/history"
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
CROSSWALK_SCENARIO = re.compile(
    r"^dynamic_(?P<actor>kid|adult|bicycle|deer|dog|human_dog)"
    r"_loc(?P<location>\d+)"
    r"_obj(?P<object_speed>[0-9p]+)"
    r"_(?P<crosswalk>front|opposite)_crosswalk"
    r"_ego(?P<ego_speed>[0-9p]+)"
    r"_act(?P<activation_distance>[0-9p]+)"
    r"_pos(?P<position>\d+)$",
    re.IGNORECASE,
)
VEHICLE_SCENARIO = re.compile(
    r"^dynamic_car_(?P<location>left|right)"
    r"_obj(?P<object_speed>[0-9p]+)"
    r"_(?P<trigger>before_stopline|arrow_tip|first_dashed_line)"
    r"_ego(?P<ego_speed>[0-9p]+)"
    r"_pos(?P<position>\d+)$",
    re.IGNORECASE,
)
OBJECT_GROUPS = (
    ("npc", "npc_list", "num_of_npcs"),
    ("pedestrian", "pedestrian_list", "num_of_pedestrian"),
    ("obstacle", "obstacle_list", "num_of_obstacle"),
)


class ValidationError(RuntimeError):
    """Raised when the validation cannot continue safely."""


def scenario_parameters(scenario: Path) -> Dict[str, Any]:
    """Extract the generated validation dimensions from a scenario filename."""
    result: Dict[str, Any] = {
        "kind": "other",
        "is_dynamic": False,
        "actor": None,
        "location": None,
        "crosswalk": None,
        "trigger": None,
        "object_speed_kph": None,
        "ego_speed_kph": None,
        "activation_distance_m": None,
        "position": None,
    }
    match = CROSSWALK_SCENARIO.match(scenario.stem)
    if match:
        groups = match.groupdict()
        result.update(
            {
                "kind": "crosswalk_dynamic_object",
                "is_dynamic": True,
                "actor": groups["actor"].lower(),
                "location": "loc" + groups["location"],
                "crosswalk": groups["crosswalk"].lower(),
                "object_speed_kph": float(groups["object_speed"].replace("p", ".")),
                "ego_speed_kph": float(groups["ego_speed"].replace("p", ".")),
                "activation_distance_m": float(
                    groups["activation_distance"].replace("p", ".")
                ),
                "position": int(groups["position"]),
            }
        )
        return result

    match = VEHICLE_SCENARIO.match(scenario.stem)
    if match:
        groups = match.groupdict()
        result.update(
            {
                "kind": "intersection_vehicle",
                "is_dynamic": True,
                "actor": "car",
                "location": groups["location"].lower(),
                "trigger": groups["trigger"].lower(),
                "object_speed_kph": float(groups["object_speed"].replace("p", ".")),
                "ego_speed_kph": float(groups["ego_speed"].replace("p", ".")),
                "position": int(groups["position"]),
            }
        )
    return result


def object_groups(message: Any) -> Dict[str, List[Any]]:
    """Read all MORAI object categories; pedestrians are not obstacles."""
    return {
        name: list(getattr(message, list_name, []) or [])
        for name, list_name, _count_name in OBJECT_GROUPS
    }


def object_counts(message: Any) -> Dict[str, int]:
    """Return robust object counts from message lists and MORAI count fields."""
    groups = object_groups(message)
    counts = {}
    for name, _list_name, count_name in OBJECT_GROUPS:
        try:
            reported = int(getattr(message, count_name, 0))
        except (TypeError, ValueError):
            reported = 0
        counts[name] = max(len(groups[name]), reported, 0)
    return counts


def trajectory_validation_error(message: Any, minimum_points: int = 3) -> Optional[str]:
    """Return the controller's reason for rejecting a trajectory, if any."""
    frame_id = str(getattr(getattr(message, "header", None), "frame_id", "")).strip()
    if not frame_id:
        return "header.frame_id is empty"
    points = list(getattr(message, "points", []) or [])
    if len(points) < minimum_points:
        return "has {} points; controller requires at least {}".format(
            len(points), minimum_points
        )
    for index, point in enumerate(points):
        pose = getattr(point, "pose", None)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        coordinates = (
            getattr(position, "x", float("nan")),
            getattr(position, "y", float("nan")),
            getattr(position, "z", float("nan")),
            getattr(orientation, "x", float("nan")),
            getattr(orientation, "y", float("nan")),
            getattr(orientation, "z", float("nan")),
            getattr(orientation, "w", float("nan")),
            getattr(point, "longitudinal_velocity_mps", float("nan")),
        )
        if not all(math.isfinite(float(value)) for value in coordinates):
            return "point {} contains a non-finite pose or velocity".format(index)
    return None


def wait_for_valid_trajectory(
    rospy: Any, trajectory_type: Any, topic: str, timeout: float
) -> Any:
    """Wait for a controller-acceptable trajectory, ignoring startup placeholders."""
    deadline = time.monotonic() + timeout
    last_error = "no trajectory received"
    while time.monotonic() < deadline:
        remaining = max(0.1, deadline - time.monotonic())
        try:
            message = rospy.wait_for_message(
                topic, trajectory_type, timeout=min(1.0, remaining)
            )
        except Exception as error:
            last_error = "no message ({})".format(type(error).__name__)
            continue
        validation_error = trajectory_validation_error(message)
        if validation_error is None:
            return message
        last_error = validation_error
    raise ValidationError(
        "planner did not publish a valid {} within {:.1f}s: {}".format(
            topic, timeout, last_error
        )
    )


def distance_xy(first: Tuple[float, float], second: Tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


@dataclass
class DynamicReactionMonitor:
    """Observe whether ego stopped for a MORAI dynamic object and resumed.

    The monitor intentionally uses MORAI's Ego/Object coordinates together, so
    it does not depend on the map-frame TF timing of the planner stack.
    """

    encounter_distance_m: float
    stop_speed_threshold_mps: float
    resume_speed_threshold_mps: float
    object_freshness_seconds: float
    require_resume: bool
    required: bool = False
    active: bool = False
    last_ego_position: Optional[Tuple[float, float]] = None
    last_ego_speed_mps: Optional[float] = None
    has_moved: bool = False
    object_seen_categories: Set[str] = field(default_factory=set)
    maximum_object_counts: Dict[str, int] = field(
        default_factory=lambda: {name: 0 for name, _list_name, _count_name in OBJECT_GROUPS}
    )
    encounter_detected: bool = False
    stopped_for_dynamic_object: bool = False
    resumed_after_dynamic_object: bool = False
    first_encounter_at: Optional[str] = None
    stopped_at: Optional[str] = None
    resumed_at: Optional[str] = None
    minimum_ego_object_distance_m: Optional[float] = None
    _last_encounter_monotonic: Optional[float] = None

    def activate(self, required: bool) -> None:
        self.required = required
        self.active = True
        self.last_ego_position = None
        self.last_ego_speed_mps = None
        self.has_moved = False
        self.object_seen_categories.clear()
        self.maximum_object_counts = {
            name: 0 for name, _list_name, _count_name in OBJECT_GROUPS
        }
        self.encounter_detected = False
        self.stopped_for_dynamic_object = False
        self.resumed_after_dynamic_object = False
        self.first_encounter_at = None
        self.stopped_at = None
        self.resumed_at = None
        self.minimum_ego_object_distance_m = None
        self._last_encounter_monotonic = None

    def deactivate(self) -> None:
        self.active = False

    def update_ego(
        self, position: Tuple[float, float], speed_mps: float, now: float, now_iso: str
    ) -> None:
        self.last_ego_position = position
        self.last_ego_speed_mps = speed_mps
        if not self.active:
            return
        if speed_mps >= self.resume_speed_threshold_mps:
            self.has_moved = True
            if self.stopped_for_dynamic_object and not self.resumed_after_dynamic_object:
                self.resumed_after_dynamic_object = True
                self.resumed_at = now_iso
        if (
            self.has_moved
            and self._encounter_is_fresh(now)
            and speed_mps <= self.stop_speed_threshold_mps
            and not self.stopped_for_dynamic_object
        ):
            self.stopped_for_dynamic_object = True
            self.stopped_at = now_iso

    def update_objects(self, message: Any, now: float, now_iso: str) -> None:
        groups = object_groups(message)
        counts = object_counts(message)
        for category, count in counts.items():
            self.maximum_object_counts[category] = max(
                self.maximum_object_counts[category], count
            )
            if count:
                self.object_seen_categories.add(category)
        if not self.active or self.last_ego_position is None:
            return
        positions = [
            (float(item.position.x), float(item.position.y))
            for items in groups.values()
            for item in items
        ]
        if not positions:
            return
        nearest = min(distance_xy(self.last_ego_position, position) for position in positions)
        if (
            self.minimum_ego_object_distance_m is None
            or nearest < self.minimum_ego_object_distance_m
        ):
            self.minimum_ego_object_distance_m = nearest
        if nearest <= self.encounter_distance_m:
            self._last_encounter_monotonic = now
            if not self.encounter_detected:
                self.encounter_detected = True
                self.first_encounter_at = now_iso
            if (
                self.has_moved
                and self.last_ego_speed_mps is not None
                and self.last_ego_speed_mps <= self.stop_speed_threshold_mps
                and not self.stopped_for_dynamic_object
            ):
                self.stopped_for_dynamic_object = True
                self.stopped_at = now_iso

    def _encounter_is_fresh(self, now: float) -> bool:
        return (
            self._last_encounter_monotonic is not None
            and now - self._last_encounter_monotonic <= self.object_freshness_seconds
        )

    def summary(self) -> Dict[str, Any]:
        failure_reasons = []
        object_seen = bool(self.object_seen_categories)
        if self.required:
            if not object_seen:
                failure_reasons.append("dynamic_object_not_observed")
            if not self.encounter_detected:
                failure_reasons.append("dynamic_object_never_reached_encounter_distance")
            if not self.stopped_for_dynamic_object:
                failure_reasons.append("ego_never_stopped_for_dynamic_object")
            if self.require_resume and not self.resumed_after_dynamic_object:
                failure_reasons.append("ego_never_resumed_after_dynamic_object")
        return {
            "required": self.required,
            "encounter_distance_m": self.encounter_distance_m,
            "stop_speed_threshold_mps": self.stop_speed_threshold_mps,
            "resume_speed_threshold_mps": self.resume_speed_threshold_mps,
            "object_freshness_seconds": self.object_freshness_seconds,
            "require_resume": self.require_resume,
            "object_seen": object_seen,
            "object_seen_categories": sorted(self.object_seen_categories),
            "maximum_object_counts": self.maximum_object_counts,
            "encounter_detected": self.encounter_detected,
            "first_encounter_at": self.first_encounter_at,
            "minimum_ego_object_distance_m": self.minimum_ego_object_distance_m,
            "stopped_for_dynamic_object": self.stopped_for_dynamic_object,
            "stopped_at": self.stopped_at,
            "resumed_after_dynamic_object": self.resumed_after_dynamic_object,
            "resumed_at": self.resumed_at,
            "passed": not failure_reasons,
            "failure_reasons": failure_reasons,
        }


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


def atomic_csv(path: Path, rows: Sequence[Dict[str, Any]], columns: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(columns), extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


RESULT_COLUMNS = (
    "session_id",
    "scenario",
    "scenario_load_transport",
    "kind",
    "actor",
    "location",
    "crosswalk",
    "trigger",
    "object_speed_kph",
    "ego_speed_kph",
    "activation_distance_m",
    "position",
    "passed",
    "end_reason",
    "error",
    "collision_detected",
    "reached_goal",
    "stopped_too_long",
    "lane_departure",
    "dynamic_reaction_required",
    "dynamic_object_seen",
    "dynamic_object_categories",
    "dynamic_max_npcs",
    "dynamic_max_pedestrians",
    "dynamic_max_obstacles",
    "dynamic_encounter_detected",
    "dynamic_first_encounter_at",
    "dynamic_minimum_distance_m",
    "dynamic_stopped",
    "dynamic_stopped_at",
    "dynamic_resumed",
    "dynamic_resumed_at",
    "dynamic_failure_reasons",
    "scenario_summary",
    "rosbag_file",
)


def result_table_row(summary: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten a per-scenario JSON result into one spreadsheet row."""
    parameters = summary.get("scenario_parameters") or {}
    reaction = summary.get("dynamic_reaction") or {}
    maximum_counts = reaction.get("maximum_object_counts") or {}
    return {
        "session_id": summary.get("session_id", ""),
        "scenario": summary.get("scenario", ""),
        "scenario_load_transport": summary.get("scenario_load_transport", ""),
        "kind": parameters.get("kind", ""),
        "actor": parameters.get("actor", "") or "",
        "location": parameters.get("location", "") or "",
        "crosswalk": parameters.get("crosswalk", "") or "",
        "trigger": parameters.get("trigger", "") or "",
        "object_speed_kph": parameters.get("object_speed_kph", "") or "",
        "ego_speed_kph": parameters.get("ego_speed_kph", "") or "",
        "activation_distance_m": parameters.get("activation_distance_m", "") or "",
        "position": parameters.get("position", "") or "",
        "passed": summary.get("passed", False),
        "end_reason": summary.get("end_reason", ""),
        "error": summary.get("error", "") or "",
        "collision_detected": summary.get("collision_detected", False),
        "reached_goal": summary.get("reached_goal", False),
        "stopped_too_long": summary.get("stopped_too_long", False),
        "lane_departure": summary.get("lane_departure", False),
        "dynamic_reaction_required": reaction.get("required", False),
        "dynamic_object_seen": reaction.get("object_seen", False),
        "dynamic_object_categories": ";".join(
            reaction.get("object_seen_categories", [])
        ),
        "dynamic_max_npcs": maximum_counts.get("npc", 0),
        "dynamic_max_pedestrians": maximum_counts.get("pedestrian", 0),
        "dynamic_max_obstacles": maximum_counts.get("obstacle", 0),
        "dynamic_encounter_detected": reaction.get("encounter_detected", False),
        "dynamic_first_encounter_at": reaction.get("first_encounter_at", "") or "",
        "dynamic_minimum_distance_m": reaction.get(
            "minimum_ego_object_distance_m", ""
        ),
        "dynamic_stopped": reaction.get("stopped_for_dynamic_object", False),
        "dynamic_stopped_at": reaction.get("stopped_at", "") or "",
        "dynamic_resumed": reaction.get("resumed_after_dynamic_object", False),
        "dynamic_resumed_at": reaction.get("resumed_at", "") or "",
        "dynamic_failure_reasons": ";".join(reaction.get("failure_reasons", [])),
        "scenario_summary": summary.get("summary_file", ""),
        "rosbag_file": summary.get("rosbag_file", "") or "",
    }


def write_session_results_table(session_dir: Path, summaries: Sequence[Dict[str, Any]]) -> Path:
    """Write a spreadsheet-ready table beside the aggregate session JSON."""
    table_path = session_dir / "results.csv"
    rows = [result_table_row(summary) for summary in summaries]
    atomic_csv(table_path, rows, RESULT_COLUMNS)
    return table_path


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


def wait_for_tcp_port(
    host: str, port: int, process: ManagedProcess, timeout: float, label: str
) -> None:
    """Wait for a child process to expose a TCP endpoint."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        process.assert_running()
        if tcp_port_open(host, port):
            return
        time.sleep(0.25)
    raise ValidationError(
        "{} did not listen on {}:{} within {:.1f}s".format(
            label, host, port, timeout
        )
    )


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


def wait_for_scenario_transport(
    rospy: Any, args: argparse.Namespace, topic_publisher: Any
) -> str:
    """Choose MORAI's available ScenarioLoad transport without UI changes.

    MORAI exposes ScenarioLoad either as the Service Model service
    ``/Service_MoraiSL`` or as a Publish-Subscribe Model subscriber on
    ``/ScenarioLoad``.  The latter cannot acknowledge a request, but it is the
    correct transport for the default Ego/Simulator Network topic setup.
    """
    print(
        "[action required] In MORAI, connect Simulator Network > ROS to "
        "ws://{}:{}; "
        "waiting up to {:.0f}s for {} or {}.".format(
            args.morai_bridge_host,
            args.rosbridge_port,
            args.morai_connect_timeout,
            args.scenario_service,
            args.scenario_topic,
        )
    )
    deadline = time.monotonic() + args.morai_connect_timeout
    while time.monotonic() < deadline:
        service_ready = False
        if args.scenario_transport != "topic":
            try:
                service_ready = args.scenario_service in run_checked(
                    ("rosservice", "list"), timeout=2.0
                ).splitlines()
            except (ValidationError, subprocess.TimeoutExpired):
                pass
        topic_ready = (
            args.scenario_transport != "service"
            and topic_publisher.get_num_connections() > 0
        )
        if service_ready:
            return "service"
        if topic_ready:
            return "topic"
        if rospy.is_shutdown():
            break
        time.sleep(0.25)
    if args.scenario_transport == "service":
        target = "service {}".format(args.scenario_service)
    elif args.scenario_transport == "topic":
        target = "subscriber on topic {}".format(args.scenario_topic)
    else:
        target = "service {} or subscriber on topic {}".format(
            args.scenario_service, args.scenario_topic
        )
    raise ValidationError(
        "timeout exceeded while waiting for MORAI ScenarioLoad {}".format(target)
    )


def wait_for_topic_transport_after_load(
    rospy: Any,
    topic_publisher: Any,
    topic: str,
    timeout: float,
    reconnect_detection_seconds: float = 2.0,
) -> None:
    """Wait through MORAI's ROS reset caused by a topic ScenarioLoad request.

    MORAI tears down its rosbridge clients while applying a scenario, then
    recreates them.  Publishing the next load request or reading the first
    object message during that gap produces false ``no objects`` failures.
    If a simulator version applies the scene in place, no disconnect is seen
    during the short detection window and it can proceed immediately.
    """
    started_at = time.monotonic()
    deadline = started_at + timeout
    detection_deadline = started_at + min(timeout, reconnect_detection_seconds)
    saw_disconnect = topic_publisher.get_num_connections() <= 0
    reconnect_notice_printed = False
    while time.monotonic() < deadline:
        connected = topic_publisher.get_num_connections() > 0
        if not connected:
            saw_disconnect = True
            if not reconnect_notice_printed:
                print(
                    "[action required] MORAI applied ScenarioLoad and reset its "
                    "ROS clients. In MORAI, reconnect both Ego Network > "
                    "Publisher, Subscriber, Service > ROS and Simulator Network "
                    "> ROS using the same bridge address; waiting up to {:.0f}s."
                    .format(deadline - time.monotonic())
                )
                reconnect_notice_printed = True
        elif saw_disconnect:
            print("[ready] MORAI ROS transport reconnected after ScenarioLoad")
            return
        elif time.monotonic() >= detection_deadline:
            # Some MORAI releases preserve their rosbridge clients while a
            # scenario is applied. Do not add a needless full timeout there.
            return
        if rospy.is_shutdown():
            break
        time.sleep(0.25)
    raise ValidationError(
        "MORAI did not reconnect {} within {:.1f}s after ScenarioLoad".format(
            topic, timeout
        )
    )


def make_scenario_load_message(message_type: Any, scenario_name: str) -> Any:
    message = message_type()
    message.file_name = scenario_name
    message.load_network_connection_data = False
    message.delete_all = True
    message.load_ego_vehicle_data = True
    message.load_surrounding_vehicle_data = True
    message.load_pedestrian_data = True
    message.load_obstacle_data = True
    message.set_pause = False
    return message


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
        # MORAI's Simulator Network uses rosbridge for ScenarioLoad.  Ego
        # control and sensor telemetry remain on the project's existing UDP
        # interface, so this is an additional endpoint rather than a
        # replacement for the Ego Network configuration.
        "rosbridge": [
            "roslaunch",
            "rosbridge_server",
            "rosbridge_websocket.launch",
            "port:={}".format(args.rosbridge_port),
        ],
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
    for name in (
        "roscore",
        "rosbridge",
        "udp",
        "sensing",
        "localization",
        "planner",
    ):
        print("  {}: {}".format(name, shell_join(commands[name])))
    print("\nPer scenario:")
    print(
        "  1. auto-select {} or {} and load the scenario filename".format(
            args.scenario_service, args.scenario_topic
        )
    )
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
        "     fail if an unexpected stop lasts {:.1f}s (0 disables it), or outside "
        "BPP drivable bounds for {:.1f}s".format(
            args.stopped_duration,
            args.lane_departure_duration,
        )
    )
    print(
        "     dynamic scenarios require object observation within {:.1f}m, a stop "
        "at <= {:.2f}m/s, and resume={} (policy={})".format(
            args.dynamic_encounter_distance,
            args.dynamic_stop_speed_threshold,
            args.dynamic_require_resume,
            args.dynamic_reaction_policy,
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
        (
            "roscore",
            "roslaunch",
            "rosnode",
            "rosservice",
            "rostopic",
            "rosbag",
            "rosparam",
        )
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
        bridge = start_process(
            "rosbridge", commands["rosbridge"], shared_logs, shared
        )
        wait_for_tcp_port(
            "127.0.0.1",
            args.rosbridge_port,
            bridge,
            args.startup_timeout,
            "rosbridge",
        )

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
            ScenarioLoad,
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
        reaction_monitor = DynamicReactionMonitor(
            encounter_distance_m=args.dynamic_encounter_distance,
            stop_speed_threshold_mps=args.dynamic_stop_speed_threshold,
            resume_speed_threshold_mps=args.dynamic_resume_speed_threshold,
            object_freshness_seconds=args.dynamic_object_freshness,
            require_resume=args.dynamic_require_resume,
        )

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
                        if (
                            args.stopped_duration > 0.0
                            and duration >= args.stopped_duration
                        ):
                            if stopped_state["failed_at"] is None:
                                stopped_state["failed_at"] = now_iso
                            stopped_event.set()
                    else:
                        stopped_state["stopped_since_monotonic"] = None
                        stopped_state["continuous_stopped_s"] = 0.0
                reaction_monitor.update_ego((x, y), speed, now_monotonic, now_iso)

        def object_callback(message: Any) -> None:
            now_monotonic = time.monotonic()
            now_iso = datetime.now().astimezone().isoformat()
            with monitor_lock:
                reaction_monitor.update_objects(message, now_monotonic, now_iso)

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
        object_subscriber = rospy.Subscriber(
            "/Object_topic", ObjectStatusList, object_callback, queue_size=20
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
        scenario_topic_publisher = rospy.Publisher(
            args.scenario_topic, ScenarioLoad, queue_size=1
        )
        scenario_transport = wait_for_scenario_transport(
            rospy, args, scenario_topic_publisher
        )
        scenario_loader = None
        if scenario_transport == "service":
            scenario_loader = rospy.ServiceProxy(
                args.scenario_service, MoraiScenarioLoadSrv, persistent=False
            )
        print(
            "[ready] MORAI ScenarioLoad transport: {} ({})".format(
                scenario_transport,
                args.scenario_service
                if scenario_transport == "service"
                else args.scenario_topic,
            )
        )

        critical_shared = (core, bridge, udp, sensing, localization, planner)
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
            parameters = scenario_parameters(scenario)
            reaction_required = args.dynamic_reaction_policy == "required" or (
                args.dynamic_reaction_policy == "auto"
                and bool(parameters["is_dynamic"])
            )
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

                load_message = make_scenario_load_message(ScenarioLoad, scenario.name)
                if scenario_transport == "service":
                    request = MoraiScenarioLoadSrvRequest()
                    request.request = load_message
                    response = scenario_loader(request)
                    if not bool(response.response.result):
                        raise ValidationError(
                            "MORAI rejected scenario load: {}".format(scenario.name)
                        )
                else:
                    scenario_topic_publisher.publish(load_message)
                    time.sleep(args.scenario_load_settle_seconds)
                    wait_for_topic_transport_after_load(
                        rospy,
                        scenario_topic_publisher,
                        args.scenario_topic,
                        args.scenario_reconnect_timeout,
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
                # A trigger-distance scenario can legitimately publish zero
                # dynamic actors before ego reaches its activation distance.
                # The reaction monitor evaluates object observation during
                # the drive, instead of rejecting the scenario at load time.
                object_callback(object_message)
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
                wait_for_valid_trajectory(
                    rospy,
                    Trajectory,
                    "/control/trajectory",
                    args.startup_timeout,
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
                    reaction_monitor.activate(reaction_required)

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
                    reaction_monitor.deactivate()
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
                dynamic_reaction_data = reaction_monitor.summary()
            if (
                error_message is None
                and end_reason == "goal_reached"
                and not dynamic_reaction_data["passed"]
            ):
                end_reason = "dynamic_reaction_failed"
            scenario_summary_path = scenario_dir / "summary.json"
            summary: Dict[str, Any] = {
                "session_id": session_id,
                "scenario": scenario.name,
                "scenario_index": index,
                "scenario_parameters": parameters,
                "scenario_load_transport": scenario_transport,
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
                    and dynamic_reaction_data["passed"]
                ),
                "fixed_goal": fixed_goal_data,
                "stopped_failure": stopped_data,
                "lane_departure_failure": lane_departure_data,
                "dynamic_reaction": dynamic_reaction_data,
                "validator_finished_before_fixed_goal": (
                    validator_finished_before_goal
                ),
                "validator_return_code": validator_return_code,
                "rosbag_file": str(scenario_dir / "topics.bag")
                if not args.no_rosbag
                else None,
                "summary_file": str(scenario_summary_path),
                "validator": validator_data,
            }
            atomic_json(scenario_summary_path, summary)
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
            "dynamic_reaction_failures": sum(
                1
                for item in session_results
                if item["dynamic_reaction"]["required"]
                and not item["dynamic_reaction"]["passed"]
            ),
            "errors": sum(1 for item in session_results if item["error"]),
            "results": session_results,
        }
        results_csv = write_session_results_table(session_dir, session_results)
        aggregate["results_csv"] = str(results_csv)
        atomic_json(session_dir / "summary.json", aggregate)
        print("\nSession summary: {}".format(session_dir / "summary.json"))
        print("Session results table: {}".format(results_csv))
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
    parser.add_argument(
        "--scenario-service",
        default="/Service_MoraiSL",
        help=(
            "MORAI ScenarioLoad service; MORAI Service Model normally exposes "
            "/Service_MoraiSL, while /ScenarioLoad is a topic"
        ),
    )
    parser.add_argument(
        "--scenario-topic",
        default="/ScenarioLoad",
        help="MORAI Publish-Subscribe Model ScenarioLoad topic",
    )
    parser.add_argument(
        "--scenario-transport",
        choices=("auto", "service", "topic"),
        default="auto",
        help="prefer the available MORAI ScenarioLoad service or topic (default: auto)",
    )
    parser.add_argument(
        "--rosbridge-port",
        type=int,
        default=9090,
        help="local rosbridge WebSocket port for MORAI Simulator Network",
    )
    parser.add_argument(
        "--morai-bridge-host",
        default=os.environ.get("MORAI_ROSBRIDGE_HOST", "127.0.0.1"),
        help=(
            "address entered in MORAI Simulator Network; defaults to "
            "MORAI_ROSBRIDGE_HOST or 127.0.0.1"
        ),
    )
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
        default=0.0,
        help="fail after this many unexpected continuous stopped seconds; 0 disables it",
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
    parser.add_argument(
        "--morai-connect-timeout",
        type=float,
        default=120.0,
        help=(
            "seconds to wait for MORAI Simulator Network to connect and expose "
            "ScenarioLoad"
        ),
    )
    parser.add_argument("--scenario-timeout", type=float, default=120.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    parser.add_argument("--route-settle-seconds", type=float, default=2.0)
    parser.add_argument("--post-collision-seconds", type=float, default=2.0)
    parser.add_argument(
        "--scenario-load-settle-seconds",
        type=float,
        default=1.0,
        help="wait after topic-based scenario load because that transport has no response",
    )
    parser.add_argument(
        "--scenario-reconnect-timeout",
        type=float,
        default=45.0,
        help="wait for MORAI rosbridge clients to reconnect after a topic ScenarioLoad",
    )
    parser.add_argument(
        "--dynamic-reaction-policy",
        choices=("auto", "required", "ignore"),
        default="auto",
        help="require stop/resume for generated dynamic_* scenarios, every scenario, or none",
    )
    parser.add_argument(
        "--dynamic-encounter-distance",
        type=float,
        default=18.0,
        help="MORAI Ego/Object distance that begins a dynamic-object encounter (m)",
    )
    parser.add_argument(
        "--dynamic-stop-speed-threshold",
        type=float,
        default=0.3,
        help="speed at or below which ego is considered stopped for a dynamic object (m/s)",
    )
    parser.add_argument(
        "--dynamic-resume-speed-threshold",
        type=float,
        default=0.7,
        help="speed at or above which ego is considered resumed after a dynamic object (m/s)",
    )
    parser.add_argument(
        "--dynamic-object-freshness",
        type=float,
        default=1.0,
        help="seconds an observed nearby dynamic object remains associated with an ego stop",
    )
    resume_group = parser.add_mutually_exclusive_group()
    resume_group.add_argument(
        "--dynamic-require-resume",
        dest="dynamic_require_resume",
        action="store_true",
        help="require ego to resume after stopping for a dynamic object (default)",
    )
    resume_group.add_argument(
        "--dynamic-no-require-resume",
        dest="dynamic_require_resume",
        action="store_false",
        help="allow a scenario to finish after a verified dynamic-object stop",
    )
    parser.set_defaults(dynamic_require_resume=True)
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
        "stopped_duration": 0.0,
        "stopped_speed_threshold": 0.0,
        "lane_departure_duration": 0.001,
        "startup_timeout": 0.001,
        "morai_connect_timeout": 0.001,
        "scenario_timeout": 0.001,
        "settle_seconds": 0.0,
        "route_settle_seconds": 0.0,
        "post_collision_seconds": 0.0,
        "scenario_load_settle_seconds": 0.0,
        "scenario_reconnect_timeout": 0.001,
        "dynamic_encounter_distance": 0.001,
        "dynamic_stop_speed_threshold": 0.0,
        "dynamic_resume_speed_threshold": 0.001,
        "dynamic_object_freshness": 0.001,
    }
    for name, minimum in minimum_values.items():
        value = float(getattr(args, name))
        if not math.isfinite(value) or value < minimum:
            parser.error("--{} must be >= {}".format(name.replace("_", "-"), minimum))
    if args.startup_timeout <= 0.0 or args.scenario_timeout <= 0.0:
        parser.error("timeouts must be positive")
    if args.goal_radius <= 0.0:
        parser.error("--goal-radius must be positive")
    if args.stopped_duration < 0.0:
        parser.error("--stopped-duration must be non-negative")
    if args.stopped_speed_threshold < 0.0:
        parser.error("--stopped-speed-threshold must be non-negative")
    if args.lane_departure_duration <= 0.0:
        parser.error("--lane-departure-duration must be positive")
    if not math.isfinite(args.goal_x) or not math.isfinite(args.goal_y):
        parser.error("--goal-x and --goal-y must be finite")
    if not 1 <= args.rosbridge_port <= 65535:
        parser.error("--rosbridge-port must be between 1 and 65535")
    if not args.morai_bridge_host.strip():
        parser.error("--morai-bridge-host must not be empty")
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
