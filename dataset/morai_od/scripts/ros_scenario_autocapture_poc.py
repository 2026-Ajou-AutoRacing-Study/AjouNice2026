#!/usr/bin/env python3
"""Load MORAI scenarios and trigger verified multi-sensor captures over ROS.

The first validation mode keeps Ego stationary and proves ScenarioLoad plus
SaveSensorData.  The route-follow mode then moves Ego only a few metres along
the official competition waypoint path with a bounded low-speed controller.
Wall time is used internally because loading a scenario can reset simulation
time.
"""

import argparse
import ctypes
import hashlib
import json
import math
import os
import queue
import shutil
import socket
import threading
import time
from collections import deque
from pathlib import Path

from dynamic_capture_metadata import (
    enrich_derived_motion,
    serialize_ego,
    serialize_objects,
)
from dynamic_timing_sidecar import (
    bind_trigger_timing,
    samples_in_window,
    timing_summary,
)

import rospy
from morai_msgs.msg import (
    CollisionData,
    CtrlCmd,
    EgoVehicleStatus,
    MultiEgoSetting,
    ObjectStatusList,
    SaveSensorData,
    ScenarioLoad,
)
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, PointCloud2


class ScenarioLoadUdpPacket(ctypes.Structure):
    """MORAI NetworkModule-compatible 69-byte ScenarioLoad packet."""

    _pack_ = 1
    _fields_ = [
        ("header", ctypes.c_char * 14),
        ("data_length", ctypes.c_int32),
        ("aux_data", ctypes.c_int32 * 3),
        ("filename", ctypes.c_char * 30),
        ("delete_all", ctypes.c_bool),
        ("network", ctypes.c_bool),
        ("ego", ctypes.c_bool),
        ("npc", ctypes.c_bool),
        ("pedestrian", ctypes.c_bool),
        ("object", ctypes.c_bool),
        ("pause", ctypes.c_bool),
        ("tail", ctypes.c_char * 2),
    ]


def build_scenario_load_payload(filename, request):
    encoded = filename.encode("ascii")
    if not encoded or len(encoded) > 30:
        raise ValueError("ScenarioLoad filename must be 1..30 ASCII bytes")
    packet = ScenarioLoadUdpPacket()
    packet.header = b"#ScenarioLoad$"
    packet.data_length = 37
    packet.aux_data = (ctypes.c_int32 * 3)(0, 0, 0)
    packet.filename = encoded.ljust(30, b" ")
    packet.delete_all = request["delete_all"]
    packet.network = request["load_network_connection_data"]
    packet.ego = request["load_ego_vehicle_data"]
    packet.npc = request["load_surrounding_vehicle_data"]
    packet.pedestrian = request["load_pedestrian_data"]
    packet.object = request["load_obstacle_data"]
    packet.pause = request["set_pause"]
    packet.tail = b"\r\n"
    return ctypes.string_at(ctypes.addressof(packet), ctypes.sizeof(packet))


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
            "Base dataset directory managed by this script. Unless "
            "--no-timestamp-prefix is used, YYMMDD_HHMMSS_ is prepended to "
            "the final directory name."
        ),
    )
    parser.add_argument(
        "--no-timestamp-prefix",
        action="store_true",
        help=(
            "Use --capture-dir exactly as supplied. By default a "
            "YYMMDD_HHMMSS_ prefix is added unless one is already present."
        ),
    )
    parser.add_argument(
        "--morai-sensor-data-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/"
            "SensorData"
        ),
        help=(
            "MORAI default SensorData directory. The personal simulator omits "
            "3D LiDAR when SaveSensorData uses a custom file_dir, so captures "
            "are verified here and then relocated to --capture-dir."
        ),
    )
    parser.add_argument(
        "--morai-scenario-dir",
        type=Path,
        default=Path.home()
        / (
            "MoraiLauncher_Lin_per/MoraiLauncher_Lin_Data/SaveFile/"
            "Scenario/R_KR_PR_K-city_2025"
        ),
        help=(
            "Current-map MORAI Scenario directory containing each scenario "
            "and EgoNetwork/<stem>_MN.json."
        ),
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="Scene id or file_name to run. Repeatable; omitted means all scenes.",
    )
    parser.add_argument(
        "--skip-scenario-load",
        action="store_true",
        help=(
            "Do not publish ScenarioLoad. Validate and use the currently "
            "loaded scene. This is intended for teleport testing after the "
            "Simulator Network was reconnected manually."
        ),
    )
    parser.add_argument(
        "--motion-mode",
        choices=(
            "stationary",
            "route-follow",
            "rolling-safe",
            "rolling-fast",
            "rolling-grouped",
            "timed-grouped",
            "teleport",
        ),
        default="stationary",
    )
    parser.add_argument("--captures-per-scene", type=int, default=None)
    parser.add_argument("--capture-spacing-m", type=float, default=None)
    parser.add_argument(
        "--capture-period-sec",
        type=float,
        default=None,
        help=(
            "Wall-clock interval between SaveSensorData requests in "
            "timed-grouped mode."
        ),
    )
    parser.add_argument(
        "--actor-ready-timeout-sec",
        type=float,
        default=None,
        help="Wait for the configured dynamic actor before timed capture.",
    )
    parser.add_argument(
        "--capture-motion-arm-m",
        type=float,
        default=None,
        help=(
            "Require Ego to move this route distance before the first "
            "timed-grouped frame."
        ),
    )
    parser.add_argument(
        "--minimum-capture-progress-m",
        type=float,
        default=None,
        help=(
            "Minimum Ego route progress between timed-grouped frames in "
            "addition to the time interval."
        ),
    )
    parser.add_argument(
        "--motion-stall-timeout-sec",
        type=float,
        default=None,
        help=(
            "Fail timed-grouped instead of capturing stationary duplicates "
            "when the next progress gate is not met."
        ),
    )
    parser.add_argument("--speed-kph", type=float, default=None)
    parser.add_argument("--maximum-motion-m", type=float, default=None)
    parser.add_argument("--load-timeout-sec", type=float, default=15.0)
    parser.add_argument(
        "--scenario-load-attempts",
        type=int,
        default=3,
        help=(
            "Retry ScenarioLoad when MORAI keeps publishing the previous "
            "scene or drops an actor while rosbridge clients are rebuilding."
        ),
    )
    parser.add_argument(
        "--network-reconnect-timeout-sec",
        type=float,
        default=20.0,
        help=(
            "Wall-clock timeout for MORAI ROS subscribers to reconnect after "
            "ScenarioLoad recreates Ego."
        ),
    )
    parser.add_argument("--load-settle-sec", type=float, default=2.0)
    parser.add_argument("--capture-timeout-sec", type=float, default=8.0)
    parser.add_argument(
        "--capture-stable-sec",
        type=float,
        default=1.0,
        help=(
            "Require the new-file set to remain unchanged for this wall-clock "
            "duration before considering asynchronous sensor writes complete."
        ),
    )
    parser.add_argument("--motion-timeout-sec", type=float, default=30.0)
    parser.add_argument("--minimum-files-per-capture", type=int, default=2)
    parser.add_argument(
        "--required-sensor-dir",
        action="append",
        default=None,
        help=(
            "Required top-level output directory below --capture-dir. "
            "Repeatable; defaults to CAMERA_1..4 and LIDAR_5."
        ),
    )
    parser.add_argument("--ego-position-tolerance-m", type=float, default=5.0)
    parser.add_argument("--route-deviation-limit-m", type=float, default=None)
    parser.add_argument("--heading-error-limit-deg", type=float, default=None)
    parser.add_argument("--lookahead-m", type=float, default=None)
    parser.add_argument("--wheelbase-m", type=float, default=3.0)
    parser.add_argument("--maximum-steering-rad", type=float, default=None)
    parser.add_argument(
        "--adaptive-curve-control",
        action="store_true",
        default=None,
        help=(
            "Adapt speed, lookahead, and steering limit to upcoming route "
            "heading change. Generated production manifests may enable it."
        ),
    )
    parser.add_argument("--curve-control-window-m", type=float, default=None)
    parser.add_argument(
        "--curve-control-full-turn-deg", type=float, default=None
    )
    parser.add_argument("--minimum-curve-speed-kph", type=float, default=None)
    parser.add_argument(
        "--minimum-curve-lookahead-m", type=float, default=None
    )
    parser.add_argument(
        "--curve-maximum-steering-rad", type=float, default=None
    )
    parser.add_argument("--ego-setting-topic", default="/ego_setting")
    parser.add_argument("--teleport-timeout-sec", type=float, default=5.0)
    parser.add_argument(
        "--teleport-position-tolerance-m",
        type=float,
        default=0.25,
        help=(
            "Maximum horizontal XY error after teleport. Vertical road "
            "settling is checked separately."
        ),
    )
    parser.add_argument(
        "--teleport-z-tolerance-m",
        type=float,
        default=0.75,
        help=(
            "Maximum absolute Z error after teleport. MORAI may settle the "
            "vehicle collider onto the road surface after /ego_setting."
        ),
    )
    parser.add_argument("--teleport-settle-sec", type=float, default=0.5)
    parser.add_argument(
        "--personal-max-steering-angle-deg",
        type=float,
        default=36.25,
        help=(
            "Maximum front wheel angle used to normalize steering for the "
            "personal MORAI front_steer schema."
        ),
    )
    parser.add_argument(
        "--personal-steering-sign",
        type=float,
        default=None,
        help=(
            "Sign converting mathematical steering (left positive) to the "
            "personal MORAI normalized front_steer field. The personal "
            "launcher uses right-positive steering, so generated manifests "
            "default this to -1. Competition CtrlCmd.steering is unaffected."
        ),
    )
    parser.add_argument("--scenario-topic", default="/ScenarioLoad")
    parser.add_argument(
        "--scenario-load-transport",
        choices=("ros", "udp"),
        default="ros",
        help=(
            "Send ScenarioLoad through the ROS topic or the independent MORAI "
            "UDP port. UDP requires a matching full UDP Ego Network profile."
        ),
    )
    parser.add_argument("--scenario-load-udp-host", default="127.0.0.1")
    parser.add_argument("--scenario-load-udp-port", type=int, default=9095)
    parser.add_argument("--capture-topic", default="/SaveSensorData")
    parser.add_argument("--control-topic", default="/ctrl_cmd")
    parser.add_argument("--ego-topic", default="/Ego_topic")
    parser.add_argument("--object-topic", default="/Object_topic")
    parser.add_argument("--collision-topic", default="/CollisionData")
    parser.add_argument("--clock-topic", default="/clock")
    parser.add_argument("--lidar-timing-topic", default="/sensor/lidar1")
    parser.add_argument(
        "--camera-timing-topic",
        action="append",
        default=None,
        metavar="NAME=TOPIC",
        help=(
            "Optional compressed-camera topic whose header timing is stored. "
            "Repeatable. Defaults to front/front_left/front_right/back."
        ),
    )
    parser.add_argument(
        "--timing-history-sec",
        type=float,
        default=20.0,
        help="Retention window for lightweight ROS timing/state samples.",
    )
    parser.add_argument(
        "--timing-window-padding-sec",
        type=float,
        default=0.25,
        help="Context retained before and after each timed capture clip.",
    )
    parser.add_argument(
        "--no-timing-sidecar",
        action="store_true",
        help="Disable continuous clock/Ego/Object and optional sensor timing metadata.",
    )
    parser.add_argument(
        "--control-mode-service",
        default="/Service_MoraiEventCmd",
        help="MORAI service used to select AV-ExternalCtrl after every load.",
    )
    parser.add_argument(
        "--control-mode-timeout-sec",
        type=float,
        default=5.0,
        help=(
            "Wall-clock timeout for the MoraiEventCmd service response. "
            "Prevents an unhealthy rosbridge service from hanging a batch."
        ),
    )
    parser.add_argument(
        "--ego-heartbeat-timeout-sec",
        type=float,
        default=3.0,
        help=(
            "Require a fresh /Ego_topic message within this wall-clock time "
            "after selecting external control mode."
        ),
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
        "--continue-on-scene-error",
        action="store_true",
        help=(
            "Record a failed scene and continue with the next manifest "
            "entry. Intended for multi-scene stress tests and unattended "
            "collection batches."
        ),
    )
    parser.add_argument(
        "--resume-report",
        type=Path,
        help=(
            "Resume an interrupted run from its partial/final report. "
            "Completed scenes are preserved and skipped; failed and "
            "unattempted scenes are run again in the same capture directory."
        ),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate manifest, paths, naming, and policy without ROS publishing.",
    )
    return parser.parse_args()


def timestamped_capture_dir(path, timestamp=None):
    """Prepend YYMMDD_HHMMSS unless the directory already has that prefix."""

    parts = path.name.split("_", 2)
    already_prefixed = (
        len(parts) == 3
        and len(parts[0]) == 6
        and parts[0].isdigit()
        and len(parts[1]) == 6
        and parts[1].isdigit()
    )
    if already_prefixed:
        return path
    if timestamp is None:
        timestamp = time.strftime("%y%m%d_%H%M%S", time.localtime())
    return path.with_name("{}_{}".format(timestamp, path.name))


def parse_named_topics(values):
    """Parse repeatable NAME=TOPIC arguments into stable stream names."""

    if values is None:
        values = (
            "camera_front=/sensor/camera/front/compressed",
            "camera_front_left=/sensor/camera/front_left/compressed",
            "camera_front_right=/sensor/camera/front_right/compressed",
            "camera_back=/sensor/camera/back/compressed",
        )
    result = {}
    for value in values:
        if "=" not in value:
            raise ValueError(
                "--camera-timing-topic must use NAME=TOPIC: {}".format(value)
            )
        name, topic = value.split("=", 1)
        name = name.strip()
        topic = topic.strip()
        if not name or not topic.startswith("/"):
            raise ValueError(
                "invalid --camera-timing-topic NAME=TOPIC: {}".format(value)
            )
        if name in result:
            raise ValueError("duplicate camera timing stream: {}".format(name))
        result[name] = topic
    return result


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json_atomic(path, document):
    """Atomically replace a small JSON state file."""

    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, separators=(",", ":"))
        + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(path))


def append_jsonl(path, document):
    """Append one compact record without rewriting prior scene results."""

    with path.open("a", encoding="utf-8") as stream:
        stream.write(
            json.dumps(
                document,
                ensure_ascii=False,
                separators=(",", ":"),
            )
            + "\n"
        )


def compact_scene_result(result):
    """Keep only fields needed for progress, handoff, and resume."""

    metrics = result.get("metrics", {})
    compact_metrics = {
        key: metrics[key]
        for key in (
            "scene_duration_sec",
            "trigger_count",
            "trigger_span_sec",
            "drain_duration_sec",
            "complete_timestamp_group_count",
            "relocated_file_count",
            "quality_verified",
            "trigger_rate_hz",
            "capture_period_sec",
            "frames_metadata",
            "clip_metadata",
        )
        if key in metrics
    }
    error = result.get("error")
    if error is not None and len(error) > 1000:
        error = error[:1000] + "...<truncated; see failures.jsonl>"
    return {
        "id": result.get("id"),
        "base_scenario_id": result.get("base_scenario_id"),
        "file_name": result.get("file_name"),
        "split": result.get("split"),
        "status": result.get("status"),
        "capture_count": int(
            result.get("capture_count", len(result.get("captures", [])))
        ),
        "error": error,
        "metrics": compact_metrics,
    }


def detailed_failure_result(result):
    """Bound failure detail so one malformed error cannot explode a report."""

    failure = {
        "id": result.get("id"),
        "base_scenario_id": result.get("base_scenario_id"),
        "file_name": result.get("file_name"),
        "split": result.get("split"),
        "status": "failed",
        "expected_counts": result.get("expected_counts"),
        "actual_objects": result.get("actual_objects"),
        "metrics": {
            key: result.get("metrics", {})[key]
            for key in (
                "scene_duration_sec",
                "trigger_count",
                "trigger_span_sec",
                "complete_timestamp_group_count",
                "relocated_file_count",
            )
            if key in result.get("metrics", {})
        },
    }
    error = str(result.get("error", ""))
    failure["error"] = error[:8192]
    failure["error_truncated"] = len(error) > 8192
    return failure


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


def observed_sensor_directories(root, files):
    observed = set()
    for path in files:
        try:
            relative = path.relative_to(root)
        except ValueError:
            continue
        if len(relative.parts) >= 2:
            observed.add(relative.parts[0])
    return observed


def required_capture_suffixes(sensor_dir, require_object_labels):
    """Return training artifacts that MORAI must emit for one sensor."""

    if sensor_dir.startswith("CAMERA_"):
        return {
            "_Intensity.png",
            "_Semantic.png",
            "_Instance.png",
            "_Depth.png",
        }
    if sensor_dir.startswith("LIDAR_"):
        suffixes = {
            "_Intensity.bin",
            "_Semantic.bin",
            "_Instance.bin",
        }
        if require_object_labels:
            suffixes.update(
                {
                    "_Instance.txt",
                    "_Instance_8Points.txt",
                }
            )
        return suffixes
    return set()


def capture_artifact_status(
    source_root,
    files,
    required_sensor_dirs,
    require_object_labels,
):
    """Summarize observed and missing training artifacts by sensor."""

    observed = {sensor: set() for sensor in required_sensor_dirs}
    for path in files:
        try:
            relative = path.relative_to(source_root)
        except ValueError:
            continue
        if len(relative.parts) < 2:
            continue
        sensor = relative.parts[0]
        if sensor in observed:
            observed[sensor].add(capture_suffix(path.name))

    required = {
        sensor: required_capture_suffixes(sensor, require_object_labels)
        for sensor in required_sensor_dirs
    }
    missing = {
        sensor: sorted(required[sensor] - observed[sensor])
        for sensor in required_sensor_dirs
        if required[sensor] - observed[sensor]
    }
    return observed, required, missing


def capture_suffix(filename):
    for suffix in capture_known_suffixes():
        if filename.endswith(suffix):
            return suffix
    marker = filename.find("_")
    if marker < 0:
        return "_" + filename
    return filename[marker:]


def capture_known_suffixes():
    return (
        "_Instance_8Points.txt",
        "_BBox2D.txt",
        "_BBox3D.txt",
        "_Intensity.png",
        "_Semantic.png",
        "_Instance.png",
        "_Depth.png",
        "_Intensity.bin",
        "_Semantic.bin",
        "_Instance.bin",
        "_Instance.txt",
    )


def capture_timestamp_stem(filename):
    """Return the MORAI timestamp shared by one multisensor snapshot."""

    for suffix in capture_known_suffixes():
        if filename.endswith(suffix):
            return filename[: -len(suffix)]
    return None


def group_capture_files_by_timestamp(files):
    """Group Camera/LiDAR artifacts emitted by one SaveSensorData request."""

    grouped = {}
    unrecognized = []
    for path in sorted(files):
        timestamp_stem = capture_timestamp_stem(path.name)
        if timestamp_stem is None:
            unrecognized.append(path)
            continue
        grouped.setdefault(timestamp_stem, set()).add(path)
    return grouped, unrecognized


def relocate_capture_files(
    source_root,
    destination_root,
    capture_id,
    files,
    allow_empty_suffixes=(),
):
    """Move one snapshot while preserving valid empty-label files.

    MORAI writes a zero-byte LiDAR ``_Instance.txt`` (and, depending on the
    build, ``_Instance_8Points.txt``) when a scene intentionally contains no
    actors.  That is a valid negative label, not a failed sensor write.  Image
    and point-cloud payloads must still be non-empty.
    """

    allow_empty_suffixes = set(allow_empty_suffixes)
    destinations = []
    plan = []
    for source in sorted(files):
        relative = source.relative_to(source_root)
        if len(relative.parts) < 2:
            raise RuntimeError(
                "MORAI sensor file is outside a sensor directory: {}".format(
                    source
                )
            )
        sensor_dir = relative.parts[0]
        destination = (
            destination_root
            / sensor_dir
            / (capture_id + capture_suffix(source.name))
        )
        if destination.exists():
            raise FileExistsError(
                "capture destination already exists: {}".format(destination)
            )
        plan.append((source, destination))

    for source, destination in plan:
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(source), str(destination))
        suffix = capture_suffix(destination.name)
        empty_is_valid = (
            destination.is_file()
            and destination.stat().st_size == 0
            and suffix in allow_empty_suffixes
        )
        if not destination.is_file() or (
            destination.stat().st_size <= 0 and not empty_is_valid
        ):
            raise RuntimeError(
                "relocated capture file is invalid: {}".format(destination)
            )
        destinations.append(destination)
    return destinations


def set_ctrl_steering(
    message,
    steering_rad,
    personal_max_steering_angle_deg=36.25,
    personal_steering_sign=-1.0,
):
    """Set steering on competition and personal MORAI CtrlCmd schemas."""

    if hasattr(message, "steering"):
        message.steering = steering_rad
        return "steering"
    if hasattr(message, "front_steer") and hasattr(message, "rear_steer"):
        maximum_rad = math.radians(personal_max_steering_angle_deg)
        if maximum_rad <= 0.0:
            raise ValueError("personal maximum steering angle must be positive")
        normalized = max(
            -1.0,
            min(
                1.0,
                personal_steering_sign * steering_rad / maximum_rad,
            ),
        )
        message.front_steer = normalized
        message.rear_steer = 0.0
        return "front_steer"
    raise RuntimeError(
        "unsupported morai_msgs/CtrlCmd steering fields: {}".format(
            getattr(message, "__slots__", ())
        )
    )


def route_pose_at_distance(waypoints, start_index, distance):
    """Interpolate a pose a given XY arc distance along the waypoint path."""

    if distance < 0.0:
        raise ValueError("route distance must be non-negative")
    remaining = distance
    index = start_index
    while index + 1 < len(waypoints):
        start = waypoints[index]
        end = waypoints[index + 1]
        segment = distance_xy(start, end)
        if segment <= 1e-9:
            index += 1
            continue
        if remaining <= segment:
            ratio = remaining / segment
            position = tuple(
                start[axis] + ratio * (end[axis] - start[axis])
                for axis in range(3)
            )
            yaw_deg = math.degrees(
                math.atan2(end[1] - start[1], end[0] - start[0])
            )
            return position, yaw_deg, index
        remaining -= segment
        index += 1
    raise RuntimeError(
        "requested teleport distance {:.2f} m exceeds waypoint path".format(
            distance
        )
    )


def make_multi_ego_teleport_message(ego, position, yaw_deg):
    """Build the one-Ego transform command supported by MORAI ROS."""

    unique_id = int(ego.unique_id)
    message = MultiEgoSetting()
    message.number_of_ego_vehicle = 1
    message.camera_index = unique_id
    message.ego_index = [unique_id]
    message.global_position_x = [float(position[0])]
    message.global_position_y = [float(position[1])]
    message.global_position_z = [float(position[2])]
    message.global_roll = [0.0]
    message.global_pitch = [0.0]
    message.global_yaw = [float(yaw_deg)]
    message.velocity = [0.0]
    message.gear = [4]
    # MultiEgoSetting uses 16 for automatic/external control even though the
    # separate EventInfo service reports its external mode as 3.
    message.ctrl_mode = [16]

    # Newer personal MORAI schemas add these arrays. Keep compatibility with
    # the competition schema by setting them only when they exist.
    optional_arrays = {
        "steering_angle": [0.0],
        "vehicle_speed": [0.0],
        "turn_signal": [0],
        "brake_light": [True],
    }
    for name, value in optional_arrays.items():
        if hasattr(message, name):
            setattr(message, name, value)
    return message


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def configured_ros_topics(network_sidecar):
    document = load_json(network_sidecar)
    ego_network_data = document.get("egoNetworkData")
    if not isinstance(ego_network_data, list) or not ego_network_data:
        raise ValueError(
            "network sidecar has no egoNetworkData: {}".format(
                network_sidecar
            )
        )

    topics = set()
    ros_bridge_entries = 0
    for ego in ego_network_data:
        if ego.get("connectedType") == 3:
            ros_bridge_entries += 1
        for entry in ego.get("listEgoNetworkInfo", []):
            if entry.get("netType") == 3:
                ros_bridge_entries += 1
            else:
                continue
            topic = (
                entry.get("config", {})
                .get("rosConfig", {})
                .get("Topic")
            )
            if topic:
                topics.add(topic)
    if ros_bridge_entries == 0:
        raise ValueError(
            "network sidecar is not a ROS bridge profile: {}".format(
                network_sidecar
            )
        )
    return topics


def configured_network_unique_ids(network_sidecar):
    document = load_json(network_sidecar)
    ego_network_data = document.get("egoNetworkData")
    if not isinstance(ego_network_data, list) or not ego_network_data:
        raise ValueError(
            "network sidecar has no egoNetworkData: {}".format(
                network_sidecar
            )
        )

    unique_ids = set()
    for entry in ego_network_data:
        value = None
        for key in ("uniqueId", "UNIQUEID", "uniqueID"):
            if key in entry:
                value = int(entry[key])
                break
        unique_ids.add(value)
    return unique_ids


def validate_installed_scenes(args, manifest, scenes, project_root):
    network_policy = manifest.get("network_policy")
    if not isinstance(network_policy, dict):
        raise ValueError(
            "manifest has no network_policy; regenerate the PoC set with "
            "generate_ros_capture_poc_set.py"
        )
    if network_policy.get("transport") != "rosbridge":
        raise ValueError("manifest network transport must be rosbridge")
    if network_policy.get("load_network_connection_data") is not True:
        raise ValueError(
            "manifest must enable load_network_connection_data"
        )

    required_topics = set(network_policy.get("required_topics", []))
    required_unique_ids = set(
        network_policy.get("required_unique_ids", [])
    )
    forbidden_unique_ids = set(
        network_policy.get("forbidden_unique_ids", [])
    )
    expected_digest = network_policy.get("template_sha256")
    if not expected_digest:
        raise ValueError(
            "manifest has no ROS network template SHA256; regenerate and "
            "install the PoC set with --install-dir"
        )

    install_dir = args.morai_scenario_dir.expanduser().resolve()
    if not install_dir.is_dir():
        raise FileNotFoundError(
            "MORAI scenario directory does not exist: {}".format(install_dir)
        )

    for scene in scenes:
        source_scenario = Path(scene["scenario"])
        if not source_scenario.is_absolute():
            source_scenario = project_root / source_scenario
        installed_scenario = install_dir / (scene["file_name"] + ".json")
        if not installed_scenario.is_file():
            raise FileNotFoundError(
                "installed scenario does not exist: {}".format(
                    installed_scenario
                )
            )
        if load_json(installed_scenario) != load_json(source_scenario):
            raise ValueError(
                "installed scenario differs from generated source: {}".format(
                    installed_scenario
                )
            )

        network_relative = Path(
            scene.get(
                "network_sidecar",
                "EgoNetwork/{}_MN.json".format(scene["file_name"]),
            )
        )
        if network_relative.is_absolute() or ".." in network_relative.parts:
            raise ValueError(
                "invalid network sidecar path in manifest: {}".format(
                    network_relative
                )
            )
        network_sidecar = install_dir / network_relative
        if not network_sidecar.is_file():
            raise FileNotFoundError(
                "scenario network sidecar does not exist: {}".format(
                    network_sidecar
                )
            )
        actual_digest = sha256_file(network_sidecar)
        if actual_digest != expected_digest:
            raise ValueError(
                "network sidecar SHA256 differs from the installed template: "
                "{}".format(network_sidecar)
            )
        missing_topics = sorted(
            required_topics - configured_ros_topics(network_sidecar)
        )
        if missing_topics:
            raise ValueError(
                "network sidecar misses required ROS topics {}: {}".format(
                    missing_topics, network_sidecar
                )
            )
        unique_ids = configured_network_unique_ids(network_sidecar)
        missing_unique_ids = sorted(required_unique_ids - unique_ids)
        forbidden_present = sorted(forbidden_unique_ids & unique_ids)
        if missing_unique_ids or forbidden_present:
            raise ValueError(
                "network sidecar has invalid UNIQUEIDs: {}; missing={}, "
                "forbidden_present={}".format(
                    network_sidecar,
                    missing_unique_ids,
                    forbidden_present,
                )
            )
    return install_dir


class SharedState:
    def __init__(self, timing_history_sec=20.0, timing_enabled=True):
        self.lock = threading.Lock()
        self.timing_history_sec = float(timing_history_sec)
        self.timing_enabled = bool(timing_enabled)
        self.histories = {
            "clock": deque(),
            "ego": deque(),
            "objects": deque(),
        }
        self.sample_sequences = {}
        self.ego = None
        self.ego_wall_time = 0.0
        self.ego_unix_time = 0.0
        self.objects = None
        self.objects_wall_time = 0.0
        self.objects_unix_time = 0.0
        self.collision = None
        self.collision_wall_time = 0.0

    def _append_timing(self, stream, record):
        if not self.timing_enabled:
            return
        sequence = self.sample_sequences.get(stream, 0)
        self.sample_sequences[stream] = sequence + 1
        record["sample_id"] = "{}:{:09d}".format(stream, sequence)
        history = self.histories.setdefault(stream, deque())
        history.append(record)
        cutoff = record["callback_monotonic"] - self.timing_history_sec
        while history and history[0]["callback_monotonic"] < cutoff:
            history.popleft()

    def ego_callback(self, message):
        callback_monotonic = time.monotonic()
        callback_unix = time.time()
        with self.lock:
            self.ego = message
            self.ego_wall_time = callback_monotonic
            self.ego_unix_time = callback_unix
            state = serialize_ego(message, callback_unix, callback_monotonic)
            self._append_timing(
                "ego",
                {
                    "stream": "ego",
                    "callback_monotonic": callback_monotonic,
                    "callback_unix": callback_unix,
                    "message_timestamp": state.get("timestamp"),
                    "state": state,
                },
            )

    def object_callback(self, message):
        callback_monotonic = time.monotonic()
        callback_unix = time.time()
        with self.lock:
            self.objects = message
            self.objects_wall_time = callback_monotonic
            self.objects_unix_time = callback_unix
            state = serialize_objects(message, callback_unix, callback_monotonic)
            self._append_timing(
                "objects",
                {
                    "stream": "objects",
                    "callback_monotonic": callback_monotonic,
                    "callback_unix": callback_unix,
                    "message_timestamp": state.get("timestamp"),
                    "state": state,
                },
            )

    def collision_callback(self, message):
        with self.lock:
            self.collision = message
            self.collision_wall_time = time.monotonic()

    def clock_callback(self, message):
        callback_monotonic = time.monotonic()
        callback_unix = time.time()
        with self.lock:
            self._append_timing(
                "clock",
                {
                    "stream": "clock",
                    "callback_monotonic": callback_monotonic,
                    "callback_unix": callback_unix,
                    "message_timestamp": message.clock.to_sec(),
                    "simulation_time": message.clock.to_sec(),
                },
            )

    def pointcloud_callback(self, message, stream):
        callback_monotonic = time.monotonic()
        callback_unix = time.time()
        with self.lock:
            self._append_timing(
                stream,
                {
                    "stream": stream,
                    "callback_monotonic": callback_monotonic,
                    "callback_unix": callback_unix,
                    "message_timestamp": message.header.stamp.to_sec(),
                    "frame_id": message.header.frame_id,
                    "width": int(message.width),
                    "height": int(message.height),
                    "point_step": int(message.point_step),
                    "row_step": int(message.row_step),
                    "is_dense": bool(message.is_dense),
                    "fields": [field.name for field in message.fields],
                },
            )

    def camera_callback(self, message, stream):
        callback_monotonic = time.monotonic()
        callback_unix = time.time()
        with self.lock:
            self._append_timing(
                stream,
                {
                    "stream": stream,
                    "callback_monotonic": callback_monotonic,
                    "callback_unix": callback_unix,
                    "message_timestamp": message.header.stamp.to_sec(),
                    "frame_id": message.header.frame_id,
                    "format": str(message.format),
                    "compressed_size_bytes": len(message.data),
                },
            )

    def timing_window(self, start_monotonic, end_monotonic, padding_sec):
        with self.lock:
            return {
                stream: samples_in_window(
                    list(samples), start_monotonic, end_monotonic, padding_sec
                )
                for stream, samples in self.histories.items()
            }

    def snapshot(self):
        with self.lock:
            return {
                "ego": self.ego,
                "ego_wall_time": self.ego_wall_time,
                "ego_unix_time": self.ego_unix_time,
                "objects": self.objects,
                "objects_wall_time": self.objects_wall_time,
                "objects_unix_time": self.objects_unix_time,
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
        self.default_waypoint_path = waypoint_path
        self.waypoints = []
        self.waypoint_arc_m = []
        self.load_scene_waypoints(waypoint_path)
        self.base_motion_args = {
            key: getattr(args, key)
            for key in (
                "speed_kph",
                "maximum_motion_m",
                "motion_stall_timeout_sec",
                "route_deviation_limit_m",
                "lookahead_m",
                "adaptive_curve_control",
            )
        }
        self.scene_minimum_ego_speed_kph = 0.0
        self.state = SharedState(
            timing_history_sec=args.timing_history_sec,
            timing_enabled=not args.no_timing_sidecar,
        )
        self.run_id = time.strftime("r%m%d%H%M%S", time.localtime())
        self.dataset_id = manifest.get("dataset_id")
        self.environment = dict(manifest.get("environment", {}))
        self.environment_id = self.environment.get("id")

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
        self.ego_setting_pub = rospy.Publisher(
            args.ego_setting_topic, MultiEgoSetting, queue_size=1
        )
        steering_field = set_ctrl_steering(
            CtrlCmd(),
            0.0,
            args.personal_max_steering_angle_deg,
            args.personal_steering_sign,
        )
        rospy.loginfo(
            "CtrlCmd schema detected: steering field=%s, personal sign=%+.1f",
            steering_field,
            args.personal_steering_sign,
        )
        rospy.loginfo(
            "Curve control: adaptive=%s, speed=%.2f->%.2f km/h, "
            "lookahead=%.2f->%.2f m, steering=%.3f->%.3f rad",
            args.adaptive_curve_control,
            args.speed_kph,
            args.minimum_curve_speed_kph,
            args.lookahead_m,
            args.minimum_curve_lookahead_m,
            args.maximum_steering_rad,
            args.curve_maximum_steering_rad,
        )
        self.control_mode_client = rospy.ServiceProxy(
            args.control_mode_service, MoraiEventCmdSrv
        )
        self.scenario_udp_socket = (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if args.scenario_load_transport == "udp"
            else None
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
        self.timing_topics = {
            "clock": args.clock_topic,
            "ego": args.ego_topic,
            "objects": args.object_topic,
            "lidar": args.lidar_timing_topic,
        }
        self.timing_topics.update(args.camera_timing_topics)
        if not args.no_timing_sidecar:
            self.subscribers.append(
                rospy.Subscriber(
                    args.clock_topic,
                    Clock,
                    self.state.clock_callback,
                    queue_size=100,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    args.lidar_timing_topic,
                    PointCloud2,
                    self.state.pointcloud_callback,
                    callback_args="lidar",
                    queue_size=4,
                )
            )
            for stream, topic in sorted(args.camera_timing_topics.items()):
                self.subscribers.append(
                    rospy.Subscriber(
                        topic,
                        CompressedImage,
                        self.state.camera_callback,
                        callback_args=stream,
                        queue_size=4,
                    )
                )

    def load_scene_waypoints(self, waypoint_path):
        self.waypoints = load_waypoints(waypoint_path)
        self.waypoint_arc_m = [0.0]
        for before, after in zip(self.waypoints, self.waypoints[1:]):
            self.waypoint_arc_m.append(
                self.waypoint_arc_m[-1] + distance_xy(before, after)
            )

    def configure_scene_motion(self, scene):
        for key, value in self.base_motion_args.items():
            setattr(self.args, key, value)
        overrides = dict(scene.get("capture_policy_overrides", {}))
        override_to_arg = {
            "ego_target_speed_kph": "speed_kph",
        }
        for override_key, argument_key in override_to_arg.items():
            if override_key in overrides:
                setattr(self.args, argument_key, overrides[override_key])
        for key in self.base_motion_args:
            if key in overrides:
                setattr(self.args, key, overrides[key])
        self.scene_minimum_ego_speed_kph = float(
            overrides.get("minimum_ego_speed_kph", 0.0)
        )
        waypoint_path = Path(
            scene.get("waypoints", self.default_waypoint_path)
        )
        if not waypoint_path.is_absolute():
            waypoint_path = self.project_root / waypoint_path
        self.load_scene_waypoints(waypoint_path)
        rospy.loginfo(
            "Scene motion policy: route=%s, speed=%.1fkm/h, "
            "minimum_capture_speed=%.1fkm/h, maximum_motion=%.1fm, "
            "stall_timeout=%.1fs, adaptive_curve=%s",
            waypoint_path,
            self.args.speed_kph,
            self.scene_minimum_ego_speed_kph,
            self.args.maximum_motion_m,
            self.args.motion_stall_timeout_sec,
            self.args.adaptive_curve_control,
        )
    def capture_identifier(self, scene, frame_index):
        if self.dataset_id and self.environment_id:
            return "{}_{}_f{:02d}_{}".format(
                scene["id"],
                self.environment_id,
                frame_index,
                self.run_id,
            )
        return "{}_{}_f{:02d}".format(
            self.run_id, scene["id"], frame_index
        )

    def wait_for_connections(
        self,
        timeout_sec=None,
        context="startup",
        include_motion_interface=True,
    ):
        required = (("SaveSensorData", self.capture_pub),)
        if self.args.scenario_load_transport == "ros":
            required = (("ScenarioLoad", self.scenario_pub),) + required
        if include_motion_interface:
            if self.uses_ctrl_cmd():
                required += (("CtrlCmd", self.control_pub),)
            elif self.args.motion_mode == "teleport":
                required += (("EgoSetting", self.ego_setting_pub),)
        if timeout_sec is None:
            timeout_sec = self.args.network_reconnect_timeout_sec
        started = time.monotonic()
        deadline = time.monotonic() + timeout_sec
        missing = []
        warned = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            missing = [
                name
                for name, publisher in required
                if publisher.get_num_connections() == 0
            ]
            if not missing:
                rospy.loginfo(
                    "MORAI ROS connections ready (%s): %s",
                    context,
                    ", ".join(name for name, _publisher in required),
                )
                return
            if not warned and time.monotonic() - started >= 1.0:
                network_hint = (
                    "Simulator Network"
                    if "EgoSetting" in missing
                    else "Ego Network"
                )
                rospy.logwarn(
                    "Waiting for MORAI ROS connections (%s): %s. "
                    "Reconnect %s in MORAI if needed.",
                    context,
                    ", ".join(missing),
                    network_hint,
                )
                warned = True
            time.sleep(0.1)
        hint = ""
        if "EgoSetting" in missing:
            hint = (
                " MORAI Network Settings > Simulator Network > ROS에서 "
                "SaveFile/Network/26.R1.H3/ros.json을 Load한 뒤 Connect하고, "
                "`rostopic info /ego_setting`에 /rosbridge_websocket "
                "subscriber가 나타나는지 확인하세요. ScenarioLoad의 "
                "network sidecar는 Ego Network만 복구하며 Simulator "
                "Network의 Connect 상태를 켜지 않습니다."
            )
        elif "CtrlCmd" in missing:
            hint = (
                " MORAI Network Settings에서 Ego Network의 ROS 연결을 "
                "확인하세요. rolling-grouped에 필요한 /ctrl_cmd와 "
                "/Service_MoraiEventCmd는 scenario EgoNetwork sidecar의 "
                "UNIQUEID=0에 포함됩니다. Simulator Network는 필요하지 "
                "않습니다."
            )
        if rospy.is_shutdown():
            raise InterruptedError("ROS shutdown requested")
        raise RuntimeError(
            "MORAI subscriber connection missing after {} ({:.1f}s): {}.{}".format(
                context, timeout_sec, ", ".join(missing), hint
            )
        )

    def uses_ctrl_cmd(self):
        return self.args.motion_mode in (
            "route-follow",
            "rolling-safe",
            "rolling-fast",
            "rolling-grouped",
            "timed-grouped",
        )

    def publish_scenario_load(self, scene, file_name):
        message = ScenarioLoad()
        message.file_name = file_name
        # MORAI applies the individual Ego/NPC/pedestrian/obstacle load flags
        # when delete_all is false. With true, the observed result is that Ego
        # is restored but surrounding actors are removed.
        message.delete_all = False
        preserve_ego_and_network = self.args.motion_mode == "teleport"
        message.load_network_connection_data = not preserve_ego_and_network
        message.load_ego_vehicle_data = not preserve_ego_and_network
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
        if self.args.scenario_load_transport == "udp":
            request = {
                "delete_all": bool(message.delete_all),
                "load_network_connection_data": bool(
                    message.load_network_connection_data
                ),
                "load_ego_vehicle_data": bool(
                    message.load_ego_vehicle_data
                ),
                "load_surrounding_vehicle_data": bool(
                    message.load_surrounding_vehicle_data
                ),
                "load_pedestrian_data": bool(message.load_pedestrian_data),
                "load_obstacle_data": bool(
                    getattr(
                        message,
                        "load_obstacle_data",
                        getattr(message, "load_object_data", False),
                    )
                ),
                "set_pause": bool(message.set_pause),
            }
            payload = build_scenario_load_payload(file_name, request)
            sent = self.scenario_udp_socket.sendto(
                payload,
                (
                    self.args.scenario_load_udp_host,
                    self.args.scenario_load_udp_port,
                ),
            )
            if sent != len(payload):
                raise RuntimeError(
                    "ScenarioLoad UDP payload partially sent: {}/{}".format(
                        sent, len(payload)
                    )
                )
            rospy.loginfo(
                "ScenarioLoad UDP sent: %s -> %s:%d "
                "(delete_all=false, load_network_connection_data=%s, "
                "load_ego_vehicle_data=%s)",
                file_name,
                self.args.scenario_load_udp_host,
                self.args.scenario_load_udp_port,
                str(message.load_network_connection_data).lower(),
                str(message.load_ego_vehicle_data).lower(),
            )
        else:
            self.scenario_pub.publish(message)
            rospy.loginfo(
                "ScenarioLoad ROS published: %s (delete_all=false, "
                "load_network_connection_data=%s, load_ego_vehicle_data=%s)",
                file_name,
                str(message.load_network_connection_data).lower(),
                str(message.load_ego_vehicle_data).lower(),
            )
        # The message is delivered before MORAI tears down and recreates its
        # rosbridge clients. Use wall time because /clock can reset here.
        time.sleep(0.2)
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

        if (
            self.args.motion_mode != "teleport"
            and scene.get("ego_readiness_policy") == "route_aligned"
        ):
            ego = snapshot["ego"]
            nearest = self.nearest_waypoint_index(
                (ego.position.x, ego.position.y),
                hint=scene.get("anchor_index"),
                heading_deg=ego.heading,
            )
            route_error = distance_xy(
                (ego.position.x, ego.position.y), self.waypoints[nearest]
            )
            if route_error > self.args.route_deviation_limit_m:
                return False, "Ego route error {:.2f} m".format(route_error)
        elif self.args.motion_mode != "teleport":
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
        dynamic_policy = scene.get("dynamic_actor_policy")
        if dynamic_policy == "observe_after_warmup":
            # Spawn-point NPCs may appear before the post-load verification
            # callback. Their runtime count is intentionally not fixed.
            comparable = dict(actual)
            comparable["npc"] = expected["npc"]
            counts_match = comparable == expected
        else:
            counts_match = actual == expected
        if not counts_match:
            return False, "object counts {} != {}".format(actual, expected)
        return True, "ready"

    def wait_for_scene(self, scene, issued):
        deadline = time.monotonic() + self.args.load_timeout_sec
        reason = "no data"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            ready, reason = self.scene_is_ready(scene, issued)
            if ready:
                time.sleep(self.args.load_settle_sec)
                ready, reason = self.scene_is_ready(scene, issued)
                if not ready:
                    continue
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
            parts.append(
                "actual object identities={}".format(
                    self.object_observation(objects)["identities"]
                )
            )
        parts.append("expected objects={}".format(scene["expected_counts"]))
        return "; ".join(parts)

    @staticmethod
    def object_observation(objects):
        if objects is None:
            return None

        def identities(items):
            return [
                {
                    "unique_id": int(item.unique_id),
                    "type": int(item.type),
                    "name": str(item.name),
                }
                for item in items
            ]

        return {
            "counts": {
                "npc": int(objects.num_of_npcs),
                "pedestrian": int(objects.num_of_pedestrian),
                "obstacle": int(objects.num_of_obstacle),
            },
            "identities": {
                "npc": identities(objects.npc_list),
                "pedestrian": identities(objects.pedestrian_list),
                "obstacle": identities(objects.obstacle_list),
            },
        }

    def load_and_verify_scene(self, scene):
        # The currently loaded legacy sidecar may not yet contain a newly
        # introduced motion interface such as /ego_setting. ScenarioLoad and
        # SaveSensorData are sufficient to install the new sidecar first.
        last_error = None
        for attempt in range(1, self.args.scenario_load_attempts + 1):
            self.wait_for_connections(
                context="before scenario load",
                include_motion_interface=self.args.motion_mode == "teleport",
            )
            issued = self.publish_scenario_load(scene, scene["file_name"])
            if self.args.motion_mode == "teleport":
                rospy.loginfo(
                    "Waiting for actor-only load while preserving Ego and "
                    "ROS networks after %s",
                    scene["file_name"],
                )
            else:
                rospy.loginfo(
                    "Waiting for MORAI rosbridge clients to reconnect after %s",
                    scene["file_name"],
                )
            self.wait_for_connections(
                context="scenario {} network recovery".format(
                    scene["file_name"]
                )
            )
            try:
                # Validate the new scene before calling a Simulator Network
                # service. Immediately after ScenarioLoad, rosbridge may still
                # expose the old service briefly and return handler=None.
                self.wait_for_scene(scene, issued)
                last_error = None
                break
            except RuntimeError as error:
                last_error = error
                if attempt == self.args.scenario_load_attempts:
                    raise
                rospy.logwarn(
                    "ScenarioLoad attempt %d/%d failed for %s: %s; retrying",
                    attempt,
                    self.args.scenario_load_attempts,
                    scene["file_name"],
                    error,
                )
                time.sleep(0.5)
        if last_error is not None:
            raise last_error
        control_prepared = False
        if self.uses_ctrl_cmd():
            # Once the new scene identity is confirmed, enter external mode.
            # Do not publish the brake command after this service call: the
            # personal MORAI launcher can retain that brake state even after
            # velocity-mode CtrlCmd messages begin, leaving Ego stationary.
            # The loaded scenario already starts at zero velocity and the
            # motion routine immediately starts publishing its drive command.
            control_error = None
            for attempt in range(1, 4):
                try:
                    self.select_external_control_mode()
                    control_error = None
                    break
                except RuntimeError as error:
                    control_error = error
                    if attempt == 3:
                        raise
                    rospy.logwarn(
                        "Control-mode attempt %d/3 failed after %s: %s; "
                        "waiting for Simulator Network",
                        attempt,
                        scene["file_name"],
                        error,
                    )
                    time.sleep(1.0)
            if control_error is not None:
                raise control_error
            control_prepared = True
        return control_prepared

    def select_external_control_mode(self):
        if self.args.skip_control_mode_service:
            rospy.logwarn(
                "Control-mode service skipped; verify AV-ExternalCtrl in MORAI UI"
            )
            return
        rospy.loginfo(
            "Waiting for MORAI control-mode service: %s",
            self.args.control_mode_service,
        )
        try:
            rospy.wait_for_service(
                self.args.control_mode_service,
                timeout=self.args.network_reconnect_timeout_sec,
            )
        except rospy.ROSException as error:
            raise RuntimeError(
                "{} is unavailable. Select AV-ExternalCtrl manually and "
                "rerun with --skip-control-mode-service, or enable the MORAI "
                "event service.".format(self.args.control_mode_service)
            ) from error

        request = MoraiEventCmdSrvRequest()
        # EventInfo option is a bit mask: 1=control mode, 2=gear.
        request.request.option = 1 | 2
        request.request.ctrl_mode = 3
        request.request.gear = 4
        request.request.lamps.turnSignal = 3
        request.request.lamps.emergencySignal = 0
        request.request.set_pause = False
        result_queue = queue.Queue(maxsize=1)

        def invoke_service():
            try:
                result_queue.put(("response", self.control_mode_client(request)))
            except BaseException as error:  # Preserve rospy service errors.
                result_queue.put(("error", error))

        worker = threading.Thread(target=invoke_service)
        worker.daemon = True
        worker.start()
        try:
            result_type, result = result_queue.get(
                timeout=self.args.control_mode_timeout_sec
            )
        except queue.Empty as error:
            raise RuntimeError(
                "{} did not respond within {:.1f}s. MORAI Ego Network or "
                "rosbridge service is stale; restart/reconnect before "
                "continuing the batch.".format(
                    self.args.control_mode_service,
                    self.args.control_mode_timeout_sec,
                )
            ) from error
        if result_type == "error":
            raise RuntimeError(
                "{} call failed: {}".format(
                    self.args.control_mode_service, result
                )
            ) from result
        response = result
        rospy.loginfo(
            "MORAI control response: ctrl_mode=%d, gear=%d",
            response.response.ctrl_mode,
            response.response.gear,
        )
        if response.response.ctrl_mode not in (3, 4, 5):
            raise RuntimeError(
                "MORAI did not enter an external control mode: {}".format(
                    response.response.ctrl_mode
                )
            )
        if response.response.gear != 4:
            raise RuntimeError(
                "MORAI did not enter drive gear: {}".format(
                    response.response.gear
                )
            )
        deadline = time.monotonic() + self.args.ego_heartbeat_timeout_sec
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            snapshot = self.state.snapshot()
            if (
                snapshot["ego"] is not None
                and time.monotonic() - snapshot["ego_wall_time"] <= 1.0
            ):
                return
            time.sleep(0.05)
        raise RuntimeError(
            "/Ego_topic heartbeat did not recover within {:.1f}s after "
            "control-mode selection".format(
                self.args.ego_heartbeat_timeout_sec
            )
        )

    def trigger_capture(self, scene, frame_index):
        self.wait_for_connections(context="before sensor capture")
        capture_id = self.capture_identifier(scene, frame_index)
        source_root = self.args.morai_sensor_data_dir
        before = recursive_files(source_root)
        triggered_unix = time.time()
        started = time.monotonic()
        self.publish_capture_request()

        deadline = time.monotonic() + self.args.capture_timeout_sec
        new_files = set()
        last_files = set()
        stable_since = None
        observed_sensors = set()
        missing_sensors = set(self.args.required_sensor_dirs)
        expected = scene["expected_counts"]
        require_object_labels = sum(expected.values()) > 0
        observed_artifacts = {}
        required_artifacts = {}
        missing_artifacts = {}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            new_files = recursive_files(source_root) - before
            observed_sensors = observed_sensor_directories(
                source_root, new_files
            )
            missing_sensors = (
                set(self.args.required_sensor_dirs) - observed_sensors
            )
            (
                observed_artifacts,
                required_artifacts,
                missing_artifacts,
            ) = capture_artifact_status(
                source_root,
                new_files,
                self.args.required_sensor_dirs,
                require_object_labels,
            )
            if new_files != last_files:
                last_files = set(new_files)
                stable_since = time.monotonic()
            complete = (
                len(new_files) >= self.args.minimum_files_per_capture
                and not missing_sensors
                and not missing_artifacts
            )
            stable = (
                stable_since is not None
                and time.monotonic() - stable_since
                >= self.args.capture_stable_sec
            )
            if complete and stable:
                break
            time.sleep(0.1)

        verified = (
            len(new_files) >= self.args.minimum_files_per_capture
            and not missing_sensors
            and not missing_artifacts
            and stable_since is not None
            and time.monotonic() - stable_since
            >= self.args.capture_stable_sec
        )
        if not verified and not self.args.allow_unverified_capture:
            raise RuntimeError(
                "Capture {} is incomplete: files={}, observed_sensors={}, "
                "missing_sensors={}, missing_artifacts={}. Check Camera/LiDAR "
                "Capture Mode, file_dir support, and simulator permissions."
                .format(
                    capture_id,
                    len(new_files),
                    sorted(observed_sensors),
                    sorted(missing_sensors),
                    missing_artifacts,
                )
            )
        source_files = sorted(
            str(path.relative_to(source_root)) for path in new_files
        )
        relocated_files = []
        if verified:
            allow_empty_suffixes = (
                {"_Instance.txt", "_Instance_8Points.txt"}
                if not require_object_labels
                else set()
            )
            relocated_files = relocate_capture_files(
                source_root,
                self.args.capture_dir,
                capture_id,
                new_files,
                allow_empty_suffixes=allow_empty_suffixes,
            )
        completed_unix = time.time()
        duration_sec = time.monotonic() - started
        rospy.loginfo(
            "Capture %s: files=%d, sensors=%s, relocated=%d, "
            "verified=%s, duration=%.3fs",
            capture_id,
            len(new_files),
            ",".join(sorted(observed_sensors)),
            len(relocated_files),
            verified,
            duration_sec,
        )
        return {
            "capture_id": capture_id,
            "verified": verified,
            "triggered_unix": triggered_unix,
            "completed_unix": completed_unix,
            "duration_sec": duration_sec,
            "new_file_count": len(new_files),
            "observed_sensor_dirs": sorted(observed_sensors),
            "missing_sensor_dirs": sorted(missing_sensors),
            "require_object_labels": require_object_labels,
            "observed_artifacts": {
                sensor: sorted(suffixes)
                for sensor, suffixes in sorted(observed_artifacts.items())
            },
            "required_artifacts": {
                sensor: sorted(suffixes)
                for sensor, suffixes in sorted(required_artifacts.items())
            },
            "missing_artifacts": missing_artifacts,
            "morai_source_files": source_files,
            "new_files": sorted(
                str(path.relative_to(self.args.capture_dir))
                for path in relocated_files
            ),
        }

    def publish_capture_request(self):
        message = SaveSensorData()
        # The personal MORAI build writes cameras but omits 3D LiDAR when
        # is_custom_file_name=true. Capture every sensor to the simulator's
        # default SensorData directory, then relocate the verified set.
        message.is_custom_file_name = False
        message.custom_file_name = ""
        message.file_dir = ""
        self.capture_pub.publish(message)

    def prime_timed_sensor_writers(self, scene):
        """Discard one warm-up snapshot after ScenarioLoad.

        The personal launcher can create a zero-byte LiDAR Instance label on
        the first SaveSensorData request of a newly loaded high-speed scene.
        Keeping that request outside the temporal clip prevents a known
        writer-initialization artifact without accepting an incomplete GT
        frame or shifting any frame metadata.
        """

        source_root = self.args.morai_sensor_data_dir
        before = recursive_files(source_root)
        self.wait_for_connections(context="before timed sensor prime")
        self.publish_capture_request()
        deadline = time.monotonic() + self.args.capture_timeout_sec
        last_files = set()
        stable_since = None
        new_files = set()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            new_files = recursive_files(source_root) - before
            if new_files != last_files:
                last_files = set(new_files)
                stable_since = time.monotonic()
            if (
                new_files
                and stable_since is not None
                and time.monotonic() - stable_since
                >= self.args.capture_stable_sec
            ):
                break
            time.sleep(0.1)
        if not new_files:
            raise RuntimeError("timed sensor priming produced no files")
        prime_root = self.args.capture_dir / "sensor_primes" / scene["id"]
        for source in sorted(new_files):
            relative = source.relative_to(source_root)
            destination = prime_root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(source), str(destination))
        rospy.loginfo(
            "Timed sensor writers primed for %s: files=%d",
            scene["id"],
            len(new_files),
        )

    def publish_stop(self):
        message = CtrlCmd()
        message.longlCmdType = 1
        message.accel = 0.0
        message.brake = 1.0
        set_ctrl_steering(
            message,
            0.0,
            self.args.personal_max_steering_angle_deg,
            self.args.personal_steering_sign,
        )
        message.velocity = 0.0
        message.acceleration = 0.0
        for _ in range(10):
            self.control_pub.publish(message)
            time.sleep(0.05)

    def nearest_waypoint_index(
        self,
        position,
        hint=None,
        window=100,
        heading_deg=None,
    ):
        if hint is None:
            candidates = range(len(self.waypoints))
        else:
            # Ego only moves forward during capture.  A small backward window
            # tolerates localization jitter without allowing a nearby branch
            # at an intersection to pull the controller far backwards.
            start = max(0, hint - 8)
            stop = min(len(self.waypoints), hint + window)
            candidates = range(start, stop)

        def score(index):
            distance_squared = (
                (self.waypoints[index][0] - position[0]) ** 2
                + (self.waypoints[index][1] - position[1]) ** 2
            )
            if heading_deg is None:
                return distance_squared
            before = self.waypoints[max(0, index - 1)]
            after = self.waypoints[min(len(self.waypoints) - 1, index + 1)]
            route_heading = math.degrees(
                math.atan2(after[1] - before[1], after[0] - before[0])
            )
            heading_error = abs(
                math.degrees(
                    wrapped_angle(
                        math.radians(route_heading - heading_deg)
                    )
                )
            )
            # A 45-degree heading mismatch costs one metre of position error.
            return distance_squared + (heading_error / 45.0) ** 2

        return min(
            candidates,
            key=score,
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

    def route_coordinate_m(self, position, nearest):
        """Project an XY position onto the local route and return arc length."""

        candidates = []
        for before_index, after_index in (
            (nearest - 1, nearest),
            (nearest, nearest + 1),
        ):
            if before_index < 0 or after_index >= len(self.waypoints):
                continue
            before = self.waypoints[before_index]
            after = self.waypoints[after_index]
            dx = after[0] - before[0]
            dy = after[1] - before[1]
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                continue
            ratio = (
                (position[0] - before[0]) * dx
                + (position[1] - before[1]) * dy
            ) / length_squared
            ratio = max(0.0, min(1.0, ratio))
            projected = (
                before[0] + ratio * dx,
                before[1] + ratio * dy,
            )
            error = distance_xy(position, projected)
            coordinate = self.waypoint_arc_m[before_index] + ratio * math.sqrt(
                length_squared
            )
            candidates.append((error, coordinate))
        if not candidates:
            return self.waypoint_arc_m[nearest]
        return min(candidates, key=lambda item: item[0])[1]

    def curve_control_limits(self, nearest):
        """Return speed, lookahead, steering limit, and route turn severity."""

        if not self.args.adaptive_curve_control:
            return (
                self.args.speed_kph,
                self.args.lookahead_m,
                self.args.maximum_steering_rad,
                0.0,
            )

        start_before = self.waypoints[max(0, nearest - 1)]
        start_after = self.waypoints[min(len(self.waypoints) - 1, nearest + 1)]
        end_index = self.lookahead_index(
            nearest, self.args.curve_control_window_m
        )
        end_before = self.waypoints[max(0, end_index - 1)]
        end_after = self.waypoints[
            min(len(self.waypoints) - 1, end_index + 1)
        ]
        start_heading = math.atan2(
            start_after[1] - start_before[1],
            start_after[0] - start_before[0],
        )
        end_heading = math.atan2(
            end_after[1] - end_before[1],
            end_after[0] - end_before[0],
        )
        turn_deg = abs(
            math.degrees(wrapped_angle(end_heading - start_heading))
        )
        severity = min(
            1.0,
            turn_deg / self.args.curve_control_full_turn_deg,
        )

        speed = self.args.speed_kph - severity * (
            self.args.speed_kph - self.args.minimum_curve_speed_kph
        )
        lookahead = self.args.lookahead_m - severity * (
            self.args.lookahead_m - self.args.minimum_curve_lookahead_m
        )
        steering_limit = self.args.maximum_steering_rad + severity * (
            self.args.curve_maximum_steering_rad
            - self.args.maximum_steering_rad
        )
        return speed, lookahead, steering_limit, turn_deg

    def teleport_ego(self, scene, target_position, target_yaw_deg):
        snapshot = self.state.snapshot()
        if snapshot["ego"] is None:
            raise RuntimeError("Ego telemetry unavailable before teleport")

        command = make_multi_ego_teleport_message(
            snapshot["ego"], target_position, target_yaw_deg
        )
        issued = time.monotonic()
        self.ego_setting_pub.publish(command)
        rospy.loginfo(
            "EgoSetting published: id=%d, target=(%.3f, %.3f, %.3f), "
            "yaw=%.3f deg",
            command.ego_index[0],
            target_position[0],
            target_position[1],
            target_position[2],
            target_yaw_deg,
        )

        # Some MORAI versions rebuild Ego ROS clients when applying
        # MultiEgoSetting. Wait for both publisher reconnection and fresh Ego
        # telemetry rather than assuming the command was accepted.
        time.sleep(0.2)
        self.wait_for_connections(
            timeout_sec=self.args.teleport_timeout_sec,
            context="Ego teleport recovery",
        )
        deadline = time.monotonic() + self.args.teleport_timeout_sec
        last_reason = "waiting for fresh Ego telemetry"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            current = self.state.snapshot()
            ego = current["ego"]
            if ego is None or current["ego_wall_time"] <= issued:
                time.sleep(0.05)
                continue

            xy_error = math.hypot(
                ego.position.x - target_position[0],
                ego.position.y - target_position[1],
            )
            z_error = abs(ego.position.z - target_position[2])
            heading_error = abs(
                math.degrees(
                    wrapped_angle(
                        math.radians(ego.heading - target_yaw_deg)
                    )
                )
            )
            objects = current["objects"]
            expected = scene["expected_counts"]
            object_counts = None
            if objects is not None:
                object_counts = {
                    "npc": objects.num_of_npcs,
                    "pedestrian": objects.num_of_pedestrian,
                    "obstacle": objects.num_of_obstacle,
                }

            if (
                xy_error
                <= self.args.teleport_position_tolerance_m
                and z_error <= self.args.teleport_z_tolerance_m
                and heading_error <= 5.0
                and object_counts == expected
            ):
                time.sleep(self.args.teleport_settle_sec)
                rospy.loginfo(
                    "Ego teleport verified: XY error=%.3f m, "
                    "Z error=%.3f m, heading error=%.3f deg",
                    xy_error,
                    z_error,
                    heading_error,
                )
                return
            last_reason = (
                "XY error={:.3f} m, Z error={:.3f} m, "
                "heading error={:.3f} deg, "
                "objects={} expected={}".format(
                    xy_error,
                    z_error,
                    heading_error,
                    object_counts,
                    expected,
                )
            )
            time.sleep(0.05)

        raise RuntimeError(
            "Ego teleport was not verified within {:.1f}s: {}".format(
                self.args.teleport_timeout_sec, last_reason
            )
        )

    def teleport_and_capture(self, scene, capture_count):
        expected = scene["expected_ego"]
        initial_position = (
            expected["x"],
            expected["y"],
            expected["z"],
        )
        self.teleport_ego(scene, initial_position, expected["yaw"])
        captures = [self.trigger_capture(scene, 0)]
        if capture_count == 1:
            return captures

        snapshot = self.state.snapshot()
        if snapshot["ego"] is None:
            raise RuntimeError("Ego telemetry unavailable before teleport run")
        start = (
            snapshot["ego"].position.x,
            snapshot["ego"].position.y,
            snapshot["ego"].position.z,
        )
        start_index = self.nearest_waypoint_index(
            start, hint=scene["anchor_index"]
        )

        for frame_index in range(1, capture_count):
            route_distance = self.args.capture_spacing_m * frame_index
            if route_distance > self.args.maximum_motion_m:
                raise RuntimeError(
                    "teleport distance {:.2f} m exceeds safety limit {:.2f} m".format(
                        route_distance, self.args.maximum_motion_m
                    )
                )
            target, yaw_deg, _segment_index = route_pose_at_distance(
                self.waypoints, start_index, route_distance
            )
            self.teleport_ego(scene, target, yaw_deg)
            captures.append(self.trigger_capture(scene, frame_index))
        return captures

    def collision_observations_after(self, issued):
        """Return MORAI collision participants reported after motion starts."""

        snapshot = self.state.snapshot()
        if (
            snapshot["collision"] is None
            or snapshot["collision_wall_time"] <= issued
        ):
            return []

        observations = []
        for field_name in ("collision_object", "collision_objecta"):
            for item in getattr(snapshot["collision"], field_name, []):
                if item.type == -1:
                    continue
                observations.append(
                    {
                        "source": field_name,
                        "unique_id": int(item.unique_id),
                        "type": int(item.type),
                        "name": str(item.name),
                        "x": round(float(item.position.x), 3),
                        "y": round(float(item.position.y), 3),
                        "z": round(float(item.position.z), 3),
                    }
                )
        return observations

    def drive_and_capture(
        self, scene, capture_count, stop_for_capture=True
    ):
        # Keep Ego stationary while MORAI writes the multi-sensor frame.
        # Otherwise the most recent velocity command remains active during the
        # roughly 1--2 s blocking capture and skips several spacing intervals.
        self.publish_stop()
        captures = [self.trigger_capture(scene, 0)]
        if capture_count == 1:
            return captures

        snapshot = self.state.snapshot()
        ego = snapshot["ego"]
        nearest_hint = scene["anchor_index"]
        start = (ego.position.x, ego.position.y)
        nearest_hint = self.nearest_waypoint_index(
            start,
            hint=nearest_hint,
            heading_deg=ego.heading,
        )
        start_route_m = self.route_coordinate_m(start, nearest_hint)
        last_capture_route_m = start_route_m
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
                collision = self.collision_observations_after(issued)
                if collision:
                    raise RuntimeError(
                        "collision reported during PoC motion: {}".format(
                            collision
                        )
                    )

                nearest_hint = self.nearest_waypoint_index(
                    position,
                    hint=nearest_hint,
                    heading_deg=ego.heading,
                )
                route_position_m = self.route_coordinate_m(
                    position, nearest_hint
                )
                travelled = max(0.0, route_position_m - start_route_m)
                if travelled > self.args.maximum_motion_m:
                    raise RuntimeError(
                        "maximum route motion reached before all captures: "
                        "{:.2f} m".format(travelled)
                    )
                route_point = self.waypoints[nearest_hint]
                route_error = distance_xy(position, route_point)
                if route_error > self.args.route_deviation_limit_m:
                    raise RuntimeError(
                        "route deviation {:.2f} m exceeds safety limit".format(
                            route_error
                        )
                    )

                (
                    command_speed_kph,
                    command_lookahead_m,
                    command_steering_limit,
                    route_turn_deg,
                ) = self.curve_control_limits(nearest_hint)
                target_index = self.lookahead_index(
                    nearest_hint, command_lookahead_m
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
                    -command_steering_limit,
                    min(command_steering_limit, steering),
                )

                command = CtrlCmd()
                command.longlCmdType = 2
                command.velocity = command_speed_kph
                set_ctrl_steering(
                    command,
                    steering,
                    self.args.personal_max_steering_angle_deg,
                    self.args.personal_steering_sign,
                )
                command.accel = 0.0
                command.brake = 0.0
                command.acceleration = 0.0
                self.control_pub.publish(command)

                if (
                    route_position_m - last_capture_route_m
                    >= self.args.capture_spacing_m
                ):
                    if stop_for_capture:
                        self.publish_stop()
                        stopped_snapshot = self.state.snapshot()
                        if stopped_snapshot["ego"] is None:
                            raise RuntimeError(
                                "Ego telemetry disappeared before sensor capture"
                            )
                    else:
                        rospy.logwarn(
                            "Rolling-safe capture starts while Ego is moving; "
                            "the last control command remains active during "
                            "blocking file verification"
                        )
                    captures.append(
                        self.trigger_capture(scene, len(captures))
                    )
                    last_capture_route_m = route_position_m
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

    def dynamic_actor_ready(self, scene, objects):
        if objects is None:
            return False
        family = scene.get("mission_family", "")
        if family.endswith("vehicle"):
            return objects.num_of_npcs > 0
        if family == "crosswalk_pedestrian":
            return objects.num_of_pedestrian > 0
        if family.endswith("obstacle"):
            return objects.num_of_obstacle > 0
        return True

    def wait_for_dynamic_actor(self, scene):
        deadline = time.monotonic() + self.args.actor_ready_timeout_sec
        initial_pedestrian_positions = {}
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            snapshot = self.state.snapshot()
            objects = snapshot["objects"]
            present = self.dynamic_actor_ready(scene, objects)
            moving = True
            if present and scene.get("mission_family") == "crosswalk_pedestrian":
                moving = False
                for actor in objects.pedestrian_list:
                    key = int(actor.unique_id)
                    position = (float(actor.position.x), float(actor.position.y))
                    initial = initial_pedestrian_positions.setdefault(
                        key, position
                    )
                    speed = math.hypot(
                        float(actor.velocity.x), float(actor.velocity.y)
                    )
                    displacement = distance_xy(position, initial)
                    if speed > 0.05 or displacement > 0.02:
                        moving = True
                        break
            if present and moving:
                rospy.loginfo(
                    "Dynamic actor ready for %s (%s)",
                    scene["id"],
                    scene.get("mission_family", "unknown"),
                )
                return
            time.sleep(0.05)
        raise RuntimeError(
            "dynamic actor did not become ready and moving within {:.1f}s "
            "for {}".format(
                self.args.actor_ready_timeout_sec,
                scene.get("mission_family", "unknown"),
            )
        )

    def finalize_timed_grouped_captures(self, scene, before, triggers):
        """Verify, relocate, and annotate one timed SaveSensorData clip."""

        source_root = self.args.morai_sensor_data_dir
        # Snapshot trigger-adjacent state before waiting for asynchronous
        # SaveSensorData writers. Otherwise a slow writer could let the
        # bounded live-history deque prune the exact clip interval.
        timing_histories = {}
        timing_path = None
        timing_availability = {}
        if not self.args.no_timing_sidecar:
            timing_histories = self.state.timing_window(
                triggers[0]["triggered_monotonic"],
                triggers[-1]["triggered_monotonic"],
                self.args.timing_window_padding_sec,
            )
            bind_trigger_timing(triggers, timing_histories)
            timing_availability = timing_summary(
                timing_histories, self.timing_topics
            )
        drain_started = time.monotonic()
        deadline = drain_started + self.args.capture_timeout_sec
        last_files = set()
        stable_since = None
        new_files = set()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            new_files = recursive_files(source_root) - before
            if new_files != last_files:
                last_files = set(new_files)
                stable_since = time.monotonic()
            if (
                new_files
                and stable_since is not None
                and time.monotonic() - stable_since
                >= self.args.capture_stable_sec
            ):
                break
            time.sleep(0.1)

        timestamp_groups, unrecognized = group_capture_files_by_timestamp(
            new_files
        )
        complete_groups = []
        incomplete_groups = {}
        # Keep object-label artifacts mandatory, but do not require a row in
        # the text label. A dynamic actor can legitimately be outside the
        # LiDAR field of view (or fully occluded) at an individual keyframe.
        # The fixed-size Instance/Semantic binary payloads remain mandatory,
        # so a missing sensor write is still rejected.
        require_object_labels = True
        for timestamp_stem, group_files in sorted(timestamp_groups.items()):
            observed, required, missing = capture_artifact_status(
                source_root,
                group_files,
                self.args.required_sensor_dirs,
                require_object_labels,
            )
            observed_sensors = observed_sensor_directories(
                source_root, group_files
            )
            missing_sensors = (
                set(self.args.required_sensor_dirs) - observed_sensors
            )
            if missing_sensors or missing:
                incomplete_groups[timestamp_stem] = {
                    "file_count": len(group_files),
                    "missing_sensor_dirs": sorted(missing_sensors),
                    "missing_artifacts": missing,
                }
                continue
            complete_groups.append(
                (
                    timestamp_stem,
                    group_files,
                    observed_sensors,
                    observed,
                    required,
                )
            )

        if (
            len(complete_groups) != len(triggers)
            or incomplete_groups
            or unrecognized
        ):
            rejected_root = (
                self.args.capture_dir / "timed_grouped_rejected" / scene["id"]
            )
            for source in sorted(new_files):
                relative = source.relative_to(source_root)
                destination = rejected_root / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(source), str(destination))
            raise RuntimeError(
                "timed-grouped produced {} complete groups for {} triggers; "
                "incomplete_groups={}, unrecognized_files={}. Raw files: {}"
                .format(
                    len(complete_groups),
                    len(triggers),
                    incomplete_groups,
                    [str(path.relative_to(source_root)) for path in unrecognized],
                    rejected_root,
                )
            )

        completed_unix = time.time()
        relocated_count = 0
        for index, group in enumerate(complete_groups):
            timestamp_stem, group_files, sensors, observed, required = group
            trigger = triggers[index]
            destinations = relocate_capture_files(
                source_root,
                self.args.capture_dir,
                trigger["capture_id"],
                group_files,
                allow_empty_suffixes=(
                    "_Instance.txt",
                    "_Instance_8Points.txt",
                ),
            )
            relocated_count += len(destinations)
            trigger.update(
                {
                    "verified": True,
                    "completed_unix": completed_unix,
                    "morai_timestamp_stem": timestamp_stem,
                    "new_file_count": len(group_files),
                    "observed_sensor_dirs": sorted(sensors),
                    "observed_artifacts": {
                        sensor: sorted(suffixes)
                        for sensor, suffixes in sorted(observed.items())
                    },
                    "required_artifacts": {
                        sensor: sorted(suffixes)
                        for sensor, suffixes in sorted(required.items())
                    },
                    "sensor_files": sorted(
                        str(path.relative_to(self.args.capture_dir))
                        for path in destinations
                    ),
                }
            )

        enrich_derived_motion(triggers)
        metadata_dir = self.args.capture_dir / "dynamic_metadata"
        metadata_dir.mkdir(parents=True, exist_ok=True)
        if not self.args.no_timing_sidecar:
            timing_path = metadata_dir / "{}.timing.jsonl".format(scene["id"])
            timing_path.write_text(
                "".join(
                    json.dumps(sample, ensure_ascii=False, separators=(",", ":"))
                    + "\n"
                    for stream in sorted(timing_histories)
                    for sample in timing_histories[stream]
                ),
                encoding="utf-8",
            )
        frames_path = metadata_dir / "{}.frames.jsonl".format(scene["id"])
        frames_path.write_text(
            "".join(
                json.dumps(frame, ensure_ascii=False, separators=(",", ":"))
                + "\n"
                for frame in triggers
            ),
            encoding="utf-8",
        )
        clip_document = {
            "schema_version": 2,
            "dataset_id": self.dataset_id,
            "clip_id": scene["id"],
            "scene_id": scene["id"],
            "scenario_stem": scene["file_name"],
            "mission_family": scene.get("mission_family"),
            "split": scene.get("split"),
            "environment": self.environment,
            "capture_format": "morai_save_sensor_data_timed_grouped",
            "capture_period_sec": self.args.capture_period_sec,
            "capture_motion_arm_m": self.args.capture_motion_arm_m,
            "minimum_capture_progress_m": self.args.minimum_capture_progress_m,
            "motion_stall_timeout_sec": self.args.motion_stall_timeout_sec,
            "frame_count": len(triggers),
            "frames_jsonl": str(frames_path.relative_to(self.args.capture_dir)),
            "timing_sidecar": (
                str(timing_path.relative_to(self.args.capture_dir))
                if timing_path is not None
                else None
            ),
            "timing_streams": timing_availability,
            "timing_notes": {
                "sensor_payload_source": "MORAI SaveSensorData files",
                "sidecar_payload_policy": "headers_and_state_only",
                "missing_sensor_topics_are_fatal": False,
                "capture_mode_limitation": (
                    "MORAI Capture Mode may not publish camera/LiDAR ROS topics; "
                    "missing streams are recorded as unavailable."
                ),
            },
            "sensor_config": self.manifest.get("sensor_config"),
            "sensor_config_sha256": self.manifest.get(
                "sensor_config_sha256"
            ),
            "coordinate_frames": {
                "authoritative_pose": "morai_map",
                "relative_pose": "ego_at_capture_request",
                "raw_velocity": "MORAI message axes, unmodified",
                "derived_velocity": "finite difference in morai_map",
            },
            "source_scene_metadata": scene.get("metadata"),
        }
        clip_path = metadata_dir / "{}.clip.json".format(scene["id"])
        write_json_atomic(clip_path, clip_document)

        span = 0.0
        rate = 0.0
        if len(triggers) > 1:
            span = triggers[-1]["triggered_unix"] - triggers[0]["triggered_unix"]
            if span > 0.0:
                rate = (len(triggers) - 1) / span
        metrics = {
            "experimental": False,
            "quality_verified": True,
            "trigger_count": len(triggers),
            "trigger_span_sec": span,
            "trigger_rate_hz": rate,
            "capture_period_sec": self.args.capture_period_sec,
            "capture_motion_arm_m": self.args.capture_motion_arm_m,
            "minimum_capture_progress_m": self.args.minimum_capture_progress_m,
            "complete_timestamp_group_count": len(complete_groups),
            "relocated_file_count": relocated_count,
            "frames_metadata": str(frames_path.relative_to(self.args.capture_dir)),
            "clip_metadata": str(clip_path.relative_to(self.args.capture_dir)),
            "timing_metadata": (
                str(timing_path.relative_to(self.args.capture_dir))
                if timing_path is not None
                else None
            ),
            "timing_streams": timing_availability,
            "drain_duration_sec": time.monotonic() - drain_started,
        }
        compact = [
            {
                "capture_id": item["capture_id"],
                "frame_index": item["frame_index"],
                "triggered_unix": item["triggered_unix"],
                "morai_timestamp_stem": item["morai_timestamp_stem"],
                "verified": True,
                "new_file_count": item["new_file_count"],
                "metadata_jsonl": str(frames_path.relative_to(self.args.capture_dir)),
            }
            for item in triggers
        ]
        rospy.loginfo(
            "Timed-grouped complete: triggers=%d, frames=%d, span=%.3fs, "
            "rate=%.3fHz, files=%d",
            len(triggers),
            len(complete_groups),
            span,
            rate,
            relocated_count,
        )
        return compact, metrics

    def drive_and_trigger_timed_grouped(self, scene, capture_count):
        """Drive Ego and issue verified SaveSensorData keyframes by time."""

        self.wait_for_dynamic_actor(scene)
        if self.args.dynamic_warmup_sec > 0.0:
            rospy.loginfo(
                "Dynamic warm-up: %.2fs", self.args.dynamic_warmup_sec
            )
            time.sleep(self.args.dynamic_warmup_sec)
        if self.args.prime_capture_before_clip:
            # Priming waits for asynchronous writers to settle. Hold Ego in
            # place during that wait; otherwise a moving scenario can drift
            # without fresh steering commands before the control loop starts.
            self.publish_stop()
            self.prime_timed_sensor_writers(scene)
        source_root = self.args.morai_sensor_data_dir
        before = recursive_files(source_root)
        motion_started = time.monotonic()
        deadline = motion_started + self.args.motion_timeout_sec
        clip_started = None
        next_trigger = None
        last_capture_route_m = None
        triggers = []
        snapshot = self.state.snapshot()
        if snapshot["ego"] is None:
            raise RuntimeError("Ego telemetry unavailable before timed-grouped")
        position = (snapshot["ego"].position.x, snapshot["ego"].position.y)
        nearest_hint = self.nearest_waypoint_index(
            position,
            hint=scene["anchor_index"],
            heading_deg=snapshot["ego"].heading,
        )
        start_route_m = self.route_coordinate_m(position, nearest_hint)

        try:
            while (
                not rospy.is_shutdown()
                and len(triggers) < capture_count
                and time.monotonic() < deadline
            ):
                snapshot = self.state.snapshot()
                ego = snapshot["ego"]
                if ego is None or snapshot["objects"] is None:
                    raise RuntimeError("Ego/Object telemetry unavailable")
                collision = self.collision_observations_after(motion_started)
                if collision:
                    raise RuntimeError(
                        "collision reported during timed-grouped: {}".format(collision)
                    )
                position = (ego.position.x, ego.position.y)
                nearest_hint = self.nearest_waypoint_index(
                    position,
                    hint=nearest_hint,
                    heading_deg=ego.heading,
                )
                route_position_m = self.route_coordinate_m(position, nearest_hint)
                travelled = max(0.0, route_position_m - start_route_m)
                if travelled > self.args.maximum_motion_m:
                    raise RuntimeError(
                        "maximum route motion reached during timed-grouped: "
                        "{:.2f} m".format(travelled)
                    )
                route_error = distance_xy(position, self.waypoints[nearest_hint])
                if route_error > self.args.route_deviation_limit_m:
                    raise RuntimeError(
                        "route deviation {:.2f} m exceeds safety limit".format(
                            route_error
                        )
                    )
                speed, lookahead, steering_limit, turn = self.curve_control_limits(
                    nearest_hint
                )
                target = self.waypoints[
                    self.lookahead_index(nearest_hint, lookahead)
                ]
                target_heading = math.atan2(
                    target[1] - position[1], target[0] - position[0]
                )
                heading_error = wrapped_angle(
                    target_heading - math.radians(ego.heading)
                )
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
                steering = math.atan(
                    self.args.wheelbase_m
                    * 2.0
                    * math.sin(heading_error)
                    / target_distance
                )
                steering = max(-steering_limit, min(steering_limit, steering))
                command = CtrlCmd()
                command.longlCmdType = 2
                command.velocity = speed
                set_ctrl_steering(
                    command,
                    steering,
                    self.args.personal_max_steering_angle_deg,
                    self.args.personal_steering_sign,
                )
                command.accel = 0.0
                command.brake = 0.0
                command.acceleration = 0.0
                self.control_pub.publish(command)

                now = time.monotonic()
                if clip_started is None:
                    ego_speed_kph = 3.6 * math.sqrt(
                        ego.velocity.x * ego.velocity.x
                        + ego.velocity.y * ego.velocity.y
                        + ego.velocity.z * ego.velocity.z
                    )
                    motion_ready = (
                        travelled >= self.args.capture_motion_arm_m
                    )
                    speed_ready = (
                        ego_speed_kph >= self.scene_minimum_ego_speed_kph
                    )
                    if not motion_ready or not speed_ready:
                        if (
                            now - motion_started
                            > self.args.motion_stall_timeout_sec
                        ):
                            raise RuntimeError(
                                "Ego did not arm timed-grouped capture: "
                                "progress={:.3f}/{:.3f}m, "
                                "speed={:.2f}/{:.2f}km/h in {:.3f}s. "
                                "CtrlCmd is connected but may not be "
                                "applied by MORAI.".format(
                                    travelled,
                                    self.args.capture_motion_arm_m,
                                    ego_speed_kph,
                                    self.scene_minimum_ego_speed_kph,
                                    now - motion_started,
                                )
                            )
                        time.sleep(0.01)
                        continue
                    clip_started = now
                    next_trigger = now
                    rospy.loginfo(
                        "Timed-grouped motion armed after %.3fm at "
                        "%.2fkm/h in %.3fs",
                        travelled,
                        ego_speed_kph,
                        now - motion_started,
                    )

                progress_since_capture = (
                    travelled - last_capture_route_m
                    if last_capture_route_m is not None
                    else travelled
                )
                time_ready = now >= next_trigger
                progress_ready = (
                    progress_since_capture
                    >= self.args.minimum_capture_progress_m
                )
                if (
                    time_ready
                    and not progress_ready
                    and now - next_trigger
                    > self.args.motion_stall_timeout_sec
                ):
                    raise RuntimeError(
                        "Ego motion stalled before timed-grouped frame {}: "
                        "progress={:.3f}m, required={:.3f}m, waited={:.3f}s"
                        .format(
                            len(triggers) + 1,
                            progress_since_capture,
                            self.args.minimum_capture_progress_m,
                            now - next_trigger,
                        )
                    )
                if time_ready and progress_ready:
                    self.wait_for_connections(
                        context="before timed-grouped trigger"
                    )
                    trigger_unix = time.time()
                    objects = serialize_objects(
                        snapshot["objects"],
                        snapshot["objects_unix_time"],
                        snapshot["objects_wall_time"],
                    )
                    ego_state = serialize_ego(
                        ego,
                        snapshot["ego_unix_time"],
                        snapshot["ego_wall_time"],
                    )
                    self.publish_capture_request()
                    frame_index = len(triggers)
                    triggers.append(
                        {
                            "schema_version": 1,
                            "dataset_id": self.dataset_id,
                            "clip_id": scene["id"],
                            "scene_id": scene["id"],
                            "mission_family": scene.get("mission_family"),
                            "split": scene.get("split"),
                            "frame_index": frame_index,
                            "frame_id": "{}_f{:06d}".format(
                                scene["id"], frame_index
                            ),
                            "capture_id": self.capture_identifier(
                                scene, frame_index
                            ),
                            "triggered_unix": trigger_unix,
                            "triggered_monotonic": now,
                            "scheduled_monotonic": next_trigger,
                            "schedule_lateness_sec": now - next_trigger,
                            "trigger_distance_m": travelled,
                            "capture_progress_since_previous_m": (
                                progress_since_capture
                            ),
                            "route_turn_deg": turn,
                            "command_speed_kph": speed,
                            "command_lookahead_m": lookahead,
                            "ego": ego_state,
                            "objects": objects,
                            "quality": {
                                "ego_state_age_sec": max(
                                    0.0, trigger_unix - snapshot["ego_unix_time"]
                                ),
                                "object_state_age_sec": max(
                                    0.0,
                                    trigger_unix - snapshot["objects_unix_time"],
                                ),
                                "actor_present": self.dynamic_actor_ready(
                                    scene, snapshot["objects"]
                                ),
                            },
                            "verified": False,
                        }
                    )
                    rospy.loginfo(
                        "Timed-grouped trigger %d/%d: t=%.3fs, distance=%.3fm",
                        len(triggers),
                        capture_count,
                        now - clip_started,
                        travelled,
                    )
                    last_capture_route_m = travelled
                    next_trigger = clip_started + (
                        len(triggers) * self.args.capture_period_sec
                    )
                time.sleep(0.01)
        finally:
            self.publish_stop()

        if len(triggers) != capture_count:
            raise RuntimeError(
                "timed-grouped ended with {}/{} triggers".format(
                    len(triggers), capture_count
                )
            )
        return self.finalize_timed_grouped_captures(scene, before, triggers)

    def drive_and_trigger_burst(
        self,
        scene,
        capture_count,
        group_verified_frames=False,
    ):
        """Trigger captures by distance without waiting for each file set.

        This intentionally prioritizes trigger throughput over dataset
        completeness. It is an experimental benchmark, not a production
        collection mode.
        """

        source_root = self.args.morai_sensor_data_dir
        before = recursive_files(source_root)
        snapshot = self.state.snapshot()
        if snapshot["ego"] is None:
            raise RuntimeError("Ego telemetry unavailable before rolling-fast")
        start = (
            snapshot["ego"].position.x,
            snapshot["ego"].position.y,
        )
        nearest_hint = scene["anchor_index"]
        nearest_hint = self.nearest_waypoint_index(
            start,
            hint=nearest_hint,
            heading_deg=snapshot["ego"].heading,
        )
        start_route_m = self.route_coordinate_m(start, nearest_hint)
        issued = time.monotonic()
        deadline = issued + self.args.motion_timeout_sec
        triggers = []

        try:
            while (
                not rospy.is_shutdown()
                and len(triggers) < capture_count
                and time.monotonic() < deadline
            ):
                snapshot = self.state.snapshot()
                if snapshot["ego"] is None:
                    raise RuntimeError(
                        "Ego telemetry disappeared during rolling-fast"
                    )
                ego = snapshot["ego"]
                position = (ego.position.x, ego.position.y)
                collision = self.collision_observations_after(issued)
                if collision:
                    raise RuntimeError(
                        "collision reported during rolling-fast: {}".format(
                            collision
                        )
                    )

                nearest_hint = self.nearest_waypoint_index(
                    position,
                    hint=nearest_hint,
                    heading_deg=ego.heading,
                )
                route_position_m = self.route_coordinate_m(
                    position, nearest_hint
                )
                travelled = max(0.0, route_position_m - start_route_m)
                if travelled > self.args.maximum_motion_m:
                    raise RuntimeError(
                        "maximum route motion reached before all rolling-fast "
                        "triggers: {:.2f} m".format(travelled)
                    )
                route_point = self.waypoints[nearest_hint]
                route_error = distance_xy(position, route_point)
                if route_error > self.args.route_deviation_limit_m:
                    raise RuntimeError(
                        "route deviation {:.2f} m exceeds safety limit".format(
                            route_error
                        )
                    )

                (
                    command_speed_kph,
                    command_lookahead_m,
                    command_steering_limit,
                    route_turn_deg,
                ) = self.curve_control_limits(nearest_hint)
                target_index = self.lookahead_index(
                    nearest_hint, command_lookahead_m
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
                    -command_steering_limit,
                    min(command_steering_limit, steering),
                )

                command = CtrlCmd()
                command.longlCmdType = 2
                command.velocity = command_speed_kph
                set_ctrl_steering(
                    command,
                    steering,
                    self.args.personal_max_steering_angle_deg,
                    self.args.personal_steering_sign,
                )
                command.accel = 0.0
                command.brake = 0.0
                command.acceleration = 0.0
                self.control_pub.publish(command)

                threshold = self.args.capture_spacing_m * (
                    len(triggers) + 1
                )
                if travelled >= threshold:
                    self.wait_for_connections(
                        context="before rolling-fast trigger"
                    )
                    trigger_unix = time.time()
                    self.publish_capture_request()
                    trigger = {
                        "capture_id": self.capture_identifier(
                            scene, len(triggers)
                        ),
                        "triggered_unix": trigger_unix,
                        "trigger_distance_m": travelled,
                        "route_turn_deg": route_turn_deg,
                        "command_speed_kph": command_speed_kph,
                        "command_lookahead_m": command_lookahead_m,
                        "command_steering_limit_rad": command_steering_limit,
                        "ego_position": {
                            "x": ego.position.x,
                            "y": ego.position.y,
                            "z": ego.position.z,
                        },
                        "verified": False,
                    }
                    triggers.append(trigger)
                    rospy.loginfo(
                        "Rolling-fast trigger %d/%d: distance=%.3fm, "
                        "turn=%.1fdeg, speed=%.2fkm/h, lookahead=%.2fm, "
                        "steering_limit=%.3frad",
                        len(triggers),
                        capture_count,
                        travelled,
                        route_turn_deg,
                        command_speed_kph,
                        command_lookahead_m,
                        command_steering_limit,
                    )
                time.sleep(0.05)
        finally:
            self.publish_stop()

        if len(triggers) != capture_count:
            raise RuntimeError(
                "rolling-fast ended with {}/{} triggers".format(
                    len(triggers), capture_count
                )
            )

        # Wait only once after the burst. MORAI gives all artifacts belonging
        # to one SaveSensorData snapshot the same timestamp stem, so the
        # grouped mode can build verified frames without blocking per trigger.
        drain_started = time.monotonic()
        deadline = drain_started + self.args.capture_timeout_sec
        last_files = set()
        stable_since = None
        new_files = set()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            new_files = recursive_files(source_root) - before
            if new_files != last_files:
                last_files = set(new_files)
                stable_since = time.monotonic()
            if (
                new_files
                and stable_since is not None
                and time.monotonic() - stable_since
                >= self.args.capture_stable_sec
            ):
                break
            time.sleep(0.1)

        relocated = []
        complete_group_count = 0
        incomplete_groups = {}
        timestamp_groups, unrecognized = group_capture_files_by_timestamp(
            new_files
        )
        if group_verified_frames:
            require_object_labels = (
                sum(scene["expected_counts"].values()) > 0
            )
            complete_groups = []
            for timestamp_stem, group_files in sorted(
                timestamp_groups.items()
            ):
                (
                    observed_artifacts,
                    required_artifacts,
                    missing_artifacts,
                ) = capture_artifact_status(
                    source_root,
                    group_files,
                    self.args.required_sensor_dirs,
                    require_object_labels,
                )
                observed_sensors = observed_sensor_directories(
                    source_root, group_files
                )
                missing_sensors = (
                    set(self.args.required_sensor_dirs) - observed_sensors
                )
                if missing_sensors or missing_artifacts:
                    incomplete_groups[timestamp_stem] = {
                        "file_count": len(group_files),
                        "missing_sensor_dirs": sorted(missing_sensors),
                        "missing_artifacts": missing_artifacts,
                    }
                    continue
                complete_groups.append(
                    (
                        timestamp_stem,
                        group_files,
                        observed_sensors,
                        observed_artifacts,
                        required_artifacts,
                    )
                )

            if (
                len(complete_groups) != capture_count
                or incomplete_groups
                or unrecognized
            ):
                rejected_root = (
                    self.args.capture_dir
                    / "rolling_grouped_rejected"
                    / scene["id"]
                )
                for source in sorted(new_files):
                    relative = source.relative_to(source_root)
                    destination = rejected_root / relative
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.move(str(source), str(destination))
                raise RuntimeError(
                    "rolling-grouped produced {} complete timestamp groups "
                    "for {} triggers; incomplete_groups={}, "
                    "unrecognized_files={}. Raw files were preserved at {}"
                    .format(
                        len(complete_groups),
                        capture_count,
                        incomplete_groups,
                        [
                            str(path.relative_to(source_root))
                            for path in unrecognized
                        ],
                        rejected_root,
                    )
                )

            completed_unix = time.time()
            for index, (
                timestamp_stem,
                group_files,
                observed_sensors,
                observed_artifacts,
                required_artifacts,
            ) in enumerate(complete_groups):
                trigger = triggers[index]
                source_files = sorted(
                    str(path.relative_to(source_root))
                    for path in group_files
                )
                frame_destinations = relocate_capture_files(
                    source_root,
                    self.args.capture_dir,
                    trigger["capture_id"],
                    group_files,
                    allow_empty_suffixes=(
                        {"_Instance.txt", "_Instance_8Points.txt"}
                        if not require_object_labels
                        else set()
                    ),
                )
                relocated.extend(frame_destinations)
                trigger.update(
                    {
                        "verified": True,
                        "completed_unix": completed_unix,
                        "duration_sec": (
                            completed_unix - trigger["triggered_unix"]
                        ),
                        "morai_timestamp_stem": timestamp_stem,
                        "new_file_count": len(group_files),
                        "observed_sensor_dirs": sorted(observed_sensors),
                        "missing_sensor_dirs": [],
                        "require_object_labels": require_object_labels,
                        "observed_artifacts": {
                            sensor: sorted(suffixes)
                            for sensor, suffixes in sorted(
                                observed_artifacts.items()
                            )
                        },
                        "required_artifacts": {
                            sensor: sorted(suffixes)
                            for sensor, suffixes in sorted(
                                required_artifacts.items()
                            )
                        },
                        "missing_artifacts": {},
                        "morai_source_files": source_files,
                        "new_files": sorted(
                            str(
                                path.relative_to(self.args.capture_dir)
                            )
                            for path in frame_destinations
                        ),
                    }
                )
            complete_group_count = len(complete_groups)
            burst_root = None
        else:
            burst_root = (
                self.args.capture_dir / "rolling_fast_raw" / scene["id"]
            )
            for source in sorted(new_files):
                relative = source.relative_to(source_root)
                destination = burst_root / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                if destination.exists():
                    raise FileExistsError(destination)
                shutil.move(str(source), str(destination))
                relocated.append(destination)

        trigger_span = 0.0
        trigger_hz = 0.0
        if len(triggers) > 1:
            trigger_span = (
                triggers[-1]["triggered_unix"]
                - triggers[0]["triggered_unix"]
            )
            if trigger_span > 0.0:
                trigger_hz = (len(triggers) - 1) / trigger_span
        metrics = {
            "experimental": not group_verified_frames,
            "quality_verified": group_verified_frames,
            "trigger_count": len(triggers),
            "trigger_span_sec": trigger_span,
            "trigger_rate_hz": trigger_hz,
            "drain_duration_sec": time.monotonic() - drain_started,
            "observed_file_count": len(new_files),
            "relocated_file_count": len(relocated),
            "timestamp_group_count": len(timestamp_groups),
            "complete_timestamp_group_count": complete_group_count,
            "incomplete_timestamp_groups": incomplete_groups,
            "raw_output_dir": (
                str(burst_root) if burst_root is not None else None
            ),
        }
        if group_verified_frames:
            rospy.loginfo(
                "Rolling-grouped complete: triggers=%d, frames=%d, "
                "span=%.3fs, rate=%.3fHz, files=%d",
                len(triggers),
                complete_group_count,
                trigger_span,
                trigger_hz,
                len(new_files),
            )
        else:
            rospy.logwarn(
                "Rolling-fast benchmark: triggers=%d, span=%.3fs, "
                "rate=%.3fHz, files=%d (quality not verified)",
                len(triggers),
                trigger_span,
                trigger_hz,
                len(new_files),
            )
        return triggers, metrics

    def run_scene(self, scene):
        self.configure_scene_motion(scene)
        scene_capture_count = int(
            scene.get("capture_policy_overrides", {}).get(
                "captures_per_scene", self.args.captures_per_scene
            )
        )
        scene_started = time.time()
        mode_metrics = {}
        control_prepared = False
        if self.args.skip_scenario_load:
            issued = time.monotonic()
            rospy.loginfo(
                "ScenarioLoad skipped; validating current scene as %s",
                scene["file_name"],
            )
            self.wait_for_scene(scene, issued)
        else:
            control_prepared = self.load_and_verify_scene(scene)
        if self.args.motion_mode == "stationary":
            captures = [
                self.trigger_capture(scene, index)
                for index in range(scene_capture_count)
            ]
        elif self.args.motion_mode == "route-follow":
            if not control_prepared:
                self.select_external_control_mode()
            captures = self.drive_and_capture(
                scene, scene_capture_count
            )
        elif self.args.motion_mode == "rolling-safe":
            if not control_prepared:
                self.select_external_control_mode()
            captures = self.drive_and_capture(
                scene,
                scene_capture_count,
                stop_for_capture=False,
            )
        elif self.args.motion_mode == "rolling-fast":
            if not control_prepared:
                self.select_external_control_mode()
            captures, mode_metrics = self.drive_and_trigger_burst(
                scene, scene_capture_count
            )
        elif self.args.motion_mode == "rolling-grouped":
            if not control_prepared:
                self.select_external_control_mode()
            captures, mode_metrics = self.drive_and_trigger_burst(
                scene,
                scene_capture_count,
                group_verified_frames=True,
            )
        elif self.args.motion_mode == "timed-grouped":
            if not control_prepared:
                self.select_external_control_mode()
            captures, mode_metrics = self.drive_and_trigger_timed_grouped(
                scene, scene_capture_count
            )
        else:
            captures = self.teleport_and_capture(
                scene, scene_capture_count
            )
        scene_finished = time.time()
        trigger_times = [
            item["triggered_unix"]
            for item in captures
            if "triggered_unix" in item
        ]
        scene_duration = scene_finished - scene_started
        mode_metrics.update(
            {
                "scene_duration_sec": scene_duration,
                "end_to_end_capture_rate_hz": (
                    len(captures) / scene_duration
                    if scene_duration > 0.0
                    else 0.0
                ),
            }
        )
        if (
            "trigger_rate_hz" not in mode_metrics
            and len(trigger_times) > 1
        ):
            trigger_span = trigger_times[-1] - trigger_times[0]
            mode_metrics["trigger_span_sec"] = trigger_span
            mode_metrics["trigger_rate_hz"] = (
                (len(trigger_times) - 1) / trigger_span
                if trigger_span > 0.0
                else 0.0
            )
        return {
            "id": scene["id"],
            "base_scenario_id": scene.get(
                "base_scenario_id", scene["id"]
            ),
            "file_name": scene["file_name"],
            "split": scene.get("split"),
            "status": "completed",
            "expected_counts": dict(scene["expected_counts"]),
            "actual_objects": self.object_observation(
                self.state.snapshot()["objects"]
            ),
            "captures": captures,
            "metrics": mode_metrics,
        }

    def run(
        self,
        scenes,
        prior_completed=None,
        requested_scene_count=None,
        original_started_unix=None,
    ):
        self.wait_for_connections(
            include_motion_interface=self.args.motion_mode == "teleport"
        )
        scenes_path = self.args.capture_dir / "scenes.jsonl"
        failures_path = self.args.capture_dir / "failures.jsonl"
        summary_path = self.args.capture_dir / "summary.json"
        if original_started_unix is None:
            scenes_path.write_text("", encoding="utf-8")
            failures_path.write_text("", encoding="utf-8")
        report = {
            "schema_version": 2,
            "dataset_id": self.dataset_id,
            "environment": self.environment,
            "started_unix": original_started_unix or time.time(),
            "resumed_unix": (
                time.time() if prior_completed else None
            ),
            "motion_mode": self.args.motion_mode,
            "capture_dir": str(self.args.capture_dir),
            "scenes": [
                compact_scene_result(item)
                for item in (prior_completed or [])
            ],
        }
        requested_scene_count = requested_scene_count or len(scenes)
        interrupted = False

        def current_summary(finished=False):
            completed = [
                item
                for item in report["scenes"]
                if item.get("status") == "completed"
            ]
            failed = [
                item
                for item in report["scenes"]
                if item.get("status") == "failed"
            ]
            document = {
                "schema_version": 2,
                "dataset_id": self.dataset_id,
                "environment": self.environment,
                "started_unix": report["started_unix"],
                "resumed_unix": report.get("resumed_unix"),
                "checkpoint_unix": time.time(),
                "motion_mode": self.args.motion_mode,
                "capture_dir": str(self.args.capture_dir),
                "interrupted": interrupted,
                "finished": finished,
                "requested_scenes": requested_scene_count,
                "attempted_scenes": len(report["scenes"]),
                "completed_scenes": len(completed),
                "failed_scenes": len(failed),
                "completed_captures": sum(
                    int(item.get("capture_count", 0))
                    for item in completed
                ),
                "completed_scene_ids": [item["id"] for item in completed],
                "failed_scene_ids": [item["id"] for item in failed],
                "recent_failed_scene_ids": [
                    item["id"] for item in failed[-8:]
                ],
                "latest_scene": (
                    report["scenes"][-1] if report["scenes"] else None
                ),
                "files": {
                    "scene_records": "scenes.jsonl",
                    "failure_details": "failures.jsonl",
                },
            }
            if finished:
                document["finished_unix"] = time.time()
            return document

        # Mark an existing summary active immediately on resume. Otherwise a
        # monitor started before the first retried scene can see the previous
        # finished=true state and exit prematurely.
        write_json_atomic(summary_path, current_summary())

        for scene in scenes:
            if rospy.is_shutdown():
                interrupted = True
                rospy.logwarn("ROS shutdown requested; capture batch paused")
                break
            scene_started = time.time()
            try:
                result = self.run_scene(scene)
            except Exception as error:
                if rospy.is_shutdown() or isinstance(error, InterruptedError):
                    interrupted = True
                    rospy.logwarn(
                        "ROS shutdown requested during %s; scene left pending",
                        scene["id"],
                    )
                    break
                if not self.args.continue_on_scene_error:
                    raise
                result = {
                    "id": scene["id"],
                    "base_scenario_id": scene.get(
                        "base_scenario_id", scene["id"]
                    ),
                    "file_name": scene["file_name"],
                    "split": scene.get("split"),
                    "status": "failed",
                    "error": "{}: {}".format(type(error).__name__, error),
                    "expected_counts": dict(scene["expected_counts"]),
                    "actual_objects": self.object_observation(
                        self.state.snapshot()["objects"]
                    ),
                    "captures": [],
                    "metrics": {
                        "scene_duration_sec": time.time() - scene_started,
                    },
                }
                rospy.logerr(
                    "Scene %s failed and will be skipped: %s",
                    scene["id"],
                    error,
                )
            compact = compact_scene_result(result)
            report["scenes"].append(compact)
            append_jsonl(scenes_path, compact)
            if compact["status"] == "failed":
                append_jsonl(failures_path, detailed_failure_result(result))
            write_json_atomic(summary_path, current_summary())
        report["interrupted"] = interrupted
        summary = current_summary(finished=not interrupted)
        write_json_atomic(summary_path, summary)
        return summary


def select_scenes(manifest, requested):
    scenes = manifest["scenes"]
    if not requested:
        return [
            scene for scene in scenes
            if scene.get("run_by_default", True)
        ]
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
        args.capture_spacing_m = float(policy.get("capture_spacing_m", 1.5))
    if args.capture_period_sec is None:
        args.capture_period_sec = float(policy.get("capture_period_sec", 0.5))
    if args.capture_motion_arm_m is None:
        args.capture_motion_arm_m = float(
            policy.get("capture_motion_arm_m", 0.30)
        )
    if args.minimum_capture_progress_m is None:
        args.minimum_capture_progress_m = float(
            policy.get("minimum_capture_progress_m", 0.20)
        )
    if args.motion_stall_timeout_sec is None:
        args.motion_stall_timeout_sec = float(
            policy.get("motion_stall_timeout_sec", 5.0)
        )
    if args.actor_ready_timeout_sec is None:
        args.actor_ready_timeout_sec = float(
            policy.get("actor_ready_timeout_sec", 15.0)
        )
    args.dynamic_warmup_sec = float(policy.get("warmup_sec", 0.0))
    args.prime_capture_before_clip = bool(
        policy.get("prime_capture_before_clip", False)
    )
    if args.speed_kph is None:
        args.speed_kph = float(policy["ego_target_speed_kph"])
    if args.maximum_motion_m is None:
        args.maximum_motion_m = float(policy["maximum_motion_m"])
    if args.route_deviation_limit_m is None:
        args.route_deviation_limit_m = float(
            policy.get("route_deviation_limit_m", 2.5)
        )
    if args.heading_error_limit_deg is None:
        args.heading_error_limit_deg = float(
            policy.get("heading_error_limit_deg", 40.0)
        )
    if args.lookahead_m is None:
        args.lookahead_m = float(policy.get("lookahead_m", 3.0))
    if args.maximum_steering_rad is None:
        args.maximum_steering_rad = float(
            policy.get("maximum_steering_rad", 0.20)
        )
    if args.adaptive_curve_control is None:
        args.adaptive_curve_control = bool(
            policy.get("adaptive_curve_control", False)
        )
    if args.curve_control_window_m is None:
        args.curve_control_window_m = float(
            policy.get("curve_control_window_m", 8.0)
        )
    if args.curve_control_full_turn_deg is None:
        args.curve_control_full_turn_deg = float(
            policy.get("curve_control_full_turn_deg", 18.0)
        )
    if args.minimum_curve_speed_kph is None:
        args.minimum_curve_speed_kph = float(
            policy.get("minimum_curve_speed_kph", 3.6)
        )
    if args.minimum_curve_lookahead_m is None:
        args.minimum_curve_lookahead_m = float(
            policy.get("minimum_curve_lookahead_m", 1.2)
        )
    if args.curve_maximum_steering_rad is None:
        args.curve_maximum_steering_rad = float(
            policy.get("curve_maximum_steering_rad", 0.45)
        )
    if args.personal_steering_sign is None:
        args.personal_steering_sign = float(
            policy.get("personal_steering_sign", -1.0)
        )

    if args.captures_per_scene < 1:
        raise ValueError("--captures-per-scene must be >= 1")
    if args.scenario_load_attempts < 1:
        raise ValueError("--scenario-load-attempts must be >= 1")
    if not 1 <= args.scenario_load_udp_port <= 65535:
        raise ValueError("--scenario-load-udp-port must be within 1..65535")
    if args.capture_stable_sec <= 0.0:
        raise ValueError("--capture-stable-sec must be positive")
    if args.capture_period_sec <= 0.0:
        raise ValueError("--capture-period-sec must be positive")
    if args.capture_motion_arm_m <= 0.0:
        raise ValueError("--capture-motion-arm-m must be positive")
    if args.minimum_capture_progress_m <= 0.0:
        raise ValueError("--minimum-capture-progress-m must be positive")
    if args.motion_stall_timeout_sec <= 0.0:
        raise ValueError("--motion-stall-timeout-sec must be positive")
    if args.actor_ready_timeout_sec <= 0.0:
        raise ValueError("--actor-ready-timeout-sec must be positive")
    if args.teleport_timeout_sec <= 0.0:
        raise ValueError("--teleport-timeout-sec must be positive")
    if args.teleport_position_tolerance_m <= 0.0:
        raise ValueError(
            "--teleport-position-tolerance-m must be positive"
        )
    if args.teleport_z_tolerance_m <= 0.0:
        raise ValueError("--teleport-z-tolerance-m must be positive")
    if args.teleport_settle_sec < 0.0:
        raise ValueError("--teleport-settle-sec must be non-negative")
    if args.control_mode_timeout_sec <= 0.0:
        raise ValueError("--control-mode-timeout-sec must be positive")
    if args.ego_heartbeat_timeout_sec <= 0.0:
        raise ValueError("--ego-heartbeat-timeout-sec must be positive")
    if args.timing_history_sec <= 0.0:
        raise ValueError("--timing-history-sec must be positive")
    if args.timing_window_padding_sec < 0.0:
        raise ValueError("--timing-window-padding-sec must be non-negative")
    args.camera_timing_topics = parse_named_topics(args.camera_timing_topic)
    if args.required_sensor_dir is None:
        args.required_sensor_dirs = (
            "CAMERA_1",
            "CAMERA_2",
            "CAMERA_3",
            "CAMERA_4",
            "LIDAR_5",
        )
    else:
        args.required_sensor_dirs = tuple(args.required_sensor_dir)
    if not args.required_sensor_dirs or any(
        not name or "/" in name or name in (".", "..")
        for name in args.required_sensor_dirs
    ):
        raise ValueError("--required-sensor-dir values must be directory names")
    if args.capture_spacing_m <= 0.0:
        raise ValueError("--capture-spacing-m must be positive")
    if args.route_deviation_limit_m <= 0.0:
        raise ValueError("--route-deviation-limit-m must be positive")
    if not 0.0 < args.heading_error_limit_deg < 180.0:
        raise ValueError("--heading-error-limit-deg must be within 0..180")
    if args.lookahead_m <= 0.0:
        raise ValueError("--lookahead-m must be positive")
    if args.maximum_steering_rad <= 0.0:
        raise ValueError("--maximum-steering-rad must be positive")
    if args.curve_control_window_m <= 0.0:
        raise ValueError("--curve-control-window-m must be positive")
    if args.curve_control_full_turn_deg <= 0.0:
        raise ValueError("--curve-control-full-turn-deg must be positive")
    if not 0.5 <= args.minimum_curve_speed_kph <= args.speed_kph:
        raise ValueError(
            "--minimum-curve-speed-kph must be within 0.5..--speed-kph"
        )
    if not 0.5 <= args.minimum_curve_lookahead_m <= args.lookahead_m:
        raise ValueError(
            "--minimum-curve-lookahead-m must be within 0.5..--lookahead-m"
        )
    if args.curve_maximum_steering_rad < args.maximum_steering_rad:
        raise ValueError(
            "--curve-maximum-steering-rad must be >= "
            "--maximum-steering-rad"
        )
    if args.personal_steering_sign not in (-1.0, 1.0):
        raise ValueError("--personal-steering-sign must be either -1 or +1")
    if not 0.5 <= args.speed_kph <= 7.2:
        raise ValueError("--speed-kph must be within the PoC safety range 0.5..7.2")
    capture_intervals = max(0, args.captures_per_scene - 1)
    if args.motion_mode in ("rolling-fast", "rolling-grouped"):
        # rolling-fast takes its first frame after one full spacing interval,
        # so every requested frame consumes route distance.
        capture_intervals = args.captures_per_scene
    required_motion = args.capture_spacing_m * capture_intervals
    if args.motion_mode == "timed-grouped":
        required_motion = 0.0
    if (
        args.motion_mode
        in (
            "route-follow",
            "rolling-safe",
            "rolling-fast",
            "rolling-grouped",
            "timed-grouped",
            "teleport",
        )
        and required_motion > args.maximum_motion_m
    ):
        raise ValueError(
            "capture policy requires {:.2f} m but maximum motion is {:.2f} m".format(
                required_motion, args.maximum_motion_m
            )
        )


def main():
    args = parse_args()
    project_root = Path(__file__).resolve().parents[3]
    args.manifest = args.manifest.expanduser().resolve()
    resume_report = None
    if args.resume_report is not None:
        args.resume_report = args.resume_report.expanduser().resolve()
        resume_report = load_json(args.resume_report)
        if resume_report.get("dataset_id") != load_json(args.manifest).get(
            "dataset_id"
        ):
            raise ValueError(
                "--resume-report dataset_id differs from --manifest"
            )
    requested_capture_dir = args.capture_dir.expanduser()
    if not args.no_timestamp_prefix:
        requested_capture_dir = timestamped_capture_dir(
            requested_capture_dir
        )
    args.capture_dir = requested_capture_dir.resolve()
    if resume_report is not None:
        if not args.no_timestamp_prefix:
            raise ValueError("--resume-report requires --no-timestamp-prefix")
        if args.capture_dir != args.resume_report.parent:
            raise ValueError(
                "--capture-dir must be the resume report directory: {}".format(
                    args.resume_report.parent
                )
            )
    args.morai_sensor_data_dir = (
        args.morai_sensor_data_dir.expanduser().resolve()
    )
    args.morai_scenario_dir = (
        args.morai_scenario_dir.expanduser().resolve()
    )
    manifest = load_json(args.manifest)
    validate_policy(args, manifest)
    selected_scenes = select_scenes(manifest, args.scenario)
    requested_scene_count = len(selected_scenes)
    prior_completed = []
    if resume_report is not None:
        if "completed_scene_ids" in resume_report:
            selected_by_id = {
                scene["id"]: scene for scene in selected_scenes
            }
            completed_by_id = {
                scene_id: {
                    "id": scene_id,
                    "base_scenario_id": selected_by_id[scene_id].get(
                        "base_scenario_id", scene_id
                    ),
                    "file_name": selected_by_id[scene_id]["file_name"],
                    "split": selected_by_id[scene_id].get("split"),
                    "status": "completed",
                    "capture_count": args.captures_per_scene,
                    "metrics": {},
                }
                for scene_id in resume_report["completed_scene_ids"]
                if scene_id in selected_by_id
            }
        else:
            completed_by_id = {
                item["id"]: item
                for item in resume_report.get("scenes", [])
                if item.get("status") == "completed"
            }
        selected_ids = {scene["id"] for scene in selected_scenes}
        prior_completed = [
            completed_by_id[scene["id"]]
            for scene in selected_scenes
            if scene["id"] in completed_by_id
        ]
        scenes = [
            scene
            for scene in selected_scenes
            if scene["id"] not in completed_by_id
        ]
        unknown_completed = set(completed_by_id) - selected_ids
        if unknown_completed:
            raise ValueError(
                "resume report contains completed scenes outside selection: {}".format(
                    ", ".join(sorted(unknown_completed))
                )
            )
    else:
        scenes = selected_scenes
    if args.skip_scenario_load and len(scenes) != 1:
        raise ValueError(
            "--skip-scenario-load requires exactly one --scenario"
        )

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
    install_dir = validate_installed_scenes(
        args, manifest, scenes, project_root
    )

    print("MORAI ROS auto-capture PoC")
    if len(scenes) <= 20:
        print("scenes:", ", ".join(scene["id"] for scene in scenes))
    else:
        print(
            "scenes: {} total ({} ... {})".format(
                len(scenes), scenes[0]["id"], scenes[-1]["id"]
            )
        )
    print("motion mode:", args.motion_mode)
    print("scenario load transport:", args.scenario_load_transport)
    print("captures per scene:", args.captures_per_scene)
    if args.motion_mode == "timed-grouped":
        print("capture period sec:", args.capture_period_sec)
        print("capture motion arm m:", args.capture_motion_arm_m)
        print("minimum capture progress m:", args.minimum_capture_progress_m)
        print("motion stall timeout sec:", args.motion_stall_timeout_sec)
    else:
        print("capture spacing m:", args.capture_spacing_m)
    print("capture dir:", args.capture_dir)
    print("MORAI SensorData dir:", args.morai_sensor_data_dir)
    print("required sensor dirs:", ", ".join(args.required_sensor_dirs))
    print("MORAI scenario dir:", install_dir)
    print("ROS network sidecars: verified")
    if resume_report is not None:
        print("resume report:", args.resume_report)
        print("completed scenes preserved:", len(prior_completed))
        print("scenes remaining/retrying:", len(scenes))
    if args.dry_run:
        print("dry-run validation: OK")
        return

    if not args.morai_sensor_data_dir.is_dir():
        raise FileNotFoundError(
            "MORAI SensorData directory does not exist: {}".format(
                args.morai_sensor_data_dir
            )
        )
    args.capture_dir.mkdir(parents=True, exist_ok=True)
    runner = AutoCapture(args, manifest, project_root)
    report = runner.run(
        scenes,
        prior_completed=prior_completed,
        requested_scene_count=requested_scene_count,
        original_started_unix=(
            resume_report.get("started_unix")
            if resume_report is not None
            else None
        ),
    )
    summary_path = args.capture_dir / "summary.json"
    if report.get("interrupted"):
        print("auto-capture paused by ROS shutdown")
        print("resume report:", summary_path)
        return
    print("auto-capture complete")
    print("summary:", summary_path)


if __name__ == "__main__":
    main()
