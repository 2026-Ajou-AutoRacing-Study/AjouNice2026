#!/usr/bin/env python3
"""Pure helpers for SaveSensorData dynamic-clip metadata.

This module deliberately has no ROS import so metadata math can be tested
without a running ROS master.  Runtime messages only need to expose the
standard MORAI message attributes used below.
"""

import hashlib
import json
import math
from pathlib import Path


CAMERA_OPTICAL_TO_PHYSICAL = (
    (0.0, 0.0, 1.0),
    (-1.0, 0.0, 0.0),
    (0.0, -1.0, 0.0),
)


def vector3(value):
    return {
        "x": float(value.x),
        "y": float(value.y),
        "z": float(value.z),
    }


def vector_norm(value):
    return math.sqrt(value.x * value.x + value.y * value.y + value.z * value.z)


def message_stamp_sec(message):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    seconds = float(getattr(stamp, "secs", 0.0))
    nanoseconds = float(getattr(stamp, "nsecs", 0.0))
    if seconds == 0.0 and nanoseconds == 0.0:
        return None
    return seconds + nanoseconds * 1e-9


def wrapped_degrees(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def capture_actor_gate_observation(ego, objects, gate):
    """Evaluate an actor-relative capture gate and return its best candidate."""

    if not gate:
        return True, None
    if not ego or not objects:
        return False, None
    ego_position = ego["position_m"]
    yaw = math.radians(float(ego["heading_deg"]))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    candidates = []
    for actor in objects.get("actors", []):
        if gate.get("class") and actor.get("class") != gate["class"]:
            continue
        position = actor["position_m"]
        dx = float(position["x"]) - float(ego_position["x"])
        dy = float(position["y"]) - float(ego_position["y"])
        longitudinal = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        distance = math.hypot(dx, dy)
        actor_speed = actor.get("derived_speed_mps")
        if actor_speed is None:
            actor_speed = actor.get("morai_speed_raw")
            if actor_speed is not None:
                # MORAI ObjectStatus velocity magnitude is reported in km/h.
                actor_speed = float(actor_speed) / 3.6
        if actor_speed is None:
            actor_speed = 0.0
        observation = {
            "unique_id": actor.get("unique_id"),
            "class": actor.get("class"),
            "longitudinal_m": longitudinal,
            "lateral_m": lateral,
            "distance_m": distance,
            "speed_mps": float(actor_speed),
        }
        ready = (
            longitudinal
            >= float(gate.get("minimum_longitudinal_m", -math.inf))
            and longitudinal
            <= float(gate.get("maximum_longitudinal_m", math.inf))
            and abs(lateral)
            <= float(gate.get("maximum_abs_lateral_m", math.inf))
            and distance >= float(gate.get("minimum_distance_m", 0.0))
            and distance <= float(gate.get("maximum_distance_m", math.inf))
            and observation["speed_mps"]
            >= float(gate.get("minimum_actor_speed_mps", 0.0))
        )
        candidates.append((not ready, distance, observation))
    if not candidates:
        return False, None
    not_ready, _distance, observation = min(
        candidates, key=lambda item: (item[0], item[1])
    )
    return not not_ready, observation


def _matrix_multiply(left, right):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def _rotation_multiply(left, right):
    return [
        [
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _rpy_rotation(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = map(
        math.radians, (float(roll_deg), float(pitch_deg), float(yaw_deg))
    )
    rx = (
        (1.0, 0.0, 0.0),
        (0.0, math.cos(roll), -math.sin(roll)),
        (0.0, math.sin(roll), math.cos(roll)),
    )
    ry = (
        (math.cos(pitch), 0.0, math.sin(pitch)),
        (0.0, 1.0, 0.0),
        (-math.sin(pitch), 0.0, math.cos(pitch)),
    )
    rz = (
        (math.cos(yaw), -math.sin(yaw), 0.0),
        (math.sin(yaw), math.cos(yaw), 0.0),
        (0.0, 0.0, 1.0),
    )
    return _rotation_multiply(_rotation_multiply(rz, ry), rx)


def _rigid_transform(rotation, translation):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], float(translation[0])],
        [rotation[1][0], rotation[1][1], rotation[1][2], float(translation[1])],
        [rotation[2][0], rotation[2][1], rotation[2][2], float(translation[2])],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _invert_rigid(transform):
    rotation = [[transform[row][column] for column in range(3)] for row in range(3)]
    transpose = [[rotation[column][row] for column in range(3)] for row in range(3)]
    translation = [transform[row][3] for row in range(3)]
    inverse_translation = [
        -sum(transpose[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return _rigid_transform(transpose, inverse_translation)


def _sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def normalized_sensor_calibration(sensor_config_path):
    """Normalize MORAI Sensor JSON into explicit intrinsic/extrinsic matrices."""

    sensor_config_path = Path(sensor_config_path).resolve()
    raw = json.loads(sensor_config_path.read_text(encoding="utf-8"))
    lidar = raw["Lidar3DList"][0]
    lidar_rotation = _rpy_rotation(
        lidar["rot"]["roll"], lidar["rot"]["pitch"], lidar["rot"]["yaw"]
    )
    lidar_to_ego = _rigid_transform(
        lidar_rotation, [lidar["pos"][axis] for axis in "xyz"]
    )
    result = {
        "schema_version": 1,
        "source": str(sensor_config_path),
        "source_sha256": _sha256_file(sensor_config_path),
        "units": {"translation": "metre", "rotation": "degree"},
        "matrix_convention": (
            "4x4 row-major matrix with column vectors: "
            "point_destination = T_destination_from_source @ point_source"
        ),
        "coordinate_axes": {
            "morai_map": "x/y map plane, z up",
            "ego": "x forward, y left, z up",
            "lidar": "x forward, y left, z up",
            "camera_optical": "x right, y down, z forward",
        },
        "lidar": {
            "sensor_unique_id": int(lidar["m_SensorUniqueID"]),
            "sensor_name": lidar["lc"]["sensorName"],
            "topic": lidar["lc"].get("rosConfig", {}).get("Topic"),
            "period_sec": float(lidar["lc"]["sensorPeriod"]),
            "lidar_to_ego": lidar_to_ego,
            "ego_to_lidar": _invert_rigid(lidar_to_ego),
        },
        "cameras": {},
        "limitations": [
            "SaveSensorData does not expose per-image exposure time in the saved filename.",
            "Capture Mode may omit camera/LiDAR ROS topics; unavailable timestamps are not synthesized.",
        ],
    }
    for item in raw["cameraList"]:
        camera = item["cc"]
        name = str(camera["sensorName"])
        width = int(camera["cameraResWidth"])
        height = int(camera["cameraResHeight"])
        fov = float(camera["cameraFOV"])
        effective_focal = width / (2.0 * math.tan(math.radians(fov) / 2.0))
        cx = float(camera["principalPoint"]["x"])
        cy = float(camera["principalPoint"]["y"])
        physical_rotation = _rpy_rotation(
            item["rot"]["roll"], item["rot"]["pitch"], item["rot"]["yaw"]
        )
        optical_rotation = _rotation_multiply(
            physical_rotation, CAMERA_OPTICAL_TO_PHYSICAL
        )
        camera_to_ego = _rigid_transform(
            optical_rotation, [item["pos"][axis] for axis in "xyz"]
        )
        result["cameras"][name] = {
            "sensor_unique_id": int(item["m_SensorUniqueID"]),
            "topic": camera.get("rosConfig", {}).get("Topic"),
            "period_sec": float(camera["sensorPeriod"]),
            "resolution_px": [width, height],
            "horizontal_fov_deg": fov,
            "source_focal_length_pixel": float(camera["focalLengthpixel"]),
            "effective_pinhole_focal_pixel": effective_focal,
            "intrinsic": [
                [effective_focal, 0.0, cx],
                [0.0, effective_focal, cy],
                [0.0, 0.0, 1.0],
            ],
            "distortion": [float(value) for value in camera.get("lensDistortion", [])],
            "sensor_to_ego": camera_to_ego,
            "ego_to_sensor": _invert_rigid(camera_to_ego),
        }
    return result


def enrich_frame_transforms(frames):
    """Attach absolute and consecutive Ego transforms to capture frames."""

    first_ego_to_global = None
    previous_ego_to_global = None
    for frame in frames:
        ego = frame.get("ego")
        if not ego:
            frame["transforms"] = None
            continue
        position = ego["position_m"]
        ego_to_global = _rigid_transform(
            _rpy_rotation(0.0, 0.0, ego["heading_deg"]),
            [position["x"], position["y"], position["z"]],
        )
        global_to_ego = _invert_rigid(ego_to_global)
        if first_ego_to_global is None:
            first_ego_to_global = ego_to_global
        transforms = {
            "ego_to_global": ego_to_global,
            "global_to_ego": global_to_ego,
            "previous_ego_to_current_ego": None,
            "current_ego_to_previous_ego": None,
            "first_ego_to_current_ego": _matrix_multiply(
                global_to_ego, first_ego_to_global
            ),
            "current_ego_to_first_ego": _matrix_multiply(
                _invert_rigid(first_ego_to_global), ego_to_global
            ),
        }
        if previous_ego_to_global is not None:
            transforms["previous_ego_to_current_ego"] = _matrix_multiply(
                global_to_ego, previous_ego_to_global
            )
            transforms["current_ego_to_previous_ego"] = _matrix_multiply(
                _invert_rigid(previous_ego_to_global), ego_to_global
            )
        frame["transforms"] = transforms
        previous_ego_to_global = ego_to_global
    return frames


def actor_class_lists(message):
    return (
        ("vehicle", getattr(message, "npc_list", [])),
        ("pedestrian", getattr(message, "pedestrian_list", [])),
        ("obstacle", getattr(message, "obstacle_list", [])),
    )


def serialize_ego(message, callback_unix=None, callback_monotonic=None):
    if message is None:
        return None
    result = {
        "timestamp": message_stamp_sec(message),
        "callback_unix": callback_unix,
        "callback_monotonic": callback_monotonic,
        "unique_id": int(message.unique_id),
        "position_m": vector3(message.position),
        "heading_deg": float(message.heading),
        "velocity_raw": vector3(message.velocity),
        "acceleration_raw": vector3(message.acceleration),
        "speed_raw": vector_norm(message.velocity),
    }
    result["control"] = {
        name: float(getattr(message, name))
        for name in ("accel", "brake", "wheel_angle")
        if hasattr(message, name)
    }
    return result


def serialize_objects(message, callback_unix=None, callback_monotonic=None):
    if message is None:
        return None
    actors = []
    for class_name, values in actor_class_lists(message):
        for actor in values:
            actors.append(
                {
                    "class": class_name,
                    "unique_id": int(actor.unique_id),
                    "type": int(actor.type),
                    "name": str(actor.name),
                    "position_m": vector3(actor.position),
                    "heading_deg": float(actor.heading),
                    "size_m": vector3(actor.size),
                    "morai_velocity_raw": vector3(actor.velocity),
                    "morai_acceleration_raw": vector3(actor.acceleration),
                    "morai_speed_raw": vector_norm(actor.velocity),
                    "derived_velocity_mps": None,
                    "derived_acceleration_mps2": None,
                    "derived_speed_mps": None,
                    "derived_yaw_rate_dps": None,
                    "derived_longitudinal_acceleration_mps2": None,
                    "motion_phase": "unresolved",
                }
            )
    return {
        "timestamp": message_stamp_sec(message),
        "callback_unix": callback_unix,
        "callback_monotonic": callback_monotonic,
        "counts": {
            "vehicle": int(message.num_of_npcs),
            "pedestrian": int(message.num_of_pedestrian),
            "obstacle": int(message.num_of_obstacle),
        },
        "actors": actors,
    }


def motion_phase(speed_mps, acceleration_mps2, yaw_rate_dps):
    if speed_mps < 0.15:
        return "stationary"
    if acceleration_mps2 > 0.5:
        return "accelerating"
    if acceleration_mps2 < -0.5:
        return "braking"
    if abs(yaw_rate_dps) > 5.0:
        return "turning"
    return "cruising"


def _state_time(frame, state):
    # SaveSensorData may throttle MORAI wall-clock publication while physical
    # simulation advances much more slowly.  Temporal capture frames therefore
    # carry a kinematically integrated sequence time; prefer it for finite
    # differences so velocity labels are not divided by disk-I/O latency.
    sequence_time = frame.get("sequence_time_sec")
    if sequence_time is not None:
        return float(sequence_time)
    value = state.get("timestamp") if state else None
    if value is not None:
        return float(value)
    return float(frame["triggered_unix"])


def enrich_derived_motion(frames):
    """Add finite-difference Ego/actor motion labels in place.

    Central differences are used when both neighbours exist.  Actor tracks
    never cross scene boundaries because this function receives one clip.
    """

    actor_tracks = {}
    ego_track = []
    for frame_index, frame in enumerate(frames):
        ego = frame.get("ego")
        if ego:
            ego_track.append((frame_index, _state_time(frame, ego), ego))
        objects = frame.get("objects") or {}
        for actor in objects.get("actors", []):
            key = (actor["class"], actor["unique_id"])
            actor_tracks.setdefault(key, []).append(
                (frame_index, _state_time(frame, objects), actor)
            )

    def derive(track):
        for index, (_frame_index, _timestamp, state) in enumerate(track):
            left = track[max(0, index - 1)]
            right = track[min(len(track) - 1, index + 1)]
            dt = right[1] - left[1]
            if dt <= 1e-4:
                continue
            velocity = {
                axis: (
                    right[2]["position_m"][axis]
                    - left[2]["position_m"][axis]
                )
                / dt
                for axis in ("x", "y", "z")
            }
            state["derived_velocity_mps"] = velocity
            state["derived_speed_mps"] = math.hypot(
                velocity["x"], velocity["y"]
            )
            state["derived_yaw_rate_dps"] = wrapped_degrees(
                right[2]["heading_deg"] - left[2]["heading_deg"]
            ) / dt

        for index, (_frame_index, _timestamp, state) in enumerate(track):
            left = track[max(0, index - 1)]
            right = track[min(len(track) - 1, index + 1)]
            left_velocity = left[2].get("derived_velocity_mps")
            right_velocity = right[2].get("derived_velocity_mps")
            dt = right[1] - left[1]
            if left_velocity is None or right_velocity is None or dt <= 1e-4:
                continue
            acceleration = {
                axis: (right_velocity[axis] - left_velocity[axis]) / dt
                for axis in ("x", "y", "z")
            }
            state["derived_acceleration_mps2"] = acceleration
            velocity = state.get("derived_velocity_mps")
            speed = state.get("derived_speed_mps") or 0.0
            longitudinal = 0.0
            if velocity is not None and speed > 0.1:
                longitudinal = (
                    acceleration["x"] * velocity["x"]
                    + acceleration["y"] * velocity["y"]
                ) / speed
            state["derived_longitudinal_acceleration_mps2"] = longitudinal
            state["motion_phase"] = motion_phase(
                speed,
                longitudinal,
                state.get("derived_yaw_rate_dps") or 0.0,
            )

    for track in actor_tracks.values():
        derive(track)
    derive(ego_track)

    # Provide a detector-friendly Ego-relative representation while retaining
    # the authoritative MORAI map coordinates.  The current route controller
    # and scenario generator both interpret heading=0 along map +X.
    for frame in frames:
        ego = frame.get("ego")
        if not ego:
            continue
        yaw = math.radians(ego["heading_deg"])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        objects = frame.get("objects") or {}
        for actor in objects.get("actors", []):
            dx = actor["position_m"]["x"] - ego["position_m"]["x"]
            dy = actor["position_m"]["y"] - ego["position_m"]["y"]
            relative = {
                "position_m": {
                    "x": cosine * dx + sine * dy,
                    "y": -sine * dx + cosine * dy,
                    "z": actor["position_m"]["z"] - ego["position_m"]["z"],
                },
                "heading_deg": wrapped_degrees(
                    actor["heading_deg"] - ego["heading_deg"]
                ),
                "derived_velocity_mps": None,
                "derived_acceleration_mps2": None,
            }
            for source_key in (
                "derived_velocity_mps",
                "derived_acceleration_mps2",
            ):
                source = actor.get(source_key)
                if source is None:
                    continue
                relative[source_key] = {
                    "x": cosine * source["x"] + sine * source["y"],
                    "y": -sine * source["x"] + cosine * source["y"],
                    "z": source["z"],
                }
            actor["relative_to_ego"] = relative
    return frames
