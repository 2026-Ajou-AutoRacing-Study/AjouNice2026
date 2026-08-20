#!/usr/bin/env python3
"""Pure helpers for SaveSensorData dynamic-clip metadata.

This module deliberately has no ROS import so metadata math can be tested
without a running ROS master.  Runtime messages only need to expose the
standard MORAI message attributes used below.
"""

import math


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
