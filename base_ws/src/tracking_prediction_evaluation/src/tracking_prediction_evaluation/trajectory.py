"""MORAI GT trajectory extraction without legacy evaluator dependencies."""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float
    longitudinal_velocity: float
    lateral_velocity: float
    longitudinal_acceleration: float
    lateral_acceleration: float
    class_name: str
    object_id: int
    asset_name: str
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0


@dataclass(frozen=True)
class TrajectorySegment:
    key: str
    class_name: str
    object_id: int
    asset_name: str
    points: tuple[TrajectoryPoint, ...]


@dataclass
class _ActiveTrack:
    internal_id: int
    class_name: str
    points: list[TrajectoryPoint]


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def morai_heading_radians(
    heading_deg: float, heading_correction_deg: float = 0.0
) -> float:
    """Convert a MORAI heading after applying its launcher convention.

    The correction belongs to the recorded launcher/interface contract, not
    to ROS versus UDP transport. MR recordings currently require -1.347 deg;
    audited MRP synchronized captures require 0 deg.
    """
    return normalize_angle(math.radians(heading_deg + heading_correction_deg))


def unwrap_angles(angles: Sequence[float]) -> list[float]:
    if not angles:
        return []
    output = [angles[0]]
    for angle in angles[1:]:
        output.append(output[-1] + normalize_angle(angle - output[-1]))
    return output


def _message_timestamp(message, bag_timestamp) -> float:
    stamp = message.header.stamp
    if stamp and (stamp.secs != 0 or stamp.nsecs != 0):
        return stamp.to_sec()
    return bag_timestamp.to_sec()


def _predict_track_position(
    track: _ActiveTrack, timestamp: float
) -> tuple[float, float]:
    """Propagate a correspondence track from causal position differences.

    MORAI recordings in this repository disagree on whether ObjectStatus
    velocity is encoded in m/s or km/h. Position differences are consistent,
    so identity reconstruction deliberately ignores the velocity field.
    """
    last = track.points[-1]
    if len(track.points) < 2:
        return last.x, last.y
    before = track.points[-2]
    sample_dt = last.timestamp - before.timestamp
    if sample_dt <= 1.0e-6:
        return last.x, last.y
    prediction_dt = max(0.0, timestamp - last.timestamp)
    return (
        last.x + (last.x - before.x) / sample_dt * prediction_dt,
        last.y + (last.y - before.y) / sample_dt * prediction_dt,
    )


def reconstruct_trajectory_segments(
    frames: Sequence[tuple[float, Sequence[TrajectoryPoint]]],
    maximum_gap_sec: float = 0.5,
    maximum_assignment_distance_m: float = 2.0,
    minimum_points: int = 8,
) -> list[TrajectorySegment]:
    """Reconstruct persistent GT identities from frame-local MORAI objects.

    MORAI ``ObjectStatus.unique_id`` is frame-local in some recordings: the
    same physical actor changes ID while an ID is later assigned to another
    actor.  E0 therefore performs deterministic frame-to-frame correspondence
    from GT position, velocity and heading.  This is input reconstruction for
    the oracle benchmark, not an evaluation of the runtime tracker.
    """
    # SciPy is needed only by the offline correspondence reconstruction.  Keep
    # it out of the module import path so the online oracle adapter can reuse
    # TrajectoryPoint and normalization helpers without loading SciPy.
    from scipy.optimize import linear_sum_assignment

    active: dict[int, _ActiveTrack] = {}
    finished: list[_ActiveTrack] = []
    next_internal_id = 1

    for timestamp, observations_value in sorted(frames, key=lambda item: item[0]):
        observations = list(observations_value)

        expired = [
            track_id
            for track_id, track in active.items()
            if timestamp - track.points[-1].timestamp > maximum_gap_sec
        ]
        for track_id in expired:
            finished.append(active.pop(track_id))

        for class_name in sorted({point.class_name for point in observations}):
            class_observations = [
                point for point in observations if point.class_name == class_name
            ]
            class_tracks = [
                track for track in active.values() if track.class_name == class_name
            ]
            matched_observations: set[int] = set()

            if class_tracks and class_observations:
                distances = np.empty(
                    (len(class_tracks), len(class_observations)), dtype=np.float64
                )
                costs = np.empty_like(distances)
                for row, track in enumerate(class_tracks):
                    last = track.points[-1]
                    predicted_x, predicted_y = _predict_track_position(
                        track, timestamp
                    )
                    for column, observation in enumerate(class_observations):
                        distance = math.hypot(
                            predicted_x - observation.x,
                            predicted_y - observation.y,
                        )
                        distances[row, column] = distance
                        # Asset names are useful but not unique. Keep them as
                        # a small tie-breaker rather than an identity key.
                        asset_penalty = (
                            0.25 if observation.asset_name != last.asset_name else 0.0
                        )
                        heading_penalty = 0.05 * abs(
                            normalize_angle(observation.yaw - last.yaw)
                        )
                        costs[row, column] = distance + asset_penalty + heading_penalty

                rows, columns = linear_sum_assignment(costs)
                for row, column in zip(rows, columns):
                    track = class_tracks[int(row)]
                    observation = class_observations[int(column)]
                    dt = timestamp - track.points[-1].timestamp
                    gate = maximum_assignment_distance_m + 2.0 * max(0.0, dt)
                    if distances[row, column] > gate:
                        continue
                    track.points.append(
                        replace(observation, object_id=track.internal_id)
                    )
                    matched_observations.add(int(column))

            for index, observation in enumerate(class_observations):
                if index in matched_observations:
                    continue
                internal_id = next_internal_id
                next_internal_id += 1
                active[internal_id] = _ActiveTrack(
                    internal_id=internal_id,
                    class_name=class_name,
                    points=[replace(observation, object_id=internal_id)],
                )

    finished.extend(active.values())
    segments = []
    for track in sorted(finished, key=lambda value: value.internal_id):
        if len(track.points) < minimum_points:
            continue
        segments.append(
            TrajectorySegment(
                key=f"{track.class_name}:reconstructed:{track.internal_id}",
                class_name=track.class_name,
                object_id=track.internal_id,
                asset_name=track.points[0].asset_name,
                points=tuple(track.points),
            )
        )
    return segments


def read_morai_trajectories(
    bag_path: str | Path,
    topic: str = "/Object_topic",
    include_classes: Iterable[str] = ("vehicle", "pedestrian"),
    maximum_assignment_distance_m: float = 2.0,
    maximum_gap_sec: float = 0.5,
    minimum_points: int = 8,
    heading_correction_deg: float = 0.0,
    asset_name_map: dict[str, str] | None = None,
) -> list[TrajectorySegment]:
    """Read MORAI GT frames and reconstruct persistent actor trajectories."""
    try:
        import rosbag
    except ModuleNotFoundError as error:
        if error.name and error.name.startswith("Cryptodome"):
            raise RuntimeError(
                "ROS Noetic rosbag cannot be imported from the active Conda "
                "environment. Run this evaluator with /usr/bin/python3; the "
                "Fast-BEV Conda environment is not required for E0 evaluation."
            ) from error
        raise

    include = set(include_classes)
    # Some bags contain the exact same ObjectStatusList more than once with an
    # identical header stamp. Retain only the most complete copy of each frame.
    frames_by_timestamp: dict[float, list[TrajectoryPoint]] = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, bag_timestamp in bag.read_messages(topics=[topic]):
            timestamp = _message_timestamp(message, bag_timestamp)
            frame: list[TrajectoryPoint] = []
            groups = (
                ("vehicle", message.npc_list),
                ("pedestrian", message.pedestrian_list),
                ("obstacle", message.obstacle_list),
            )
            for class_name, objects in groups:
                if class_name not in include:
                    continue
                for obj in objects:
                    frame.append(
                        TrajectoryPoint(
                            timestamp=timestamp,
                            x=float(obj.position.x),
                            y=float(obj.position.y),
                            z=float(obj.position.z),
                            yaw=morai_heading_radians(
                                float(obj.heading), heading_correction_deg
                            ),
                            # Retained for diagnostics only. Existing MORAI
                            # bags disagree on m/s versus km/h, so E0 state
                            # estimation uses causal GT position history.
                            longitudinal_velocity=float(obj.velocity.x),
                            lateral_velocity=float(obj.velocity.y),
                            longitudinal_acceleration=float(obj.acceleration.x),
                            lateral_acceleration=float(obj.acceleration.y),
                            class_name=class_name,
                            object_id=int(obj.unique_id),
                            asset_name=(asset_name_map or {}).get(
                                f"{class_name}:{int(obj.unique_id)}", str(obj.name)
                            ),
                            length=float(obj.size.x),
                            width=float(obj.size.y),
                            height=float(obj.size.z),
                        )
                    )

            previous = frames_by_timestamp.get(timestamp)
            if previous is None or len(frame) > len(previous):
                frames_by_timestamp[timestamp] = frame

    return reconstruct_trajectory_segments(
        sorted(frames_by_timestamp.items()),
        maximum_gap_sec=maximum_gap_sec,
        maximum_assignment_distance_m=maximum_assignment_distance_m,
        minimum_points=minimum_points,
    )


def transform_segments_planar(
    segments: Sequence[TrajectorySegment],
    translation_xyz: Sequence[float],
    yaw: float,
) -> list[TrajectorySegment]:
    """Transform MORAI-local GT trajectories into the Lanelet map frame."""
    if len(translation_xyz) != 3:
        raise ValueError("translation_xyz must contain exactly three values")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    transformed = []
    for segment in segments:
        points = []
        for point in segment.points:
            points.append(
                replace(
                    point,
                    x=cosine * point.x - sine * point.y + translation_xyz[0],
                    y=sine * point.x + cosine * point.y + translation_xyz[1],
                    z=point.z + translation_xyz[2],
                    yaw=normalize_angle(point.yaw + yaw),
                )
            )
        transformed.append(replace(segment, points=tuple(points)))
    return transformed


def interpolate_point(
    points: Sequence[TrajectoryPoint],
    timestamp: float,
    maximum_bracket_sec: float,
) -> TrajectoryPoint | None:
    timestamps = [point.timestamp for point in points]
    right = bisect.bisect_left(timestamps, timestamp)
    if right < len(points) and abs(points[right].timestamp - timestamp) < 1.0e-9:
        return points[right]
    if right == 0 or right >= len(points):
        return None
    before = points[right - 1]
    after = points[right]
    dt = after.timestamp - before.timestamp
    if dt <= 0.0 or dt > maximum_bracket_sec:
        return None
    ratio = (timestamp - before.timestamp) / dt
    yaw = before.yaw + ratio * normalize_angle(after.yaw - before.yaw)
    return TrajectoryPoint(
        timestamp=timestamp,
        x=before.x + ratio * (after.x - before.x),
        y=before.y + ratio * (after.y - before.y),
        z=before.z + ratio * (after.z - before.z),
        yaw=yaw,
        longitudinal_velocity=before.longitudinal_velocity
        + ratio * (after.longitudinal_velocity - before.longitudinal_velocity),
        lateral_velocity=before.lateral_velocity
        + ratio * (after.lateral_velocity - before.lateral_velocity),
        longitudinal_acceleration=before.longitudinal_acceleration
        + ratio
        * (after.longitudinal_acceleration - before.longitudinal_acceleration),
        lateral_acceleration=before.lateral_acceleration
        + ratio * (after.lateral_acceleration - before.lateral_acceleration),
        class_name=before.class_name,
        object_id=before.object_id,
        asset_name=before.asset_name,
        length=before.length + ratio * (after.length - before.length),
        width=before.width + ratio * (after.width - before.width),
        height=before.height + ratio * (after.height - before.height),
    )
