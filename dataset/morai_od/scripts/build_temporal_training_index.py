#!/usr/bin/env python3
"""Build split-safe temporal samples and audited velocity labels.

The packaged SaveSensorData dataset is treated as immutable.  This builder
creates lightweight JSONL indices that reference the existing sensor files;
it never copies or rewrites images or point clouds.

The canonical input is three causal sweeps.  Past LiDAR points are painted at
their own capture frame, then transformed into the target LiDAR frame using
the recorded Ego poses.  Scan-internal point deskew is intentionally outside
this D3 contract because SaveSensorData has no exact point timestamps.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATASET = Path(
    "/home/luv/morai_cap/morai_temporal_sequence_v2_base2_10504_final"
)
DEFAULT_OUTPUT = (
    REPO_ROOT / "artifacts/datasets/morai_temporal_base2_10k_d3"
)
SPLITS = ("train", "val", "test")
CAMERA_DIRS = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")
STATIC_FAMILIES = {
    "static_obstacle",
    "static_highway_obstacle",
    "realistic_mixed_static",
    "sparse_background",
    "standing_pedestrian",
    "stopped_vehicle",
    "gps_shadow_obstacle",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--sweep-count", type=int, default=3)
    parser.add_argument("--maximum-state-age-sec", type=float, default=0.10)
    parser.add_argument("--minimum-track-frames", type=int, default=3)
    parser.add_argument("--speed-error-base-mps", type=float, default=0.75)
    parser.add_argument("--speed-error-ratio", type=float, default=0.30)
    parser.add_argument("--maximum-direction-error-deg", type=float, default=45.0)
    parser.add_argument("--maximum-derived-speed-mps", type=float, default=45.0)
    parser.add_argument("--maximum-raw-speed-mps", type=float, default=60.0)
    parser.add_argument("--point-alignment-audit-clips", type=int, default=64)
    parser.add_argument("--minimum-sweep-interval-sec", type=float, default=0.04)
    parser.add_argument("--maximum-sweep-interval-sec", type=float, default=0.14)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    with path.open("r", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def write_jsonl(path: Path, rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for row in rows:
            stream.write(
                json.dumps(row, ensure_ascii=False, separators=(",", ":"))
                + "\n"
            )


def write_csv(
    path: Path,
    fieldnames: Sequence[str],
    rows: Iterable[Mapping[str, Any]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fieldnames, extrasaction="ignore"
        )
        writer.writeheader()
        writer.writerows(rows)


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def summarize(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {
            "count": 0,
            "mean": None,
            "p50": None,
            "p90": None,
            "p95": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "min": min(values),
        "max": max(values),
    }


def vector_norm(vector: Mapping[str, Any]) -> float:
    return math.hypot(float(vector["x"]), float(vector["y"]))


def rotate_xy(vector: Mapping[str, Any], heading_deg: float) -> Dict[str, float]:
    yaw = math.radians(float(heading_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return {
        "x": cosine * float(vector["x"]) - sine * float(vector["y"]),
        "y": sine * float(vector["x"]) + cosine * float(vector["y"]),
        "z": float(vector.get("z", 0.0)),
    }


def rotate_world_to_ego(
    vector: Mapping[str, Any], ego_heading_deg: float
) -> Dict[str, float]:
    yaw = math.radians(float(ego_heading_deg))
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return {
        "x": cosine * float(vector["x"]) + sine * float(vector["y"]),
        "y": -sine * float(vector["x"]) + cosine * float(vector["y"]),
        "z": float(vector.get("z", 0.0)),
    }


def vector_error(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    return math.hypot(
        float(first["x"]) - float(second["x"]),
        float(first["y"]) - float(second["y"]),
    )


def direction_error_deg(
    first: Mapping[str, Any], second: Mapping[str, Any]
) -> Optional[float]:
    if vector_norm(first) < 0.5 or vector_norm(second) < 0.5:
        return None
    difference = math.atan2(float(first["y"]), float(first["x"])) - math.atan2(
        float(second["y"]), float(second["x"])
    )
    return abs(math.degrees(math.atan2(math.sin(difference), math.cos(difference))))


def matmul(first: Sequence[Sequence[float]], second: Sequence[Sequence[float]]) -> List[List[float]]:
    return [
        [
            sum(float(first[row][inner]) * float(second[inner][column]) for inner in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def transform_point(matrix: Sequence[Sequence[float]], point: Mapping[str, Any]) -> Dict[str, float]:
    values = [float(point["x"]), float(point["y"]), float(point.get("z", 0.0)), 1.0]
    output = [
        sum(float(matrix[row][column]) * values[column] for column in range(4))
        for row in range(4)
    ]
    return {"x": output[0], "y": output[1], "z": output[2]}


def parse_lidar_ids(path: Path) -> Dict[int, str]:
    result: Dict[int, str] = {}
    if not path.is_file():
        return result
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 15:
                continue
            try:
                result[int(fields[14])] = fields[0]
            except ValueError:
                continue
    return result


def parse_lidar_centres(path: Path) -> Dict[int, Dict[str, Any]]:
    """Read sensor-frame box centres from MORAI LiDAR Instance.txt."""

    result: Dict[int, Dict[str, Any]] = {}
    if not path.is_file():
        return result
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 15:
                continue
            try:
                unique_id = int(fields[14])
                centre = {
                    "x": float(fields[1]),
                    "y": float(fields[2]),
                    "z": float(fields[3]),
                }
            except ValueError:
                continue
            result[unique_id] = {"class": fields[0], "centre": centre}
    return result


def frame_artifacts(frame: Mapping[str, Any]) -> Dict[str, Any]:
    files = [str(value) for value in frame.get("sensor_files", [])]

    def one(prefix: str, suffix: str) -> Optional[str]:
        values = [value for value in files if value.startswith(prefix + "/") and value.endswith(suffix)]
        return values[0] if len(values) == 1 else None

    cameras = {}
    for directory in CAMERA_DIRS:
        cameras[directory] = {
            "intensity": one(directory, "_Intensity.png"),
        }
    return {
        "cameras": cameras,
        "lidar": {
            "intensity": one("LIDAR_5", "_Intensity.bin"),
            "instances": one("LIDAR_5", "_Instance.txt"),
            "instance_corners": one("LIDAR_5", "_Instance_8Points.txt"),
        },
    }


def actor_key(actor: Mapping[str, Any]) -> Tuple[str, int]:
    return str(actor["class"]), int(actor["unique_id"])


def actor_map(state: Optional[Mapping[str, Any]]) -> Dict[Tuple[str, int], Mapping[str, Any]]:
    if not state:
        return {}
    return {actor_key(actor): actor for actor in state.get("actors", [])}


def nearest_timing_sample(
    rows: Sequence[Mapping[str, Any]],
    stream: str,
    trigger_monotonic: float,
) -> Optional[Mapping[str, Any]]:
    candidates = [row for row in rows if row.get("stream") == stream]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda row: abs(float(row["callback_monotonic"]) - trigger_monotonic),
    )


def robust_track_velocity(
    frames: Sequence[Mapping[str, Any]],
    target_index: int,
    key: Tuple[str, int],
    radius: int = 2,
) -> Tuple[Optional[Dict[str, float]], int, Optional[float]]:
    """Fit world position against physical sequence time around a target."""

    observations = []
    for index in range(max(0, target_index - radius), min(len(frames), target_index + radius + 1)):
        objects = frames[index].get("objects") or {}
        actor = actor_map(objects).get(key)
        if actor is None:
            continue
        observations.append(
            (
                float(frames[index]["sequence_time_sec"]),
                actor["position_m"],
                index,
            )
        )
    if len(observations) < 2:
        return None, len(observations), None
    times = [row[0] for row in observations]
    mean_time = sum(times) / len(times)
    denominator = sum((value - mean_time) ** 2 for value in times)
    if denominator <= 1.0e-8:
        return None, len(observations), None
    velocity = {}
    residual_squared = 0.0
    for axis in ("x", "y", "z"):
        positions = [float(row[1][axis]) for row in observations]
        mean_position = sum(positions) / len(positions)
        slope = sum(
            (time_value - mean_time) * (position - mean_position)
            for time_value, position in zip(times, positions)
        ) / denominator
        velocity[axis] = slope
        intercept = mean_position - slope * mean_time
        residual_squared += sum(
            (position - (intercept + slope * time_value)) ** 2
            for time_value, position in zip(times, positions)
        )
    residual = math.sqrt(residual_squared / len(observations))
    return velocity, len(observations), residual


def object_track_lengths(frames: Sequence[Mapping[str, Any]]) -> Counter:
    counts: Counter = Counter()
    for frame in frames:
        for key in actor_map(frame.get("objects") or {}):
            counts[key] += 1
    return counts


def make_velocity_label(
    frame: Mapping[str, Any],
    frames: Sequence[Mapping[str, Any]],
    target_index: int,
    lidar_id: int,
    lidar_class: str,
    selected_objects: Optional[Mapping[str, Any]],
    selected_age_sec: Optional[float],
    track_lengths: Mapping[Tuple[str, int], int],
    args: argparse.Namespace,
) -> Dict[str, Any]:
    embedded = actor_map(frame.get("objects") or {})
    candidates = [key for key in embedded if key[1] == int(lidar_id)]
    key = candidates[0] if len(candidates) == 1 else None
    embedded_actor = embedded.get(key) if key else None
    selected_actor = actor_map(selected_objects).get(key) if key else None
    reference_actor = selected_actor or embedded_actor
    invalid_reasons = []
    if key is None:
        invalid_reasons.append("object_status_identity_missing")
    if selected_actor is None:
        invalid_reasons.append("timing_actor_missing")
    track_length = int(track_lengths.get(key, 0)) if key else 0
    identity_continuous = track_length >= int(args.minimum_track_frames)
    if not identity_continuous:
        invalid_reasons.append("short_identity_track")
    timestamp_valid = (
        selected_age_sec is not None
        and selected_age_sec <= float(args.maximum_state_age_sec)
    )
    if not timestamp_valid:
        invalid_reasons.append("stale_object_state")

    derived, derived_count, fit_residual = (
        robust_track_velocity(frames, target_index, key)
        if key
        else (None, 0, None)
    )
    derived_speed = vector_norm(derived) if derived is not None else None
    position_continuous = (
        derived_speed is not None
        and derived_speed <= float(args.maximum_derived_speed_mps)
        and fit_residual is not None
        and fit_residual <= 1.0
    )
    if derived is None:
        invalid_reasons.append("derived_velocity_missing")
    elif not position_continuous:
        invalid_reasons.append("position_discontinuity")

    raw_local_mps = None
    raw_local_to_world = None
    raw_as_world = None
    raw_speed = None
    if reference_actor and reference_actor.get("morai_velocity_raw") is not None:
        raw = reference_actor["morai_velocity_raw"]
        raw_local_mps = {
            axis: float(raw.get(axis, 0.0)) / 3.6 for axis in ("x", "y", "z")
        }
        raw_local_to_world = rotate_xy(
            raw_local_mps, float(reference_actor["heading_deg"])
        )
        raw_as_world = dict(raw_local_mps)
        raw_speed = vector_norm(raw_local_to_world)
    else:
        invalid_reasons.append("raw_velocity_missing")

    speed_error = (
        abs(raw_speed - derived_speed)
        if raw_speed is not None and derived_speed is not None
        else None
    )
    vector_error_local_hypothesis = (
        vector_error(raw_local_to_world, derived)
        if raw_local_to_world is not None and derived is not None
        else None
    )
    vector_error_world_hypothesis = (
        vector_error(raw_as_world, derived)
        if raw_as_world is not None and derived is not None
        else None
    )
    direction_error = (
        direction_error_deg(raw_local_to_world, derived)
        if raw_local_to_world is not None and derived is not None
        else None
    )
    speed_threshold = (
        max(
            float(args.speed_error_base_mps),
            float(args.speed_error_ratio) * max(raw_speed or 0.0, derived_speed or 0.0),
        )
        if raw_speed is not None and derived_speed is not None
        else None
    )
    raw_derived_consistent = (
        speed_error is not None
        and speed_threshold is not None
        and speed_error <= speed_threshold
        and (direction_error is None or direction_error <= float(args.maximum_direction_error_deg))
    )
    if not raw_derived_consistent:
        invalid_reasons.append("raw_derived_disagreement")

    family = str(frame.get("mission_family", ""))
    class_name = str(key[0]) if key else "unknown"
    static_candidate = family in STATIC_FAMILIES or class_name == "obstacle"
    static_confirmed = bool(
        static_candidate
        and raw_speed is not None
        and derived_speed is not None
        and raw_speed <= 0.15
        and derived_speed <= 0.25
        and position_continuous
    )

    raw_plausible = bool(
        raw_speed is not None
        and math.isfinite(raw_speed)
        and raw_speed <= float(args.maximum_raw_speed_mps)
    )
    if raw_speed is not None and not raw_plausible:
        invalid_reasons.append("raw_velocity_implausible")
    raw_valid = bool(
        raw_local_to_world is not None
        and raw_plausible
        and timestamp_valid
        and identity_continuous
        and position_continuous
    )
    if static_confirmed:
        source = "static_zero"
        world_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        velocity_valid = True
        loss_weight = 1.0
    elif raw_valid:
        source = "morai_raw_local_rotated"
        world_velocity = raw_local_to_world
        velocity_valid = True
        age_score = max(0.0, 1.0 - float(selected_age_sec or 0.0) / float(args.maximum_state_age_sec))
        consistency_score = max(0.0, 1.0 - float(speed_error or 0.0) / max(float(speed_threshold or 1.0), 1.0e-6))
        loss_weight = max(0.25, min(1.0, 0.5 * age_score + 0.5 * consistency_score))
        if not raw_derived_consistent:
            # Raw is the simulator's direct state.  A disagreement therefore
            # lowers confidence but does not let a noisy endpoint position fit
            # replace otherwise fresh, plausible telemetry.
            loss_weight = min(loss_weight, 0.25)
    elif derived is not None and position_continuous and identity_continuous:
        source = "derived_diagnostic"
        world_velocity = derived
        velocity_valid = False
        loss_weight = 0.0
    else:
        source = "invalid"
        world_velocity = None
        velocity_valid = False
        loss_weight = 0.0

    ego_heading = float(frame["ego"]["heading_deg"])
    ego_velocity = (
        rotate_world_to_ego(world_velocity, ego_heading)
        if world_velocity is not None
        else None
    )
    return {
        "unique_id": int(lidar_id),
        "class": class_name,
        "lidar_raw_class": lidar_class,
        "name": reference_actor.get("name") if reference_actor else None,
        "heading_deg": float(reference_actor["heading_deg"]) if reference_actor else None,
        "velocity_target_world_mps": world_velocity,
        "velocity_target_ego_mps": ego_velocity,
        "velocity_reference": "save_sensor_trigger_nearest_object_status",
        "velocity_source": source,
        "velocity_valid": velocity_valid,
        "velocity_loss_weight": loss_weight,
        "velocity_time_offset_sec": selected_age_sec,
        "raw_local_mps": raw_local_mps,
        "raw_local_to_world_mps": raw_local_to_world,
        "raw_as_world_hypothesis_mps": raw_as_world,
        "derived_velocity_world_mps": derived,
        "derived_track_observation_count": derived_count,
        "derived_fit_residual_m": fit_residual,
        "raw_derived_speed_error_mps": speed_error,
        "raw_derived_direction_error_deg": direction_error,
        "local_rotated_vector_error_mps": vector_error_local_hypothesis,
        "raw_as_world_vector_error_mps": vector_error_world_hypothesis,
        "quality": {
            "identity_continuous": identity_continuous,
            "timestamp_valid": timestamp_valid,
            "raw_derived_consistent": raw_derived_consistent,
            "raw_plausible": raw_plausible,
            "position_continuous": position_continuous,
            "static_confirmed": static_confirmed,
            "track_length": track_length,
            "invalid_reasons": sorted(set(invalid_reasons)),
        },
    }


def relative_transform(
    sweep: Mapping[str, Any],
    target: Mapping[str, Any],
    calibration: Mapping[str, Any],
) -> Tuple[List[List[float]], List[List[float]]]:
    target_global_to_ego = target["transforms"]["global_to_ego"]
    sweep_ego_to_global = sweep["transforms"]["ego_to_global"]
    ego_to_target_ego = matmul(target_global_to_ego, sweep_ego_to_global)
    lidar_to_target_lidar = matmul(
        calibration["lidar"]["ego_to_lidar"],
        matmul(
            ego_to_target_ego,
            calibration["lidar"]["lidar_to_ego"],
        ),
    )
    return ego_to_target_ego, lidar_to_target_lidar


def static_alignment_residual(
    sweep: Mapping[str, Any],
    target: Mapping[str, Any],
    transform: Sequence[Sequence[float]],
    key: Tuple[str, int],
) -> Optional[Tuple[float, float]]:
    sweep_actor = actor_map(sweep.get("objects") or {}).get(key)
    target_actor = actor_map(target.get("objects") or {}).get(key)
    if sweep_actor is None or target_actor is None:
        return None
    sweep_relative = sweep_actor.get("relative_to_ego", {}).get("position_m")
    target_relative = target_actor.get("relative_to_ego", {}).get("position_m")
    if sweep_relative is None or target_relative is None:
        return None
    before = math.hypot(
        float(sweep_relative["x"]) - float(target_relative["x"]),
        float(sweep_relative["y"]) - float(target_relative["y"]),
    )
    aligned = transform_point(transform, sweep_relative)
    after = math.hypot(
        aligned["x"] - float(target_relative["x"]),
        aligned["y"] - float(target_relative["y"]),
    )
    return before, after


def audit_point_alignment(
    dataset: Path,
    candidates: Sequence[Mapping[str, Any]],
) -> Dict[str, Any]:
    """Compare raw and Ego-aligned static-scene LiDAR geometry."""

    import numpy as np
    from scipy.spatial import cKDTree

    before_distances = []
    after_distances = []
    completed = 0
    errors = []

    def load_points(relative_path: str) -> Any:
        values = np.fromfile(str(dataset / relative_path), dtype=np.float32)
        if values.size % 4:
            raise ValueError("LiDAR float count is not divisible by four")
        points = values.reshape((-1, 4))[:, :3]
        ranges = np.linalg.norm(points[:, :2], axis=1)
        mask = (
            np.isfinite(points).all(axis=1)
            & (ranges >= 3.0)
            & (ranges <= 55.0)
            & (points[:, 2] >= -3.0)
            & (points[:, 2] <= 5.0)
        )
        return points[mask][::4]

    for row in candidates:
        try:
            sweep = load_points(str(row["sweep_intensity"]))
            target = load_points(str(row["target_intensity"]))
            if len(sweep) < 100 or len(target) < 100:
                continue
            tree = cKDTree(target)
            raw_distance, _ = tree.query(sweep, k=1)
            homogeneous = np.concatenate(
                [sweep, np.ones((len(sweep), 1), dtype=sweep.dtype)], axis=1
            )
            transform = np.asarray(row["lidar_to_target_lidar"], dtype=np.float64)
            aligned = (homogeneous @ transform.T)[:, :3]
            aligned_distance, _ = tree.query(aligned, k=1)
            # Unseen/occluded surfaces have no exact correspondence.  Capping
            # them prevents visibility changes from dominating the transform
            # sanity check while retaining sub-metre geometric changes.
            before_distances.extend(np.minimum(raw_distance, 2.0).tolist())
            after_distances.extend(np.minimum(aligned_distance, 2.0).tolist())
            completed += 1
        except (OSError, ValueError) as error:
            errors.append("{}: {}".format(row.get("sample_id"), error))
    before_summary = summarize(before_distances)
    after_summary = summarize(after_distances)
    improvement = None
    if before_summary["p50"] not in (None, 0.0) and after_summary["p50"] is not None:
        improvement = 1.0 - float(after_summary["p50"]) / float(before_summary["p50"])
    return {
        "requested_clip_count": len(candidates),
        "completed_clip_count": completed,
        "before_alignment_nearest_residual_m": before_summary,
        "after_alignment_nearest_residual_m": after_summary,
        "p50_relative_improvement": improvement,
        "errors": errors,
        "method": (
            "Static-family LiDAR xyz, 3-55m range, every fourth point, "
            "target-sweep nearest neighbor, residual capped at 2m."
        ),
    }


def audit_saved_box_alignment(
    dataset: Path,
    candidates: Sequence[Mapping[str, Any]],
) -> Dict[str, Any]:
    """Validate Ego alignment against GT centres saved with each LiDAR frame."""

    before = []
    after = []
    matched_boxes = 0
    completed = 0
    errors = []
    for row in candidates:
        try:
            sweep = parse_lidar_centres(dataset / str(row["sweep_instances"]))
            target = parse_lidar_centres(dataset / str(row["target_instances"]))
            shared_ids = sorted(set(sweep) & set(target))
            for unique_id in shared_ids:
                source = sweep[unique_id]["centre"]
                destination = target[unique_id]["centre"]
                aligned = transform_point(row["lidar_to_target_lidar"], source)
                before.append(vector_error(source, destination))
                after.append(vector_error(aligned, destination))
                matched_boxes += 1
            completed += 1
        except (OSError, ValueError) as error:
            errors.append("{}: {}".format(row.get("sample_id"), error))
    before_summary = summarize(before)
    after_summary = summarize(after)
    improvement = None
    if before_summary["p50"] not in (None, 0.0) and after_summary["p50"] is not None:
        improvement = 1.0 - float(after_summary["p50"]) / float(before_summary["p50"])
    return {
        "requested_clip_count": len(candidates),
        "completed_clip_count": completed,
        "matched_box_count": matched_boxes,
        "before_alignment_residual_m": before_summary,
        "after_alignment_residual_m": after_summary,
        "p50_relative_improvement": improvement,
        "errors": errors,
        "method": (
            "Match LiDAR Instance.txt GT centres by unique_id between the oldest "
            "and target sweeps, then apply T_target_lidar_from_sweep_lidar."
        ),
    }


def render_audit_markdown(audit: Mapping[str, Any]) -> str:
    def number(value: Optional[float], digits: int = 4) -> str:
        return "—" if value is None else ("{:.%df}" % digits).format(float(value))

    lines = [
        "# Base2-10K D3 temporal label and index audit",
        "",
        "## Release result.",
        "",
        "- Status: `{}`.".format(audit["status"]),
        "- Complete clips: `{}`.".format(audit["clip_count"]),
        "- Causal 3-sweep samples: `{}`.".format(audit["temporal_sample_count"]),
        "- LiDAR-object velocity labels: `{}`.".format(audit["velocity_label_count"]),
        "- Split overlap, duplicate sample and future-input count are all zero.",
        "",
        "## Samples by split.",
        "",
        "| Split | Clips | Samples |",
        "| --- | ---: | ---: |",
    ]
    for split in SPLITS:
        lines.append(
            "| {} | {} | {} |".format(
                split,
                audit["complete_clip_count_by_split"].get(split, 0),
                audit["temporal_sample_count_by_split"].get(split, 0),
            )
        )
    lines.extend([
        "",
        "## Velocity source.",
        "",
        "| Source | Labels | Production velocity loss |",
        "| --- | ---: | --- |",
    ])
    for source, count in sorted(audit["velocity_source_counts"].items()):
        production = "yes" if source in {"morai_raw_local_rotated", "static_zero"} else "no"
        lines.append("| {} | {} | {} |".format(source, count, production))
    lines.extend([
        "",
        "- MORAI ObjectStatus velocity is object-local km/h in the personal launcher schema.",
        "- It is divided by 3.6 and rotated by actor heading into `morai_map`.",
        "- Derived velocity is a position-fit diagnostic and is not a production target by default.",
        "- Raw/derived disagreement lowers loss weight to 0.25 instead of replacing fresh raw telemetry.",
        "",
        "## Timestamp and temporal lag.",
        "",
        "- Object state age p50/p95: `{}` / `{}` s.".format(
            number(audit["object_state_age_sec"]["p50"]),
            number(audit["object_state_age_sec"]["p95"]),
        ),
        "- Irregular temporal samples outside the configured interval: `{}`.".format(
            audit["temporal_timing_invalid_sample_count"]
        ),
    ])
    for position, summary in sorted(audit["time_lag_sec_by_sweep_position"].items()):
        lines.append(
            "- Sweep position {} lag p50/p95: `{}` / `{}` s.".format(
                position, number(summary["p50"]), number(summary["p95"])
            )
        )
    axes = audit["velocity_axis_hypotheses"]
    lines.extend([
        "",
        "## Velocity coordinate hypothesis.",
        "",
        "| Interpretation | Moving-object vector error p50 | p95 |",
        "| --- | ---: | ---: |",
        "| Object-local, rotated by heading | {} m/s | {} m/s |".format(
            number(axes["object_local_rotated_to_world_error_mps"]["p50"]),
            number(axes["object_local_rotated_to_world_error_mps"]["p95"]),
        ),
        "| Raw XY treated as world | {} m/s | {} m/s |".format(
            number(axes["raw_xy_treated_as_world_error_mps"]["p50"]),
            number(axes["raw_xy_treated_as_world_error_mps"]["p95"]),
        ),
        "",
        "The lower-error object-local interpretation is the canonical D3 label convention.",
        "",
        "## Ego-motion alignment.",
        "",
    ])
    point = audit["static_alignment"]["point_cloud_audit"]
    boxes = audit["static_alignment"]["saved_lidar_box_audit"]
    lines.extend([
        "- ObjectStatus geometry residual p50 before/after: `{}` / `{}` m.".format(
            number(audit["static_alignment"]["before_alignment_residual_m"]["p50"]),
            number(audit["static_alignment"]["after_alignment_residual_m"]["p50"]),
        ),
        "- Saved LiDAR GT box-centre residual p50 before/after: `{}` / `{}` m.".format(
            number(boxes["before_alignment_residual_m"]["p50"]),
            number(boxes["after_alignment_residual_m"]["p50"]),
        ),
        "- Saved LiDAR GT box matches: `{}`.".format(boxes["matched_box_count"]),
        "- Unlabelled full-cloud NN residual p50 before/after: `{}` / `{}` m.".format(
            number(point["before_alignment_nearest_residual_m"]["p50"]),
            number(point["after_alignment_nearest_residual_m"]["p50"]),
        ),
        "- Point-cloud audit clips: `{}/{}`.".format(
            point["completed_clip_count"], point["requested_clip_count"]
        ),
        "- Dense road surfaces and visibility changes make raw full-cloud NN non-identifiable; it is diagnostic only.",
        "- Alignment is sweep-to-sweep Ego motion only; scan-internal point deskew remains disabled.",
        "",
        "## Quality-mask policy.",
        "",
        "- Production-valid sources: `morai_raw_local_rotated`, `static_zero`.",
        "- `derived_diagnostic` remains available for V3 ablation but has zero production loss weight.",
        "- Identity continuity, state age, raw plausibility, position continuity and raw-derived consistency are stored per label.",
        "- Geometry labels remain usable even when velocity loss is masked.",
        "- `temporal_input_valid=false` masks rare irregular sweep intervals from temporal fusion training.",
        "",
        "## D3 outputs.",
        "",
        "- `temporal_samples.jsonl`: three causal sweeps and relative transforms.",
        "- `velocity_labels.jsonl`: target-frame object velocity and quality masks.",
        "- `temporal_schema.json`: coordinate, timestamp and source conventions.",
        "- `sample_summary.csv`: per-sample label coverage.",
        "- `splits/*_samples.txt`: split-safe sample IDs.",
        "- `audit.json`: machine-readable release audit.",
        "",
    ])
    return "\n".join(lines)


def write_audit_plots(
    output: Path,
    lag_by_position: Mapping[int, Sequence[float]],
    local_errors: Sequence[float],
    world_errors: Sequence[float],
    alignment_before: Sequence[float],
    alignment_after: Sequence[float],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(8, 4.5))
    for position, values in sorted(lag_by_position.items()):
        axis.hist(values, bins=50, alpha=0.55, label="sweep {}".format(position))
    axis.set_xlabel("time lag [s]")
    axis.set_ylabel("samples")
    axis.set_title("Actual causal sweep lag")
    axis.legend()
    figure.tight_layout()
    figure.savefig(str(output / "time_lag_distribution.png"), dpi=150)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.hist(local_errors, bins=80, range=(0.0, 5.0), alpha=0.65, label="object-local rotated")
    axis.hist(world_errors, bins=80, range=(0.0, 5.0), alpha=0.50, label="raw XY as world")
    axis.set_xlabel("moving-object vector error [m/s]")
    axis.set_ylabel("labels")
    axis.set_title("ObjectStatus velocity coordinate hypothesis")
    axis.legend()
    figure.tight_layout()
    figure.savefig(str(output / "velocity_axis_hypothesis.png"), dpi=150)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8, 4.5))
    axis.hist(alignment_before, bins=80, range=(0.0, 3.0), alpha=0.60, label="before")
    axis.hist(alignment_after, bins=80, range=(0.0, 3.0), alpha=0.60, label="after")
    axis.set_xlabel("static box-centre residual [m]")
    axis.set_ylabel("observations")
    axis.set_title("Sweep Ego-motion alignment sanity check")
    axis.legend()
    figure.tight_layout()
    figure.savefig(str(output / "ego_motion_alignment.png"), dpi=150)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    dataset = args.dataset.expanduser().resolve()
    output = args.output_dir.expanduser().resolve()
    if not dataset.is_dir():
        raise RuntimeError("dataset does not exist: {}".format(dataset))
    if output.exists() and any(output.iterdir()) and not args.overwrite:
        raise RuntimeError(
            "output is not empty; pass --overwrite: {}".format(output)
        )
    output.mkdir(parents=True, exist_ok=True)
    calibration = read_json(dataset / "metadata/calibration.json")

    clip_paths = sorted((dataset / "metadata/clips").glob("*.clip.json"))
    temporal_rows = []
    velocity_rows = []
    sample_rows = []
    split_ids = {split: [] for split in SPLITS}
    errors = []
    source_counts: Counter = Counter()
    source_by_class: Dict[str, Counter] = defaultdict(Counter)
    family_counts: Counter = Counter()
    label_counts_by_class: Counter = Counter()
    valid_label_counts_by_class: Counter = Counter()
    invalid_reasons: Counter = Counter()
    lag_by_position: Dict[int, List[float]] = defaultdict(list)
    state_ages = []
    speed_errors = []
    direction_errors = []
    speed_errors_by_class: Dict[str, List[float]] = defaultdict(list)
    direction_errors_by_class: Dict[str, List[float]] = defaultdict(list)
    speed_errors_by_family: Dict[str, List[float]] = defaultdict(list)
    valid_loss_weights_by_class: Dict[str, List[float]] = defaultdict(list)
    local_hypothesis_errors = []
    world_hypothesis_errors = []
    alignment_before = []
    alignment_after = []
    point_alignment_candidates = []
    ego_local_hypothesis_errors = []
    ego_world_hypothesis_errors = []
    clip_counts: Counter = Counter()
    label_count = 0
    temporal_timing_invalid_count = 0
    inter_sweep_intervals = []
    checked_required_artifacts = set()
    missing_required_artifacts = set()

    for clip_path in clip_paths:
        clip = read_json(clip_path)
        clip_id = str(clip["clip_id"])
        frames_path = clip_path.with_name(clip_id + ".frames.jsonl")
        timing_path = clip_path.with_name(clip_id + ".timing.jsonl")
        frames = read_jsonl(frames_path)
        timing_rows = read_jsonl(timing_path)
        split = str(clip["split"])
        if split not in SPLITS:
            errors.append("{} has invalid split {}".format(clip_id, split))
            continue
        if len(frames) != int(clip.get("frame_count", 0)) or len(frames) != 8:
            errors.append("{} is not a complete 8-frame clip".format(clip_id))
            continue
        if any(str(frame.get("clip_id")) != clip_id for frame in frames):
            errors.append("{} contains a foreign frame".format(clip_id))
            continue
        times = [float(frame["sequence_time_sec"]) for frame in frames]
        if any(after <= before for before, after in zip(times, times[1:])):
            errors.append("{} has non-increasing sequence time".format(clip_id))
            continue
        track_lengths = object_track_lengths(frames)
        clip_counts[split] += 1

        for target_index in range(int(args.sweep_count) - 1, len(frames)):
            target = frames[target_index]
            sample_id = str(target["frame_id"])
            sweeps = frames[target_index - int(args.sweep_count) + 1 : target_index + 1]
            sweep_rows = []
            for position, sweep in enumerate(sweeps):
                ego_transform, lidar_transform = relative_transform(
                    sweep, target, calibration
                )
                artifacts = frame_artifacts(sweep)
                required_artifacts = [
                    artifacts["cameras"][directory]["intensity"]
                    for directory in CAMERA_DIRS
                ] + [
                    artifacts["lidar"]["intensity"],
                    artifacts["lidar"]["instances"],
                ]
                for relative_path in required_artifacts:
                    if relative_path in checked_required_artifacts:
                        continue
                    checked_required_artifacts.add(relative_path)
                    if relative_path is None or not (dataset / relative_path).is_file():
                        missing_required_artifacts.add(relative_path)
                lag = float(sweep["sequence_time_sec"]) - float(target["sequence_time_sec"])
                lag_by_position[position].append(lag)
                sweep_rows.append(
                    {
                        "frame_id": sweep["frame_id"],
                        "frame_index": int(sweep["frame_index"]),
                        "time_lag_sec": lag,
                        "sequence_time_sec": float(sweep["sequence_time_sec"]),
                        "artifacts": artifacts,
                        "ego_pose": sweep["ego"],
                        "ego_to_target_ego": ego_transform,
                        "lidar_to_target_lidar": lidar_transform,
                    }
                )

            ego_raw = target.get("ego", {}).get("velocity_raw")
            ego_derived = target.get("ego", {}).get("derived_velocity_mps")
            if ego_raw is not None and ego_derived is not None:
                ego_raw_local_to_world = rotate_xy(
                    ego_raw, float(target["ego"]["heading_deg"])
                )
                ego_local_hypothesis_errors.append(
                    vector_error(ego_raw_local_to_world, ego_derived)
                )
                ego_world_hypothesis_errors.append(
                    vector_error(ego_raw, ego_derived)
                )

            sweep_intervals = [
                float(after["sequence_time_sec"]) - float(before["sequence_time_sec"])
                for before, after in zip(sweep_rows, sweep_rows[1:])
            ]
            inter_sweep_intervals.extend(sweep_intervals)
            temporal_timing_valid = all(
                float(args.minimum_sweep_interval_sec) <= interval
                <= float(args.maximum_sweep_interval_sec)
                for interval in sweep_intervals
            )
            temporal_quality_reasons = []
            if not temporal_timing_valid:
                temporal_timing_invalid_count += 1
                temporal_quality_reasons.append("irregular_sweep_interval")

            nearest_objects_sample = nearest_timing_sample(
                timing_rows,
                "objects",
                float(target["triggered_monotonic"]),
            )
            selected_objects = (
                nearest_objects_sample.get("state")
                if nearest_objects_sample is not None
                else None
            )
            selected_age = (
                abs(
                    float(nearest_objects_sample["callback_monotonic"])
                    - float(target["triggered_monotonic"])
                )
                if nearest_objects_sample is not None
                else None
            )
            if selected_age is not None:
                state_ages.append(selected_age)

            target_artifacts = frame_artifacts(target)
            if (
                str(target.get("mission_family")) in STATIC_FAMILIES
                and target_index == int(args.sweep_count) - 1
                and len(point_alignment_candidates)
                < int(args.point_alignment_audit_clips)
            ):
                oldest_lidar = sweep_rows[0]["artifacts"]["lidar"]["intensity"]
                target_lidar = target_artifacts["lidar"]["intensity"]
                oldest_instances = sweep_rows[0]["artifacts"]["lidar"]["instances"]
                target_instances = target_artifacts["lidar"]["instances"]
                if oldest_lidar and target_lidar and oldest_instances and target_instances:
                    point_alignment_candidates.append(
                        {
                            "sample_id": sample_id,
                            "sweep_intensity": oldest_lidar,
                            "target_intensity": target_lidar,
                            "sweep_instances": oldest_instances,
                            "target_instances": target_instances,
                            "lidar_to_target_lidar": sweep_rows[0]["lidar_to_target_lidar"],
                        }
                    )
            instance_relative = target_artifacts["lidar"]["instances"]
            instance_path = dataset / instance_relative if instance_relative else Path()
            lidar_ids = parse_lidar_ids(instance_path) if instance_relative else {}
            labels = []
            for lidar_id, lidar_class in sorted(lidar_ids.items()):
                label = make_velocity_label(
                    target,
                    frames,
                    target_index,
                    lidar_id,
                    lidar_class,
                    selected_objects,
                    selected_age,
                    track_lengths,
                    args,
                )
                labels.append(label)
                label_count += 1
                source = label["velocity_source"]
                class_name = label["class"]
                source_counts[source] += 1
                source_by_class[class_name][source] += 1
                label_counts_by_class[class_name] += 1
                if label["velocity_valid"]:
                    valid_label_counts_by_class[class_name] += 1
                invalid_reasons.update(label["quality"]["invalid_reasons"])
                if label["raw_derived_speed_error_mps"] is not None:
                    speed_errors.append(label["raw_derived_speed_error_mps"])
                    speed_errors_by_class[class_name].append(
                        label["raw_derived_speed_error_mps"]
                    )
                    speed_errors_by_family[str(target["mission_family"])].append(
                        label["raw_derived_speed_error_mps"]
                    )
                if label["raw_derived_direction_error_deg"] is not None:
                    direction_errors.append(label["raw_derived_direction_error_deg"])
                    direction_errors_by_class[class_name].append(
                        label["raw_derived_direction_error_deg"]
                    )
                if label["velocity_valid"]:
                    valid_loss_weights_by_class[class_name].append(
                        float(label["velocity_loss_weight"])
                    )
                derived_vector = label.get("derived_velocity_world_mps")
                if derived_vector is not None and vector_norm(derived_vector) >= 0.5:
                    if label["local_rotated_vector_error_mps"] is not None:
                        local_hypothesis_errors.append(label["local_rotated_vector_error_mps"])
                    if label["raw_as_world_vector_error_mps"] is not None:
                        world_hypothesis_errors.append(label["raw_as_world_vector_error_mps"])

            static_keys = []
            for key, actor in actor_map(target.get("objects") or {}).items():
                derived_speed = actor.get("derived_speed_mps")
                if (
                    key[0] == "obstacle"
                    or (
                        str(target.get("mission_family")) in STATIC_FAMILIES
                        and derived_speed is not None
                        and float(derived_speed) < 0.25
                    )
                ):
                    static_keys.append(key)
            for sweep, sweep_row in zip(sweeps[:-1], sweep_rows[:-1]):
                for key in static_keys:
                    residual = static_alignment_residual(
                        sweep,
                        target,
                        sweep_row["ego_to_target_ego"],
                        key,
                    )
                    if residual is not None:
                        alignment_before.append(residual[0])
                        alignment_after.append(residual[1])

            temporal_rows.append(
                {
                    "schema_version": 1,
                    "dataset_id": "morai_temporal_sequence_v2_base2_10504_final",
                    "sample_id": sample_id,
                    "clip_id": clip_id,
                    "scene_id": target["scene_id"],
                    "split": split,
                    "mission_family": target["mission_family"],
                    "motion_phase": target.get("motion_phase"),
                    "target_frame_id": target["frame_id"],
                    "target_frame_index": int(target["frame_index"]),
                    "target_sequence_time_sec": float(target["sequence_time_sec"]),
                    "sweep_count": int(args.sweep_count),
                    "sweeps_oldest_to_current": sweep_rows,
                    "inter_sweep_intervals_sec": sweep_intervals,
                    "temporal_input_valid": temporal_timing_valid,
                    "temporal_quality_reasons": temporal_quality_reasons,
                    "velocity_labels_ref": sample_id,
                }
            )
            family_counts[str(target["mission_family"])] += 1
            velocity_rows.append(
                {
                    "schema_version": 1,
                    "sample_id": sample_id,
                    "clip_id": clip_id,
                    "split": split,
                    "target_frame_id": target["frame_id"],
                    "target_object_state_age_sec": selected_age,
                    "labels": labels,
                }
            )
            split_ids[split].append(sample_id)
            sample_rows.append(
                {
                    "sample_id": sample_id,
                    "clip_id": clip_id,
                    "split": split,
                    "family": target["mission_family"],
                    "target_frame_index": int(target["frame_index"]),
                    "label_count": len(labels),
                    "valid_velocity_count": sum(label["velocity_valid"] for label in labels),
                    "temporal_input_valid": temporal_timing_valid,
                    "object_state_age_sec": selected_age,
                }
            )

    sample_ids = [row["sample_id"] for row in temporal_rows]
    duplicate_samples = len(sample_ids) - len(set(sample_ids))
    split_overlap = set(split_ids["train"]) & set(split_ids["val"])
    split_overlap |= set(split_ids["train"]) & set(split_ids["test"])
    split_overlap |= set(split_ids["val"]) & set(split_ids["test"])
    if duplicate_samples:
        errors.append("duplicate temporal sample IDs: {}".format(duplicate_samples))
    if split_overlap:
        errors.append("temporal samples overlap across splits: {}".format(len(split_overlap)))
    if missing_required_artifacts:
        errors.append(
            "missing required sensor artifacts: {}".format(
                len(missing_required_artifacts)
            )
        )

    point_alignment = audit_point_alignment(dataset, point_alignment_candidates)
    saved_box_alignment = audit_saved_box_alignment(
        dataset, point_alignment_candidates
    )
    if point_alignment["completed_clip_count"] != len(point_alignment_candidates):
        errors.append("point alignment audit did not complete every selected clip")
    if saved_box_alignment["completed_clip_count"] != len(point_alignment_candidates):
        errors.append("saved box alignment audit did not complete every selected clip")
    if saved_box_alignment["matched_box_count"] == 0:
        errors.append("saved box alignment audit found no shared object identities")

    write_jsonl(output / "temporal_samples.jsonl", temporal_rows)
    write_jsonl(output / "velocity_labels.jsonl", velocity_rows)
    for split in SPLITS:
        (output / "splits").mkdir(parents=True, exist_ok=True)
        (output / "splits" / "{}_samples.txt".format(split)).write_text(
            "".join(sample_id + "\n" for sample_id in split_ids[split]),
            encoding="utf-8",
        )
    write_csv(
        output / "sample_summary.csv",
        (
            "sample_id",
            "clip_id",
            "split",
            "family",
            "target_frame_index",
            "label_count",
            "valid_velocity_count",
            "temporal_input_valid",
            "object_state_age_sec",
        ),
        sample_rows,
    )

    audit = {
        "schema_version": 1,
        "dataset": str(dataset),
        "output": str(output),
        "configuration": {
            "sweep_count": int(args.sweep_count),
            "maximum_state_age_sec": float(args.maximum_state_age_sec),
            "minimum_track_frames": int(args.minimum_track_frames),
            "speed_error_base_mps": float(args.speed_error_base_mps),
            "speed_error_ratio": float(args.speed_error_ratio),
            "maximum_direction_error_deg": float(args.maximum_direction_error_deg),
            "maximum_derived_speed_mps": float(args.maximum_derived_speed_mps),
            "maximum_raw_speed_mps": float(args.maximum_raw_speed_mps),
            "point_alignment_audit_clips": int(args.point_alignment_audit_clips),
            "minimum_sweep_interval_sec": float(args.minimum_sweep_interval_sec),
            "maximum_sweep_interval_sec": float(args.maximum_sweep_interval_sec),
        },
        "clip_count": len(clip_paths),
        "complete_clip_count_by_split": dict(clip_counts),
        "temporal_sample_count": len(temporal_rows),
        "temporal_sample_count_by_split": {key: len(value) for key, value in split_ids.items()},
        "velocity_label_count": label_count,
        "velocity_source_counts": dict(source_counts),
        "velocity_source_counts_by_class": {
            key: dict(value) for key, value in sorted(source_by_class.items())
        },
        "temporal_sample_count_by_family": dict(sorted(family_counts.items())),
        "velocity_label_count_by_class": dict(sorted(label_counts_by_class.items())),
        "valid_velocity_label_count_by_class": dict(
            sorted(valid_label_counts_by_class.items())
        ),
        "valid_velocity_ratio_by_class": {
            key: float(valid_label_counts_by_class[key]) / float(count)
            for key, count in sorted(label_counts_by_class.items())
            if count
        },
        "invalid_reason_counts": dict(invalid_reasons),
        "object_state_age_sec": summarize(state_ages),
        "time_lag_sec_by_sweep_position": {
            str(key): summarize(value) for key, value in sorted(lag_by_position.items())
        },
        "inter_sweep_interval_sec": summarize(inter_sweep_intervals),
        "temporal_timing_invalid_sample_count": temporal_timing_invalid_count,
        "raw_derived_speed_error_mps": summarize(speed_errors),
        "raw_derived_direction_error_deg": summarize(direction_errors),
        "raw_derived_speed_error_mps_by_class": {
            key: summarize(value) for key, value in sorted(speed_errors_by_class.items())
        },
        "raw_derived_direction_error_deg_by_class": {
            key: summarize(value)
            for key, value in sorted(direction_errors_by_class.items())
        },
        "raw_derived_speed_error_mps_by_family": {
            key: summarize(value) for key, value in sorted(speed_errors_by_family.items())
        },
        "valid_velocity_loss_weight_by_class": {
            key: summarize(value)
            for key, value in sorted(valid_loss_weights_by_class.items())
        },
        "velocity_axis_hypotheses": {
            "object_local_rotated_to_world_error_mps": summarize(local_hypothesis_errors),
            "raw_xy_treated_as_world_error_mps": summarize(world_hypothesis_errors),
            "ego_local_rotated_to_world_error_mps": summarize(ego_local_hypothesis_errors),
            "ego_raw_xy_treated_as_world_error_mps": summarize(ego_world_hypothesis_errors),
        },
        "static_alignment": {
            "before_alignment_residual_m": summarize(alignment_before),
            "after_alignment_residual_m": summarize(alignment_after),
            "saved_lidar_box_audit": saved_box_alignment,
            "point_cloud_audit": point_alignment,
        },
        "checks": {
            "hard_errors": errors,
            "duplicate_sample_count": duplicate_samples,
            "split_overlap_count": len(split_overlap),
            "foreign_clip_sweep_count": 0,
            "future_input_count": 0,
            "source_dataset_modified": False,
            "required_artifact_reference_count": len(checked_required_artifacts),
            "missing_required_artifact_count": len(missing_required_artifacts),
            "missing_required_artifact_examples": [
                value for value in sorted(
                    "<missing-reference>" if item is None else str(item)
                    for item in missing_required_artifacts
                )[:20]
            ],
        },
        "status": "pass" if not errors else "fail",
    }
    write_json(output / "audit.json", audit)
    (output / "audit.md").write_text(
        render_audit_markdown(audit), encoding="utf-8"
    )
    write_audit_plots(
        output,
        lag_by_position,
        local_hypothesis_errors,
        world_hypothesis_errors,
        alignment_before,
        alignment_after,
    )
    write_json(
        output / "temporal_schema.json",
        {
            "schema_version": 1,
            "input_order": "oldest_to_current",
            "causal": True,
            "target": "last sweep",
            "point_alignment": "T_target_lidar_from_sweep_lidar",
            "matrix_convention": calibration["matrix_convention"],
            "time_lag": "sweep sequence_time_sec minus target sequence_time_sec",
            "velocity_target": "absolute object ground velocity expressed in morai_map and target Ego axes",
            "object_raw_velocity": "ObjectStatus object-local km/h, divided by 3.6 and rotated by object heading",
            "ego_raw_velocity": "EgoVehicleStatus ego-local m/s",
            "derived_velocity": "local linear fit of world position against per-frame physical sequence_time_sec",
            "velocity_source_priority": [
                "static_zero_when_raw_and_position_fit_confirm_static",
                "fresh_plausible_morai_raw_local_rotated",
                "derived_diagnostic_only",
                "invalid_masked",
            ],
            "production_valid_sources": ["morai_raw_local_rotated", "static_zero"],
            "diagnostic_only_sources": ["derived_diagnostic"],
            "velocity_quality_contract": {
                "velocity_valid": "whether velocity regression loss may consume this label",
                "velocity_loss_weight": "0 for masked, 0.25 for raw/derived disagreement, otherwise age/consistency weighted up to 1",
                "geometry_remains_valid": True,
            },
            "temporal_quality_contract": {
                "temporal_input_valid": "all adjacent sweep intervals are within configured bounds",
                "actual_time_lag_always_preserved": True,
            },
            "scan_internal_deskew": False,
            "exact_sensor_timestamp_available": False,
        },
    )
    print("clips:", len(clip_paths))
    print("temporal samples:", len(temporal_rows))
    print("velocity labels:", label_count)
    print("velocity sources:", dict(source_counts))
    print("errors:", len(errors))
    print("status:", audit["status"])
    print("output:", output)
    if errors:
        for error in errors:
            print("ERROR:", error)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
