#!/usr/bin/env python3
"""Audit a packaged MORAI temporal dataset without modifying its contents.

The audit covers three release gates:

* D0: file/clip/split integrity and conservative leakage checks.
* D1: object, distance, direction, motion and obstacle-shape distributions.
* D2: Ego coverage against the competition route and Lanelet2 v60.

All generated files are written below ``--output-dir``.  The source dataset is
opened read-only and is never rewritten.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_DATASET = Path(
    "/home/luv/morai_cap/morai_temporal_sequence_v2_base2_10504_final"
)
DEFAULT_OUTPUT = REPO_ROOT / "artifacts/datasets/morai_temporal_base2_10k_audit"
CAMERA_DIRS = ("CAMERA_1", "CAMERA_2", "CAMERA_3", "CAMERA_4")
SPLITS = ("train", "val", "test")
DISTANCE_BINS = ((0.0, 10.0), (10.0, 20.0), (20.0, 30.0), (30.0, 50.0), (50.0, math.inf))
MAP_TRANSLATION = (131.559130220, 545.911338186)


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None, "min": None, "max": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "p50": percentile(values, 0.50),
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "min": min(values),
        "max": max(values),
    }


def coarse_class(raw_class: str) -> str:
    lowered = raw_class.lower()
    if "pedestrian" in lowered or "person" in lowered:
        return "pedestrian"
    if lowered in {"vehicle", "sedan", "suv", "truck", "bus", "mpv", "wagon"} or "car" in lowered:
        return "vehicle"
    return "obstacle"


def obstacle_shape(size: Sequence[float], ratio: float = 2.0) -> str:
    length, width, height = (float(value) for value in size)
    horizontal_max = max(length, width)
    horizontal_min = max(min(length, width), 1.0e-8)
    if height > ratio * horizontal_max:
        return "tall"
    if horizontal_max > ratio * horizontal_min or horizontal_max > ratio * max(height, 1.0e-8):
        return "long"
    return "compact"


def parse_lidar_instances(path: Path) -> Dict[int, Dict[str, Any]]:
    records: Dict[int, Dict[str, Any]] = {}
    if not path.is_file():
        return records
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 15:
                continue
            try:
                values = [float(value) for value in fields[1:14]]
                unique_id = int(fields[14])
            except ValueError:
                continue
            records[unique_id] = {
                "raw_class": fields[0],
                "center": values[0:3],
                "rotation_rpy": values[3:6],
                "size": values[6:9],
                "distance": values[9],
            }
    return records


def parse_camera_ids(path: Path) -> set[int]:
    ids: set[int] = set()
    if not path.is_file():
        return ids
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) < 46:
                continue
            try:
                ids.add(int(fields[-5]))
            except ValueError:
                continue
    return ids


def direction_bucket(x: float, y: float) -> str:
    angle = abs(math.degrees(math.atan2(y, x)))
    if angle <= 60.0:
        return "front"
    if angle < 120.0:
        return "left_side" if y >= 0.0 else "right_side"
    return "rear"


def distance_bucket(distance: float) -> str:
    for lower, upper in DISTANCE_BINS:
        if lower <= distance < upper:
            return f"{int(lower)}-{int(upper)}m" if math.isfinite(upper) else f"{int(lower)}m+"
    return "unknown"


def actor_motion(actor: Optional[Mapping[str, Any]]) -> Tuple[Optional[float], Optional[float], str]:
    if not actor:
        return None, None, "metadata_missing"
    speed = actor.get("derived_speed_mps")
    if speed is None:
        raw = actor.get("morai_speed_raw")
        speed = float(raw) / 3.6 if raw is not None else None
    accel = actor.get("derived_longitudinal_acceleration_mps2")
    if speed is None:
        status = "unknown"
    elif float(speed) < 0.2:
        status = "stationary"
    elif float(speed) < 2.0:
        status = "slow"
    else:
        status = "moving"
    return (float(speed) if speed is not None else None,
            float(accel) if accel is not None else None,
            status)


def project_polyline(points: Sequence[Tuple[float, float]], x: float, y: float) -> Optional[Dict[str, float]]:
    best = None
    walked = 0.0
    for first, second in zip(points, points[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length_sq = dx * dx + dy * dy
        length = math.sqrt(length_sq)
        if length < 1.0e-9:
            continue
        ratio = max(0.0, min(1.0, ((x - first[0]) * dx + (y - first[1]) * dy) / length_sq))
        px, py = first[0] + ratio * dx, first[1] + ratio * dy
        distance = math.hypot(x - px, y - py)
        candidate = {"x": px, "y": py, "distance": distance,
                     "arc_length": walked + ratio * length,
                     "yaw": math.atan2(dy, dx)}
        if best is None or distance < best["distance"]:
            best = candidate
        walked += length
    return best


def load_route(path: Path) -> List[Tuple[float, float]]:
    points = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if len(fields) >= 2:
                points.append((float(fields[0]), float(fields[1])))
    return points


def load_lanelets(map_path: Path, projector_path: Path) -> Dict[int, List[Tuple[float, float]]]:
    source_dir = REPO_ROOT / "base_ws/src/perception_object_pipeline/src"
    sys.path.insert(0, str(source_dir))
    from perception_object_pipeline.map_based_prediction import LaneletNetwork  # pylint: disable=import-error

    network = LaneletNetwork.load(map_path, projector_path)
    lanelets = {}
    tx, ty = MAP_TRANSLATION
    for lanelet_id, record in network.records.items():
        lanelets[lanelet_id] = [(point[0] - tx, point[1] - ty) for point in record.centerline]
    return lanelets


def nearest_lanelet(lanelets: Mapping[int, Sequence[Tuple[float, float]]], x: float, y: float) -> Dict[str, Any]:
    best: Optional[Dict[str, Any]] = None
    for lanelet_id, points in lanelets.items():
        projection = project_polyline(points, x, y)
        if projection is None:
            continue
        if best is None or projection["distance"] < best["distance"]:
            best = dict(projection, lanelet_id=lanelet_id)
    return best or {"lanelet_id": None, "distance": math.inf, "arc_length": None, "yaw": None}


def source_inventory(dataset: Path, output: Path) -> Dict[str, Any]:
    rows = []
    suffix_counts: Counter[str] = Counter()
    total_bytes = 0
    file_count = 0
    for root, directories, filenames in os.walk(dataset):
        directories.sort()
        filenames.sort()
        for filename in filenames:
            path = Path(root) / filename
            stat = path.stat()
            relative = path.relative_to(dataset).as_posix()
            suffix = "".join(path.suffixes) or "<none>"
            suffix_counts[suffix] += 1
            total_bytes += stat.st_size
            file_count += 1
            rows.append({"relative_path": relative, "size_bytes": stat.st_size, "mtime_ns": stat.st_mtime_ns})
    write_csv(output / "file_inventory.csv", ("relative_path", "size_bytes", "mtime_ns"), rows)
    protected = []
    for pattern in ("metadata/*.json", "metadata/*.jsonl", "metadata/clips/*.json", "metadata/clips/*.jsonl", "splits/*.txt", "README.md", "VALIDATED_COMPLETE"):
        for path in sorted(dataset.glob(pattern)):
            if path.is_file():
                protected.append({"path": path.relative_to(dataset).as_posix(), "size_bytes": path.stat().st_size, "sha256": sha256(path)})
    write_csv(output / "metadata_checksums.csv", ("path", "size_bytes", "sha256"), protected)
    return {"file_count": file_count, "total_bytes": total_bytes,
            "suffix_counts": dict(sorted(suffix_counts.items())),
            "metadata_checksum_count": len(protected)}


def audit(args: argparse.Namespace) -> Dict[str, Any]:
    dataset = args.dataset.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    descriptor = read_json(dataset / "metadata/dataset.json")
    clip_paths = sorted((dataset / "metadata/clips").glob("*.clip.json"))

    inventory = source_inventory(dataset, output)
    split_lists: Dict[str, List[str]] = {}
    for split in SPLITS:
        split_lists[split] = [line.strip() for line in (dataset / f"splits/{split}_scenes.txt").read_text().splitlines() if line.strip()]

    errors: List[str] = []
    warnings: List[str] = []
    scene_rows: List[Dict[str, Any]] = []
    object_rows: List[Dict[str, Any]] = []
    ego_points: List[Dict[str, Any]] = []
    split_membership: Dict[str, str] = {}
    scenario_splits: Dict[str, set[str]] = defaultdict(set)
    frame_ids: set[str] = set()
    capture_ids: set[str] = set()
    near_signatures: Dict[str, set[str]] = defaultdict(set)
    family_counts: Counter[str] = Counter()
    family_split_counts: Counter[Tuple[str, str]] = Counter()
    class_counts: Counter[str] = Counter()
    visible_class_counts: Counter[str] = Counter()
    asset_counts: Counter[Tuple[str, str]] = Counter()
    shape_counts: Counter[str] = Counter()
    distance_counts: Counter[Tuple[str, str]] = Counter()
    direction_counts: Counter[Tuple[str, str]] = Counter()
    quadrant_counts: Counter[Tuple[str, str]] = Counter()
    motion_counts: Counter[Tuple[str, str]] = Counter()
    visibility_counts: Counter[Tuple[str, int]] = Counter()
    period_values: List[float] = []
    ego_age_values: List[float] = []
    object_age_values: List[float] = []
    object_age_by_family: Dict[str, List[float]] = defaultdict(list)
    timing_stream_counts: Counter[str] = Counter()
    timing_coverage_failures: List[str] = []
    missing_sensor_files: Counter[str] = Counter()

    for split, scenes in split_lists.items():
        for scene_id in scenes:
            if scene_id in split_membership:
                errors.append(f"scene appears in multiple splits: {scene_id}")
            split_membership[scene_id] = split

    for clip_path in clip_paths:
        clip = read_json(clip_path)
        scene_id = str(clip["scene_id"])
        split = str(clip["split"])
        family = str(clip.get("mission_family", "unknown"))
        scenario_stem = str(clip.get("scenario_stem", ""))
        frames_path = clip_path.with_name(scene_id + ".frames.jsonl")
        timing_path = clip_path.with_name(scene_id + ".timing.jsonl")
        if split_membership.get(scene_id) != split:
            errors.append(f"split mismatch for {scene_id}: clip={split}, list={split_membership.get(scene_id)}")
        scenario_splits[scenario_stem].add(split)
        if not frames_path.is_file() or not timing_path.is_file():
            errors.append(f"missing clip sidecar for {scene_id}")
            continue
        frames = read_jsonl(frames_path)
        timing_rows = read_jsonl(timing_path)
        expected = int(clip.get("sequence_contract", {}).get("clip_length", 8))
        if len(frames) != expected:
            errors.append(f"clip length mismatch {scene_id}: frames={len(frames)}, expected={expected}")
        # timing.jsonl is intentionally a high-rate raw state window, not one
        # row per capture.  Require both streams and coverage around all eight
        # SaveSensorData trigger times instead of comparing row counts.
        by_stream: Dict[str, List[Mapping[str, Any]]] = defaultdict(list)
        for timing_row in timing_rows:
            by_stream[str(timing_row.get("stream", "unknown"))].append(timing_row)
        for stream_name in ("ego", "objects"):
            samples = by_stream.get(stream_name, [])
            timing_stream_counts[stream_name] += len(samples)
            if len(samples) < 2:
                errors.append(f"timing stream missing/short {scene_id}: {stream_name}={len(samples)}")
                continue
            sample_times = [float(item["callback_unix"]) for item in samples if item.get("callback_unix") is not None]
            trigger_times = [float(frame["triggered_unix"]) for frame in frames if frame.get("triggered_unix") is not None]
            if trigger_times and sample_times and (min(sample_times) > min(trigger_times) + 0.2 or max(sample_times) < max(trigger_times) - 0.2):
                timing_coverage_failures.append(f"{scene_id}/{stream_name}")
        family_counts[family] += 1
        family_split_counts[(family, split)] += 1

        sequence_times = [float(frame["sequence_time_sec"]) for frame in frames]
        period_values.extend(b - a for a, b in zip(sequence_times, sequence_times[1:]))
        first_ego = frames[0].get("ego") or {}
        position = first_ego.get("position_m") or {}
        x, y = float(position.get("x", math.nan)), float(position.get("y", math.nan))
        heading = float(first_ego.get("heading_deg", math.nan))
        signature = f"{family}|{round(x * 2) / 2:.1f}|{round(y * 2) / 2:.1f}|{round(heading / 5) * 5:.0f}|{len(frames[0].get('actors') or [])}"
        near_signatures[signature].add(split)
        scene_rows.append({"scene_id": scene_id, "scenario_stem": scenario_stem,
                           "split": split, "family": family,
                           "motion_phase": clip.get("motion_phase", ""),
                           "sampling_role": clip.get("sampling_role", ""),
                           "frame_count": len(frames), "ego_x_m": x, "ego_y_m": y,
                           "ego_heading_deg": heading, "near_signature": signature})

        for frame_index, frame in enumerate(frames):
            frame_id = str(frame["frame_id"])
            capture_id = str(frame["capture_id"])
            if frame_id in frame_ids:
                errors.append(f"duplicate frame_id: {frame_id}")
            if capture_id in capture_ids:
                errors.append(f"duplicate capture_id: {capture_id}")
            frame_ids.add(frame_id)
            capture_ids.add(capture_id)
            quality = frame.get("quality") or {}
            if quality.get("ego_state_age_sec") is not None:
                ego_age_values.append(float(quality["ego_state_age_sec"]))
            if quality.get("object_state_age_sec") is not None:
                object_age = float(quality["object_state_age_sec"])
                object_age_values.append(object_age)
                object_age_by_family[family].append(object_age)
            ego = frame.get("ego") or {}
            pos = ego.get("position_m") or {}
            ego_points.append({"scene_id": scene_id, "frame_id": frame_id, "frame_index": frame_index,
                               "split": split, "family": family,
                               "x_m": float(pos.get("x", math.nan)), "y_m": float(pos.get("y", math.nan)),
                               "heading_deg": float(ego.get("heading_deg", math.nan))})
            object_state = frame.get("objects") or {}
            actors = {int(actor["unique_id"]): actor for actor in (object_state.get("actors") or []) if actor.get("unique_id") is not None}

            lidar_path = dataset / "LIDAR_5" / f"{capture_id}_Instance.txt"
            if not lidar_path.is_file():
                missing_sensor_files["LIDAR_5_Instance"] += 1
            lidar_boxes = parse_lidar_instances(lidar_path)
            camera_ids: Counter[int] = Counter()
            for camera in CAMERA_DIRS:
                path = dataset / camera / f"{capture_id}_BBox3D.txt"
                if path.is_file():
                    camera_ids.update(parse_camera_ids(path))
            for unique_id, box in lidar_boxes.items():
                cls = coarse_class(box["raw_class"])
                actor = actors.get(unique_id)
                speed, accel, motion = actor_motion(actor)
                asset = str((actor or {}).get("name") or box["raw_class"])
                distance = float(box["distance"])
                center = box["center"]
                direction = direction_bucket(float(center[0]), float(center[1]))
                quadrant = ("front" if float(center[0]) >= 0.0 else "rear") + "_" + ("left" if float(center[1]) >= 0.0 else "right")
                visible_cameras = int(camera_ids[unique_id])
                shape = obstacle_shape(box["size"]) if cls == "obstacle" else ""
                class_counts[cls] += 1
                if visible_cameras:
                    visible_class_counts[cls] += 1
                asset_counts[(cls, asset)] += 1
                if shape:
                    shape_counts[shape] += 1
                distance_counts[(cls, distance_bucket(distance))] += 1
                direction_counts[(cls, direction)] += 1
                quadrant_counts[(cls, quadrant)] += 1
                motion_counts[(cls, motion)] += 1
                visibility_counts[(cls, visible_cameras)] += 1
                object_rows.append({
                    "scene_id": scene_id, "frame_id": frame_id, "capture_id": capture_id,
                    "frame_index": frame_index, "split": split, "family": family,
                    "unique_id": unique_id, "class": cls, "raw_class": box["raw_class"],
                    "asset": asset, "shape": shape, "distance_m": distance,
                    "distance_bin": distance_bucket(distance), "direction": direction,
                    "quadrant": quadrant,
                    "center_x_m": center[0], "center_y_m": center[1], "center_z_m": center[2],
                    "length_m": box["size"][0], "width_m": box["size"][1], "height_m": box["size"][2],
                    "visible_camera_count": visible_cameras,
                    "derived_speed_mps": speed, "derived_longitudinal_acceleration_mps2": accel,
                    "motion_status": motion,
                })

    listed = set(split_membership)
    actual = {str(read_json(path)["scene_id"]) for path in clip_paths}
    for missing in sorted(listed - actual):
        errors.append(f"split-listed scene has no clip: {missing}")
    for missing in sorted(actual - listed):
        errors.append(f"clip scene not listed in splits: {missing}")
    scenario_leaks = {key: sorted(value) for key, value in scenario_splits.items() if key and len(value) > 1}
    signature_overlaps = {key: sorted(value) for key, value in near_signatures.items() if len(value) > 1}
    if signature_overlaps:
        warnings.append(f"{len(signature_overlaps)} coarse spatial signatures occur across splits; review as near-duplicates, not proof of leakage")
    if percentile(object_age_values, 0.95) and float(percentile(object_age_values, 0.95)) > 0.1:
        warnings.append(
            "frame-level Object_topic snapshot age has a long tail above 0.1 s; "
            "D3 labels must align from timing.jsonl instead of trusting only frames.jsonl snapshots"
        )

    route = load_route(args.route)
    lanelets = load_lanelets(args.map, args.projector)
    route_rows = []
    lanelet_counts: Counter[Tuple[str, int]] = Counter()
    route_bin_counts: Counter[Tuple[str, int]] = Counter()
    route_distances: List[float] = []
    lane_distances: List[float] = []
    for row in scene_rows:
        route_projection = project_polyline(route, float(row["ego_x_m"]), float(row["ego_y_m"]))
        lane_projection = nearest_lanelet(lanelets, float(row["ego_x_m"]), float(row["ego_y_m"]))
        route_bin = int(route_projection["arc_length"] // args.route_bin_m) if route_projection else -1
        route_distance = float(route_projection["distance"]) if route_projection else math.inf
        lane_distance = float(lane_projection["distance"])
        route_distances.append(route_distance)
        lane_distances.append(lane_distance)
        route_bin_counts[(row["split"], route_bin)] += 1
        if lane_projection["lanelet_id"] is not None:
            lanelet_counts[(row["split"], int(lane_projection["lanelet_id"]))] += 1
        route_rows.append(dict(row,
            route_arc_m=route_projection["arc_length"] if route_projection else None,
            route_distance_m=route_distance, route_bin=route_bin,
            lanelet_id=lane_projection["lanelet_id"], lanelet_distance_m=lane_distance,
            lanelet_arc_m=lane_projection["arc_length"], lanelet_yaw_rad=lane_projection["yaw"]))

    write_csv(output / "scene_coverage.csv", tuple(route_rows[0].keys()), route_rows)
    write_csv(output / "ego_trajectory_points.csv",
              ("scene_id", "frame_id", "frame_index", "split", "family", "x_m", "y_m", "heading_deg"),
              ego_points)
    write_csv(output / "object_observations.csv", tuple(object_rows[0].keys()) if object_rows else ("scene_id",), object_rows)
    write_csv(output / "family_distribution.csv", ("family", "total", "train", "val", "test"), [
        {"family": family, "total": family_counts[family], **{split: family_split_counts[(family, split)] for split in SPLITS}}
        for family in sorted(family_counts)
    ])
    write_csv(output / "asset_distribution.csv", ("class", "asset", "observations"), [
        {"class": key[0], "asset": key[1], "observations": value}
        for key, value in sorted(asset_counts.items())
    ])
    write_csv(output / "route_bin_distribution.csv", ("split", "route_bin", "start_m", "end_m", "clips"), [
        {"split": split, "route_bin": route_bin, "start_m": route_bin * args.route_bin_m,
         "end_m": (route_bin + 1) * args.route_bin_m, "clips": count}
        for (split, route_bin), count in sorted(route_bin_counts.items())
    ])
    write_csv(output / "lanelet_distribution.csv", ("split", "lanelet_id", "clips"), [
        {"split": split, "lanelet_id": lanelet_id, "clips": count}
        for (split, lanelet_id), count in sorted(lanelet_counts.items())
    ])
    write_csv(output / "object_state_age_by_family.csv",
              ("family", "frames", "mean_sec", "p50_sec", "p95_sec", "max_sec", "over_100ms"), [
        {"family": family, "frames": len(values), "mean_sec": sum(values) / len(values),
         "p50_sec": percentile(values, 0.5), "p95_sec": percentile(values, 0.95),
         "max_sec": max(values), "over_100ms": sum(value > 0.1 for value in values)}
        for family, values in sorted(object_age_by_family.items())
    ])

    distribution = {
        "family_scene_counts": dict(sorted(family_counts.items())),
        "class_observation_counts_lidar": dict(sorted(class_counts.items())),
        "class_observation_counts_camera_visible": dict(sorted(visible_class_counts.items())),
        "obstacle_shape_counts": dict(sorted(shape_counts.items())),
        "distance_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(distance_counts.items())},
        "direction_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(direction_counts.items())},
        "quadrant_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(quadrant_counts.items())},
        "motion_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(motion_counts.items())},
        "camera_visibility_counts": {f"{key[0]}/{key[1]}cam": value for key, value in sorted(visibility_counts.items())},
    }
    integrity = {
        "errors": errors,
        "warnings": warnings,
        "scene_count": len(scene_rows), "frame_count": len(frame_ids), "capture_id_count": len(capture_ids),
        "split_counts": {split: len(scenes) for split, scenes in split_lists.items()},
        "scenario_stem_cross_split": scenario_leaks,
        "coarse_signature_cross_split_count": len(signature_overlaps),
        "coarse_signature_cross_split_examples": dict(list(sorted(signature_overlaps.items()))[:50]),
        "missing_sensor_files": dict(missing_sensor_files),
        "sequence_period_sec": summarize(period_values),
        "ego_state_age_sec": summarize(ego_age_values),
        "object_state_age_sec": summarize(object_age_values),
        "timing_stream_sample_counts": dict(timing_stream_counts),
        "timing_window_coverage_failures": timing_coverage_failures,
    }
    coverage = {
        "route_distance_m": summarize(route_distances),
        "lanelet_distance_m": summarize(lane_distances),
        "clips_within_route_5m": sum(value <= 5.0 for value in route_distances),
        "clips_within_lanelet_3m": sum(value <= 3.0 for value in lane_distances),
        "unique_lanelets": len({row["lanelet_id"] for row in route_rows if row["lanelet_id"] is not None}),
        "occupied_route_bins": len({row["route_bin"] for row in route_rows if row["route_bin"] >= 0}),
        "route_bin_size_m": args.route_bin_m,
    }
    summary = {
        "schema": "morai_temporal_dataset_audit_v1",
        "dataset": str(dataset), "dataset_id": descriptor.get("dataset_id"),
        "source_descriptor": descriptor, "inventory": inventory,
        "D0_integrity": integrity, "D1_distribution": distribution,
        "D2_coverage": coverage,
        "status": "pass" if not errors else "fail",
    }
    write_json(output / "audit_summary.json", summary)
    make_plots(output, route, lanelets, route_rows, object_rows)
    write_report(output / "audit_report.md", summary)
    return summary


def make_plots(output: Path, route: Sequence[Tuple[float, float]],
               lanelets: Mapping[int, Sequence[Tuple[float, float]]],
               scenes: Sequence[Mapping[str, Any]], objects: Sequence[Mapping[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    family_names = sorted({str(row["family"]) for row in scenes})
    color_map = {name: plt.cm.tab20(index % 20) for index, name in enumerate(family_names)}
    figure, axis = plt.subplots(figsize=(14, 11), constrained_layout=True)
    for points in lanelets.values():
        axis.plot([p[0] for p in points], [p[1] for p in points], color="#c8c8c8", linewidth=0.35, alpha=0.55)
    axis.plot([p[0] for p in route], [p[1] for p in route], color="#202020", linewidth=1.2, alpha=0.8, label="competition route")
    for family in family_names:
        rows = [row for row in scenes if row["family"] == family]
        axis.scatter([row["ego_x_m"] for row in rows], [row["ego_y_m"] for row in rows],
                     s=9, alpha=0.7, color=color_map[family], label=f"{family} ({len(rows)})")
    axis.set_title("Base2-10K clip start coverage by scenario family")
    axis.set_xlabel("MORAI x [m]"); axis.set_ylabel("MORAI y [m]")
    axis.set_aspect("equal", adjustable="box"); axis.grid(alpha=0.15)
    axis.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=7, markerscale=1.6)
    figure.savefig(output / "ego_coverage_by_family.png", dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(1, 3, figsize=(18, 7), constrained_layout=True, sharex=True, sharey=True)
    for axis, split in zip(axes, SPLITS):
        for points in lanelets.values():
            axis.plot([p[0] for p in points], [p[1] for p in points], color="#d0d0d0", linewidth=0.3)
        axis.plot([p[0] for p in route], [p[1] for p in route], color="#303030", linewidth=0.9)
        rows = [row for row in scenes if row["split"] == split]
        axis.scatter([row["ego_x_m"] for row in rows], [row["ego_y_m"] for row in rows], s=8, alpha=0.65)
        axis.set_title(f"{split}: {len(rows)} clips"); axis.set_aspect("equal", adjustable="box"); axis.grid(alpha=0.12)
    figure.savefig(output / "ego_coverage_by_split.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(12, 4.5), constrained_layout=True)
    arcs = [float(row["route_arc_m"]) for row in scenes if row.get("route_arc_m") is not None and float(row["route_distance_m"]) <= 10.0]
    axis.hist(arcs, bins=max(10, int((max(arcs) - min(arcs)) / 20.0)) if arcs else 10, color="#3f7fba")
    axis.set_title("Clip starts along nearest competition-route arc (<=10 m)")
    axis.set_xlabel("nearest route arc length [m]"); axis.set_ylabel("clips"); axis.grid(axis="y", alpha=0.2)
    figure.savefig(output / "route_arc_histogram.png", dpi=180)
    plt.close(figure)

    directions = ("front", "left_side", "right_side", "rear")
    classes = ("vehicle", "pedestrian", "obstacle")
    values = [[sum(1 for row in objects if row["class"] == cls and row["direction"] == direction) for direction in directions] for cls in classes]
    figure, axis = plt.subplots(figsize=(9, 5), constrained_layout=True)
    bottom = [0] * len(classes)
    for direction_index, direction in enumerate(directions):
        column = [values[class_index][direction_index] for class_index in range(len(classes))]
        axis.bar(classes, column, bottom=bottom, label=direction)
        bottom = [a + b for a, b in zip(bottom, column)]
    axis.set_title("LiDAR GT observations by Ego-relative sector"); axis.set_ylabel("frame-object observations")
    axis.legend(); axis.grid(axis="y", alpha=0.2)
    figure.savefig(output / "object_direction_distribution.png", dpi=180)
    plt.close(figure)


def write_report(path: Path, summary: Mapping[str, Any]) -> None:
    d0 = summary["D0_integrity"]
    d1 = summary["D1_distribution"]
    d2 = summary["D2_coverage"]
    lines = [
        "# Base2-10K D0-D2 audit", "",
        f"- Dataset: `{summary['dataset_id']}`.",
        f"- Result: **{summary['status'].upper()}**.",
        f"- Clips/frames: {d0['scene_count']:,} / {d0['frame_count']:,}.",
        "- Source dataset was read-only; every output in this directory is derived.", "",
        "## D0. Integrity and leakage", "",
        f"- Hard errors: {len(d0['errors'])}.",
        f"- Warnings: {len(d0['warnings'])}.",
        f"- Exact scenario stems shared across splits: {len(d0['scenario_stem_cross_split'])}.",
        f"- Coarse 0.5 m/5 deg spatial signatures shared across splits: {d0['coarse_signature_cross_split_count']}.",
        f"- Sequence period p50/p95: {1000*d0['sequence_period_sec']['p50']:.2f} / {1000*d0['sequence_period_sec']['p95']:.2f} ms.", "",
        "## D1. Distribution", "",
        "| Class | LiDAR GT observations | Visible in >=1 camera | Visibility rate |", "| --- | ---: | ---: | ---: |",
    ]
    for cls in ("vehicle", "pedestrian", "obstacle"):
        total = int(d1["class_observation_counts_lidar"].get(cls, 0))
        visible = int(d1["class_observation_counts_camera_visible"].get(cls, 0))
        lines.append(f"| {cls} | {total:,} | {visible:,} | {100.0*visible/max(total,1):.1f}% |")
    lines += ["", "Obstacle shapes: " + ", ".join(f"{key} {value:,}" for key, value in d1["obstacle_shape_counts"].items()) + ".", "",
              "## D2. Route and Lanelet coverage", "",
              f"- Nearest Lanelet distance p50/p95: {d2['lanelet_distance_m']['p50']:.3f} / {d2['lanelet_distance_m']['p95']:.3f} m.",
              f"- Clips within 3 m of a road Lanelet: {d2['clips_within_lanelet_3m']:,}/{d0['scene_count']:,}.",
              f"- Clips within 5 m of the competition global path: {d2['clips_within_route_5m']:,}/{d0['scene_count']:,}.",
              f"- Unique nearest Lanelets: {d2['unique_lanelets']:,}.",
              f"- Occupied nearest-route bins ({d2['route_bin_size_m']:.0f} m): {d2['occupied_route_bins']:,}.", "",
              "## Files", "",
              "- `audit_summary.json`: machine-readable complete summary.",
              "- `file_inventory.csv`, `metadata_checksums.csv`: D0 release inventory.",
              "- `object_observations.csv`: one row per LiDAR GT box observation.",
              "- `scene_coverage.csv`: one row per clip with route/Lanelet projection.",
              "- `*_distribution.csv`, `*.png`: D1-D2 tables and plots.", ""]
    if d0["errors"]:
        lines += ["## Errors", ""] + [f"- {value}" for value in d0["errors"]] + [""]
    if d0["warnings"]:
        lines += ["## Warnings", ""] + [f"- {value}" for value in d0["warnings"]] + [""]
    path.write_text("\n".join(lines), encoding="utf-8")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    result.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    result.add_argument("--map", type=Path, default=REPO_ROOT / "kcity_map/lanelet_map/lanelet2_map_v63.osm")
    result.add_argument("--projector", type=Path, default=REPO_ROOT / "kcity_map/lanelet_map/map_projector_info_v1.yaml")
    result.add_argument("--route", type=Path, default=REPO_ROOT / "configs/global_path/2026_molit_comp_global_path.txt")
    result.add_argument("--route-bin-m", type=float, default=20.0)
    return result


def main() -> None:
    args = parser().parse_args()
    summary = audit(args)
    print(f"status: {summary['status']}")
    print(f"report: {args.output_dir.resolve() / 'audit_report.md'}")
    print(f"summary: {args.output_dir.resolve() / 'audit_summary.json'}")
    if summary["D0_integrity"]["errors"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
