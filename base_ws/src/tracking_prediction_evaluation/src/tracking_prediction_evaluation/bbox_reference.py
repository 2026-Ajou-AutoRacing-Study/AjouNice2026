"""Audit MORAI actor reference positions against geometric 3D box centers."""

from __future__ import annotations

import bisect
import hashlib
import json
import math
from dataclasses import replace
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from .pipeline_evaluator import dominant_label, message_timestamp, quaternion_yaw, statistics
from .trajectory import TrajectorySegment


VEHICLE_LABELS = frozenset((1, 2, 3, 4, 5, 6))
PASSENGER_BBOX_REFERENCE_V1 = {
    "policy": "bbox-center-calibrated-v1",
    "longitudinal_offset_m": 1.34,
    "maximum_length_m": 5.5,
    "source_vehicle_boxes": 2813,
    "source_scenes": 190,
    "source_longitudinal_p95_m": 1.48,
    "source_lateral_p50_m": -0.009,
}
ASSET_CATALOG_BBOX_REFERENCE_V1 = {
    "policy": "asset-catalog-bbox-center-v1",
    "vehicle_size_match_max_error_m": 0.12,
    "vehicle_size_match_min_margin_m": 0.02,
    "description": (
        "Use the audited per-asset geometric box and pivot offset. Older "
        "RandomMode vehicle bags whose ObjectStatus.name is an MGeo link are "
        "resolved from their stable ObjectStatus.size profile."
    ),
}
EXPLICIT_VEHICLE_REFERENCE_PROFILES_V1 = (
    {
        "profile": "data_id_30400002_medium_vehicle",
        "length_m": 6.557,
        "width_m": 2.517,
        "height_m": 2.505,
        "longitudinal_offset_m": 1.383,
        "dimension_tolerance_m": 0.08,
        "source_vehicle_boxes": 126,
    },
)


@dataclass(frozen=True)
class EgoState:
    timestamp: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class StatusBox:
    unique_id: int
    name: str
    x: float
    y: float
    z: float
    yaw: float
    length: float
    width: float
    height: float


@dataclass(frozen=True)
class DetectionBox:
    x: float
    y: float
    z: float
    yaw: float
    length: float
    width: float
    height: float


def _segment_dimensions(segment):
    dimensions = [
        (point.length, point.width, point.height)
        for point in segment.points
        if point.length > 0.0 and point.width > 0.0 and point.height > 0.0
    ]
    if not dimensions:
        return 0.0, 0.0, 0.0
    return tuple(float(value) for value in np.median(dimensions, axis=0))


def load_asset_bbox_catalog(path):
    path = Path(path).expanduser().resolve()
    payload = path.read_bytes()
    document = json.loads(payload.decode("utf-8"))
    assets = document.get("assets")
    if not isinstance(assets, dict) or not assets:
        raise ValueError("asset bbox catalog must contain a non-empty assets object")
    normalized = {}
    for name, value in assets.items():
        class_name = str(value.get("class", "")).strip().lower()
        offset = tuple(float(item) for item in value["pivot_to_bbox_center_local_m"])
        size = tuple(float(item) for item in value["bbox_size_m"])
        if class_name not in ("vehicle", "pedestrian", "obstacle"):
            raise ValueError("invalid asset class for {}: {}".format(name, class_name))
        if len(offset) != 3 or len(size) != 3 or any(item <= 0.0 for item in size):
            raise ValueError("invalid bbox geometry for asset {}".format(name))
        normalized[str(name)] = {
            "class": class_name,
            "pivot_to_bbox_center_local_m": offset,
            "bbox_size_m": size,
        }
    return normalized, {
        "path": str(path),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "schema_version": document.get("schema_version"),
        "asset_count": len(normalized),
    }


def _resolve_segment_asset(
    segment,
    assets,
    *,
    vehicle_size_match_max_error_m,
    vehicle_size_match_min_margin_m,
):
    names = [segment.asset_name]
    names.extend(point.asset_name for point in segment.points)
    exact_names = sorted(
        {
            str(name)
            for name in names
            if str(name) in assets
            and assets[str(name)]["class"] == segment.class_name
        }
    )
    if len(exact_names) == 1:
        name = exact_names[0]
        return name, assets[name], "exact-name", 0.0, None
    if len(exact_names) > 1:
        return None, None, None, None, "multiple-asset-names"

    # Old MORAI RandomMode bags put the current MGeo link in ``name``. Vehicle
    # ObjectStatus dimensions are nevertheless stable and agree closely with
    # the audited asset catalog, so resolve only this measured legacy case.
    # Pedestrian/obstacle size inference is intentionally forbidden: several
    # assets have similar pivots/sizes and guessing would corrupt the GT.
    if segment.class_name != "vehicle":
        return None, None, None, None, "asset-name-not-in-catalog"
    dimensions = _segment_dimensions(segment)
    if min(dimensions) <= 0.0:
        return None, None, None, None, "missing-object-status-size"
    candidates = []
    for name, asset in assets.items():
        if asset["class"] != segment.class_name:
            continue
        error = math.sqrt(
            sum(
                (actual - expected) ** 2
                for actual, expected in zip(dimensions, asset["bbox_size_m"])
            )
        )
        candidates.append((error, name, asset))
    candidates.sort(key=lambda item: (item[0], item[1]))
    if not candidates or candidates[0][0] > vehicle_size_match_max_error_m:
        return None, None, None, None, "vehicle-size-no-match"
    second_error = candidates[1][0] if len(candidates) > 1 else math.inf
    margin = second_error - candidates[0][0]
    if margin < vehicle_size_match_min_margin_m:
        return None, None, None, None, "vehicle-size-ambiguous"
    error, name, asset = candidates[0]
    return name, asset, "vehicle-size", float(error), None


def transform_segments_to_asset_catalog_bbox_center(
    segments,
    catalog_path,
    *,
    vehicle_size_match_max_error_m=(
        ASSET_CATALOG_BBOX_REFERENCE_V1["vehicle_size_match_max_error_m"]
    ),
    vehicle_size_match_min_margin_m=(
        ASSET_CATALOG_BBOX_REFERENCE_V1["vehicle_size_match_min_margin_m"]
    ),
    exclude_unresolved=True,
):
    """Transform MORAI actor pivots using audited per-asset box geometry.

    New generated bags carry an exact asset name. Older highway RandomMode
    bags carry a link ID instead, so their vehicle asset is inferred from the
    stable 3-D size profile under strict error and ambiguity gates. Unknown
    assets are excluded from the primary metric rather than receiving the old
    one-size-fits-all longitudinal offset.

    Updating every point position also gives downstream finite-difference
    velocity metrics the correct rotating BBox-center trajectory without
    trusting the historically ambiguous ObjectStatus velocity units.
    """
    assets, catalog_report = load_asset_bbox_catalog(catalog_path)
    transformed = []
    supported_segment_count = 0
    resolution_counts = defaultdict(int)
    asset_counts = defaultdict(int)
    unresolved_reasons = defaultdict(int)
    unresolved_keys = defaultdict(list)
    size_match_errors = []
    for segment in segments:
        name, asset, method, error, unresolved_reason = _resolve_segment_asset(
            segment,
            assets,
            vehicle_size_match_max_error_m=float(vehicle_size_match_max_error_m),
            vehicle_size_match_min_margin_m=float(vehicle_size_match_min_margin_m),
        )
        if asset is None:
            unresolved_reasons[unresolved_reason] += 1
            unresolved_keys[unresolved_reason].append(segment.key)
            if not exclude_unresolved:
                transformed.append(segment)
            continue
        offset = asset["pivot_to_bbox_center_local_m"]
        size = asset["bbox_size_m"]
        corrected_points = []
        for point in segment.points:
            cosine = math.cos(point.yaw)
            sine = math.sin(point.yaw)
            corrected_points.append(
                replace(
                    point,
                    x=point.x + cosine * offset[0] - sine * offset[1],
                    y=point.y + sine * offset[0] + cosine * offset[1],
                    z=point.z + offset[2],
                    asset_name=name,
                    length=size[0],
                    width=size[1],
                    height=size[2],
                )
            )
        transformed.append(
            replace(segment, asset_name=name, points=tuple(corrected_points))
        )
        supported_segment_count += 1
        resolution_counts[method] += 1
        asset_counts[name] += 1
        if error is not None:
            size_match_errors.append(error)

    report = {
        **ASSET_CATALOG_BBOX_REFERENCE_V1,
        **catalog_report,
        "vehicle_size_match_max_error_m": float(vehicle_size_match_max_error_m),
        "vehicle_size_match_min_margin_m": float(vehicle_size_match_min_margin_m),
        "input_segment_count": len(segments),
        "supported_segment_count": supported_segment_count,
        "unresolved_segment_count": sum(unresolved_reasons.values()),
        "exclude_unresolved": bool(exclude_unresolved),
        "resolution_counts": dict(sorted(resolution_counts.items())),
        "resolved_asset_counts": dict(sorted(asset_counts.items())),
        "vehicle_size_match_error_m": statistics(size_match_errors),
        "unresolved_reasons": dict(sorted(unresolved_reasons.items())),
        "unresolved_segment_keys": {
            reason: keys for reason, keys in sorted(unresolved_keys.items())
        },
        "velocity_reference": (
            "finite differences of the corrected geometric BBox-center trajectory"
        ),
    }
    return tuple(transformed), report


def _reference_profile(length, width, height, maximum_passenger_length_m):
    if 0.0 < length <= maximum_passenger_length_m:
        return {
            "profile": "passenger_under_5_5m",
            "longitudinal_offset_m": PASSENGER_BBOX_REFERENCE_V1[
                "longitudinal_offset_m"
            ],
        }
    for profile in EXPLICIT_VEHICLE_REFERENCE_PROFILES_V1:
        tolerance = profile["dimension_tolerance_m"]
        if all(
            abs(actual - profile[key]) <= tolerance
            for actual, key in (
                (length, "length_m"),
                (width, "width_m"),
                (height, "height_m"),
            )
        ):
            return profile
    return None


def transform_segments_to_calibrated_bbox_center(
    segments,
    *,
    longitudinal_offset_m=PASSENGER_BBOX_REFERENCE_V1["longitudinal_offset_m"],
    maximum_length_m=PASSENGER_BBOX_REFERENCE_V1["maximum_length_m"],
    exclude_unsupported=True,
):
    """Transform supported MORAI vehicle actor pivots to bbox-center GT.

    The current calibration supports passenger-sized vehicles plus explicitly
    audited larger dimension profiles. Every other large vehicle has a
    materially different pivot-to-center offset and is excluded from the
    primary metric instead of being silently mis-corrected.
    """
    transformed = []
    supported = []
    supported_profiles = defaultdict(int)
    unsupported_missing_size = []
    unsupported_large_vehicle = []
    unchanged_non_vehicle = []
    for segment in segments:
        if segment.class_name != "vehicle":
            transformed.append(segment)
            unchanged_non_vehicle.append(segment.key)
            continue
        median_length, median_width, median_height = _segment_dimensions(segment)
        if median_length <= 0.0:
            unsupported_missing_size.append(segment.key)
            if not exclude_unsupported:
                transformed.append(segment)
            continue
        profile = _reference_profile(
            median_length, median_width, median_height, maximum_length_m
        )
        if profile is None:
            unsupported_large_vehicle.append(segment.key)
            if not exclude_unsupported:
                transformed.append(segment)
            continue
        offset = (
            float(longitudinal_offset_m)
            if profile["profile"] == "passenger_under_5_5m"
            else float(profile["longitudinal_offset_m"])
        )
        corrected_points = tuple(
            replace(
                point,
                x=point.x + offset * math.cos(point.yaw),
                y=point.y + offset * math.sin(point.yaw),
            )
            for point in segment.points
        )
        transformed.append(replace(segment, points=corrected_points))
        supported.append(segment.key)
        supported_profiles[profile["profile"]] += 1
    report = {
        **PASSENGER_BBOX_REFERENCE_V1,
        "longitudinal_offset_m": float(longitudinal_offset_m),
        "maximum_length_m": float(maximum_length_m),
        "input_segment_count": len(segments),
        "supported_segment_count": len(supported),
        "supported_profiles": dict(supported_profiles),
        "explicit_profiles": [
            dict(profile) for profile in EXPLICIT_VEHICLE_REFERENCE_PROFILES_V1
        ],
        "unsupported_segment_count": (
            len(unsupported_missing_size) + len(unsupported_large_vehicle)
        ),
        "unsupported_missing_size_count": len(unsupported_missing_size),
        "unsupported_large_vehicle_count": len(unsupported_large_vehicle),
        "unchanged_non_vehicle_count": len(unchanged_non_vehicle),
        "exclude_unsupported": bool(exclude_unsupported),
        "unsupported_segment_keys": {
            "missing_size": unsupported_missing_size,
            "large_vehicle": unsupported_large_vehicle,
        },
    }
    return tuple(transformed), report


# Compatibility name retained for the audit/test code written before the
# explicit calibrated medium-vehicle profile was added.
transform_segments_to_passenger_bbox_center = (
    transform_segments_to_calibrated_bbox_center
)


def _read_jsonl(path: Path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def _project_error(dx: float, dy: float, yaw: float):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return dx * cosine + dy * sine, -dx * sine + dy * cosine


def _group_stats(rows, key):
    return statistics([float(row[key]) for row in rows])


def _rectangle_corners(x, y, length, width, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in (
            (-length / 2.0, -width / 2.0),
            (length / 2.0, -width / 2.0),
            (length / 2.0, width / 2.0),
            (-length / 2.0, width / 2.0),
        )
    ]


def _polygon_area(points):
    if len(points) < 3:
        return 0.0
    return abs(
        sum(
            points[index][0] * points[(index + 1) % len(points)][1]
            - points[(index + 1) % len(points)][0] * points[index][1]
            for index in range(len(points))
        )
    ) / 2.0


def _polygon_clip(subject, clip):
    """Sutherland-Hodgman clip for counter-clockwise convex polygons."""
    output = list(subject)
    for index, edge_start in enumerate(clip):
        edge_end = clip[(index + 1) % len(clip)]
        input_points = output
        output = []
        if not input_points:
            break

        def inside(point):
            return (
                (edge_end[0] - edge_start[0]) * (point[1] - edge_start[1])
                - (edge_end[1] - edge_start[1]) * (point[0] - edge_start[0])
            ) >= -1.0e-9

        def intersection(first, second):
            edge_x = edge_end[0] - edge_start[0]
            edge_y = edge_end[1] - edge_start[1]
            segment_x = second[0] - first[0]
            segment_y = second[1] - first[1]
            denominator = segment_x * edge_y - segment_y * edge_x
            if abs(denominator) < 1.0e-12:
                return second
            ratio = (
                (edge_start[0] - first[0]) * edge_y
                - (edge_start[1] - first[1]) * edge_x
            ) / denominator
            return first[0] + ratio * segment_x, first[1] + ratio * segment_y

        previous = input_points[-1]
        for current in input_points:
            if inside(current):
                if not inside(previous):
                    output.append(intersection(previous, current))
                output.append(current)
            elif inside(previous):
                output.append(intersection(previous, current))
            previous = current
    return output


def oriented_bev_iou(first, second):
    first_polygon = _rectangle_corners(*first)
    second_polygon = _rectangle_corners(*second)
    intersection = _polygon_area(_polygon_clip(first_polygon, second_polygon))
    union = first[2] * first[3] + second[2] * second[3] - intersection
    return intersection / union if union > 0.0 else 0.0


def audit_capture_dataset(dataset_root: str | Path):
    """Compare scenario actor initPosition with SaveSensorData Instance boxes.

    Generated collection scenes use stationary actors while Ego moves through six
    capture poses.  The tangent of the recorded Ego trigger positions recovers
    the planar LiDAR heading.  A tiny lateral residual is an independent sanity
    check that the inferred transform is not manufacturing the longitudinal
    offset.
    """
    root = Path(dataset_root)
    frames = {
        item["frame_id"]: item
        for item in _read_jsonl(root / "metadata" / "frames.jsonl")
    }
    annotations = {
        item["frame_id"]: item["boxes"]
        for item in _read_jsonl(root / "annotations" / "boxes_3d.jsonl")
    }
    scenes = {
        item["scene_id"]: item
        for item in _read_jsonl(root / "metadata" / "scenes.jsonl")
    }
    scene_frames = defaultdict(list)
    for frame in frames.values():
        scene_frames[frame["scene_id"]].append(frame)

    rows = []
    for scene_id, ordered_frames in scene_frames.items():
        ordered_frames.sort(key=lambda item: int(item["frame_index"]))
        scenario_path = Path(scenes[scene_id]["scenario"])
        if not scenario_path.is_absolute():
            # Dataset paths are repository-relative. The caller normally runs
            # from the repository root, as do the other evaluation scripts.
            scenario_path = Path.cwd() / scenario_path
        scenario = json.loads(scenario_path.read_text())
        actors = {
            int(actor["UNIQUEID"]): actor for actor in scenario["vehicleList"]
        }
        if not actors or len(ordered_frames) < 2:
            continue
        ego_xy = np.asarray(
            [
                [
                    frame["trigger_ego_position"]["x"],
                    frame["trigger_ego_position"]["y"],
                ]
                for frame in ordered_frames
            ],
            dtype=np.float64,
        )
        for index, frame in enumerate(ordered_frames):
            before = max(0, index - 1)
            after = min(len(ordered_frames) - 1, index + 1)
            tangent = ego_xy[after] - ego_xy[before]
            if float(np.linalg.norm(tangent)) < 1.0e-6:
                continue
            ego_yaw = math.atan2(float(tangent[1]), float(tangent[0]))
            cosine = math.cos(ego_yaw)
            sine = math.sin(ego_yaw)
            ego = frame["trigger_ego_position"]
            for box in annotations.get(frame["frame_id"], ()):
                unique_id = int(box["instance_id"])
                if box["class"] != "vehicle" or unique_id not in actors:
                    continue
                actor = actors[unique_id]
                position = actor["initPosition"]["pos"]
                actor_yaw = math.radians(
                    float(actor["initPosition"]["rot"]["yaw"])
                )
                box_values = box["box_lidar"]
                local_x = 0.855 + float(box_values[0])
                local_y = float(box_values[1])
                center_x = float(ego["x"]) + cosine * local_x - sine * local_y
                center_y = float(ego["y"]) + sine * local_x + cosine * local_y
                center_z = float(ego["z"]) + 1.388 + float(box_values[2])
                dx = center_x - float(position["x"])
                dy = center_y - float(position["y"])
                longitudinal, lateral = _project_error(dx, dy, actor_yaw)
                rows.append(
                    {
                        "scene_id": scene_id,
                        "data_id": int(actor["DataID"]),
                        "length_m": float(box_values[3]),
                        "width_m": float(box_values[4]),
                        "height_m": float(box_values[5]),
                        "longitudinal_offset_m": longitudinal,
                        "lateral_offset_m": lateral,
                        "vertical_offset_m": center_z - float(position["z"]),
                        "center_distance_m": math.hypot(dx, dy),
                    }
                )

    passenger_rows = [row for row in rows if row["length_m"] < 5.5]
    length_bins = []
    for minimum, maximum in ((0.0, 4.5), (4.5, 5.5), (5.5, 7.5), (7.5, 20.0)):
        selected = [
            row for row in rows if minimum <= row["length_m"] < maximum
        ]
        length_bins.append(
            {
                "minimum_m": minimum,
                "maximum_m": maximum,
                "count": len(selected),
                "longitudinal_offset_m": _group_stats(
                    selected, "longitudinal_offset_m"
                ),
            }
        )

    per_asset = []
    grouped = defaultdict(list)
    for row in rows:
        grouped[row["data_id"]].append(row)
    for data_id, selected in sorted(grouped.items()):
        per_asset.append(
            {
                "data_id": data_id,
                "count": len(selected),
                "length_m": float(np.median([row["length_m"] for row in selected])),
                "width_m": float(np.median([row["width_m"] for row in selected])),
                "height_m": float(np.median([row["height_m"] for row in selected])),
                "longitudinal_offset_m": _group_stats(
                    selected, "longitudinal_offset_m"
                ),
                "lateral_offset_m": _group_stats(selected, "lateral_offset_m"),
            }
        )

    return {
        "source": str(root),
        "vehicle_box_count": len(rows),
        "scene_count": len({row["scene_id"] for row in rows}),
        "all_assets": {
            "longitudinal_offset_m": _group_stats(rows, "longitudinal_offset_m"),
            "lateral_offset_m": _group_stats(rows, "lateral_offset_m"),
            "vertical_offset_m": _group_stats(rows, "vertical_offset_m"),
        },
        "passenger_sized_under_5_5m": {
            "count": len(passenger_rows),
            "longitudinal_offset_m": _group_stats(
                passenger_rows, "longitudinal_offset_m"
            ),
            "lateral_offset_m": _group_stats(passenger_rows, "lateral_offset_m"),
        },
        "length_bins": length_bins,
        "per_asset": per_asset,
        "method_caveat": (
            "Ego yaw is inferred from adjacent 1.5 m trigger positions. The "
            "near-zero lateral residual supports the planar transform, but the "
            "reported offset remains an empirical MORAI scenario-position to "
            "SaveSensorData Instance-box-center mapping."
        ),
    }


def _nearest(items, timestamps, timestamp, maximum_delta_sec=0.03):
    index = bisect.bisect_left(timestamps, timestamp)
    candidates = [item for item in (index - 1, index) if 0 <= item < len(items)]
    if not candidates:
        return None
    selected = min(candidates, key=lambda item: abs(timestamps[item] - timestamp))
    if abs(timestamps[selected] - timestamp) > maximum_delta_sec:
        return None
    return items[selected]


def _read_raw_states(raw_bag):
    import rosbag

    ego_states = []
    status_frames = []
    with rosbag.Bag(str(raw_bag), "r") as bag:
        for topic, message, bag_timestamp in bag.read_messages(
            topics=["/Ego_topic", "/Object_topic"]
        ):
            timestamp = message_timestamp(message, bag_timestamp)
            if topic == "/Ego_topic":
                ego_states.append(
                    EgoState(
                        timestamp,
                        float(message.position.x),
                        float(message.position.y),
                        float(message.position.z),
                        math.radians(float(message.heading)),
                    )
                )
                continue
            status_frames.append(
                (
                    timestamp,
                    tuple(
                        StatusBox(
                            int(obj.unique_id),
                            str(obj.name),
                            float(obj.position.x),
                            float(obj.position.y),
                            float(obj.position.z),
                            math.radians(float(obj.heading)),
                            float(obj.size.x),
                            float(obj.size.y),
                            float(obj.size.z),
                        )
                        for obj in message.npc_list
                    ),
                )
            )
    return ego_states, status_frames


def _read_detection_boxes(output_bag, detection_topic):
    import rosbag

    frames = []
    with rosbag.Bag(str(output_bag), "r") as bag:
        for _, message, bag_timestamp in bag.read_messages(topics=[detection_topic]):
            boxes = []
            for obj in message.objects:
                if not obj.classification:
                    continue
                label = int(
                    max(obj.classification, key=lambda item: item.probability).label
                )
                if label not in VEHICLE_LABELS:
                    continue
                pose = obj.kinematics.pose_with_covariance.pose
                boxes.append(
                    DetectionBox(
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                        quaternion_yaw(pose.orientation),
                        float(obj.shape.dimensions.x),
                        float(obj.shape.dimensions.y),
                        float(obj.shape.dimensions.z),
                    )
                )
            frames.append((message_timestamp(message, bag_timestamp), tuple(boxes)))
    return frames


def audit_online_detection_geometry(
    raw_bag,
    output_bag,
    *,
    reference_offset_m: float,
    detection_topic="/perception/object_recognition/detection/objects",
    maximum_association_distance_m=4.0,
):
    """Compare online detector boxes with raw MORAI ObjectStatus geometry."""
    ego_states, status_frames = _read_raw_states(raw_bag)
    detections = _read_detection_boxes(output_bag, detection_topic)
    ego_times = [item.timestamp for item in ego_states]
    status_times = [item[0] for item in status_frames]
    rows = []
    for timestamp, source_boxes in detections:
        ego = _nearest(ego_states, ego_times, timestamp)
        status_frame = _nearest(status_frames, status_times, timestamp)
        if ego is None or status_frame is None or not source_boxes or not status_frame[1]:
            continue
        cosine = math.cos(ego.yaw)
        sine = math.sin(ego.yaw)
        map_boxes = []
        for box in source_boxes:
            local_x = 0.855 + box.x
            local_y = box.y
            map_boxes.append(
                DetectionBox(
                    ego.x + cosine * local_x - sine * local_y,
                    ego.y + sine * local_x + cosine * local_y,
                    ego.z + 1.388 + box.z,
                    (box.yaw + ego.yaw + math.pi) % (2.0 * math.pi) - math.pi,
                    box.length,
                    box.width,
                    box.height,
                )
            )
        statuses = status_frame[1]
        distances = np.asarray(
            [
                [math.hypot(box.x - status.x, box.y - status.y) for status in statuses]
                for box in map_boxes
            ],
            dtype=np.float64,
        )
        detector_indices, status_indices = linear_sum_assignment(distances)
        for detector_index, status_index in zip(detector_indices, status_indices):
            distance = float(distances[detector_index, status_index])
            if distance > maximum_association_distance_m:
                continue
            box = map_boxes[int(detector_index)]
            status = statuses[int(status_index)]
            dx = box.x - status.x
            dy = box.y - status.y
            longitudinal, lateral = _project_error(dx, dy, status.yaw)
            corrected_dx = dx - reference_offset_m * math.cos(status.yaw)
            corrected_dy = dy - reference_offset_m * math.sin(status.yaw)
            status_raw_box = (
                status.x,
                status.y,
                status.length,
                status.width,
                status.yaw,
            )
            status_corrected_box = (
                status.x + reference_offset_m * math.cos(status.yaw),
                status.y + reference_offset_m * math.sin(status.yaw),
                status.length,
                status.width,
                status.yaw,
            )
            detector_box = (
                box.x,
                box.y,
                box.length,
                box.width,
                box.yaw,
            )
            yaw_error = abs(
                (box.yaw - status.yaw + math.pi) % (2.0 * math.pi) - math.pi
            )
            rows.append(
                {
                    "longitudinal_offset_m": longitudinal,
                    "lateral_offset_m": lateral,
                    "object_status_reference_distance_m": distance,
                    "reference_corrected_center_distance_m": math.hypot(
                        corrected_dx, corrected_dy
                    ),
                    "length_error_m": box.length - status.length,
                    "width_error_m": box.width - status.width,
                    "height_error_m": box.height - status.height,
                    "yaw_error_deg": math.degrees(yaw_error),
                    "raw_reference_bev_iou": oriented_bev_iou(
                        detector_box, status_raw_box
                    ),
                    "corrected_reference_bev_iou": oriented_bev_iou(
                        detector_box, status_corrected_box
                    ),
                }
            )
    return {
        "raw_bag": str(raw_bag),
        "output_bag": str(output_bag),
        "match_count": len(rows),
        "reference_offset_m": reference_offset_m,
        "detector_minus_object_status": {
            "longitudinal_offset_m": _group_stats(rows, "longitudinal_offset_m"),
            "lateral_offset_m": _group_stats(rows, "lateral_offset_m"),
            "center_distance_m": _group_stats(
                rows, "object_status_reference_distance_m"
            ),
        },
        "after_empirical_reference_correction": {
            "center_distance_m": _group_stats(
                rows, "reference_corrected_center_distance_m"
            ),
            "bev_iou": _group_stats(rows, "corrected_reference_bev_iou"),
        },
        "raw_reference_bev_iou": _group_stats(rows, "raw_reference_bev_iou"),
        "yaw_error_deg": _group_stats(rows, "yaw_error_deg"),
        "detector_minus_status_size": {
            "length_error_m": _group_stats(rows, "length_error_m"),
            "width_error_m": _group_stats(rows, "width_error_m"),
            "height_error_m": _group_stats(rows, "height_error_m"),
        },
        "method_caveat": (
            "Association is fixed by nearest header time and Hungarian center "
            "distance. Online detector centers can additionally contain model "
            "error and rolling LiDAR scan distortion."
        ),
    }
