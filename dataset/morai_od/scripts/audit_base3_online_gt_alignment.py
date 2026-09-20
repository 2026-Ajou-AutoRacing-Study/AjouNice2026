#!/usr/bin/env python3
"""Validate online rosbag ObjectStatus GT against LiDAR and camera payloads.

The audit samples LiDAR frames inside marker-delimited capture windows, aligns
the closest Ego/Object state by message timestamp, converts MORAI map pivots to
the LiDAR frame, applies the Base3 asset-specific pivot-to-box-centre catalog,
and reports point-in-box counts before/after the correction.  It also writes
BEV and camera projection images for manual oracle inspection.
"""

from __future__ import annotations

import argparse
import bisect
import copy
import io
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from PIL import Image, ImageDraw
from sensor_msgs import point_cloud2

from audit_base3_online_bag import build_capture_windows, message_stamp


MARKER_TOPIC = "/dataset/scenario_marker"
LIDAR_TOPIC = "/sensor/lidar1"
EGO_TOPIC = "/Ego_topic"
OBJECT_TOPIC = "/Object_topic"
CAMERAS = {
    "front": "/sensor/camera/front/compressed",
    "front_left": "/sensor/camera/front_left/compressed",
    "front_right": "/sensor/camera/front_right/compressed",
    "back": "/sensor/camera/back/compressed",
}
CALIBRATION_CAMERA_NAMES = {
    "front": "Camera_front",
    "front_left": "Camera_front_left",
    "front_right": "Camera_front_right",
    "back": "Camera_back",
}
COLORS = {
    "vehicle": (0, 255, 0),
    "pedestrian": (255, 160, 0),
    "obstacle": (0, 160, 255),
}
BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bags", nargs="+", type=Path,
        help="one bag or an ordered set of rosbag --split parts",
    )
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--asset-offsets", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--samples-per-scene", type=int, default=3)
    parser.add_argument("--max-state-age-sec", type=float, default=0.10)
    parser.add_argument(
        "--state-alignment", choices=("interpolate", "nearest"),
        default="interpolate",
    )
    parser.add_argument("--max-extrapolation-sec", type=float, default=0.05)
    return parser.parse_args()


def wrapped_radians(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def nearest(stamp, samples):
    stamps = [row[0] for row in samples]
    index = bisect.bisect_left(stamps, stamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda row: abs(row[0] - stamp))


def state_support(stamp, samples):
    """Return the samples immediately before/after ``stamp``.

    Rows are expected to be sorted by their message timestamp.  Exact matches
    are returned as both endpoints so callers do not introduce numerical
    motion into an already synchronized state.
    """
    stamps = [row[0] for row in samples]
    index = bisect.bisect_left(stamps, stamp)
    if index < len(samples) and samples[index][0] == stamp:
        return samples[index], samples[index]
    before = samples[index - 1] if index else None
    after = samples[index] if index < len(samples) else None
    return before, after


def interpolate_scalar(first, second, alpha):
    return float(first) + alpha * (float(second) - float(first))


def interpolate_heading_degrees(first, second, alpha):
    delta = (float(second) - float(first) + 180.0) % 360.0 - 180.0
    return (float(first) + alpha * delta) % 360.0


def interpolate_vector(target, first, second, alpha):
    for axis in ("x", "y", "z"):
        setattr(
            target, axis,
            interpolate_scalar(getattr(first, axis), getattr(second, axis), alpha),
        )


def extrapolate_pose_with_body_velocity(message, delta_sec, velocity_scale=1.0):
    """Move a MORAI pose using its body-frame velocity for a short interval."""
    result = copy.deepcopy(message)
    heading = math.radians(float(message.heading))
    forward = float(message.velocity.x) * float(velocity_scale)
    lateral = float(message.velocity.y) * float(velocity_scale)
    result.position.x += delta_sec * (
        math.cos(heading) * forward - math.sin(heading) * lateral
    )
    result.position.y += delta_sec * (
        math.sin(heading) * forward + math.cos(heading) * lateral
    )
    result.position.z += delta_sec * float(message.velocity.z) * float(
        velocity_scale
    )
    return result


def interpolate_pose(first, second, alpha):
    result = copy.deepcopy(first)
    interpolate_vector(result.position, first.position, second.position, alpha)
    result.heading = interpolate_heading_degrees(
        first.heading, second.heading, alpha
    )
    for field in ("velocity", "acceleration"):
        if hasattr(result, field) and hasattr(first, field) and hasattr(second, field):
            interpolate_vector(
                getattr(result, field), getattr(first, field),
                getattr(second, field), alpha,
            )
    return result


def align_ego_state(stamp, samples, mode="interpolate", max_extrapolation_sec=0.05):
    closest = nearest(stamp, samples)
    closest_age = abs(float(closest[0]) - float(stamp))
    if mode == "nearest":
        return closest[2], {
            "mode": "nearest", "nearest_age_sec": closest_age,
            "support_span_sec": 0.0, "motion_m": 0.0,
        }

    before, after = state_support(stamp, samples)
    if before is not None and after is not None:
        if before[0] == after[0]:
            return copy.deepcopy(before[2]), {
                "mode": "exact", "nearest_age_sec": 0.0,
                "support_span_sec": 0.0, "motion_m": 0.0,
            }
        span = float(after[0] - before[0])
        alpha = float(stamp - before[0]) / span
        aligned = interpolate_pose(before[2], after[2], alpha)
        motion = math.hypot(
            float(aligned.position.x) - float(closest[2].position.x),
            float(aligned.position.y) - float(closest[2].position.y),
        )
        return aligned, {
            "mode": "interpolated", "nearest_age_sec": closest_age,
            "support_span_sec": span, "alpha": alpha, "motion_m": motion,
        }

    if closest_age > max_extrapolation_sec:
        raise ValueError(
            "ego extrapolation {:.6f}s exceeds {:.6f}s".format(
                closest_age, max_extrapolation_sec
            )
        )
    delta = float(stamp - closest[0])
    aligned = extrapolate_pose_with_body_velocity(closest[2], delta)
    motion = math.hypot(
        float(aligned.position.x) - float(closest[2].position.x),
        float(aligned.position.y) - float(closest[2].position.y),
    )
    return aligned, {
        "mode": "extrapolated", "nearest_age_sec": closest_age,
        "support_span_sec": 0.0, "delta_sec": delta, "motion_m": motion,
    }


def actor_key(actor):
    return int(actor.unique_id), str(actor.name)


def actor_lists(message):
    return (
        ("npc_list", "vehicle"),
        ("pedestrian_list", "pedestrian"),
        ("obstacle_list", "obstacle"),
    )


def actor_with_key(message, key):
    """Return one actor from an ObjectStatusList by stable (id, name)."""
    for field, _class_name in actor_lists(message):
        for actor in getattr(message, field):
            if actor_key(actor) == key:
                return actor
    return None


def actor_yaw_rate_at(
    stamp,
    samples,
    actor,
    max_support_span_sec=0.10,
    max_abs_yaw_rate_radps=2.0,
):
    """Estimate causal-neutral actor yaw rate from bracketing ObjectStatus rows.

    A strict sample on each side of the LiDAR timestamp is required.  This
    keeps spawn/despawn boundaries and long communication gaps from injecting
    a synthetic rotational velocity into the BBox-centre target.
    """
    stamps = [row[0] for row in samples]
    right = bisect.bisect_right(stamps, stamp)
    left = bisect.bisect_left(stamps, stamp) - 1
    if left < 0 or right >= len(samples):
        return None, {"valid": False, "reason": "unbracketed"}

    before = samples[left]
    after = samples[right]
    span = float(after[0] - before[0])
    # Decimal ROS periods such as 0.1 s are not exactly representable as a
    # binary float, so keep a tiny numerical tolerance at the validity gate.
    if span <= 0.0 or span > float(max_support_span_sec) + 1e-9:
        return None, {
            "valid": False,
            "reason": "support_span",
            "support_span_sec": span,
        }

    key = actor_key(actor)
    before_actor = actor_with_key(before[2], key)
    after_actor = actor_with_key(after[2], key)
    if before_actor is None or after_actor is None:
        return None, {
            "valid": False,
            "reason": "actor_not_bracketed",
            "support_span_sec": span,
        }

    delta_deg = (
        float(after_actor.heading) - float(before_actor.heading) + 180.0
    ) % 360.0 - 180.0
    yaw_rate = math.radians(delta_deg) / span
    if abs(yaw_rate) > float(max_abs_yaw_rate_radps):
        return None, {
            "valid": False,
            "reason": "yaw_rate_limit",
            "support_span_sec": span,
            "yaw_rate_radps": yaw_rate,
        }
    return yaw_rate, {
        "valid": True,
        "reason": "bracketed_heading",
        "support_span_sec": span,
        "yaw_rate_radps": yaw_rate,
    }


def align_object_state(
    stamp, samples, mode="interpolate", max_extrapolation_sec=0.05
):
    closest = nearest(stamp, samples)
    closest_age = abs(float(closest[0]) - float(stamp))
    if mode == "nearest":
        return closest[2], {
            "mode": "nearest", "nearest_age_sec": closest_age,
            "support_span_sec": 0.0, "motion_m": 0.0,
            "interpolated_actors": 0, "fallback_actors": 0,
        }

    before, after = state_support(stamp, samples)
    if before is None or after is None:
        if closest_age > max_extrapolation_sec:
            raise ValueError(
                "object extrapolation {:.6f}s exceeds {:.6f}s".format(
                    closest_age, max_extrapolation_sec
                )
            )
        delta = float(stamp - closest[0])
        result = copy.deepcopy(closest[2])
        motions = []
        count = 0
        for field, _class_name in actor_lists(result):
            values = []
            for actor in getattr(result, field):
                # MORAI ObjectStatus velocity is km/h, whereas EgoVehicleStatus
                # velocity is m/s. Keep extrapolation in physical SI units.
                aligned = extrapolate_pose_with_body_velocity(
                    actor, delta, velocity_scale=1.0 / 3.6
                )
                motions.append(math.hypot(
                    float(aligned.position.x) - float(actor.position.x),
                    float(aligned.position.y) - float(actor.position.y),
                ))
                values.append(aligned)
                count += 1
            setattr(result, field, values)
        return result, {
            "mode": "extrapolated", "nearest_age_sec": closest_age,
            "support_span_sec": 0.0, "delta_sec": delta,
            "motion_m": max(motions, default=0.0),
            "interpolated_actors": 0, "fallback_actors": count,
        }

    if before[0] == after[0]:
        return copy.deepcopy(before[2]), {
            "mode": "exact", "nearest_age_sec": 0.0,
            "support_span_sec": 0.0, "motion_m": 0.0,
            "interpolated_actors": 0, "fallback_actors": 0,
        }

    span = float(after[0] - before[0])
    alpha = float(stamp - before[0]) / span
    result = copy.deepcopy(closest[2])
    motions = []
    interpolated_count = 0
    fallback_count = 0
    for field, _class_name in actor_lists(result):
        before_by_key = {
            actor_key(actor): actor for actor in getattr(before[2], field)
        }
        after_by_key = {
            actor_key(actor): actor for actor in getattr(after[2], field)
        }
        aligned_values = []
        for actor in getattr(result, field):
            key = actor_key(actor)
            if key in before_by_key and key in after_by_key:
                aligned = interpolate_pose(
                    before_by_key[key], after_by_key[key], alpha
                )
                interpolated_count += 1
            else:
                # Preserve the nearest message's actor membership at a
                # spawn/despawn boundary instead of hallucinating an actor
                # from the future or silently deleting a current one.
                aligned = copy.deepcopy(actor)
                fallback_count += 1
            motions.append(math.hypot(
                float(aligned.position.x) - float(actor.position.x),
                float(aligned.position.y) - float(actor.position.y),
            ))
            aligned_values.append(aligned)
        setattr(result, field, aligned_values)
    return result, {
        "mode": "interpolated", "nearest_age_sec": closest_age,
        "support_span_sec": span, "alpha": alpha,
        "motion_m": max(motions, default=0.0),
        "interpolated_actors": interpolated_count,
        "fallback_actors": fallback_count,
    }


def cloud_xyz_i(message):
    rows = list(
        point_cloud2.read_points(
            message, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
    )
    return np.asarray(rows, dtype=np.float32).reshape(-1, 4)


def ego_relative_position(ego, actor):
    heading = math.radians(float(ego.heading))
    cosine, sine = math.cos(heading), math.sin(heading)
    dx = float(actor.position.x) - float(ego.position.x)
    dy = float(actor.position.y) - float(ego.position.y)
    return np.asarray(
        [
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            float(actor.position.z) - float(ego.position.z),
            1.0,
        ],
        dtype=np.float64,
    )


def actors(message):
    for class_name, values in (
        ("vehicle", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    ):
        for actor in values:
            yield class_name, actor


def corrected_box(ego, class_name, actor, calibration, catalog):
    pivot_ego = ego_relative_position(ego, actor)
    ego_to_lidar = np.asarray(calibration["lidar"]["ego_to_lidar"], dtype=np.float64)
    pivot_lidar = (ego_to_lidar @ pivot_ego)[:3]
    yaw = wrapped_radians(math.radians(float(actor.heading) - float(ego.heading)))
    asset = catalog.get(str(actor.name))
    if asset is None:
        return None, None
    offset = np.asarray(asset["pivot_to_bbox_center_local_m"], dtype=np.float64)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    offset_lidar = np.asarray(
        [
            cosine * offset[0] - sine * offset[1],
            sine * offset[0] + cosine * offset[1],
            offset[2],
        ]
    )
    raw_size = np.asarray(
        [float(actor.size.x), float(actor.size.y), float(actor.size.z)]
    )
    corrected = np.r_[pivot_lidar + offset_lidar, asset["bbox_size_m"], yaw]
    naive = np.r_[pivot_lidar, raw_size, yaw]
    return naive, corrected


def points_in_box(points, box, tolerance=0.03):
    translated = points[:, :3] - box[:3]
    cosine, sine = math.cos(float(box[6])), math.sin(float(box[6]))
    local_x = cosine * translated[:, 0] + sine * translated[:, 1]
    local_y = -sine * translated[:, 0] + cosine * translated[:, 1]
    dims = np.asarray(box[3:6]) + 2.0 * tolerance
    mask = (
        (np.abs(local_x) <= dims[0] * 0.5)
        & (np.abs(local_y) <= dims[1] * 0.5)
        & (np.abs(translated[:, 2]) <= dims[2] * 0.5)
    )
    return int(mask.sum()), mask


def box_corners(box):
    length, width, height = box[3:6]
    corners = np.asarray(
        [
            [length / 2, width / 2, -height / 2],
            [length / 2, -width / 2, -height / 2],
            [-length / 2, -width / 2, -height / 2],
            [-length / 2, width / 2, -height / 2],
            [length / 2, width / 2, height / 2],
            [length / 2, -width / 2, height / 2],
            [-length / 2, -width / 2, height / 2],
            [-length / 2, width / 2, height / 2],
        ],
        dtype=np.float64,
    )
    cosine, sine = math.cos(float(box[6])), math.sin(float(box[6]))
    rotation = np.asarray([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
    return corners @ rotation.T + box[:3]


def draw_bev(path, points, box_rows, title):
    figure, axis = plt.subplots(figsize=(8, 8))
    keep = (np.abs(points[:, 0]) <= 50) & (np.abs(points[:, 1]) <= 30)
    axis.scatter(points[keep, 0], points[keep, 1], s=0.25, c=points[keep, 2], cmap="gray")
    for row in box_rows:
        box = row["corrected"]
        corners = box_corners(box)[:4]
        polygon = np.vstack([corners[:, :2], corners[0, :2]])
        color = np.asarray(COLORS[row["class"]]) / 255.0
        axis.plot(polygon[:, 0], polygon[:, 1], color=color, linewidth=2)
        axis.text(box[0], box[1], "{}:{} pts".format(row["name"], row["corrected_points"]), fontsize=7)
    axis.set_xlim(-30, 50)
    axis.set_ylim(-30, 30)
    axis.set_aspect("equal")
    axis.grid(True, alpha=0.2)
    axis.set_xlabel("LiDAR x forward [m]")
    axis.set_ylabel("LiDAR y left [m]")
    axis.set_title(title)
    figure.tight_layout()
    figure.savefig(str(path), dpi=160)
    plt.close(figure)


def draw_camera(path, message, box_rows, camera_name, calibration):
    image = Image.open(io.BytesIO(bytes(message.data))).convert("RGB")
    draw = ImageDraw.Draw(image)
    camera = calibration["cameras"][CALIBRATION_CAMERA_NAMES[camera_name]]
    lidar_to_ego = np.asarray(calibration["lidar"]["lidar_to_ego"], dtype=np.float64)
    ego_to_camera = np.asarray(camera["ego_to_sensor"], dtype=np.float64)
    lidar_to_camera = ego_to_camera @ lidar_to_ego
    intrinsic = np.asarray(camera["intrinsic"], dtype=np.float64)
    for row in box_rows:
        corners = box_corners(row["corrected"])
        homogeneous = np.c_[corners, np.ones(len(corners))]
        camera_xyz = (lidar_to_camera @ homogeneous.T).T[:, :3]
        projected = (intrinsic @ camera_xyz.T).T
        valid = camera_xyz[:, 2] > 0.2
        uv = np.zeros((8, 2), dtype=np.float64)
        uv[valid] = projected[valid, :2] / projected[valid, 2:3]
        color = COLORS[row["class"]]
        for first, second in BOX_EDGES:
            if valid[first] and valid[second]:
                draw.line(
                    [tuple(uv[first]), tuple(uv[second])], fill=color, width=3
                )
    image.save(str(path), quality=92)


def selected_indices(count, target):
    if count <= target:
        return list(range(count))
    return sorted(set(int(round(value)) for value in np.linspace(0, count - 1, target + 2)[1:-1]))


def render_window(window, messages, args, calibration, catalog, unknown_assets):
    scene_lidar = messages[LIDAR_TOPIC]
    scene_states = {
        topic: sorted(messages[topic], key=lambda row: row[0])
        for topic in (EGO_TOPIC, OBJECT_TOPIC, *CAMERAS.values())
    }
    required = (LIDAR_TOPIC, EGO_TOPIC, OBJECT_TOPIC, *CAMERAS.values())
    missing = [topic for topic in required if not messages[topic]]
    if missing:
        raise SystemExit(
            "capture window {} has no messages for {}".format(
                window["scene_id"], ", ".join(missing)
            )
        )

    rows = []
    for sample_number, lidar_index in enumerate(
        selected_indices(len(scene_lidar), args.samples_per_scene)
    ):
        lidar_stamp, _lidar_record, lidar_message = scene_lidar[lidar_index]
        ego_row = nearest(lidar_stamp, scene_states[EGO_TOPIC])
        object_row = nearest(lidar_stamp, scene_states[OBJECT_TOPIC])
        ego_age = abs(ego_row[0] - lidar_stamp)
        object_age = abs(object_row[0] - lidar_stamp)
        if max(ego_age, object_age) > args.max_state_age_sec:
            raise SystemExit(
                "state age exceeds limit for {} at {:.6f}".format(
                    window["scene_id"], lidar_stamp
                )
            )
        ego_state, ego_alignment = align_ego_state(
            lidar_stamp, scene_states[EGO_TOPIC], args.state_alignment,
            args.max_extrapolation_sec,
        )
        object_state, object_alignment = align_object_state(
            lidar_stamp, scene_states[OBJECT_TOPIC], args.state_alignment,
            args.max_extrapolation_sec,
        )
        points = cloud_xyz_i(lidar_message)
        box_rows = []
        nearest_actors = {
            (class_name, actor_key(actor)): actor
            for class_name, actor in actors(object_row[2])
        }
        for class_name, actor in actors(object_state):
            naive, corrected = corrected_box(
                ego_state, class_name, actor, calibration, catalog
            )
            if corrected is None:
                unknown_assets.add(str(actor.name))
                continue
            nearest_actor = nearest_actors.get((class_name, actor_key(actor)))
            nearest_corrected = None
            nearest_corrected_count = None
            if nearest_actor is not None:
                _nearest_naive, nearest_corrected = corrected_box(
                    ego_row[2], class_name, nearest_actor, calibration, catalog
                )
                if nearest_corrected is not None:
                    nearest_corrected_count, _ = points_in_box(
                        points, nearest_corrected
                    )
            naive_count, _ = points_in_box(points, naive)
            corrected_count, _ = points_in_box(points, corrected)
            box_rows.append(
                {
                    "class": class_name,
                    "name": str(actor.name),
                    "unique_id": int(actor.unique_id),
                    "naive": naive,
                    "corrected": corrected,
                    "naive_points": naive_count,
                    "corrected_points": corrected_count,
                    "nearest_corrected": nearest_corrected,
                    "nearest_corrected_points": nearest_corrected_count,
                }
            )
        stem = "{}_s{:02d}".format(window["scene_id"], sample_number)
        draw_bev(
            args.output_dir / (stem + "_bev.png"), points, box_rows,
            "{} @ {:.3f}".format(window["scene_id"], lidar_stamp),
        )
        camera_ages = {}
        for camera_name, topic in CAMERAS.items():
            camera_row = nearest(lidar_stamp, scene_states[topic])
            camera_ages[camera_name] = abs(camera_row[0] - lidar_stamp)
            draw_camera(
                args.output_dir / (stem + "_{}.jpg".format(camera_name)),
                camera_row[2], box_rows, camera_name, calibration,
            )
        rows.append(
            {
                "scene_id": window["scene_id"],
                "lidar_stamp": lidar_stamp,
                "lidar_points": len(points),
                "ego_age_ms": ego_age * 1000.0,
                "object_age_ms": object_age * 1000.0,
                "ego_alignment": ego_alignment,
                "object_alignment": object_alignment,
                "camera_age_ms": {
                    key: value * 1000.0 for key, value in camera_ages.items()
                },
                "boxes": [
                    {
                        **{
                            key: value for key, value in row.items()
                            if key not in (
                                "naive", "corrected", "nearest_corrected"
                            )
                        },
                        "naive_box_lidar": row["naive"].tolist(),
                        "corrected_box_lidar": row["corrected"].tolist(),
                        "nearest_corrected_box_lidar": (
                            row["nearest_corrected"].tolist()
                            if row["nearest_corrected"] is not None else None
                        ),
                    }
                    for row in box_rows
                ],
            }
        )
    return rows


def main():
    args = arguments()
    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    catalog_doc = json.loads(args.asset_offsets.read_text(encoding="utf-8"))
    catalog = catalog_doc["assets"]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Pass 1 stores only tiny JSON markers.  The old implementation retained
    # every image and point cloud in RAM and was killed on the 24 GiB 1K bag.
    markers = []
    for bag_path in args.bags:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for _topic, message, bag_time in bag.read_messages(topics=[MARKER_TOPIC]):
                markers.append((bag_time.to_sec(), json.loads(message.data)))
    windows, errors = build_capture_windows(markers)
    if errors:
        raise SystemExit("invalid capture markers: {}".format(errors))

    # Pass 2 retains one 3--4 second capture window at a time, renders it, and
    # immediately releases its sensor payloads before advancing.
    report_rows = []
    unknown_assets = set()
    topics = (LIDAR_TOPIC, EGO_TOPIC, OBJECT_TOPIC, *CAMERAS.values())
    messages = {topic: [] for topic in topics}
    window_index = 0

    def finish_current_window():
        nonlocal messages, window_index
        report_rows.extend(
            render_window(
                windows[window_index], messages, args, calibration, catalog,
                unknown_assets,
            )
        )
        messages = {topic: [] for topic in topics}
        window_index += 1

    for bag_path in args.bags:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for topic, message, bag_time in bag.read_messages(topics=topics):
                if window_index >= len(windows):
                    break
                record_time = bag_time.to_sec()
                while (
                    window_index < len(windows)
                    and record_time > windows[window_index]["end"]
                ):
                    finish_current_window()
                if window_index >= len(windows):
                    break
                window = windows[window_index]
                if window["start"] <= record_time <= window["end"]:
                    messages[topic].append(
                        (message_stamp(message, bag_time), record_time, message)
                    )
    while window_index < len(windows):
        finish_current_window()

    naive_counts = [box["naive_points"] for row in report_rows for box in row["boxes"]]
    corrected_counts = [box["corrected_points"] for row in report_rows for box in row["boxes"]]
    nearest_corrected_counts = [
        box["nearest_corrected_points"]
        for row in report_rows for box in row["boxes"]
        if box["nearest_corrected_points"] is not None
    ]
    report = {
        "schema_version": 2,
        "bags": [str(path.resolve()) for path in args.bags],
        "calibration": str(args.calibration.resolve()),
        "asset_offsets": str(args.asset_offsets.resolve()),
        "state_alignment": args.state_alignment,
        "max_extrapolation_sec": args.max_extrapolation_sec,
        "summary": {
            "scenes": len(windows),
            "sampled_frames": len(report_rows),
            "boxes": len(corrected_counts),
            "unknown_assets": sorted(unknown_assets),
            "naive_points_in_box": {
                "total": sum(naive_counts),
                "median": float(np.median(naive_counts)) if naive_counts else None,
                "zero_boxes": sum(value == 0 for value in naive_counts),
            },
            "corrected_points_in_box": {
                "total": sum(corrected_counts),
                "median": float(np.median(corrected_counts)) if corrected_counts else None,
                "zero_boxes": sum(value == 0 for value in corrected_counts),
            },
            "nearest_corrected_points_in_box": {
                "total": sum(nearest_corrected_counts),
                "median": (
                    float(np.median(nearest_corrected_counts))
                    if nearest_corrected_counts else None
                ),
                "zero_boxes": sum(
                    value == 0 for value in nearest_corrected_counts
                ),
            },
        },
        "samples": report_rows,
    }
    target = args.output_dir / "gt_alignment_audit.json"
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
