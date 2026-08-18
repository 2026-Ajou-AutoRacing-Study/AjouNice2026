"""Quantify fixed-vector and Frenet offsets against Lanelet centerlines."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from perception_object_pipeline.map_based_prediction import LaneletNetwork

from .pipeline_evaluator import statistics


def signed_statistics(values):
    values = list(values)
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(array.mean()),
        "p05": float(np.percentile(array, 5)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "minimum": float(array.min()),
        "maximum": float(array.max()),
    }


def summarize_centerline_residuals(rows, input_count=None):
    """Summarize projection-minus-reference residual vectors.

    A non-zero global-vector mean/median is evidence for a map-frame
    translation bias. Longitudinal/lateral components use the source heading
    and separate that fixed-vector hypothesis from ordinary lane-following
    offset.
    """
    rows = list(rows)
    dx = np.asarray([row["correction_dx_m"] for row in rows], dtype=np.float64)
    dy = np.asarray([row["correction_dy_m"] for row in rows], dtype=np.float64)
    if rows:
        mean_vector = [float(dx.mean()), float(dy.mean())]
        median_vector = [float(np.median(dx)), float(np.median(dy))]
    else:
        mean_vector = None
        median_vector = None

    def magnitude(vector):
        return math.hypot(*vector) if vector is not None else None

    total = len(rows) if input_count is None else int(input_count)
    return {
        "input_count": total,
        "matched_count": len(rows),
        "match_rate": len(rows) / total if total else None,
        "distance_m": statistics(row["distance_m"] for row in rows),
        "global_correction_dx_m": signed_statistics(dx),
        "global_correction_dy_m": signed_statistics(dy),
        "global_correction_mean_vector_xy_m": mean_vector,
        "global_correction_mean_magnitude_m": magnitude(mean_vector),
        "global_correction_median_vector_xy_m": median_vector,
        "global_correction_median_magnitude_m": magnitude(median_vector),
        "source_longitudinal_correction_m": signed_statistics(
            row["longitudinal_correction_m"] for row in rows
        ),
        "source_lateral_correction_m": signed_statistics(
            row["lateral_correction_m"] for row in rows
        ),
        "absolute_heading_error_deg": statistics(
            row["absolute_heading_error_deg"] for row in rows
        ),
    }


def project_samples_to_lanelets(network: LaneletNetwork, samples):
    rows = []
    samples = list(samples)
    for sample in samples:
        x = float(sample["x"])
        y = float(sample["y"])
        yaw = float(sample["yaw"])
        candidates = network.lane_candidates(x, y, yaw)
        if not candidates:
            continue
        projection = candidates[0].projection
        dx = projection.x - x
        dy = projection.y - y
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        yaw_error = math.atan2(
            math.sin(projection.yaw - yaw), math.cos(projection.yaw - yaw)
        )
        rows.append(
            {
                "correction_dx_m": dx,
                "correction_dy_m": dy,
                "distance_m": projection.distance,
                "longitudinal_correction_m": dx * cosine + dy * sine,
                "lateral_correction_m": -dx * sine + dy * cosine,
                "absolute_heading_error_deg": abs(math.degrees(yaw_error)),
                "lanelet_id": int(candidates[0].lanelet_id),
            }
        )
    return rows, summarize_centerline_residuals(rows, len(samples))


def trajectory_samples(segments, stride=10):
    stride = max(1, int(stride))
    for segment in segments:
        for point in segment.points[::stride]:
            yield {
                "x": point.x,
                "y": point.y,
                "yaw": point.yaw,
                "segment_key": segment.key,
            }


def transform_local_point(x, y, translation_xyz, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        cosine * x - sine * y + translation_xyz[0],
        sine * x + cosine * y + translation_xyz[1],
    )


def read_mgeo_samples(
    path: str | Path,
    translation_xyz,
    transform_yaw,
    stride=4,
):
    """Read direction-aware MGeo link samples in the Lanelet map frame."""
    links = json.loads(Path(path).read_text(encoding="utf-8"))
    stride = max(1, int(stride))
    samples = []
    for link in links:
        points = link.get("points") or ()
        if len(points) < 2:
            continue
        for index in range(0, len(points), stride):
            before = points[max(0, index - 1)]
            after = points[min(len(points) - 1, index + 1)]
            local_yaw = math.atan2(after[1] - before[1], after[0] - before[0])
            x, y = transform_local_point(
                float(points[index][0]),
                float(points[index][1]),
                translation_xyz,
                transform_yaw,
            )
            samples.append(
                {
                    "x": x,
                    "y": y,
                    "yaw": local_yaw + transform_yaw,
                    "link_id": str(link.get("idx", "")),
                }
            )
    return samples


def select_samples_near_trajectories(samples, trajectory_segments, radius_m=5.0):
    """Keep map samples near the bag's observed NPC operating region."""
    from scipy.spatial import cKDTree

    reference = np.asarray(
        [(point.x, point.y) for segment in trajectory_segments for point in segment.points],
        dtype=np.float64,
    )
    if reference.size == 0:
        return []
    tree = cKDTree(reference)
    selected = []
    for sample in samples:
        distance, _ = tree.query((sample["x"], sample["y"]), k=1)
        if float(distance) <= radius_m:
            selected.append(sample)
    return selected
