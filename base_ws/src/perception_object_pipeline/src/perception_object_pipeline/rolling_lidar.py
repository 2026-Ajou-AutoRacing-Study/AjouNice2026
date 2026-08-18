"""Timing helpers for detector arrays produced from a rolling LiDAR scan."""

import math

import numpy as np


def representative_detection_time_offset(
    points: np.ndarray,
    point_time_offsets: np.ndarray,
    boxes: np.ndarray,
    extent_scale: float = 0.65,
    minimum_points_per_box: int = 3,
) -> float:
    """Return one robust measurement-time offset for a detection array.

    A standard DetectedObjects array has one header although a rolling scan
    observes different boxes at different instants.  Calculate each box's
    median point time first, then take their median so large/dense boxes do not
    dominate the array timestamp.  Fall back to the scan median when no box
    contains enough timed points, and to zero when no usable timing exists.
    """
    if len(points) != len(point_time_offsets) or len(points) == 0:
        return 0.0
    finite_time = np.isfinite(point_time_offsets)
    if not finite_time.any():
        return 0.0

    object_offsets = []
    for box in boxes:
        x, y, _, length, width, _, yaw = [float(value) for value in box[:7]]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        dx = points[:, 0] - x
        dy = points[:, 1] - y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        mask = (
            finite_time
            & (np.abs(local_x) <= max(0.2, length) * extent_scale)
            & (np.abs(local_y) <= max(0.2, width) * extent_scale)
        )
        if int(mask.sum()) >= minimum_points_per_box:
            object_offsets.append(float(np.median(point_time_offsets[mask])))

    if object_offsets:
        return float(np.median(np.asarray(object_offsets, dtype=np.float64)))
    return float(np.median(point_time_offsets[finite_time]))
