"""Diagnostic GT-box semantic painting helpers.

This module is deliberately detector-agnostic and has no ROS dependency so
the geometry can be unit-tested.  It is an oracle ablation tool, not a
production inference path.
"""

from __future__ import annotations

import numpy as np


def override_semantics_in_oriented_boxes(
    painted_points: np.ndarray,
    boxes: np.ndarray,
    semantic_index: int,
    *,
    semantic_offset: int = 4,
    semantic_count: int = 5,
    xy_margin_m: float = 0.08,
    bottom_clearance_m: float = 0.03,
    top_margin_m: float = 0.08,
    eligible_mask: np.ndarray | None = None,
    mode: str = "one_hot",
) -> tuple[np.ndarray, int]:
    """Set one-hot semantic features for points inside oriented GT boxes.

    Boxes use ``[x, y, z, length, width, height, yaw]`` in the same frame as
    the points.  ``z`` is the geometric box centre.  A small bottom clearance
    rejects road returns contained by MORAI's ground-touching actor boxes.
    """
    points = np.asarray(painted_points)
    box_array = np.asarray(boxes, dtype=np.float32).reshape(-1, 7)
    if points.ndim != 2 or points.shape[1] < semantic_offset + semantic_count:
        raise ValueError("painted_points does not contain the semantic features")
    if not 0 <= semantic_index < semantic_count:
        raise ValueError("semantic_index is outside the semantic feature range")
    if not len(points) or not len(box_array):
        return points, 0

    selected = np.zeros(len(points), dtype=bool)
    xyz = points[:, :3]
    for x, y, z, length, width, height, yaw in box_array:
        delta = xyz - np.asarray([x, y, z], dtype=np.float32)
        cosine = np.cos(yaw)
        sine = np.sin(yaw)
        local_x = cosine * delta[:, 0] + sine * delta[:, 1]
        local_y = -sine * delta[:, 0] + cosine * delta[:, 1]
        selected |= (
            (np.abs(local_x) <= 0.5 * length + xy_margin_m)
            & (np.abs(local_y) <= 0.5 * width + xy_margin_m)
            & (delta[:, 2] >= -0.5 * height + bottom_clearance_m)
            & (delta[:, 2] <= 0.5 * height + top_margin_m)
        )
    if eligible_mask is not None:
        eligible = np.asarray(eligible_mask, dtype=bool).reshape(-1)
        if len(eligible) != len(points):
            raise ValueError("eligible_mask length does not match painted_points")
        selected &= eligible

    count = int(np.count_nonzero(selected))
    if count:
        if mode == "one_hot":
            points[selected, semantic_offset : semantic_offset + semantic_count] = 0.0
            points[selected, semantic_offset + semantic_index] = 1.0
        elif mode == "swap_background":
            selected_indices = np.flatnonzero(selected)
            background = points[selected_indices, semantic_offset].copy()
            target = points[
                selected_indices, semantic_offset + semantic_index
            ].copy()
            needs_swap = background > target
            swap_indices = selected_indices[needs_swap]
            points[swap_indices, semantic_offset] = target[needs_swap]
            points[swap_indices, semantic_offset + semantic_index] = background[
                needs_swap
            ]
        else:
            raise ValueError("mode must be one_hot or swap_background")
    return points, count
