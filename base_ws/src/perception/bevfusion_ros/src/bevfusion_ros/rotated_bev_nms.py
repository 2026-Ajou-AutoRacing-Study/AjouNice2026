"""Dependency-free class-aware rotated BEV NMS for detector dictionaries."""

import math

from bevfusion_ros.score_thresholds import normalize_class_name


def _rectangle_corners(center, size, yaw):
    """Return a counter-clockwise oriented rectangle in the XY plane."""
    cx, cy = float(center[0]), float(center[1])
    length, width = abs(float(size[0])), abs(float(size[1]))
    cosine = math.cos(float(yaw))
    sine = math.sin(float(yaw))
    corners = []
    for local_x, local_y in (
        (-0.5 * length, -0.5 * width),
        (0.5 * length, -0.5 * width),
        (0.5 * length, 0.5 * width),
        (-0.5 * length, 0.5 * width),
    ):
        corners.append((
            cx + cosine * local_x - sine * local_y,
            cy + sine * local_x + cosine * local_y,
        ))
    return corners


def _inside(point, edge_start, edge_end):
    return (
        (edge_end[0] - edge_start[0]) * (point[1] - edge_start[1])
        - (edge_end[1] - edge_start[1]) * (point[0] - edge_start[0])
    ) >= -1.0e-9


def _line_intersection(segment_start, segment_end, edge_start, edge_end):
    segment_x = segment_end[0] - segment_start[0]
    segment_y = segment_end[1] - segment_start[1]
    edge_x = edge_end[0] - edge_start[0]
    edge_y = edge_end[1] - edge_start[1]
    denominator = segment_x * edge_y - segment_y * edge_x
    if abs(denominator) < 1.0e-12:
        return segment_end
    offset_x = edge_start[0] - segment_start[0]
    offset_y = edge_start[1] - segment_start[1]
    fraction = (offset_x * edge_y - offset_y * edge_x) / denominator
    return (
        segment_start[0] + fraction * segment_x,
        segment_start[1] + fraction * segment_y,
    )


def _polygon_intersection(subject, clip):
    output = list(subject)
    for index, edge_start in enumerate(clip):
        edge_end = clip[(index + 1) % len(clip)]
        input_polygon = output
        output = []
        if not input_polygon:
            break
        segment_start = input_polygon[-1]
        for segment_end in input_polygon:
            end_inside = _inside(segment_end, edge_start, edge_end)
            start_inside = _inside(segment_start, edge_start, edge_end)
            if end_inside:
                if not start_inside:
                    output.append(_line_intersection(
                        segment_start, segment_end, edge_start, edge_end))
                output.append(segment_end)
            elif start_inside:
                output.append(_line_intersection(
                    segment_start, segment_end, edge_start, edge_end))
            segment_start = segment_end
    return output


def _polygon_area(polygon):
    if len(polygon) < 3:
        return 0.0
    twice_area = 0.0
    for index, point in enumerate(polygon):
        following = polygon[(index + 1) % len(polygon)]
        twice_area += point[0] * following[1] - following[0] * point[1]
    return abs(twice_area) * 0.5


def rotated_bev_iou(first, second):
    """Calculate rotated XY IoU for two detector dictionaries."""
    first_size = first.get("size", ())
    second_size = second.get("size", ())
    if len(first_size) < 2 or len(second_size) < 2:
        return 0.0
    first_area = abs(float(first_size[0]) * float(first_size[1]))
    second_area = abs(float(second_size[0]) * float(second_size[1]))
    if first_area <= 0.0 or second_area <= 0.0:
        return 0.0
    center_dx = float(first["center"][0]) - float(second["center"][0])
    center_dy = float(first["center"][1]) - float(second["center"][1])
    first_radius = 0.5 * math.hypot(
        float(first_size[0]), float(first_size[1]))
    second_radius = 0.5 * math.hypot(
        float(second_size[0]), float(second_size[1]))
    if center_dx * center_dx + center_dy * center_dy > (
        first_radius + second_radius
    ) ** 2:
        return 0.0
    first_polygon = _rectangle_corners(
        first["center"], first_size, first.get("yaw", 0.0))
    second_polygon = _rectangle_corners(
        second["center"], second_size, second.get("yaw", 0.0))
    intersection = _polygon_area(
        _polygon_intersection(first_polygon, second_polygon))
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def class_aware_rotated_bev_nms(detections, iou_threshold):
    """Keep the highest-scoring box from same-class rotated-IoU clusters.

    The returned detections preserve their original input order.  This keeps
    marker IDs and downstream replay deterministic while using confidence only
    to decide which member of a duplicate cluster survives.
    """
    threshold = float(iou_threshold)
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("rotated BEV NMS IoU threshold must be in [0, 1]")

    ranked_indices = sorted(
        range(len(detections)),
        key=lambda index: (-float(detections[index].get("score", 0.0)), index),
    )
    kept_indices = []
    for candidate_index in ranked_indices:
        candidate = detections[candidate_index]
        candidate_class = normalize_class_name(candidate.get("class_name"))
        suppressed = False
        for kept_index in kept_indices:
            kept = detections[kept_index]
            if normalize_class_name(kept.get("class_name")) != candidate_class:
                continue
            if rotated_bev_iou(candidate, kept) > threshold:
                suppressed = True
                break
        if not suppressed:
            kept_indices.append(candidate_index)
    return [detections[index] for index in sorted(kept_indices)]
