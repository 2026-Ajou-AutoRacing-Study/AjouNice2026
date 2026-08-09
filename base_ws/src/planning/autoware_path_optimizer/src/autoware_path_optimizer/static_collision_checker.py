#!/usr/bin/env python3

import math
from dataclasses import dataclass

from .trajectory_utils import distance_2d, project_point


@dataclass
class StaticObstacle:
    object_id: str
    polygon: list
    center_x: float
    center_y: float
    bounding_radius: float


@dataclass
class CollisionHit:
    object_id: str
    station: float


def _uuid_string(value):
    values = getattr(value, "uuid", value)
    return "".join("{:02x}".format(int(item)) for item in values)


def _yaw(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _rectangle(center_x, center_y, yaw, front, rear, half_width):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            center_x + cosine * longitudinal - sine * lateral,
            center_y + sine * longitudinal + cosine * lateral,
        )
        for longitudinal, lateral in (
            (front, half_width),
            (front, -half_width),
            (-rear, -half_width),
            (-rear, half_width),
        )
    ]


def _orientation(first, second, third):
    value = (second[1] - first[1]) * (third[0] - second[0]) - (
        second[0] - first[0]
    ) * (third[1] - second[1])
    if abs(value) < 1.0e-9:
        return 0
    return 1 if value > 0.0 else 2


def _on_segment(first, point, second):
    return (
        min(first[0], second[0]) - 1.0e-9
        <= point[0]
        <= max(first[0], second[0]) + 1.0e-9
        and min(first[1], second[1]) - 1.0e-9
        <= point[1]
        <= max(first[1], second[1]) + 1.0e-9
    )


def _segments_intersect(first_a, first_b, second_a, second_b):
    values = (
        _orientation(first_a, first_b, second_a),
        _orientation(first_a, first_b, second_b),
        _orientation(second_a, second_b, first_a),
        _orientation(second_a, second_b, first_b),
    )
    if values[0] != values[1] and values[2] != values[3]:
        return True
    return (
        (values[0] == 0 and _on_segment(first_a, second_a, first_b))
        or (values[1] == 0 and _on_segment(first_a, second_b, first_b))
        or (values[2] == 0 and _on_segment(second_a, first_a, second_b))
        or (values[3] == 0 and _on_segment(second_a, first_b, second_b))
    )


def _point_in_polygon(point, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1] + 1.0e-12)
                + current[0]
            )
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def polygons_intersect(first, second):
    if not first or not second:
        return False
    if _point_in_polygon(first[0], second) or _point_in_polygon(second[0], first):
        return True
    return any(
        _segments_intersect(first_a, first_b, second_a, second_b)
        for first_a, first_b in zip(first, first[1:] + first[:1])
        for second_a, second_b in zip(second, second[1:] + second[:1])
    )


def static_obstacles_from_message(message, speed_threshold=1.0, margin=0.3):
    obstacles = []
    if message is None:
        return obstacles
    for item in message.objects:
        twist = item.kinematics.initial_twist_with_covariance.twist.linear
        if math.sqrt(twist.x * twist.x + twist.y * twist.y + twist.z * twist.z) > speed_threshold:
            continue
        pose = item.kinematics.initial_pose_with_covariance.pose
        length = max(float(item.shape.dimensions.x), 0.05)
        width = max(float(item.shape.dimensions.y), 0.05)
        footprint_message = getattr(item.shape, "footprint", None)
        footprint = list(getattr(footprint_message, "points", []))
        if footprint:
            length = max(length, 2.0 * max(abs(float(point.x)) for point in footprint))
            width = max(width, 2.0 * max(abs(float(point.y)) for point in footprint))
        half_length = 0.5 * length + margin
        half_width = 0.5 * width + margin
        polygon = _rectangle(
            float(pose.position.x),
            float(pose.position.y),
            _yaw(pose.orientation),
            half_length,
            half_length,
            half_width,
        )
        obstacles.append(
            StaticObstacle(
                object_id=_uuid_string(item.object_id),
                polygon=polygon,
                center_x=float(pose.position.x),
                center_y=float(pose.position.y),
                bounding_radius=math.hypot(half_length, half_width),
            )
        )
    return obstacles


def first_static_collision(
    points,
    obstacles,
    ego_position,
    vehicle_width=1.892,
    base_link_to_front=3.845,
    base_link_to_rear=0.790,
    sampling_interval=0.25,
    backward_distance=1.0,
    forward_distance=60.0,
    ignore_initial_overlap=False,
    maximum_initial_overlap_distance=0.0,
):
    if len(points) < 2 or not obstacles:
        return None
    _, ego_station, _, _ = project_point(
        points, float(ego_position[0]), float(ego_position[1])
    )
    minimum_station = ego_station - max(float(backward_distance), 0.0)
    maximum_station = ego_station + max(float(forward_distance), 0.0)
    half_width = 0.5 * float(vehicle_width)
    sample_interval = max(float(sampling_interval), 0.05)
    vehicle_radius = math.hypot(
        max(float(base_link_to_front), float(base_link_to_rear)),
        half_width,
    )
    first_forward_sample_seen = False
    initial_overlap_hit = None
    initial_overlap_active = False
    maximum_escape_distance = max(
        float(maximum_initial_overlap_distance),
        0.0,
    )
    station = 0.0
    for segment_index, (first, second) in enumerate(zip(points[:-1], points[1:])):
        segment_length = distance_2d(first, second)
        if segment_length < 1.0e-6:
            continue
        yaw = math.atan2(second.y - first.y, second.x - first.x)
        sample_count = max(int(math.ceil(segment_length / sample_interval)), 1)
        for sample_index in range(sample_count + (1 if segment_index == len(points) - 2 else 0)):
            ratio = min(float(sample_index) / float(sample_count), 1.0)
            sample_station = station + ratio * segment_length
            if sample_station < minimum_station:
                continue
            if sample_station > maximum_station:
                return initial_overlap_hit if initial_overlap_active else None
            x = first.x + ratio * (second.x - first.x)
            y = first.y + ratio * (second.y - first.y)
            vehicle_polygon = _rectangle(
                x,
                y,
                yaw,
                float(base_link_to_front),
                float(base_link_to_rear),
                half_width,
            )
            sample_hit = None
            for obstacle in obstacles:
                if math.hypot(x - obstacle.center_x, y - obstacle.center_y) > (
                    vehicle_radius + obstacle.bounding_radius
                ):
                    continue
                if polygons_intersect(vehicle_polygon, obstacle.polygon):
                    sample_hit = CollisionHit(obstacle.object_id, sample_station)
                    break

            if not first_forward_sample_seen:
                first_forward_sample_seen = True
                if sample_hit is None:
                    continue
                if not ignore_initial_overlap:
                    return sample_hit
                initial_overlap_hit = sample_hit
                initial_overlap_active = True
                continue

            if initial_overlap_active:
                escape_distance = sample_station - initial_overlap_hit.station
                if sample_hit is not None:
                    if escape_distance > maximum_escape_distance + 1.0e-6:
                        return initial_overlap_hit
                    continue
                if escape_distance > maximum_escape_distance + 1.0e-6:
                    return initial_overlap_hit
                initial_overlap_active = False
                continue

            if sample_hit is not None:
                return sample_hit
        station += segment_length
    return initial_overlap_hit if initial_overlap_active else None
