#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass

from autoware_internal_planning_msgs.msg import PathPointWithLaneId
from autoware_planning_msgs.msg import PathPoint
from geometry_msgs.msg import Pose

from bpp_map_tools import (
    closest_lane_id,
    cumulative_lengths,
    interpolate_polyline,
    project_point_to_polyline,
    resample_polyline,
    trim_polyline,
)
from bpp_path_tools import clamp, project_pose_to_path, quaternion_from_yaw, smoothstep


@dataclass
class RouteLaneChangeTarget:
    source_lane_id: int
    target_lane_id: int
    lateral_offset: float

    @property
    def direction(self):
        return "left" if self.lateral_offset > 0.0 else "right"


def route_lane_change_target(route, centerline_map, ego_pose):
    if route is None or centerline_map is None or ego_pose is None:
        return None

    ego = (
        float(ego_pose.position.x),
        float(ego_pose.position.y),
        float(ego_pose.position.z),
    )
    for segment in route.segments:
        primitive_ids = [
            int(primitive.id)
            for primitive in segment.primitives
            if int(primitive.id) != 0 and centerline_map.has_lane(primitive.id)
        ]
        target_lane_id = int(segment.preferred_primitive.id)
        if target_lane_id and target_lane_id not in primitive_ids:
            primitive_ids.append(target_lane_id)
        if len(primitive_ids) < 2 or not centerline_map.has_lane(target_lane_id):
            continue

        source_lane_id = closest_lane_id(centerline_map, primitive_ids, ego)
        if source_lane_id is None or source_lane_id == target_lane_id:
            continue

        source = centerline_map.get_centerline(source_lane_id)
        target = centerline_map.get_centerline(target_lane_id)
        source_projection = project_point_to_polyline(source, ego)
        if source_projection is None:
            continue
        source_s = source_projection[0]
        source_point = interpolate_polyline(source, source_s)
        target_projection = project_point_to_polyline(target, source_point)
        if target_projection is None:
            continue
        target_point = interpolate_polyline(target, target_projection[0])
        before = interpolate_polyline(source, max(0.0, source_s - 0.5))
        after = interpolate_polyline(source, source_s + 0.5)
        tangent_x = after[0] - before[0]
        tangent_y = after[1] - before[1]
        tangent_length = max(math.hypot(tangent_x, tangent_y), 1.0e-6)
        lateral_offset = (
            tangent_x * (target_point[1] - source_point[1])
            - tangent_y * (target_point[0] - source_point[0])
        ) / tangent_length
        if abs(lateral_offset) < 0.1:
            continue
        return RouteLaneChangeTarget(
            source_lane_id=source_lane_id,
            target_lane_id=target_lane_id,
            lateral_offset=lateral_offset,
        )
    return None


def _path_polyline(path):
    return [
        (
            float(item.point.pose.position.x),
            float(item.point.pose.position.y),
            float(item.point.pose.position.z),
        )
        for item in path.points
    ]


def _velocity_at(path, distances, distance):
    if not path.points:
        return 0.0
    index = min(range(len(distances)), key=lambda item: abs(distances[item] - distance))
    return float(path.points[index].point.longitudinal_velocity_mps)


def _sample_distances(start, end, interval):
    if end <= start:
        return [start]
    result = []
    distance = start
    while distance < end - 1.0e-6:
        result.append(distance)
        distance += max(interval, 0.1)
    result.append(end)
    return result


def build_route_lane_change_path(
    path,
    centerline_map,
    ego_pose,
    target,
    prepare_distance=3.0,
    lane_change_distance=28.0,
    target_forward_length=300.0,
    transition_interval=1.0,
    target_interval=4.0,
    prepare_velocity_ratio=2.0 / 3.0,
):
    if not path.points or target is None:
        return deepcopy(path)

    source_polyline = _path_polyline(path)
    source_distances = cumulative_lengths(source_polyline)
    ego_projection = project_pose_to_path(path, ego_pose)
    ego_s = ego_projection.longitudinal_distance if ego_projection is not None else 0.0
    start_s = clamp(
        ego_s + max(float(prepare_distance), 0.0),
        0.0,
        source_distances[-1],
    )
    finish_s = clamp(
        start_s + max(float(lane_change_distance), 0.1),
        start_s,
        source_distances[-1],
    )

    target_centerline = centerline_map.get_centerline(target.target_lane_id)
    generated = []
    generated_lane_ids = []
    generated_velocities = []
    for distance in _sample_distances(0.0, finish_s, transition_interval):
        source_point = interpolate_polyline(source_polyline, distance)
        target_projection = project_point_to_polyline(target_centerline, source_point)
        target_point = interpolate_polyline(target_centerline, target_projection[0])
        ratio = smoothstep(start_s, finish_s, distance)
        point = (
            source_point[0] + ratio * (target_point[0] - source_point[0]),
            source_point[1] + ratio * (target_point[1] - source_point[1]),
            target_point[2] if distance > start_s else source_point[2],
        )
        generated.append(point)
        if distance <= start_s:
            generated_lane_ids.append([target.source_lane_id])
        elif distance < finish_s:
            generated_lane_ids.append([target.source_lane_id, target.target_lane_id])
        else:
            generated_lane_ids.append([target.target_lane_id])

        base_velocity = _velocity_at(path, source_distances, distance)
        if distance <= start_s:
            base_velocity *= max(float(prepare_velocity_ratio), 0.0)
        generated_velocities.append(base_velocity)

    target_start = generated[-1]
    target_tail = trim_polyline(
        target_centerline,
        start_point=target_start,
        max_forward_length=max(float(target_forward_length), 0.0),
    )
    target_tail = resample_polyline(target_tail, target_interval)
    target_velocity = _velocity_at(path, source_distances, finish_s)
    for point in target_tail[1:]:
        generated.append(point)
        generated_lane_ids.append([target.target_lane_id])
        generated_velocities.append(target_velocity)

    output = deepcopy(path)
    output.points = []
    for index, point in enumerate(generated):
        neighbor = generated[index + 1] if index + 1 < len(generated) else generated[index - 1]
        if index + 1 < len(generated):
            yaw = math.atan2(neighbor[1] - point[1], neighbor[0] - point[0])
        else:
            yaw = math.atan2(point[1] - neighbor[1], point[0] - neighbor[0])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point
        pose.orientation = quaternion_from_yaw(yaw)

        path_point = PathPoint()
        path_point.pose = pose
        path_point.longitudinal_velocity_mps = generated_velocities[index]
        path_point.lateral_velocity_mps = 0.0
        path_point.heading_rate_rps = 0.0
        path_point.is_final = False

        item = PathPointWithLaneId()
        item.point = path_point
        item.lane_ids = generated_lane_ids[index]
        output.points.append(item)
    return output
