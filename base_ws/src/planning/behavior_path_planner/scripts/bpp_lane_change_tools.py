#!/usr/bin/env python3

import math
from copy import deepcopy
from dataclasses import dataclass

from autoware_internal_planning_msgs.msg import PathPointWithLaneId
from autoware_planning_msgs.msg import PathPoint
from geometry_msgs.msg import Pose

from bpp_map_tools import (
    cumulative_lengths,
    distance_2d,
    interpolate_polyline,
    project_point_to_polyline,
    resample_polyline,
    stitch_route_centerlines,
    trim_polyline,
)
from bpp_path_tools import clamp, project_pose_to_path, quaternion_from_yaw, smoothstep
from bpp_route_tools import current_route_lane, route_sections


@dataclass
class RouteLaneChangeTarget:
    source_lane_id: int
    target_lane_id: int
    lateral_offset: float
    section_index: int

    @property
    def direction(self):
        return "left" if self.lateral_offset > 0.0 else "right"


def _next_lateral_lane(centerline_map, section_lane_ids, source_lane_id, preferred_lane_id):
    if source_lane_id == preferred_lane_id:
        return None

    allowed = list(dict.fromkeys(int(lane_id) for lane_id in section_lane_ids))
    if source_lane_id not in allowed or preferred_lane_id not in allowed:
        return None

    queue = [(source_lane_id, [])]
    visited = {source_lane_id}
    while queue:
        current_lane_id, steps = queue.pop(0)
        for candidate_lane_id in allowed:
            if candidate_lane_id in visited or not centerline_map.is_lateral_neighbor(
                current_lane_id, candidate_lane_id
            ):
                continue
            next_steps = steps + [candidate_lane_id]
            if candidate_lane_id == preferred_lane_id:
                return next_steps[0]
            visited.add(candidate_lane_id)
            queue.append((candidate_lane_id, next_steps))
    return None


def route_lane_change_target(
    route,
    centerline_map,
    ego_pose,
    current_section_index=None,
    current_lane_id=None,
):
    if route is None or centerline_map is None or ego_pose is None:
        return None

    sections = route_sections(route, centerline_map)
    section_index, source_lane_id = current_section_index, current_lane_id
    if section_index is None or source_lane_id is None:
        section_index, source_lane_id = current_route_lane(
            route, centerline_map, ego_pose
        )
    if section_index is None or source_lane_id is None:
        return None
    if section_index < 0 or section_index >= len(sections):
        return None

    # A future route section may require a lane change, but it must not become
    # executable while ego is still driving an earlier preferred lane.  The
    # route tracker advances this index as ego reaches each section, so only
    # the current section is eligible here.
    section = sections[section_index]
    if source_lane_id not in section.lane_ids:
        return None

    preferred_lane_id = section.preferred_lane_id
    if not preferred_lane_id or source_lane_id == preferred_lane_id:
        return None
    target_lane_id = _next_lateral_lane(
        centerline_map,
        section.lane_ids,
        source_lane_id,
        preferred_lane_id,
    )
    if target_lane_id is None:
        return None

    source = centerline_map.get_centerline(source_lane_id)
    target = centerline_map.get_centerline(target_lane_id)
    ego = (
        float(ego_pose.position.x),
        float(ego_pose.position.y),
        float(ego_pose.position.z),
    )
    source_projection = project_point_to_polyline(source, ego)
    if source_projection is None:
        return None
    source_s = source_projection[0]
    source_point = interpolate_polyline(source, source_s)
    target_projection = project_point_to_polyline(target, source_point)
    if target_projection is None:
        return None
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
        return None
    return RouteLaneChangeTarget(
        source_lane_id=source_lane_id,
        target_lane_id=target_lane_id,
        lateral_offset=lateral_offset,
        section_index=section_index,
    )


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


def _lane_station_range(path, distances, lane_id):
    stations = [
        distances[index]
        for index, item in enumerate(path.points)
        if lane_id in item.lane_ids
    ]
    if not stations:
        return None
    return min(stations), max(stations)


def _candidate_geometry_is_valid(points, transition_interval, target_interval):
    if len(points) < 2:
        return False
    if any(not all(math.isfinite(value) for value in point[:2]) for point in points):
        return False

    maximum_spacing = 1.5 * max(
        float(transition_interval),
        float(target_interval),
        0.1,
    )
    headings = []
    for first, second in zip(points[:-1], points[1:]):
        delta_x = second[0] - first[0]
        delta_y = second[1] - first[1]
        spacing = math.hypot(delta_x, delta_y)
        if spacing < 1.0e-6 or spacing > maximum_spacing + 1.0e-6:
            return False
        headings.append(math.atan2(delta_y, delta_x))

    for first, second in zip(headings[:-1], headings[1:]):
        delta = abs(math.atan2(math.sin(second - first), math.cos(second - first)))
        if delta > math.radians(60.0):
            return False
    return True


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
    prepare_velocity_ratio=1.0,
    target_lane_ids=None,
    goal_point=None,
    stop_at_goal=True,
):
    if not path.points or target is None:
        return None

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

    target_lane_ids = list(target_lane_ids or [target.target_lane_id])
    if target.target_lane_id not in target_lane_ids:
        target_lane_ids.insert(0, target.target_lane_id)
    target_centerline, used_target_lane_ids = stitch_route_centerlines(
        centerline_map, target_lane_ids
    )
    if len(target_centerline) < 2:
        return None

    source_range = _lane_station_range(
        path,
        source_distances,
        target.source_lane_id,
    )
    if source_range is None:
        return None
    source_lane_start_s, source_lane_end_s = source_range
    local_tolerance = max(float(prepare_distance), float(transition_interval), 0.1)
    if (
        ego_s < source_lane_start_s - local_tolerance
        or ego_s > source_lane_end_s + local_tolerance
    ):
        return None
    if not centerline_map.is_lateral_neighbor(
        target.source_lane_id,
        target.target_lane_id,
    ):
        return None

    start_s = max(
        start_s,
        min(
            source_lane_start_s + max(float(prepare_distance), 0.0),
            source_distances[-1],
        ),
    )
    finish_s = clamp(
        start_s + max(float(lane_change_distance), 0.1),
        start_s,
        source_distances[-1],
    )

    generated = []
    generated_lane_ids = []
    generated_velocities = []
    maximum_target_distance = max(
        abs(float(target.lateral_offset)) * 1.5,
        abs(float(target.lateral_offset)) + 2.0,
        3.0,
    )
    for distance in _sample_distances(0.0, finish_s, transition_interval):
        source_point = interpolate_polyline(source_polyline, distance)
        target_projection = project_point_to_polyline(target_centerline, source_point)
        if target_projection is None:
            return None
        if distance >= start_s and target_projection[1] > maximum_target_distance:
            return None
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
    transition_point_count = len(generated)

    target_start = generated[-1]
    target_tail = trim_polyline(
        target_centerline,
        start_point=target_start,
        goal_point=goal_point,
        max_forward_length=max(float(target_forward_length), 0.0),
    )
    target_tail = resample_polyline(target_tail, target_interval)
    reaches_goal = goal_point is not None and distance_2d(
        target_tail[-1], goal_point
    ) <= max(float(target_interval), 0.1)
    if reaches_goal and distance_2d(target_tail[-1], goal_point) > 1.0e-3:
        target_tail.append(goal_point)
    target_velocity = _velocity_at(path, source_distances, finish_s)
    for point in target_tail[1:]:
        generated.append(point)
        generated_lane_ids.append(
            centerline_map.lane_ids_near_point(
                used_target_lane_ids or target_lane_ids, point
            )
        )
        generated_velocities.append(target_velocity)

    if not _candidate_geometry_is_valid(
        generated[: min(len(generated), transition_point_count + 2)],
        transition_interval,
        target_interval,
    ):
        return None

    if reaches_goal and stop_at_goal and generated_velocities:
        generated_velocities[-1] = 0.0

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
