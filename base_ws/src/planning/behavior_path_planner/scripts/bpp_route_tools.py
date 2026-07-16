#!/usr/bin/env python3

import math
from dataclasses import dataclass

from bpp_map_tools import closest_lane_id, distance_2d


@dataclass
class RouteSectionView:
    preferred_lane_id: int
    lane_ids: list


def _pose_yaw(pose):
    orientation = getattr(pose, "orientation", None)
    if orientation is None:
        return None
    norm = math.sqrt(
        float(orientation.x) ** 2
        + float(orientation.y) ** 2
        + float(orientation.z) ** 2
        + float(orientation.w) ** 2
    )
    if norm < 0.5:
        return None
    return math.atan2(
        2.0
        * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0
        - 2.0
        * (
            float(orientation.y) ** 2
            + float(orientation.z) ** 2
        ),
    )


def _lane_pose_match(centerline, point, pose_yaw):
    best = None
    for first, second in zip(centerline[:-1], centerline[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_sq = dx * dx + dy * dy
        if length_sq < 1.0e-9:
            continue
        ratio = max(
            0.0,
            min(
                1.0,
                ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy)
                / length_sq,
            ),
        )
        projected = (first[0] + ratio * dx, first[1] + ratio * dy)
        lane_yaw = math.atan2(dy, dx)
        heading_error = (
            abs(math.atan2(math.sin(lane_yaw - pose_yaw), math.cos(lane_yaw - pose_yaw)))
            if pose_yaw is not None
            else 0.0
        )
        score = (distance_2d(point, projected), heading_error)
        if best is None or score < best:
            best = score
    return best


def _is_near_route_start(route, ego, threshold=5.0):
    start_pose = getattr(route, "start_pose", None)
    if start_pose is None or _pose_yaw(start_pose) is None:
        return False
    start = (
        float(start_pose.position.x),
        float(start_pose.position.y),
        float(start_pose.position.z),
    )
    return distance_2d(ego, start) <= threshold


def route_sections(route, centerline_map):
    sections = []
    if route is None:
        return sections

    for segment in route.segments:
        lane_ids = []
        for primitive in segment.primitives:
            lane_id = int(primitive.id)
            if lane_id and centerline_map.has_lane(lane_id) and lane_id not in lane_ids:
                lane_ids.append(lane_id)

        preferred_lane_id = int(segment.preferred_primitive.id)
        if (
            preferred_lane_id
            and centerline_map.has_lane(preferred_lane_id)
            and preferred_lane_id not in lane_ids
        ):
            lane_ids.append(preferred_lane_id)
        if lane_ids:
            sections.append(RouteSectionView(preferred_lane_id, lane_ids))
    return sections


def current_route_lane(route, centerline_map, ego_pose):
    sections = route_sections(route, centerline_map)
    if not sections or ego_pose is None:
        return None, None

    ego = (
        float(ego_pose.position.x),
        float(ego_pose.position.y),
        float(ego_pose.position.z),
    )
    # A looped route can pass close to its start again. At route initialization,
    # constrain the match to the first section so that a later parallel lane is
    # not selected just because its centerline is a few centimeters closer.
    if _is_near_route_start(route, ego):
        lane_id = closest_lane_id(centerline_map, sections[0].lane_ids, ego)
        return (0, lane_id) if lane_id is not None else (None, None)

    pose_yaw = _pose_yaw(ego_pose)
    candidates = []
    for section_index, section in enumerate(sections):
        for lane_id in section.lane_ids:
            match = _lane_pose_match(
                centerline_map.get_centerline(lane_id), ego, pose_yaw
            )
            if match is not None:
                candidates.append((match[0], match[1], section_index, lane_id))
    if not candidates:
        return None, None

    if pose_yaw is None:
        _, _, section_index, lane_id = min(candidates)
        return section_index, lane_id

    aligned = [item for item in candidates if item[1] <= math.pi * 0.5]
    if aligned:
        candidates = aligned
    minimum_distance = min(item[0] for item in candidates)
    nearby = [item for item in candidates if item[0] <= minimum_distance + 1.0]
    _, _, section_index, lane_id = min(
        nearby, key=lambda item: (item[1], item[2], item[0], item[3])
    )
    return section_index, lane_id


def following_lane(centerline_map, current_lane_id, section):
    candidates = [
        lane_id
        for lane_id in section.lane_ids
        if centerline_map.is_following(current_lane_id, lane_id)
    ]
    if not candidates:
        return None
    if section.preferred_lane_id in candidates:
        return section.preferred_lane_id

    current = centerline_map.get_centerline(current_lane_id)
    return min(
        candidates,
        key=lambda lane_id: distance_2d(
            current[-1], centerline_map.get_centerline(lane_id)[0]
        ),
    )


def forward_lane_sequence(route, centerline_map, section_index, start_lane_id):
    sections = route_sections(route, centerline_map)
    if (
        section_index is None
        or start_lane_id is None
        or section_index < 0
        or section_index >= len(sections)
        or start_lane_id not in sections[section_index].lane_ids
    ):
        return []

    lane_ids = [int(start_lane_id)]
    current_lane_id = int(start_lane_id)
    for section in sections[section_index + 1 :]:
        next_lane_id = following_lane(centerline_map, current_lane_id, section)
        if next_lane_id is None:
            break
        lane_ids.append(next_lane_id)
        current_lane_id = next_lane_id
    return lane_ids


def reference_lane_sequence(route, centerline_map, ego_pose):
    section_index, lane_id = current_route_lane(route, centerline_map, ego_pose)
    return forward_lane_sequence(
        route, centerline_map, section_index, lane_id
    )
