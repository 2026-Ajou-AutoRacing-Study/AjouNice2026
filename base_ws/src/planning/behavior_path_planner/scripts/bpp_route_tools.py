#!/usr/bin/env python3

import math
from dataclasses import dataclass

from bpp_map_tools import cumulative_lengths, distance_2d


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


def _point_to_segment_distance(point, first, second):
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length_sq = dx * dx + dy * dy
    if length_sq < 1.0e-9:
        return distance_2d(point, first)
    ratio = max(
        0.0,
        min(
            1.0,
            ((point[0] - first[0]) * dx + (point[1] - first[1]) * dy)
            / length_sq,
        ),
    )
    projected = (first[0] + ratio * dx, first[1] + ratio * dy)
    return distance_2d(point, projected)


def _point_in_polygon(point, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _lane_polygon_distance(centerline_map, lane_id, point, centerline_distance):
    boundaries = centerline_map.get_boundaries(lane_id)
    if boundaries is None:
        return centerline_distance
    polygon = list(boundaries[0]) + list(reversed(boundaries[1]))
    if len(polygon) < 3:
        return centerline_distance
    if _point_in_polygon(point, polygon):
        return 0.0
    return min(
        _point_to_segment_distance(point, first, second)
        for first, second in zip(polygon, polygon[1:] + polygon[:1])
    )


def _lane_pose_match(centerline_map, lane_id, point, pose_yaw):
    centerline = centerline_map.get_centerline(lane_id)
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
    if best is None:
        return None
    return (
        _lane_polygon_distance(centerline_map, lane_id, point, best[0]),
        best[1],
        best[0],
    )


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


def route_lane_position(route, centerline_map, lane_id):
    sections = route_sections(route, centerline_map)
    preferred_match = None
    any_match = None
    for index, section in enumerate(sections):
        if int(lane_id) not in section.lane_ids:
            continue
        any_match = index
        if section.preferred_lane_id == int(lane_id):
            preferred_match = index
    index = preferred_match if preferred_match is not None else any_match
    return (index, int(lane_id)) if index is not None else (None, None)


def _closest_lane_match(
    sections,
    centerline_map,
    ego_pose,
    lane_positions=None,
    dist_threshold=None,
    yaw_threshold=None,
):
    if not sections or ego_pose is None:
        return None, None

    ego = (
        float(ego_pose.position.x),
        float(ego_pose.position.y),
        float(ego_pose.position.z),
    )
    pose_yaw = _pose_yaw(ego_pose)
    candidates = []
    for section_index, section in enumerate(sections):
        for lane_id in section.lane_ids:
            position = (section_index, lane_id)
            if lane_positions is not None and position not in lane_positions:
                continue
            match = _lane_pose_match(centerline_map, lane_id, ego, pose_yaw)
            if match is not None:
                if dist_threshold is not None and match[0] > float(dist_threshold):
                    continue
                if (
                    pose_yaw is not None
                    and yaw_threshold is not None
                    and match[1] > abs(float(yaw_threshold))
                ):
                    continue
                candidates.append(
                    (match[0], match[1], match[2], section_index, lane_id)
                )
    if not candidates:
        return None, None
    _, _, _, section_index, lane_id = min(
        candidates, key=lambda item: (item[0], item[1], item[2], item[3], item[4])
    )
    return section_index, lane_id


def current_route_lane(route, centerline_map, ego_pose):
    """Equivalent of RouteHandler::getClosestLaneletWithinRoute for ROS 1."""
    return _closest_lane_match(route_sections(route, centerline_map), centerline_map, ego_pose)


def previous_lanes(centerline_map, sections, section_index, lane_id):
    if section_index is None or section_index <= 0:
        return []
    return [
        candidate
        for candidate in sections[section_index - 1].lane_ids
        if centerline_map.is_following(candidate, lane_id)
    ]


def next_lanes(centerline_map, sections, section_index, lane_id):
    if section_index is None or section_index + 1 >= len(sections):
        return []
    return [
        candidate
        for candidate in sections[section_index + 1].lane_ids
        if centerline_map.is_following(lane_id, candidate)
    ]


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


def _lane_length(centerline_map, lane_id):
    distances = cumulative_lengths(centerline_map.get_centerline(lane_id) or [])
    return distances[-1] if distances else 0.0


def lane_sequence_around(
    route,
    centerline_map,
    section_index,
    lane_id,
    backward_length,
    forward_length,
):
    sections = route_sections(route, centerline_map)
    if (
        section_index is None
        or lane_id is None
        or section_index < 0
        or section_index >= len(sections)
        or lane_id not in sections[section_index].lane_ids
    ):
        return []

    backward = []
    current_index = int(section_index)
    current_lane_id = int(lane_id)
    accumulated = 0.0
    while current_index > 0 and accumulated < max(float(backward_length), 0.0):
        candidates = previous_lanes(
            centerline_map, sections, current_index, current_lane_id
        )
        if not candidates:
            break
        previous_section = sections[current_index - 1]
        previous_lane_id = (
            previous_section.preferred_lane_id
            if previous_section.preferred_lane_id in candidates
            else min(
                candidates,
                key=lambda candidate: distance_2d(
                    centerline_map.get_centerline(candidate)[-1],
                    centerline_map.get_centerline(current_lane_id)[0],
                ),
            )
        )
        if previous_lane_id in backward:
            break
        backward.append(previous_lane_id)
        accumulated += _lane_length(centerline_map, previous_lane_id)
        current_index -= 1
        current_lane_id = previous_lane_id

    forward = [int(lane_id)]
    current_index = int(section_index)
    current_lane_id = int(lane_id)
    accumulated = _lane_length(centerline_map, current_lane_id)
    while current_index + 1 < len(sections) and accumulated < max(
        float(forward_length), 0.0
    ):
        candidate = following_lane(
            centerline_map, current_lane_id, sections[current_index + 1]
        )
        if candidate is None or candidate in forward or candidate in backward:
            break
        forward.append(candidate)
        accumulated += _lane_length(centerline_map, candidate)
        current_index += 1
        current_lane_id = candidate
    return list(reversed(backward)) + forward


def reference_lane_sequence(route, centerline_map, ego_pose):
    section_index, lane_id = current_route_lane(route, centerline_map, ego_pose)
    return forward_lane_sequence(route, centerline_map, section_index, lane_id)


class RouteLaneletTracker:
    """ROS 1 counterpart of Autoware BPP's current_route_lanelet_ tracking."""

    def __init__(self, dist_threshold=3.0, yaw_threshold=1.046):
        self.dist_threshold = max(float(dist_threshold), 0.0)
        self.yaw_threshold = max(float(yaw_threshold), 0.0)
        self.section_index = None
        self.lane_id = None

    def clear(self):
        self.section_index = None
        self.lane_id = None

    def reset(self, route, centerline_map, ego_pose):
        self.section_index, self.lane_id = current_route_lane(
            route, centerline_map, ego_pose
        )
        return self.section_index, self.lane_id

    def set_lane(self, route, centerline_map, lane_id):
        section_index, matched_lane_id = route_lane_position(
            route, centerline_map, lane_id
        )
        if section_index is None:
            return False
        self.section_index = section_index
        self.lane_id = matched_lane_id
        return True

    def lane_matches_pose(self, centerline_map, lane_id, ego_pose):
        if lane_id is None or ego_pose is None or not centerline_map.has_lane(lane_id):
            return False
        point = (
            float(ego_pose.position.x),
            float(ego_pose.position.y),
            float(ego_pose.position.z),
        )
        match = _lane_pose_match(
            centerline_map, lane_id, point, _pose_yaw(ego_pose)
        )
        return bool(
            match is not None
            and match[0] <= self.dist_threshold
            and match[1] <= self.yaw_threshold
        )

    def lane_change_completed(self, centerline_map, target_lane_id, ego_pose):
        if (
            target_lane_id is None
            or ego_pose is None
            or not centerline_map.has_lane(target_lane_id)
        ):
            return False
        point = (
            float(ego_pose.position.x),
            float(ego_pose.position.y),
            float(ego_pose.position.z),
        )
        match = _lane_pose_match(
            centerline_map, target_lane_id, point, _pose_yaw(ego_pose)
        )
        if match is None or match[1] > self.yaw_threshold:
            return False
        if centerline_map.get_boundaries(target_lane_id) is not None:
            return match[0] <= 1.0e-3
        return match[2] <= min(self.dist_threshold, 1.0)

    def update(
        self,
        route,
        centerline_map,
        ego_pose,
        backward_length,
        forward_length,
        autonomous_control=False,
        module_running=False,
    ):
        sections = route_sections(route, centerline_map)
        if not sections or ego_pose is None:
            self.clear()
            return self.section_index, self.lane_id
        if (
            self.section_index is None
            or self.lane_id is None
            or self.section_index >= len(sections)
            or self.lane_id not in sections[self.section_index].lane_ids
        ):
            return self.reset(route, centerline_map, ego_pose)

        local_positions = {(self.section_index, self.lane_id)}
        for candidate in previous_lanes(
            centerline_map, sections, self.section_index, self.lane_id
        ):
            local_positions.add((self.section_index - 1, candidate))
        for candidate in next_lanes(
            centerline_map, sections, self.section_index, self.lane_id
        ):
            local_positions.add((self.section_index + 1, candidate))

        section_index, lane_id = _closest_lane_match(
            sections,
            centerline_map,
            ego_pose,
            lane_positions=local_positions,
            dist_threshold=self.dist_threshold,
            yaw_threshold=self.yaw_threshold,
        )
        if section_index is not None:
            self.section_index, self.lane_id = section_index, lane_id
        else:
            sequence = lane_sequence_around(
                route,
                centerline_map,
                self.section_index,
                self.lane_id,
                max(float(backward_length), 0.0) + 10.0,
                forward_length,
            )
            positions = {
                route_lane_position(route, centerline_map, candidate)
                for candidate in sequence
            }
            positions.discard((None, None))
            section_index, lane_id = _closest_lane_match(
                sections,
                centerline_map,
                ego_pose,
                lane_positions=positions,
                dist_threshold=self.dist_threshold,
                yaw_threshold=self.yaw_threshold,
            )
            if section_index is None:
                section_index, lane_id = _closest_lane_match(
                    sections,
                    centerline_map,
                    ego_pose,
                    lane_positions=positions,
                )
            if section_index is not None:
                self.section_index, self.lane_id = section_index, lane_id
            elif not module_running:
                self.reset(route, centerline_map, ego_pose)

        # Autoware suppresses global route snapping while autonomous control is active.
        if not autonomous_control:
            constrained_index, constrained_lane = _closest_lane_match(
                sections,
                centerline_map,
                ego_pose,
                dist_threshold=self.dist_threshold,
                yaw_threshold=self.yaw_threshold,
            )
            if constrained_index is None and not module_running:
                self.reset(route, centerline_map, ego_pose)
            elif constrained_index is not None and not self.lane_matches_pose(
                centerline_map, self.lane_id, ego_pose
            ):
                self.section_index, self.lane_id = (
                    constrained_index,
                    constrained_lane,
                )
        return self.section_index, self.lane_id

    def sequence(self, route, centerline_map, backward_length, forward_length):
        return lane_sequence_around(
            route,
            centerline_map,
            self.section_index,
            self.lane_id,
            backward_length,
            forward_length,
        )
