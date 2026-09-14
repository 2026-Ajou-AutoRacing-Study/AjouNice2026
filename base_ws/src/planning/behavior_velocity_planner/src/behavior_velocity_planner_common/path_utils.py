import math
from copy import deepcopy

from autoware_planning_msgs.msg import Path, PathPoint
from geometry_msgs.msg import Pose, Quaternion

from .geometry import cumulative_lengths, project_point_to_polyline


def path_positions(path):
    return [
        (
            item.pose.position.x,
            item.pose.position.y,
            item.pose.position.z,
        )
        for item in path.points
    ]


def path_with_lane_id_positions(path):
    return [
        (
            item.point.pose.position.x,
            item.point.pose.position.y,
            item.point.pose.position.z,
        )
        for item in path.points
    ]


def to_path(path_with_lane_id):
    output = Path()
    output.header = deepcopy(path_with_lane_id.header)
    output.points = [deepcopy(item.point) for item in path_with_lane_id.points]
    output.left_bound = deepcopy(path_with_lane_id.left_bound)
    output.right_bound = deepcopy(path_with_lane_id.right_bound)
    return output


def quaternion_from_yaw(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(float(yaw) * 0.5)
    quaternion.w = math.cos(float(yaw) * 0.5)
    return quaternion


def interpolate_angle(first, second, ratio):
    delta = math.atan2(math.sin(second - first), math.cos(second - first))
    return first + delta * ratio


def yaw_from_quaternion(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose_at_station(path, station):
    if not path.points:
        return Pose()
    if len(path.points) == 1:
        return deepcopy(path.points[0].pose)

    positions = path_positions(path)
    lengths = cumulative_lengths(positions)
    station = max(0.0, min(float(station), lengths[-1]))
    for index in range(len(path.points) - 1):
        if station > lengths[index + 1]:
            continue
        segment_length = max(lengths[index + 1] - lengths[index], 1.0e-9)
        ratio = (station - lengths[index]) / segment_length
        first = path.points[index].pose
        second = path.points[index + 1].pose
        pose = Pose()
        pose.position.x = first.position.x + (second.position.x - first.position.x) * ratio
        pose.position.y = first.position.y + (second.position.y - first.position.y) * ratio
        pose.position.z = first.position.z + (second.position.z - first.position.z) * ratio
        if segment_length > 1.0e-6:
            yaw = math.atan2(
                second.position.y - first.position.y,
                second.position.x - first.position.x,
            )
        else:
            yaw = interpolate_angle(
                yaw_from_quaternion(first.orientation),
                yaw_from_quaternion(second.orientation),
                ratio,
            )
        pose.orientation = quaternion_from_yaw(yaw)
        return pose
    return deepcopy(path.points[-1].pose)


def insert_path_point(path, station, tolerance=1.0e-3):
    """Insert an interpolated point and return its index and pose."""
    if not path.points:
        return None, Pose()
    if len(path.points) == 1:
        return 0, deepcopy(path.points[0].pose)

    positions = path_positions(path)
    lengths = cumulative_lengths(positions)
    station = max(0.0, min(float(station), lengths[-1]))
    for index, current_station in enumerate(lengths):
        if abs(current_station - station) <= tolerance:
            return index, deepcopy(path.points[index].pose)

    for index in range(len(path.points) - 1):
        if station >= lengths[index + 1]:
            continue
        segment_length = max(lengths[index + 1] - lengths[index], 1.0e-9)
        ratio = (station - lengths[index]) / segment_length
        inserted = PathPoint()
        inserted.pose = pose_at_station(path, station)
        first = path.points[index]
        second = path.points[index + 1]
        inserted.longitudinal_velocity_mps = (
            first.longitudinal_velocity_mps
            + (second.longitudinal_velocity_mps - first.longitudinal_velocity_mps) * ratio
        )
        inserted.lateral_velocity_mps = (
            first.lateral_velocity_mps
            + (second.lateral_velocity_mps - first.lateral_velocity_mps) * ratio
        )
        inserted.heading_rate_rps = (
            first.heading_rate_rps
            + (second.heading_rate_rps - first.heading_rate_rps) * ratio
        )
        inserted.is_final = False
        path.points.insert(index + 1, inserted)
        return index + 1, deepcopy(inserted.pose)
    return len(path.points) - 1, deepcopy(path.points[-1].pose)


def set_velocity_from_index(path, index, velocity):
    if index is None:
        return
    velocity = max(float(velocity), 0.0)
    for point in path.points[max(int(index), 0) :]:
        point.longitudinal_velocity_mps = min(
            float(point.longitudinal_velocity_mps), velocity
        )


def set_velocity_between_indices(path, start_index, end_index, velocity):
    if start_index is None or end_index is None:
        return
    velocity = max(float(velocity), 0.0)
    first = max(min(int(start_index), int(end_index)), 0)
    last = min(max(int(start_index), int(end_index)), len(path.points) - 1)
    for point in path.points[first : last + 1]:
        point.longitudinal_velocity_mps = min(
            float(point.longitudinal_velocity_mps), velocity
        )


def project_pose_to_path(path, pose):
    projection = project_point_to_polyline(
        path_positions(path),
        (pose.position.x, pose.position.y, pose.position.z),
    )
    return projection


def ordered_lane_ids_from_path(path_with_lane_id):
    """경로(path_with_lane_id)를 따라 처음 등장하는 순서대로 lane_id 목록을 만듦.
    D-1처럼 lanelet 자체가 직/좌/우로 갈라지는 구조에서, "경로가 실제로 다음에
    어느 lanelet으로 이어지는지"를 판단하는 데 사용됨(신호등/blind_spot 모듈 공용)."""
    ordered = []
    for path_point in path_with_lane_id.points:
        for lane_id in path_point.lane_ids:
            lane_id = int(lane_id)
            if lane_id not in ordered:
                ordered.append(lane_id)
    return ordered


def downstream_turn_direction(lanelet_map, lane_id, ordered_lane_ids):
    """경로(route)상 이 lane_id 바로 다음에 오는 lanelet의 turn_direction을
    반환. lane_id 자체의 turn_direction 태그가 비어있거나(직/좌/우가 섞인 lanelet),
    경로가 실제로 어느 방향으로 이어지는지 알아야 할 때 사용. 정보가 없으면 None.
    (2026-09-14, D-1 신호등/blind_spot 모듈 공통 문제 해결을 위해 도입)"""
    lane_id = int(lane_id)
    if lane_id not in ordered_lane_ids:
        return None
    idx = ordered_lane_ids.index(lane_id)
    if idx + 1 >= len(ordered_lane_ids):
        return None
    next_lanelet = lanelet_map.road_lanelet_by_id(ordered_lane_ids[idx + 1])
    return next_lanelet.turn_direction if next_lanelet else None


def ego_speed(odometry):
    if odometry is None:
        return 0.0
    linear = odometry.twist.twist.linear
    return math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
