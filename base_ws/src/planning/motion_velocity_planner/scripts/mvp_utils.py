#!/usr/bin/env python3

import math
from copy import deepcopy
from geometry_msgs.msg import Quaternion


def param_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def clamp(value, low, high):
    return max(low, min(high, value))


def distance_2d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def point_data(path_point):
    return path_point.point if hasattr(path_point, "point") else path_point


def point_pose(path_point):
    return point_data(path_point).pose


def point_position(path_point):
    return point_pose(path_point).position


def point_velocity(path_point):
    return float(getattr(point_data(path_point), "longitudinal_velocity_mps", 0.0))


def set_point_velocity(path_point, velocity_mps):
    point_data(path_point).longitudinal_velocity_mps = float(velocity_mps)


def path_cumulative_lengths(path):
    if not path.points:
        return []

    distances = [0.0]
    for prev, curr in zip(path.points[:-1], path.points[1:]):
        a = point_position(prev)
        b = point_position(curr)
        distances.append(distances[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return distances


def path_length(path):
    lengths = path_cumulative_lengths(path)
    return lengths[-1] if lengths else 0.0


class PathProjection:
    def __init__(
        self,
        index,
        longitudinal_distance,
        signed_lateral_distance,
        distance,
        segment_ratio=0.0,
        segment_yaw=0.0,
    ):
        self.index = index
        self.longitudinal_distance = longitudinal_distance
        self.signed_lateral_distance = signed_lateral_distance
        self.distance = distance
        self.segment_ratio = segment_ratio
        self.segment_yaw = segment_yaw


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def project_pose_to_path(path, pose, yaw_threshold=None):
    if not path.points:
        return None

    position = pose.position
    if len(path.points) == 1:
        path_pose = point_pose(path.points[0])
        yaw = yaw_from_quaternion(path_pose.orientation)
        dx = position.x - path_pose.position.x
        dy = position.y - path_pose.position.y
        signed_lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return PathProjection(0, 0.0, signed_lateral, math.hypot(dx, dy), 0.0, yaw)

    cumulative = path_cumulative_lengths(path)
    best = None
    pose_yaw = yaw_from_quaternion(pose.orientation)

    for idx in range(len(path.points) - 1):
        a = point_position(path.points[idx])
        b = point_position(path.points[idx + 1])
        vx = b.x - a.x
        vy = b.y - a.y
        segment_length = math.hypot(vx, vy)
        if segment_length < 1.0e-6:
            continue
        segment_yaw = math.atan2(vy, vx)
        if yaw_threshold is not None and abs(
            normalize_angle(segment_yaw - pose_yaw)
        ) > max(float(yaw_threshold), 0.0):
            continue

        wx = position.x - a.x
        wy = position.y - a.y
        raw_ratio = (wx * vx + wy * vy) / (segment_length * segment_length)
        ratio = clamp(raw_ratio, 0.0, 1.0)
        closest_x = a.x + vx * ratio
        closest_y = a.y + vy * ratio
        dx = position.x - closest_x
        dy = position.y - closest_y
        distance = math.hypot(dx, dy)
        signed_lateral = (vx * dy - vy * dx) / segment_length
        longitudinal = cumulative[idx] + segment_length * ratio
        # Preserve the along-track offset outside the path endpoints.  Clamping
        # it to zero made an object far behind the first point look as if it
        # were located at s=0 with only a small lateral error.
        if idx == 0 and raw_ratio < 0.0:
            longitudinal = segment_length * raw_ratio
        elif idx == len(path.points) - 2 and raw_ratio > 1.0:
            longitudinal = cumulative[idx] + segment_length * raw_ratio

        if best is None or distance < best.distance:
            best = PathProjection(
                idx,
                longitudinal,
                signed_lateral,
                distance,
                ratio,
                segment_yaw,
            )

    return best


def _interpolate_trajectory_point(first, second, ratio, segment_yaw):
    ratio = clamp(float(ratio), 0.0, 1.0)
    output = deepcopy(first)
    first_data = point_data(first)
    second_data = point_data(second)
    output_data = point_data(output)

    first_position = first_data.pose.position
    second_position = second_data.pose.position
    output_data.pose.position.x = first_position.x + ratio * (
        second_position.x - first_position.x
    )
    output_data.pose.position.y = first_position.y + ratio * (
        second_position.y - first_position.y
    )
    output_data.pose.position.z = first_position.z + ratio * (
        second_position.z - first_position.z
    )
    output_data.pose.orientation = quaternion_from_yaw(segment_yaw)

    for field in (
        "longitudinal_velocity_mps",
        "lateral_velocity_mps",
        "acceleration_mps2",
        "heading_rate_rps",
        "front_wheel_angle_rad",
        "rear_wheel_angle_rad",
    ):
        if not hasattr(output_data, field) or not hasattr(second_data, field):
            continue
        first_value = float(getattr(first_data, field))
        second_value = float(getattr(second_data, field))
        setattr(output_data, field, first_value + ratio * (second_value - first_value))
    return output


def trajectory_point_at_distance(path, distances, station):
    if not path.points or not distances:
        return None
    if station <= 0.0:
        return deepcopy(path.points[0])
    if station >= distances[-1]:
        return deepcopy(path.points[-1])

    for index in range(len(path.points) - 1):
        start_s = distances[index]
        end_s = distances[index + 1]
        if station > end_s:
            continue
        segment_length = end_s - start_s
        if segment_length < 1.0e-6:
            return deepcopy(path.points[index])
        first_position = point_position(path.points[index])
        second_position = point_position(path.points[index + 1])
        segment_yaw = math.atan2(
            second_position.y - first_position.y,
            second_position.x - first_position.x,
        )
        return _interpolate_trajectory_point(
            path.points[index],
            path.points[index + 1],
            (station - start_s) / segment_length,
            segment_yaw,
        )
    return deepcopy(path.points[-1])


def crop_trajectory_around_ego(
    path,
    ego_pose,
    forward_length=200.0,
    backward_length=0.0,
    yaw_threshold=1.0472,
):
    """Extract an arc-length window around ego using Autoware-style yaw matching."""
    if not path.points or ego_pose is None:
        return deepcopy(path)

    projection = project_pose_to_path(path, ego_pose, yaw_threshold=yaw_threshold)
    if projection is None:
        projection = project_pose_to_path(path, ego_pose)
    if projection is None:
        return deepcopy(path)

    distances = path_cumulative_lengths(path)
    if not distances:
        return deepcopy(path)
    start_s = max(0.0, projection.longitudinal_distance - max(backward_length, 0.0))
    end_s = min(
        distances[-1],
        projection.longitudinal_distance + max(forward_length, 0.0),
    )

    output = deepcopy(path)
    output.points = []
    start_point = trajectory_point_at_distance(path, distances, start_s)
    if start_point is not None:
        output.points.append(start_point)
    for point, station in zip(path.points, distances):
        if station > start_s + 1.0e-6 and station < end_s - 1.0e-6:
            output.points.append(deepcopy(point))
    end_point = trajectory_point_at_distance(path, distances, end_s)
    if end_point is not None:
        if not output.points or distance_2d(
            point_position(output.points[-1]), point_position(end_point)
        ) > 1.0e-6:
            output.points.append(end_point)
    return output


def set_velocity_window(path, start_distance, end_distance, velocity_mps, mode="min"):
    output = deepcopy(path)
    distances = path_cumulative_lengths(output)
    for point, distance in zip(output.points, distances):
        if distance < start_distance or distance > end_distance:
            continue
        if mode == "set":
            set_point_velocity(point, velocity_mps)
        else:
            set_point_velocity(point, min(point_velocity(point), velocity_mps))
    return output


def feasible_stop_velocity(remaining_distance, deceleration):
    if remaining_distance <= 0.0:
        return 0.0
    decel = max(abs(deceleration), 0.1)
    return math.sqrt(2.0 * decel * remaining_distance)


def object_pose(predicted_object):
    return predicted_object.kinematics.initial_pose_with_covariance.pose


def object_speed(predicted_object):
    twist = predicted_object.kinematics.initial_twist_with_covariance.twist
    linear = twist.linear
    return math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)


def object_velocity_xy(predicted_object):
    """Return an object's measured velocity in the trajectory frame."""
    linear = predicted_object.kinematics.initial_twist_with_covariance.twist.linear
    return float(linear.x), float(linear.y)


def oriented_box_corners(x, y, yaw, length, width, center_offset=0.0):
    """Return the four corners of a 2-D oriented bounding box.

    ``x, y`` is the pose origin.  ``center_offset`` is useful for ego poses
    whose origin is at the rear axle rather than at the middle of the vehicle.
    """
    length = max(float(length), 0.01)
    width = max(float(width), 0.01)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    center_x = float(x) + center_offset * cos_yaw
    center_y = float(y) + center_offset * sin_yaw
    half_length = length * 0.5
    half_width = width * 0.5
    corners = []
    for longitudinal, lateral in (
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ):
        corners.append((
            center_x + longitudinal * cos_yaw - lateral * sin_yaw,
            center_y + longitudinal * sin_yaw + lateral * cos_yaw,
        ))
    return corners


def _projection_interval(corners, axis):
    values = [corner[0] * axis[0] + corner[1] * axis[1] for corner in corners]
    return min(values), max(values)


def oriented_boxes_intersect(first_corners, second_corners):
    """Separating-axis intersection test for two convex oriented rectangles."""
    for corners in (first_corners, second_corners):
        for first, second in zip(corners, corners[1:] + corners[:1]):
            edge_x = second[0] - first[0]
            edge_y = second[1] - first[1]
            length = math.hypot(edge_x, edge_y)
            if length < 1.0e-9:
                continue
            axis = (-edge_y / length, edge_x / length)
            first_min, first_max = _projection_interval(first_corners, axis)
            second_min, second_max = _projection_interval(second_corners, axis)
            if first_max < second_min or second_max < first_min:
                return False
    return True
