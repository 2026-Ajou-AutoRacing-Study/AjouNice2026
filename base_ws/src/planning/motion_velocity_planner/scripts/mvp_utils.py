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


def point_position(path_point_with_lane_id):
    return path_point_with_lane_id.point.pose.position


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
    def __init__(self, index, longitudinal_distance, signed_lateral_distance, distance):
        self.index = index
        self.longitudinal_distance = longitudinal_distance
        self.signed_lateral_distance = signed_lateral_distance
        self.distance = distance


def project_pose_to_path(path, pose):
    if not path.points:
        return None

    position = pose.position
    if len(path.points) == 1:
        path_pose = path.points[0].point.pose
        yaw = yaw_from_quaternion(path_pose.orientation)
        dx = position.x - path_pose.position.x
        dy = position.y - path_pose.position.y
        signed_lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        return PathProjection(0, 0.0, signed_lateral, math.hypot(dx, dy))

    cumulative = path_cumulative_lengths(path)
    best = None

    for idx in range(len(path.points) - 1):
        a = path.points[idx].point.pose.position
        b = path.points[idx + 1].point.pose.position
        vx = b.x - a.x
        vy = b.y - a.y
        segment_length = math.hypot(vx, vy)
        if segment_length < 1.0e-6:
            continue

        wx = position.x - a.x
        wy = position.y - a.y
        ratio = clamp((wx * vx + wy * vy) / (segment_length * segment_length), 0.0, 1.0)
        closest_x = a.x + vx * ratio
        closest_y = a.y + vy * ratio
        dx = position.x - closest_x
        dy = position.y - closest_y
        distance = math.hypot(dx, dy)
        signed_lateral = (vx * dy - vy * dx) / segment_length
        longitudinal = cumulative[idx] + segment_length * ratio

        if best is None or distance < best.distance:
            best = PathProjection(idx, longitudinal, signed_lateral, distance)

    return best


def set_velocity_window(path, start_distance, end_distance, velocity_mps, mode="min"):
    output = deepcopy(path)
    distances = path_cumulative_lengths(output)
    for point, distance in zip(output.points, distances):
        if distance < start_distance or distance > end_distance:
            continue
        if mode == "set":
            point.point.longitudinal_velocity_mps = velocity_mps
        else:
            point.point.longitudinal_velocity_mps = min(
                point.point.longitudinal_velocity_mps, velocity_mps
            )
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
