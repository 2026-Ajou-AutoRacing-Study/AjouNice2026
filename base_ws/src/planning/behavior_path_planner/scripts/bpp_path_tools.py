#!/usr/bin/env python3

import math
from copy import deepcopy

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import String

from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import Path as AutowarePath
from tier4_planning_msgs.msg import AvoidanceDebugMsg, AvoidanceDebugMsgArray


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def smoothstep(edge0, edge1, value):
    if edge1 <= edge0:
        return 1.0 if value >= edge1 else 0.0
    ratio = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


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
    distances = path_cumulative_lengths(path)
    return distances[-1] if distances else 0.0


def shifted_pose(pose, lateral_offset):
    shifted = deepcopy(pose)
    yaw = yaw_from_quaternion(shifted.orientation)
    shifted.position.x += -math.sin(yaw) * lateral_offset
    shifted.position.y += math.cos(yaw) * lateral_offset
    return shifted


def apply_shift_profile(path, offset_at_distance):
    output = deepcopy(path)
    distances = path_cumulative_lengths(output)
    for point, distance in zip(output.points, distances):
        offset = offset_at_distance(distance)
        if abs(offset) > 1.0e-6:
            point.point.pose = shifted_pose(point.point.pose, offset)
    return output


def apply_s_curve_shift(path, target_offset, start_distance, end_distance):
    if abs(target_offset) < 1.0e-6:
        return deepcopy(path)

    def profile(distance):
        return target_offset * smoothstep(start_distance, end_distance, distance)

    return apply_shift_profile(path, profile)


def apply_window_shift(path, target_offset, peak_distance, prepare_distance, return_distance):
    if abs(target_offset) < 1.0e-6:
        return deepcopy(path)

    start_distance = max(0.0, peak_distance - max(prepare_distance, 0.1))
    end_distance = peak_distance + max(return_distance, 0.1)

    def profile(distance):
        if distance <= peak_distance:
            return target_offset * smoothstep(start_distance, peak_distance, distance)
        return target_offset * (1.0 - smoothstep(peak_distance, end_distance, distance))

    return apply_shift_profile(path, profile)


def apply_pull_out_shift(path, start_offset, merge_distance):
    if abs(start_offset) < 1.0e-6:
        return deepcopy(path)

    def profile(distance):
        return start_offset * (1.0 - smoothstep(0.0, max(merge_distance, 0.1), distance))

    return apply_shift_profile(path, profile)


def apply_goal_shift(path, goal_offset, pull_over_distance):
    if abs(goal_offset) < 1.0e-6:
        return deepcopy(path)

    total_length = path_length(path)
    start_distance = max(0.0, total_length - max(pull_over_distance, 0.1))

    def profile(distance):
        return goal_offset * smoothstep(start_distance, total_length, distance)

    return apply_shift_profile(path, profile)


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


class PathProjection:
    def __init__(
        self,
        index,
        longitudinal_distance,
        signed_lateral_distance,
        distance,
        segment_ratio=0.0,
        projected_position=None,
    ):
        self.index = index
        self.longitudinal_distance = longitudinal_distance
        self.signed_lateral_distance = signed_lateral_distance
        self.distance = distance
        self.segment_ratio = segment_ratio
        self.projected_position = projected_position


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
        return PathProjection(
            0,
            0.0,
            signed_lateral,
            math.hypot(dx, dy),
            segment_ratio=0.0,
            projected_position=deepcopy(path_pose.position),
        )

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
        closest_z = a.z + (b.z - a.z) * ratio
        dx = position.x - closest_x
        dy = position.y - closest_y
        distance = math.hypot(dx, dy)
        signed_lateral = (vx * dy - vy * dx) / segment_length
        longitudinal = cumulative[idx] + segment_length * ratio

        if best is None or distance < best.distance:
            best = PathProjection(
                idx,
                longitudinal,
                signed_lateral,
                distance,
                segment_ratio=ratio,
                projected_position=Point(
                    x=closest_x,
                    y=closest_y,
                    z=closest_z,
                ),
            )

    return best


def object_pose(predicted_object):
    return predicted_object.kinematics.initial_pose_with_covariance.pose


def object_speed(predicted_object):
    twist = predicted_object.kinematics.initial_twist_with_covariance.twist
    linear = twist.linear
    return math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)


def object_width(predicted_object):
    dimensions = predicted_object.shape.dimensions
    if dimensions.y > 1.0e-3:
        return dimensions.y

    points = predicted_object.shape.footprint.points
    if not points:
        return 0.0
    ys = [point.y for point in points]
    return max(ys) - min(ys)


def uuid_to_string(uuid_msg):
    return "".join("{:02x}".format(value) for value in uuid_msg.uuid)


def to_debug_path(path_with_lane_id):
    msg = Path()
    msg.header = path_with_lane_id.header
    for point in path_with_lane_id.points:
        pose_stamped = PoseStamped()
        pose_stamped.header = path_with_lane_id.header
        pose_stamped.pose = point.point.pose
        msg.poses.append(pose_stamped)
    return msg


def to_autoware_path(path_with_lane_id):
    msg = AutowarePath()
    msg.header = path_with_lane_id.header
    msg.points = [deepcopy(item.point) for item in path_with_lane_id.points]
    msg.left_bound = deepcopy(path_with_lane_id.left_bound)
    msg.right_bound = deepcopy(path_with_lane_id.right_bound)
    return msg


def make_avoidance_debug_array(header, candidates):
    msg = AvoidanceDebugMsgArray()
    msg.header = header
    for candidate in candidates:
        debug = AvoidanceDebugMsg()
        debug.object_id = candidate.get("object_id", "")
        debug.allow_avoidance = bool(candidate.get("allow_avoidance", False))
        debug.longitudinal_distance = float(candidate.get("longitudinal_distance", 0.0))
        debug.lateral_distance_from_centerline = float(candidate.get("lateral_distance", 0.0))
        debug.to_furthest_linestring_distance = float(candidate.get("path_distance", 0.0))
        debug.max_shift_length = float(candidate.get("shift_length", 0.0))
        debug.required_jerk = float(candidate.get("required_jerk", 0.0))
        debug.maximum_jerk = float(candidate.get("maximum_jerk", 0.0))
        debug.failed_reason = candidate.get("failed_reason", "")
        msg.avoidance_info.append(debug)
    return msg


class PathModuleBase:
    def __init__(self, module_name):
        self.module_name = module_name
        self.input_path = None

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.planning_hz = float(rospy.get_param("~planning_hz", 10.0))
        input_path_topic = rospy.get_param("~input_path_topic", "~input/path")

        rospy.Subscriber(input_path_topic, PathWithLaneId, self.path_callback, queue_size=1)

        self.path_pub = rospy.Publisher("~output/path", PathWithLaneId, queue_size=1)
        self.debug_path_pub = rospy.Publisher("~debug/path", Path, queue_size=1)
        self.status_pub = rospy.Publisher("~debug/status", String, queue_size=1)

        period = rospy.Duration(1.0 / max(self.planning_hz, 0.1))
        self.timer = rospy.Timer(period, self.on_timer)

    def path_callback(self, msg):
        self.input_path = msg

    def process_path(self, path, stamp):
        return path, "pass-through"

    def on_timer(self, _event):
        if rospy.is_shutdown():
            return
        if self.input_path is None:
            rospy.logwarn_throttle(5.0, "%s: waiting for input PathWithLaneId", self.module_name)
            return

        stamp = rospy.Time.now()
        try:
            processed = self.process_path(deepcopy(self.input_path), stamp)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise
        if isinstance(processed, tuple):
            output_path, status = processed
        else:
            output_path = processed
            status = "ok"

        if output_path is None:
            return

        output_path.header.stamp = stamp
        if not output_path.header.frame_id:
            output_path.header.frame_id = self.frame_id

        try:
            self.path_pub.publish(output_path)
            self.debug_path_pub.publish(to_debug_path(output_path))

            status_msg = String()
            status_msg.data = status
            self.status_pub.publish(status_msg)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise
