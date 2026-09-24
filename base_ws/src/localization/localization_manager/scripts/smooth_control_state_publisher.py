#!/usr/bin/env python3

"""Publish a controller-only pose with a smoothed global/local alignment.

The global EKF remains the source of map accuracy for planning.  The local EKF
remains the source of continuous short-term motion.  This node low-pass filters
only their planar ``map -> odom`` alignment and composes it with the local pose;
it never owns TF and therefore cannot perturb the production map graph.
"""

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z**2),
    )


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def planar_alignment(map_pose, local_pose):
    """Return map->odom SE(2) that maps ``local_pose`` onto ``map_pose``."""
    map_x, map_y, map_yaw = map_pose
    local_x, local_y, local_yaw = local_pose
    alignment_yaw = normalize_angle(map_yaw - local_yaw)
    cosine = math.cos(alignment_yaw)
    sine = math.sin(alignment_yaw)
    return (
        map_x - cosine * local_x + sine * local_y,
        map_y - sine * local_x - cosine * local_y,
        alignment_yaw,
    )


def compose_planar(alignment, local_pose):
    align_x, align_y, align_yaw = alignment
    local_x, local_y, local_yaw = local_pose
    cosine = math.cos(align_yaw)
    sine = math.sin(align_yaw)
    return (
        align_x + cosine * local_x - sine * local_y,
        align_y + sine * local_x + cosine * local_y,
        normalize_angle(align_yaw + local_yaw),
    )


def exponential_alpha(dt_sec, tau_sec):
    if not math.isfinite(dt_sec) or dt_sec <= 0.0:
        return 0.0
    if not math.isfinite(tau_sec) or tau_sec <= 0.0:
        return 1.0
    return 1.0 - math.exp(-dt_sec / tau_sec)


def limit_vector_step(dx, dy, limit):
    magnitude = math.hypot(dx, dy)
    if limit <= 0.0 or magnitude <= limit:
        return dx, dy
    scale = limit / magnitude
    return dx * scale, dy * scale


class PlanarAlignmentSmoother:
    """Low-pass and rate-limit the global correction, not vehicle motion."""

    def __init__(
        self,
        translation_tau_sec,
        yaw_tau_sec,
        max_translation_rate_mps=0.0,
        max_yaw_rate_radps=0.0,
        reset_gap_sec=0.5,
        reset_translation_error_m=0.0,
        reset_yaw_error_rad=0.0,
    ):
        values = (
            translation_tau_sec,
            yaw_tau_sec,
            max_translation_rate_mps,
            max_yaw_rate_radps,
            reset_gap_sec,
            reset_translation_error_m,
            reset_yaw_error_rad,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("smoother parameters must be finite and non-negative")
        if reset_gap_sec <= 0.0:
            raise ValueError("reset_gap_sec must be positive")
        self.translation_tau_sec = translation_tau_sec
        self.yaw_tau_sec = yaw_tau_sec
        self.max_translation_rate_mps = max_translation_rate_mps
        self.max_yaw_rate_radps = max_yaw_rate_radps
        self.reset_gap_sec = reset_gap_sec
        self.reset_translation_error_m = reset_translation_error_m
        self.reset_yaw_error_rad = reset_yaw_error_rad
        self.value = None
        self.stamp_sec = None
        self.last_reset_reason = None

    def reset(self, alignment, stamp_sec):
        self.value = tuple(float(value) for value in alignment)
        self.stamp_sec = float(stamp_sec)
        return self.value

    def update(self, target, stamp_sec, reference_pose=None):
        stamp_sec = float(stamp_sec)
        if self.value is None:
            self.last_reset_reason = "initialization"
            return self.reset(target, stamp_sec)
        dt_sec = stamp_sec - self.stamp_sec
        if dt_sec <= 0.0:
            raise ValueError("alignment timestamps must be strictly increasing")
        if dt_sec > self.reset_gap_sec:
            self.last_reset_reason = "timestamp_gap"
            return self.reset(target, stamp_sec)

        if reference_pose is None:
            translation_error = math.hypot(
                target[0] - self.value[0], target[1] - self.value[1]
            )
        else:
            target_pose = compose_planar(target, reference_pose)
            current_pose = compose_planar(self.value, reference_pose)
            translation_error = math.hypot(
                target_pose[0] - current_pose[0],
                target_pose[1] - current_pose[1],
            )
        yaw_error = abs(normalize_angle(target[2] - self.value[2]))
        if (
            self.reset_translation_error_m > 0.0
            and translation_error > self.reset_translation_error_m
        ):
            self.last_reset_reason = "translation_discontinuity"
            return self.reset(target, stamp_sec)
        if (
            self.reset_yaw_error_rad > 0.0
            and yaw_error > self.reset_yaw_error_rad
        ):
            self.last_reset_reason = "yaw_discontinuity"
            return self.reset(target, stamp_sec)

        self.last_reset_reason = None

        alpha_xy = exponential_alpha(dt_sec, self.translation_tau_sec)
        step_x = alpha_xy * (target[0] - self.value[0])
        step_y = alpha_xy * (target[1] - self.value[1])
        if self.max_translation_rate_mps > 0.0:
            step_x, step_y = limit_vector_step(
                step_x,
                step_y,
                self.max_translation_rate_mps * dt_sec,
            )

        alpha_yaw = exponential_alpha(dt_sec, self.yaw_tau_sec)
        step_yaw = alpha_yaw * normalize_angle(target[2] - self.value[2])
        if self.max_yaw_rate_radps > 0.0:
            yaw_limit = self.max_yaw_rate_radps * dt_sec
            step_yaw = max(-yaw_limit, min(yaw_limit, step_yaw))

        self.value = (
            self.value[0] + step_x,
            self.value[1] + step_y,
            normalize_angle(self.value[2] + step_yaw),
        )
        self.stamp_sec = stamp_sec
        return self.value


def extrapolated_local_pose(message, target_stamp_sec):
    pose = message.pose.pose
    twist = message.twist.twist
    source_stamp_sec = message.header.stamp.to_sec()
    dt_sec = target_stamp_sec - source_stamp_sec
    yaw = yaw_from_quaternion(pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    velocity_x = cosine * twist.linear.x - sine * twist.linear.y
    velocity_y = sine * twist.linear.x + cosine * twist.linear.y
    return (
        pose.position.x + velocity_x * dt_sec,
        pose.position.y + velocity_y * dt_sec,
        normalize_angle(yaw + twist.angular.z * dt_sec),
    )


class SmoothControlStatePublisher:
    def __init__(self):
        self.global_topic = rospy.get_param(
            "~global_topic", "/localization/kinematic_state"
        )
        self.local_topic = rospy.get_param(
            "~local_topic", "/localization/odometry/local"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/control_kinematic_state"
        )
        self.map_frame = rospy.get_param("~map_frame", "map").lstrip("/")
        self.odom_frame = rospy.get_param("~odom_frame", "odom").lstrip("/")
        self.base_frame = rospy.get_param("~base_frame", "base_link").lstrip("/")
        self.max_local_age_sec = float(rospy.get_param("~max_local_age_sec", 0.05))
        if not math.isfinite(self.max_local_age_sec) or self.max_local_age_sec <= 0.0:
            raise rospy.ROSInitException("~max_local_age_sec must be positive")

        self.smoother = PlanarAlignmentSmoother(
            translation_tau_sec=float(
                rospy.get_param("~translation_tau_sec", 0.5)
            ),
            yaw_tau_sec=float(rospy.get_param("~yaw_tau_sec", 0.5)),
            max_translation_rate_mps=float(
                rospy.get_param("~max_translation_rate_mps", 0.0)
            ),
            max_yaw_rate_radps=math.radians(
                float(rospy.get_param("~max_yaw_rate_degps", 0.0))
            ),
            reset_gap_sec=float(rospy.get_param("~reset_gap_sec", 0.5)),
            reset_translation_error_m=float(
                rospy.get_param("~reset_translation_error_m", 0.5)
            ),
            reset_yaw_error_rad=math.radians(
                float(rospy.get_param("~reset_yaw_error_deg", 20.0))
            ),
        )
        self.local_message = None
        self.last_global_stamp = None
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=100)
        self.local_subscriber = rospy.Subscriber(
            self.local_topic, Odometry, self.local_callback, queue_size=100
        )
        self.global_subscriber = rospy.Subscriber(
            self.global_topic, Odometry, self.global_callback, queue_size=100
        )
        rospy.loginfo(
            "smooth_control_state_publisher: %s + %s -> %s; tau=(%.3f, %.3f)s",
            self.global_topic,
            self.local_topic,
            self.output_topic,
            self.smoother.translation_tau_sec,
            self.smoother.yaw_tau_sec,
        )

    def local_callback(self, message):
        if message.header.stamp.is_zero():
            return
        if message.header.frame_id.lstrip("/") != self.odom_frame:
            rospy.logerr_throttle(2.0, "Control local state is not in odom")
            return
        if message.child_frame_id.lstrip("/") != self.base_frame:
            rospy.logerr_throttle(2.0, "Control local child is not base_link")
            return
        self.local_message = message

    def global_callback(self, message):
        stamp = message.header.stamp
        if stamp.is_zero() or self.local_message is None:
            return
        if message.header.frame_id.lstrip("/") != self.map_frame:
            rospy.logerr_throttle(2.0, "Control global state is not in map")
            return
        if message.child_frame_id.lstrip("/") != self.base_frame:
            rospy.logerr_throttle(2.0, "Control global child is not base_link")
            return
        if self.last_global_stamp is not None and stamp <= self.last_global_stamp:
            rospy.logwarn_throttle(2.0, "Rejecting duplicate control-state timestamp")
            return

        stamp_sec = stamp.to_sec()
        local_age_sec = stamp_sec - self.local_message.header.stamp.to_sec()
        if abs(local_age_sec) > self.max_local_age_sec:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for fresh local odometry: signed age %.3fs exceeds %.3fs",
                local_age_sec,
                self.max_local_age_sec,
            )
            return

        local_pose = extrapolated_local_pose(self.local_message, stamp_sec)
        global_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(message.pose.pose.orientation),
        )
        raw_alignment = planar_alignment(global_pose, local_pose)
        try:
            smooth_alignment = self.smoother.update(
                raw_alignment, stamp_sec, reference_pose=local_pose
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, str(error))
            return
        if self.smoother.last_reset_reason not in (None, "initialization"):
            rospy.logwarn(
                "Reset smooth control alignment: %s",
                self.smoother.last_reset_reason,
            )
        output_pose = compose_planar(smooth_alignment, local_pose)

        output = copy.deepcopy(message)
        output.pose.pose.position.x = output_pose[0]
        output.pose.pose.position.y = output_pose[1]
        quaternion = quaternion_from_yaw(output_pose[2])
        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]
        output.twist = copy.deepcopy(self.local_message.twist)

        self.last_global_stamp = stamp
        self.publisher.publish(output)


def main():
    rospy.init_node("smooth_control_state_publisher")
    SmoothControlStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
