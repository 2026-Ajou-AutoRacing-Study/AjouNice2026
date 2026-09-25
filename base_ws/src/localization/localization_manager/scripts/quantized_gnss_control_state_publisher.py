#!/usr/bin/env python3

"""Publish an experimental controller state constrained by the GNSS cell.

The local EKF supplies continuous motion.  A quantized GNSS sample represents
an interval rather than an exact point: while the local prediction remains in
that interval no translation correction is applied.  Once it leaves, only the
minimum correction back to the interval boundary is applied.  Planning and TF
continue to use the production Global EKF state.
"""

import copy
import math

import rospy
from nav_msgs.msg import Odometry

from smooth_control_state_publisher import (
    compose_planar,
    extrapolated_local_pose,
    normalize_angle,
    planar_alignment,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def quantized_cell_correction(
    predicted_x,
    predicted_y,
    measured_x,
    measured_y,
    half_width_x,
    half_width_y,
    grid_yaw,
):
    """Return the minimum map-frame delta into a rotated measurement cell."""
    cosine = math.cos(grid_yaw)
    sine = math.sin(grid_yaw)
    delta_x = predicted_x - measured_x
    delta_y = predicted_y - measured_y
    grid_x = cosine * delta_x + sine * delta_y
    grid_y = -sine * delta_x + cosine * delta_y
    clamped_x = clamp(grid_x, -half_width_x, half_width_x)
    clamped_y = clamp(grid_y, -half_width_y, half_width_y)
    correction_grid_x = clamped_x - grid_x
    correction_grid_y = clamped_y - grid_y
    return (
        cosine * correction_grid_x - sine * correction_grid_y,
        sine * correction_grid_x + cosine * correction_grid_y,
    )


def preserve_global_longitudinal(global_pose, candidate_pose):
    """Use candidate lateral displacement but keep Global longitudinal pose."""
    delta_x = candidate_pose[0] - global_pose[0]
    delta_y = candidate_pose[1] - global_pose[1]
    sine = math.sin(global_pose[2])
    cosine = math.cos(global_pose[2])
    lateral_delta = -delta_x * sine + delta_y * cosine
    return (
        global_pose[0] - lateral_delta * sine,
        global_pose[1] + lateral_delta * cosine,
        candidate_pose[2],
    )


class QuantizedGnssAlignment:
    """Maintain map->odom alignment with interval-valued GNSS translation."""

    def __init__(
        self,
        half_width_x,
        half_width_y,
        grid_yaw=0.0,
        yaw_tau_sec=0.5,
        reset_translation_error_m=0.5,
        reset_yaw_error_rad=math.radians(20.0),
    ):
        values = (
            half_width_x,
            half_width_y,
            yaw_tau_sec,
            reset_translation_error_m,
            reset_yaw_error_rad,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("alignment parameters must be finite and non-negative")
        if half_width_x <= 0.0 or half_width_y <= 0.0:
            raise ValueError("GNSS cell half-widths must be positive")
        if not math.isfinite(grid_yaw):
            raise ValueError("grid yaw must be finite")
        self.half_width_x = half_width_x
        self.half_width_y = half_width_y
        self.grid_yaw = grid_yaw
        self.yaw_tau_sec = yaw_tau_sec
        self.reset_translation_error_m = reset_translation_error_m
        self.reset_yaw_error_rad = reset_yaw_error_rad
        self.value = None
        self.stamp_sec = None
        self.last_reset_reason = None

    def reset(self, alignment, stamp_sec, reason):
        self.value = tuple(float(value) for value in alignment)
        self.stamp_sec = float(stamp_sec)
        self.last_reset_reason = reason
        return self.value

    def update_global(self, target, local_pose, stamp_sec):
        if self.value is None:
            return self.reset(target, stamp_sec, "initialization")
        dt_sec = float(stamp_sec) - self.stamp_sec
        if dt_sec <= 0.0:
            raise ValueError("alignment timestamps must be strictly increasing")
        target_pose = compose_planar(target, local_pose)
        current_pose = compose_planar(self.value, local_pose)
        translation_error = math.hypot(
            target_pose[0] - current_pose[0], target_pose[1] - current_pose[1]
        )
        yaw_error = abs(normalize_angle(target[2] - self.value[2]))
        if (
            self.reset_translation_error_m > 0.0
            and translation_error > self.reset_translation_error_m
        ):
            return self.reset(target, stamp_sec, "translation_discontinuity")
        if (
            self.reset_yaw_error_rad > 0.0
            and yaw_error > self.reset_yaw_error_rad
        ):
            return self.reset(target, stamp_sec, "yaw_discontinuity")

        alpha = 1.0
        if self.yaw_tau_sec > 0.0:
            alpha = 1.0 - math.exp(-dt_sec / self.yaw_tau_sec)
        # Changing map->odom yaw around the odom origin can move a vehicle that
        # is hundreds of metres away.  Re-anchor translation so the current
        # vehicle position remains fixed while only its orientation changes.
        new_yaw = normalize_angle(
            self.value[2]
            + alpha * normalize_angle(target[2] - self.value[2])
        )
        preserved_pose = (
            current_pose[0],
            current_pose[1],
            normalize_angle(new_yaw + local_pose[2]),
        )
        self.value = planar_alignment(preserved_pose, local_pose)
        self.stamp_sec = float(stamp_sec)
        self.last_reset_reason = None
        return self.value

    def constrain_position(self, local_pose, gnss_x, gnss_y):
        if self.value is None:
            raise ValueError("alignment must be initialized before GNSS correction")
        predicted = compose_planar(self.value, local_pose)
        correction_x, correction_y = quantized_cell_correction(
            predicted[0],
            predicted[1],
            gnss_x,
            gnss_y,
            self.half_width_x,
            self.half_width_y,
            self.grid_yaw,
        )
        self.value = (
            self.value[0] + correction_x,
            self.value[1] + correction_y,
            self.value[2],
        )
        return correction_x, correction_y


class QuantizedGnssControlStatePublisher:
    def __init__(self):
        self.global_topic = rospy.get_param(
            "~global_topic", "/localization/kinematic_state"
        )
        self.local_topic = rospy.get_param(
            "~local_topic", "/localization/odometry/local"
        )
        self.gnss_topic = rospy.get_param(
            "~gnss_topic", "/localization/gnss/odom"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/control_kinematic_state"
        )
        self.map_frame = rospy.get_param("~map_frame", "map").lstrip("/")
        self.odom_frame = rospy.get_param("~odom_frame", "odom").lstrip("/")
        self.base_frame = rospy.get_param("~base_frame", "base_link").lstrip("/")
        self.max_local_age_sec = float(rospy.get_param("~max_local_age_sec", 0.05))
        self.max_gnss_age_sec = float(rospy.get_param("~max_gnss_age_sec", 0.2))
        self.preserve_global_longitudinal = bool(
            rospy.get_param("~preserve_global_longitudinal", True)
        )
        cell_scale = float(rospy.get_param("~cell_scale", 1.25))
        if cell_scale <= 0.0 or not math.isfinite(cell_scale):
            raise rospy.ROSInitException("~cell_scale must be positive")
        self.alignment = QuantizedGnssAlignment(
            half_width_x=0.5
            * float(rospy.get_param("~cell_width_x_m", 0.148))
            * cell_scale,
            half_width_y=0.5
            * float(rospy.get_param("~cell_width_y_m", 0.185))
            * cell_scale,
            grid_yaw=math.radians(float(rospy.get_param("~grid_yaw_deg", 0.0))),
            yaw_tau_sec=float(rospy.get_param("~yaw_tau_sec", 0.0)),
            reset_translation_error_m=float(
                rospy.get_param("~reset_translation_error_m", 0.5)
            ),
            reset_yaw_error_rad=math.radians(
                float(rospy.get_param("~reset_yaw_error_deg", 20.0))
            ),
        )
        self.local_message = None
        self.gnss_message = None
        self.last_gnss_stamp = None
        self.last_global_stamp = None
        self.publisher = rospy.Publisher(self.output_topic, Odometry, queue_size=100)
        self.local_subscriber = rospy.Subscriber(
            self.local_topic, Odometry, self.local_callback, queue_size=100
        )
        self.gnss_subscriber = rospy.Subscriber(
            self.gnss_topic, Odometry, self.gnss_callback, queue_size=100
        )
        self.global_subscriber = rospy.Subscriber(
            self.global_topic, Odometry, self.global_callback, queue_size=100
        )
        rospy.loginfo(
            "quantized_gnss_control_state: %s + %s + %s -> %s; "
            "cell_half_width=(%.4f, %.4f)m",
            self.global_topic,
            self.local_topic,
            self.gnss_topic,
            self.output_topic,
            self.alignment.half_width_x,
            self.alignment.half_width_y,
        )

    def local_callback(self, message):
        if message.header.stamp.is_zero():
            return
        if message.header.frame_id.lstrip("/") != self.odom_frame:
            rospy.logerr_throttle(2.0, "Quantized control local state is not in odom")
            return
        if message.child_frame_id.lstrip("/") != self.base_frame:
            rospy.logerr_throttle(
                2.0, "Quantized control local child is not base_link"
            )
            return
        self.local_message = message

    def gnss_callback(self, message):
        if message.header.stamp.is_zero():
            return
        if message.header.frame_id.lstrip("/") != self.map_frame:
            rospy.logerr_throttle(2.0, "Quantized control GNSS state is not in map")
            return
        self.gnss_message = message

    def global_callback(self, message):
        stamp = message.header.stamp
        if stamp.is_zero() or self.local_message is None:
            return
        if message.header.frame_id.lstrip("/") != self.map_frame:
            rospy.logerr_throttle(2.0, "Quantized control global state is not in map")
            return
        if message.child_frame_id.lstrip("/") != self.base_frame:
            rospy.logerr_throttle(
                2.0, "Quantized control global child is not base_link"
            )
            return
        if self.last_global_stamp is not None and stamp <= self.last_global_stamp:
            return
        stamp_sec = stamp.to_sec()
        local_age = stamp_sec - self.local_message.header.stamp.to_sec()
        if abs(local_age) > self.max_local_age_sec:
            rospy.logwarn_throttle(2.0, "Waiting for fresh local odometry")
            return
        local_pose = extrapolated_local_pose(self.local_message, stamp_sec)
        global_pose = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw_from_quaternion(message.pose.pose.orientation),
        )
        target = planar_alignment(global_pose, local_pose)
        try:
            self.alignment.update_global(target, local_pose, stamp_sec)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, str(error))
            return
        if self.alignment.last_reset_reason not in (None, "initialization"):
            rospy.logwarn(
                "Reset quantized GNSS control alignment: %s",
                self.alignment.last_reset_reason,
            )

        if (
            self.gnss_message is not None
            and self.gnss_message.header.stamp != self.last_gnss_stamp
        ):
            gnss_stamp_sec = self.gnss_message.header.stamp.to_sec()
            gnss_age = stamp_sec - gnss_stamp_sec
            if abs(gnss_age) <= self.max_gnss_age_sec:
                local_at_gnss = extrapolated_local_pose(
                    self.local_message, gnss_stamp_sec
                )
                self.alignment.constrain_position(
                    local_at_gnss,
                    self.gnss_message.pose.pose.position.x,
                    self.gnss_message.pose.pose.position.y,
                )
                self.last_gnss_stamp = self.gnss_message.header.stamp

        output_pose = compose_planar(self.alignment.value, local_pose)
        if self.preserve_global_longitudinal:
            output_pose = preserve_global_longitudinal(global_pose, output_pose)
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
    rospy.init_node("quantized_gnss_control_state_publisher")
    QuantizedGnssControlStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
