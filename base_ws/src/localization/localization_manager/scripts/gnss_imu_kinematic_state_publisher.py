#!/usr/bin/env python3

import copy
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def normalized_frame(frame):
    return frame.lstrip("/")


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def normalized_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def z_sample_state(has_sample, age_sec, max_age_sec):
    """Classify the auxiliary GNSS height without gating planar state.

    The localization filters run in 2D and continue producing valid x/y/yaw
    and twist through a GNSS outage.  Height is display/map metadata, so once
    one finite sample exists its age must never stop the planar state stream.
    """
    if not has_sample:
        return "missing"
    if not math.isfinite(age_sec) or not math.isfinite(max_age_sec):
        return "invalid"
    if abs(age_sec) > max_age_sec:
        return "held"
    return "fresh"


def compose_planar_map_to_odom(map_to_base, odom_to_base):
    """Return planar map->odom from co-timed map->base and odom->base poses."""
    map_yaw = yaw_from_quaternion(map_to_base.orientation)
    odom_yaw = yaw_from_quaternion(odom_to_base.rotation)
    map_to_odom_yaw = normalized_angle(map_yaw - odom_yaw)
    cosine = math.cos(map_to_odom_yaw)
    sine = math.sin(map_to_odom_yaw)

    rotated_odom_x = (
        cosine * odom_to_base.translation.x
        - sine * odom_to_base.translation.y
    )


def extrapolated_planar_pose(message, target_stamp_sec):
    """Extrapolate child-frame odometry to a nearby target timestamp."""
    pose = message.pose.pose
    twist = message.twist.twist
    dt_sec = target_stamp_sec - message.header.stamp.to_sec()
    yaw = yaw_from_quaternion(pose.orientation)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    velocity_x = cosine * twist.linear.x - sine * twist.linear.y
    velocity_y = sine * twist.linear.x + cosine * twist.linear.y
    return (
        pose.position.x + velocity_x * dt_sec,
        pose.position.y + velocity_y * dt_sec,
        pose.position.z + twist.linear.z * dt_sec,
        normalized_angle(yaw + twist.angular.z * dt_sec),
    )


def compose_planar_pose_map_to_odom(map_to_base, odom_pose):
    """Return map->odom from map->base and an ``(x, y, z, yaw)`` tuple."""
    map_yaw = yaw_from_quaternion(map_to_base.orientation)
    odom_x, odom_y, odom_z, odom_yaw = odom_pose
    map_to_odom_yaw = normalized_angle(map_yaw - odom_yaw)
    cosine = math.cos(map_to_odom_yaw)
    sine = math.sin(map_to_odom_yaw)
    return (
        map_to_base.position.x - cosine * odom_x + sine * odom_y,
        map_to_base.position.y - sine * odom_x - cosine * odom_y,
        map_to_base.position.z - odom_z,
        map_to_odom_yaw,
    )
    rotated_odom_y = (
        sine * odom_to_base.translation.x
        + cosine * odom_to_base.translation.y
    )
    return (
        map_to_base.position.x - rotated_odom_x,
        map_to_base.position.y - rotated_odom_y,
        map_to_base.position.z - odom_to_base.translation.z,
        map_to_odom_yaw,
    )


class GnssImuKinematicStatePublisher:
    """Expose the selected fusion backend through the Planning state contract."""

    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/localization/odom"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/kinematic_state"
        )
        self.z_input_topic = rospy.get_param(
            "~z_input_topic", "/localization/gnss/odom"
        )
        self.local_input_topic = rospy.get_param(
            "~local_input_topic", "/localization/odometry/local"
        )
        self.map_frame = normalized_frame(
            rospy.get_param("~map_frame", "map")
        )
        self.odom_frame = normalized_frame(
            rospy.get_param("~odom_frame", "odom")
        )
        self.base_frame = normalized_frame(
            rospy.get_param("~base_frame", "base_link")
        )
        self.max_z_age = float(rospy.get_param("~max_z_age_sec", 0.5))
        if not math.isfinite(self.max_z_age) or self.max_z_age <= 0.0:
            raise rospy.ROSInitException(
                "~max_z_age_sec must be positive and finite"
            )
        self.max_local_age = float(rospy.get_param("~max_local_age_sec", 0.05))
        if not math.isfinite(self.max_local_age) or self.max_local_age <= 0.0:
            raise rospy.ROSInitException(
                "~max_local_age_sec must be positive and finite"
            )
        self.last_stamp = None
        self.local_message = None
        self.z_stamp = None
        self.z_position = None
        self.z_variance = None
        self.z_was_held = False

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.odom_callback, queue_size=100
        )
        self.z_subscriber = rospy.Subscriber(
            self.z_input_topic, Odometry, self.z_callback, queue_size=100
        )
        self.local_subscriber = rospy.Subscriber(
            self.local_input_topic, Odometry, self.local_callback, queue_size=100
        )
        rospy.loginfo(
            "gnss_imu_kinematic_state_publisher: %s + local(%s) + z(%s) "
            "-> %s and %s -> %s",
            self.input_topic,
            self.local_input_topic,
            self.z_input_topic,
            self.output_topic,
            self.map_frame,
            self.odom_frame,
        )

    def local_callback(self, message):
        if message.header.stamp.is_zero():
            return
        if normalized_frame(message.header.frame_id) != self.odom_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected local parent frame %s, received %s",
                self.odom_frame,
                message.header.frame_id,
            )
            return
        if normalized_frame(message.child_frame_id) != self.base_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected local child frame %s, received %s",
                self.base_frame,
                message.child_frame_id,
            )
            return
        self.local_message = message

    def z_callback(self, message):
        if message.header.stamp.is_zero():
            return
        if normalized_frame(message.header.frame_id) != self.map_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected z parent frame %s, received %s",
                self.map_frame,
                message.header.frame_id,
            )
            return
        if normalized_frame(message.child_frame_id) != self.base_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected z child frame %s, received %s",
                self.base_frame,
                message.child_frame_id,
            )
            return
        z_position = message.pose.pose.position.z
        z_variance = message.pose.covariance[14]
        if not math.isfinite(z_position) or not math.isfinite(z_variance):
            rospy.logwarn_throttle(2.0, "Rejecting non-finite GNSS z sample")
            return
        self.z_stamp = message.header.stamp
        self.z_position = z_position
        self.z_variance = z_variance

    @staticmethod
    def values(message):
        pose = message.pose.pose
        twist = message.twist.twist
        return (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
            *message.pose.covariance,
            *message.twist.covariance,
        )

    def odom_callback(self, message):
        if message.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Rejecting odometry with zero timestamp")
            return
        if self.last_stamp is not None and message.header.stamp <= self.last_stamp:
            rospy.logwarn_throttle(
                2.0, "Rejecting duplicate/out-of-order fused odometry"
            )
            return
        if normalized_frame(message.header.frame_id) != self.map_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected parent frame %s, received %s",
                self.map_frame,
                message.header.frame_id,
            )
            return
        if normalized_frame(message.child_frame_id) != self.base_frame:
            rospy.logerr_throttle(
                2.0,
                "Expected child frame %s, received %s",
                self.base_frame,
                message.child_frame_id,
            )
            return
        if not all(math.isfinite(value) for value in self.values(message)):
            rospy.logwarn_throttle(
                2.0, "Rejecting fused odometry with non-finite value"
            )
            return

        if self.z_stamp is None:
            rospy.logwarn_throttle(2.0, "Waiting for GNSS z sample")
            return
        z_age = (message.header.stamp - self.z_stamp).to_sec()
        z_state = z_sample_state(True, z_age, self.max_z_age)
        if z_state == "invalid":
            rospy.logwarn_throttle(
                2.0,
                "Rejecting fused odometry with invalid GNSS z age %.3fs",
                z_age,
            )
            return
        if z_state == "held":
            rospy.logwarn_throttle(
                5.0,
                "Holding last valid GNSS z: age %.3fs exceeds %.3fs; "
                "planar fused state remains valid",
                z_age,
                self.max_z_age,
            )
            self.z_was_held = True
        elif self.z_was_held:
            rospy.loginfo(
                "Fresh GNSS z resumed after hold; age=%.3fs",
                z_age,
            )
            self.z_was_held = False

        if self.local_message is None:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for local odometry %s",
                self.local_input_topic,
            )
            return
        target_stamp_sec = message.header.stamp.to_sec()
        local_age = target_stamp_sec - self.local_message.header.stamp.to_sec()
        if abs(local_age) > self.max_local_age:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for co-timed local odometry: signed age %.3fs exceeds %.3fs",
                local_age,
                self.max_local_age,
            )
            return
        local_pose = extrapolated_planar_pose(
            self.local_message, target_stamp_sec
        )

        output = copy.deepcopy(message)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.z = self.z_position
        pose_covariance = list(output.pose.covariance)
        pose_covariance[14] = self.z_variance
        output.pose.covariance = pose_covariance

        x, y, z, yaw = compose_planar_pose_map_to_odom(
            output.pose.pose,
            local_pose,
        )
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.z = math.sin(0.5 * yaw)
        transform.transform.rotation.w = math.cos(0.5 * yaw)

        self.last_stamp = message.header.stamp
        self.publisher.publish(output)
        self.tf_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("gnss_imu_kinematic_state_publisher")
    GnssImuKinematicStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
