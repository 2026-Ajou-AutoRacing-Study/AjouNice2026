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


def compose_planar_map_to_odom(map_to_base, odom_to_base):
    """Return planar map->odom from map->base and odom->base poses."""
    map_yaw = yaw_from_quaternion(map_to_base.orientation)
    odom_yaw = yaw_from_quaternion(odom_to_base.rotation)
    map_to_odom_yaw = normalized_angle(map_yaw - odom_yaw)
    cosine = math.cos(map_to_odom_yaw)
    sine = math.sin(map_to_odom_yaw)

    rotated_odom_x = (
        cosine * odom_to_base.translation.x
        - sine * odom_to_base.translation.y
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
        self.tf_timeout = float(rospy.get_param("~tf_timeout_sec", 0.05))
        self.last_stamp = None
        self.z_stamp = None
        self.z_position = None
        self.z_variance = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
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
        rospy.loginfo(
            "gnss_imu_kinematic_state_publisher: %s + z(%s) -> %s and %s -> %s",
            self.input_topic,
            self.z_input_topic,
            self.output_topic,
            self.map_frame,
            self.odom_frame,
        )

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
        z_age = abs((message.header.stamp - self.z_stamp).to_sec())
        if z_age > self.max_z_age:
            rospy.logwarn_throttle(
                2.0,
                "Rejecting fused odometry: GNSS z age %.3fs exceeds %.3fs",
                z_age,
                self.max_z_age,
            )
            return

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                self.odom_frame,
                self.base_frame,
                str(error),
            )
            return

        output = copy.deepcopy(message)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        output.pose.pose.position.z = self.z_position
        pose_covariance = list(output.pose.covariance)
        pose_covariance[14] = self.z_variance
        output.pose.covariance = pose_covariance

        x, y, z, yaw = compose_planar_map_to_odom(
            output.pose.pose,
            odom_to_base.transform,
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
