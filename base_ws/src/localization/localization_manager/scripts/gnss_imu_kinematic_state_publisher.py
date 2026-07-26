#!/usr/bin/env python3

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def normalized_frame(frame):
    return frame.lstrip("/")


class GnssImuKinematicStatePublisher:
    """Expose the selected fusion backend through the Planning state contract."""

    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/localization/odom"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/kinematic_state"
        )
        self.map_frame = normalized_frame(
            rospy.get_param("~map_frame", "map")
        )
        self.base_frame = normalized_frame(
            rospy.get_param("~base_frame", "base_link")
        )
        self.last_stamp = None

        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=100
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self.odom_callback, queue_size=100
        )
        rospy.loginfo(
            "gnss_imu_kinematic_state_publisher: %s -> %s",
            self.input_topic,
            self.output_topic,
        )

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

        output = copy.deepcopy(message)
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        self.last_stamp = message.header.stamp
        self.publisher.publish(output)


def main():
    rospy.init_node("gnss_imu_kinematic_state_publisher")
    GnssImuKinematicStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
