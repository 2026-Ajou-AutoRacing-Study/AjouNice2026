#!/usr/bin/env python3

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from nav_msgs.msg import Odometry


PARAM_NS = "/localization_manager"


def param(name, default=None):
    return rospy.get_param(PARAM_NS + "/" + name, default)


def normalized_frame(frame):
    return frame.lstrip("/")


def transform_rotation(transform):
    quaternion = transform.rotation
    return tft.quaternion_matrix(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )[0:3, 0:3]


class FastLioKinematicStatePublisher:
    def __init__(self):
        self.map_frame = normalized_frame(param("frames/map", "map"))
        self.camera_init_frame = normalized_frame(
            param("frames/camera_init", "camera_init")
        )
        self.body_frame = normalized_frame(param("frames/body", "body"))
        self.base_link_frame = normalized_frame(
            param("frames/base_link", "base_link")
        )
        self.odom_topic = param("topics/fast_lio_odom", "/localization/odom")
        self.output_topic = param(
            "topics/kinematic_state", "/localization/kinematic_state"
        )
        self.tf_timeout = float(param("runtime/tf_lookup_timeout_sec", 0.05))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=50
        )
        self.subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=100
        )

        rospy.loginfo(
            "fast_lio_kinematic_state_publisher: %s -> %s; covariance is unavailable",
            self.odom_topic,
            self.output_topic,
        )

    def odom_callback(self, odom):
        if odom.header.stamp.is_zero():
            rospy.logwarn_throttle(2.0, "Skipping odometry with zero timestamp")
            return
        if (
            normalized_frame(odom.header.frame_id) != self.camera_init_frame
            or normalized_frame(odom.child_frame_id) != self.body_frame
        ):
            rospy.logerr_throttle(
                2.0,
                "Expected odom frames %s -> %s, received %s -> %s",
                self.camera_init_frame,
                self.body_frame,
                odom.header.frame_id,
                odom.child_frame_id,
            )
            return

        try:
            map_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                odom.header.stamp,
                rospy.Duration(self.tf_timeout),
            )
            body_base = self.tf_buffer.lookup_transform(
                self.body_frame,
                self.base_link_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Cannot publish kinematic state at %.9f: %s",
                odom.header.stamp.to_sec(),
                str(error),
            )
            return

        raw_twist = np.array(
            [
                odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                odom.twist.twist.linear.z,
                odom.twist.twist.angular.x,
                odom.twist.twist.angular.y,
                odom.twist.twist.angular.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(raw_twist)):
            rospy.logwarn_throttle(2.0, "Skipping odometry with invalid twist")
            return

        rotation_body_base = transform_rotation(body_base.transform)
        rotation_base_body = rotation_body_base.transpose()
        body_to_base = np.array(
            [
                body_base.transform.translation.x,
                body_base.transform.translation.y,
                body_base.transform.translation.z,
            ],
            dtype=float,
        )

        velocity_body_origin = raw_twist[0:3]
        angular_velocity_body = raw_twist[3:6]
        velocity_base_origin_in_body = velocity_body_origin + np.cross(
            angular_velocity_body, body_to_base
        )
        velocity_base = np.dot(rotation_base_body, velocity_base_origin_in_body)
        angular_velocity_base = np.dot(
            rotation_base_body, angular_velocity_body
        )

        output = Odometry()
        output.header.seq = odom.header.seq
        output.header.stamp = odom.header.stamp
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_link_frame
        output.pose.pose.position.x = map_base.transform.translation.x
        output.pose.pose.position.y = map_base.transform.translation.y
        output.pose.pose.position.z = map_base.transform.translation.z
        output.pose.pose.orientation = map_base.transform.rotation
        output.twist.twist.linear.x = velocity_base[0]
        output.twist.twist.linear.y = velocity_base[1]
        output.twist.twist.linear.z = velocity_base[2]
        output.twist.twist.angular.x = angular_velocity_base[0]
        output.twist.twist.angular.y = angular_velocity_base[1]
        output.twist.twist.angular.z = angular_velocity_base[2]
        output.pose.covariance = [0.0] * 36
        output.twist.covariance = [0.0] * 36
        self.publisher.publish(output)


def main():
    rospy.init_node("fast_lio_kinematic_state_publisher")
    FastLioKinematicStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
