#!/usr/bin/env python3

import math

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry
from tf.transformations import (
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def pose_to_matrix(position, orientation):
    translation = translation_matrix([position.x, position.y, position.z])
    rotation = quaternion_matrix([
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    ])
    return np.dot(translation, rotation)


def xyz_rpy_to_matrix(xyz, rpy):
    translation = translation_matrix(xyz)
    q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    rotation = quaternion_matrix(q)
    return np.dot(translation, rotation)


def matrix_to_pose(matrix, pose):
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    q = quaternion_from_matrix(matrix)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]


def ego_to_map_base_matrix(msg):
    yaw = math.radians(msg.heading)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return pose_to_matrix(msg.position, orientation)


class FastLioOdomMapBridge:
    def __init__(self):
        self.lio_odom_topic = rospy.get_param("~lio_odom_topic", "/localization/odom")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.output_topic = rospy.get_param("~output_topic", "/localization/odom_map")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.use_ego_initial_alignment = get_bool_param("~use_ego_initial_alignment", True)
        self.initial_max_dt = float(rospy.get_param("~initial_max_dt", 0.2))
        self.publish_tf = get_bool_param("~publish_tf", False)
        self.copy_twist = get_bool_param("~copy_twist", True)

        body_to_base_xyz = rospy.get_param("~body_to_base_xyz", [0.0, 0.0, 0.0])
        body_to_base_rpy = rospy.get_param("~body_to_base_rpy", [0.0, 0.0, 0.0])
        self.t_body_base = xyz_rpy_to_matrix(body_to_base_xyz, body_to_base_rpy)

        self.latest_ego = None
        self.t_map_camera_init = None
        self.last_tf_stamp = None

        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        self.ego_sub = rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=50)
        self.odom_sub = rospy.Subscriber(self.lio_odom_topic, Odometry, self.odom_callback, queue_size=50)

        rospy.loginfo("fast_lio_odom_map_bridge:")
        rospy.loginfo("  input odom: %s", self.lio_odom_topic)
        rospy.loginfo("  input ego:  %s", self.ego_topic if self.use_ego_initial_alignment else "(disabled)")
        rospy.loginfo("  output:     %s (%s -> %s)", self.output_topic, self.map_frame, self.base_frame)
        rospy.loginfo("  publish_tf: %s", self.publish_tf)

    def ego_callback(self, msg):
        self.latest_ego = msg

    def try_initialize_alignment(self, odom_msg):
        if self.t_map_camera_init is not None:
            return True

        t_camera_body0 = pose_to_matrix(odom_msg.pose.pose.position, odom_msg.pose.pose.orientation)

        if not self.use_ego_initial_alignment:
            self.t_map_camera_init = inverse_matrix(t_camera_body0)
            rospy.logwarn("Initialized map->camera_init as inverse(first Fast-LIO pose); no map alignment source used")
            return True

        if self.latest_ego is None:
            rospy.logwarn_throttle(2.0, "Waiting for %s before initializing map alignment", self.ego_topic)
            return False

        odom_stamp = odom_msg.header.stamp if not odom_msg.header.stamp.is_zero() else rospy.Time.now()
        ego_stamp = self.latest_ego.header.stamp if not self.latest_ego.header.stamp.is_zero() else rospy.Time.now()
        dt = abs((odom_stamp - ego_stamp).to_sec())
        if dt > self.initial_max_dt:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for synchronized initial ego/odom pair: dt=%.3fs > %.3fs",
                dt,
                self.initial_max_dt,
            )
            return False

        t_map_base0 = ego_to_map_base_matrix(self.latest_ego)
        self.t_map_camera_init = (
            t_map_base0
            @ inverse_matrix(self.t_body_base)
            @ inverse_matrix(t_camera_body0)
        )

        _, _, yaw = euler_from_quaternion(quaternion_from_matrix(self.t_map_camera_init))
        rospy.loginfo(
            "Initialized map->camera_init from Ego_topic and Fast-LIO: x=%.3f y=%.3f z=%.3f yaw=%.3f deg",
            self.t_map_camera_init[0, 3],
            self.t_map_camera_init[1, 3],
            self.t_map_camera_init[2, 3],
            math.degrees(yaw),
        )
        return True

    def odom_callback(self, msg):
        if not self.try_initialize_alignment(msg):
            return

        t_camera_body = pose_to_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        t_map_base = self.t_map_camera_init @ t_camera_body @ self.t_body_base

        output = Odometry()
        output.header.stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_frame
        matrix_to_pose(t_map_base, output.pose.pose)
        output.pose.covariance = msg.pose.covariance
        if self.copy_twist:
            output.twist = msg.twist

        self.odom_pub.publish(output)
        self.publish_transform(output)

    def publish_transform(self, odom):
        if self.tf_broadcaster is None:
            return
        if odom.header.stamp == self.last_tf_stamp:
            return
        self.last_tf_stamp = odom.header.stamp

        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = odom.header.frame_id
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("fast_lio_odom_map_bridge")
    FastLioOdomMapBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
