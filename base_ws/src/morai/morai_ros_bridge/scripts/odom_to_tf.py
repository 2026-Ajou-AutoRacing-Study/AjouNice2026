#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class OdomToTf:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/localization/odom")
        self.frame_id = rospy.get_param("~frame_id", "morai_map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.use_msg_frame_ids = get_bool_param("~use_msg_frame_ids", False)
        self.last_stamp_by_child = {}

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom_subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=50
        )

        rospy.loginfo(
            "odom_to_tf: %s -> /tf using %s -> %s",
            self.odom_topic,
            self.frame_id,
            self.child_frame_id,
        )

    def odom_callback(self, msg):
        parent_frame = self.frame_id
        child_frame = self.child_frame_id
        if self.use_msg_frame_ids:
            parent_frame = msg.header.frame_id or parent_frame
            child_frame = msg.child_frame_id or child_frame

        if not parent_frame or not child_frame:
            rospy.logwarn_throttle(2.0, "Skipping odometry with empty TF frame")
            return

        stamp = msg.header.stamp if not msg.header.stamp.is_zero() else rospy.Time.now()
        if self.last_stamp_by_child.get(child_frame) == stamp:
            return
        self.last_stamp_by_child[child_frame] = stamp

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rospy.init_node("odom_to_tf")
    OdomToTf()
    rospy.spin()


if __name__ == "__main__":
    main()
