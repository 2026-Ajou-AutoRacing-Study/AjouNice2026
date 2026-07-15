#!/usr/bin/env python3

import threading

import numpy as np
import rospy
import tf2_ros
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Odometry


PARAM_NS = "/localization_manager"


def param(name, default=None):
    return rospy.get_param(PARAM_NS + "/" + name, default)


def normalized_frame(frame):
    return frame.lstrip("/")


class MoraiGtKinematicStatePublisher:
    """Publish the Planning odometry contract from MORAI ground truth.

    ego_tf_broadcaster owns morai_map -> base_link. This node waits until that
    transform is available at the Ego message timestamp, then snapshots the
    composed map -> base_link pose and copies MORAI's base-link-frame twist.
    """

    def __init__(self):
        self.map_frame = normalized_frame(param("frames/map", "map"))
        self.base_link_frame = normalized_frame(
            param("frames/base_link", "base_link")
        )
        self.ego_topic = param("topics/ego", "/Ego_topic")
        self.output_topic = param(
            "topics/kinematic_state", "/localization/kinematic_state"
        )
        self.tf_timeout = float(param("runtime/tf_lookup_timeout_sec", 0.05))
        self.retry_period = float(
            param("runtime/morai_gt_retry_period_sec", 0.01)
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, Odometry, queue_size=50
        )

        self.pending_lock = threading.Lock()
        self.pending_ego = None
        self.last_published_stamp = rospy.Time(0)
        self.subscriber = rospy.Subscriber(
            self.ego_topic,
            EgoVehicleStatus,
            self.ego_callback,
            queue_size=100,
        )
        self.retry_timer = rospy.Timer(
            rospy.Duration(max(self.retry_period, 0.001)), self.retry_pending
        )

        rospy.loginfo(
            "morai_gt_kinematic_state_publisher: %s -> %s; "
            "pose=%s->%s, twist=base_link frame, covariance is unavailable",
            self.ego_topic,
            self.output_topic,
            self.map_frame,
            self.base_link_frame,
        )

    def ego_callback(self, ego):
        stamp = ego.header.stamp
        if stamp.is_zero():
            rospy.logwarn_throttle(
                2.0, "Skipping MORAI Ego ground truth with zero timestamp"
            )
            return

        values = np.array(
            [
                ego.position.x,
                ego.position.y,
                ego.position.z,
                ego.heading,
                ego.velocity.x,
                ego.velocity.y,
                ego.velocity.z,
                ego.angular_velocity.x,
                ego.angular_velocity.y,
                ego.angular_velocity.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            rospy.logwarn_throttle(
                2.0, "Skipping MORAI Ego ground truth with non-finite state"
            )
            return

        with self.pending_lock:
            self.pending_ego = ego

    def retry_pending(self, _event):
        with self.pending_lock:
            ego = self.pending_ego
        if ego is not None:
            self.try_publish(ego)

    def try_publish(self, ego):
        stamp = ego.header.stamp
        if stamp == self.last_published_stamp:
            self.clear_pending_if_same(ego)
            return

        try:
            map_base = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_link_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
        except tf2_ros.TransformException as error:
            rospy.logwarn_throttle(
                2.0,
                "Waiting for MORAI GT TF %s -> %s at %.9f: %s",
                self.map_frame,
                self.base_link_frame,
                stamp.to_sec(),
                str(error),
            )
            return

        output = Odometry()
        output.header.seq = ego.header.seq
        output.header.stamp = stamp
        output.header.frame_id = self.map_frame
        output.child_frame_id = self.base_link_frame
        output.pose.pose.position.x = map_base.transform.translation.x
        output.pose.pose.position.y = map_base.transform.translation.y
        output.pose.pose.position.z = map_base.transform.translation.z
        output.pose.pose.orientation = map_base.transform.rotation
        output.twist.twist.linear = ego.velocity
        output.twist.twist.angular = ego.angular_velocity
        output.pose.covariance = [0.0] * 36
        output.twist.covariance = [0.0] * 36
        self.publisher.publish(output)

        self.last_published_stamp = stamp
        self.clear_pending_if_same(ego)

    def clear_pending_if_same(self, ego):
        with self.pending_lock:
            if self.pending_ego is ego:
                self.pending_ego = None


def main():
    rospy.init_node("morai_gt_kinematic_state_publisher")
    MoraiGtKinematicStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
