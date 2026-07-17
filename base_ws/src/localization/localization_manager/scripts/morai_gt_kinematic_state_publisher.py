#!/usr/bin/env python3

import math
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
        self.pending_angular_velocity = None
        self.previous_heading_deg = None
        self.previous_heading_stamp = None
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

        state_values = np.array(
            [
                ego.position.x,
                ego.position.y,
                ego.position.z,
                ego.heading,
                ego.velocity.x,
                ego.velocity.y,
                ego.velocity.z,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(state_values)):
            rospy.logwarn_throttle(
                2.0, "Skipping MORAI Ego ground truth with non-finite state"
            )
            return

        with self.pending_lock:
            angular_velocity = self.resolve_angular_velocity(ego, stamp)
            if not np.all(np.isfinite(angular_velocity)):
                rospy.logwarn_throttle(
                    2.0,
                    "Skipping MORAI Ego ground truth with non-finite angular velocity",
                )
                return
            self.pending_ego = ego
            self.pending_angular_velocity = angular_velocity

    def resolve_angular_velocity(self, ego, stamp):
        angular_velocity = getattr(ego, "angular_velocity", None)
        if angular_velocity is not None:
            return (
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z,
            )

        yaw_rate = 0.0
        if self.previous_heading_stamp is not None:
            dt = (stamp - self.previous_heading_stamp).to_sec()
            if dt > 1e-6:
                heading_delta = ego.heading - self.previous_heading_deg
                heading_delta = (heading_delta + 180.0) % 360.0 - 180.0
                yaw_rate = math.radians(heading_delta / dt)

        if (
            self.previous_heading_stamp is None
            or stamp > self.previous_heading_stamp
        ):
            self.previous_heading_deg = ego.heading
            self.previous_heading_stamp = stamp

        return (0.0, 0.0, yaw_rate)

    def retry_pending(self, _event):
        with self.pending_lock:
            ego = self.pending_ego
            angular_velocity = self.pending_angular_velocity
        if ego is not None:
            self.try_publish(ego, angular_velocity)

    def try_publish(self, ego, angular_velocity):
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
        output.twist.twist.angular.x = angular_velocity[0]
        output.twist.twist.angular.y = angular_velocity[1]
        output.twist.twist.angular.z = angular_velocity[2]
        output.pose.covariance = [0.0] * 36
        output.twist.covariance = [0.0] * 36
        self.publisher.publish(output)

        self.last_published_stamp = stamp
        self.clear_pending_if_same(ego)

    def clear_pending_if_same(self, ego):
        with self.pending_lock:
            if self.pending_ego is ego:
                self.pending_ego = None
                self.pending_angular_velocity = None


def main():
    rospy.init_node("morai_gt_kinematic_state_publisher")
    MoraiGtKinematicStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
