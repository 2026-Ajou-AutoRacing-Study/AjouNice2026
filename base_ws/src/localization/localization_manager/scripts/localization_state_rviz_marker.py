#!/usr/bin/env python3

"""Visualize the current localization speed and twist above the vehicle."""

import math
import time

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class LocalizationStateRvizMarker:
    def __init__(self):
        self.state_topic = rospy.get_param(
            "~state_topic", "/localization/kinematic_state"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/localization/debug/state_markers"
        )
        self.text_height = float(rospy.get_param("~text_height", 2.4))
        self.text_size = float(rospy.get_param("~text_size", 0.42))
        self.arrow_height = float(rospy.get_param("~arrow_height", 0.65))
        self.arrow_scale = float(rospy.get_param("~arrow_scale", 0.35))
        self.arrow_max_length = float(rospy.get_param("~arrow_max_length", 12.0))
        self.rate_alpha = float(rospy.get_param("~rate_alpha", 0.15))

        self.last_arrival = None
        self.filtered_rate = None
        self.publisher = rospy.Publisher(
            self.output_topic, MarkerArray, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            self.state_topic, Odometry, self.state_callback, queue_size=10
        )
        rospy.loginfo(
            "localization_state_rviz_marker: %s -> %s",
            self.state_topic,
            self.output_topic,
        )

    @staticmethod
    def marker_base(frame_id, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = frame_id
        # Zero time asks RViz to use the newest available vehicle transform.
        marker.header.stamp = rospy.Time(0)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def update_rate(self):
        now = time.monotonic()
        if self.last_arrival is not None:
            dt = now - self.last_arrival
            if dt > 1e-6:
                rate = 1.0 / dt
                if self.filtered_rate is None:
                    self.filtered_rate = rate
                else:
                    alpha = min(1.0, max(0.0, self.rate_alpha))
                    self.filtered_rate += alpha * (rate - self.filtered_rate)
        self.last_arrival = now

    @staticmethod
    def message_age(message):
        if message.header.stamp.is_zero():
            return None
        now = rospy.Time.now()
        if now.is_zero():
            return None
        age = (now - message.header.stamp).to_sec()
        if not math.isfinite(age) or age < -0.1 or age > 3600.0:
            return None
        return max(0.0, age)

    def make_text_marker(self, message, speed, age):
        twist = message.twist.twist
        frame_id = message.child_frame_id.lstrip("/") or "base_link"
        marker = self.marker_base(
            frame_id, "localization_state_text", 0, Marker.TEXT_VIEW_FACING
        )
        marker.pose.position.z = self.text_height
        marker.scale.z = self.text_size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95

        rate_text = (
            "{:.1f} Hz".format(self.filtered_rate)
            if self.filtered_rate is not None
            else "-- Hz"
        )
        age_text = "{:.0f} ms".format(age * 1000.0) if age is not None else "-- ms"
        marker.text = (
            "Speed  {:.1f} km/h  ({:.2f} m/s)\n"
            "vx {:+.2f}  vy {:+.2f} m/s\n"
            "yaw rate {:+.1f} deg/s\n"
            "state {}  age {}"
        ).format(
            speed * 3.6,
            speed,
            twist.linear.x,
            twist.linear.y,
            math.degrees(twist.angular.z),
            rate_text,
            age_text,
        )
        return marker

    def make_velocity_arrow(self, message, speed):
        frame_id = message.child_frame_id.lstrip("/") or "base_link"
        marker = self.marker_base(
            frame_id, "localization_velocity_vector", 1, Marker.ARROW
        )
        if speed < 1e-3:
            marker.action = Marker.DELETE
            return marker

        twist = message.twist.twist
        length = min(self.arrow_max_length, speed * self.arrow_scale)
        unit_x = twist.linear.x / speed
        unit_y = twist.linear.y / speed
        marker.points = [
            Point(0.0, 0.0, self.arrow_height),
            Point(unit_x * length, unit_y * length, self.arrow_height),
        ]
        marker.scale.x = 0.12
        marker.scale.y = 0.28
        marker.scale.z = 0.38
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        return marker

    def state_callback(self, message):
        twist = message.twist.twist
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(
                2.0, "Skipping non-finite localization twist visualization"
            )
            return

        self.update_rate()
        speed = math.hypot(twist.linear.x, twist.linear.y)
        markers = MarkerArray()
        markers.markers = [
            self.make_text_marker(message, speed, self.message_age(message)),
            self.make_velocity_arrow(message, speed),
        ]
        self.publisher.publish(markers)


def main():
    rospy.init_node("localization_state_rviz_marker")
    LocalizationStateRvizMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
