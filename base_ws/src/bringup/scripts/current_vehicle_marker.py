#!/usr/bin/env python3

import copy
import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(
        r=float(red),
        g=float(green),
        b=float(blue),
        a=float(alpha),
    )


def yaw_from_quaternion(value):
    return math.atan2(
        2.0 * (value.w * value.z + value.x * value.y),
        1.0 - 2.0 * (value.y * value.y + value.z * value.z),
    )


def transformed_position(origin, yaw, forward, left, up):
    return Point(
        x=origin.x + math.cos(yaw) * forward - math.sin(yaw) * left,
        y=origin.y + math.sin(yaw) * forward + math.cos(yaw) * left,
        z=origin.z + up,
    )


def make_marker(message, namespace, marker_id, marker_type, lifetime):
    marker = Marker()
    marker.header = copy.deepcopy(message.header)
    marker.header.frame_id = marker.header.frame_id or "map"
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation = copy.deepcopy(message.pose.pose.orientation)
    marker.lifetime = rospy.Duration(max(float(lifetime), 0.0))
    return marker


def make_vehicle_marker_array(
    message,
    vehicle_length=4.635,
    vehicle_width=1.892,
    wheelbase=3.0,
    rear_overhang=0.79,
    marker_lifetime=0.5,
    icon_scale=1.0,
):
    """Create a top-down car icon whose origin follows the odometry base_link."""
    origin = message.pose.pose.position
    yaw = yaw_from_quaternion(message.pose.pose.orientation)
    icon_scale = max(float(icon_scale), 0.1)
    front_overhang = max(vehicle_length - wheelbase - rear_overhang, 0.0)
    center_x = 0.5 * (wheelbase + front_overhang - rear_overhang)

    def scaled_forward(value):
        return center_x + icon_scale * (value - center_x)

    def scaled_left(value):
        return icon_scale * value

    body = make_marker(message, "current_vehicle/body", 0, Marker.CUBE, marker_lifetime)
    body.pose.position = transformed_position(origin, yaw, center_x, 0.0, 0.38)
    body.scale.x = icon_scale * vehicle_length
    body.scale.y = icon_scale * vehicle_width
    body.scale.z = 0.42
    body.color = color(0.04, 0.72, 1.0, 0.92)

    cabin = make_marker(message, "current_vehicle/cabin", 0, Marker.CUBE, marker_lifetime)
    cabin.pose.position = transformed_position(
        origin, yaw, center_x + icon_scale * 0.15, 0.0, 0.67
    )
    cabin.scale.x = icon_scale * 2.15
    cabin.scale.y = icon_scale * 1.55
    cabin.scale.z = 0.24
    cabin.color = color(0.05, 0.16, 0.24, 0.96)

    outline = make_marker(
        message, "current_vehicle/outline", 0, Marker.LINE_STRIP, marker_lifetime
    )
    outline.pose.position = copy.deepcopy(origin)
    outline.scale.x = 0.11
    outline.color = color(0.72, 0.94, 1.0, 1.0)
    front = wheelbase + front_overhang
    rear = -rear_overhang
    half_width = 0.5 * vehicle_width
    outline.points = [
        Point(x=scaled_forward(rear), y=scaled_left(half_width), z=0.62),
        Point(
            x=scaled_forward(front - 0.35),
            y=scaled_left(half_width),
            z=0.62,
        ),
        Point(
            x=scaled_forward(front),
            y=scaled_left(0.55 * half_width),
            z=0.62,
        ),
        Point(
            x=scaled_forward(front),
            y=scaled_left(-0.55 * half_width),
            z=0.62,
        ),
        Point(
            x=scaled_forward(front - 0.35),
            y=scaled_left(-half_width),
            z=0.62,
        ),
        Point(x=scaled_forward(rear), y=scaled_left(-half_width), z=0.62),
        Point(x=scaled_forward(rear), y=scaled_left(half_width), z=0.62),
    ]

    markers = [body, cabin, outline]
    wheel_positions = (
        (0.0, half_width + 0.03),
        (0.0, -half_width - 0.03),
        (wheelbase, half_width + 0.03),
        (wheelbase, -half_width - 0.03),
    )
    for marker_id, (forward, left) in enumerate(wheel_positions):
        wheel = make_marker(
            message,
            "current_vehicle/wheels",
            marker_id,
            Marker.CUBE,
            marker_lifetime,
        )
        wheel.pose.position = transformed_position(
            origin,
            yaw,
            scaled_forward(forward),
            scaled_left(left),
            0.26,
        )
        wheel.scale.x = icon_scale * 0.72
        wheel.scale.y = icon_scale * 0.18
        wheel.scale.z = 0.22
        wheel.color = color(0.015, 0.02, 0.025, 1.0)
        markers.append(wheel)

    for marker_id, left in enumerate((0.58, -0.58)):
        headlight = make_marker(
            message,
            "current_vehicle/headlights",
            marker_id,
            Marker.SPHERE,
            marker_lifetime,
        )
        headlight.pose.position = transformed_position(
            origin,
            yaw,
            scaled_forward(front - 0.05),
            scaled_left(left),
            0.48,
        )
        headlight.scale.x = icon_scale * 0.24
        headlight.scale.y = icon_scale * 0.22
        headlight.scale.z = 0.16
        headlight.color = color(1.0, 0.92, 0.34, 1.0)
        markers.append(headlight)

    heading = make_marker(
        message, "current_vehicle/heading", 0, Marker.ARROW, marker_lifetime
    )
    heading.pose.position = transformed_position(origin, yaw, center_x, 0.0, 1.02)
    heading.scale.x = icon_scale * 2.0
    heading.scale.y = icon_scale * 0.32
    heading.scale.z = icon_scale * 0.32
    heading.color = color(0.25, 1.0, 0.45, 1.0)
    markers.append(heading)
    return MarkerArray(markers=markers)


class CurrentVehicleMarker:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/localization/kinematic_state"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/viewer/current_vehicle_marker"
        )
        self.vehicle_length = max(float(rospy.get_param("~vehicle_length", 4.635)), 0.5)
        self.vehicle_width = max(float(rospy.get_param("~vehicle_width", 1.892)), 0.3)
        self.wheelbase = max(float(rospy.get_param("~wheelbase", 3.0)), 0.3)
        self.rear_overhang = max(float(rospy.get_param("~rear_overhang", 0.79)), 0.0)
        self.marker_lifetime = max(
            float(rospy.get_param("~marker_lifetime", 0.5)), 0.0
        )
        self.icon_scale = max(float(rospy.get_param("~icon_scale", 1.0)), 0.1)
        self.publisher = rospy.Publisher(
            self.output_topic, MarkerArray, queue_size=1
        )
        rospy.Subscriber(self.input_topic, Odometry, self.on_odometry, queue_size=1)
        rospy.loginfo(
            "current_vehicle_marker: input=%s output=%s",
            self.input_topic,
            self.output_topic,
        )

    def on_odometry(self, message):
        self.publisher.publish(
            make_vehicle_marker_array(
                message,
                self.vehicle_length,
                self.vehicle_width,
                self.wheelbase,
                self.rear_overhang,
                self.marker_lifetime,
                self.icon_scale,
            )
        )


def main():
    rospy.init_node("current_vehicle_marker")
    CurrentVehicleMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
