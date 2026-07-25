#!/usr/bin/env python3

import copy

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class CurrentVehicleMarker:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/localization/kinematic_state")
        output_topic = rospy.get_param("~output_topic", "/viewer/current_vehicle_marker")
        self.length_m = float(rospy.get_param("~length_m", 4.5))
        self.width_m = float(rospy.get_param("~width_m", 1.8))
        self.height_m = float(rospy.get_param("~height_m", 1.5))
        self.publisher = rospy.Publisher(output_topic, Marker, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, Odometry, self.odometry_callback, queue_size=1
        )

    def odometry_callback(self, message):
        marker = Marker()
        marker.header = message.header
        marker.ns = "current_vehicle"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(message.pose.pose)
        marker.scale.x = self.length_m
        marker.scale.y = self.width_m
        marker.scale.z = self.height_m
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.2)
        self.publisher.publish(marker)


def main():
    rospy.init_node("current_vehicle_marker")
    CurrentVehicleMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
