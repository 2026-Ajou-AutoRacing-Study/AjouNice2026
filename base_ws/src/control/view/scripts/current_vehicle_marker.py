#!/usr/bin/env python3

import copy

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class CurrentVehicleMarker:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/localization/kinematic_state")
        output_topic = rospy.get_param("~output_topic", "/viewer/current_vehicle_marker")
        self.mesh_resource = rospy.get_param(
            "~mesh_resource",
            "package://ioniq5/meshes/ioniq5/ioniq5_base_link.dae",
        )
        self.mesh_scale = float(rospy.get_param("~mesh_scale", 1.0))
        self.mesh_x_offset = float(rospy.get_param("~mesh_x_offset", -0.05))
        self.mesh_z_offset = float(rospy.get_param("~mesh_z_offset", -0.3705))
        self.mesh_use_embedded_materials = bool(
            rospy.get_param("~mesh_use_embedded_materials", False)
        )
        self.publisher = rospy.Publisher(output_topic, Marker, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, Odometry, self.odometry_callback, queue_size=1
        )

    def odometry_callback(self, message):
        marker = Marker()
        marker.header = message.header
        marker.ns = "current_vehicle"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(message.pose.pose)
        orientation = marker.pose.orientation
        # Apply the mesh's rear-axle-frame offset in the vehicle's local frame.
        marker.pose.position.x += self.mesh_x_offset * (
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )
        marker.pose.position.y += self.mesh_x_offset * 2.0 * (
            orientation.x * orientation.y + orientation.w * orientation.z
        )
        marker.pose.position.z += self.mesh_z_offset
        marker.scale.x = self.mesh_scale
        marker.scale.y = self.mesh_scale
        marker.scale.z = self.mesh_scale
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = self.mesh_use_embedded_materials
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.2)
        self.publisher.publish(marker)


def main():
    rospy.init_node("current_vehicle_marker")
    CurrentVehicleMarker()
    rospy.spin()


if __name__ == "__main__":
    main()
