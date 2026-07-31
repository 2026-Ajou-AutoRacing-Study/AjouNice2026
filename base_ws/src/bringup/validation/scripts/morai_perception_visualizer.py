#!/usr/bin/env python3
"""Render MORAI object input as class-colored 3D bounding boxes in RViz."""

import math

import rospy
from geometry_msgs.msg import Point
from morai_msgs.msg import ObjectStatusList
from visualization_msgs.msg import Marker, MarkerArray


CLASS_STYLES = {
    "vehicle": ("VEHICLE", (0.10, 0.65, 1.00)),
    "pedestrian": ("PEDESTRIAN", (1.00, 0.18, 0.32)),
    "obstacle": ("OBSTACLE", (1.00, 0.68, 0.12)),
}

BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def bounded_dimension(value):
    return max(0.10, abs(float(value)))


def point(x, y, z):
    output = Point()
    output.x = float(x)
    output.y = float(y)
    output.z = float(z)
    return output


def box_corners(size_x, size_y, size_z):
    x = size_x * 0.5
    y = size_y * 0.5
    z = size_z * 0.5
    return (
        point(-x, -y, -z),
        point(x, -y, -z),
        point(x, y, -z),
        point(-x, y, -z),
        point(-x, -y, z),
        point(x, -y, z),
        point(x, y, z),
        point(-x, y, z),
    )


def apply_box_pose(marker, detected_object, position_is_bottom):
    size_z = bounded_dimension(detected_object.size.z)
    marker.pose.position.x = detected_object.position.x
    marker.pose.position.y = detected_object.position.y
    marker.pose.position.z = detected_object.position.z
    if position_is_bottom:
        marker.pose.position.z += size_z * 0.5

    yaw = math.radians(float(detected_object.heading))
    marker.pose.orientation.z = math.sin(yaw * 0.5)
    marker.pose.orientation.w = math.cos(yaw * 0.5)


def make_object_markers(
    header,
    detected_object,
    category,
    marker_id,
    lifetime,
    position_is_bottom=True,
    show_labels=True,
):
    label_text, color = CLASS_STYLES[category]
    size_x = bounded_dimension(detected_object.size.x)
    size_y = bounded_dimension(detected_object.size.y)
    size_z = bounded_dimension(detected_object.size.z)
    namespace = "morai_perception/{}".format(category)

    fill = Marker()
    fill.header = header
    fill.ns = namespace + "/fill"
    fill.id = marker_id
    fill.type = Marker.CUBE
    fill.action = Marker.ADD
    apply_box_pose(fill, detected_object, position_is_bottom)
    fill.scale.x = size_x
    fill.scale.y = size_y
    fill.scale.z = size_z
    fill.color.r, fill.color.g, fill.color.b = color
    fill.color.a = 0.18
    fill.lifetime = lifetime

    outline = Marker()
    outline.header = header
    outline.ns = namespace + "/outline"
    outline.id = marker_id
    outline.type = Marker.LINE_LIST
    outline.action = Marker.ADD
    outline.pose = fill.pose
    outline.scale.x = 0.08
    outline.color.r, outline.color.g, outline.color.b = color
    outline.color.a = 1.0
    outline.lifetime = lifetime
    corners = box_corners(size_x, size_y, size_z)
    for start, end in BOX_EDGES:
        outline.points.append(corners[start])
        outline.points.append(corners[end])

    markers = [fill, outline]
    if show_labels:
        text = Marker()
        text.header = header
        text.ns = namespace + "/label"
        text.id = marker_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = detected_object.position.x
        text.pose.position.y = detected_object.position.y
        text.pose.position.z = fill.pose.position.z + size_z * 0.5 + 0.45
        text.pose.orientation.w = 1.0
        text.scale.z = 0.45
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = "{} #{}".format(label_text, detected_object.unique_id)
        text.lifetime = lifetime
        markers.append(text)
    return markers


def make_marker_array(
    message,
    marker_lifetime_sec=0.35,
    position_is_bottom=True,
    show_labels=True,
    fallback_frame="morai_map",
):
    header = message.header
    if not header.frame_id:
        header.frame_id = fallback_frame

    lifetime = rospy.Duration(max(0.05, float(marker_lifetime_sec)))
    delete_all = Marker()
    delete_all.header = header
    delete_all.action = Marker.DELETEALL
    markers = [delete_all]

    groups = (
        ("vehicle", message.npc_list),
        ("pedestrian", message.pedestrian_list),
        ("obstacle", message.obstacle_list),
    )
    marker_id = 0
    for category, objects in groups:
        for detected_object in objects:
            markers.extend(
                make_object_markers(
                    header,
                    detected_object,
                    category,
                    marker_id,
                    lifetime,
                    position_is_bottom,
                    show_labels,
                )
            )
            marker_id += 1
    return MarkerArray(markers=markers)


class MoraiPerceptionVisualizer:
    def __init__(self):
        self.marker_lifetime_sec = float(
            rospy.get_param("~marker_lifetime_sec", 0.35)
        )
        self.position_is_bottom = bool(
            rospy.get_param("~position_is_bottom", True)
        )
        self.show_labels = bool(rospy.get_param("~show_labels", True))
        self.fallback_frame = rospy.get_param("~fallback_frame", "morai_map")
        input_topic = rospy.get_param("~input_topic", "/Object_topic")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/morai/objects/markers"
        )

        self.publisher = rospy.Publisher(
            output_topic, MarkerArray, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self.on_objects, queue_size=1
        )
        rospy.loginfo(
            "MORAI perception RViz boxes: %s -> %s", input_topic, output_topic
        )

    def on_objects(self, message):
        self.publisher.publish(
            make_marker_array(
                message,
                self.marker_lifetime_sec,
                self.position_is_bottom,
                self.show_labels,
                self.fallback_frame,
            )
        )


def main():
    rospy.init_node("morai_perception_visualizer")
    MoraiPerceptionVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
