#!/usr/bin/env python3
"""Render MORAI ObjectStatus ground-truth boxes as map-frame wireframes."""

from __future__ import annotations

import copy
import math
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Point
from morai_msgs.msg import ObjectStatusList
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tracking_prediction_evaluation.bbox_reference import (
    EXPLICIT_VEHICLE_REFERENCE_PROFILES_V1,
    PASSENGER_BBOX_REFERENCE_V1,
)


BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def quaternion_yaw(heading_deg, transform_yaw):
    return math.radians(float(heading_deg)) + transform_yaw


def vehicle_bbox_offset(length, width, height):
    if 0.0 < length <= PASSENGER_BBOX_REFERENCE_V1["maximum_length_m"]:
        return PASSENGER_BBOX_REFERENCE_V1["longitudinal_offset_m"]
    for profile in EXPLICIT_VEHICLE_REFERENCE_PROFILES_V1:
        tolerance = float(profile["dimension_tolerance_m"])
        if all(
            abs(actual - float(profile[key])) <= tolerance
            for actual, key in (
                (length, "length_m"),
                (width, "width_m"),
                (height, "height_m"),
            )
        ):
            return float(profile["longitudinal_offset_m"])
    return 0.0


def box_corners(center, dimensions, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    half_x = max(0.01, dimensions[0]) * 0.5
    half_y = max(0.01, dimensions[1]) * 0.5
    half_z = max(0.01, dimensions[2]) * 0.5
    corners = []
    for local_z in (-half_z, half_z):
        for local_x, local_y in (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ):
            corners.append(
                Point(
                    x=center[0] + cosine * local_x - sine * local_y,
                    y=center[1] + sine * local_x + cosine * local_y,
                    z=center[2] + local_z,
                )
            )
    return corners


class MoraiGtBboxVisualization:
    def __init__(self):
        static_tf_file = Path(rospy.get_param("~morai_static_tf")).expanduser()
        document = yaml.safe_load(static_tf_file.read_text(encoding="utf-8")) or {}
        transform = document["map_to_morai_map"]
        self.translation = tuple(float(value) for value in transform["xyz"])
        rpy = tuple(float(value) for value in transform["rpy"])
        if len(self.translation) != 3 or len(rpy) != 3:
            raise ValueError("map_to_morai_map xyz/rpy must contain three values")
        if abs(rpy[0]) > 1.0e-9 or abs(rpy[1]) > 1.0e-9:
            raise ValueError("GT BBox visualizer supports a yaw-only transform")
        self.transform_yaw = rpy[2]
        self.line_width = float(rospy.get_param("~line_width_m", 0.09))
        self.lifetime = rospy.Duration(
            float(rospy.get_param("~marker_lifetime_sec", 0.5))
        )
        rgba = rospy.get_param("~color_rgba", [1.0, 0.0, 0.85, 1.0])
        if len(rgba) != 4:
            raise ValueError("~color_rgba must contain RGBA")
        self.color = ColorRGBA(*(float(value) for value in rgba))

        input_topic = rospy.get_param("~input", "/Object_topic")
        output_topic = rospy.get_param(
            "~output", "/evaluation/visualization/gt_boxes"
        )
        self.publisher = rospy.Publisher(output_topic, MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[morai_gt_bbox_visualization] %s -> %s, color=magenta, frame=map",
            input_topic,
            output_topic,
        )

    def transform_center(self, status, class_name):
        cosine = math.cos(self.transform_yaw)
        sine = math.sin(self.transform_yaw)
        x = cosine * status.position.x - sine * status.position.y + self.translation[0]
        y = sine * status.position.x + cosine * status.position.y + self.translation[1]
        z = status.position.z + self.translation[2]
        yaw = quaternion_yaw(status.heading, self.transform_yaw)
        if class_name == "vehicle":
            offset = vehicle_bbox_offset(
                float(status.size.x), float(status.size.y), float(status.size.z)
            )
            x += offset * math.cos(yaw)
            y += offset * math.sin(yaw)
        return x, y, z, yaw

    def make_box(self, header, status, class_name, marker_id):
        x, y, z, yaw = self.transform_center(status, class_name)
        corners = box_corners(
            (x, y, z),
            (float(status.size.x), float(status.size.y), float(status.size.z)),
            yaw,
        )
        marker = Marker()
        marker.header = copy.deepcopy(header)
        marker.ns = "morai_gt_boxes"
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color = copy.deepcopy(self.color)
        marker.lifetime = self.lifetime
        for first, second in BOX_EDGES:
            marker.points.extend((corners[first], corners[second]))
        return marker

    def callback(self, message):
        header = copy.deepcopy(message.header)
        header.frame_id = "map"
        if header.stamp.secs == 0 and header.stamp.nsecs == 0:
            header.stamp = rospy.Time.now()
        output = MarkerArray()
        clear = Marker()
        clear.header = copy.deepcopy(header)
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        groups = (
            ("vehicle", message.npc_list),
            ("pedestrian", message.pedestrian_list),
            ("obstacle", message.obstacle_list),
        )
        marker_id = 0
        for class_name, statuses in groups:
            for status in statuses:
                output.markers.append(
                    self.make_box(header, status, class_name, marker_id)
                )
                marker_id += 1
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gt_bbox_visualization")
    MoraiGtBboxVisualization()
    rospy.spin()


if __name__ == "__main__":
    main()
