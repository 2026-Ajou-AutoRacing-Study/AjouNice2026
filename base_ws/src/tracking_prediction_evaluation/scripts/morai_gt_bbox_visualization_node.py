#!/usr/bin/env python3
"""Render MORAI ObjectStatus ground-truth boxes as map-frame wireframes."""

from __future__ import annotations

import copy
import json
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
        self.flatten_z = bool(rospy.get_param("~flatten_z", False))
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.lifetime = rospy.Duration(
            float(rospy.get_param("~marker_lifetime_sec", 0.5))
        )
        rgba = rospy.get_param("~color_rgba", [1.0, 0.0, 0.85, 1.0])
        if len(rgba) != 4:
            raise ValueError("~color_rgba must contain RGBA")
        self.color = ColorRGBA(*(float(value) for value in rgba))
        raw_rgba = rospy.get_param(
            "~raw_comparison_color_rgba", [1.0, 0.0, 0.85, 0.8]
        )
        if len(raw_rgba) != 4:
            raise ValueError("~raw_comparison_color_rgba must contain RGBA")
        self.raw_comparison_color = ColorRGBA(
            *(float(value) for value in raw_rgba)
        )
        self.show_raw_comparison = bool(
            rospy.get_param("~show_raw_comparison", False)
        )
        catalog_path = str(rospy.get_param("~asset_bbox_catalog", "")).strip()
        self.asset_catalog = {}
        if catalog_path:
            catalog_file = Path(catalog_path).expanduser()
            catalog = json.loads(catalog_file.read_text(encoding="utf-8"))
            self.asset_catalog = catalog.get("assets", {})
            if not isinstance(self.asset_catalog, dict):
                raise ValueError("asset bbox catalog must contain an assets object")

        input_topic = rospy.get_param("~input", "/Object_topic")
        output_topic = rospy.get_param(
            "~output", "/evaluation/visualization/gt_boxes"
        )
        self.publisher = rospy.Publisher(output_topic, MarkerArray, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, ObjectStatusList, self.callback, queue_size=1
        )
        rospy.loginfo(
            "[morai_gt_bbox_visualization] %s -> %s, frame=map, catalog_assets=%d, raw_comparison=%s",
            input_topic,
            output_topic,
            len(self.asset_catalog),
            self.show_raw_comparison,
        )

    def transform_geometry(self, status, class_name, apply_catalog=True):
        cosine = math.cos(self.transform_yaw)
        sine = math.sin(self.transform_yaw)
        x = cosine * status.position.x - sine * status.position.y + self.translation[0]
        y = sine * status.position.x + cosine * status.position.y + self.translation[1]
        z = status.position.z + self.translation[2]
        yaw = quaternion_yaw(status.heading, self.transform_yaw)
        dimensions = (
            float(status.size.x), float(status.size.y), float(status.size.z)
        )
        asset = self.asset_catalog.get(str(status.name)) if apply_catalog else None
        if asset is not None:
            offset = tuple(
                float(value) for value in asset["pivot_to_bbox_center_local_m"]
            )
            dimensions = tuple(float(value) for value in asset["bbox_size_m"])
            x += math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1]
            y += math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1]
            z += offset[2]
        elif class_name == "vehicle":
            offset = vehicle_bbox_offset(*dimensions)
            x += offset * math.cos(yaw)
            y += offset * math.sin(yaw)
        if self.flatten_z:
            # ObjectStatus z is an altitude-like MORAI map coordinate, whereas
            # the production planning visualization intentionally keeps its
            # planar localization at z=0. Place only this debug wireframe on
            # that plane; do not change the source message or planning TF.
            z = self.ground_z + 0.5 * max(0.01, dimensions[2])
        return x, y, z, yaw, dimensions, asset is not None

    def make_box(
        self, header, status, class_name, marker_id, *, apply_catalog, namespace, color
    ):
        x, y, z, yaw, dimensions, corrected = self.transform_geometry(
            status, class_name, apply_catalog=apply_catalog
        )
        corners = box_corners(
            (x, y, z),
            dimensions,
            yaw,
        )
        marker = Marker()
        marker.header = copy.deepcopy(header)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color = copy.deepcopy(color)
        marker.lifetime = self.lifetime
        for first, second in BOX_EDGES:
            marker.points.extend((corners[first], corners[second]))
        return marker, corrected

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
                corrected_marker, corrected = self.make_box(
                    header,
                    status,
                    class_name,
                    marker_id,
                    apply_catalog=True,
                    namespace="morai_gt_boxes",
                    color=self.color,
                )
                output.markers.append(corrected_marker)
                marker_id += 1
                if self.show_raw_comparison and corrected:
                    raw_marker, _ = self.make_box(
                        header,
                        status,
                        class_name,
                        marker_id,
                        apply_catalog=False,
                        namespace="morai_gt_boxes_raw_object_topic",
                        color=self.raw_comparison_color,
                    )
                    output.markers.append(raw_marker)
                    marker_id += 1
        self.publisher.publish(output)


def main():
    rospy.init_node("morai_gt_bbox_visualization")
    MoraiGtBboxVisualization()
    rospy.spin()


if __name__ == "__main__":
    main()
