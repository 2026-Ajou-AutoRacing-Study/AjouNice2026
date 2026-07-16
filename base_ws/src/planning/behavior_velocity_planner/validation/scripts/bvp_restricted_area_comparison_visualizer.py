#!/usr/bin/env python3

import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from behavior_velocity_planner_common import BehaviorVelocityMap


ROS2_COLOR = (0.0, 0.85, 1.0, 1.0)
ROS1_COLOR = (1.0, 0.0, 0.95, 1.0)
PASS_COLOR = (0.25, 1.0, 0.45, 1.0)
FAIL_COLOR = (1.0, 0.15, 0.10, 1.0)


def read_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


def point(value, z_offset=0.0):
    result = Point()
    if isinstance(value, dict):
        result.x = float(value["x"])
        result.y = float(value["y"])
        result.z = float(value.get("z", 0.0)) + float(z_offset)
    elif hasattr(value, "x") and hasattr(value, "y"):
        result.x = float(value.x)
        result.y = float(value.y)
        result.z = float(getattr(value, "z", 0.0)) + float(z_offset)
    else:
        result.x = float(value[0])
        result.y = float(value[1])
        result.z = (float(value[2]) if len(value) > 2 else 0.0) + float(z_offset)
    return result


class RestrictedAreaComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.module = rospy.get_param("~module")
        self.reference_available = bool(rospy.get_param("~reference_available", True))
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.report = read_json(rospy.get_param("~report_json"))
        self.display_offset = float(rospy.get_param("~display_offset", 1.2))
        self.target_id = int(self.reference["scenario"].get("target_lanelet_id", 200208))
        self.lanelet_map = BehaviorVelocityMap.load(
            rospy.get_param("~map_file"),
            rospy.get_param("~projector_file", ""),
            rospy.get_param("~map_projector_type", "MGRS"),
            False,
        )
        documents = [self.candidate]
        if self.reference_available:
            documents.append(self.reference)
        velocities = [
            float(item["longitudinal_velocity_mps"])
            for document in documents
            for item in document["output_path"]["points"]
        ]
        self.maximum_velocity = max(velocities or [1.0])

        self.context_pub = rospy.Publisher(
            "/validation/bvp_restricted_area/context",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros2_pub = rospy.Publisher(
            "/validation/bvp_restricted_area/ros2_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros1_pub = rospy.Publisher(
            "/validation/bvp_restricted_area/ros1_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.context_pub.publish(self.context_markers())
        if self.reference_available:
            self.ros2_pub.publish(
                self.speed_markers(
                    self.reference,
                    "ros2",
                    self.display_offset,
                    color(*ROS2_COLOR),
                    "ROS2 PLANNING SIM",
                )
            )
        else:
            self.ros2_pub.publish(MarkerArray())
        candidate_offset = -self.display_offset if self.reference_available else 0.0
        self.ros1_pub.publish(
            self.speed_markers(
                self.candidate,
                "ros1",
                candidate_offset,
                color(*ROS1_COLOR),
                "ROS1 PORT",
            )
        )

    def marker(self, namespace, marker_id, marker_type):
        result = Marker()
        result.header.frame_id = self.frame_id
        result.header.stamp = rospy.Time.now()
        result.ns = namespace
        result.id = int(marker_id)
        result.type = marker_type
        result.action = Marker.ADD
        result.pose.orientation.w = 1.0
        result.lifetime = rospy.Duration(0)
        return result

    def line(self, namespace, marker_id, points, width, marker_color, closed=False):
        result = self.marker(namespace, marker_id, Marker.LINE_STRIP)
        result.scale.x = float(width)
        result.color = marker_color
        result.points = list(points)
        if closed and result.points:
            result.points.append(result.points[0])
        return result

    def text(self, namespace, marker_id, message, position, marker_color, height=0.65):
        result = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        result.pose.position = point(position, 3.0)
        result.scale.z = float(height)
        result.color = marker_color
        result.text = str(message)
        return result

    def polygon(self, namespace, marker_id, polygon, marker_color):
        result = self.marker(namespace, marker_id, Marker.TRIANGLE_LIST)
        result.color = marker_color
        if len(polygon) < 3:
            return result
        anchor = polygon[0]
        for index in range(1, len(polygon) - 1):
            result.points.extend(
                [
                    point(anchor, 0.18),
                    point(polygon[index], 0.18),
                    point(polygon[index + 1], 0.18),
                ]
            )
        return result

    @staticmethod
    def document_points(document, key="output_path", z_offset=0.0):
        return [
            point(item["pose"]["position"], z_offset)
            for item in document[key]["points"]
        ]

    def offset_points(self, document, offset):
        source = self.document_points(document, "output_path", 0.45)
        shifted = []
        for index, current in enumerate(source):
            previous = source[max(index - 1, 0)]
            following = source[min(index + 1, len(source) - 1)]
            dx = following.x - previous.x
            dy = following.y - previous.y
            norm = math.hypot(dx, dy)
            result = Point(x=current.x, y=current.y, z=current.z)
            if norm > 1.0e-9:
                result.x += float(offset) * (-dy / norm)
                result.y += float(offset) * (dx / norm)
            shifted.append(result)
        return shifted

    def speed_color(self, velocity):
        ratio = min(max(float(velocity) / max(self.maximum_velocity, 1.0e-6), 0.0), 1.0)
        if ratio <= 0.5:
            blend = ratio * 2.0
            return color(1.0, 0.08 + 0.84 * blend, 0.04)
        blend = (ratio - 0.5) * 2.0
        return color(1.0 - blend, 0.92, 0.04 + 0.96 * blend)

    def speed_markers(self, document, namespace, offset, source_color, label):
        markers = MarkerArray()
        shifted = self.offset_points(document, offset)
        markers.markers.append(
            self.line(namespace + "_outline", 0, shifted, 1.18, source_color)
        )
        speed = self.marker(namespace + "_speed", 0, Marker.LINE_LIST)
        speed.scale.x = 0.86
        values = document["output_path"]["points"]
        for index, (first, second) in enumerate(zip(shifted[:-1], shifted[1:])):
            speed.points.extend([first, second])
            speed.colors.extend(
                [
                    self.speed_color(values[index]["longitudinal_velocity_mps"]),
                    self.speed_color(values[index + 1]["longitudinal_velocity_mps"]),
                ]
            )
        markers.markers.append(speed)

        summary = document["summary"]
        if summary["stop_active"]:
            index = max(0, min(int(summary["first_zero_index"]), len(shifted) - 1))
            stop = self.marker(namespace + "_stop", 0, Marker.SPHERE)
            stop.pose.position = shifted[index]
            stop.pose.position.z += 0.55
            stop.scale.x = stop.scale.y = stop.scale.z = 1.0
            stop.color = source_color
            markers.markers.append(stop)
            result_text = "stop {:.3f} m".format(float(summary["first_zero_station_m"]))
        else:
            result_text = "GO"
        label_index = min(6 if offset >= 0.0 else 12, len(shifted) - 1)
        label_location = self.offset_points(document, offset * 4.0)[label_index]
        markers.markers.append(
            self.text(
                namespace + "_label",
                0,
                "{} | {}".format(label, result_text),
                label_location,
                source_color,
            )
        )
        return markers

    def target_geometry(self):
        if self.module == "no_stopping_area":
            candidates = [
                area
                for area in self.lanelet_map.no_stopping_area_candidates()
                if not area.lane_ids or self.target_id in area.lane_ids
            ]
            if not candidates:
                return (), "NO STOPPING AREA"
            area = candidates[0]
            return area.points, "NO STOPPING AREA {}".format(area.regulatory_id)
        lanelet = self.lanelet_map.road_lanelets[self.target_id]
        return lanelet.polygon, "NO DRIVABLE LANE {}".format(self.target_id)

    def object_markers(self):
        markers = []
        objects = (self.reference.get("dynamic_objects") or {}).get("objects", [])
        for index, detected in enumerate(objects):
            pose = detected["pose"]
            dimensions = detected["shape"]["dimensions"]
            body = self.marker("scenario_objects", index, Marker.CUBE)
            body.pose.position = point(pose["position"], 0.5 * float(dimensions["z"]))
            orientation = pose["orientation"]
            body.pose.orientation.x = float(orientation["x"])
            body.pose.orientation.y = float(orientation["y"])
            body.pose.orientation.z = float(orientation["z"])
            body.pose.orientation.w = float(orientation["w"])
            body.scale.x = float(dimensions["x"])
            body.scale.y = float(dimensions["y"])
            body.scale.z = float(dimensions["z"])
            body.color = color(1.0, 0.08, 0.04, 0.90)
            markers.append(body)
        return markers

    def ego_marker(self):
        pose = self.reference["scenario"]["odometry"]["pose"]
        result = self.marker("scenario_ego", 0, Marker.ARROW)
        result.pose.position = point(pose["position"], 0.6)
        orientation = pose["orientation"]
        result.pose.orientation.x = float(orientation["x"])
        result.pose.orientation.y = float(orientation["y"])
        result.pose.orientation.z = float(orientation["z"])
        result.pose.orientation.w = float(orientation["w"])
        result.scale.x = 4.6
        result.scale.y = 1.7
        result.scale.z = 1.0
        result.color = color(1.0, 1.0, 1.0)
        return result

    def comparison_text(self):
        if not self.reference_available:
            return (
                "ROS1 SEMANTIC PASS | ROS2 REFERENCE BLOCKED: upstream SIGSEGV\n"
                "speed: red=0, yellow=mid, cyan=max"
            )
        diagnostics = self.report.get("diagnostics", {})
        stop_error = diagnostics.get("stop_station_error_m")
        stop_text = "n/a" if stop_error is None else "{:.3f} m".format(float(stop_error))
        return (
            "{} | {} | stop error={} | velocity RMSE={:.4f} m/s\n"
            "speed: red=0, yellow=mid, cyan=max"
        ).format(
            "PASS" if self.report.get("passed", False) else "FAIL",
            self.reference["scenario"]["case"].upper(),
            stop_text,
            float(diagnostics.get("velocity_rmse_mps", math.inf)),
        )

    def context_markers(self):
        markers = MarkerArray()
        markers.markers.append(
            self.line(
                "input_path",
                0,
                self.document_points(self.reference, "input_path", 0.12),
                0.18,
                color(0.75, 0.78, 0.82, 0.70),
            )
        )
        polygon, label = self.target_geometry()
        if polygon:
            area_color = (
                color(1.0, 0.52, 0.02, 0.30)
                if self.module == "no_stopping_area"
                else color(0.95, 0.10, 0.95, 0.26)
            )
            markers.markers.append(self.polygon("restricted_area", 0, polygon, area_color))
            markers.markers.append(
                self.line(
                    "restricted_area_outline",
                    0,
                    [point(value, 0.24) for value in polygon],
                    0.32,
                    color(area_color.r, area_color.g, area_color.b, 0.95),
                    closed=True,
                )
            )
            center = (
                sum(value[0] for value in polygon) / len(polygon),
                sum(value[1] for value in polygon) / len(polygon),
                sum(value[2] for value in polygon) / len(polygon),
            )
            markers.markers.append(
                self.text("restricted_area_label", 0, label, center, color(1.0, 0.75, 0.15))
            )
        else:
            center = self.reference["scenario"]["odometry"]["pose"]["position"]
        markers.markers.extend(self.object_markers())
        markers.markers.append(self.ego_marker())
        markers.markers.append(
            self.text(
                "comparison_summary",
                0,
                self.comparison_text(),
                (float(center[0]) - 10.0, float(center[1]) - 8.0, float(center[2])),
                color(*(PASS_COLOR if self.report.get("passed", False) else FAIL_COLOR)),
            )
        )
        return markers


def main():
    rospy.init_node("bvp_restricted_area_comparison_visualizer")
    RestrictedAreaComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
