#!/usr/bin/env python3

import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from behavior_velocity_planner_common import BehaviorVelocityMap


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
    output = Point()
    if isinstance(value, dict):
        output.x = float(value["x"])
        output.y = float(value["y"])
        output.z = float(value.get("z", 0.0)) + float(z_offset)
    else:
        output.x = float(value[0])
        output.y = float(value[1])
        output.z = (float(value[2]) if len(value) > 2 else 0.0) + float(z_offset)
    return output


class PedestrianCrossingComparisonVisualizer:
    def __init__(self):
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.report = read_json(rospy.get_param("~report_json"))
        self.module = self.reference["scenario"]["module"]
        self.crossing_id = int(self.reference["scenario"]["crossing_lanelet_id"])
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.offset = float(rospy.get_param("~display_offset", 1.2))
        self.lanelet_map = BehaviorVelocityMap.load(
            rospy.get_param("~map_file"),
            rospy.get_param("~projector_file", ""),
            rospy.get_param("~map_projector_type", "MGRS"),
            False,
        )
        self.crossing = self.lanelet_map.pedestrian_crossings[self.crossing_id]
        self.maximum_velocity = max(
            float(item["longitudinal_velocity_mps"])
            for document in (self.reference, self.candidate)
            for item in document["output_path"]["points"]
        )
        self.context_pub = rospy.Publisher(
            "/validation/bvp_pedestrian_crossing/context",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros2_pub = rospy.Publisher(
            "/validation/bvp_pedestrian_crossing/ros2_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros1_pub = rospy.Publisher(
            "/validation/bvp_pedestrian_crossing/ros1_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.context_pub.publish(self.context_markers())
        self.ros2_pub.publish(
            self.path_markers(
                self.reference, "ros2", self.offset, color(0.0, 0.85, 1.0), "ROS2 PLANNING SIM"
            )
        )
        self.ros1_pub.publish(
            self.path_markers(
                self.candidate, "ros1", -self.offset, color(1.0, 0.0, 0.95), "ROS1 PORT"
            )
        )

    def marker(self, namespace, marker_id, marker_type):
        value = Marker()
        value.header.frame_id = self.frame_id
        value.header.stamp = rospy.Time.now()
        value.ns = namespace
        value.id = int(marker_id)
        value.type = marker_type
        value.action = Marker.ADD
        value.pose.orientation.w = 1.0
        value.lifetime = rospy.Duration(0)
        return value

    def line(self, namespace, marker_id, points, width, marker_color):
        value = self.marker(namespace, marker_id, Marker.LINE_STRIP)
        value.scale.x = float(width)
        value.color = marker_color
        value.points = points
        return value

    def text(self, namespace, marker_id, message, position, marker_color, height=0.65):
        value = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        value.pose.position = point(position, 2.8)
        value.scale.z = float(height)
        value.color = marker_color
        value.text = message
        return value

    @staticmethod
    def output_points(document, z_offset=0.0):
        return [
            point(item["pose"]["position"], z_offset)
            for item in document["output_path"]["points"]
        ]

    def offset_points(self, document, offset):
        source = self.output_points(document, 0.45)
        shifted = []
        for index, current in enumerate(source):
            previous = source[max(0, index - 1)]
            following = source[min(len(source) - 1, index + 1)]
            dx = following.x - previous.x
            dy = following.y - previous.y
            norm = math.hypot(dx, dy)
            item = Point(x=current.x, y=current.y, z=current.z)
            if norm > 1.0e-9:
                item.x += float(offset) * (-dy / norm)
                item.y += float(offset) * (dx / norm)
            shifted.append(item)
        return shifted

    def speed_color(self, velocity):
        ratio = min(max(float(velocity) / max(self.maximum_velocity, 1.0e-6), 0.0), 1.0)
        if ratio <= 0.5:
            blend = ratio * 2.0
            return color(1.0, 0.08 + 0.84 * blend, 0.04)
        blend = (ratio - 0.5) * 2.0
        return color(1.0 - blend, 0.92, 0.04 + 0.96 * blend)

    def path_markers(self, document, namespace, offset, source_color, label):
        markers = MarkerArray()
        shifted = self.offset_points(document, offset)
        markers.markers.append(
            self.line(namespace + "_outline", 0, shifted, 1.2, source_color)
        )
        speed = self.marker(namespace + "_speed", 0, Marker.LINE_LIST)
        speed.scale.x = 0.88
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
        stop_index = int(document["summary"]["first_zero_index"])
        stop = self.marker(namespace + "_stop", 0, Marker.SPHERE)
        stop.pose.position = shifted[stop_index]
        stop.pose.position.z += 0.55
        stop.scale.x = stop.scale.y = stop.scale.z = 1.0
        stop.color = source_color
        markers.markers.append(stop)
        label_point = shifted[5 if offset > 0 else 12]
        markers.markers.append(
            self.text(
                namespace + "_label",
                0,
                "{} | stop {:.3f} m".format(
                    label, float(document["summary"]["first_zero_station_m"])
                ),
                {"x": label_point.x, "y": label_point.y, "z": label_point.z},
                source_color,
            )
        )
        return markers

    def crossing_markers(self):
        markers = []
        fill = self.marker("crossing_area", 0, Marker.TRIANGLE_LIST)
        fill.color = (
            color(0.1, 0.95, 1.0, 0.38)
            if self.module == "crosswalk"
            else color(0.2, 1.0, 0.35, 0.38)
        )
        anchor = self.crossing.points[0]
        for index in range(1, len(self.crossing.points) - 1):
            fill.points.extend(
                [
                    point(anchor, 0.18),
                    point(self.crossing.points[index], 0.18),
                    point(self.crossing.points[index + 1], 0.18),
                ]
            )
        markers.append(fill)
        outline = [point(item, 0.30) for item in self.crossing.points]
        outline.append(point(self.crossing.points[0], 0.30))
        markers.append(
            self.line("crossing_outline", 0, outline, 0.35, color(0.2, 1.0, 0.4))
        )
        center = self.crossing.points[len(self.crossing.points) // 2]
        markers.append(
            self.text(
                "crossing_label",
                0,
                "{} {}".format(self.module.upper(), self.crossing_id),
                {"x": center[0], "y": center[1], "z": center[2]},
                color(0.35, 1.0, 0.45),
            )
        )
        return markers

    def object_markers(self):
        objects = (self.reference.get("dynamic_objects") or {}).get("objects", [])
        if not objects:
            return []
        detected = objects[0]
        pose = detected["pose"]
        pedestrian = self.marker("pedestrian", 0, Marker.CYLINDER)
        pedestrian.pose.position = point(pose["position"], 0.9)
        pedestrian.scale.x = pedestrian.scale.y = 0.7
        pedestrian.scale.z = 1.8
        pedestrian.color = color(1.0, 0.15, 0.1)
        predicted = detected["predicted_paths"][0]["poses"]
        path = self.line(
            "pedestrian_prediction",
            0,
            [point(item["position"], 0.75) for item in predicted],
            0.35,
            color(1.0, 0.4, 0.05),
        )
        return [pedestrian, path]

    def context_markers(self):
        markers = MarkerArray()
        input_points = [
            point(item["pose"]["position"], 0.12)
            for item in self.reference["input_path"]["points"]
        ]
        markers.markers.append(
            self.line("input_path", 0, input_points, 0.18, color(0.75, 0.78, 0.82, 0.7))
        )
        markers.markers.extend(self.crossing_markers())
        markers.markers.extend(self.object_markers())
        odometry_pose = self.reference["scenario"]["odometry"]["pose"]
        ego = self.marker("ego", 0, Marker.ARROW)
        ego.pose.position = point(odometry_pose["position"], 0.6)
        orientation = odometry_pose["orientation"]
        ego.pose.orientation.x = float(orientation["x"])
        ego.pose.orientation.y = float(orientation["y"])
        ego.pose.orientation.z = float(orientation["z"])
        ego.pose.orientation.w = float(orientation["w"])
        ego.scale.x, ego.scale.y, ego.scale.z = 4.6, 1.7, 1.0
        ego.color = color(1.0, 1.0, 1.0)
        markers.markers.append(ego)
        diagnostics = self.report["diagnostics"]
        center = self.crossing.points[0]
        markers.markers.append(
            self.text(
                "summary",
                0,
                "{} | stop error {:.3f} m | velocity RMSE {:.3f} m/s\nred=0, yellow=mid, cyan=max speed".format(
                    "PASS" if self.report["passed"] else "FAIL",
                    float(diagnostics["stop_station_error_m"]),
                    float(diagnostics["velocity_rmse_mps"]),
                ),
                {"x": center[0] - 8.0, "y": center[1] - 5.0, "z": center[2]},
                color(0.25, 1.0, 0.45) if self.report["passed"] else color(1.0, 0.1, 0.1),
            )
        )
        return markers


def main():
    rospy.init_node("bvp_pedestrian_crossing_comparison_visualizer")
    PedestrianCrossingComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
