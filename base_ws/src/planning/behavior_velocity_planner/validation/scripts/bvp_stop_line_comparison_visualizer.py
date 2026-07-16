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
INPUT_COLOR = (0.72, 0.75, 0.80, 0.65)
STOP_LINE_COLOR = (1.0, 0.55, 0.0, 1.0)
PASS_COLOR = (0.25, 1.0, 0.45, 1.0)
FAIL_COLOR = (1.0, 0.18, 0.12, 1.0)


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
    output.x = float(value["x"])
    output.y = float(value["y"])
    output.z = float(value["z"]) + float(z_offset)
    return output


def pose_position(document, key):
    return document["summary"][key]["position"]


class StopLineComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.report = read_json(rospy.get_param("~report_json"))
        self.map_file = rospy.get_param("~map_file")
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.stop_line_id = int(self.reference["scenario"]["stop_line_id"])
        self.display_offset = float(rospy.get_param("~display_offset", 1.2))
        self.maximum_velocity = max(
            float(point_value["longitudinal_velocity_mps"])
            for document in (self.reference, self.candidate)
            for point_value in document["output_path"]["points"]
        )
        self.context_publisher = rospy.Publisher(
            rospy.get_param("~context_topic", "/validation/bvp_stop_line/context"),
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros2_publisher = rospy.Publisher(
            rospy.get_param(
                "~ros2_speed_topic", "/validation/bvp_stop_line/ros2_speed_path"
            ),
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros1_publisher = rospy.Publisher(
            rospy.get_param(
                "~ros1_speed_topic", "/validation/bvp_stop_line/ros1_speed_path"
            ),
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.lanelet_map = BehaviorVelocityMap.load(
            self.map_file,
            self.projector_file,
            self.projector_type,
            False,
        )
        self.context_publisher.publish(self.build_context_markers())
        self.ros2_publisher.publish(
            self.build_speed_markers(
                self.reference,
                "ros2",
                self.display_offset,
                color(*ROS2_COLOR),
                "ROS2 PLANNING SIM",
            )
        )
        self.ros1_publisher.publish(
            self.build_speed_markers(
                self.candidate,
                "ros1",
                -self.display_offset,
                color(*ROS1_COLOR),
                "ROS1 PORT",
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

    def make_line(self, namespace, marker_id, points, width, marker_color):
        marker = self.marker(namespace, marker_id, Marker.LINE_STRIP)
        marker.scale.x = float(width)
        marker.color = marker_color
        marker.points = points
        return marker

    def path_points(self, document, key, z_offset):
        return [
            point(item["pose"]["position"], z_offset)
            for item in document[key]["points"]
        ]

    def offset_path_points(self, document, offset, z_offset):
        source = self.path_points(document, "output_path", z_offset)
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
        ratio = min(
            max(float(velocity) / max(self.maximum_velocity, 1.0e-6), 0.0),
            1.0,
        )
        if ratio <= 0.5:
            blend = ratio * 2.0
            return color(1.0, 0.08 + 0.84 * blend, 0.04, 1.0)
        blend = (ratio - 0.5) * 2.0
        return color(1.0 - blend, 0.92, 0.04 + 0.96 * blend, 1.0)

    def make_speed_path(self, document, namespace, shifted_points):
        marker = self.marker(namespace + "_speed_path", 0, Marker.LINE_LIST)
        marker.scale.x = 0.72
        values = document["output_path"]["points"]
        for index, (first, second) in enumerate(
            zip(shifted_points[:-1], shifted_points[1:])
        ):
            marker.points.extend([first, second])
            marker.colors.extend(
                [
                    self.speed_color(values[index]["longitudinal_velocity_mps"]),
                    self.speed_color(values[index + 1]["longitudinal_velocity_mps"]),
                ]
            )
        return marker

    def make_stop_line(self):
        stop_line = self.lanelet_map.stop_lines.get(self.stop_line_id)
        if stop_line is None:
            raise ValueError("stop line {} is not in map".format(self.stop_line_id))
        points = []
        for x, y, z in stop_line.points:
            points.append(point({"x": x, "y": y, "z": z}, 0.60))
        return self.make_line(
            "map_stop_line", 0, points, 1.0, color(*STOP_LINE_COLOR)
        )

    def make_stop_marker(self, document, shifted_points, namespace, marker_color):
        stop_index = int(document["summary"]["control_point_path_index"])
        stop_index = max(0, min(stop_index, len(shifted_points) - 1))
        stop_point = shifted_points[stop_index]
        marker = self.marker(namespace, 0, Marker.SPHERE)
        marker.pose.position = Point(
            x=stop_point.x,
            y=stop_point.y,
            z=stop_point.z + 0.55,
        )
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = marker_color
        return marker

    def make_text(self, namespace, marker_id, text, position, marker_color, height=1.0):
        marker = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position = point(position, 3.0)
        marker.scale.z = float(height)
        marker.color = marker_color
        marker.text = text
        return marker

    def make_ego(self):
        pose = self.reference["scenario"]["odometry"]["pose"]
        marker = self.marker("scenario_ego", 0, Marker.ARROW)
        marker.pose.position = point(pose["position"], 0.6)
        orientation = pose["orientation"]
        marker.pose.orientation.x = float(orientation["x"])
        marker.pose.orientation.y = float(orientation["y"])
        marker.pose.orientation.z = float(orientation["z"])
        marker.pose.orientation.w = float(orientation["w"])
        marker.scale.x = 4.6
        marker.scale.y = 1.7
        marker.scale.z = 1.0
        marker.color = color(1.0, 1.0, 1.0, 0.95)
        return marker

    def make_goal(self):
        goal = self.reference["scenario"]["route"]["goal_pose"]
        marker = self.marker("scenario_goal", 0, Marker.SPHERE)
        marker.pose.position = point(goal["position"], 0.7)
        marker.scale.x = marker.scale.y = marker.scale.z = 1.3
        marker.color = color(1.0, 0.85, 0.05, 1.0)
        return marker

    def build_context_markers(self):
        markers = MarkerArray()
        markers.markers.append(
            self.make_line(
                "bpp_input_path",
                0,
                self.path_points(self.reference, "input_path", 0.12),
                0.18,
                color(*INPUT_COLOR),
            )
        )
        markers.markers.append(self.make_stop_line())
        markers.markers.extend([self.make_ego(), self.make_goal()])

        passed = bool(self.report.get("passed", False))
        diagnostics = self.report.get("diagnostics", {})
        summary_position = self.reference["scenario"]["odometry"]["pose"]["position"]
        markers.markers.append(
            self.make_text(
                "comparison_summary",
                0,
                (
                    "{} | SPEED: red 0.0 | yellow {:.2f} | cyan {:.2f} m/s\n"
                    "stop pose error {:.3f} m | station error {:.3f} m | "
                    "velocity RMSE {:.3f} m/s"
                ).format(
                    "PASS" if passed else "FAIL",
                    self.maximum_velocity * 0.5,
                    self.maximum_velocity,
                    float(diagnostics.get("stop_pose_error_m", math.inf)),
                    float(diagnostics.get("stop_station_error_m", math.inf)),
                    float(diagnostics.get("velocity_rmse_mps", math.inf)),
                ),
                summary_position,
                color(*(PASS_COLOR if passed else FAIL_COLOR)),
                1.0,
            )
        )
        stop_position = pose_position(self.reference, "control_point_pose")
        markers.markers.append(
            self.make_text(
                "stop_line_label",
                0,
                "MAP STOP LINE {}".format(self.stop_line_id),
                stop_position,
                color(*STOP_LINE_COLOR),
                0.8,
            )
        )
        return markers

    def build_speed_markers(
        self, document, namespace, offset, source_color, label
    ):
        markers = MarkerArray()
        shifted_points = self.offset_path_points(document, offset, 0.42)
        markers.markers.append(
            self.make_line(
                namespace + "_source_outline",
                0,
                shifted_points,
                1.05,
                source_color,
            )
        )
        markers.markers.append(
            self.make_speed_path(document, namespace, shifted_points)
        )
        markers.markers.append(
            self.make_stop_marker(
                document,
                shifted_points,
                namespace + "_stop_pose",
                source_color,
            )
        )
        label_index = min(8, len(shifted_points) - 1) if offset > 0.0 else 0
        label_point = shifted_points[label_index]
        station = float(document["summary"]["control_point_station_m"])
        markers.markers.append(
            self.make_text(
                namespace + "_label",
                0,
                "{} | {:+.1f} m | stop {:.3f} m".format(
                    label, offset, station
                ),
                {"x": label_point.x, "y": label_point.y, "z": label_point.z},
                source_color,
                0.75,
            )
        )
        return markers


def main():
    rospy.init_node("bvp_stop_line_comparison_visualizer")
    StopLineComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
