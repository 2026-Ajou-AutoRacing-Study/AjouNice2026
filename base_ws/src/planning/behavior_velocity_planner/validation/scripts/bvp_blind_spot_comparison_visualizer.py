#!/usr/bin/env python3

import json
import math

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from behavior_velocity_planner_common import BehaviorVelocityMap
from behavior_velocity_planner_common.geometry import (
    cumulative_lengths,
    first_polyline_intersection,
    point_at_station,
    project_point_to_polyline,
)


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


class BlindSpotComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.report = read_json(rospy.get_param("~report_json"))
        self.display_offset = float(rospy.get_param("~display_offset", 1.2))
        scenario = self.reference["scenario"]
        self.target_id = int(scenario["target_lanelet_id"])
        self.turn_direction = str(scenario["turn_direction"])
        self.lanelet_map = BehaviorVelocityMap.load(
            rospy.get_param("~map_file"),
            rospy.get_param("~projector_file", ""),
            rospy.get_param("~map_projector_type", "MGRS"),
            False,
        )
        self.target = self.lanelet_map.road_lanelets[self.target_id]
        self.path_xyz = [
            (
                float(item["pose"]["position"]["x"]),
                float(item["pose"]["position"]["y"]),
                float(item["pose"]["position"]["z"]),
            )
            for item in self.reference["input_path"]["points"]
        ]
        self.attention_area, self.conflict_area = self.module_areas()
        self.maximum_velocity = max(
            float(item["longitudinal_velocity_mps"])
            for document in (self.reference, self.candidate)
            for item in document["output_path"]["points"]
        )

        self.context_pub = rospy.Publisher(
            "/validation/bvp_blind_spot/context",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros2_pub = rospy.Publisher(
            "/validation/bvp_blind_spot/ros2_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.ros1_pub = rospy.Publisher(
            "/validation/bvp_blind_spot/ros1_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.context_pub.publish(self.context_markers())
        self.ros2_pub.publish(
            self.speed_markers(
                self.reference,
                "ros2",
                self.display_offset,
                color(*ROS2_COLOR),
                "ROS2 PLANNING SIM",
            )
        )
        self.ros1_pub.publish(
            self.speed_markers(
                self.candidate,
                "ros1",
                -self.display_offset,
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

    def polygon(self, namespace, marker_id, polygon, marker_color, z_offset=0.18):
        result = self.marker(namespace, marker_id, Marker.TRIANGLE_LIST)
        result.color = marker_color
        if len(polygon) < 3:
            return result
        anchor = polygon[0]
        for index in range(1, len(polygon) - 1):
            result.points.extend(
                [
                    point(anchor, z_offset),
                    point(polygon[index], z_offset),
                    point(polygon[index + 1], z_offset),
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
        source = self.document_points(document, "output_path", 0.48)
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
            return color(1.0, 0.08 + 0.84 * blend, 0.04)
        blend = (ratio - 0.5) * 2.0
        return color(1.0 - blend, 0.92, 0.04 + 0.96 * blend)

    def speed_markers(self, document, namespace, offset, source_color, label):
        markers = MarkerArray()
        shifted = self.offset_points(document, offset)
        markers.markers.append(
            self.line(namespace + "_outline", 0, shifted, 1.20, source_color)
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

        summary = document["summary"]
        if summary["stop_active"]:
            index = max(0, min(int(summary["first_zero_index"]), len(shifted) - 1))
            stop = self.marker(namespace + "_stop", 0, Marker.SPHERE)
            stop.pose.position = shifted[index]
            stop.pose.position.z += 0.55
            stop.scale.x = stop.scale.y = stop.scale.z = 1.0
            stop.color = source_color
            markers.markers.append(stop)
            result_text = "STOP {:.3f} m".format(
                float(summary["first_zero_station_m"])
            )
        else:
            result_text = "SAFE / GO"
        label_index = min(4 if offset > 0.0 else 10, len(shifted) - 1)
        label_location = self.offset_points(document, offset * 4.5)[label_index]
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

    def module_areas(self):
        lengths = cumulative_lengths(self.path_xyz)
        points = self.reference["input_path"]["points"]
        lane_indices = [
            index
            for index, item in enumerate(points)
            if self.target_id in item.get("lane_ids", [])
        ]
        entry = first_polyline_intersection(
            self.path_xyz, (self.target.left_bound[0], self.target.right_bound[0])
        )
        exit_point = first_polyline_intersection(
            self.path_xyz, (self.target.left_bound[-1], self.target.right_bound[-1])
        )
        entry_station = (
            float(entry["station"])
            if entry is not None
            else float(lengths[max(lane_indices[0] - 1, 0)])
        )
        exit_station = (
            float(exit_point["station"])
            if exit_point is not None
            else float(lengths[min(lane_indices[-1] + 1, len(lengths) - 1)])
        )
        side = (
            self.target.left_bound
            if self.turn_direction == "left"
            else self.target.right_bound
        )
        widths = []
        for value in side:
            projection = project_point_to_polyline(self.target.centerline, value)
            if projection is not None:
                widths.append(float(projection[1]))
        far_offset = max(sum(widths) / len(widths), 1.60) if widths else 1.75
        sign = 1.0 if self.turn_direction == "left" else -1.0
        attention = self.offset_strip(
            max(0.0, entry_station - 100.0),
            exit_station,
            sign * 0.95,
            sign * far_offset,
        )
        conflict = self.offset_strip(entry_station, exit_station, -0.95, 0.95)
        return attention, conflict

    def offset_strip(self, start_station, end_station, first_offset, second_offset):
        lengths = cumulative_lengths(self.path_xyz)
        stations = [float(start_station)]
        stations.extend(value for value in lengths if start_station < value < end_station)
        stations.append(float(end_station))

        def shifted(station, offset):
            current = point_at_station(self.path_xyz, station)
            before = point_at_station(self.path_xyz, max(station - 0.25, 0.0))
            after = point_at_station(
                self.path_xyz, min(station + 0.25, lengths[-1])
            )
            dx = float(after[0]) - float(before[0])
            dy = float(after[1]) - float(before[1])
            norm = max(math.hypot(dx, dy), 1.0e-9)
            return (
                float(current[0]) - dy / norm * float(offset),
                float(current[1]) + dx / norm * float(offset),
                float(current[2]),
            )

        first = [shifted(station, first_offset) for station in stations]
        second = [shifted(station, second_offset) for station in stations]
        return tuple(first + list(reversed(second)))

    def object_markers(self):
        markers = []
        objects = (self.reference.get("dynamic_objects") or {}).get("objects", [])
        for index, detected in enumerate(objects):
            pose = detected["pose"]
            dimensions = detected["shape"]["dimensions"]
            body = self.marker("scenario_objects", index, Marker.CUBE)
            body.pose.position = point(
                pose["position"], 0.5 * float(dimensions["z"])
            )
            orientation = pose["orientation"]
            body.pose.orientation.x = float(orientation["x"])
            body.pose.orientation.y = float(orientation["y"])
            body.pose.orientation.z = float(orientation["z"])
            body.pose.orientation.w = float(orientation["w"])
            body.scale.x = float(dimensions["x"])
            body.scale.y = float(dimensions["y"])
            body.scale.z = float(dimensions["z"])
            body.color = color(1.0, 0.1, 0.05, 0.90)
            markers.append(body)
            predicted_paths = detected.get("predicted_paths", [])
            if predicted_paths:
                markers.append(
                    self.line(
                        "object_predictions",
                        index,
                        [
                            point(item["position"], 0.85)
                            for item in predicted_paths[0]["poses"]
                        ],
                        0.35,
                        color(1.0, 0.48, 0.02),
                    )
                )
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

    def context_markers(self):
        markers = MarkerArray()
        markers.markers.append(
            self.line(
                "input_path",
                0,
                self.document_points(self.reference, "input_path", 0.12),
                0.18,
                color(0.75, 0.78, 0.82, 0.7),
            )
        )
        markers.markers.extend(
            [
                self.polygon(
                    "target_turn_lanelet",
                    0,
                    self.target.polygon,
                    color(0.10, 1.0, 0.30, 0.24),
                    0.20,
                ),
                self.polygon(
                    "blind_attention_area",
                    0,
                    self.attention_area,
                    color(0.10, 0.48, 1.0, 0.30),
                    0.28,
                ),
                self.polygon(
                    "ego_conflict_area",
                    0,
                    self.conflict_area,
                    color(1.0, 0.28, 0.02, 0.27),
                    0.32,
                ),
                self.line(
                    "target_centerline",
                    0,
                    [point(value, 0.38) for value in self.target.centerline],
                    0.32,
                    color(0.20, 1.0, 0.42),
                ),
            ]
        )
        markers.markers.extend(self.object_markers())
        markers.markers.append(self.ego_marker())
        center = self.target.centerline[len(self.target.centerline) // 2]
        markers.markers.append(
            self.text(
                "target_label",
                0,
                "BLIND SPOT {} | {} TURN".format(
                    self.target_id, self.turn_direction.upper()
                ),
                center,
                color(0.30, 1.0, 0.48),
            )
        )

        diagnostics = self.report.get("diagnostics", {})
        stop_error = diagnostics.get("stop_station_error_m")
        stop_text = "n/a" if stop_error is None else "{:.3f} m".format(float(stop_error))
        markers.markers.append(
            self.text(
                "comparison_summary",
                0,
                (
                    "{} | {} | ROS2={} ROS1={} | stop error={} | velocity RMSE={:.3f} m/s\n"
                    "blue=attention, orange=conflict, speed red=0 yellow=mid cyan=max"
                ).format(
                    "PASS" if self.report.get("passed", False) else "FAIL",
                    self.reference["scenario"]["case"].upper(),
                    diagnostics.get("reference_decision_kind", "safe"),
                    diagnostics.get("candidate_decision_kind", "safe"),
                    stop_text,
                    float(diagnostics.get("velocity_rmse_mps", math.inf)),
                ),
                (center[0] + 13.0, center[1] - 12.0, center[2]),
                color(
                    *(PASS_COLOR if self.report.get("passed", False) else FAIL_COLOR)
                ),
            )
        )
        return markers


def main():
    rospy.init_node("bvp_blind_spot_comparison_visualizer")
    BlindSpotComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
