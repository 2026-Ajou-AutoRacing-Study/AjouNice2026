#!/usr/bin/env python3

import json
import math

import rospy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def read_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


def point(value, z_offset=0.0):
    return Point(
        x=float(value["x"]),
        y=float(value["y"]),
        z=float(value.get("z", 0.0)) + z_offset,
    )


def nearest_point_on_polyline(position, polyline):
    best = None
    for first, second in zip(polyline[:-1], polyline[1:]):
        dx = second.x - first.x
        dy = second.y - first.y
        length_squared = dx * dx + dy * dy
        ratio = 0.0
        if length_squared > 1.0e-12:
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((position.x - first.x) * dx + (position.y - first.y) * dy)
                    / length_squared,
                ),
            )
        projected = Point(
            x=first.x + ratio * dx,
            y=first.y + ratio * dy,
            z=position.z,
        )
        distance = math.hypot(position.x - projected.x, position.y - projected.y)
        if best is None or distance < best[0]:
            best = (distance, projected)
    return best if best is not None else (math.inf, Point())


class ComparisonVisualizer:
    def __init__(self):
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.frame_id = self.reference["output_trajectory"].get("frame_id", "map")
        self.candidate = None
        self.report = None
        self.publisher = rospy.Publisher(
            "/validation/path_optimizer/markers", MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
            Trajectory,
            self.on_candidate,
            queue_size=1,
        )
        rospy.Subscriber(
            "/validation/path_optimizer/result", String, self.on_result, queue_size=1
        )

    def marker(self, namespace, marker_id, marker_type):
        value = Marker()
        value.header.frame_id = self.frame_id
        value.header.stamp = rospy.Time.now()
        value.ns = namespace
        value.id = marker_id
        value.type = marker_type
        value.action = Marker.ADD
        value.pose.orientation.w = 1.0
        return value

    def line(self, namespace, marker_id, points, width, marker_color):
        value = self.marker(namespace, marker_id, Marker.LINE_STRIP)
        value.points = points
        value.scale.x = width
        value.color = marker_color
        return value

    def point_list(self, namespace, marker_id, points, diameter, marker_color):
        value = self.marker(namespace, marker_id, Marker.SPHERE_LIST)
        value.points = points
        value.scale.x = diameter
        value.scale.y = diameter
        value.scale.z = diameter
        value.color = marker_color
        return value

    def sphere(self, namespace, marker_id, position, diameter, marker_color):
        value = self.marker(namespace, marker_id, Marker.SPHERE)
        value.pose.position = position
        value.scale.x = value.scale.y = value.scale.z = diameter
        value.color = marker_color
        return value

    def text(self, label, position, marker_color):
        value = self.marker("summary", 0, Marker.TEXT_VIEW_FACING)
        value.pose.position = position
        value.pose.position.z += 3.0
        value.scale.z = 0.72
        value.color = marker_color
        value.text = label
        return value

    @staticmethod
    def document_points(document, z_offset):
        return [point(item["pose"]["position"], z_offset) for item in document["points"]]

    @staticmethod
    def message_points(message, z_offset):
        return [
            Point(x=item.pose.position.x, y=item.pose.position.y, z=item.pose.position.z + z_offset)
            for item in message.points
        ]

    @staticmethod
    def first_stop_document(document):
        for item in document["points"]:
            if abs(float(item["longitudinal_velocity_mps"])) < 1.0e-3:
                return point(item["pose"]["position"], 0.9)
        return None

    @staticmethod
    def first_stop_message(message):
        for item in message.points:
            if abs(float(item.longitudinal_velocity_mps)) < 1.0e-3:
                return Point(x=item.pose.position.x, y=item.pose.position.y, z=item.pose.position.z + 0.9)
        return None

    def on_candidate(self, message):
        if message.points:
            self.candidate = message
            self.publish()

    def on_result(self, message):
        try:
            self.report = json.loads(message.data)
            self.publish()
        except (TypeError, ValueError):
            rospy.logwarn("path optimizer visualizer: invalid result JSON")

    def publish(self):
        if self.candidate is None or rospy.is_shutdown():
            return
        markers = MarkerArray()
        input_points = self.document_points(self.reference["input_path"], 0.12)
        reference_points = self.document_points(self.reference["output_trajectory"], 0.20)
        candidate_points = self.message_points(self.candidate, 0.28)
        cyan = color(0.0, 0.85, 1.0, 0.98)
        magenta = color(1.0, 0.0, 0.82, 0.98)
        markers.markers.append(
            self.point_list(
                "path_smoother_input_points",
                0,
                input_points,
                0.42,
                color(0.75, 0.75, 0.78, 0.82),
            )
        )
        markers.markers.append(self.line("left_bound", 0, [point(v, 0.12) for v in self.reference["input_path"]["left_bound"]], 0.16, color(0.2, 1.0, 0.35, 0.9)))
        markers.markers.append(self.line("right_bound", 0, [point(v, 0.12) for v in self.reference["input_path"]["right_bound"]], 0.16, color(1.0, 0.72, 0.1, 0.9)))
        markers.markers.append(
            self.point_list("ros2_optimizer_points", 0, reference_points, 0.28, cyan)
        )
        markers.markers.append(
            self.point_list("ros1_optimizer_points", 0, candidate_points, 0.14, magenta)
        )

        effects = [
            (*nearest_point_on_polyline(position, input_points), position)
            for position in reference_points
        ]
        maximum_effect, smoother_position, optimizer_position = max(
            effects, key=lambda value: value[0]
        )
        candidate_effects = [
            (*nearest_point_on_polyline(position, input_points), position)
            for position in candidate_points
        ]
        maximum_candidate_effect, _, candidate_effect_position = max(
            candidate_effects, key=lambda value: value[0]
        )
        markers.markers.append(
            self.sphere(
                "strongest_smoother_input",
                0,
                smoother_position,
                0.82,
                color(1.0, 0.55, 0.0, 0.42),
            )
        )
        markers.markers.append(
            self.sphere(
                "strongest_optimizer_output",
                0,
                optimizer_position,
                0.52,
                color(1.0, 1.0, 0.0, 0.92),
            )
        )
        markers.markers.append(
            self.sphere(
                "strongest_ros1_optimizer_output",
                0,
                candidate_effect_position,
                0.34,
                color(1.0, 0.05, 0.45, 1.0),
            )
        )
        reference_stop = self.first_stop_document(self.reference["output_trajectory"])
        candidate_stop = self.first_stop_message(self.candidate)
        if reference_stop is not None:
            markers.markers.append(self.sphere("ros2_stop", 0, reference_stop, 1.1, cyan))
        if candidate_stop is not None:
            markers.markers.append(self.sphere("ros1_stop", 0, candidate_stop, 0.65, magenta))
        ego = self.reference["scenario"]["odometry"]["pose"]
        ego_marker = self.marker("ego", 0, Marker.ARROW)
        ego_marker.pose.position = point(ego["position"], 0.8)
        for name in ("x", "y", "z", "w"):
            setattr(ego_marker.pose.orientation, name, float(ego["orientation"].get(name, 1.0 if name == "w" else 0.0)))
        ego_marker.scale.x = 4.5
        ego_marker.scale.y = 1.7
        ego_marker.scale.z = 1.0
        ego_marker.color = color(1.0, 1.0, 1.0, 0.95)
        markers.markers.append(ego_marker)
        label_point = optimizer_position
        if self.report is None:
            label = "SMOOTHER GRAY / ROS2 CYAN / ROS1 MAGENTA | CHANGE {:.3f} / {:.3f} m".format(
                maximum_effect, maximum_candidate_effect
            )
            label_color = color(1.0, 1.0, 1.0, 1.0)
        else:
            diagnostics = self.report["diagnostics"]
            label = "{} | change ROS2 {:.3f} / ROS1 {:.3f} m | strong RMSE {:.3f} m | max {:.3f} m".format(
                "PASS" if self.report["passed"] else "FAIL",
                diagnostics["reference_input_to_output_max_m"],
                diagnostics["candidate_input_to_output_max_m"],
                diagnostics["strong_effect_geometry_rmse_m"],
                diagnostics["strong_effect_geometry_max_error_m"],
            )
            label_color = color(0.15, 1.0, 0.35, 1.0) if self.report["passed"] else color(1.0, 0.2, 0.1, 1.0)
        markers.markers.append(self.text(label, label_point, label_color))
        if not rospy.is_shutdown():
            self.publisher.publish(markers)


def main():
    rospy.init_node("path_optimizer_comparison_visualizer")
    ComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
