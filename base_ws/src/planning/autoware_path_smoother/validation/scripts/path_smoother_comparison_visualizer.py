#!/usr/bin/env python3

import json

import rospy
from autoware_planning_msgs.msg import Path
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


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


def json_point(value, z_offset=0.0):
    point = Point()
    point.x = float(value["x"])
    point.y = float(value["y"])
    point.z = float(value.get("z", 0.0)) + float(z_offset)
    return point


class ComparisonVisualizer:
    def __init__(self):
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.frame_id = self.reference["output_path"].get("frame_id", "map")
        self.report = None
        self.candidate = None
        self.publisher = rospy.Publisher(
            "/validation/path_smoother/markers", MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/path_smoother/path",
            Path,
            self.on_candidate,
            queue_size=1,
        )
        rospy.Subscriber(
            "/validation/path_smoother/result", String, self.on_result, queue_size=1
        )

    def marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0)
        return marker

    def point_list(self, namespace, marker_id, points, diameter, value):
        marker = self.marker(namespace, marker_id, Marker.SPHERE_LIST)
        marker.points = points
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        marker.color = value
        return marker

    def sphere(self, namespace, marker_id, point, diameter, value):
        marker = self.marker(namespace, marker_id, Marker.SPHERE)
        marker.pose.position = point
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color = value
        return marker

    def text(self, namespace, marker_id, label, point, value):
        marker = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position = point
        marker.pose.position.z += 3.0
        marker.scale.z = 0.72
        marker.color = value
        marker.text = label
        return marker

    @staticmethod
    def document_points(document, z_offset=0.0):
        return [
            json_point(item["pose"]["position"], z_offset)
            for item in document["points"]
        ]

    @staticmethod
    def message_points(message, z_offset=0.0):
        return [
            Point(
                x=item.pose.position.x,
                y=item.pose.position.y,
                z=item.pose.position.z + z_offset,
            )
            for item in message.points
        ]

    @staticmethod
    def first_stop_document(document):
        for item in document["points"]:
            if abs(float(item["longitudinal_velocity_mps"])) < 1.0e-3:
                return json_point(item["pose"]["position"], 0.9)
        return None

    @staticmethod
    def first_stop_message(message):
        for item in message.points:
            if abs(float(item.longitudinal_velocity_mps)) < 1.0e-3:
                return Point(
                    x=item.pose.position.x,
                    y=item.pose.position.y,
                    z=item.pose.position.z + 0.9,
                )
        return None

    def on_result(self, message):
        try:
            self.report = json.loads(message.data)
            self.publish()
        except (TypeError, ValueError):
            rospy.logwarn("path_smoother visualizer: invalid result JSON")

    def on_candidate(self, message):
        if not message.points:
            return
        self.candidate = message
        self.publish()

    def publish(self):
        if self.candidate is None:
            return
        markers = MarkerArray()
        input_points = self.document_points(self.reference["input_path"], 0.12)
        reference_points = self.document_points(self.reference["output_path"], 0.20)
        candidate_points = self.message_points(self.candidate, 0.28)
        cyan = color(0.0, 0.85, 1.0, 0.98)
        magenta = color(1.0, 0.0, 0.82, 0.98)
        markers.markers.append(
            self.point_list(
                "bvp_input_points",
                0,
                input_points,
                0.42,
                color(0.72, 0.75, 0.8, 0.82),
            )
        )
        markers.markers.append(
            self.point_list("ros2_smoother_points", 0, reference_points, 0.28, cyan)
        )
        markers.markers.append(
            self.point_list("ros1_smoother_points", 0, candidate_points, 0.14, magenta)
        )
        reference_stop = self.first_stop_document(self.reference["output_path"])
        candidate_stop = self.first_stop_message(self.candidate)
        if reference_stop is not None:
            markers.markers.append(
                self.sphere("ros2_stop", 0, reference_stop, 1.1, color(0.0, 1.0, 1.0, 1.0))
            )
        if candidate_stop is not None:
            markers.markers.append(
                self.sphere("ros1_stop", 0, candidate_stop, 0.65, color(1.0, 0.0, 0.82, 1.0))
            )
        ego = self.reference["scenario"]["odometry"]["pose"]
        ego_marker = self.marker("ego", 0, Marker.ARROW)
        ego_marker.pose.position = json_point(ego["position"], 0.8)
        orientation = ego["orientation"]
        ego_marker.pose.orientation.x = orientation["x"]
        ego_marker.pose.orientation.y = orientation["y"]
        ego_marker.pose.orientation.z = orientation["z"]
        ego_marker.pose.orientation.w = orientation["w"]
        ego_marker.scale.x = 4.5
        ego_marker.scale.y = 1.7
        ego_marker.scale.z = 1.0
        ego_marker.color = color(1.0, 1.0, 1.0, 0.95)
        markers.markers.append(ego_marker)
        label_point = Point(
            x=reference_points[min(12, len(reference_points) - 1)].x,
            y=reference_points[min(12, len(reference_points) - 1)].y,
            z=reference_points[min(12, len(reference_points) - 1)].z,
        )
        if self.report is None:
            label = "BVP GRAY | ROS2 SMOOTHER CYAN | ROS1 SMOOTHER MAGENTA"
            status_color = color(1.0, 1.0, 1.0, 1.0)
        else:
            diagnostics = self.report["diagnostics"]
            label = (
                "{} | BVP GRAY / ROS2 CYAN / ROS1 MAGENTA | RMSE {:.3f} m | "
                "max {:.3f} m | stop {:.3f} m"
            ).format(
                "PASS" if self.report["passed"] else "FAIL",
                diagnostics["geometry_rmse_m"],
                diagnostics["geometry_max_error_m"],
                diagnostics["first_stop_pose_error_m"],
            )
            status_color = (
                color(0.15, 1.0, 0.35, 1.0)
                if self.report["passed"]
                else color(1.0, 0.2, 0.1, 1.0)
            )
        markers.markers.append(self.text("summary", 0, label, label_point, status_color))
        self.publisher.publish(markers)


def main():
    rospy.init_node("path_smoother_comparison_visualizer")
    ComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
