#!/usr/bin/env python3

import json
import math
import os

import rospy
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import Path
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


REFERENCE_COLOR = (0.0, 0.85, 1.0, 1.0)
LIVE_COLOR = (1.0, 0.0, 0.95, 1.0)
BPP_COLOR = (0.78, 0.80, 0.84, 0.7)


def read_json(path):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        return json.load(stream)


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


def json_point(value, z_offset=0.0):
    output = Point()
    output.x = float(value["x"])
    output.y = float(value["y"])
    output.z = float(value.get("z", 0.0)) + float(z_offset)
    return output


def ros_point(value, z_offset=0.0):
    output = Point()
    output.x = float(value.x)
    output.y = float(value.y)
    output.z = float(value.z) + float(z_offset)
    return output


def offset_points(points, offset):
    shifted = []
    for index, current in enumerate(points):
        previous = points[max(0, index - 1)]
        following = points[min(len(points) - 1, index + 1)]
        dx = following.x - previous.x
        dy = following.y - previous.y
        norm = math.hypot(dx, dy)
        result = Point(x=current.x, y=current.y, z=current.z)
        if norm > 1.0e-9:
            result.x += float(offset) * (-dy / norm)
            result.y += float(offset) * (dx / norm)
        shifted.append(result)
    return shifted


class MissionBppBvpLiveVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.display_offset = float(rospy.get_param("~display_offset", 1.2))
        self.bpp_path = None
        self.bvp_path = None
        self.result_text = "WAITING FOR PIPELINE"
        self.reference_velocities = [
            float(item["longitudinal_velocity_mps"])
            for item in self.reference["output_path"]["points"]
        ]
        self.maximum_velocity = max(max(self.reference_velocities), 1.0)

        self.context_pub = rospy.Publisher(
            "/validation/mission_bpp_bvp/context", MarkerArray, queue_size=1, latch=True
        )
        self.reference_pub = rospy.Publisher(
            "/validation/mission_bpp_bvp/reference_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.live_pub = rospy.Publisher(
            "/validation/mission_bpp_bvp/live_speed_path",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
            PathWithLaneId,
            self.on_bpp_path,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path",
            Path,
            self.on_bvp_path,
            queue_size=1,
        )
        rospy.Subscriber(
            "/validation/mission_bpp_bvp/result",
            String,
            self.on_result,
            queue_size=1,
        )
        self.reference_pub.publish(self.make_reference_markers())
        self.publish_context()

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

    def text(self, namespace, marker_id, content, position, marker_color, height=0.9):
        value = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        value.pose.position = Point(x=position.x, y=position.y, z=position.z + 3.0)
        value.scale.z = float(height)
        value.color = marker_color
        value.text = content
        return value

    def speed_color(self, velocity):
        ratio = min(max(float(velocity) / self.maximum_velocity, 0.0), 1.0)
        if ratio <= 0.5:
            blend = ratio * 2.0
            return color(1.0, 0.08 + 0.84 * blend, 0.04, 1.0)
        blend = (ratio - 0.5) * 2.0
        return color(1.0 - blend, 0.92, 0.04 + 0.96 * blend, 1.0)

    def speed_line(self, namespace, points, velocities):
        value = self.marker(namespace, 0, Marker.LINE_LIST)
        value.scale.x = 0.72
        for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
            value.points.extend([first, second])
            value.colors.extend(
                [self.speed_color(velocities[index]), self.speed_color(velocities[index + 1])]
            )
        return value

    def reference_points(self):
        return [
            json_point(item["pose"]["position"], 0.45)
            for item in self.reference["output_path"]["points"]
        ]

    def make_reference_markers(self):
        points = offset_points(self.reference_points(), self.display_offset)
        markers = MarkerArray()
        markers.markers.append(
            self.line("ros2_outline", 0, points, 1.02, color(*REFERENCE_COLOR))
        )
        markers.markers.append(
            self.speed_line("ros2_velocity", points, self.reference_velocities)
        )
        label_index = min(5, len(points) - 1)
        markers.markers.append(
            self.text(
                "ros2_label",
                0,
                "ROS2 PLANNING SIM  +{:.1f} m".format(self.display_offset),
                points[label_index],
                color(*REFERENCE_COLOR),
            )
        )
        return markers

    def make_live_markers(self):
        markers = MarkerArray()
        live_points = [ros_point(item.pose.position, 0.45) for item in self.bvp_path.points]
        velocities = [float(item.longitudinal_velocity_mps) for item in self.bvp_path.points]
        self.maximum_velocity = max(
            self.maximum_velocity, max(velocities) if velocities else self.maximum_velocity
        )
        shifted = offset_points(live_points, -self.display_offset)
        markers.markers.append(
            self.line("ros1_outline", 0, shifted, 1.02, color(*LIVE_COLOR))
        )
        markers.markers.append(self.speed_line("ros1_velocity", shifted, velocities))
        label_index = min(5, len(shifted) - 1)
        markers.markers.append(
            self.text(
                "ros1_label",
                0,
                "ROS1 FULL PIPELINE  -{:.1f} m".format(self.display_offset),
                shifted[label_index],
                color(*LIVE_COLOR),
            )
        )
        if self.bpp_path is not None:
            bpp_points = [
                ros_point(item.point.pose.position, 0.22) for item in self.bpp_path.points
            ]
            markers.markers.append(
                self.line("live_bpp_path", 0, bpp_points, 0.18, color(*BPP_COLOR))
            )
        return markers

    def make_arrow(self, namespace, marker_id, pose, marker_color):
        value = self.marker(namespace, marker_id, Marker.ARROW)
        value.pose.position = json_point(pose["position"], 0.7)
        orientation = pose["orientation"]
        value.pose.orientation.x = float(orientation.get("x", 0.0))
        value.pose.orientation.y = float(orientation.get("y", 0.0))
        value.pose.orientation.z = float(orientation.get("z", 0.0))
        value.pose.orientation.w = float(orientation.get("w", 1.0))
        value.scale.x = 4.5
        value.scale.y = 1.5
        value.scale.z = 1.0
        value.color = marker_color
        return value

    def publish_context(self):
        scenario = self.reference["scenario"]
        start_pose = scenario["odometry"]["pose"]
        goal_pose = scenario["route"]["goal_pose"]
        start_point = json_point(start_pose["position"], 0.0)
        markers = MarkerArray()
        markers.markers.append(
            self.make_arrow("scenario_start", 0, start_pose, color(0.2, 1.0, 0.3, 1.0))
        )
        markers.markers.append(
            self.make_arrow("scenario_goal", 0, goal_pose, color(1.0, 0.2, 0.2, 1.0))
        )
        reference_stop = self.reference.get("summary", {}).get("control_point_pose")
        if reference_stop is not None:
            stop = self.marker("reference_stop", 0, Marker.SPHERE)
            stop.pose.position = json_point(reference_stop["position"], 0.8)
            stop.scale.x = stop.scale.y = stop.scale.z = 1.25
            stop.color = color(1.0, 0.5, 0.0, 1.0)
            markers.markers.append(stop)
        verdict_color = color(0.25, 1.0, 0.45, 1.0)
        if "FAIL" in self.result_text:
            verdict_color = color(1.0, 0.18, 0.12, 1.0)
        elif "WAITING" in self.result_text:
            verdict_color = color(1.0, 0.85, 0.1, 1.0)
        markers.markers.append(
            self.text(
                "validation_result",
                0,
                "{}\nSPEED: red=0 | yellow=mid | cyan=max".format(self.result_text),
                start_point,
                verdict_color,
                0.9,
            )
        )
        self.context_pub.publish(markers)

    def on_bpp_path(self, message):
        if message.points:
            self.bpp_path = message

    def on_bvp_path(self, message):
        if not message.points:
            return
        self.bvp_path = message
        self.live_pub.publish(self.make_live_markers())

    def on_result(self, message):
        self.result_text = str(message.data)
        self.publish_context()


def main():
    rospy.init_node("mission_bpp_bvp_live_visualizer")
    MissionBppBvpLiveVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
