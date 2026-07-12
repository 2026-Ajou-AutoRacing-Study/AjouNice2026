#!/usr/bin/env python3

import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from lanelet2_route_planner import Lanelet2RoutePlanner  # noqa: E402


REFERENCE_COLOR = (0.0, 0.85, 1.0, 1.0)
CANDIDATE_COLOR = (1.0, 0.0, 0.95, 1.0)
PASS_COLOR = (0.25, 1.0, 0.45, 1.0)
FAIL_COLOR = (1.0, 0.18, 0.12, 1.0)


def read_route(path):
    with open(path, "r", encoding="utf-8") as stream:
        document = json.load(stream)
    route = document.get("route")
    if not isinstance(route, dict):
        raise ValueError("route JSON has no route object: {}".format(path))
    return document, route


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


def point(x, y, z):
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


def preferred_ids(route):
    return [int(segment["preferred_primitive"]["id"]) for segment in route["segments"]]


def primitive_groups(route):
    return [
        sorted(
            (int(item["id"]), str(item.get("primitive_type", "")))
            for item in segment["primitives"]
        )
        for segment in route["segments"]
    ]


def all_lane_ids(route):
    lane_ids = set()
    for segment in route["segments"]:
        lane_ids.add(int(segment["preferred_primitive"]["id"]))
        lane_ids.update(int(item["id"]) for item in segment["primitives"])
    return lane_ids


class MissionRouteComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.map_file = rospy.get_param("~map_file")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.projector_file = rospy.get_param("~projector_file", "")
        self.reference_path = rospy.get_param("~reference_json")
        self.candidate_path = rospy.get_param("~candidate_json")
        self.report_path = rospy.get_param("~report_json", "")
        self.output_topic = rospy.get_param(
            "~output_topic", "/planning/mission_planning/validation/route_overlay"
        )
        self.route_z_offset = float(rospy.get_param("~route_z_offset", 0.35))
        self.publish_rate = float(rospy.get_param("~publish_rate", 0.0))

        _reference_document, self.reference = read_route(self.reference_path)
        _candidate_document, self.candidate = read_route(self.candidate_path)
        self.report = self.read_report(self.report_path)
        self.map = Lanelet2RoutePlanner(
            self.map_file,
            projector_type=self.projector_type,
            projector_file=self.projector_file,
            check_footprint_inside_lanes=False,
        )
        self.publisher = rospy.Publisher(
            self.output_topic, MarkerArray, queue_size=1, latch=True
        )
        self.marker_array = self.build_markers()

    def read_report(self, path):
        if not path or not os.path.isfile(path):
            return None
        with open(path, "r", encoding="utf-8") as stream:
            return json.load(stream)

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

    def external_point(self, lanelet_point, z_offset=0.0):
        x, y, z = self.map.to_external_xyz(lanelet_point)
        return point(x, y, z + z_offset)

    def append_segments(self, marker, points):
        for first, second in zip(points[:-1], points[1:]):
            marker.points.extend([first, second])

    def make_map_boundaries(self):
        marker = self.marker("map_boundaries", 0, Marker.LINE_LIST)
        marker.scale.x = 0.08
        marker.color = color(0.62, 0.66, 0.70, 0.42)
        seen_boundaries = set()
        for lanelet in self.map.map.laneletLayer:
            for boundary in (lanelet.leftBound, lanelet.rightBound):
                boundary_id = int(boundary.id)
                if boundary_id in seen_boundaries:
                    continue
                seen_boundaries.add(boundary_id)
                points = [self.external_point(item, 0.02) for item in boundary]
                self.append_segments(marker, points)
        return marker

    def make_route_fill(self, namespace, marker_id, lane_ids, marker_color, z_offset):
        marker = self.marker(namespace, marker_id, Marker.TRIANGLE_LIST)
        marker.color = marker_color
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        for lane_id in sorted(lane_ids):
            try:
                lanelet = self.map.map.laneletLayer[int(lane_id)]
            except Exception:
                continue
            left = [self.external_point(item, z_offset) for item in lanelet.leftBound]
            right = [self.external_point(item, z_offset) for item in lanelet.rightBound]
            count = min(len(left), len(right))
            for index in range(count - 1):
                marker.points.extend(
                    [left[index], right[index], left[index + 1]]
                )
                marker.points.extend(
                    [left[index + 1], right[index], right[index + 1]]
                )
        return marker

    def route_points(self, route, additional_z_offset=0.0):
        points = self.map.centerline_points(preferred_ids(route))
        return [
            point(x, y, z + self.route_z_offset + additional_z_offset)
            for x, y, z in points
        ]

    def make_reference_line(self):
        marker = self.marker("ros2_reference", 0, Marker.LINE_STRIP)
        marker.scale.x = 0.58
        marker.color = color(*REFERENCE_COLOR)
        marker.points = self.route_points(self.reference)
        return marker

    def make_candidate_dashes(self):
        marker = self.marker("ros1_candidate", 0, Marker.LINE_LIST)
        marker.scale.x = 0.90
        marker.color = color(*CANDIDATE_COLOR)
        points = self.route_points(self.candidate, additional_z_offset=0.15)
        for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
            if index % 2 == 0:
                marker.points.extend([first, second])
        return marker

    def make_pose_arrow(self, namespace, marker_id, pose_data, marker_color, z_offset):
        marker = self.marker(namespace, marker_id, Marker.ARROW)
        position = pose_data["position"]
        orientation = pose_data["orientation"]
        marker.pose.position.x = float(position["x"])
        marker.pose.position.y = float(position["y"])
        marker.pose.position.z = float(position["z"]) + z_offset
        marker.pose.orientation.x = float(orientation["x"])
        marker.pose.orientation.y = float(orientation["y"])
        marker.pose.orientation.z = float(orientation["z"])
        marker.pose.orientation.w = float(orientation["w"])
        marker.scale.x = 4.0
        marker.scale.y = 0.85
        marker.scale.z = 0.85
        marker.color = marker_color
        return marker

    def section_center(self, lane_id):
        lanelet = self.map.map.laneletLayer[int(lane_id)]
        centerline = list(lanelet.centerline)
        item = centerline[len(centerline) // 2]
        return self.external_point(item, self.route_z_offset + 1.3)

    def make_section_labels(self):
        markers = []
        reference_ids = preferred_ids(self.reference)
        candidate_ids = preferred_ids(self.candidate)
        section_count = max(len(reference_ids), len(candidate_ids))
        for index in range(section_count):
            reference_id = reference_ids[index] if index < len(reference_ids) else None
            candidate_id = candidate_ids[index] if index < len(candidate_ids) else None
            anchor_id = reference_id if reference_id is not None else candidate_id
            if anchor_id is None:
                continue
            try:
                center = self.section_center(anchor_id)
            except Exception:
                continue
            marker = self.marker("section_labels", index, Marker.TEXT_VIEW_FACING)
            marker.pose.position = center
            marker.scale.z = 1.05
            if reference_id == candidate_id:
                marker.text = "{}: {}  OK".format(index + 1, reference_id)
                marker.color = color(*PASS_COLOR)
            else:
                marker.text = "{}: ROS2={}  ROS1={}".format(
                    index + 1, reference_id, candidate_id
                )
                marker.color = color(*FAIL_COLOR)
            markers.append(marker)
        return markers

    def comparison_passed(self):
        if self.report is not None:
            return bool(self.report.get("passed", False))
        return (
            preferred_ids(self.reference) == preferred_ids(self.candidate)
            and primitive_groups(self.reference) == primitive_groups(self.candidate)
        )

    def summary_text(self):
        reference_ids = all_lane_ids(self.reference)
        candidate_ids = all_lane_ids(self.candidate)
        union = reference_ids | candidate_ids
        jaccard = len(reference_ids & candidate_ids) / float(len(union)) if union else 1.0
        verdict = "PASS" if self.comparison_passed() else "FAIL"
        return (
            "{}  |  ROS2 cyan solid  |  ROS1 magenta dashed  |  "
            "sections {}/{}  |  lane Jaccard {:.3f}"
        ).format(
            verdict,
            len(self.reference["segments"]),
            len(self.candidate["segments"]),
            jaccard,
        )

    def make_summary(self):
        marker = self.marker("comparison_summary", 0, Marker.TEXT_VIEW_FACING)
        start = self.reference["start_pose"]["position"]
        marker.pose.position.x = float(start["x"])
        marker.pose.position.y = float(start["y"])
        marker.pose.position.z = float(start["z"]) + 5.0
        marker.scale.z = 1.4
        marker.text = self.summary_text()
        marker.color = color(*(PASS_COLOR if self.comparison_passed() else FAIL_COLOR))
        return marker

    def build_markers(self):
        reference_lane_ids = all_lane_ids(self.reference)
        candidate_lane_ids = all_lane_ids(self.candidate)
        reference_only = reference_lane_ids - candidate_lane_ids
        candidate_only = candidate_lane_ids - reference_lane_ids
        common = reference_lane_ids & candidate_lane_ids

        cleanup = self.marker("cleanup", 0, Marker.LINE_LIST)
        cleanup.action = Marker.DELETEALL
        markers = [
            cleanup,
            self.make_map_boundaries(),
            self.make_route_fill(
                "matching_route_lanelets",
                0,
                common,
                color(0.15, 0.85, 0.45, 0.10),
                0.05,
            ),
            self.make_route_fill(
                "ros2_only_lanelets",
                0,
                reference_only,
                color(1.0, 0.62, 0.0, 0.55),
                0.08,
            ),
            self.make_route_fill(
                "ros1_only_lanelets",
                0,
                candidate_only,
                color(1.0, 0.0, 0.0, 0.62),
                0.10,
            ),
            self.make_reference_line(),
            self.make_candidate_dashes(),
            self.make_pose_arrow(
                "ros2_pose",
                0,
                self.reference["start_pose"],
                color(*REFERENCE_COLOR),
                0.25,
            ),
            self.make_pose_arrow(
                "ros2_pose",
                1,
                self.reference["goal_pose"],
                color(*REFERENCE_COLOR),
                0.25,
            ),
            self.make_pose_arrow(
                "ros1_pose",
                0,
                self.candidate["start_pose"],
                color(*CANDIDATE_COLOR),
                0.48,
            ),
            self.make_pose_arrow(
                "ros1_pose",
                1,
                self.candidate["goal_pose"],
                color(*CANDIDATE_COLOR),
                0.48,
            ),
            self.make_summary(),
        ]
        markers.extend(self.make_section_labels())
        result = MarkerArray()
        result.markers = [
            marker
            for marker in markers
            if marker.action == Marker.DELETEALL
            or marker.type in (Marker.ARROW, Marker.TEXT_VIEW_FACING)
            or marker.points
        ]
        return result

    def publish(self, _event=None):
        stamp = rospy.Time.now()
        for marker in self.marker_array.markers:
            marker.header.stamp = stamp
        self.publisher.publish(self.marker_array)

    def spin(self):
        rospy.sleep(0.3)
        self.publish()
        rospy.loginfo(
            "mission route overlay published: topic=%s, verdict=%s",
            self.output_topic,
            "PASS" if self.comparison_passed() else "FAIL",
        )
        if self.publish_rate > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish)
        rospy.spin()


def main():
    rospy.init_node("mission_route_comparison_visualizer")
    MissionRouteComparisonVisualizer().spin()


if __name__ == "__main__":
    main()
