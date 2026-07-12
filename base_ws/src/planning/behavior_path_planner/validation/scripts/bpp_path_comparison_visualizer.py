#!/usr/bin/env python3

import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_map_tools import (
    CenterlineMap,
    closest_lane_id,
    cumulative_lengths,
    interpolate_polyline,
    project_point_to_polyline,
)


REFERENCE_COLOR = (0.0, 0.85, 1.0, 1.0)
CANDIDATE_COLOR = (1.0, 0.0, 0.95, 1.0)
PASS_COLOR = (0.25, 1.0, 0.45, 1.0)
FAIL_COLOR = (1.0, 0.18, 0.12, 1.0)
SOURCE_LANE_COLOR = (0.15, 0.55, 1.0, 1.0)
TARGET_LANE_COLOR = (0.35, 1.0, 0.25, 1.0)
GOAL_COLOR = (1.0, 0.82, 0.12, 1.0)
PROPOSED_COLOR = (1.0, 0.58, 0.05, 0.95)
OBSTACLE_COLOR = (1.0, 0.08, 0.06, 0.9)
EGO_COLOR = (1.0, 1.0, 1.0, 0.95)


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


def path_points(document, path_key="path", z_offset=0.0):
    return [
        point(item["pose"]["position"], z_offset)
        for item in document[path_key]["points"]
    ]


class BppPathComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.path_key = rospy.get_param("~path_key", "path")
        self.context_path_key = rospy.get_param("~context_path_key", "")
        self.proposed_path_key = rospy.get_param("~proposed_path_key", "")
        self.show_objects = rospy.get_param("~show_objects", False)
        self.object_label = rospy.get_param("~object_label", "BLOCKING OBJECT")
        self.show_ego = rospy.get_param("~show_ego", True)
        self.show_route_goal = rospy.get_param("~show_route_goal", False)
        self.scenario_label = rospy.get_param("~scenario_label", "")
        self.show_lane_change_lanelets = rospy.get_param(
            "~show_lane_change_lanelets", False
        )
        self.centerline_map = None
        map_file = rospy.get_param("~map_file", "")
        if map_file:
            self.centerline_map = CenterlineMap.load(
                map_file,
                rospy.get_param("~projector_file", ""),
                False,
                rospy.get_param("~map_projector_type", "MGRS"),
            )
        report_path = rospy.get_param("~report_json", "")
        self.report = read_json(report_path) if report_path else None
        self.output_topic = rospy.get_param(
            "~output_topic", "/planning/debug/bpp/lane_following_validation_overlay"
        )
        self.publisher = rospy.Publisher(
            self.output_topic, MarkerArray, queue_size=1, latch=True
        )
        self.markers = self.build_markers()

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

    def make_candidate_dashes(self):
        marker = self.marker("ros1_candidate", 0, Marker.LINE_LIST)
        marker.scale.x = 0.9
        marker.color = color(*CANDIDATE_COLOR)
        points = path_points(self.candidate, self.path_key, 0.20)
        for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
            if index % 2 == 0:
                marker.points.extend([first, second])
        return marker

    def make_endpoint(self, namespace, marker_id, value, marker_color, z_offset):
        marker = self.marker(namespace, marker_id, Marker.SPHERE)
        marker.pose.position.x = float(value.x)
        marker.pose.position.y = float(value.y)
        marker.pose.position.z = float(value.z) + float(z_offset)
        marker.scale.x = marker.scale.y = marker.scale.z = 1.2
        marker.color = marker_color
        return marker

    def make_text(self, namespace, marker_id, text, position, marker_color):
        marker = self.marker(namespace, marker_id, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + 4.0
        marker.scale.z = 1.6
        marker.color = marker_color
        marker.text = text
        return marker

    def make_summary(self):
        marker = self.marker("comparison_summary", 0, Marker.TEXT_VIEW_FACING)
        start = self.reference[self.path_key]["points"][0]["pose"]["position"]
        marker.pose.position = point(start, 4.0)
        marker.scale.z = 1.2
        passed = bool(self.report and self.report.get("passed", False))
        if self.report:
            diagnostics = self.report.get("diagnostics", {})
            lateral_profile = diagnostics.get("lateral_profile")
            if lateral_profile:
                marker.text = (
                    "{} | ROS2 cyan | ROS1 magenta dashed | "
                    "lateral RMSE {:.3f} m | max {:.3f} m"
                ).format(
                    "PASS" if passed else "FAIL",
                    float(lateral_profile.get("rmse_m", 0.0)),
                    float(lateral_profile.get("max_error_m", 0.0)),
                )
                marker.color = color(*(PASS_COLOR if passed else FAIL_COLOR))
                return marker
            dynamic_cut = diagnostics.get("left_cut")
            if dynamic_cut:
                marker.text = (
                    "{} | ROS2 cyan | ROS1 magenta dashed | "
                    "bound RMSE {:.3f} m | peak {:.3f}/{:.3f} m"
                ).format(
                    "PASS" if passed else "FAIL",
                    float(dynamic_cut.get("profile_rmse_m", 0.0)),
                    float(dynamic_cut.get("reference_peak_m", 0.0)),
                    float(dynamic_cut.get("candidate_peak_m", 0.0)),
                )
                marker.color = color(*(PASS_COLOR if passed else FAIL_COLOR))
                return marker
            geometry = diagnostics.get(
                "centerline_geometry",
                diagnostics.get(
                    "candidate_geometry", diagnostics.get("final_geometry", {})
                ),
            )
            path_description = "ROS2 cyan | ROS1 magenta dashed"
            if self.proposed_path_key:
                path_description += " | ROS2 proposal orange"
            marker.text = (
                "{} | {} | "
                "length {:.3f}/{:.3f} m | RMSE {:.3f} m"
            ).format(
                "PASS" if passed else "FAIL",
                path_description,
                float(
                    diagnostics.get(
                        "reference_path_length_m",
                        diagnostics.get(
                            "reference_length_m",
                            diagnostics.get("reference_final_length_m", 0.0),
                        ),
                    )
                ),
                float(
                    diagnostics.get(
                        "candidate_path_length_m",
                        diagnostics.get(
                            "candidate_length_m",
                            diagnostics.get("candidate_final_length_m", 0.0),
                        ),
                    )
                ),
                float(geometry.get("rmse_m", 0.0)),
            )
        else:
            marker.text = "ROS2 cyan | ROS1 magenta dashed"
        marker.color = color(*(PASS_COLOR if passed else FAIL_COLOR))
        return marker

    def make_object_markers(self):
        markers = []
        objects = self.reference.get("scenario", {}).get("objects", [])
        for index, predicted_object in enumerate(objects):
            kinematics = predicted_object.get("kinematics", {})
            pose = kinematics.get("initial_pose_with_covariance", {}).get(
                "pose", {}
            )
            position = pose.get("position", {})
            orientation = pose.get("orientation", {})
            dimensions = predicted_object.get("shape", {}).get("dimensions", {})

            body = self.marker("blocking_objects", index, Marker.CUBE)
            body.pose.position = point(position, 0.15)
            body.pose.orientation.x = float(orientation.get("x", 0.0))
            body.pose.orientation.y = float(orientation.get("y", 0.0))
            body.pose.orientation.z = float(orientation.get("z", 0.0))
            body.pose.orientation.w = float(orientation.get("w", 1.0))
            body.scale.x = max(float(dimensions.get("x", 0.1)), 0.1)
            body.scale.y = max(float(dimensions.get("y", 0.1)), 0.1)
            body.scale.z = max(float(dimensions.get("z", 0.1)), 0.1)
            body.color = color(*OBSTACLE_COLOR)
            markers.append(body)

            velocity = kinematics.get("initial_twist_with_covariance", {}).get(
                "linear", {}
            )
            speed = math.sqrt(
                float(velocity.get("x", 0.0)) ** 2
                + float(velocity.get("y", 0.0)) ** 2
                + float(velocity.get("z", 0.0)) ** 2
            )
            markers.append(
                self.make_text(
                    "blocking_object_labels",
                    index,
                    "{} {} | {:.2f} m/s".format(
                        self.object_label, index + 1, speed
                    ),
                    (
                        float(position.get("x", 0.0)),
                        float(position.get("y", 0.0)),
                        float(position.get("z", 0.0)) + body.scale.z * 0.5,
                    ),
                    color(*OBSTACLE_COLOR),
                )
            )

            predicted_paths = kinematics.get("predicted_paths", [])
            if predicted_paths and predicted_paths[0].get("path"):
                predicted_points = [
                    point(item["position"], 0.20)
                    for item in predicted_paths[0]["path"]
                ]
                markers.append(
                    self.make_line(
                        "blocking_object_predicted_paths",
                        index,
                        predicted_points,
                        0.18,
                        color(1.0, 0.25, 0.12, 0.85),
                    )
                )
        return markers

    def make_scene_context_markers(self):
        markers = []
        scenario = self.reference.get("scenario", {})
        odometry = scenario.get("odometry", {})
        ego_pose = odometry.get("pose")
        if self.show_ego and ego_pose:
            body = self.marker("validation_ego", 0, Marker.ARROW)
            body.pose.position = point(ego_pose["position"], 0.35)
            orientation = ego_pose.get("orientation", {})
            body.pose.orientation.x = float(orientation.get("x", 0.0))
            body.pose.orientation.y = float(orientation.get("y", 0.0))
            body.pose.orientation.z = float(orientation.get("z", 0.0))
            body.pose.orientation.w = float(orientation.get("w", 1.0))
            body.scale.x = 4.6
            body.scale.y = 1.6
            body.scale.z = 1.0
            body.color = color(*EGO_COLOR)
            markers.append(body)

            label = self.scenario_label or "EGO"
            request = self.reference.get("lateral_offset_request_m")
            if request is not None:
                label += " | lateral request {:+.2f} m".format(float(request))
            markers.append(
                self.make_text(
                    "validation_ego_label",
                    0,
                    label,
                    (
                        float(ego_pose["position"]["x"]),
                        float(ego_pose["position"]["y"]),
                        float(ego_pose["position"]["z"]),
                    ),
                    color(*EGO_COLOR),
                )
            )

        route = scenario.get("route", {})
        goal_pose = route.get("goal_pose")
        if self.show_route_goal and goal_pose:
            goal = point(goal_pose["position"], 0.7)
            markers.append(
                self.make_endpoint("validation_route_goal", 0, goal, color(*GOAL_COLOR), 0.0)
            )
            markers.append(
                self.make_text(
                    "validation_route_goal_label",
                    0,
                    "FIXED ROUTE GOAL",
                    (goal.x, goal.y, goal.z),
                    color(*GOAL_COLOR),
                )
            )

        for marker_id, (document, label, marker_color) in enumerate(
            (
                (self.reference, "ROS2 MODIFIED GOAL", color(*REFERENCE_COLOR)),
                (self.candidate, "ROS1 MODIFIED GOAL", color(*CANDIDATE_COLOR)),
            )
        ):
            modified = document.get("modified_goal")
            if not modified:
                continue
            modified_point = point(modified["pose"]["position"], 1.0 + marker_id * 0.25)
            markers.append(
                self.make_endpoint(
                    "validation_modified_goals",
                    marker_id,
                    modified_point,
                    marker_color,
                    0.0,
                )
            )
            markers.append(
                self.make_text(
                    "validation_modified_goal_labels",
                    marker_id,
                    label,
                    (modified_point.x, modified_point.y, modified_point.z),
                    marker_color,
                )
            )
        return markers

    def make_map_boundaries(self):
        marker = self.marker("lanelet_map_boundaries", 0, Marker.LINE_LIST)
        marker.scale.x = 0.10
        marker.color = color(0.58, 0.62, 0.68, 0.42)
        if self.centerline_map is None:
            return marker
        for left, right in self.centerline_map.boundaries.values():
            for boundary in (left, right):
                for first, second in zip(boundary[:-1], boundary[1:]):
                    marker.points.extend(
                        [
                            point({"x": first[0], "y": first[1], "z": first[2]}),
                            point({"x": second[0], "y": second[1], "z": second[2]}),
                        ]
                    )
        return marker

    def lane_change_lane_ids(self):
        if self.centerline_map is None:
            return None, None
        route = self.reference.get("scenario", {}).get("route", {})
        segments = route.get("segments", [])
        odometry = self.reference.get("scenario", {}).get("odometry", {})
        ego = odometry.get("pose", {}).get("position")
        if not segments or ego is None:
            return None, None

        segment = segments[0]
        target_lane_id = int(segment["preferred_primitive"]["id"])
        primitive_ids = [
            int(item["id"])
            for item in segment.get("primitives", [])
            if int(item["id"]) != 0
        ]
        if target_lane_id and target_lane_id not in primitive_ids:
            primitive_ids.append(target_lane_id)
        ego_point = (float(ego["x"]), float(ego["y"]), float(ego["z"]))
        source_lane_id = closest_lane_id(
            self.centerline_map, primitive_ids, ego_point
        )
        return source_lane_id, target_lane_id

    def lane_label_position(self, lane_id, forward_offset):
        centerline = self.centerline_map.get_centerline(lane_id)
        ego = self.reference["scenario"]["odometry"]["pose"]["position"]
        ego_point = (float(ego["x"]), float(ego["y"]), float(ego["z"]))
        projection = project_point_to_polyline(centerline, ego_point)
        station = projection[0] + float(forward_offset)
        return interpolate_polyline(centerline, station)

    def make_lanelet_fill(self, namespace, lane_id, marker_color):
        marker = self.marker(namespace, 0, Marker.TRIANGLE_LIST)
        marker.color = marker_color
        boundaries = self.centerline_map.get_boundaries(lane_id)
        if not boundaries:
            return marker
        left = list(boundaries[0])
        right = list(boundaries[1])
        direct = math.dist(left[0], right[0]) + math.dist(left[-1], right[-1])
        crossed = math.dist(left[0], right[-1]) + math.dist(left[-1], right[0])
        if crossed < direct:
            right.reverse()

        left_length = cumulative_lengths(left)[-1]
        right_length = cumulative_lengths(right)[-1]
        sample_count = max(
            2,
            int(math.ceil(max(left_length, right_length) / 4.0)) + 1,
        )
        left_samples = []
        right_samples = []
        for index in range(sample_count):
            ratio = float(index) / float(sample_count - 1)
            left_samples.append(interpolate_polyline(left, ratio * left_length))
            right_samples.append(interpolate_polyline(right, ratio * right_length))

        for index in range(sample_count - 1):
            left_start = left_samples[index]
            left_end = left_samples[index + 1]
            right_start = right_samples[index]
            right_end = right_samples[index + 1]
            marker.points.extend(
                [
                    point({"x": left_start[0], "y": left_start[1], "z": left_start[2]}, 0.03),
                    point({"x": right_start[0], "y": right_start[1], "z": right_start[2]}, 0.03),
                    point({"x": left_end[0], "y": left_end[1], "z": left_end[2]}, 0.03),
                    point({"x": left_end[0], "y": left_end[1], "z": left_end[2]}, 0.03),
                    point({"x": right_start[0], "y": right_start[1], "z": right_start[2]}, 0.03),
                    point({"x": right_end[0], "y": right_end[1], "z": right_end[2]}, 0.03),
                ]
            )
        return marker

    def make_lane_change_lanelet_markers(self):
        source_lane_id, target_lane_id = self.lane_change_lane_ids()
        if source_lane_id is None or target_lane_id is None:
            return []

        source_centerline = self.centerline_map.get_centerline(source_lane_id)
        target_centerline = self.centerline_map.get_centerline(target_lane_id)
        source_points = [
            point({"x": item[0], "y": item[1], "z": item[2]}, 0.04)
            for item in source_centerline
        ]
        target_points = [
            point({"x": item[0], "y": item[1], "z": item[2]}, 0.05)
            for item in target_centerline
        ]
        source_label = self.lane_label_position(source_lane_id, 16.0)
        target_label = self.lane_label_position(target_lane_id, 32.0)
        route = self.reference["scenario"]["route"]
        goal = route["goal_pose"]["position"]
        goal_point = point(goal, 0.8)

        return [
            self.make_lanelet_fill(
                "source_lanelet_fill",
                source_lane_id,
                color(
                    SOURCE_LANE_COLOR[0],
                    SOURCE_LANE_COLOR[1],
                    SOURCE_LANE_COLOR[2],
                    0.16,
                ),
            ),
            self.make_lanelet_fill(
                "target_lanelet_fill",
                target_lane_id,
                color(
                    TARGET_LANE_COLOR[0],
                    TARGET_LANE_COLOR[1],
                    TARGET_LANE_COLOR[2],
                    0.22,
                ),
            ),
            self.make_line(
                "source_lanelet_centerline",
                0,
                source_points,
                0.42,
                color(*SOURCE_LANE_COLOR),
            ),
            self.make_line(
                "target_lanelet_centerline",
                0,
                target_points,
                0.62,
                color(*TARGET_LANE_COLOR),
            ),
            self.make_text(
                "source_lanelet_label",
                0,
                "SOURCE LANELET {}".format(source_lane_id),
                source_label,
                color(*SOURCE_LANE_COLOR),
            ),
            self.make_text(
                "target_lanelet_label",
                0,
                "TARGET LANELET {}".format(target_lane_id),
                target_label,
                color(*TARGET_LANE_COLOR),
            ),
            self.make_endpoint(
                "route_goal",
                0,
                goal_point,
                color(*GOAL_COLOR),
                0.0,
            ),
            self.make_text(
                "route_goal_label",
                0,
                "GOAL ON LANELET {}".format(target_lane_id),
                (goal_point.x, goal_point.y, goal_point.z),
                color(*GOAL_COLOR),
            ),
        ]

    def build_markers(self):
        cleanup = self.marker("cleanup", 0, Marker.LINE_LIST)
        cleanup.action = Marker.DELETEALL
        ref_path = path_points(self.reference, self.path_key, 0.10)
        candidate_path = path_points(self.candidate, self.path_key, 0.20)
        markers = [
            cleanup,
            self.make_map_boundaries(),
            self.make_line(
                "ros2_reference",
                0,
                ref_path,
                0.55,
                color(*REFERENCE_COLOR),
            ),
            self.make_candidate_dashes(),
        ]
        markers.extend(self.make_scene_context_markers())

        if self.proposed_path_key:
            markers.append(
                self.make_line(
                    "ros2_proposed_path",
                    0,
                    path_points(self.reference, self.proposed_path_key, 0.32),
                    0.38,
                    color(*PROPOSED_COLOR),
                )
            )

        if self.show_objects:
            markers.extend(self.make_object_markers())

        if self.show_lane_change_lanelets:
            markers.extend(self.make_lane_change_lanelet_markers())

        if self.context_path_key:
            markers.extend(
                [
                    self.make_line(
                        "ros2_reference_root",
                        0,
                        path_points(self.reference, self.context_path_key, 0.06),
                        0.22,
                        color(0.95, 0.95, 0.95, 0.8),
                    ),
                    self.make_line(
                        "ros1_candidate_root",
                        0,
                        path_points(self.candidate, self.context_path_key, 0.12),
                        0.22,
                        color(1.0, 0.72, 0.10, 0.9),
                    ),
                ]
            )

        for index, key in enumerate(("left_bound", "right_bound")):
            reference_path = self.reference[self.path_key]
            candidate_output = self.candidate[self.path_key]
            markers.append(
                self.make_line(
                    "ros2_drivable_bounds",
                    index,
                    [point(item, 0.04) for item in reference_path.get(key, [])],
                    0.14,
                    color(REFERENCE_COLOR[0], REFERENCE_COLOR[1], REFERENCE_COLOR[2], 0.65),
                )
            )
            markers.append(
                self.make_line(
                    "ros1_drivable_bounds",
                    index,
                    [point(item, 0.12) for item in candidate_output.get(key, [])],
                    0.20,
                    color(CANDIDATE_COLOR[0], CANDIDATE_COLOR[1], CANDIDATE_COLOR[2], 0.75),
                )
            )

        markers.extend(
            [
                self.make_endpoint(
                    "ros2_endpoints", 0, ref_path[0], color(*REFERENCE_COLOR), 0.0
                ),
                self.make_endpoint(
                    "ros2_endpoints", 1, ref_path[-1], color(*REFERENCE_COLOR), 0.0
                ),
                self.make_endpoint(
                    "ros1_endpoints", 0, candidate_path[0], color(*CANDIDATE_COLOR), 0.25
                ),
                self.make_endpoint(
                    "ros1_endpoints", 1, candidate_path[-1], color(*CANDIDATE_COLOR), 0.25
                ),
                self.make_summary(),
            ]
        )
        output = MarkerArray()
        output.markers = markers
        return output

    def spin(self):
        rospy.sleep(0.3)
        self.publisher.publish(self.markers)
        rospy.loginfo(
            "BPP path comparison overlay published: %s", self.output_topic
        )
        rospy.spin()


def main():
    rospy.init_node("bpp_path_comparison_visualizer")
    BppPathComparisonVisualizer().spin()


if __name__ == "__main__":
    main()
