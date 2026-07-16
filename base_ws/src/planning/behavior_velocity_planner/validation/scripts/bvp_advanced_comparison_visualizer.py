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
    result = ColorRGBA()
    result.r = float(red)
    result.g = float(green)
    result.b = float(blue)
    result.a = float(alpha)
    return result


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


class AdvancedComparisonVisualizer:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.module = rospy.get_param("~module")
        self.reference = read_json(rospy.get_param("~reference_json"))
        self.candidate = read_json(rospy.get_param("~candidate_json"))
        self.report = read_json(rospy.get_param("~report_json"))
        self.display_offset = float(rospy.get_param("~display_offset", 1.2))
        self.lanelet_map = BehaviorVelocityMap.load(
            rospy.get_param("~map_file"),
            rospy.get_param("~projector_file", ""),
            rospy.get_param("~map_projector_type", "MGRS"),
            False,
        )
        velocities = [
            float(item["longitudinal_velocity_mps"])
            for document in (self.reference, self.candidate)
            for item in document["output_path"]["points"]
        ]
        self.maximum_velocity = max(velocities or [1.0])
        prefix = "/validation/bvp_advanced"
        self.context_pub = rospy.Publisher(prefix + "/context", MarkerArray, queue_size=1, latch=True)
        self.ros2_pub = rospy.Publisher(prefix + "/ros2_speed_path", MarkerArray, queue_size=1, latch=True)
        self.ros1_pub = rospy.Publisher(prefix + "/ros1_speed_path", MarkerArray, queue_size=1, latch=True)
        self.context_pub.publish(self.context_markers())
        self.ros2_pub.publish(self.speed_markers(self.reference, "ros2", self.display_offset, color(*ROS2_COLOR), "ROS2 PLANNING SIM"))
        self.ros1_pub.publish(self.speed_markers(self.candidate, "ros1", -self.display_offset, color(*ROS1_COLOR), "ROS1 PORT"))

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
        for index in range(1, len(polygon) - 1):
            result.points.extend([point(polygon[0], 0.18), point(polygon[index], 0.18), point(polygon[index + 1], 0.18)])
        return result

    @staticmethod
    def document_points(document, key="output_path", z_offset=0.0):
        return [point(item["pose"]["position"], z_offset) for item in document[key]["points"]]

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
        markers.markers.append(self.line(namespace + "_outline", 0, shifted, 1.18, source_color))
        speed = self.marker(namespace + "_speed", 0, Marker.LINE_LIST)
        speed.scale.x = 0.86
        values = document["output_path"]["points"]
        for index, (first, second) in enumerate(zip(shifted[:-1], shifted[1:])):
            speed.points.extend([first, second])
            speed.colors.extend([self.speed_color(values[index]["longitudinal_velocity_mps"]), self.speed_color(values[index + 1]["longitudinal_velocity_mps"])])
        markers.markers.append(speed)
        summary = document["summary"]
        result_text = "GO"
        if summary["stop_active"]:
            index = max(0, min(int(summary["first_zero_index"]), len(shifted) - 1))
            stop = self.marker(namespace + "_stop", 0, Marker.SPHERE)
            stop.pose.position = shifted[index]
            stop.pose.position.z += 0.55
            stop.scale.x = stop.scale.y = stop.scale.z = 1.0
            stop.color = source_color
            markers.markers.append(stop)
            result_text = "stop {:.3f} m".format(float(summary["first_zero_station_m"]))
        label_index = min(8 if offset >= 0.0 else 16, len(shifted) - 1)
        markers.markers.append(self.text(namespace + "_label", 0, "{} | {}".format(label, result_text), self.offset_points(document, offset * 4.0)[label_index], source_color))
        return markers

    def lanelet_markers(self, lanelet_id, namespace, marker_id, marker_color, label):
        lanelet = self.lanelet_map.road_lanelets.get(int(lanelet_id))
        if lanelet is None:
            return []
        center = lanelet.centerline[len(lanelet.centerline) // 2]
        return [
            self.polygon(namespace, marker_id, lanelet.polygon, marker_color),
            self.line(namespace + "_outline", marker_id, [point(value, 0.23) for value in lanelet.polygon], 0.26, color(marker_color.r, marker_color.g, marker_color.b, 0.95), True),
            self.text(namespace + "_label", marker_id, label, center, color(marker_color.r, marker_color.g, marker_color.b, 1.0), 0.55),
        ]

    def detection_context(self):
        markers = []
        candidates = [area for area in self.lanelet_map.detection_area_candidates() if 200208 in area.lane_ids]
        for area_index, area in enumerate(candidates[:1]):
            for polygon_index, polygon_value in enumerate(area.polygons):
                markers.append(self.polygon("detection_area", polygon_index, polygon_value, color(1.0, 0.55, 0.02, 0.28)))
                markers.append(self.line("detection_area_outline", polygon_index, [point(value, 0.22) for value in polygon_value], 0.3, color(1.0, 0.65, 0.08, 0.95), True))
            if area.stop_line is not None:
                markers.append(self.line("detection_stop_line", area_index, [point(value, 0.35) for value in area.stop_line.points], 0.55, color(1.0, 0.1, 0.08, 1.0)))
        cloud = self.marker("detection_pointcloud", 0, Marker.SPHERE_LIST)
        cloud.scale.x = cloud.scale.y = cloud.scale.z = 0.55
        cloud.color = color(1.0, 0.1, 0.05, 1.0)
        for dx in (-0.4, 0.0, 0.4):
            for dy in (-0.4, 0.0, 0.4):
                cloud.points.append(point((2556.0 + dx, 23878.8 + dy, 28.55), 0.4))
        markers.append(cloud)
        markers.append(self.text("scenario_label", 0, "POINTCLOUD IN DETECTION AREA", (2556.0, 23878.8, 28.55), color(1.0, 0.7, 0.15)))
        return markers

    def merge_context(self):
        markers = []
        markers.extend(self.lanelet_markers(200208, "private_lane", 0, color(0.75, 0.15, 1.0, 0.25), "PRIVATE 200208"))
        markers.extend(self.lanelet_markers(200166, "public_lane", 0, color(0.0, 0.85, 1.0, 0.22), "PUBLIC 200166"))
        markers.extend(self.lanelet_markers(200295, "conflict_lane", 0, color(1.0, 0.45, 0.0, 0.28), "FIRST CONFLICT 200295"))
        return markers

    def vtl_context(self):
        markers = []
        candidates = [value for value in self.lanelet_map.virtual_traffic_light_candidates() if 200208 in value.lane_ids]
        if not candidates:
            return markers
        item = candidates[0]
        lines = [(item.instrument, "INSTRUMENT 9901401", color(0.7, 0.2, 1.0, 1.0)), (item.start_line, "START", color(1.0, 0.85, 0.05, 1.0))]
        if item.stop_line is not None:
            lines.append((item.stop_line, "STOP", color(1.0, 0.08, 0.05, 1.0)))
        lines.extend((line, "END", color(0.1, 1.0, 0.45, 1.0)) for line in item.end_lines)
        for index, (line_value, label, line_color) in enumerate(lines):
            markers.append(self.line("vtl_lines", index, [point(value, 0.35) for value in line_value.points], 0.5, line_color))
            markers.append(self.text("vtl_labels", index, label, line_value.points[len(line_value.points) // 2], line_color, 0.5))
        command = (self.reference.get("infrastructure_commands") or [{}])[0]
        center = item.stop_line.points[0] if item.stop_line is not None else item.start_line.points[0]
        markers.append(self.text("vtl_state", 0, "REQUESTING | approval=false | command state={}".format(command.get("state", "n/a")), center, color(1.0, 0.65, 0.1), 0.6))
        return markers

    def occlusion_context(self):
        boundary = [(2525.0, 23845.0, 28.0), (2585.0, 23845.0, 28.0), (2585.0, 23895.0, 28.0), (2525.0, 23895.0, 28.0)]
        return [
            self.line("unknown_grid_boundary", 0, [point(value, 0.2) for value in boundary], 0.3, color(0.75, 0.35, 1.0, 0.8), True),
            self.text("unknown_grid_label", 0, "UNKNOWN OCCUPANCY GRID | value=50", (2551.0, 23868.0, 28.0), color(0.85, 0.5, 1.0), 0.55),
        ]

    def object_markers(self):
        markers = []
        for index, detected in enumerate((self.reference.get("dynamic_objects") or {}).get("objects", [])):
            pose = detected["pose"]
            dimensions = detected["shape"]["dimensions"]
            body = self.marker("run_out_object", index, Marker.CUBE)
            body.pose.position = point(pose["position"], 0.5 * float(dimensions["z"]))
            orientation = pose["orientation"]
            body.pose.orientation.x = float(orientation["x"])
            body.pose.orientation.y = float(orientation["y"])
            body.pose.orientation.z = float(orientation["z"])
            body.pose.orientation.w = float(orientation["w"])
            body.scale.x = float(dimensions["x"])
            body.scale.y = float(dimensions["y"])
            body.scale.z = float(dimensions["z"])
            body.color = color(1.0, 0.12, 0.04, 0.95)
            markers.append(body)
            for path_index, prediction in enumerate(detected.get("predicted_paths", [])):
                markers.append(self.line("run_out_prediction", path_index, [point(value["position"], 0.45) for value in prediction["poses"]], 0.38, color(1.0, 0.65, 0.05, 0.95)))
            markers.append(self.text("run_out_label", index, "PEDESTRIAN 0.5 m/s", pose["position"], color(1.0, 0.7, 0.1), 0.55))
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
        diagnostics = self.report.get("diagnostics", {})
        stop_error = diagnostics.get("stop_station_error_m")
        stop_text = "n/a" if stop_error is None else "{:.3f} m".format(float(stop_error))
        extra = ""
        if self.module == "run_out":
            extra = " | origin align={:.3f} m".format(float(diagnostics.get("candidate_path_origin_offset_m", 0.0)))
        return "{} | {} | stop error={} | velocity RMSE={:.4f} m/s{}\nspeed: red=0, yellow=mid, cyan=max".format(
            "PASS" if self.report.get("passed", False) else "FAIL",
            self.reference["scenario"]["case"].upper(),
            stop_text,
            float(diagnostics.get("velocity_rmse_mps", math.inf)),
            extra,
        )

    def context_markers(self):
        markers = MarkerArray()
        markers.markers.append(self.line("input_path", 0, self.document_points(self.reference, "input_path", 0.12), 0.18, color(0.75, 0.78, 0.82, 0.7)))
        builders = {
            "detection_area": self.detection_context,
            "merge_from_private": self.merge_context,
            "virtual_traffic_light": self.vtl_context,
            "occlusion_spot": self.occlusion_context,
            "run_out": self.object_markers,
        }
        markers.markers.extend(builders[self.module]())
        markers.markers.append(self.ego_marker())
        ego = self.reference["scenario"]["odometry"]["pose"]["position"]
        markers.markers.append(self.text("comparison_summary", 0, self.comparison_text(), (float(ego["x"]) - 8.0, float(ego["y"]) - 8.0, float(ego["z"])), color(*(PASS_COLOR if self.report.get("passed", False) else FAIL_COLOR)), 0.58))
        return markers


def main():
    rospy.init_node("bvp_advanced_comparison_visualizer")
    AdvancedComparisonVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
