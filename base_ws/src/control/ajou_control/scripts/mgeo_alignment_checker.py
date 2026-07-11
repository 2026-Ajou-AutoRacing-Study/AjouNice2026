#!/usr/bin/env python3

import json
import math
import os

import rospy
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def norm2d(dx, dy):
    return math.hypot(dx, dy)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_between(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


class MGeoAlignmentChecker:
    def __init__(self):
        self.link_set_path = rospy.get_param(
            "~link_set_path",
            "/mnt/c/Users/lenovo/Downloads/link_set.json",
        )
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.marker_topic = rospy.get_param("~marker_topic", "/mgeo_alignment/markers")
        self.full_centerline_topic = rospy.get_param("~full_centerline_topic", "/mgeo_alignment/full_centerlines")
        self.full_lane_edge_topic = rospy.get_param("~full_lane_edge_topic", "/mgeo_alignment/full_lane_edges")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.local_radius = float(rospy.get_param("~local_radius", 80.0))
        self.point_decimation = max(1, int(rospy.get_param("~point_decimation", 5)))
        self.static_map_decimation = max(1, int(rospy.get_param("~static_map_decimation", 1)))
        self.preview_link_count = max(1, int(rospy.get_param("~preview_link_count", 30)))
        self.show_all_links = self.as_bool(rospy.get_param("~show_all_links", True))
        self.show_estimated_lane_edges = self.as_bool(rospy.get_param("~show_estimated_lane_edges", True))
        self.publish_map_in_dynamic_markers = self.as_bool(rospy.get_param("~publish_map_in_dynamic_markers", False))
        self.default_lane_width = float(rospy.get_param("~default_lane_width", 3.5))

        self.offset_x = float(rospy.get_param("~offset_x", 0.0))
        self.offset_y = float(rospy.get_param("~offset_y", 0.0))
        self.offset_z = float(rospy.get_param("~offset_z", 0.0))
        self.offset_yaw = math.radians(float(rospy.get_param("~offset_yaw_deg", 0.0)))

        self.ego_offset_x = float(rospy.get_param("~ego_offset_x", 0.0))
        self.ego_offset_y = float(rospy.get_param("~ego_offset_y", 0.0))
        self.ego_offset_z = float(rospy.get_param("~ego_offset_z", 0.0))

        self.ego_coordinate_mode = str(rospy.get_param("~ego_coordinate_mode", "local")).strip().lower()
        self.ego_plane = str(rospy.get_param("~ego_plane", "xy")).strip().lower()
        self.ego_origin_x = float(rospy.get_param("~ego_origin_x", 0.0))
        self.ego_origin_y = float(rospy.get_param("~ego_origin_y", 0.0))
        self.ego_origin_z = float(rospy.get_param("~ego_origin_z", 0.0))

        self.links = self.load_links()
        self.ego = None

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.full_centerline_pub = rospy.Publisher(self.full_centerline_topic, Marker, queue_size=1, latch=True)
        self.full_lane_edge_pub = rospy.Publisher(self.full_lane_edge_topic, Marker, queue_size=1, latch=True)
        self.nearest_distance_pub = rospy.Publisher("/mgeo_alignment/nearest_distance", Float32, queue_size=1)
        self.heading_error_pub = rospy.Publisher("/mgeo_alignment/heading_error", Float32, queue_size=1)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)

        rospy.loginfo(
            "MGeo alignment checker loaded %d links from %s. show_all_links=%s offset=(%.3f, %.3f, %.3f, %.3fdeg) ego_mode=%s ego_plane=%s",
            len(self.links),
            self.link_set_path,
            self.show_all_links,
            self.offset_x,
            self.offset_y,
            self.offset_z,
            math.degrees(self.offset_yaw),
            self.ego_coordinate_mode,
            self.ego_plane,
        )

    def as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def load_links(self):
        if not os.path.exists(self.link_set_path):
            raise RuntimeError("MGeo link_set file does not exist: %s" % self.link_set_path)

        with open(self.link_set_path, "r", encoding="utf-8") as json_file:
            raw_links = json.load(json_file)

        links = []
        for link in raw_links:
            points = []
            for raw_point in link.get("points", []):
                if len(raw_point) < 3:
                    continue
                points.append(self.transform_point(float(raw_point[0]), float(raw_point[1]), float(raw_point[2])))
            if len(points) >= 2:
                links.append(
                    {
                        "id": str(link.get("idx", "")),
                        "points": points,
                        "width_start": self.link_width(link.get("width_start")),
                        "width_end": self.link_width(link.get("width_end")),
                    }
                )
        return links

    def link_width(self, value):
        try:
            width = float(value)
        except (TypeError, ValueError):
            return self.default_lane_width
        if width <= 0.1:
            return self.default_lane_width
        return width

    def transform_point(self, x, y, z):
        cos_yaw = math.cos(self.offset_yaw)
        sin_yaw = math.sin(self.offset_yaw)
        return (
            cos_yaw * x - sin_yaw * y + self.offset_x,
            sin_yaw * x + cos_yaw * y + self.offset_y,
            z + self.offset_z,
        )

    def ego_callback(self, msg):
        self.ego = msg

    def marker_base(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def delete_all_marker(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def line_marker(self, marker_id, namespace, points, width, color, z_offset=0.15):
        marker = self.marker_base(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x, y, z + z_offset) for x, y, z in points]
        return marker

    def full_centerline_marker(self):
        marker = self.marker_base(0, "mgeo_full_centerlines", Marker.LINE_LIST)
        marker.scale.x = 0.16
        marker.color.r = 0.0
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.95

        for link in self.links:
            points = self.decimated_points(link["points"], self.static_map_decimation)
            for index in range(len(points) - 1):
                start = points[index]
                end = points[index + 1]
                marker.points.append(Point(start[0], start[1], start[2] + 0.25))
                marker.points.append(Point(end[0], end[1], end[2] + 0.25))
        return marker

    def full_lane_edge_marker(self):
        marker = self.marker_base(0, "mgeo_full_lane_edges", Marker.LINE_LIST)
        marker.scale.x = 0.06
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 0.65

        for link in self.links:
            for side in ("left", "right"):
                points = self.decimated_points(self.estimated_lane_edge_points(link, side), self.static_map_decimation)
                for index in range(len(points) - 1):
                    start = points[index]
                    end = points[index + 1]
                    marker.points.append(Point(start[0], start[1], start[2] + 0.18))
                    marker.points.append(Point(end[0], end[1], end[2] + 0.18))
        return marker

    def sphere_marker(self, marker_id, namespace, point, scale, color, z_offset=0.7):
        marker = self.marker_base(marker_id, namespace, Marker.SPHERE)
        marker.pose.position = Point(point[0], point[1], point[2] + z_offset)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def arrow_marker(self, marker_id, namespace, start, yaw, length, color, z_offset=1.3):
        marker = self.marker_base(marker_id, namespace, Marker.ARROW)
        marker.scale.x = 0.45
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [
            Point(start[0], start[1], start[2] + z_offset),
            Point(start[0] + length * math.cos(yaw), start[1] + length * math.sin(yaw), start[2] + z_offset),
        ]
        return marker

    def text_marker(self, marker_id, point, nearest):
        marker = self.marker_base(marker_id, "mgeo_alignment_text", Marker.TEXT_VIEW_FACING)
        marker.pose.position = Point(point[0], point[1], point[2] + 3.0)
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        if nearest:
            marker.text = "MGeo align | link %s | dist %.2fm | heading %.1fdeg" % (
                nearest["link_id"],
                nearest["distance"],
                math.degrees(nearest["heading_error"]),
            )
        else:
            marker.text = "MGeo align | waiting for nearby centerline"
        return marker

    def ego_pose(self):
        if self.ego is None:
            return None
        position = self.ego.position
        if self.ego_plane == "xz":
            point = (position.x, position.z, position.y)
        elif self.ego_plane == "x_neg_y":
            point = (position.x, -position.y, position.z)
        elif self.ego_plane == "x_neg_z":
            point = (position.x, -position.z, position.y)
        else:
            point = (position.x, position.y, position.z)

        if self.ego_coordinate_mode == "utm":
            point = (
                point[0] - self.ego_origin_x,
                point[1] - self.ego_origin_y,
                point[2] - self.ego_origin_z,
            )

        point = (
            point[0] + self.ego_offset_x,
            point[1] + self.ego_offset_y,
            point[2] + self.ego_offset_z,
        )

        return point, math.radians(float(self.ego.heading))

    def nearest_centerline(self, ego_point, ego_yaw):
        best = None
        for link in self.links:
            points = link["points"]
            for index, point in enumerate(points):
                distance = norm2d(point[0] - ego_point[0], point[1] - ego_point[1])
                if best is not None and distance >= best["distance"]:
                    continue

                if index < len(points) - 1:
                    centerline_yaw = yaw_between(point, points[index + 1])
                else:
                    centerline_yaw = yaw_between(points[index - 1], point)

                best = {
                    "link_id": link["id"],
                    "point": point,
                    "distance": distance,
                    "centerline_yaw": centerline_yaw,
                    "heading_error": normalize_angle(ego_yaw - centerline_yaw),
                }
        return best

    def selected_links(self, ego_point):
        if self.show_all_links:
            return self.links

        if ego_point is None:
            return self.links[: self.preview_link_count]

        selected = []
        radius_sq = self.local_radius * self.local_radius
        for link in self.links:
            for point in link["points"][:: self.point_decimation]:
                dx = point[0] - ego_point[0]
                dy = point[1] - ego_point[1]
                if dx * dx + dy * dy <= radius_sq:
                    selected.append(link)
                    break
        return selected[: self.preview_link_count * 3]

    def decimated_points(self, points, step=None):
        step = self.point_decimation if step is None else max(1, int(step))
        decimated = points[:: step]
        if points[-1] not in decimated:
            decimated.append(points[-1])
        return decimated

    def estimated_lane_edge_points(self, link, side):
        points = link["points"]
        edge_points = []
        count = len(points)
        sign = 1.0 if side == "left" else -1.0

        for index, point in enumerate(points):
            if index == 0:
                prev_point = points[index]
                next_point = points[index + 1]
            elif index == count - 1:
                prev_point = points[index - 1]
                next_point = points[index]
            else:
                prev_point = points[index - 1]
                next_point = points[index + 1]

            dx = next_point[0] - prev_point[0]
            dy = next_point[1] - prev_point[1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                edge_points.append(point)
                continue

            ratio = float(index) / float(max(1, count - 1))
            width = link["width_start"] + (link["width_end"] - link["width_start"]) * ratio
            nx = -dy / length
            ny = dx / length
            half_width = 0.5 * width
            edge_points.append(
                (
                    point[0] + sign * nx * half_width,
                    point[1] + sign * ny * half_width,
                    point[2],
                )
            )
        return edge_points

    def build_markers(self):
        markers = MarkerArray()
        markers.markers.append(self.delete_all_marker())

        ego = self.ego_pose()
        ego_point = ego[0] if ego else None
        ego_yaw = ego[1] if ego else 0.0
        nearest = self.nearest_centerline(ego_point, ego_yaw) if ego else None

        marker_id = 1
        if self.publish_map_in_dynamic_markers:
            for link in self.selected_links(ego_point):
                if self.show_estimated_lane_edges:
                    markers.markers.append(
                        self.line_marker(
                            marker_id,
                            "mgeo_alignment_estimated_lane_edges",
                            self.decimated_points(self.estimated_lane_edge_points(link, "left")),
                            0.08,
                            (0.95, 0.95, 0.95, 0.65),
                            0.18,
                        )
                    )
                    marker_id += 1
                    markers.markers.append(
                        self.line_marker(
                            marker_id,
                            "mgeo_alignment_estimated_lane_edges",
                            self.decimated_points(self.estimated_lane_edge_points(link, "right")),
                            0.08,
                            (0.95, 0.95, 0.95, 0.65),
                            0.18,
                        )
                    )
                    marker_id += 1
                markers.markers.append(
                    self.line_marker(
                        marker_id,
                        "mgeo_alignment_centerlines",
                        self.decimated_points(link["points"]),
                        0.18,
                        (0.0, 0.65, 1.0, 0.75),
                    )
                )
                marker_id += 1

        if ego:
            markers.markers.append(
                self.sphere_marker(marker_id, "mgeo_alignment_ego", ego_point, 1.4, (1.0, 1.0, 1.0, 1.0))
            )
            marker_id += 1
            markers.markers.append(
                self.arrow_marker(marker_id, "mgeo_alignment_ego_heading", ego_point, ego_yaw, 7.0, (1.0, 1.0, 1.0, 1.0))
            )
            marker_id += 1

        if nearest:
            nearest_point = nearest["point"]
            markers.markers.append(
                self.sphere_marker(marker_id, "mgeo_alignment_nearest_centerline", nearest_point, 1.2, (1.0, 0.0, 0.0, 1.0))
            )
            marker_id += 1
            markers.markers.append(
                self.arrow_marker(
                    marker_id,
                    "mgeo_alignment_centerline_heading",
                    nearest_point,
                    nearest["centerline_yaw"],
                    7.0,
                    (0.0, 1.0, 0.2, 1.0),
                )
            )
            marker_id += 1
            markers.markers.append(
                self.line_marker(
                    marker_id,
                    "mgeo_alignment_ego_to_centerline",
                    [ego_point, nearest_point],
                    0.25,
                    (1.0, 0.0, 1.0, 1.0),
                    0.9,
                )
            )
            marker_id += 1
            markers.markers.append(self.text_marker(marker_id, ego_point, nearest))
        elif ego:
            markers.markers.append(self.text_marker(marker_id, ego_point, None))

        return markers, nearest

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        self.full_centerline_pub.publish(self.full_centerline_marker())
        if self.show_estimated_lane_edges:
            self.full_lane_edge_pub.publish(self.full_lane_edge_marker())
        while not rospy.is_shutdown():
            markers, nearest = self.build_markers()
            self.marker_pub.publish(markers)

            if nearest:
                self.nearest_distance_pub.publish(Float32(data=nearest["distance"]))
                self.heading_error_pub.publish(Float32(data=nearest["heading_error"]))
                rospy.loginfo_throttle(
                    1.0,
                    "MGeo alignment: nearest_link=%s dist=%.2fm heading_error=%.1fdeg",
                    nearest["link_id"],
                    nearest["distance"],
                    math.degrees(nearest["heading_error"]),
                )
            else:
                rospy.loginfo_throttle(1.0, "MGeo alignment waiting for ego on %s", self.ego_topic)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mgeo_alignment_checker")
    try:
        MGeoAlignmentChecker().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("mgeo_alignment_checker failed: %s", exc)
        raise
