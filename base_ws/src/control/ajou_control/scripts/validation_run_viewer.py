#!/usr/bin/env python3

import csv
import glob
import os

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def norm_path(path):
    return os.path.abspath(os.path.expanduser(str(path)))


class ValidationRunViewer:
    def __init__(self):
        self.input_csv = norm_path(rospy.get_param("~input_csv", ""))
        self.history_csvs = rospy.get_param("~history_csvs", "")
        self.history_glob = rospy.get_param("~history_glob", "")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.marker_topic = rospy.get_param("~marker_topic", "/validation/markers")
        self.path_topic = rospy.get_param("~path_topic", "/validation/selected_run_path")
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.z_offset = float(rospy.get_param("~z_offset", 0.75))
        self.display_offset_x = float(rospy.get_param("~display_offset_x", 0.0))
        self.display_offset_y = float(rospy.get_param("~display_offset_y", 0.0))
        self.display_offset_z = float(rospy.get_param("~display_offset_z", 0.0))

        self.selected_run = self.load_run(self.input_csv) if self.input_csv else []
        self.history_paths = self.resolve_history_csvs()
        self.history_runs = []
        for path in self.history_paths:
            run = self.load_run(path)
            if run:
                self.history_runs.append((path, run))

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)

        rospy.loginfo("Validation run viewer selected CSV: %s", self.input_csv if self.input_csv else "(none)")
        rospy.loginfo("Loaded selected run points: %d", len(self.selected_run))
        rospy.loginfo("Loaded previous run(s): %d", len(self.history_runs))

    def resolve_history_csvs(self):
        paths = []
        for item in str(self.history_csvs).split(","):
            item = item.strip()
            if item:
                paths.append(norm_path(item))
        if self.history_glob:
            paths.extend(norm_path(path) for path in glob.glob(os.path.expanduser(str(self.history_glob))))

        selected = norm_path(self.input_csv) if self.input_csv else ""
        unique = []
        seen = set()
        for path in sorted(paths):
            if path == selected or path in seen:
                continue
            seen.add(path)
            unique.append(path)
        return unique

    def load_run(self, path):
        if not path or not os.path.exists(path):
            if path:
                rospy.logwarn("Validation CSV does not exist: %s", path)
            return []

        points = []
        target_points = []
        with open(path, "r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                try:
                    points.append(self.apply_display_offset(float(row["ego_x"]), float(row["ego_y"])))
                    if "path_x" in row and "path_y" in row:
                        target_points.append(self.apply_display_offset(float(row["path_x"]), float(row["path_y"])))
                except (KeyError, TypeError, ValueError):
                    continue

        return {"ego": points, "target": target_points}

    def apply_display_offset(self, x, y):
        return x + self.display_offset_x, y + self.display_offset_y

    def display_z(self, z):
        return z + self.display_offset_z

    def marker_base(self, marker_id, ns, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def line_marker(self, marker_id, ns, points, rgba, width, z):
        marker = self.marker_base(marker_id, ns, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.points = [Point(x, y, z) for x, y in points]
        return marker

    def sphere_marker(self, marker_id, ns, point, rgba, scale, z):
        marker = self.marker_base(marker_id, ns, Marker.SPHERE)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.pose.position = Point(point[0], point[1], z)
        return marker

    def text_marker(self, marker_id, text, point, rgba, z):
        marker = self.marker_base(marker_id, "validation_saved_run_labels", Marker.TEXT_VIEW_FACING)
        marker.scale.z = 1.0
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        marker.pose.position = Point(point[0], point[1], z)
        marker.text = text
        return marker

    def selected_path_msg(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        if not self.selected_run:
            return path

        for x, y in self.selected_run["ego"]:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.display_z(self.z_offset)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def build_markers(self):
        markers = MarkerArray()
        marker_id = 0

        palette = [
            (0.72, 0.72, 0.72, 0.65),
            (0.35, 0.95, 0.35, 0.65),
            (1.0, 0.35, 0.35, 0.65),
            (0.75, 0.35, 1.0, 0.65),
            (0.2, 0.8, 1.0, 0.65),
        ]
        for index, (path, run) in enumerate(self.history_runs):
            ego_points = run["ego"]
            if len(ego_points) < 2:
                continue
            color = palette[index % len(palette)]
            markers.markers.append(
                self.line_marker(marker_id, "validation_saved_previous_runs", ego_points, color, 0.18, self.display_z(self.z_offset))
            )
            marker_id += 1
            markers.markers.append(
                self.text_marker(
                    marker_id,
                    os.path.basename(path),
                    ego_points[-1],
                    color,
                    self.display_z(self.z_offset + 1.1),
                )
            )
            marker_id += 1

        if self.selected_run and len(self.selected_run["target"]) >= 2:
            markers.markers.append(
                self.line_marker(
                    marker_id,
                    "validation_saved_target_path",
                    self.selected_run["target"],
                    (0.0, 0.65, 1.0, 0.85),
                    0.24,
                    self.display_z(self.z_offset - 0.15),
                )
            )
            marker_id += 1

        if self.selected_run and len(self.selected_run["ego"]) >= 2:
            ego_points = self.selected_run["ego"]
            markers.markers.append(
                self.line_marker(
                    marker_id,
                    "validation_saved_selected_run",
                    ego_points,
                    (1.0, 0.85, 0.0, 1.0),
                    0.34,
                    self.display_z(self.z_offset + 0.1),
                )
            )
            marker_id += 1
            markers.markers.append(
                self.sphere_marker(marker_id, "validation_saved_start", ego_points[0], (0.0, 1.0, 0.0, 1.0), 1.0, self.display_z(self.z_offset + 0.4))
            )
            marker_id += 1
            markers.markers.append(
                self.sphere_marker(marker_id, "validation_saved_end", ego_points[-1], (1.0, 0.0, 0.0, 1.0), 1.0, self.display_z(self.z_offset + 0.4))
            )
            marker_id += 1
            markers.markers.append(
                self.text_marker(
                    marker_id,
                    "selected: %s" % os.path.basename(self.input_csv),
                    ego_points[-1],
                    (1.0, 0.85, 0.0, 1.0),
                    self.display_z(self.z_offset + 1.4),
                )
            )

        return markers

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.build_markers())
            self.path_pub.publish(self.selected_path_msg())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("validation_run_viewer")
    try:
        ValidationRunViewer().run()
    except rospy.ROSInterruptException:
        pass
