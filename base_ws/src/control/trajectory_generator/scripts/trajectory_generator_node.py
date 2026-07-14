#!/usr/bin/env python3
"""Publish one immutable finite reference trajectory from the v44 Lanelet catalog."""

import csv
import hashlib
import json
import os
import sys

import rospy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", "..", ".."))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
from competition_route_loader import RouteLoadError, load_catalog, select_route
from centerline_stitcher import annotate_geometry, resample, stitch
from trajectory_profile import build_profile


REPORT_COLUMNS = [
    "s_m", "x_m", "y_m", "z_m", "yaw_rad", "lanelet_id", "speed_limit_kph",
    "requested_speed_kph", "curvature", "target_speed_mps", "acceleration_mps2",
    "time_from_start_sec",
]
MAP_FRAME = "map"


class TrajectoryGenerator:
    def __init__(self):
        self.config = rospy.get_param("~generator", {})
        self.vehicle = rospy.get_param("~vehicle", {})
        self.config["catalog_csv"] = self._resolve_repo_path(self.config.get("catalog_csv", ""))
        self.config["route_selection_csv"] = self._resolve_repo_path(self.config.get("route_selection_csv", ""))
        self.frame = MAP_FRAME
        self.trajectory_pub = rospy.Publisher("/planning/trajectory/reference", Trajectory, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher("/planning/trajectory/reference_path", Path, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher("/planning/trajectory/reference_markers", MarkerArray, queue_size=1, latch=True)
        self.ids_pub = rospy.Publisher("/planning/trajectory/route_lanelet_ids", Int64MultiArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/planning/trajectory/generator_status", String, queue_size=1, latch=True)
        self.trajectory = None
        self.path = None
        self.markers = None
        self.lanelet_ids = None
        self.status = None
        self._load()
        rate = max(0.1, float(self.config.get("publish_rate_hz", 1.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self._publish)

    @staticmethod
    def _sha256(filename):
        digest = hashlib.sha256()
        with open(filename, "rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        return digest.hexdigest()

    @staticmethod
    def _resolve_repo_path(value):
        """Interpret YAML paths relative to the AjouNice2026 repository root."""
        value = os.path.expandvars(os.path.expanduser(str(value or "")))
        if not value or os.path.isabs(value):
            return value
        return os.path.normpath(os.path.join(REPO_ROOT, value))

    def _artifact_path(self, route_id, profile_id):
        package_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        directory = os.path.join(package_root, "artifacts", "trajectory_generator", route_id, profile_id)
        os.makedirs(directory, exist_ok=True)
        return os.path.join(directory, "profile_report.csv")

    def _write_report(self, points, route_id):
        profile_key = json.dumps(self.config.get("speed_profile", {}), sort_keys=True).encode("utf-8")
        profile_id = hashlib.sha256(profile_key).hexdigest()[:12]
        filename = self._artifact_path(route_id, profile_id)
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=REPORT_COLUMNS)
            writer.writeheader()
            for point in points:
                xyz = point["xyz"]
                writer.writerow({
                    "s_m": "%.6f" % point["s_m"], "x_m": "%.6f" % xyz[0], "y_m": "%.6f" % xyz[1],
                    "z_m": "%.6f" % xyz[2], "yaw_rad": "%.8f" % point["yaw_rad"],
                    "lanelet_id": point["lanelet"]["lanelet_id"],
                    "speed_limit_kph": "%.3f" % point["speed_limit_kph"],
                    "requested_speed_kph": "%.3f" % point["requested_speed_kph"],
                    "curvature": "%.9f" % point["curvature"],
                    "target_speed_mps": "%.6f" % point["target_speed_mps"],
                    "acceleration_mps2": "%.6f" % point["acceleration_mps2"],
                    "time_from_start_sec": "%.6f" % point["time_from_start_sec"],
                })
        return filename, profile_id

    def _load(self):
        try:
            catalog = load_catalog(self.config.get("catalog_csv", ""))
            selected = select_route(catalog, self.config.get("route_selection_csv", ""))
            stitched = stitch(selected, float(self.config.get("stitch_gap_m", 0.25)))
            points = resample(stitched, float(self.config.get("resample_ds_m", 0.5)))
            annotate_geometry(points, float(self.vehicle.get("wheelbase_m", 3.0)))
            build_profile(points, self.config.get("speed_profile", {}))
            report, profile_id = self._write_report(points, self.config.get("route_id", "unnamed_route"))
            self.trajectory, self.path = self._messages(points)
            self.markers = self._markers(points)
            self.lanelet_ids = Int64MultiArray(data=[row["lanelet_id"] for row in selected])
            self.status = String(data=json.dumps({
                "ok": True,
                "route_id": self.config.get("route_id", "unnamed_route"),
                "map_version": self.config.get("map_version", "unknown"),
                "frame_id": self.frame,
                "point_count": len(points),
                "length_m": points[-1]["s_m"],
                "profile_id": profile_id,
                "profile_report_path": report,
                "profile_checksum": self._sha256(report),
                "catalog_checksum": self._sha256(self.config["catalog_csv"]),
                "loop": False,
            }, sort_keys=True))
            rospy.loginfo("trajectory_generator: %d points, %.1f m, report=%s",
                          len(points), points[-1]["s_m"], report)
        except (RouteLoadError, OSError, ValueError, KeyError) as error:
            self.status = String(data=json.dumps({"ok": False, "error": str(error)}, sort_keys=True))
            rospy.logfatal("trajectory_generator load failed: %s", error)
            raise rospy.ROSInitException(str(error))

    def _messages(self, points):
        stamp = rospy.Time.now()
        trajectory = Trajectory()
        trajectory.header.stamp = stamp
        trajectory.header.frame_id = self.frame
        path = Path()
        path.header = trajectory.header
        for source in points:
            point = TrajectoryPoint()
            point.time_from_start = rospy.Duration.from_sec(source["time_from_start_sec"])
            point.pose.position.x, point.pose.position.y, point.pose.position.z = source["xyz"]
            half = source["yaw_rad"] * 0.5
            point.pose.orientation.z = __import__("math").sin(half)
            point.pose.orientation.w = __import__("math").cos(half)
            point.longitudinal_velocity_mps = source["target_speed_mps"]
            point.lateral_velocity_mps = 0.0
            point.acceleration_mps2 = source["acceleration_mps2"]
            point.heading_rate_rps = source["curvature"] * source["target_speed_mps"]
            point.front_wheel_angle_rad = source["front_wheel_angle_rad"]
            point.rear_wheel_angle_rad = 0.0
            trajectory.points.append(point)
            pose = PoseStamped()
            pose.header = trajectory.header
            pose.pose = point.pose
            path.poses.append(pose)
        return trajectory, path

    def _markers(self, points):
        markers = MarkerArray()
        line = Marker()
        line.header.frame_id = self.frame
        line.header.stamp = rospy.Time.now()
        line.ns, line.id, line.type, line.action = "reference", 0, Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.35
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 0.85, 1.0, 1.0
        line.points = [Point(x=p["xyz"][0], y=p["xyz"][1], z=p["xyz"][2] + 0.15) for p in points]
        markers.markers.append(line)
        for marker_id, point, color, label in ((1, points[0], (0.1, 1.0, 0.1), "START"),
                                                (2, points[-1], (1.0, 0.1, 0.1), "FINISH")):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.frame, line.header.stamp
            marker.ns, marker.id, marker.type, marker.action = "reference_gate", marker_id, Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = point["xyz"][0], point["xyz"][1], point["xyz"][2] + 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color[0], color[1], color[2], 0.9
            markers.markers.append(marker)
            text = Marker()
            text.header, text.ns, text.id, text.type, text.action = marker.header, "reference_gate_text", marker_id, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose = marker.pose
            text.pose.position.z += 0.9
            text.scale.z, text.color.r, text.color.g, text.color.b, text.color.a, text.text = 0.6, color[0], color[1], color[2], 1.0, label
            markers.markers.append(text)
        return markers

    def _publish(self, _event):
        self.trajectory.header.stamp = rospy.Time.now()
        self.path.header.stamp = self.trajectory.header.stamp
        self.trajectory_pub.publish(self.trajectory)
        self.path_pub.publish(self.path)
        self.markers_pub.publish(self.markers)
        self.ids_pub.publish(self.lanelet_ids)
        self.status_pub.publish(self.status)


if __name__ == "__main__":
    rospy.init_node("trajectory_generator")
    TrajectoryGenerator()
    rospy.spin()
