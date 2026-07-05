#!/usr/bin/env python3

import csv
import os

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def package_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def resolve_package_path(path_value):
    if path_value.startswith("$(find ajou_trajectory_recorder)"):
        return path_value.replace("$(find ajou_trajectory_recorder)", package_root(), 1)
    if os.path.isabs(path_value):
        return path_value
    return os.path.join(package_root(), path_value)


class CsvLineMarkerPublisher:
    def __init__(self):
        input_csv = rospy.get_param(
            "~input_csv",
            rospy.get_param("input_csv", "$(find ajou_trajectory_recorder)/data/recorded_trajectory.csv"),
        )
        self.input_csv = resolve_package_path(input_csv)
        self.line_topic = rospy.get_param("~line_topic", rospy.get_param("line_topic", "/recorded_trajectory_line"))
        self.local_line_topic = rospy.get_param(
            "~local_line_topic",
            rospy.get_param("local_line_topic", "/recorded_trajectory_line_local"),
        )
        self.frame_id = rospy.get_param("~frame_id", rospy.get_param("frame_id", "map"))
        self.local_frame_id = rospy.get_param("~local_frame_id", rospy.get_param("local_frame_id", "map"))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.line_width = float(rospy.get_param("~line_width", 0.8))
        self.z_offset = float(rospy.get_param("~z_offset", 1.0))
        self.local_scale = float(rospy.get_param("~local_scale", 1.0))

        self.points = self.load_points()
        self.local_points = self.make_local_points(self.points)
        self.pub = rospy.Publisher(self.line_topic, Marker, queue_size=1, latch=True)
        self.local_pub = rospy.Publisher(self.local_line_topic, Marker, queue_size=1, latch=True)
        rospy.loginfo("%s 파일에서 라인 점 %d개를 읽었습니다.", self.input_csv, len(self.points))

    def load_points(self):
        points = []
        if not os.path.exists(self.input_csv):
            rospy.logwarn("CSV 파일이 없습니다: %s", self.input_csv)
            return points

        with open(self.input_csv, "r", newline="") as csv_file:
            reader = csv.DictReader(csv_file)
            for index, row in enumerate(reader, start=2):
                try:
                    points.append(
                        Point(
                            x=float(row["x"]),
                            y=float(row["y"]),
                            z=float(row.get("z", 0.0) or 0.0) + self.z_offset,
                        )
                    )
                except Exception as exc:
                    rospy.logwarn("잘못된 CSV %d번째 줄을 건너뜁니다: %s", index, exc)
        return points

    def make_local_points(self, points):
        if not points:
            return []
        origin = points[0]
        return [
            Point(
                x=(point.x - origin.x) * self.local_scale,
                y=(point.y - origin.y) * self.local_scale,
                z=(point.z - origin.z) * self.local_scale + self.z_offset,
            )
            for point in points
        ]

    def build_marker(self, points, frame_id, namespace, red, green, blue):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time(0)
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.line_width
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0)
        marker.points = points
        return marker

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            marker = self.build_marker(
                self.points,
                self.frame_id,
                "recorded_trajectory_line_only",
                0.0,
                1.0,
                1.0,
            )
            local_marker = self.build_marker(
                self.local_points,
                self.local_frame_id,
                "recorded_trajectory_line_local",
                1.0,
                0.9,
                0.0,
            )
            self.pub.publish(marker)
            self.local_pub.publish(local_marker)
            rospy.loginfo_throttle(
                5.0,
                "라인 마커 발행 중: global=%d on %s local=%d on %s scale=%.3f",
                len(marker.points),
                self.line_topic,
                len(local_marker.points),
                self.local_line_topic,
                self.local_scale,
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("publish_csv_line_marker")
    try:
        CsvLineMarkerPublisher().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("publish_csv_line_marker 실행 실패: %s", exc)
