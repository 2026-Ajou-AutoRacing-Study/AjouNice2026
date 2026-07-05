#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryVisualizer:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", rospy.get_param("path_topic", "/recorded_trajectory_path"))
        self.marker_topic = rospy.get_param(
            "~marker_topic",
            rospy.get_param("marker_topic", "/recorded_trajectory_marker"),
        )
        self.line_topic = rospy.get_param(
            "~line_topic",
            rospy.get_param("line_topic", "/recorded_trajectory_line"),
        )
        self.sphere_stride = int(rospy.get_param("~sphere_stride", rospy.get_param("sphere_stride", 20)))

        self.pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.line_pub = rospy.Publisher(self.line_topic, Marker, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.loginfo("%s 토픽을 %s 및 %s 마커로 시각화합니다.", self.path_topic, self.marker_topic, self.line_topic)

    def make_line_marker(self, path):
        marker = Marker()
        marker.header = path.header
        marker.ns = "recorded_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.15
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [pose.pose.position for pose in path.poses]
        return marker

    def make_sphere_marker(self, path, marker_id, pose):
        marker = Marker()
        marker.header = path.header
        marker.ns = "recorded_trajectory_points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = 1.0
        marker.color.g = 0.45
        marker.color.b = 0.05
        marker.color.a = 0.9
        marker.pose.position = Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        marker.pose.orientation.w = 1.0
        return marker

    def make_endpoint_marker(self, path, marker_id, pose, ns, red, green, blue, scale):
        marker = Marker()
        marker.header = path.header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.pose.position = Point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.5)
        marker.pose.orientation.w = 1.0
        return marker

    def make_direction_marker(self, path, marker_id, start_pose, end_pose):
        marker = Marker()
        marker.header = path.header
        marker.ns = "recorded_trajectory_direction"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.8
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.r = 0.0
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [
            Point(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z + 0.8),
            Point(end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z + 0.8),
        ]
        return marker

    def path_callback(self, path):
        markers = MarkerArray()
        line_marker = self.make_line_marker(path)
        markers.markers.append(line_marker)

        if path.poses:
            markers.markers.append(
                self.make_endpoint_marker(path, 100000, path.poses[0], "recorded_trajectory_start", 0.0, 1.0, 0.0, 1.5)
            )
            markers.markers.append(
                self.make_endpoint_marker(path, 100001, path.poses[-1], "recorded_trajectory_end", 1.0, 0.0, 0.0, 1.5)
            )

        if len(path.poses) >= 8:
            markers.markers.append(self.make_direction_marker(path, 100002, path.poses[0], path.poses[7]))

        marker_id = 1
        stride = max(1, self.sphere_stride)
        for index, pose in enumerate(path.poses):
            if index % stride == 0:
                markers.markers.append(self.make_sphere_marker(path, marker_id, pose))
                marker_id += 1

        self.pub.publish(markers)
        self.line_pub.publish(line_marker)
        rospy.loginfo_throttle(5.0, "경로 pose %d개에 대한 궤적 마커를 발행했습니다.", len(path.poses))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("trajectory_visualizer")
    try:
        TrajectoryVisualizer().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("trajectory_visualizer 실행 실패: %s", exc)
