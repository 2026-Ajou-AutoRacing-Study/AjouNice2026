#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def distance2d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class EgoTrajectoryComparisonVisualizer:
    def __init__(self):
        self.path_topic = rospy.get_param("~path_topic", "/recorded_trajectory_path")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.marker_topic = rospy.get_param("~marker_topic", "/mgeo_trajectory/ego_comparison_markers")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.heading_arrow_length = float(rospy.get_param("~heading_arrow_length", 5.0))
        self.ego_offset_x = float(rospy.get_param("~ego_offset_x", 0.0))
        self.ego_offset_y = float(rospy.get_param("~ego_offset_y", 0.0))
        self.ego_offset_z = float(rospy.get_param("~ego_offset_z", 0.0))

        self.path = None
        self.ego = None

        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.loginfo("Comparing ego %s against path %s on %s", self.ego_topic, self.path_topic, self.marker_topic)

    def path_callback(self, msg):
        self.path = msg

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
        marker.lifetime = rospy.Duration(0.25)
        return marker

    def sphere_marker(self, marker_id, namespace, point, scale, red, green, blue):
        marker = self.marker_base(marker_id, namespace, Marker.SPHERE)
        marker.pose.position = Point(point.x, point.y, point.z + 1.2)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        return marker

    def line_marker(self, marker_id, namespace, start, end, width, red, green, blue):
        marker = self.marker_base(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.points = [
            Point(start.x, start.y, start.z + 1.0),
            Point(end.x, end.y, end.z + 1.0),
        ]
        return marker

    def heading_marker(self, marker_id, ego_position, heading_rad):
        marker = self.marker_base(marker_id, "ego_heading", Marker.ARROW)
        marker.scale.x = 0.45
        marker.scale.y = 0.9
        marker.scale.z = 0.9
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [
            Point(ego_position.x, ego_position.y, ego_position.z + 1.6),
            Point(
                ego_position.x + self.heading_arrow_length * math.cos(heading_rad),
                ego_position.y + self.heading_arrow_length * math.sin(heading_rad),
                ego_position.z + 1.6,
            ),
        ]
        return marker

    def text_marker(self, marker_id, ego_position, nearest_distance):
        marker = self.marker_base(marker_id, "ego_path_error_text", Marker.TEXT_VIEW_FACING)
        marker.pose.position = Point(ego_position.x, ego_position.y, ego_position.z + 3.0)
        marker.scale.z = 1.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "cte %.2f m" % nearest_distance
        return marker

    def nearest_path_pose(self, ego_position):
        if self.path is None or not self.path.poses:
            return None, float("inf")

        nearest = None
        best_distance = float("inf")
        for pose in self.path.poses:
            current_distance = distance2d(ego_position, pose.pose.position)
            if current_distance < best_distance:
                best_distance = current_distance
                nearest = pose
        return nearest, best_distance

    def build_markers(self):
        markers = MarkerArray()
        if self.ego is None:
            return markers

        ego_position = Point(
            self.ego.position.x + self.ego_offset_x,
            self.ego.position.y + self.ego_offset_y,
            self.ego.position.z + self.ego_offset_z,
        )
        ego_heading = math.radians(float(self.ego.heading))
        nearest_pose, nearest_distance = self.nearest_path_pose(ego_position)

        markers.markers.append(self.sphere_marker(0, "ego_current_position", ego_position, 1.6, 1.0, 1.0, 1.0))
        markers.markers.append(self.heading_marker(1, ego_position, ego_heading))

        if nearest_pose is not None:
            nearest_position = nearest_pose.pose.position
            markers.markers.append(
                self.sphere_marker(2, "trajectory_nearest_to_ego", nearest_position, 1.3, 1.0, 0.0, 0.0)
            )
            markers.markers.append(
                self.line_marker(3, "ego_to_trajectory_error", ego_position, nearest_position, 0.3, 1.0, 0.0, 1.0)
            )
            markers.markers.append(self.text_marker(4, ego_position, nearest_distance))

        return markers

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.marker_pub.publish(self.build_markers())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ego_trajectory_comparison_visualizer")
    try:
        EgoTrajectoryComparisonVisualizer().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("ego_trajectory_comparison_visualizer failed: %s", exc)
        raise
