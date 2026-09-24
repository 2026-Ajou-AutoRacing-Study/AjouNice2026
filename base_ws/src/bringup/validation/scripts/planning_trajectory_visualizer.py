#!/usr/bin/env python3

import copy

import rospy
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(
        r=float(red),
        g=float(green),
        b=float(blue),
        a=float(alpha),
    )


class PlanningTrajectoryVisualizer:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/"
            "motion_velocity_planner/trajectory",
        )
        self.path_topic = rospy.get_param(
            "~path_topic", "/planning/debug/final_trajectory/path"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/planning/debug/final_trajectory/markers"
        )
        self.stop_velocity = max(
            float(rospy.get_param("~stop_velocity_threshold_mps", 0.05)), 0.0
        )
        self.nominal_velocity = max(
            float(rospy.get_param("~nominal_velocity_mps", 4.1666665)), 0.1
        )
        self.path_publisher = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )
        self.marker_publisher = rospy.Publisher(
            self.marker_topic, MarkerArray, queue_size=1, latch=True
        )
        rospy.Subscriber(
            self.input_topic, Trajectory, self.on_trajectory, queue_size=1
        )
        rospy.loginfo(
            "planning_trajectory_visualizer: input=%s path=%s markers=%s",
            self.input_topic,
            self.path_topic,
            self.marker_topic,
        )

    @staticmethod
    def display_points(message):
        return [
            (copy.deepcopy(point.pose), float(point.longitudinal_velocity_mps))
            for point in message.points
        ]

    def velocity_color(self, velocity):
        speed = max(float(velocity), 0.0)
        if speed <= self.stop_velocity:
            return color(1.0, 0.03, 0.03, 1.0)
        ratio = min(speed / self.nominal_velocity, 1.0)
        if ratio < 0.5:
            blend = ratio / 0.5
            return color(1.0, 0.35 + 0.55 * blend, 0.02, 1.0)
        blend = (ratio - 0.5) / 0.5
        return color(1.0 - 0.82 * blend, 0.90, 0.04 + 0.24 * blend, 1.0)

    def marker(self, namespace, marker_id, marker_type, header):
        marker = Marker()
        marker.header = copy.deepcopy(header)
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0)
        return marker

    def on_trajectory(self, message):
        if not message.points:
            return
        self.publish_trajectory(message)

    def publish_trajectory(self, message):
        if rospy.is_shutdown():
            return
        display_points = self.display_points(message)
        if not display_points:
            return

        path = Path()
        path.header = copy.deepcopy(message.header)
        path.poses = []
        for point_pose, _velocity in display_points:
            pose = PoseStamped()
            pose.header = copy.deepcopy(message.header)
            pose.pose = copy.deepcopy(point_pose)
            path.poses.append(pose)
        self.path_publisher.publish(path)

        line = self.marker("final_trajectory_line", 0, Marker.LINE_STRIP, message.header)
        line.scale.x = 1.1
        points = self.marker("final_trajectory_points", 0, Marker.SPHERE_LIST, message.header)
        points.scale.x = points.scale.y = points.scale.z = 1.6
        for point_pose, velocity in display_points:
            position = point_pose.position
            value = Point(x=position.x, y=position.y, z=position.z + 0.75)
            marker_color = self.velocity_color(velocity)
            line.points.append(value)
            line.colors.append(marker_color)
            points.points.append(value)
            points.colors.append(marker_color)

        start = self.marker("final_trajectory_ends", 0, Marker.SPHERE, message.header)
        start.pose.position = copy.deepcopy(display_points[0][0].position)
        start.pose.position.z += 1.5
        start.scale.x = start.scale.y = start.scale.z = 3.0
        start.color = color(0.1, 1.0, 0.25, 1.0)

        horizon = self.marker(
            "final_trajectory_horizon", 0, Marker.SPHERE, message.header
        )
        horizon.pose.position = copy.deepcopy(display_points[-1][0].position)
        horizon.pose.position.z += 1.5
        horizon.scale.x = horizon.scale.y = horizon.scale.z = 3.0
        horizon.color = color(1.0, 0.12, 0.12, 1.0)

        self.marker_publisher.publish(
            MarkerArray(markers=[line, points, start, horizon])
        )
        velocities = [velocity for _pose, velocity in display_points]
        rospy.logdebug_throttle(
            3.0,
            "final trajectory visualized: points=%d velocity=%.2f..%.2f m/s",
            len(display_points),
            min(velocities),
            max(velocities),
        )


def main():
    rospy.init_node("planning_trajectory_visualizer")
    PlanningTrajectoryVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
