#!/usr/bin/env python3

import math
from copy import deepcopy

import rospy
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red, green, blue, alpha=1.0):
    return ColorRGBA(r=float(red), g=float(green), b=float(blue), a=float(alpha))


def velocity_color(velocity, maximum):
    ratio = max(0.0, min(1.0, float(velocity) / max(float(maximum), 0.1)))
    if ratio < 0.5:
        local = ratio / 0.5
        return color(1.0, local, 0.05, 0.98)
    local = (ratio - 0.5) / 0.5
    return color(1.0 - local, 1.0, 0.1, 0.98)


def cumulative_lengths(points):
    values = [0.0]
    for first, second in zip(points[:-1], points[1:]):
        values.append(
            values[-1]
            + math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        )
    return values


class MotionVelocityPlannerVisualizer:
    def __init__(self):
        input_topic = rospy.get_param(
            "~input_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory",
        )
        output_topic = rospy.get_param(
            "~output_trajectory_topic",
            "/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/trajectory",
        )
        objects_topic = rospy.get_param(
            "~objects_topic", "/perception/object_recognition/objects"
        )
        odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/kinematic_state"
        )
        self.input_trajectory = None
        self.output_trajectory = None
        self.objects = None
        self.odometry = None
        self.publisher = rospy.Publisher(
            "/validation/motion_velocity_planner/markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        rospy.Subscriber(input_topic, Trajectory, self.on_input, queue_size=1)
        rospy.Subscriber(output_topic, Trajectory, self.on_output, queue_size=1)
        rospy.Subscriber(objects_topic, PredictedObjects, self.on_objects, queue_size=1)
        rospy.Subscriber(odometry_topic, Odometry, self.on_odometry, queue_size=1)

    def marker(self, namespace, marker_id, marker_type, frame_id="map"):
        value = Marker()
        value.header.frame_id = frame_id or "map"
        value.header.stamp = rospy.Time.now()
        value.ns = namespace
        value.id = marker_id
        value.type = marker_type
        value.action = Marker.ADD
        value.pose.orientation.w = 1.0
        return value

    def on_input(self, message):
        if message.points:
            self.input_trajectory = message
            self.publish()

    def on_output(self, message):
        if message.points:
            self.output_trajectory = message
            self.publish()

    def on_objects(self, message):
        self.objects = message
        self.publish()

    def on_odometry(self, message):
        self.odometry = message
        self.publish()

    def publish(self):
        if self.input_trajectory is None or self.output_trajectory is None:
            return
        frame_id = self.output_trajectory.header.frame_id or "map"
        markers = MarkerArray()

        input_marker = self.marker("path_optimizer_input_points", 0, Marker.SPHERE_LIST, frame_id)
        input_marker.scale.x = input_marker.scale.y = input_marker.scale.z = 0.38
        input_marker.color = color(0.72, 0.75, 0.8, 0.82)
        input_marker.points = [
            Point(
                x=item.pose.position.x,
                y=item.pose.position.y,
                z=item.pose.position.z + 0.10,
            )
            for item in self.input_trajectory.points
        ]
        markers.markers.append(input_marker)

        maximum_velocity = max(
            [abs(item.longitudinal_velocity_mps) for item in self.output_trajectory.points]
            + [0.1]
        )
        output_marker = self.marker("mvp_velocity_points", 0, Marker.SPHERE_LIST, frame_id)
        output_marker.scale.x = output_marker.scale.y = output_marker.scale.z = 0.20
        for item in self.output_trajectory.points:
            output_marker.points.append(
                Point(
                    x=item.pose.position.x,
                    y=item.pose.position.y,
                    z=item.pose.position.z + 0.22,
                )
            )
            output_marker.colors.append(
                velocity_color(abs(item.longitudinal_velocity_mps), maximum_velocity)
            )
        markers.markers.append(output_marker)

        lengths = cumulative_lengths(self.output_trajectory.points)
        stop_index = next(
            (
                index
                for index, item in enumerate(self.output_trajectory.points)
                if abs(item.longitudinal_velocity_mps) < 1.0e-3
            ),
            None,
        )
        if stop_index is not None:
            stop = self.marker("first_stop", 0, Marker.SPHERE, frame_id)
            stop.pose.position = deepcopy(
                self.output_trajectory.points[stop_index].pose.position
            )
            stop.pose.position.z += 0.65
            stop.scale.x = stop.scale.y = stop.scale.z = 0.72
            stop.color = color(1.0, 0.05, 0.05, 1.0)
            markers.markers.append(stop)

        if self.objects is not None:
            for index, detected in enumerate(self.objects.objects):
                obstacle = self.marker("objects", index, Marker.CUBE, frame_id)
                obstacle.pose = deepcopy(
                    detected.kinematics.initial_pose_with_covariance.pose
                )
                obstacle.pose.position.z += 0.5 * max(detected.shape.dimensions.z, 0.2)
                obstacle.scale.x = max(detected.shape.dimensions.x, 0.2)
                obstacle.scale.y = max(detected.shape.dimensions.y, 0.2)
                obstacle.scale.z = max(detected.shape.dimensions.z, 0.2)
                obstacle.color = color(1.0, 0.45, 0.05, 0.72)
                markers.markers.append(obstacle)

        if self.odometry is not None:
            ego = self.marker("ego", 0, Marker.ARROW, frame_id)
            ego.pose = deepcopy(self.odometry.pose.pose)
            ego.pose.position.z += 0.8
            ego.scale.x = 4.5
            ego.scale.y = 1.7
            ego.scale.z = 1.0
            ego.color = color(0.15, 0.65, 1.0, 0.98)
            markers.markers.append(ego)

        summary = self.marker("summary", 0, Marker.TEXT_VIEW_FACING, frame_id)
        anchor = self.output_trajectory.points[min(10, len(self.output_trajectory.points) - 1)]
        summary.pose.position = deepcopy(anchor.pose.position)
        summary.pose.position.z += 3.0
        summary.scale.z = 0.70
        summary.color = color(1.0, 1.0, 1.0, 1.0)
        stop_text = "none" if stop_index is None else "{:.1f} m".format(lengths[stop_index])
        summary.text = "OPTIMIZER GRAY | MVP SPEED RED-YELLOW-GREEN | FIRST STOP {}".format(
            stop_text
        )
        markers.markers.append(summary)

        if not rospy.is_shutdown():
            self.publisher.publish(markers)


def main():
    rospy.init_node("motion_velocity_planner_visualizer")
    MotionVelocityPlannerVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
