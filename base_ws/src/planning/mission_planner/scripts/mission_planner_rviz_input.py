#!/usr/bin/env python3

import copy
import threading
import uuid

import rospy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.msg import Scenario, VelocityLimit
from autoware_internal_planning_msgs.srv import (
    ClearRoute,
    SetWaypointRoute,
    SetWaypointRouteRequest,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def color(red, green, blue, alpha=1.0):
    value = ColorRGBA()
    value.r = float(red)
    value.g = float(green)
    value.b = float(blue)
    value.a = float(alpha)
    return value


class MissionPlannerRvizInput:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.default_z = float(rospy.get_param("~default_z", 28.5))
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 10.0)), 0.1)
        self.max_velocity = max(float(rospy.get_param("~max_velocity", 4.1666665)), 0.0)
        self.allow_modification = bool(rospy.get_param("~allow_modification", False))
        self.publish_static_odometry = bool(
            rospy.get_param("~publish_static_odometry", True)
        )
        self.clear_route_on_initial_pose = bool(
            rospy.get_param("~clear_route_on_initial_pose", True)
        )
        self.clear_service = rospy.get_param(
            "~clear_service", "/planning/mission_planning/mission_planner/clear_route"
        )
        self.route_service = rospy.get_param(
            "~route_service",
            "/planning/mission_planning/mission_planner/set_waypoint_route",
        )
        self.lock = threading.RLock()
        self.start_pose = None
        self.current_pose = None
        self.goal_pose = None

        self.odometry_pub = rospy.Publisher(
            "/localization/kinematic_state", Odometry, queue_size=1
        )
        self.operation_mode_pub = rospy.Publisher(
            "/system/operation_mode/state",
            OperationModeState,
            queue_size=1,
            latch=True,
        )
        self.scenario_pub = rospy.Publisher(
            "/planning/scenario_planning/scenario", Scenario, queue_size=1, latch=True
        )
        self.velocity_limit_pub = rospy.Publisher(
            "/planning/scenario_planning/max_velocity",
            VelocityLimit,
            queue_size=1,
            latch=True,
        )
        self.marker_pub = rospy.Publisher(
            "/planning/mission_planning/rviz/input_markers",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.on_initial_pose)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.on_goal)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.on_timer)

        rospy.loginfo(
            "mission_planner_rviz_input: use 2D Pose Estimate for the %s ego "
            "pose and 2D Nav Goal for the route goal",
            "fixed" if self.publish_static_odometry else "externally driven",
        )

    def normalized_pose(self, pose):
        output = copy.deepcopy(pose)
        if abs(output.position.z) < 1.0e-6:
            output.position.z = self.default_z
        norm = (
            output.orientation.x**2
            + output.orientation.y**2
            + output.orientation.z**2
            + output.orientation.w**2
        ) ** 0.5
        if norm < 1.0e-9:
            output.orientation.w = 1.0
        return output

    def clear_route(self):
        try:
            rospy.wait_for_service(self.clear_service, timeout=2.0)
            response = rospy.ServiceProxy(self.clear_service, ClearRoute)()
            if not response.status.success:
                rospy.logwarn("RViz route clear rejected: %s", response.status.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("RViz route clear unavailable: %s", error)

    def on_initial_pose(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            rospy.logwarn("Ignoring initial pose in frame %s", message.header.frame_id)
            return
        with self.lock:
            self.start_pose = self.normalized_pose(message.pose.pose)
            self.current_pose = copy.deepcopy(self.start_pose)
            self.goal_pose = None
            self.publish_odometry(rospy.Time.now())
            self.publish_markers()
        if self.clear_route_on_initial_pose:
            self.clear_route()
        rospy.loginfo(
            "RViz start pose set: x=%.2f y=%.2f",
            self.start_pose.position.x,
            self.start_pose.position.y,
        )

    def on_goal(self, message):
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            rospy.logwarn("Ignoring goal in frame %s", message.header.frame_id)
            return
        with self.lock:
            if self.start_pose is None:
                rospy.logwarn("Set the start with 2D Pose Estimate before setting a goal")
                return
            self.goal_pose = self.normalized_pose(message.pose)
            self.current_pose = copy.deepcopy(self.start_pose)
            self.publish_odometry(rospy.Time.now())
            self.publish_markers()

        self.clear_route()
        try:
            rospy.wait_for_service(self.route_service, timeout=3.0)
            request = SetWaypointRouteRequest()
            request.header.stamp = rospy.Time.now()
            request.header.frame_id = self.frame_id
            request.goal_pose = copy.deepcopy(self.goal_pose)
            request.waypoints = []
            request.uuid.uuid = list(uuid.uuid4().bytes)
            request.allow_modification = self.allow_modification
            response = rospy.ServiceProxy(self.route_service, SetWaypointRoute)(request)
            if response.status.success:
                rospy.loginfo(
                    "RViz goal accepted: x=%.2f y=%.2f",
                    self.goal_pose.position.x,
                    self.goal_pose.position.y,
                )
            else:
                rospy.logerr("RViz goal rejected: %s", response.status.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("Could not call Mission Planner route service: %s", error)

    def publish_odometry(self, stamp):
        if not self.publish_static_odometry or self.current_pose is None:
            return
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.child_frame_id = "base_link"
        message.pose.pose = copy.deepcopy(self.current_pose)
        self.odometry_pub.publish(message)

    def publish_operation_mode(self, stamp):
        message = OperationModeState()
        message.stamp = stamp
        message.mode = OperationModeState.AUTONOMOUS
        message.is_autoware_control_enabled = True
        message.is_stop_mode_available = True
        message.is_autonomous_mode_available = True
        message.is_local_mode_available = True
        message.is_remote_mode_available = True
        self.operation_mode_pub.publish(message)

    def publish_scenario(self):
        message = Scenario()
        message.current_scenario = Scenario.LANEDRIVING
        message.activating_scenarios = [Scenario.LANEDRIVING]
        self.scenario_pub.publish(message)

    def publish_velocity_limit(self, stamp):
        message = VelocityLimit()
        message.stamp = stamp
        message.max_velocity = self.max_velocity
        message.use_constraints = False
        message.sender = "mission_planner_rviz_input"
        self.velocity_limit_pub.publish(message)

    def arrow_marker(self, marker_id, namespace, pose, marker_color):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose)
        marker.scale.x = 4.0
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color = marker_color
        return marker

    def publish_markers(self):
        markers = []
        if self.start_pose is not None:
            markers.append(
                self.arrow_marker(0, "rviz_start", self.start_pose, color(0.2, 1.0, 0.3))
            )
        if self.goal_pose is not None:
            markers.append(
                self.arrow_marker(1, "rviz_goal", self.goal_pose, color(1.0, 0.2, 0.2))
            )
        self.marker_pub.publish(MarkerArray(markers=markers))

    def on_timer(self, _event):
        stamp = rospy.Time.now()
        with self.lock:
            self.publish_odometry(stamp)
            self.publish_markers()
            self.publish_operation_mode(stamp)
            self.publish_scenario()
            self.publish_velocity_limit(stamp)


def main():
    rospy.init_node("mission_planner_rviz_input")
    MissionPlannerRvizInput()
    rospy.spin()


if __name__ == "__main__":
    main()
