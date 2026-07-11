#!/usr/bin/env python3

import math

import rospy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


class MissionPlannerMinimalInputPublisher:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.request_delay = float(rospy.get_param("~request_delay", 0.5))
        self.allow_modification = bool(rospy.get_param("~allow_modification", True))
        self.route_service = rospy.get_param(
            "~route_service", "/planning/mission_planning/mission_planner/set_waypoint_route"
        )

        self.start_x = float(rospy.get_param("~start_x", 0.0))
        self.start_y = float(rospy.get_param("~start_y", 0.0))
        self.start_yaw = float(rospy.get_param("~start_yaw", 0.0))
        self.goal_x = float(rospy.get_param("~goal_x", 60.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))

        odometry_topic = rospy.get_param("~odometry_topic", "/localization/kinematic_state")
        operation_mode_topic = rospy.get_param(
            "~operation_mode_topic", "/system/operation_mode/state"
        )

        self.odom_pub = rospy.Publisher(odometry_topic, Odometry, queue_size=1)
        self.operation_mode_pub = rospy.Publisher(
            operation_mode_topic, OperationModeState, queue_size=1, latch=True
        )
        self.request_sent = False
        self.start_time = rospy.Time.now()

        rospy.loginfo(
            "mission_planner_minimal_input_publisher: start=(%.2f, %.2f) goal=(%.2f, %.2f)",
            self.start_x,
            self.start_y,
            self.goal_x,
            self.goal_y,
        )

    def make_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation = quaternion_from_yaw(yaw)
        return pose

    def make_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = "base_link"
        odom.pose.pose = self.make_pose(self.start_x, self.start_y, self.start_yaw)
        return odom

    def make_operation_mode(self, stamp):
        msg = OperationModeState()
        msg.stamp = stamp
        msg.mode = OperationModeState.AUTONOMOUS
        msg.is_autoware_control_enabled = True
        msg.is_in_transition = False
        msg.is_stop_mode_available = True
        msg.is_autonomous_mode_available = True
        msg.is_local_mode_available = True
        msg.is_remote_mode_available = True
        return msg

    def maybe_send_route_request(self):
        if self.request_sent:
            return
        if (rospy.Time.now() - self.start_time).to_sec() < self.request_delay:
            return

        try:
            rospy.wait_for_service(self.route_service, timeout=2.0)
            set_route = rospy.ServiceProxy(self.route_service, SetWaypointRoute)
            req = SetWaypointRouteRequest()
            req.header.stamp = rospy.Time.now()
            req.header.frame_id = self.frame_id
            req.goal_pose = self.make_pose(self.goal_x, self.goal_y, self.goal_yaw)
            req.waypoints = []
            req.uuid.uuid = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
            req.allow_modification = self.allow_modification
            res = set_route(req)
            self.request_sent = True
            rospy.loginfo(
                "mission_planner_minimal_input_publisher: set_waypoint_route success=%s message=%s",
                res.status.success,
                res.status.message,
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "mission_planner_minimal_input_publisher: waiting to call route service: %s",
                exc,
            )

    def spin(self):
        rate = rospy.Rate(max(self.publish_rate, 0.1))
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.odom_pub.publish(self.make_odometry(stamp))
            self.operation_mode_pub.publish(self.make_operation_mode(stamp))
            self.maybe_send_route_request()
            rate.sleep()


def main():
    rospy.init_node("mission_planner_minimal_input_publisher")
    MissionPlannerMinimalInputPublisher().spin()


if __name__ == "__main__":
    main()
