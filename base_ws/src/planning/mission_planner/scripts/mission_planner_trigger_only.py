#!/usr/bin/env python3
"""
mission_planner_trigger_only.py

mission_planner_minimal_input_publisher.py에서 odometry 발행 부분만 제거한 버전.
실제 GT localization(localization_morai_gt.launch)이 이미 /localization/kinematic_state를
발행하고 있으므로, 이 노드가 odometry를 같이 쏘면 충돌함.

이 노드는 다음 두 가지만 함:
  1. /system/operation_mode/state 를 AUTONOMOUS 상태로 latched 발행 (mission_planner가
     "ready" 상태로 인식하는 데 필요)
  2. request_delay초 뒤, set_waypoint_route 서비스를 호출해서 route 생성 요청

사용법:
  rosrun mission_planner mission_planner_trigger_only.py \
    _goal_x:=-60.96 _goal_y:=75.0878 _frame_id:=map
"""
import math

import rospy
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_internal_planning_msgs.srv import SetWaypointRoute, SetWaypointRouteRequest
from geometry_msgs.msg import Pose, Quaternion


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


class MissionPlannerTriggerOnly:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.request_delay = float(rospy.get_param("~request_delay", 1.0))
        self.allow_modification = bool(rospy.get_param("~allow_modification", True))
        self.route_service = rospy.get_param(
            "~route_service", "/planning/mission_planning/mission_planner/set_waypoint_route"
        )

        self.goal_x = float(rospy.get_param("~goal_x", 60.0))
        self.goal_y = float(rospy.get_param("~goal_y", 0.0))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))

        operation_mode_topic = rospy.get_param(
            "~operation_mode_topic", "/system/operation_mode/state"
        )
        self.operation_mode_pub = rospy.Publisher(
            operation_mode_topic, OperationModeState, queue_size=1, latch=True
        )
        self.request_sent = False
        self.start_time = rospy.Time.now()

        rospy.loginfo(
            "mission_planner_trigger_only: goal=(%.2f, %.2f), route_service=%s",
            self.goal_x, self.goal_y, self.route_service,
        )

    def make_pose(self, x, y, yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        pose.orientation = quaternion_from_yaw(yaw)
        return pose

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
                "mission_planner_trigger_only: set_waypoint_route success=%s message=%s",
                res.status.success, res.status.message,
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "mission_planner_trigger_only: waiting to call route service: %s", exc,
            )

    def spin(self):
        rate = rospy.Rate(max(self.publish_rate, 0.1))
        while not rospy.is_shutdown():
            stamp = rospy.Time.now()
            self.operation_mode_pub.publish(self.make_operation_mode(stamp))
            self.maybe_send_route_request()
            rate.sleep()


def main():
    rospy.init_node("mission_planner_trigger_only")
    MissionPlannerTriggerOnly().spin()


if __name__ == "__main__":
    main()
