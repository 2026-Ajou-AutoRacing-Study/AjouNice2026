#!/usr/bin/env python3

import json
import math
import sys

import rospy
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_planning_msgs.msg import LaneletRoute, RouteState
from std_msgs.msg import String


class MissionBppPipelineValidator:
    def __init__(self):
        self.expected_lane_id = int(rospy.get_param("~expected_lane_id", 1001))
        self.expected_goal_x = float(rospy.get_param("~expected_goal_x", 60.0))
        self.expected_goal_y = float(rospy.get_param("~expected_goal_y", 0.0))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.1))
        self.timeout = float(rospy.get_param("~timeout", 15.0))
        self.started_at = rospy.Time.now()
        self.exit_code = 2
        self.route = None
        self.route_state = None
        self.path = None
        self.module_status = None

        rospy.Subscriber(
            "/planning/mission_planning/route",
            LaneletRoute,
            self.route_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/mission_planning/state",
            RouteState,
            self.route_state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
            PathWithLaneId,
            self.path_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/planning/debug/bpp/module_status",
            String,
            self.module_status_callback,
            queue_size=1,
        )
        rospy.Timer(rospy.Duration(0.1), self.on_timer)

    def route_callback(self, message):
        self.route = message

    def route_state_callback(self, message):
        self.route_state = message

    def path_callback(self, message):
        if message.points:
            self.path = message

    def module_status_callback(self, message):
        self.module_status = message.data

    def ready(self):
        return all(
            value is not None
            for value in (self.route, self.route_state, self.path, self.module_status)
        )

    def on_timer(self, _event):
        if self.ready():
            errors = self.validate()
            if not errors:
                self.exit_code = 0
                rospy.loginfo("MISSION+BPP PIPELINE VALIDATION: PASS")
                rospy.loginfo(
                    "route segments=%d, path points=%d, goal=(%.2f, %.2f), final_velocity=%.2f m/s",
                    len(self.route.segments),
                    len(self.path.points),
                    self.path.points[-1].point.pose.position.x,
                    self.path.points[-1].point.pose.position.y,
                    self.path.points[-1].point.longitudinal_velocity_mps,
                )
                rospy.signal_shutdown("validation passed")
                return
            if self.route_state.state in (RouteState.ABORTED, RouteState.INTERRUPTED):
                self.fail(errors)
                return

        if (rospy.Time.now() - self.started_at).to_sec() > self.timeout:
            errors = self.validate() if self.ready() else ["timed out waiting for pipeline outputs"]
            self.fail(errors)

    def validate(self):
        errors = []
        if self.route_state.state != RouteState.SET:
            errors.append("route state is {}, expected SET".format(self.route_state.state))
        if not self.route.segments:
            errors.append("route has no segments")
        route_lane_ids = {
            int(primitive.id)
            for segment in self.route.segments
            for primitive in segment.primitives
        }
        if self.expected_lane_id not in route_lane_ids:
            errors.append("expected lane {} missing from route".format(self.expected_lane_id))

        if len(self.path.points) < 2:
            errors.append("BPP path has fewer than two points")
            return errors
        path_lane_ids = {
            int(lane_id) for item in self.path.points for lane_id in item.lane_ids
        }
        if self.expected_lane_id not in path_lane_ids:
            errors.append("expected lane {} missing from BPP path".format(self.expected_lane_id))
        if any(
            not math.isfinite(value)
            for item in self.path.points
            for value in (
                item.point.pose.position.x,
                item.point.pose.position.y,
                item.point.longitudinal_velocity_mps,
            )
        ):
            errors.append("BPP path contains a non-finite value")

        endpoint = self.path.points[-1].point.pose.position
        endpoint_error = math.hypot(
            endpoint.x - self.expected_goal_x,
            endpoint.y - self.expected_goal_y,
        )
        if endpoint_error > self.goal_tolerance:
            errors.append("BPP endpoint error is {:.3f} m".format(endpoint_error))
        if abs(self.path.points[-1].point.longitudinal_velocity_mps) > 1.0e-3:
            errors.append("BPP terminal velocity is not zero")

        try:
            statuses = json.loads(self.module_status)
        except (TypeError, ValueError):
            statuses = []
            errors.append("module status is not valid JSON")
        if "goal_planner: disabled" not in statuses:
            errors.append("Goal Planner is not disabled")
        if "side_shift: disabled" not in statuses:
            errors.append("Side Shift is not disabled")
        return errors

    def fail(self, errors):
        self.exit_code = 1
        for error in errors:
            rospy.logerr("MISSION+BPP PIPELINE VALIDATION: %s", error)
        rospy.signal_shutdown("validation failed")


def main():
    rospy.init_node("mission_bpp_pipeline_validator")
    validator = MissionBppPipelineValidator()
    rospy.spin()
    sys.exit(validator.exit_code)


if __name__ == "__main__":
    main()
