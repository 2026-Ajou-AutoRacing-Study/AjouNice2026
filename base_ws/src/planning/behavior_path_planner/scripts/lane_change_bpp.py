#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import LaneletRoute, Path
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import LateralOffset
from tier4_rtc_msgs.msg import Module, State

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_lane_change_tools import build_route_lane_change_path, route_lane_change_target
from bpp_route_tools import forward_lane_sequence
from bpp_map_tools import CenterlineMap
from bpp_path_tools import (
    PathModuleBase,
    apply_s_curve_shift,
    object_pose,
    project_pose_to_path,
    to_autoware_path,
)
from rtc_interface_ros1 import RTCInterfaceROS1, RTCSceneController


class LaneChangeBPP(PathModuleBase):
    def __init__(self):
        self.external_request = None
        self.external_request_time = None
        self.objects = None
        self.route = None
        self.odometry = None
        self.centerline_map = None

        self.lane_width = float(rospy.get_param("~lane_width", 3.5))
        self.direction = rospy.get_param("~direction", "none").lower()
        self.target_lateral_offset = float(rospy.get_param("~target_lateral_offset", 0.0))
        self.min_request_offset = float(rospy.get_param("~min_request_offset", 0.1))
        self.request_timeout = float(rospy.get_param("~request_timeout", 0.0))
        self.prepare_distance = float(rospy.get_param("~prepare_distance", 3.0))
        self.lane_change_distance = float(rospy.get_param("~lane_change_distance", 28.0))
        self.target_forward_length = float(
            rospy.get_param("~target_forward_length", 300.0)
        )
        self.transition_interval = float(rospy.get_param("~transition_interval", 1.0))
        self.target_interval = float(rospy.get_param("~target_interval", 4.0))
        self.prepare_velocity_ratio = float(
            rospy.get_param("~prepare_velocity_ratio", 1.0)
        )
        self.route_driven = bool(rospy.get_param("~route_driven", True))
        self.map_file = rospy.get_param("~map_file", "")
        self.projector_file = rospy.get_param("~projector_file", "")
        self.projector_type = rospy.get_param("~map_projector_type", "MGRS")
        self.enable_safety_check = bool(rospy.get_param("~enable_safety_check", True))
        self.safety_lateral_margin = float(rospy.get_param("~safety_lateral_margin", 1.2))
        self.safety_forward_margin = float(rospy.get_param("~safety_forward_margin", 10.0))
        self.enable_rtc_left = bool(rospy.get_param("~rtc/enable_left", False))
        self.enable_rtc_right = bool(rospy.get_param("~rtc/enable_right", False))

        lateral_offset_topic = rospy.get_param("~lateral_offset_topic", "~input/lateral_offset")
        objects_topic = rospy.get_param("~objects_topic", "~input/objects")
        route_topic = rospy.get_param("~route_topic", "~input/route")
        odometry_topic = rospy.get_param("~odometry_topic", "~input/odometry")

        rospy.Subscriber(lateral_offset_topic, LateralOffset, self.lateral_offset_callback, queue_size=1)
        rospy.Subscriber(objects_topic, PredictedObjects, self.objects_callback, queue_size=1)
        rospy.Subscriber(route_topic, LaneletRoute, self.route_callback, queue_size=1)
        rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback, queue_size=1)
        self.turn_pub = rospy.Publisher("~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1)
        self.candidate_path_pubs = {
            "left": rospy.Publisher("~output/candidate_path_left", Path, queue_size=1),
            "right": rospy.Publisher("~output/candidate_path_right", Path, queue_size=1),
        }
        self.reference_path_pubs = {
            "left": rospy.Publisher("~output/reference_path_left", Path, queue_size=1),
            "right": rospy.Publisher("~output/reference_path_right", Path, queue_size=1),
        }
        self.rtc_interfaces = {
            "left": RTCInterfaceROS1(
                Module.LANE_CHANGE_LEFT,
                rospy.get_param(
                    "~rtc/left/status_topic",
                    "/planning/cooperate_status/lane_change_left",
                ),
                rospy.get_param(
                    "~rtc/left/command_service",
                    "/planning/cooperate_commands/lane_change_left",
                ),
                rospy.get_param(
                    "~rtc/left/auto_mode_service",
                    "/planning/enable_auto_mode/lane_change_left",
                ),
                self.enable_rtc_left,
            ),
            "right": RTCInterfaceROS1(
                Module.LANE_CHANGE_RIGHT,
                rospy.get_param(
                    "~rtc/right/status_topic",
                    "/planning/cooperate_status/lane_change_right",
                ),
                rospy.get_param(
                    "~rtc/right/command_service",
                    "/planning/cooperate_commands/lane_change_right",
                ),
                rospy.get_param(
                    "~rtc/right/auto_mode_service",
                    "/planning/enable_auto_mode/lane_change_right",
                ),
                self.enable_rtc_right,
            ),
        }
        self.rtc_controller = RTCSceneController(self.rtc_interfaces)

        if self.map_file:
            try:
                self.centerline_map = CenterlineMap.load(
                    self.map_file,
                    self.projector_file,
                    False,
                    self.projector_type,
                )
                rospy.loginfo(
                    "lane_change_bpp: loaded %d lane centerlines",
                    self.centerline_map.lane_count(),
                )
            except Exception as exc:
                rospy.logwarn("lane_change_bpp: failed to load map: %s", exc)

        super().__init__("lane_change_bpp")

    def route_callback(self, msg):
        self.route = msg

    def odometry_callback(self, msg):
        self.odometry = msg

    def lateral_offset_callback(self, msg):
        self.external_request = msg
        self.external_request_time = rospy.Time.now()

    def objects_callback(self, msg):
        self.objects = msg

    def requested_offset(self):
        if self.external_request is not None:
            if self.request_timeout <= 0.0:
                return self.external_request.lateral_offset
            age = (rospy.Time.now() - self.external_request_time).to_sec()
            if age <= self.request_timeout:
                return self.external_request.lateral_offset

        if abs(self.target_lateral_offset) >= self.min_request_offset:
            return self.target_lateral_offset
        if self.direction == "left":
            return abs(self.lane_width)
        if self.direction == "right":
            return -abs(self.lane_width)
        return 0.0

    def is_lane_change_safe(self, path, target_offset):
        if not self.enable_safety_check or self.objects is None:
            return True

        check_end = self.prepare_distance + self.lane_change_distance + self.safety_forward_margin
        for obj in self.objects.objects:
            projection = project_pose_to_path(path, object_pose(obj))
            if projection is None:
                continue
            if projection.longitudinal_distance < 0.0 or projection.longitudinal_distance > check_end:
                continue
            target_lane_error = abs(projection.signed_lateral_distance - target_offset)
            if target_lane_error <= self.safety_lateral_margin:
                return False
        return True

    def publish_turn_signal(self, stamp, target_offset):
        msg = TurnIndicatorsCommand()
        msg.stamp = stamp
        if target_offset > self.min_request_offset:
            msg.command = TurnIndicatorsCommand.ENABLE_LEFT
        elif target_offset < -self.min_request_offset:
            msg.command = TurnIndicatorsCommand.ENABLE_RIGHT
        else:
            msg.command = TurnIndicatorsCommand.NO_COMMAND
        self.turn_pub.publish(msg)

    def publish_rtc_paths(self, direction, reference, candidate, stamp):
        reference.header.stamp = stamp
        candidate.header.stamp = stamp
        self.reference_path_pubs[direction].publish(to_autoware_path(reference))
        self.candidate_path_pubs[direction].publish(to_autoware_path(candidate))

    def apply_rtc_gate(
        self,
        reference,
        candidate,
        safe,
        target_offset,
        signature,
        stamp,
        description,
    ):
        direction = "left" if target_offset > 0.0 else "right"
        activated, state, _, auto_mode = self.rtc_controller.evaluate(
            direction,
            signature,
            safe,
            self.prepare_distance,
            self.prepare_distance + self.lane_change_distance,
            stamp,
        )
        self.publish_rtc_paths(direction, reference, candidate, stamp)
        self.publish_turn_signal(stamp, target_offset)

        state_name = {
            State.WAITING_FOR_EXECUTION: "waiting",
            State.RUNNING: "running",
            State.ABORTING: "aborting",
            State.SUCCEEDED: "succeeded",
            State.FAILED: "failed",
        }.get(state, "unknown")
        mode_name = "auto" if auto_mode else "manual"
        safety_name = "safe" if safe else "unsafe"
        if not safe:
            description = "lane_change: blocked by object in target lane; " + description
        status = "{}; rtc={} {} {}, activated={}".format(
            description,
            mode_name,
            safety_name,
            state_name,
            activated,
        )
        return (candidate if activated else reference), status

    def process_path(self, path, stamp):
        route_target = None
        if self.route_driven and self.odometry is not None:
            route_target = route_lane_change_target(
                self.route,
                self.centerline_map,
                self.odometry.pose.pose,
            )
        if route_target is not None:
            target_offset = route_target.lateral_offset
            target_lane_ids = forward_lane_sequence(
                self.route,
                self.centerline_map,
                route_target.section_index,
                route_target.target_lane_id,
            )
            candidate = build_route_lane_change_path(
                path,
                self.centerline_map,
                self.odometry.pose.pose,
                route_target,
                self.prepare_distance,
                self.lane_change_distance,
                self.target_forward_length,
                self.transition_interval,
                self.target_interval,
                self.prepare_velocity_ratio,
                target_lane_ids,
            )
            safe = self.is_lane_change_safe(path, target_offset)
            return self.apply_rtc_gate(
                path,
                candidate,
                safe,
                target_offset,
                (
                    "route",
                    int(route_target.source_lane_id),
                    int(route_target.target_lane_id),
                    str(route_target.direction),
                ),
                stamp,
                "lane_change: route {} -> {} ({})".format(
                    route_target.source_lane_id,
                    route_target.target_lane_id,
                    route_target.direction,
                ),
            )

        target_offset = self.requested_offset()
        if abs(target_offset) < self.min_request_offset:
            self.rtc_controller.complete(stamp)
            self.publish_turn_signal(stamp, 0.0)
            return path, "lane_change: pass-through, no request"

        candidate = apply_s_curve_shift(
            path,
            target_offset,
            self.prepare_distance,
            self.prepare_distance + self.lane_change_distance,
        )
        safe = self.is_lane_change_safe(path, target_offset)
        return self.apply_rtc_gate(
            path,
            candidate,
            safe,
            target_offset,
            ("offset", round(float(target_offset), 3)),
            stamp,
            "lane_change: applied lateral offset {:.2f} m".format(target_offset),
        )


def main():
    rospy.init_node("lane_change_bpp")
    LaneChangeBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
