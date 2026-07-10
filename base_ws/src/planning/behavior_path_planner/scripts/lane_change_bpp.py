#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_perception_msgs.msg import PredictedObjects
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from tier4_planning_msgs.msg import LateralOffset

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, apply_s_curve_shift, object_pose, project_pose_to_path


class LaneChangeBPP(PathModuleBase):
    def __init__(self):
        self.external_request = None
        self.external_request_time = None
        self.objects = None

        self.lane_width = float(rospy.get_param("~lane_width", 3.5))
        self.direction = rospy.get_param("~direction", "none").lower()
        self.target_lateral_offset = float(rospy.get_param("~target_lateral_offset", 0.0))
        self.min_request_offset = float(rospy.get_param("~min_request_offset", 0.1))
        self.request_timeout = float(rospy.get_param("~request_timeout", 0.0))
        self.prepare_distance = float(rospy.get_param("~prepare_distance", 15.0))
        self.lane_change_distance = float(rospy.get_param("~lane_change_distance", 30.0))
        self.enable_safety_check = bool(rospy.get_param("~enable_safety_check", True))
        self.safety_lateral_margin = float(rospy.get_param("~safety_lateral_margin", 1.2))
        self.safety_forward_margin = float(rospy.get_param("~safety_forward_margin", 10.0))

        lateral_offset_topic = rospy.get_param("~lateral_offset_topic", "~input/lateral_offset")
        objects_topic = rospy.get_param("~objects_topic", "~input/objects")

        rospy.Subscriber(lateral_offset_topic, LateralOffset, self.lateral_offset_callback, queue_size=1)
        rospy.Subscriber(objects_topic, PredictedObjects, self.objects_callback, queue_size=1)
        self.turn_pub = rospy.Publisher("~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1)

        super().__init__("lane_change_bpp")

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

    def process_path(self, path, stamp):
        target_offset = self.requested_offset()
        if abs(target_offset) < self.min_request_offset:
            self.publish_turn_signal(stamp, 0.0)
            return path, "lane_change: pass-through, no request"

        if not self.is_lane_change_safe(path, target_offset):
            self.publish_turn_signal(stamp, 0.0)
            return path, "lane_change: blocked by object in target lane"

        output = apply_s_curve_shift(
            path,
            target_offset,
            self.prepare_distance,
            self.prepare_distance + self.lane_change_distance,
        )
        self.publish_turn_signal(stamp, target_offset)
        return output, "lane_change: applied lateral offset {:.2f} m".format(target_offset)


def main():
    rospy.init_node("lane_change_bpp")
    LaneChangeBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
