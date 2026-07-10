#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from tier4_planning_msgs.msg import LateralOffset

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, apply_s_curve_shift


class SideShiftBPP(PathModuleBase):
    def __init__(self):
        self.external_request = None
        self.external_request_time = None

        self.default_lateral_offset = float(rospy.get_param("~default_lateral_offset", 0.0))
        self.min_distance_to_start_shifting = float(
            rospy.get_param("~min_distance_to_start_shifting", 5.0)
        )
        self.min_shifting_distance = float(rospy.get_param("~min_shifting_distance", 5.0))
        self.min_request_offset = float(rospy.get_param("~min_request_offset", 0.05))
        self.shift_request_time_limit = float(rospy.get_param("~shift_request_time_limit", 0.0))

        lateral_offset_topic = rospy.get_param("~lateral_offset_topic", "~input/lateral_offset")
        rospy.Subscriber(lateral_offset_topic, LateralOffset, self.lateral_offset_callback, queue_size=1)
        self.turn_pub = rospy.Publisher("~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1)

        super().__init__("side_shift_bpp")

    def lateral_offset_callback(self, msg):
        self.external_request = msg
        self.external_request_time = rospy.Time.now()

    def requested_offset(self):
        if self.external_request is not None:
            if self.shift_request_time_limit <= 0.0:
                return self.external_request.lateral_offset
            age = (rospy.Time.now() - self.external_request_time).to_sec()
            if age <= self.shift_request_time_limit:
                return self.external_request.lateral_offset
        return self.default_lateral_offset

    def publish_turn_signal(self, stamp, offset):
        msg = TurnIndicatorsCommand()
        msg.stamp = stamp
        if offset > self.min_request_offset:
            msg.command = TurnIndicatorsCommand.ENABLE_LEFT
        elif offset < -self.min_request_offset:
            msg.command = TurnIndicatorsCommand.ENABLE_RIGHT
        else:
            msg.command = TurnIndicatorsCommand.NO_COMMAND
        self.turn_pub.publish(msg)

    def process_path(self, path, stamp):
        offset = self.requested_offset()
        if abs(offset) < self.min_request_offset:
            self.publish_turn_signal(stamp, 0.0)
            return path, "side_shift: pass-through, no offset request"

        output = apply_s_curve_shift(
            path,
            offset,
            self.min_distance_to_start_shifting,
            self.min_distance_to_start_shifting + self.min_shifting_distance,
        )
        self.publish_turn_signal(stamp, offset)
        return output, "side_shift: applied lateral offset {:.2f} m".format(offset)


def main():
    rospy.init_node("side_shift_bpp")
    SideShiftBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
