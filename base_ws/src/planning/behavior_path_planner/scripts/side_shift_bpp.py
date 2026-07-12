#!/usr/bin/env python3

import os
import sys

import rospy
from nav_msgs.msg import Odometry

from autoware_planning_msgs.msg import Path
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from tier4_planning_msgs.msg import LateralOffset

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, to_autoware_path
from bpp_side_shift_tools import SideShiftScene


class SideShiftBPP(PathModuleBase):
    def __init__(self):
        self.odometry = None
        self.scene = SideShiftScene("~side_shift")
        rospy.Subscriber(
            rospy.get_param("~lateral_offset_topic", "~input/lateral_offset"),
            LateralOffset,
            self.lateral_offset_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~odometry_topic", "~input/odometry"),
            Odometry,
            self.odometry_callback,
            queue_size=1,
        )
        self.turn_pub = rospy.Publisher(
            "~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1
        )
        self.reference_pub = rospy.Publisher(
            "~output/reference_path", Path, queue_size=1
        )
        self.candidate_pub = rospy.Publisher(
            "~output/candidate_path", Path, queue_size=1
        )
        super().__init__("side_shift_bpp")

    def lateral_offset_callback(self, message):
        self.scene.update_request(message.lateral_offset, message.stamp)

    def odometry_callback(self, message):
        self.odometry = message

    def process_path(self, path, stamp):
        result = self.scene.process(path, self.odometry)
        result.reference_path.header.stamp = stamp
        result.candidate_path.header.stamp = stamp
        self.reference_pub.publish(to_autoware_path(result.reference_path))
        self.candidate_pub.publish(to_autoware_path(result.candidate_path))
        command = TurnIndicatorsCommand()
        command.stamp = stamp
        command.command = self.scene.turn_command(result)
        self.turn_pub.publish(command)
        return result.output_path, result.status


def main():
    rospy.init_node("side_shift_bpp")
    SideShiftBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
