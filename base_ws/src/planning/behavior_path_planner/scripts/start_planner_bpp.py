#!/usr/bin/env python3

import os
import sys

import rospy
from nav_msgs.msg import Odometry

from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, make_avoidance_debug_array, to_autoware_path
from bpp_start_planner_tools import StartPlannerScene


class StartPlannerBPP(PathModuleBase):
    def __init__(self):
        self.objects = None
        self.odometry = None
        self.scene = StartPlannerScene("~start_planner")
        rospy.Subscriber(
            rospy.get_param("~objects_topic", "~input/objects"),
            PredictedObjects,
            self.objects_callback,
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
        self.safety_debug_pub = rospy.Publisher(
            "~debug/safety", AvoidanceDebugMsgArray, queue_size=1
        )
        super().__init__("start_planner_bpp")

    def objects_callback(self, message):
        self.objects = message

    def odometry_callback(self, message):
        self.odometry = message

    def process_path(self, path, stamp):
        result = self.scene.process(path, self.objects, self.odometry, stamp)
        result.reference_path.header.stamp = stamp
        result.candidate_path.header.stamp = stamp
        self.reference_pub.publish(to_autoware_path(result.reference_path))
        self.candidate_pub.publish(to_autoware_path(result.candidate_path))
        self.safety_debug_pub.publish(
            make_avoidance_debug_array(
                path.header, [candidate.debug_dict() for candidate in result.candidates]
            )
        )
        command = TurnIndicatorsCommand()
        command.stamp = stamp
        command.command = self.scene.turn_command(result)
        self.turn_pub.publish(command)
        return result.output_path, result.status


def main():
    rospy.init_node("start_planner_bpp")
    StartPlannerBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
