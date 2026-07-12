#!/usr/bin/env python3

import os
import sys

import rospy
from nav_msgs.msg import Odometry

from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_dynamic_avoidance_tools import DynamicAvoidanceScene
from bpp_path_tools import PathModuleBase, make_avoidance_debug_array, to_autoware_path


class DynamicObstacleAvoidanceBPP(PathModuleBase):
    def __init__(self):
        self.objects = None
        self.odometry = None
        self.scene = DynamicAvoidanceScene("~dynamic_obstacle_avoidance")

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
        self.avoidance_debug_pub = rospy.Publisher(
            "~debug/avoidance", AvoidanceDebugMsgArray, queue_size=1
        )
        self.reference_path_pub = rospy.Publisher(
            "~output/reference_path", Path, queue_size=1
        )
        self.candidate_path_pub = rospy.Publisher(
            "~output/candidate_path", Path, queue_size=1
        )
        super().__init__("dynamic_obstacle_avoidance_bpp")

    def objects_callback(self, message):
        self.objects = message

    def odometry_callback(self, message):
        self.odometry = message

    def process_path(self, path, stamp):
        result = self.scene.process(path, self.objects, self.odometry, stamp)
        debug = [candidate.debug_dict() for candidate in result.candidates]
        self.avoidance_debug_pub.publish(make_avoidance_debug_array(path.header, debug))
        self.reference_path_pub.publish(to_autoware_path(path))
        self.candidate_path_pub.publish(to_autoware_path(result.output_path))
        return result.output_path, result.status


def main():
    rospy.init_node("dynamic_obstacle_avoidance_bpp")
    DynamicObstacleAvoidanceBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
