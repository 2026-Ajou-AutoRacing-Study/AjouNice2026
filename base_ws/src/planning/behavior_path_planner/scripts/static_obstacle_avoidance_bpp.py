#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from nav_msgs.msg import Odometry
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_map_tools import CenterlineMap
from bpp_path_tools import PathModuleBase, make_avoidance_debug_array, to_autoware_path
from bpp_static_avoidance_tools import StaticAvoidanceScene


class StaticObstacleAvoidanceBPP(PathModuleBase):
    def __init__(self):
        self.objects = None
        self.odometry = None
        self.centerline_map = None

        map_file = rospy.get_param("~map_file", "")
        projector_file = rospy.get_param("~projector_file", "")
        projector_type = rospy.get_param("~map_projector_type", "MGRS")
        if map_file:
            try:
                self.centerline_map = CenterlineMap.load(
                    map_file, projector_file, False, projector_type
                )
                rospy.loginfo(
                    "static_obstacle_avoidance_bpp: loaded %d lanelets",
                    self.centerline_map.lane_count(),
                )
            except Exception as exc:
                rospy.logwarn(
                    "static_obstacle_avoidance_bpp: failed to load map: %s", exc
                )

        self.scene = StaticAvoidanceScene("~avoidance", self.centerline_map)
        objects_topic = rospy.get_param("~objects_topic", "~input/objects")
        odometry_topic = rospy.get_param("~odometry_topic", "~input/odometry")
        rospy.Subscriber(
            objects_topic, PredictedObjects, self.objects_callback, queue_size=1
        )
        rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback, queue_size=1)

        self.avoidance_debug_pub = rospy.Publisher(
            "~debug/avoidance", AvoidanceDebugMsgArray, queue_size=1
        )
        self.candidate_path_pub = rospy.Publisher(
            "~output/candidate_path", Path, queue_size=1
        )
        self.reference_path_pub = rospy.Publisher(
            "~output/reference_path", Path, queue_size=1
        )
        self.turn_pub = rospy.Publisher(
            "~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1
        )

        super().__init__("static_obstacle_avoidance_bpp")

    def objects_callback(self, msg):
        self.objects = msg

    def odometry_callback(self, msg):
        self.odometry = msg

    def publish_turn_signal(self, stamp, result):
        command = TurnIndicatorsCommand.DISABLE
        if self.scene.engine.turn_signal_required(result):
            command = (
                TurnIndicatorsCommand.ENABLE_LEFT
                if result.direction == "left"
                else TurnIndicatorsCommand.ENABLE_RIGHT
            )
        message = TurnIndicatorsCommand()
        message.stamp = stamp
        message.command = command
        self.turn_pub.publish(message)

    def process_path(self, path, stamp):
        result = self.scene.process(path, self.objects, self.odometry, stamp)
        self.avoidance_debug_pub.publish(
            make_avoidance_debug_array(
                path.header, [candidate.debug_dict() for candidate in result.candidates]
            )
        )
        result.reference_path.header.stamp = stamp
        result.candidate_path.header.stamp = stamp
        self.reference_path_pub.publish(to_autoware_path(result.reference_path))
        self.candidate_path_pub.publish(to_autoware_path(result.candidate_path))
        self.publish_turn_signal(stamp, result)
        return result.output_path, result.status


def main():
    rospy.init_node("static_obstacle_avoidance_bpp")
    StaticObstacleAvoidanceBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
