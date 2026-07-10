#!/usr/bin/env python3

import os
import sys

import rospy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bpp_path_tools import PathModuleBase, apply_pull_out_shift, path_length, set_velocity_window


class StartPlannerBPP(PathModuleBase):
    def __init__(self):
        self.operation_mode = None

        self.pull_out_lateral_offset = float(rospy.get_param("~pull_out_lateral_offset", 0.0))
        self.pull_out_distance = float(rospy.get_param("~pull_out_distance", 20.0))
        self.hold_until_autonomous = bool(rospy.get_param("~hold_until_autonomous", False))
        self.require_operation_mode = bool(rospy.get_param("~require_operation_mode", False))
        self.min_offset_to_signal = float(rospy.get_param("~min_offset_to_signal", 0.1))

        operation_mode_topic = rospy.get_param("~operation_mode_topic", "~input/operation_mode")
        rospy.Subscriber(
            operation_mode_topic, OperationModeState, self.operation_mode_callback, queue_size=1
        )
        self.turn_pub = rospy.Publisher("~output/turn_indicators_cmd", TurnIndicatorsCommand, queue_size=1)

        super().__init__("start_planner_bpp")

    def operation_mode_callback(self, msg):
        self.operation_mode = msg

    def is_autonomous_enabled(self):
        if self.operation_mode is None:
            return not self.require_operation_mode
        return (
            self.operation_mode.mode == OperationModeState.AUTONOMOUS
            and self.operation_mode.is_autoware_control_enabled
            and not self.operation_mode.is_in_transition
        )

    def publish_turn_signal(self, stamp, start_offset):
        msg = TurnIndicatorsCommand()
        msg.stamp = stamp
        if start_offset > self.min_offset_to_signal:
            msg.command = TurnIndicatorsCommand.ENABLE_RIGHT
        elif start_offset < -self.min_offset_to_signal:
            msg.command = TurnIndicatorsCommand.ENABLE_LEFT
        else:
            msg.command = TurnIndicatorsCommand.NO_COMMAND
        self.turn_pub.publish(msg)

    def process_path(self, path, stamp):
        output = path
        status_parts = []

        if abs(self.pull_out_lateral_offset) > self.min_offset_to_signal:
            output = apply_pull_out_shift(output, self.pull_out_lateral_offset, self.pull_out_distance)
            status_parts.append(
                "pull-out merge from offset {:.2f} m".format(self.pull_out_lateral_offset)
            )
        else:
            status_parts.append("no pull-out offset")

        if self.hold_until_autonomous and not self.is_autonomous_enabled():
            output = set_velocity_window(output, 0.0, path_length(output), 0.0, mode="set")
            status_parts.append("holding velocity at 0 until autonomous")

        self.publish_turn_signal(stamp, self.pull_out_lateral_offset)
        return output, "start_planner: " + ", ".join(status_parts)


def main():
    rospy.init_node("start_planner_bpp")
    StartPlannerBPP()
    rospy.spin()


if __name__ == "__main__":
    main()
