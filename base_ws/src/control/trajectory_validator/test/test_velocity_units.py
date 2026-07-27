#!/usr/bin/env python3

import unittest

import rospy
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint

from trajectory_validator.history_publisher import debug_from_row


class VelocityUnitsTest(unittest.TestCase):
    def test_legacy_raw_ctrl_velocity_is_replayed_as_kph(self):
        gt = Trajectory()
        gt.header.frame_id = "map"
        gt.points.append(TrajectoryPoint())
        ego = Trajectory()
        ego.header.frame_id = "map"
        row = {
            "sample_index": "0",
            "matched_gt_index": "0",
            # Legacy schema v3/v4 used the wrong suffix but stored the raw
            # MORAI CtrlCmd.velocity number, whose unit is km/h.
            "cmd_velocity_mps": "20.0",
        }

        debug = debug_from_row(
            row, gt, ego, "sample", "legacy_run", rospy.Time(0)
        )

        self.assertEqual(debug.cmd_velocity_kph, 20.0)


if __name__ == "__main__":
    unittest.main()
