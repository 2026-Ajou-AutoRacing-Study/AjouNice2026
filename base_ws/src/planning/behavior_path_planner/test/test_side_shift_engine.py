#!/usr/bin/env python3

import os
import sys
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_path_tools import quaternion_from_yaw
from bpp_side_shift_tools import AFTER_SHIFT, BEFORE_SHIFT, SHIFTING, SideShiftEngine, SideShiftParameters


def make_path():
    path = PathWithLaneId()
    for x in range(121):
        point = PathPoint()
        point.pose.position.x = float(x)
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = point
        path.points.append(item)
    path.left_bound = [Point(x=0.0, y=2.5), Point(x=120.0, y=2.5)]
    path.right_bound = [Point(x=0.0, y=-2.5), Point(x=120.0, y=-2.5)]
    return path


def make_odometry(x=0.0, speed=0.0):
    message = Odometry()
    message.pose.pose.position.x = x
    message.pose.pose.orientation = quaternion_from_yaw(0.0)
    message.twist.twist.linear.x = speed
    return message


class SideShiftEngineTest(unittest.TestCase):
    def setUp(self):
        self.path = make_path()
        self.engine = SideShiftEngine(SideShiftParameters())

    def test_generates_jerk_constrained_left_shift(self):
        self.assertTrue(self.engine.update_request(1.0, 0.0))
        result = self.engine.plan(self.path, make_odometry())
        self.assertEqual(result.state, BEFORE_SHIFT)
        self.assertAlmostEqual(result.applied_offset, 1.0)
        self.assertGreater(result.finish_distance, result.start_distance)
        self.assertAlmostEqual(result.output_path.points[-1].point.pose.position.y, 1.0)

    def test_clamps_request_to_drivable_bound(self):
        self.engine.update_request(3.0, 0.0)
        result = self.engine.plan(self.path, make_odometry())
        self.assertTrue(result.clamped)
        self.assertAlmostEqual(result.applied_offset, 1.252, delta=0.02)

    def test_request_during_shift_is_deferred_until_after_shift(self):
        self.engine.update_request(1.0, 0.0)
        first = self.engine.plan(self.path, make_odometry())
        shifting = self.engine.plan(
            self.path, make_odometry(x=first.start_distance + 1.0)
        )
        self.assertEqual(shifting.state, SHIFTING)
        self.assertFalse(self.engine.update_request(-1.0, 2.0))
        self.assertEqual(self.engine.pending_offset, -1.0)
        next_plan = self.engine.plan(
            self.path, make_odometry(x=first.finish_distance + 1.0)
        )
        self.assertEqual(next_plan.state, BEFORE_SHIFT)
        self.assertAlmostEqual(next_plan.requested_offset, -1.0)

    def test_zero_request_is_pass_through(self):
        result = self.engine.plan(self.path, make_odometry())
        self.assertEqual(result.state, AFTER_SHIFT)
        self.assertIn("pass-through", result.status)


if __name__ == "__main__":
    unittest.main()
