#!/usr/bin/env python3

import os
import sys
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import LaneletRoute, PathPoint

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_goal_planner_tools import GoalPlannerEngine, GoalPlannerParameters
from bpp_path_tools import quaternion_from_yaw


def make_path():
    path = PathWithLaneId()
    for x in range(101):
        point = PathPoint()
        point.pose.position.x = float(x)
        point.pose.orientation = quaternion_from_yaw(0.0)
        point.longitudinal_velocity_mps = 5.0
        item = PathPointWithLaneId()
        item.point = point
        item.lane_ids = [10]
        path.points.append(item)
    return path


def make_route(goal_x=80.0):
    route = LaneletRoute()
    route.goal_pose.position.x = goal_x
    route.goal_pose.orientation = quaternion_from_yaw(0.0)
    return route


class GoalPlannerEngineTest(unittest.TestCase):
    def test_non_parking_goal_planner_remains_dormant(self):
        path = make_path()
        path.points[-1].point.longitudinal_velocity_mps = 0.0
        path.points[-1].point.is_final = True
        result = GoalPlannerEngine(GoalPlannerParameters()).plan(
            path, make_route()
        )
        self.assertEqual(len(result.output_path.points), len(path.points))
        self.assertAlmostEqual(result.output_path.points[-1].point.pose.position.x, 100.0)
        self.assertEqual(result.output_path.points[-1].point.longitudinal_velocity_mps, 0.0)
        self.assertTrue(result.output_path.points[-1].point.is_final)
        self.assertIsNone(result.modified_goal_pose)
        self.assertIn("parking mission excluded", result.status)

    def test_parking_mode_is_explicitly_out_of_scope(self):
        parameters = GoalPlannerParameters(parking_enabled=True)
        with self.assertRaises(RuntimeError):
            GoalPlannerEngine(parameters).plan(make_path(), make_route())


if __name__ == "__main__":
    unittest.main()
