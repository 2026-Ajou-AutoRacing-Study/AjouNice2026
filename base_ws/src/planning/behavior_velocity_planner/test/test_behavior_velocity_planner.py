#!/usr/bin/env python3

import importlib.util
import os
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId


def load_node_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__), "..", "scripts", "behavior_velocity_planner.py"
        )
    )
    spec = importlib.util.spec_from_file_location("behavior_velocity_planner_node", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_path(point_count):
    path = PathWithLaneId()
    for x in range(point_count):
        point = PathPointWithLaneId()
        point.point.pose.position.x = float(x)
        path.points.append(point)
    return path


class BehaviorVelocityPlannerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node_module = load_node_module()

    def make_planner(self, max_processing_length):
        planner = self.node_module.BehaviorVelocityPlanner.__new__(
            self.node_module.BehaviorVelocityPlanner
        )
        planner.max_processing_length = max_processing_length
        return planner

    def test_processing_path_preserves_tail_after_local_horizon(self):
        original = make_path(6)

        processing_path, retained_points = self.make_planner(2.5).processing_path(
            original
        )

        self.assertEqual(
            [point.point.pose.position.x for point in processing_path.points],
            [0.0, 1.0, 2.0, 3.0],
        )
        self.assertEqual(
            [point.point.pose.position.x for point in retained_points], [4.0, 5.0]
        )
        self.assertEqual(len(original.points), 6)

    def test_processing_path_keeps_short_path_unchanged(self):
        original = make_path(3)

        processing_path, retained_points = self.make_planner(10.0).processing_path(
            original
        )

        self.assertIs(processing_path, original)
        self.assertEqual(retained_points, [])


if __name__ == "__main__":
    unittest.main()
