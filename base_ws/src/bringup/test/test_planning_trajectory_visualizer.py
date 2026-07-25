#!/usr/bin/env python3

import importlib.util
import os
import unittest

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint


def load_visualizer_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "validation",
            "scripts",
            "planning_trajectory_visualizer.py",
        )
    )
    spec = importlib.util.spec_from_file_location(
        "planning_trajectory_visualizer", script_path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PlanningTrajectoryVisualizerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_visualizer_module()

    def test_static_visualization_keeps_complete_trajectory(self):
        message = Trajectory()
        for index in range(11):
            point = TrajectoryPoint()
            point.pose.position.x = float(index)
            point.longitudinal_velocity_mps = float(index)
            message.points.append(point)

        display_points = self.module.PlanningTrajectoryVisualizer.display_points(
            message
        )

        self.assertEqual(len(display_points), 11)
        self.assertEqual(display_points[0][0].position.x, 0.0)
        self.assertEqual(display_points[-1][0].position.x, 10.0)
        self.assertEqual(display_points[-1][1], 10.0)


if __name__ == "__main__":
    unittest.main()
