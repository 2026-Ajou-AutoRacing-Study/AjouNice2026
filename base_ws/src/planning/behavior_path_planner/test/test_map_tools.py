#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_map_tools import project_point_to_polyline, trim_polyline


class MapToolsTest(unittest.TestCase):
    def test_goal_projection_is_selected_after_route_start(self):
        route = [
            (0.0, 0.0, 0.0),
            (20.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
        ]

        unrestricted = project_point_to_polyline(route, (5.0, 0.0, 0.0))
        after_start = project_point_to_polyline(
            route, (5.0, 0.0, 0.0), min_s=19.0
        )
        trimmed = trim_polyline(
            route,
            start_point=(18.0, 0.0, 0.0),
            goal_point=(5.0, 0.0, 0.0),
        )

        self.assertAlmostEqual(unrestricted[0], 5.0)
        self.assertAlmostEqual(after_start[0], 35.0)
        self.assertAlmostEqual(trimmed[-1][0], 5.0)
        self.assertAlmostEqual(trimmed[-1][1], 0.0)


if __name__ == "__main__":
    unittest.main()
