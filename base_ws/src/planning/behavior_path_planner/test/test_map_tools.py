#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_map_tools import (
    CenterlineMap,
    project_point_to_polyline,
    stitch_route_bounds,
    stitch_route_centerlines,
    trim_polyline,
)


class MapToolsTest(unittest.TestCase):
    def test_lanelet_direction_is_preserved_when_start_is_near_downstream_end(self):
        centerline_map = CenterlineMap(
            "directed_lanelets",
            {
                "10": [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)],
                "20": [(100.0, 0.0, 0.0), (120.0, 0.0, 0.0)],
            },
            boundaries={
                "10": (
                    [(0.0, 1.0, 0.0), (100.0, 1.0, 0.0)],
                    [(0.0, -1.0, 0.0), (100.0, -1.0, 0.0)],
                ),
                "20": (
                    [(100.0, 1.0, 0.0), (120.0, 1.0, 0.0)],
                    [(100.0, -1.0, 0.0), (120.0, -1.0, 0.0)],
                ),
            },
            boundary_node_ids={
                "10": ((1, 2), (3, 4)),
                "20": ((2, 5), (4, 6)),
            },
        )

        centerline, _ = stitch_route_centerlines(
            centerline_map, [10, 20], start_point=(80.0, 0.0, 0.0)
        )
        left, right = stitch_route_bounds(
            centerline_map, [10, 20], start_point=(80.0, 0.0, 0.0)
        )

        self.assertEqual(centerline[0][:2], (0.0, 0.0))
        self.assertEqual(centerline[-1][:2], (120.0, 0.0))
        self.assertEqual(left[0][:2], (0.0, 1.0))
        self.assertEqual(left[-1][:2], (120.0, 1.0))
        self.assertEqual(right[0][:2], (0.0, -1.0))
        self.assertEqual(right[-1][:2], (120.0, -1.0))

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
