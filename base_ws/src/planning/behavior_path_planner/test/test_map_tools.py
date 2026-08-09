#!/usr/bin/env python3

import os
import sys
import unittest
import xml.etree.ElementTree as ET


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
    def test_duplicate_geometric_boundary_is_a_lateral_neighbor(self):
        centerline_map = CenterlineMap(
            "duplicated_boundary_ways",
            {
                "10": [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)],
                "11": [(0.0, -4.0, 0.0), (100.0, -4.0, 0.0)],
            },
            boundaries={
                "10": (
                    [(0.0, 2.0, 0.0), (100.0, 2.0, 0.0)],
                    [(0.0, -2.0, 0.0), (100.0, -2.0, 0.0)],
                ),
                "11": (
                    [(0.2, -2.0, 0.0), (100.0, -2.0, 0.0)],
                    [(0.0, -6.0, 0.0), (100.0, -6.0, 0.0)],
                ),
            },
            boundary_ids={"10": (100, 101), "11": (102, 103)},
        )

        self.assertTrue(centerline_map.is_lateral_neighbor(10, 11))
        self.assertEqual(
            centerline_map.adjacent_lane(10, "right", "same_direction_lane"),
            (11, "right"),
        )

    def test_osm_intersection_area_expands_lateral_drivable_distance(self):
        root = ET.fromstring(
            """
            <osm>
              <node id="1" lat="0" lon="0"/>
              <node id="2" lat="0" lon="10"/>
              <node id="3" lat="1" lon="0"/>
              <node id="4" lat="1" lon="10"/>
              <node id="5" lat="-1" lon="0"/>
              <node id="6" lat="-1" lon="10"/>
              <node id="11" lat="-3" lon="-1"/>
              <node id="12" lat="-3" lon="11"/>
              <node id="13" lat="3" lon="11"/>
              <node id="14" lat="3" lon="-1"/>
              <way id="100"><nd ref="1"/><nd ref="2"/></way>
              <way id="101"><nd ref="3"/><nd ref="4"/></way>
              <way id="102"><nd ref="5"/><nd ref="6"/></way>
              <way id="500">
                <nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="14"/><nd ref="11"/>
                <tag k="type" v="intersection_area"/>
                <tag k="area" v="yes"/>
              </way>
              <relation id="10">
                <member type="way" ref="101" role="left"/>
                <member type="way" ref="102" role="right"/>
                <member type="way" ref="100" role="centerline"/>
                <tag k="type" v="lanelet"/>
                <tag k="intersection_area" v="500"/>
              </relation>
            </osm>
            """
        )
        centerline_map = CenterlineMap.from_osm_root(
            root, projector=None, source="synthetic"
        )

        self.assertEqual(centerline_map.intersection_area_count(), 1)
        self.assertEqual(centerline_map.get_intersection_area_id(10), "500")
        self.assertEqual(
            centerline_map.expanded_bound_distances(
                10, (5.0, 0.0, 0.0), use_lane_type="current_lane"
            ),
            (1.0, 1.0),
        )
        expanded = centerline_map.expanded_bound_distances(
            10,
            (5.0, 0.0, 0.0),
            use_lane_type="current_lane",
            use_intersection_areas=True,
        )
        self.assertAlmostEqual(expanded[0], 3.0)
        self.assertAlmostEqual(expanded[1], 3.0)
        expanded_points = centerline_map.expanded_bound_points(
            10,
            (5.0, 0.0, 0.0),
            use_lane_type="current_lane",
            use_intersection_areas=True,
        )
        self.assertAlmostEqual(expanded_points[0][1], 3.0)
        self.assertAlmostEqual(expanded_points[1][1], -3.0)

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
