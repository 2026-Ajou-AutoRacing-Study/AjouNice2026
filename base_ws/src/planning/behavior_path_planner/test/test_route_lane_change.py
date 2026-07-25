#!/usr/bin/env python3

import os
import sys
import unittest

from autoware_internal_planning_msgs.msg import PathPointWithLaneId, PathWithLaneId
from autoware_planning_msgs.msg import PathPoint

SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from bpp_lane_change_tools import (
    RouteLaneChangeTarget,
    build_route_lane_change_path,
    route_lane_change_target,
)
from bpp_map_tools import CenterlineMap, stitch_route_centerlines
from bpp_route_tools import current_route_lane, forward_lane_sequence, reference_lane_sequence


class Primitive:
    def __init__(self, lane_id):
        self.id = lane_id


class Segment:
    def __init__(self, preferred_lane_id, lane_ids):
        self.preferred_primitive = Primitive(preferred_lane_id)
        self.primitives = [Primitive(lane_id) for lane_id in lane_ids]


class Route:
    def __init__(self):
        self.segments = [
            Segment(200262, [200261, 200262]),
            Segment(200315, [200315, 200316]),
            Segment(200272, [200272]),
        ]


class PreferredLaneRoute:
    def __init__(self):
        self.segments = [
            Segment(200261, [200261]),
            Segment(200315, [200315, 200316]),
            Segment(200272, [200272]),
        ]


class MultiLaneChangeRoute:
    def __init__(self):
        self.segments = [
            Segment(200188, [200188, 200187, 200102]),
        ]


class Position:
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Orientation:
    def __init__(self, yaw=0.0):
        import math

        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(yaw * 0.5)
        self.w = math.cos(yaw * 0.5)


class Pose:
    def __init__(self, x, y, z=0.0, yaw=None):
        self.position = Position(x, y, z)
        if yaw is not None:
            self.orientation = Orientation(yaw)


class LoopingRoute:
    def __init__(self):
        self.start_pose = Pose(5.0, 1.0, yaw=0.0)
        self.segments = [
            Segment(300001, [300001]),
            Segment(300002, [300002]),
        ]


class RouteLaneChangeTest(unittest.TestCase):
    def setUp(self):
        centerlines = {
            "200261": [(0.0, 4.0, 0.0), (10.0, 4.0, 0.0)],
            "200262": [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)],
            "200315": [(10.0, 4.0, 0.0), (40.0, 4.0, 0.0)],
            "200316": [(10.0, 0.0, 0.0), (40.0, 0.0, 0.0)],
            "200272": [(40.0, 4.0, 0.0), (80.0, 4.0, 0.0)],
            "200188": [(0.0, 8.0, 0.0), (30.0, 8.0, 0.0)],
            "200187": [(0.0, 4.0, 0.0), (30.0, 4.0, 0.0)],
            "200102": [(0.0, 0.0, 0.0), (30.0, 0.0, 0.0)],
            "300001": [(0.0, 1.0, 0.0), (10.0, 1.0, 0.0)],
            "300002": [(0.0, 0.9, 0.0), (10.0, 0.9, 0.0)],
            "300003": [(0.0, 0.2, 0.0), (10.0, 0.2, 0.0)],
            "300004": [(10.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
        }
        boundary_ids = {
            "200261": (101, 102),
            "200262": (102, 103),
            "200315": (104, 105),
            "200316": (105, 106),
            "200272": (107, 108),
            "200188": (201, 202),
            "200187": (202, 203),
            "200102": (203, 204),
        }
        boundary_node_ids = {
            "200261": ((1, 2), (3, 4)),
            "200262": ((3, 4), (5, 6)),
            "200315": ((2, 8), (4, 10)),
            "200316": ((4, 11), (6, 12)),
            "200272": ((8, 13), (10, 14)),
        }
        self.map = CenterlineMap(
            "synthetic",
            centerlines,
            boundary_ids=boundary_ids,
            boundary_node_ids=boundary_node_ids,
        )
        self.route = Route()

    def test_reference_stays_on_longitudinal_successor_before_lane_change(self):
        lane_ids = reference_lane_sequence(
            self.route, self.map, Pose(5.0, 0.0)
        )
        self.assertEqual(lane_ids, [200262, 200316])

        centerline, _ = stitch_route_centerlines(self.map, lane_ids)
        self.assertEqual(centerline[0][:2], (0.0, 0.0))
        self.assertEqual(centerline[-1][:2], (40.0, 0.0))

    def test_target_sequence_continues_forward_after_lane_change(self):
        lane_ids = forward_lane_sequence(
            self.route, self.map, 1, 200315
        )
        self.assertEqual(lane_ids, [200315, 200272])

        centerline, _ = stitch_route_centerlines(self.map, lane_ids)
        xs = [point[0] for point in centerline]
        self.assertEqual(xs, sorted(xs))
        self.assertEqual(centerline[-1][:2], (80.0, 4.0))

    def test_lane_change_target_uses_route_progression(self):
        target = route_lane_change_target(
            self.route, self.map, Pose(5.0, 0.0)
        )
        self.assertIsNotNone(target)
        self.assertEqual(target.source_lane_id, 200316)
        self.assertEqual(target.target_lane_id, 200315)

    def test_no_lane_change_when_preferred_lane_is_longitudinal_successor(self):
        route = PreferredLaneRoute()
        self.assertEqual(
            reference_lane_sequence(route, self.map, Pose(5.0, 4.0)),
            [200261, 200315, 200272],
        )
        self.assertIsNone(
            route_lane_change_target(route, self.map, Pose(5.0, 4.0))
        )

    def test_multi_lane_change_selects_adjacent_lane_first(self):
        target = route_lane_change_target(
            MultiLaneChangeRoute(), self.map, Pose(5.0, 0.0)
        )
        self.assertIsNotNone(target)
        self.assertEqual(target.source_lane_id, 200102)
        self.assertEqual(target.target_lane_id, 200187)
        self.assertEqual(target.direction, "left")

    def test_route_start_does_not_jump_to_a_later_parallel_section(self):
        section_index, lane_id = current_route_lane(
            LoopingRoute(), self.map, Pose(5.0, 0.9, yaw=0.0)
        )
        self.assertEqual((section_index, lane_id), (0, 300001))

    def test_current_lane_rejects_a_closer_opposite_direction_lane(self):
        route = type(
            "DirectionRoute",
            (),
            {
                "segments": [
                    Segment(300003, [300003]),
                    Segment(300004, [300004]),
                ]
            },
        )()
        section_index, lane_id = current_route_lane(
            route, self.map, Pose(5.0, 0.0, yaw=0.0)
        )
        self.assertEqual((section_index, lane_id), (0, 300003))

    def test_candidate_uses_smooth_forward_target_path_without_shortcut(self):
        path = PathWithLaneId()
        for x in range(41):
            point = PathPoint()
            point.pose.position.x = float(x)
            point.longitudinal_velocity_mps = 5.0
            item = PathPointWithLaneId()
            item.point = point
            item.lane_ids = [200262 if x < 10 else 200316]
            path.points.append(item)

        target = RouteLaneChangeTarget(200316, 200315, 4.0, 1)
        candidate = build_route_lane_change_path(
            path,
            self.map,
            Pose(0.0, 0.0),
            target,
            prepare_distance=3.0,
            lane_change_distance=28.0,
            target_forward_length=100.0,
            transition_interval=1.0,
            target_interval=2.0,
            target_lane_ids=[200315, 200272],
        )

        points = [
            (item.point.pose.position.x, item.point.pose.position.y)
            for item in candidate.points
        ]
        self.assertGreater(len(points), 40)
        self.assertEqual([point[0] for point in points], sorted(point[0] for point in points))
        self.assertAlmostEqual(points[-1][0], 80.0, delta=0.1)
        self.assertAlmostEqual(points[-1][1], 4.0, delta=0.1)
        self.assertLessEqual(
            max(
                ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                for a, b in zip(points[:-1], points[1:])
            ),
            2.1,
        )

    def test_candidate_stops_at_route_goal(self):
        path = PathWithLaneId()
        for x in range(41):
            point = PathPoint()
            point.pose.position.x = float(x)
            point.longitudinal_velocity_mps = 5.0
            item = PathPointWithLaneId()
            item.point = point
            item.lane_ids = [200262 if x < 10 else 200316]
            path.points.append(item)

        target = RouteLaneChangeTarget(200316, 200315, 4.0, 1)
        candidate = build_route_lane_change_path(
            path,
            self.map,
            Pose(0.0, 0.0),
            target,
            prepare_distance=3.0,
            lane_change_distance=28.0,
            target_forward_length=100.0,
            transition_interval=1.0,
            target_interval=2.0,
            target_lane_ids=[200315, 200272],
            goal_point=(60.0, 4.0, 0.0),
            stop_at_goal=True,
        )

        final = candidate.points[-1].point
        self.assertAlmostEqual(final.pose.position.x, 60.0)
        self.assertAlmostEqual(final.pose.position.y, 4.0)
        self.assertAlmostEqual(final.longitudinal_velocity_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
