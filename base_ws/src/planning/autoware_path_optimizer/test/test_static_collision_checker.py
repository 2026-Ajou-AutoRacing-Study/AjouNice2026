#!/usr/bin/env python3

import math
import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_path_optimizer.static_collision_checker import (
    StaticObstacle,
    _rectangle,
    first_static_collision,
)
from autoware_path_optimizer.trajectory_utils import TrajectorySample


def straight_path(lateral=0.0):
    return [
        TrajectorySample(
            x=float(index) * 0.5,
            y=float(lateral),
            yaw=0.0,
            longitudinal_velocity_mps=2.0,
        )
        for index in range(81)
    ]


def obstacle():
    polygon = _rectangle(10.0, 0.0, 0.0, 1.3, 1.3, 1.3)
    return StaticObstacle("target", polygon, 10.0, 0.0, 1.84)


def rear_margin_obstacle(margin):
    half_length = 0.2 + margin
    half_width = 0.96 + margin
    polygon = _rectangle(8.82, 2.15, 0.0, half_length, half_length, half_width)
    return StaticObstacle(
        "rear",
        polygon,
        8.82,
        2.15,
        math.hypot(half_length, half_width),
    )


class StaticCollisionCheckerTest(unittest.TestCase):
    def test_detects_vehicle_footprint_before_center_reaches_obstacle(self):
        hit = first_static_collision(
            straight_path(),
            [obstacle()],
            ego_position=(0.0, 0.0),
        )

        self.assertIsNotNone(hit)
        self.assertEqual(hit.object_id, "target")
        self.assertLess(hit.station, 6.0)

    def test_lateral_avoidance_path_is_clear(self):
        hit = first_static_collision(
            straight_path(lateral=3.0),
            [obstacle()],
            ego_position=(0.0, 3.0),
        )

        self.assertIsNone(hit)

    def test_ignores_initial_margin_overlap_that_clears_forward(self):
        strict_hit = first_static_collision(
            straight_path(),
            [rear_margin_obstacle(0.3)],
            ego_position=(10.0, 0.0),
            backward_distance=0.0,
        )
        escaped_hit = first_static_collision(
            straight_path(),
            [rear_margin_obstacle(0.3)],
            ego_position=(10.0, 0.0),
            backward_distance=0.0,
            ignore_initial_overlap=True,
            maximum_initial_overlap_distance=1.0,
        )
        physical_hit = first_static_collision(
            straight_path(),
            [rear_margin_obstacle(0.0)],
            ego_position=(10.0, 0.0),
            backward_distance=0.0,
        )

        self.assertIsNotNone(strict_hit)
        self.assertIsNone(escaped_hit)
        self.assertIsNone(physical_hit)

    def test_does_not_ignore_initial_overlap_that_persists(self):
        persistent = StaticObstacle(
            "blocking",
            _rectangle(12.0, 0.0, 0.0, 1.3, 1.3, 1.3),
            12.0,
            0.0,
            1.84,
        )

        hit = first_static_collision(
            straight_path(),
            [persistent],
            ego_position=(10.0, 0.0),
            backward_distance=0.0,
            ignore_initial_overlap=True,
            maximum_initial_overlap_distance=1.0,
        )

        self.assertIsNotNone(hit)
        self.assertEqual(hit.object_id, "blocking")


if __name__ == "__main__":
    unittest.main()
