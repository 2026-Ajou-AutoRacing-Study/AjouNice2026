#!/usr/bin/env python3

import math
import os
import sys
import unittest

from geometry_msgs.msg import Pose


SCRIPT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "validation", "scripts"
)
sys.path.insert(0, SCRIPT_DIR)

from kcity_dynamic_ego_validator import PolylineCourse, quaternion_from_yaw


def pose(x, y, yaw=0.0):
    value = Pose()
    value.position.x = float(x)
    value.position.y = float(y)
    value.orientation = quaternion_from_yaw(yaw)
    return value


class PolylineCourseTest(unittest.TestCase):
    def test_sample_and_projection_use_arc_length(self):
        course = PolylineCourse([pose(0, 0), pose(10, 0), pose(10, 10)])
        sampled = course.sample(15.0)
        self.assertAlmostEqual(sampled.position.x, 10.0)
        self.assertAlmostEqual(sampled.position.y, 5.0)
        projection = course.project(sampled.position, minimum_station=12.0)
        self.assertAlmostEqual(projection.station, 15.0)

    def test_station_window_resolves_closed_course_ambiguity(self):
        course = PolylineCourse(
            [pose(0, 0), pose(10, 0), pose(10, 10), pose(0, 10), pose(0, 0.2)]
        )
        query = pose(0, 0.1, -math.pi / 2.0)
        projection = course.project(
            query.position,
            minimum_station=25.0,
            maximum_station=course.length,
            yaw=-math.pi / 2.0,
            yaw_threshold=0.5,
        )
        self.assertGreater(projection.station, 35.0)


if __name__ == "__main__":
    unittest.main()
