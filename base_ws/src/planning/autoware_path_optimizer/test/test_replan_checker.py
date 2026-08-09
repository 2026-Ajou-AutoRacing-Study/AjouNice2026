#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.join(os.path.dirname(os.path.dirname(__file__)), "src")
sys.path.insert(0, PACKAGE_SRC)

from autoware_path_optimizer.replan_checker import ReplanChecker
from autoware_path_optimizer.trajectory_utils import TrajectorySample


def straight_path(start, count=100, lateral=0.0):
    return [
        TrajectorySample(x=float(start + index), y=float(lateral))
        for index in range(count)
    ]


class ReplanCheckerTest(unittest.TestCase):
    def make_checker(self):
        return ReplanChecker(
            max_path_shape_forward_lon_dist=50.0,
            max_path_shape_forward_lat_dist=0.1,
            max_path_shape_forward_reset_lat_dist=0.3,
            max_delta_time_sec=10.0,
        )

    def test_cropped_start_index_does_not_trigger_shape_replan(self):
        checker = self.make_checker()
        previous = straight_path(0)
        current = straight_path(2)
        checker.update(previous, (5.0, 0.0), True, 0.0)

        self.assertFalse(checker.replan_required(current, 1.0, (7.0, 0.0)))

    def test_real_lateral_shape_change_triggers_replan(self):
        checker = self.make_checker()
        previous = straight_path(0)
        current = straight_path(2, lateral=0.25)
        checker.update(previous, (5.0, 0.0), True, 0.0)

        self.assertTrue(checker.replan_required(current, 1.0, (7.0, 0.0)))
        self.assertFalse(checker.reset_required(current, (7.0, 0.0)))

    def test_large_forward_shape_change_resets_previous_optimization(self):
        checker = self.make_checker()
        previous = straight_path(0)
        current = straight_path(2, lateral=0.5)
        checker.update(previous, (5.0, 0.0), True, 0.0)

        self.assertTrue(checker.reset_required(current, (7.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
