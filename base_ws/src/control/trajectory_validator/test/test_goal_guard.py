#!/usr/bin/env python3

import unittest

from trajectory_validator.metrics import (
    goal_conditions_met,
    plausible_ego_travel_step_m,
)


class GoalGuardTest(unittest.TestCase):
    def test_early_overlapping_endpoint_is_rejected_by_travel_ratio(self):
        self.assertFalse(
            goal_conditions_met(
                final_distance_m=0.03,
                goal_tolerance_m=3.0,
                matched_station_m=999.0,
                final_station_m=1000.0,
                goal_progress_margin_m=5.0,
                ego_travel_distance_m=70.0,
                min_travel_ratio_for_goal=0.70,
            )
        )

    def test_goal_is_accepted_after_minimum_physical_travel(self):
        self.assertTrue(
            goal_conditions_met(
                final_distance_m=1.0,
                goal_tolerance_m=3.0,
                matched_station_m=998.0,
                final_station_m=1000.0,
                goal_progress_margin_m=5.0,
                ego_travel_distance_m=700.0,
                min_travel_ratio_for_goal=0.70,
            )
        )

    def test_existing_endpoint_and_station_conditions_are_still_required(self):
        common = {
            "goal_tolerance_m": 3.0,
            "final_station_m": 1000.0,
            "goal_progress_margin_m": 5.0,
            "ego_travel_distance_m": 900.0,
            "min_travel_ratio_for_goal": 0.70,
        }
        self.assertFalse(
            goal_conditions_met(
                final_distance_m=3.1,
                matched_station_m=999.0,
                **common
            )
        )
        self.assertFalse(
            goal_conditions_met(
                final_distance_m=1.0,
                matched_station_m=994.9,
                **common
            )
        )

    def test_normal_motion_contributes_to_travel_distance(self):
        self.assertAlmostEqual(
            plausible_ego_travel_step_m(
                previous_x=0.0,
                previous_y=0.0,
                current_x=4.0,
                current_y=0.0,
                dt_s=0.1,
                previous_velocity_mps=40.0,
                current_velocity_mps=40.0,
                trajectory_length_m=1000.0,
            ),
            4.0,
        )

    def test_position_teleport_is_rejected(self):
        self.assertIsNone(
            plausible_ego_travel_step_m(
                previous_x=0.0,
                previous_y=0.0,
                current_x=100.0,
                current_y=0.0,
                dt_s=0.1,
                previous_velocity_mps=0.0,
                current_velocity_mps=0.0,
                trajectory_length_m=1000.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
