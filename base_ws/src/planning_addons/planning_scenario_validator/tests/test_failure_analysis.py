"""Failure labels must follow observations, not assume a model or planner bug."""
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.failure_analysis import (
    analyze_failure, compact_summary, log_offsets, scan_log_signals)


class FailureAnalysisTest(unittest.TestCase):
    def test_no_departure_with_zero_near_trajectory(self):
        metrics = {"elapsed_sec": 80.0, "max_displacement_m": 0.08,
                   "maximum_speed_mps": 0.0, "minimum_goal_distance_m": 69.8,
                   "trajectory_near_max_speed_mps": 0.0,
                   "control_command_count": 1600, "max_accel_command": 0.0,
                   "latest_accel_command": 0.0, "latest_brake_command": 1.0}
        signals = [{"code": "MPT_NO_ALIGNED_PATH", "count": 72,
                    "log": "planning.log", "sample": "no aligned previous path"}]
        result = analyze_failure("TIMEOUT", metrics, log_signals=signals)
        self.assertEqual(result["category"], "NO_DEPARTURE")
        self.assertEqual(result["suspected_stage"], "planning_output")
        self.assertFalse(result["root_cause_confirmed"])
        self.assertTrue(any("72회" in item for item in result["evidence"]))
        self.assertIn("no aligned previous path", compact_summary(result))

    def test_mid_drive_stop_is_distinct_from_no_departure(self):
        metrics = {"elapsed_sec": 100.0, "max_displacement_m": 47.0,
                   "maximum_speed_mps": 10.0, "final_speed_mps": 0.03,
                   "stationary_duration_sec": 16.0, "last_5s_displacement_m": 0.02,
                   "minimum_goal_distance_m": 28.0,
                   "latest_trajectory_near_speed_mps": 0.0,
                   "control_command_count": 1800, "max_accel_command": 0.7}
        result = analyze_failure("TIMEOUT", metrics)
        self.assertEqual(result["category"], "MID_DRIVE_STOP")
        self.assertEqual(result["suspected_stage"], "planning_output")

    def test_positive_plan_and_acceleration_do_not_get_called_planning_failure(self):
        metrics = {"max_displacement_m": 0.1, "maximum_speed_mps": 0.0,
                   "minimum_goal_distance_m": 60.0,
                   "trajectory_near_max_speed_mps": 4.0,
                   "control_command_count": 100, "max_accel_command": 0.5}
        result = analyze_failure("TIMEOUT", metrics)
        self.assertEqual(result["category"], "NO_DEPARTURE")
        self.assertEqual(result["suspected_stage"], "vehicle_or_command_interface")
        self.assertEqual(result["stage_confidence"], "low")

    def test_goal_area_without_stop_is_not_general_timeout(self):
        metrics = {"max_displacement_m": 80.0, "maximum_speed_mps": 10.0,
                   "minimum_goal_distance_m": 0.8, "final_speed_mps": 3.0}
        result = analyze_failure("TIMEOUT", metrics, goal_tolerance_m=2.0)
        self.assertEqual(result["category"], "ARRIVAL_CONDITION_UNMET")

    def test_missing_drive_observation_is_not_no_departure(self):
        result = analyze_failure("TIMEOUT", {"elapsed_sec": 80.0})
        self.assertEqual(result["category"], "INSUFFICIENT_OBSERVATION")

    def test_direct_failure_reasons(self):
        self.assertEqual(analyze_failure("COLLISION", {"collision_count": 1})["category"],
                         "COLLISION")
        self.assertEqual(analyze_failure("ROUTE_FAILURE", {}, error="empty route")["category"],
                         "ROUTE_FAILURE")
        self.assertEqual(analyze_failure("NO_TRAJECTORY", {}, error="no trajectory")["category"],
                         "NO_TRAJECTORY")
        self.assertEqual(analyze_failure("SYSTEM_ERROR", {}, error="topic lost")["category"],
                         "SYSTEM_ERROR")
        self.assertEqual(analyze_failure("SYSTEM_ERROR", {},
            error="localization topic이 4초 이상 끊겼습니다")["category"], "TOPIC_LOSS")

    def test_log_scan_is_scoped_to_current_case(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "planning.log"
            log.write_text("no aligned previous path\n", encoding="utf-8")
            offsets = log_offsets(directory)
            with log.open("a", encoding="utf-8") as stream:
                stream.write("[WARN] no aligned previous path\n")
                stream.write("[WARN] no aligned previous path\n")
            signals = scan_log_signals(directory, offsets)
            self.assertEqual(len(signals), 1)
            self.assertEqual(signals[0]["count"], 2)


if __name__ == "__main__":
    unittest.main()
