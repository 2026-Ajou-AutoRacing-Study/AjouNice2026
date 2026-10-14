"""Opt-in localization safety uses measured raw Ego feedback, never sent commands."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.runtime import LiveMonitor, SafetyFailure


KEYS = ("ego", "gear", "control_mode", "collision", "competition", "gps", "imu",
        "control_state", "odom", "control")


class Clock:
    def __init__(self, monitor, *, refresh=True):
        self.tick = 10.0
        self.monitor = monitor
        self.refresh = refresh

    def now(self):
        return self.tick

    def sleep(self, duration):
        self.tick += duration
        if self.refresh:
            self.monitor.stamps.update({key: self.tick for key in KEYS})


def monitor():
    value = LiveMonitor.__new__(LiveMonitor)
    value.rospy = SimpleNamespace(is_shutdown=lambda: False)
    value.ego = SimpleNamespace(velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                                position=SimpleNamespace(x=0.0, y=0.0))
    value.gps = SimpleNamespace(latitude=37.0, longitude=127.0, status=1)
    value.gear = 1
    value.control_mode = 2
    value.stamps = {key: 10.0 for key in KEYS}
    value.preparation_collisions = {}
    value.armed = False
    value.events_path = Path("not_written.jsonl")
    value.events = []
    value.event = lambda name, **fields: value.events.append((name, fields))
    value.collision_ids = set()
    value.collision_count = 0
    return value


class LocalizationSafetyRuntimeTest(unittest.TestCase):
    def run_with_clock(self, value, callback, *, refresh=True):
        clock = Clock(value, refresh=refresh)
        with patch("planning_scenario_validator.runtime.time.monotonic", clock.now), \
             patch("planning_scenario_validator.runtime.time.sleep", clock.sleep):
            return callback(clock)

    def test_native_raw_speed_is_si_norm_not_a_second_division(self):
        value = monitor()
        value.ego.velocity.x, value.ego.velocity.y = 3.0, 4.0
        self.assertEqual(value.raw_ego_speed_mps(), 5.0)

    def test_raw_speed_rejects_nan_or_malformed_message(self):
        value = monitor()
        value.ego.velocity.x = math.nan
        with self.assertRaises(SafetyFailure):
            value.raw_ego_speed_mps()
        value.ego.velocity = None
        with self.assertRaises(SafetyFailure):
            value.raw_ego_speed_mps()

    def test_old_packets_do_not_establish_stationary_feedback(self):
        value = monitor()
        with self.assertRaises(SafetyFailure):
            self.run_with_clock(value, lambda clock: value.wait_stationary(
                after=10.0, timeout=0.25, hold_sec=0.1), refresh=False)

    def test_one_fresh_packet_cannot_fake_a_continuous_measured_hold(self):
        value = monitor()
        with self.assertRaises(SafetyFailure):
            self.run_with_clock(value, lambda clock: value.wait_stationary(
                after=9.0, timeout=0.3, hold_sec=0.1), refresh=False)

    def test_stationary_park_requires_new_feedback_and_valid_startup_gps(self):
        value = monitor()
        result = self.run_with_clock(value, lambda clock: value.wait_stationary(
            after=10.0, timeout=1, hold_sec=0.1, require_valid_gps=True))
        self.assertTrue(result["park_verified"])
        self.assertTrue(result["gps_valid"])
        self.assertGreater(result["verified_at_monotonic"], 10.0)
        self.assertIn("gps", result["required"])

    def test_moving_raw_ego_is_not_stationary_even_if_estimated_odom_is_zero(self):
        value = monitor()
        value.ego.velocity.x = 1.0
        value.odom = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0))))
        with self.assertRaisesRegex(SafetyFailure, "실제 Ego 속도"):
            self.run_with_clock(value, lambda clock: value.wait_stationary(
                after=10.0, timeout=0.25, hold_sec=0.1))

    def test_d_gear_is_not_park_but_can_be_reported_as_measured_stop(self):
        value = monitor()
        value.gear = 4
        result = self.run_with_clock(value, lambda clock: value.verify_stopped(
            after=10.0, timeout=1, hold_sec=0.1))
        self.assertFalse(result["park_verified"])
        self.assertEqual(result["gear"], 4)
        self.assertEqual(value.events[-1][0], "SAFETY_STOP_CONFIRMED")
        with self.assertRaises(SafetyFailure):
            self.run_with_clock(value, lambda clock: value.verify_stopped(
                after=10.0, timeout=0.25, hold_sec=0.1, require_park=True))

    def test_invalid_gps_is_startup_failure_not_shadow_stream_failure(self):
        value = monitor()
        value.gps = SimpleNamespace(latitude=0.0, longitude=0.0, status=0)
        with self.assertRaisesRegex(SafetyFailure, "GNSS"):
            self.run_with_clock(value, lambda clock: value.wait_stationary(
                after=10.0, timeout=0.25, hold_sec=0.1, require_valid_gps=True))
        value.stamps = {key: 10.0 for key in KEYS}
        result = self.run_with_clock(value, lambda clock: value.localization_watchdog())
        self.assertFalse(result["gps_valid"])

    def test_missing_gps_stream_is_different_from_invalid_gps(self):
        value = monitor()
        value.stamps.pop("gps")
        with self.assertRaisesRegex(SafetyFailure, "topic gps"):
            self.run_with_clock(value, lambda clock: value.localization_watchdog())

    def test_gt_pose_does_not_require_smooth_state_provider(self):
        value = monitor()
        for key in ("gps", "imu", "competition", "control_state"):
            value.stamps.pop(key)
        self.run_with_clock(value, lambda clock: value.localization_watchdog(
            pose_source="maptf_morai"))

    def test_preparing_contact_has_its_own_event_and_blocks_opted_in_start(self):
        value = monitor()
        collision = SimpleNamespace(collision_object=[SimpleNamespace(unique_id=41,
            position=SimpleNamespace(x=1.0, y=0.0))])
        def attempt(clock):
            clock.sleep(0.1)
            value._collision(collision)
            value._collision(collision)
            self.assertEqual(value.collision_count, 0)
            self.assertEqual([event[0] for event in value.events], ["PREPARATION_COLLISION"])
            value.wait_stationary(after=10.0, timeout=1, hold_sec=0.1)
        with self.assertRaisesRegex(SafetyFailure, "접촉"):
            self.run_with_clock(value, attempt)

    def test_stop_check_does_not_reject_the_collision_that_triggered_braking(self):
        value = monitor()
        value.preparation_collisions = {1: {"object_id": 1,
            "receipt_monotonic": 10.1, "distance_m": 1.0}}
        result = self.run_with_clock(value, lambda clock: value.verify_stopped(
            after=10.0, timeout=1, hold_sec=0.1))
        self.assertTrue(result["park_verified"])

    def test_owned_process_exit_is_not_a_stationary_confirmation(self):
        value = monitor()
        process = SimpleNamespace(poll=lambda: 1)
        with self.assertRaisesRegex(SafetyFailure, "프로세스"):
            self.run_with_clock(value, lambda clock: value.wait_stationary(
                after=10.0, timeout=1, process=process))


if __name__ == "__main__":
    unittest.main()
