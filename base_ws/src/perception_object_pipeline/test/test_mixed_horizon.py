#!/usr/bin/env python3
"""Pure policy regression; no ROS, Lanelet2 bindings or simulator required."""

import math
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

# The package __init__ intentionally imports ROS message converters.  Load this
# independent policy directly so this suite genuinely runs without ROS setup.
_SPEC = importlib.util.spec_from_file_location(
    "mixed_horizon_policy_test_module",
    Path(__file__).resolve().parents[1] / "src/perception_object_pipeline/mixed_horizon.py",
)
_POLICY = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _POLICY
_SPEC.loader.exec_module(_POLICY)
MixedHorizonSelector = _POLICY.MixedHorizonSelector
build_highway_horizons = _POLICY.build_highway_horizons


def uuid(value=1):
    return bytes([0] * 15 + [value])


def path(*ids):
    return SimpleNamespace(lanelet_ids=ids)


def network():
    return SimpleNamespace(records={
        101: SimpleNamespace(attributes=(("speed_limit", "150"),)),
        102: SimpleNamespace(attributes=(("speed_limit", "150.0"),)),
        201: SimpleNamespace(attributes=(("speed_limit", "50"),)),
        202: SimpleNamespace(attributes=()),
    })


class HighwayHorizonConfigurationTest(unittest.TestCase):
    BASE = tuple(.1 * index for index in range(1, 31))

    def test_eight_seconds_is_80_future_samples_with_exact_baseline_prefix(self):
        result = build_highway_horizons(self.BASE, .1, 8.)
        self.assertEqual(len(result), 80)
        self.assertEqual(result[-1], 8.)
        self.assertEqual(result[:30], self.BASE)

    def test_six_seconds_remains_compatible(self):
        result = build_highway_horizons(self.BASE, .1, 6.)
        self.assertEqual(result, tuple(.1 * index for index in range(1, 61)))

    def test_wire_capacity_includes_current_pose(self):
        result = build_highway_horizons(self.BASE, .1, 10.0)
        self.assertEqual(len(result) + 1, _POLICY.MAX_PREDICTED_PATH_POSES)
        with self.assertRaises(ValueError):
            build_highway_horizons(self.BASE, .1, 10.1)

    def test_invalid_horizon_is_rejected_before_unbounded_allocation(self):
        for value in (None, 'invalid', True, False, [], math.nan, math.inf,
                      -math.inf, -8., 0., 2.9, 3., 3.01, 8.05, 1.e308):
            with self.subTest(value=value), self.assertRaises(ValueError):
                build_highway_horizons(self.BASE, .1, value)

    def test_different_baseline_or_step_cannot_silently_change_prefix(self):
        for base, step in ((self.BASE, .2), (self.BASE, math.nan),
                           (self.BASE, True), (self.BASE[:-1] + (3.05,), .1),
                           (None, .1), (tuple(range(1, 31)), .1)):
            with self.subTest(base=base, step=step), self.assertRaises(ValueError):
                build_highway_horizons(base, step, 8.)

    def test_configurable_rotary_baseline_four_and_six_seconds(self):
        for seconds in (4., 6.):
            base = tuple(.1 * i for i in range(1, int(seconds * 10) + 1))
            result = build_highway_horizons(base, .1, 8.)
            self.assertEqual(result[:len(base)], base)
            self.assertEqual(result[-1], 8.)


class MixedHorizonSelectorTest(unittest.TestCase):
    def decide(self, selector, stamp=0.0, *, ident=None, x=0.0, y=0.0,
               speed=10.0, paths=None):
        return selector.decide(uuid() if ident is None else ident, stamp, x, y,
                               speed, [path(101, 102)] if paths is None else paths)

    def test_highway_metadata_is_derived_not_hardcoded_or_npc_speed(self):
        selector = MixedHorizonSelector(network(), entry_confirm_sec=0.0)
        self.assertEqual(selector.highway_lanelet_ids, frozenset((101, 102)))
        self.assertTrue(selector.metadata_available)
        self.assertTrue(self.decide(selector, speed=1.0).extend)
        self.assertFalse(self.decide(selector, speed=150.0, paths=[path(201)]).extend)

    def test_missing_map_or_attributes_is_baseline_not_startup_failure(self):
        for source in (None, SimpleNamespace(), SimpleNamespace(records={}),
                       SimpleNamespace(records={1: SimpleNamespace()}),
                       SimpleNamespace(records={1: SimpleNamespace(attributes=None)})):
            with self.subTest(source=source):
                selector = MixedHorizonSelector(source)
                self.assertFalse(selector.metadata_available)
                self.assertFalse(self.decide(selector).extend)
                self.assertEqual(selector.state_count, 0)

    def test_contradictory_or_unitful_metadata_is_not_eligible(self):
        source = SimpleNamespace(records={
            1: SimpleNamespace(attributes=(("speed_limit", "150"), ("speed_limit", "50"))),
            2: SimpleNamespace(attributes={"speed_limit": "150 km/h"}),
            3: SimpleNamespace(attributes={"speed_limit": "nan"}),
        })
        self.assertEqual(MixedHorizonSelector(source).highway_lanelet_ids, frozenset())

    def test_all_selected_paths_must_be_unambiguously_highway(self):
        selector = MixedHorizonSelector(network(), entry_confirm_sec=0.0)
        for paths, reason in (([], "no_baseline_map_paths"),
                              ([path()], "missing_route_lanelets"),
                              ([path(101), path(999)], "route_unknown_lanelet"),
                              ([path(101), path(201)], "route_not_highway"),
                              ([path(101, 202)], "route_not_highway"),
                              ([object()], "invalid_baseline_map_paths")):
            with self.subTest(paths=paths):
                decision = self.decide(selector, paths=paths)
                self.assertFalse(decision.extend)
                self.assertEqual(decision.reason, reason)
        self.assertTrue(self.decide(selector, paths=[path(101), path(102)]).extend)

    def test_native_current_lane_scope_ignores_future_region_and_unknown_successor(self):
        selector = MixedHorizonSelector(network(), entry_confirm_sec=0., current_lanelet_only=True)
        for paths in ([path(101, 201)], [path(101, 999)], [path(101, 201), path(102, 202)]):
            with self.subTest(paths=paths):
                self.assertTrue(self.decide(selector, paths=paths).extend)
        self.assertFalse(self.decide(selector, paths=[path(201, 101)]).extend)
        self.assertFalse(self.decide(selector, paths=[path(101), path(201)]).extend)
        self.assertFalse(self.decide(selector, paths=[path(999, 101)]).extend)

    def test_native_current_lane_scope_still_requires_identity_and_confirmation(self):
        selector = MixedHorizonSelector(network(), current_lanelet_only=True)
        self.assertFalse(self.decide(selector, 10., paths=[path(101, 201)]).extend)
        self.assertTrue(self.decide(selector, 10.5, paths=[path(101, 201)]).extend)
        self.assertFalse(self.decide(selector, 10.6, paths=[path(201, 101)]).extend)
        self.assertFalse(self.decide(selector, 10.7, paths=[path(101)]).extend)
        self.assertFalse(self.decide(selector, 11.2, ident=bytes(16)).extend)

    def test_native_geographic_coverage_does_not_require_forward_motion(self):
        native = MixedHorizonSelector(network(), entry_confirm_sec=0., current_lanelet_only=True)
        append = MixedHorizonSelector(network(), entry_confirm_sec=0.)
        for speed in (0., .1, .49, -10.):
            with self.subTest(speed=speed):
                self.assertTrue(self.decide(native, speed=speed).extend)
                self.assertFalse(self.decide(append, speed=speed).extend)

    def test_entry_waits_half_second_and_exit_is_immediate(self):
        selector = MixedHorizonSelector(network())
        for stamp in (10.0, 10.1, 10.2, 10.49):
            self.assertFalse(self.decide(selector, stamp).extend)
        self.assertTrue(self.decide(selector, 10.5).extend)
        self.assertFalse(self.decide(selector, 10.6, paths=[path(201)]).extend)
        self.assertFalse(self.decide(selector, 10.7).extend)
        self.assertTrue(self.decide(selector, 11.2).extend)

    def test_thirty_hz_outputs_are_only_elapsed_classification_stability(self):
        selector = MixedHorizonSelector(network())
        # There is deliberately no invented detector-count/freshness claim.
        for index in range(15):
            self.assertFalse(self.decide(selector, index / 30.0).extend)
        self.assertTrue(self.decide(selector, 0.5).extend)
        self.assertTrue(self.decide(selector, 0.5).extend)

    def test_repeated_timestamp_does_not_accumulate_confirmation(self):
        selector = MixedHorizonSelector(network())
        for _ in range(100):
            self.assertFalse(self.decide(selector, 1.0).extend)
        self.assertFalse(self.decide(selector, 1.49).extend)
        self.assertTrue(self.decide(selector, 1.5).extend)

    def test_unknown_zero_malformed_uuid_never_shared_or_cached(self):
        selector = MixedHorizonSelector(network(), entry_confirm_sec=0.0)
        for ident in (bytes(16), b"x", [1] * 15, [256] * 16, [-1] * 16,
                      [True] * 16, "not-a-uuid", None, 16):
            with self.subTest(ident=ident):
                result = selector.decide(ident, 0.0, 0.0, 0.0, 1.0, [path(101)])
                self.assertFalse(result.extend)
                self.assertEqual(result.reason, "invalid_track_id")
        self.assertEqual(selector.state_count, 0)
        self.assertTrue(selector.decide(list(uuid()), 0.0, 0.0, 0.0, 1.0, [path(101)]).extend)

    def test_each_uuid_confirms_independently(self):
        selector = MixedHorizonSelector(network())
        self.decide(selector, 1.0)
        self.assertFalse(self.decide(selector, 1.4, ident=uuid(2)).extend)
        self.assertTrue(self.decide(selector, 1.5).extend)
        self.assertFalse(self.decide(selector, 1.5, ident=uuid(2)).extend)

    def test_absence_and_id_reuse_restart_confirmation(self):
        selector = MixedHorizonSelector(network())
        self.decide(selector, 0.0)
        self.assertTrue(self.decide(selector, 0.5).extend)
        selector.prune(0.6, [])
        self.assertEqual(selector.state_count, 0)
        self.assertFalse(self.decide(selector, 0.7).extend)
        self.assertTrue(self.decide(selector, 1.2).extend)
        self.assertFalse(self.decide(selector, 3.21).extend)

    def test_position_jump_resets_even_if_uuid_is_reused_quickly(self):
        selector = MixedHorizonSelector(network())
        self.decide(selector, 0.0)
        self.assertTrue(self.decide(selector, 0.5, x=5.0).extend)
        decision = self.decide(selector, 0.6, x=100.0)
        self.assertFalse(decision.extend)
        self.assertEqual(decision.reason, "position_jump_reset")
        self.assertTrue(self.decide(selector, 1.1, x=105.0).extend)

    def test_backwards_time_resets_all_uuids(self):
        selector = MixedHorizonSelector(network())
        self.decide(selector, 10.0)
        self.decide(selector, 10.0, ident=uuid(2))
        self.assertTrue(self.decide(selector, 10.5).extend)
        self.assertTrue(self.decide(selector, 10.5, ident=uuid(2)).extend)
        selector.prune(1.0, [uuid(), uuid(2)])
        self.assertFalse(self.decide(selector, 1.0).extend)
        self.assertFalse(self.decide(selector, 1.0, ident=uuid(2)).extend)

    def test_track_state_is_bounded_and_pruned_without_predictions(self):
        selector = MixedHorizonSelector(network(), maximum_tracks=2)
        for value in (1, 2, 3):
            self.decide(selector, 0.0, ident=uuid(value))
        self.assertEqual(selector.state_count, 2)
        self.assertFalse(self.decide(selector, 0.5, ident=uuid(1)).extend)
        selector.prune(3.0)
        self.assertEqual(selector.state_count, 0)

    def test_invalid_speed_state_or_path_resets_confirmation(self):
        selector = MixedHorizonSelector(network())
        for invalid in ({"speed": 0.0}, {"speed": -1.0}, {"speed": math.nan},
                        {"x": math.inf}, {"paths": None}):
            self.decide(selector, 1.0)
            self.assertTrue(self.decide(selector, 1.5).extend)
            # None means default in this test helper; pass malformed explicitly.
            value = dict(invalid)
            if "paths" in value:
                value["paths"] = [SimpleNamespace(lanelet_ids=None)]
            self.assertFalse(self.decide(selector, 1.6, **value).extend)
            self.assertFalse(self.decide(selector, 1.7).extend)
            selector.prune(0.0, [])

    def test_invalid_prune_time_clears_stale_confirmation(self):
        selector = MixedHorizonSelector(network(), entry_confirm_sec=0.0)
        self.assertTrue(self.decide(selector).extend)
        selector.prune(float("nan"))
        self.assertEqual(selector.state_count, 0)

    def test_configuration_validation_does_not_allow_unbounded_cache(self):
        for kwargs in ({"maximum_tracks": 0}, {"maximum_tracks": 2.5},
                       {"entry_confirm_sec": -1.0}, {"state_timeout_sec": 0.0},
                       {"minimum_speed_mps": float("nan")}, {"current_lanelet_only": 'native'}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                MixedHorizonSelector(network(), **kwargs)


if __name__ == "__main__":
    unittest.main()
