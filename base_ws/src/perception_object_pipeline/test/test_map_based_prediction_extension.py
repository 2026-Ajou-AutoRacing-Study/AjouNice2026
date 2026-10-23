#!/usr/bin/env python3
"""Optional extension must never select a new 3-second hypothesis."""

import math
import pickle
import unittest
from unittest import mock

from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork,
    LaneletRecord,
    MapPredictionConfig,
    PredictedTrajectory,
)


class RoutePreservingExtensionTest(unittest.TestCase):
    BASE = (1.0, 2.0, 3.0)
    EXTENDED = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)

    def network(self, records=None, **settings):
        records = records or (
            LaneletRecord(1, ((0., 0., 0.), (30., 0., 1.)), (2,)),
            LaneletRecord(2, ((30., 0., 1.), (60., 0., 2.)), (3,)),
            LaneletRecord(3, ((60., 0., 2.), (100., 0., 3.))),
        )
        settings = dict(maximum_start_lanelets=1,
                        lateral_convergence_time_sec=8.0, **settings)
        return LaneletNetwork(records, MapPredictionConfig(**settings))

    def baseline(self, network, **overrides):
        arguments = dict(x=1., y=0.5, z=2., yaw=0., speed=10., horizons=self.BASE)
        arguments.update(overrides)
        return network.predict(**arguments)

    def extend(self, network, trajectories, **overrides):
        arguments = dict(x=1., y=0.5, z=2., yaw=0., speed=10.,
                         base_horizons=self.BASE, extended_horizons=self.EXTENDED,
                         trajectories=trajectories)
        arguments.update(overrides)
        return network.extend_trajectories(**arguments)

    def test_success_appends_without_prediction_reranking_and_preserves_prefix(self):
        network = self.network()
        baseline = self.baseline(network)
        serialized = pickle.dumps(baseline)
        with mock.patch.object(network, "predict", side_effect=AssertionError("rerank")), \
             mock.patch.object(network, "lane_candidates", side_effect=AssertionError("rematch")), \
             mock.patch.object(network, "_distinct_trajectory_hypotheses",
                               side_effect=AssertionError("cluster")):
            result = self.extend(network, baseline, allowed_lanelet_ids={1, 2, 3})
        self.assertEqual(len(result), len(baseline))
        self.assertEqual(result[0].poses[:3], baseline[0].poses)
        self.assertEqual(result[0].confidence, baseline[0].confidence)
        self.assertEqual(result[0].lanelet_ids, (1, 2, 3))
        self.assertEqual(result[0].poses[-1][0], 61.)
        self.assertEqual(pickle.dumps(baseline), serialized)
        # Existing predict() is unaffected by metadata or warmed route caches.
        self.assertEqual(pickle.dumps(self.baseline(network)), serialized)

    def test_already_long_enough_route_retains_original_ids(self):
        network = self.network((LaneletRecord(1, ((0., 0., 0.), (100., 0., 0.))),))
        baseline = self.baseline(network)
        result = self.extend(network, baseline)
        self.assertEqual(result[0].lanelet_ids, (1,))
        self.assertEqual(result[0].poses[:3], baseline[0].poses)
        self.assertEqual(result[0].poses[-1][0], 61.)

    def test_branch_after_baseline_is_not_guessed_even_when_only_one_is_allowed(self):
        network = self.network((
            LaneletRecord(1, ((0., 0., 0.), (40., 0., 0.)), (2, 3)),
            LaneletRecord(2, ((40., 0., 0.), (100., 0., 0.))),
            LaneletRecord(3, ((40., 0., 0.), (100., 10., 0.))),
        ))
        baseline = self.baseline(network)
        self.assertEqual(baseline[0].lanelet_ids, (1,))
        self.assertIs(self.extend(network, baseline), baseline)
        self.assertIs(self.extend(network, baseline, allowed_lanelet_ids={1, 2}), baseline)

    def test_original_multi_branch_order_confidences_and_prefix_are_exact(self):
        network = self.network((
            LaneletRecord(1, ((0., 0., 0.), (10., 0., 0.)), (2, 3)),
            LaneletRecord(2, ((10., 0., 0.), (100., 0., 0.))),
            LaneletRecord(3, ((10., 0., 0.), (80., 20., 0.))),
        ))
        baseline = self.baseline(network)
        self.assertEqual(len(baseline), 2)
        result = self.extend(network, baseline)
        self.assertEqual(len(result), 2)
        for old, new in zip(baseline, result):
            self.assertEqual(new.poses[:3], old.poses)
            self.assertEqual(new.confidence, old.confidence)
            self.assertEqual(new.lanelet_ids, old.lanelet_ids)

    def test_any_failed_path_keeps_whole_object_baseline(self):
        network = self.network((
            LaneletRecord(1, ((0., 0., 0.), (10., 0., 0.)), (2, 3)),
            LaneletRecord(2, ((10., 0., 0.), (100., 0., 0.))),
            LaneletRecord(3, ((10., 0., 0.), (45., 10., 0.))),
        ))
        baseline = self.baseline(network)
        self.assertEqual(len(baseline), 2)
        self.assertIs(self.extend(network, baseline), baseline)

    def test_no_clamping_padding_or_region_escape(self):
        network = self.network()
        baseline = self.baseline(network)
        self.assertIs(self.extend(network, baseline, allowed_lanelet_ids={1, 2}), baseline)
        self.assertIs(self.extend(network, baseline, allowed_lanelet_ids={2, 3}), baseline)
        self.assertIs(self.extend(network, baseline, allowed_lanelet_ids=set()), baseline)
        network.records[3] = LaneletRecord(3, ((60., 0., 2.), (60.9, 0., 3.)))
        self.assertIs(self.extend(network, baseline), baseline)

    def test_exact_route_endpoint_is_a_real_sample_not_padding(self):
        network = self.network((LaneletRecord(1, ((0., 0., 0.), (61., 0., 0.))),))
        baseline = self.baseline(network)
        result = self.extend(network, baseline)
        self.assertEqual(result[0].poses[-1][0], 61.)
        self.assertNotEqual(result[0].poses[-2], result[0].poses[-1])

    def test_baseline_already_clamped_by_legacy_tolerance_is_not_extended(self):
        network = self.network((LaneletRecord(1, ((0., 0., 0.), (30.9, 0., 0.)), (2,)),
                                LaneletRecord(2, ((30.9, 0., 0.), (100., 0., 0.)))))
        baseline = (PredictedTrajectory(((11., 0., 0., 0.), (21., 0., 0., 0.),
                                        (30.9, 0., 0., 0.)), 1., (1,)),)
        self.assertIs(self.extend(network, baseline), baseline)

    def test_later_nearby_route_segment_does_not_change_original_arc_origin(self):
        network = self.network((
            LaneletRecord(1, ((0., 0., 0.), (40., 0., 0.)), (2,)),
            LaneletRecord(2, ((40., 0., 0.), (40., 10., 0.),
                              (1., .4, 0.), (100., .4, 0.))),
        ))
        baseline = self.baseline(network)
        self.assertEqual(baseline[0].lanelet_ids, (1,))
        result = self.extend(network, baseline)
        self.assertEqual(result[0].poses[:3], baseline[0].poses)
        # A re-projection on the completed route would jump to the later
        # segment near (1, .4); the causal original route reaches (40, 1).
        self.assertEqual(result[0].poses[3][0], 40.)
        self.assertAlmostEqual(result[0].poses[3][1], 1.25)

    def test_depth_cap_and_loop_cap_bound_extension(self):
        network = self.network(maximum_route_depth=2)
        baseline = self.baseline(network)
        self.assertIs(self.extend(network, baseline), baseline)
        loop = self.network((LaneletRecord(1, ((0., 0., 0.), (20., 0., 0.)), (1,)),))
        baseline = self.baseline(loop)
        self.assertEqual(baseline[0].lanelet_ids, (1, 1))
        self.assertIs(self.extend(loop, baseline), baseline)

    def test_low_negative_zero_nonfinite_speed_do_not_extend(self):
        network = self.network()
        baseline = self.baseline(network)
        for speed in (0., -1., .1, math.inf, math.nan):
            with self.subTest(speed=speed):
                self.assertIs(self.extend(network, baseline, speed=speed), baseline)
        self.assertIs(self.extend(network, ()), ())

    def test_malformed_horizons_and_paths_keep_original_tuple(self):
        network = self.network()
        baseline = self.baseline(network)
        for extended in ((), self.BASE, (1., 2., 3.1, 4.),
                         (1., 2., 3., 3.), (1., 2., 3., math.inf),
                         (1., 2., 3., math.nan), (1., 2., 3., -1.)):
            with self.subTest(extended=extended):
                self.assertIs(self.extend(network, baseline,
                                          extended_horizons=extended), baseline)
        malformed = (PredictedTrajectory((), .123, (1,)),)
        self.assertIs(self.extend(network, malformed), malformed)
        missing = (PredictedTrajectory(baseline[0].poses, .123, (999,)),)
        self.assertIs(self.extend(network, missing), missing)

    def test_map_geometry_exceptions_are_not_blanket_suppressed(self):
        network = self.network()
        baseline = self.baseline(network)
        with mock.patch.object(network, "_route_geometry", side_effect=RuntimeError("map broken")):
            with self.assertRaisesRegex(RuntimeError, "map broken"):
                self.extend(network, baseline)

    def test_record_metadata_is_optional_immutable_and_does_not_change_predictions(self):
        bare = self.network()
        tagged = self.network(tuple(
            LaneletRecord(r.lanelet_id, r.centerline, r.following_ids,
                          (("subtype", "road"), ("speed_limit", "150")))
            for r in bare.records.values()
        ))
        self.assertEqual(self.baseline(bare), self.baseline(tagged))
        self.assertEqual(LaneletRecord(1, ()).attributes, ())
        self.assertEqual(dict(tagged.records[1].attributes)["speed_limit"], "150")
        with self.assertRaises(AttributeError):
            tagged.records[1].attributes = ()


if __name__ == "__main__":
    unittest.main()
