#!/usr/bin/env python3
"""Current physical lane association and single continuation, without ROS."""
import unittest
from unittest.mock import patch

from perception_object_pipeline.highway_lane_policy import HighwayCurrentLanePolicy
from perception_object_pipeline.map_based_prediction import (
    LaneletNetwork, LaneletRecord, MapPredictionConfig,
)


def identity(value=1):
    return bytes([value]) * 16


def road(lanelet_id, x0, x1, y, following=(), speed='150', half_width=1.75):
    return LaneletRecord(
        lanelet_id, ((x0, y, 0.), (x1, y, 0.)), tuple(following),
        (('speed_limit', speed),),
        left_boundary=((x0, y + half_width, 0.), (x1, y + half_width, 0.)),
        right_boundary=((x0, y - half_width, 0.), (x1, y - half_width, 0.)))


class HighwayLanePolicyTest(unittest.TestCase):
    def setUp(self):
        self.network = LaneletNetwork([
            road(1, -10., 200., 0.), road(2, -10., 200., 3.5),
            road(3, -10., 200., 30., speed='50'),
        ], MapPredictionConfig(maximum_start_lanelets=3))
        self.policy = HighwayCurrentLanePolicy(self.network, boundary_hysteresis_m=.2,
                                               state_timeout_sec=1.)

    def choose(self, stamp, y, x=5., policy=None, track_id=None):
        return (policy or self.policy).select(
            identity() if track_id is None else track_id, stamp,
            x, y, 0., 10.).lanelet_id

    def test_polygon_membership_wins_over_neighbor_candidate_score(self):
        # Both centerlines fit the ordinary 3m association gate. Polygon
        # membership, rather than route confidence/rank, identifies lane 1.
        candidates = self.network.lane_candidates(5., 1.6, 0.)
        self.assertEqual({candidate.lanelet_id for candidate in candidates}, {1, 2})
        with patch.object(self.network, 'lane_candidates', return_value=tuple(reversed(candidates))):
            self.assertEqual(self.choose(10., 1.6), 1)

    def test_shared_boundary_jitter_stays_on_previous_physical_lane(self):
        self.assertEqual(self.choose(10., 0.), 1)
        for index, y in enumerate((1.70, 1.78, 1.73, 1.82, 1.76)):
            with self.subTest(y=y):
                self.assertEqual(self.choose(10.1 + index * .1, y), 1)

    def test_real_lateral_entry_switches_and_does_not_jump_back_with_jitter(self):
        self.assertEqual(self.choose(10., 0.), 1)
        self.assertEqual(self.choose(10.1, 2.05), 2)
        for index, y in enumerate((1.82, 1.72, 1.80, 1.68)):
            with self.subTest(y=y):
                self.assertEqual(self.choose(10.2 + index * .1, y), 2)

    def test_legitimate_return_to_previous_lane_is_still_allowed(self):
        self.assertEqual(self.choose(10., 0.), 1)
        self.assertEqual(self.choose(10.1, 2.05), 2)
        self.assertEqual(self.choose(10.2, 1.4), 1)

    def test_longitudinal_successor_seam_advances_instead_of_sticking_to_old_lane(self):
        network = LaneletNetwork([
            road(1, -10., 10., 0., following=(2,)), road(2, 10., 200., 0.),
        ])
        policy = HighwayCurrentLanePolicy(network, boundary_hysteresis_m=.2)
        self.assertEqual(self.choose(10., 0., x=9.9, policy=policy), 1)
        self.assertEqual(self.choose(10.1, 0., x=10.1, policy=policy), 2)
        self.assertEqual(self.choose(10.2, 0., x=11., policy=policy), 2)

    def test_uuid_disappearance_clears_boundary_memory(self):
        self.assertEqual(self.choose(10., 0.), 1)
        self.assertEqual(self.choose(10.1, 1.8), 1)
        self.policy.prune(10.2, [])
        self.assertEqual(self.choose(10.3, 1.8), 2)

    def test_timestamp_regression_clears_boundary_memory(self):
        self.assertEqual(self.choose(10., 0.), 1)
        self.assertEqual(self.choose(10.1, 1.8), 1)
        self.assertEqual(self.choose(9., 1.8), 2)

    def test_long_gap_clears_boundary_memory(self):
        self.assertEqual(self.choose(10., 0.), 1)
        self.assertEqual(self.choose(10.1, 1.8), 1)
        self.assertEqual(self.choose(11.2, 1.8), 2)

    def test_different_uuid_does_not_share_boundary_memory(self):
        self.assertEqual(self.choose(10., 0., track_id=identity(1)), 1)
        self.assertEqual(self.choose(10.1, 1.8, track_id=identity(1)), 1)
        self.assertEqual(self.choose(10.1, 1.8, track_id=identity(2)), 2)

    def test_geometry_helper_can_identify_ordinary_lane_but_never_invents_far_away_lane(self):
        # The caller owns geographic highway eligibility; the geometry helper
        # must identify the actual ordinary lane in an overlapping map seam.
        self.assertEqual(self.choose(10., 30.), 3)
        self.assertIsNone(self.choose(10.1, 100.))

    def test_invalid_uuid_or_state_never_creates_or_retains_membership(self):
        self.assertIsNone(self.choose(10., 0., track_id=bytes(16)))
        self.assertEqual(self.policy.state_count, 0)
        self.assertEqual(self.choose(10., 0.), 1)
        decision = self.policy.select(identity(), 10.1, float('nan'), 0., 0., 10.)
        self.assertIsNone(decision.lanelet_id)
        self.assertEqual(self.policy.state_count, 0)
        self.assertEqual(self.choose(10.2, 1.8), 2)

    def test_past_actual_terminal_does_not_clamp_membership_to_endpoint(self):
        network = LaneletNetwork([road(1, -10., 20., 0.)])
        policy = HighwayCurrentLanePolicy(network, boundary_hysteresis_m=.2)
        self.assertEqual(self.choose(10., 0., x=19.9, policy=policy), 1)
        self.assertIsNone(self.choose(10.1, 0., x=20.2, policy=policy))

    def test_missing_boundaries_use_current_nearest_centerline_not_future_score(self):
        network = LaneletNetwork([
            LaneletRecord(1, ((-10., 0., 0.), (200., 0., 0.))),
            LaneletRecord(2, ((-10., 3.5, 0.), (200., 3.5, 0.))),
        ])
        policy = HighwayCurrentLanePolicy(network)
        self.assertEqual(self.choose(10., 1.6, policy=policy), 1)
        self.assertEqual(self.choose(10.1, 2.2, policy=policy), 2)

    def test_actual_boundaries_reject_outside_road_even_within_centerline_gate(self):
        network = LaneletNetwork([road(1, -10., 200., 0.)])
        self.assertIsNone(self.choose(10., 2., policy=HighwayCurrentLanePolicy(network)))

    def test_overlapping_polygons_switch_only_after_current_center_is_clearly_closer(self):
        network = LaneletNetwork([road(1, -10., 200., 0., half_width=4.),
                                 road(2, -10., 200., 3.5, half_width=4.)])
        policy = HighwayCurrentLanePolicy(network, boundary_hysteresis_m=.2)
        self.assertEqual(self.choose(10., 0., policy=policy), 1)
        self.assertEqual(self.choose(10.1, 1.80, policy=policy), 1)
        self.assertEqual(self.choose(10.2, 1.90, policy=policy), 2)
        self.assertEqual(self.choose(10.3, 1.72, policy=policy), 2)
        self.assertEqual(self.choose(10.4, 1.67, policy=policy), 2)
        self.assertEqual(self.choose(10.5, 1.60, policy=policy), 1)

    def test_closer_future_centerline_cannot_steal_when_its_polygon_does_not_contain_center(self):
        network = LaneletNetwork([road(1, -10., 200., 0., half_width=4.),
                                 road(2, -10., 200., 3.5, half_width=.5)])
        policy = HighwayCurrentLanePolicy(network, boundary_hysteresis_m=.2)
        self.assertEqual(self.choose(10., 0., policy=policy), 1)
        # The second centerline is closer, but y=2.5 is not in its polygon.
        self.assertEqual(self.choose(10.1, 2.5, policy=policy), 1)

    def test_state_capacity_is_bounded_and_evicted_uuid_has_no_boundary_memory(self):
        policy = HighwayCurrentLanePolicy(self.network, maximum_tracks=2)
        self.assertEqual(self.choose(10., 0., policy=policy, track_id=identity(1)), 1)
        self.assertEqual(self.choose(10., 0., policy=policy, track_id=identity(2)), 1)
        self.assertEqual(self.choose(10., 1.8, policy=policy, track_id=identity(1)), 1)
        self.assertEqual(self.choose(10., 0., policy=policy, track_id=identity(3)), 1)
        self.assertEqual(policy.state_count, 2)
        self.assertEqual(self.choose(10.1, 1.8, policy=policy, track_id=identity(2)), 2)
        self.assertEqual(policy.state_count, 2)


class SingleCurrentLaneRouteTest(unittest.TestCase):
    def setUp(self):
        self.network = LaneletNetwork([
            road(1, -10., 10., 0., following=(4, 3)),
            road(3, 10., 200., 0.),
            LaneletRecord(4, ((10., 0., 0.), (200., 15., 0.)), (),
                          (('speed_limit', '150'),)),
        ], MapPredictionConfig(maximum_start_lanelets=3))
        self.values = dict(x=0., y=0., z=0., yaw=0., speed=10.,
                           horizons=tuple(.1 * i for i in range(1, 101)),
                           truncate_at_route_end=True, allow_low_speed=True)

    def test_close_confidence_fork_follows_geometric_continuation_not_both(self):
        multiple = self.network.predict(**self.values)
        self.assertEqual(len(multiple), 2)
        self.assertLess(abs(multiple[0].confidence - multiple[1].confidence), .01)
        single = self.network.predict(**self.values, start_lanelet_ids=(1,), single_route=True)
        self.assertEqual(len(single), 1)
        self.assertEqual(single[0].lanelet_ids, (1, 3))
        self.assertEqual(len(single[0].poses), 100)
        self.assertAlmostEqual(single[0].poses[-1][1], 0.)

    def test_selected_start_lane_excludes_more_confident_adjacent_lane(self):
        network = LaneletNetwork([road(1, -10., 200., 0.), road(2, -10., 200., 3.5)],
                                 MapPredictionConfig(maximum_start_lanelets=3))
        values = dict(self.values, y=1.8)
        ordinary = network.predict(**values)
        self.assertEqual(ordinary[0].lanelet_ids, (2,))
        single = network.predict(**values, start_lanelet_ids=(1,), single_route=True)
        self.assertEqual(len(single), 1)
        self.assertEqual(single[0].lanelet_ids, (1,))
        self.assertAlmostEqual(single[0].poses[-1][1], 0.)

    def test_true_selected_terminal_keeps_available_prefix(self):
        network = LaneletNetwork([road(1, -10., 20., 0.)])
        paths = network.predict(**self.values, start_lanelet_ids=(1,), single_route=True)
        self.assertEqual(len(paths), 1)
        self.assertEqual(paths[0].lanelet_ids, (1,))
        self.assertEqual(len(paths[0].poses), 20)
        self.assertEqual(paths[0].poses[-1][:2], (20., 0.))

    def test_no_single_route_option_preserves_existing_multiple_prediction(self):
        original = self.network.predict(**self.values)
        explicit = self.network.predict(**self.values, start_lanelet_ids=None, single_route=False)
        self.assertEqual(original, explicit)

    def test_nonexistent_start_lane_does_not_invent_a_road(self):
        self.assertEqual(self.network.predict(**self.values,
                                             start_lanelet_ids=(999,), single_route=True), ())


if __name__ == '__main__':
    unittest.main()
