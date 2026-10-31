#!/usr/bin/env python3
"""Shared-endcap numerical tolerance must not snap off-road or extend terminals."""
import unittest

from perception_object_pipeline.highway_lane_policy import HighwayCurrentLanePolicy
from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord


def seam_network(terminal=False, shifted_following=False):
    old = LaneletRecord(1, ((0., 0., 0.), (10., 0., 0.)),
        () if terminal else (2,), (('speed_limit', '150'),),
        ((0., 1.75, 0.), (10.05, 1.75, 0.)),
        ((0., -1.75, 0.), (9.95, -1.75, 0.)))
    delta = .05 if shifted_following else 0.
    following = LaneletRecord(2, ((10., 0., 0.), (200., 0., 0.)), (),
        (('speed_limit', '150'),),
        ((10.05 + delta, 1.75, 0.), (200., 1.75, 0.)),
        ((9.95 + delta, -1.75, 0.), (200., -1.75, 0.)))
    return LaneletNetwork([old] if terminal else [old, following])


class SharedForwardSeamNumericsTest(unittest.TestCase):
    identity = bytes([31]) * 16

    def seed(self, policy):
        self.assertEqual(policy.select(self.identity, 10., 9.9, .4, 0., 11.).lanelet_id, 1)

    def test_inside_polygon_submillimeter_past_center_end_retains_then_advances(self):
        policy = HighwayCurrentLanePolicy(seam_network())
        self.seed(policy)
        decision = policy.select(self.identity, 10.1, 10.0004, .4, 0., 11.)
        self.assertEqual(decision.lanelet_id, 1)
        self.assertEqual(decision.reason, 'retained_forward_seam_numerics')
        self.assertEqual(policy.select(self.identity, 10.2, 10.1, .4, 0., 11.).lanelet_id, 2)

    def test_more_than_two_centimeters_is_not_retained(self):
        policy = HighwayCurrentLanePolicy(seam_network())
        self.seed(policy)
        self.assertIsNone(policy.select(self.identity, 10.1, 10.025, 1., 0., 11.).lanelet_id)

    def test_absent_history_does_not_snap_into_seam(self):
        policy = HighwayCurrentLanePolicy(seam_network())
        self.assertIsNone(policy.select(self.identity, 10.1, 10.0004, .4, 0., 11.).lanelet_id)

    def test_real_terminal_never_retains_past_center_end(self):
        policy = HighwayCurrentLanePolicy(seam_network(terminal=True))
        self.seed(policy)
        self.assertIsNone(policy.select(self.identity, 10.1, 10.0004, .4, 0., 11.).lanelet_id)

    def test_lateral_polygon_outside_is_not_relaxed(self):
        policy = HighwayCurrentLanePolicy(seam_network())
        self.seed(policy)
        self.assertIsNone(policy.select(self.identity, 10.1, 10.0004, 1.76, 0., 11.).lanelet_id)

    def test_nonshared_successor_boundary_does_not_relax_endpoint(self):
        policy = HighwayCurrentLanePolicy(seam_network(shifted_following=True))
        self.seed(policy)
        self.assertIsNone(policy.select(self.identity, 10.1, 10.0004, .4, 0., 11.).lanelet_id)

    def test_negative_velocity_still_invalid_and_clears_history(self):
        policy = HighwayCurrentLanePolicy(seam_network())
        self.seed(policy)
        self.assertEqual(policy.select(self.identity, 10.1, 10.0004, .4, 0., -.7).reason,
                         'invalid_state')
        self.assertIsNone(policy.select(self.identity, 10.2, 10.0004, .4, 0., 11.).lanelet_id)

    def test_short_terminal_forecast_is_not_extended(self):
        network = seam_network(terminal=True)
        paths = network.predict(x=9.9, y=.4, z=0., yaw=0., speed=1.,
            horizons=(.1, .2, .3), truncate_at_route_end=True, allow_low_speed=True,
            start_lanelet_ids=(1,), single_route=True)
        self.assertEqual(len(paths), 1)
        self.assertEqual(len(paths[0].poses), 1)


if __name__ == '__main__':
    unittest.main()
