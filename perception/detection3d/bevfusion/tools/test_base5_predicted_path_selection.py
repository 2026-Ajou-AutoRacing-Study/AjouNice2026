import unittest
from dataclasses import dataclass
from types import SimpleNamespace as N

from audit_base5_predicted_path_selection import selected_indices, sample, separation, stats
from probe_base5_map_lane_history import history_choice, lanelet_coordinates, future_point, history_weighted_network


class SelectionTests(unittest.TestCase):
    def test_empty(self):
        for mode in ('lc_max_ties', 'cruise_one', 'roundabout_threshold'):
            self.assertEqual(selected_indices([], mode), [])

    def test_lc_is_not_strict_top1(self):
        self.assertEqual(selected_indices([.4, .4, .2], 'lc_max_ties'), [0, 1])

    def test_lc_not_near_ties(self):
        self.assertEqual(selected_indices([.4000001, .4], 'lc_max_ties'), [0])

    def test_cruise_one(self):
        self.assertEqual(len(selected_indices([.5, .5], 'cruise_one')), 1)

    def test_roundabout_not_top1(self):
        self.assertEqual(selected_indices([.8, .15, .05, .04], 'roundabout_threshold'), [0, 1, 2])

    def test_invalid_confidence(self):
        with self.assertRaises(ValueError):
            selected_indices([float('nan')], 'lc_max_ties')

    def test_no_extrapolation(self):
        p = dict(dt=1., points=[(0., 0.), (1., 0.)])
        self.assertIsNone(sample(p, 1.1))
        self.assertIsNone(sample(p, -.1))
        self.assertEqual(sample(p, .5), (.5, 0.))

    def test_common_future_removes_normal_translation(self):
        old = dict(dt=1., points=[(0., 0.), (10., 0.), (20., 0.)])
        new = dict(dt=1., points=[(1., 0.), (11., 0.), (21., 0.)])
        self.assertAlmostEqual(separation(old, new, 1., .9), 0.)

    def test_missing_statistic(self):
        self.assertIsNone(stats([])['p95'])

    def test_history_holds_near_tie_but_not_clear_lane_change(self):
        records = {1: N(following_ids=(3,)), 2: N(following_ids=())}
        self.assertEqual(history_choice([N(lanelet_id=2, score=.51), N(lanelet_id=1, score=.49)],
                                        (1., 1), 1.1, records), 1)
        self.assertEqual(history_choice([N(lanelet_id=2, score=.8), N(lanelet_id=1, score=.2)],
                                        (1., 1), 1.1, records), 2)

    def test_history_never_revives_missing_or_expired_lane(self):
        records = {1: N(following_ids=(3,))}
        self.assertEqual(history_choice([N(lanelet_id=2, score=.5)], (1., 1), 1.1, records), 2)
        self.assertEqual(history_choice([N(lanelet_id=2, score=.51), N(lanelet_id=1, score=.49)],
                                        (1., 1), 2., records), 2)
        self.assertIsNone(history_choice([], (1., 1), 1.1, records))

    def test_history_allows_successor_without_choosing_future_branch(self):
        records = {1: N(following_ids=(3, 4))}
        self.assertEqual(history_choice([N(lanelet_id=2, score=.51), N(lanelet_id=3, score=.49)],
                                        (1., 1), 1.1, records), 3)

    def test_map_translation_applied_once(self):
        tf = dict(parent='map', child='morai_map', xyz=[131.559130220, 545.911338186, 0.],
                  rpy=[0., 0., 0.])
        p, yaw = lanelet_coordinates([-131.689798, -428.331023, 28.54396], 1., tf)
        self.assertAlmostEqual(p[0], -.13066778)
        self.assertAlmostEqual(p[1], 117.580315186)
        self.assertEqual(yaw, 1.)
        self.assertEqual(lanelet_coordinates(p, yaw, None), (p, yaw))

    def test_wrong_transform_rejected(self):
        with self.assertRaises(ValueError):
            lanelet_coordinates([0., 0., 0.], 0.,
                                dict(parent='morai_map', child='map', rpy=[0., 0., 0.]))

    def test_future_gt_no_extrapolation_or_large_gap(self):
        series=[(1.,(0.,0.)),(1.1,(1.,0.)),(2.,(2.,0.))]
        self.assertAlmostEqual(future_point(series,1.05)[0],.5)
        self.assertIsNone(future_point(series,.9))
        self.assertIsNone(future_point(series,1.5))
        self.assertIsNone(future_point(series,2.1))

    def test_soft_prior_keeps_candidates_and_original_network(self):
        @dataclass(frozen=True)
        class Candidate:
            lanelet_id: int
            score: float
        candidates=(Candidate(1,.51),Candidate(2,.49))
        original=N(lane_candidates=lambda x,y,yaw:candidates)
        temporary=history_weighted_network(original,candidates,2)
        self.assertEqual(original.lane_candidates(0,0,0),candidates)
        selected=temporary.lane_candidates(0,0,0)
        self.assertEqual({c.lanelet_id for c in selected},{1,2})
        self.assertEqual(selected[0].lanelet_id,2)
        self.assertAlmostEqual(selected[0].score,.49*1.25)


if __name__ == '__main__':
    unittest.main()
