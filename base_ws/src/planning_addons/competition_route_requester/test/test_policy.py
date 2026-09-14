import copy
import math
from pathlib import Path
import unittest
import yaml
from competition_route_requester import policy
from competition_route_requester.geometry import Geometry

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[3]


class PolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = yaml.safe_load((PACKAGE/'config/competition_route_requester.param.yaml').read_text())
        cls.routes = yaml.safe_load((PACKAGE/'config/kcity_v64_competition_route.yaml').read_text())
        cls.geometry = Geometry(REPO, cls.routes)

    def test_all_five_cases(self):
        expected = [(True, False, False), (False, True, False), (True, True, False),
                    (True, False, True), (True, True, True)]
        for case, row in zip(policy.CASES, expected):
            cfg = dict(self.cfg, route_operation_case=case)
            policy.validate(cfg, self.routes)
            self.assertEqual(tuple(policy.allowed(case, key) for key in
                ('route_a', 'route_b', 'fallback_route')), row)

    def test_operating_default_is_manual_a_with_automatic_b_and_fallback(self):
        self.assertEqual(self.cfg['route_operation_case'], 'route_a_to_b_with_fallback')
        self.assertFalse(self.cfg['auto_request_initial_route'])
        self.assertTrue(policy.allowed(self.cfg['route_operation_case'], 'route_a'))
        self.assertTrue(policy.allowed(self.cfg['route_operation_case'], 'route_b'))
        self.assertTrue(policy.allowed(self.cfg['route_operation_case'], 'fallback_route'))

    def test_route_a_start_tolerance_is_six_metres(self):
        self.assertEqual(self.routes['start']['expected_pose_tolerance_m'], 6.0)

    def test_fail_closed_config(self):
        for case in ('', 'ROUTE_A_ONLY', 'route_a', None, False):
            with self.assertRaises(ValueError):
                policy.validate(dict(self.cfg, route_operation_case=case), self.routes)
        for field, value in [('service_timeout', -1), ('odometry_max_age', float('nan')),
                             ('require_new_trajectory', False), ('never_clear_route', False),
                             ('allow_runtime_case_change', True), ('reroute_max_attempts', -1),
                             ('route_a_refresh_max_speed_mps', 0)]:
            with self.assertRaises(ValueError):
                policy.validate(dict(self.cfg, **{field: value}), self.routes)

    def test_freshness(self):
        self.assertTrue(policy.fresh(10, 9.6, .5))
        for value in (0, 9, 10.1, float('nan'), float('inf')):
            self.assertFalse(policy.fresh(10, value, .5))

    def test_deadline(self):
        r = self.routes
        self.assertFalse(policy.in_window(r, 'route_b', 200178, 7.9))
        self.assertTrue(policy.in_window(r, 'route_b', 200178, 8))
        self.assertTrue(policy.in_window(r, 'route_b', 200348, 1))
        self.assertFalse(policy.in_window(r, 'route_b', 200346, 0))
        self.assertFalse(policy.in_window(r, 'fallback_route', 201446, 7.9))
        self.assertTrue(policy.in_window(r, 'fallback_route', 201446, 8))
        self.assertFalse(policy.in_window(r, 'fallback_route', 200360, 8))
        self.assertFalse(policy.in_window(r, 'fallback_route', 200183, 10))

    def test_identity_sequence_goal(self):
        for key in ('route_a', 'route_b', 'fallback_route'):
            definition = self.routes[key]
            goal = self.routes['poses'][definition['goal_ref']]
            seq = definition['expected_path_lanelet_ids']
            args = [self.routes, key, definition['uuid'], seq, (goal['x'], goal['y']), .5]
            self.assertTrue(policy.route_matches(*args))
            bad = list(args); bad[2] = '00000000-0000-0000-0000-000000000000'
            self.assertFalse(policy.route_matches(*bad))
            bad = list(args); bad[3] = seq[::-1]
            self.assertFalse(policy.route_matches(*bad))
            bad = list(args); bad[4] = (goal['x']+1, goal['y'])
            self.assertFalse(policy.route_matches(*bad))

    def test_geometry_and_hash(self):
        for ref in ('wp_200178', 'wp_200360', 'fallback_cutoff_201446'):
            p = self.routes['poses'][ref]
            lane, arc = self.geometry.locate(p['x'], p['y'], math.radians(p['yaw_deg']))
            self.assertEqual(lane, p['lanelet_id'])
            self.assertAlmostEqual(arc, p['lanelet_s_m'], places=4)
        p = self.routes['start']['map_pose']
        location = self.geometry.locate(p['x'], p['y'], math.radians(p['yaw_deg']))
        self.assertIn(location[0], self.routes['start']['accepted_start_lanelet_ids'])
        self.assertIsNone(self.geometry.locate(999999, 999999, 0))
        bad = copy.deepcopy(self.routes); bad['map']['sha256'] = 'bad'
        with self.assertRaises(ValueError):
            Geometry(REPO, bad)

    def test_moving_retry_exact_suffix_only(self):
        definition = self.routes['route_b']
        seq = definition['expected_path_lanelet_ids']
        goal = self.routes['poses'][definition['goal_ref']]
        args = [self.routes, 'route_b', definition['uuid'], seq[1:], (goal['x'], goal['y']), .5]
        self.assertTrue(policy.route_matches(*args))
        args[3] = seq[seq.index(200346):]
        self.assertFalse(policy.route_matches(*args))
        args[3] = seq[1:4] + seq[5:]
        self.assertFalse(policy.route_matches(*args))

    def test_policy_transitions(self):
        for case in policy.CASES:
            self.assertEqual(policy.next_normal(case, None, 200889, 0, self.routes),
                             None if case == 'route_b_only' else 'route_a')
            self.assertEqual(policy.next_normal(case, 'route_a', 200178, 8, self.routes),
                             'route_b' if case in policy.NORMAL else None)
            self.assertIsNone(policy.next_normal(case, 'route_b', 200360, 8, self.routes))


if __name__ == '__main__':
    unittest.main()
