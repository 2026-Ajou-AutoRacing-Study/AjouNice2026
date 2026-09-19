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
        cls.routes = yaml.safe_load((PACKAGE/'config/kcity_v84_competition_route.yaml').read_text())
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
        self.assertTrue(self.cfg['auto_request_initial_route'])
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

    def test_route_b_startup_after_trigger(self):
        r = self.routes
        self.assertFalse(policy.in_window(r, 'route_b', 200178, 8, initial=True))
        self.assertFalse(policy.in_window(r, 'route_b', 200348, 8, initial=True))
        self.assertTrue(policy.in_window(r, 'route_b', 200258, 0, initial=True))
        self.assertTrue(policy.in_window(r, 'route_b', 200261, 0, initial=True))
        self.assertTrue(policy.in_window(r, 'route_b', 200181, 0, initial=True))
        self.assertTrue(policy.in_window(r, 'route_b', 200892, 0, initial=True))
        self.assertFalse(policy.in_window(r, 'route_b', 200892, 18.858427, initial=True))
        self.assertFalse(policy.in_window(r, 'route_b', 200220, 0, initial=True))
        self.assertFalse(policy.in_window(r, 'route_b', 200210, 0, initial=True))
        # 200889 is physically reused immediately after the 200892 endpoint.
        self.assertFalse(policy.in_window(r, 'route_b', 200889, 8, initial=True))

    def test_route_a_to_b_reroute_trigger(self):
        r = self.routes
        self.assertFalse(policy.in_window(r, 'route_b', 200258, 7.9))
        self.assertTrue(policy.in_window(r, 'route_b', 200258, 8))
        self.assertFalse(policy.in_window(r, 'route_b', 200348, 1))
        self.assertFalse(policy.in_window(r, 'route_b', 200346, 8))
        self.assertTrue(policy.in_window(r, 'route_b', 200261, 0))
        self.assertTrue(policy.in_window(r, 'route_b', 200311, 0))
        self.assertFalse(policy.in_window(r, 'fallback_route', 201439, 1.9))
        self.assertTrue(policy.in_window(r, 'fallback_route', 201439, 2.0))
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
            self.assertEqual(policy.route_matches(*bad), key == 'route_b')
            bad = list(args); bad[3] = []
            self.assertFalse(policy.route_matches(*bad))
            bad = list(args); bad[4] = (goal['x']+1, goal['y'])
            self.assertFalse(policy.route_matches(*bad))

    def test_route_b_accepts_selector_branch_when_uuid_and_goal_match(self):
        definition = self.routes['route_b']
        goal = self.routes['poses'][definition['goal_ref']]
        observed = [200311, 200968, 201095, 201098, 201096, 201087,
                    201086, 201456, 201438, 201468, 200138, 201444]
        self.assertTrue(policy.route_matches(
            self.routes, 'route_b', definition['uuid'], observed,
            (goal['x'], goal['y']), .5, 'waypoint_path_lanelet_ids'))
        self.assertEqual(policy.preferred_sequence(self.routes, 'route_b', observed), observed)

    def test_waypoint_route_maps_to_configured_preferred_lanes(self):
        for key in ('route_a', 'route_b'):
            initial = self.routes[key]['waypoint_path_lanelet_ids']
            final = self.routes[key]['expected_path_lanelet_ids']
            self.assertEqual(policy.preferred_sequence(self.routes, key, initial), final)
            self.assertEqual(len(initial), len(final))
            self.assertIn(201459, initial)
            self.assertIn(201459, final)
            self.assertIn(200138, final)
            self.assertIn(201444, final)
            self.assertNotIn(200187, final)
        incoming = self.routes['start']['incoming_lanelet_id']
        initial_a = self.routes['route_a']['waypoint_path_lanelet_ids']
        final_a = self.routes['route_a']['expected_path_lanelet_ids']
        self.assertEqual(policy.preferred_sequence(self.routes, 'route_a', [incoming] + initial_a),
                         [incoming] + final_a)
        initial = self.routes['route_b']['waypoint_path_lanelet_ids']
        final = self.routes['route_b']['expected_path_lanelet_ids']
        self.assertEqual(policy.preferred_sequence(self.routes, 'route_b', initial[1:]), final[1:])
        self.assertEqual(policy.preferred_sequence(self.routes, 'route_b', initial[4:]), final[4:])

    def test_v83_highway_lane_change_contract(self):
        """Prefer the leftmost highway corridor through the mandatory bottleneck."""
        for key in ('route_a', 'route_b'):
            initial = self.routes[key]['waypoint_path_lanelet_ids']
            preferred = self.routes[key]['expected_path_lanelet_ids']

            # Keep the same leftmost corridor before and after 201444, so
            # entering it early cannot demand a return to the middle 200187.
            initial_window = initial[initial.index(200138):initial.index(201439) + 1]
            self.assertEqual(
                initial_window,
                [200138, 201444, 201469, 200127, 201473, 201439],
            )
            preferred_window = preferred[preferred.index(201459):preferred.index(201399) + 1]
            self.assertEqual(
                preferred_window,
                [201459, 201436, 201465, 200138, 201444, 201469, 200127,
                 201473, 201439, 200360, 201477, 201479, 201447,
                 201399],
            )

            # Move to the left bottleneck approach early, then enter 200360 immediately
            # after the bottleneck and stay on the lower split pair.
            initial_bottleneck = initial[initial.index(201439):initial.index(201477)]
            self.assertEqual(initial_bottleneck, [201439, 200360])
            initial_split = initial[initial.index(201477):initial.index(201399) + 1]
            self.assertEqual(initial_split, [201477, 201479, 201447, 201399])

            refs = self.routes[key]['waypoint_refs']
            self.assertLess(refs.index('wp_200187_merged'), refs.index('wp_201473'))
            self.assertLess(refs.index('wp_201473'), refs.index('wp_200360'))
            self.assertNotIn('wp_201444_mid', refs)
            self.assertNotIn('wp_200185', refs)
            self.assertEqual(self.routes['poses']['wp_200187_merged']['lanelet_id'], 200187)
            self.assertEqual(self.routes['poses']['wp_200360']['lanelet_id'], 200360)

    def test_route_a_accepts_start_to_trigger_suffixes(self):
        initial = self.routes['route_a']['waypoint_path_lanelet_ids']
        final = self.routes['route_a']['expected_path_lanelet_ids']
        trigger_index = initial.index(200258)
        for index in (1, 2, trigger_index):
            self.assertIn(initial[index:], policy.accepted_sequences(
                self.routes, 'route_a', 'waypoint_path_lanelet_ids'))
            self.assertEqual(policy.preferred_sequence(self.routes, 'route_a', initial[index:]),
                             final[index:])

    def test_geometry_and_hash(self):
        for ref in ('wp_200178', 'wp_200258', 'wp_201473', 'wp_200360',
                    'wp_201447_after_lc', 'fallback_cutoff_201439'):
            p = self.routes['poses'][ref]
            lane, arc = self.geometry.locate(p['x'], p['y'], math.radians(p['yaw_deg']))
            self.assertEqual(lane, p['lanelet_id'])
            self.assertAlmostEqual(arc, p['lanelet_s_m'], places=4)
        # Progress guards locate preferred lanes, not intermediate waypoint lanes.
        p = self.routes['poses']['wp_200137']
        self.assertIsNone(self.geometry.locate(p['x'], p['y'], math.radians(p['yaw_deg'])))
        p = self.routes['start']['map_pose']
        location = self.geometry.locate(p['x'], p['y'], math.radians(p['yaw_deg']))
        self.assertIn(location[0], self.routes['start']['accepted_start_lanelet_ids'])
        self.assertIsNone(self.geometry.locate(999999, 999999, 0))
        bad = copy.deepcopy(self.routes); bad['map']['sha256'] = 'bad'
        with self.assertRaises(ValueError):
            Geometry(REPO, bad)

    def test_moving_route_b_keeps_identity_and_goal_checks(self):
        definition = self.routes['route_b']
        seq = definition['expected_path_lanelet_ids']
        goal = self.routes['poses'][definition['goal_ref']]
        args = [self.routes, 'route_b', definition['uuid'], seq[1:], (goal['x'], goal['y']), .5]
        self.assertTrue(policy.route_matches(*args))
        args[3] = seq[seq.index(200346):]
        self.assertTrue(policy.route_matches(*args))
        args[3] = seq[1:4] + seq[5:]
        self.assertTrue(policy.route_matches(*args))

    def test_route_b_drops_waypoints_behind_startup_pose(self):
        refs = policy.remaining_waypoint_refs(self.routes, 'route_b', 200311)
        self.assertNotIn('wp_200346', refs)
        self.assertNotIn('wp_200315', refs)
        self.assertNotIn('wp_200311', refs)
        self.assertEqual(refs[0], 'wp_201095')

    def test_policy_transitions(self):
        for case in policy.CASES:
            self.assertEqual(policy.next_normal(case, None, 200889, 0, self.routes),
                             None if case == 'route_b_only' else 'route_a')
            self.assertEqual(policy.next_normal(case, None, 200178, 7.9, self.routes),
                             None if case == 'route_b_only' else 'route_a')
            self.assertEqual(policy.next_normal(case, None, 200178, 8, self.routes),
                             None if case == 'route_b_only' else 'route_a')
            for lane in (200337, 200287, 200348, 200346, 200291, 200292,
                         200305, 200841, 200950, 200310, 200308):
                self.assertEqual(policy.next_normal(case, None, lane, 100, self.routes),
                                 None if case == 'route_b_only' else 'route_a')
            self.assertEqual(policy.next_normal(case, None, 200258, 0, self.routes),
                             'route_b' if policy.allowed(case, 'route_b') else None)
            self.assertEqual(policy.next_normal(case, None, 200258, 8, self.routes),
                             'route_b' if policy.allowed(case, 'route_b') else None)
            self.assertEqual(policy.next_normal(case, None, 200892, 0, self.routes),
                             'route_b' if policy.allowed(case, 'route_b') else None)
            self.assertEqual(policy.next_normal(case, None, 200892, 18.858427, self.routes),
                             None if case == 'route_b_only' else 'route_a')
            self.assertIsNone(policy.next_normal(case, None, 200220, 0, self.routes))
            self.assertEqual(policy.next_normal(case, 'route_a', 200258, 8, self.routes),
                             'route_b' if case in policy.NORMAL else None)
            self.assertIsNone(policy.next_normal(case, 'route_a', 200889, 8, self.routes))
            self.assertIsNone(policy.next_normal(case, 'route_a', 200311, 8, self.routes))
            self.assertIsNone(policy.next_normal(case, 'route_b', 200360, 8, self.routes))


if __name__ == '__main__':
    unittest.main()
