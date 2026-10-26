#!/usr/bin/env python3
"""Pure ordered-route progress tests: no ROS node, map load or native/Ego input."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


class Duration:
    def __init__(self, value): self.value = value
    def to_sec(self): return self.value


class Stamp:
    clock = 10.
    def __init__(self, value): self.value = value
    def __sub__(self, other): return Duration(self.value-other.value)
    @classmethod
    def now(cls): return cls(cls.clock)


def load_matcher():
    path = Path(__file__).resolve().parents[1]/'scripts/lanelet_route_matcher.py'
    sys.path.insert(0,str(path.parent))
    spec = importlib.util.spec_from_file_location('route_progress_matcher_fake', path)
    result = importlib.util.module_from_spec(spec)
    fake_ros = types.SimpleNamespace(Time=Stamp, logwarn=Mock(), loginfo=Mock())
    with patch.dict(sys.modules, {'rospy': fake_ros}):
        spec.loader.exec_module(result)
    return result


MODULE = load_matcher()
N = types.SimpleNamespace
GROUPS = (201351, 201365, 201290, 201326)


def route(ids, uuid=1):
    return N(uuid=N(uuid=[uuid]), segments=[N(preferred_primitive=N(id=key),
             primitives=[N(id=key)]) for key in ids])


def odometry(x, y=0., yaw=0., speed=0.):
    return N(pose=N(pose=N(position=N(x=x, y=y), orientation=N(x=0., y=0.,
        z=math.sin(yaw/2), w=math.cos(yaw/2)))), twist=N(twist=N(linear=N(x=speed))))


class RouteProgressTest(unittest.TestCase):
    def setUp(self): Stamp.clock = 10.

    def matcher(self, guard=True, points=None, mapping=None, stoplines=None, ids=None, judging=None):
        matcher = MODULE.LaneletRouteGroupIdMatcher(None, None, None,
            route_progress_guard=guard,signal_judging_lines=judging)
        matcher.available = True
        points = points or {i: [(10.*(i-1), 0.), (10.*i, 0.)] for i in range(1, 6)}
        matcher.map = N(laneletLayer={key: N(centerline=[N(x=x, y=y) for x, y in rows])
                                     for key, rows in points.items()})
        matcher.lanelet_group_id = mapping if mapping is not None else dict(zip(range(1, 5), GROUPS))
        matcher._stopline_positions = stoplines if stoplines is not None else {
            gid: (10.*(i+1), 0.) for i, gid in enumerate(GROUPS)}
        matcher.crossing_info = {gid: (100.+i, 0., 1., 0.) for i, gid in enumerate(GROUPS)}
        matcher._route_callback(route(list(points) if ids is None else ids))
        matcher._route_active = True
        return matcher

    def move(self, matcher, x, y=0., yaw=0., dt=.1):
        Stamp.clock += dt
        matcher._odom_callback(odometry(x, y, yaw))
        return matcher.get_group_id()

    def test_original_api_defaults_off_and_keeps_legacy_bulb_halfplane(self):
        matcher = self.matcher(guard=False)
        self.assertEqual(self.move(matcher, 42.), GROUPS[0])
        self.assertFalse(MODULE.LaneletRouteGroupIdMatcher(None, None, None).route_progress_guard)

    def test_stopline_progress_selects_a_b_c_d_then_none_not_pole_halfplane(self):
        matcher = self.matcher()
        self.assertEqual([self.move(matcher, x) for x in (2., 12., 22., 32., 42.)],
                         list(GROUPS)+[None])
        self.assertEqual(matcher._progress_debug['reason'], 'all_route_stoplines_passed')

    def test_one_meter_hysteresis_and_noise_do_not_reselect_passed_head(self):
        matcher = self.matcher()
        self.assertEqual(self.move(matcher, 9.), GROUPS[0])
        self.assertEqual(self.move(matcher, 10.9), GROUPS[0])
        self.assertEqual(self.move(matcher, 11.1), GROUPS[1])
        self.assertEqual(self.move(matcher, 10.5), GROUPS[1])
        self.assertAlmostEqual(matcher._progress_s, 11.1)

    def test_mid_route_start_does_not_first_select_a_or_b(self):
        matcher = self.matcher()
        self.assertEqual(self.move(matcher, 25.), GROUPS[2])

    def test_identical_route_repeat_does_not_reset_progress_but_new_uuid_does(self):
        matcher = self.matcher()
        self.move(matcher, 22.)
        matcher._route_callback(route([1, 2, 3, 4, 5]))
        self.assertAlmostEqual(matcher._progress_s, 22.)
        matcher._route_callback(route([1, 2, 3, 4, 5], uuid=2))
        self.assertIsNone(matcher._progress_s)
        self.assertEqual(matcher.get_group_id(), GROUPS[2])

    def test_cp_pose_jump_backward_reacquires_current_route_not_permanent_blacklist(self):
        matcher = self.matcher()
        self.assertIsNone(self.move(matcher, 42.))
        self.assertEqual(self.move(matcher, 2.), GROUPS[0])
        self.assertAlmostEqual(matcher._progress_s, 2.)

    def test_same_uuid_waypoint_route_replacement_rebases_from_current_pose(self):
        matcher = self.matcher()
        self.assertEqual(self.move(matcher, 25.), GROUPS[2])
        matcher._route_callback(route([3, 4, 5]))
        self.assertIsNone(matcher._progress_s)
        self.assertEqual(matcher.get_group_id(), GROUPS[2])
        self.assertAlmostEqual(matcher._progress_s, 5.)

    def test_route_alternative_lane_is_projected_without_preferred_lane_forcing(self):
        matcher = self.matcher(points={1: [(0., 0.), (10., 0.)],
            2: [(0., 3.), (10., 3.)], 3: [(10., 3.), (20., 3.)]},
            mapping={1: GROUPS[0], 2: GROUPS[0]}, stoplines={GROUPS[0]: (10., 1.5)}, ids=[1, 3])
        message = route([1, 3], uuid=2)
        message.segments[0].primitives.append(N(id=2))
        matcher._route_callback(message)
        self.assertEqual(self.move(matcher, 5., 3.), GROUPS[0])
        self.assertEqual(matcher._progress_debug['lanelet_id'], 2)
        self.assertAlmostEqual(matcher._progress_debug['lateral_m'], 0.)

    def test_unset_and_reroute_reset_while_rerouting_state_keeps_valid_route(self):
        matcher = self.matcher()
        self.move(matcher, 22.)
        states = N(SET=1, REROUTING=2, UNSET=0)
        with patch.dict(sys.modules, {'autoware_planning_msgs.msg': N(RouteState=states)}):
            matcher._route_state_callback(N(state=states.REROUTING))
            self.assertAlmostEqual(matcher._progress_s, 22.)
            matcher._route_state_callback(N(state=states.UNSET))
            self.assertIsNone(matcher.get_group_id())
            self.assertIsNone(matcher._progress_s)
            matcher._route_callback(route([3, 4, 5], uuid=2))
            matcher._route_state_callback(N(state=states.SET))
        self.assertEqual(matcher.get_group_id(), GROUPS[2])

    def test_time_gap_or_backward_clock_clears_old_progress(self):
        for dt in (3., -1.):
            with self.subTest(dt=dt):
                matcher = self.matcher()
                self.move(matcher, 22.)
                self.assertEqual(self.move(matcher, 22., dt=dt), GROUPS[2])
                self.assertAlmostEqual(matcher._progress_s, 22.)

    def test_missing_stopline_or_off_route_is_missing_not_legacy_fallback(self):
        matcher = self.matcher(stoplines={})
        self.assertIsNone(self.move(matcher, 2.))
        self.assertEqual(matcher._progress_debug['reason'], 'route_signal_stopline_missing')
        matcher = self.matcher()
        self.assertIsNone(self.move(matcher, 2., 20.))
        self.assertEqual(matcher._progress_debug['reason'], 'localization_not_on_active_route')

    def test_brief_off_route_rejection_does_not_permanently_skip_current_mission(self):
        matcher=self.matcher()
        self.assertEqual(self.move(matcher,22.,7.5),GROUPS[2])
        # A rejected lateral excursion must not advance route progress.
        self.assertIsNone(self.move(matcher,26.,8.5))
        self.assertAlmostEqual(matcher._progress_s,22.)
        self.assertEqual(self.move(matcher,23.),GROUPS[2])

    def test_reverse_heading_rejection_recovers_without_blacklisting_signal(self):
        matcher=self.matcher()
        self.assertEqual(self.move(matcher,22.),GROUPS[2])
        self.assertIsNone(self.move(matcher,23.,yaw=math.pi))
        self.assertEqual(self.move(matcher,23.),GROUPS[2])

    def test_duplicate_lanelet_on_loop_remains_a_distinct_stopline_occurrence(self):
        points = {1: [(0., 0.), (10., 0.)], 2: [(10., 0.), (20., 0.)],
                  3: [(20., 0.), (20., 10.)], 4: [(20., 10.), (0., 10.)],
                  5: [(0., 10.), (0., 0.)]}
        matcher = self.matcher(points=points, mapping={1: GROUPS[0]},
            stoplines={GROUPS[0]: (10., 0.)}, ids=[1, 2, 3, 4, 5, 1])
        self.assertEqual(len(matcher._route_events), 2)
        self.assertNotEqual(matcher._route_events[0]['progress_m'], matcher._route_events[1]['progress_m'])
        self.assertEqual(self.move(matcher, 2.), GROUPS[0])
        for x, y, yaw in ((8., 0., 0.), (14., 0., 0.), (20., 0., 0.),
                          (20., 5., math.pi/2), (20., 10., math.pi/2),
                          (14., 10., math.pi), (8., 10., math.pi), (2., 10., math.pi),
                          (0., 8., -math.pi/2), (0., 2., -math.pi/2), (2., 0., 0.)):
            self.assertEqual(self.move(matcher, x, y, yaw), GROUPS[0])
        self.assertGreater(matcher._progress_s, matcher._route_events[0]['progress_m'])

    def test_ambiguous_mid_loop_start_does_not_guess_which_stopline_occurrence(self):
        points = {1: [(0., 0.), (30., 0.)], 2: [(30., 0.), (30., 10.)],
                  3: [(30., 10.), (0., 10.)], 4: [(0., 10.), (0., 0.)]}
        matcher = self.matcher(points=points, mapping={1: GROUPS[0]},
            stoplines={GROUPS[0]: (30., 0.)}, ids=[1, 2, 3, 4, 1])
        self.assertIsNone(self.move(matcher, 15.))
        self.assertEqual(matcher._progress_debug['reason'], 'route_progress_occurrence_ambiguous')
        self.assertIsNone(matcher._progress_s)

    def test_strict_guard_configuration_does_not_enable_from_a_truthy_string(self):
        for value in ('true', 1, None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MODULE.LaneletRouteGroupIdMatcher(None, None, None, route_progress_guard=value)
        for name in ('route_progress_hysteresis_m', 'route_progress_max_lateral_m', 'route_progress_pose_jump_m'):
            for value in (0., -1., math.nan, math.inf, '1', True):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    MODULE.LaneletRouteGroupIdMatcher(None, None, None, **{name: value})

    def test_original_line_on_successor_lane_keeps_mission_past_virtual_target(self):
        original=[dict(regulatory_id=GROUPS[0],line_xy=[17.,-2.,17.,2.],approach_xy=[1.,0.])]
        matcher=self.matcher(judging=original)
        event=matcher._route_events[0]
        self.assertEqual(event['virtual_progress_m'],10.)
        self.assertEqual(event['progress_m'],17.)
        self.assertEqual([self.move(matcher,x) for x in (9.,11.,15.,17.9)], [GROUPS[0]]*4)
        self.assertEqual(self.move(matcher,18.1),GROUPS[1])
        self.assertEqual(self.move(matcher,17.5),GROUPS[1])

    def test_moving_virtual_target_does_not_change_original_release_reference(self):
        original=[dict(regulatory_id=GROUPS[0],line_xy=[17.,-2.,17.,2.],approach_xy=[1.,0.])]
        for virtual in (5.,7.,10.):
            with self.subTest(virtual=virtual):
                stops={gid:(10.*(i+1),0.) for i,gid in enumerate(GROUPS)}
                stops[GROUPS[0]]=(virtual,0.)
                matcher=self.matcher(judging=original,stoplines=stops)
                self.assertEqual(matcher._route_events[0]['progress_m'],17.)
                self.assertEqual(self.move(matcher,12.),GROUPS[0])

    def test_original_line_outside_short_route_is_not_prematurely_retired(self):
        original=[dict(regulatory_id=GROUPS[0],line_xy=[17.,-2.,17.,2.],approach_xy=[1.,0.])]
        matcher=self.matcher(points={1:[(0.,0.),(10.,0.)],2:[(10.,0.),(14.,0.)]},
            mapping={1:GROUPS[0]},stoplines={GROUPS[0]:(10.,0.)},judging=original)
        self.assertEqual(self.move(matcher,9.),GROUPS[0])
        self.assertEqual(self.move(matcher,13.9),GROUPS[0])
        self.assertEqual(matcher._progress_debug['judging_line_reason'],'original_judging_line_not_on_route')
        # A replacement route, not a permanent signal blacklist, owns reset.
        matcher._route_callback(route([2],uuid=2))
        self.assertIsNone(matcher.get_group_id())

    def test_invalid_optional_judging_profile_keeps_legacy_release_without_startup_refusal(self):
        for profile in ({},[dict(regulatory_id=GROUPS[0],line_xy=[math.nan,0.],approach_xy=[1.,0.])]):
            with self.subTest(profile=profile):
                matcher=self.matcher(judging=profile)
                self.assertTrue(matcher._judging_line_errors)
                self.assertEqual(matcher._route_events[0]['progress_m'],10.)
                self.assertEqual(self.move(matcher,11.1),GROUPS[1])


if __name__ == '__main__': unittest.main()
