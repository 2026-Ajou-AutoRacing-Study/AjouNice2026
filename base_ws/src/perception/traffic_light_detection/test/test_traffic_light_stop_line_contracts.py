#!/usr/bin/env python3
"""Optional original-crossing profile tests: no live ROS/map/vehicle mutation."""
import copy
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_traffic_light_route_progress as fixtures

MODULE, GROUPS = fixtures.MODULE, fixtures.GROUPS

N = types.SimpleNamespace


def contract(gid=GROUPS[0], expected=None, crossing=None):
    return dict(regulatory_id=gid, expected_stop_line_xy=expected or [10., -1., 10., 1.],
                crossing_line_xy=crossing or [16., -1., 16., 1.],
                additional_stop_margin_m=.75, max_anchor_error_m=.5)


class StopLineContractsTest(unittest.TestCase):
    def matcher(self, **kwargs):
        helper = fixtures.RouteProgressTest()
        helper.setUp()
        return helper, helper.matcher(**kwargs)

    def apply(self, matcher, contracts, actual):
        matcher._signal_stop_line_contracts = MODULE._stop_line_contracts(contracts)
        matcher.projector = N(forward=lambda p: N(x=p.lat, y=p.lon))
        fake_core = N(GPSPoint=lambda lat, lon, altitude: N(lat=lat, lon=lon))
        with patch.object(MODULE, 'core', fake_core), patch.object(MODULE, '_parse_stop_line_latlon', return_value=actual):
            matcher._apply_stop_line_contracts('unused.osm')

    def test_optional_missing_or_wrong_type_keeps_legacy_behavior(self):
        for value in (None, [], {}, 'bad'):
            with self.subTest(value=value):
                matcher = MODULE.LaneletRouteGroupIdMatcher(None, None, None, signal_stop_line_contracts=value)
                self.assertEqual(matcher._signal_stop_line_contracts, {})
                self.assertEqual(matcher._contract_crossing_positions, {})

    def test_invalid_entries_warn_and_do_not_poison_valid_other_ids(self):
        bad_values = [dict(regulatory_id=True), dict(crossing_line_xy=[0, 1]),
                      dict(expected_stop_line_xy=[0, 0, 0, 0]),
                      dict(crossing_line_xy=[0, 0, 0, 0]),
                      dict(additional_stop_margin_m=-1.), dict(max_anchor_error_m=0.),
                      dict(additional_stop_margin_m=5.001), dict(max_anchor_error_m=2.001),
                      dict(max_anchor_error_m=True), dict(crossing_line_xy=[0, math.nan, 0, 1])]
        for update in bad_values:
            bad = contract(); bad.update(update)
            with self.subTest(update=update):
                good = contract(GROUPS[1])
                self.assertEqual(set(MODULE._stop_line_contracts([bad, good])), {GROUPS[1]})

    def test_numeric_limits_match_cpp_closed_and_open_boundaries(self):
        for margin in (0., 5.):
            for tolerance in (.001, 2.):
                entry=contract();entry.update(additional_stop_margin_m=margin,max_anchor_error_m=tolerance)
                self.assertEqual(set(MODULE._stop_line_contracts([entry])), {GROUPS[0]})
        for margin, tolerance in ((-1e-9,.5),(5.000001,.5),(0.,0.),(0.,2.000001)):
            entry=contract();entry.update(additional_stop_margin_m=margin,max_anchor_error_m=tolerance)
            self.assertEqual(MODULE._stop_line_contracts([entry]), {})

    def test_minimum_segment_lengths_match_cpp_point_one_meter(self):
        for key in ('expected_stop_line_xy','crossing_line_xy'):
            entry=contract();entry[key]=[16. if key=='crossing_line_xy' else 10.,0.,
                                       16. if key=='crossing_line_xy' else 10.,.099]
            self.assertEqual(MODULE._stop_line_contracts([entry]), {})
            entry[key][-1]=.1
            self.assertEqual(set(MODULE._stop_line_contracts([entry])), {GROUPS[0]})

    def test_every_physical_segment_requires_point_zero_five_anchor_separation(self):
        for points in ([10.,-1.,10.,1.], [10.049,-1.,10.049,1.],
                       [16.,-1.,16.,0.,4.,0.]):
            self.assertEqual(MODULE._stop_line_contracts([contract(crossing=points)]), {})
        self.assertEqual(set(MODULE._stop_line_contracts([contract(crossing=[10.05,-1.,10.05,1.])])), {GROUPS[0]})

    def test_duplicate_ids_are_entirely_ignored_not_last_value_wins(self):
        self.assertEqual(MODULE._stop_line_contracts([contract(), contract(), contract()]), {})

    def test_anchor_reverse_endpoints_is_accepted_and_midpoint_updates_bulb(self):
        _, matcher = self.matcher()
        self.apply(matcher, [contract()], {GROUPS[0]: [(10., 1.), (10., -1.)]})
        self.assertEqual(matcher._contract_crossing_positions, {GROUPS[0]: (16., 0.)})
        self.assertEqual(matcher.crossing_info[GROUPS[0]], (16., 0., 1., 0.))
        self.assertEqual(matcher._stopline_positions[GROUPS[0]], (10., 0.))

    def test_anchor_mismatch_and_missing_are_warnings_not_startup_refusal(self):
        for actual in ({}, {GROUPS[0]: [(11., -1.), (11., 1.)]},
                       {GROUPS[0]: [(10., -1.), (10., 0.), (10., 1.)]}):
            _, matcher = self.matcher()
            old = dict(matcher.crossing_info)
            self.apply(matcher, [contract()], actual)
            self.assertTrue(matcher.available)
            self.assertEqual(matcher._contract_crossing_positions, {})
            self.assertEqual(matcher.crossing_info, old)

    def test_profile_never_mutates_caller_data_or_stopline_map(self):
        source = [contract()]; before = copy.deepcopy(source)
        _, matcher = self.matcher(); old = dict(matcher._stopline_positions)
        self.apply(matcher, source, {GROUPS[0]: [(10., -1.), (10., 1.)]})
        self.assertEqual(source, before)
        self.assertEqual(matcher._stopline_positions, old)

    def test_physical_crossing_on_successor_does_not_clamp_to_virtual_lane_endpoint(self):
        helper, matcher = self.matcher(points={1:[(0., 0.), (10., 0.)],
            2:[(10., 0.), (20., 0.)], 3:[(20., 0.), (30., 0.)]},
            mapping={1:GROUPS[0], 3:GROUPS[1]},
            stoplines={GROUPS[0]:(10., 0.), GROUPS[1]:(30., 0.)})
        matcher._contract_crossing_positions = {GROUPS[0]:(16., 0.)}
        matcher._build_route_progress_geometry()
        self.assertAlmostEqual(matcher._route_events[0]['progress_m'], 16.)
        self.assertEqual(helper.move(matcher, 12.), GROUPS[0])
        self.assertEqual(helper.move(matcher, 16.9), GROUPS[0])
        self.assertEqual(helper.move(matcher, 17.1), GROUPS[1])

    def test_missing_profile_retains_existing_virtual_stop_event(self):
        _, matcher = self.matcher()
        self.assertAlmostEqual(matcher._route_events[0]['progress_m'], 10.)

    def test_truncated_route_does_not_release_before_unreachable_physical_line(self):
        helper, matcher = self.matcher(points={1:[(0., 0.), (10., 0.)]},
            mapping={1:GROUPS[0]}, stoplines={GROUPS[0]:(10., 0.)})
        matcher._contract_crossing_positions = {GROUPS[0]:(16., 0.)}
        matcher._build_route_progress_geometry()
        self.assertAlmostEqual(matcher._route_events[0]['progress_m'], 16.)
        self.assertEqual(helper.move(matcher, 10.), GROUPS[0])

    def test_midpoint_is_arc_length_not_vertex_average(self):
        self.assertEqual(MODULE._line_midpoint([(0., 0.), (0., 1.), (0., 10.)]), (0., 5.))

    def test_three_point_physical_line_is_accepted(self):
        _, matcher = self.matcher()
        self.apply(matcher, [contract(crossing=[16.,-1.,16.,0.,16.,3.])],
                   {GROUPS[0]:[(10.,-1.),(10.,1.)]})
        self.assertEqual(matcher._contract_crossing_positions[GROUPS[0]], (16.,1.))

    def test_v94_uses_map_stop_lines_without_per_signal_overrides(self):
        import hashlib
        import yaml
        root = Path(__file__).resolve().parents[5]
        document = yaml.safe_load((root/'base_ws/src/perception/traffic_light_detection/config/traffic_signal_stop_lines.yaml').read_text())
        self.assertEqual(document['signal_stop_line_contracts'], [])
        self.assertEqual(MODULE._stop_line_contracts(document['signal_stop_line_contracts']), {})
        metadata = document['signal_stop_line_contract_metadata']
        self.assertTrue(metadata['map_file'].endswith('lanelet2_map_v94.osm'))
        self.assertEqual(hashlib.sha256((root/metadata['map_file']).read_bytes()).hexdigest(), metadata['map_sha256'])
        self.assertEqual(set(MODULE._parse_stop_line_latlon(root/metadata['map_file'])) & set(GROUPS), set(GROUPS))


if __name__ == '__main__': unittest.main()
