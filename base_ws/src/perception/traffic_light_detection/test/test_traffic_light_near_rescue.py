#!/usr/bin/env python3
"""Pure near-rescue guard tests: no ROS, Torch, detector, network, or simulator."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from traffic_light_near_rescue import (NearRescueConfig, near_rescue_config,
                                     near_rescue_decision, accept_near_rescue_selection)


GROUPS = (201351, 201365, 201290, 201326)


def missing(**extra):
    return dict(status='missing', reason='no_unambiguous_route_detection', rejected=[], **extra)


def roi(group=201326, head='near-head', depth=10., width=80.):
    return dict(group_id=group, head_id=head, eligible=True, depth_m=depth,
                bbox_xyxy=[100., 20., 100. + width, 40.])


class NearRescueConfigTest(unittest.TestCase):
    def test_default_is_off_and_scale_fixed(self):
        config = near_rescue_config()
        self.assertFalse(config.enabled)
        self.assertEqual(config.imgsz, 640)
        self.assertEqual(config.max_depth_m, 15.)
        self.assertEqual(config.min_projected_width_px, 60.)
        self.assertFalse(near_rescue_decision(missing(), [roi()], 201326)['retry'])

    def test_configuration_is_strict_and_bounded(self):
        for name, values in {
                'enabled': (1, 0, None, 'true', []),
                'imgsz': (True, 640., 320, 960, '640'),
                'max_depth_m': (True, '15', None, math.nan, math.inf, 2.99, 15.01),
                'min_projected_width_px': (False, '60', None, math.nan, math.inf, 31.99, 1280.01)
        }.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    NearRescueConfig(**{name: value})
        for value in ([], 'on', True, {'surprise': 1}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                near_rescue_config(value)
        config = NearRescueConfig(enabled=True, max_depth_m=3., min_projected_width_px=1280.)
        self.assertIs(near_rescue_config(config), config)
        self.assertTrue(near_rescue_config({'enabled': True}).enabled)


class NearRescueDecisionTest(unittest.TestCase):
    def setUp(self):
        self.config = NearRescueConfig(enabled=True)

    def decide(self, selection=None, rois=None, group=201326):
        return near_rescue_decision(missing() if selection is None else selection,
                                    [roi(group)] if rois is None else rois, group, self.config)

    def test_all_four_missions_not_a_d_special_case(self):
        for group in GROUPS:
            with self.subTest(group=group):
                result = self.decide(group=group)
                self.assertTrue(result['retry'])
                self.assertEqual(result['eligible_head_ids'], ['near-head'])
                self.assertEqual(result['imgsz'], 640)
                self.assertEqual(result['group_id'], group)

    def test_boundary_depth_and_width_are_inclusive(self):
        for depth in (3., 15.):
            result = self.decide(rois=[roi(depth=depth, width=60.)])
            self.assertTrue(result['retry'])
        for depth, width in ((2.999, 80), (15.001, 80), (10, 59.999)):
            self.assertFalse(self.decide(rois=[roi(depth=depth, width=width)])['retry'])

    def test_matched_classifier_unknown_legacy_stale_and_invalid_do_not_retry(self):
        selections = [dict(status='matched', reason='map_head_association', head_id='near-head'),
                      dict(status='unknown', reason='low_confidence_bits:red'),
                      dict(status='missing', reason='no_selected_crop'),
                      dict(status='missing', reason='map_packet_missing_or_stale'),
                      missing(mode='legacy_center'), missing(mode=None),
                      missing(projection_reason='localization_stale'),
                      missing(projection_reason='projection_uncertainty_exceeds_bound'),
                      missing(projection_reason='no_visible_route_head')]
        for selection in selections:
            with self.subTest(selection=selection):
                self.assertFalse(self.decide(selection)['retry'])

    def test_other_head_ambiguity_is_not_overridden_by_a_second_scale(self):
        selection = missing()
        selection['rejected'] = [dict(box_index=0, reason='no_route_head_match'),
                                 dict(box_index=1, reason='other_head_identity_ambiguous')]
        result = self.decide(selection)
        self.assertFalse(result['retry'])
        self.assertEqual(result['reason'], 'other_head_identity_ambiguous')

    def test_regular_no_route_head_match_can_retry_but_never_foreign_or_ineligible_roi(self):
        selection = missing(mode='map_association', projection_reason='projected')
        selection['rejected'] = [dict(box_index=0, reason='no_route_head_match')]
        self.assertTrue(self.decide(selection)['retry'])
        foreign = roi(group=201351)
        ineligible = roi(); ineligible['eligible'] = False
        self.assertFalse(self.decide(selection, [foreign, ineligible])['retry'])

    def test_malformed_input_does_not_authorize_retry(self):
        for group in (0, 201397, '201326', 201326., None, True, []):
            with self.subTest(group=group):
                self.assertFalse(self.decide(group=group)['retry'])
        for selection in ([], 'missing', 1):
            self.assertFalse(self.decide(selection)['retry'])
        for rejected in (None, {}, 'ambiguous', [None], [{}]):
            selection = missing(); selection['rejected'] = rejected
            self.assertFalse(self.decide(selection)['retry'])
        for rois in ([], {}, 'rois', [None]):
            self.assertFalse(self.decide(rois=rois)['retry'])

    def test_malformed_roi_or_nonfinite_geometry_is_not_eligible(self):
        variants = [('depth_m', value) for value in (True, None, '10', math.nan, math.inf)]
        variants += [('head_id', value) for value in ('', ' ', ' head ', None, 3)]
        variants += [('group_id', value) for value in ('201326', 201326., True)]
        variants += [('eligible', value) for value in (1, 'true', None)]
        variants += [('bbox_xyxy', value) for value in (
            None, '100 20 180 40', [100, 20, 180], [100, 20, 100, 40],
            [100, 40, 180, 20], [-1, 20, 180, 40], [100, -1, 180, 40],
            [100, 20, math.nan, 40], [100, 20, math.inf, 40], [True, 20, 180, 40])]
        for key, value in variants:
            candidate = roi(); candidate[key] = value
            with self.subTest(key=key, value=value):
                self.assertFalse(self.decide(rois=[candidate])['retry'])
        for key in ('depth_m', 'head_id', 'group_id', 'eligible', 'bbox_xyxy'):
            candidate = roi(); candidate.pop(key)
            self.assertFalse(self.decide(rois=[candidate])['retry'])

    def test_same_program_multiple_heads_allowed_and_inputs_unchanged(self):
        rois = [roi(head='C018'), roi(head='C016'), roi(head='C016'),
                roi(group=201351, head='not-this-route')]
        selection = missing()
        before = copy.deepcopy((selection, rois))
        result = self.decide(selection, rois)
        self.assertEqual(result['eligible_head_ids'], ['C016', 'C018'])
        self.assertEqual((selection, rois), before)

    def test_zero_image_border_coordinate_is_valid_geometry(self):
        candidate = roi(); candidate['bbox_xyxy'] = [0., 0., 60., 20.]
        self.assertTrue(self.decide(rois=[candidate])['retry'])


class NearRescueAcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.roi = roi()
        self.decision = near_rescue_decision(missing(), [self.roi], 201326,
                                             NearRescueConfig(enabled=True))
        self.selection = dict(status='matched', reason='map_head_association',
                              head_id='near-head', roi=self.roi)

    def test_only_a_full_association_to_an_originally_eligible_head_can_be_accepted(self):
        self.assertTrue(accept_near_rescue_selection(self.selection, self.decision))
        for field, value in (('status', 'missing'), ('reason', 'other_head_identity_ambiguous'),
                             ('head_id', 'another-head'), ('roi', None)):
            selection = copy.deepcopy(self.selection); selection[field] = value
            self.assertFalse(accept_near_rescue_selection(selection, self.decision))
        for field, value in (('eligible', False), ('group_id', 201351), ('head_id', 'wrong')):
            selection = copy.deepcopy(self.selection); selection['roi'][field] = value
            self.assertFalse(accept_near_rescue_selection(selection, self.decision))

    def test_no_implicit_authorization_from_disabled_or_malformed_decision(self):
        for decision in ({}, None, [], dict(self.decision, retry=False),
                         dict(self.decision, eligible_head_ids=[]),
                         dict(self.decision, group_id=201397)):
            self.assertFalse(accept_near_rescue_selection(self.selection, decision))
        self.assertFalse(accept_near_rescue_selection(None, self.decision))


if __name__ == '__main__':
    unittest.main()
