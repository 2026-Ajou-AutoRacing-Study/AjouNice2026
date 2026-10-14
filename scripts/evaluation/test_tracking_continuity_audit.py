from pathlib import Path
import sys
import unittest
import json
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_tracking_continuity import (
    band, evaluate_gt, identity_events, in_roi, label_name, match, output_identity_summary,
    overlap_pairs, point_segment_distance, sample_series, scenario_bindings, stats)
from audit_tracking_continuity_events import compound_case, crossing_case, gaussian_xy_rank, matrix_at
from audit_tracking_continuity_saved_outputs import nonped_states_equal
from audit_tracking_continuity_planner_windows import nearest as nearest_planner, summarize as summarize_planner
from audit_tracking_continuity_packets import (
    cadence as packet_cadence, eligible_pairs, lifecycle as packet_lifecycle, paired_correspondence,
    switch_lifecycle_evidence)
from audit_tracking_continuity_fastpath import ANCHOR, FAST, fast_source
from audit_tracking_continuity_variants import declared_infos_sha, finite_difference_velocity, paired_outputs
from audit_tracking_continuity_variant_review import xy_cost


def box(x=0., y=0., cls='vehicle', identity='a', yaw=0., size=(4., 2., 1.)):
    return dict(center=[x, y, 0.], yaw=yaw, size=list(size), class_name=cls, id=identity)


class TrackingContinuityAuditTest(unittest.TestCase):
    def test_empty_match(self):
        self.assertEqual(match([], [box()], 2.), ([], [], [0]))

    def test_gate_maximizes_valid_cardinality(self):
        # A greedy nearest pair would strand the second observation.
        pairs, missing, extra = match([box(0), box(1)], [box(.4), box(-.8)], 1.)
        self.assertEqual({(i, j) for i, j, _ in pairs}, {(0, 1), (1, 0)})
        self.assertFalse(missing or extra)

    def test_class_agnostic_is_explicit(self):
        self.assertFalse(match([box()], [box(cls='obstacle')], 2.)[0])
        self.assertEqual(len(match([box()], [box(cls='obstacle')], 2., False)[0]), 1)

    def test_no_extrapolation_or_long_bracket(self):
        series = ([1., 2.], [box(), box(10.)])
        for stamp in (0., 1.5, 2.1):
            self.assertIsNone(sample_series(series, stamp))
        self.assertEqual(sample_series(series, 1.)['center'][0], 0.)

    def test_gt_audit_honors_stricter_interpolation_gap(self):
        ego, gt = box(), dict(box(), key='actor', asset='asset')
        output = dict(box(), score=.8)
        case = dict(ego=([1., 1.18], [ego, ego]), gt=[([1., 1.18], [gt, gt])], unresolved=[],
                    frames={'/perception/object_recognition/tracking/objects': [dict(stamp=1.09, objects=[output])]})
        self.assertEqual(evaluate_gt(case, 2.)[0]['evaluated_frames'], 1)
        case['maximum_interpolation_gap_sec'] = .15
        self.assertEqual(evaluate_gt(case, 2.)[0]['excluded_frames'], {'no_bracketed_ego': 1})

    def test_short_bracket_and_yaw_wrap(self):
        import math
        series = ([1., 1.1], [box(yaw=math.radians(179)), box(1., yaw=math.radians(-179))])
        value = sample_series(series, 1.05)
        self.assertAlmostEqual(value['center'][0], .5)
        self.assertAlmostEqual(value['yaw'], math.pi)

    def test_roi_is_ego_aligned(self):
        import math
        ego = box(yaw=math.pi/2)
        self.assertTrue(in_roi(box(0., 90.), ego))
        self.assertFalse(in_roi(box(40., 0.), ego))

    def test_switch_vs_long_gap_reacquisition(self):
        rows = [dict(stamp=s, gt_key='actor', id=i, center_error_m=.1) for s, i in
                [(0., 'a'), (.1, 'a'), (.2, 'b'), (2., 'c')]]
        switches, reacquisitions = identity_events(rows)
        self.assertEqual(len(switches), 1)
        self.assertEqual(switches[0]['previous_id'], 'a')
        self.assertEqual(len(reacquisitions), 1)

    def test_no_switch_merely_from_match_dropout(self):
        rows = [dict(stamp=s, gt_key='actor', id='a', center_error_m=.1) for s in (0., .9)]
        self.assertEqual(identity_events(rows), ([], []))

    def test_same_and_cross_class_overlap_are_distinct(self):
        rows = overlap_pairs([box(), box(.1, identity='b', cls='obstacle'), box(20.)])
        self.assertEqual(len(rows), 1)
        self.assertFalse(rows[0]['same_class'])

    def test_band_boundaries(self):
        self.assertEqual(band(30.), '30_50m')
        self.assertEqual(band(100.), '100_plusm')

    def test_labels_and_empty_stats(self):
        self.assertEqual(label_name(7), 'pedestrian')
        self.assertEqual(label_name(0), 'obstacle')
        self.assertIsNone(stats([])['mean'])

    def test_point_to_segment_not_infinite_line(self):
        self.assertAlmostEqual(point_segment_distance([3., 0.], [0., 0.], [1., 0.]), 2.)

    def test_identity_summary_covers_class_change(self):
        first, second = box(), box(cls='obstacle')
        first['score'] = second['score'] = .6
        result = output_identity_summary([dict(stamp=1., objects=[first]), dict(stamp=1.1, objects=[second])])
        self.assertEqual(len(result['uuid_class_changes']), 1)
        self.assertEqual(len(result['ids']), 1)

    def test_low_iou_duplicate_not_missed(self):
        gt, ego = box(), box()
        gt.update(key='gt', asset='asset')
        objects = [box(-1.4, identity='a'), box(1.4, identity='b')]
        for obj in objects:
            obj['score'] = .7
        case = dict(ego=([1.], [ego]), gt=[([1.], [gt])], unresolved=[],
                    frames={'/perception/object_recognition/tracking/objects': [dict(stamp=1., objects=objects)]})
        report, _ = evaluate_gt(case, 2.)
        self.assertEqual(report['overlap_candidates'], 0)
        self.assertEqual(report['same_class_multiple_uuids_near_one_clear_gt'], 1)

    def test_recorded_tf_interpolation_not_extrapolation(self):
        rows = ([1., 1.1], [([0., 0., 0.], [0., 0., 0., 1.]), ([1., 0., 0.], [0., 0., 0., 1.])])
        self.assertIsNone(matrix_at(rows, .9))
        self.assertAlmostEqual(matrix_at(rows, 1.05)[0, 3], .5)
        self.assertEqual(matrix_at(rows, 2., static=True)[0, 3], 0.)

    def test_compound_diagnostic_preserves_two_people(self):
        rows = compound_case(.7, .35)
        self.assertEqual(len(rows[9]['world_objects']), 1)
        self.assertEqual(len(rows[10]['world_objects']), 2)
        self.assertNotEqual(rows[10]['world_objects'][0]['size'][2], rows[10]['world_objects'][1]['size'][2])

    def test_scenario_binding_rejects_id_reuse(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenario = dict(objectList=[dict(UNIQUEID=2, DataID=40100001, pos=dict(x=0., y=0., z=0.),
                                            scale=dict(x=2., y=3., z=2.))])
            (root/'scenario.json').write_text(json.dumps(scenario))
            bindings, _ = scenario_bindings(root/'sample.bag', {'obstacle:2': [([0., 0., 0.], '')]})
            self.assertEqual(bindings['obstacle:2']['name'], 'CargoBox')
            with self.assertRaises(ValueError):
                scenario_bindings(root/'sample.bag', {'obstacle:2': [([2., 0., 0.], '')]})

    def test_compound_catalog_without_semantic_class_is_not_guessed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scene = dict(pedestrianList=[dict(UNIQUEID=301,DataID=50300018,pos=dict(x=0.,y=0.,z=0.))])
            (root/'scenario.json').write_text(json.dumps(scene))
            bindings,_ = scenario_bindings(root/'sample.bag',{'pedestrian:301':[([0.,0.,0.],'')]})
            self.assertEqual(bindings,{})

    def test_nonped_state_check_ignores_only_counter(self):
        a = [dict(stamp=1., tracks=[dict(id=1, core_class=1, center=[0., 0.])])]
        b = [dict(stamp=1., tracks=[dict(id=8, core_class=1, center=[0., 0.])])]
        self.assertTrue(nonped_states_equal(a, b))
        b[0]['tracks'][0]['center'][0] = .1
        self.assertFalse(nonped_states_equal(a, b))

    def test_gaussian_rank_accounts_for_covariance_volume(self):
        import numpy as np
        residual = [.3, .3]
        fresh = gaussian_xy_rank(residual, np.eye(2)*.1, .16)
        broad = gaussian_xy_rank(residual, np.eye(2)*1000., .16)
        self.assertLess(fresh, broad)
        # This NLL exceeds the old threshold, but it is a ranking value:
        # the generated MatchPairs must keep the original cost as its gate.
        self.assertGreater(broad, 5.)

    def test_gaussian_unit_change_is_common_constant(self):
        import numpy as np
        import math
        shift = gaussian_xy_rank([30., 30.], np.eye(2)*1000., 1600.) - gaussian_xy_rank([.3, .3], np.eye(2)*.1, .16)
        self.assertAlmostEqual(shift, 2*math.log(100.))

    def test_gaussian_rank_rejects_indefinite_covariance(self):
        with self.assertRaises(ValueError):
            gaussian_xy_rank([.1, .1], [[-1., 0.], [0., -1.]], .16)

    def test_crossing_keeps_explicit_two_actor_truth(self):
        frames, truth = crossing_case(True)
        self.assertEqual(len(frames[34]['world_objects']), 1)
        self.assertEqual(len(truth['gt']), 2)
        self.assertNotEqual(truth['gt'][0][1][34]['center'][1], truth['gt'][1][1][34]['center'][1])

    def test_planner_time_pairing_is_bounded(self):
        rows = [dict(stamp=1., value=2.), dict(stamp=1.1, value=3.)]
        self.assertEqual(nearest_planner(rows, 1.04)['value'], 2.)
        self.assertIsNone(nearest_planner(rows, 1.3))

    def test_planner_missing_nearest_samples_not_zero(self):
        rows = [dict(speed=2.), dict(speed=None), dict(speed=0.)]
        result = summarize_planner(rows, ('speed',))
        self.assertEqual(result['speed']['n'], 2)
        self.assertEqual(result['speed_adjacent_abs_change']['n'], 0)

    def test_packet_lifecycle_uses_last_association_not_output_count(self):
        track = dict(id=1, core_class=3, confirmed=True, associated=True, last_measurement_time=1.)
        rows = [dict(stamp=1., tracks=[track]),
                dict(stamp=1.3, tracks=[dict(track, associated=False)]), dict(stamp=1.6, tracks=[])]
        report, evidence = packet_lifecycle(rows)
        self.assertAlmostEqual(report['coasting_last_association_gap_sec']['pedestrian']['mean'], .3)
        self.assertAlmostEqual(evidence['deaths'][0]['last_association_gap_sec'], .6)
        self.assertEqual(report['confirmations_by_class'], {'pedestrian': 1})

    def test_packet_cadence_preserves_subset_holes(self):
        value = packet_cadence([dict(stamp=t) for t in (1., 1.1, 1.8)])
        self.assertEqual(value['gaps_over_threshold_sec']['0.5'], 1)
        self.assertAlmostEqual(value['interframe_sec']['maximum'], .7)

    def test_packet_missing_own_pose_excludes_all_policy_anchor(self):
        ego = ([1., 1.1, 2.], [box(), box(), box()])
        groups = {'good': {'a': dict(stamp=1.02, token='a1'), 'b': dict(stamp=1.04, token='b1')},
                  'bad': {'a': dict(stamp=1.09, token='a2'), 'b': dict(stamp=1.11, token='b2')}}
        kept, excluded = eligible_pairs(groups, ego)
        self.assertEqual(set(kept), {'good'})
        self.assertEqual(excluded[0]['failures'][0]['policy'], 'b')
        self.assertAlmostEqual(excluded[0]['failures'][0]['ego_bracket_gap_sec'], .9)

    def test_packet_paired_errors_use_actor_anchor_not_numeric_uuid(self):
        row = dict(stamp=1., gt_key='actor', id='1', class_name='vehicle', band='30_50m', center_error_m=.5)
        after = dict(row, stamp=1.04, id='99', center_error_m=.3)
        report, rows = paired_correspondence(dict(matched_rows=[row]), dict(matched_rows=[after]),
                                             {1.: 'anchor'}, {1.04: 'anchor'})
        self.assertEqual(report['groups']['all']['paired_actor_observations'], 1)
        self.assertAlmostEqual(rows[0]['error_delta_candidate_minus_baseline_m'], -.2)

    def test_packet_match_switch_can_follow_normal_coast_expiry(self):
        old = dict(id=1, core_class=1, confirmed=True, associated=False,
                   velocity=[5., 0.], last_measurement_time=1.)
        new = dict(old, id=2, associated=True, last_measurement_time=1.6)
        frames = [dict(stamp=1.4, tracks=[old]), dict(stamp=1.55, tracks=[]), dict(stamp=1.6, tracks=[new])]
        _, life = packet_lifecycle(frames)
        config = dict(lifecycle_stationary_speed_threshold_mps=2., confirmed_stationary_track_max_coast_time_sec=1.,
                      confirmed_track_max_coast_time_sec=.5)
        evidence = switch_lifecycle_evidence([dict(stamp=1.6, previous_id='1', id='2', gap_sec=.2)], frames, life, config)[0]
        self.assertTrue(evidence['old_deletion_timeout_consistent'])
        self.assertTrue(evidence['old_deleted_before_new_gt_match'])
        self.assertEqual(evidence['confirmed_old_new_coexistence_stamps'], [])

    def test_cpp_inactive_fastpath_is_pinned_and_active_order_preserved(self):
        original = 'prefix'+ANCHOR+'expression); suffix'
        patched = fast_source(original)
        self.assertEqual(patched, 'prefix'+FAST+'expression); suffix')
        self.assertIn('cost_matrix(meas_idx, track_idx) = 1000.0;', patched)
        with self.assertRaises(ValueError):
            fast_source('different original')

    def test_gt_center_velocity_is_bounded_and_not_extrapolated(self):
        series = ([1., 1.1, 1.2, 2.], [box(0), box(1), box(2), box(50)])
        self.assertAlmostEqual(finite_difference_velocity(series, 1.1)[0], 10.)
        self.assertAlmostEqual(finite_difference_velocity(series, 1.2)[0], 10.)
        self.assertIsNone(finite_difference_velocity(series, 2.))
        self.assertIsNone(finite_difference_velocity(series, 1.15))

    def test_variant_output_pairing_does_not_compare_numeric_id(self):
        a = dict(box(identity='1'), score=.7, velocity=[1., 0.], associated=True, last_measurement_time=1.)
        b = dict(a, id='99', center=[.1, 0., 0.], velocity=[1.2, 0.])
        summary, rows = paired_outputs([dict(stamp=1., objects=[a])], [dict(stamp=1., objects=[b])])
        self.assertEqual(summary['classes']['vehicle']['paired'], 1)
        self.assertAlmostEqual(rows[0]['center_m'], .1)
        self.assertAlmostEqual(rows[0]['velocity_delta_mps'], .2)

    def test_variant_original_infos_lineage_must_be_consistent(self):
        self.assertEqual(declared_infos_sha({'source_lineage': {'original_infos_sha256': 'same'}}), 'same')
        self.assertEqual(declared_infos_sha({'info_sha256': 'same'}), 'same')
        with self.assertRaises(ValueError):
            declared_infos_sha({'info_sha256': 'one', 'source_lineage': {'original_infos_sha256': 'two'}})

    def test_original_cost_diagnostic_caps_and_does_not_mutate(self):
        prior = dict(center=[0., 0.], covariance=[100., 0., 100.], core_class=1)
        before = json.dumps(prior, sort_keys=True)
        value = xy_cost(box(1.), prior)
        self.assertAlmostEqual(value['raw_mahalanobis'], .1)
        self.assertAlmostEqual(value['cost'], .2)
        self.assertTrue(value['eligible'])
        self.assertEqual(json.dumps(prior, sort_keys=True), before)
        self.assertAlmostEqual(xy_cost(box(1.), dict(prior, covariance=[.001, 0., .001]))['cost'], 3.)

    def test_original_cost_diagnostic_ped_class_gate_and_covariance(self):
        prior = dict(center=[0., 0.], covariance=[1., 0., 1.], core_class=3)
        self.assertFalse(xy_cost(box(0., cls='vehicle'), prior)['eligible'])
        self.assertFalse(xy_cost(box(2.01, cls='pedestrian'), prior)['eligible'])
        self.assertTrue(xy_cost(box(1., cls='pedestrian'), prior)['eligible'])
        self.assertIsNone(xy_cost(box(1.), dict(prior, covariance=[0., 0., 0.]))['cost'])


if __name__ == '__main__':
    unittest.main()
