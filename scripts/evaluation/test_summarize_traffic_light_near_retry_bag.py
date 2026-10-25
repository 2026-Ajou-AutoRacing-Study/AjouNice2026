#!/usr/bin/env python3
"""Pure diagnostic regression; never opens bags or starts ROS/model processes."""
import json
import unittest

from summarize_traffic_light_near_retry_bag import analyze_records, frame_key

D, C = 201326, 201290


def selection(t, status='matched', group=D, retry=False, accepted=False, source=None):
    stamp = source if source is not None else int((100+t)*1e9)
    rescue = dict(attempted=retry, accepted=accepted)
    if retry:
        rescue['primary_reason'] = 'no_unambiguous_route_detection'
    return dict(kind='selection', t=t, payload=dict(source_stamp_ns=stamp, group_id=group,
        status=status, reason='map_head_association' if status == 'matched' else 'no_unambiguous_route_detection',
        head_id='D_head' if status == 'matched' else None, near_rescue=rescue))


def runtime(selected, callback=.03, **timings):
    payload = selected['payload']
    return dict(kind='runtime', t=selected['t']+.01, payload=dict(
        source_stamp_ns=payload['source_stamp_ns'], frame_decision=dict(group_id=payload['group_id']),
        frame_timing=dict(callback_to_output_sec=callback, **timings)))


def classifier(selected, group=None):
    payload = selected['payload']
    return dict(kind='classifier', t=selected['t']+.02, payload=dict(
        source_stamp_ns=payload['source_stamp_ns'], group_id=payload['group_id'] if group is None else group,
        known=payload['status'] == 'matched', reason='observed_lamps'))


class NearRetryBagSummaryTest(unittest.TestCase):
    def test_exact_source_and_group_join_excludes_duplicates_and_reports_orphans(self):
        first, second = selection(1.), selection(2.)
        rows = [first, second, runtime(first), runtime(second, .04), runtime(second, .09),
                classifier(first), classifier(second, group=C),
                dict(kind='camera', t=.99, source_stamp_ns=first['payload']['source_stamp_ns'])]
        result = analyze_records(rows)
        quality = result['exact_source_and_group_join_quality']
        self.assertEqual(quality['runtime']['duplicate_keys'], 1)
        self.assertEqual(quality['classifier']['valid_keys_without_selection'], 1)
        count = result['groups'][str(D)]['counts']
        self.assertEqual(count['selection_frames'], 2)
        self.assertEqual(count['runtime_exact_unique_joined'], 1)
        self.assertEqual(count['classifier_exact_unique_joined'], 1)
        self.assertEqual(count['camera_header_exact_joined'], 1)
        timing = result['timing_ms']['primary_matched_frames']['callback_to_output_ms']
        self.assertEqual(timing['n'], 1)
        self.assertEqual(timing['p50'], 30.)
        result = analyze_records(rows+[dict(first)])
        self.assertEqual(result['exact_source_and_group_join_quality']['selection']['duplicate_keys'], 1)
        self.assertEqual(result['groups'][str(D)]['counts']['selection_frames'], 1)

    def test_long_missing_runs_split_at_source_gap_without_filling_unseen_frames(self):
        rows = [selection(t, status='missing') for t in (1., 1.5, 2., 3.)]
        rows.append(selection(3.1))
        result = analyze_records(rows)
        missing = result['missing_runs']
        self.assertEqual(missing['total'], 2)
        self.assertEqual(missing['long_count'], 1)
        longest, short = missing['longest_runs']
        self.assertEqual(longest['frames'], 3)
        self.assertEqual(longest['observed_missing_source_span_sec'], 1.)
        self.assertEqual(longest['close_reason'], 'group_change_or_source_gap')
        self.assertNotIn('recovered_source_stamp_ns', longest)
        self.assertEqual(short['frames'], 1)
        self.assertEqual(short['observed_missing_source_span_sec'], 0.)
        self.assertAlmostEqual(short['missing_start_to_recovery_source_sec'], .1)
        self.assertEqual(result['gaps']['selection']['source_clock']['interval_sec']['max'], 1.)

    def test_retry_timing_is_reported_as_subset_not_added_to_callback(self):
        retried = selection(1., retry=True, accepted=True)
        primary = selection(2.)
        stages = dict(decode_sec=.002, detector_and_box_readback_sec=.010,
            selection_including_map_wait_sec=.008, crop_and_classifier_sec=.003,
            policy_and_publish_sec=.001, near_rescue_detector_and_association_sec=.006)
        result = analyze_records([retried, runtime(retried, callback=.024, **stages),
                                  primary, runtime(primary, callback=.018)])
        retry = result['timing_ms']['retry_frames']
        self.assertEqual(retry['callback_to_output_ms']['p50'], 24.)
        self.assertEqual(retry['near_rescue_detector_and_association_ms']['p50'], 6.)
        self.assertLessEqual(retry['near_rescue_detector_and_association_ms']['p95'],
                             retry['selection_including_map_wait_ms']['p95'])
        additive = sum(retry[field]['p50'] for field in ('decode_ms', 'detector_and_box_readback_ms',
            'selection_including_map_wait_ms', 'crop_and_classifier_ms', 'policy_and_publish_ms'))
        self.assertAlmostEqual(retry['callback_to_output_ms']['p50'], additive)
        self.assertEqual(result['timing_ms']['primary_matched_frames']['callback_to_output_ms']['p50'], 18.)
        self.assertEqual(result['groups'][str(D)]['counts']['retry_accepted'], 1)

    def test_d_stop_restart_trace_is_explicit_asof_observation_not_permission(self):
        rows = [dict(kind='route', t=t, group_id=D) for t in (0., 1., 2., 3.)]
        for kind in ('ego', 'localization'):
            for t, speed in ((0., 2.), (1., 0.), (1.5, 0.), (2., 0.), (2.5, 2.)):
                rows.append(dict(kind=kind, t=t, speed_mps=speed, x=t, y=1.,
                                 source_stamp_ns=int((100+t)*1e9)))
        rows.extend([selection(1.4, status='missing'), selection(1.9, retry=True, accepted=True)])
        result = analyze_records(rows)
        observations = result['D_stop_and_restart_observations']
        self.assertIn('bag-time as-of', observations['route_context'])
        self.assertIn('not an exact source join', observations['selection_trace_context'])
        for source in ('ego', 'localization'):
            stopped, = observations['by_pose_source'][source]
            self.assertEqual(stopped['observed_stop_span_sec'], 1.)
            self.assertEqual(stopped['first_restart_speed_t'], 2.5)
            self.assertEqual(stopped['last_stop_to_restart_sec'], .5)
            self.assertEqual(stopped['stopped_retry_accepted'], 1)
            at_two, = [row for row in stopped['trace'] if row['t'] == 2.]
            self.assertEqual(at_two['asof_selection']['t'], 1.9)
            self.assertEqual(at_two['asof_route_group_id'], D)
        self.assertTrue(any('does not infer why' in text for text in result['limitations']))
        self.assertNotIn('PASS', result)

    def test_empty_and_malformed_keys_do_not_authorize_observations(self):
        self.assertIsNone(frame_key({'source_stamp_ns':True, 'group_id':D}))
        self.assertIsNone(frame_key({'source_stamp_ns':1, 'frame_decision':['bad']}, runtime=True))
        rows = [dict(kind='runtime', t=0., payload={'source_stamp_ns':1, 'frame_decision':['bad']}),
                dict(kind='selection', t=0., payload={'source_stamp_ns':0, 'group_id':D})]
        result = analyze_records(rows)
        self.assertEqual(result['groups'], {})
        self.assertEqual(result['missing_runs']['total'], 0)
        self.assertEqual(result['exact_source_and_group_join_quality']['runtime']['invalid_source_or_group'], 1)
        self.assertEqual(analyze_records([])['D_stop_and_restart_observations']['by_pose_source']['ego'], [])
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
