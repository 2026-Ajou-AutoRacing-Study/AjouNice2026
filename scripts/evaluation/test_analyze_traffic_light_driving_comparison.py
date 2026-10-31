import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location('driving_comparison', Path(__file__).with_name(
    'analyze_traffic_light_driving_comparison.py'))
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


def frame(t, raw='G', final=None, group=201365, segment=1):
    final = raw if final is None else final
    return dict(source_ns=round(t*1e9), receipt_ns=round((t+.04)*1e9), group=group,
        raw=raw, final=final, raw_permission=m.permission(raw, group),
        final_permission=m.permission(final, group), output_origin='direct_observation',
        selection_status='matched', segment=segment)


def target(t, present, xy=None, group=201365):
    return dict(receipt_ns=round(t*1e9), groups=[group] if present else [],
        target_xy_m=xy or [10., 2.], target_line_distance_m={group: 3.})


def data():
    return dict(bag_start_ns=0, motion=[], control=[], targets=[])


class DrivingComparisonTests(unittest.TestCase):
    def test_actual_runtime_summary_keeps_experimental_vote_contract(self):
        body = dict(stamped_vote_window_size=3, stamped_vote_max_age_sec=.5,
                    source_stamp_ns=123, frame_decision={'per_frame': True})
        self.assertEqual(m.runtime_config(body), dict(
            stamped_vote_window_size=3, stamped_vote_max_age_sec=.5))

    def test_raw_not_post_normalized_and_D_direction(self):
        body = dict(source_stamp_ns=1000000000, frame_decision=dict(group_id=201326,
            observed_tuples=[['green', 'circle', False]], normalized_tuples=[['red', 'circle', False]],
            final_tuples=[['red', 'circle', False], ['green', 'arrow', True]],
            observation_timing=dict(output_origin='held_observation', output_is_held=True)))
        row = m.normalize_runtime(body, 1100000000, 'bag')
        self.assertEqual(row['raw'], 'G')
        self.assertEqual(row['raw_permission'], 'STOP')
        self.assertEqual(row['final_permission'], 'GO')
        self.assertEqual(m.permission('G+R', 201351), 'UNKNOWN')
        self.assertEqual(m.permission('G+G_LEFT', 201290), 'GO')
        self.assertEqual(m.lamp_label([['green', 'arrow', False]]), 'G_RIGHT')
        self.assertEqual(m.lamp_label([]), 'UNKNOWN')
        self.assertEqual(m.lamp_label(None), 'MISSING')

    def test_interior_denominator_excludes_initial_and_trailing_and_all_unseen(self):
        frames = [frame(1, 'UNKNOWN'), frame(1.1, 'G'), frame(1.2, 'UNKNOWN'),
                  frame(1.3, 'G'), frame(1.4, 'UNKNOWN'), frame(2, 'UNKNOWN', segment=2)]
        report = m.summarize_mission(201365, frames, data(), {}, 1.)
        interior = report['interior_raw_unknown']
        self.assertEqual(interior['denominator_frames'], 3)
        self.assertEqual(interior['unknown_frames'], 1)
        self.assertEqual(interior['unknown_islands'], 1)
        self.assertEqual(interior['excluded_frames'], dict(initial_unseen_frames=1,
            trailing_no_recovery_frames=1, all_unseen_segment_frames=1))
        self.assertEqual(len(report['short_islands']), 2)  # Raw and final separately.

    def test_short_islands_not_across_segment_or_unrecovered_ends(self):
        frames = [frame(1, 'G'), frame(1.1, 'UNKNOWN'), frame(1.2, 'G', segment=2),
                  frame(1.3, 'UNKNOWN', segment=2)]
        self.assertEqual(m.summarize_mission(201365, frames, data(), {}, 1.)['short_islands'], [])
        frames = [frame(1, 'R'), frame(1.1, 'G'), frame(1.2, 'R')]
        islands = m.summarize_mission(201365, frames, data(), {}, 1.)['short_islands']
        self.assertEqual(islands[0]['kind'], 'STOP_GO_STOP')
        self.assertAlmostEqual(islands[0]['duration_sec'], .1)
        frames = [frame(1, 'G'), frame(1.1, 'R'), frame(1.2, 'G')]
        self.assertEqual(m.summarize_mission(201365, frames, data(), {}, 1.)['short_islands'][0]['kind'], 'GO_STOP_GO')

    def test_first_GO_motion_only_for_actual_stationary_start(self):
        frames = [frame(1, 'R'), frame(2, 'G'), frame(3, 'G')]
        sample = data()
        sample['motion'] = [dict(receipt_ns=2030000000, speed_mps=0.),
                            dict(receipt_ns=2200000000, speed_mps=.1),
                            dict(receipt_ns=2400000000, speed_mps=.4)]
        sample['control'] = [dict(receipt_ns=2100000000, brake=0., accel=.1)]
        result = m.first_permission_to_motion(frames, sample, 'raw_permission')
        self.assertTrue(result['applicable'])
        self.assertAlmostEqual(result['GO_receipt_to_measured_motion_sec'], .36)
        sample['motion'][0]['speed_mps'] = 3.
        self.assertFalse(m.first_permission_to_motion(frames, sample, 'raw_permission')['applicable'])

    def test_video_source_join_and_simulator_proxy_explicit(self):
        row = dict(source_ns=1000000000, write_ns=1100000000, frame_index=5,
                   pts_sec=.5, repeated=False, exact_runtime_image_match=True)
        index = dict(source_rows=[row], source_keys=[1000000000],
                     receipt_rows=[row], receipt_keys=[1100000000])
        when = m.time_reference(1000000000, 1100000000, 0)
        m.annotate_video_times(when, index, .2)
        self.assertTrue(when['front_video_review']['exact_source_match'])
        self.assertEqual(when['front_video_review']['pts_sec'], .5)
        self.assertAlmostEqual(when['simulator_video_elapsed_sec_proxy'], .9)

    def test_first_insertion_not_reinsertion_goal_and_stale_gap_ignored(self):
        frames = [frame(1, 'R'), frame(4, 'R')]
        targets = [target(1.1, False), target(1.2, True), target(1.3, False),
                   target(1.4, True), target(1.5, False), target(3, True), target(5, True)]
        rows = m.target_reinsertions(targets, 201365, frames)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['receipt_ns'], 1400000000)
        # An unrelated goal zero is never spatially classified as same-line.
        self.assertGreater(m.point_distance([0, 100], [[10, 0], [10, 5]]), 5.)

    def test_all_zero_and_goal_only_not_claimed_as_boundary(self):
        def p(speed, x):
            return NS(longitudinal_velocity_mps=speed, pose=NS(position=NS(x=x, y=0)))
        self.assertIsNone(m.zero_target([p(0, 1), p(0, 2)]))
        self.assertIsNone(m.zero_target([p(3, 1), p(3, 2)]))
        self.assertEqual(m.zero_target([p(0, 1), p(3, 2), p(0, 3), p(0, 4)]), [3, 0])

    def test_all_zero_ambiguous_is_not_stop_target_removal(self):
        frames = [frame(1, 'R'), frame(2, 'R')]
        ambiguous = target(1.2, False)
        ambiguous['target_evidence_unambiguous'] = False
        self.assertEqual(m.target_reinsertions([
            target(1.1, True), ambiguous, target(1.3, True)], 201365, frames), [])

    def test_absence_requires_positive_coverage_across_same_signal_line(self):
        def p(speed, x):
            return NS(longitudinal_velocity_mps=speed, pose=NS(position=NS(x=x, y=2)))
        line = {201365: [[10, 0], [10, 5]]}
        self.assertEqual(m.line_target_states([p(3, 6), p(3, 12)], line, None)[201365],
                         'absent_positive_coverage_across_same_line_zone')
        self.assertEqual(m.line_target_states([p(3, 6), p(3, 8)], line, None)[201365], 'ambiguous')
        self.assertEqual(m.line_target_states([p(0, 6), p(0, 12)], line, [0, 2])[201365], 'ambiguous')
        self.assertEqual(m.line_target_states([p(3, 6), p(0, 12)], line, [6, 2])[201365], 'present')

    def test_missing_kinematics_and_measured_vs_command_separation(self):
        self.assertIsNone(m.measured_motion({}, 1, 'observer'))
        motion = m.measured_motion(dict(velocity=dict(x=2, y=0, z=0),
            acceleration=dict(x=-1, y=0, z=0), accel=.1, brake=.2), 1000000000, 'observer')
        self.assertEqual(motion['speed_mps'], 2.)
        self.assertEqual(motion['acceleration_along_velocity_mps2'], -1.)
        result = m.around([motion], 1000000000, ('speed_mps', 'brake_pedal'))
        self.assertEqual(result['fields']['brake_pedal']['max'], .2)
        self.assertTrue(result['missing_or_sparse'])

    def test_closed_wrapper_required_and_discovery_never_chooses_newest(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'selection.json').write_text('{}')
            with self.assertRaisesRegex(ValueError, 'validator_returned'):
                m.discover(root, [], [])
            (root/'wrapper_events.jsonl').write_text(json.dumps(dict(event='validator_returned', returncode=0))+'\n')
            paths = m.discover(root, [], [])
            self.assertIsNone(paths['bag'])
            self.assertIsNone(paths['campaign'])

    def test_jsonl_bad_line_retained_in_provenance(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'events.jsonl'
            path.write_text('{}\nnot json\n[]\n')
            evidence, warnings = [], []
            self.assertEqual(list(m.jsonl(path, evidence, warnings)), [{}])
            self.assertEqual(evidence[0]['invalid_lines'], 2)
            self.assertEqual(evidence[0]['lines'], 3)
            self.assertEqual(len(evidence[0]['sha256']), 64)

    def test_wrong_observer_drive_never_attached_by_nearest_time(self):
        crossing = dict(crossings=[dict(estimated_source_stamp_ns=1000)])
        m.validate_crossing_window(crossing, 500, 1500)
        with self.assertRaisesRegex(ValueError, 'pairing is unverified'):
            m.validate_crossing_window(crossing, 1001, 2000)


if __name__ == '__main__':
    unittest.main()
