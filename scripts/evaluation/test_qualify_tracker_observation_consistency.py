"""Native-replay report semantics; no ROS master, simulator or production writes."""
from copy import deepcopy
import argparse
from pathlib import Path
import tempfile
import unittest

from qualify_tracker_observation_consistency import (
    compare_rows, distribution, episode_summary, greedy_geometry, gt_consistency, lifecycle_summary, nested_assignments, parse_variant, run, timing_harness,
)


def track(identity=0, x=0., label=1, confirmed=True):
    return dict(id=identity, center=[x, 0., 1.], velocity=[1., 0.], core_class=label,
                confirmed=confirmed, associated=True, age=3, last_measurement_time=100.)


def row(stamp=100., tracks=None):
    tracks = [track()] if tracks is None else tracks
    return dict(stamp=stamp, tracks=tracks, expected_tracks=len(tracks), before=[])


class ObservationConsistencyTest(unittest.TestCase):
    def test_empty_distribution_does_not_invent_zero_measurement(self):
        self.assertEqual(distribution([]), {'count': 0})

    def test_policy_off_requires_exact_covariance_and_all_state(self):
        baseline = [row()]
        candidate = deepcopy(baseline)
        self.assertTrue(compare_rows(baseline, candidate)['exact_native_state_and_prior_covariance'])
        candidate[0]['before'] = [{'covariance': [1., 0., 1.]}]
        report = compare_rows(baseline, candidate)
        self.assertFalse(report['exact_native_state_and_prior_covariance'])
        self.assertEqual(report['changed_frames'], 1)

    def test_geometric_correspondence_does_not_assume_numeric_identity(self):
        baseline = [row(tracks=[track(1, 0.)])]
        candidate = [row(tracks=[track(9, .1, label=0)])]
        report = compare_rows(baseline, candidate)
        self.assertEqual(report['counts']['geometric_pairs'], 1)
        self.assertEqual(report['counts']['geometric_pair_class_changes'], 1)
        self.assertEqual(report['counts']['geometric_pair_numeric_id_changes'], 1)
        self.assertAlmostEqual(report['geometric_center_delta_m']['maximum'], .1)

    def test_nearest_pairs_are_one_to_one_and_bounded(self):
        pairs, a, b = greedy_geometry([track(0, 0.), track(1, .2)], [track(9, .1), track(10, 20.)])
        self.assertEqual((len(pairs), a, b), (1, 1, 1))

    def test_lifecycle_reports_birth_confirmation_and_actual_class_flip(self):
        rows = [row(100., [track(0, confirmed=False)]), row(100.1), row(100.2, [track(label=0)])]
        report = lifecycle_summary(rows)
        self.assertEqual(report['total_births'], 1)
        self.assertEqual(report['total_confirmations'], 1)
        self.assertEqual(report['representative_class_flip_count'], 1)
        self.assertAlmostEqual(report['confirmation_delay_sec']['maximum'], .1)

    def test_episode_excludes_outside_time_radius_and_tentative_count(self):
        rows = [row(100., [track(0, 0.), track(1, .1, confirmed=False), track(2, 10.)]), row(101.)]
        report = episode_summary(rows, 100., (0., .5, [0., 0.], 1.))
        self.assertEqual(report['unique_native_ids'], [0])
        self.assertEqual(report['confirmed_samples'], 1)

    def test_gt_velocity_never_bridges_original_index_gap(self):
        frames = [dict(stamp=100., native_index=0, scoring_eligible=True,
                       gt=[dict(key='a', class_name='vehicle', center=[0., 0., 1.])]),
                  dict(stamp=100.1, native_index=2, scoring_eligible=True,
                       gt=[dict(key='a', class_name='vehicle', center=[.1, 0., 1.])])]
        report = gt_consistency([row(), row(100.1, [track(x=.1)])], frames)
        self.assertEqual(report['matched_center_error_m']['count'], 2)
        self.assertEqual(report['matched_adjacent_secant_velocity_error_mps']['count'], 0)
        frames[1]['native_index'] = 1
        report = gt_consistency([row(), row(100.1, [track(x=.1)])], frames)
        self.assertEqual(report['matched_adjacent_secant_velocity_error_mps']['count'], 1)
        self.assertAlmostEqual(report['matched_adjacent_secant_velocity_error_mps']['maximum'], 0.)

    def test_nested_native_configuration_is_allowlisted_and_typed(self):
        self.assertEqual(nested_assignments({'class_transition_guard_config.enabled': True}),
                         ['  config.class_transition_guard_config.enabled = true;'])
        with self.assertRaises(ValueError):
            nested_assignments({'class_transition_guard_config.enabled': 'true; malicious();'})
        with self.assertRaises(ValueError):
            nested_assignments({'class_transition_guard_config.required_consecutive': True})

    def test_named_ablation_requires_safe_name_and_json_mapping(self):
        self.assertEqual(parse_variant('shape_only={"shape_consistent_association_ranking":true}'),
                         ('shape_only', {'shape_consistent_association_ranking': True}))
        for invalid in ('../bad={}', 'baseline={}', 'shape_only=[]', 'invalid'):
            with self.assertRaises(argparse.ArgumentTypeError):
                parse_variant(invalid)

    def test_existing_output_rejected_before_inputs_or_build(self):
        with tempfile.TemporaryDirectory() as path:
            with self.assertRaises(FileExistsError):
                run(argparse.Namespace(output=Path(path)))

    def test_no_io_harness_moves_all_track_formatting_after_input_loop(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'perception/detection3d/bevfusion/tools'))
        from build_base5_low_score_probe import HARNESS
        text = timing_harness(HARNESS, ['  config.shape_consistent_association_ranking = true;'])
        loop_end = text.index('    previous = time;\n  }')
        self.assertNotIn('std::cout << "TRACK', text[:loop_end])
        self.assertNotIn('GetTrackResults()', text[:loop_end])
        self.assertEqual(text.count('tracker->RunUpdate(frame);'), 1)
        self.assertIn('"FRAME " << previous', text)


if __name__ == '__main__':
    unittest.main()
