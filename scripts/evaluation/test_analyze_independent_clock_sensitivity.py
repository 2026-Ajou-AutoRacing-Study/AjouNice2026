from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import analyze_independent_clock_sensitivity as analysis


def truth(keys, centers, labels=None):
    return dict(token="frame", stamp=10., parent_uids=keys,
                gt_boxes=[[x, 0., 0., 4., 2., 2., 0.] for x in centers],
                gt_labels=labels or [0]*len(keys))


def prediction(centers, labels=None):
    return dict(token="frame", pred_boxes=[[x, 0., 0., 4., 2., 2., 0.] for x in centers],
                pred_labels=labels or [0]*len(centers))


class ClockSensitivityTests(unittest.TestCase):
    def test_header_and_footer_use_identical_predictions_and_local_xy_only(self):
        p = prediction([.5])
        before, after = truth(["car"], [0.]), truth(["car"], [.4])
        report, rows = analysis.paired_errors([p], [before], [after], 1.)
        self.assertEqual(report["counts"]["matched_both_same_prediction"], 1)
        self.assertAlmostEqual(rows[0]["header_error_m"], .5)
        self.assertAlmostEqual(rows[0]["footer_error_m"], .1)
        self.assertAlmostEqual(rows[0]["footer_minus_header_m"], -.4)

    def test_gt_entry_exit_does_not_disappear_into_paired_denominator(self):
        p = prediction([0., 10., 20.])
        report, _ = analysis.paired_errors([p], [truth(["same", "exit"], [0., 10.])],
                                          [truth(["same", "entry"], [0., 20.])], 1.)
        self.assertEqual(report["counts"]["header_only_gt_observations"], 1)
        self.assertEqual(report["counts"]["footer_only_gt_observations"], 1)
        self.assertEqual(report["counts"]["same_actor_frame_observations"], 1)

    def test_match_appearing_disappearing_and_unmatched_both_are_explicit(self):
        p = prediction([0.])
        report, rows = analysis.paired_errors([p], [truth(["a", "b", "c"], [0., 10., 20.])],
                                             [truth(["a", "b", "c"], [10., 0., 20.])], 1.)
        self.assertEqual(report["counts"]["matched_header_only"], 1)
        self.assertEqual(report["counts"]["matched_footer_only"], 1)
        self.assertEqual(report["counts"]["unmatched_both"], 1)
        self.assertEqual(rows, [])

    def test_changed_assigned_prediction_is_not_hidden_in_same_prediction_cohort(self):
        p = prediction([0., 2.])
        report, rows = analysis.paired_errors([p], [truth(["a"], [0.1])], [truth(["a"], [1.9])], 1.)
        self.assertEqual(report["counts"]["matched_both_changed_prediction"], 1)
        self.assertFalse(rows[0]["same_prediction"])
        self.assertEqual(report["same_prediction_distribution"], {})

    def test_classes_and_matching_thresholds_are_preserved(self):
        report = analysis.summarize([prediction([0.], [1])], [truth(["car"], [0.], [0])], 1.)
        self.assertEqual(report["classes"]["vehicle"]["matched"], 0)
        self.assertEqual(report["classes"]["pedestrian"]["unmatched_operating"], 1)
        self.assertIsNone(report["classes"]["pedestrian"]["gt_match_fraction"])

    def test_gate_boundary_is_strict_and_two_meter_recovers_one_meter_miss(self):
        p, g = prediction([1.]), truth(["a"], [0.])
        self.assertEqual(analysis.summarize([p], [g], 1.)["classes"]["vehicle"]["matched"], 0)
        self.assertEqual(analysis.summarize([p], [g], 2.)["classes"]["vehicle"]["matched"], 1)

    def test_duplicate_gt_and_different_frame_are_rejected(self):
        p, g = prediction([0.]), truth(["a", "a"], [0., 0.])
        with self.assertRaises(ValueError):
            analysis.paired_errors([p], [g], [g], 1.)
        p["token"] = "different"
        with self.assertRaises(ValueError):
            analysis.summarize([p], [g], 1.)


if __name__ == "__main__":
    unittest.main()
