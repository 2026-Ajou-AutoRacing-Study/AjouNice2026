import unittest

import numpy as np

from perception.detection3d.common.evaluation.morai_detection import (
    _orientation_error,
    _scale_error,
    accumulate,
    calculate_ap,
    calculate_tp,
)


def sample(gt_boxes=(), gt_labels=(), pred_boxes=(), pred_labels=(), scores=()):
    return {
        "gt_boxes": np.asarray(gt_boxes, dtype=np.float64).reshape(-1, 7),
        "gt_labels": np.asarray(gt_labels, dtype=np.int64),
        "pred_boxes": np.asarray(pred_boxes, dtype=np.float64).reshape(-1, 7),
        "pred_labels": np.asarray(pred_labels, dtype=np.int64),
        "pred_scores": np.asarray(scores, dtype=np.float64),
    }


BOX = [0.0, 0.0, 0.0, 4.0, 2.0, 1.5, 0.0]


class MoraiDetectionEvalTest(unittest.TestCase):
    def test_perfect_prediction(self):
        metric, positives, predictions = accumulate(
            [sample([BOX], [0], [BOX], [0], [0.9])], 0, 0.5)
        self.assertEqual((positives, predictions), (1, 1))
        self.assertAlmostEqual(calculate_ap(metric), 1.0)
        self.assertAlmostEqual(calculate_tp(metric, "translation_error"), 0.0)
        self.assertAlmostEqual(calculate_tp(metric, "scale_error"), 0.0)
        self.assertAlmostEqual(calculate_tp(metric, "orientation_error"), 0.0)

    def test_duplicate_prediction_is_false_positive(self):
        duplicate = BOX.copy()
        duplicate[0] = 0.1
        metric, positives, predictions = accumulate(
            [sample([BOX], [0], [BOX, duplicate], [0, 0], [0.9, 0.8])],
            0, 0.5)
        self.assertEqual((positives, predictions), (1, 2))
        # The duplicate is a false positive at the same maximum-recall point,
        # so the official interpolation gives a small AP penalty.
        self.assertGreater(calculate_ap(metric), 0.9)
        self.assertLess(calculate_ap(metric), 1.0)

    def test_wrong_class_does_not_match(self):
        metric, positives, predictions = accumulate(
            [sample([BOX], [0], [BOX], [1], [0.9])], 0, 0.5)
        self.assertEqual((positives, predictions), (1, 0))
        self.assertAlmostEqual(calculate_ap(metric), 0.0)

    def test_threshold_is_strict_like_nuscenes(self):
        shifted = BOX.copy()
        shifted[0] = 0.5
        metric, _, _ = accumulate(
            [sample([BOX], [0], [shifted], [0], [0.9])], 0, 0.5)
        self.assertAlmostEqual(calculate_ap(metric), 0.0)

    def test_scale_and_wrapped_yaw_errors(self):
        self.assertAlmostEqual(_scale_error([2, 2, 2], [2, 2, 2]), 0.0)
        self.assertAlmostEqual(
            _orientation_error(-np.pi + 0.1, np.pi - 0.1), 0.2)


if __name__ == "__main__":
    unittest.main()
