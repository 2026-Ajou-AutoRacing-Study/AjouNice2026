#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_base5_offline_regression import (analyze_rows, angle_error_pi,
                                          diagnostic_nms_floor, duplicate_diagnostics)


def row():
    return dict(token='x', gt_boxes=[[20, 0, -1, 2, 1, 1, 0]], gt_labels=[2],
                gt_points=[6], gt_assets=['Box'], gt_asset_keys=['Box'],
                pred_boxes=[[20.1, 0, -1, 2, 1, 1, 0]], pred_labels=[2], pred_scores=[.9])


class RegressionAuditTest(unittest.TestCase):
    def test_basic_tp(self):
        r = analyze_rows([row()])['groups']['all']
        self.assertEqual((r['gt'], r['tp05m'], r['tp1m'], r['tp2m']), (1, 1, 1, 1))
        self.assertEqual(r['miss_partition'], {})

    def test_threshold_miss(self):
        r = row()
        r['pred_scores'] = [.05]
        v = analyze_rows([r])['groups']['all']
        self.assertEqual(v['miss_partition'], {'near_correct_class_below_threshold': 1})

    def test_other_class_proximity(self):
        r = row()
        r['pred_labels'] = [0]
        self.assertEqual(analyze_rows([r])['groups']['all']['miss_partition'],
                         {'eligible_other_class_box_within1m': 1})

    def test_center_gate_loss(self):
        r = row()
        r['pred_boxes'][0][0] = 21.5
        v = analyze_rows([r])['groups']['all']
        self.assertEqual((v['tp1m'], v['tp2m']), (0, 1))
        self.assertEqual(v['miss_partition'], {'eligible_correct_class_center1_to2m': 1})

    def test_no_prediction(self):
        r = row()
        r['pred_boxes'] = r['pred_labels'] = r['pred_scores'] = []
        self.assertEqual(analyze_rows([r])['groups']['all']['miss_partition'],
                         {'no_eligible_same_class_box_within4m': 1})

    def test_unique_matching(self):
        r = row()
        r['gt_boxes'] *= 2
        r['gt_labels'] *= 2
        r['gt_points'] *= 2
        r['gt_assets'] *= 2
        r['gt_asset_keys'] *= 2
        v = analyze_rows([r])['groups']['all']
        self.assertEqual((v['gt'], v['tp1m']), (2, 1))
        self.assertEqual(v['miss_partition'], {'near_eligible_box_taken_by_other_gt': 1})

    def test_yaw_pi_invariance(self):
        self.assertAlmostEqual(angle_error_pi(0., math.pi), 0.)
        self.assertAlmostEqual(angle_error_pi(0., math.pi / 2), math.pi / 2)

    def test_low_floor_is_explicit_not_default(self):
        r = row()
        r['pred_scores'] = [.05]
        self.assertEqual(diagnostic_nms_floor(r, .1)['pred_scores'], [])
        self.assertEqual(diagnostic_nms_floor(r, .025)['pred_scores'], [.05])
        self.assertEqual(r['pred_scores'], [.05])

    def test_near_single_gt_duplicate_diagnostic(self):
        r = row()
        r['pred_boxes'].append([20.12, .02, -1, 2, 1, 1, math.pi / 2])
        r['pred_labels'].append(2)
        r['pred_scores'].append(.6)
        result = duplicate_diagnostics([r])
        self.assertEqual(result['extra_box_observations'], 1)
        self.assertEqual(result['anchored_gt_observations'], 1)
        self.assertLess(result['pair_iou']['p50'], .5)

    def test_close_distinct_gt_not_called_duplicate(self):
        r = row()
        r['gt_boxes'].append([20.3, .1, -1, 1, 1, 1, 0])
        r['gt_labels'].append(2)
        r['gt_points'].append(6)
        r['gt_assets'].append('Box')
        r['gt_asset_keys'].append('Box')
        r['pred_boxes'] += [[20.32, .1, -1, 1, 1, 1, 0],
                            [20.12, .02, -1, 2, 1, 1, math.pi / 2]]
        r['pred_labels'] += [2, 2]
        r['pred_scores'] += [.8, .6]
        self.assertEqual(duplicate_diagnostics([r])['extra_box_observations'], 0)


if __name__ == '__main__':
    unittest.main()
