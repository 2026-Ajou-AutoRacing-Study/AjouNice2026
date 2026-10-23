"""Small CPU-only canonical-AP population and boundary regressions."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('_e12_pair_scorer_tests',
    ROOT/'perception/detection3d/bevfusion/scripts/score_base5_execution_pair.py')
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)


def row(token, gt, pred, points=None, pred_labels=None):
    return dict(token=token, gt_boxes=np.asarray(gt,float).reshape(-1,7),
        gt_labels=np.zeros(len(gt),np.int64),gt_points=np.asarray(points if points is not None else [1]*len(gt)),
        pred_boxes=np.asarray(pred,float).reshape(-1,7),
        pred_labels=np.asarray(pred_labels if pred_labels is not None else [0]*len(pred),np.int64),
        pred_scores=np.full(len(pred),.9))


def box(x,y=0.,z=0.): return [x,y,z,1.,1.,1.,0.]


class PairScorerTests(unittest.TestCase):
    def test_zero_point_GT_and_empty_frames_retained(self):
        rows = [row('empty',[],[]),row('point0',[box(10)],[box(10)],[0]),
                row('point1',[box(20)],[box(20)],[1])]
        result = S.ap(S.filtered(rows))
        self.assertEqual(result['frames'],3)
        self.assertEqual(result['empty_GT_frames'],1)
        self.assertEqual(result['classes']['vehicle']['GT'],2)
        self.assertEqual(result['classes']['vehicle']['zero_point_GT'],1)
        self.assertGreater(result['classes']['vehicle']['mean_AP'],.95)

    def test_absent_GT_is_null_but_output_count_is_real(self):
        rows = [row('empty',[],[box(5)],pred_labels=[1])]
        result = S.ap(rows)
        pedestrian = result['classes']['pedestrian']
        self.assertEqual(pedestrian['GT'],0)
        self.assertEqual(pedestrian['predictions'],1)
        self.assertIsNone(pedestrian['mean_AP'])
        self.assertTrue(all(value is None for value in pedestrian['AP_by_gate_m'].values()))
        self.assertIsNone(result['present_classes_mAP'])

    def test_radial_halfopen_and_directional_boundaries(self):
        rows = [row('boundaries',[box(80),box(90),box(100),box(-80),box(84,10)],
                    [box(80),box(90),box(100),box(-80),box(84,10)])]
        self.assertEqual(len(S.filtered(rows,(80,90),'front')[0]['gt_boxes']),2)
        self.assertEqual(len(S.filtered(rows,(90,100),'front')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(rows,(80,90),'rear')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(rows)[0]['gt_boxes']),5)  # ROI x100 inclusive
        zero = [row('xzero',[box(0,20)],[box(0,20)])]
        self.assertEqual(len(S.filtered(zero,(0,30),'front')[0]['gt_boxes']),1)
        self.assertEqual(len(S.filtered(zero,(0,30),'rear')[0]['gt_boxes']),0)

    def test_filters_match_canonical_ROI_and_ranges(self):
        rows = [row('x',[box(10),box(100),box(100.01),box(80,31),box(-85),box(0,z=-3.01)],
                         [box(10),box(100),box(100.01),box(80,31),box(-85),box(0,z=-3.01)])]
        canonical = S.METRIC._filter_center_box_range(rows,S.ROI)
        for band in S.BANDS:
            actual = S.filtered(rows,band)
            expected = S.METRIC._filter_range(canonical,band)
            for key,value in expected[0].items(): self.assertTrue(np.array_equal(actual[0][key],value))

    def test_delta_does_not_require_numeric_identity(self):
        rows = [row('one',[box(10)],[box(10)])]
        old = S.ap(rows)
        new = S.ap([row('one',[box(10)],[box(10.6)])])
        changes = S.delta(old,new)
        self.assertLess(changes['classes']['vehicle']['mean_AP_delta_pp'],0)
        self.assertIsNone(changes['classes']['pedestrian']['mean_AP_delta_pp'])
        self.assertEqual(changes['classes']['vehicle']['GT'],1)


if __name__ == '__main__': unittest.main()
