import unittest

import numpy as np

from compare_base5_common30 import scope
from summarize_base3_cross_validation import unmatched_kind, operating, role, CHECKPOINTS
from prepare_base3_self_return_probe import mask_roof, selection


class CrossValidationTests(unittest.TestCase):
    def test_probe_removes_only_finite_nonzero_near_roof_points(self):
        points=np.array([[0,0,0,0],[0,.2,-.2,255],[1.01,0,-.2,255],
                         [0,.2,-1.4,255],[np.nan,0,-.2,255]],float)
        np.testing.assert_array_equal(mask_roof(points),[False,True,False,False,False])

    def test_probe_selection_is_uniform_not_prediction_dependent(self):
        rows=[{'instances':[{'bbox_label_3d':label}]} for label in range(3) for _ in range(12)]
        rows += [{'instances':[]} for _ in range(12)]
        chosen=selection(rows)
        self.assertEqual(len(chosen),32)
        self.assertEqual(chosen,sorted(set(chosen)))
        self.assertEqual([sum(i//12==label for i in chosen) for label in range(4)],[8]*4)

    def test_frozen_roles_are_checkpoint_hashes_not_file_names(self):
        self.assertEqual(role({'checkpoint_sha256': CHECKPOINTS['E9'], 'name': 'E25'}), 'E9')
        with self.assertRaises(ValueError):
            role({'checkpoint_sha256': 'unknown'})

    def test_omitted_annotation_proximity_is_not_a_true_positive(self):
        row=dict(token='x',gt_boxes=[],gt_labels=[],gt_points=[],gt_assets=[],
                 pred_boxes=[[5.,0.,0.,1.,1.,1.,0.]],pred_labels=[2],pred_scores=[.8])
        missing={'x':[dict(box=[5.,0.,0.,1.,1.,1.,0.],label=2,points=0,id=1)]}
        result=operating([row],(.4,.15,.1),missing)['obstacle']
        self.assertEqual((result['gt'],result['tp'],result['fp']),(0,0,1))
        self.assertEqual(result['unmatched_annotation_proximity'],
                         {'omitted_sparse_same_class_annotation_lt1m':1})
        self.assertIsNone(result['recall'])

    def test_retained_duplicate_has_priority_over_near_omitted_annotation(self):
        boxes=np.array([[5.,0.,0.,1.,1.,1.,0.]])
        labels=np.array([2])
        missing=[dict(box=boxes[0],label=2)]
        self.assertEqual(unmatched_kind(boxes[0],2,boxes,labels,missing),
                         'retained_same_class_gt_lt1m')

    def test_other_class_diagnostic_does_not_remap_label(self):
        boxes=np.array([[5.,0.,0.,1.,1.,1.,0.]])
        self.assertEqual(unmatched_kind(boxes[0],2,boxes,np.array([0]),[]),
                         'retained_other_class_gt_lt2m')

    def test_selected_threshold_changes_only_operating_point(self):
        row=dict(token='x',gt_boxes=[[5.,0.,0.,1.,1.,1.,0.]],gt_labels=[2],
                 gt_points=[5],gt_assets=['box'],pred_boxes=[[5.,0.,0.,1.,1.,1.,0.]],
                 pred_labels=[2],pred_scores=[.11])
        a=operating([row],(.4,.15,.1),{'x':[]})['obstacle']
        b=operating([row],(.3,.2,.12),{'x':[]})['obstacle']
        self.assertEqual(a['gt'],b['gt'])
        self.assertEqual((a['tp'],b['tp']),(1,0))
        self.assertEqual(row['pred_scores'],[.11])

    def test_excluded_roi_corners_are_not_radial30(self):
        row=dict(gt_boxes=[[29.,29.,0.,1.,1.,1.,0.]],gt_labels=[2],
                 pred_boxes=[],pred_labels=[],pred_scores=[])
        self.assertEqual(len(scope(row)['gt_boxes']),0)
        self.assertEqual(len(scope(row,False)['gt_boxes']),1)


if __name__ == '__main__':
    unittest.main()
