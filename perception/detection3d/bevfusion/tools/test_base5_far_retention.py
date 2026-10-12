import unittest

import numpy as np

from prepare_base5_far_bag_gt import inside_roi
from evaluate_base5_far_retention import (
    band_name, classify_unmatched, full_prepare, range_operating,
    persistent_unmatched, ap_by_range,
    score_policies,
)


def row(gt_x=49.9,pred_x=50.1):
    return dict(token='a',case='test',stamp=1.,native_index=0,
        gt_boxes=[[gt_x,0.,0.,4.,2.,1.,0.]],gt_labels=[0],gt_points=[0],
        gt_assets=['car'],gt_track_keys=['gt1'],
        pred_boxes=[[pred_x,0.,0.,4.,2.,1.,0.]],pred_labels=[0],pred_scores=[.8],
        pred_track_ids=[1],known_gt_boxes=[[gt_x,0.,0.,4.,2.,1.,0.]],known_gt_labels=[0])


class FarRetentionTests(unittest.TestCase):
    def test_explicit_checkpoint_policies_do_not_mutate_defaults(self):
        self.assertEqual(score_policies(None)['local'], (.4, .1, .1))
        self.assertEqual(score_policies([['e9', '.4', '.2', '.12']]), {'e9': (.4, .2, .12)})
        self.assertEqual(score_policies(None)['local'], (.4, .1, .1))

    def test_policy_floor_duplicates_and_nonfinite_are_rejected(self):
        for values in ([['a', '.4', '.09', '.1']], [['a', 'nan', '.1', '.1']],
                       [['a', '.4', '.1', '.1'], ['a', '.4', '.2', '.1']]):
            with self.assertRaises(ValueError):
                score_policies(values)

    def test_model_z_roi_is_bottom_not_geometric_center(self):
        self.assertTrue(inside_roi([90.,29.,3.,4.,2.,3.,0.]))
        self.assertFalse(inside_roi([90.,29.,-2.,4.,2.,3.,0.]))
        self.assertFalse(inside_roi([101.,0.,0.,4.,2.,1.,0.]))

    def test_radial_bands_include_corners_separately(self):
        self.assertEqual(band_name(30.),'30_50m')
        self.assertEqual(band_name(100.),'roi_corner_ge100m')

    def test_band_crossing_match_not_false_positive(self):
        prepared=full_prepare([row()])
        result=range_operating(prepared,0,.4,.1)
        self.assertEqual(result['30_50m']['matched_gt_counts']['2.0'],1)
        self.assertEqual(result['50_80m']['gt'],0)
        self.assertEqual(result['50_80m']['unmatched_predictions_by_center_gate']['2.0'],0)
        self.assertEqual(result['50_80m']['precision_by_center_gate']['2.0'],1.)

    def test_gt_point_count_is_not_visibility_filter(self):
        result=range_operating(full_prepare([row()]),0,.4,.1)['30_50m']
        self.assertEqual(result['gt'],1)
        self.assertEqual(result['ge5point_gt'],0)

    def test_no_gt_ap_is_not_zero_performance(self):
        result=ap_by_range(full_prepare([row()]))
        self.assertIsNone(result['30_50m']['pedestrian']['ap'])
        self.assertEqual(result['30_50m']['pedestrian']['gt'],0)

    def test_spatial_diagnostic_does_not_call_background_nonexistent(self):
        self.assertEqual(classify_unmatched([0,0,0,1,1,1,0],0,[],[])[0],
                         'no_catalog_actor_within4m')
        self.assertEqual(classify_unmatched([0,0,0,1,1,1,0],0,
            [[1.5,0,0,1,1,1,0]],[0])[0],'same_gt_1_to_2m')
        self.assertEqual(classify_unmatched([0,0,0,1,1,1,0],0,
            [[.5,0,0,1,1,1,0]],[2])[0],'near_other_class_gt_lt4m')

    def test_persistent_unmatched_splits_at_excluded_frame(self):
        a=row(10.,60.);b=row(10.,60.)
        b.update(token='b',stamp=1.2,native_index=2)
        result=persistent_unmatched(full_prepare([a,b]),0,.1)
        self.assertEqual(result['episodes'],2)
        self.assertAlmostEqual(result['durations']['maximum'],.1)
        self.assertEqual(result['kinds']['no_catalog_actor_within4m'],2)


if __name__=='__main__':unittest.main()
