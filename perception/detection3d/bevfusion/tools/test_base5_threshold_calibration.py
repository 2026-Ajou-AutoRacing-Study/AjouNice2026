import unittest
import json

import numpy as np

from calibrate_base5_thresholds import assignments, prepare, score_curve, roi
from compare_base5_common30 import fixed_matches, scope
from evaluate_base5_threshold_tracking import local_objects


def row():
    return dict(token='a',gt_boxes=[[0,0,0,2,1,1,0],[2,0,0,2,1,1,0]],
                gt_labels=[2,2],gt_points=[5,0],gt_assets=['box','box'],
                gt_track_keys=['one','two'],stamp=1.,native_index=0,
                pred_boxes=[[.1,0,0,2,1,1,0],[.2,0,0,2,1,1,0],[3.5,0,0,2,1,1,0]],
                pred_labels=[2,2,2],pred_scores=[.8,.3,.2])


class ThresholdCalibrationTests(unittest.TestCase):
    def test_randomized_cached_matching_preserves_reference_metric(self):
        rng=np.random.default_rng(17)
        for _ in range(10):
            r=dict(token='random',gt_boxes=rng.normal(size=(5,7)),pred_boxes=rng.normal(size=(12,7)),
                gt_labels=rng.integers(0,3,5),pred_labels=rng.integers(0,3,12),
                gt_points=np.ones(5),pred_scores=np.round(rng.uniform(.1,.9,12),1))
            prepared=prepare([r])[0]
            for label in range(3):
                for tolerance in (1.,2.):
                    pairs=prepared['assignments'][label][tolerance]
                    for threshold in (.1,.3,.5,.8):
                        reference,_=fixed_matches(prepared,label,threshold,tolerance)
                        self.assertEqual(reference,{g for p,g in pairs.items() if prepared['pred_scores'][p]>=threshold})

    def test_actual_ros_world_pose_returns_to_lidar_before_range_scoring(self):
        ego=np.array([[1.,10.,20.,0.,np.pi/2],[1.1,10.,20.,0.,np.pi/2]])
        extrinsic=np.eye(4);extrinsic[0,3]=1.
        boxes=local_objects([dict(center=[10.,23.,0.],size=[4.,2.,1.],yaw=np.pi/2)],1.,ego,extrinsic)
        np.testing.assert_allclose(boxes[0],[2.,0.,0.,4.,2.,1.,0.],atol=1e-12)

    def test_cached_prefix_matching_equals_existing_greedy(self):
        r=prepare([row()])[0]
        for tol in (1.,2.):
            pairs=assignments(r,2,tol)
            for threshold in (.1,.2,.3,.4,.8):
                actual,count=fixed_matches(r,2,threshold,tol)
                cached={g for p,g in pairs.items() if r['pred_scores'][p]>=threshold}
                self.assertEqual(actual,cached)
                self.assertEqual(count,sum(r['pred_scores']>=threshold))

    def test_proximity_diagnostics_do_not_silently_change_tp(self):
        r=score_curve(prepare([row()]),2,.1)
        self.assertEqual((r['gt'],r['tp'],r['fp']),(2,1,2))
        self.assertEqual(r['fp_proximity_diagnostics']['same_class_gt_within_1m'],1)
        self.assertEqual(r['fp_proximity_diagnostics']['same_class_gt_1_to_2m'],1)
        self.assertEqual(r['center2m']['tp'],2)
        json.dumps(r,allow_nan=False)

    def test_strong_returns_are_diagnostic_not_a_gt_deletion(self):
        r=score_curve(prepare([row()]),2,.1)
        self.assertEqual(r['gt'],2)
        self.assertEqual(r['ge5points']['gt'],1)

    def test_ranges_exclude_unannotated_bag_outputs(self):
        r=row();r['pred_boxes'].append([50,0,0,2,1,1,0]);r['pred_labels'].append(2);r['pred_scores'].append(.9)
        selected=scope(r)
        self.assertEqual(len(selected['pred_boxes']),3)
        self.assertEqual(len(roi(r)['pred_boxes']),4)

    def test_temporal_runs_split_on_native_frame_exclusions(self):
        a=row();a['pred_scores']=[0.,0.,0.]
        b=row();b.update(token='b',stamp=1.1,native_index=2);b['pred_scores']=[0.,0.,0.]
        r=score_curve(prepare([a,b]),2,.1,True,.1)
        self.assertEqual(r['miss_runs']['miss_episodes'],4)
        self.assertAlmostEqual(r['miss_runs']['max_seconds'],.1)
        self.assertEqual(r['hit_miss_transitions']['adjacent_pairs'],0)

    def test_no_gt_does_not_invent_a_recall(self):
        r=row();r['gt_boxes']=[];r['gt_labels']=[];r['gt_points']=[];r['gt_assets']=[]
        result=score_curve(prepare([r]),2,.1)
        self.assertIsNone(result['recall'])
        self.assertEqual(result['fp'],3)


if __name__=='__main__':unittest.main()
