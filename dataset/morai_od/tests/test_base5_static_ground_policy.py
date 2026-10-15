import copy
import tempfile
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_static_ground_policy import POLICY, lightweight_static, static_decision


class StaticGroundPolicyTest(unittest.TestCase):
    def meta(self, motions=('static',)):
        return dict(ground_support_check=dict(enabled=True,static_policy=POLICY),
                    runtime_actors=[dict(motion=m) for m in motions])

    def test_explicit_only_all_static_not_calibration(self):
        self.assertFalse(lightweight_static(dict(runtime_actors=[dict(motion='static')])))
        self.assertTrue(lightweight_static(self.meta(('static','standing'))))
        self.assertFalse(lightweight_static(self.meta(('static','walking'))))
        self.assertFalse(lightweight_static(dict(self.meta(),calibration_only=True)))
        meta=self.meta();meta['ground_support_check']['static_policy']='typo'
        with self.assertRaises(ValueError):lightweight_static(meta)

    def test_missing_plane_and_marginal_gap_are_warnings_not_certification(self):
        rows=[dict(uid=1,reason='actor_ground_plane_unverified',verified=False),
              dict(uid=2,reason='actor_annotation_above_ground',plane={'inliers':25},minimum_annotation_ground_gap_m=.19,verified=False),
              dict(uid=3,reason='actor_annotation_deep_below_ground',plane={'inliers':25},minimum_annotation_ground_gap_m=-.35,verified=False)]
        ground=dict(actors=rows);before=copy.deepcopy(ground)
        result=static_decision(ground)
        self.assertEqual(result['review_reasons'],[])
        self.assertEqual(len(result['warnings']),3)
        self.assertFalse(result['extra_ground_captures_allowed'])
        self.assertEqual(ground,before)

    def test_severe_floating_burial_and_bad_evidence_remain_review(self):
        for gap,expected in ((.58,'static_ground_severe_above'),(-.5,'static_ground_severe_below'),
                             (float('nan'),'static_ground_invalid_measurement')):
            result=static_decision(dict(actors=[dict(uid=1,reason='actor_annotation_above_ground',
                plane={'inliers':25},minimum_annotation_ground_gap_m=gap)]))
            self.assertIn(expected,result['review_reasons'])
        self.assertIn('required_actor_ground_annotation_missing',static_decision(dict(
            actors=[dict(uid=7,reason='required_actor_ground_annotation_missing')]))['review_reasons'])

    def test_audit_switch_keeps_ground_record_gt_and_collision_review(self):
        import numpy as np
        from base5_capture_audit import audit_capture
        box=dict(unique_id=1,raw_class='Vehicle',center=np.array([10.,0.,1.]),
                 size=np.array([2.,2.,2.]),rotation_rpy=[0.,0.,0.])
        ground=dict(actors=[dict(uid=1,verified=False,reason='actor_ground_plane_unverified')],
                    maximum_gap_m=.15,review_reasons=['actor_ground_plane_unverified'])
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'LIDAR_5').mkdir()
            np.array([[10.,0.,1.,1.]],np.float32).tofile(root/'LIDAR_5/t_Intensity.bin')
            (root/'LIDAR_5/t_Instance.txt').write_text('Vehicle '+' '.join(['0']*14)+'\n')
            meta=self.meta();meta.update(lidar_to_ego=np.eye(4).tolist(),spec=dict(family='highway',distance_bounds=[5,15]))
            meta['runtime_actors'][0].update(unique_id=1,name='Car',data_id=1,training_class='vehicle')
            with patch('base5_capture_audit.parse_instances',return_value=[box]), \
                 patch('base5_ground_support.ground_review',side_effect=lambda *a,**k:copy.deepcopy(ground)):
                light=audit_capture(root,meta)
                self.assertEqual(light['review_reasons'],[])
                self.assertEqual(len(light['ground_warnings']),1)
                self.assertFalse(light['ground_support']['actors'][0]['verified'])
                self.assertTrue(light['components'][0]['positive_lidar1'])
                for extra in (dict(calibration_only=True),dict(runtime_actors=[dict(meta['runtime_actors'][0],motion='walking')])):
                    strict=audit_capture(root,dict(meta,**extra))
                    self.assertIn('actor_ground_plane_unverified',strict['review_reasons'])
                meta['capture_safety']=dict(collision=dict(observed=True,non_ego_uids=[7]),support=dict(stationary_support_observed=True))
                self.assertIn('physical_collision_during_capture_window',audit_capture(root,meta)['review_reasons'])

    def test_severe_static_review_never_launches_ground_recovery(self):
        import run_base5_quality_loop as loop
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);loop.atomic_json(root/'meta.json',self.meta())
            loop.atomic_json(root/'manifest.json',dict(config={},scenes=[dict(id='a',metadata=str(root/'meta.json'))]))
            row=dict(id='a',status='review',audit=dict(review_reasons=['static_ground_severe_above']))
            with patch.object(loop,'validate_seed',return_value=(row,root,dict(park_verified=True))), \
                 patch.object(loop.subprocess,'Popen') as popen,patch.object(loop,'empty_probe') as probe, \
                 patch.object(loop,'height_retry') as height:
                self.assertEqual(loop.execute(root/'manifest.json',[0],root/'out',seed_capture=root/'seed'),0)
            popen.assert_not_called();probe.assert_not_called();height.assert_not_called()
            self.assertEqual(loop.gen.geo.load_json(root/'out/report.json')['selected'],[])


if __name__=='__main__':unittest.main()
