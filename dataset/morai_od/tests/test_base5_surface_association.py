"""A competing far roof is uncertain support, never a mesh/GT correction."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_ground_support import plane_observation, static_surface_association
from base5_static_ground_policy import POLICY, static_decision


def fixture():
    road = dict(policy='mgeo-segment-associated-boundary-v1', usable=True, confidence='road_strip',
        reason=None, height_m=10., nearest_road_distance_m=.01,query_xy_m=[58.,0.],
        source_map_link_sha256='a'*64, source_map_boundary_sha256='b'*64,
        footprint=dict(all_supported=True, map_height_span_m=.02,
            samples=[dict(usable=True,confidence='road_strip',height_m=z) for z in (9.99,10.,10.01)]))
    meta = dict(ground_support_check=dict(static_policy=POLICY),
        capture_runtime_ego=dict(x=0.,y=0.,z=10.,heading=1.347), runtime_actors=[dict(unique_id=8501,
            source_list='objectList',motion='static',position=dict(x=58.,y=0.,z=10.),
            geometric_center_estimate=[58.,0.],map_ground_initializer=road)])
    row = dict(uid=8501,verified=False,reason='actor_annotation_deep_below_ground',
        minimum_annotation_ground_gap_m=-6.,plane=dict(coefficients=[0.,0.,6.],inliers=24,residual_rms_m=.01),
        surface_observation=dict(actor_center_ego_xy_m=[58.,0.],patch_point_count=29,
            native_parent_component_count=1,
            lower_by_one_m_count=3,minimum_signed_plane_residual_m=-6.,native_lowest_corner_ego_z_m=.1))
    return dict(max_range_m=20.,actors=[row]), meta


class StaticSurfaceAssociationTests(unittest.TestCase):
    def test_far_sparse_competing_level_is_warning_and_raw_is_unchanged(self):
        ground,meta = fixture();before = copy.deepcopy((ground,meta))
        result = static_decision(ground,meta)
        self.assertEqual(result['review_reasons'],[])
        self.assertEqual(result['warnings'][0]['reason'],'actor_ground_surface_association_unverified')
        proof = result['warnings'][0]['surface_association']
        self.assertEqual(proof['original_gap_m'],-6.)
        self.assertFalse(proof['mesh_ground_or_contact_certified'])
        self.assertEqual((ground,meta),before)

    def test_no_metadata_keeps_historical_severe_result(self):
        ground,_ = fixture()
        self.assertEqual(static_decision(ground)['review_reasons'],['static_ground_severe_below'])

    def test_dynamic_calibration_and_independent_reference_remain_strict(self):
        for change in ('dynamic','calibration','reference','row_reference','disagreement'):
            with self.subTest(change=change):
                ground,meta=fixture();row=ground['actors'][0]
                if change=='dynamic':meta['runtime_actors'][0]['motion']='walking'
                elif change=='calibration':meta['calibration_only']=True
                elif change=='reference':meta['static_ground_references']=[{'source':'independent'}]
                elif change=='row_reference':row['used_independent_reference']=True
                else:row['reason']='actor_ground_reference_disagreement'
                self.assertIsNone(static_surface_association(row,ground,meta))
                self.assertTrue(static_decision(ground,meta)['review_reasons'])

    def test_large_gap_alone_near_dense_or_single_level_never_relaxes(self):
        for change in ('no_observation','near','dense','one_lower','no_lower','untrusted_road','unsupported',
                       'bad_hash','missing_ego','map_offset','buried_on_map','not_multilevel','bad_plane','compound',
                       'native_compound','native_count_unknown','wrong_map_xy','runtime_xy_mismatch'):
            with self.subTest(change=change):
                ground,meta=fixture();row=ground['actors'][0];obs=row['surface_observation'];actor=meta['runtime_actors'][0]
                if change=='no_observation':row.pop('surface_observation')
                elif change=='near':obs['actor_center_ego_xy_m']=[19.,0.]
                elif change=='dense':obs['patch_point_count']=65
                elif change=='one_lower':obs['lower_by_one_m_count']=1
                elif change=='no_lower':obs['lower_by_one_m_count']=0
                elif change=='untrusted_road':actor['map_ground_initializer']['confidence']='roadside_estimate'
                elif change=='unsupported':actor['map_ground_initializer']['footprint']['all_supported']=False
                elif change=='bad_hash':actor['map_ground_initializer']['source_map_link_sha256']='short'
                elif change=='missing_ego':meta.pop('capture_runtime_ego')
                elif change=='map_offset':actor['position']['z']=11.
                elif change=='buried_on_map':obs['native_lowest_corner_ego_z_m']=-.5
                elif change=='not_multilevel':row['plane']['coefficients']=[0.,0.,1.5]
                elif change=='bad_plane':row['plane']['residual_rms_m']=float('nan')
                elif change=='compound':actor['is_compound']=True
                elif change=='native_compound':obs['native_parent_component_count']=2
                elif change=='native_count_unknown':obs.pop('native_parent_component_count')
                elif change=='wrong_map_xy':actor['map_ground_initializer']['query_xy_m']=[50.,0.]
                elif change=='runtime_xy_mismatch':meta['capture_runtime_ego']['x']=1.
                self.assertEqual(static_decision(ground,meta)['review_reasons'],['static_ground_severe_below'])

    def test_real_road_associated_burial_and_floating_stay_severe(self):
        ground,meta=fixture();row=ground['actors'][0]
        row['plane']['coefficients']=[0.,0.,0.]
        row['surface_observation']['native_lowest_corner_ego_z_m']=-.7
        row['minimum_annotation_ground_gap_m']=-.7
        self.assertEqual(static_decision(ground,meta)['review_reasons'],['static_ground_severe_below'])
        row['reason']='actor_annotation_above_ground';row['minimum_annotation_ground_gap_m']=.7
        self.assertEqual(static_decision(ground,meta)['review_reasons'],['static_ground_severe_above'])

    def test_missing_native_gt_is_never_a_surface_warning(self):
        ground,meta=fixture();ground['actors'].append(dict(uid=8502,reason='required_actor_ground_annotation_missing'))
        self.assertEqual(static_decision(ground,meta)['review_reasons'],['required_actor_ground_annotation_missing'])

    def test_plane_observation_uses_signed_residual_without_mutation(self):
        patch=np.array([[0.,0.,0.],[1.,0.,6.],[0.,1.,6.],[1.,1.,6.]])
        corners=np.array([[.4,.4,.1],[.6,.6,1.]])
        before=patch.copy();result=plane_observation(patch,dict(coefficients=[0.,0.,6.]),corners)
        self.assertEqual(result['lower_by_one_m_count'],1)
        self.assertEqual(result['minimum_signed_plane_residual_m'],-6.)
        self.assertTrue(np.array_equal(patch,before))
        self.assertIsNone(result['native_parent_component_count'])
        for count in (1,2):
            actual_corners=np.tile(np.array([[x,y,z] for x in (-1.,1.) for y in (-1.,1.)
                for z in (-1.,1.)]),(count,1))
            observed=plane_observation(patch,dict(coefficients=[0.,0.,6.]),actual_corners)
            self.assertEqual(observed['native_parent_component_count'],count)

    def test_audit_explicit_metadata_connection_keeps_collision_and_strict_modes(self):
        from base5_capture_audit import audit_capture
        ground,meta=fixture();ground.update(review_reasons=['actor_annotation_deep_below_ground'],maximum_gap_m=.15)
        meta.update(lidar_to_ego=np.eye(4).tolist(),spec=dict(family='obstacle',distance_bounds=[50,80]))
        meta['ground_support_check'].update(enabled=True,max_range_m=20.,required_uids=[8501])
        meta['runtime_actors'][0].update(name='Barrier',data_id=40100052,training_class='obstacle')
        box=dict(unique_id=8501,raw_class='Obstacle',center=np.array([58.,0.,.5]),
            size=np.ones(3),rotation_rpy=[0.,0.,0.])
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'LIDAR_5').mkdir()
            np.array([[58.,0.,.5,1.]],np.float32).tofile(root/'LIDAR_5/t_Intensity.bin')
            (root/'LIDAR_5/t_Instance.txt').write_text('Obstacle '+' '.join(['0']*14)+'\n')
            with patch('base5_capture_audit.parse_instances',return_value=[box]), \
                    patch('base5_ground_support.ground_review',side_effect=lambda *a,**k:copy.deepcopy(ground)):
                result=audit_capture(root,meta)
                self.assertEqual(result['review_reasons'],[])
                self.assertFalse(result['ground_support']['actors'][0]['verified'])
                self.assertEqual(result['ground_support']['actors'][0]['minimum_annotation_ground_gap_m'],-6.)
                meta['capture_safety']=dict(collision=dict(observed=True,non_ego_uids=[7]),
                    support=dict(stationary_support_observed=True))
                self.assertIn('physical_collision_during_capture_window',audit_capture(root,meta)['review_reasons'])
                for change in ('calibration','dynamic'):
                    current=copy.deepcopy(meta)
                    if change=='calibration':current['calibration_only']=True
                    else:current['runtime_actors'][0]['motion']='walking'
                    self.assertIn('actor_annotation_deep_below_ground',audit_capture(root,current)['review_reasons'])


if __name__=='__main__':unittest.main()
