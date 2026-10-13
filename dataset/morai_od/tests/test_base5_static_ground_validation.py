import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_ground_support import ground_review
from prepare_base5_ground_retry import shift_actor
from prepare_base5_empty_ground_probe import target_probe_pose


class StaticGroundTest(unittest.TestCase):
    def test_far_required_actor_not_silently_exempt(self):
        box=dict(unique_id=1,center=np.array([60.,0.,1.]),size=np.array([2.,2.,2.]),rotation_rpy=np.zeros(3))
        legacy=ground_review(np.zeros((0,3)),[box],np.eye(4))
        self.assertFalse(legacy['actors'][0]['required'])
        strict=ground_review(np.zeros((0,3)),[box],np.eye(4),required_uids=[1])
        self.assertIn('actor_ground_plane_unverified',strict['review_reasons'])
        self.assertTrue(strict['actors'][0]['required'])

    def test_missing_required_annotation_is_review(self):
        report=ground_review(np.zeros((0,3)),[],np.eye(4),required_uids=[7])
        self.assertIn('required_actor_ground_annotation_missing',report['review_reasons'])

    def test_vehicle_shift_updates_actual_payload_and_cumulative_metadata(self):
        scene=dict(vehicleList=[dict(UNIQUEID=1,initPosition=dict(pos=dict(x=5.,y=2.,z=28.36,_z='28.360')))])
        meta=dict(runtime_actors=[dict(unique_id=1,source_list='vehicleList',motion='static',position=dict(z=28.))])
        shift_actor(scene,meta,1,-.2);shift_actor(scene,meta,1,-.1)
        self.assertAlmostEqual(scene['vehicleList'][0]['initPosition']['pos']['z'],28.06)
        self.assertEqual(scene['vehicleList'][0]['initPosition']['pos']['x'],5.)
        self.assertAlmostEqual(meta['runtime_actors'][0]['input_ground_z_shift_m'],-.3)

    def test_arbitrary_upward_and_large_shift_refused(self):
        for delta in (.05,-.8,float('nan')):
            with self.assertRaises(ValueError):shift_actor({}, {},1,delta)

    def test_moving_vehicle_not_mutated(self):
        scene=dict(vehicleList=[dict(UNIQUEID=1,initPosition=dict(pos=dict(z=28.)))])
        meta=dict(runtime_actors=[dict(unique_id=1,source_list='vehicleList',motion='moving',position=dict(z=28.))])
        with self.assertRaises(ValueError):shift_actor(scene,meta,1,-.2)
        self.assertEqual(scene['vehicleList'][0]['initPosition']['pos']['z'],28.)

    def test_retry_cannot_accumulate_large_downward_shift(self):
        scene=dict(objectList=[dict(UNIQUEID=1,pos=dict(z=28.))])
        meta=dict(runtime_actors=[dict(unique_id=1,source_list='objectList',position=dict(z=28.),input_ground_z_shift_m=-.6)])
        with self.assertRaises(ValueError):shift_actor(scene,meta,1,-.2)
        self.assertEqual(scene['objectList'][0]['pos']['z'],28.)

    def test_far_actor_with_supported_terrain_is_verified(self):
        x,y=np.meshgrid(np.linspace(55,65,30),np.linspace(-5,5,30))
        points=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        box=dict(unique_id=1,center=np.array([60.,0.,1.]),size=np.array([2.,2.,2.]),rotation_rpy=np.zeros(3))
        report=ground_review(points,[box],np.eye(4),required_uids=[1])
        self.assertTrue(report['actors'][0]['verified'])

    def test_target_probe_refuses_wrong_level_or_short_link(self):
        actor=dict(position=dict(x=10.,y=0.,z=28.))
        for points in ([[0,0,50],[10,0,50]],[[9,0,28],[10,0,28]]):
            with self.assertRaises(ValueError):target_probe_pose({'a':dict(points=points)},actor)

    def test_independent_plane_must_support_whole_footprint(self):
        from base5_ground_probe import surface_at
        x,y=np.meshgrid(np.linspace(-2,2,30),np.linspace(-2,2,30))
        points=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        self.assertIsNotNone(surface_at(points,[0.,0.]))
        self.assertIsNone(surface_at(points,[0.,0.],support_xy=[[0.,0.],[8.,0.]]))

    def test_ineffective_shift_is_not_permission_to_keep_lowering(self):
        from base5_ground_support import height_response_review
        meta=dict(probed_retry_provenance=dict(input_z_corrections=[dict(uid=1,before_ground_gap_m=.3,delta_z_m=-.33)]))
        ground=dict(maximum_gap_m=.15,actors=[dict(uid=1,minimum_annotation_ground_gap_m=.29)])
        self.assertIn('actor_input_z_correction_ineffective',height_response_review(ground,meta)['review_reasons'])
        ground['actors'][0]['minimum_annotation_ground_gap_m']=-.03
        self.assertEqual(height_response_review(ground,meta)['review_reasons'],[])

    def test_saved_pivot_initializer_is_asset_specific_and_not_ground_proof(self):
        from base5_static_height import saved_height_candidate
        ctx=dict(links={'a':dict(points=[[0.,0.,28.],[.5,0.,28.]])})
        def asset(uid,z):return dict(data_id=uid,source_list='vehicleList',template=dict(initPosition=dict(pos=dict(x=0.,y=0.,z=z))))
        a=saved_height_candidate(asset(1,28.31),ctx);b=saved_height_candidate(asset(2,28.42),ctx)
        self.assertTrue(a['usable']);self.assertAlmostEqual(a['pivot_height_candidate_m'],.31)
        self.assertAlmostEqual(b['pivot_height_candidate_m'],.42)
        self.assertIn('not measured mesh contact',a['limitation'])

    def test_saved_initializer_refuses_distant_or_multilevel_map(self):
        from base5_static_height import saved_height_candidate
        asset=dict(data_id=1,source_list='objectList',template=dict(pos=dict(x=0.,y=0.,z=28.)))
        for points in ([[10,0,28],[11,0,28]],[[0,0,28],[0,0,35]]):
            self.assertFalse(saved_height_candidate(asset,dict(links={'a':dict(points=points)}))['usable'])


if __name__=='__main__':unittest.main()
