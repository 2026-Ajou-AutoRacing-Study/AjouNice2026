import copy
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import generate_base5_independent as gen
from base5_ground_support import fit_plane,ground_review
from prepare_base5_ground_retry import shift_actor


class AppearanceGroundTest(unittest.TestCase):
    def test_zero_xyz_placeholders_cannot_dominate_ground_fit(self):
        x,y=np.meshgrid(np.linspace(-8,8,25),np.linspace(-8,8,25))
        ground=np.c_[x.ravel(),y.ravel(),np.full(x.size,-1.388)]
        cloud=np.r_[np.zeros((10010,3)),ground]
        transform=np.eye(4);transform[2,3]=1.388
        box=dict(unique_id=1,center=np.array([3.5,0.,-.888]),size=np.array([.3,.3,1.]),rotation_rpy=np.zeros(3))
        r=ground_review(cloud,[box],transform)
        self.assertTrue(r['actors'][0]['verified'])
        self.assertAlmostEqual(r['actors'][0]['minimum_annotation_ground_gap_m'],0.,places=8)

    def test_ego_mesh_returns_are_not_ground(self):
        x,y=np.meshgrid(np.linspace(-1,3.5,30),np.linspace(-1,1,30))
        bonnet=np.c_[x.ravel(),y.ravel(),np.ones(x.size)]
        box=dict(unique_id=1,center=np.array([3.5,0.,.5]),size=np.array([.3,.3,1.]),rotation_rpy=np.zeros(3))
        result=ground_review(bonnet,[box],np.eye(4))
        self.assertIn('actor_ground_plane_unverified',result['review_reasons'])
        self.assertNotIn('actor_annotation_deep_below_ground',result['review_reasons'])

    def test_empty_ground_reference_resolves_blind_spot(self):
        from base5_ground_probe import ego_to_world
        x,y=np.meshgrid(np.linspace(-8,8,25),np.linspace(-8,8,25))
        plane=fit_plane(np.c_[x.ravel(),y.ravel(),np.zeros(x.size)])
        reference=dict(uid=1,plane_world=plane)
        box=dict(unique_id=1,center=np.array([3.,0.,.5]),size=np.array([.4,.4,1.]),rotation_rpy=np.zeros(3))
        result=ground_review(np.zeros((0,3)),[box],np.eye(4),references=[reference],runtime_ego=dict(x=0,y=0,z=0,heading=1.347))
        self.assertTrue(result['actors'][0]['verified'])
        self.assertTrue(result['actors'][0]['used_independent_reference'])
        np.testing.assert_allclose(ego_to_world([[1,0,0]],dict(x=2,y=3,z=4,heading=91.347)),[[2,4,4]],atol=1e-9)

    def test_training_roi_is_lidar_frame_rectangular_not_radial(self):
        from build_base5_bevfusion_infos import in_roi
        self.assertTrue(in_roi([99,29,0]))
        self.assertTrue(in_roi([-100,-30,-3]))
        self.assertFalse(in_roi([100,0,0]))
        self.assertFalse(in_roi([0,30,0]))
        self.assertFalse(in_roi([0,0,2]))

    def test_asset_policy_does_not_generalize_unobserved_models(self):
        p=gen.geo.load_json(gen.APPEARANCE_POLICY)
        for name in ('BabyStroller2','Bike3','Scooter1'):
            self.assertEqual(gen.appearance_motion(dict(name=name,source_list='objectList'),'walking',p)[0],'standing')
        self.assertEqual(gen.appearance_motion(dict(name='BabyStroller_unseen',source_list='objectList'),'walking',p)[0],'walking')
        self.assertEqual(gen.appearance_motion(dict(name='Dog1',source_list='pedestrianList'),'walking',p)[0],'walking')

    def test_natural_path_arrives_without_lowering_natural_speed(self):
        pose=dict(x=3.5,y=0.,z=28.,yaw=90.)
        path=gen.natural_target_path(pose,[3.5,0.],dict(x=0.,y=0.),[3,5],'running',dict(ontology_class='Pedestrian'),9.)
        self.assertEqual(path['nominal_speed_mps'],2.2)
        start=np.array([path['points'][0][k] for k in ('x','y','z')])
        target=np.array([pose[k] for k in ('x','y','z')])
        self.assertAlmostEqual(np.linalg.norm(target-start),9*2.2)
        self.assertEqual(path['points'][0]['z'],path['points'][1]['z'])

    def test_plane_and_floating_detection(self):
        x,y=np.meshgrid(np.linspace(-5,5,35),np.linspace(-5,5,35))
        points=np.c_[x.ravel(),y.ravel(),(.01*x-.02*y).ravel()]
        p=fit_plane(points)
        np.testing.assert_allclose(p['coefficients'],[.01,-.02,0],atol=1e-9)
        def box(z):return dict(unique_id=1,center=np.array([0.,0.,z]),size=np.array([.5,.5,1.]),rotation_rpy=np.zeros(3))
        grounded=ground_review(points,[box(.5)],np.eye(4))
        self.assertTrue(grounded['actors'][0]['verified'])
        floating=ground_review(points,[box(.8)],np.eye(4))
        self.assertIn('actor_annotation_above_ground',floating['review_reasons'])
        self.assertLess(floating['actors'][0]['suggested_z_shift_m'],0)

    def test_sparse_line_is_not_certified_ground(self):
        self.assertIsNone(fit_plane(np.c_[np.linspace(-4,4,50),np.zeros(50),np.zeros(50)]))
        self.assertIsNone(fit_plane(np.zeros((8,3))))

    def test_retry_changes_input_and_path_not_xy_or_gt(self):
        scene=dict(pedestrianList=[dict(UNIQUEID=1,pos=dict(x=2.,y=3.,z=28.),initPos=dict(y=28.),standardPos=dict(y=28.))],
            waypointDataList=[dict(pathName='test - 1',waypointData=[dict(pos=dict(x=2,y=28,z=3))])])
        meta=dict(runtime_actors=[dict(unique_id=1,source_list='pedestrianList',position=dict(x=2.,y=3.,z=28.),
            motion_path=dict(points=[dict(x=2.,y=3.,z=28.)],target_position=dict(x=2.,y=3.,z=28.)))])
        shift_actor(scene,meta,1,-.3)
        self.assertEqual(scene['pedestrianList'][0]['pos']['x'],2.)
        self.assertAlmostEqual(scene['pedestrianList'][0]['pos']['z'],27.7)
        self.assertAlmostEqual(scene['waypointDataList'][0]['waypointData'][0]['pos']['y'],27.7)
        with self.assertRaises(ValueError):shift_actor(scene,meta,1,-2.)


if __name__=='__main__':unittest.main()
