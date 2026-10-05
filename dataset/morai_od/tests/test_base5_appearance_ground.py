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
