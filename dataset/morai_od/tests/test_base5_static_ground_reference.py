import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_static_ground_reference import transfer_plane,matching_status,actor_payload,prepare,validate_view_transform,apply_reference,reaudit_original
from base5_capture_safety import atomic_json
from base5_capture_audit import rotation_matrix


class StaticRigidReferenceTest(unittest.TestCase):
    def test_full_rotation_preserves_ground_gap_at_long_range(self):
        near=dict(center=np.array([6.,0.,1.]),rotation_rpy=[0,0,0],size=np.array([4.,2.,2.]))
        # Current sensor is pitched; a yaw-only cross-view transform is wrong.
        R=rotation_matrix([.004,.007,.1]);t=np.array([-65.,2.,-.3])
        far=dict(center=R.T@(near['center']-t),rotation_rpy=[0,0,0],size=near['size'])
        inverse=R.T
        far['rotation_rpy']=[np.arctan2(inverse[2,1],inverse[2,2]),np.arcsin(-inverse[2,0]),np.arctan2(inverse[1,0],inverse[0,0])]
        plane=dict(coefficients=[0.,0.,0.],support_polygon_xy=[[3,-3],[9,-3],[9,3],[3,3],[3,-3]])
        transferred,actual_R,actual_t=transfer_plane(near,far,np.eye(4),np.eye(4),plane)
        np.testing.assert_allclose(actual_R,R,atol=1e-12)
        np.testing.assert_allclose(actual_t,t,atol=1e-12)
        surface=R.T@(np.array([6.,0.,0.])-t)
        a,b,c=transferred['coefficients']
        self.assertAlmostEqual(surface[2],a*surface[0]+b*surface[1]+c,places=10)

    def status(self):return dict(type=1,heading=0.,position=dict(x=1.,y=2.,z=3.),
        size=dict(x=4.,y=2.,z=1.5),velocity=dict(x=0.,y=0.,z=0.))

    def test_physical_movement_refused(self):
        a=self.status();b=self.status();b['position']['z']+=.03
        with self.assertRaises(ValueError):matching_status(a,b)

    def test_heading_size_and_motion_changes_refused(self):
        a=self.status()
        for key,field,value in (('size','x',5.),('velocity','x',.1),('position','x',2.)):
            b=self.status();b[key][field]=value
            with self.assertRaises(ValueError):matching_status(a,b)
        b=self.status();b['heading']=1.
        with self.assertRaises(ValueError):matching_status(a,b)

    def test_payload_z_and_scale_changes_invalidate_reference(self):
        a=dict(objectList=[dict(UNIQUEID=1,pos=dict(z=28.),scale=dict(x=1))])
        b=dict(objectList=[dict(UNIQUEID=1,pos=dict(z=28.1),scale=dict(x=1))])
        self.assertNotEqual(actor_payload(a,'objectList',1),actor_payload(b,'objectList',1))

    def test_failed_near_reference_cannot_certify_floating_actor(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);atomic_json(root/'shutdown_safety.json',dict(park_verified=True))
            (root/'ledger.jsonl').write_text('{"status":"review"}\n')
            with self.assertRaisesRegex(ValueError,'not accepted'):prepare(root/'unused',root,root/'output')

    def test_axis_flip_and_wrong_horizontal_transform_refused(self):
        ego=dict(x=0,y=0,z=28,heading=1.347)
        validate_view_transform(np.eye(3),np.zeros(3),ego,ego)
        with self.assertRaises(ValueError):validate_view_transform(rotation_matrix([0,0,np.pi]),np.zeros(3),ego,ego)
        with self.assertRaises(ValueError):validate_view_transform(np.eye(3),np.array([1.,0.,0.]),ego,ego)

    def test_changed_witness_is_fatal_not_ground_approval(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'metadata.json';atomic_json(p,dict(calibration_only=True))
            ground=dict(actors=[dict(uid=1,required=True,verified=False,reason='actor_ground_plane_unverified')])
            meta=dict(static_ground_references=[dict(uid=1,capture_root=str(root),sha256={str(p):'wrong'})])
            with self.assertRaisesRegex(ValueError,'witness changed'):apply_reference(ground,[],root,meta)
            self.assertFalse(ground['actors'][0]['verified'])

    def test_moving_actor_cannot_use_static_witness(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);p=root/'metadata.json';atomic_json(p,dict(calibration_only=True))
            ground=dict(actors=[dict(uid=1,required=True,verified=False,reason='actor_ground_plane_unverified')])
            meta=dict(static_ground_references=[dict(uid=1,capture_root=str(root),sha256={})],
                runtime_actors=[dict(unique_id=1,motion='walking')])
            with self.assertRaisesRegex(ValueError,'static calibration'):apply_reference(ground,[],root,meta)

    def test_reaudit_never_waives_other_failures(self):
        for reasons in (['primary_no_sensor_evidence'],['actor_ground_plane_unverified','physical_collision_during_capture_window']):
            row=dict(status='review',audit=dict(review_reasons=reasons))
            with self.assertRaisesRegex(ValueError,'Only isolated'):reaudit_original(row,Path('unused'),Path('unused'))


if __name__=='__main__':unittest.main()
