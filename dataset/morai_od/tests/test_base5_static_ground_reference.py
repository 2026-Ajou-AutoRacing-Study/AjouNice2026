import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_static_ground_reference import transfer_plane,matching_status,actor_payload,prepare,validate_view_transform,apply_reference,reaudit_original,select_anchor,validate_components,validate_reaudit_output
from base5_capture_safety import atomic_json,sha256
from base5_capture_audit import rotation_matrix
from prepare_base5_static_height_reference import prepare as nearby_reference


class StaticRigidReferenceTest(unittest.TestCase):
    def test_secondary_reaudit_keeps_prior_result_as_proof_not_sensor_raw(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source';source.mkdir()
            missing=dict(review_reasons=['actor_ground_plane_unverified'])
            accepted=dict(review_reasons=[])
            atomic_json(source/'audit.json',missing);atomic_json(source/'metadata.json',{})
            atomic_json(source/'scenario.json',{})
            atomic_json(source/'telemetry.json',dict(objects_before={},objects_after={},ego_before={}))
            atomic_json(root/'request.json',dict(static_ground_references=[1]))
            digest=sha256(source/'scenario.json')
            atomic_json(root/'manifest.json',dict(scenes=[dict(id='one',metadata=str(root/'request.json'),scenario_sha256=digest)]))
            row=dict(id='raw',status='review',audit=missing,capture_root=str(source),scenario_sha256=digest)
            def audit(_,meta):return missing if len(meta['static_ground_references'])==1 else accepted
            with patch('base5_capture_audit.audit_capture',side_effect=audit):
                first=reaudit_original(row,root/'manifest.json',root/'first')
                first_result_sha=sha256(root/'first'/'reaudit_result.json')
                atomic_json(root/'request.json',dict(static_ground_references=[1,2]))
                second=reaudit_original(first,root/'manifest.json',root/'second')
                self.assertEqual(second['status'],'accepted_candidate')
                self.assertEqual(sha256(root/'first'/'reaudit_result.json'),first_result_sha)
                proof=json.loads((root/'second'/'metadata.json').read_text())['static_ground_reaudit']
                self.assertNotIn('reaudit_result.json',proof['raw_sha256'])
                self.assertEqual(validate_reaudit_output(root/'second'),second)
                atomic_json(root/'first'/'reaudit_result.json',dict(first,status='accepted_candidate'))
                with self.assertRaisesRegex(RuntimeError,'status'):
                    validate_reaudit_output(root/'second')

    def test_composite_anchor_and_correspondence_do_not_depend_on_row_order(self):
        import copy
        boxes=[dict(raw_class=c,center=np.array(p,float),size=np.array(s,float),rotation_rpy=[0,0,0])
            for c,p,s in [('Obstacle',[6,0,.5],[.8,.6,1]),('Pedestrian',[5,0,.8],[.4,.4,1.6]),
                ('Pedestrian',[6,0,.7],[.3,.2,.4])]]
        self.assertEqual(select_anchor(boxes,dict(name='BabyStroller1'))['raw_class'],'Obstacle')
        self.assertEqual(len(validate_components(boxes,list(reversed(boxes)),np.eye(4),np.eye(4),np.eye(3),np.zeros(3))),3)
        changed=copy.deepcopy(boxes);changed[1]['center'][0]+=.05
        with self.assertRaisesRegex(ValueError,'correspondence'):validate_components(boxes,changed,np.eye(4),np.eye(4),np.eye(3),np.zeros(3))
        with self.assertRaisesRegex(ValueError,'count'):validate_components(boxes,boxes[:2],np.eye(4),np.eye(4),np.eye(3),np.zeros(3))
        with self.assertRaisesRegex(ValueError,'anchor'):select_anchor(boxes,dict(name='Ped_Pet1'))
        with self.assertRaisesRegex(ValueError,'anchor'):select_anchor(boxes+[boxes[0]],dict(name='BabyStroller1'))
    def test_existing_reaudit_compares_json_values_and_halts_on_real_difference(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source';output=root/'out';source.mkdir();output.mkdir()
            audit=dict(review_reasons=['actor_ground_plane_unverified'])
            atomic_json(source/'audit.json',audit);atomic_json(source/'scenario.json',{})
            atomic_json(source/'metadata.json',{})
            atomic_json(source/'telemetry.json',dict(objects_before={},objects_after={},ego_before={}))
            atomic_json(root/'request.json',dict(static_ground_references=[]))
            digest=sha256(source/'scenario.json')
            atomic_json(root/'manifest.json',dict(scenes=[dict(metadata=str(root/'request.json'),scenario_sha256=digest)]))
            result=dict(raw_source_capture=str(source),scenario_sha256=digest,audit=dict(polygon=[[1.,2.]]))
            atomic_json(output/'reaudit_result.json',result)
            atomic_json(output/'metadata.json',dict(static_ground_references=[],static_ground_reaudit=dict(raw_sha256={})))
            row=dict(status='review',audit=audit,capture_root=str(source),scenario_sha256=digest)
            with patch('base5_capture_audit.audit_capture',return_value=dict(polygon=[(1.,2.)])):
                self.assertEqual(reaudit_original(row,root/'manifest.json',output),result)
            with patch('base5_capture_audit.audit_capture',return_value=dict(polygon=[(1.,2.01)])):
                with self.assertRaisesRegex(RuntimeError,'verification changed'):
                    reaudit_original(row,root/'manifest.json',output)

    def test_secondary_calibration_keeps_exact_target_and_excludes_other_actors(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);scene=dict(egoVehicle={},objectList=[dict(UNIQUEID=1,pos=dict(z=28))],
                vehicleList=[dict(UNIQUEID=2,pos=dict(z=29))])
            actors=[dict(unique_id=u,source_list=c,motion='static',geometric_center_estimate=[u*10,0])
                for u,c in ((1,'objectList'),(2,'vehicleList'))]
            meta=dict(runtime_actors=actors,spec={},ground_support_check=dict(required_uids=[1,2]))
            atomic_json(root/'scene.json',scene);atomic_json(root/'meta.json',meta)
            atomic_json(root/'manifest.json',dict(scenes=[dict(id='test',scenario=str(root/'scene.json'),
                metadata=str(root/'meta.json'),scenario_sha256='source')]))
            pose=dict(x=15,y=0,z=28,yaw=0)
            with patch('prepare_base5_static_height_reference.gen.context',return_value={'links':[]}), \
                 patch('prepare_base5_static_height_reference.target_probe_pose',return_value=pose), \
                 patch('prepare_base5_static_height_reference.gen.geo.set_ego_pose'):
                result=nearby_reference(root/'manifest.json',0,root/'out',target_uid=2)
            import json
            row=json.loads(result.read_text())['scenes'][0]
            new_scene=json.loads(Path(row['scenario']).read_text());new_meta=json.loads(Path(row['metadata']).read_text())
            self.assertEqual(new_scene['objectList'],[])
            self.assertEqual(new_scene['vehicleList'],scene['vehicleList'])
            self.assertEqual(new_meta['ground_support_check']['required_uids'],[2])
            self.assertTrue(new_meta['calibration_only'])
            self.assertEqual(new_meta['runtime_actors'][0]['unique_id'],2)
            self.assertEqual(json.loads((root/'scene.json').read_text()),scene)

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
            with self.assertRaisesRegex(RuntimeError,'witness changed'):apply_reference(ground,[],root,meta)
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
