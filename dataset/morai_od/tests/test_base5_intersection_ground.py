import copy
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_intersection_ground as g
from base5_capture_safety import atomic_json,sha256
import generate_dynamic_object_capture_set as geo


class EditorGroundTest(unittest.TestCase):
    def profile(self):
        return dict(asset='Woman2',data_id=1,center=dict(x=0.,y=0.),axis=[1.,0.],maximum_path_offset_m=1.,
            knots=[dict(s=0.,l=.1,x=0.,y=.1,z=28.),dict(s=10.,l=.3,x=10.,y=.3,z=28.2)])

    def test_actual_xy_height_not_snapped_world_coordinates(self):
        q=g.height(self.profile(),5.,.25)
        self.assertAlmostEqual(q['z'],28.1)
        self.assertAlmostEqual(q['path_l'],.2)
        self.assertAlmostEqual(q['lateral_distance_m'],.05)

    def test_length_and_parallel_band_are_bounded(self):
        for x,y in ((-1.,0.),(11.,0.),(5.,2.)):
            with self.assertRaises(ValueError):g.height(self.profile(),x,y)

    def test_source_tamper_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.json';atomic_json(source,{})
            lookup=root/'lookup.json';atomic_json(lookup,dict(policy=g.POLICY,sources=[dict(path=str(source),sha256=sha256(source))]))
            g.load_lookup(lookup,sha256(lookup));atomic_json(source,dict(changed=True))
            with self.assertRaisesRegex(ValueError,'source hash'):g.load_lookup(lookup)

    def test_pending_cannot_enter_training_and_waypoint_tamper_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);lookup=root/'lookup.json'
            atomic_json(lookup,dict(policy=g.POLICY,status='MANUAL_SAMPLES_PENDING_LIVE',sources=[],profiles={'k':self.profile()}))
            points=[dict(x=1.,y=.12,z=28.02),dict(x=9.,y=.28,z=28.18)]
            proof=dict(policy=g.POLICY,lookup_file=str(lookup),lookup_sha256=sha256(lookup),profile_key='k')
            actor=dict(name='Woman2',data_id=1,unique_id=1,source_list='pedestrianList',motion='walking',position=points[0],
                       motion_path=dict(points=points,target_position=points[0],ground_path_evidence=proof))
            scene=dict(pedestrianList=[dict(UNIQUEID=1,pos=points[0])],waypointDataList=[dict(pathName='PedPath - 1',
                       waypointData=[dict(pos=geo.world_to_morai_waypoint(p)) for p in points])])
            g.validate_actor(actor,dict(calibration_only=True),scene)
            with self.assertRaisesRegex(ValueError,'diagnostic only'):g.validate_actor(actor,{},scene)
            scene['waypointDataList'][0]['waypointData'][0]['pos']['y']+=.1
            with self.assertRaisesRegex(ValueError,'raw waypoint'):g.validate_actor(actor,dict(calibration_only=True),scene)

    def test_manual_qa_keeps_severe_but_not_uncertainty(self):
        ground=dict(actors=[dict(uid=1,reason='actor_ground_plane_unverified'),
            dict(uid=2,reason='actor_annotation_above_ground',minimum_annotation_ground_gap_m=.2,plane={"observed":True})])
        result=g.ground_decision(ground)
        self.assertFalse(result['review_reasons']);self.assertEqual(len(result['warnings']),2)
        ground['actors'][1]['minimum_annotation_ground_gap_m']=.5
        self.assertIn('manual_ground_severe_above',g.ground_decision(ground)['review_reasons'])

    def test_unknown_or_unsupported_qa_policy_refused(self):
        self.assertFalse(g.explicit_manual_mode({}))
        for meta in (dict(ground_support_check=dict(manual_policy='invented')),
                     dict(ground_support_check=dict(manual_policy=g.QA_POLICY),runtime_actors=[])):
            with self.assertRaises(ValueError):g.explicit_manual_mode(meta)

    def test_sloped_path_vertical_residual_small_and_floating_is_severe(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);lookup=root/'lookup.json'
            atomic_json(lookup,dict(policy=g.POLICY,sources=[],profiles={'k':self.profile()}))
            actor=dict(unique_id=1,source_list='pedestrianList',motion='walking',motion_path=dict(ground_path_evidence=
                dict(lookup_file=str(lookup),lookup_sha256=sha256(lookup),profile_key='k')))
            trace=[dict(uid=1,t=i*.1,x=float(i),y=.1+.02*i,z=28.+.02*i) for i in range(5)]
            result=g.vertical_review(actor,trace,0.)
            self.assertFalse(result['review_reasons']);self.assertAlmostEqual(result['maximum_height_error_m'],0.)
            for p in trace:p['z']+=.5
            self.assertIn('motion_manual_height_severe_mismatch',g.vertical_review(actor,trace,0.)['review_reasons'])


if __name__=='__main__':unittest.main()
