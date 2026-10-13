import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_intersection_ground as ground
import base5_intersection_placement as placement
import generate_base5_independent as gen
from prepare_base5_endpoint_linear import endpoint_profile
from base5_capture_safety import atomic_json,sha256
from report_base5_intersection_ground import report


class EndpointLinearTest(unittest.TestCase):
    def profile(self,end=12.):
        return dict(asset='Woman2',data_id=1,center=dict(x=0.,y=0.),axis=[1.,0.],maximum_path_offset_m=1.,
            knots=[dict(s=-12.,l=0.,x=-12.,y=0.,z=28.),dict(s=0.,l=0.,x=0.,y=0.,z=28.1),
                   dict(s=end,l=0.,x=end,y=0.,z=28.2)],s_bounds_m=[-12.,end])

    def test_profile_is_two_endpoint_chord_not_interior_terrain(self):
        original=self.profile();original['knots'][1]['z']+=.15;before=copy.deepcopy(original)
        result=endpoint_profile(original,original,'self')
        self.assertEqual(original,before)
        self.assertEqual(len(result['knots']),2)
        self.assertFalse(result['endpoint_derivation']['borrowed_upper'])
        first,last=result['knots'];t=-first['s']/(last['s']-first['s'])
        z=ground.height(result,0.,0.)['z']
        self.assertAlmostEqual(z,first['z']+t*(last['z']-first['z']))
        self.assertGreater(abs(z-ground.height(original,0.,0.)['z']),.14)
        with self.assertRaises(ValueError):ground.height(result,13.,0.)

    def test_short_path_extension_is_explicit_pending_donor_candidate(self):
        result=endpoint_profile(self.profile(5.),self.profile(),'cw/Ped_Pet1')
        self.assertTrue(result['endpoint_derivation']['borrowed_upper'])
        self.assertEqual(result['endpoint_derivation']['donor_profile'],'cw/Ped_Pet1')
        self.assertFalse(result['live_operational_qa_passed'])
        donor=self.profile();donor['knots']=[dict(k,z=k['z']+1.) for k in donor['knots']]
        with self.assertRaisesRegex(ValueError,'incompatible'):endpoint_profile(self.profile(5.),donor,'bad')

    def test_actual_generation_serializes_two_waypoints_and_linear_target(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);report=root/'report.json';atomic_json(report,{})
            profile=endpoint_profile(self.profile(),self.profile(),'self')
            lookup=root/'lookup.json';atomic_json(lookup,dict(policy=ground.POLICY,status='LIVE_VALIDATED',sources=[],
                profiles={'cw/Woman2':profile},live_evidence=dict(path=str(report),sha256=sha256(report),validated_profiles=['cw/Woman2'])))
            cfg=dict(capture=dict(intersection_ground_lookup=dict(path=str(lookup),sha256=sha256(lookup),
                waypoint_mode=placement.ENDPOINT_MODE,require_covered_moving_paths=True)))
            source=dict(points=[dict(x=-8.,y=0.,z=99.),dict(x=3.,y=0.,z=99.)],target_position=dict(x=0.,y=0.,z=99.))
            result,_=placement.apply_path(source,dict(name='Woman2',data_id=1),'cw',cfg,{})
            self.assertEqual(len(result['points']),2)
            first,last=result['points'];t=-first['x']/(last['x']-first['x'])
            self.assertAlmostEqual(result['target_position']['z'],first['z']+t*(last['z']-first['z']))
            actor=dict(name='Woman2',data_id=1,unique_id=1,source_list='pedestrianList',motion='walking',
                position=first,motion_path=result)
            scene=dict(pedestrianList=[dict(UNIQUEID=1,pos=first)],waypointDataList=[dict(pathName='PedPath - 1',
                waypointData=[dict(pos=gen.geo.world_to_morai_waypoint(p)) for p in result['points']])])
            ground.validate_actor(actor,{},scene)
            actor['motion_path']['points'].insert(1,result['target_position'])
            with self.assertRaisesRegex(ValueError,'exactly two'):ground.validate_actor(actor,{},scene)

    def test_direction_deck_balanced_and_retry_does_not_reroll_sign(self):
        config=dict(seed=100,capture={},mission_dynamic_observation=dict(balance_crossing_directions=True))
        rows=[dict(family_index=i,subfamily='person',asset_index=0,motion='running',mission=True,
                   dynamic_observation_phase='crossing') for i in range(10)]
        ctx=dict(mission_templates={'dynamic_obstacle':[('ref',{})]},pools={'person':[dict(name='Woman2')]})
        with patch.object(gen,'dynamic_crossing_candidates',return_value=[dict(crosswalk_id='a'),dict(crosswalk_id='b')]), \
             patch.object(gen,'population_appearance',return_value=('running','verified')):
            gen.assign_crossing_directions(rows,ctx,config)
        for parity in (0,1):
            signs=[r['crossing_direction_sign'] for r in rows if r['family_index']%2==parity]
            self.assertLessEqual(abs(sum(signs)),1)
        crossing=dict(center=dict(x=0.,y=0.,z=28.),axis=[1.,0.])
        import random
        for seed in range(10):
            self.assertEqual(gen.crossing_target(crossing,random.Random(seed),3.,-1)['yaw'],180.)

    def test_odd_person_and_rare_groups_do_not_accumulate_one_direction(self):
        config=dict(seed=100,capture={},mission_dynamic_observation=dict(balance_crossing_directions=True))
        rows=[dict(family_index=0,subfamily=kind,asset_index=0,motion='walking',mission=True,
                   dynamic_observation_phase='crossing') for kind,n in [('person',5),('animal',3),('pet_person',3)] for _ in range(n)]
        ctx=dict(mission_templates={'dynamic_obstacle':[('ref',{})]},
                 pools={k:[dict(name=k)] for k in ('person','animal','pet_person')})
        with patch.object(gen,'dynamic_crossing_candidates',return_value=[dict(crosswalk_id='a')]), \
             patch.object(gen,'population_appearance',return_value=('walking','verified')):
            gen.assign_crossing_directions(rows,ctx,config)
        self.assertLessEqual(abs(sum(r['crossing_direction_sign'] for r in rows)),1)
        for kind in ctx['pools']:
            self.assertLessEqual(abs(sum(r['crossing_direction_sign'] for r in rows if r['subfamily']==kind)),1)

    def test_old_direction_contract_unchanged_when_not_requested(self):
        rows=[dict(mission=True,motion='walking')];before=copy.deepcopy(rows)
        gen.assign_crossing_directions(rows,{},dict(capture={}))
        self.assertEqual(rows,before)

    def test_prior_review_retained_but_never_counted_as_profile_approval(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);lookup=root/'lookup.json';atomic_json(lookup,dict(policy=ground.POLICY,sources=[],
                profiles={'cw/Woman2':self.profile()}))
            def session(name,directions,accepted):
                folder=root/name;folder.mkdir()
                atomic_json(folder/'shutdown_safety.json',dict(park_verified=True))
                atomic_json(folder/'session_contract.json',{})
                atomic_json(folder/'status.json',dict(stage='complete' if accepted else 'halted',elapsed_sec=10.))
                rows=[]
                for i,sign in enumerate(directions):
                    cap=folder/f'capture{i}';cap.mkdir()
                    audit=dict(accepted_candidate=accepted,review_reasons=[] if accepted else ['missing_actor_annotation'],
                        capture_safety=dict(collision=dict(no_collision_verified=True)),
                        vertical_motion_actors=[dict(maximum_height_error_m=0.,maximum_observed_z_step_m=0.)],
                        ground_support=dict(actors=[],manual_policy_decision=dict(warnings=[])))
                    meta=dict(runtime_actors=[dict(motion='running',motion_path=dict(ground_path_evidence=
                        dict(lookup_sha256=sha256(lookup),profile_key='cw/Woman2')))],manual_ground_pilot=dict(direction=sign))
                    for file,value in [('metadata.json',meta),('audit.json',audit),('scenario.json',{}),
                                       ('telemetry.json',{}),('collision_history.json',{})]:atomic_json(cap/file,value)
                    rows.append(dict(id=name+str(i),status='accepted_candidate' if accepted else 'review',audit=audit,
                        capture_root=str(cap),scenario_sha256=sha256(cap/'scenario.json'),duration_sec=5.))
                (folder/'ledger.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
                return folder
            successful=session('successful',[-1,1],True);prior=session('prior',[-1],False)
            path=report(lookup,[successful],root/'report',[prior])
            data=json.loads(path.read_text());evidence=json.loads(Path(data['live_evidence']['path']).read_text())
            self.assertEqual(evidence['captures'],2)
            self.assertEqual(evidence['total_actual_capture_attempts'],3)
            self.assertEqual(evidence['prior_review_captures'],1)
            self.assertEqual(evidence['validated_profiles'],['cw/Woman2'])
            self.assertFalse(evidence['prior_diagnostic_attempts'][0]['used_for_profile_approval'])


if __name__=='__main__':unittest.main()
