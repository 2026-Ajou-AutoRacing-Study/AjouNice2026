import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import runpy

sys.path.insert(0,str(Path(__file__).resolve().parent))
import reinforcement_profile as r


class ReinforcementProfileTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        evidence=self.root/'actual_training.json';r.atomic_json(evidence,dict(measured=True))
        self.profile=dict(schema=r.SCHEMA,policy=r.POLICY,name='shared-midrange-v1',seed=123,
            evidence=[r.pin(evidence)],maximum_changed_scenes=20,minimum_remaining_near_actors=0,
            maximum_near_loss_fraction=.25,
            cpu_feasibility=dict(maximum_attempts=4,maximum_candidate_tests=32),
            unchanged_classes=dict(vehicle='Front/rear visibility diagnosis before extra allocation.',
                pedestrian='Ordinary human, animal and compound diagnosis remains separate.'),
            rules=[dict(id='mid',assets={'Box':4},source_bounds_m=[5,20],target_bounds_m=[20,30],
                roles=['front','rear','left','right'],maximum_asset_near_fraction=.4,
                minimum_asset_near_remaining=3)])
        self.sizes={'Box':[1.,1.,1.],'Thin':[.2,.2,1.]}

    def row(self,index=0,name='Box',distance=10,**kwargs):
        actor=dict(name=name,data_id=1 if name=='Box' else 2,unique_id=8500,
            source_list='objectList',population_pool='obstacle_box',motion='static',
            planned_center_distance_m=distance,actual_ego_role='front')
        spec=dict(family='obstacle',mission=False,role='front',motion='static',
            topology='route_general',seed=index,actors=1,distance_bounds=[5,15],distance_bin='near')
        spec.update(kwargs)
        meta=dict(spec=spec,runtime_actors=[actor],split='train',retry=0)
        filename=self.root/f'meta{index}.json';r.atomic_json(filename,meta)
        return dict(id=f'slot{index}',entry=dict(id=f'slot{index}',metadata=str(filename),scenario_sha256='a'*64),meta=meta)

    def test_unknown_or_environment_selected_policy_rejected(self):
        self.assertEqual(r.validate_profile(self.profile),self.profile)
        for key,value in (('environment','foggy13'),('schema','unknown'),('maximum_changed_scenes',True)):
            changed=copy.deepcopy(self.profile);changed[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):r.validate_profile(changed)

    def test_evidence_actual_bytes_are_verified(self):
        Path(self.profile['evidence'][0]['path']).write_text('changed')
        with self.assertRaisesRegex(ValueError,'evidence changed'):r.validate_profile(self.profile)

    def test_all_class_no_allocation_reasons_are_required(self):
        changed=copy.deepcopy(self.profile);changed['unchanged_classes'].pop('pedestrian')
        with self.assertRaisesRegex(ValueError,'no-allocation reasons'):r.validate_profile(changed)

    def test_invalid_windows_or_unbounded_fraction_rejected(self):
        for field,value in (('target_bounds_m',[30,20]),('source_bounds_m',[0,30]),
                            ('maximum_asset_near_fraction',.9),('minimum_asset_near_remaining',0),
                            ('assets',{'Box':500}),('roles',['front','front'])):
            changed=copy.deepcopy(self.profile);changed['rules'][0][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):r.validate_profile(changed)

    def test_static_generic_only_and_native_cap_not_overridden(self):
        rule=self.profile['rules'][0];baseline=self.row()
        self.assertTrue(r.eligible(baseline['meta'],rule,self.sizes))
        for field,value in (('mission',True),('avoidance_phase','entry'),('family','shadow'),
                            ('topology','highway'),('close_primary_observation',{'x':1})):
            changed=copy.deepcopy(baseline);changed['meta']['spec'][field]=value
            with self.subTest(field=field):self.assertFalse(r.eligible(changed['meta'],rule,self.sizes))
        changed=copy.deepcopy(baseline);changed['meta']['runtime_actors'][0]['motion']='walking'
        self.assertFalse(r.eligible(changed['meta'],rule,self.sizes))
        thin_rule=copy.deepcopy(rule);thin_rule['assets']={'Thin':4}
        self.assertFalse(r.eligible(self.row(99,name='Thin')['meta'],thin_rule,self.sizes))

    def test_30plus_side_not_targeted_but_front_can_be(self):
        rule=copy.deepcopy(self.profile['rules'][0]);rule['target_bounds_m']=[30,45]
        meta=self.row()['meta']
        self.assertTrue(r.eligible(meta,rule,self.sizes))
        meta['spec']['role']='left'
        self.assertFalse(r.eligible(meta,rule,self.sizes))

    def test_previous_stricter_native_cap_is_not_overridden(self):
        meta=self.row()['meta'];meta['spec']['primary_obstacle_range']={'cap_m':20}
        self.assertFalse(r.eligible(meta,self.profile['rules'][0],self.sizes))

    def test_clustered_primary_change_cannot_force_unprotected_near_secondaries_out(self):
        meta=self.row(population_layout='clustered')['meta']
        secondary=copy.deepcopy(meta['runtime_actors'][0]);secondary['unique_id']=8501
        meta['runtime_actors'].append(secondary)
        self.assertFalse(r.eligible(meta,self.profile['rules'][0],self.sizes))
        meta['spec']['secondary_obstacle_support']={'protected_near_ordinals':[1]}
        self.assertTrue(r.eligible(meta,self.profile['rules'][0],self.sizes))

    def test_allocation_is_deterministic_and_bounded_without_source_mutation(self):
        rows=[self.row(i) for i in range(20)];before=copy.deepcopy(rows)
        a=r.allocate(rows,self.profile,self.sizes,'a'*64)
        b=r.allocate(list(reversed(rows)),self.profile,self.sizes,'a'*64)
        self.assertEqual(a,b);self.assertEqual(rows,before)
        self.assertEqual(len(a['assignments']),4)
        self.assertFalse(a['native_outcomes_used_for_selection']);self.assertFalse(a['bulk_ready'])

    def test_global_near_budget_and_per_asset_floor(self):
        rows=[self.row(i) for i in range(20)]
        p=copy.deepcopy(self.profile);p['minimum_remaining_near_actors']=18
        a=r.allocate(rows,p,self.sizes,'a'*64);self.assertEqual(len(a['assignments']),2)
        p=copy.deepcopy(self.profile);p['rules'][0]['minimum_asset_near_remaining']=19
        a=r.allocate(rows,p,self.sizes,'a'*64);self.assertEqual(len(a['assignments']),1)
        p['rules'][0]['minimum_asset_near_remaining']=20
        with self.assertRaisesRegex(ValueError,'No eligible'):r.allocate(rows,p,self.sizes,'a'*64)

    def test_last_near_asset_role_is_retained(self):
        rows=[self.row(i) for i in range(20)]
        rows[0]['meta']['runtime_actors'][0]['actual_ego_role']='rear'
        rows[0]['meta']['spec']['role']='rear'
        allocation=r.allocate(rows,self.profile,self.sizes,'a'*64)
        self.assertNotIn(rows[0]['id'],{row['id'] for row in allocation['assignments']})

    def test_multiple_rules_cannot_reselect_or_exceed_cumulative_cap(self):
        rows=[self.row(i) for i in range(20)]
        p=copy.deepcopy(self.profile);p['rules'][0]['assets']['Box']=20
        second=copy.deepcopy(p['rules'][0]);second['id']='second';p['rules'].append(second)
        a=r.allocate(rows,p,self.sizes,'a'*64)
        self.assertEqual(len(a['assignments']),5)
        self.assertEqual(len({x['id'] for x in a['assignments']}),5)

    def test_pilot_round_robin_is_independent_of_outcome(self):
        a=dict(assignments=[dict(id=str(i),rule='mid',asset='A' if i<4 else 'B') for i in range(8)])
        rows=r.pilot_assignments(a,4)
        self.assertEqual([x['asset'] for x in rows],['A','B','A','B'])
        for size in (0,49,True):
            with self.assertRaises(ValueError):r.pilot_assignments(a,size)

    def test_same_role_mid_helper_is_not_a_compatible_far_primary_source(self):
        rule=copy.deepcopy(self.profile['rules'][0]);rule['target_bounds_m']=[30,45]
        meta=self.row()['meta'];helper=copy.deepcopy(meta['runtime_actors'][0])
        helper.update(unique_id=8501,planned_center_distance_m=23)
        meta['runtime_actors'].append(helper)
        self.assertFalse(r.eligible(meta,rule,self.sizes))
        helper['actual_ego_role']='rear'
        self.assertTrue(r.eligible(meta,rule,self.sizes))
        helper['actual_ego_role']='front';helper['planned_center_distance_m']=9
        self.assertTrue(r.eligible(meta,rule,self.sizes))

    def test_cpu_rejection_is_rule_specific_and_preserves_same_asset_request(self):
        rows=[self.row(i) for i in range(20)]
        initial=r.allocate(rows,self.profile,self.sizes,'a'*64)
        rejected=initial['assignments'][0]
        revised=r.allocate(rows,self.profile,self.sizes,'a'*64,
            cpu_rejected_pairs={(rejected['id'],rejected['rule'])})
        self.assertEqual(len(revised['assignments']),len(initial['assignments']))
        self.assertNotIn(rejected['id'],{x['id'] for x in revised['assignments']})
        other=r.allocate(rows,self.profile,self.sizes,'a'*64,
            cpu_rejected_pairs={(rejected['id'],'other-rule')})
        self.assertEqual(initial,other)

    def test_cumulative_cap_uses_strictest_declared_rule(self):
        rows=[self.row(i) for i in range(20)]
        p=copy.deepcopy(self.profile);p['rules'][0].update(assets={'Box':2},maximum_asset_near_fraction=.15)
        second=copy.deepcopy(p['rules'][0]);second.update(id='other',assets={'Box':10},maximum_asset_near_fraction=.4)
        p['rules'].append(second)
        allocation=r.allocate(rows,p,self.sizes,'a'*64)
        self.assertEqual(len(allocation['assignments']),3)

    def test_runpy_preparation_binds_its_own_driver_not_callers_main_module(self):
        namespace=runpy.run_path(r.__file__,run_name='unregistered_runpy_namespace')
        prepare=namespace['prepare'];output=self.root/'new-output'
        request=dict(profile=copy.deepcopy(self.profile))

        def first_cpu_step(*args,driver,**kwargs):
            self.assertEqual(driver.__file__,r.__file__)
            self.assertTrue(callable(driver.physical))
            self.assertTrue(callable(driver.verify_transition))
            raise RuntimeError('verified exact runpy driver')

        with mock.patch.dict(prepare.__globals__,{
                'plan':mock.Mock(return_value=request),'safe_output':lambda *_:output,
                'records_from_manifest':mock.Mock(return_value=({'config':{}},[])),
                'read':mock.Mock(return_value=self.profile)}), \
             mock.patch.object(r.cpu_screen,'prepare',side_effect=first_cpu_step), \
             self.assertRaisesRegex(RuntimeError,'verified exact runpy driver'):
            prepare('source','build','profile',output)

    def test_changed_spec_archives_old_plan_but_never_forges_legacy_cap(self):
        rows=[self.row(i) for i in range(20)];rows[0]['meta']['spec']['primary_obstacle_range']={'old':'plan'}
        allocation=r.allocate(rows,self.profile,self.sizes,'a'*64)
        assignment=allocation['assignments'][0];lookup={x['id']:x for x in rows}
        meta=lookup[assignment['id']]['meta'];before=copy.deepcopy(meta)
        spec=r.changed_spec(meta,assignment)
        self.assertNotIn('primary_obstacle_range',spec)
        self.assertEqual(spec['distance_bounds'],[20,30])
        self.assertEqual(spec['frozen_population_data_ids'],[1]);self.assertEqual(spec['layout_split'],'train')
        self.assertEqual(meta,before)

    def test_transition_rejects_roster_split_range_role_and_unapproved_field_changes(self):
        rows=[self.row(i) for i in range(20)];assignment=r.allocate(rows,self.profile,self.sizes,'a'*64)['assignments'][0]
        before=next(x['meta'] for x in rows if x['id']==assignment['id'])
        after=copy.deepcopy(before);after['spec']=r.changed_spec(before,assignment)
        after['runtime_actors'][0]['planned_center_distance_m']=25
        self.assertTrue(r.verify_transition(before,after,assignment))
        variants=[]
        x=copy.deepcopy(after);x['runtime_actors'][0]['data_id']=999;variants.append(x)
        x=copy.deepcopy(after);x['split']='val';variants.append(x)
        x=copy.deepcopy(after);x['runtime_actors'][0]['planned_center_distance_m']=19;variants.append(x)
        x=copy.deepcopy(after);x['runtime_actors'][0]['actual_ego_role']='rear';variants.append(x)
        x=copy.deepcopy(after);x['spec']['yaw_limit']=0;variants.append(x)
        for value in variants:
            with self.assertRaises(ValueError):r.verify_transition(before,value,assignment)


if __name__=='__main__':unittest.main()
