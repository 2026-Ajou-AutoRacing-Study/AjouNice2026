"""원형-only 사전 선택: 비확대·원문보존·독립 geometry와 고정 반복 검사."""
import copy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_unit_only_stability as prep


class UnitOnlyPreparationTests(unittest.TestCase):
    def scene(self,container='objectList'):
        actor=dict(UNIQUEID=8500,DataID=40100001)
        if container=='objectList':actor['scale']=dict(x=1.,y=1.,z=1.,_x='1.000',_y='1.000',_z='1.000')
        return {container:[actor]}

    def test_explicit111_and_serializer_default_are_distinct_and_no_input_is_normalized(self):
        scene=self.scene();scene.update(self.scene('vehicleList'))
        before=copy.deepcopy(scene)
        result=prep.raw_unit_scales(scene)
        self.assertEqual(result,{'objectList/explicit111':1,'vehicleList/serializer_default_no_override':1})
        self.assertEqual(scene,before)

    def test_object_scale_omission_and_any_explicit_nonunit_or_malformed_scale_are_rejected(self):
        for container in prep.CONTAINERS:
            for scale in (dict(x=1.,y=2.,z=1.),dict(x=True,y=1.,z=1.),dict(x=1.,y=1.),
                    dict(x=1.,y=1.,z=float('nan'))):
                with self.subTest(container=container,scale=scale),self.assertRaises(ValueError):
                    prep.raw_unit_scales({container:[dict(scale=scale)]})
        with self.assertRaisesRegex(ValueError,'explicit scale111'):
            prep.raw_unit_scales(dict(objectList=[{}]))

    def test_ui_mirror_if_present_must_agree_with_the_numeric_unit_axes(self):
        for mirror in ('1.250','NaN','garbage',True,None):
            scene=self.scene();scene['objectList'][0]['scale']['_z']=mirror
            with self.subTest(mirror=mirror),self.assertRaises((ValueError,TypeError)):
                prep.raw_unit_scales(scene)
        scene=self.scene();scene['objectList'][0]['scale'].pop('_z')
        self.assertEqual(prep.raw_unit_scales(scene),{'objectList/explicit111':1})

    def test_scale_assignment_plan_or_probe_cannot_be_disguised_as_unit_source(self):
        for metadata in (dict(spec=dict(asset_scale_plan={})),dict(scale_probe={}),
                dict(asset_scale_preparation={}),dict(runtime_actors=[dict(asset_scale=dict(profile_id='original'))]),
                dict(runtime_actors=[dict(asset_scale_geometry={})])):
            with self.subTest(metadata=metadata),self.assertRaises(ValueError):
                prep.reject_scale_metadata(metadata)
        prep.reject_scale_metadata(dict(spec={},runtime_actors=[]))

    def test_metadata_and_raw_actor_rosters_are_bound_including_uid_container_and_name(self):
        catalog={40100001:dict(name='CargoBox',source_list='objectList')}
        meta=dict(runtime_actors=[dict(unique_id=8500,data_id=40100001,name='CargoBox',source_list='objectList')])
        scene=self.scene();prep.bind_actors(scene,meta,catalog)
        for kind in ('uid','name','container','raw_extra'):
            altered,raw=copy.deepcopy(meta),copy.deepcopy(scene)
            if kind=='uid':altered['runtime_actors'][0]['unique_id']=9999
            if kind=='name':altered['runtime_actors'][0]['name']='WoodBox'
            if kind=='container':altered['runtime_actors'][0]['source_list']='vehicleList'
            if kind=='raw_extra':raw['pedestrianList']=[dict(UNIQUEID=9000,DataID=50100001)]
            with self.subTest(kind=kind),self.assertRaises(ValueError):prep.bind_actors(raw,altered,catalog)

    def records(self):
        physical=sorted(asset['name'] for asset in prep.read(prep.DEFAULT_CATALOG)['assets']
            if asset.get('semantic_class')=='obstacle')
        others=[name for name in physical if name not in prep.PROBLEM_ASSETS]
        data_ids={asset['name']:asset['data_id'] for asset in prep.read(prep.DEFAULT_CATALOG)['assets']}
        records=[]
        for index in range(128):
            names=['WoodBox','RedBarrel',others[index%len(others)],others[(index+5)%len(others)]]
            actors=[]
            for ordinal,name in enumerate(names):
                x,y=10.+ordinal*3.,(index%3)*4.+ordinal
                actors.append(dict(name=name,data_id=data_ids[name],source_list='objectList',
                    unique_id=8500+ordinal,position=dict(x=x+index*50.,y=y,z=0.),
                    geometric_center_estimate=[x+index*50.,y],estimated_size=[1.,1.,1.],
                    placement_yaw_deg=float(index%4*90),motion='static',
                    planned_center_distance_m=15.+index%4*10,actual_ego_role='front'))
            meta=dict(spec=dict(family='obstacle' if index%2 else 'shadow',mission=bool(index%2)),
                split='train',ego=dict(x=index*50.,y=0.,z=0.,yaw=0.),runtime_actors=actors)
            records.append(dict(id='source_'+str(index),meta=meta,scenario_sha256=str(index)))
        return records,set(physical)

    def test_exact64_independent_layouts_plus16_fixed_payload_repeats_are_deterministic(self):
        records,physical=self.records();before=copy.deepcopy(records)
        selected,repeats,report=prep.select(records,physical)
        self.assertEqual(records,before)
        self.assertEqual(len(selected),64);self.assertEqual(len(set(selected)),64)
        self.assertEqual(len(repeats),16);self.assertEqual(len(set(repeats)),16)
        self.assertLessEqual(set(repeats),set(selected))
        self.assertEqual(len({report['group_of'][ident] for ident in selected}),64)
        for name in physical:
            self.assertGreaterEqual(report['selected_feature_counts']['asset/'+name],16 if name in prep.PROBLEM_ASSETS else 4)
        self.assertFalse(report['native_outcomes_used_for_selection'])
        self.assertEqual(prep.select(list(reversed(records)),physical)[:2],(selected,repeats))

    def test_relabeling_same_geometry_cannot_meet_independent_inventory_or_raise_capture_cap(self):
        records,physical=self.records()
        copies=[dict(copy.deepcopy(records[0]),id='renamed_'+str(index)) for index in range(128)]
        with self.assertRaisesRegex(ValueError,'independent static mixed geometry inventory'):
            prep.select(copies,physical)
        for args in ((65,16),(64,17),(True,16)):
            with self.subTest(args=args),self.assertRaisesRegex(ValueError,'fixed at64'):
                prep.select(records,physical,*args)


if __name__=='__main__':unittest.main()
