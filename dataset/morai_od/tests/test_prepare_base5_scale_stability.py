"""Synthetic matched-control and outcome-free selection checks."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_scale_stability as prep


class ScaleStabilityPreparationTests(unittest.TestCase):
    def meta(self,name='CargoBox',profile='iso150',index=0):
        return dict(spec=dict(family='obstacle',asset_scale_plan=dict(source_spec=dict(family='obstacle'))),
            ego=dict(x=0.,y=0.,z=0.,yaw=0.),base=dict(yaw=0.),split='train',runtime_actors=[dict(
                unique_id=8500,data_id=40100001,name=name,source_list='objectList',motion='static',
                position=dict(x=10.+index,y=0.,z=0.),estimated_size=[2.,2.,2.],
                geometric_center_estimate=[10.+index,0.],planned_center_distance_m=10.+index,
                placement_yaw_deg=0.,map_ground_initializer=dict(usable=True,height_m=0.),
                asset_scale=dict(profile_id=profile),asset_scale_geometry=dict(placement_pivot_m=[0.,0.,1.]))])

    def test_counterfactual_preserves_identity_xy_yaw_and_source(self):
        meta=self.meta()
        scene=dict(objectList=[dict(UNIQUEID=8500,DataID=40100001,pos=dict(x=10.,y=0.,z=0.),
            scale=dict(x=1.5,y=1.5,z=1.5),rot=dict(yaw=0.))])
        before=copy.deepcopy((scene,meta))
        units=dict(CargoBox=dict(bbox_size_m=[1.,1.,1.],pivot_to_bbox_center_local_m=[0.,0.,.5]))
        raw,control=prep.unit_control(scene,meta,units)
        self.assertEqual((scene,meta),before)
        self.assertEqual(raw['objectList'][0]['UNIQUEID'],8500)
        self.assertEqual(raw['objectList'][0]['pos'],scene['objectList'][0]['pos'])
        self.assertEqual(raw['objectList'][0]['scale']['x'],1.)
        self.assertNotIn('asset_scale_plan',control['spec'])
        self.assertNotIn('asset_scale',control['runtime_actors'][0])
        self.assertFalse(control['runtime_actors'][0]['static_height_initializer']['contact_verified'])

    def test_unknown_unit_and_enlarging_unit_fail_closed(self):
        scene=dict(objectList=[])
        for units in ({},dict(CargoBox=dict(bbox_size_m=[3.,3.,3.],pivot_to_bbox_center_local_m=[0.,0.,1.5]))):
            with self.subTest(units=units),self.assertRaises(ValueError):
                prep.unit_control(scene,self.meta(),units)

    def test_active_training_tags_are_detached_but_source_split_and_spec_stay(self):
        import base5_layout_split as layout
        original=dict(split='train',spec=dict(split='original-source-field',seed=7),
            training_split_overlay=dict(path='/not-read',sha256='source-only'),
            split_overlay=dict(path='/also-not-read'),derived_training_split='val',
            derived_layout_group_id='layout_1',runtime_actors=[])
        before=copy.deepcopy(original)
        clean,provenance=prep.diagnostic_copy(original)
        self.assertEqual(original,before)
        self.assertEqual(clean['split'],'train')
        self.assertEqual(clean['spec'],original['spec'])
        self.assertEqual(set(provenance),{'training_split_overlay','split_overlay',
            'derived_training_split','derived_layout_group_id'})
        self.assertIsNone(layout.validate_manifest_overlay(clean,allow_subset=True))

    def test_prepared_ground_id_tag_is_detached_but_motion_geometry_and_unit_qa_stay(self):
        import base5_shadow_ground_reuse as shadow
        import base5_unit_geometry_contract as unit
        source=dict(self.meta(),**{shadow.KEY:dict(source_id='production_1'),
            unit.KEY:dict(path='/native.json',sha256='kept')},
            motion_ground_deck=dict(actual_waypoints=[1.,2.]))
        before=copy.deepcopy(source)
        clean,removed=prep.diagnostic_copy(source)
        self.assertEqual(source,before)
        self.assertNotIn(shadow.KEY,clean)
        self.assertEqual(removed[shadow.KEY],source[shadow.KEY])
        self.assertEqual(clean[unit.KEY],source[unit.KEY])
        self.assertEqual(clean['motion_ground_deck'],source['motion_ground_deck'])

    def test_source_shadow_and_layout_validate_before_registry_detachment(self):
        import base5_layout_split as layout
        import base5_shadow_ground_reuse as shadow
        source=dict(config={shadow.KEY:dict(registration=dict(path='/original',sha256='pinned'))},
            split_overlay=dict(path='/split',sha256='pinned'),scenes=[])
        before=copy.deepcopy(source)
        with patch.object(shadow,'validate_manifest',return_value={'/terrain':'sha'}) as ground, \
                patch.object(layout,'validate_manifest_overlay',return_value={'valid':True}) as split:
            clean,removed,pins=prep.diagnostic_source(source)
        ground.assert_called_once_with(source)
        split.assert_called_once_with(source)
        self.assertEqual(source,before)
        self.assertNotIn(shadow.KEY,clean['config'])
        self.assertIn('config/'+shadow.KEY,removed)
        self.assertEqual(pins,{'/terrain':'sha'})
        with patch.object(shadow,'validate_manifest',side_effect=RuntimeError('source changed')):
            with self.assertRaisesRegex(RuntimeError,'source changed'):
                prep.diagnostic_source(source)

    def test_selection_is_deterministic_independent_and_enforces_profile_minimum(self):
        metadata={str(i):self.meta(profile='a' if i%2 else 'b',index=i) for i in range(8)}
        profiles={('CargoBox','a'),('CargoBox','b')}
        chosen,coverage=prep.select(metadata,profiles,4,2)
        self.assertEqual(len(set(chosen)),4)
        self.assertEqual(set(coverage['profile_scene_counts'].values()),{2})
        self.assertEqual(chosen,prep.select(dict(reversed(list(metadata.items()))),profiles,4,2)[0])
        with self.assertRaises(ValueError):
            prep.select(metadata,profiles,3,2)
        with self.assertRaises(ValueError):
            prep.select(metadata,profiles,9,2)

    def test_prepare_snapshots_only_common_unit_references_not_growing_scene_pins(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def write(path,value):
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(json.dumps(value)+'\n')
                return path
            reference=dict(bbox_size_m=[1.,1.,1.],pivot_to_bbox_center_local_m=[0.,0.,.5])
            refs=[write(root/'references'/name,dict(assets={asset:reference}))
                  for name,asset in (('cargo.json','CargoBox'),('wood.json','WoodBox'))]
            empty=write(root/'source'/'empty_metadata.json',dict(runtime_actors=[]))
            entries=[]
            for index in range(2):
                meta=self.meta(index=index)
                raw=dict(objectList=[dict(UNIQUEID=8500,DataID=40100001,
                    pos=dict(meta['runtime_actors'][0]['position']),scale=dict(x=1.5,y=1.5,z=1.5),rot=dict(yaw=0.))])
                scene=write(root/'source'/f'{index}.json',raw)
                mp=write(root/'source'/f'{index}_metadata.json',meta)
                entries.append(dict(id='source_'+str(index),scenario=str(scene),metadata=str(mp),
                    scenario_sha256=prep.sha256(scene)))
            entries.extend(dict(id='empty_'+str(index),metadata=str(empty)) for index in range(4998))
            policy=dict(assets=dict(CargoBox=dict(profiles=dict(iso150=dict(status='qualified')))))
            source=write(root/'source'/'manifest.json',dict(count=5000,scenes=entries,
                config=dict(asset_scale_policy=policy)))
            # Native-policy validation is an existing separate gate. Exercise
            # the real reference merge, selection, controls and JSON snapshots.
            with patch.object(prep.scale,'freeze_policy',side_effect=lambda value:copy.deepcopy(value)), \
                    patch.object(prep.scale,'_qualified',return_value=True), \
                    patch.object(prep.scale,'verify_frozen_policy',return_value=None), \
                    patch.object(prep,'validate_entry',return_value=None):
                manifest=prep.prepare(source,root/'prepared',refs,pairs=2,minimum_per_profile=1)
            deck=json.loads(manifest.read_text())
            expected={str(path.resolve()):prep.sha256(path) for path in refs}
            actual=[json.loads(Path(entry['metadata']).read_text())['scale_stability']['unit_reference_sources']
                    for entry in deck['scenes']]
            self.assertEqual(len(actual),4)
            self.assertTrue(all(value==expected for value in actual))
            inventory=deck['scale_stability_registration']['source_files']
            self.assertGreater(len(inventory),len(expected))
            self.assertIn(str(source.resolve()),inventory)


if __name__=='__main__':
    unittest.main()
