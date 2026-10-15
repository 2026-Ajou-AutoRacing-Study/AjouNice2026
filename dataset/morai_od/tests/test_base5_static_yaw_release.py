import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json
from prepare_base5_static_yaw_pilot import prepare
from revise_base5_static_yaw import target_scope
from report_base5_static_yaw_validation import raw_snapshot
from audit_base5_scenarios import original_character_gap_bounds


class YawReleaseTests(unittest.TestCase):
    def test_clearance_change_is_explicit_and_person_only(self):
        row=dict(family='obstacle',mission=True,avoidance_phase='alongside',avoidance_gap_bounds_m=[.6,.8],
            character_spawn_gap_original_bounds_m=[.2,.3],character_spawn_gap_policy='near-person-spawn-envelope-v129')
        config=dict(stationary_character_spawn=dict(policy='near-person-spawn-envelope-v129',
            primary_pool='person',phase='alongside',minimum_requested_surface_gap_m=.6))
        self.assertEqual(original_character_gap_bounds(row,'person',config),[.2,.3])
        for invalid,pool in ((row,'vehicle'),(dict(row,mission=False),'person'),(dict(row,avoidance_gap_bounds_m=[.8,1.2]),'person')):
            with self.assertRaises(ValueError):original_character_gap_bounds(invalid,pool,config)

    def test_scope_excludes_moving_crossing_and_shadow(self):
        a=dict(source_list='pedestrianList',motion='static',population_pool='person')
        m=dict(spec=dict(mission=False),runtime_actors=[a])
        self.assertEqual(target_scope(m),'generic')
        for field in ('shadow_phase','dynamic_observation_phase','roundabout_phase'):
            self.assertIsNone(target_scope(dict(m,spec=dict(m['spec'],**{field:'phase'}))))
        self.assertIsNone(target_scope(dict(m,runtime_actors=[dict(a,motion='walking')])))
        self.assertEqual(target_scope(dict(m,spec=dict(mission=True,avoidance_phase='alongside'))),'stationary_avoidance')

    def test_add_four_distinct_people_without_editing_inputs(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td);entries=[]
            for i,name in enumerate(('Man1','Man2','Man2','Man3','Man4','Woman1')):
                meta=p/f'{i}.json'
                atomic_json(meta,dict(spec=dict(character_spawn_gap_policy='test'),runtime_actors=[dict(name=name)]))
                entries.append(dict(id=str(i),metadata=str(meta)))
            source=p/'source.json';pilot=p/'pilot.json';out=p/'extended.json'
            atomic_json(source,dict(scenes=entries));atomic_json(pilot,dict(scenes=entries[:1]))
            before=(source.read_bytes(),pilot.read_bytes());prepare(source,pilot,out)
            result=json.loads(out.read_text())
            self.assertEqual(len(result['scenes']),5)
            self.assertEqual(len({x['id'] for x in result['scenes']}),5)
            self.assertEqual(before,(source.read_bytes(),pilot.read_bytes()))
            with self.assertRaises(FileExistsError):prepare(source,pilot,out)

    def test_raw_snapshot_detects_content_change_and_ignores_dashboard(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);cam=root/'CAMERA_1';cam.mkdir();raw=cam/'0.png';raw.write_bytes(b'raw')
            initial=raw_snapshot(root);self.assertEqual(initial['files'],1)
            (root/'status.json').write_text('{}')
            self.assertEqual(initial,raw_snapshot(root))
            raw.write_bytes(b'new')
            self.assertNotEqual(initial,raw_snapshot(root))
