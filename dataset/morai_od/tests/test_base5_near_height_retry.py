import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import atomic_json,sha256
from prepare_base5_ground_retry import prepare_from_near


class NearHeightTest(unittest.TestCase):
    def fixture(self,root):
        scene=dict(pedestrianList=[dict(UNIQUEID=1,pos=dict(x=5.,y=1.,z=28.))])
        actor=dict(unique_id=1,source_list='pedestrianList',motion='standing',position=dict(x=5.,y=1.,z=28.))
        atomic_json(root/'scene.json',scene);atomic_json(root/'meta.json',dict(runtime_actors=[actor]))
        digest=sha256(root/'scene.json')
        atomic_json(root/'manifest.json',dict(scenes=[dict(id='test',scenario=str(root/'scene.json'),metadata=str(root/'meta.json'),scenario_sha256=digest)]))
        status=dict(unique_id=1,type=2,heading=0,position=dict(x=5.,y=1.,z=28.),size=dict(x=1.,y=.5,z=1.),velocity=dict(x=0.,y=0.,z=0.))
        tele=dict(objects_before=dict(pedestrian_list=[status]),objects_after=dict(pedestrian_list=[status]))
        for name in ('old','near'):
            session=root/name;cap=session/'capture';cap.mkdir(parents=True)
            atomic_json(session/'shutdown_safety.json',dict(park_verified=True));atomic_json(cap/'telemetry.json',tele)
            (session/'ledger.jsonl').write_text(json.dumps(dict(capture_root=str(cap),scenario_sha256=digest))+'\n')
        nr=root/'near/capture';atomic_json(nr/'scenario.json',scene)
        atomic_json(nr/'metadata.json',dict(calibration_only=True,static_height_reference=dict(source_scenario_sha256=digest)))
        atomic_json(nr/'collision_history.json',{})
        lid=nr/'LIDAR_5';lid.mkdir();(lid/'test_Instance.txt').write_text('fixture')
        audit=dict(review_reasons=['actor_annotation_above_ground'],capture_safety=dict(collision=dict(no_collision_verified=True)),
            ground_support=dict(actors=[dict(uid=1,plane=dict(coefficients=[0,0,0]),reason='actor_annotation_above_ground',
                minimum_annotation_ground_gap_m=.3,suggested_z_shift_m=-.33)]))
        atomic_json(nr/'audit.json',audit)
        (root/'near/ledger.jsonl').write_text(json.dumps(dict(capture_root=str(nr),scenario_sha256=digest,audit=audit))+'\n')
        return root/'manifest.json'

    def test_measured_near_correction_changes_input_not_original_raw(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);before=sha256(root/'scene.json')
            out=prepare_from_near(source,root/'old',root/'near',root/'out',1)
            entry=json.loads(out.read_text())['scenes'][0]
            scene=json.loads(Path(entry['scenario']).read_text())
            self.assertAlmostEqual(scene['pedestrianList'][0]['pos']['z'],27.67)
            self.assertEqual(sha256(root/'scene.json'),before)

    def test_actual_object_movement_invalidates_near_height_proof(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=self.fixture(root);path=root/'old/capture/telemetry.json'
            data=json.loads(path.read_text());data['objects_before']['pedestrian_list'][0]['position']['x']+=.1;atomic_json(path,data)
            with self.assertRaisesRegex(ValueError,'position changed'):
                prepare_from_near(source,root/'old',root/'near',root/'out',1)


if __name__=='__main__':unittest.main()
