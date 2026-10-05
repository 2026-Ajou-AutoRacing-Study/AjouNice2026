import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import collision_window,load_ledger,prepare_session,archive_incomplete,runtime_presence_review,confirm_park


class CaptureSafetyTest(unittest.TestCase):
    def test_park_retry_and_wrong_gear_are_not_silently_successful(self):
        calls=[]
        def request():
            calls.append(1)
            if len(calls)==1:raise RuntimeError('response timeout')
            return 1
        result=confirm_park(request,retry_delay=0)
        self.assertTrue(result['park_verified']);self.assertEqual(result['attempts'],2)
        result=confirm_park(lambda:4,retry_delay=0)
        self.assertFalse(result['park_verified']);self.assertEqual(result['attempts'],2)

    def test_ai_annotation_exception_never_waives_ordinary_or_invisible_actor(self):
        actor=dict(unique_id=8500,source_list='aiPedstrianList',motion='standing')
        components=[dict(parent_uid=8500,positive_lidar1_camera0=True)]
        self.assertEqual(runtime_presence_review(dict(runtime_actors=[actor]),{}, {},components),([],[8500]))
        self.assertEqual(runtime_presence_review(dict(runtime_actors=[actor]),{}, {},[]),([8500],[]))
        actor['motion']='walking'
        self.assertEqual(runtime_presence_review(dict(runtime_actors=[actor]),{}, {},components),([8500],[]))
        actor.update(motion='standing',source_list='pedestrianList')
        self.assertEqual(runtime_presence_review(dict(runtime_actors=[actor]),{}, {},components),([8500],[]))

    def test_ego_only_and_both_arrays(self):
        rows=[dict(t=t,message=dict(collision_object=[dict(unique_id=0)],collision_objecta=[])) for t in (.1,.5,.9)]
        self.assertTrue(collision_window(rows,0.,1.)['no_collision_verified'])
        rows[1]['message']['collision_objecta']=[dict(unique_id=8500)]
        result=collision_window(rows,0.,1.)
        self.assertFalse(result['no_collision_verified']);self.assertEqual(result['non_ego_uids'],[8500])
        # A later empty observation never erases earlier contact.
        self.assertEqual(result['messages'],3)
    def test_stale_or_absent_not_safe(self):
        self.assertFalse(collision_window([],0.,4.)['no_collision_verified'])
        self.assertFalse(collision_window([dict(t=.1,message={}),dict(t=3.9,message={})],0.,4.)['observed'])
    def test_torn_last_record_preserved_and_repaired(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'ledger.jsonl';p.write_bytes(b'{"id":"a"}\n{"id":')
            self.assertEqual(load_ledger(p),[dict(id='a')])
            self.assertEqual(len(list(Path(d).glob('*.torn.*'))),1)
            p.write_bytes(b'broken\n{"id":"b"}\n')
            with self.assertRaises(ValueError):load_ledger(p)
            p.write_bytes(b'{"id":"a"}')
            self.assertEqual(load_ledger(p),[dict(id='a')])
            self.assertTrue(p.read_bytes().endswith(b'\n'))
    def test_contract_and_orphan_archive(self):
        with tempfile.TemporaryDirectory() as d:
            r=Path(d);files=[]
            for name in ('manifest','network','sensor'):
                p=r/name;p.write_text('{}');files.append(p)
            out=r/'out';self.assertEqual(prepare_session(out,*files),([],{}))
            capture=out/'captures'/'a';capture.mkdir(parents=True);(capture/'raw.bin').write_bytes(b'raw')
            dest=archive_incomplete(out,'a');self.assertEqual((Path(dest)/'raw.bin').read_bytes(),b'raw')
            self.assertFalse(capture.exists())
            files[1].write_text('{"changed":true}')
            with self.assertRaises(ValueError):prepare_session(out,*files)
    def test_completed_record_requires_sensor_files(self):
        from base5_capture_safety import sha256
        with tempfile.TemporaryDirectory() as d:
            r=Path(d);files=[]
            for name in ('manifest','network','sensor'):
                p=r/name;p.write_text('{}');files.append(p)
            out=r/'out';prepare_session(out,*files)
            cap=out/'captures'/'a';cap.mkdir(parents=True)
            for name in ('scenario.json','metadata.json','audit.json','telemetry.json','collision_history.json'):
                (cap/name).write_text('{}')
            row=dict(id='a',status='review',capture_root=str(cap),scenario_sha256=sha256(cap/'scenario.json'))
            (out/'ledger.jsonl').write_text(json.dumps(row)+'\n')
            with self.assertRaises(ValueError):prepare_session(out,*files)
            for i in range(1,5):
                p=cap/f'CAMERA_{i}';p.mkdir();(p/'a_Intensity.png').write_bytes(b'image')
            p=cap/'LIDAR_5';p.mkdir();(p/'a_Intensity.bin').write_bytes(b'cloud');(p/'a_Instance.txt').write_text('')
            previous,completed=prepare_session(out,*files)
            self.assertEqual(len(previous),1);self.assertIn('a',completed)


if __name__=='__main__':unittest.main()
