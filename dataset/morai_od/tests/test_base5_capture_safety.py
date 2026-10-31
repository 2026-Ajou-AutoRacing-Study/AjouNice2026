import json
import sys
import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_safety import collision_window,load_ledger,prepare_session,archive_incomplete,runtime_presence_review,confirm_park,park_request_fields


class CaptureSafetyTest(unittest.TestCase):
    def test_shutdown_requires_explicit_valid_park_receipt(self):
        from base5_capture_safety import verified_shutdown
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'shutdown_safety.json'
            self.assertFalse(verified_shutdown(path))
            for value in ('broken','null','[]','{}','{"park_verified":1}',
                          '{"park_verified":"true"}','{"park_verified":false}'):
                path.write_text(value)
                self.assertFalse(verified_shutdown(path),value)
            path.write_text('{"park_verified":true}')
            self.assertTrue(verified_shutdown(path))

    def test_repeated_pose_failures_preserve_both_original_evidence_hashes(self):
        from base5_capture_safety import write_scene_pose_failure,sha256
        with tempfile.TemporaryDirectory() as temp:
            a=write_scene_pose_failure(temp,'same_scene','scenario',{'no_capture_requested':True},[{'t':1}])
            original=Path(a['failure_evidence']).read_bytes()
            b=write_scene_pose_failure(temp,'same_scene','scenario',{'no_capture_requested':True},[{'t':2}])
            self.assertNotEqual(a['failure_evidence'],b['failure_evidence'])
            self.assertEqual(Path(a['failure_evidence']).read_bytes(),original)
            for result in (a,b):
                self.assertEqual(sha256(result['failure_evidence']),result['failure_evidence_sha256'])
                self.assertEqual(result['failure_kind'],'collision_scene_pose_unverified')

    def test_front_support_helper_is_pinned_only_for_explicit_candidate(self):
        import base5_capture_safety as safety
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);manifest=root/'manifest.json';network=root/'network';sensor=root/'sensor'
            manifest.write_text(json.dumps(dict(config=dict(obstacle_front_support_policy='v151'))))
            network.write_text('{}');sensor.write_text('{}')
            safety.prepare_session(root/'out',manifest,network,sensor)
            contract=json.loads((root/'out'/'session_contract.json').read_text())
            self.assertIn('base5_obstacle_front_support.py',contract['safety_code_sha256'])
            real_hash=safety.sha256
            def changed(path):
                return 'changed' if Path(path).name=='base5_obstacle_front_support.py' else real_hash(path)
            with patch.object(safety,'sha256',side_effect=changed):
                with self.assertRaisesRegex(ValueError,'contract mismatch'):
                    safety.prepare_session(root/'out',manifest,network,sensor)
            manifest.write_text('{}')
            safety.prepare_session(root/'legacy',manifest,network,sensor)
            legacy=json.loads((root/'legacy'/'session_contract.json').read_text())
            self.assertNotIn('base5_obstacle_front_support.py',legacy['safety_code_sha256'])

    def test_scene_pose_mismatch_preserves_evidence_and_still_halts(self):
        from base5_capture_safety import scene_collision_epoch,ScenePoseUnverifiedError
        peer=dict(unique_id=8500)
        ego=dict(unique_id=0,position=dict(x=2.,y=3.,z=28.),heading=195.)
        rows=[dict(t=2.,message=dict(collision_object=[peer],collision_objecta=[ego]))]
        with self.assertRaises(ScenePoseUnverifiedError) as caught:
            scene_collision_epoch(rows,1.,dict(x=2.,y=3.,yaw=15.))
        evidence=caught.exception.evidence
        self.assertIsInstance(caught.exception,RuntimeError)
        self.assertTrue(evidence['no_capture_requested'])
        self.assertEqual(evidence['collision_samples'],rows)
        self.assertEqual(evidence['ego_observations'][0]['position_error_m'],0.)
        self.assertEqual(evidence['ego_observations'][0]['raw_heading_minus_requested_deg'],-180.)
        self.assertEqual(evidence['heading_correction_deg'],-1.347)
        with self.assertRaises(ScenePoseUnverifiedError):
            scene_collision_epoch([],1.,dict(x=2.,y=3.,yaw=15.))

    def test_batch_mode_is_part_of_resume_contract(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);files=[]
            for name in ('manifest','network','sensor'):
                path=root/name;path.write_text('{}');files.append(path)
            prepare_session(root/'out',*files,runner_options=dict(batch_reuse_network=True))
            with self.assertRaises(ValueError):
                prepare_session(root/'out',*files,runner_options=dict(batch_reuse_network=False))
            options=dict(shared_ros_batch=True,selected_scene_ids=['a','b'])
            prepare_session(root/'plan',*files,runner_options=options)
            with self.assertRaises(ValueError):
                prepare_session(root/'plan',*files,runner_options=dict(shared_ros_batch=True,selected_scene_ids=['b','a']))
            (root/'out'/'ledger.jsonl').write_text(json.dumps(dict(id='a',status='accepted_candidate',between_scene_park=dict(park_verified=False)))+'\n')
            with self.assertRaisesRegex(ValueError,'boundary Park'):
                prepare_session(root/'out',*files,runner_options=dict(batch_reuse_network=True))

    def test_exit_park_changes_only_gear_not_control_mode(self):
        self.assertEqual(park_request_fields(),dict(option=2,gear=1))

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
