import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_batch_campaign as batch
import base5_campaign_replenishment as replenish


class ReplenishmentTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.output=self.root/'collection'
        self.source=self.root/'manifest.json';self.records={};self.calls=[]
        scenes=[]
        for i,ident in enumerate('abc'):
            scenario=self.root/(ident+'.json');meta=self.root/(ident+'_metadata.json')
            batch.atomic_json(scenario,dict(test_actor=ident))
            batch.atomic_json(meta,dict(spec=dict(index=i,family='obstacle',role='left',distance_bounds=[3,5]),
                split='val',retry=0,scene_group_id='group_'+ident,
                runtime_actors=[dict(data_id=100+i,motion='static')]))
            scenes.append(dict(id=ident,scenario=str(scenario),scenario_sha256=batch.sha256(scenario),metadata=str(meta)))
        batch.atomic_json(self.source,dict(config=dict(environment='sunny13'),scenes=scenes))
        patches=[patch.object(batch.campaign,'contract',return_value={}),
            patch.object(batch,'run_child',side_effect=self.initial_capture),
            patch.object(batch,'inspect_batch',side_effect=lambda root,manifest:self.records[str(root)]),
            patch.object(batch,'validate_collision_failure'),
            patch.object(batch,'verify_recovery',return_value=0),
            patch.object(batch,'recover',side_effect=self.capture),
            patch.object(batch.campaign.loop,'replacement',side_effect=self.replacement)]
        for p in patches:p.start();self.addCleanup(p.stop)
        self.assertEqual(batch.execute(self.source,[0,1,2],self.output,1),0,
            (self.output/'status.json').read_text())
        self.initial_ledger=(self.output/'selected_ledger.jsonl').read_bytes()
        self.source_bytes=self.source.read_bytes()

    def initial_capture(self,manifest,root,stop,heartbeat):
        root.mkdir(exist_ok=True);batch.atomic_json(root/'session_contract.json',{})
        ident=replenish.read(manifest)['scenes'][0]['id']
        self.records[str(root)]=[dict(id=ident,status='failed',failure_kind='physical_collision')] if ident in 'ab' else [dict(id='c',status='accepted_candidate',family='obstacle')]
        return 1 if ident in 'ab' else 0

    def replacement(self,source,output):
        deck=replenish.read(source);entry=deck['scenes'][0];meta=replenish.read(entry['metadata'])
        output.mkdir();meta['retry']+=1;entry['id']+='_replace'
        scenario=output/'scene.json';metadata=output/'metadata.json'
        batch.atomic_json(scenario,dict(retry=meta['retry']));batch.atomic_json(metadata,meta)
        entry.update(metadata=str(metadata),scenario=str(scenario),scenario_sha256=batch.sha256(scenario))
        manifest=output/'manifest.json';batch.atomic_json(manifest,deck)
        return manifest

    def capture(self,manifest,index,child,stop,heartbeat):
        entry=replenish.read(manifest)['scenes'][0];ident=entry['source_cell_id'];self.calls.append(ident)
        child.mkdir(parents=True,exist_ok=True);root=child/'raw';root.mkdir(exist_ok=True)
        batch.atomic_json(root/'metadata.json',replenish.read(entry['metadata']))
        row=dict(id=entry['id'],source_cell_id=entry['id'],status='accepted_candidate',family='obstacle',capture_root=str(root))
        batch.atomic_json(child/'quota.json',dict(cells=[]))
        (child/'selected_ledger.jsonl').write_text(json.dumps(row)+'\n')
        return 0

    def run_campaign(self,**kw):
        return batch.execute(self.source,[0,1,2],self.output,1,resume=True,**kw)

    def test_only_deficits_merge_and_completed_resume_is_read_only(self):
        self.assertEqual(self.run_campaign(replenish=True),0)
        self.assertEqual(self.calls,['a','b'])
        rows=batch.load_ledger(self.output/'selected_ledger.jsonl')
        self.assertEqual([r['source_cell_id'] for r in rows],list('abc'))
        self.assertEqual(rows[-1],json.loads(self.initial_ledger))
        self.assertEqual([r['scene_group_id'] for r in rows[:2]],['group_a','group_b'])
        self.assertEqual(replenish.read(self.output/'quota.json')['deficit'],0)
        before=(self.output/'selected_ledger.jsonl').stat();elapsed=replenish.read(self.output/'status.json')['elapsed_sec']
        self.assertEqual(self.run_campaign(),0)
        self.assertEqual(self.run_campaign(replenish=True),0)
        self.assertEqual(self.calls,['a','b'])
        after=(self.output/'selected_ledger.jsonl').stat()
        self.assertEqual((before.st_ino,before.st_mtime_ns),(after.st_ino,after.st_mtime_ns))
        self.assertEqual(replenish.read(self.output/'status.json')['elapsed_sec'],elapsed)
        self.assertEqual(self.source.read_bytes(),self.source_bytes)

    def test_pause_resume_keeps_completed_cell(self):
        self.assertEqual(self.run_campaign(replenish=True,stop_after_chunks=1),0)
        self.assertEqual(self.calls,['a'])
        self.assertEqual(replenish.read(self.output/'status.json')['stage'],'paused')
        self.assertEqual(self.run_campaign(),0);self.assertEqual(self.calls,['a'])
        self.assertEqual(self.run_campaign(replenish=True),0)
        self.assertEqual(self.calls,['a','b'])

    def test_pending_child_resumes_in_same_folder_without_new_placement(self):
        self.assertEqual(self.run_campaign(replenish=True,stop_after_chunks=1),0)
        state=replenish.read(self.output/'replenishment/state.json')
        # Power loss after child completion, before journal commit: the child
        # evidence exists, but the pending attempt is authoritative on restart.
        state['attempts'][0]['result']=None
        batch.atomic_json(self.output/'replenishment/state.json',state)
        def verify_existing(manifest,index,child,stop,heartbeat):
            if (child/'selected_ledger.jsonl').exists():return 0
            return self.capture(manifest,index,child,stop,heartbeat)
        with patch.object(batch,'recover',side_effect=verify_existing),patch.object(replenish,'prepare',wraps=replenish.prepare) as prepare:
            self.assertEqual(self.run_campaign(replenish=True),0)
            self.assertEqual(prepare.call_count,1) # only b; a uses existing evidence
        self.assertEqual(self.calls,['a','b'])

    def test_replay_repairs_stale_publication_without_capture(self):
        self.assertEqual(self.run_campaign(replenish=True),0)
        expected=(self.output/'selected_ledger.jsonl').read_bytes()
        (self.output/'selected_ledger.jsonl').write_bytes(self.initial_ledger)
        batch.atomic_json(self.output/'quota.json',dict(deficit=2))
        self.assertEqual(self.run_campaign(),0)
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),expected)
        self.assertEqual(self.calls,['a','b'])

    def test_bounded_failures_distinct_geometry_and_explicit_cap_extension(self):
        def miss(manifest,index,child,stop,heartbeat):
            entry=replenish.read(manifest)['scenes'][0];self.calls.append(entry['source_cell_id'])
            child.mkdir(parents=True,exist_ok=True)
            (child/'selected_ledger.jsonl').write_text('')
            batch.atomic_json(child/'quota.json',dict(cells=[dict(reasons=['not_visible'])]))
            return 0
        with patch.object(batch,'recover',side_effect=miss):
            self.assertEqual(self.run_campaign(replenish=True),0)
            self.assertEqual(self.calls,['a','b','a','b'])
            state=replenish.read(self.output/'replenishment/state.json')
            self.assertEqual([replenish.read(replenish.read(a['manifest'])['scenes'][0]['metadata'])['retry'] for a in state['attempts']],[1,1,2,2])
            before=replenish.read(self.output/'status.json')['elapsed_sec']
            self.assertEqual(self.run_campaign(replenish=True),0)
            self.assertEqual(len(self.calls),4)
            self.assertEqual(replenish.read(self.output/'status.json')['elapsed_sec'],before)
            self.assertEqual(self.run_campaign(replenish=True,replenish_rounds=3),0)
            self.assertEqual(len(self.calls),6)
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),self.initial_ledger)
        self.assertEqual(replenish.read(self.output/'status.json')['stage'],'complete_with_deficit')

    def test_stop_before_capture_and_pause_marker(self):
        for marker in ('STOP','PAUSE'):
            (self.output/marker).touch()
            self.assertEqual(self.run_campaign(replenish=True),0)
            self.assertEqual(self.calls,[])
            (self.output/marker).unlink()
        self.assertEqual(self.run_campaign(replenish=True),0)

    def test_unsafe_child_halts_no_next_load_and_no_force_resume(self):
        with patch.object(batch,'recover',return_value=1):
            self.assertEqual(self.run_campaign(replenish=True),1)
        self.assertEqual(replenish.read(self.output/'status.json')['stage'],'halted')
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),self.initial_ledger)
        self.assertEqual(self.run_campaign(replenish=True),1)
        self.assertEqual(self.calls,[])

    def test_unavailable_placement_is_bounded_deficit_not_unsafe_halt(self):
        with patch.object(batch.campaign.loop,'replacement',side_effect=ValueError('Same-cell/split replacement unavailable')):
            self.assertEqual(self.run_campaign(replenish=True),0)
            self.assertEqual(self.run_campaign(replenish=True),0)
        self.assertEqual(self.calls,[])
        state=replenish.read(self.output/'replenishment/state.json')
        self.assertEqual(state['stage'],'complete_with_deficit')
        self.assertEqual(len(state['attempts']),4)
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),self.initial_ledger)

    def test_identity_environment_and_scenario_tamper_rejected(self):
        entry=replenish.read(self.source)['scenes'][0];folder=self.root/'placement_test';folder.mkdir()
        manifest=replenish.prepare(entry,replenish.read(self.source),None,folder)
        original=replenish.read(manifest);meta_path=Path(original['scenes'][0]['metadata']);meta=replenish.read(meta_path)
        for key,value in [('split','train'),('scene_group_id','other_group'),('spec',dict(changed=True))]:
            altered=copy.deepcopy(meta);altered[key]=value;batch.atomic_json(meta_path,altered)
            with self.assertRaises(ValueError):replenish.check_placement(entry,manifest,dict(environment='sunny13'))
        batch.atomic_json(meta_path,meta)
        altered=copy.deepcopy(original);altered['config']['environment']='foggy11';batch.atomic_json(manifest,altered)
        with self.assertRaises(ValueError):replenish.check_placement(entry,manifest,dict(environment='sunny13'))
        batch.atomic_json(manifest,original);Path(original['scenes'][0]['scenario']).write_text('{}')
        with self.assertRaises(ValueError):replenish.check_placement(entry,manifest,dict(environment='sunny13'))

    def test_merged_evidence_tamper_never_overwrites_selection(self):
        self.assertEqual(self.run_campaign(replenish=True),0)
        before=(self.output/'selected_ledger.jsonl').read_bytes()
        state=replenish.read(self.output/'replenishment/state.json')
        state['attempts'][0]['result']['selected']['id']='forged'
        batch.atomic_json(self.output/'replenishment/state.json',state)
        self.assertEqual(self.run_campaign(),1)
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),before)
        self.assertEqual(self.calls,['a','b'])

    def test_preconditions_no_initial_or_incomplete_or_changed_runtime(self):
        with self.assertRaises(ValueError):
            batch.execute(self.source,[0,1,2],self.output,1,replenish=True)
        batch.atomic_json(self.output/'status.json',dict(stage='paused'))
        self.assertEqual(self.run_campaign(replenish=True),1)
        self.assertEqual(self.calls,[])
        batch.atomic_json(self.output/'status.json',dict(stage='complete_with_deficit',elapsed_sec=1))
        contract=replenish.read(self.output/'batch_contract.json');contract['batch_driver_sha256']='old_runtime'
        batch.atomic_json(self.output/'batch_contract.json',contract)
        self.assertEqual(self.run_campaign(replenish=True),1)
        self.assertEqual(self.calls,[])

    def test_completed_manifest_missing_is_not_silently_recreated(self):
        manifest=self.output/'chunk_0000/manifest.json';manifest.unlink()
        self.assertEqual(self.run_campaign(replenish=True),1)
        self.assertFalse(manifest.exists())
        self.assertEqual((self.output/'selected_ledger.jsonl').read_bytes(),self.initial_ledger)
        self.assertEqual(self.calls,[])


if __name__=='__main__':unittest.main()
