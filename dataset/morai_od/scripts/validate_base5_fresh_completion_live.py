#!/usr/bin/env python3
"""Small real negative-control parent and finite completion; never training data.

Run serially AFTER other MR qualification. Legacy failed payloads are explicitly
negative-control fixtures, not claimed newly sampled environment geometry.
"""
import argparse
from argparse import Namespace
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
import run_base5_fresh_batch as batch
import run_base5_fresh_completion as completion


def run(source,output):
    if output.exists():raise FileExistsError('Fresh validation root required')
    original=json.loads(source.read_text());lookup={e['id']:e for e in original['scenes']}
    ids=('b5v2_00437','b5v2_01584','b5v2_01760','b5v2_02166')
    deck=copy.deepcopy(original);config=deck['config'];config['environment_tag']='qualification_sunny13'
    config['collection_environment']=dict(weather='sunny',hour=13,policy='Current UI qualification only; NEVER use fixtures in 30K training')
    config['fresh_geometry_policy']=dict(policy='negative-control-legacy-failed-payloads',source_sha256=sha256(source))
    output.mkdir(parents=True);entries=[]
    for ident in ids:
        entry=copy.deepcopy(lookup[ident]);meta=json.loads(Path(entry['metadata']).read_text())
        meta['scene_group_id']='negative_control_v141:'+ident
        meta['negative_control_fixture']=dict(source_manifest_sha256=sha256(source),source_cell=ident,
            policy='Actual formerly deficient payload, not manufactured ledger failure or training geometry')
        mp=output/(ident+'_metadata.json');atomic_json(mp,meta)
        entry['metadata']=str(mp.resolve());entry['scene_group_id']=meta['scene_group_id'];entries.append(entry)
    deck['scenes']=entries;deck['count']=len(entries);manifest=output/'manifest.json';atomic_json(manifest,deck)
    parent=output/'parent'
    code=batch.run(Namespace(manifest=manifest,output=parent,resume=False,limit=4,chunk_size=4,
        setup_profile='static-ready',runtime_overlap=False,prepare_only=False,stop_after_chunks=0,approve_bulk_capture=False))
    if code:raise RuntimeError('Negative-control parent safety failure; no next load')
    before={n:sha256(parent/n) for n in ('selected_ledger.jsonl','quota.json','status.json')}
    result=output/'completed';a=Namespace(manifest=manifest,baseline=parent,output=result,resume=False,
        run=True,attempts=3,time_budget_sec=600)
    code=completion.execute(a)
    if code:raise RuntimeError('Completion safety failure; inspect evidence')
    state=json.loads((result/'state.json').read_text());status=json.loads((result/'status.json').read_text())
    if status['stage'] not in completion.TERMINAL:raise ValueError('Validation exhausted time before terminal; no qualification')
    selection=sha256(result/'selected_ledger.jsonl');raw={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}
    a.resume=True
    if completion.execute(a):raise RuntimeError('Completed resume proof failed')
    if selection!=sha256(result/'selected_ledger.jsonl') or raw!={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}:raise ValueError('Completed resume added/changed capture')
    if before!={n:sha256(parent/n) for n in before}:raise ValueError('Parent modified by completion')
    proof=dict(parent_review=json.loads((parent/'status.json').read_text())['review'],
        initial_accepted=status['initial_accepted'],final_accepted=status['accepted_candidate'],
        terminal_stage=status['stage'],deficit=status['review'],attempts=len(state['attempts']),
        additional_completed_resume_raw=0,parent_preserved=True,
        fixture='Four real legacy failed thin-object payloads under Sunny13; never fake ledger failures or training samples',
        limitations='Static completion native roundtrip only; animated preparation separately regression-tested and primary gait/minimum modes unit-tested')
    atomic_json(output/'live_validation.json',proof);print(json.dumps(proof,ensure_ascii=False,indent=2));return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.manifest,a.output)
