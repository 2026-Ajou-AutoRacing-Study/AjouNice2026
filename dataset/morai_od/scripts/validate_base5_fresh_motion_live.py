#!/usr/bin/env python3
"""Eight newly seeded moving geometries under current Sunny13; NOT training data."""
import argparse
from argparse import Namespace
import copy
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
from run_base5_motion_contract_repair import motion_matches
import run_base5_fresh_batch as fresh


def run(root):
    source=root/'fresh_geometry_registration/animated8/manifest.json';deck=json.loads(source.read_text())
    deck['config']['environment_tag']='qualification_sunny13'
    deck['config']['collection_environment']=dict(weather='sunny',hour=13,policy='Current Sunny13 qualification only, not real sunny11 training samples')
    manifest=root/'fresh_geometry_registration/validation_animated_manifest.json'
    if not manifest.exists():atomic_json(manifest,deck)
    elif json.loads(manifest.read_text())!=deck:raise ValueError('Animated qualification source changed')
    output=root/'fresh_geometry_animated8';a=Namespace(manifest=manifest,output=output,resume=output.exists(),
        limit=8,chunk_size=4,setup_profile='static-ready',runtime_overlap=False,prepare_only=False,
        stop_after_chunks=0,approve_bulk_capture=False)
    code=fresh.run(a)
    if code:raise RuntimeError('New moving geometry safety failure; do not start another load')
    status=json.loads((output/'status.json').read_text());lookup={e['id']:json.loads(Path(e['metadata']).read_text()) for e in deck['scenes']}
    failures=[]
    for line in (output/'selected_ledger.jsonl').read_text().splitlines():
        row=json.loads(line);actual=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
        if not motion_matches(lookup[row['source_cell_id']],actual):failures.append(row['source_cell_id'])
    digest=sha256(output/'selected_ledger.jsonl');raw={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}
    a.resume=True
    if fresh.run(a):raise RuntimeError('New moving geometry terminal resume failed')
    if digest!=sha256(output/'selected_ledger.jsonl') or raw!={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}:raise ValueError('Moving resume added/changed raw')
    proof=dict(accepted=status['accepted_candidate'],review=status['review'],stage=status['stage'],
        primary_gait_and_minimum_mover_failures=failures,additional_completed_resume_raw=0,
        qualified=status['stage']=='complete' and status['accepted_candidate']==8 and not failures,
        actual_environment='sunny13',geometry_seed_environment='sunny11',training_data=False)
    atomic_json(output/'live_motion_validation.json',proof);print(json.dumps(proof,ensure_ascii=False,indent=2));return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);a=p.parse_args();run(a.root)
