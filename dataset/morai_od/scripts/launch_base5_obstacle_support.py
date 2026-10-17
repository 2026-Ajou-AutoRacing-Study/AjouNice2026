#!/usr/bin/env python3
"""Prepare a frozen support release; bulk capture requires explicit --run.

Default is CPU-only, including resume. This entrypoint is not used by the
experiment supervisor to start a 5K collection.
"""
import argparse
from argparse import Namespace
import json
from pathlib import Path
import shutil

from base5_capture_safety import atomic_json,sha256
from base5_front_release_inputs import verify_hashes
import run_base5_fresh_batch as fresh
import run_base5_fresh_completion as completion


def preflight(path,environment):
    doc=json.loads(path.read_text())
    if doc.get('schema')!='base5_obstacle_support_release_v1':raise ValueError('Support release required')
    if environment not in doc.get('qualified_environments',[]):raise ValueError('Environment live qualification pending')
    for source,digest in doc['runtime_hashes'].items():
        if sha256(source)!=digest:raise ValueError('Frozen support runtime changed: '+source)
    if 'input_hashes' in doc:
        verify_hashes(doc['input_hashes'], 'Frozen support input')
    env=doc['environments'][environment]
    if sha256(env['manifest'])!=env['manifest_sha256']:raise ValueError('Frozen support manifest changed')
    deck=json.loads(Path(env['manifest']).read_text())
    if deck['count']!=5000 or not deck['config'].get('obstacle_support_policy'):
        raise ValueError('Qualified 5K support plan required')
    from base5_layout_split import validate_manifest_overlay
    validate_manifest_overlay(deck)
    return doc,env


def run(a):
    doc,env=preflight(a.release,a.environment)
    if a.run and not doc.get('bulk_ready'):
        raise ValueError('Bulk qualification incomplete: '+ '; '.join(doc.get('bulk_blockers',[])))
    if a.run and a.confirmed_environment!=a.environment:
        raise ValueError('Set MR UI and pass --confirmed-environment matching requested environment')
    if a.complete and not doc.get('completion_live_qualified'):
        raise ValueError('Completion qualification pending')
    output=a.output or Path(env['capture_output']);manifest=Path(env['manifest'])
    if a.run:
        ancestor=output
        while not ancestor.exists():ancestor=ancestor.parent
        free=shutil.disk_usage(ancestor).free/2**30
        status=output/'status.json'
        done=json.loads(status.read_text()).get('accepted_candidate',0) if status.exists() else 0
        required=doc['operational_storage_budget_gib']*max(0,5000-done)/5000+10
        if free<required:raise ValueError(f'Insufficient headroom {free:.1f}GiB / {required:.1f}GiB')
        output.mkdir(parents=True,exist_ok=True)
        atomic_json(output/'operator_environment_confirmation.json',dict(environment=a.environment,
            manual_confirmation=True,manifest_sha256=env['manifest_sha256']))
    result=fresh.run(Namespace(manifest=manifest,output=output,resume=True,limit=5000,
        chunk_size=16,setup_profile='static-ready',runtime_overlap=False,prepare_only=not a.run,
        stop_after_chunks=a.stop_after_chunks,approve_bulk_capture=True))
    if result or not a.run or not a.complete:return result
    if json.loads((output/'status.json').read_text())['stage'] not in completion.TERMINAL:return 0
    completed=output.with_name(output.name+'_completed')
    return completion.execute(Namespace(manifest=manifest,baseline=output,output=completed,run=True,
        resume=completed.exists(),attempts=3,time_budget_sec=1800))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--release',type=Path,required=True)
    p.add_argument('--environment',required=True);p.add_argument('--output',type=Path)
    p.add_argument('--run',action='store_true',help='Explicitly start/resume 5K bulk collection')
    p.add_argument('--confirmed-environment');p.add_argument('--complete',action='store_true')
    p.add_argument('--stop-after-chunks',type=int,default=0);a=p.parse_args();raise SystemExit(run(a))
