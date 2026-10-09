#!/usr/bin/env python3
"""One entrypoint for a qualified fresh environment, resume and finite completion."""
import argparse
import json
from pathlib import Path
from argparse import Namespace

from base5_capture_safety import atomic_json,sha256
import run_base5_fresh_batch as fresh
import run_base5_fresh_completion as completion


def preflight(release,environment,confirmed=None):
    doc=json.loads(Path(release).read_text())
    if not doc.get('collector_qualified'):raise ValueError('Collector qualification not complete')
    for path,digest in doc['runtime_hashes'].items():
        if sha256(path)!=digest:raise ValueError('Qualified runtime changed: '+path)
    suite=json.loads(Path(doc['suite']).read_text())
    choices=[e for e in suite['environments'] if e['environment']==environment and environment!='sunny13']
    if len(choices)!=1:raise ValueError('Choose one of the five remaining environments')
    env=choices[0]
    if sha256(env['manifest'])!=env['manifest_sha256']:raise ValueError('Frozen environment manifest changed')
    if confirmed is not None and confirmed!=environment:raise ValueError('UI confirmation does not match requested environment')
    return doc,env


def run(a):
    doc,env=preflight(a.release,a.environment,a.confirmed_environment)
    if a.complete and not a.prepare_only and not doc.get('completion_live_qualified'):
        raise ValueError('Generic completion live qualification pending; do not start unattended --complete')
    if not a.prepare_only and a.confirmed_environment!=a.environment:
        raise ValueError('Set MR UI weather/hour, then explicitly pass --confirmed-environment '+a.environment)
    output=Path(env['capture_output']);manifest=Path(env['manifest'])
    if not a.prepare_only:
        output.mkdir(parents=True,exist_ok=True)
        atomic_json(output/'operator_environment_confirmation.json',dict(environment=a.environment,
            weather=env['weather'],hour=env['hour'],manifest_sha256=env['manifest_sha256'],
            policy='Operator confirmation, not simulator auto-verification; valid only while UI stays unchanged'))
    args=Namespace(manifest=manifest,output=output,resume=True,limit=5000,
        chunk_size=doc['recommended_chunk_size'],setup_profile=doc['recommended_setup_profile'],
        runtime_overlap=doc['recommended_runtime_overlap'],prepare_only=a.prepare_only,
        # Native bulk flag defines the 5000-slot contract even in CPU-only
        # preparation. prepare_only still cannot launch a ROS child.
        stop_after_chunks=a.stop_after_chunks,approve_bulk_capture=True)
    code=fresh.run(args)
    if code or a.prepare_only or not a.complete:return code
    status=json.loads((output/'status.json').read_text())
    if status['stage'] not in completion.TERMINAL:return 0
    completed=output.parent/(output.name+'_completed')
    code=completion.execute(Namespace(manifest=manifest,baseline=output,output=completed,run=True,
        resume=completed.exists(),attempts=3,time_budget_sec=1800))
    if code:return code
    print('CANONICAL LEDGER:',completed/'selected_ledger.jsonl')
    print('Finite completion may still end with an explicit deficit; never skip safety or relabel missing raw.')
    return 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--release',type=Path,required=True)
    p.add_argument('--environment',choices=['sunny11','sunny15','foggy11','foggy13','foggy15'],required=True)
    p.add_argument('--confirmed-environment',help='Explicit operator confirmation of current MR UI weather/time')
    p.add_argument('--prepare-only',action='store_true',help='CPU/contract preflight without ROS or UI changes')
    p.add_argument('--stop-after-chunks',type=int,default=0)
    p.add_argument('--complete',action='store_true',help='After terminal capture, bounded family replacement of remaining deficits')
    raise SystemExit(run(p.parse_args()))
