#!/usr/bin/env python3
"""Validate finite completion on an ACTUAL fresh pilot deficit, not fake failures."""
import argparse
from argparse import Namespace
import json
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
import run_base5_fresh_completion as completion


def run(manifest,baseline,output):
    if output.exists():raise FileExistsError('Fresh qualification output required')
    before={n:sha256(baseline/n) for n in ('selected_ledger.jsonl','quota.json','status.json')}
    original=json.loads((baseline/'status.json').read_text())
    if original['stage']!='complete_with_deficit' or not original['review']:
        raise ValueError('A real terminal fresh pilot deficit is required')
    a=Namespace(manifest=manifest,baseline=baseline,output=output,run=True,resume=False,
        attempts=3,time_budget_sec=600)
    if completion.execute(a):raise RuntimeError('Completion safety failure; inspect before another load')
    state=json.loads((output/'state.json').read_text());status=json.loads((output/'status.json').read_text())
    if status['stage'] not in completion.TERMINAL:raise ValueError('Finite run not terminal; no qualification')
    saved={n:sha256(output/n) for n in ('state.json','selected_ledger.jsonl','status.json')}
    raw={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}
    a.resume=True
    if completion.execute(a):raise RuntimeError('Terminal completion verification failed')
    if saved!={n:sha256(output/n) for n in saved}:raise ValueError('Completed resume changed journal/selection/time')
    if raw!={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}:raise ValueError('Completed resume added capture')
    if before!={n:sha256(baseline/n) for n in before}:raise ValueError('Successful parent was modified')
    proof=dict(parent_review=original['review'],initial_accepted=original['accepted_candidate'],
        final_accepted=status['accepted_candidate'],deficit=status['review'],terminal_stage=status['stage'],
        attempts=len(state['attempts']),additional_completed_resume_raw=0,parent_preserved=True,
        fixture='Actual independently generated native pilot deficit; not manufactured or training data',
        selection=str(output/'selected_ledger.jsonl'),actual_environment='sunny13',training_data=False,
        limitations='Native finite completion roundtrip; movement source gait/minima require the separate completed-motion audit')
    atomic_json(output/'live_validation.json',proof);print(json.dumps(proof,ensure_ascii=False,indent=2));return proof


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.manifest,a.baseline,a.output)
