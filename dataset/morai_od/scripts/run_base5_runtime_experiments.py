#!/usr/bin/env python3
"""Serial bounded A/B experiments; never starts another environment's bulk data."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from base5_capture_safety import atomic_json


def run(root):
    baseline=root/'baseline_a1b';deadline=time.monotonic()+1200
    while True:
        status=json.loads((baseline/'status.json').read_text())
        if status['stage'] in ('complete','complete_with_deficit'):break
        if status['stage']=='halted':raise ValueError('Baseline halted; inspect safety, no experiment starts')
        if time.monotonic()>deadline:raise ValueError('Baseline did not finish in bounded wait')
        time.sleep(1)
    cases=[('delay_b1',4,'static-ready'),('combined_c1',16,'static-ready'),
           ('combined_c2',16,'static-ready'),('baseline_a2',4,'baseline')]
    rows=[]
    for name,chunk,profile in cases:
        output=root/name
        command=[sys.executable,str(Path(__file__).with_name('run_base5_batch_optimized.py')),
            '--manifest',str(root/'static64/manifest.json'),'--output',str(output),
            '--limit','64','--chunk-size',str(chunk),'--setup-profile',profile]
        if output.exists():command.append('--resume')
        print('START',name,flush=True);started=time.monotonic()
        with (root/(name+'.log')).open('a') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=1800)
        status=json.loads((output/'status.json').read_text())
        rows.append(dict(name=name,chunk_size=chunk,profile=profile,parent_wall_sec=time.monotonic()-started,
            elapsed_sec=status['elapsed_sec'],accepted=status['accepted_candidate'],review=status['review'],stage=status['stage']))
        atomic_json(root/'experiment_progress.json',rows)
        print('DONE',rows[-1],flush=True)
        if result.returncode or status['stage'] not in ('complete','complete_with_deficit'):
            raise ValueError('Experiment halted; no next MR load')
    native=json.loads((baseline/'status.json').read_text())
    rows.insert(0,dict(name='baseline_a1b',chunk_size=4,profile='baseline',elapsed_sec=native['elapsed_sec'],
        accepted=native['accepted_candidate'],review=native['review'],stage=native['stage']))
    a=sum(r['elapsed_sec'] for r in rows if r['profile']=='baseline')/2
    c=sum(r['elapsed_sec'] for r in rows if r['chunk_size']==16)/2
    atomic_json(root/'comparison.json',dict(cases=rows,baseline_mean_sec=a,combined_mean_sec=c,
        combined_reduction_fraction=1-c/a,all_64_approved=all(r['accepted']==64 and not r['review'] for r in rows),
        limitation='Static64 prospective stratified repeated A/B and B/A; not full 5K, animated or 200-scene long-run qualification.',bulk_promoted=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    run(p.parse_args().root)
