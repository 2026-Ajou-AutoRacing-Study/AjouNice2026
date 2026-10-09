#!/usr/bin/env python3
"""Fresh geometry capture; separately guarded runtime with spatial QA recovery."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys
import run_base5_batch_optimized as optimized
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json,sha256


def run(a):
    if a.output.exists() and not a.resume:raise FileExistsError('Fresh output or resume required')
    deck=json.loads(a.manifest.read_text())
    if not deck['config'].get('fresh_geometry_policy'):raise ValueError('Fresh geometry manifest required')
    a.output.mkdir(parents=True,exist_ok=True)
    with (a.output/'.fresh.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard=a.output/'fresh_runtime_contract.json'
        expected=dict(driver_sha256=sha256(__file__),spatial_sha256=sha256(spatial.__file__),
            recovery_sha256=sha256(Path(__file__).with_name('run_base5_capture_campaign_spatial.py')),
            optimized_driver_sha256=sha256(optimized.__file__),manifest_sha256=sha256(a.manifest))
        if guard.exists():
            if json.loads(guard.read_text())!=expected:raise ValueError('Fresh runtime contract mismatch')
        else:
            if (a.output/'batch_contract.json').exists():raise ValueError('Cannot convert old output to fresh runtime')
            atomic_json(guard,expected)
        def recover(source,index,output,stop,heartbeat=lambda root:None):
            output.mkdir(parents=True,exist_ok=True)
            command=[sys.executable,str(Path(__file__).with_name('run_base5_capture_campaign_spatial.py')),
                '--manifest',str(source),'--indices',str(index),'--output',str(output),'--resume']
            seed=output.parent/(output.name+'_seed.json')
            if seed.exists():command.extend(['--seed-capture',str(seed)])
            with (output.parent/(output.name+'.log')).open('a') as log:
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                return optimized.batch.wait_child(child,output,stop,heartbeat,800)
        previous=optimized.batch.recover;optimized.batch.recover=recover
        a.resume=True
        try:return optimized.run(a)
        finally:optimized.batch.recover=previous


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=int,default=64);p.add_argument('--chunk-size',type=int,default=4)
    p.add_argument('--setup-profile',choices=['baseline','static-ready'],default='baseline')
    p.add_argument('--resume',action='store_true');p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--stop-after-chunks',type=int,default=0);p.add_argument('--approve-bulk-capture',action='store_true')
    raise SystemExit(run(p.parse_args()))
