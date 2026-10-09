#!/usr/bin/env python3
"""Guarded experimental overlapping preparation; separate outputs only."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys

import run_base5_batch_campaign as batch
from base5_capture_safety import atomic_json,sha256


def run(a):
    deck=json.loads(a.manifest.read_text());ids=list(range(min(a.limit,len(deck['scenes']))))
    if a.output.exists() and not a.resume:raise FileExistsError('Fresh output or explicit resume')
    a.output.mkdir(parents=True,exist_ok=True)
    with (a.output/'.overlap.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard=a.output/'overlap_contract.json'
        expected=dict(driver_sha256=sha256(__file__),chunk_size=a.chunk_size,
            overlap_sha256=sha256(Path(__file__).with_name('run_base5_capture_overlap.py')),
            delay_sha256=sha256(Path(__file__).with_name('run_base5_capture_optimized.py')),
            legacy=batch.campaign.contract(a.manifest,ids,2))
        if guard.exists():
            if json.loads(guard.read_text())!=expected:raise ValueError('Overlap runtime contract mismatch')
        else:
            if (a.output/'batch_contract.json').exists():raise ValueError('Cannot adapt legacy output')
            atomic_json(guard,expected)
        def child(manifest,root,stop,heartbeat):
            root.mkdir(parents=True,exist_ok=True);n=len(json.loads(manifest.read_text())['scenes'])
            cmd=[sys.executable,str(Path(__file__).with_name('run_base5_capture_overlap.py')),
                '--manifest',str(manifest),'--output',str(root),'--shared-ros-batch','--limit',str(n)]
            with (root.parent/'batch.log').open('a') as log:
                worker=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                return batch.wait_child(worker,root,stop,heartbeat,100*n)
        old=batch.run_child;batch.run_child=child
        try:return batch.execute(a.manifest,ids,a.output,a.chunk_size,True,a.stop_after_chunks,a.approve_bulk_capture,
            prepare_only=getattr(a,'prepare_only',False))
        finally:batch.run_child=old


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int,default=64)
    p.add_argument('--chunk-size',type=int,default=4);p.add_argument('--stop-after-chunks',type=int,default=0)
    p.add_argument('--resume',action='store_true');p.add_argument('--approve-bulk-capture',action='store_true')
    p.add_argument('--prepare-only',action='store_true')
    raise SystemExit(run(p.parse_args()))
