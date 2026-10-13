#!/usr/bin/env python3
"""Separately guarded post-collection batch experiment. Legacy code is unedited."""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import sys

import run_base5_batch_campaign as batch
from base5_capture_safety import atomic_json, sha256


def run(args):
    deck = json.loads(args.manifest.read_text())
    indices = list(range(min(args.limit,len(deck['scenes']))))
    if args.output.exists() and not args.resume: raise FileExistsError('Fresh output or explicit resume required')
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'.optimized.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard = args.output/'optimized_contract.json'
        contract = dict(schema=1,manifest_sha256=sha256(args.manifest),
            driver_sha256=sha256(__file__),
            adapter_sha256=sha256(Path(__file__).with_name('run_base5_capture_optimized.py')),
            setup_profile=args.setup_profile,chunk_size=args.chunk_size,
            legacy=batch.campaign.contract(args.manifest,indices,2))
        if guard.exists():
            if json.loads(guard.read_text()) != contract: raise ValueError('Optimized runtime guard mismatch')
        else:
            if (args.output/'batch_contract.json').exists(): raise ValueError('Cannot attach adapter to legacy output')
            atomic_json(guard,contract)
        def child(manifest,root,stop,heartbeat):
            root.mkdir(parents=True,exist_ok=True)
            count = len(json.loads(manifest.read_text())['scenes'])
            command = [sys.executable,str(Path(__file__).with_name('run_base5_capture_optimized.py')),
                '--setup-profile',args.setup_profile,'--manifest',str(manifest),
                '--output',str(root),'--shared-ros-batch','--limit',str(count)]
            with (root.parent/'batch.log').open('a') as log:
                worker = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                return batch.wait_child(worker,root,stop,heartbeat,100*count)
        old = batch.run_child; batch.run_child = child
        try:
            # The wrapper already created its guarded directory. The native
            # driver must accept that empty directory, without weakening our
            # explicit existing-output check above.
            return batch.execute(args.manifest,indices,args.output,args.chunk_size,True,
                args.stop_after_chunks,args.approve_bulk_capture,prepare_only=args.prepare_only)
        finally: batch.run_child = old


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=int,default=64);p.add_argument('--chunk-size',type=int,default=4)
    p.add_argument('--setup-profile',choices=['baseline','static-ready'],default='baseline')
    p.add_argument('--resume',action='store_true');p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--stop-after-chunks',type=int,default=0)
    p.add_argument('--approve-bulk-capture',action='store_true')
    raise SystemExit(run(p.parse_args()))
