#!/usr/bin/env python3
"""New-output-only quality campaign retaining continuous-anchor replacement plans."""
import argparse
import fcntl
import json
from pathlib import Path
import base5_spatial_diversity as spatial
import run_base5_capture_campaign as campaign
from base5_capture_safety import atomic_json,sha256


def run(a):
    if a.output.exists() and not a.resume:raise FileExistsError('Fresh spatial output or resume required')
    deck=json.loads(a.manifest.read_text())
    ids=[int(i) for i in a.indices.split(',')] if a.indices else list(range(min(a.limit,len(deck['scenes']))))
    a.output.mkdir(parents=True,exist_ok=True)
    with (a.output/'.spatial.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        guard=a.output/'spatial_contract.json'
        expected=dict(driver_sha256=sha256(__file__),adapter_sha256=sha256(spatial.__file__),
            legacy=campaign.contract(a.manifest,ids,a.quota_rounds))
        if guard.exists():
            if json.loads(guard.read_text())!=expected:raise ValueError('Spatial campaign contract mismatch')
        else:
            if (a.output/'campaign_contract.json').exists():raise ValueError('Cannot adapt old campaign')
            atomic_json(guard,expected)
        with spatial.installed():
            return campaign.execute(a.manifest,ids,a.output,True,a.quota_rounds,0,a.approve_bulk_capture,a.seed_capture)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--indices');p.add_argument('--limit',type=int,default=64)
    p.add_argument('--resume',action='store_true');p.add_argument('--approve-bulk-capture',action='store_true')
    p.add_argument('--quota-rounds',type=int,default=2);p.add_argument('--seed-capture',type=Path)
    raise SystemExit(run(p.parse_args()))
