#!/usr/bin/env python3
"""Verify terminal bounded pilot resume without adding sensor captures."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from base5_capture_safety import atomic_json,sha256


def snapshot(roots):
    native={}
    for root in roots:
        for path in root.rglob('*_Intensity.bin'):
            st=path.stat();native[str(path.resolve())]=[st.st_size,st.st_ino]
    return native


def verify(a):
    roots=[a.baseline,a.completion]
    before=snapshot(roots)
    ledgers={str(p/'selected_ledger.jsonl'):sha256(p/'selected_ledger.jsonl') for p in roots}
    scripts=Path(__file__).parent
    commands=[
        [sys.executable,str(scripts/'run_base5_fresh_batch.py'),'--manifest',str(a.manifest),
         '--output',str(a.baseline),'--limit','64','--chunk-size','16','--setup-profile','static-ready','--resume'],
        [sys.executable,str(scripts/'run_base5_fresh_completion.py'),'--manifest',str(a.manifest),
         '--baseline',str(a.baseline),'--output',str(a.completion),'--run','--resume','--attempts','3','--time-budget-sec','900']]
    for command in commands:subprocess.run(command,check=True,timeout=120)
    after=snapshot(roots)
    stable=before==after and all(sha256(p)==digest for p,digest in ledgers.items())
    result=dict(passed=stable,additional_native_captures=len(set(after)-set(before)),
        native_files_before=len(before),native_files_after=len(after),ledgers_unchanged=all(sha256(p)==v for p,v in ledgers.items()),
        commands=commands,ledger_sha256=ledgers)
    atomic_json(a.output,result)
    if not stable:raise ValueError('Completed resume changed raw or selection')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('manifest','baseline','completion','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();print(json.dumps(verify(a),indent=2))
