#!/usr/bin/env python3
"""Offline exact-output A/B of per-capture image decode caching on preserved raw."""
import argparse
import copy
import json
from pathlib import Path
import statistics
import time
from base5_capture_audit import audit_capture
from base5_capture_safety import atomic_json
from run_base5_capture_campaign import subset_deck


def benchmark(ledger, manifest, output, repeats=2):
    if output.exists():
        raise FileExistsError('Keep previous benchmark evidence; use a new output')
    rows=[json.loads(line) for line in ledger.read_text().splitlines() if line]
    evidence=[]
    for row in rows:
        root=Path(row['capture_root'])
        metadata=json.loads((root/'metadata.json').read_text())
        values={False:[],True:[]};reference=None
        for repeat in range(repeats):
            for cached in ((False,True) if repeat%2==0 else (True,False)):
                start=time.perf_counter();result=audit_capture(root,metadata,cache_camera_pixels=cached)
                values[cached].append(time.perf_counter()-start)
                if reference is None:reference=result
                elif result!=reference:raise ValueError('Cache changed audit output: '+str(root))
        evidence.append(dict(id=row['id'],components=len(reference['components']),
            uncached_sec=statistics.median(values[False]),cached_sec=statistics.median(values[True]),
            exact_output_equal=True))
    deck=json.loads(manifest.read_text());copy_cost={False:[],True:[]}
    for repeat in range(10):
        for efficient in ((False,True) if repeat%2==0 else (True,False)):
            start=time.perf_counter()
            if efficient:result=subset_deck(deck,[deck['scenes'][0]])
            else:
                result=copy.deepcopy(deck);result.update(scenes=[copy.deepcopy(deck['scenes'][0])],count=1)
            copy_cost[efficient].append(time.perf_counter()-start)
            expected=subset_deck(deck,[deck['scenes'][0]])
            if result!=expected:raise ValueError('Subset copy changed manifest output')
    result=dict(rows=evidence,repeats=repeats,frames=len(rows),
        uncached_total_sec=sum(r['uncached_sec'] for r in evidence),
        cached_total_sec=sum(r['cached_sec'] for r in evidence),
        full_scene_count=len(deck['scenes']),
        full_deepcopy_median_sec=statistics.median(copy_cost[False]),
        subset_deepcopy_median_sec=statistics.median(copy_cost[True]),
        limitation='Offline warm-cache CPU A/B. Not whole-campaign speedup; timing noise and simulator contention possible.')
    atomic_json(output,result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ledger',type=Path,required=True)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,default=2)
    args=parser.parse_args()
    if args.repeats<1:parser.error('Positive repeats required')
    result=benchmark(args.ledger,args.manifest,args.output,args.repeats)
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
