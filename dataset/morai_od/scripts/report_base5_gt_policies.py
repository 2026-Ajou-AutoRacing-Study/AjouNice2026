#!/usr/bin/env python3
"""Read-only provisional GT comparison; not model evaluation or exit safety approval."""
import argparse
from collections import Counter
import json
from pathlib import Path
from base5_capture_safety import atomic_json,sha256

POLICIES=('lidar5','lidar1','lidar1_camera0')


def compare(ledgers):
    latest={};sources=[]
    for ledger in ledgers:
        sources.append(dict(path=str(ledger.resolve()),sha256=sha256(ledger)))
        for line in ledger.read_text().splitlines():
            row=json.loads(line);latest[row.get('source_cell_id',row['id'])]=row
    results={}
    for policy in POLICIES:
        classes=Counter();bins=Counter();usable=[];review=[];excluded=Counter()
        for cell,row in latest.items():
            if row['status']!='accepted_candidate':review.append(cell);continue
            root=Path(row['capture_root']);meta=json.loads((root/'metadata.json').read_text())
            components=json.loads((root/'audit.json').read_text())['components']
            primary=meta['runtime_actors'][0]['unique_id'] if meta['runtime_actors'] else None
            if primary is not None and not any(c['parent_uid']==primary and c['positive_'+policy] for c in components):
                review.append(cell);continue
            usable.append(cell)
            for c in components:
                (classes if c['positive_'+policy] else excluded)[c['training_class']]+=1
            bins[str((row.get('family'),row.get('distance_bounds'),row.get('role')))]+=1
        results[policy]=dict(usable_scenes=len(usable),usable_ids=usable,review_ids=review,
            kept_gt=dict(classes),excluded_gt=dict(excluded),usable_cells=dict(bins))
    return dict(sources=sources,unique_cells=len(latest),policies=results,
        limitations=['Small diagnostic sample, not full-catalog validation or model-quality comparison.',
            'Capture acceptance and exit Park confirmation are separate; this report compares GT only.',
            'No raw GT modifications; camera policy uses actual model-crop instance evidence.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledgers',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=compare(a.ledgers);atomic_json(a.output,report)
    print(json.dumps({k:{n:v for n,v in r.items() if n in ('usable_scenes','kept_gt','excluded_gt')}
        for k,r in report['policies'].items()},indent=2))
