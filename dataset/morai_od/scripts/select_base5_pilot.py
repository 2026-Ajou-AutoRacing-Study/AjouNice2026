#!/usr/bin/env python3
"""Select reproducible coverage-oriented pilot indices; does not load MR."""
import argparse
import json
from pathlib import Path


def select(manifest, limit=48):
    tags=[]
    for row in manifest['scenes']:
        tags.append({('family',row['family']),('subfamily',row.get('subfamily')),
            ('distance',row['family'],tuple(row.get('distance_bounds') or [])),
            ('role',row['family'],row.get('role')),('motion',row.get('motion')),('split',row.get('split'))})
    picked=[]; covered=set(); remaining=set(range(len(tags)))
    while remaining and len(picked)<limit:
        index=max(sorted(remaining),key=lambda i:(len(tags[i]-covered),-i))
        picked.append(index);covered.update(tags[index]);remaining.remove(index)
    return picked


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=int,default=48);a=p.parse_args()
    if not 1<=a.limit<=64:p.error('Pilot limit must be 1–64')
    m=json.loads(a.manifest.read_text());indices=select(m,a.limit)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(manifest=str(a.manifest.resolve()),indices=indices,
        limitations=['Coverage-oriented sample, not exhaustive asset or mission validation.']),indent=2)+'\n')
    print(','.join(map(str,indices)))
