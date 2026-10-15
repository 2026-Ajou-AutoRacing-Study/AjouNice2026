#!/usr/bin/env python3
"""Select eight stationary bypass observations; never starts capture."""
import argparse
import json
from pathlib import Path


def select(manifest):
    candidates=[]
    for i,e in enumerate(manifest['scenes']):
        if e['family'] not in ('obstacle','other_vehicle') or not e['mission']:continue
        m=json.loads(Path(e['metadata']).read_text());r=m['spec']
        candidates.append((i,r,m['runtime_actors'][0]['geometry_reference_known']))
    indices=[]
    for family in ('obstacle','other_vehicle'):
        for phase in ('entry','alongside'):
            for side in ('left','right'):
                choices=[(r['actors'],not known,i) for i,r,known in candidates
                    if r['family']==family and r.get('avoidance_phase')==phase and r.get('avoidance_side')==side]
                if not choices:raise ValueError('Missing avoidance coverage '+str((family,phase,side)))
                indices.append(min(choices)[2])
    return indices


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError('Selection overwrite refused')
    ids=select(json.loads(a.manifest.read_text()));a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(indices=ids,scope='2 families x entry/alongside x left/right; single actor preferred; review only'),indent=2)+'\n')
    print(','.join(map(str,ids)))
