#!/usr/bin/env python3
"""Select deterministic real raw cost samples, not performance cherry-picking."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Use a new evidence output')
    payload=a.ledger.read_bytes();rows=[json.loads(x) for x in payload.splitlines()]
    accepted=[r for r in rows if r['status']=='accepted_candidate']
    ranked=[]
    for r in accepted:
        cloud=list(Path(r['capture_root']).glob('LIDAR_*/*_Intensity.bin'))
        if len(cloud)!=1:raise ValueError(r['id'])
        points=np.fromfile(cloud[0],dtype='<f4').reshape(-1,4)
        valid=np.isfinite(points).all(axis=1)
        valid &= ((points[:,:3]>=[-100,-30,-3]) & (points[:,:3]<[100,30,2])).all(axis=1)
        # Fixed beam count is not scene density. Rank nonzero returns inside ROI;
        # preserve the complete raw cloud in the sample, including zero returns.
        valid &= (points[:,:3]!=0).any(axis=1)
        ranked.append((int(valid.sum()),r,cloud[0]))
    ranked.sort(key=lambda r:(r[0],r[1]['id']))
    chosen={}
    # Sampling protocol fixed before any model execution: point-count quartiles,
    # largest clouds, and one deterministic sample from each semantic family.
    for q in (0,.25,.5,.75,1):
        r=ranked[int(q*(len(ranked)-1))];chosen[r[1]['id']]=(r,'point_quantile_'+str(q))
    for r in ranked[-3:]:chosen[r[1]['id']]=(r,'dense_tail')
    for family in sorted({r['family'] for r in accepted}):
        candidates=[r for r in ranked if r[1]['family']==family]
        r=min(candidates,key=lambda r:hashlib.sha256(r[1]['id'].encode()).digest())
        chosen.setdefault(r[1]['id'],(r,'family_'+family))
    a.output.mkdir(parents=True)
    manifest=[]
    for i,(item,reason) in enumerate(chosen.values()):
        count,row,cloud=item;root=Path(row['capture_root'])
        data={'points':np.fromfile(cloud,dtype='<f4').reshape(-1,4)}
        for k in range(1,5):
            files=list((root/f'CAMERA_{k}').glob('*_Intensity.png'))
            if len(files)!=1:raise ValueError('Camera file mismatch')
            data[f'camera{k}']=np.frombuffer(files[0].read_bytes(),dtype=np.uint8)
        target=a.output/f'sample_{i:02d}.npz';np.savez_compressed(target,**data)
        manifest.append(dict(file=target.name,id=row['id'],reason=reason,raw_points=len(data['points']),
                            density_rank_nonzero_roi_returns=count,
                            family=row['family'],capture_root=str(root),
                            sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    (a.output/'manifest.json').write_text(json.dumps(dict(source=str(a.ledger),
        source_sha256=hashlib.sha256(payload).hexdigest(),samples=manifest,
        nonzero_roi_return_quantiles=np.quantile([r[0] for r in ranked],[0,.25,.5,.75,1]).tolist()),indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
