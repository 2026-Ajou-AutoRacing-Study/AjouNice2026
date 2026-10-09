#!/usr/bin/env python3
"""Replay one frozen placement, with an empty-actor control, without changing it."""
import argparse
import copy
import json
from pathlib import Path
from base5_capture_safety import atomic_json, sha256
from run_base5_capture_campaign import subset_deck


def prepare(source, index, output, repeats=3, empty=False):
    data=json.loads(source.read_text());entry=data['scenes'][index]
    output.mkdir(parents=True,exist_ok=False);rows=[]
    for n in range(repeats):
        row=copy.deepcopy(entry);row['id']=f"{entry['id']}_stationary_{'empty' if empty else 'actor'}_{n}"
        meta=json.loads(Path(entry['metadata']).read_text())
        scenario=json.loads(Path(entry['scenario']).read_text())
        if empty:
            for key in ('vehicleList','pedestrianList','objectList','aiPedstrianList'):
                if key in scenario:scenario[key]=[]
            meta['runtime_actors']=[];row['motion']='static'
        meta['calibration_only']=True
        sp=output/(row['id']+'.json');mp=output/(row['id']+'_metadata.json')
        atomic_json(sp,scenario);atomic_json(mp,meta)
        row.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))
        rows.append(row)
    manifest=output/'manifest.json';atomic_json(manifest,subset_deck(data,rows));return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--index',type=int,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--empty',action='store_true');a=p.parse_args()
    print(prepare(a.manifest,a.index,a.output,a.repeats,a.empty))
