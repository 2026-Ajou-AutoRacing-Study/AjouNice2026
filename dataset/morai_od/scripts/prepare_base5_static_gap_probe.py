#!/usr/bin/env python3
"""Bounded matched placement probe; never changes a frozen release or its raw."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256


def prepare(source,output,cell,gaps):
    if output.exists():raise FileExistsError('Fresh gap-probe output required')
    if not 1<=len(gaps)<=8 or any(not .3<=g<=2 for g in gaps):raise ValueError('Bounded gap probe only')
    deck=gen.geo.load_json(source)
    entry=next(e for e in deck['scenes'] if e['id']==cell)
    old=gen.geo.load_json(Path(entry['metadata']))
    if old['spec'].get('avoidance_phase')!='alongside':raise ValueError('Alongside source required')
    if any(a['motion'] not in ('static','standing') for a in old['runtime_actors']):raise ValueError('Static probe only')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO);output.mkdir(parents=True)
    rows=[]
    for gap in gaps:
        row=copy.deepcopy(old['spec']);row['avoidance_gap_bounds_m']=[gap,gap+.01]
        scene,meta=gen.build(row,deck['config'],ctx,old['retry'])
        sid=cell+'_gap'+str(round(gap*100))
        meta['scene_group_id']=sid;meta['gap_probe']=dict(source_cell=cell,requested_gap_m=gap,
            source_manifest_sha256=sha256(source),not_bulk_training_selection=True)
        sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(entry);new.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),scene_group_id=sid,split=meta['split'])
        rows.append(new)
    deck.update(scenes=rows,count=len(rows),status='GAP_DIAGNOSTIC_ONLY; no bulk approval')
    atomic_json(output/'manifest.json',deck);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--cell',required=True)
    p.add_argument('--gaps',type=float,nargs='+',default=[.3,.45,.6,.8])
    a=p.parse_args();print(prepare(a.manifest,a.output,a.cell,a.gaps))
