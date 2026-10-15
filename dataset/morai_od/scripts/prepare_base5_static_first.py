#!/usr/bin/env python3
"""Regenerate stationary cells with current clearance policy; defer, never erase, motion quota."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from base5_motion_ground import moving_actors
from base5_capture_safety import atomic_json, sha256


def prepare(source,output,limit=None):
    if output.exists():raise FileExistsError('New static-first output required')
    m=gen.geo.load_json(source);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    output.mkdir(parents=True);rows=[];deferred=[];unfilled=[]
    for old in m['scenes']:
        meta=gen.geo.load_json(Path(old['metadata']))
        if moving_actors(meta):deferred.append(dict(id=old['id'],reason='motion_ground_calibration_required'));continue
        if limit is not None and len(rows)>=limit:break
        for retry in range(meta['retry'],meta['retry']+512):
            try:scene,new=gen.build(meta['spec'],m['config'],ctx,retry)
            except ValueError:continue
            if new['split']!=meta['split']:continue
            if moving_actors(new):continue
            if new['runtime_actors'] and new['runtime_actors'][0]['data_id']!=meta['runtime_actors'][0]['data_id']:
                raise ValueError('Primary asset changed')
            sp=output/(old['id']+'.json');mp=output/(old['id']+'_metadata.json')
            new['static_first_provenance']=dict(source_scenario_sha256=old['scenario_sha256'],source_metadata_sha256=sha256(Path(old['metadata'])),
                contract='same requested quota/primary asset/split; current conservative clearance; no moving cells converted to standing')
            atomic_json(sp,scene);atomic_json(mp,new)
            rows.append(dict(old,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp)));break
        else:unfilled.append(dict(id=old['id'],reason='stationary_same_cell_unfillable'))
    deck=copy.deepcopy({k:v for k,v in m.items() if k!='scenes'})
    deck.update(count=len(rows),scenes=rows,status='V46_STATIC_FIRST_PENDING_LIVE; NOT COMPLETE_10K')
    atomic_json(output/'manifest.json',deck)
    atomic_json(output/'preparation_report.json',dict(prepared=len(rows),deferred_motion=deferred,unfilled=unfilled,
        source_manifest_sha256=sha256(source),partial=limit is not None))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int)
    a=p.parse_args();print(prepare(a.manifest,a.output,a.limit))
