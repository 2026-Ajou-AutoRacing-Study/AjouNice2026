#!/usr/bin/env python3
"""Probe road-aligned long parked NPCs in the same shadow mission context."""
import argparse
import copy
import json
from pathlib import Path
import base5_spatial_diversity as spatial
from base5_capture_safety import atomic_json,sha256
from run_base5_failure_aware_replenishment_fast import prewarm


def prepare(source,output):
    if output.exists():raise FileExistsError('New probe output required')
    deck=json.loads(source.read_text())
    original=next(e for e in deck['scenes'] if e['id'].endswith('00993'))
    old=json.loads(Path(original['metadata']).read_text());ctx=prewarm(spatial.gen.context(spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,spatial.gen.geo.DEFAULT_MGEO),deck['config'])
    entries=[]
    for ordinal in range(4):
        row=copy.deepcopy(old['spec']);row['seed']+=ordinal*100019
        row['static_yaw_policy']['primary_mode']='road_aligned'
        row['static_yaw_policy'].pop('primary_sector',None)
        for attempt in range(512):
            try:
                scene,m=spatial.build(row,deck['config'],ctx,attempt)
                if m['split']!=old['split']:continue
                break
            except ValueError:continue
        else:raise ValueError('Long NPC road-aligned probe unfillable')
        m['scene_group_id']=old['scene_group_id'];m['long_vehicle_probe']=dict(original_id=original['id'],
            original_yaw_mode=old['spec']['static_yaw_policy']['primary_mode'],new_yaw_mode='road_aligned',
            original_asset=old['runtime_actors'][0]['name'],native_qa_unchanged=True)
        ident=f'long_npc_aligned_{ordinal:02}';sp=output/(ident+'.json');mp=output/(ident+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,m)
        entries.append(dict(original,id=ident,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp)))
    result=copy.deepcopy(deck);result.update(scenes=entries,count=4,status='LONG_VEHICLE_DIAGNOSTIC_ONLY')
    atomic_json(output/'manifest.json',result)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
