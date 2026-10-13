#!/usr/bin/env python3
"""Fresh same-geometry GV80 Park/stored-spin probe; never edits source raw."""
import argparse
import copy
from pathlib import Path

from base5_capture_safety import atomic_json,sha256
import generate_base5_independent as gen


def clear_spin(actor):
    saved=actor['initPosition'].get('vehicleSaveData',{})
    for wheel in saved.get('wheels',[]):
        for key in ('L','Tr'):
            if key in wheel:wheel[key]=0.
        if isinstance(wheel.get('lastTireForce'),dict):
            for key in wheel['lastTireForce']:wheel['lastTireForce'][key]=0.
    for container,keys in (('engine',('L','Treaction')),('gearbox',('L',))):
        for key in keys:
            if key in saved.get(container,{}):saved[container][key]=0.


def park(actor):
    actor['initPosition']['gear']=1
    saved=actor['initPosition'].get('vehicleSaveData',{})
    if isinstance(saved.get('inputData'),list) and len(saved['inputData'])>6:saved['inputData'][6]=1
    if isinstance(saved.get('gearbox'),dict):
        saved['gearbox']['gearMode']=1;saved['gearbox']['vehicleMoving']=False


def prepare(manifest,output,scene_id):
    if output.exists():raise FileExistsError('Fresh output required')
    deck=gen.geo.load_json(manifest);source=next(e for e in deck['scenes'] if e['id']==scene_id)
    scene=gen.geo.load_json(Path(source['scenario']));meta=gen.geo.load_json(Path(source['metadata']))
    target=next(a['unique_id'] for a in meta['runtime_actors'] if a['name']=='2021_Hyundai_Genesis_GV80')
    output.mkdir(parents=True);entries=[]
    for label,use_park,use_clear in (('baseline',False,False),('park',True,False),('clearspin',False,True),('park_clearspin',True,True)):
        current=copy.deepcopy(scene);metadata=copy.deepcopy(meta)
        actor=next(a for a in current['vehicleList'] if a['UNIQUEID']==target)
        if use_park:park(actor)
        if use_clear:clear_spin(actor)
        sid=source['id']+'_'+label;sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        metadata['stationary_vehicle_probe']=dict(label=label,target_uid=target,source_sha256=source['scenario_sha256'],
            park=use_park,clear_spin=use_clear,geometric_placement_unchanged=True)
        atomic_json(sp,current);atomic_json(mp,metadata)
        entry=copy.deepcopy(source);entry.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp))
        entries.append(entry)
    result=copy.deepcopy({k:v for k,v in deck.items() if k!='scenes'})
    result.update(scenes=entries,count=4,status='STATIC_STORED_STATE_AB_PENDING')
    atomic_json(output/'manifest.json',result)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--scene-id',default='b5v2_02967')
    a=p.parse_args();print(prepare(a.manifest,a.output,a.scene_id))
