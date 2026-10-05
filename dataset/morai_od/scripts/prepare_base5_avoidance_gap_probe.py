#!/usr/bin/env python3
"""Controlled Ego-only lateral shift of selected failed layouts; never installs MR files."""
import argparse
import copy
import json
import math
from pathlib import Path
import generate_base5_independent as gen


def prepare(manifest,indices,output,extra_gap):
    if output.exists() and any(output.iterdir()):raise FileExistsError('Nonempty probe output refused')
    source=gen.geo.load_json(manifest);output.mkdir(parents=True,exist_ok=True);entries=[]
    for i in indices:
        entry=copy.deepcopy(source['scenes'][i]);scene=gen.geo.load_json(Path(entry['scenario']));meta=gen.geo.load_json(Path(entry['metadata']))
        ref=meta['avoidance_observation'];sign=1 if ref['side']=='left' else -1
        yaw=math.radians(meta['base']['yaw']);dx=-math.sin(yaw)*sign*extra_gap;dy=math.cos(yaw)*sign*extra_gap
        meta['ego']['x']+=dx;meta['ego']['y']+=dy
        gen.geo.set_ego_pose(scene['egoVehicle'],meta['ego'],initial_speed_kph=0.)
        gen.geo.zero_vector(scene['egoVehicle']['initPosition'].get('vehicleSaveData',{}).get('vehicle',{}).get('lastVelocity'))
        meta['controlled_gap_probe']=dict(source_manifest=str(manifest.resolve()),source_scene_id=entry['id'],
            ego_only_lateral_shift_m=extra_gap,training_candidate=False)
        ref['lateral_center_offset_m']+=sign*extra_gap
        ref['requested_surface_gap_m']+=extra_gap;ref['actual_surface_gap_m']+=extra_gap
        entry['id']+='gap_probe';sp=output/(entry['id']+'.json');mp=output/(entry['id']+'_metadata.json')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        entry.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp));entries.append(entry)
    source.update(scenes=entries,count=len(entries),start=0,status='CONTROLLED_PROBE_NOT_TRAINING',provenance=dict(source_manifest=str(manifest.resolve())))
    gen.geo.write_json(output/'manifest.json',source)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--indices',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--extra-gap-m',type=float,default=1.);a=p.parse_args()
    if not 0<a.extra_gap_m<=2.:p.error('bounded outward probe shift required')
    prepare(a.manifest,list(map(int,a.indices.split(','))),a.output,a.extra_gap_m)
