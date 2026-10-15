#!/usr/bin/env python3
"""Revise only low-quota vehicle mission cells; preserve frozen source files."""
import argparse
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
import generate_base5_independent as gen


def comparable(config):
    result=copy.deepcopy(config);result.pop('dataset_id',None)
    result.get('mission_dynamic_observation',{}).pop('vehicle_primary_scenes',None)
    result.get('mission_dynamic_observation',{}).pop('vehicle_max_length_m',None)
    result.get('mission_dynamic_observation',{}).pop('vehicle_yielding_distance_bounds_m',None)
    return result


def revise(source,output,config):
    if output.exists():raise FileExistsError('Use a new revision directory')
    old=gen.geo.load_json(source);config=copy.deepcopy(config)
    if comparable(old['config'])!=comparable(config):raise ValueError('Unrelated config changes refused')
    if len(old['scenes'])!=config['target_scenes']:raise ValueError('Complete source required')
    config['dataset_id']='base5_independent_5k_sunny13_static_all_dynamic_vehicle_v112'
    specs={r['index']:r for r in gen.specs(config)}
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    result=copy.deepcopy(old);entries=[];phases=Counter();indices=[];names=[]
    output.mkdir(parents=True)
    for entry in old['scenes']:
        if gen.geo.sha256(Path(entry['scenario']))!=entry['scenario_sha256']:raise ValueError('Source checksum mismatch')
        meta=gen.geo.load_json(Path(entry['metadata']));row=specs[meta['spec']['index']]
        if row.get('dynamic_mission_pool'):
            for attempt in range(512):
                try:scene,meta=gen.build(row,config,ctx,attempt);break
                except ValueError:
                    if attempt==511:raise
            sp=output/(entry['id']+'.json');mp=output/(entry['id']+'_metadata.json')
            gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
            entry=dict(entry,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),
                split=meta['split'],motion='static',subfamily='vehicle',distance_bounds=row['distance_bounds'],role=row['role'])
            phases[row['dynamic_observation_phase']]+=1;indices.append(row['index']);names.append(meta['runtime_actors'][0]['name'])
        entries.append((entry,any(a['motion'] not in ('static','standing') for a in meta['runtime_actors'])))
    # Newly static vehicle observations belong to the static-first prefix.
    entries.sort(key=lambda pair:pair[1])
    report=dict(revised_scenes=len(indices),unchanged_scenes=len(entries)-len(indices),
        phases=dict(phases),source_indices=indices,primary_assets=names,
        stationary_scenes=sum(not moving for _,moving in entries),animated_scenes=sum(moving for _,moving in entries),
        live_validation=False,policy='road-supported crossing vehicle snapshot; no temporal or velocity supervision')
    if len(indices)!=config['mission_dynamic_observation']['vehicle_primary_scenes']:raise ValueError('Vehicle quota mismatch')
    result.update(dataset_id=config['dataset_id'],config=config,scenes=[e for e,_ in entries],
        status='DYNAMIC_VEHICLE_MISSION_REVISED_OFFLINE; live validation pending')
    result['provenance'].update(generator_sha256=gen.geo.sha256(Path(gen.__file__)),
        config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
        dynamic_vehicle_revision=dict(source_manifest=str(source.resolve()),source_sha256=gen.geo.sha256(source),
            revision_script_sha256=gen.geo.sha256(Path(__file__)),**report))
    gen.geo.write_json(output/'manifest.json',result);gen.geo.write_json(output/'dynamic_vehicle_revision.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,default=gen.CONFIG.with_name('base5_independent_5k_sunny13.json'))
    a=p.parse_args();print(json.dumps(revise(a.manifest,a.output,gen.geo.load_json(a.config)),ensure_ascii=False,indent=2))
