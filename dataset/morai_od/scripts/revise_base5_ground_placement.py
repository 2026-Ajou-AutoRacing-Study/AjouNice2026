#!/usr/bin/env python3
"""New frozen height revision: preserve XY/yaw/quota/splits and original files."""
import argparse
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
import generate_base5_independent as gen
from base5_map_surface import configure,initial_pose


def revise(source,output,config):
    if output.exists():raise FileExistsError('New height revision directory required')
    old=gen.geo.load_json(source);config=copy.deepcopy(config)
    def comparable(c):
        c=copy.deepcopy(c);c.pop('dataset_id',None);c.pop('placement_ground',None)
        c.get('capture',{}).pop('static_quality_budget',None);return c
    if comparable(old['config'])!=comparable(config):raise ValueError('Unrelated config changes refused')
    if not config.get('placement_ground',{}).get('enabled'):raise ValueError('Final XY height policy required')
    ctx=configure(gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO),config['placement_ground'])
    assets={a['data_id']:a for pool in ctx['pools'].values() for a in pool}
    counts=Counter();deltas=[];entries=[];output.mkdir(parents=True)
    config['dataset_id']='base5_independent_5k_sunny13_final_xy_ground_v113'
    for entry in old['scenes']:
        if gen.geo.sha256(Path(entry['scenario']))!=entry['scenario_sha256']:raise ValueError('Source checksum mismatch')
        scene=gen.geo.load_json(Path(entry['scenario']));meta=gen.geo.load_json(Path(entry['metadata']))
        for record in meta['runtime_actors']:
            if record['motion'] not in ('static','standing'):continue
            if record.get('input_ground_z_shift_m'):raise ValueError('Measured retry input cannot be treated as original placement')
            actor=next(a for a in scene[record['source_list']] if a['UNIQUEID']==record['unique_id'])
            raw=actor['initPosition'] if 'initPosition' in actor else actor
            previous=record.get('static_height_initializer',{});legacy=.36 if record['source_list']=='vehicleList' else 0.
            base_z=record['position']['z']-(previous['pivot_height_candidate_m']-legacy if previous.get('usable') else 0.)
            old_proof=record.get('map_ground_initializer')
            if old_proof:base_z=old_proof['previous_reference_z_m']
            pose=dict(record['position'],z=base_z,yaw=float(raw['rot']['yaw']),
                link_idx=old_proof.get('preferred_link') if old_proof else record.get('sample_highway_placement',{}).get('link_id'))
            asset=assets[record['data_id']];size,offset,_=gen.geometry(asset,ctx)
            new,initializer,ground=initial_pose(pose,asset,size,offset,ctx)
            delta=new['z']-record['position']['z'];deltas.append(delta)
            raw['pos']['z']=new['z']+legacy
            if '_z' in raw['pos']:raw['pos']['_z']=f"{raw['pos']['z']:.3f}"
            for key in ('initPos','standardPos'):
                if key in actor:
                    actor[key]['y']+=delta
                    if '_y' in actor[key]:actor[key]['_y']=f"{actor[key]['y']:.3f}"
            record['position']['z']=new['z'];record['static_height_initializer']=initializer
            record['map_ground_initializer']=ground
            counts[ground['confidence']]+=1
        sp=output/(entry['id']+'.json');mp=output/(entry['id']+'_metadata.json')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        entries.append(dict(entry,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp)))
    report=dict(source_manifest=str(source.resolve()),source_sha256=gen.geo.sha256(source),
        static_actors=len(deltas),map_support=dict(counts),
        changed_actors=sum(abs(d)>1e-6 for d in deltas),
        minimum_z_change_m=min(deltas,default=0.),maximum_z_change_m=max(deltas,default=0.),
        xy_yaw_roles_splits_preserved=True,contact_verified=False,
        revision_script_sha256=gen.geo.sha256(Path(__file__)),
        map_surface_script_sha256=gen.geo.sha256(Path(__file__).with_name('base5_map_surface.py')),
        limitation='Initial input height only; raw geometric GT and live audit gates unchanged')
    result=copy.deepcopy(old);result.update(dataset_id=config['dataset_id'],config=config,scenes=entries,
        status='FINAL_XY_GROUND_REVISED_OFFLINE; live validation pending')
    result['provenance'].update(generator_sha256=gen.geo.sha256(Path(gen.__file__)),ground_placement_revision=report,
        config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest())
    gen.geo.write_json(output/'manifest.json',result);gen.geo.write_json(output/'ground_placement_revision.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path,default=gen.CONFIG)
    a=p.parse_args();print(revise(a.manifest,a.output,gen.geo.load_json(a.config)))
