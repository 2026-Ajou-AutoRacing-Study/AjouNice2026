#!/usr/bin/env python3
"""Offline same-XY old/new height pairs from retained static captures."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen
from revise_base5_ground_placement import revise


def prepare(captures,output,isolate_first=False):
    if output.exists():raise FileExistsError('New A/B output required')
    output.mkdir(parents=True);config=gen.geo.load_json(gen.CONFIG)
    config.pop('placement_ground',None);config['capture'].pop('static_quality_budget',None)
    rows=[]
    for i,root in enumerate(captures):
        scene=gen.geo.load_json(root/'scenario.json');meta=gen.geo.load_json(root/'metadata.json')
        if any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']):
            raise ValueError('Static A/B only')
        for key in ('capture_safety','capture_runtime_ego','capture_runtime_objects','lidar_to_ego',
                'static_ground_references','motion_probe','static_ground_reaudit'):
            meta.pop(key,None)
        if isolate_first and i==0:
            target=meta['runtime_actors'][0];meta['runtime_actors']=[target];meta['spec']['actors']=1
            if meta.get('ground_support_check',{}).get('required_uids'):
                meta['ground_support_check']['required_uids']=[target['unique_id']]
            meta.pop('population_budget',None)
            for key in ('vehicleList','objectList','pedestrianList','aiPedstrianList'):
                scene[key]=[a for a in scene.get(key,[]) if a['UNIQUEID']==target['unique_id']]
        meta['height_ab_probe']=dict(source_capture=str(root.resolve()),isolation_applied=bool(isolate_first and i==0),
            variant='old',source_scenario_sha256=gen.geo.sha256(root/'scenario.json'))
        meta['live_validated']=False
        sid=f'b5height113_{i:02d}_old';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta);r=meta['spec']
        rows.append(dict(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),
            family=r['family'],role=r['role'],distance_bounds=r['distance_bounds'],subfamily=r['subfamily'],motion='static',split=meta['split']))
    source=dict(config=config,scenes=rows,count=len(rows),provenance={})
    gen.geo.write_json(output/'old_manifest.json',source)
    revise(output/'old_manifest.json',output/'new',gen.geo.load_json(gen.CONFIG))
    new=gen.geo.load_json(output/'new/manifest.json');pairs=[]
    for old,current in zip(rows,new['scenes']):
        current['id']=old['id'].replace('_old','_new')
        meta=gen.geo.load_json(Path(current['metadata']));meta['height_ab_probe']['variant']='new'
        gen.geo.write_json(Path(current['metadata']),meta)
        pairs.extend([old,current])
    new.update(scenes=pairs,count=len(pairs),status='SAME_XY_HEIGHT_AB_PENDING_LIVE')
    gen.geo.write_json(output/'manifest.json',new)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--captures',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--isolate-first',action='store_true')
    a=p.parse_args();print(prepare(a.captures,a.output,a.isolate_first))
