#!/usr/bin/env python3
"""Seeded family-stratified static timing deck; never starts capture or bulk."""
import argparse
import copy
import random
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_static_ground_policy import lightweight_static


def prepare(source, config_path, output):
    if output.exists():
        raise FileExistsError('Fresh static cost pilot output required')
    original=gen.geo.load_json(source);config=gen.geo.load_json(config_path)
    candidates={};weights=Counter()
    for index,entry in enumerate(original['scenes']):
        meta=gen.geo.load_json(Path(entry['metadata']))
        if all(a['motion'] in ('static','standing') for a in meta['runtime_actors']):
            candidates.setdefault(entry['family'],[]).append((index,entry,meta))
            weights[entry['family']]+=1
    # Fixed before seeing live outcomes. 40 slots approximate 4171 static family proportions.
    counts=dict(highway=14,obstacle=14,pedestrian_related=4,roundabout=4,background=3,other_vehicle=1)
    rng=random.Random(261007115);sample=[]
    for family,n in counts.items():
        sample.extend(rng.sample(candidates[family],n))
    rng.shuffle(sample)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    output.mkdir(parents=True);scenes=[];lineage=[]
    for index,old,previous in sample:
        for retry in range(previous['retry'],previous['retry']+512):
            try:scene,meta=gen.build(previous['spec'],config,ctx,retry)
            except ValueError:continue
            if lightweight_static(meta):break
        else:raise ValueError('Static pilot cell not fillable: '+old['id'])
        if previous['runtime_actors']:
            assert meta['runtime_actors'][0]['data_id']==previous['runtime_actors'][0]['data_id']
        sid=f'zzb5v115cost_{index:05d}'
        sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        meta['cost_pilot_source']=dict(index=index,id=old['id'],original_scenario_sha256=old['scenario_sha256'])
        atomic_json(sp,scene);atomic_json(mp,meta)
        entry=copy.deepcopy(old);entry.update(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),motion='static')
        scenes.append(entry);lineage.append(dict(index=index,id=old['id'],retry=retry))
    deck=copy.deepcopy({k:v for k,v in original.items() if k!='scenes'})
    deck.update(config=config,scenes=scenes,count=len(scenes),status='STATIC_COST_PILOT_ONLY',
        provenance=dict(source_manifest=str(source),source_manifest_sha256=sha256(source),
            config_sha256=sha256(config_path),generator_sha256=sha256(Path(gen.__file__)),
            static_policy_sha256=sha256(Path(__file__).with_name('base5_static_ground_policy.py')),
            seed=261007115,pilot_family_counts=counts,static_population_family_counts=dict(weights),
            source_cells=lineage,limitation='New runtime rebuild of old planned cells; not an exact paired performance comparison. No bulk capture.'))
    atomic_json(output/'manifest.json',deck)
    print(output/'manifest.json')
    print('Static population:',dict(weights),'pilot:',counts)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--config',type=Path,default=gen.geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_independent_5k_sunny13.json')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    prepare(a.source,a.config,a.output)
