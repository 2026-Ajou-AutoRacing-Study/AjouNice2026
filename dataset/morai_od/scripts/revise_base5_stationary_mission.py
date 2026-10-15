#!/usr/bin/env python3
"""New static-mission revision; frozen source captures/manifests are never edited."""
import argparse
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
import generate_base5_independent as gen


def revise(source,output,config):
    if output.exists():raise FileExistsError('Use a new revision directory')
    config=copy.deepcopy(config)
    config['dataset_id']='base5_independent_5k_sunny13_static_all_v111'
    old=gen.geo.load_json(source)
    def comparable(c):
        return {k:v for k,v in c.items() if k not in ('dataset_id','stationary_mission_assets')}
    if comparable(old['config'])!=comparable(config):
        raise ValueError('Revision only changes static mission pools; other source config must match')
    if len(old['scenes'])!=config['target_scenes']:
        raise ValueError('A complete source deck is required')
    specs={r['index']:r for r in gen.specs(config)}
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    result=copy.deepcopy(old);entries=[];primary=Counter();groups=Counter();revised=0
    output.mkdir(parents=True)
    for original in old['scenes']:
        if gen.geo.sha256(Path(original['scenario']))!=original['scenario_sha256']:
            raise ValueError('Source scenario checksum mismatch')
        meta=gen.geo.load_json(Path(original['metadata']));index=meta['spec']['index']
        spec=specs[index]
        if spec.get('stationary_mission_pool'):
            for attempt in range(512):
                try:scene,metadata=gen.build(spec,config,ctx,attempt);break
                except ValueError:
                    if attempt==511:raise
            sp=output/(original['id']+'.json');mp=output/(original['id']+'_metadata.json')
            gen.geo.write_json(sp,scene);gen.geo.write_json(mp,metadata)
            entry=dict(original,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
                scenario_sha256=gen.geo.sha256(sp),split=metadata['split'],motion=metadata['runtime_actors'][0]['motion'])
            revised+=1;groups[spec['stationary_mission_pool']]+=1
        else:
            entry=copy.deepcopy(original);metadata=meta
        if spec['mission'] and spec['family'] in ('obstacle','other_vehicle'):
            primary[metadata['runtime_actors'][0]['data_id']]+=1
            if any(a['motion'] not in ('static','standing') or a.get('motion_path') for a in metadata['runtime_actors']):
                raise ValueError('Moving actor leaked into stationary mission')
        entries.append(entry)
    expected={a['data_id']:a['name'] for key in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider') for a in ctx['pools'][key]}
    # Generic primary exposure counts toward the dataset-wide floor. Stationary
    # mission itself deliberately retains at least three views for small pools.
    mission_missing=[name for uid,name in expected.items() if primary[uid]<3]
    if mission_missing:raise ValueError('Static mission needs at least three primary scenes per asset: '+str(mission_missing))
    report=dict(revised_scenes=revised,unchanged_scenes=len(entries)-revised,
        primary_pool_counts=dict(groups),eligible_assets=len(expected),missing_primary_assets=mission_missing,
        minimum_primary_count=min(primary[uid] for uid in expected),
        primary_counts={expected[uid]:primary[uid] for uid in expected},
        policy='all-positive-pools-stationary-v111',live_validation=False,
        note='Family is scene context, not GT class. Component annotation ontology remains unchanged.')
    result.update(dataset_id='base5_independent_5k_sunny13_static_all_v111',config=copy.deepcopy(config),scenes=entries,
        status='STATIC_MISSION_REVISED_OFFLINE; representative live validation pending')
    result['provenance'].update(generator_sha256=gen.geo.sha256(Path(gen.__file__)),
        config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
        stationary_mission_revision=dict(source_manifest=str(source.resolve()),source_sha256=gen.geo.sha256(source),
            revision_script_sha256=gen.geo.sha256(Path(__file__)),revised_scenes=revised,unchanged_scenes=len(entries)-revised))
    gen.geo.write_json(output/'manifest.json',result)
    gen.geo.write_json(output/'stationary_mission_coverage.json',report)
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=gen.CONFIG.with_name('base5_independent_5k_sunny13.json'))
    args=parser.parse_args()
    print(json.dumps(revise(args.manifest,args.output,gen.geo.load_json(args.config)),ensure_ascii=False,indent=2))
