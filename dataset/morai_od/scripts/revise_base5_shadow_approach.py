#!/usr/bin/env python3
"""Fresh v122 overlay: joint outside-Ego/inside-target quotas, no bulk capture."""
import argparse
import copy
import json
import random
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_shadow_observation import APPROACH_CONTRACT, validate_overlay, validate_approach


# Total 500, positive 480, prior 24 interior movers preserved exactly.
# Per-stage columns: empty / 3-10 / 10-30 / 30-60 / 60-100.
SCHEDULE = {
    'approach_far': (2, 0, 0, 35, 23),
    'approach_near': (2, 0, 30, 20, 8),
    'approach_threshold': (1, 0, 20, 8, 1),
    'entrance': (4, 0, 60, 23, 3),
    'interior': (8, 30, 66, 60, 2),
    'exit': (3, 10, 40, 14, 3),
}


def slots():
    positive=[]; empty=[]
    for stage, counts in SCHEDULE.items():
        empty.extend([(stage, [10,30])]*counts[0])
        for interval, count in zip(([3,10],[10,30],[30,60],[60,100]), counts[1:]):
            positive.extend([(stage, interval)]*count)
    random.Random(261008122).shuffle(positive)
    random.Random(122).shuffle(empty)
    return positive, empty


def revise(source, output):
    if output.exists(): raise FileExistsError('Fresh release required')
    original=gen.geo.load_json(source)
    if len(original['scenes'])!=5000: raise ValueError('Complete 5K source required')
    source_metas=[gen.geo.load_json(Path(e['metadata'])) for e in original['scenes']]
    shadow=[(e,m) for e,m in zip(original['scenes'],source_metas) if m['spec'].get('shadow_phase')]
    if len(shadow)!=500: raise ValueError('Source must already contain 500 shadow cells')
    movers=[(e,m) for e,m in shadow if m['spec'].get('shadow_moving')]
    if len(movers)!=24: raise ValueError('Expected preserved 24 shadow movers')
    config=copy.deepcopy(original['config']); plan=config['shadow_observation']
    config['dataset_id']='base5_independent_5k_sunny13_shadow_approach_v122'
    plan.update(observation_contract=APPROACH_CONTRACT,
        phase_counts=dict(approach=150,entrance=90,interior=190,exit=70),
        stage_counts=dict(approach_far=60,approach_near=60,approach_threshold=30,entrance=90,interior=190,exit=70),
        primary_right_lane_static_positive=365, primary_left_lane_static_positive=91,
        moving_scenes=24, moving_policy='Preserve v121 proposals/identities/paths; independent terrain QA remains mandatory',
        position_policy='Joint Ego stage, inside-target band, radial bin, real road support and Ego-frame ROI',
        uncertainty='Soft design priors, not official spawning probabilities; shaded volume not photometric boundary')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    identity={a['data_id']:(k,i) for k in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')
              for i,a in enumerate(ctx['pools'][k])}
    positive,empty=slots(); pi=ei=0; rows={}
    static=[(e,m) for e,m in shadow if not m['spec'].get('shadow_moving')]
    lane_deck=[0.]*365+[3.5]*91; random.Random(1220).shuffle(lane_deck)
    for e,m in static:
        r=copy.deepcopy(m['spec']); has_actor=bool(m['runtime_actors'])
        stage,interval=positive[pi] if has_actor else empty[ei]
        if has_actor:
            primary=m['runtime_actors'][0];pool,index=identity[primary['data_id']]
            r.update(shadow_primary_pool=pool,shadow_primary_index=index)
        lane=lane_deck[pi] if has_actor else 0.
        phase='approach' if stage.startswith('approach') else stage
        role='rear' if stage=='exit' else ('rear' if stage=='interior' and pi%4==0 and interval!=[60,100] else 'front')
        band=[.1,.4] if phase=='approach' else [.1,.5] if stage=='entrance' else [.2,.9] if stage=='exit' else [.2,.8]
        if interval==[60,100] and stage in ('approach_threshold','entrance'): band=[.6,.9]
        if stage=='interior' and interval==[60,100]: band=[.8,.9]
        elif stage=='interior' and pi%5==0: band=[.1,.2] if role=='rear' else [.8,.9]
        layout='staggered' if has_actor and r['actors']>=2 and pi%3==0 else 'same_lane'
        # The upstream road is not a uniform two-lane strip. Keep approach
        # Ego on its real route; adjacent observing positions are inside only.
        ego_lane=3.5 if has_actor and (interval==[3,10] or stage=='entrance' and pi%4==0) else 0.
        if interval==[3,10]: ego_lane=3.5-lane
        r.update(shadow_phase=phase,shadow_observation_stage=stage,shadow_moving=False,
            shadow_target_fraction_bounds=band,shadow_primary_lane_offset_m=lane,
            shadow_ego_lane_offset_m=ego_lane,shadow_layout=layout,
            role=role,distance_bounds=interval,distance_bin='shadow_'+str(interval),motion='static')
        rows[e['id']]=r
        if has_actor:pi+=1
        else:ei+=1
    if (pi,ei)!=(456,20): raise ValueError('Static schedule mismatch')
    plan['spec_overrides']=[rows.get(e['id'],m['spec']) for e,m in shadow]
    output.mkdir(parents=True);directory=output/'scenarios_5k';directory.mkdir()
    entries=[];metas=[];attempts=Counter();rejections=Counter()
    for e,m in zip(original['scenes'],source_metas):
        if sha256(Path(e['scenario']))!=e['scenario_sha256']: raise ValueError('Changed source')
        if e['id'] not in rows:
            entries.append(copy.deepcopy(e));metas.append(m);continue
        row=rows[e['id']]
        for attempt in range(512):
            try:scene,meta=gen.build(row,config,ctx,attempt);break
            except ValueError as exc: rejections[str(exc)]+=1
        else:
            atomic_json(output/'failed_cell.json',dict(id=e['id'],spec=row,rejections=dict(rejections)))
            raise ValueError('Unfillable approach cell '+e['id'])
        sp=directory/(e['id']+'.json');mp=directory/(e['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(e);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),split=meta['split'],role=row['role'],distance_bounds=row['distance_bounds'],
            motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else 'static')
        entries.append(new);metas.append(meta);attempts[str(attempt)]+=1
    deck=copy.deepcopy(original);deck.update(config=config,dataset_id=config['dataset_id'],scenes=entries,
        status='APPROACH_OFFLINE_GENERATED; live pilot pending; no bulk approval')
    deck['provenance'].update(shadow_source_sha256=sha256(source),shadow_approach_revision_sha256=sha256(__file__),
        generator_sha256=sha256(gen.__file__),shadow_module_sha256=sha256(Path(__file__).with_name('base5_shadow_observation.py')),
        config_sha256=gen.hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest())
    coverage=validate_overlay(deck,metas)
    primary=Counter(m['runtime_actors'][0]['data_id'] for m in metas if m['runtime_actors'])
    if len(primary)!=122 or min(primary.values())<4:raise ValueError('Global primary floor lost')
    geometry=[validate_approach(gen,ctx,m) for m in metas if m['spec'].get('shadow_observation_stage')]
    atomic_json(directory/'manifest.json',deck);atomic_json(output/'config.json',config)
    atomic_json(output/'revision_report.json',dict(coverage=coverage,source_manifest=str(source),
        changed_scenes=476,preserved_entries=4524,preserved_shadow_movers=24,global_primary_min=min(primary.values()),
        stages=dict(Counter(p['stage'] for p in geometry)),attempts=dict(attempts),rejections=dict(rejections),bulk_capture_started=False))
    candidates=[(e,m) for e,m in zip(entries,metas) if e['id'] in rows and m['runtime_actors']]
    pilot=[];used=set()
    conditions=[lambda m,st=st,k=k:m['spec']['shadow_observation_stage']==st and m['runtime_actors'][0]['population_pool']==k
                for st in ('approach_far','approach_near','approach_threshold','entrance','interior','exit') for k in ('vehicle','obstacle')]
    conditions += [lambda m,st=st:m['spec']['shadow_observation_stage']==st and m['spec']['shadow_layout']=='staggered'
                   for st in ('approach_far','entrance','interior')]
    conditions += [lambda m:m['spec']['shadow_primary_lane_offset_m']==3.5 and m['spec']['shadow_observation_stage'].startswith('approach'),
                   lambda m:m['spec']['distance_bounds']==[3,10],
                   lambda m:m['runtime_actors'][0]['population_pool'] in ('pet_person','stroller','rider') and m['spec']['shadow_observation_stage'].startswith('approach')]
    for condition in conditions:
        match=next(((e,m) for e,m in candidates if e['id'] not in used and condition(m)),None)
        if match is None:raise ValueError('Missing pilot condition')
        pilot.append(match[0]);used.add(match[0]['id'])
    for stage in ('approach_far','interior'):
        pilot.append(next(e for e,m in zip(entries,metas) if m['spec'].get('shadow_observation_stage')==stage and not m['runtime_actors']))
    p=copy.deepcopy(deck);p.update(scenes=pilot,count=len(pilot));atomic_json(output/'pilot_manifest.json',p)
    return directory/'manifest.json'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-manifest',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(revise(args.source_manifest,args.output))
