#!/usr/bin/env python3
"""Replace only unprotected generic views with tunnel snapshots; never capture bulk."""
import argparse
import copy
import json
import random
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_shadow_observation import POLICY, validate_overlay


def select(original, ctx):
    available=[];background=[]
    for entry in original['scenes']:
        meta=gen.geo.load_json(Path(entry['metadata']));r=meta['spec']
        if r['mission'] or r['family']=='roundabout' or r.get('avoidance_phase') or r.get('vehicle_close_phase'):
            continue
        if meta['runtime_actors']:available.append((entry,meta))
        else:background.append((entry,meta))
    # Start with a primary for every available identity. Keep these primary
    # identities when replacing the view, so rare global quotas cannot vanish.
    chosen={}
    for entry,meta in available:
        uid=meta['runtime_actors'][0]['data_id']
        if uid not in chosen:chosen[uid]=(entry,meta)
    first=list(chosen.values());used={e['id'] for e,m in first}
    rest=[item for item in available if item[0]['id'] not in used]
    random.Random(261008120).shuffle(rest)
    positive=first+rest[:480-len(first)]
    all_assets={a['data_id']:(key,i) for key in ('vehicle','static_vehicle','obstacle','person','dummy','animal',
                    'pet_person','stroller','rider') for i,a in enumerate(ctx['pools'][key])}
    missing=sorted(set(all_assets)-set(chosen))
    # Two static vehicle assets have their original generic close cells
    # protected. ADD tunnel views by replacing abundant donors, not those cells.
    for j,uid in enumerate(missing):
        entry,meta=positive[len(first)+j]
        meta=copy.deepcopy(meta);key,index=all_assets[uid]
        meta['spec'].update(shadow_primary_pool=key,shadow_primary_index=index)
        positive[len(first)+j]=(entry,meta)
    return positive+background[:20],all_assets


def revise(source, output):
    if output.exists():raise FileExistsError('Fresh release output required')
    original=gen.geo.load_json(source)
    if len(original['scenes'])!=5000:raise ValueError('Expected complete standalone 5K source')
    config=copy.deepcopy(original['config']);ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    config['dataset_id']='base5_independent_5k_sunny13_shadow_v121'
    config['shadow_observation']=dict(policy=POLICY,scenes=500,primary_assets=122,moving_scenes=24,
        phase_counts=dict(entrance=120,interior=260,exit=120),
        positive_distance_bins_m=[[3,10,40],[10,30,240],[30,60,160],[60,100,40]],
        protected='all original missions, roundabout generic, avoidance and vehicle-close cells',
        motion_policy='appearance-changing primary only; strict independent tunnel terrain preparation',
        vehicle_policy='Park snapshots incl. opposite/crossing heading; not a dynamic rollout',
        environment='operator Sunny13; road volume does not certify image darkness')
    selected,_=select(original,ctx)
    eligible=[]
    for e,m in selected[:480]:
        actor=m['runtime_actors'][0]
        if (actor['population_pool'] in ('person','animal','pet_person') and 'NCAP' not in actor['name']
                and actor['source_list']!='aiPedstrianList'):
            eligible.append((e['id'],actor['population_pool'],actor['data_id']))
    # Include ALL three animals and the pet-person; an AI destination actor
    # with unverified movement must never fill a moving-scene quota.
    mandatory=[];identities=set()
    for uid,pool,identity in eligible:
        if pool in ('animal','pet_person') and identity not in identities:
            mandatory.append(uid);identities.add(identity)
    moving=mandatory+[uid for uid,pool,identity in eligible if pool=='person'][:24-len(mandatory)]
    if len(moving)!=24:raise ValueError('Insufficient appearance mover donors')
    # Balance phases independently of asset ordering. Movers stay in the
    # interior and get longitudinal safe paths, no intersection-height reuse.
    phases=['entrance']*120+['interior']*236+['exit']*120
    random.Random(120).shuffle(phases)
    distances=[[3,10]]*40+[[10,30]]*216+[[30,60]]*160+[[60,100]]*40
    random.Random(121).shuffle(distances)
    rows={};phase_index=0;distance_index=0;move_index=0
    for ordinal,(entry,meta) in enumerate(selected):
        r=copy.deepcopy(meta['spec']);is_moving=entry['id'] in moving
        phase='interior' if is_moving else phases[phase_index]
        if not is_moving:phase_index+=1
        positive=bool(meta['runtime_actors'])
        interval=[10,30] if is_moving else distances[distance_index] if positive else [10,30]
        if positive and not is_moving:distance_index+=1
        side=positive and interval==[3,10] and ordinal%2==0
        role=('left' if ordinal%4==0 else 'right') if side else ('rear' if ordinal%5==0 else 'front')
        if positive and interval==[60,100] and phase in ('entrance','exit'):
            role='front' if phase=='entrance' else 'rear'
        count=0 if not positive else 1 if is_moving else [1,1,2,2,3][ordinal%5]
        r.update(shadow_phase=phase,shadow_moving=is_moving,shadow_direction_sign=1 if move_index%2==0 else -1,
                 topology='route_general',yaw_limit=3,role=role,distance_bounds=interval,
                 distance_bin='shadow_'+str(interval),
                 actors=count,population_mode='shadow_situation',population_layout='road_corridor',
                 motion=('walking' if move_index%4<2 else 'running') if is_moving else 'static')
        if is_moving:
            # Approach a close target from farther away; do not sweep across
            # the stationary Ego while waiting for the capture preparation.
            r['shadow_direction_sign']=-1 if role=='front' else 1
        if is_moving:move_index+=1
        r.pop('population_budget_bounds',None);r.pop('population_max_actors',None)
        rows[entry['id']]=r
    config['shadow_observation']['spec_overrides']=list(rows.values())
    output.mkdir(parents=True);directory=output/'scenarios_5k';directory.mkdir()
    entries=[];metas=[];changed=[];rejected=Counter();attempts=Counter()
    for entry in original['scenes']:
        if sha256(Path(entry['scenario']))!=entry['scenario_sha256']:raise ValueError('Source payload changed')
        if entry['id'] not in rows:
            entries.append(copy.deepcopy(entry));metas.append(gen.geo.load_json(Path(entry['metadata'])));continue
        r=rows[entry['id']]
        for attempt in range(512):
            try:scene,meta=gen.build(r,config,ctx,attempt);break
            except ValueError as exc:rejected[str(exc)]+=1
        else:
            atomic_json(output/'failed_cell.json',dict(row=r,rejections=dict(rejected)))
            raise ValueError('Unfillable shadow cell '+entry['id'])
        sp=directory/(entry['id']+'.json');mp=directory/(entry['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(entry);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),split=meta['split'],distance_bounds=r['distance_bounds'],role=r['role'],
            motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else 'static')
        entries.append(new);metas.append(meta);changed.append(entry['id']);attempts[str(attempt)]+=1
    deck=copy.deepcopy(original);deck.update(config=config,dataset_id=config['dataset_id'],scenes=entries,
        status='SHADOW_OFFLINE_GENERATED; live validation pending; no bulk approval')
    deck['provenance'].update(generator_sha256=sha256(gen.__file__),shadow_module_sha256=sha256(
        Path(__file__).with_name('base5_shadow_observation.py')),shadow_source_sha256=sha256(source),
        shadow_revision_script_sha256=sha256(__file__),config_sha256=gen.hashlib.sha256(
        json.dumps(config,sort_keys=True).encode()).hexdigest())
    coverage=validate_overlay(deck,metas)
    actual_movers=sum(any(a['motion'] not in ('static','standing') for a in m['runtime_actors'])
                     for m in metas if m['spec'].get('shadow_phase'))
    if actual_movers!=config['shadow_observation']['moving_scenes']:
        raise ValueError('Moving request was downgraded; it cannot fill a moving quota')
    primary=Counter(m['runtime_actors'][0]['data_id'] for m in metas if m['runtime_actors'])
    if len(primary)!=122 or min(primary.values())<4:raise ValueError('Global rare-primary floor broken')
    atomic_json(directory/'manifest.json',deck);atomic_json(output/'config.json',config)
    atomic_json(output/'revision_report.json',dict(coverage=coverage,global_primary_min=min(primary.values()),
        changed_ids=changed,preserved_payload_scenes=4500,source_manifest=str(source.resolve()),
        rejections=dict(rejected),attempts=dict(attempts),bulk_capture_started=False))
    candidates=[(e,m) for e,m in zip(entries,metas) if m['spec'].get('shadow_phase') and not m['spec']['shadow_moving']]
    pilot=[];used=set()
    conditions=[lambda m,p=p: m['spec']['shadow_phase']==p and not m['runtime_actors'] for p in ('entrance','interior','exit')]
    conditions += [lambda m,p=p,n=n: m['spec']['shadow_phase']==p and len(m['runtime_actors'])==n
                   for p in ('entrance','interior','exit') for n in (1,2,3)]
    conditions += [lambda m,key=key: any(a['population_pool']==key for a in m['runtime_actors'])
                   for key in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')]
    conditions += [lambda m,name=name: any(a['name']==name for a in m['runtime_actors']) for name in ('WoodBox','CargoBox')]
    for condition in conditions:
        item=next(((e,m) for e,m in candidates if e['id'] not in used and condition(m)),None)
        if item is None:raise ValueError('Required static pilot cell unavailable')
        pilot.append(item[0]);used.add(item[0]['id'])
    def subset(name,chosen):
        d=copy.deepcopy(deck);d.update(scenes=chosen,count=len(chosen));atomic_json(output/name,d)
    subset('pilot_static_manifest.json',pilot)
    movers=[e for e,m in zip(entries,metas) if m['spec'].get('shadow_moving')]
    selected_movers=[]
    for name in ('Dog1','Dog2','Deer1','Ped_Pet1'):
        match=next(e for e,m in zip(entries,metas) if m['spec'].get('shadow_moving') and m['runtime_actors'][0]['name']==name)
        selected_movers.append(match)
    used_movers={e['id'] for e in selected_movers}
    for predicate in (lambda m:m['runtime_actors'][0]['population_pool']=='person' and m['spec']['motion']=='walking',
                      lambda m:m['runtime_actors'][0]['population_pool']=='person' and m['spec']['motion']=='running',
                      lambda m:m['runtime_actors'][0]['population_pool']=='person' and m['spec']['shadow_direction_sign']==1):
        match=next(e for e,m in zip(entries,metas) if m['spec'].get('shadow_moving') and
                   e['id'] not in used_movers and predicate(m))
        selected_movers.append(match);used_movers.add(match['id'])
    subset('pilot_moving_manifest.json',selected_movers)
    return directory/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(revise(a.source_manifest,a.output))
