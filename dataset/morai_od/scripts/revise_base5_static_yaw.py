#!/usr/bin/env python3
"""Fresh v129 yaw/character-clearance revision of v123. Never edit old raw or load MR."""
import argparse
import copy
import json
import math
import random
from collections import Counter, defaultdict
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_region_contract import validate_quotas
from base5_static_yaw import POLICY, PED_POOLS
from prepare_base5_environment_suite import prepare
from prepare_base5_batch_order import prepare as prepare_order


def target_scope(meta):
    r=meta['spec'];actors=meta['runtime_actors']
    if not actors or r.get('shadow_phase') or r.get('dynamic_observation_phase') or r.get('roundabout_phase'):
        return None
    if any(a['motion'] not in ('static','standing') for a in actors):return None
    if not r['mission'] and any(a['source_list'] in ('pedestrianList','aiPedstrianList') for a in actors):
        return 'generic'
    if r['mission'] and r.get('avoidance_phase') and any(a['population_pool'] in PED_POOLS for a in actors):
        return 'stationary_avoidance'
    return None


def yaw_hist(metas):
    hist=defaultdict(Counter)
    for m in metas:
        scope=target_scope(m)
        if not scope:continue
        for a in m['runtime_actors']:
            if a['population_pool'] not in PED_POOLS:continue
            if scope=='generic' and a['source_list']=='objectList':continue
            yaw=a.get('placement_yaw_deg')
            if yaw is None:
                scene=gen.geo.load_json(Path(m['_scenario']))
                actor=next(x for x in scene[a['source_list']] if x['UNIQUEID']==a['unique_id'])
                yaw=float(actor['rot']['yaw'])
            angle=(yaw-m['base']['yaw']+180)%360-180
            hist[scope]['actors']+=1
            hist[scope]['octant_'+str(int((angle+180)//45))]+=1
            hist[scope]['off_cardinal_over_15deg']+=int(abs(angle-round(angle/90)*90)>15.002)
            if a.get('static_yaw_policy'):hist[scope][a['static_yaw_policy']['mode']]+=1
    return {k:dict(v) for k,v in hist.items()}


def select_pilot(entries,metas):
    # Freeze strata before simulator results; every output remains evidence.
    conditions=[]
    for pool in PED_POOLS:
        for mode in ('cardinal','uniform'):
            conditions.append((pool+'_'+mode,lambda m,p=pool,mode=mode:any(
                a['population_pool']==p and a.get('static_yaw_policy',{}).get('mode')==mode
                for a in m['runtime_actors'])))
    for side in ('left','right'):
        conditions.append(('near_'+side,lambda m,side=side:m['spec'].get('avoidance_phase')=='alongside'
            and m['spec']['avoidance_side']==side and any(a.get('static_yaw_policy') for a in m['runtime_actors'])))
    for mode in ('generic_regular','generic_dense'):
        conditions.append((mode,lambda m,mode=mode:m['spec'].get('population_mode')==mode and
            any(a.get('static_yaw_policy') for a in m['runtime_actors'])))
    for role in ('front','rear','left','right'):
        conditions.append(('view_'+role,lambda m,role=role:m['spec']['role']==role and
            any(a.get('static_yaw_policy') for a in m['runtime_actors'])))
    for pool in ('person','animal','pet_person','stroller','rider'):
        conditions.append(('avoidance_'+pool,lambda m,pool=pool:m['spec'].get('static_yaw_policy',{}).get('scope')=='stationary_avoidance'
            and m['runtime_actors'][0]['population_pool']==pool))
    conditions.extend([
        ('crossing_walk_control',lambda m:any(a['motion']=='walking' for a in m['runtime_actors']) and not m['spec'].get('shadow_phase')),
        ('crossing_run_control',lambda m:any(a['motion']=='running' for a in m['runtime_actors']) and not m['spec'].get('shadow_phase')),
        ('shadow_static_control',lambda m:m['spec'].get('shadow_phase') and m['runtime_actors'] and
            all(a['motion'] in ('static','standing') for a in m['runtime_actors'])),
        ('highway_control',lambda m:m['spec']['family']=='highway' and not m['spec']['mission']),
        ('roundabout_control',lambda m:m['spec']['family']=='roundabout'),
        ('rigid_obstacle_control',lambda m:m['spec']['family']=='obstacle' and not m['spec']['mission'] and
            m['runtime_actors'] and not any(a.get('static_yaw_policy') for a in m['runtime_actors'])),
    ])
    picked=[];used=set()
    for tag,predicate in conditions:
        pair=next(((e,m) for e,m in zip(entries,metas) if e['id'] not in used and predicate(m)),None)
        if pair is None:raise ValueError('Missing pilot stratum '+tag)
        picked.append(dict(tag=tag,scene=pair[0]));used.add(pair[0]['id'])
    return picked


def revise(source,output):
    if output.exists():raise FileExistsError('Fresh output required')
    original=gen.geo.load_json(source)
    if len(original['scenes'])!=5000:raise ValueError('Exactly 5K source cells required')
    old=[]
    for e in original['scenes']:
        if sha256(Path(e['scenario']))!=e['scenario_sha256']:raise ValueError('Source scenario modified')
        m=gen.geo.load_json(Path(e['metadata']));m['_scenario']=e['scenario'];old.append(m)
    config=copy.deepcopy(original['config']);config['dataset_id']='base5_independent_5k_sunny13_yaw_v129'
    config['static_appearance_yaw']=dict(policy=POLICY,cardinal_percent=70,uniform_percent=30,
        cardinal_angles_deg=[0,90,180,-90],cardinal_jitter_deg=15,
        moving_and_route_vehicle_yaw='Unchanged; no arbitrary motion yaw jitter',
        rigid_obstacle_generic_yaw='Existing uniform + barricade road-aligned mixture preserved')
    config['stationary_character_spawn']=dict(policy='near-person-spawn-envelope-v129',
        primary_pool='person',phase='alongside',minimum_requested_surface_gap_m=.6,
        evidence='Man2 matched gap probe: .30/.45m missing; .60/.80m captured. Conservative person-only prior, not collider certification.')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    eligible=sorted({a['data_id'] for pool in PED_POOLS for a in ctx['pools'].get(pool,[])})
    rows=[copy.deepcopy(m['spec']) for m in old];groups=defaultdict(list)
    character_gap_changes=[]
    for i,m in enumerate(old):
        scope=target_scope(m)
        if not scope:continue
        r=rows[i]
        r['static_yaw_policy']=dict(policy=POLICY,scope=scope,eligible_data_ids=eligible)
        r['primary_identity']=m['runtime_actors'][0]['data_id']
        r['primary_pool_override']=m['runtime_actors'][0]['population_pool']
        if scope=='stationary_avoidance' and r['primary_pool_override']=='person' and r.get('avoidance_phase')=='alongside':
            lo,hi=r['avoidance_gap_bounds_m']
            if lo<.6:
                r['character_spawn_gap_original_bounds_m']=[lo,hi]
                r['character_spawn_gap_policy']='near-person-spawn-envelope-v129'
                r['avoidance_gap_bounds_m']=[.6,max(.8,hi)]
                character_gap_changes.append(dict(id=original['scenes'][i]['id'],old=[lo,hi],new=r['avoidance_gap_bounds_m']))
        groups[(scope,r['primary_pool_override'])].append(i)
    for group,indices in sorted(groups.items()):
        rng=random.Random(261008127+sum(map(ord,str(group))))
        modes=gen.exact_deck([('cardinal',70),('uniform',30)],len(indices),rng)
        for i,mode in zip(indices,modes):rows[i]['static_yaw_policy']['primary_mode']=mode
    output.mkdir(parents=True);directory=output/'scenarios_5k';directory.mkdir()
    metas=[];entries=[];errors=Counter();tries=Counter();changed=[];preserved=[]
    for number,(e,prior,row) in enumerate(zip(original['scenes'],old,rows)):
        if row.get('static_yaw_policy'):
            # Try the old layout seed first; retry only when rotated footprints cannot fit.
            start=prior.get('retry',0)
            for step in range(512):
                attempt=(start+step)%512
                try:scene,meta=gen.build(row,config,ctx,attempt);break
                except ValueError as exc:errors[str(exc)]+=1
            else:
                atomic_json(output/'failed_cell.json',dict(id=e['id'],spec=row,rejections=dict(errors)))
                raise ValueError('Unfillable yaw cell '+e['id'])
            tries[str(step)]+=1;changed.append(e['id'])
            if meta['runtime_actors'][0]['data_id']!=prior['runtime_actors'][0]['data_id']:
                raise ValueError('Primary identity changed')
        else:
            scene=gen.geo.load_json(Path(e['scenario']));meta=copy.deepcopy(prior);meta.pop('_scenario',None)
            preserved.append(e['id'])
        meta['scene_group_id']=e['id']
        meta['environment_split_contract']='Same scene split across six environments; no environment-level split'
        sp=directory/(e['id']+'.json');mp=directory/(e['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(e);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),
            scenario_sha256=sha256(sp),split=meta['split'],scene_group_id=e['id'])
        metas.append(meta);entries.append(new)
        if number%250==0:print(json.dumps(dict(generated=number+1,changed=len(changed),rejections=sum(errors.values()))),flush=True)
    quotas=validate_quotas(metas)
    primary=Counter(m['runtime_actors'][0]['data_id'] for m in metas if m['runtime_actors'])
    oldprimary=Counter(m['runtime_actors'][0]['data_id'] for m in old if m['runtime_actors'])
    if primary!=oldprimary:raise ValueError('Primary coverage changed')
    frozen=output/'frozen_specs.json';atomic_json(frozen,[m['spec'] for m in metas])
    config.update(frozen_specs_path=str(frozen.resolve()),frozen_specs_sha256=sha256(frozen))
    deck=copy.deepcopy(original);deck.update(config=config,dataset_id=config['dataset_id'],scenes=entries,
        status='OFFLINE_YAW_V129; live pilot pending; bulk not started')
    deck['provenance'].update(yaw_source_manifest_sha256=sha256(source),yaw_revision_sha256=sha256(__file__),
        generator_sha256=sha256(gen.__file__),static_yaw_policy_sha256=sha256(Path(__file__).with_name('base5_static_yaw.py')),
        avoidance_geometry_sha256=sha256(Path(__file__).with_name('base5_avoidance_geometry.py')),
        config_sha256=gen.hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest())
    manifest=directory/'manifest.json';atomic_json(manifest,deck);atomic_json(output/'config.json',config)
    report=dict(policy=POLICY,source_manifest=str(source),source_sha256=sha256(source),manifest_sha256=sha256(manifest),
        changed_cells=len(changed),preserved_payloads=len(preserved),changed_ids=changed,
        primary_assets=len(primary),primary_floor=min(primary.values()),primary_coverage_unchanged=True,
        quotas=quotas,retry_steps=dict(tries),rejections=dict(errors),before_yaw=yaw_hist(old),after_yaw=yaw_hist(metas),
        character_gap_changes=character_gap_changes,
        split_changes=sum(a['split']!=b['split'] for a,b in zip(old,metas)),
        split_note='Regenerated scenes keep whole-link split rule. Six environment variants share new split; old raw is separate reference only.',
        bulk_started=False)
    atomic_json(output/'revision_report.json',report)
    strata=select_pilot(entries,metas);pilot=copy.deepcopy(deck);pilot['scenes']=[s['scene'] for s in strata]
    pilot['status']='PREREGISTERED_YAW_PILOT; not full collection'
    atomic_json(output/'pilot_manifest.json',pilot)
    atomic_json(output/'pilot_strata.json',dict(strata=[dict(tag=s['tag'],id=s['scene']['id']) for s in strata]))
    ordered=prepare_order(manifest,output/'ordered')
    prepare(ordered,output/'environment_suite','base5_yaw_v129','collection_5k_yaw_v129')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(revise(a.source,a.output),ensure_ascii=False))
