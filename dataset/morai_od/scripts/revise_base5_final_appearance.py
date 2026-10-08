#!/usr/bin/env python3
"""Fresh 5K release: static free yaw, measured parent envelopes, tunnel direction balance."""
import argparse
import copy
import json
import random
from collections import Counter,defaultdict
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json,sha256
from base5_placement_geometry import POLICY as GEOMETRY_POLICY
from base5_static_yaw import FINAL_POLICY
from base5_region_contract import validate_quotas
from prepare_base5_environment_suite import prepare as prepare_suite
from prepare_base5_batch_order import prepare as prepare_order


def scope(meta):
    r=meta['spec']
    if (not meta['runtime_actors'] or r['family'] in ('highway','roundabout','background')
            or r.get('shadow_moving') or r.get('dynamic_observation_phase')
            or any(a['motion'] not in ('static','standing') for a in meta['runtime_actors'])):
        return None
    if r.get('shadow_phase'):return 'shadow_static'
    if r.get('avoidance_phase') and r['mission']:return 'stationary_avoidance'
    if not r['mission']:return 'generic'
    return None


def assign_yaw(rows,metas,ctx):
    assets={a['data_id']:a for pool in ctx['pools'].values() for a in pool}
    eligible=sorted(assets)
    obj_ids=sorted(a['data_id'] for a in ctx['pools']['static_vehicle'])
    groups=defaultdict(list)
    for i,meta in enumerate(metas):
        sc=scope(meta)
        if not sc:continue
        r=rows[i];a=meta['runtime_actors'][0]
        r.update(primary_identity=a['data_id'],primary_pool_override=a['population_pool'])
        r['static_yaw_policy']=dict(policy=FINAL_POLICY,scope=sc,eligible_data_ids=eligible,obj_vehicle_data_ids=obj_ids)
        # Ordinary traffic primary has no arbitrary yaw, even with mixed secondaries.
        if sc=='generic' and a['source_list']=='vehicleList':continue
        groups[(sc,a['population_pool'])].append(i)
    for group,indices in sorted(groups.items()):
        sc,pool=group;road=70 if pool=='vehicle' else 30 if pool=='static_vehicle' else 0
        rng=random.Random(261008130+sum(map(ord,str(group))))
        modes=gen.exact_deck([('road_aligned',road),('uniform',100-road)],len(indices),rng)
        free=[i for i,mode in zip(indices,modes) if mode=='uniform']
        sectors=gen.exact_deck([(j,1) for j in range(12)],len(free),rng)
        for i,mode in zip(indices,modes):rows[i]['static_yaw_policy']['primary_mode']=mode
        for i,sector in zip(free,sectors):rows[i]['static_yaw_policy']['primary_sector']=sector


def assign_shadow_direction(rows,metas):
    groups=defaultdict(list)
    for i,m in enumerate(metas):
        if m['spec'].get('shadow_moving'):groups[m['spec']['motion']].append(i)
    for motion,indices in sorted(groups.items()):
        rng=random.Random(261008130+len(motion))
        directions=gen.exact_deck([('approaching',1),('receding',1)],len(indices),rng)
        for i,direction in zip(indices,directions):
            r=rows[i];approaching_sign=-1 if r['role']=='front' else 1
            r['shadow_direction_sign']=approaching_sign if direction=='approaching' else -approaching_sign
            r['shadow_relative_motion']=direction
            r['shadow_relative_motion_policy']='longitudinal-approach-recede-balanced-v130'


def pilot_strata(entries,metas):
    pairs=list(zip(entries,metas));conditions=[]
    for sc in ('generic','stationary_avoidance','shadow_static'):
        for pool in ('obstacle','static_vehicle','person','animal','pet_person','stroller','rider','dummy'):
            conditions.append((sc+'_'+pool,lambda m,sc=sc,pool=pool:
                m['spec'].get('static_yaw_policy',{}).get('scope')==sc and
                (m['runtime_actors'][0]['population_pool']==pool or
                 (pool=='obstacle' and m['runtime_actors'][0]['population_pool'].startswith('obstacle_')))))
    for sector in (0,1,3,5,7,9,11):
        conditions.append(('free_sector_'+str(sector),lambda m,sector=sector:
            m['spec'].get('static_yaw_policy',{}).get('primary_sector')==sector))
    for mode in ('road_aligned','uniform'):
        conditions.append(('parked_vehicle_'+mode,lambda m,mode=mode:
            m['spec'].get('static_yaw_policy',{}).get('scope')=='stationary_avoidance'
            and m['runtime_actors'][0]['population_pool']=='vehicle'
            and m['spec']['static_yaw_policy']['primary_mode']==mode))
    for side in ('left','right'):
        for pool in ('static_vehicle','stroller','pet_person','rider','animal'):
            conditions.append(('near_'+side+'_'+pool,lambda m,side=side,pool=pool:
                m['spec'].get('avoidance_phase')=='alongside' and m['spec']['avoidance_side']==side
                and m['runtime_actors'][0]['population_pool']==pool))
    for mode in ('generic_regular','generic_dense'):
        conditions.append((mode,lambda m,mode=mode:m['spec'].get('population_mode')==mode
                           and m['spec'].get('static_yaw_policy')))
    for motion in ('walking','running'):
        for direction in ('approaching','receding'):
            conditions.append(('shadow_'+motion+'_'+direction,lambda m,motion=motion,direction=direction:
                m['spec'].get('shadow_moving') and m['spec']['motion']==motion
                and m['spec']['shadow_relative_motion']==direction))
    for name in ('Dog1','Dog2','Deer1','Ped_Pet1'):
        conditions.append(('shadow_move_'+name,lambda m,name=name:m['spec'].get('shadow_moving')
                          and m['runtime_actors'][0]['name']==name))
    conditions += [('highway_control',lambda m:m['spec']['family']=='highway'),
                   ('roundabout_control',lambda m:m['spec']['family']=='roundabout'),
                   ('crossing_move_control',lambda m:m['spec'].get('dynamic_observation_phase')
                    and any(a['motion'] not in ('static','standing') for a in m['runtime_actors'])),
                   ('background_control',lambda m:m['spec']['family']=='background')]
    selected=[];used=set();missing=[]
    for tag,predicate in conditions:
        match=next(((e,m) for e,m in pairs if e['id'] not in used and predicate(m)),None)
        if match is None:
            covered=next((e for e,m in pairs if e['id'] in used and predicate(m)),None)
            if covered:
                next(s for s in selected if s['id']==covered['id']).setdefault('also_covers',[]).append(tag)
            else:missing.append(tag)
            continue
        selected.append(dict(tag=tag,id=match[0]['id']));used.add(match[0]['id'])
    return selected,missing


def revise(source,reference,output):
    if output.exists():raise FileExistsError('Fresh release required; never overwrite raw')
    original=gen.geo.load_json(source)
    if len(original['scenes'])!=5000:raise ValueError('Exactly 5K input required')
    old=[]
    for e in original['scenes']:
        if sha256(e['scenario'])!=e['scenario_sha256']:raise ValueError('Old input modified')
        old.append(gen.geo.load_json(Path(e['metadata'])))
    config=copy.deepcopy(original['config']);config['dataset_id']='base5_independent_5k_sunny13_final_v131'
    config['static_appearance_yaw']=dict(policy=FINAL_POLICY,free_sectors=12,sector_width_deg=30,
        rigid_and_stationary_character_uniform_percent=100,obj_vehicle_uniform_percent=70,
        parked_vehicle_road_aligned_percent=70,road_aligned_jitter_deg=15,
        moving_and_traffic_policy='Keep actual path yaw; standing intersection actors retain crossing context')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    from base5_map_surface import configure
    configure(ctx,config.get('placement_ground'))
    rows=[copy.deepcopy(m['spec']) for m in old]
    assign_yaw(rows,old,ctx);assign_shadow_direction(rows,old)
    output.mkdir(parents=True);directory=output/'scenarios_5k';directory.mkdir()
    copied=output/'placement_geometry.json';atomic_json(copied,gen.geo.load_json(reference))
    config['static_placement_geometry']=dict(policy=GEOMETRY_POLICY,path=str(copied.resolve()),sha256=sha256(copied))
    entries=[];metas=[];rejections=Counter();tries=Counter();changed=[];preserved=[]
    for n,(e,prior,row) in enumerate(zip(original['scenes'],old,rows)):
        regenerate=bool(row.get('static_yaw_policy',{}).get('policy')==FINAL_POLICY or row.get('shadow_moving'))
        if regenerate:
            for step in range(512):
                retry=prior.get('retry',0)+step
                try:scene,meta=gen.build(row,config,ctx,retry);break
                except ValueError as exc:rejections[str(exc)]+=1
            else:
                atomic_json(output/'failed_cell.json',dict(id=e['id'],row=row,rejections=dict(rejections)))
                raise ValueError('Unfillable final appearance cell '+e['id'])
            tries[str(step)]+=1;changed.append(e['id'])
            if meta['runtime_actors'][0]['data_id']!=prior['runtime_actors'][0]['data_id']:
                raise ValueError('Primary identity changed')
        else:
            scene=gen.geo.load_json(Path(e['scenario']));meta=copy.deepcopy(prior);preserved.append(e['id'])
        meta['scene_group_id']=e['id']
        meta['environment_split_contract']='Same geometry retains same split across six environment observations'
        sp=directory/(e['id']+'.json');mp=directory/(e['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(e);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
            split=meta['split'],scene_group_id=e['id'])
        entries.append(new);metas.append(meta)
        if n%250==0:print(json.dumps(dict(generated=n+1,changed=len(changed),rejections=sum(rejections.values()))),flush=True)
    quotas=validate_quotas(metas)
    primary=lambda ms:Counter(m['runtime_actors'][0]['data_id'] for m in ms if m['runtime_actors'])
    if primary(old)!=primary(metas):raise ValueError('Primary quotas changed')
    # Future same-quota replacements use exactly these modes/sectors/directions.
    frozen=output/'frozen_specs.json';atomic_json(frozen,[m['spec'] for m in metas])
    config.update(frozen_specs_path=str(frozen.resolve()),frozen_specs_sha256=sha256(frozen))
    config['shadow_observation']['spec_overrides']=[m['spec'] for m in metas if m['spec'].get('shadow_phase')]
    deck=copy.deepcopy(original);deck.update(config=config,dataset_id=config['dataset_id'],scenes=entries,
        status='FINAL_V131_OFFLINE_GENERATED; representative live pending; bulk not started')
    deck['provenance'].update(final_source_manifest_sha256=sha256(source),final_revision_sha256=sha256(__file__),
        generator_sha256=sha256(gen.__file__),static_yaw_sha256=sha256(Path(__file__).with_name('base5_static_yaw.py')),
        placement_geometry_sha256=sha256(copied),config_sha256=gen.hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest())
    manifest=directory/'manifest.json';atomic_json(manifest,deck);atomic_json(output/'config.json',config)
    strata,missing=pilot_strata(entries,metas)
    by_id={e['id']:e for e in entries};pilot=copy.deepcopy(deck)
    pilot.update(scenes=[by_id[s['id']] for s in strata],count=len(strata),status='PREREGISTERED_FINAL_V131_PILOT')
    atomic_json(output/'pilot_manifest.json',pilot);atomic_json(output/'pilot_strata.json',dict(strata=strata,unavailable=missing))
    ordered=prepare_order(manifest,output/'ordered')
    prepare_suite(ordered,output/'environment_suite','base5_final_v131','collection_5k_final_v131')
    report=dict(changed_cells=len(changed),preserved_payloads=len(preserved),changed_ids=changed,
        primary_assets=len(primary(metas)),primary_floor=min(primary(metas).values()),primary_coverage_unchanged=True,
        quotas=quotas,rejections=dict(rejections),retry_steps=dict(tries),pilot_cells=len(strata),unavailable_pilot_strata=missing,
        split_changes=sum(a['split']!=b['split'] for a,b in zip(old,metas)),
        bulk_started=False,manifest_sha256=sha256(manifest))
    atomic_json(output/'revision_report.json',report)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=revise(a.source,a.reference,a.output)
    print(json.dumps({k:v for k,v in result.items() if k!='changed_ids'},ensure_ascii=False))
