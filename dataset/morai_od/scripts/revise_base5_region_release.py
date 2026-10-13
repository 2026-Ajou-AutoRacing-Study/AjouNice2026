#!/usr/bin/env python3
"""Fresh region-conditional 5K from the validated v122 release; never bulk-capture."""
import argparse
import copy
import json
import random
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_region_contract import POLICY, QUOTAS, category, validate, validate_quotas


def revise(source, output):
    if output.exists(): raise FileExistsError('Fresh output required')
    original=gen.geo.load_json(source)
    if len(original['scenes'])!=5000:raise ValueError('Complete v122 5K required')
    old=[gen.geo.load_json(Path(e['metadata'])) for e in original['scenes']]
    config=copy.deepcopy(original['config']);config['dataset_id']='base5_independent_5k_sunny13_regions_v123'
    config['regional_policy']=dict(policy=POLICY, corridor_half_width_m=4.5,
        physical_obstacles_removed_from_traffic='Replace in permitted generic scenes; do not suppress GT',
        obstacle_compensation_mixed_scenes=270,
        semantic_quotas={k:dict(generic=g,mission=m) for k,(g,m) in QUOTAS.items()})
    config['families']={k:sum(v) for k,v in QUOTAS.items()}
    config['mission_counts']={k:v[1] for k,v in QUOTAS.items()}
    config['environment_suite']=dict(total_frames=30000,frames_per_environment=5000,
        environments=[dict(weather=w,hour=h) for w in ('sunny','foggy') for h in (11,13,15)],
        split_policy='All six variants of the same scene share split and scene_group_id',
        setting_policy='Operator UI verification; manifest does not set weather/time')
    plan=config['mission_roundabout_observation']
    plan['phase_counts']=dict(approach=70,entry=80,circulation=130,exit=70)
    plan['approach_states']=dict(approaching=30,waiting=40)
    rows=[copy.deepcopy(m['spec']) for m in old]
    for r,m in zip(rows,old):
        if r.get('shadow_phase'):r['family']='shadow'
        if m['runtime_actors']:
            r['primary_identity']=m['runtime_actors'][0]['data_id']
            r['primary_pool_override']=m['runtime_actors'][0]['population_pool']
        if r['family']=='roundabout':
            r.update(mission=True,population_mode='mission_situation',population_layout='mission')
            r.pop('population_budget_bounds',None)
        if r['family'] in ('highway','roundabout'):r['vehicle_only_population']=True
    # Reuse four overrepresented generic slots; never consume a mission,
    # close observation, rare primary or shadow cell to round the totals.
    recycled=[]
    for family,n in (('obstacle',1),('pedestrian_related',3)):
        eligible=[i for i,r in enumerate(rows) if r['family']==family and not r['mission']
                  and not r.get('avoidance_phase') and (family!='pedestrian_related' or r['subfamily']=='person')]
        chosen=eligible[-n:];recycled.extend(chosen)
    for i,family in zip(recycled,('highway','highway','other_vehicle','other_vehicle')):
        template=next(r for r in rows if r['family']==family and not r['mission'] and not r.get('avoidance_phase'))
        r=copy.deepcopy(template);r.update(index=rows[i]['index'],family_index=10000+i,seed=261008123+i,
                                         asset_index=i,primary_identity=template['primary_identity'])
        rows[i]=r
    # Exact generic direction/distance/population decks. Close observations
    # are reassigned only inside eligible near bins, never replacing far bins.
    for family in ('highway','other_vehicle','obstacle','pedestrian_related'):
        generic=[r for r in rows if r['family']==family and not r['mission']]
        rng=random.Random(261008123+sum(map(ord,family)))
        directions=[('front',1),('rear',1)] if family=='highway' else [('front',4),('left',2),('right',2),('rear',2)] if family=='obstacle' else [('front',5),('left',2),('right',2),('rear',1)]
        modes=gen.exact_deck(list(config['population_budget']['generic_modes'].items()),len(generic),rng)
        layouts=gen.exact_deck(list(config['population_budget']['generic_layouts'].items()),len(generic),rng)
        for r,role,mode,layout in zip(generic,gen.exact_deck(directions,len(generic),rng),modes,layouts):
            for key in ('avoidance_phase','avoidance_side','avoidance_origin','avoidance_gap_bounds_m','original_distance_bounds','original_role'):
                r.pop(key,None)
            r.update(role=role,population_mode=mode,population_layout=layout,actors=1,
                     population_max_actors=config['population_budget']['max_actors'])
            if mode=='generic_single':r.pop('population_budget_bounds',None)
            else:r['population_budget_bounds']=config['population_budget']['budget_bounds'][mode]
        if family=='pedestrian_related':
            mixed=[r for r in generic if r['population_mode']!='generic_single']
            rng.shuffle(mixed)
            for r in mixed[:config['regional_policy']['obstacle_compensation_mixed_scenes']]:
                # Only a bounded subset is conditioned, not every mixed scene.
                r['required_secondary_pool']='obstacle'
        for role in ('front','rear','left','right'):
            group=[r for r in generic if r['role']==role]
            key=('obstacle_'+('side' if role in ('left','right') else role) if family=='obstacle' else
                 family+'_side' if family in ('pedestrian_related','other_vehicle') and role in ('left','right') else family)
            bins=config['distance_decks'][key]
            for r,b in zip(group,gen.exact_deck([(i,b[2]) for i,b in enumerate(bins)],len(group),rng)):
                r.update(distance_bin=b,distance_bounds=bins[b][:2])
        if family=='other_vehicle':
            for r in generic:
                if not r['static_vehicle'] and r['role'] in ('left','right'):
                    r['topology']='route_general_two_lane_'+('right' if r['role']=='left' else 'left')
        if family in ('obstacle','other_vehicle'):
            cp=config['generic_close_observation'];eligible=[r for r in generic if r['distance_bounds'][1]<=cp['eligible_max_center_distance_m']
                and (family!='other_vehicle' or r['static_vehicle'])]
            rng.shuffle(eligible);chosen=eligible[:round(len(eligible)*cp['eligible_fraction_percent']/100)]
            for r,side,gap in zip(chosen,gen.exact_deck([('left',1),('right',1)],len(chosen),rng),
                                 gen.exact_deck([(b[:2],b[2]) for b in cp['surface_gap_bins_m']],len(chosen),rng)):
                r.update(original_distance_bounds=r['distance_bounds'][:],original_role=r['role'],
                    avoidance_origin='generic_close',avoidance_phase='alongside',avoidance_side=side,
                    avoidance_gap_bounds_m=gap,distance_bounds=[0.,30.],distance_bin='generic_surface_gap',
                    role='right' if side=='left' else 'left')
    gen.roundabout_mission.assign(gen,rows,config)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    distance_swaps=[]
    # A 12m bus cannot physically fill a 6–10m directly-ahead center ring
    # (Ego origin is rearward of its center). Swap the ring with a smaller
    # primary in the SAME family/direction, preserving both asset and bin quotas.
    vehicles={a['data_id']:a for a in ctx['pools']['vehicle']}
    generic=[r for r in rows if r['family']=='highway' and not r['mission'] and r['role']=='front']
    for r in generic:
        length=gen.geometry(vehicles[r['primary_identity']],ctx)[0][0]
        minimum=(length+4.8)/2+1.35+.3
        if r['distance_bounds'][1]>minimum:continue
        donor=next(q for q in generic if q['distance_bounds'][0]>=10 and
            gen.geometry(vehicles[q['primary_identity']],ctx)[0][0]<6)
        distance_swaps.append([r['index'],donor['index']])
        for key in ('distance_bounds','distance_bin'):r[key],donor[key]=donor[key],r[key]
    output.mkdir(parents=True);directory=output/'scenarios_5k';directory.mkdir()
    metas=[];entries=[];rejections=Counter();attempts=Counter();regenerated=0;preserved_geometry=0
    for number,(e,prior,r) in enumerate(zip(original['scenes'],old,rows)):
        if sha256(Path(e['scenario']))!=e['scenario_sha256']:raise ValueError('Source modified')
        # Every generic positive is rebuilt; all 500 shadow geometries and all
        # non-roundabout mission geometries remain exactly the prior release.
        rebuild=category(r)!='shadow' and r['family']!='background' and (not r['mission'] or r['family']=='roundabout')
        if not rebuild:
            scene=gen.geo.load_json(Path(e['scenario']));meta=copy.deepcopy(prior);meta['spec']=r
            proof=validate(gen,ctx,meta,config)
            if proof:meta['regional_placement']=proof
            preserved_geometry+=1
        else:
            for attempt in range(512):
                try:scene,meta=gen.build(r,config,ctx,attempt);break
                except ValueError as exc:rejections[str(exc)]+=1
            else:
                atomic_json(output/'failed_cell.json',dict(id=e['id'],spec=r,rejections=dict(rejections)))
                raise ValueError('Unfillable region cell '+e['id'])
            attempts[str(attempt)]+=1;regenerated+=1
        meta['scene_group_id']=e['id']
        meta['environment_split_contract']='Same scene split across six environments; no random environment-level split'
        sp=directory/(e['id']+'.json');mp=directory/(e['id']+'_metadata.json')
        atomic_json(sp,scene);atomic_json(mp,meta)
        new=copy.deepcopy(e);new.update(scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
            split=meta['split'],family=r['family'],mission=r['mission'],role=r['role'],subfamily=r.get('subfamily'),
            distance_bounds=r.get('distance_bounds',[]),motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else 'static',
            scene_group_id=e['id'])
        entries.append(new);metas.append(meta)
        if number%250==0:print(json.dumps(dict(generated=number+1,regenerated=regenerated,rejections=sum(rejections.values()))),flush=True)
    quotas=validate_quotas(metas)
    primary=Counter(m['runtime_actors'][0]['data_id'] for m in metas if m['runtime_actors'])
    if len(primary)!=122 or min(primary.values())<4:raise ValueError('Primary coverage floor lost')
    frozen=output/'frozen_specs.json';atomic_json(frozen,[m['spec'] for m in metas])
    config.update(frozen_specs_path=str(frozen.resolve()),frozen_specs_sha256=sha256(frozen))
    config['ped_subfamilies']=dict(Counter(r.get('subfamily') for r in rows if r['family']=='pedestrian_related'))
    config['shadow_observation']['spec_overrides']=[r for r in rows if r.get('shadow_phase')]
    deck=copy.deepcopy(original);deck.update(config=config,dataset_id=config['dataset_id'],scenes=entries,
        status='OFFLINE_REGIONS_V123; live pilot pending; bulk not started')
    deck['provenance'].update(region_source_manifest_sha256=sha256(source),region_revision_sha256=sha256(__file__),
        generator_sha256=sha256(gen.__file__),region_policy_sha256=sha256(Path(__file__).with_name('base5_region_contract.py')),
        config_sha256=gen.hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest())
    atomic_json(directory/'manifest.json',deck);atomic_json(output/'config.json',config)
    atomic_json(output/'revision_report.json',dict(quotas=quotas,regenerated=regenerated,preserved_geometry=preserved_geometry,
        source_manifest=str(source),recycled_ids=[original['scenes'][i]['id'] for i in recycled],
        primary_assets=len(primary),primary_min=min(primary.values()),distance_identity_swaps=distance_swaps,
        attempts=dict(attempts),rejections=dict(rejections),bulk_started=False))
    selected=[];used=set()
    conditions=[lambda m,p=p,d=d:m['spec'].get('roundabout_phase')==p and m['spec'].get('roundabout_density')==d
                for p in ('approach','entry','circulation','exit') for d in ('tight','normal','loose')]
    conditions += [lambda m,b=b,r=r:category(m['spec'])=='highway' and not m['spec']['mission'] and
                   m['spec']['distance_bounds']==b and m['spec']['role']==r for b in ([6,10],[30,45],[80,99]) for r in ('front','rear')]
    conditions += [lambda m,f=f,mode=mode:category(m['spec'])==f and not m['spec']['mission'] and
                   m['spec']['population_mode']==mode for f in ('obstacle','pedestrian_related','other_vehicle')
                   for mode in ('generic_single','generic_dense')]
    conditions += [
        lambda m:m['spec'].get('avoidance_origin')=='generic_close',
        lambda m:m['spec'].get('dynamic_observation_phase')=='crossing' and any(a['motion']=='running' for a in m['runtime_actors']),
        lambda m:m['spec'].get('dynamic_observation_phase')=='yielding' and any(a['motion']=='walking' for a in m['runtime_actors']),
        lambda m:m['spec'].get('shadow_observation_stage')=='approach_far' and bool(m['runtime_actors']),
        lambda m:category(m['spec'])=='background']
    conditions += [lambda m,k=k:category(m['spec'])=='pedestrian_related' and not m['spec']['mission']
                   and m['runtime_actors'][0]['population_pool']==k for k in ('animal','pet_person','stroller','rider')]
    for condition in conditions:
        pair=next(((e,m) for e,m in zip(entries,metas) if e['id'] not in used and condition(m)),None)
        if pair is None:raise ValueError('Missing preregistered pilot stratum')
        selected.append(pair[0]);used.add(pair[0]['id'])
    pilot=copy.deepcopy({k:v for k,v in deck.items() if k!='scenes'});pilot.update(scenes=selected,count=len(selected))
    atomic_json(output/'pilot_manifest.json',pilot)
    return directory/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(revise(a.source_manifest,a.output))
