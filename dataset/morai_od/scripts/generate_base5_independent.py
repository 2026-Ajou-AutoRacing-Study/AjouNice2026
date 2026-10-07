#!/usr/bin/env python3
"""Independent Base5 v2. Offline generation only; never writes MR runtime folders."""
import argparse
import copy
import hashlib
import json
import math
import random
import sys
from collections import Counter
from pathlib import Path

import generate_dynamic_object_capture_set as geo
import generate_static_classcentric_base3_batch as b3
import build_base5_mr_asset_catalog as catalog
import base5_avoidance_geometry as avoidance
import base5_population_budget as population
import base5_vehicle_close as vehicle_close
from base5_reference_frames import references

CONFIG = geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_independent_5k_sunny11.json'
GEOMETRY = geo.PROJECT_ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json'
LIVE_GEOMETRY = geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_stationary_geometry_mr_261006.json'
APPEARANCE_POLICY = geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_asset_appearance_policy_v1.json'
HIGHWAY_SAMPLE = geo.PROJECT_ROOT/'configs/scenario/2026_molit_comp_sample_scene.json'

def sample_highway_corridors(links):
    """Sample reference-point paths, clipped at start/end pointIdx (not box edges)."""
    paths={6:('406','409','420','179','1105'),
           8:('175','410','430','434','422'),
           10:('029','418','431','435','423','432')}
    spawns={a['UNIQUEID']:a for a in geo.load_json(HIGHWAY_SAMPLE)['spawnPointList']}
    corridors=[]
    for uid,suffixes in paths.items():
        spawn=spawns[uid];path=['A2256W'+s.zfill(6) for s in suffixes]
        if path[0]!=spawn['startLinkInfo']['linkIdx'] or path[-1]!=spawn['endLinkInfo']['linkIdx']:
            raise ValueError('Sample NPC path endpoints changed')
        for left,right in zip(path,path[1:]):
            if links[left]['to_node_idx']!=links[right]['from_node_idx']:
                raise ValueError('Disconnected sample NPC path')
        clipped=copy.deepcopy(links)
        clipped[path[0]]['points']=clipped[path[0]]['points'][spawn['startLinkInfo']['pointIdx']:]
        clipped[path[-1]]['points']=clipped[path[-1]]['points'][:spawn['endLinkInfo']['pointIdx']+1]
        route,arc=geo.route_from_link_path(clipped,path)
        for p in route:
            if p['link_idx']==path[0]: p['point_idx']+=spawn['startLinkInfo']['pointIdx']
            p['point_ratio']=p['point_idx']/max(1,len(links[p['link_idx']]['points'])-1)
        corridors.append(dict(spawn_uid=uid,path=path,start=spawn['startLinkInfo'],
            end=spawn['endLinkInfo'],route=route,arc=arc))
    return corridors

def mission_vehicle_candidates(ctx,ego,lower,upper,direction):
    candidates=[];a=math.radians(ego['yaw'])
    for corridor in ctx['sample_corridors']:
        for j,p in enumerate(corridor['route']):
            dx,dy=p['x']-ego['x'],p['y']-ego['y']
            lon=math.cos(a)*dx+math.sin(a)*dy
            if lower-3<=math.hypot(dx,dy)<upper+3 and lon*direction>0:
                candidates.append((corridor,j))
    return candidates

def exact_deck(weighted, count, rng):
    total = sum(float(w) for _, w in weighted)
    raw = [count*float(w)/total for _,w in weighted]
    counts = [int(v) for v in raw]
    for i in sorted(range(len(raw)), key=lambda i: raw[i]-counts[i], reverse=True)[:count-sum(counts)]:
        counts[i] += 1
    values = [v for (v,_),n in zip(weighted,counts) for _ in range(n)]
    rng.shuffle(values)
    return values

def specs(config):
    if sum(config['families'].values()) != config['target_scenes'] or config['capture']['per_scene'] != 1:
        raise ValueError('Independent scene count contract violated')
    output = []
    for fidx,(family,n) in enumerate(config['families'].items()):
        rng = random.Random(config['seed']+fidx*7919)
        missions = exact_deck([(True,config['mission_counts'][family]),(False,n-config['mission_counts'][family])],n,rng)
        roles = exact_deck(([('front',1),('rear',1)] if family=='highway' else
                           [('front',4),('left',2),('right',2),('rear',2)] if family=='obstacle' else
                           [('front',5),('left',2),('right',2),('rear',1)]),n,rng)
        subfamilies = exact_deck(list(config['ped_subfamilies'].items()),n,rng) if family=='pedestrian_related' else [None]*n
        yaws = exact_deck([(3,6),(10,3),(15,1)],n,rng)
        multiplicity = exact_deck([(1,70),(2,21),(3,6),(4,3)],n,rng) if family!='background' else [0]*n
        motion = exact_deck([('standing',20),('walking',50),('running',30)],n,rng)
        topologies = exact_deck([('highway_merge_lane',2),('highway_three_lane',5),('highway_post_taper_two_usable',2),('highway_exit_split',1)],n,rng) if family=='highway' else ['roundabout' if family=='roundabout' else 'route_general']*n
        groups=exact_deck([('barrier',45),('box_barrel',35),('bollard',10),('cycle',10)],n,rng) if family=='obstacle' else [None]*n
        static_vehicle=exact_deck([(True,1),(False,1)],n,rng) if family=='other_vehicle' else [False]*n
        for i in range(n):
            output.append(dict(family=family, family_index=i, mission=missions[i], role=roles[i],
                subfamily=subfamilies[i], yaw_limit=yaws[i], actors=multiplicity[i],
                motion=('standing' if subfamilies[i]=='dummy' else motion[i]) if family=='pedestrian_related' else 'static',
                topology=topologies[i],asset_group=groups[i],static_vehicle=static_vehicle[i],seed=config['seed']+fidx*100000+i))
    # Protect small-family quota and DataID coverage independently of shuffled scene order.
    for subfamily,n in config['ped_subfamilies'].items():
        rows=[r for r in output if r['subfamily']==subfamily]
        deck=exact_deck([(True,1),(False,4)],n,random.Random(config['seed']+len(subfamily)))
        for r,mission in zip(rows,deck): r['mission']=mission
    # Static avoidance references are single-lane: never demand an NPC in a
    # nonexistent adjacent lane there. Preserve 150 static / 150 ordinary cars.
    ordinary=[r for r in output if r['family']=='other_vehicle' and not r['mission']]
    static_deck=exact_deck([(True,1),(False,3)],len(ordinary),random.Random(config['seed']+31))
    for r,flag in zip(ordinary,static_deck):r['static_vehicle']=flag
    for r in output:
        if r['family']=='other_vehicle' and r['mission']:r['static_vehicle']=True
    counters=Counter()
    for r in output:
        key=(r['family'],r['subfamily'],r['asset_group'],r['static_vehicle'])
        r['asset_index']=counters[key];counters[key]+=1
    random.Random(config['seed']+17).shuffle(output)
    for i,row in enumerate(output): row['index']=i
    # Conditional distance decks keep each direction's quota, not only global counts.
    for family in config['families']:
        for role in ('front','left','right','rear'):
            rows=[r for r in output if r['family']==family and r['role']==role]
            if not rows or family=='background': continue
            key='obstacle_'+('side' if role in ('left','right') else role) if family=='obstacle' else family
            if family=='pedestrian_related' and role in ('left','right'): key='pedestrian_related_side'
            if family=='other_vehicle' and role in ('left','right'): key='other_vehicle_side'
            bins=config['distance_decks'][key]
            bins_deck=exact_deck([(i,b[2]) for i,b in enumerate(bins)],len(rows),random.Random(config['seed']+len(rows)+sum(map(ord,family+role))))
            for row,b in zip(rows,bins_deck): row.update(distance_bin=b,distance_bounds=bins[b][:2])
    if config.get('mission_avoidance'):
        plan=config['mission_avoidance']
        for family in ('obstacle','other_vehicle'):
            rows=[r for r in output if r['family']==family and r['mission']]
            rng=random.Random(config['seed']+sum(map(ord,family))+73)
            phases=exact_deck(list(plan['phases'].items()),len(rows),rng)
            sides=exact_deck([('left',1),('right',1)],len(rows),rng)
            for row,phase,side in zip(rows,phases,sides):
                row.update(avoidance_phase=phase,avoidance_side=side,
                    avoidance_gap_bounds_m=[.3,.5],distance_bounds=[0.,100.],distance_bin='avoidance_geometry',
                    role=('right' if side=='left' else 'left') if phase=='alongside' else 'front' if phase in ('approach','entry') else 'rear')
            # Exact side balance inside each phase, not just the family total.
            for phase in plan['phases']:
                subset=[r for r in rows if r['avoidance_phase']==phase]
                for row,side in zip(subset,exact_deck([('left',1),('right',1)],len(subset),rng)):
                    row['avoidance_side']=side
                    if phase=='alongside':row['role']='right' if side=='left' else 'left'
            subset=[r for r in rows if r['avoidance_phase']=='alongside']
            deck=exact_deck([(b[:2],b[2]) for b in plan['alongside_gap_bins_m']],len(subset),rng)
            for row,bounds in zip(subset,deck):row['avoidance_gap_bounds_m']=bounds
    dynamic=config.get('mission_dynamic_observation')
    if dynamic:
        rows=[r for r in output if r['family']=='pedestrian_related' and r['mission']]
        phases=exact_deck(list(dynamic['phases'].items()),len(rows),random.Random(config['seed']+904))
        for row,phase in zip(rows,phases):
            row.update(dynamic_observation_phase=phase,distance_bin='dynamic_'+phase,
                distance_bounds=dynamic['distance_bounds_m'][phase],role='rear' if phase=='clearing' else 'front')
    plan=config.get('population_budget')
    if plan:
        for family in config['families']:
            if family=='background':continue
            generic=[r for r in output if r['family']==family and not r['mission']]
            rng=random.Random(config['seed']+sum(map(ord,family))+903)
            modes=exact_deck(list(plan['generic_modes'].items()),len(generic),rng)
            layouts=exact_deck(list(plan['generic_layouts'].items()),len(generic),rng)
            for r,mode,layout in zip(generic,modes,layouts):
                r.update(population_mode=mode,population_layout=layout,
                    population_max_actors=plan['max_actors'],actors=1)
                if mode!='generic_single':
                    r['population_budget_bounds']=plan['budget_bounds'][mode]
            missions=[r for r in output if r['family']==family and r['mission']]
            counts=exact_deck([(int(k),v) for k,v in plan['mission_actor_counts'][family].items()],len(missions),rng)
            for r,n in zip(missions,counts):
                r.update(actors=n,population_mode='mission_situation',population_layout='mission')
    close=config.get('generic_close_observation')
    if close:
        for family in close['families']:
            eligible=[r for r in output if r['family']==family and not r['mission']
                and r['distance_bounds'][1]<=close['eligible_max_center_distance_m']
                and (family!='other_vehicle' or r['static_vehicle'])]
            rng=random.Random(config['seed']+sum(map(ord,family))+905)
            rng.shuffle(eligible)
            chosen=eligible[:round(len(eligible)*close['eligible_fraction_percent']/100)]
            sides=exact_deck([('left',1),('right',1)],len(chosen),rng)
            gaps=exact_deck([(b[:2],b[2]) for b in close['surface_gap_bins_m']],len(chosen),rng)
            for row,side,bounds in zip(chosen,sides,gaps):
                row.update(original_distance_bounds=list(row['distance_bounds']),original_role=row['role'],
                    avoidance_origin='generic_close',avoidance_phase='alongside',avoidance_side=side,
                    avoidance_gap_bounds_m=bounds,distance_bounds=[0.,30.],distance_bin='generic_surface_gap',
                    role='right' if side=='left' else 'left')
    vehicle_close.assign(sys.modules[__name__],output,config)
    assign_stationary_mission_pools(output,config)
    assign_dynamic_mission_vehicles(output,config)
    return output

def assign_dynamic_mission_vehicles(rows,config):
    """Small vehicle observation quota; no velocity supervision or forced motion."""
    count=config.get('mission_dynamic_observation',{}).get('vehicle_primary_scenes',0)
    if not isinstance(count,int) or isinstance(count,bool) or count<0:
        raise ValueError('Dynamic vehicle quota must be a nonnegative integer')
    eligible=[r for r in rows if r['mission'] and r['family']=='pedestrian_related' and r['subfamily']=='person']
    if count>len(eligible):raise ValueError('Dynamic vehicle quota exceeds person mission slots')
    plan=config.get('mission_dynamic_observation',{})
    rng=random.Random(config['seed']+1112)
    phases=exact_deck(list(plan['phases'].items()),count,rng) if count else []
    selected=[]
    for phase in plan.get('phases',{}):
        candidates=[r for r in eligible if r.get('dynamic_observation_phase')==phase]
        rng.shuffle(candidates)
        n=phases.count(phase)
        if n>len(candidates):raise ValueError('Insufficient dynamic vehicle phase slots')
        selected.extend(candidates[:n])
    for index,row in enumerate(selected):
        row.update(dynamic_mission_pool='vehicle',subfamily='vehicle',motion='static',asset_index=index)
        if row['dynamic_observation_phase']=='yielding':
            row['distance_bounds']=list(plan.get('vehicle_yielding_distance_bounds_m',[8,20]))

def assign_stationary_mission_pools(rows,config):
    """Static mission is an observation context, not the Obstacle label class."""
    plan=config.get('stationary_mission_assets')
    if not plan:return
    eligible=[r for r in rows if r['mission'] and r['family']=='obstacle']
    counts=plan['primary_pool_counts']
    allowed={'vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider'}
    if set(counts)!=allowed or any(type(n) is not int or n<1 for n in counts.values()):
        raise ValueError('Static mission requires all nine positive asset pools')
    if sum(counts.values())!=len(eligible):
        raise ValueError('Static mission pool quotas must equal obstacle mission scene count')
    pools=exact_deck(list(counts.items()),len(eligible),random.Random(config['seed']+110))
    counters=Counter()
    for row,pool in zip(eligible,pools):
        row.update(stationary_mission_pool=pool,asset_index=counters[pool],motion='static')
        counters[pool]+=1
    # The original index counter also included generic cars. A sparse shuffled
    # subset can under-cover a static asset; cycle the 50 mission slots directly.
    for index,row in enumerate(r for r in rows if r['mission'] and r['family']=='other_vehicle'):
        row.update(stationary_mission_pool='static_vehicle',asset_index=index,motion='static')

def context(runtime, mgeo):
    inventory=json.loads(catalog.DEFAULT_CATALOG.read_text())
    geometry_rows=geo.load_json(GEOMETRY)['assets']
    geometry_sources={str(GEOMETRY):geo.sha256(GEOMETRY)}
    for row in inventory['assets']:
        if row.get('geometric_gt_reference'):
            ref=geo.PROJECT_ROOT/row['geometric_gt_reference']
            geometry_rows.update(geo.load_json(ref)['assets']);geometry_sources[str(ref)]=geo.sha256(ref)
    if LIVE_GEOMETRY.exists():
        geometry_rows.update(geo.load_json(LIVE_GEOMETRY)['assets'])
        geometry_sources[str(LIVE_GEOMETRY)]=geo.sha256(LIVE_GEOMETRY)
    documents={f:geo.load_json(runtime/f) for f in inventory['source_sha256']}
    templates={}
    for f,doc in documents.items():
        for container in ('vehicleList','pedestrianList','aiPedstrianList','objectList'):
            for actor in doc.get(container,[]): templates.setdefault((f,container,actor['DataID']),actor)
    pools={k:[] for k in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider')}
    for row in inventory['assets']:
        if int(row['data_id']) in catalog.COMPETITION_EXCLUDED_IDS:
            continue
        name=row['name']; key=(row['source_file'],row['source_list'],row['data_id'])
        asset=dict(row,template=templates[key])
        if row['collection_role']=='positive':
            if row.get('semantic_class')=='vehicle': pool='vehicle' if row['source_list']=='vehicleList' else 'static_vehicle'
            elif row.get('semantic_class')=='obstacle': pool='obstacle'
            elif row.get('semantic_class')=='pedestrian': pool='dummy' if 'NCAP' in name else 'person'
            else: continue
            pools[pool].append(asset)
        elif row.get('ontology_class')=='Animal': pools['animal'].append(asset)
        elif name.startswith('Ped_Pet'): pools['pet_person'].append(asset)
        elif 'Stroller' in name: pools['stroller'].append(asset)
        elif ('Bike' in name or 'Bicycle' in name or 'Scooter' in name): pools['rider'].append(asset)
    for key,pool in pools.items():
        if not pool: raise ValueError('Empty pool '+key)
    groups={k:[] for k in ('barrier','box_barrel','bollard','cycle')}
    for a in pools['obstacle']:
        n=a['name'].lower()
        group='bollard' if 'bollard' in n else 'cycle' if ('bicycle' in n or 'scooter' in n) else 'barrier' if any(t in n for t in ('barrier','barricade','firewall','separation')) else 'box_barrel'
        groups[group].append(a)
    for k,v in groups.items(): pools['obstacle_'+k]=v
    links={r['idx']:r for r in geo.load_json(mgeo/'link_set.json')}
    guide,_=geo.load_route(geo.DEFAULT_ROUTE)
    route=geo.build_mgeo_guided_competition_route(guide,list(links.values()))
    if isinstance(route,tuple): route=route[0]
    mission_templates={}
    validator=geo.PROJECT_ROOT/'base_ws/src/planning_addons/planning_scenario_validator/config/missions'
    for mission in ('static_obstacle','dynamic_obstacle'):
        paths=sorted((validator/mission/'base/scenarios').glob('*.json'))
        mission_templates[mission]=[(str(p),geo.load_json(p)) for p in paths]
    return dict(pools=pools,links=links,route=route,arc=geo.route_arc_coordinates(route),appearance_policy=geo.load_json(APPEARANCE_POLICY),
        geometry=geometry_rows,geometry_sources=geometry_sources,template=geo.load_json(geo.DEFAULT_TEMPLATE),anchors={},
        mission_templates=mission_templates,crosswalks=geo.load_crosswalk_catalog(mgeo),
        sample_corridors=sample_highway_corridors(links),
        lane_boundaries={r['idx']:r for r in geo.load_json(mgeo/'lane_boundary_set.json')} if (mgeo/'lane_boundary_set.json').exists() else {},
        static_height_reference_sources=dict(scenarios={f:geo.sha256(runtime/f) for f in documents},
            map_link_sha256=geo.sha256(mgeo/'link_set.json'),
            map_boundary_sha256=geo.sha256(mgeo/'lane_boundary_set.json') if (mgeo/'lane_boundary_set.json').exists() else None))

def anchor_bank(ctx,topology):
    if topology in ctx['anchors']: return ctx['anchors'][topology]
    bank=[]
    family=dict(count=192,roundabout_phases=b3.ROUNDABOUT_PHASES,global_spatial_stratum_schedule=[topology]*192)
    for i in range(192):
        route,arc,p,details=geo.static_spatial_target(family,i,random.Random(17000+i),ctx['route'],ctx['arc'],ctx['links'])
        bank.append((route,arc,p,details))
    ctx['anchors'][topology]=bank
    return bank

def geometry(asset,ctx):
    row=ctx['geometry'].get(asset['name'])
    if row: return row['bbox_size_m'],row['pivot_to_bbox_center_local_m'],True
    cls=asset.get('ontology_class')
    size=[5.,2.,1.8] if cls=='Vehicle' else [1.1,.5,.9] if cls=='Animal' else [1.,1.,1.9]
    if cls=='Obstacle':size=[3.,3.,2.5] # conservative placement proxy, never training GT
    if any(s in asset['name'].lower() for s in ('truck','bus','xcient')): size=[12.,2.8,3.5]
    return size,[0.,0.,size[2]/2],False

def dynamic_crossing_candidates(ctx,reference,plan):
    """Map crossings near the mission starter, perpendicular to the Ego route."""
    starter=reference['egoVehicle']['initPosition']['pos'];out=[]
    for key,cw in sorted(ctx['crosswalks'].items()):
        pts=cw['points'];axis=cw.get('pedestrian_axis')
        if not pts or not axis:continue
        center={k:sum(p[k] for p in pts)/len(pts) for k in ('x','y','z')}
        if math.hypot(center['x']-starter['x'],center['y']-starter['y'])>plan['maximum_starter_distance_m']:continue
        j=geo.nearest_index(ctx['route'],center['x'],center['y'])
        road=geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],float(ctx['arc'][j]))
        if math.hypot(center['x']-road['x'],center['y']-road['y'])>plan['maximum_crosswalk_route_distance_m']:continue
        a=math.radians(road['yaw'])
        if abs(axis[0]*math.cos(a)+axis[1]*math.sin(a))>plan['maximum_axis_route_dot']:continue
        out.append(dict(crosswalk_id=key,center=center,axis=list(axis),route_arc_m=float(ctx['arc'][j])))
    if not out:raise ValueError('No route-crossing dynamic mission candidate')
    return out

def crossing_target(crossing,rng,jitter,direction_sign=None):
    if direction_sign is not None and direction_sign not in (-1,1):raise ValueError('Invalid crossing direction')
    t=rng.uniform(-jitter,jitter)
    return dict(crossing['center'],x=crossing['center']['x']+t*crossing['axis'][0],
        y=crossing['center']['y']+t*crossing['axis'][1],
        yaw=math.degrees(math.atan2(crossing['axis'][1],crossing['axis'][0]))+
            (rng.choice([0.,180.]) if direction_sign is None else 0. if direction_sign==1 else 180.))

def assign_crossing_directions(rows,ctx,config):
    """Balance actual moving primaries per crossing and walk/run; retries retain the sign."""
    if not config.get('mission_dynamic_observation',{}).get('balance_crossing_directions'):return
    plan=config['mission_dynamic_observation']
    crossings=dynamic_crossing_candidates(ctx,ctx['mission_templates']['dynamic_obstacle'][0][1],plan)
    groups={}
    for row in rows:
        if not row.get('dynamic_observation_phase') or row.get('dynamic_mission_pool'):continue
        asset=ctx['pools'][row['subfamily']][row['asset_index']%len(ctx['pools'][row['subfamily']])]
        motion,_=population_appearance(asset,row['motion'],row,0,0,config,ctx.get('appearance_policy',{}))
        if motion in ('static','standing'):continue
        crossing=crossings[row['family_index']%len(crossings)]['crosswalk_id']
        groups.setdefault((crossing,motion,row['subfamily']),[]).append(row)
    totals=Counter()
    for group,subset in sorted(groups.items()):
        rng=random.Random(config['seed']+sum(map(ord,str(group)))+117)
        aggregate=group[:2]
        preferred=1 if totals[aggregate]<0 else -1
        signs=[-1,1]*(len(subset)//2)
        if len(subset)%2:signs.append(preferred)
        rng.shuffle(signs)
        for row,sign in zip(subset,signs):
            row['crossing_direction_sign']=sign
            totals[aggregate]+=sign

def dynamic_vehicle_target(ctx,crossing,rng,offset):
    """Use nearby crossing-direction road links, never a pedestrian walkway yaw."""
    cache=ctx.setdefault('dynamic_vehicle_anchors',{})
    key=crossing['crosswalk_id']
    if key not in cache:
        poses=[]
        for link_id,link in sorted(ctx['links'].items()):
            points=link['points']
            for j in range(0,len(points)-1,2):
                p,q=points[j:j+2];dx,dy=q[0]-p[0],q[1]-p[1];length=math.hypot(dx,dy)
                if length<.1:continue
                if math.hypot(p[0]-crossing['center']['x'],p[1]-crossing['center']['y'])>12.:continue
                if abs((dx*crossing['axis'][0]+dy*crossing['axis'][1])/length)<.8:continue
                poses.append(dict(x=p[0],y=p[1],z=p[2],yaw=math.degrees(math.atan2(dy,dx)),
                    link_id=link_id,link_idx=link_id,point_index=j,point_ratio=j/(len(points)-1)))
        cache[key]=poses
    if not cache[key]:raise ValueError('No road-supported dynamic mission vehicle anchor')
    pivot=dict(rng.choice(cache[key]));a=math.radians(pivot['yaw'])
    return dict(pivot,x=pivot['x']+math.cos(a)*offset[0]-math.sin(a)*offset[1],
        y=pivot['y']+math.sin(a)*offset[0]+math.cos(a)*offset[1],road_pivot=pivot)

def pivot_for_center(target,offset,yaw):
    a=math.radians(yaw)
    return dict(target,x=target['x']-math.cos(a)*offset[0]+math.sin(a)*offset[1],
        y=target['y']-math.sin(a)*offset[0]-math.cos(a)*offset[1],yaw=yaw)

def rectangle(center,size,yaw):
    from shapely.geometry import Polygon
    a=math.radians(yaw); c,s=math.cos(a),math.sin(a)
    return Polygon([(center[0]+c*x-s*y,center[1]+s*x+c*y) for x,y in
                    [(-size[0]/2,-size[1]/2),(-size[0]/2,size[1]/2),(size[0]/2,size[1]/2),(size[0]/2,-size[1]/2)]])

def bounded_motion_path(pose,center,ego,bounds):
    """Short tangent shuttle: same radial cell throughout, closed without teleport."""
    dx,dy=center[0]-ego['x'],center[1]-ego['y'];radius=math.hypot(dx,dy)
    lo,hi=bounds
    if not lo<=radius<hi-.025:raise ValueError('Dynamic range reserve missing')
    length=min(4.,math.sqrt(max(0.,(hi-.025)**2-radius**2)))
    if length<.5:raise ValueError('Dynamic shuttle too short')
    tangent=(-dy/radius,dx/radius)
    end=dict(x=pose['x']+length*tangent[0],y=pose['y']+length*tangent[1],z=pose['z'])
    return dict(policy='bounded-shuttle-v1',length_m=length,tangent=list(tangent),distance_bounds_m=list(bounds),
        points=[{k:pose[k] for k in ('x','y','z')},end,{k:pose[k] for k in ('x','y','z')}],
        limitation='Single-frame animated appearance only; not a competition trajectory or velocity label.')


def appearance_motion(asset,requested,policy):
    if asset['source_list']=='aiPedstrianList':return 'standing','ai_motion_unverified'
    if asset['source_list']=='vehicleList':return 'static','vehicle_static_capture'
    if asset['source_list']=='objectList' and asset.get('semantic_class') in ('obstacle','vehicle'):
        return 'static','official_static_asset'
    if asset['name'] in policy.get('static_primary',[]):return 'standing' if requested!='static' else 'static','observed_fixed_pose'
    if asset['name'] in policy.get('articulated_observed',[]):return requested,'observed_articulated'
    if asset.get('ontology_class')=='Animal' or asset.get('semantic_class')=='pedestrian':
        return requested,'articulated_candidate_requires_capture_validation'
    return 'static' if requested=='static' else 'standing','unverified_static_pending_appearance_review'


def validate_natural_path_roi(points,target,center,ego,road_yaw):
    # Translate pivot waypoints to the same geometric center used by the auditor.
    # For a straight path, endpoint checks bound every intermediate center.
    dx,dy=center[0]-target['x'],center[1]-target['y']
    for point in points:
        ref=references([point['x']+dx,point['y']+dy],ego,road_yaw)
        if abs(ref['ego_lateral_m'])>29.5 or abs(ref['ego_longitudinal_m'])>99.5:
            raise ValueError('Natural geometric-center path leaves input ROI')


def population_appearance(asset,requested,row,index,active_count,config,policy):
    from base5_intersection_placement import regional_motion
    regional=regional_motion(asset,requested,row,config)
    if regional is not None:return regional
    motion,decision=appearance_motion(asset,requested,policy)
    maximum=config['capture'].get('generic_max_animated_actors')
    if maximum is not None and not 1<=maximum<=8:
        raise ValueError('Generic animated population limit must be 1–8')
    if (maximum is not None and index>0 and not row.get('mission') and active_count>=maximum
            and motion not in ('static','standing')):
        return 'standing','generic-concurrent-motion-cap-standing'
    return motion,decision


def natural_target_path(pose,center,ego,bounds,motion,asset,budget=9.,direction=None):
    """Target is the quota position; departure is backwards at natural appearance speed."""
    if not 6.<=budget<=30.:raise ValueError('Natural target setup budget out of bounds')
    speed=1.2 if asset.get('ontology_class')=='Animal' else 2.2 if motion=='running' else 1.1
    dx,dy=center[0]-ego['x'],center[1]-ego['y'];r=math.hypot(dx,dy)
    if r<.1:raise ValueError('Natural target at Ego')
    # Tangential crossing keeps the close target from driving through Ego.
    direction=direction or (-dy/r,dx/r);yaw=math.degrees(math.atan2(direction[1],direction[0]))
    start=dict(x=pose['x']-direction[0]*speed*budget,y=pose['y']-direction[1]*speed*budget,z=pose['z'])
    end=dict(x=pose['x']+direction[0]*3.,y=pose['y']+direction[1]*3.,z=pose['z'])
    return dict(policy='natural-target-v1',points=[start,end],target_position={k:pose[k] for k in ('x','y','z')},
        target_tolerance_m=.4,arrival_timeout_sec=20.,setup_budget_sec=budget,departure_yaw_deg=yaw,
        distance_bounds_m=list(bounds),nominal_speed_mps=speed,speed_field_value=speed*3.6,
        length_m=speed*budget+3.,nominal_duration_sec=budget+3./speed,
        ground_path_required=True,
        limitation='Unverified constant-Z proposal. No capture until independent terrain path preparation. Natural appearance only, not velocity training GT.')

def one_way_motion_path(pose,center,ego,bounds,motion,asset,setup_budget_sec=12.,appearance_profile=False):
    """Heading-aligned finite segment; slow enough to survive normal setup, no return."""
    a=math.radians(pose['yaw']);dx,dy=math.cos(a),math.sin(a)
    lo,hi=bounds
    length=None
    for tenth in range(160 if appearance_profile else 40,4,-1):
        candidate=tenth/10.
        cx,cy=center[0]-ego['x'],center[1]-ego['y']
        closest=max(0.,min(candidate,-cx*dx-cy*dy))
        minimum=math.hypot(cx+closest*dx,cy+closest*dy)
        maximum=max(math.hypot(cx,cy),math.hypot(cx+candidate*dx,cy+candidate*dy))
        # Exact segment extrema, not just 21 discrete points through a radial cell.
        if lo<=minimum and maximum<hi-.025:
            length=candidate;break
    if length is None:raise ValueError('One-way segment too short inside distance cell')
    # Raw scene speed fields are treated as km/h candidates; actual displacement
    # must be measured in live validation, especially objectList.moveSpeed.
    nominal=.25 if asset.get('ontology_class')=='Animal' else .6 if motion=='running' else .35
    if appearance_profile:
        nominal=1.2 if asset.get('ontology_class')=='Animal' else 2.2 if motion=='running' else 2.5 if asset.get('name','').startswith(('Bike','Scooter','Bicycle')) else 1.1
    if not 4.<=setup_budget_sec<=30.:raise ValueError('Bounded one-way setup budget required')
    speed=min(nominal,length/setup_budget_sec)
    end=dict(x=pose['x']+length*dx,y=pose['y']+length*dy,z=pose['z'])
    return dict(policy='one-way-slow-v1',length_m=length,distance_bounds_m=list(bounds),
        points=[{k:pose[k] for k in ('x','y','z')},end],speed_field_value=speed*3.6,
        nominal_speed_mps=speed,nominal_duration_sec=length/speed,appearance_profile=appearance_profile,
        limitation='Single-frame appearance probe; reduced-speed animation realism and field units need live validation.')


def set_pose(actor,pose,uid,asset,scene,motion,motion_path=None):
    position={k:pose[k] for k in ('x','y','z')}
    container=asset['source_list']
    if container=='vehicleList':
        geo.set_static_vehicle_actor(actor,pose,position,uid)
        saved=actor['initPosition'].get('vehicleSaveData',{})
        saved['velocity']=0.
        geo.zero_vector(saved.get('vehicle',{}).get('lastVelocity'))
    else:
        geo.set_actor_world_position(actor,position,pose['yaw'])
        actor['UNIQUEID']=uid
        if container=='aiPedstrianList':
            # AI police uses destinations/customSpeed, not ordinary waypoints.
            # Keep the asset, but do not invent a moving-AI interface contract.
            actor.update(speed=0.,customSpeed=0.,activeDistance=0.)
            scene[container].append(actor)
            return
        if container=='objectList':
            actor.update(scale=geo.formatted_vector(1.,1.,1.),moveSpeed=0.,isWaypointMode=False,waypointEventList=[])
            if motion not in ('static','standing'):
                actor.update(isWaypointMode=True,moveSpeed=4. if motion=='walking' else 8.,activeDistance=1000.)
                a=math.radians(pose['yaw']);end=dict(x=pose['x']+8*math.cos(a),y=pose['y']+8*math.sin(a),z=pose['z'])
                scene['waypointDataList'].append(dict(pathName='ObstaclePath - '+str(uid),waypointData=[
                    dict(pos=geo.world_to_morai_waypoint(position),rot=geo.formatted_rotation(0.)),
                    dict(pos=geo.world_to_morai_waypoint(end),rot=geo.formatted_rotation(0.))]))
        else:
            actor.update(active=True,loop=False,activeDistance=1000.,speed=0.,
                         isWaypointMode=False,waypointEventList=[],movingDistanceAmount=0.)
            if motion not in ('static','standing'):
                speed=3.6 if asset.get('ontology_class')=='Animal' else 4. if motion=='walking' else 8.
                actor.update(speed=speed,isWaypointMode=True,waypointEventList=[dict(index=-1,waypointEventData=[])]*2)
                # Cross the view, not into Ego. The capture wait is short; endpoint stays on a bounded segment.
                a=math.radians(pose['yaw'])
                end=dict(x=pose['x']+8*math.cos(a),y=pose['y']+8*math.sin(a),z=pose['z'])
                scene['waypointDataList'].append(dict(pathName='PedPath - '+str(uid),waypointData=[
                    dict(pos=geo.world_to_morai_waypoint(position),rot=geo.formatted_rotation(0.)),
                    dict(pos=geo.world_to_morai_waypoint(end),rot=geo.formatted_rotation(0.))]))
        if motion_path is not None:
            # One-way probes use exactly two points. Historical shuttle uses
            # finite contiguous returns without respawn/teleport.
            points=motion_path['points'] if motion_path['policy'] in ('one-way-slow-v1','natural-target-v1') else motion_path['points'][:2]*64+[motion_path['points'][0]]
            if motion_path['policy'] in ('one-way-slow-v1','natural-target-v1'):
                actor['moveSpeed' if container=='objectList' else 'speed']=motion_path['speed_field_value']
            actor['waypointEventList']=[dict(index=-1,waypointEventData=[]) for _ in points]
            scene['waypointDataList'][-1]['waypointData']=[
                dict(pos=geo.world_to_morai_waypoint(p),rot=geo.formatted_rotation(0.)) for p in points]
    scene[container].append(actor)

def build(row,config,ctx,attempt=0):
    row=copy.deepcopy(row)
    from base5_map_surface import configure
    ctx=configure(ctx,config.get('placement_ground'))
    from base5_map_surface import MapSurface,ego_road_support
    if 'map_surface_index' not in ctx:
        ctx['map_surface_index']=MapSurface(ctx['links'],ctx.get('lane_boundaries'))
    if row.get('dynamic_mission_pool'):
        # Low-quota crossing observations use passenger/small vehicles. Large
        # buses/trucks cannot fit the existing 5–10m yielding observation cell.
        limit=config['mission_dynamic_observation'].get('vehicle_max_length_m',6.)
        pool=[a for a in ctx['pools']['vehicle'] if geometry(a,ctx)[0][0]<=limit]
        if len(pool)<config['mission_dynamic_observation']['vehicle_primary_scenes']:
            raise ValueError('Insufficient distinct small vehicles for dynamic quota')
        ctx=dict(ctx,pools=dict(ctx['pools'],vehicle=pool))
    rng=random.Random(row['seed']+attempt*10000019)
    poolkey=row['subfamily'] if row['family']=='pedestrian_related' else 'obstacle' if row['family']=='obstacle' else 'vehicle'
    if row['family']=='other_vehicle' and row['static_vehicle']: poolkey='static_vehicle'
    if row.get('dynamic_mission_pool'):poolkey=row['dynamic_mission_pool']
    if row['family']=='obstacle': poolkey='obstacle_'+row['asset_group']
    if row.get('stationary_mission_pool'):
        if not row['mission'] or row['family'] not in ('obstacle','other_vehicle') or row['motion']!='static':
            raise ValueError('Expanded stationary pool is restricted to the static mission')
        poolkey=row['stationary_mission_pool']
    asset_index=row['asset_index']
    if row.get('dynamic_mission_pool'):
        asset_index=asset_index*len(ctx['pools'][poolkey])//config['mission_dynamic_observation']['vehicle_primary_scenes']
    asset=ctx['pools'][poolkey][asset_index%len(ctx['pools'][poolkey])]
    route,arc,base,details=rng.choice(anchor_bank(ctx,row['topology']))
    base=dict(base);dynamic_crossing=None;dynamic_target=None
    if row['mission'] and row['family'] in ('obstacle','pedestrian_related','other_vehicle'):
        kind='dynamic_obstacle' if row['family']=='pedestrian_related' else 'static_obstacle'
        if not ctx['mission_templates'][kind]: raise ValueError('Mission starter references missing')
        ref,reference=rng.choice(ctx['mission_templates'][kind])
        pos=reference['egoVehicle']['initPosition']['pos']
        j=geo.nearest_index(ctx['route'],pos['x'],pos['y'])
        route,arc=ctx['route'],ctx['arc']
        # Ego observations at approach, yielding, passing and clearing positions,
        # not always the starter's same fixed view. Only genuine local route anchors.
        if row.get('dynamic_observation_phase'):
            plan=config['mission_dynamic_observation']
            candidates=dynamic_crossing_candidates(ctx,reference,plan)
            # Deterministic crossing coverage; retries do not silently pick an easier crossing.
            dynamic_crossing=candidates[row['family_index']%len(candidates)]
            dynamic_target=(dynamic_vehicle_target(ctx,dynamic_crossing,rng,geometry(asset,ctx)[1])
                if row.get('dynamic_mission_pool') else crossing_target(dynamic_crossing,rng,plan['target_axis_jitter_m'],
                    row.get('crossing_direction_sign')))
            lower,upper=row['distance_bounds'];distance=rng.uniform(lower+.5,upper-.5)
            direction=1 if row['dynamic_observation_phase']=='clearing' else -1
            target_arc=dynamic_crossing['route_arc_m']+direction*distance
        else:target_arc=float(arc[j])+rng.uniform(-35.,55.)
        base=geo.continuous_route_pose_at_arc(route,arc,max(0.,min(float(arc[-1]),target_arc)))
        details=dict(spatial_stratum=kind,mission_reference=ref,
            mission_reference_sha256=geo.sha256(Path(ref)),anchor_route_displacement_m=target_arc-float(arc[j]))
        if dynamic_crossing:
            details['dynamic_observation']=dict(phase=row['dynamic_observation_phase'],crossing=dynamic_crossing,
                target_center=dynamic_target,ego_crossing_arc_displacement_m=target_arc-dynamic_crossing['route_arc_m'],
                limitation='Map-anchored observation; not a validated trigger or actor trajectory. Terrain preparation required for motion.')
    if row['family']=='highway':
        if row['topology']=='highway_three_lane':
            routes=[geo.route_from_link_path(ctx['links'],p) for p in geo.HIGHWAY_LANE_LINK_PATHS.values()]
            route,arc=rng.choice(routes)
        elif details.get('highway_route_links'):
            path=details['highway_route_links'];first=path[0]
            for full in geo.HIGHWAY_LANE_LINK_PATHS.values():
                if first in full:
                    path=list(full[:full.index(first)])+path;break
            if first=='A2256W000411':path=['A2256W000846']+path
            route,arc=geo.route_from_link_path(ctx['links'],path)
        lower,upper=row['distance_bounds']
        margin=upper+8
        if arc[-1] < margin+15: raise ValueError('Short road for requested range')
        lo=margin if row['role']=='rear' else 8
        hi=arc[-1]-margin if row['role']=='front' else arc[-1]-8
        if details.get('highway_target_link'):
            interval=geo.link_arc_interval(route,arc,details['highway_target_link'])
            lo=max(lo,interval[0]);hi=min(hi,interval[1])
        if hi<=lo: raise ValueError('No feasible anchor interval')
        base=geo.continuous_route_pose_at_arc(route,arc,rng.uniform(lo,hi))
    yaw_floor={3:0.,10:3.,15:10.}[row['yaw_limit']]
    yaw_delta=rng.choice([-1,1])*rng.uniform(yaw_floor,row['yaw_limit'])
    offset=rng.choice([0.,0.,.15,-.15,.30,-.30])
    if row['topology']!='route_general' and row['topology']!='highway_three_lane': offset=max(-.15,min(.15,offset))
    a=math.radians(base['yaw'])
    # Conservative lane envelope, not an assertion of exact road polygons.
    lane_width=3.2 if row['topology'] in ('roundabout','highway_merge_lane') else 3.5
    envelope=4.8*abs(math.sin(math.radians(yaw_delta)))+2.0*abs(math.cos(math.radians(yaw_delta)))+2*abs(offset)
    if envelope>lane_width: raise ValueError('Ego yaw/offset lane envelope exceeds width proxy')
    ego=dict(base,x=base['x']-math.sin(a)*offset,y=base['y']+math.cos(a)*offset,yaw=base['yaw']+yaw_delta)
    avoidance_ref=None;primary_pose=None
    if row.get('avoidance_phase'):
        ego,primary_pose,avoidance_ref=avoidance.observation(sys.modules[__name__],ctx,route,arc,base,asset,row,rng)
        yaw_delta=ego['yaw']-base['yaw']
        offset=0. if row.get('avoidance_origin')=='generic_close' else avoidance_ref['lateral_center_offset_m']
        if avoidance_ref.get('road_relocation'):
            anchor=avoidance_ref['road_relocation']['road_anchor'];a=math.radians(anchor['yaw'])
            offset=-math.sin(a)*(ego['x']-anchor['x'])+math.cos(a)*(ego['y']-anchor['y'])
    ego_support=ego_road_support(ctx['map_surface_index'],ego)
    if not ego_support['road_supported']:
        # Retry the same spec/quota with another deterministic placement. Do
        # not move Ego alone (that would invalidate the requested close gap),
        # and do not extrapolate road Z across a curb/verge.
        raise ValueError('Final Ego footprint outside MGeo road support')
    ego=dict(ego,z=ego_support['height_m'])
    scene=copy.deepcopy(ctx['template']); geo.clear_actor_lists(scene)
    geo.set_ego_pose(scene['egoVehicle'],ego,initial_speed_kph=0.)
    if config['capture'].get('dynamic_region')=='intersection-only-v116':
        from prepare_base5_intersection_manual_ground import park
        park(scene['egoVehicle'])
    geo.zero_vector(scene['egoVehicle']['initPosition'].get('vehicleSaveData',{}).get('vehicle',{}).get('lastVelocity'))
    ea=math.radians(ego['yaw'])
    footprint=rectangle([ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2.0],ego['yaw'])
    actor_records=[]; footprints=[]
    desired=None
    selected=population.choose(asset,poolkey,ctx['pools'],lambda a:geometry(a,ctx),row,rng)
    row['actors']=len(selected)
    for i in range(row['actors']):
        sample_placement=None;vehicle_close_proof=None
        selected_pool=poolkey
        asset=selected[i]
        if i:
            selected_pool=next(k for k in (poolkey,'vehicle','obstacle','person') if asset in ctx['pools'][k])
        actor_motion=row['motion'] if selected_pool in ('person','animal','pet_person','stroller','rider') else 'static'
        if row.get('stationary_mission_pool'):actor_motion='static'
        requested_motion=actor_motion
        actor_motion,motion_decision=population_appearance(asset,actor_motion,row,i,
            sum(a['motion'] not in ('static','standing') for a in actor_records),config,
            ctx.get('appearance_policy',{}) if config['capture'].get('asset_appearance_policy',False) else {})
        size,off,known=geometry(asset,ctx)
        lower,upper=row['distance_bounds']
        if i: lower,upper=6.,45.
        distance=rng.uniform(lower+.05,upper-.05)
        role=row['role'] if not i else rng.choice(['front','left','right','rear'])
        if i and row.get('population_layout')=='clustered':
            role=row['role']
            if asset['source_list']!='vehicleList':
                # Cluster by view sector, not by cramming eight actors into the
                # primary's narrow 3–5m ring. Primary distance quota stays exact.
                lower=max(3.,min(30.,row['distance_bounds'][0]))
                upper=min(45.,max(lower+15.,row['distance_bounds'][1]))
                if upper<=lower+.1:lower,upper=6.,45.
                distance=rng.uniform(lower+.05,upper-.05)
        elif i and row.get('population_layout')=='distributed':
            role=('front','left','rear','right')[(i+('front','left','rear','right').index(row['role']))%4]
        mission_target=None
        actor_crossing=None
        if dynamic_crossing:
            actor_crossing=dynamic_crossing if i==0 else candidates[(row['family_index']+i)%len(candidates)]
            # Multiple actors on the same narrow crossing produce overlapping swept corridors.
            # Use the neighboring real crossing rather than arbitrary off-road displacement.
            mission_target=dynamic_target if i==0 else (dynamic_vehicle_target(ctx,actor_crossing,rng,off)
                if row.get('dynamic_mission_pool') else crossing_target(actor_crossing,rng,config['mission_dynamic_observation']['target_axis_jitter_m'],
                    row.get('crossing_direction_sign')*((-1)**i) if row.get('crossing_direction_sign') is not None else None))
        if mission_target:
            pose=pivot_for_center(mission_target,off,mission_target['yaw'])
        elif i==0 and primary_pose is not None:
            pose=primary_pose
        elif i==0 and row.get('vehicle_close_phase'):
            pose,sample_placement,vehicle_close_proof=vehicle_close.primary_pose(sys.modules[__name__],ctx,ego,asset,row,rng)
        elif asset['source_list']=='vehicleList':
            direction=1 if role in ('front','left') else -1
            if row['family']=='highway' and row['mission']:
                candidates=mission_vehicle_candidates(ctx,ego,lower,upper,direction)
                if not candidates: raise ValueError('No sample-bounded NPC in requested mission ring')
                corridor,j=rng.choice(candidates)
                pose=geo.continuous_route_pose_at_arc(corridor['route'],corridor['arc'],float(corridor['arc'][j]))
                sample_placement=dict(spawn_uid=corridor['spawn_uid'],route_links=corridor['path'],
                    start=corridor['start'],end=corridor['end'],reference_arc_m=float(corridor['arc'][j]),
                    corridor_length_m=float(corridor['arc'][-1]))
            elif row['family']=='other_vehicle' and role in ('left','right'):
                source=ctx['links'][base['link_idx']]
                destination=source.get(role+'_lane_change_dst_link_idx')
                if not source.get('can_move_'+role+'_lane') or destination not in ctx['links']:
                    raise ValueError('No legal requested ordinary adjacent lane')
                pose=geo.legal_path_pose_near_target(ctx['links'],(destination,),base,rng.uniform(-distance,distance))
            elif row['family']=='roundabout' and (row['mission'] or (size[0]>8. and upper<=10.)):
                nr,na=geo.route_from_link_path(ctx['links'],geo.ROUNDABOUT_NPC_LINK_PATH)
                candidates=[]
                for j in range(0,len(nr),2):
                    p=geo.continuous_route_pose_at_arc(nr,na,float(na[j]))
                    d=math.hypot(p['x']-ego['x'],p['y']-ego['y'])
                    if lower-2<=d<upper+2: candidates.append(p)
                if not candidates: raise ValueError('No roundabout NPC path in requested distance ring')
                pose=rng.choice(candidates)
            else:
                origin=base['route_arc_m']; goal=origin+direction*distance
                if not 0<=goal<=arc[-1]: raise ValueError('Actor outside legal route')
                pose=geo.continuous_route_pose_at_arc(route,arc,goal)
            # Independent real neighboring lane only; no lateral jitter of route NPC.
            if row['family']=='highway' and not row['mission'] and rng.random()<.5:
                neighbors=[]
                for path in geo.HIGHWAY_LANE_LINK_PATHS.values():
                    nr,na=geo.route_from_link_path(ctx['links'],path)
                    j=geo.nearest_index(nr,pose['x'],pose['y']); np=nr[j]
                    gap=math.hypot(np['x']-pose['x'],np['y']-pose['y'])
                    if 2.<gap<5.5:
                        neighbors.append(geo.continuous_route_pose_at_arc(nr,na,float(na[j])))
                if neighbors: pose=dict(rng.choice(neighbors))
        else:
            angles={'front':0.,'left':90.,'right':-90.,'rear':180.}
            bearing=math.radians(ego['yaw']+angles[role]+rng.uniform(-22.,22.))
            # Road-supported long-range front/rear obstacles, tangent local placement for side/near actors.
            if role in ('front','rear') and distance>15. and row['family']=='obstacle':
                goal=base['route_arc_m']+(1 if role=='front' else -1)*distance
                if not 0<=goal<=arc[-1]: raise ValueError('Obstacle beyond road')
                pose=geo.continuous_route_pose_at_arc(route,arc,goal)
            else:
                pose=dict(base,x=ego['x']+distance*math.cos(bearing),y=ego['y']+distance*math.sin(bearing))
            actor_yaw=base['yaw']+rng.uniform(0,360)
            if 'barrier' in asset['name'].lower() or 'barricade' in asset['name'].lower():
                if rng.random()<.5: actor_yaw=base['yaw']+rng.choice([0.,90.])+rng.uniform(-8.,8.)
            pose=dict(pose,yaw=actor_yaw)
            if asset['source_list']!='objectList': pose['yaw']=base['yaw']+rng.choice([-90.,90.])
        pa=math.radians(pose['yaw']); center=[pose['x']+math.cos(pa)*off[0]-math.sin(pa)*off[1],pose['y']+math.sin(pa)*off[0]+math.cos(pa)*off[1]]
        box=rectangle(center,size,pose['yaw'])
        gap=footprint.distance(box)
        minimum_gap=.10 if i==0 and avoidance_ref and row['avoidance_phase']=='alongside' else .3
        if gap<minimum_gap-1e-6 or any(box.distance(other)<.20 for other in footprints): raise ValueError('Footprint clearance failure')
        motion_path=None
        if actor_motion not in ('static','standing'):
            policy=config['capture'].get('dynamic_path_policy')
            if policy=='natural-target-v1':
                direction=None
                if mission_target:
                    # Real crossing axis, not a new Ego-relative tangent on every observation.
                    ma=math.radians(mission_target['yaw']);direction=(math.cos(ma),math.sin(ma))
                motion_path=natural_target_path(pose,center,ego,[lower,upper],actor_motion,asset,
                    config['capture'].get('natural_setup_budget_sec',9.),direction=direction)
                from base5_intersection_placement import apply_path
                motion_path,manual_path_decision=apply_path(motion_path,asset,
                    actor_crossing['crosswalk_id'] if actor_crossing else None,config,ctx)
                # Rotate the target pivot before departure so the geometric quota remains exact.
                pa=math.radians(motion_path['departure_yaw_deg']);pose=dict(pose,yaw=motion_path['departure_yaw_deg'])
                center=[pose['x']+math.cos(pa)*off[0]-math.sin(pa)*off[1],pose['y']+math.sin(pa)*off[0]+math.cos(pa)*off[1]]
                box=rectangle(center,size,pose['yaw']);gap=footprint.distance(box)
                if gap<minimum_gap or any(box.distance(other)<.20 for other in footprints):raise ValueError('Rotated natural target clearance failure')
                end=motion_path['points'][-1]
                end_center=[center[0]+end['x']-pose['x'],center[1]+end['y']-pose['y']]
                validate_natural_path_roi(motion_path['points'],motion_path['target_position'],center,ego,base['yaw'])
            elif policy in ('bounded-shuttle-v1','one-way-slow-v1'):
                if asset['source_list'] not in ('pedestrianList','aiPedstrianList','objectList'):raise ValueError('Shuttle unsupported actor container')
                motion_path=one_way_motion_path(pose,center,ego,[lower,upper],actor_motion,asset,
                    config['capture'].get('one_way_setup_budget_sec',12.),
                    config['capture'].get('one_way_appearance_profile',False)) if policy=='one-way-slow-v1' else bounded_motion_path(pose,center,ego,[lower,upper])
                end=motion_path['points'][1]
                end_center=[center[0]+end['x']-pose['x'],center[1]+end['y']-pose['y']]
                end_lon=math.cos(ea)*(end_center[0]-ego['x'])+math.sin(ea)*(end_center[1]-ego['y'])
                end_lat=-math.sin(ea)*(end_center[0]-ego['x'])+math.cos(ea)*(end_center[1]-ego['y'])
                end_role=('front' if end_lon>=0 else 'rear') if abs(end_lon)>=abs(end_lat) else ('left' if end_lat>=0 else 'right')
                if abs(end_lat)>29.5 or abs(end_lon)>99.5 or (i==0 and end_role!=row['role']):
                    raise ValueError('Dynamic shuttle leaves ROI/direction cell')
            else:end_center=[center[0]+8*math.cos(pa),center[1]+8*math.sin(pa)]
            swept=box.union(rectangle(end_center,size,pose['yaw'])).convex_hull
            if motion_path is not None:
                from shapely.geometry import LineString
                # Bound every heading during the return turn, not just the
                # initial fixed-yaw rectangle. This remains a geometry proxy.
                swept=LineString([(motion_path['points'][0]['x'],motion_path['points'][0]['y']),(end['x'],end['y'])]).buffer(
                    math.hypot(size[0],size[1])/2+math.hypot(off[0],off[1]))
            if swept.distance(footprint)<.3 or any(swept.distance(other)<.2 for other in footprints):
                raise ValueError('Dynamic actor path crosses stationary Ego/other actors')
        actual=math.hypot(center[0]-ego['x'],center[1]-ego['y'])
        if not i and not lower<=actual<upper: raise ValueError('Geometric center outside requested bin')
        if actual>100. or abs(-math.sin(math.radians(ego['yaw']))*(center[0]-ego['x'])+math.cos(math.radians(ego['yaw']))*(center[1]-ego['y']))>29.5:
            raise ValueError('Outside candidate input ROI')
        lon=math.cos(ea)*(center[0]-ego['x'])+math.sin(ea)*(center[1]-ego['y'])
        lat=-math.sin(ea)*(center[0]-ego['x'])+math.cos(ea)*(center[1]-ego['y'])
        actual_role=('front' if lon>=0 else 'rear') if abs(lon)>=abs(lat) else ('left' if lat>=0 else 'right')
        if not i and not avoidance_ref and row['family'] in ('obstacle','pedestrian_related') and actual_role!=row['role']:
            raise ValueError('Actual Ego-relative direction differs from quota')
        if not i and row['family']=='highway' and not row.get('vehicle_close_phase') and ((lon<0)!=(row['role']=='rear')):
            raise ValueError('Highway longitudinal sign differs from quota')
        uid=8500+i
        runtime_pose=dict(pose,**motion_path['points'][0]) if motion_path and motion_path['policy']=='natural-target-v1' else pose
        height_initializer=None;ground_initializer=None
        if actor_motion in ('static','standing'):
            from base5_map_surface import initial_pose
            runtime_pose,height_initializer,ground_initializer=initial_pose(runtime_pose,asset,size,off,ctx)
        set_pose(copy.deepcopy(asset['template']),runtime_pose,uid,asset,scene,actor_motion,motion_path)
        footprints.append(swept if motion_path is not None else box)
        actor_records.append(dict(unique_id=uid,data_id=asset['data_id'],name=asset['name'],
            source_list=asset['source_list'],motion=actor_motion,position={k:runtime_pose[k] for k in ('x','y','z')},
            appearance_motion_decision=motion_decision,requested_motion=requested_motion,
            population_cost=population.cost(size),population_pool=selected_pool,
            geometric_center_estimate=center,geometry_reference_known=known,estimated_size=size,
            planned_center_distance_m=actual,planned_surface_clearance_m=gap,
            longitudinal_m=lon,lateral_m=lat,actual_ego_role=actual_role,
            reference_frames=references(center,ego,base['yaw']),
            training_class='animal' if asset.get('ontology_class')=='Animal' else 'component_annotation',
            runtime_mapping_verified=False if selected_pool in ('animal','pet_person','stroller','rider') else True))
        if height_initializer is not None:actor_records[-1]['static_height_initializer']=height_initializer
        if ground_initializer is not None:actor_records[-1]['map_ground_initializer']=ground_initializer
        if actor_crossing:
            actor_records[-1]['mission_crossing_id']=actor_crossing['crosswalk_id']
        if not i: desired=actual
        if asset['source_list']=='aiPedstrianList':actor_records[-1]['motion_capability_note']='AI destination API unvalidated; retained as standing, not removed'
        if motion_path is not None:
            actor_records[-1]['motion_path']=motion_path
            if motion_path['policy']=='natural-target-v1':
                actor_records[-1]['manual_path_decision']=manual_path_decision
            if motion_path['policy'] in ('one-way-slow-v1','natural-target-v1'):
                actor_records[-1]['motion_capability_note']='Requested walking/running category is not animation certification; actual translation verified live.'
        if sample_placement is not None: actor_records[-1]['sample_highway_placement']=sample_placement
        if vehicle_close_proof is not None:actor_records[-1]['vehicle_close_observation']=vehicle_close_proof
    group=base['link_idx']  # whole-link split; still neighboring-link visibility leakage possible
    split='val' if int(hashlib.sha256(group.encode()).hexdigest()[:8],16)/2**32<.15 else 'train'
    meta=dict(spec=row,ego=ego,base=base,details=details,runtime_actors=actor_records,
        ego_ground_initializer=ego_support,
        split=split,split_group=group,split_limit='whole link; adjacent links can share 100m views; held-out mission bag required',
        ego_road_yaw_offset_deg=yaw_delta,ego_lateral_offset_m=offset,
        planned_primary_distance_m=desired,retry=attempt,live_validated=False)
    if row.get('population_mode'):
        charges=[population.cost(a['estimated_size']) for a in actor_records]
        meta['population_budget']=dict(mode=row['population_mode'],layout=row['population_layout'],
            actor_count=len(actor_records),costs=charges,total_cost=sum(charges),
            requested_bounds=row.get('population_budget_bounds'),
            policy='footprint-area-and-longest-edge-v1; placement count, not visible GT count',
            visibility_certified=False)
        if row['population_mode'] in ('generic_regular','generic_dense'):
            fraction=config['population_budget'].get('minimum_visible_fraction',.5)
            meta['population_budget']['required_visible_actors']=max(2,math.ceil(len(actor_records)*fraction))
    if config['capture'].get('ground_support_check',False):
        meta['ground_support_check']=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15,
            required_uids=[a['unique_id'] for a in actor_records],
            policy='all-actors-observed-ground-v1')
        if config['capture'].get('static_ground_policy'):
            meta['ground_support_check']['static_policy']=config['capture']['static_ground_policy']
    if avoidance_ref:
        avoidance_ref['origin']=row.get('avoidance_origin','mission_static_avoidance')
        avoidance_ref['actual_surface_gap_m']=actor_records[0]['planned_surface_clearance_m']
        meta['avoidance_observation']=avoidance_ref
    from base5_intersection_placement import finish_metadata
    return scene,finish_metadata(meta)

def generate(config,output,start,count,runtime=geo.DEFAULT_RUNTIME_ASSET_DIR,mgeo=geo.DEFAULT_MGEO):
    output=Path(output)
    if output.exists() and any(output.iterdir()): raise FileExistsError('Nonempty output refused')
    all_rows=specs(config)
    if not 0<=start<len(all_rows) or count<1 or start+count>len(all_rows): raise ValueError('Invalid deck interval')
    ctx=context(runtime,mgeo); output.mkdir(parents=True,exist_ok=True)
    assign_crossing_directions(all_rows,ctx,config)
    code_sha256=geo.sha256(Path(__file__))
    entries=[]; failures=[]
    for row in all_rows[start:start+count]:
        reasons=Counter()
        for attempt in range(512):
            try: scene,meta=build(row,config,ctx,attempt); break
            except ValueError as exc:
                reasons[str(exc)]+=1
                if attempt==511:
                    geo.write_json(output/'failed_cell.json',dict(spec=row,reasons=dict(reasons),completed=len(entries)))
                    raise ValueError('Unfillable design cell '+str(row)+' '+str(reasons))
        name='b5v2_'+str(row['index']).zfill(5)
        path=output/(name+'.json'); mpath=output/(name+'_metadata.json')
        geo.write_json(path,scene); geo.write_json(mpath,meta)
        entries.append(dict(id=name,scenario=str(path.resolve()),metadata=str(mpath.resolve()),
            scenario_sha256=geo.sha256(path),split=meta['split'],family=row['family'],mission=row['mission'],
            distance_bounds=row.get('distance_bounds'),role=row['role'],motion=meta['runtime_actors'][0]['motion'] if meta['runtime_actors'] else row['motion'],subfamily=row['subfamily']))
        if (len(entries)%500)==0: print('Generated',len(entries),flush=True)
    manifest=dict(schema_version=2,dataset_id=config['dataset_id'],config=config,start=start,count=count,
        status='OFFLINE_GENERATED; per-scene live validation pending',scenes=entries,
        provenance=dict(config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
            generator_sha256=code_sha256,
            population_budget_sha256=geo.sha256(Path(population.__file__)),
            vehicle_close_sha256=geo.sha256(Path(vehicle_close.__file__)),
            appearance_policy_sha256=geo.sha256(APPEARANCE_POLICY),
            avoidance_geometry_sha256=geo.sha256(Path(avoidance.__file__)),
            map_surface_sha256=geo.sha256(Path(sys.modules['base5_map_surface'].__file__)),
            intersection_placement_sha256=geo.sha256(Path(sys.modules['base5_intersection_placement'].__file__)),
            highway_sample_sha256=geo.sha256(HIGHWAY_SAMPLE),
            geometry_sha256=geo.sha256(GEOMETRY),catalog_sha256=geo.sha256(catalog.DEFAULT_CATALOG),
            geometry_references=ctx['geometry_sources'],
            runtime_sources={k:geo.sha256(runtime/k) for k in json.loads(catalog.DEFAULT_CATALOG.read_text())['source_sha256']},
            map_sha256=geo.sha256(mgeo/'link_set.json'),
            map_boundary_sha256=geo.sha256(mgeo/'lane_boundary_set.json') if (mgeo/'lane_boundary_set.json').exists() else None),
        pool_names={k:[a['name'] for a in v] for k,v in ctx['pools'].items()})
    geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=CONFIG);p.add_argument('--output',type=Path)
    p.add_argument('--start',type=int,default=0);p.add_argument('--count',type=int,default=48)
    p.add_argument('--plan-only',action='store_true');args=p.parse_args()
    c=geo.load_json(args.config)
    if args.plan_only: print(json.dumps(dict(scenes=len(specs(c)),families=Counter(r['family'] for r in specs(c))),indent=2))
    else:
        if not args.output:p.error('--output required')
        print(generate(c,args.output,args.start,args.count))
if __name__=='__main__':main()
