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

CONFIG = geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_independent_v2.json'
GEOMETRY = geo.PROJECT_ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_base3_v1.json'
LIVE_GEOMETRY = geo.PROJECT_ROOT/'dataset/morai_od/configs/base5_stationary_geometry_mr_261006.json'
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
    return output

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
    return dict(pools=pools,links=links,route=route,arc=geo.route_arc_coordinates(route),
        geometry=geometry_rows,geometry_sources=geometry_sources,template=geo.load_json(geo.DEFAULT_TEMPLATE),anchors={},
        mission_templates=mission_templates,crosswalks=geo.load_crosswalk_catalog(mgeo),
        sample_corridors=sample_highway_corridors(links))

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
    if any(s in asset['name'].lower() for s in ('truck','bus','xcient')): size=[12.,2.8,3.5]
    return size,[0.,0.,size[2]/2],False

def rectangle(center,size,yaw):
    from shapely.geometry import Polygon
    a=math.radians(yaw); c,s=math.cos(a),math.sin(a)
    return Polygon([(center[0]+c*x-s*y,center[1]+s*x+c*y) for x,y in
                    [(-size[0]/2,-size[1]/2),(-size[0]/2,size[1]/2),(size[0]/2,size[1]/2),(size[0]/2,-size[1]/2)]])

def set_pose(actor,pose,uid,asset,scene,motion):
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
    scene[container].append(actor)

def build(row,config,ctx,attempt=0):
    rng=random.Random(row['seed']+attempt*10000019)
    poolkey=row['subfamily'] if row['family']=='pedestrian_related' else 'obstacle' if row['family']=='obstacle' else 'vehicle'
    if row['family']=='other_vehicle' and row['static_vehicle']: poolkey='static_vehicle'
    if row['family']=='obstacle': poolkey='obstacle_'+row['asset_group']
    asset=ctx['pools'][poolkey][row['asset_index']%len(ctx['pools'][poolkey])]
    route,arc,base,details=rng.choice(anchor_bank(ctx,row['topology']))
    base=dict(base)
    if row['mission'] and row['family'] in ('obstacle','pedestrian_related','other_vehicle'):
        kind='dynamic_obstacle' if row['family']=='pedestrian_related' else 'static_obstacle'
        if not ctx['mission_templates'][kind]: raise ValueError('Mission starter references missing')
        ref,reference=rng.choice(ctx['mission_templates'][kind])
        pos=reference['egoVehicle']['initPosition']['pos']
        j=geo.nearest_index(ctx['route'],pos['x'],pos['y'])
        route,arc=ctx['route'],ctx['arc']
        # Ego observations at approach, yielding, passing and clearing positions,
        # not always the starter's same fixed view. Only genuine local route anchors.
        target_arc=float(arc[j])+rng.uniform(-35.,55.)
        base=geo.continuous_route_pose_at_arc(route,arc,max(0.,min(float(arc[-1]),target_arc)))
        details=dict(spatial_stratum=kind,mission_reference=ref,
            mission_reference_sha256=geo.sha256(Path(ref)),anchor_route_displacement_m=target_arc-float(arc[j]))
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
        offset=avoidance_ref['lateral_center_offset_m']
    scene=copy.deepcopy(ctx['template']); geo.clear_actor_lists(scene)
    geo.set_ego_pose(scene['egoVehicle'],ego,initial_speed_kph=0.)
    geo.zero_vector(scene['egoVehicle']['initPosition'].get('vehicleSaveData',{}).get('vehicle',{}).get('lastVelocity'))
    ea=math.radians(ego['yaw'])
    footprint=rectangle([ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2.0],ego['yaw'])
    actor_records=[]; footprints=[]
    desired=None
    for i in range(row['actors']):
        sample_placement=None
        selected_pool=poolkey
        if i:
            # Mixed secondary classes without changing the primary family quota.
            selected_pool=rng.choices([poolkey,'vehicle','obstacle','person'],weights=[6,1,2,1])[0]
            pool=ctx['pools'][selected_pool];asset=pool[rng.randrange(len(pool))]
        actor_motion=row['motion'] if selected_pool in ('person','animal','pet_person','stroller','rider') else 'static'
        size,off,known=geometry(asset,ctx)
        lower,upper=row['distance_bounds']
        if i: lower,upper=6.,45.
        distance=rng.uniform(lower+.05,upper-.05)
        role=row['role'] if not i else rng.choice(['front','left','right','rear'])
        if i==0 and primary_pose is not None:
            pose=primary_pose
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
        if actor_motion not in ('static','standing'):
            end_center=[center[0]+8*math.cos(pa),center[1]+8*math.sin(pa)]
            swept=box.union(rectangle(end_center,size,pose['yaw'])).convex_hull
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
        if not i and row['family']=='highway' and ((lon<0)!=(row['role']=='rear')):
            raise ValueError('Highway longitudinal sign differs from quota')
        uid=8500+i
        set_pose(copy.deepcopy(asset['template']),pose,uid,asset,scene,actor_motion)
        footprints.append(box)
        actor_records.append(dict(unique_id=uid,data_id=asset['data_id'],name=asset['name'],
            source_list=asset['source_list'],motion=actor_motion,position={k:pose[k] for k in ('x','y','z')},
            geometric_center_estimate=center,geometry_reference_known=known,estimated_size=size,
            planned_center_distance_m=actual,planned_surface_clearance_m=gap,
            longitudinal_m=lon,lateral_m=lat,actual_ego_role=actual_role,
            training_class='animal' if asset.get('ontology_class')=='Animal' else 'component_annotation',
            runtime_mapping_verified=False if selected_pool in ('animal','pet_person','stroller','rider') else True))
        if not i: desired=actual
        if sample_placement is not None: actor_records[-1]['sample_highway_placement']=sample_placement
    group=base['link_idx']  # whole-link split; still neighboring-link visibility leakage possible
    split='val' if int(hashlib.sha256(group.encode()).hexdigest()[:8],16)/2**32<.15 else 'train'
    meta=dict(spec=row,ego=ego,base=base,details=details,runtime_actors=actor_records,
        split=split,split_group=group,split_limit='whole link; adjacent links can share 100m views; held-out mission bag required',
        ego_road_yaw_offset_deg=yaw_delta,ego_lateral_offset_m=offset,
        planned_primary_distance_m=desired,retry=attempt,live_validated=False)
    if avoidance_ref:
        avoidance_ref['actual_surface_gap_m']=actor_records[0]['planned_surface_clearance_m']
        meta['avoidance_observation']=avoidance_ref
    return scene,meta

def generate(config,output,start,count,runtime=geo.DEFAULT_RUNTIME_ASSET_DIR,mgeo=geo.DEFAULT_MGEO):
    output=Path(output)
    if output.exists() and any(output.iterdir()): raise FileExistsError('Nonempty output refused')
    all_rows=specs(config)
    if not 0<=start<len(all_rows) or count<1 or start+count>len(all_rows): raise ValueError('Invalid deck interval')
    ctx=context(runtime,mgeo); output.mkdir(parents=True,exist_ok=True)
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
            distance_bounds=row.get('distance_bounds'),role=row['role'],motion=row['motion'],subfamily=row['subfamily']))
        if (len(entries)%500)==0: print('Generated',len(entries),flush=True)
    manifest=dict(schema_version=2,dataset_id=config['dataset_id'],config=config,start=start,count=count,
        status='OFFLINE_GENERATED; per-scene live validation pending',scenes=entries,
        provenance=dict(config_sha256=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest(),
            generator_sha256=code_sha256,
            avoidance_geometry_sha256=geo.sha256(Path(avoidance.__file__)),
            highway_sample_sha256=geo.sha256(HIGHWAY_SAMPLE),
            geometry_sha256=geo.sha256(GEOMETRY),catalog_sha256=geo.sha256(catalog.DEFAULT_CATALOG),
            geometry_references=ctx['geometry_sources'],
            runtime_sources={k:geo.sha256(runtime/k) for k in json.loads(catalog.DEFAULT_CATALOG.read_text())['source_sha256']},
            map_sha256=geo.sha256(mgeo/'link_set.json')),pool_names={k:[a['name'] for a in v] for k,v in ctx['pools'].items()})
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
