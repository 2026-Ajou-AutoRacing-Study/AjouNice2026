"""Route-exact static mission snapshots, not a traffic or control simulator."""
import copy
import math
import random

import numpy as np


POLICY='sample-route-snapshots-v118'
PATH=('A2256W000144','A2256W000133','A2256W000134','A2256W000135','A2256W000102')


def project(route, arc, xy):
    """Continuous projection, including segment interiors rather than only nodes."""
    points = np.asarray([[p['x'], p['y']] for p in route], float)
    start, delta = points[:-1], np.diff(points, axis=0)
    norm = (delta * delta).sum(axis=1)
    t = np.clip(((np.asarray(xy)-start)*delta).sum(axis=1)/np.maximum(norm, 1e-12), 0., 1.)
    residual = np.asarray(xy)-start-t[:, None]*delta
    j = int(np.argmin((residual*residual).sum(axis=1)))
    signed = (delta[j, 0]*residual[j, 1]-delta[j, 1]*residual[j, 0])/max(math.sqrt(norm[j]), 1e-12)
    return float(arc[j]+t[j]*(arc[j+1]-arc[j])), float(np.linalg.norm(residual[j])), float(signed), j


def corridor(gen,ctx):
    if 'roundabout_mission_corridor' in ctx:
        return ctx['roundabout_mission_corridor']
    sample=gen.geo.load_json(gen.HIGHWAY_SAMPLE)
    spawn=next(p for p in sample['spawnPointList'] if p['UNIQUEID']==4)
    start,end=spawn['startLinkInfo'],spawn['endLinkInfo']
    if start['linkIdx']!=PATH[0] or end['linkIdx']!=PATH[-1] or spawn['isLaneChange']:
        raise ValueError('Official roundabout spawn contract changed')
    links=dict(ctx['links'])
    for key in (PATH[0],PATH[-1]):links[key]=copy.deepcopy(links[key])
    links[PATH[0]]['points']=links[PATH[0]]['points'][start['pointIdx']:]
    links[PATH[-1]]['points']=links[PATH[-1]]['points'][:end['pointIdx']+1]
    for left,right in zip(PATH,PATH[1:]):
        if links[left]['to_node_idx']!=links[right]['from_node_idx']:
            raise ValueError('Disconnected roundabout sample path')
    route,arc=gen.geo.route_from_link_path(links,PATH)
    for point in route:
        if point['link_idx']==PATH[0]:point['point_idx']+=start['pointIdx']
        point['point_ratio']=point['point_idx']/max(1,len(ctx['links'][point['link_idx']]['points'])-1)
    result=dict(route=route,arc=arc,path=list(PATH),start=start,end=end,spawn_uid=4,
                sample_goal_velocity_kph=spawn['MinDesiredVelocity_Custom'],
                sample_period_sec=spawn['minSpawnPeriod'])
    ctx['roundabout_mission_corridor']=result
    return result


def assign(gen,rows,config):
    plan=config.get('mission_roundabout_observation')
    if not plan:return
    if plan.get('policy')!=POLICY:raise ValueError('Unknown roundabout mission policy')
    if set(plan['phase_counts'])!=set(('approach','entry','circulation','exit')):
        raise ValueError('Roundabout phase keys differ')
    if sum(plan['approach_states'].values())!=plan['phase_counts']['approach']:
        raise ValueError('Roundabout approach state quota mismatch')
    selected=[r for r in rows if r['family']=='roundabout' and r['mission']]
    if sum(plan['phase_counts'].values())!=len(selected):raise ValueError('Roundabout phase quota mismatch')
    rng=random.Random(config['seed']+118)
    phases=gen.exact_deck(list(plan['phase_counts'].items()),len(selected),rng)
    for row,phase in zip(selected,phases):
        for key in ('vehicle_close_phase','vehicle_close_side','original_vehicle_distance_bounds','original_vehicle_role',
                    'roundabout_observation_state'):
            row.pop(key,None)
        row.update(roundabout_phase=phase, distance_bounds=[0.,60.],distance_bin='roundabout_route_snapshot',
                   yaw_limit=3,asset_index=0,motion='static',population_layout='mission')
    for index,row in enumerate(selected):row['asset_index']=index
    for phase in plan['phase_counts']:
        group=[r for r in selected if r['roundabout_phase']==phase]
        counts=gen.exact_deck([(int(k),v) for k,v in plan['actor_count_weights'][phase].items()],len(group),rng)
        densities=gen.exact_deck(list(plan['density_weights'].items()),len(group),rng)
        signs=gen.exact_deck([(1,1),(-1,1)],len(group),rng)
        for ordinal,(row,count,density,sign) in enumerate(zip(group,counts,densities,signs)):
            row.update(actors=count,roundabout_density=density,roundabout_primary_sign=sign,
                       roundabout_phase_ordinal=ordinal,roundabout_phase_total=len(group))
    approaching=[r for r in selected if r['roundabout_phase']=='approach']
    states=gen.exact_deck(list(plan['approach_states'].items()),len(approaching),rng)
    for row,state in zip(approaching,states):row['roundabout_observation_state']=state
    for state in plan['approach_states']:
        group=[r for r in approaching if r['roundabout_observation_state']==state]
        for ordinal,row in enumerate(group):
            row.update(roundabout_phase_ordinal=ordinal,roundabout_phase_total=len(group))


def anchor(gen,ctx,row,config,rng):
    phase=row['roundabout_phase'];bounds=config['mission_roundabout_observation']['ego_arc_bounds_m'][phase]
    if row.get('roundabout_observation_state'):
        bounds=config['mission_roundabout_observation']['approach_state_arc_bounds_m'][row['roundabout_observation_state']]
    # Stratify Ego progression; retry jitter stays within the same progression cell.
    width=(bounds[1]-bounds[0])/row['roundabout_phase_total']
    lo=bounds[0]+row['roundabout_phase_ordinal']*width
    arc_m=lo+width*rng.uniform(.1,.9)
    pose=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],arc_m)
    return ctx['route'],ctx['arc'],pose,dict(spatial_stratum='roundabout',roundabout_phase=phase,
        ego_progression_cell_m=[lo,lo+width], policy=POLICY)


def place(gen,ctx,ego,assets,row,config,rng):
    path=corridor(gen,ctx);route,arc=path['route'],path['arc']
    projection,_,_,_=project(route,arc,[ego['x'],ego['y']])
    angle=math.radians(ego['yaw']);c,s=math.cos(angle),math.sin(angle)
    ego_poly=gen.rectangle([ego['x']+1.35*c,ego['y']+1.35*s],[4.8,2.],ego['yaw'])
    phase=row['roundabout_phase'];density=row['roundabout_density']
    goal_bounds=config['mission_roundabout_observation']['ego_gap_bounds_m'][density]
    target=rng.uniform(*goal_bounds)
    signs=[row['roundabout_primary_sign']*(-1 if i%2 else 1) for i in range(len(assets))]
    if phase in ('approach','entry'):
        # NPCs downstream and upstream of the merge; Ego has not necessarily
        # joined the circular lane yet. A left-side threat is not a rear follower.
        signs=[1 if i%2==0 else -1 for i in range(len(assets))]
    poses=[];proofs=[];polys=[];arc_values=[]
    for index,(asset,sign) in enumerate(zip(assets,signs)):
        size,offset,known=gen.geometry(asset,ctx)
        if not known:raise ValueError('Roundabout snapshot requires measured vehicle geometry')
        candidates=[]
        for value in range(0,int(float(arc[-1])*4)+1):
            at=value/4
            if (at-projection)*sign<.5:continue
            pose=gen.geo.continuous_route_pose_at_arc(route,arc,at)
            pose=dict(pose,yaw=round(pose['yaw'],3))
            pa=math.radians(pose['yaw'])
            center=[pose['x']+math.cos(pa)*offset[0]-math.sin(pa)*offset[1],
                    pose['y']+math.sin(pa)*offset[0]+math.cos(pa)*offset[1]]
            dx,dy=center[0]-ego['x'],center[1]-ego['y']
            lon,lat=c*dx+s*dy,-s*dx+c*dy
            if math.hypot(dx,dy)>50 or abs(lat)>29.5:continue
            poly=gen.rectangle(center,size,pose['yaw']);gap=ego_poly.distance(poly)
            if gap<.5 or any(poly.distance(p)<config['mission_roundabout_observation']['npc_minimum_pair_gap_m'] for p in polys):continue
            if phase=='circulation' and index<2 and not goal_bounds[0]-.01<=gap<=goal_bounds[1]+.01:continue
            if phase=='circulation' and (lon>0)!=(sign>0):continue
            # A third NPC belongs farther along the same route, not arbitrarily
            # beside Ego or stacked at a different location on the circular loop.
            if any(abs(at-old)<max(3.,size[0]) for old in arc_values):continue
            desired=target if index<2 else target+size[0]+3.
            score=abs(gap-desired)+.025*abs(at-projection)+rng.uniform(0,.03)
            candidates.append((score,pose,poly,at,gap,lon,lat))
        if not candidates:raise ValueError('No feasible roundabout snapshot NPC')
        _,pose,poly,at,gap,lon,lat=min(candidates,key=lambda x:x[0])
        poses.append(pose);polys.append(poly);arc_values.append(at)
        proofs.append(dict(policy=POLICY,spawn_uid=4,route_links=path['path'],start=path['start'],end=path['end'],
            reference_arc_m=at,corridor_length_m=float(arc[-1]),npc_pivot_route_exact=True,
            npc_jitter_applied=False,phase=phase,density=density,ego_projection_arc_m=projection,
            arc_side=sign,planned_surface_gap_m=gap,longitudinal_m=lon,lateral_m=lat))
        proofs[-1]['reference_yaw_deg']=pose['yaw']
    return poses,dict(policy=POLICY,phase=phase,density=density,ego_route_arc_m=ego['route_arc_m'],
        npc_sample_sha256=gen.geo.sha256(gen.HIGHWAY_SAMPLE),npc_route_length_m=float(arc[-1]),
        actors=proofs,npc_pair_surface_gaps_m=[polys[j].distance(polys[i]) for i in range(len(polys)) for j in range(i)],
        observed_or_assumed='Geometry snapshots; density is a design envelope, not reconstructed future traffic.',
        collider_verified=False)


def validate(gen,ctx,meta,scene,config):
    """Independently reconstruct serialized poses and gaps; metadata is not proof."""
    row=meta['spec'];proof=meta.get('roundabout_observation')
    if not (row['family']=='roundabout' and row['mission'] and proof and proof['policy']==POLICY):
        raise ValueError('Roundabout mission context/proof missing')
    if proof['npc_sample_sha256']!=gen.geo.sha256(gen.HIGHWAY_SAMPLE):
        raise ValueError('Roundabout sample provenance changed')
    plan=config['mission_roundabout_observation'];phase=row['roundabout_phase']
    lo,hi=plan['ego_arc_bounds_m'][phase]
    if not lo<=meta['base']['route_arc_m']<=hi:raise ValueError('Ego outside phase route interval')
    if row.get('roundabout_observation_state'):
        lo,hi=plan['approach_state_arc_bounds_m'][row['roundabout_observation_state']]
        if not lo<=meta['base']['route_arc_m']<=hi:raise ValueError('Ego outside approach subphase')
    cell_lo,cell_hi=meta['details']['ego_progression_cell_m']
    if not cell_lo<=meta['base']['route_arc_m']<=cell_hi:
        raise ValueError('Ego progression stratum changed')
    base=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],meta['base']['route_arc_m'])
    ego=meta['ego'];raw_ego=scene['egoVehicle']['initPosition'];angle=math.radians(base['yaw'])
    # Ego metadata Z is road height; the existing Ioniq serializer uses +0.34m
    # as the initial vehicle reference. Do not mistake that for floating GT.
    if any(abs(raw_ego['pos'][key]-ego[key])>1e-5 for key in ('x','y')) or abs(raw_ego['pos']['z']-ego['z']-.34)>1e-5 or abs(float(raw_ego['rot']['yaw'])-ego['yaw'])>.001:
        raise ValueError('Serialized Ego differs from geometry metadata')
    dx,dy=raw_ego['pos']['x']-base['x'],raw_ego['pos']['y']-base['y']
    if abs(math.cos(angle)*dx+math.sin(angle)*dy)>1e-5 or math.hypot(dx,dy)>.30001:
        raise ValueError('Ego no longer follows its route observation envelope')
    if abs((float(raw_ego['rot']['yaw'])-base['yaw']+180)%360-180)>plan['ego_yaw_limit_deg']+.001:
        raise ValueError('Ego snapshot yaw envelope changed')
    angle=math.radians(ego['yaw'])
    ep=gen.rectangle([ego['x']+1.35*math.cos(angle),ego['y']+1.35*math.sin(angle)],[4.8,2.],ego['yaw'])
    path=corridor(gen,ctx);polys=[];longitudes=[];sides=[]
    for index,actor in enumerate(meta['runtime_actors']):
        evidence=actor.get('sample_roundabout_placement')
        if not evidence or actor['motion']!='static' or actor.get('motion_path'):
            raise ValueError('Roundabout NPC must have static route evidence')
        at=evidence['reference_arc_m']
        if not 0<=at<=path['arc'][-1] or evidence['route_links']!=path['path']:
            raise ValueError('NPC beyond official despawn corridor')
        expected=gen.geo.continuous_route_pose_at_arc(path['route'],path['arc'],at)
        raw=next(a for a in scene['vehicleList'] if a['UNIQUEID']==actor['unique_id'])
        pos=raw['initPosition']['pos'];yaw=float(raw['initPosition']['rot']['yaw'])
        if any(abs(pos[k]-expected[k])>1e-5 for k in ('x','y')) or abs((yaw-expected['yaw']+180)%360-180)>.001:
            raise ValueError('Serialized NPC moved away from sample route')
        size,offset,known=gen.geometry(actor,ctx)
        pa=math.radians(yaw)
        center=[pos['x']+math.cos(pa)*offset[0]-math.sin(pa)*offset[1],pos['y']+math.sin(pa)*offset[0]+math.cos(pa)*offset[1]]
        if not known or max(abs(a-b) for a,b in zip(center,actor['geometric_center_estimate']))>1e-5:
            raise ValueError('Roundabout center geometry witness differs')
        poly=gen.rectangle(center,size,yaw);gap=ep.distance(poly)
        if abs(gap-evidence['planned_surface_gap_m'])>1e-5 or gap<.5-1e-5:
            raise ValueError('Roundabout Ego clearance witness differs')
        if any(poly.distance(p)<plan['npc_minimum_pair_gap_m']-1e-5 for p in polys):
            raise ValueError('Roundabout NPC pair clearance violated')
        if phase=='circulation' and index<2:
            lower,upper=plan['ego_gap_bounds_m'][row['roundabout_density']]
            if not lower-.01<=gap<=upper+.01:raise ValueError('Circulation density quota differs')
        angle=math.radians(ego['yaw']);dx,dy=center[0]-ego['x'],center[1]-ego['y']
        longitudes.append(math.cos(angle)*dx+math.sin(angle)*dy);sides.append(evidence['arc_side']);polys.append(poly)
    if phase=='circulation' and len(longitudes)>=2 and (longitudes[0]*longitudes[1]>=0 or sides[0]*sides[1]>=0):
        raise ValueError('Circulation lacks front/back Ego insertion')
    return True


def visibility_review(metadata,components,clearances=None):
    """A placed pair cannot satisfy an insertion quota if sensor evidence loses it."""
    proof=metadata.get('roundabout_observation')
    if not proof or proof.get('policy')!=POLICY:return [],None
    actors=metadata['runtime_actors'];first_two=[a['unique_id'] for a in actors[:2]]
    visible=[c for c in components if c['parent_uid'] in first_two and c['positive_lidar1_camera0']]
    uids={c['parent_uid'] for c in visible}
    phase=proof['phase'];reasons=[]
    pair_present=len(first_two)==2 and all(uid in uids for uid in first_two)
    front_back=pair_present and any(c.get('center_ego_m',[0])[0]>0 for c in visible) and any(c.get('center_ego_m',[0])[0]<0 for c in visible)
    if phase in ('entry','circulation') and not pair_present:
        reasons.append('roundabout_visible_pair_deficit')
    if phase=='circulation' and not front_back:
        reasons.append('roundabout_visible_front_back_deficit')
    gaps=[]
    if phase=='circulation' and pair_present:
        bounds={'tight':(.5,2.5),'normal':(4.,10.),'loose':(10.,22.)}[proof['density']]
        for uid in first_two:
            found=[c['annotation_surface_gap_m'] for c in (clearances or []) if c['parent_uid']==uid]
            if found:gaps.append(min(found))
        if len(gaps)<2:reasons.append('roundabout_actual_gap_unavailable')
        elif any(g<bounds[0]-.3 or g>bounds[1]+.3 for g in gaps):
            reasons.append('roundabout_actual_density_gap_mismatch')
    return reasons,dict(policy=POLICY,phase=phase,first_two_uids=first_two,
        visible_pair=pair_present,visible_front_back=front_back,first_two_actual_gaps_m=gaps,
        actual_gap_tolerance_m=.3,
        limitation='Raw positive component evidence, not detector accuracy or collider shape.')
