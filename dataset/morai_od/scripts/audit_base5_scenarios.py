#!/usr/bin/env python3
"""Offline generated scene audit; explicitly not a simulator validation."""
import argparse
from collections import Counter
import json
from pathlib import Path

import generate_base5_independent as gen


def bounded_vehicle_pose_matches(pos,expected,initializer=None):
    offset=initializer['pivot_height_candidate_m'] if initializer and initializer.get('usable') else .36
    return all(abs(pos[k]-expected[k])<=1e-5 for k in ('x','y')) and abs(pos['z']-expected['z']-offset)<=1e-5


def audit(manifest_path):
    m=json.loads(Path(manifest_path).read_text()); families=Counter();missions=Counter();bins=Counter()
    groups=Counter();assets=Counter();roles=Counter();splits=Counter();retries=[];unknown=Counter();hashes=set();errors=[]
    links={r['idx']:r for r in gen.geo.load_json(gen.geo.DEFAULT_MGEO/'link_set.json')}
    corridors={c['spawn_uid']:c for c in gen.sample_highway_corridors(links)}
    bounded_vehicles=0;avoidance_phases=Counter();avoidance_gaps=[];avoidance_bins=Counter()
    primary_assets=Counter();primary_roles=Counter();motion_counts=Counter();motion_decisions=Counter()
    inventory={r['data_id']:r for r in gen.geo.load_json(gen.catalog.DEFAULT_CATALOG)['assets']}
    appearance=gen.geo.load_json(gen.APPEARANCE_POLICY)
    height_ctx=None;height_assets=None;dynamic_ctx=None;dynamic_phases=Counter();dynamic_crossings=Counter()
    generic_close=Counter();vehicle_close_phases=Counter();vehicle_close_gaps=[]
    stationary_pools=Counter();stationary_primary=Counter()
    for entry in m['scenes']:
        s=json.loads(Path(entry['scenario']).read_text());meta=json.loads(Path(entry['metadata']).read_text());r=meta['spec']
        digest=gen.geo.sha256(Path(entry['scenario']))
        if digest!=entry['scenario_sha256']:errors.append((entry['id'],'checksum'))
        if digest in hashes:errors.append((entry['id'],'duplicate scene payload'))
        hashes.add(digest);families[r['family']]+=1;missions[r['family']]+=int(r['mission']);splits[meta['split']]+=1
        if r.get('stationary_mission_pool'):
            stationary_pools[r['stationary_mission_pool']]+=1
            if meta['runtime_actors']:stationary_primary[meta['runtime_actors'][0]['data_id']]+=1
            if meta.get('details',{}).get('spatial_stratum')!='static_obstacle' or s['waypointDataList']:
                errors.append((entry['id'],'static mission context/path mismatch'))
        bins[str((r['family'],r['role'],r.get('distance_bounds')))]+=1;groups[str((r['family'],r['asset_group']))]+=1
        if s['egoVehicle']['currentVelocity']!=0 or s['egoVehicle']['initPosition']['vehicleSaveData']['velocity']!=0:
            errors.append((entry['id'],'nonzero ego'))
        if s['spawnPointList'] or s['pedspawnPointList']:errors.append((entry['id'],'traffic spawn'))
        if len(meta['runtime_actors'])!=r['actors']:errors.append((entry['id'],'actor count'))
        pop=meta.get('population_budget')
        if pop:
            costs=[gen.population.cost(a['estimated_size']) for a in meta['runtime_actors']]
            if pop['costs']!=costs or pop['total_cost']!=sum(costs):errors.append((entry['id'],'population cost mismatch'))
            if pop['mode'] in ('generic_regular','generic_dense'):
                lo,hi=r['population_budget_bounds']
                if not (2<=len(costs)<=r['population_max_actors'] and lo<=sum(costs)<=hi):
                    errors.append((entry['id'],'population budget bounds'))
                if r['mission']:errors.append((entry['id'],'mission used generic budget'))
                if 'minimum_visible_fraction' in m['config']['population_budget']:
                    expected=max(2,gen.math.ceil(len(costs)*m['config']['population_budget']['minimum_visible_fraction']))
                    if pop.get('required_visible_actors')!=expected:errors.append((entry['id'],'population visible quota mismatch'))
        footprints=[];ea=gen.math.radians(meta['ego']['yaw']);e=meta['ego']
        observation=meta['details'].get('dynamic_observation')
        if r.get('dynamic_observation_phase'):
            if not observation:errors.append((entry['id'],'missing map-anchored dynamic observation'))
            else:
                phase=r['dynamic_observation_phase'];delta=observation['ego_crossing_arc_displacement_m']
                dynamic_phases[phase]+=1;dynamic_crossings[observation['crossing']['crosswalk_id']]+=1
                if (delta>0)!=(phase=='clearing'):errors.append((entry['id'],'dynamic Ego crossing side'))
                target=observation['target_center'];center=meta['runtime_actors'][0]['geometric_center_estimate']
                if gen.math.hypot(center[0]-target['x'],center[1]-target['y'])>1e-5:
                    errors.append((entry['id'],'dynamic primary left map target'))
                if dynamic_ctx is None:dynamic_ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
                ref=gen.geo.load_json(Path(meta['details']['mission_reference']))
                candidates=gen.dynamic_crossing_candidates(dynamic_ctx,ref,m['config']['mission_dynamic_observation'])
                expected=next((c for c in candidates if c['crosswalk_id']==observation['crossing']['crosswalk_id']),None)
                if expected!=observation['crossing']:errors.append((entry['id'],'dynamic crossing differs from map'))
                else:
                    dx,dy=target['x']-expected['center']['x'],target['y']-expected['center']['y'];ax,ay=expected['axis']
                    jitter=m['config']['mission_dynamic_observation']['target_axis_jitter_m']
                    if abs(dx*ay-dy*ax)>1e-5 or abs(dx*ax+dy*ay)>jitter+1e-5:
                        errors.append((entry['id'],'dynamic target outside crossing axis quota'))
                    if abs(meta['base']['route_arc_m']-expected['route_arc_m']-delta)>1e-5:
                        errors.append((entry['id'],'dynamic Ego arc witness mismatch'))
        avoidance=meta.get('avoidance_observation')
        if r.get('avoidance_phase'):
            if not avoidance:errors.append((entry['id'],'missing avoidance observation'))
            else:
                if r.get('avoidance_origin')=='generic_close':
                    generic_close[str((r['family'],r['avoidance_side'],r['avoidance_gap_bounds_m']))]+=1
                    if r['mission'] or r['original_distance_bounds'][1]>m['config']['generic_close_observation']['eligible_max_center_distance_m']:
                        errors.append((entry['id'],'generic close changed mission or distant quota'))
                else:avoidance_phases[str((r['family'],avoidance['phase'],avoidance['side']))]+=1
                if avoidance['phase']=='alongside':
                    avoidance_gaps.append(avoidance['actual_surface_gap_m'])
                    if r.get('avoidance_origin')!='generic_close':
                        avoidance_bins[str((r['family'],r['avoidance_gap_bounds_m']))]+=1
        ego_poly=gen.rectangle([e['x']+1.35*gen.math.cos(ea),e['y']+1.35*gen.math.sin(ea)],[4.8,2.],e['yaw'])
        for i,a in enumerate(meta['runtime_actors']):
            if r.get('stationary_mission_pool'):
                if a['motion'] not in ('static','standing') or a.get('motion_path'):
                    errors.append((entry['id'],'static mission actor has motion'))
            if a['data_id'] in gen.catalog.COMPETITION_EXCLUDED_IDS:
                errors.append((entry['id'],'official competition-excluded asset'))
            if m['config']['capture'].get('asset_appearance_policy',False):
                policy_asset=dict(inventory[a['data_id']],name=a['name'],source_list=a['source_list'])
                expected_motion,decision=gen.population_appearance(policy_asset,a['requested_motion'],r,i,
                    sum(p['motion'] not in ('static','standing') for p in meta['runtime_actors'][:i]),m['config'],appearance)
                if a['motion']!=expected_motion or a['appearance_motion_decision']!=decision:
                    errors.append((entry['id'],'asset appearance policy mismatch'))
            motion_counts[str((a['name'],a['motion']))]+=1
            motion_decisions[a.get('appearance_motion_decision','legacy')]+=1
            if i==0:
                primary_assets[str((r['family'],r['subfamily'],a['name']))]+=1
                primary_roles[str((r['family'],a['actual_ego_role']))]+=1
            assets[a['name']]+=1;roles[str((r['family'],a['actual_ego_role']))]+=1
            if not a['geometry_reference_known']:unknown[a['name']]+=1
            if a['planned_center_distance_m']>100 or abs(a['lateral_m'])>29.5: errors.append((entry['id'],'ROI'))
            minimum=.10 if i==0 and avoidance and avoidance['phase']=='alongside' else .3
            if a['planned_surface_clearance_m']<minimum-1e-6:errors.append((entry['id'],'ego collision'))
            # Actual generated rot is authoritative, not an independent random estimate.
            actor=next(t for k in ('vehicleList','objectList','pedestrianList','aiPedstrianList') for t in s.get(k,[]) if t['UNIQUEID']==a['unique_id'])
            if r.get('stationary_mission_pool'):
                if any(float(actor.get(k,0))!=0 for k in ('speed','moveSpeed','customSpeed','currentVelocity')) or actor.get('isWaypointMode'):
                    errors.append((entry['id'],'static mission runtime movement enabled'))
                if actor.get('initPosition',{}).get('vehicleSaveData',{}).get('velocity',0)!=0:
                    errors.append((entry['id'],'static mission saved vehicle velocity'))
            bounded=r['family']=='highway' and r['mission'] and a['source_list']=='vehicleList'
            placement=a.get('sample_highway_placement')
            if bounded:
                if not placement or placement.get('spawn_uid') not in corridors:
                    errors.append((entry['id'],'missing sample NPC corridor'))
                else:
                    c=corridors[placement['spawn_uid']];distance=placement['reference_arc_m']
                    if not 0<=distance<=c['arc'][-1]: errors.append((entry['id'],'sample NPC arc bounds'))
                    expected=gen.geo.continuous_route_pose_at_arc(c['route'],c['arc'],distance)
                    pos=actor['initPosition']['pos']
                    initializer=a.get('static_height_initializer')
                    if initializer is not None:
                        # Recompute the candidate from F2/map sources, not a self-declared Z exemption.
                        from base5_static_height import saved_height_candidate
                        if height_ctx is None:
                            height_ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
                            height_assets={asset['data_id']:asset for pool in height_ctx['pools'].values() for asset in pool}
                        expected_initializer=saved_height_candidate(height_assets[a['data_id']],height_ctx)
                        if initializer!=expected_initializer:
                            errors.append((entry['id'],'static height initializer differs from sources'))
                    if not bounded_vehicle_pose_matches(pos,expected,initializer):
                        errors.append((entry['id'],'scenario NPC differs from bounded route'))
                    if abs(actor['initPosition']['initLinkRatio']-expected['point_ratio'])>1e-6:
                        errors.append((entry['id'],'sample NPC original link ratio'))
                    bounded_vehicles+=1
            elif placement: errors.append((entry['id'],'unexpected generic NPC restriction'))
            yaw=float(actor['initPosition']['rot']['yaw']) if 'initPosition' in actor else float(actor['rot']['yaw'])
            poly=gen.rectangle(a['geometric_center_estimate'],a['estimated_size'],yaw)
            computed_gap=ego_poly.distance(poly)
            if computed_gap<minimum-1e-6:errors.append((entry['id'],'recomputed ego footprint'))
            close=a.get('vehicle_close_observation')
            if i==0 and r.get('vehicle_close_phase'):
                if not close:errors.append((entry['id'],'missing legal vehicle close observation'))
                else:
                    vehicle_close_phases[str((r['family'],r['vehicle_close_phase']))]+=1
                    vehicle_close_gaps.append(computed_gap)
                    if not .3-1e-6<=computed_gap<=2.5+1e-6 or abs(close['planned_surface_gap_m']-computed_gap)>1e-5:
                        errors.append((entry['id'],'vehicle close footprint gap'))
                    if close['phase']!=r['vehicle_close_phase'] or close['npc_jitter_applied']:
                        errors.append((entry['id'],'vehicle close policy mismatch'))
                    lon,lat=a['longitudinal_m'],a['lateral_m'];phase=close['phase']
                    if phase=='alongside' and (abs(lon)>1.2 or (lat>0)!=(r['vehicle_close_side']=='left')):
                        errors.append((entry['id'],'vehicle alongside relative phase'))
                    if phase in ('front_side','following_front') and lon<=0:errors.append((entry['id'],'vehicle front phase sign'))
                    if phase in ('rear_side','following_rear') and lon>=0:errors.append((entry['id'],'vehicle rear phase sign'))
                    if r['family']=='roundabout':
                        nr,na=gen.geo.route_from_link_path(links,gen.geo.ROUNDABOUT_NPC_LINK_PATH)
                        if close['route_links']!=list(gen.geo.ROUNDABOUT_NPC_LINK_PATH):errors.append((entry['id'],'roundabout close route'))
                        expected=gen.geo.continuous_route_pose_at_arc(nr,na,close['reference_arc_m'])
                        if not bounded_vehicle_pose_matches(actor['initPosition']['pos'],expected,a.get('static_height_initializer')):
                            errors.append((entry['id'],'roundabout close NPC off route'))
            if i==0 and avoidance and avoidance['phase']=='alongside':
                lo,hi=r['avoidance_gap_bounds_m']
                requested=avoidance.get('original_gap_request_m',computed_gap)
                if not lo-1e-6<=requested<=hi+1e-6:errors.append((entry['id'],'avoidance requested gap quota'))
                if avoidance.get('clearance_policy')=='measured-geometry-min30cm; unknown-geometry-min1m-v46':
                    floor=.30 if a['geometry_reference_known'] else 1.
                    if abs(computed_gap-max(requested,floor))>.001:
                        errors.append((entry['id'],'conservative effective clearance mismatch'))
                elif not lo-1e-6<=computed_gap<=hi+1e-6:errors.append((entry['id'],'avoidance effective gap quota'))
                if abs(computed_gap-avoidance['requested_surface_gap_m'])>.001:
                    errors.append((entry['id'],'avoidance solved gap'))
                if a['actual_ego_role']!=r['role']:errors.append((entry['id'],'avoidance actual side'))
            if any(poly.distance(p)<.199 for p in footprints):errors.append((entry['id'],'actor overlap'))
            path=a.get('motion_path')
            envelope=poly
            if path is not None:
                from shapely.geometry import LineString
                points=path['points'];start=points[0];end=points[-1] if path.get('ground_path_evidence') else points[1]
                if path['policy'] not in ('bounded-shuttle-v1','one-way-slow-v1','natural-target-v1'):
                    errors.append((entry['id'],'unknown motion path policy'))
                if path['policy']=='bounded-shuttle-v1' and points[0]!=points[-1]:errors.append((entry['id'],'open shuttle return'))
                one_way=path['policy'] in ('one-way-slow-v1','natural-target-v1')
                expected_points=points if one_way else points[:2]*64+[points[0]]
                if one_way and ((len(points)!=2 and not path.get('ground_path_evidence')) or points[0]==points[-1]):
                    errors.append((entry['id'],'one-way endpoint count'))
                if one_way:
                    runtime_actor=next(a2 for a2 in s[a['source_list']] if a2.get('UNIQUEID')==a['unique_id'])
                    field='moveSpeed' if a['source_list']=='objectList' else 'speed'
                    if runtime_actor[field]!=path['speed_field_value'] or len(runtime_actor['waypointEventList'])!=len(points):
                        errors.append((entry['id'],'one-way speed/waypoint event mismatch'))
                    if path.get('ground_path_evidence'):
                        from base5_motion_ground import validate_motion_ground
                        try:validate_motion_ground(meta,s)
                        except ValueError as exc:errors.append((entry['id'],'motion ground evidence '+str(exc)))
                runtime_path=next((p for p in s['waypointDataList'] if p['pathName'].endswith(' - '+str(a['unique_id']))),None)
                if runtime_path is None or runtime_path['waypointData']!=[
                        dict(pos=gen.geo.world_to_morai_waypoint(p),rot=gen.geo.formatted_rotation(0.)) for p in expected_points]:
                    errors.append((entry['id'],'runtime shuttle differs from metadata'))
                cx,cy=a['geometric_center_estimate'];lo,hi=path['distance_bounds_m']
                pivot=path.get('target_position',start)
                offset_x,offset_y=cx-pivot['x'],cy-pivot['y']
                for step in range(21):
                    t=step/20.;x=start['x']+offset_x+t*(end['x']-start['x']);y=start['y']+offset_y+t*(end['y']-start['y'])
                    if path['policy']!='natural-target-v1' and not lo<=gen.math.hypot(x-e['x'],y-e['y'])<hi:errors.append((entry['id'],'shuttle distance cell'))
                    if abs(-gen.math.sin(ea)*(x-e['x'])+gen.math.cos(ea)*(y-e['y']))>29.5:
                        errors.append((entry['id'],'shuttle lateral ROI'))
                    if abs(gen.math.cos(ea)*(x-e['x'])+gen.math.sin(ea)*(y-e['y']))>99.5:
                        errors.append((entry['id'],'motion longitudinal ROI'))
                radius=gen.math.hypot(*a['estimated_size'][:2])/2+gen.math.hypot(offset_x,offset_y)
                envelope=LineString([(start['x'],start['y']),(end['x'],end['y'])]).buffer(radius)
                if envelope.distance(ego_poly)<.3 or any(envelope.distance(p)<.199 for p in footprints):
                    errors.append((entry['id'],'shuttle swept clearance'))
            footprints.append(envelope)
            if i==0:
                lo,hi=r['distance_bounds']
                if not lo<=a['planned_center_distance_m']<hi:errors.append((entry['id'],'distance quota'))
        retries.append(meta['retry'])
    if m['count']==m['config']['target_scenes']:
        if m['config'].get('stationary_mission_assets'):
            expected_pools=Counter(m['config']['stationary_mission_assets']['primary_pool_counts'])
            expected_pools['static_vehicle']+=m['config']['mission_counts']['other_vehicle']
            if stationary_pools!=expected_pools:errors.append(('all','stationary mission pool quota'))
            ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
            eligible={a['data_id'] for key in expected_pools for a in ctx['pools'][key]}
            if any(stationary_primary[uid]<3 for uid in eligible):
                errors.append(('all','stationary mission primary coverage'))
        if dict(families)!=m['config']['families']:errors.append(('all','family quota'))
        if dict(missions)!=m['config']['mission_counts']:errors.append(('all','mission quota'))
        if m['config'].get('mission_vehicle_close'):
            expected=Counter(str((r['family'],r['vehicle_close_phase'])) for r in gen.specs(m['config']) if r.get('vehicle_close_phase'))
            if vehicle_close_phases!=expected:errors.append(('all','vehicle close phase quota'))
        if m['config'].get('mission_dynamic_observation'):
            expected=Counter(r['dynamic_observation_phase'] for r in gen.specs(m['config']) if r.get('dynamic_observation_phase'))
            if dynamic_phases!=expected:errors.append(('all','dynamic observation phase quota'))
        plan=m['config'].get('mission_avoidance')
        if plan:
            for family in ('obstacle','other_vehicle'):
                total=m['config']['mission_counts'][family]
                for phase,weight in plan['phases'].items():
                    for side in ('left','right'):
                        if avoidance_phases[str((family,phase,side))]!=total*weight//200:
                            errors.append(('all','avoidance phase/side quota'))
                alongside=total*plan['phases']['alongside']//100
                for lo,hi,weight in plan['alongside_gap_bins_m']:
                    if avoidance_bins[str((family,[lo,hi]))]!=alongside*weight//100:
                        errors.append(('all','avoidance gap-bin quota'))
    return dict(status='OFFLINE_PASS' if not errors else 'OFFLINE_FAIL',live_validation=False,
        scenes=len(m['scenes']),unique_payloads=len(hashes),families=dict(families),mission_counts=dict(missions),
        sample_bounded_highway_vehicle_count=bounded_vehicles,
        stationary_mission_pool_counts=dict(stationary_pools),
        stationary_mission_primary_assets=len(stationary_primary),
        avoidance_phase_side_counts=dict(avoidance_phases),generic_close_counts=dict(generic_close),
        vehicle_close_phase_counts=dict(vehicle_close_phases),
        vehicle_close_gap_min_max_m=[min(vehicle_close_gaps),max(vehicle_close_gaps)] if vehicle_close_gaps else [],
        avoidance_gap_bin_counts=dict(avoidance_bins),
        alongside_gap_min_max_m=[min(avoidance_gaps),max(avoidance_gaps)] if avoidance_gaps else [],
        distance_direction_bins=dict(bins),asset_groups=dict(groups),asset_counts=dict(assets),splits=dict(splits),
        actual_actor_directions=dict(roles),geometry_without_capture_reference=dict(unknown),
        primary_asset_counts=dict(primary_assets),primary_actual_directions=dict(primary_roles),
        actual_asset_motion_counts=dict(motion_counts),appearance_motion_decisions=dict(motion_decisions),
        mean_resamples=sum(retries)/max(1,len(retries)),max_resamples=max(retries,default=0),errors=errors,
        limitations=['Footprints use capture-derived or conservative estimated size, not live mesh/collider.',
            'No ground support, actor animation or sensor visibility certification.',
            'Link split may share neighboring views; final generalization requires held-out bags.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=audit(a.manifest)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('distance_direction_bins','asset_counts','actual_actor_directions',
        'primary_asset_counts','primary_actual_directions','actual_asset_motion_counts')},indent=2))
    if r['errors']:raise SystemExit(1)
