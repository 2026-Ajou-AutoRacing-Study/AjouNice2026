#!/usr/bin/env python3
"""Offline generated scene audit; explicitly not a simulator validation."""
import argparse
from collections import Counter
import json
from pathlib import Path

import generate_base5_independent as gen


def audit(manifest_path):
    m=json.loads(Path(manifest_path).read_text()); families=Counter();missions=Counter();bins=Counter()
    groups=Counter();assets=Counter();roles=Counter();splits=Counter();retries=[];unknown=Counter();hashes=set();errors=[]
    links={r['idx']:r for r in gen.geo.load_json(gen.geo.DEFAULT_MGEO/'link_set.json')}
    corridors={c['spawn_uid']:c for c in gen.sample_highway_corridors(links)}
    bounded_vehicles=0;avoidance_phases=Counter();avoidance_gaps=[];avoidance_bins=Counter()
    for entry in m['scenes']:
        s=json.loads(Path(entry['scenario']).read_text());meta=json.loads(Path(entry['metadata']).read_text());r=meta['spec']
        digest=gen.geo.sha256(Path(entry['scenario']))
        if digest!=entry['scenario_sha256']:errors.append((entry['id'],'checksum'))
        if digest in hashes:errors.append((entry['id'],'duplicate scene payload'))
        hashes.add(digest);families[r['family']]+=1;missions[r['family']]+=int(r['mission']);splits[meta['split']]+=1
        bins[str((r['family'],r['role'],r.get('distance_bounds')))]+=1;groups[str((r['family'],r['asset_group']))]+=1
        if s['egoVehicle']['currentVelocity']!=0 or s['egoVehicle']['initPosition']['vehicleSaveData']['velocity']!=0:
            errors.append((entry['id'],'nonzero ego'))
        if s['spawnPointList'] or s['pedspawnPointList']:errors.append((entry['id'],'traffic spawn'))
        if len(meta['runtime_actors'])!=r['actors']:errors.append((entry['id'],'actor count'))
        footprints=[];ea=gen.math.radians(meta['ego']['yaw']);e=meta['ego']
        avoidance=meta.get('avoidance_observation')
        if r.get('avoidance_phase'):
            if not avoidance:errors.append((entry['id'],'missing avoidance observation'))
            else:
                avoidance_phases[str((r['family'],avoidance['phase'],avoidance['side']))]+=1
                if avoidance['phase']=='alongside':
                    avoidance_gaps.append(avoidance['actual_surface_gap_m'])
                    avoidance_bins[str((r['family'],r['avoidance_gap_bounds_m']))]+=1
        ego_poly=gen.rectangle([e['x']+1.35*gen.math.cos(ea),e['y']+1.35*gen.math.sin(ea)],[4.8,2.],e['yaw'])
        for i,a in enumerate(meta['runtime_actors']):
            assets[a['name']]+=1;roles[str((r['family'],a['actual_ego_role']))]+=1
            if not a['geometry_reference_known']:unknown[a['name']]+=1
            if a['planned_center_distance_m']>100 or abs(a['lateral_m'])>29.5: errors.append((entry['id'],'ROI'))
            minimum=.10 if i==0 and avoidance and avoidance['phase']=='alongside' else .3
            if a['planned_surface_clearance_m']<minimum-1e-6:errors.append((entry['id'],'ego collision'))
            # Actual generated rot is authoritative, not an independent random estimate.
            actor=next(t for k in ('vehicleList','objectList','pedestrianList','aiPedstrianList') for t in s.get(k,[]) if t['UNIQUEID']==a['unique_id'])
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
                    # set_static_vehicle_actor preserves the existing +0.36m spawn-height contract.
                    if any(abs(pos[k]-expected[k])>1e-5 for k in ('x','y')) or abs(pos['z']-expected['z']-.36)>1e-5:
                        errors.append((entry['id'],'scenario NPC differs from bounded route'))
                    if abs(actor['initPosition']['initLinkRatio']-expected['point_ratio'])>1e-6:
                        errors.append((entry['id'],'sample NPC original link ratio'))
                    bounded_vehicles+=1
            elif placement: errors.append((entry['id'],'unexpected generic NPC restriction'))
            yaw=float(actor['initPosition']['rot']['yaw']) if 'initPosition' in actor else float(actor['rot']['yaw'])
            poly=gen.rectangle(a['geometric_center_estimate'],a['estimated_size'],yaw)
            computed_gap=ego_poly.distance(poly)
            if computed_gap<minimum-1e-6:errors.append((entry['id'],'recomputed ego footprint'))
            if i==0 and avoidance and avoidance['phase']=='alongside':
                lo,hi=r['avoidance_gap_bounds_m']
                if not lo-1e-6<=computed_gap<=hi+1e-6:errors.append((entry['id'],'avoidance gap quota'))
                if abs(computed_gap-avoidance['requested_surface_gap_m'])>.001:
                    errors.append((entry['id'],'avoidance solved gap'))
                if a['actual_ego_role']!=r['role']:errors.append((entry['id'],'avoidance actual side'))
            if any(poly.distance(p)<.199 for p in footprints):errors.append((entry['id'],'actor overlap'))
            footprints.append(poly)
            if i==0:
                lo,hi=r['distance_bounds']
                if not lo<=a['planned_center_distance_m']<hi:errors.append((entry['id'],'distance quota'))
        retries.append(meta['retry'])
    if m['count']==m['config']['target_scenes']:
        if dict(families)!=m['config']['families']:errors.append(('all','family quota'))
        if dict(missions)!=m['config']['mission_counts']:errors.append(('all','mission quota'))
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
        avoidance_phase_side_counts=dict(avoidance_phases),
        avoidance_gap_bin_counts=dict(avoidance_bins),
        alongside_gap_min_max_m=[min(avoidance_gaps),max(avoidance_gaps)] if avoidance_gaps else [],
        distance_direction_bins=dict(bins),asset_groups=dict(groups),asset_counts=dict(assets),splits=dict(splits),
        actual_actor_directions=dict(roles),geometry_without_capture_reference=dict(unknown),
        mean_resamples=sum(retries)/max(1,len(retries)),max_resamples=max(retries,default=0),errors=errors,
        limitations=['Footprints use capture-derived or conservative estimated size, not live mesh/collider.',
            'No ground support, actor animation or sensor visibility certification.',
            'Link split may share neighboring views; final generalization requires held-out bags.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();r=audit(a.manifest)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('distance_direction_bins','asset_counts','actual_actor_directions')},indent=2))
    if r['errors']:raise SystemExit(1)
