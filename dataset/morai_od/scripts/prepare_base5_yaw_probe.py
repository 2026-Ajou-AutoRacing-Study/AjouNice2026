#!/usr/bin/env python3
"""Frozen-world paired yaw probe; only Ego yaw varies, no off-road NPC fabrication."""
import argparse
import copy
import math
from pathlib import Path
import generate_base5_independent as gen
from base5_reference_frames import references,project_xy

ANGLES=(-15,-10,-3,0,3,10,15)


def prepare(source,output):
    if output.exists():raise FileExistsError('Probe output overwrite refused')
    m=gen.geo.load_json(source);cache={}
    def meta(i):
        if i not in cache:cache[i]=gen.geo.load_json(Path(m['scenes'][i]['metadata']))
        return cache[i]
    # Five fixed observed road layouts plus an independently selected far straight rear.
    chosen=[('straight_front',214),('adjacent_left',4),('adjacent_right',508),
        ('curve_right',9155),('curve_left',9408)]
    rear=[]
    for i,row in enumerate(m['scenes']):
        if row['family']!='highway' or row['role']!='rear' or row['distance_bounds']!=[80,99]:continue
        a=meta(i)
        if len(a['runtime_actors'])!=1 or abs(a['ego_lateral_offset_m'])>.15:continue
        actor=a['runtime_actors'][0]
        if actor['name'].startswith('DEFAULT_'):continue
        x,y=project_xy(actor['geometric_center_estimate'],a['ego'],a['base']['yaw'])
        if x<-90 and abs(y)<1:rear.append(i)
    if not rear:raise ValueError('Missing controlled rear road layout')
    chosen.insert(1,('straight_rear',min(rear)))
    output.mkdir(parents=True);entries=[];groups=[]
    for group,index in chosen:
        row=m['scenes'][index];original=meta(index);scene=gen.geo.load_json(Path(row['scenario']))
        if len(original['runtime_actors'])!=1 or original['runtime_actors'][0]['motion']!='static':
            raise ValueError('Controlled static single-actor source required')
        actor=original['runtime_actors'][0]
        if actor['source_list']!='vehicleList':raise ValueError('Road vehicle required')
        if abs(original['ego_lateral_offset_m'])>.15:raise ValueError('Yaw envelope unsupported')
        road_yaw=original['base']['yaw'];fixed_sha=gen.hashlib.sha256(gen.json.dumps(scene['vehicleList'],sort_keys=True).encode()).hexdigest()
        groups.append(dict(group=group,source_index=index,source_id=row['id'],asset=actor['name'],
            source_sha256=row['scenario_sha256'],road_link=original['base']['link_idx'],fixed_actor_sha256=fixed_sha,
            road_reference=references(actor['geometric_center_estimate'],dict(original['ego'],yaw=road_yaw),road_yaw)))
        for angle in ANGLES:
            name=f'b5yaw_{group}_{angle:+03d}'.replace('+','p').replace('-','m')
            s=copy.deepcopy(scene);a=copy.deepcopy(original);e=dict(a['ego'],yaw=road_yaw+angle)
            gen.geo.set_ego_pose(s['egoVehicle'],e,initial_speed_kph=0.)
            a.update(ego=e,ego_road_yaw_offset_deg=angle)
            refs=references(actor['geometric_center_estimate'],e,road_yaw)
            a['runtime_actors'][0].update(reference_frames=refs,longitudinal_m=refs['ego_longitudinal_m'],
                lateral_m=refs['ego_lateral_m'],actual_ego_role=refs['ego_role'])
            a['yaw_probe']=dict(group=group,angle_deg=angle,fixed_actor_sha256=fixed_sha,
                source_id=row['id'],road_yaw_deg=road_yaw,ego_heading_correction_deg=-1.347,diagnostic_only=True,
                limitation='Road-supported source and lane-width proxy; not planner trajectory or collider certification')
            if gen.hashlib.sha256(gen.json.dumps(s['vehicleList'],sort_keys=True).encode()).hexdigest()!=fixed_sha:
                raise AssertionError('NPC changed during paired yaw probe')
            sp=output/(name+'.json');mp=output/(name+'_metadata.json')
            gen.geo.write_json(sp,s);gen.geo.write_json(mp,a)
            entry=copy.deepcopy(row);entry.update(id=name,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
                scenario_sha256=gen.geo.sha256(sp));entries.append(entry)
    m.update(count=len(entries),start=0,scenes=entries,status='YAW_DIAGNOSTIC_ONLY; not quota replacement',
        yaw_probe_provenance=dict(groups=groups,angles=list(ANGLES),source_sha256=gen.geo.sha256(source),
            generator_sha256=gen.geo.sha256(Path(__file__))))
    gen.geo.write_json(output/'manifest.json',m);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.manifest,a.output))
