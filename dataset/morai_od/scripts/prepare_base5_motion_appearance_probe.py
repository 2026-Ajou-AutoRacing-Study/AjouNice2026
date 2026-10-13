#!/usr/bin/env python3
"""Same pedestrian/target/yaw, natural-speed backward start, 12 diagnostic captures."""
import argparse
import copy
import math
from pathlib import Path
import generate_base5_independent as gen
from base5_reference_frames import references


def backward_start(target,direction,speed,budget):
    dx,dy=direction
    if speed<0 or budget<=0 or abs(math.hypot(dx,dy)-1)>1e-6:
        raise ValueError('Nonnegative speed, positive budget and unit direction required')
    return dict(x=target['x']-speed*budget*dx,y=target['y']-speed*budget*dy,z=target['z'])


def prepare(capture,output):
    if output.exists():raise FileExistsError('Probe overwrite refused')
    original=gen.geo.load_json(capture/'metadata.json');source=gen.geo.load_json(capture/'scenario.json')
    actor=original['runtime_actors'][0]
    if len(original['runtime_actors'])!=1 or actor['source_list']!='pedestrianList':
        raise ValueError('One ordinary pedestrian source required')
    target=actor['position'];ego=original['ego'];center=actor['geometric_center_estimate']
    # Tangential passage keeps minimum radial distance at the target and avoids Ego.
    radius=math.hypot(center[0]-ego['x'],center[1]-ego['y'])
    dx,dy=-(center[1]-ego['y'])/radius,(center[0]-ego['x'])/radius
    yaw=math.degrees(math.atan2(dy,dx));e=gen.math.radians(ego['yaw'])
    ego_box=gen.rectangle([ego['x']+1.35*math.cos(e),ego['y']+1.35*math.sin(e)],[4.8,2],ego['yaw'])
    output.mkdir(parents=True);entries=[]
    from shapely.geometry import LineString
    for speed in (0.,.35,1.1,2.2):
        for repeat in range(3):
            # Deliberately vary arrival phase/time instead of always capturing one pose.
            budget=5.+repeat*.25;backward=speed*budget;post=4.
            start=dict(backward_start(target,(dx,dy),speed,budget),yaw=yaw)
            end=dict(x=target['x']+post*dx,y=target['y']+post*dy,z=target['z'])
            start_center=[center[0]-backward*dx,center[1]-backward*dy]
            for pt in (start_center,center,[center[0]+post*dx,center[1]+post*dy]):
                refs=references(pt,ego,original['base']['yaw'])
                if abs(refs['ego_longitudinal_m'])>=99.5 or abs(refs['ego_lateral_m'])>=29.5:
                    raise ValueError('Probe path outside ROI')
            swept=LineString([(start['x'],start['y']),(end['x'],end['y'])]).buffer(
                math.hypot(*actor['estimated_size'][:2])/2+1.)
            if swept.distance(ego_box)<.3:raise ValueError('Probe swept path crosses Ego')
            s=copy.deepcopy(source);gen.geo.clear_actor_lists(s)
            a=copy.deepcopy(original);record=copy.deepcopy(actor)
            asset=dict(name=actor['name'],source_list='pedestrianList',ontology_class='Pedestrian')
            motion='standing' if speed==0 else 'walking' if speed<=1.1 else 'running'
            path=None if speed==0 else dict(policy='one-way-slow-v1',length_m=backward+post,
                distance_bounds_m=[10,30],points=[{k:start[k] for k in ('x','y','z')},end],
                nominal_speed_mps=speed,speed_field_value=speed*3.6,nominal_duration_sec=(backward+post)/speed,
                appearance_profile=True)
            gen.set_pose(copy.deepcopy(source['pedestrianList'][0]),start,actor['unique_id'],asset,s,motion,path)
            record.update(position={k:start[k] for k in ('x','y','z')},geometric_center_estimate=start_center,
                motion=motion,planned_center_distance_m=math.hypot(start_center[0]-ego['x'],start_center[1]-ego['y']))
            record.pop('motion_path',None)
            if path:record['motion_path']=path
            a.update(runtime_actors=[record],motion_probe=dict(target_position=target,target_center_xy=center,
                commanded_speed_mps=speed,estimated_arrival_sec=budget,backward_start_m=backward,
                target_tolerance_m=.30,arrival_timeout_sec=12.,repeat=repeat,
                limitation='Speed-only API comparison; no explicit walk/run animation switch; not a training quota'))
            a['spec'].update(motion=motion,distance_bounds=[10,30],actors=1)
            name=f'b5motion_s{round(speed*100):03d}_r{repeat}'
            sp=output/(name+'.json');mp=output/(name+'_metadata.json')
            gen.geo.write_json(sp,s);gen.geo.write_json(mp,a)
            entries.append(dict(id=name,scenario=str(sp.resolve()),metadata=str(mp.resolve()),
                scenario_sha256=gen.geo.sha256(sp),family='pedestrian_related',role=a['spec']['role'],
                distance_bounds=[10,30],motion=motion,subfamily='person',split='train'))
    m=dict(schema_version=1,dataset_id='base5_motion_appearance_probe',config=gen.geo.load_json(gen.CONFIG),
        scenes=entries,count=len(entries),start=0,status='DIAGNOSTIC_ONLY',source_capture=str(capture.resolve()))
    gen.geo.write_json(output/'manifest.json',m);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.capture,a.output))
