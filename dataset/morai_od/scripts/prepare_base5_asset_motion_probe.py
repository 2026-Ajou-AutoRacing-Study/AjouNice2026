#!/usr/bin/env python3
"""Representative asset static/moving paired probes; not a bulk training deck."""
import argparse
import copy
import math
import json
import statistics
from pathlib import Path
import generate_base5_independent as gen
from base5_reference_frames import references
from prepare_base5_motion_appearance_probe import backward_start

CASES=[('Dog1',1.2),('Dog2',1.2),('Deer1',1.2),('Ped_Pet1',1.1),
       ('BabyStroller2',1.1),('BabyStroller3',1.1),('Bike3',2.5),('Scooter1',2.5),
       ('NCAP_EPTa',1.1),('CargoBox',1.1),('OBJ_Hyundai_Grandeur',1.1),
       ('2016_Hyundai_Ioniq',5.),('2014_Hyundai_Xcient(Mixer)',5.)]


def appearance_height_calibration(report_path):
    if report_path is None:return {}
    rows=json.loads(report_path.read_text())['rows'];result={}
    for name,_ in CASES:
        valid=[r for r in rows if r['asset']==name and r['condition_valid']]
        static=[r['object_position_before']['z'] for r in valid if r['mode']=='static']
        moving=[r['object_position_before']['z'] for r in valid if r['mode']=='moving']
        if len(static)!=2 or len(moving)!=2:raise ValueError('Incomplete height calibration: '+name)
        if max(static)-min(static)>.02 or max(moving)-min(moving)>.02:raise ValueError('Unstable height: '+name)
        result[name]=dict(static_z=statistics.median(static),moving_z=statistics.median(moving),
            shift=statistics.median(static)-statistics.median(moving),source=str(report_path.resolve()))
    return result


def prepare(reference,output,height_reference=None):
    if output.exists():raise FileExistsError('Probe overwrite refused')
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    assets={a['name']:a for pool in ctx['pools'].values() for a in pool}
    original=gen.geo.load_json(reference/'metadata.json');source=gen.geo.load_json(reference/'scenario.json')
    heights=appearance_height_calibration(height_reference)
    output.mkdir(parents=True);entries=[]
    from shapely.geometry import LineString
    for index,(name,natural_speed) in enumerate(CASES):
        asset=assets[name];size,off,known=gen.geometry(asset,ctx)
        if not known:
            # Conservative placement envelope, never an invented training GT.
            size=[5.,3.,3.];off=[0.,0.,1.5]
        vehicle=asset['source_list']=='vehicleList'
        if vehicle:
            corridor=ctx['sample_corridors'][0];route=corridor['route'];arc=corridor['arc']
            target_pose=gen.geo.continuous_route_pose_at_arc(route,arc,180.)
            ego=gen.geo.continuous_route_pose_at_arc(route,arc,120.)
            yaw=target_pose['yaw'];target=dict(x=target_pose['x'],y=target_pose['y'],z=target_pose['z']+.36)
            base=ego
        else:
            ego=original['ego'];base=original['base'];old=original['runtime_actors'][0]
            dx,dy=old['geometric_center_estimate'][0]-ego['x'],old['geometric_center_estimate'][1]-ego['y']
            radius=math.hypot(dx,dy);direction=(-dy/radius,dx/radius)
            yaw=math.degrees(math.atan2(direction[1],direction[0]))
            target=dict(old['position'])
        a=math.radians(yaw);dx,dy=math.cos(a),math.sin(a)
        target_center=[target['x']+math.cos(a)*off[0]-math.sin(a)*off[1],
                       target['y']+math.sin(a)*off[0]+math.cos(a)*off[1]]
        ea=math.radians(ego['yaw']);ego_poly=gen.rectangle([ego['x']+1.35*math.cos(ea),ego['y']+1.35*math.sin(ea)],[4.8,2],ego['yaw'])
        for mode,speed in [('static',0.),('moving',natural_speed)]:
            for repeat in range(2):
                budget=(8. if vehicle else 6.)+repeat*.3;back=speed*budget;post=5.
                if vehicle:
                    start_pose=gen.geo.continuous_route_pose_at_arc(route,arc,180.-back)
                    start=dict(start_pose,z=start_pose['z']+.36)
                    endpoint=gen.geo.continuous_route_pose_at_arc(route,arc,185.)
                    end=dict(x=endpoint['x'],y=endpoint['y'],z=endpoint['z']+.36)
                    nodes=[gen.geo.continuous_route_pose_at_arc(route,arc,x) for x in range(math.floor(180-back),186)]
                    swept=LineString([(n['x'],n['y']) for n in nodes]).buffer(math.hypot(*size[:2])/2+math.hypot(*off[:2]))
                else:
                    start=dict(backward_start(target,(dx,dy),speed,budget),yaw=yaw)
                    end=dict(x=target['x']+post*dx,y=target['y']+post*dy,z=target['z'])
                    swept=LineString([(start['x'],start['y']),(end['x'],end['y'])]).buffer(math.hypot(*size[:2])/2+math.hypot(*off[:2]))
                    if speed and name in heights:
                        # Diagnostic-only measured waypoint Z offset, never a GT correction.
                        start['z']+=heights[name]['shift'];end['z']+=heights[name]['shift']
                if swept.distance(ego_poly)<.3:raise ValueError('Swept collision risk: '+name)
                for p in (start,target,end):
                    refs=references([p['x'],p['y']],ego,base['yaw'])
                    if abs(refs['ego_longitudinal_m'])>=99 or abs(refs['ego_lateral_m'])>=29:
                        raise ValueError('Path ROI risk: '+name)
                s=copy.deepcopy(source);gen.geo.clear_actor_lists(s);gen.geo.set_ego_pose(s['egoVehicle'],ego,initial_speed_kph=0.)
                motion='static' if speed==0 else 'walking'
                path=None if speed==0 else dict(policy='one-way-slow-v1',length_m=back+post,
                    points=[{k:start[k] for k in ('x','y','z')},end],nominal_speed_mps=speed,
                    nominal_duration_sec=(back+post)/speed,speed_field_value=speed*3.6,
                    limitation='NPC uses legal route; other assets use two waypoints. Appearance diagnostic only.')
                # Static vehicle helper adds its own +0.36m spawn-height offset.
                set_start=dict(start,z=start['z']-.36) if vehicle else start
                gen.set_pose(copy.deepcopy(asset['template']),set_start,8500,asset,s,motion,None if vehicle else path)
                if vehicle and speed:
                    gen.geo.configure_direct_dynamic_vehicle(s['vehicleList'][0],start_pose,speed*3.6,8500,
                        list(dict.fromkeys(p['link_idx'] for p in route)),ctx['links'])
                start_a=math.radians(start['yaw']);center=[start['x']+math.cos(start_a)*off[0]-math.sin(start_a)*off[1],start['y']+math.sin(start_a)*off[0]+math.cos(start_a)*off[1]]
                record=dict(unique_id=8500,data_id=asset['data_id'],name=name,source_list=asset['source_list'],
                    motion=motion,position={k:start[k] for k in ('x','y','z')},geometric_center_estimate=center,
                    geometry_reference_known=known,estimated_size=size,planned_center_distance_m=math.hypot(center[0]-ego['x'],center[1]-ego['y']),
                    training_class='animal' if asset.get('ontology_class')=='Animal' else 'component_annotation',
                    actual_ego_role=references(center,ego,base['yaw'])['ego_role'])
                if path:record['motion_path']=path
                m=copy.deepcopy(original);m.update(ego=ego,base=base,runtime_actors=[record])
                m['spec'].update(motion=motion,distance_bounds=[5,100],actors=1)
                m['motion_probe']=dict(asset=name,target_position=target,target_center_xy=target_center,
                    commanded_speed_mps=speed,estimated_arrival_sec=budget,backward_start_m=back,
                    target_tolerance_m=.4,arrival_timeout_sec=15.,repeat=repeat,mode=mode,
                    limitation='Two phases per mode; does not establish all-asset or speed-animation law')
                if speed and not vehicle and name in heights:
                    m['motion_probe']['height_calibration']=heights[name]
                ident=f'b5asset_{index:02d}_{mode}_r{repeat}';sp=output/(ident+'.json');mp=output/(ident+'_metadata.json')
                gen.geo.write_json(sp,s);gen.geo.write_json(mp,m)
                entries.append(dict(id=ident,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),
                    family='pedestrian_related' if not vehicle else 'highway',role=record['actual_ego_role'],
                    distance_bounds=[5,100],motion=motion,subfamily='asset_motion_diagnostic'))
    manifest=dict(schema_version=1,dataset_id='base5_asset_motion_probe',config=gen.geo.load_json(gen.CONFIG),
        scenes=entries,count=len(entries),start=0,status='DIAGNOSTIC_ONLY',geometry_sources=ctx['geometry_sources'])
    gen.geo.write_json(output/'manifest.json',manifest);return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--height-reference-report',type=Path,help='Optional measured stationary/moving height match; diagnostic only')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.reference,a.output,a.height_reference_report))
