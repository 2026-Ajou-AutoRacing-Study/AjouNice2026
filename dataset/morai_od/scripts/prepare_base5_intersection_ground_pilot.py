#!/usr/bin/env python3
"""Small forward/reverse editor-height diagnostic, never a bulk training deck."""
import argparse
import copy
import math
from pathlib import Path

import generate_base5_independent as gen
import prepare_base5_intersection_manual_ground as manual
from base5_capture_safety import atomic_json, sha256
from base5_intersection_ground import POLICY, QA_POLICY, height, load_lookup
from base5_map_surface import configure


def prepare(lookup_file, prepared_manifest, output, running=False, lateral_offset=0., ego_distance=20., fast_people=False):
    if output.exists():
        raise FileExistsError('New diagnostic output required')
    data=load_lookup(lookup_file)
    config=gen.geo.load_json(manual.CONFIG)
    config['capture']['natural_setup_budget_sec']=6.
    ctx=configure(gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO),config['placement_ground'])
    assets={a['name']:a for pool in ctx['pools'].values() for a in pool}
    seed=gen.geo.load_json(prepared_manifest)
    entries=[]; output.mkdir(parents=True)
    for row in seed['scenes']:
        if running and row['asset']!='Woman2':continue
        key=row['crosswalk']['crosswalk_id']+'/'+row['asset'];profile=data['profiles'][key]
        asset=assets[row['asset']];size,off,known=gen.geometry(asset,ctx)
        run=running or (fast_people and asset['name'] not in manual.ASSETS[1:])
        speed=2.2 if run else 1.2 if asset.get('ontology_class')=='Animal' else 1.1
        for sign in (1,-1):
            # Keep the whole departure/target/tail inside the measured span.
            lower,upper=profile['s_bounds_m'];reserve=.1
            lo=lower+(speed*6. if sign>0 else 3.)+reserve
            hi=upper-(3. if sign>0 else speed*6.)-reserve
            if lo>hi:raise ValueError('Measured path too short for natural-speed diagnostic')
            s=min(max(0.,lo),hi); axis=profile['axis'];c=profile['center']
            xy=lambda a: (c['x']+a*axis[0]-lateral_offset*axis[1],c['y']+a*axis[1]+lateral_offset*axis[0])
            tx,ty=xy(s);target=height(profile,tx,ty)
            yaw=math.degrees(math.atan2(sign*axis[1],sign*axis[0]))
            pose={k:target[k] for k in ('x','y','z')};pose['yaw']=yaw
            ego=gen.geo.continuous_route_pose_at_arc(ctx['route'],ctx['arc'],row['crosswalk']['route_arc_m']-ego_distance)
            a=math.radians(yaw);center=[tx+math.cos(a)*off[0]-math.sin(a)*off[1],ty+math.sin(a)*off[0]+math.cos(a)*off[1]]
            distance=math.hypot(center[0]-ego['x'],center[1]-ego['y'])
            bounds=[max(0.,distance-2.),distance+2.]
            path=gen.natural_target_path(pose,center,ego,bounds,'running' if run else 'walking',asset,6.,
                direction=(sign*axis[0],sign*axis[1]))
            mode=profile.get('waypoint_mode')
            start,end=path['points'];n=1 if mode=='endpoint-linear-v117' else max(1,math.ceil(path['length_m']/.5))
            points=[]
            for i in range(n+1):
                t=i/n; p=height(profile,start['x']+t*(end['x']-start['x']),start['y']+t*(end['y']-start['y']))
                points.append({k:p[k] for k in ('x','y','z')})
            path.update(points=points,ground_path_evidence=dict(policy=POLICY,lookup_file=str(lookup_file.resolve()),
                        lookup_sha256=sha256(lookup_file),profile_key=key),
                        limitation='Editor-height diagnostic; no terrain/static-reference recapture, no raw GT edit')
            if mode is not None:path['ground_path_evidence']['waypoint_mode']=mode
            scene,_=manual.build_scene(ctx,row['crosswalk'],asset,row['number'])
            from base5_map_surface import ego_road_support
            support=ego_road_support(ctx['map_surface_index'],ego)
            if not support['road_supported']:raise ValueError('Pilot Ego road support missing')
            ego['z']=support['height_m']
            gen.geo.set_ego_pose(scene['egoVehicle'],ego,0.);manual.park(scene['egoVehicle'])
            gen.geo.clear_actor_lists(scene)
            gen.set_pose(copy.deepcopy(asset['template']),dict(points[0],yaw=yaw),1,asset,scene,
                         'running' if run else 'walking',path)
            actor=dict(unique_id=1,data_id=asset['data_id'],name=asset['name'],source_list=asset['source_list'],
                motion='running' if run else 'walking',position=copy.deepcopy(points[0]),motion_path=path,
                geometric_center_estimate=center,estimated_size=size,geometry_reference_known=known,
                mission_crossing_id=row['crosswalk']['crosswalk_id'],training_class='animal' if asset.get('ontology_class')=='Animal' else 'component_annotation')
            version='117' if mode else '116'
            sid=f'b5ed{version}_{row["number"]:02d}_{"f" if sign>0 else "r"}'+('_run' if run else '')
            meta=dict(calibration_only=True,spec=dict(family='pedestrian_related',role='front',mission=True,
                motion=actor['motion'],distance_bounds=bounds,actors=1),ego=ego,base=ego,runtime_actors=[actor],
                split='train',retry=0,live_validated=False,manual_ground_pilot=dict(number=row['number'],
                profile_key=key,direction=sign,target_s_m=s,lateral_offset_m=lateral_offset,ego_distance_m=ego_distance,
                uncorrected_first_seed=profile['first_knot_is_unchanged_seed']),
                ground_support_check=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15,required_uids=[1],manual_policy=QA_POLICY))
            from base5_motion_ground import validate_motion_ground
            validate_motion_ground(meta,scene)
            sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
            atomic_json(sp,scene);atomic_json(mp,meta)
            entries.append(dict(id=sid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp),
                family='pedestrian_related',role='front',mission=True,distance_bounds=bounds,subfamily='person',
                motion=actor['motion'],split='train'))
    atomic_json(output/'manifest.json',dict(schema_version=2,count=len(entries),dataset_id='editor_ground_diagnostic_v116',
        status='CALIBRATION_ONLY_NOT_TRAINING',config=config,scenes=entries,
        provenance=dict(map_sha256=sha256(gen.geo.DEFAULT_MGEO/'link_set.json'),
                        map_boundary_sha256=sha256(gen.geo.DEFAULT_MGEO/'lane_boundary_set.json'),
                        lookup_sha256=sha256(lookup_file))))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--lookup',type=Path,required=True);p.add_argument('--prepared-manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--running',action='store_true')
    p.add_argument('--lateral-offset-m',type=float,default=0.)
    p.add_argument('--ego-distance-m',type=float,default=20.)
    p.add_argument('--fast-people',action='store_true')
    a=p.parse_args();print(prepare(a.lookup,a.prepared_manifest,a.output,a.running,a.lateral_offset_m,a.ego_distance_m,a.fast_people))
