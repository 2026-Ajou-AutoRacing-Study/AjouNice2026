#!/usr/bin/env python3
"""Offline real-message audit of map-terminal truncation and highway direction recovery."""
import argparse
from collections import Counter
from copy import deepcopy
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import types

import yaml
from analyze_e12_map_horizon import prepare_case, digest
from qualify_e12_mixed_horizon import build_message, wire

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pack',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--reference-commit',default='5e045053')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Use a fresh evidence directory')
    from perception_object_pipeline.map_based_prediction import LaneletNetwork,MapPredictionConfig
    from perception_object_pipeline.mixed_horizon import MixedHorizonSelector
    node_path=ROOT/'base_ws/src/perception_object_pipeline/scripts/map_based_prediction_node.py'
    core_path=ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py'
    spec=importlib.util.spec_from_file_location('terminal_horizon_node',node_path)
    node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
    old=types.ModuleType('reference_fixed_horizon_node')
    oldcode=subprocess.check_output(['git','show',a.reference_commit+':'+str(node_path.relative_to(ROOT))],cwd=ROOT)
    exec(compile(oldcode,'<git reference node>','exec'),old.__dict__)
    config_path=ROOT/'base_ws/src/perception_object_pipeline/config/map_based_prediction.yaml'
    params=yaml.safe_load(config_path.read_text())
    cfg=MapPredictionConfig(**{k:v for k,v in params.items() if k in MapPredictionConfig.__dataclass_fields__})
    mapfile=ROOT/'kcity_map/lanelet_map/lanelet2_map_v87.osm'
    projector=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml'
    tf=ROOT/'configs/tf/morai_lanelet_static_tf.yaml'
    sources=[node_path,core_path,config_path,mapfile,projector,tf,a.pack,Path(__file__)]
    hashes={str(f):digest(f) for f in sources}
    raw=json.load(gzip.open(a.pack,'rt'))
    transform=yaml.safe_load(tf.read_text())['map_to_morai_map']
    reports={};lengths=Counter();bad=[]
    base=tuple(.1*i for i in range(1,31));long=tuple(.1*i for i in range(1,81))
    for name,case in raw['cases'].items():
        net=LaneletNetwork.load(mapfile,projector,cfg)
        selector=MixedHorizonSelector(net,current_lanelet_only=True,entry_confirm_sec=0.)
        ext=node.MixedHorizonExtension(net,base,long,selector,mode='native',highway_direction_recovery=True)
        frames,_=prepare_case(case,transform);c=Counter()
        for index,(r,f) in enumerate(zip(case['frames'],frames)):
            msg=build_message(name,index,r,f,transform);before=wire(msg)
            original=old.make_map_based_predicted_objects(msg,base,net)[0]
            baseline=node.make_map_based_predicted_objects(msg,base,net)[0]
            out,mapped,fallback,_=node.make_map_based_predicted_objects(msg,base,net,ext)
            c.update(frames=1,objects=len(out.objects),map=mapped,kinematic=fallback,
                     highway_map=ext.native_map_objects,direction_recovered=ext.direction_recovered_objects,
                     native_short=ext.native_short_objects,errors=ext.extension_errors)
            if wire(msg)!=before:raise AssertionError('input mutation')
            vehicles={str(o['id']):o for o in f['objects']}
            for tracked,o,ref,short in zip(msg.objects,out.objects,original.objects,baseline.objects):
                x,y,z=(getattr(tracked.kinematics.pose_with_covariance.pose.position,k) for k in ('x','y','z'))
                tid=next((str(t['id']) for t in r['tracks'] if t['confirmed'] and
                          list(tracked.object_id.uuid)==__import__('qualify_e12_mixed_horizon').synthetic_uuid(name,t['id'])),None)
                common=[deepcopy(o),deepcopy(ref)]
                for obj in common:obj.kinematics.predicted_paths=[]
                if wire(common[0])!=wire(common[1]):raise AssertionError('current state/UUID mutation')
                paths=o.kinematics.predicted_paths
                if not paths:raise AssertionError('empty object paths')
                cap=81 if len(short.kinematics.predicted_paths[0].path)>31 or any(len(q.path)>31 for q in paths) else 31
                for path in paths:
                    n=len(path.path);lengths[n]+=1
                    if not 1<=n<=81:raise AssertionError('invalid path length')
                    for pose in path.path:
                        if not all(math.isfinite(v) for v in (pose.position.x,pose.position.y,pose.position.z,
                                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)):
                            raise AssertionError('nonfinite output')
                    if n not in (31,81):c['variable_length_paths']+=1
                if wire(o)==wire(ref):c['unchanged_objects']+=1
                if tid in vehicles:
                    c['vehicle']+=1
                    obs=vehicles[tid];v=obs['body_velocity'][0]
                    # Record failures by actual geometric highway region, not scene name alone.
                    regional=net.recover_lane_direction(x,y,selector.highway_lanelet_ids) is not None
                    if regional:c['on_highway_geometry']+=1
                    if name.find('H03')>=0 or name.find('H04')>=0 or 'ind_highway' in name:
                        if wire(o)==wire(short) and not net.predict(x,y,z,obs['yaw'],v,base,truncate_at_route_end=True):
                            c['highway_scene_kinematic']+=1
                            reason='low_speed' if v<cfg.minimum_speed_mps else 'no_lane_match'
                            c['fallback_'+reason]+=1
                            if regional:c['on_highway_geometry_fallback']+=1
                            bad.append(dict(case=name,stamp=f['stamp'],id=tid,reason=reason,
                                            on_highway_geometry=regional,speed=v,xy=[x,y]))
        reports[name]=dict(c);print(name,dict(c),flush=True)
    for f,h in hashes.items():
        if digest(Path(f))!=h:raise AssertionError('source changed during qualification')
    report=dict(source_sha256=hashes,reference_commit=a.reference_commit,
                reference_node_sha256=hashlib.sha256(oldcode).hexdigest(),cases=reports,
                totals=dict(sum((Counter(v) for v in reports.values()),Counter())),
                path_pose_counts=dict(lengths),remaining_fallbacks=bad,
                limitations=['Offline saved E12 tracks; not live ROS/planner safety certification.',
                  'Map recovery is not a false-positive classifier; current states/scores/lifetimes are unchanged.',
                  'Highway-scene and geometric-highway denominators differ.'])
    a.output.mkdir(parents=True)
    (a.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (a.output/'COMPLETE.json').write_text(json.dumps(dict(complete=True,report_sha256=digest(a.output/'report.json')))+'\n')
    print(json.dumps(report['totals'],sort_keys=True),flush=True)


if __name__=='__main__':main()
