#!/usr/bin/env python3
"""Exact output comparison and paired CPU timing of scalar/indexed route sampling."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import time
from unittest.mock import patch

import yaml

from audit_base5_predicted_path_selection import stats
from experiment_base5_map_cv_candidates import ROOT,digest
from probe_base5_map_lane_history import lanelet_coordinates


def main():
    import perception_object_pipeline.map_based_prediction as core
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--messages',type=Path,required=True)
    parser.add_argument('--packs',nargs='+',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Fresh report required')
    data=json.load(gzip.open(args.messages,'rt'))
    alignment=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    cases={}
    for case in ('O02_164012','P01_163434','H03_162301','R01_163034'):
        frames=[]
        for original in data[case+'/off/policy']:
            objects=[]
            for obj in original['objects']:
                point,yaw=lanelet_coordinates(obj['center'],obj['yaw'],alignment)
                objects.append(dict(obj,center=point,yaw=yaw))
            frames.append(dict(original,objects=objects))
        cases[case]=frames
    for pack in args.packs:
        packed=json.load(gzip.open(pack,'rt'))
        for case,values in packed['cases'].items():cases[pack.stem+'/'+case]=values['frames']
    network=core.LaneletNetwork.load(ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm',ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    indexed=core._RoutePolyline
    class ScalarReference:
        def __init__(self,points):self.points=points;self.length=core.polyline_length(points)
        def sample(self,distance):return core.sample_polyline(self.points,distance)
        def tangent(self,distance):return core.tangent_yaw(self.points,distance)
    results={}
    for case,frames in cases.items():
        counts=Counter(); timing={'scalar':[],'indexed':[]}; full_time={'scalar':[],'indexed':[]}
        for frame in frames:
            for obj in frame['objects']:
                if obj['class_name']!='vehicle':continue
                kwargs=dict(x=obj['center'][0],y=obj['center'][1],z=obj['center'][2],yaw=obj['yaw'],
                            speed=obj['body_velocity'][0],horizons=tuple(i/10 for i in range(1,31)))
                order=('scalar','indexed') if counts['vehicle_observations']%2 else ('indexed','scalar')
                outputs={};cost={}
                for mode in order:
                    with patch.object(core,'_RoutePolyline',ScalarReference if mode=='scalar' else indexed):
                        started=time.perf_counter();outputs[mode]=network.predict(**kwargs)
                        cost[mode]=(time.perf_counter()-started)*1000
                if outputs['scalar']!=outputs['indexed']:
                    raise ValueError(f'Exact path output mismatch in {case} at {frame["stamp"]}')
                counts['vehicle_observations']+=1
                counts['mapped_observations']+=bool(outputs['scalar'])
                counts['compared_paths']+=len(outputs['scalar'])
                counts['compared_future_poses']+=sum(len(p.poses) for p in outputs['scalar'])
                if counts['vehicle_observations']>10:
                    for mode in order:
                        full_time[mode].append(cost[mode])
                        if outputs[mode]:timing[mode].append(cost[mode])
        results[case]=dict(counts=dict(counts),mapped_prediction_cost_ms={k:stats(v) for k,v in timing.items()},
                           all_vehicle_cost_ms={k:stats(v) for k,v in full_time.items()},exact_output_equal=True)
        print(case,dict(counts),'mapped mean ms',
              {mode:round(value['mean'],3) if value['mean'] is not None else None for mode,value in results[case]['mapped_prediction_cost_ms'].items()},flush=True)
    report=dict(source_sha256=digest(__file__),map_source_sha256=digest(ROOT/'base_ws/src/perception_object_pipeline/src/perception_object_pipeline/map_based_prediction.py'),
                original_map_source_sha256='f15dda428c1b2472903f20fe2d8aea59c0923620f149fbb7ce81ec78d8aae968',
                input_sha256=digest(args.messages),pack_sha256={str(p):digest(p) for p in args.packs},
                results=results,exact_output_equal=True,forecast_policy_changed=False,
                caveat='Same scalar helper semantics, interleaved paired local CPU measurements. '
                       'All selected vehicle outputs compared, including unresolved GT frames. '
                       'Not ROS/LAN end-to-end latency, 4090 benchmark, or planner safety certification.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
