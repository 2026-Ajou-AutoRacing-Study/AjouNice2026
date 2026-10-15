#!/usr/bin/env python3
"""Read-only development-bag geometry reference; never reads locked evaluation bags."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from base5_roundabout_mission import project


def summarize(values):
    return dict(zip(('min','p10','p50','p90','max'), map(float, np.percentile(values,[0,10,50,90,100])))) if values else None


def report(bags, output, velocity_units):
    import rosbag
    import base5_roundabout_mission as mission
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    corridor = mission.corridor(gen, ctx)
    result = dict(status='DEVELOPMENT_REFERENCE_NOT_FINAL_SCENARIO_PARAMETERS',
        sample_sha256=sha256(gen.HIGHWAY_SAMPLE), npc_route_length_m=float(corridor['arc'][-1]),
        npc_start=corridor['start'], npc_end=corridor['end'], bags=[],
        limitation='Reported bag speeds and geometric proximity are observations, not disclosed final spawn periods or collision-safe control.')
    if len(bags)!=len(velocity_units):raise ValueError('One explicit velocity unit per bag required')
    for path,unit in zip(bags,velocity_units):
        if 'evaluation_locked' in str(path):
            raise ValueError('Locked evaluation bag cannot tune generation')
        ego_rows, npc_speeds, pairs, near_frames, pose_rows = [], [], [], [], []
        previous={};unit_ratios=[]
        with rosbag.Bag(str(path)) as bag:
            for topic, msg, stamp in bag.read_messages(topics=['/Ego_topic','/Object_topic']):
                if topic == '/Ego_topic':
                    p=msg.position
                    arc, distance, lateral, j = project(ctx['route'],ctx['arc'],[p.x,p.y])
                    if not 855 <= arc <= 940 or distance > 6:
                        continue
                    yaw=float(msg.heading)-1.347
                    tangent=math.degrees(math.atan2(ctx['route'][j+1]['y']-ctx['route'][j]['y'],ctx['route'][j+1]['x']-ctx['route'][j]['x']))
                    ego_rows.append((stamp.to_sec(),arc,lateral,(yaw-tangent+180)%360-180))
                    pose_rows.append([p.x,p.y])
                else:
                    values=[]
                    for obj in msg.npc_list:
                        arc,distance,_,_=project(corridor['route'],corridor['arc'],[obj.position.x,obj.position.y])
                        if distance > 1.5:
                            continue
                        values.append(arc);npc_speeds.append(float(obj.velocity.x)*(3.6 if unit=='mps' else 1.))
                        now=stamp.to_sec();speed=math.hypot(obj.velocity.x,obj.velocity.y)/(3.6 if unit=='kmh' else 1.)
                        old=previous.get(obj.unique_id)
                        if old and .15<=now-old[0]<=.5 and speed>1.:
                            derived=math.hypot(obj.position.x-old[1],obj.position.y-old[2])/(now-old[0])
                            if derived<30:unit_ratios.append(derived/speed)
                        if old is None or now-old[0]>=.15:
                            previous[obj.unique_id]=(now,obj.position.x,obj.position.y)
                    values=sorted(values)
                    if values:
                        near_frames.append(len(values))
                    pairs.extend(right-left for left,right in zip(values,values[1:]) if right-left>.5)
        result['bags'].append(dict(path=str(path.resolve()),sha256=sha256(path),
            ego_samples=len(ego_rows), ego_route_arc_m=summarize([r[1] for r in ego_rows]),
            ego_lateral_m=summarize([r[2] for r in ego_rows]),ego_yaw_delta_deg=summarize([r[3] for r in ego_rows]),
            npc_forward_speed_kph=summarize(npc_speeds), npc_pivot_arc_interval_m=summarize(pairs),
            observed_npc_count=summarize(near_frames), input_velocity_unit=unit,
            position_difference_to_normalized_message_speed_ratio=summarize(unit_ratios)))
        if pose_rows:
            result['bags'][-1]['ego_xy_samples']=pose_rows[::max(1,len(pose_rows)//100)]
    output.parent.mkdir(parents=True,exist_ok=True)
    atomic_json(output,result)
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--velocity-unit',choices=('mps','kmh'),action='append',required=True)
    args=p.parse_args()
    result=report(args.bag,args.output,args.velocity_unit)
    print(json.dumps(dict(result,bags=[{k:v for k,v in r.items() if k!='ego_xy_samples'} for r in result['bags']]),ensure_ascii=False,indent=2))
