#!/usr/bin/env python3
"""Offline comparison of braking, sampled stops and transient lamp-box motion.

No driving, inference, input edits or model/map changes. Existing stop reports
locate candidate windows; source bags distinguish movement from logging gaps.
"""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import statistics


def nearest(rows, times, stamp, tolerance=100_000_000):
    i=bisect_left(times,stamp)
    choices=rows[max(0,i-1):min(len(rows),i+1)]
    row=min(choices,key=lambda r:abs(r['t']-stamp)) if choices else None
    return row if row is not None and abs(row['t']-stamp)<=tolerance else None


def median(values):
    return statistics.median(values) if values else None


def analyze_run(root, name):
    import rosbag
    paths={m:root/f'map_{name}_{m}_clearance_analysis_261026_v1/measurements.json' for m in 'ABCD'}
    reports={m:json.loads(p.read_text()) for m,p in paths.items()}
    bag_path=Path(reports['A']['bag'])
    poses=[]; classifiers=[]; brakes=[]; imu=[]; decisions=[]; native=[]; targets=[]; selections=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,stamp in bag.read_messages(topics=['/localization/kinematic_state',
                '/traffic_light_node/debug_classifier','/ctrl_cmd','/sensor/imu',
                '/traffic_light_node/debug_runtime','/traffic_light_node/debug_selection',
                '/Ego_topic','/planning/trajectory']):
            t=stamp.to_nsec()
            if topic=='/localization/kinematic_state':
                p=msg.pose.pose.position;v=msg.twist.twist.linear
                poses.append(dict(t=msg.header.stamp.to_nsec(),xy=[p.x,p.y],speed=math.hypot(v.x,v.y)))
            elif topic=='/traffic_light_node/debug_classifier':
                body=json.loads(msg.data);box=body.get('detector_bbox_xyxy')
                if isinstance(box,list) and len(box)==4:
                    classifiers.append(dict(t=body['source_stamp_ns'],group=body.get('group_id'),top=box[1]))
            elif topic=='/ctrl_cmd':
                brakes.append(dict(t=t,value=msg.brake))
            elif topic=='/sensor/imu':
                q=msg.orientation
                pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
                imu.append(dict(t=msg.header.stamp.to_nsec(),pitch_deg=math.degrees(pitch)))
            elif topic=='/traffic_light_node/debug_runtime':
                body=json.loads(msg.data); d=body.get('frame_decision') or {}
                decisions.append(dict(t=body.get('source_stamp_ns',0),group=d.get('group_id'),
                    final=d.get('final_tuples'),status=d.get('selection_status'),
                    origin=(d.get('observation_timing') or {}).get('output_origin'),reason=d.get('policy_reason')))
            elif topic=='/traffic_light_node/debug_selection':
                d=json.loads(msg.data)
                selections.append(dict(t=d.get('source_stamp_ns',0),group=d.get('group_id'),
                    status=d.get('status'),reason=d.get('reason'),projection_reason=d.get('projection_reason')))
            elif topic=='/Ego_topic':
                native.append(dict(t=msg.header.stamp.to_nsec(),xy=[msg.position.x,msg.position.y],brake=msg.brake))
            elif topic=='/planning/trajectory' and poses and msg.points:
                point=min(msg.points,key=lambda p:math.dist([p.pose.position.x,p.pose.position.y],poses[-1]['xy']))
                targets.append(dict(t=t,speed=point.longitudinal_velocity_mps))
    poses.sort(key=lambda r:r['t']);imu.sort(key=lambda r:r['t'])
    pt=[p['t'] for p in poses];it=[p['t'] for p in imu]
    lamp_samples=[]
    with (root/f'map_{name}_natural_261026_v1/observer/observations.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('event')=='native_read_only_sample':lamp_samples.append(row)
    output=dict(run=name,bag=str(bag_path),groups=[])
    for m,r in reports.items():
        windows=r['stationary_windows'];frames=[f for f in classifiers if f['group']==r['group_id']]
        item=dict(mission=m,stop_windows=len(windows),pass_without_verified_stop=not bool(windows),stops=[])
        for w in windows:
            a,b=w['start_stamp_ns'],w['end_stamp_ns']
            stable=[p for p in poses if a+2e9<=p['t']<=b-.5e9 and p['speed']<.1]
            if not stable:
                stable=[p for p in poses if a<=p['t']<=b and p['speed']<.1]
            if not stable:continue
            center=[median([p['xy'][i] for p in stable]) for i in (0,1)]
            eligible=[]
            for f in frames:
                if not a-3e9<=f['t']<=b:continue
                p=nearest(poses,pt,f['t'])
                if p and p['speed']<.3 and math.dist(p['xy'],center)<=.3:
                    eligible.append(dict(f,xy=p['xy'],speed=p['speed']))
            settled=[f['top'] for f in eligible if a+2e9<=f['t']<=b-.5e9]
            worst=min(eligible,key=lambda f:f['top']) if eligible else None
            # Count separated moving brake applications, not stationary holding.
            moving=[]
            for row in brakes:
                if not a-8e9<=row['t']<=a:continue
                p=nearest(poses,pt,row['t'])
                if p and p['speed']>=.3 and row['value']>.05:moving.append(row['t'])
            bursts=[]
            for t in moving:
                if not bursts or t-bursts[-1][-1]>.3e9:bursts.append([t])
                else:bursts[-1].append(t)
            bursts=[v for v in bursts if v[-1]-v[0]>=.1e9]
            pitches=[p['pitch_deg'] for p in imu if a-3e9<=p['t']<=a+3e9]
            stop=dict(start_ns=a,end_ns=b,duration_sec=(b-a)/1e9,
                stable_position_xy=center,settled_frames=len(settled),settled_top_median_px=median(settled),
                near_same_position_min_top_px=worst['top'] if worst else None,
                transient_drop_px=median(settled)-worst['top'] if settled and worst else None,
                worst=worst,moving_brake_bursts=bursts,
                imu_pitch_minmax_deg=[min(pitches),max(pitches)] if pitches else None)
            if worst:
                pose=nearest(imu,it,worst['t'])
                stop['worst_imu_pitch_deg']=pose['pitch_deg'] if pose else None
            item['stops'].append(stop)
        separations=[]
        for left,right in zip(windows,windows[1:]):
            rows=[p for p in poses if left['end_stamp_ns']<=p['t']<=right['start_stamp_ns']]
            distance=sum(math.dist(a['xy'],b['xy']) for a,b in zip(rows,rows[1:]))
            start,end=left['end_stamp_ns'],right['start_stamp_ns']
            natives=[p for p in native if start<=p['t']<=end]
            changes=[];previous=None
            for d in decisions:
                if d['group'] not in (r['group_id'],0,None) or not start-2e9<=d['t']<=end+2e9:continue
                state=(str(d['final']),d['status'],d['origin'],d['reason'])
                if state!=previous:changes.append(dict(d,relative_to_first_stop_end_sec=(d['t']-start)/1e9))
                previous=state
            selchanges=[];previous=None
            for d in selections:
                if d['group'] not in (r['group_id'],0,None) or not start-2e9<=d['t']<=end+2e9:continue
                state=(d['status'],d['reason'],d['projection_reason'])
                if state!=previous:selchanges.append(dict(d,relative_to_first_stop_end_sec=(d['t']-start)/1e9))
                previous=state
            target_changes=[];previous=None
            for v in targets:
                if not start-2e9<=v['t']<=end+2e9:continue
                state='stop' if v['speed']<=.01 else 'move'
                if state!=previous:target_changes.append(dict(v,state=state,relative_sec=(v['t']-start)/1e9))
                previous=state
            selected_native=[v for v in lamp_samples if v.get('valid') and v['native_id'] in
                ({'A':['C1256W000077'],'B':['C1256W000091'],'C':['C1256W000016','C1256W000018'],'D':['C1256W000052']}[m])
                and start-2e9<=v['request_ros_ns']<=v['response_ros_ns']<=end+2e9]
            separations.append(dict(gap_sec=(end-start)/1e9,
                observed_path_length_m=distance,max_observed_speed_mps=max((p['speed'] for p in rows),default=None),
                max_data_gap_sec=max(((b['t']-a['t'])/1e9 for a,b in zip(rows,rows[1:])),default=None),
                native_ego_net_movement_m=math.dist(natives[0]['xy'],natives[-1]['xy']) if len(natives)>1 else None,
                signal_decision_changes=changes,selection_changes=selchanges,
                nearby_target_changes=target_changes,
                exact_native_sample_colors=sorted(set(v['color'] for v in selected_native)),
                exact_native_sample_count=len(selected_native),
                nearby_target_speed_range=[min(v['speed'] for v in targets if start-2e9<=v['t']<=end+2e9),
                    max(v['speed'] for v in targets if start-2e9<=v['t']<=end+2e9)]
                    if any(start-2e9<=v['t']<=end+2e9 for v in targets) else None,
                warning='Separated low-speed windows alone do not prove repeated braking'))
        item['between_stop_windows']=separations
        output['groups'].append(item)
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    report=dict(scope='Three historical continuous drives; maps/host load/phases differ, not a controlled causal comparison',
        criteria=dict(near_final_position_m=.3,slow_speed_mps=.3,brake_threshold=.05,
            moving_brake_release_gap_sec=.3,minimum_brake_burst_sec=.1),
        limitations=['Missing frames and messages remain missing.',
            'Detector bbox motion is not independent pixel truth or proof of chassis pitch.',
            'Different map stop positions preclude direct absolute-pixel comparisons across versions.'],
        runs=[analyze_run(a.root,n) for n in ('v89_full','v90_corridor','v91_corridor')])
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,indent=2)


if __name__=='__main__':main()
