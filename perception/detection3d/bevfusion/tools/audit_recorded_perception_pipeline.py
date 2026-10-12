#!/usr/bin/env python3
"""Audit recorded topic delivery/classes; no raw rerun or GT accuracy claim."""
import argparse
from bisect import bisect_left
from collections import Counter,defaultdict
import json
import math
from pathlib import Path

import numpy as np

PREFIX='/perception/object_recognition/'
TOPICS={PREFIX+'detection/objects':'detection',PREFIX+'tracking/measurement_objects':'measurement',
        PREFIX+'tracking/objects':'tracking',PREFIX+'objects':'prediction'}


def statistics(values):
    if not values:return dict(n=0,mean=None,p50=None,p90=None,p95=None,maximum=None)
    return dict(n=len(values),mean=float(np.mean(values)),p50=float(np.median(values)),
        p90=float(np.percentile(values,90)),p95=float(np.percentile(values,95)),maximum=float(max(values)))


def read(path):
    import rosbag
    frames=defaultdict(list);odom=[]
    with rosbag.Bag(str(path),'r') as b:
        for topic,msg,t in b.read_messages(topics=list(TOPICS)+['/localization/kinematic_state']):
            stamp=msg.header.stamp.to_sec()
            if topic=='/localization/kinematic_state':
                if msg.header.frame_id=='map':
                    p=msg.pose.pose.position;odom.append((stamp,p.x,p.y))
                continue
            objects=[]
            for o in msg.objects:
                label=max(o.classification,key=lambda c:c.probability).label if o.classification else 0
                k=o.kinematics
                pose=(k.initial_pose_with_covariance if TOPICS[topic]=='prediction' else k.pose_with_covariance).pose
                twist=(k.initial_twist_with_covariance if TOPICS[topic]=='prediction' else k.twist_with_covariance).twist
                objects.append(dict(label=int(label),id=bytes(o.object_id.uuid).hex() if hasattr(o,'object_id') else None,
                    center=[pose.position.x,pose.position.y],score=float(o.existence_probability),
                    speed=math.hypot(twist.linear.x,twist.linear.y)))
            frames[TOPICS[topic]].append(dict(stamp=stamp,received=t.to_sec(),frame=msg.header.frame_id,objects=objects))
    return frames,sorted(odom)


def audit(path):
    frames,odom=read(path);times=[r[0] for r in odom];result={}
    for stage,rows in frames.items():
        classes=Counter();ranges=Counter();scores=defaultdict(list);histories=defaultdict(list)
        for r in rows:
            ego=None
            if times:
                i=bisect_left(times,r['stamp']);candidates=odom[max(0,i-1):i+1]
                nearest=min(candidates,key=lambda a:abs(a[0]-r['stamp']))
                if abs(nearest[0]-r['stamp'])<=.2:ego=nearest[1:]
            for o in r['objects']:
                classes[o['label']]+=1;scores[str(o['label'])].append(o['score'])
                if o['id']:histories[o['id']].append((r['stamp'],o['label']))
                if r['frame'] in ('lidar1','velodyne'):d=math.hypot(*o['center'])
                elif r['frame']=='map' and ego:d=math.hypot(o['center'][0]-ego[0],o['center'][1]-ego[1])
                else:ranges['unresolved_frame_or_ego']+=1;continue
                band=next((f'{lo}_{hi}m' for lo,hi in ((0,10),(10,20),(20,30),(30,50),(50,80),(80,100)) if lo<=d<hi),'ge100m')
                ranges[str(o['label'])+'/'+band]+=1
        switches=[]
        for ident,h in histories.items():
            for old,new in zip(sorted(h),sorted(h)[1:]):
                if old[1]!=new[1]:switches.append(dict(id=ident,stamp=new[0],before=old[1],after=new[1]))
        stamps=sorted({r['stamp'] for r in rows})
        result[stage]=dict(messages=len(rows),nonempty_messages=sum(bool(r['objects']) for r in rows),
            objects=sum(classes.values()),classes=dict(classes),ranges=dict(ranges),
            score_by_label={k:statistics(v) for k,v in scores.items()},
            receive_minus_header_sec=statistics([r['received']-r['stamp'] for r in rows]),
            header_interval_sec=statistics(np.diff(stamps).tolist()),
            class_switches=len(switches),class_switch_examples=switches[:20],track_ids=len(histories),
            caveat='Delivery only. Detection ranges use LiDAR origin; map-track ranges use nearby recorded Ego origin. No same-origin range accuracy, association truth or visibility claim.')
    return dict(path=str(path.resolve()),odom_messages=len(odom),stages=result)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inventory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError('Fresh report required')
    inventory=json.loads(a.inventory.read_text())
    selected=[r for r in inventory['rows'] if r.get('perception') and (
        r.get('recorded_model',{}).get('preset')=='base5_e24' or '/static_obstacle_tracking/' in r['path'])]
    report=dict(scope='Recorded E24 receipts + historical static regression, separated by provenance.',rows=[])
    for r in selected:
        entry=audit(Path(r['path']));entry['recorded_model']=r.get('recorded_model');report['rows'].append(entry)
        print(Path(r['path']).name,[(k,v['nonempty_messages'],v['messages']) for k,v in entry['stages'].items()],flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
