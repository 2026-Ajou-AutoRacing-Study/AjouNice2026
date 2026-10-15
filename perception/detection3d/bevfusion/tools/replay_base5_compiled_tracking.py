#!/usr/bin/env python3
"""Offline real EKF-core comparison on continuous bag detections, not live ROS.

Uses the same recorded Ego/world transform for both paths. Does not test TF,
node-side orientation canonicalization, output scheduler, planning or GT AVE.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import pickle
import subprocess
import sys

import numpy as np

from analyze_base5_runtime_effects import delivered, distribution, match, POLICIES
from compare_base5_common30 import CLASSES, validate_payload

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'docker/bevfusion'))
from app.sensor4_calibration import load_sensor4_calibration


def ego_trajectory(path):
    import rosbag
    rows={}
    with rosbag.Bag(str(path)) as bag:
        for _,m,t in bag.read_messages(topics=['/Ego_topic']):
            stamp=m.header.stamp.to_sec() or t.to_sec()
            rows[stamp]=(stamp,m.position.x,m.position.y,m.position.z,math.radians(m.heading-1.347))
    a=np.asarray([rows[t] for t in sorted(rows)],float)
    if a.ndim!=2 or len(a)<2 or not np.isfinite(a).all():
        raise ValueError('Recorded Ego trajectory required')
    a[:,4]=np.unwrap(a[:,4]);return a


def world_detections(ds,t,ego,base_from_lidar):
    i=int(np.searchsorted(ego[:,0],t))
    if i<len(ego) and abs(ego[i,0]-t)<1e-9:
        pose=ego[i,1:]
    else:
        if i==0 or i==len(ego) or ego[i,0]-ego[i-1,0]>.15:
            raise ValueError('World pose requires bounded recorded interpolation')
        pose=ego[i-1,1:]+(ego[i,1:]-ego[i-1,1:])*(t-ego[i-1,0])/(ego[i,0]-ego[i-1,0])
    c,s=math.cos(pose[3]),math.sin(pose[3]);rot=np.array([[c,-s],[s,c]])
    out=[];sensor_yaw=math.atan2(base_from_lidar[1,0],base_from_lidar[0,0])
    for d in ds:
        local=base_from_lidar@np.array([*d['center'],1.])
        xy=pose[:2]+rot@local[:2]
        out.append(dict(d,center=[*xy,float(pose[2]+local[2])],yaw=d['yaw']+sensor_yaw+pose[3]))
    return out


def core_replay(binary,rows,timestamps,ego,extrinsic,thresholds,maximum_range=None):
    lines=[]
    for row in rows:
        t=timestamps[row['token']]
        ds=[dict(center=b[:3],size=b[3:6],yaw=b[6],score=s,label=int(l),class_name=CLASSES[l])
            for b,l,s in zip(row['pred_boxes'],row['pred_labels'],row['pred_scores'])]
        ds=delivered(ds,thresholds)
        if maximum_range is not None:
            ds=[d for d in ds if np.linalg.norm(d['center'][:2])<maximum_range and -3<=d['center'][2]<=2]
        ds=world_detections(ds,t,ego,extrinsic)
        lines.append(f'{t:.17g} {len(ds)}')
        lines.extend(' '.join(f'{v:.17g}' for v in [d['label'],*d['center'],d['yaw'],*d['size'],d['score']]) for d in ds)
    run=subprocess.run([str(binary.resolve())],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
    frames=[];current=None;remaining=0
    for line in run.stdout.splitlines():
        fields=line.split()
        if not fields:continue
        if fields[0]=='FRAME':
            if remaining:raise ValueError('Incomplete tracker frame')
            current=dict(stamp=float(fields[1]),tracks=[]);frames.append(current);remaining=int(fields[2])
        elif fields[0]=='TRACK':
            if current is None or remaining<=0:raise ValueError('Unexpected tracker output')
            current['tracks'].append(dict(id=int(fields[1]),label=int(fields[2]),confirmed=bool(int(fields[3])),
                associated=bool(int(fields[4])),center=[float(fields[5]),float(fields[6])],
                velocity=[float(fields[7]),float(fields[8])],yaw=float(fields[9])))
            remaining-=1
    if remaining or len(frames)!=len(rows):raise ValueError('Missing tracker frames')
    if [f['stamp'] for f in frames]!=[timestamps[r['token']] for r in rows]:
        raise ValueError('Tracker replay stamps differ')
    return frames


def compare_tracks(reference,candidate,gate=.5,state_rows=None):
    changes=[];deltas=[];identity_changes=0;id_map={};mapping_changes=[]
    if len(reference)!=len(candidate):raise ValueError('Different tracker frame counts')
    for old,new in zip(reference,candidate):
        if old['stamp']!=new['stamp']:raise ValueError('Different tracker times')
        for label in (0,1,2,3,4):
            a=[d for d in old['tracks'] if d['label']==label and d['confirmed']]
            b=[d for d in new['tracks'] if d['label']==label and d['confirmed']]
            pairs,missing,extra=match(a,b,gate=gate)
            if missing or extra:changes.append(dict(stamp=old['stamp'],label=label,removed=len(missing),added=len(extra)))
            for i,j in pairs:
                x,y=a[i],b[j];identity_changes+=x['id']!=y['id']
                if x['id'] in id_map and id_map[x['id']]!=y['id']:
                    mapping_changes.append(dict(stamp=old['stamp'],label=label,reference_id=x['id'],
                        previous_candidate_id=id_map[x['id']],candidate_id=y['id']))
                id_map[x['id']]=y['id']
                delta=x['yaw']-y['yaw']
                deltas.append(dict(stamp=old['stamp'],label=label,reference_id=x['id'],candidate_id=y['id'],
                    position_m=float(np.linalg.norm(np.array(x['center'])-y['center'])),
                    velocity_2d_mps=float(np.linalg.norm(np.array(x['velocity'])-y['velocity'])),
                    speed_mps=abs(float(np.linalg.norm(x['velocity'])-np.linalg.norm(y['velocity']))),
                    yaw_deg=abs(float(np.degrees(np.arctan2(np.sin(delta),np.cos(delta))))),
                    reference_associated=x.get('associated'),candidate_associated=y.get('associated')))
    if state_rows is not None:state_rows.extend(deltas)
    return dict(confirmed_set_changes=changes,matched_id_differences=identity_changes,
        matched_id_note='Numeric IDs across independent runs are not GT ID switches. Mapping changes are diagnostic only.',
        id_mapping_changes=mapping_changes,
        largest_state_changes=sorted(deltas,key=lambda d:-d['position_m'])[:10],
        distributions={k:distribution([d[k] for d in deltas]) for k in
            ('position_m','velocity_2d_mps','speed_mps','yaw_deg')},
        by_tracker_class={str(label):{k:distribution([d[k] for d in deltas if d['label']==label])
            for k in ('position_m','velocity_2d_mps','speed_mps','yaw_deg')} for label in (0,1,2,3,4)})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','binary','sensor-config','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--predictions',type=Path,nargs='+',required=True)
    p.add_argument('--policy',choices=list(POLICIES),default='current')
    p.add_argument('--maximum-range-m',type=float,choices=[30.,100.],
                   help='Explicit radial detection limit before world transformation; absent keeps model ROI.')
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    manifest=json.loads((a.inputs/'manifest.json').read_text())
    infos=pickle.loads((a.inputs/'infos_val.pkl').read_bytes())['data_list']
    timestamps={r['token']:r['timestamp'] for r in infos}
    extrinsic=load_sensor4_calibration(a.sensor_config)['base_from_lidar']
    result=dict(scope='Same continuous dev frames in recorded world coordinates; real EKF core, not full ROS/tracker/prediction/GT accuracy certification.',
        policy=a.policy,maximum_lidar_range_m=a.maximum_range_m,frames=len(infos),models={},binary_sha256=hashlib.sha256(a.binary.read_bytes()).hexdigest(),
        tracker_source_sha256=hashlib.sha256((ROOT/'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp').read_bytes()).hexdigest(),
        config=dict(model='CTRV',position_noise_m=.4,yaw_noise_deg=10.,velocity_fusion=0,association_gate_m=5.))
    reference={};identity=None;names=set()
    payloads=[]
    for path in a.predictions:
        with gzip.open(path,'rt') as f:payload=json.load(f)
        validate_payload(payload)
        if payload['name'] in names:raise ValueError('Distinct replay model names required')
        names.add(payload['name'])
        if payload['frames']!=len(infos) or set(r['token'] for r in payload['rows'])!=set(timestamps):
            raise ValueError('Native continuous frame contract differs')
        if identity is not None and identity!=payload['gt_identity']:raise ValueError('Different GT/scenes')
        identity=payload['gt_identity'];payloads.append(payload)
    for case,details in manifest['cases'].items():
        ego=ego_trajectory(Path(details['raw_bag']))
        for payload in payloads:
            rows=sorted([r for r in payload['rows'] if r['token'].startswith(case+'_')],key=lambda r:timestamps[r['token']])
            frames=core_replay(a.binary,rows,timestamps,ego,extrinsic,POLICIES[a.policy],a.maximum_range_m)
            name=payload['name'];result['models'].setdefault(name,{})
            if case not in reference:
                # Separate invocation proves deterministic replay; comparing a
                # frame list with itself would make the baseline vacuous.
                reference[case]=core_replay(a.binary,rows,timestamps,ego,extrinsic,POLICIES[a.policy],a.maximum_range_m)
            result['models'][name][case]=dict(frames=len(frames),**compare_tracks(reference[case],frames))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2,allow_nan=False))


if __name__=='__main__':main()
