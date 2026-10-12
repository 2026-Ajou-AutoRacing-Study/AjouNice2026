#!/usr/bin/env python3
"""Compare actual ROS replay messages, retaining timestamp and match coverage."""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_base5_runtime_effects import CLASSES, distribution, drift, match
from replay_base5_ros_tracking import stamp_key

METRICS=('center_xy_m','size_linf_m','yaw_deg','score_abs','footprint_corner_m',
         'rotated_bev_iou','velocity_2d_mps','speed_mps','yaw_rate_abs')


def indexed(frames):
    result={}
    for frame in frames:
        k=stamp_key(frame['stamp'])
        if k in result:raise ValueError('Repeated output timestamp')
        if frame['frame_id']!='map':raise ValueError('Expected actual tracker map output')
        result[k]=frame
    return result


def compare(reference,candidate,gate):
    rows=[];changes=[];mapping_changes=[];counts={k:defaultdict(int) for k in (*CLASSES,'all')}
    a,b=indexed(reference),indexed(candidate);mapping={}
    common=sorted(set(a)&set(b))
    for stamp in common:
        for label,name in enumerate(CLASSES):
            old=[d for d in a[stamp]['objects'] if d['label']==label]
            new=[d for d in b[stamp]['objects'] if d['label']==label]
            pairs,missing,extra=match(old,new,gate)
            for key in (name,'all'):
                counts[key]['reference_observations']+=len(old)
                counts[key]['candidate_observations']+=len(new)
                counts[key]['removed']+=len(missing);counts[key]['added']+=len(extra)
            if missing or extra:changes.append(dict(stamp=stamp,class_name=name,removed=len(missing),added=len(extra)))
            for i,j in pairs:
                x,y=old[i],new[j]
                key=(name,x['id'])
                if key in mapping and mapping[key]!=y['id']:
                    mapping_changes.append(dict(stamp=stamp,class_name=name,reference_id=x['id'],
                        previous_candidate_id=mapping[key],candidate_id=y['id']))
                mapping[key]=y['id']
                rows.append(dict(stamp=stamp,class_name=name,reference_id=x['id'],candidate_id=y['id'],
                    **drift(x,y),
                    velocity_2d_mps=float(np.linalg.norm(np.asarray(x['velocity'])-y['velocity'])),
                    speed_mps=abs(float(np.linalg.norm(x['velocity'])-np.linalg.norm(y['velocity']))),
                    yaw_rate_abs=abs(x['yaw_rate']-y['yaw_rate']),
                    stationary_flag_flip=x['stationary']!=y['stationary'],
                    orientation_availability_flip=x['orientation_availability']!=y['orientation_availability']))
    by_class={}
    for name in (*CLASSES,'all'):
        selected=[r for r in rows if name=='all' or r['class_name']==name]
        by_class[name]=dict(counts[name],distributions={k:distribution([r[k] for r in selected]) for k in METRICS},
            stationary_flag_flips=sum(r['stationary_flag_flip'] for r in selected),
            orientation_availability_flips=sum(r['orientation_availability_flip'] for r in selected),
            position_tail={str(t):dict(count=sum(r['center_xy_m']>t for r in selected),denominator=len(selected))
                           for t in (.01,.05,.1,.2,.5)})
    return dict(reference_frames=len(a),candidate_frames=len(b),matched_timestamps=len(common),
        reference_only_timestamps=sorted(set(a)-set(b)),candidate_only_timestamps=sorted(set(b)-set(a)),
        frame_changes=changes,id_mapping_changes=mapping_changes,by_class=by_class),rows


def policy_checks(raw,policy):
    a,b=indexed(raw),indexed(policy)
    if set(a)!=set(b):raise ValueError('Motion policy missed or added timestamps')
    unknown_count=0;unknown_raw_moving=0;max_raw_speed=0.
    for stamp in a:
        old={d['id']:d for d in a[stamp]['objects']};new={d['id']:d for d in b[stamp]['objects']}
        if set(old)!=set(new):raise ValueError('Motion policy changed object IDs')
        for identity,x in old.items():
            y=new[identity]
            for key in ('center','size','yaw','score','label','orientation_availability'):
                if x[key]!=y[key]:raise ValueError('Motion policy changed '+key)
            if y['class_name']=='obstacle':
                unknown_count+=1
                if y['velocity']!=[0.,0.] or y['yaw_rate']!=0. or not y['stationary']:
                    raise ValueError('UNKNOWN stationary policy failed')
                speed=float(np.linalg.norm(x['velocity']));max_raw_speed=max(max_raw_speed,speed)
                unknown_raw_moving+=speed>.5
            elif x!=y:raise ValueError('Motion policy modified non-UNKNOWN object')
    return dict(frames=len(a),unknown_observations=unknown_count,
                unknown_raw_speed_above_05_mps=unknown_raw_moving,unknown_raw_max_speed_mps=max_raw_speed,
                pose_shape_identity_preserved=True,unknown_output_motion_zero=True)


def analyze(path,output):
    if output.exists():raise ValueError('Fresh report required')
    receipt=json.loads((path/'receipt.json').read_text())
    with gzip.open(path/'messages.json.gz','rt') as f:data=json.load(f)
    result=dict(scope='Actual isolated ROS tracker/TF/UNKNOWN policy differences, not GT errors or full-stack/GPU certification.',
        receipt=receipt,messages_sha256=hashlib.sha256((path/'messages.json.gz').read_bytes()).hexdigest(),
        matching_note='Same map-frame timestamps; class-preserving XY Hungarian pairing; unmatched counts retained. Wider gate is sensitivity only.',
        policy_checks={},comparisons={})
    all_rows=defaultdict(list)
    comparison_name={'score_threshold':'threshold_candidate','track_lifecycle':'lifecycle_candidate'}.get(
        receipt.get('comparison_kind'),'compile')
    for c in receipt['cases']:
        case=c['case']
        for mode in ('off','on','off_repeat'):
            result['policy_checks'][case+'/'+mode]=policy_checks(data[case+'/'+mode+'/raw'],data[case+'/'+mode+'/policy'])
        for comparison,candidate in ((comparison_name,'on'),('repeat_control','off_repeat')):
            for stage in ('raw','policy'):
                for gate in (.5,2.):
                    key=f'{comparison}/{stage}/{gate}'
                    value,rows=compare(data[case+'/off/'+stage],data[case+'/'+candidate+'/'+stage],gate)
                    result['comparisons'].setdefault(key,{})[case]=value
                    all_rows[key].extend(dict(case=case,**r) for r in rows)
    result['pooled']={}
    for key,rows in all_rows.items():
        result['pooled'][key]={name:dict(
            **{field:sum(v['by_class'][name].get(field,0) for v in result['comparisons'][key].values())
               for field in ('reference_observations','candidate_observations','removed','added',
                             'stationary_flag_flips','orientation_availability_flips')},
            changed_frame_classes=sum(sum(name=='all' or e['class_name']==name for e in v['frame_changes'])
                                      for v in result['comparisons'][key].values()),
            distributions={metric:distribution([r[metric] for r in rows if name=='all' or r['class_name']==name]) for metric in METRICS},
            position_tail={str(t):dict(count=sum(r['center_xy_m']>t for r in rows if name=='all' or r['class_name']==name),
                denominator=sum(name=='all' or r['class_name']==name for r in rows)) for t in (.01,.05,.1,.2,.5)})
            for name in (*CLASSES,'all')}
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with gzip.open(output.with_suffix('.rows.json.gz'),'wt') as f:json.dump(all_rows,f,allow_nan=False)
    print(json.dumps({key:{name:v['distributions']['center_xy_m'] for name,v in block.items()}
                      for key,block in result['pooled'].items()},indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--replay',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();analyze(a.replay,a.output)
