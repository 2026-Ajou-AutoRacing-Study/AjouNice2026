#!/usr/bin/env python3
"""GT-proximity and miss diagnostics after actual ROS threshold replays.

Synchronous detections only; same authored radial30 GT, not 100m evaluation,
authoritative ID-switch or real-time certification. Do not threshold twice.
"""
import argparse
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_ros_tracking import indexed, stamp_key
from calibrate_base5_thresholds import prepare, score_curve
from compare_base5_common30 import CLASSES, scope, validate_payload
from replay_base5_compiled_tracking import ego_trajectory, load_sensor4_calibration
from replay_base5_ros_tracking import pose_at


def local_objects(objects,t,ego,extrinsic):
    pose=pose_at(ego,t);c,s=math.cos(pose[3]),math.sin(pose[3])
    transform=np.eye(4);transform[:2,:2]=[[c,-s],[s,c]];transform[:3,3]=pose[:3]
    transform=transform@extrinsic;inverse=np.linalg.inv(transform)
    sensor_yaw=math.atan2(transform[1,0],transform[0,0])
    boxes=[]
    for obj in objects:
        local=inverse@np.array([*obj['center'],1.])
        boxes.append([*local[:3],*obj['size'],obj['yaw']-sensor_yaw])
    return boxes


def evaluate(args):
    if args.output.exists():raise ValueError('Fresh tracking quality report required')
    infos=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    with gzip.open(args.predictions,'rt') as f:payload=json.load(f)
    validate_payload(payload);predicted={r['token']:r for r in payload['rows']}
    extrinsic=load_sensor4_calibration(args.sensor_config)['base_from_lidar']
    ego={case:ego_trajectory(Path(d['raw_bag'])) for case,d in manifest['cases'].items()}
    native_indices={}
    import rosbag
    for case,contract in manifest['cases'].items():
        times=set()
        with rosbag.Bag(contract['raw_bag']) as bag:
            for _,m,t in bag.read_messages(topics=['/sensor/lidar1']):times.add(m.header.stamp.to_sec() or t.to_sec())
        mapping={t:i for i,t in enumerate(sorted(times))}
        native_indices.update({r['token']:mapping[r['timestamp']] for r in infos if r['scene_id']==case})
    result=dict(scope='Actual ROS tracked/policy outputs on the same development radial30 GT; no second output threshold.',
                gt_identity=payload['gt_identity'],replays={})
    for path in args.replays:
        receipt=json.loads((path/'receipt.json').read_text())
        if receipt['fixed_output_rate_hz']!=0.:raise ValueError('Measurement-time output required for authored GT')
        if receipt['checkpoint_sha256']!=payload['checkpoint_sha256'] or receipt['gt_identity']!=payload['gt_identity']:
            raise ValueError('Weights/GT differ')
        with gzip.open(path/'messages.json.gz','rt') as f:data=json.load(f)
        models={}
        for mode in ('off','on','off_repeat'):
            cases={}
            for c in receipt['cases']:
                case=c['case'];frames=indexed(data[case+'/'+mode+'/policy']);rows=[]
                for src in sorted([r for r in infos if r['scene_id']==case],key=lambda r:r['timestamp']):
                    token=src['token'];t=src['timestamp'];r=predicted[token]
                    expected=np.asarray([x['bbox_3d'] for x in src['instances']],float).reshape(-1,7)
                    expected[:,2]-=expected[:,5]/2
                    if not np.allclose(expected,np.asarray(r['gt_boxes']).reshape(-1,7),atol=1e-5,rtol=0):
                        raise ValueError('GT geometry/track keys differ')
                    f=frames[stamp_key(t)];objs=f['objects']
                    row=scope(dict(r,pred_boxes=local_objects(objs,t,ego[case],extrinsic),
                        pred_labels=[o['label'] for o in objs],pred_scores=[o['score'] for o in objs],
                        pred_track_ids=[o['id'] for o in objs],
                        gt_track_keys=[str(x['track_key']) for x in src['instances']]))
                    row.update(native_index=native_indices[token],stamp=t);rows.append(row)
                ts=np.diff([r['stamp'] for r in rows]);period=float(np.median(ts[(ts>0)&(ts<=.25)]))
                prepared=prepare(rows);classes={}
                for i,name in enumerate(CLASSES):
                    value=score_curve(prepared,i,0.,True,period)
                    previous={};switches=adjacent=0
                    for row in prepared:
                        for p,g in row['assignments'][i][1.].items():
                            key=str(row['gt_track_keys'][g]);identity=int(row['pred_track_ids'][p])
                            current=(row['native_index'],row['stamp'],identity)
                            if key in previous:
                                old=previous[key]
                                if current[0]==old[0]+1 and 0<current[1]-old[1]<=.25:
                                    adjacent+=1;switches+=identity!=old[2]
                            previous[key]=current
                    value['gt_key_track_mapping_changes']=dict(count=switches,adjacent_matched_pairs=adjacent,
                        note='Reconstructed GT keys and nearest matching; not authoritative simulator ID switches.')
                    classes[name]=value
                cases[case]=dict(frames=len(rows),classes=classes)
            models[mode]=dict(thresholds=receipt['candidate_thresholds'] if mode=='on' else receipt['reference_thresholds'],cases=cases)
        result['replays'][path.name]=models
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Saved',args.output)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','predictions','sensor-config','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--replays',type=Path,nargs='+',required=True)
    evaluate(p.parse_args())
