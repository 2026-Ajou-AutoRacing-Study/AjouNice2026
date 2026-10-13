#!/usr/bin/env python3
"""Read-only pack of reserved recorded track states and audited catalog GT."""
import argparse
from bisect import bisect_left
import gzip
import json
import math
from pathlib import Path
import sys

import yaml

from experiment_base5_map_cv_candidates import ROOT, digest
from probe_base5_map_lane_history import lanelet_coordinates, future_point


def nearest(items,stamp,maximum_age=.15):
    times=[r[0] for r in items];i=bisect_left(times,stamp)
    choices=items[max(0,i-1):i+1]
    if not choices:return None
    best=min(choices,key=lambda r:abs(r[0]-stamp))
    return best[1] if abs(best[0]-stamp)<=maximum_age else None


def read_frames(path,current_transform,message_stride=3):
    import rosbag
    frames=[];ego=[];alignment=None
    with rosbag.Bag(str(path),'r') as bag:
        for _,msg,_ in bag.read_messages(topics=['/tf_static']):
            for t in msg.transforms:
                if t.header.frame_id.lstrip('/')=='map' and t.child_frame_id.lstrip('/')=='morai_map':
                    q=t.transform.rotation
                    if abs(q.x)>1.e-8 or abs(q.y)>1.e-8:raise ValueError('Require planar map TF')
                    value=dict(parent='map',child='morai_map',
                               xyz=[getattr(t.transform.translation,k) for k in 'xyz'],
                               rpy=[0.,0.,2*math.atan2(q.z,q.w)])
                    if alignment and value!=alignment:raise ValueError('Recorded map alignment changed')
                    alignment=value
        if alignment is None:raise ValueError('No recorded map/morai_map TF: cannot guess origin')
        a=alignment['rpy'][2];c,s=math.cos(a),math.sin(a);tx,ty,tz=alignment['xyz']
        inverse=dict(parent='map',child='morai_map',rpy=[0.,0.,-a],
                     xyz=[-c*tx-s*ty,s*tx-c*ty,-tz])
        topics=['/perception/object_recognition/tracking/objects','/Ego_topic']
        index=0
        for topic,msg,received in bag.read_messages(topics=topics):
            stamp=msg.header.stamp.to_sec() or received.to_sec()
            if topic=='/Ego_topic':
                ego.append((stamp,(msg.position.x,msg.position.y)))
                continue
            # Fixed message-index decimation, independent of score, motion and GT.
            take=index%message_stride==0;index+=1
            if not take:continue
            if msg.header.frame_id.lstrip('/')!='map':raise ValueError('Recorded tracking must be map frame')
            objects=[]
            for o in msg.objects:
                label=max(o.classification,key=lambda c:c.probability).label if o.classification else 0
                if label not in (1,2,3,4):continue
                pose=o.kinematics.pose_with_covariance.pose;q=pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                local,local_yaw=lanelet_coordinates([pose.position.x,pose.position.y,pose.position.z],yaw,inverse)
                point,yaw=lanelet_coordinates(local,local_yaw,current_transform)
                twist=o.kinematics.twist_with_covariance.twist
                vx,vy=twist.linear.x,twist.linear.y
                objects.append(dict(id=bytes(o.object_id.uuid).hex(),class_name='vehicle',center=point,
                    yaw=yaw,body_velocity=[vx,vy],velocity=[math.cos(yaw)*vx-math.sin(yaw)*vy,
                    math.sin(yaw)*vx+math.cos(yaw)*vy],score=float(o.existence_probability)))
            frames.append(dict(stamp=stamp,frame_id='map',objects=objects))
    return frames,sorted(ego),alignment


def prepare_case(path,case,current_transform,catalog,message_stride=3):
    sys.path.insert(0,str(ROOT/'base_ws/src/tracking_prediction_evaluation/src'))
    from tracking_prediction_evaluation.trajectory import read_morai_trajectories
    from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
    frames,ego,recorded_alignment=read_frames(path,current_transform,message_stride)
    raw=read_morai_trajectories(path,include_classes=('vehicle',),minimum_points=1,
                               heading_correction_deg=-1.347)
    corrected,resolution=transform_segments_to_asset_catalog_bbox_center(raw,catalog)
    trajectories={}
    for seg in corrected:
        trajectories[(case,seg.key)]=[(p.timestamp,lanelet_coordinates([p.x,p.y,p.z],p.yaw,current_transform)[0]) for p in seg.points]
    supported={s.key for s in corrected}
    unresolved=[[(p.timestamp,(p.x,p.y)) for p in s.points] for s in raw if s.key not in supported]
    by_frame={};excluded=0
    for frame in frames:
        stamp=frame['stamp'];e=nearest(ego,stamp)
        if e is None:excluded+=1;continue
        if any(p is not None and math.hypot(p[0]-e[0],p[1]-e[1])<=105.
               for p in (future_point(series,stamp) for series in unresolved)):
            excluded+=1;continue
        obs=[]
        for key,series in trajectories.items():
            point=future_point(series,stamp)
            if point is not None:obs.append(dict(center=point,key=key))
        by_frame[(case,round(stamp,6))]=obs
    return dict(frames=frames,gt_frames=[dict(stamp=k[1],objects=v) for k,v in by_frame.items()],
                gt_trajectories=[dict(key=list(k),series=v) for k,v in trajectories.items()],
                provenance=dict(path=str(path.resolve()),bytes=path.stat().st_size,
                    recorded_alignment=recorded_alignment,current_alignment=current_transform,
                    model='not independently verified',heading_correction_deg=-1.347,
                    heading_contract='MR convention used for catalog pivot rotation; not absolute heading validation',
                    fixed_message_stride=message_stride,excluded_scoring_frames=excluded,
                    gt_resolution=resolution,
                    caveat='Catalog/position-interpolated GT, reconstructed actor keys. '
                           'No sensor rerun. Recorded track history may include coasting. '
                           'Unresolved actors within105m of recorded Ego exclude the whole scoring frame.'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--development',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--message-stride',type=int,default=3)
    parser.add_argument('--case-index',type=int,choices=(1,2,3,4))
    args=parser.parse_args()
    if args.message_stride<1:raise ValueError('Positive stride required')
    if args.output.exists():raise FileExistsError('Fresh pack required')
    receipt=json.loads((args.development/'preregistered.json').read_text())
    transform=yaml.safe_load((ROOT/'configs/tf/morai_lanelet_static_tf.yaml').read_text())['map_to_morai_map']
    catalog=ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'
    cases={}
    for index,name in enumerate(receipt['reserved_bags']):
        if args.case_index is not None and index+1!=args.case_index:continue
        case='reserved_'+str(index+1)
        cases[case]=prepare_case(ROOT/name,case,transform,catalog,args.message_stride)
        p=cases[case]['provenance'];print(case,name,'frames',len(cases[case]['frames']),
              'GT segments',p['gt_resolution']['supported_segment_count'],
              'unresolved',p['gt_resolution']['unresolved_segment_count'],flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(args.output,'wt') as handle:
        json.dump(dict(cases=cases,source_sha256=digest(__file__),catalog_sha256=digest(catalog),
                       preregistration_sha256=digest(args.development/'preregistered.json')),
                  handle,allow_nan=False)


if __name__=='__main__':main()
