#!/usr/bin/env python3
"""CPU-only, deterministic common-frame pilot from audited mission dev bags.

No ROS master/playback and no new GT catalog guesses. Native compressed JPEG
bytes and XYZI are preserved. GT comes from the existing asset-catalog status
geometry/interpolation contract, not synchronous SaveSensorData annotation.
"""
import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import pickle
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'docker/bevfusion'),
               str(ROOT/'base_ws/src/tracking_prediction_evaluation/src')]
from app.sensor4_calibration import load_sensor4_calibration
from tracking_prediction_evaluation.trajectory import read_morai_trajectories, interpolate_point
from tracking_prediction_evaluation.pipeline_evaluator import read_ego_points, interpolate_ego, message_timestamp
from tracking_prediction_evaluation.bbox_reference import transform_segments_to_asset_catalog_bbox_center
from tracking_prediction_evaluation.unresolved_frame_policy import resolved_frame_mask

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
CASES = ('O02_164012', 'P01_163434', 'H03_162301', 'R01_163034')


def nearest(items, timestamp, tolerance):
    times = [r[0] for r in items]
    right = bisect.bisect_left(times, timestamp)
    indices = range(max(0, right-1), min(len(items), right+1))
    best = min(indices, key=lambda j: abs(times[j]-timestamp), default=None)
    if best is None or abs(times[best]-timestamp) > tolerance:
        return None
    return items[best]


def xyzi(message):
    types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
    fields = {f.name:f for f in message.fields}
    result = np.empty((message.height*message.width, 4), np.float32)
    for index, name in enumerate(('x','y','z','intensity')):
        f = fields[name]
        if f.count != 1:
            raise ValueError('Scalar XYZ/intensity fields required')
        dtype = np.dtype(('>' if message.is_bigendian else '<') + types[f.datatype])
        result[:,index] = np.ndarray((message.height,message.width), dtype=dtype,
            buffer=message.data, offset=f.offset,
            strides=(message.row_step,message.point_step)).reshape(-1)
    return result


def local_box(point, ego, ego_z, sensor_translation):
    dx,dy = point.x-ego.x, point.y-ego.y
    c,s = math.cos(ego.yaw), math.sin(ego.yaw)
    return [c*dx+s*dy-sensor_translation[0], -s*dx+c*dy-sensor_translation[1],
            point.z-ego_z-sensor_translation[2], point.length, point.width, point.height,
            (point.yaw-ego.yaw+math.pi)%(2*math.pi)-math.pi]


def count_support(points, box):
    relative = points[:,:3]-np.asarray(box[:3])
    c,s = math.cos(box[6]), math.sin(box[6])
    x=c*relative[:,0]+s*relative[:,1]; y=-s*relative[:,0]+c*relative[:,1]
    return int(np.count_nonzero((abs(x)<=box[3]/2+.03) & (abs(y)<=box[4]/2+.03)
                               & (abs(relative[:,2])<=box[5]/2+.03)))


def build(args):
    import rosbag
    if args.frames_per_case < 1:
        raise ValueError('Positive per-case frame budget required')
    if args.output.exists():
        raise ValueError('Fresh output required; raw bags and old evaluations are preserved')
    calibration = load_sensor4_calibration(args.sensor,
        ['camera_front','camera_front_left','camera_front_right','camera_back'])
    translation = calibration['base_from_lidar'][:3,3]
    if not np.allclose(calibration['base_from_lidar'][:3,:3], np.eye(3)):
        raise ValueError('This pilot requires the audited zero-rotation LiDAR mounting')
    args.output.mkdir(parents=True)
    rows=[]; cases={}
    for case in CASES:
        source=args.contract_root/case
        contract=json.loads((source/'case_contract.json').read_text())
        names=json.loads((source/'asset_name_map.json').read_text())['asset_names']
        bag=Path(contract['raw_bag']); heading=contract['heading_correction_deg']
        raw=read_morai_trajectories(bag, include_classes=CLASSES,
            heading_correction_deg=heading, asset_name_map=names)
        segments, resolution=transform_segments_to_asset_catalog_bbox_center(raw,args.catalog)
        ego_points=tuple(sorted(read_ego_points(bag,'/Ego_topic',(0,0,0),0,heading), key=lambda x:x.timestamp))
        topics=[c['topic'] for c in calibration['cameras']]
        sensor_messages={t:[] for t in topics+['/sensor/lidar1','/Ego_topic']}
        with rosbag.Bag(str(bag),'r') as handle:
            for topic,message,t in handle.read_messages(topics=list(sensor_messages)):
                sensor_messages[topic].append((message_timestamp(message,t),message))
        for topic,items in sensor_messages.items():
            sensor_messages[topic]=sorted({t:m for t,m in items}.items())
        lidar=sensor_messages['/sensor/lidar1']
        mask,policy=resolved_frame_mask(raw,segments,ego_points,[t for t,_ in lidar])
        eligible=[]; skipped={'unresolved_or_ego':0,'camera_sync':0,'height':0}
        for t,message in lidar:
            if not mask[t]:
                skipped['unresolved_or_ego']+=1;continue
            images=[nearest(sensor_messages[topic],t,.05) for topic in topics]
            if any(im is None for im in images):
                skipped['camera_sync']+=1;continue
            height=interpolate_scalar(sensor_messages['/Ego_topic'],t,.15)
            if height is None:
                skipped['height']+=1;continue
            eligible.append((t,message,images,height))
        if not len(eligible):
            raise ValueError('No supported synchronized frames in '+case)
        indices=np.unique(np.rint(np.linspace(0,len(eligible)-1,min(args.frames_per_case,len(eligible)))).astype(int))
        fingerprints=[]
        for index in indices:
            t,message,images,height=eligible[index]
            ego=interpolate_ego(ego_points,t,.15)
            token=f'{case}_{int(index):05d}'
            folder=args.output/'data'/token; folder.mkdir(parents=True)
            points=xyzi(message); (folder/'points.bin').write_bytes(points.astype('<f4').tobytes())
            finite=points[np.isfinite(points).all(axis=1) & (points[:,:3]!=0).any(axis=1)]
            if len(finite) and not ((finite[:,3]>=0).all() and (finite[:,3]<=255).all()):
                raise ValueError('Raw intensity outside 0..255 contract')
            instances=[]
            for segment in segments:
                point=interpolate_point(segment.points,t,.15)
                if point is None: continue
                box=local_box(point,ego,height,translation)
                # Keep radius 35m GT for the common30 scorer to filter later.
                if np.linalg.norm(box[:2])>35:continue
                instances.append(dict(bbox_3d=box,bbox_label_3d=CLASSES.index(segment.class_name),
                    num_lidar_pts=count_support(finite,box),bbox_3d_isvalid=True,
                    instance_id=len(instances),asset=segment.asset_name,track_key=segment.key))
            image_info={}; offsets={}; payload_hashes={}
            for camera,(image_t,image) in zip(calibration['cameras'],images):
                name=camera['frame_id']; data=bytes(image.data)
                (folder/(name+'.jpg')).write_bytes(data)
                image_info[name]=dict(img_path=str(Path('data')/token/(name+'.jpg')),
                    height=camera['height'],width=camera['width'],
                    cam2img=camera['cam2img'].tolist(),lidar2cam=camera['lidar2cam'].tolist(),
                    lidar2img=camera['lidar2img'].tolist(),cam2lidar=camera['cam2lidar'].tolist())
                offsets[name]=image_t-t; payload_hashes[name]=hashlib.sha256(data).hexdigest()
            point_hash=hashlib.sha256((folder/'points.bin').read_bytes()).hexdigest()
            fingerprints.append(dict(token=token,stamp=t,point_sha256=point_hash,image_sha256=payload_hashes))
            rows.append(dict(token=token,sample_idx=token,scene_id=case,scene_group_id=case,timestamp=t,
                images=image_info,lidar_points=dict(lidar_path=str(Path('data')/token/'points.bin'),num_pts_feats=4),
                instances=instances))
            (folder/'frame.json').write_text(json.dumps(dict(case=case,stamp=t,camera_offsets_sec=offsets,
                ego=dict(x=ego.x,y=ego.y,z=height,yaw=ego.yaw),track_keys=[r['track_key'] for r in instances]),indent=2)+'\n')
        cases[case]=dict(mission=contract['mission'],raw_bag=str(bag),heading_correction_deg=heading,
            catalog_resolution=resolution,frame_resolution=policy,skipped=skipped,
            eligible=len(eligible),selected=len(indices),input_payloads=fingerprints,
            contract_sha256=hashlib.sha256((source/'case_contract.json').read_bytes()).hexdigest())
        print(case, 'selected',len(indices),'eligible',len(eligible),flush=True)
    info=dict(metainfo=dict(classes=CLASSES,dataset='base5_rosbag_quick_dev',box_dim=7),data_list=rows)
    (args.output/'infos_val.pkl').write_bytes(pickle.dumps(info,protocol=4))
    report=dict(frames=len(rows),cases=cases,sensor_sha256=calibration['config_sha256'],
        catalog_sha256=hashlib.sha256(args.catalog.read_bytes()).hexdigest(),
        gt_reference='Object status interpolated geometric BBox catalog center; NOT synchronous native annotation',
        selection='Even index spacing over independently eligible whole-bag LiDAR frames; all models use identical tokens',
        support='Finite nonzero native returns, original center box +3cm; no visibility GT deletion',
        limitation='Development bag pilot; subsampling cannot certify real-time tracking or continuous dropout duration')
    (args.output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TOTAL',len(rows),flush=True)


def interpolate_scalar(items,t,gap):
    right=bisect.bisect_left([x[0] for x in items],t)
    if right<len(items) and abs(items[right][0]-t)<1e-9:
        return float(items[right][1].position.z)
    if right==0 or right==len(items):return None
    lo,a=items[right-1];hi,b=items[right]
    if hi-lo>gap:return None
    return float(a.position.z)+(float(b.position.z)-float(a.position.z))*(t-lo)/(hi-lo)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--frames-per-case',type=int,default=64)
    p.add_argument('--contract-root',type=Path,default=ROOT/'artifacts/evaluation/bevfusion/teammate_manual_261004/e25')
    p.add_argument('--sensor',type=Path,default=ROOT/'configs/sensors/Sensor_4.json')
    p.add_argument('--catalog',type=Path,default=ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json')
    build(p.parse_args())
