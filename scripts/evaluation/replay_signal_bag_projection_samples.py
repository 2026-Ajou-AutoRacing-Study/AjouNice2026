#!/usr/bin/env python3
"""Offline sampled current-candidate audit, NOT reconstruction of the old node.

Stage extract uses recorded TF/localization and untouched front JPEG bytes. Stage
infer uses already installed fixed v5 models. No ROS master/publisher or native
simulator control is used; temporal policy / real historical CNN output is unknown.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT/'base_ws/src/perception/traffic_light_detection'
sys.path.insert(0,str(PACKAGE/'scripts'))
from traffic_light_map_selector import (load_camera,load_geometry,covariance_sigmas,
    RigidTransform,project_heads,DEFAULT_SELECTOR_CONFIG,associate_boxes)

WINDOWS=(('A',30.,76.,201351),('B',79.,88.,201365),('C',100.,110.,201290),('D',128.,141.,201326))
SAMPLE_TIMES=(37.,50.,65.,71.,72.34,73.,74.46,75.,80.75,81.8,82.87,83.96,84.5,
              85.5,86.09,102.89,105.,108.18,133.4,134.54,135.56,136.58,137.59,138.6,139.69,140.5)


def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def distribution(values):
    values=sorted(v for v in values if math.isfinite(v))
    if not values:return dict(n=0)
    return dict(n=len(values),min=values[0],p50=statistics.median(values),
                p95=values[math.ceil(.95*len(values))-1],max=values[-1])


def window(t):
    return next(((k,g) for k,a,b,g in WINDOWS if a<=t<=b),('other',None))


def nearest(rows,times,stamp):
    index=bisect_left(times,stamp)
    return min((rows[i] for i in (index-1,index) if 0<=i<len(rows)),key=lambda row:abs(row[0]-stamp))


def selection_boxes(rows,width,height):
    return [(((a+c)/2)/width,((b+d)/2)/height,(c-a)/width,(d-b)/height,(a,b,c,d,confidence))
            for a,b,c,d,confidence in rows]


def extract(bag_path,output):
    import rosbag
    import rospy
    import tf2_py
    output=Path(output).resolve()
    output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh output required')
    stat=bag_path.stat();buffer=tf2_py.BufferCore(rospy.Duration(1000.))
    poses=[];cameras=[];counts=defaultdict(Counter);values=defaultdict(lambda:defaultdict(list))
    with rosbag.Bag(str(bag_path)) as bag:
        start=bag.get_start_time()
        for topic,msg,t in bag.read_messages(topics=['/tf','/tf_static','/localization/kinematic_state']):
            if topic in ('/tf','/tf_static'):
                for tf in msg.transforms:
                    (buffer.set_transform_static if topic=='/tf_static' else buffer.set_transform)(tf,'recorded_bag')
                continue
            elapsed=t.to_sec()-start;section,gid=window(elapsed);c=msg.pose.covariance
            try:
                x,z,yaw=covariance_sigmas(c,max_yaw_sigma_degrees=180.,max_position_sigma_m=100.,max_vertical_sigma_m=100.)
                outcome='accepted'
                if yaw>math.radians(3.):outcome='yaw_gt3'
                if x>1.5:outcome='xy_gt1.5'
                if z>1.5:outcome='z_gt1.5'
                for key in ('ALL',section):
                    counts[key][outcome]+=1
                    for label,value in (('xy_sigma_m',x),('z_sigma_m',z),('yaw_sigma_deg',math.degrees(yaw))):
                        values[key][label].append(value)
            except ValueError as error:
                counts[section]['invalid:'+str(error)]+=1
            poses.append((msg.header.stamp.to_nsec(),msg,t.to_nsec(),elapsed))
        # Extra 3.0..3.5deg observations probe the hard cap vs per-head pad cap.
        extra=[]
        for section,a,b,gid in WINDOWS:
            candidates=[row for row in poses if a<=row[3]<=b and
                        3.01<=math.degrees(math.sqrt(row[1].pose.covariance[35]))<=3.45]
            if candidates:extra.append(candidates[len(candidates)//2][3])
        targets=sorted(set(SAMPLE_TIMES+tuple(extra)))
        pending=list(targets)
        for _,msg,t in bag.read_messages(topics=['/sensor/camera/front/compressed']):
            elapsed=t.to_sec()-start
            if not pending or elapsed<pending[0]:continue
            requested=pending.pop(0)
            cameras.append((elapsed,msg,t.to_nsec(),requested))
    poses.sort(key=lambda row:row[0]);pose_times=[row[0] for row in poses]
    output.mkdir(parents=True);(output/'images').mkdir()
    camera=load_camera(ROOT/'configs/sensors/Sensor_competition_local.json')
    geometry=load_geometry(PACKAGE/'config/traffic_light_map_geometry.json')
    rows=[]
    for index,(elapsed,msg,record_ns,requested) in enumerate(cameras):
        section,gid=window(elapsed);stamp=msg.header.stamp.to_nsec();pose=nearest(poses,pose_times,stamp)
        row=dict(t=elapsed,requested_t=requested,section=section,group_id=gid,
                 source_stamp_ns=stamp,record_stamp_ns=record_ns,
                 pose_source_stamp_ns=pose[0],pose_image_delta_sec=abs(pose[0]-stamp)/1e9,
                 covariance_diagonal=[pose[1].pose.covariance[i] for i in (0,7,14,35)],
                 modes={},ground_truth_available=False)
        relative='images/frame_%03d_%.3f_%s.jpg'%(index,elapsed,section)
        payload=bytes(msg.data);(output/relative).write_bytes(payload)
        row.update(image_path=relative,image_sha256=hashlib.sha256(payload).hexdigest())
        try:
            tf=buffer.lookup_transform_core(camera.frame,geometry['geometry_frame'],msg.header.stamp)
            t,q=tf.transform.translation,tf.transform.rotation
            transform=RigidTransform.from_quaternion((t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
            row['recorded_tf_available']=True
            for label,cap in (('current3',3.),('candidate5',5.)):
                try:
                    xy,z,yaw=covariance_sigmas(pose[1].pose.covariance,max_yaw_sigma_degrees=cap)
                    row['yaw_sigma_deg']=math.degrees(yaw)
                    rois,rejected=project_heads(geometry,camera,transform,gid,xy,yaw,
                        vertical_sigma_m=z,**DEFAULT_SELECTOR_CONFIG.projection_kwargs())
                    row['modes'][label]=dict(status='ready',rois=rois,rejected_heads=rejected,
                        eligible_route_heads=[r['head_id'] for r in rois if r['eligible']])
                except ValueError as error:
                    row['modes'][label]=dict(status='missing',reason=str(error),rois=[],eligible_route_heads=[])
        except Exception as error:
            row.update(recorded_tf_available=False,tf_error=type(error).__name__+':'+str(error))
        rows.append(row)
    current=bag_path.stat()
    if (current.st_size,current.st_mtime_ns)!=(stat.st_size,stat.st_mtime_ns):raise RuntimeError('Bag changed')
    report=dict(schema='sampled_current_signal_covariance_projection_v1',bag=str(bag_path),
        bag_identity=dict(size=stat.st_size,mtime_ns=stat.st_mtime_ns),
        selection='fixed rosout-informed development times plus one 3.01..3.45deg pose per mission',
        covariance_counts={k:dict(v) for k,v in counts.items()},
        covariance_distributions={k:{key:distribution(v) for key,v in d.items()} for k,d in values.items()},
        sources={str(p):digest(p) for p in (Path(__file__),PACKAGE/'scripts/traffic_light_map_selector.py',
            PACKAGE/'config/traffic_light_map_geometry.json',ROOT/'configs/sensors/Sensor_competition_local.json')},
        records=rows,limitations=['Current geometry/calibration/model candidate audit, not old runtime reconstruction.',
        'No traffic output/debug/native phase GT recorded; no causal brake or accuracy claim.',
        'Window labels are manually selected route intervals, not independent head identity GT.',
        'TF projection component replay does not emulate map ROI callbacks/receipt queue timing.'])
    (output/'projection.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),frames=len(rows),covariance_counts=report['covariance_counts'])))


def infer(projection,output):
    import cv2
    import torch
    from ultralytics import YOLO
    from train_traffic_light_yolo import offline_runtime
    from traffic_light_cnn_classifier import TrafficLightCnnClassifier
    from evaluate_traffic_light_near_development_matrix import runtime_crop
    report=json.loads(projection.read_text());output=Path(output).resolve()
    output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh inference output required')
    models=Path.home()/'.local/share/ajounice/traffic_light_models/v5_3d29ee7f'
    detector=models/'yolo_final/selected.pt';cnn=models/'cnn_v5/best.pt'
    device='cuda:0' if torch.cuda.is_available() else 'cpu';cv2.setNumThreads(1);torch.set_num_threads(4)
    rows=[]
    with offline_runtime():
        model=YOLO(str(detector));classifier=TrafficLightCnnClassifier(cnn,device=device,
            min_bit_confidence=.6,expected_sha256=digest(cnn))
        for row in report['records']:
            p=projection.parent/row['image_path']
            if digest(p)!=row['image_sha256']:raise ValueError('JPEG changed')
            image=cv2.imread(str(p));predictions={}
            for size in (960,640):
                result=model.predict(image,imgsz=size,conf=.25,device=0 if device!='cpu' else 'cpu',verbose=False)[0]
                boxes=[tuple(map(float,b))+ (float(score),) for b,score in zip(
                    result.boxes.xyxy.detach().cpu().tolist(),result.boxes.conf.detach().cpu().tolist())]
                predictions[str(size)]=boxes
            modes={}
            for label,projection_mode in row['modes'].items():
                mode=dict(projection_status=projection_mode['status'],projection_reason=projection_mode.get('reason'))
                if projection_mode['status']=='ready':
                    boxes=selection_boxes(predictions['960'],image.shape[1],image.shape[0])
                    selected,selection=associate_boxes(boxes,projection_mode['rois'],row['group_id'])
                    mode.update(selection=selection)
                    if selected is not None:
                        crop,bounds=runtime_crop(image,selected[4][:4]);obs=classifier.classify_bgr(crop)
                        mode.update(crop_xyxy=bounds,cnn=obs.to_dict())
                    else:
                        # Diagnostic combined-960/640 association only, not a
                        # claim the production near-rescue trigger was met.
                        retry,retry_selection=associate_boxes(boxes+selection_boxes(predictions['640'],image.shape[1],image.shape[0]),
                            projection_mode['rois'],row['group_id'])
                        mode.update(diagnostic_combined_selection=retry_selection)
                        if retry is not None:
                            crop,bounds=runtime_crop(image,retry[4][:4]);obs=classifier.classify_bgr(crop)
                            mode.update(diagnostic_crop_xyxy=bounds,diagnostic_cnn=obs.to_dict())
                modes[label]=mode
            rows.append(dict(t=row['t'],section=row['section'],image_path=row['image_path'],
                             image_sha256=row['image_sha256'],predictions=predictions,modes=modes))
    output.mkdir(parents=True)
    result=dict(schema='sampled_current_yolo_cnn_projection_counterfactual_v1',
        source_projection=str(projection),source_projection_sha256=digest(projection),device=device,
        models={str(p):digest(p) for p in (detector,cnn)},records=rows,
        limitations=report['limitations']+['Combined640 rescue is diagnostic, not exact live pipeline replay.',
        'No native phase/head GT, scores cannot be called accuracy or old-node outputs.'])
    (output/'inference.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),frames=len(rows),device=device)))


def adjudicate(inference,output):
    """Apply actual near-rescue trigger/accept rules to saved predictions only."""
    from traffic_light_near_rescue import near_rescue_decision,accept_near_rescue_selection,NearRescueConfig
    source=json.loads(inference.read_text())
    projection_path=Path(source['source_projection']);projection=json.loads(projection_path.read_text())
    by_stamp={row['image_sha256']:row for row in projection['records']}
    rows=[];counts=Counter()
    for row in source['records']:
        geometry=by_stamp[row['image_sha256']];outcomes={}
        for mode,item in row['modes'].items():
            outcome=dict(primary_matched=item.get('selection',{}).get('status')=='matched',
                         retry=False,recovered=False)
            if item['projection_status']=='ready':
                decision=near_rescue_decision(item['selection'],geometry['modes'][mode]['rois'],geometry['group_id'],
                                             config=NearRescueConfig(enabled=True))
                candidate=item.get('diagnostic_combined_selection',{})
                accepted=bool(decision['retry'] and accept_near_rescue_selection(candidate,decision))
                outcome.update(decision=decision,retry=decision['retry'],recovered=accepted,
                    accepted_head=(candidate.get('head_id') if accepted else item.get('selection',{}).get('head_id')),
                    adopted_cnn=(item.get('diagnostic_cnn') if accepted else item.get('cnn')))
            outcomes[mode]=outcome
            for key in ('primary_matched','retry','recovered'):counts[mode+':'+key]+=int(outcome[key])
        rows.append(dict(t=row['t'],section=row['section'],image_path=row['image_path'],outcomes=outcomes))
    output=Path(output).resolve();output.relative_to(ROOT/'planner_logs/analysis')
    if output.exists():raise FileExistsError('Fresh adjudication output required')
    output.mkdir(parents=True)
    report=dict(schema='saved_current_near_rescue_rules_v1',source_inference=str(inference),
        source_inference_sha256=digest(inference),counts=dict(counts),records=rows,
        limitations=source['limitations']+['Actual pure trigger and acceptance rules applied to saved predictions.',
            'Does not recreate callback timing/route changes during retry or previous-head temporal continuity.',
            'Manually supplied mission windows remain a component-analysis assumption.'])
    (output/'gated_rescue.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),counts=report['counts'])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['extract','infer','adjudicate'])
    p.add_argument('--bag',type=Path);p.add_argument('--projection',type=Path);p.add_argument('--inference',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.stage=='extract':extract(a.bag.resolve(strict=True),a.output)
    elif a.stage=='infer':infer(a.projection.resolve(strict=True),a.output)
    else:adjudicate(a.inference.resolve(strict=True),a.output)
