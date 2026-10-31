#!/usr/bin/env python3
"""Offline detector-scale ablation. No live ROS publisher, training or defaults.

Bag frames keep original JPEG bytes and exact camera/debug timestamps. A bag's
map projections are gating proposals, never independent visible-head GT.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def digest(path):
    value=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(4*1024**2),b''):value.update(block)
    return value.hexdigest()


def distribution(values):
    values=sorted(values)
    if not values:return {'n':0}
    return dict(n=len(values),mean=statistics.mean(values),p50=statistics.median(values),
                p95=values[math.ceil(.95*len(values))-1],max=values[-1])


def choose_samples(selections, d_approach_sec=40., d_interval=.5, other_interval=2., stopped_interval=5.):
    """Choose exact inference stamps, dense D approach plus sparse long stop."""
    chosen=[];last_by_group={};first_d=None
    for stamp,row in selections:
        group=row.get('group_id')
        if group not in (201351,201365,201290,201326):continue
        if group==201326 and first_d is None:first_d=stamp
        interval=(d_interval if stamp-first_d<=d_approach_sec else stopped_interval) if group==201326 else other_interval
        if stamp-last_by_group.get(group,-float('inf'))+1e-9<interval:continue
        last_by_group[group]=stamp;chosen.append((stamp,row))
    return chosen


def extract(bag_path, output):
    import rosbag
    output=Path(output).resolve();output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():raise FileExistsError('Fresh evidence directory required')
    selections=[];packets={}
    with rosbag.Bag(str(bag_path)) as bag:
        start=bag.get_start_time()
        for topic,msg,stamp in bag.read_messages(topics=['/traffic_light_node/debug_selection','/perception/traffic_light_recognition/map_rois']):
            row=json.loads(msg.data)
            if topic.endswith('debug_selection'):selections.append((stamp.to_sec()-start,row))
            else:packets[row['source_stamp_ns']]=row
        selected=choose_samples(selections)
        by_stamp={row['source_stamp_ns']:(t,row) for t,row in selected}
        output.mkdir(parents=True);(output/'images').mkdir()
        records=[]
        for _,msg,stamp in bag.read_messages(topics=['/sensor/camera/front/compressed']):
            header=msg.header.stamp.to_nsec()
            if header not in by_stamp:continue
            t,selection=by_stamp.pop(header);payload=bytes(msg.data)
            relative='images/frame_%03d_%s.jpg'%(len(records),header)
            (output/relative).write_bytes(payload)
            records.append(dict(image_path=relative,image_sha256=hashlib.sha256(payload).hexdigest(),
                source_stamp_ns=header,record_stamp_ns=stamp.to_nsec(),bag_t=t,group_id=selection['group_id'],
                source_selection=selection,map_packet=packets.get(header),environment='Sunny13_user_verified_trial',
                truth_boxes=None))
    report=dict(schema='traffic_near_scale_original_frames_v1',bag=str(Path(bag_path).resolve()),
        bag_sha256=digest(bag_path),records=records,selected_count=len(selected),missing_camera_stamps=sorted(by_stamp),
        source_sha256=digest(__file__),image_encoding='unchanged_original_JPEG_bytes',
        limitations=['Native light state and independent image-identity GT are absent.',
                     'Map projection and source selection are observed proposals, not visible-head GT.',
                     'Long stationary D frames are correlated, not independent scenes.'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(output),'frames':len(records),'by_group':dict(Counter(r['group_id'] for r in records)),
                      'missing_camera_stamps':len(by_stamp)}))


def predicted_boxes(result):
    return [dict(xyxy=xyxy,confidence=float(conf),class_id=int(cls)) for xyxy,conf,cls in zip(
        result.boxes.xyxy.detach().cpu().tolist(),result.boxes.conf.detach().cpu().tolist(),result.boxes.cls.detach().cpu().tolist())]


def merge_extra(baseline,extra,duplicate_iou=.5):
    from traffic_light_yolo_dataset import iou
    return baseline+[box for box in extra if all(iou(box['xyxy'],old['xyxy'])<duplicate_iou for old in baseline)]


def gate_proposal(row):
    """No GT used: active map head depth, including a bounded-gate rejection."""
    packet=row.get('map_packet') or {};group=row.get('group_id')
    head={201351:'C1256W000077',201365:'C1256W000091',201290:None,201326:'C1256W000052'}.get(group)
    if head is None:return False
    candidates=[r for r in packet.get('rois',[]) if r.get('head_id')==head]
    candidates += [r for r in packet.get('rejected_heads',[]) if r.get('head_id')==head
                   and r.get('reason')=='projection_uncertainty_exceeds_bound']
    return any(3.<=r.get('depth_m',float('inf'))<=15. for r in candidates)


def evaluate(source,output,weights,dataset=False):
    output=Path(output).resolve();output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():raise FileExistsError('Fresh evidence directory required')
    if dataset:
        from traffic_light_yolo_dataset import load_dataset
        rows,meta=load_dataset(source)
        rows=[dict(r,truth_boxes=r['boxes']) for r in rows if r['split'] in ('val','test')]
    else:
        doc=json.loads(Path(source).read_text());meta={k:v for k,v in doc.items() if k!='records'}
        rows=[dict(r,resolved_image_path=str(Path(source).resolve().parent/r['image_path'])) for r in doc['records']]
    import torch,cv2
    from ultralytics import YOLO
    from train_traffic_light_yolo import offline_runtime
    from traffic_light_yolo_dataset import match_detections,detection_summary
    output.mkdir(parents=True)
    timings=defaultdict(list);evaluated=[]
    flags=dict(threads=torch.get_num_threads(),cudnn_benchmark=torch.backends.cudnn.benchmark,
               cudnn_allow_tf32=torch.backends.cudnn.allow_tf32,matmul_allow_tf32=torch.backends.cuda.matmul.allow_tf32)
    with offline_runtime():
        model=YOLO(str(weights))
        first=cv2.imread(rows[0]['resolved_image_path'])
        for size in (960,640):model.predict(first,imgsz=size,conf=.001,device=0,verbose=False)
        with (output/'predictions.jsonl').open('x') as stream:
            for index,row in enumerate(rows):
                path=Path(row['resolved_image_path'])
                if digest(path)!=row['image_sha256']:raise ValueError('Source JPEG changed')
                image=cv2.imread(str(path));predictions={}
                for size in ((960,640) if index%2==0 else (640,960)):
                    torch.cuda.synchronize();started=time.monotonic()
                    result=model.predict(image,imgsz=size,conf=.001,device=0,verbose=False)[0]
                    predictions[str(size)]=predicted_boxes(result)
                    torch.cuda.synchronize();timings[str(size)].append(1000*(time.monotonic()-started))
                item=dict(row,predictions=predictions,near_map_gate=gate_proposal(row));evaluated.append(item)
                stream.write(json.dumps(item,allow_nan=False)+'\n')
                if index%50==0:print('processed %d/%d'%(index,len(rows)),flush=True)
    fixed={size:[] for size in ('960','640','retry640_when960_empty','union_960_640')}
    for row in evaluated:
        a=[p for p in row['predictions']['960'] if p['confidence']>=.25]
        b=[p for p in row['predictions']['640'] if p['confidence']>=.25]
        for name,pred in [('960',a),('640',b),('retry640_when960_empty',a if a else b),('union_960_640',merge_extra(a,b))]:
            if row.get('truth_boxes') is not None:
                fixed[name].append(dict(row,predictions=pred,matching=match_detections(row['truth_boxes'],pred,.5)))
    report=dict(source=str(Path(source).resolve()),source_sha256=digest(source),weights=str(weights),weights_sha256=digest(weights),
        source_metadata=meta,frames=len(rows),conf=.25,low_conf_diagnostics=.001,IoU=.5,torch_flags=flags,
        timings_ms={k:distribution(v) for k,v in timings.items()},
        detection={name:detection_summary(v) for name,v in fixed.items()} if dataset else None,
        by_environment={e:{name:detection_summary([r for r in v if r['environment']==e]) for name,v in fixed.items()}
                        for e in sorted({r['environment'] for r in rows})} if dataset else None,
        by_split={s:{name:detection_summary([r for r in v if r['split']==s]) for name,v in fixed.items()}
                  for s in ('val','test')} if dataset else None,
        source_sha256_tool=digest(__file__),limitations=[
            'Previously inspected validation/test are development regression, not new independent holdout.',
            'Retry-if-empty and union are offline hypotheses, not deployed runtime behavior.',
            'No changed confidence threshold or training; neither detector score nor map projection is identity GT.',
            'Costs are synchronized decoded-BGR inference, not full ROS latency.'])
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(output),'frames':len(rows),'timings_ms':report['timings_ms']}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('extract');p.add_argument('--bag',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('evaluate');p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--dataset',action='store_true')
    args=vars(parser.parse_args());action=args.pop('action')
    if action=='extract':extract(args['bag'],args['output'])
    else:evaluate(**args)


if __name__=='__main__':main()
