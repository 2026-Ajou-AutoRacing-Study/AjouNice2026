#!/usr/bin/env python3
"""CPU-only score sweeps on frozen dev outputs and native validation scenes.

GT is never extended beyond its authored scope. Precision/recall and temporal
miss exposure diagnose development operating points, not calibrated safety.
"""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_bag_continuity import run_summary
from compare_base5_common30 import CLASSES, scope, validate_payload, gt_identity
from summarize_base5_rosbag_quick import bridge_nms
from summarize_base5_runtime_accuracy import make_row

SCORES=tuple(round(x/100,2) for x in range(10,81))
BASELINE=(.4,.15,.1)
BANDS=((0.,10.),(10.,20.),(20.,30.),(30.,50.),(50.,80.),(80.,100.))


def roi(row):
    out=dict(row)
    for prefix in ('gt','pred'):
        boxes=np.asarray(row[prefix+'_boxes'],float).reshape(-1,7)
        valid=((boxes[:,:3]>=(-100.,-30.,-3.)) & (boxes[:,:3]<=(100.,30.,2.))).all(axis=1)
        for key,value in row.items():
            if key.startswith(prefix+'_'):out[key]=np.asarray(value)[valid]
        out[prefix+'_boxes']=boxes[valid]
    return out


def assignments(row,label,tolerance):
    """Score-sorted greedy matching, same as fixed_matches. Higher scores
    form a prefix, so raising a threshold does not change earlier matches.
    """
    gt=np.flatnonzero(np.asarray(row['gt_labels'])==label)
    pred=np.flatnonzero(np.asarray(row['pred_labels'])==label)
    pred=pred[np.argsort(-np.asarray(row['pred_scores'])[pred],kind='stable')]
    taken=set();pairs={}
    for p in pred:
        available=[int(g) for g in gt if g not in taken]
        if not available:continue
        distances=np.linalg.norm(row['gt_boxes'][available,:2]-row['pred_boxes'][p,:2],axis=1)
        best=int(np.argmin(distances))
        if distances[best]<tolerance:
            g=available[best];taken.add(g);pairs[int(p)]=g
    return pairs


def prepare(rows):
    prepared=[]
    for row in rows:
        r=dict(row)
        for key in ('gt_boxes','pred_boxes'):r[key]=np.asarray(r[key],float).reshape(-1,7)
        for key in ('gt_labels','pred_labels','pred_scores','gt_points'):r[key]=np.asarray(r[key])
        r['assignments']={label:{tol:assignments(r,label,tol) for tol in (1.,2.)} for label in range(3)}
        prepared.append(r)
    return prepared


def ratio(a,b):return a/b if b else None


def score_curve(prepared,label,threshold,temporal=False,period=.1):
    gt=tp=tp2=pred=strong_gt=strong_tp=0
    histories=defaultdict(list);strong_histories=defaultdict(list)
    flips=adjacent=0;previous={}
    bands={f'{lo:g}_{hi:g}m':dict(gt=0,tp=0,pred=0,fp=0) for lo,hi in BANDS}
    fp_types=defaultdict(int)
    for row in prepared:
        gs=np.flatnonzero(row['gt_labels']==label)
        ps=np.flatnonzero((row['pred_labels']==label)&(row['pred_scores']>=threshold))
        paired={p:g for p,g in row['assignments'][label][1.].items() if p in ps}
        paired2={p:g for p,g in row['assignments'][label][2.].items() if p in ps}
        hits=set(paired.values());gt+=len(gs);tp+=len(paired);tp2+=len(paired2);pred+=len(ps)
        for g in gs:
            hit=int(g) in hits;distance=float(np.linalg.norm(row['gt_boxes'][g,:2]))
            for lo,hi in BANDS:
                if lo<=distance<hi:
                    b=bands[f'{lo:g}_{hi:g}m'];b['gt']+=1;b['tp']+=hit
            strong=bool(row['gt_points'][g]>=5)
            if strong:strong_gt+=1;strong_tp+=hit
            if temporal:
                key=str(row['gt_track_keys'][g]);record=(row['native_index'],row['stamp'],hit)
                histories[key].append(record)
                if strong:strong_histories[key].append(record)
                if key in previous:
                    old=previous[key]
                    if record[0]==old[0]+1 and 0<record[1]-old[1]<=.25:
                        adjacent+=1;flips+=hit!=old[2]
                previous[key]=record
        for p in ps:
            distance=float(np.linalg.norm(row['pred_boxes'][p,:2]))
            for lo,hi in BANDS:
                if lo<=distance<hi:
                    b=bands[f'{lo:g}_{hi:g}m'];b['pred']+=1;b['fp']+=p not in paired
            if p in paired:continue
            own=row['gt_boxes'][gs,:2]
            nearest=float(np.linalg.norm(own-row['pred_boxes'][p,:2],axis=1).min()) if len(own) else float('inf')
            others=row['gt_boxes'][row['gt_labels']!=label,:2]
            other_nearest=float(np.linalg.norm(others-row['pred_boxes'][p,:2],axis=1).min()) if len(others) else float('inf')
            kind=('same_class_gt_within_1m' if nearest<1. else 'same_class_gt_1_to_2m' if nearest<2. else
                  'other_class_gt_within_2m' if other_nearest<2. else 'unexplained_within_authored_scope')
            fp_types[kind]+=1
    precision=ratio(tp,pred);recall=ratio(tp,gt)
    return dict(score=threshold,frames=len(prepared),gt=gt,tp=tp,fp=pred-tp,fn=gt-tp,pred=pred,
        precision=precision,recall=recall,fp_per_frame=ratio(pred-tp,len(prepared)),
        f1=ratio(2*tp,2*tp+(pred-tp)+(gt-tp)),f2=ratio(5*tp,5*tp+(pred-tp)+4*(gt-tp)),
        center2m=dict(tp=tp2,precision=ratio(tp2,pred),recall=ratio(tp2,gt)),
        fp_proximity_diagnostics=dict(fp_types),
        distance_counts={k:dict(v,recall=ratio(v['tp'],v['gt'])) for k,v in bands.items()},
        ge5points=dict(gt=strong_gt,tp=strong_tp,recall=ratio(strong_tp,strong_gt)),
        hit_miss_transitions=dict(count=flips,adjacent_pairs=adjacent,fraction=ratio(flips,adjacent)),
        miss_runs=run_summary(histories,period) if temporal else None,
        strong_miss_runs=run_summary(strong_histories,period) if temporal else None)


def sweep(rows,temporal=False):
    prepared=prepare(rows)
    gaps=np.diff([r['stamp'] for r in prepared]) if temporal else np.asarray([])
    usable=gaps[(gaps>0)&(gaps<=.25)]
    period=float(np.median(usable)) if len(usable) else .1
    return dict(frames=len(rows),median_period_sec=period if temporal else None,
        classes={name:[score_curve(prepared,i,t,temporal,period) for t in SCORES]
                 for i,name in enumerate(CLASSES)})


def analyze(args):
    if args.output.exists():raise ValueError('Fresh calibration directory required')
    args.output.mkdir(parents=True)
    infos=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    authored={r['token']:r for r in infos}
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    indices={}
    for case,details in manifest['cases'].items():
        selected=[r for r in infos if r['scene_id']==case]
        if all('raw_lidar_index' in r for r in selected):
            indices.update({r['token']:r['raw_lidar_index'] for r in selected});continue
        import rosbag
        times=set()
        with rosbag.Bag(details['raw_bag']) as bag:
            for _,m,t in bag.read_messages(topics=['/sensor/lidar1']):times.add(m.header.stamp.to_sec() or t.to_sec())
        mapping={t:i for i,t in enumerate(sorted(times))}
        indices.update({r['token']:mapping[r['timestamp']] for r in selected})
    result=dict(scope='Development threshold qualification; no held-out/safety/100m bag certification.',
        scores=list(SCORES),current_scores=dict(zip(CLASSES,BASELINE)),
        postprocess='Same ROS floor0.1/top100/class NMS0.5; bag GT/scoring radial30/common E25 XYZ only.',
        limitations=['Do not label unannotated >30m bag detections as false positives.',
            'Bag GT is interpolated catalog geometry; nearest-GT diagnostics do not prove true/false object identity.',
            'Temporal miss intervals split on original LiDAR index, GT/ROI exclusions and gaps>0.25s.',
            'All GT included; >=5 point strata are diagnostics only. No invisible-GT deletion.',
            'Repeated bag observations are correlated, not independent recall probabilities.',
            'Native validation includes full semantic ROI, but no temporal continuity claims.'],
        bag_models={},native_validation={},source_sha256={},gt_identity=None)
    for path in args.predictions:
        with gzip.open(path,'rt') as f:p=json.load(f)
        validate_payload(p)
        if p['frames']!=len(infos) or set(r['token'] for r in p['rows'])!=set(authored):raise ValueError('Different complete bag frames')
        if result['gt_identity'] not in (None,p['gt_identity']):raise ValueError('Different model GT')
        result['gt_identity']=p['gt_identity'];result['source_sha256'][str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        cases={};outside={}
        for case in manifest['cases']:
            rows=[];raw=[]
            for row in p['rows']:
                src=authored[row['token']]
                if src['scene_id']!=case:continue
                expected=np.asarray([x['bbox_3d'] for x in src['instances']],float).reshape(-1,7)
                expected[:,2]-=expected[:,5]/2
                if not np.allclose(expected,np.asarray(row['gt_boxes']).reshape(-1,7),rtol=0,atol=1e-5):
                    raise ValueError('Authored GT geometry and track-key alignment differ')
                r=bridge_nms(dict(row,gt_track_keys=[str(x['track_key']) for x in src['instances']]))
                raw.append(r)
                r=scope(r);r.update(stamp=src['timestamp'],native_index=indices[row['token']])
                rows.append(r)
            rows.sort(key=lambda r:r['stamp']);cases[case]=sweep(rows,True)
            # Descriptive counts only, explicitly not precision beyond bag GT.
            outside[case]={name:[dict(score=t,count=int(sum(
                score>=t and label==i and np.linalg.norm(box[:2])>=30.
                for r in raw for box,label,score in zip(r['pred_boxes'],r['pred_labels'],r['pred_scores']))))
                for t in SCORES] for i,name in enumerate(CLASSES)}
        result['bag_models'][p['name']]=dict(checkpoint_sha256=p['checkpoint_sha256'],
            inference_precision=p.get('inference_precision'),optimization=p.get('inference_optimization'),
            cases=cases,unscored_outside30_counts=outside)
        print('Swept',p['name'],flush=True)
    if args.native_reports:
        val=pickle.loads((args.native_data/'infos/512x1056/morai_base5_infos_val.pkl').read_bytes())['data_list']
        data={r['token']:r for r in val}
        for path in args.native_reports:
            report=json.loads(path.read_text())
            if not report.get('preserve_learned_config') or report['model_geometry']['classes']!=list(CLASSES):
                raise ValueError('Learned three-class native validation required')
            case=next((i,c) for i,c in enumerate(report['cases'],1) if c['variant']=='parallel_camera')
            export=path.parent/f'{case[0]}_parallel_camera_predictions.json'
            preds=json.loads(export.read_text())
            tokens=[r['id'] for r in case[1]['per_sample']]
            if set(preds)!=set(tokens) or set(tokens)!=set(data):raise ValueError('All 745 native val scenes required; no train mixture')
            native_rows=[make_row(t,data[t],preds[t]) for t in tokens]
            validate_payload(dict(classes=CLASSES,frames=len(native_rows),rows=native_rows,
                                  gt_identity=gt_identity(native_rows)))
            rows=[roi(bridge_nms(r)) for r in native_rows]
            result['native_validation'][path.parent.name]=dict(full_roi=sweep(rows),
                radial30=sweep([scope(r) for r in rows]))
            result['source_sha256'][str(export)]=hashlib.sha256(export.read_bytes()).hexdigest()
            print('Swept native',path.parent.name,flush=True)
    (args.output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,required=True);p.add_argument('--predictions',type=Path,nargs='+',required=True)
    p.add_argument('--native-data',type=Path);p.add_argument('--native-reports',type=Path,nargs='*',default=[])
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.native_reports and not a.native_data:p.error('--native-data required for native reports')
    analyze(a)
