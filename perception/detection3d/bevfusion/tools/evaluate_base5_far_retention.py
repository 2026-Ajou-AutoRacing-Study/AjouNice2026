#!/usr/bin/env python3
"""CPU development bag range metrics and real-ROS coasting trade-offs.

All scoring frames require resolved catalog GT throughout the model ROI.
An unmatched output is not proof of a nonexistent object: status GT omits
background geometry, and catalog centers/interpolation remain approximate.
"""
import argparse
from collections import defaultdict
import gzip
import importlib.util
import json
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_bag_continuity import miss_runs, run_summary
from analyze_base5_ros_tracking import indexed, stamp_key
from analyze_base5_runtime_effects import distribution
from calibrate_base5_thresholds import roi, prepare, score_curve
from compare_base5_common30 import CLASSES, validate_payload
from evaluate_base5_threshold_tracking import local_objects
from prepare_base5_rosbag_quick import ROOT
from replay_base5_compiled_tracking import ego_trajectory, load_sensor4_calibration
from summarize_base5_rosbag_quick import bridge_nms

BANDS=((0.,30.),(30.,50.),(50.,80.),(80.,100.))
POLICIES={'local':(.4,.1,.1),'vehicle_low':(.2,.1,.1)}


def score_policies(values):
    if not values:
        return dict(POLICIES)
    result = {}
    for name, *scores in values:
        scores = tuple(float(v) for v in scores)
        if name in result or len(scores) != 3 or not all(np.isfinite(v) and .1 <= v <= 1. for v in scores):
            raise ValueError('Unique policy names and three finite scores >= decoder floor 0.1 required')
        result[name] = scores
    return result


def band_name(distance):
    for lo,hi in BANDS:
        if lo<=distance<hi:return f'{lo:g}_{hi:g}m'
    return 'roi_corner_ge100m'


def classify_unmatched(box,label,known_boxes,known_labels):
    """Spatial diagnostic, not semantic FP adjudication or ID truth."""
    boxes=np.asarray(known_boxes,float).reshape(-1,7)
    labels=np.asarray(known_labels)
    distances=np.linalg.norm(boxes[:,:2]-np.asarray(box[:2]),axis=1)
    own=distances[labels==label];other=distances[labels!=label]
    own_min=float(own.min()) if len(own) else None
    other_min=float(other.min()) if len(other) else None
    if own_min is not None and own_min<1.:kind='near_same_gt_lt1m'
    elif own_min is not None and own_min<2.:kind='same_gt_1_to_2m'
    elif own_min is not None and own_min<4.:kind='same_gt_2_to_4m'
    elif other_min is not None and other_min<4.:kind='near_other_class_gt_lt4m'
    else:kind='no_catalog_actor_within4m'
    return kind,own_min,other_min


def scalar_metrics(gt,tp,pred):
    return dict(gt=gt,tp=tp,fp=pred-tp,fn=gt-tp,
        precision=tp/pred if pred else None,recall=tp/gt if gt else None)


def range_operating(prepared,label,threshold,period):
    output={}
    for lo,hi in BANDS:
        name=f'{lo:g}_{hi:g}m';gt=pred=strong=0
        matched={tol:0 for tol in (1.,2.,4.)}
        matched_predictions={tol:0 for tol in matched}
        histories=defaultdict(list);positions=[]
        for row in prepared:
            distance_gt=np.linalg.norm(row['gt_boxes'][:,:2],axis=1)
            distance_pred=np.linalg.norm(row['pred_boxes'][:,:2],axis=1)
            gs=set(np.flatnonzero((row['gt_labels']==label)&(distance_gt>=lo)&(distance_gt<hi)))
            ps=set(np.flatnonzero((row['pred_labels']==label)&(row['pred_scores']>=threshold)))
            pred+=sum(lo<=distance_pred[p]<hi for p in ps);gt+=len(gs)
            for tol in matched:
                pairs=row['assignments'][label][tol]
                matched[tol]+=sum(g in gs for p,g in pairs.items() if p in ps)
                matched_predictions[tol]+=sum(lo<=distance_pred[p]<hi for p in pairs if p in ps)
            hits={g for p,g in row['assignments'][label][2.].items() if p in ps}
            for g in gs:
                strong+=row['gt_points'][g]>=5
                histories[str(row['gt_track_keys'][g])].append((row['native_index'],row['stamp'],g in hits))
            for p,g in row['assignments'][label][4.].items():
                if p in ps and g in gs:
                    positions.append(float(np.linalg.norm(row['pred_boxes'][p,:2]-row['gt_boxes'][g,:2])))
        # Matched GT and predictions may be on opposite band boundaries.
        # Do not use pred-band count minus GT-band matches as an FP count.
        output[name]=dict(gt=int(gt),pred_in_band=int(pred),ge5point_gt=int(strong),
            recall_by_center_gate={str(t):matched[t]/gt if gt else None for t in matched},
            matched_gt_counts={str(t):matched[t] for t in matched},
            unmatched_predictions_by_center_gate={str(t):int(pred-matched_predictions[t]) for t in matched},
            precision_by_center_gate={str(t):matched_predictions[t]/pred if pred else None for t in matched},
            center_error_gate4m=distribution(positions),miss_runs_gate2m=run_summary(histories,period))
    return output


def full_prepare(rows):
    result=prepare([roi(r) for r in rows])
    from calibrate_base5_thresholds import assignments
    for row in result:
        for label in range(3):row['assignments'][label][4.]=assignments(row,label,4.)
    return result


def metric_module():
    path=ROOT/'perception/detection3d/common/evaluation/morai_detection.py'
    spec=importlib.util.spec_from_file_location('_base5_far_numpy_metric',path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def ap_by_range(rows):
    m=metric_module();result={}
    for lo,hi in BANDS:
        samples=m._filter_range(rows,(lo,hi))
        metrics=m._evaluate_center(samples,list(CLASSES),m.DEFAULT_CENTER_THRESHOLDS,
            2.,.1,.1,'center',include_tp=False,include_operating_points=False)
        result[f'{lo:g}_{hi:g}m']={c:dict(gt=sum(int(np.count_nonzero(s['gt_labels']==i)) for s in samples),
            ap=metrics['center/'+c+'_mean_AP'] if any(np.any(s['gt_labels']==i) for s in samples) else None)
            for i,c in enumerate(CLASSES)}
    return result


def persistent_unmatched(prepared,label,period):
    kinds=defaultdict(int);histories=defaultdict(list);examples=defaultdict(list)
    no_actor_histories=defaultdict(list);corridor_counts=defaultdict(int)
    matched_history=defaultdict(list)
    for row in prepared:
        matched=set(row['assignments'][label][2.])
        for p,g in row['assignments'][label][2.].items():
            key=f"{row['case']}/{row['pred_track_ids'][p]}"
            matched_history[key].append(dict(stamp=row['stamp'],gt_key=str(row['gt_track_keys'][g])))
        for p in np.flatnonzero(row['pred_labels']==label):
            if p in matched:continue
            kind,own,other=classify_unmatched(row['pred_boxes'][p],label,
                                            row['known_gt_boxes'],row['known_gt_labels'])
            key=f"{row['case']}/{row['pred_track_ids'][p]}"
            kinds[kind]+=1
            histories[key].append((row['native_index'],row['stamp'],False))
            if kind=='no_catalog_actor_within4m':
                no_actor_histories[key].append((row['native_index'],row['stamp'],False))
            if 0<row['pred_boxes'][p,0]<100 and abs(row['pred_boxes'][p,1])<2.:
                corridor_counts[kind]+=1
            examples[key].append(dict(token=row['token'],stamp=row['stamp'],
                native_index=row['native_index'],kind=kind,nearest_same_gt_m=own,
                nearest_other_gt_m=other,box=row['pred_boxes'][p].tolist(),
                score=float(row['pred_scores'][p]),band=band_name(np.linalg.norm(row['pred_boxes'][p,:2])),
                ego_forward_corridor=bool(0<row['pred_boxes'][p,0]<100 and abs(row['pred_boxes'][p,1])<2.)))
    episodes=[]
    for key,records in histories.items():
        for run in miss_runs(records,period):
            samples=[r for r in examples[key] if run['start_stamp']<=r['stamp']<=run['end_stamp']]
            episodes.append(dict(run,track_key=key,kinds=sorted({r['kind'] for r in samples}),
                no_catalog_actor_frames=sum(r['kind']=='no_catalog_actor_within4m' for r in samples),
                forward_corridor_frames=sum(r['ego_forward_corridor'] for r in samples),
                example=samples[len(samples)//2]))
    no_actor_runs=[]
    for key,records in no_actor_histories.items():
        for run in miss_runs(records,period):
            samples=[r for r in examples[key] if run['start_stamp']<=r['stamp']<=run['end_stamp']
                     and r['kind']=='no_catalog_actor_within4m']
            before=[v for v in matched_history[key] if v['stamp']<run['start_stamp']]
            previous=dict(before[-1]) if before else None
            if previous:
                previous['gap_to_run_start_sec']=run['start_stamp']-previous['stamp']
                previous['last_actor_presence_in_package']=prepared[0].get('actor_last_stamps',{}).get(previous['gt_key'])
            no_actor_runs.append(dict(run,track_key=key,example=samples[len(samples)//2],prior_catalog_match=previous))
    durations=[r['seconds'] for r in episodes]
    return dict(unmatched_gate2m=sum(kinds.values()),kinds=dict(kinds),
                episodes=len(episodes),durations=distribution(durations),
                episodes_ge_0p5s=sum(d>=.5 for d in durations),episodes_ge_1s=sum(d>=1 for d in durations),
                forward_corridor_kinds=dict(corridor_counts),
                no_catalog_actor_runs=dict(episodes=len(no_actor_runs),
                    durations=distribution([r['seconds'] for r in no_actor_runs]),
                    worst=sorted(no_actor_runs,key=lambda r:-r['seconds'])[:15]),
                worst=sorted(episodes,key=lambda r:-r['seconds'])[:15])


def analyze(args):
    if args.output.exists():raise ValueError('Fresh output required')
    info=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    with gzip.open(args.predictions,'rt') as f:payload=json.load(f)
    validate_payload(payload)
    predicted={r['token']:r for r in payload['rows']}
    sources={r['token']:r for r in info}
    if set(predicted)!=set(sources):raise ValueError('Frozen sensor tokens differ')
    for token,source in sources.items():
        expected=np.asarray([i['bbox_3d'] for i in source['instances']],float).reshape(-1,7)
        expected[:,2]-=expected[:,5]/2
        actual=np.asarray(predicted[token]['gt_boxes'],float).reshape(-1,7)
        if expected.shape!=actual.shape or not np.allclose(expected,actual,atol=1e-5,rtol=0):
            raise ValueError('Authored GT and reconstructed track-key geometry differ')
        if [i['bbox_label_3d'] for i in source['instances']]!=predicted[token]['gt_labels']:
            raise ValueError('Authored GT classes differ')
    periods={}
    ego={}
    case_rows={}
    for case,details in manifest['cases'].items():
        src=[r for r in info if r['scene_id']==case]
        actor_last_stamps={}
        for s in src:
            for instance in s['instances']:actor_last_stamps[str(instance['track_key'])]=s['timestamp']
        ts=np.diff([r['timestamp'] for r in src]);periods[case]=float(np.median(ts[(ts>0)&(ts<=.25)]))
        ego[case]=ego_trajectory(Path(details['raw_bag']))
        rows=[]
        for source in src:
            if not source['scoring_eligible']:continue
            r=dict(predicted[source['token']],gt_track_keys=[str(i['track_key']) for i in source['instances']],
                native_index=source['raw_lidar_index'],stamp=source['timestamp'],case=case,
                actor_last_stamps=actor_last_stamps)
            rows.append(r)
        case_rows[case]=rows
    prepared={case:full_prepare([bridge_nms(r) for r in rows]) for case,rows in case_rows.items()}
    policies = score_policies(getattr(args, 'score_policies', None))
    result=dict(scope='Saved learned detections; radial bands intersect semantic X±100/Y±30/Zbottom[-3,2].',
        model_name=payload['name'], score_policies=policies,
        matching='AP uses original nuScenes-style 0.5/1/2/4m. P/R diagnostics match ALL ROI GT before range attribution.',
        gt_identity=payload['gt_identity'],source_checkpoint_sha256=payload['checkpoint_sha256'],
        scoring_frames=manifest['scoring_frames'],input_frames=len(info),
        caveats=['Known-scene dev bags, correlated observations, no native annotation or untouched test.',
                 'Never equate no catalog actor with a hallucination: background may be physically real.',
                 'Frames with unresolved actors within ROI radius guard are excluded from accuracy only, not tracker history.',
                 'AP band boundary filtering may create edge effects; operational recall matches globally first.',
                 'Tracker comparison is actual ROS replay, not planning trajectory or steering certification.'],
        detection={},tracking={})
    all_rows=[r for rows in prepared.values() for r in rows]
    result['detection']['ap_by_range']=ap_by_range(all_rows)
    for policy,thresholds in policies.items():
        result['detection'][policy]={}
        for case,rows in prepared.items():
            result['detection'][policy][case]={c:dict(
                overall=score_curve(rows,i,thresholds[i],True,periods[case]),
                by_range=range_operating(rows,i,thresholds[i],periods[case])) for i,c in enumerate(CLASSES)}
    extrinsic=load_sensor4_calibration(args.sensor_config)['base_from_lidar']
    for replay in args.replays:
        receipt=json.loads((replay/'receipt.json').read_text())
        if receipt['fixed_output_rate_hz']!=0 or receipt['maximum_range_m'] is not None:
            raise ValueError('Full-range measurement-time ROS output required')
        if receipt['checkpoint_sha256']!=payload['checkpoint_sha256']:
            raise ValueError('Tracker weights differ')
        extension=payload.get('gt_extension',{})
        if receipt['gt_identity'] not in (payload['gt_identity'],extension.get('original_gt_identity')):
            raise ValueError('Replay must originate from exactly the same saved detections')
        import hashlib
        allowed_hashes={hashlib.sha256(args.predictions.read_bytes()).hexdigest(),
                        extension.get('original_prediction_sha256')}
        if any(value not in allowed_hashes for value in receipt['input_sha256'].values()):
            raise ValueError('Actual ROS replay prediction payload differs')
        with gzip.open(replay/'messages.json.gz','rt') as f:data=json.load(f)
        modes={}
        for mode in ('off','on','off_repeat'):
            cases={}
            for case,rows in case_rows.items():
                frames=indexed(data[case+'/'+mode+'/policy']);replaced=[]
                for r in rows:
                    objects=frames[stamp_key(r['stamp'])]['objects']
                    replaced.append(dict(r,
                        pred_boxes=local_objects(objects,r['stamp'],ego[case],extrinsic),
                        pred_scores=[o['score'] for o in objects],pred_labels=[o['label'] for o in objects],
                        pred_track_ids=[o['id'] for o in objects],
                        known_gt_boxes=r['gt_boxes'],known_gt_labels=r['gt_labels']))
                ps=full_prepare(replaced)
                cases[case]={c:dict(overall=score_curve(ps,i,0.,True,periods[case]),
                    by_range=range_operating(ps,i,0.,periods[case]),
                    unmatched_persistence=persistent_unmatched(ps,i,periods[case])) for i,c in enumerate(CLASSES)}
            modes[mode]=dict(thresholds=receipt['candidate_thresholds'] if mode=='on' else receipt['reference_thresholds'],
                coast_times=receipt.get('candidate_coast_times') if mode=='on' else receipt.get('reference_coast_times'),
                cases=cases)
        result['tracking'][replay.name]=modes
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Saved',args.output,flush=True)
    print(json.dumps(result['detection']['ap_by_range'],indent=2),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','predictions','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--replays',type=Path,nargs='*',default=[])
    p.add_argument('--sensor-config',type=Path,default=ROOT/'configs/sensors/Sensor_4.json')
    p.add_argument('--score-policy', dest='score_policies', action='append', nargs=4,
                   metavar=('NAME','VEHICLE','PEDESTRIAN','OBSTACLE'),
                   help='Explicit detector policy; repeat to compare thresholds. Existing E24 policies remain the default.')
    analyze(p.parse_args())
