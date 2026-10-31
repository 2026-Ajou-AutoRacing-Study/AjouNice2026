#!/usr/bin/env python3
"""Catalog-relative tracking center AP and temporary zero-return diagnostics.

Scores are retained detector confidences, not calibrated track confidence.
This is not nuScenes AMOTA, IDF1, native-annotation AP or closed-loop safety.
"""
import argparse
from bisect import bisect_left
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np

from prepare_e12_postprocessing import ROOT, TOOLS, sha, transform_for_pose
sys.path.insert(0, str(TOOLS))
from calibrate_base5_thresholds import assignments
from evaluate_base5_far_retention import full_prepare, metric_module
from summarize_base5_frozen_bag_comparison import ap_summary
from app.sensor4_calibration import load_sensor4_calibration

CLASSES = ('vehicle', 'pedestrian', 'obstacle')
BANDS = ((0,30),(30,50),(50,80),(80,90),(90,100))


def distribution(values):
    if not values:
        return dict(n=0, mean=None, p50=None, p90=None, p95=None, maximum=None)
    a = np.asarray(values, float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite distribution')
    return dict(n=len(a), mean=float(a.mean()), p50=float(np.median(a)),
                p90=float(np.percentile(a,90)),p95=float(np.percentile(a,95)),maximum=float(a.max()))


def adjacent(a,b):
    return b['native_index'] == a['native_index']+1 and 0 < b['stamp']-a['stamp'] <= .25


def velocity_at(series, stamp, max_gap=.25):
    times = [r[0] for r in series]
    i = bisect_left(times, stamp)
    if i == len(times) or abs(times[i]-stamp) > 1e-7:
        return None
    def valid_step(a,b):
        index_valid=(len(series[a])<3 or len(series[b])<3 or series[b][2]==series[a][2]+1)
        return 0<times[b]-times[a]<=max_gap and index_valid
    low = i-1 if i and valid_step(i-1,i) else i
    high = i+1 if i+1<len(times) and valid_step(i,i+1) else i
    if low == high:
        return None
    return (np.asarray(series[high][1][:2])-np.asarray(series[low][1][:2]))/(times[high]-times[low])


def local_box(obj, frame, extrinsic):
    pose = [frame['ego'][k] for k in ('x','y','z','yaw')]
    inverse = np.linalg.inv(transform_for_pose(pose, extrinsic))
    p = inverse @ [*obj['center'],1.]
    p[2] -= obj['size'][2]/2
    yaw = obj['yaw']-pose[3]-math.atan2(extrinsic[1,0],extrinsic[0,0])
    return [*p[:3],*obj['size'],yaw]


def metric_row(frame, mode, extrinsic):
    objects = (frame['detections'] if mode == 'detector' else
               [r for r in frame['tracks'] if r['confirmed']])
    gt = frame['gt']
    boxes = [list(r['local_box']) for r in gt]
    for b in boxes:
        b[2] -= b[5]/2
    return dict(token=frame['token'],stamp=frame['stamp'],native_index=frame['native_index'],
        gt_boxes=boxes,gt_labels=[r['label'] for r in gt],gt_points=[r['lidar_points']for r in gt],
        gt_track_keys=[r['key']for r in gt],gt_assets=[r.get('asset')for r in gt],
        pred_boxes=[local_box(r,frame,extrinsic)for r in objects],
        pred_labels=[r['label']for r in objects],pred_scores=[r['score']for r in objects],
        pred_track_ids=[r.get('id',-1)for r in objects],pred_objects=objects)


def summary_counts(rows):
    output = {}
    for label,name in enumerate(CLASSES):
        gt = sum(np.count_nonzero(r['gt_labels']==label)for r in rows)
        pred = sum(np.count_nonzero(r['pred_labels']==label)for r in rows)
        hits = {str(gate):sum(len(r['assignments'][label][gate])for r in rows)for gate in (1.,2.,4.)}
        output[name] = dict(gt=int(gt),pred=int(pred),matched=hits,
            recall={k:n/gt if gt else None for k,n in hits.items()},
            catalog_precision={k:n/pred if pred else None for k,n in hits.items()})
    return output


def compare_case(case, extrinsic):
    frames = case['frames']
    eligible = [f for f in frames if f['scoring_eligible']]
    modes = {m:full_prepare([metric_row(f,m,extrinsic)for f in eligible])for m in ('detector','tracker')}
    # full_prepare filters all pred_* fields by semantic ROI. Keep the object
    # records aligned, rather than indexing the original unfiltered list.
    by_token = {m:{r['token']:r for r in rows}for m,rows in modes.items()}
    # Eligibility is a GT-quality contract, not merely a current-origin mask.
    # Never use an excluded neighbor to construct a reference velocity.
    trajectories=defaultdict(list)
    for f in eligible:
        for gt in f['gt']:
            trajectories[(gt['key'],gt['label'])].append([f['stamp'],gt['center'],f['native_index']])
    observations, velocities, unmatched, identity = [],[],[],[]
    previous, support_history = {},{}
    for frame in eligible:
        det,trk = (by_token[m][frame['token']]for m in ('detector','tracker'))
        truth = {r['key']:r for r in frame['gt']}
        paired_tracks = {}
        for label in range(3):
            for p,g in trk['assignments'][label][2.].items():
                paired_tracks[str(trk['gt_track_keys'][g])] = trk['pred_objects'][p]
        for label in range(3):
            det_hits = set(det['assignments'][label][2.].values())
            trk_hits = {g:p for p,g in trk['assignments'][label][2.].items()}
            for g in np.flatnonzero(trk['gt_labels']==label):
                key = str(trk['gt_track_keys'][g]);gt = truth[key]
                last = support_history.get((label,key))
                continuous = last is not None and adjacent(last,frame)
                seen_positive = continuous and last['seen_positive']
                acquired = continuous and last['ever_track_match']
                points = int(trk['gt_points'][g])
                current_track = trk['pred_objects'][trk_hits[g]]if g in trk_hits else None
                distance = float(np.linalg.norm(trk['gt_boxes'][g,:2]))
                band = next((f'{lo}_{hi}m'for lo,hi in BANDS if lo<=distance<hi),'outside_radial100')
                observed = dict(stamp=frame['stamp'],native_index=frame['native_index'],token=frame['token'],
                    key=key,class_name=CLASSES[label],label=label,band=band,distance_m=distance,
                    direction='front'if gt['local_box'][0]>=0 else 'rear',
                    points=points,detected=g in det_hits,tracked=g in trk_hits,
                    previous_positive_in_continuous_segment=bool(seen_positive),
                    previous_track_acquisition_in_continuous_segment=bool(acquired),
                    associated=current_track['associated']if current_track else None,
                    track_id=current_track['id']if current_track else None,
                    last_measurement_age_sec=frame['stamp']-current_track['last_measurement_stamp']if current_track else None,
                    center_error_m=float(np.linalg.norm(trk['pred_boxes'][trk_hits[g],:2]-trk['gt_boxes'][g,:2]))if current_track else None)
                observations.append(observed)
                support_history[(label,key)] = dict(stamp=frame['stamp'],native_index=frame['native_index'],
                    seen_positive=bool(seen_positive or points>0),ever_track_match=bool(acquired or current_track))
                old = previous.get((label,key))
                if old and current_track and old['track_id']is not None and adjacent(old,frame):
                    identity.append(dict(class_name=CLASSES[label],band=band,
                        switched=old['track_id']!=current_track['id'],stamp=frame['stamp'],key=key))
                previous[(label,key)] = observed
                if current_track:
                    velocity = velocity_at(trajectories[(key,label)],frame['stamp'])
                    if velocity is not None:
                        effective = [0.,0.]if label==2 else current_track['velocity']
                        velocities.append(dict(class_name=CLASSES[label],band=band,points=points,
                            stamp=frame['stamp'],key=key,
                            associated=current_track['associated'],
                            track_age_sec=frame['stamp']-current_track['birth_stamp'],
                            vector_error_mps=float(np.linalg.norm(velocity-effective)),
                            speed_abs_error_mps=abs(float(np.linalg.norm(velocity)-np.linalg.norm(effective))),
                            raw_core_vector_error_mps=float(np.linalg.norm(velocity-current_track['velocity']))))
            matched = set(trk['assignments'][label][2.])
            for p in np.flatnonzero(trk['pred_labels']==label):
                if p not in matched:
                    obj = trk['pred_objects'][p]
                    unmatched.append(dict(class_name=CLASSES[label],associated=obj['associated'],
                        age=frame['stamp']-obj['last_measurement_stamp'],stamp=frame['stamp'],id=obj['id']))
    metrics = metric_module()
    result = dict(all_frames=len(frames),scoring_frames=len(eligible),stages={},
        observations=len(observations),diagnostic_gt_id_pairs=len(identity),
        catalog_gt_mapping_changes=sum(r['switched']for r in identity))
    for mode,rows in modes.items():
        result['stages'][mode] = dict(ap=ap_summary(rows,metrics),operating=summary_counts(rows),by_range={})
        for lo,hi in BANDS:
            filtered = metrics._filter_range(rows,(lo,hi))
            result['stages'][mode]['by_range'][f'{lo}_{hi}m'] = dict(ap=ap_summary(filtered,metrics))
    result['support_groups'] = support_groups(observations)
    result['direction_support_groups'] = {direction:support_groups([r for r in observations if r['direction']==direction])
        for direction in ('front','rear')}
    result['velocity'] = motion_groups(velocities)
    result['unmatched_catalog_tracks'] = {name:dict(observations=sum(r['class_name']==name for r in unmatched),
        coasting=sum(r['class_name']==name and not r['associated']for r in unmatched),
        age_sec=distribution([r['age']for r in unmatched if r['class_name']==name]))for name in CLASSES}
    return result,dict(support=observations,velocity=velocities,identity=identity,unmatched=unmatched),modes


def support_groups(observations):
    groups = {}
    for name in CLASSES:
        for band in ['all',*[f'{a}_{b}m'for a,b in BANDS]]:
            base = [r for r in observations if r['class_name']==name and (band=='all'or r['band']==band)]
            for support in ('all','positive','zero','zero_after_positive','zero_after_track_acquisition'):
                selected = [r for r in base if support=='all'or
                    (support=='positive'and r['points']>0)or
                    (support=='zero'and r['points']==0)or
                    (support=='zero_after_positive'and r['points']==0 and r['previous_positive_in_continuous_segment'])or
                    (support=='zero_after_track_acquisition'and r['points']==0 and r['previous_track_acquisition_in_continuous_segment'])]
                n = len(selected)
                if not n:
                    continue
                groups[name+'/'+band+'/'+support] = dict(gt=n,
                    detector_hits=sum(r['detected']for r in selected),tracker_hits=sum(r['tracked']for r in selected),
                    detector_recall=sum(r['detected']for r in selected)/n,tracker_recall=sum(r['tracked']for r in selected)/n,
                    recovered_without_detection=sum(r['tracked']and not r['detected']for r in selected),
                    detection_hit_lost_by_tracker=sum(r['detected']and not r['tracked']for r in selected),
                    coasting_hits=sum(r['tracked']and not r['associated']for r in selected),
                    tracked_xy_error_m=distribution([r['center_error_m']for r in selected if r['tracked']]),
                    tracked_measurement_age_sec=distribution([r['last_measurement_age_sec']for r in selected if r['tracked']]))
    return groups


def motion_groups(rows):
    groups = {}
    for name in CLASSES:
        for band in ['all',*[f'{a}_{b}m'for a,b in BANDS]]:
            chosen = [r for r in rows if r['class_name']==name and (band=='all'or r['band']==band)]
            if chosen:
                groups[name+'/'+band] = {k:distribution([r[k]for r in chosen])for k in
                    ('vector_error_mps','speed_abs_error_mps','raw_core_vector_error_mps')}
    return groups


def analyze(args):
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    with gzip.open(args.pack,'rt')as handle:
        pack = json.load(handle)
    sensor = ROOT/'configs/sensors/Sensor_competition_lan.json'
    extrinsic = np.asarray(load_sensor4_calibration(sensor)['base_from_lidar'],float)
    before = {str(args.pack):sha(args.pack),str(sensor):sha(sensor),str(Path(__file__)):sha(__file__)}
    result = dict(schema='e12_postprocessing_tracking_support_v1',model=pack['model'],complete=False,
        production_changed=False,source_sha256=before,cases={},
        ap_definition='Catalog center AP averaged over .5/1/2/4m, at the selected production output operating point, NOT AMOTA/IDF1/native val AP.',
        velocity_definition='XY BBox-center finite-difference velocity with adjacent GT gap<=.25s; not UDP instantaneous rear-axle velocity.',
        limitations=['Cached eager outputs, measurement-time actual core only; not new production TRT/head inference or ROS scheduler/LAN/closed-loop.',
            'All history feeds core; only original frozen scoring-eligible frames enter accuracy metrics.',
            'Catalog-relative unmatched/precision is not adjudicated semantic FP truth.',
            'GT0 means finite returns in authored OBB+3cm; no point-support GT deletion.',
            'Track confidence is retained detection score; tracker center AP is not a calibrated confidence benchmark.',
            'ID changes use reconstructed actor keys and adjacent matched observations, not authoritative simulator identity.',
            'No synthetic frames in input gaps; no GT extrapolation across missing history.'])
    evidence,pooled = {},{m:[]for m in ('detector','tracker')}
    for name,case in pack['cases'].items():
        report,detail,modes = compare_case(case,extrinsic)
        result['cases'][name]=report;evidence[name]=detail
        for mode in modes:
            pooled[mode].extend(modes[mode])
        print(name, {m:round(report['stages'][m]['ap']['present_classes_mAP']*100,2)
            if report['stages'][m]['ap']['present_classes_mAP']is not None else None for m in modes},flush=True)
    all_support = [r for d in evidence.values()for r in d['support']]
    all_velocity = [r for d in evidence.values()for r in d['velocity']]
    result['pooled'] = dict(support_groups=support_groups(all_support),velocity=motion_groups(all_velocity),stages={})
    result['pooled']['direction_support_groups'] = {direction:support_groups([r for r in all_support if r['direction']==direction])
        for direction in ('front','rear')}
    result['pooled']['velocity_by_state'] = {state:motion_groups([r for r in all_velocity if
        (state=='initial_lt1s'and r['track_age_sec']<1)or(state=='mature_ge1s'and r['track_age_sec']>=1)or
        (state=='associated'and r['associated'])or(state=='coasting'and not r['associated'])])
        for state in ('initial_lt1s','mature_ge1s','associated','coasting')}
    metric = metric_module()
    for mode,rows in pooled.items():
        result['pooled']['stages'][mode] = dict(ap=ap_summary(rows,metric),operating=summary_counts(rows),
            by_range={f'{lo}_{hi}m':ap_summary(metric._filter_range(rows,(lo,hi)),metric)for lo,hi in BANDS})
    if before != {p:sha(p)for p in before}:
        raise ValueError('Input/source changed')
    result['complete']=True
    (args.output/'TRACKING_SUPPORT_REPORT.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with gzip.open(args.output/'observation_evidence.json.gz','xt')as handle:
        json.dump(evidence,handle,allow_nan=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pack',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    analyze(parser.parse_args())
