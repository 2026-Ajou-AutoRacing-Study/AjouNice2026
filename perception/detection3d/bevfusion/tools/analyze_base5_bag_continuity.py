#!/usr/bin/env python3
"""Detection-only operating points and miss runs on contiguous eligible bags.

All eligible GT participates in matching. Strong-return strata are diagnostics,
not a replacement visibility filter. Never bridge recording/GT/ROI gaps.
"""
import argparse
from collections import defaultdict
import gzip
import json
from pathlib import Path
import pickle

import numpy as np
from compare_base5_common30 import CLASSES, fixed_matches, scope, validate_payload
from summarize_base5_rosbag_quick import bridge_nms

THRESHOLDS = (.1, .15, .2, .25, .3, .4, .5, .6, .7, .8)


def miss_runs(records, nominal_period, maximum_gap=.25):
    """Records are (frame_index, stamp, matched), already gated by one GT ID.

    Break at every excluded/nonexistent GT frame or sensor hole. Duration is
    last-first stamp + nominal period, i.e. a sampled exposure estimate, not
    exact physical invisibility duration or a tracker deletion interval.
    """
    runs = []
    current = []
    previous = None
    for index, stamp, matched in records:
        continuous = previous is not None and index == previous[0]+1 and 0 < stamp-previous[1] <= maximum_gap
        if (matched or not continuous) and current:
            runs.append(dict(frames=len(current), seconds=current[-1]-current[0]+nominal_period,
                             start_stamp=current[0], end_stamp=current[-1]))
            current = []
        if not matched:
            current.append(stamp)
        previous = (index, stamp)
    if current:
        runs.append(dict(frames=len(current), seconds=current[-1]-current[0]+nominal_period,
                         start_stamp=current[0], end_stamp=current[-1]))
    return runs


def run_summary(records_by_key, period):
    runs = [dict(run, track_key=key) for key, records in records_by_key.items()
            for run in miss_runs(records, period)]
    durations = [r['seconds'] for r in runs]
    return dict(miss_episodes=len(runs), episodes_ge_0p5s=sum(d >= .5 for d in durations),
        episodes_ge_1s=sum(d >= 1 for d in durations),
        max_seconds=max(durations, default=0.),
        p95_seconds=float(np.percentile(durations, 95)) if durations else 0.,
        worst=sorted(runs, key=lambda r: -r['seconds'])[:8])


def analyze(inputs, predictions, output):
    if output.exists(): raise ValueError('Fresh diagnostics output required')
    with (inputs/'infos_val.pkl').open('rb') as f:
        authored = pickle.load(f)['data_list']
    sources = {r['token']: r for r in authored}
    manifest = json.loads((inputs/'manifest.json').read_text())
    # Preserve original LiDAR indices, not just the index in a filtered list.
    # Older prepared packages lack this field; read their original ROS headers
    # once instead of silently joining across excluded camera/GT frames.
    native_indices={}
    for case,contract in manifest['cases'].items():
        case_sources=[r for r in authored if r['scene_id']==case]
        if all('raw_lidar_index' in r for r in case_sources):
            native_indices.update({r['token']:r['raw_lidar_index'] for r in case_sources})
        else:
            import rosbag
            times=set()
            with rosbag.Bag(contract['raw_bag']) as bag:
                for _,message,stamp in bag.read_messages(topics=['/sensor/lidar1']):
                    header_time=message.header.stamp.to_sec()
                    times.add(header_time if header_time>0 else stamp.to_sec())
            mapping={t:index for index,t in enumerate(sorted(times))}
            native_indices.update({r['token']:mapping[r['timestamp']] for r in case_sources})
    report = dict(frames=len(authored), gt_identity=None, models={},
        scope='Common radial30 detection-only development bags. Score floor0.1/top100, class-aware rotated BEV NMS0.5, center distance<1m.',
        duration_definition='Miss-run last-first sensor stamp + median eligible frame period; splits at GT/ROI exclusion, nonconsecutive selected frames, or sensor gap>0.25s.',
        limitations=['Recorded sensor/GT support only; no visibility filtering, tracking or real-time queues.',
                     'GT track keys can be reconstructed trajectory associations, not authoritative simulator IDs.',
                     'Never call AP or frame recall an independent probability.',
                     'Thresholds are development diagnostics, not an automatically selected production policy.'])
    for path in predictions:
        with gzip.open(path, 'rt') as f: payload=json.load(f)
        validate_payload(payload)
        if payload['frames'] != len(authored) or {r['token'] for r in payload['rows']} != set(sources):
            raise ValueError('Incomplete continuous export')
        if report['gt_identity'] is not None and report['gt_identity'] != payload['gt_identity']:
            raise ValueError('Different model scenes/GT')
        report['gt_identity'] = payload['gt_identity']
        model = {}
        for case, contract in manifest['cases'].items():
            prepared=[]
            for row in payload['rows']:
                source = sources[row['token']]
                if source['scene_id'] != case: continue
                if len(source['instances']) != len(row['gt_boxes']):
                    raise ValueError('GT track identity alignment changed')
                for instance, box in zip(source['instances'], row['gt_boxes']):
                    expected=np.asarray(instance['bbox_3d']).copy(); expected[2]-=expected[5]/2
                    if not np.allclose(expected, box, rtol=0, atol=1e-5):
                        raise ValueError('GT geometry differs from authored track keys')
                merged=dict(row, gt_track_keys=[str(r['track_key']) for r in source['instances']])
                prepared.append((native_indices[row['token']],source['timestamp'],scope(bridge_nms(merged))))
            prepared.sort(key=lambda item:item[1])
            periods=np.diff([t for _,t,_ in prepared]); positive=periods[(periods>0)&(periods<=.25)]
            if not len(positive): raise ValueError('No contiguous exposure period')
            period=float(np.median(positive))
            classes={}
            for class_index,name in enumerate(CLASSES):
                curves=[]
                for threshold in THRESHOLDS:
                    gt=tp=pred=strong_gt=strong_tp=loose_tp=0
                    histories=defaultdict(list); strong=defaultdict(list)
                    loose_histories=defaultdict(list)
                    bands=defaultdict(lambda:dict(gt=0,tp=0))
                    for index,stamp,row in prepared:
                        matches,count=fixed_matches(row,class_index,threshold)
                        loose_matches,_=fixed_matches(row,class_index,threshold,tolerance=2.)
                        eligible=np.flatnonzero(row['gt_labels']==class_index)
                        gt+=len(eligible);tp+=len(matches);loose_tp+=len(loose_matches);pred+=count
                        for g in eligible:
                            matched=int(g) in matches
                            histories[row['gt_track_keys'][g]].append((index,stamp,matched))
                            loose_histories[row['gt_track_keys'][g]].append((index,stamp,int(g) in loose_matches))
                            distance=float(np.linalg.norm(row['gt_boxes'][g,:2]))
                            band='0_10m' if distance<10 else ('10_20m' if distance<20 else '20_30m')
                            bands[band]['gt']+=1;bands[band]['tp']+=int(matched)
                            if row['gt_points'][g]>=5:
                                strong_gt+=1;strong_tp+=int(matched)
                                strong[row['gt_track_keys'][g]].append((index,stamp,matched))
                    curves.append(dict(score=threshold,gt=gt,tp=tp,fp=pred-tp,fn=gt-tp,
                        precision=tp/pred if pred else None,recall=tp/gt if gt else None,
                        fp_per_frame=(pred-tp)/len(prepared),distance_counts=dict(bands),
                        miss_runs=run_summary(histories,period),
                        center2m=dict(tp=loose_tp,recall=loose_tp/gt if gt else None,
                                      miss_runs=run_summary(loose_histories,period)),
                        ge5points=dict(gt=strong_gt,tp=strong_tp,recall=strong_tp/strong_gt if strong_gt else None,
                                       miss_runs=run_summary(strong,period))))
                classes[name]=curves
            model[case]=dict(frames=len(prepared),nominal_period_sec=period,
                sensor_holes_over_0p25s=int(sum(periods>.25)),classes=classes)
            model[case]['excluded_native_frame_breaks']=int(sum(np.diff([i for i,_,_ in prepared])>1))
        report['models'][payload['name']]=model
        print(payload['name'],{case:{name:dict(recall=curves[0]['recall'],fp=curves[0]['fp'])
              for name,curves in info['classes'].items() if curves[0]['gt']}
              for case,info in model.items()},flush=True)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--predictions',type=Path,nargs='+',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();analyze(a.inputs,a.predictions,a.output)
