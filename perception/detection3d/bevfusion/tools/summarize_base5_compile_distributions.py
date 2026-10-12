#!/usr/bin/env python3
"""Reanalyze saved outputs, with no GPU inference or production mutations.

Compile is isolated against parallel-only, alongside the original baseline
comparison. Observation quantiles are not independent object/GT error rates.
"""
import argparse
from collections import defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import pickle

import numpy as np

from analyze_base5_runtime_effects import (
    CLASSES, POLICIES, delivered, distribution, drift, match)
from compare_base5_common30 import validate_payload
from replay_base5_compiled_tracking import (
    ROOT, compare_tracks, core_replay, ego_trajectory, load_sensor4_calibration)

DETECTION_METRICS = ('center_xy_m', 'size_linf_m', 'yaw_deg', 'score_abs',
                     'footprint_corner_m', 'rotated_bev_iou')
TRACK_METRICS = ('position_m', 'velocity_2d_mps', 'speed_mps', 'yaw_deg')
TRACK_CLASSES = {0: 'obstacle', 1: 'vehicle', 2: 'vehicle', 3: 'pedestrian', 4: 'pedestrian'}


def summarize(rows, metrics, counts):
    result = dict(counts, distributions={key: distribution([r[key] for r in rows])
                                        for key in metrics})
    pos = 'position_m' if 'position_m' in metrics else 'center_xy_m'
    result['position_tail'] = {
        str(limit): dict(count=sum(r[pos] > limit for r in rows), denominator=len(rows),
                         fraction=sum(r[pos] > limit for r in rows)/len(rows) if rows else None)
        for limit in (.001, .01, .05, .1, .2, .5)}
    return result


def from_native(rows):
    return {r['token']: delivered([
        dict(center=b[:3], size=b[3:6], yaw=b[6], label=int(l), class_name=CLASSES[l], score=s)
        for b, l, s in zip(r['pred_boxes'], r['pred_labels'], r['pred_scores'])], POLICIES['current'])
        for r in rows}


def detection_summary(reference, candidate, maximum_range=None):
    deltas = []; counts = {name: defaultdict(int) for name in (*CLASSES, 'all')}
    changes = []
    if set(reference) != set(candidate):raise ValueError('Different detection tokens')
    for token in reference:
        for label, name in enumerate(CLASSES):
            def selected(data):
                return [d for d in data[token] if d['label'] == label and
                        (maximum_range is None or
                         (np.linalg.norm(d['center'][:2]) < maximum_range and -3 <= d['center'][2] <= 2))]
            a, b = selected(reference), selected(candidate)
            pairs, missing, extra = match(a, b)
            if missing or extra:
                changes.append(dict(token=token, class_name=name, removed=len(missing), added=len(extra)))
            for key in (name, 'all'):
                counts[key]['reference_observations'] += len(a)
                counts[key]['candidate_observations'] += len(b)
                counts[key]['removed'] += len(missing)
                counts[key]['added'] += len(extra)
                counts[key]['changed_frame_classes'] += bool(missing or extra)
            deltas.extend(dict(token=token, class_name=name, **drift(a[i], b[j])) for i, j in pairs)
    return dict(by_class={name: summarize(
        [r for r in deltas if name == 'all' or r['class_name'] == name], DETECTION_METRICS, counts[name])
        for name in (*CLASSES, 'all')}, delivery_changes=changes), deltas


def tracker_summary(reference, candidate, gate):
    deltas = []; reports = {}; counts = {name: defaultdict(int) for name in (*CLASSES, 'all')}
    identities = {name: set() for name in (*CLASSES, 'all')}
    for case in reference:
        rows = []
        reports[case] = compare_tracks(reference[case], candidate[case], gate=gate, state_rows=rows)
        generations=defaultdict(int);previous=set();segments={}
        for frame in reference[case]:
            present={t['id'] for t in frame['tracks']}
            for identity in present-previous:generations[identity]+=1
            for identity in present:segments[(frame['stamp'],identity)]=generations[identity]
            previous=present
        for row in rows:
            row.update(case=case, class_name=TRACK_CLASSES[row['label']],
                       reference_segment=segments[(row['stamp'],row['reference_id'])])
        deltas.extend(rows)
        for side, frames in (('reference', reference[case]), ('candidate', candidate[case])):
            for frame in frames:
                for track in frame['tracks']:
                    if not track['confirmed']:continue
                    name=TRACK_CLASSES[track['label']]
                    for key in (name, 'all'):
                        counts[key][side+'_observations'] += 1
                        if side=='reference':identities[key].add((case,track['id']))
        for event in reports[case]['confirmed_set_changes']:
            name=TRACK_CLASSES[event['label']]
            for key in (name, 'all'):
                counts[key]['removed'] += event['removed'];counts[key]['added'] += event['added']
                counts[key]['changed_frame_classes'] += 1
    result = dict(by_case=reports, matching_gate_m=gate, by_class={})
    for name in (*CLASSES, 'all'):
        rows = [r for r in deltas if name=='all' or r['class_name']==name]
        block = summarize(rows, TRACK_METRICS, dict(counts[name], reference_track_ids=len(identities[name])))
        groups = defaultdict(list)
        for row in rows:groups[(row['case'], row['reference_id'], row['reference_segment'])].append(row)
        # One statistic per reference track, so long-lived tracks do not
        # dominate these supplemental distributions. Not GT identity matching.
        block['reference_track_balanced'] = dict(
            matched_track_segments=len(groups),
            distribution_of_track_mean_position_m=distribution(
                [np.mean([r['position_m'] for r in group]) for group in groups.values()]),
            distribution_of_track_max_position_m=distribution(
                [max(r['position_m'] for r in group) for group in groups.values()]))
        block['by_association'] = {
            bucket: summarize([r for r in rows if (
                'both_associated' if r['reference_associated'] and r['candidate_associated'] else
                'both_coasting' if not r['reference_associated'] and not r['candidate_associated'] else
                'different_association') == bucket], TRACK_METRICS, {})
            for bucket in ('both_associated', 'both_coasting', 'different_association')}
        result['by_class'][name]=block
    return result, deltas


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def analyze(args):
    if args.output.exists():raise ValueError('Fresh output directory required')
    args.output.mkdir(parents=True)
    paths = {'baseline':args.baseline, 'parallel':args.parallel, 'compiled_parallel':args.compiled}
    payloads={}
    for name,path in paths.items():
        with gzip.open(path,'rt') as f:payloads[name]=json.load(f)
        validate_payload(payloads[name])
    first=payloads['baseline']
    for p in payloads.values():
        if any(p[k]!=first[k] for k in ('checkpoint_sha256','info_sha256','gt_identity','frames','inference_precision')):
            raise ValueError('Same checkpoint, scene and precision required')
    if payloads['baseline']['inference_optimization'].get('compile_parts'):
        raise ValueError('Baseline must not compile')
    a,b=(payloads[k]['inference_optimization'] for k in ('parallel','compiled_parallel'))
    if a.get('compile_parts') != [] or b.get('compile_parts') != ['camera','neck']:
        raise ValueError('Camera and neck compile OFF/ON pair required')
    if not a.get('parallel_camera') or {k:v for k,v in a.items() if k!='compile_parts'} != {
            k:v for k,v in b.items() if k!='compile_parts'}:
        raise ValueError('Compile-only comparison must preserve all other optimization options')
    infos=pickle.loads((args.inputs/'infos_val.pkl').read_bytes())['data_list']
    times={r['token']:r['timestamp'] for r in infos}
    if set(times)!=set(r['token'] for r in first['rows']):raise ValueError('Infos and outputs differ')
    manifest=json.loads((args.inputs/'manifest.json').read_text())
    ego={case:ego_trajectory(Path(d['raw_bag'])) for case,d in manifest['cases'].items()}
    extrinsic=load_sensor4_calibration(args.sensor_config)['base_from_lidar']
    native={name:from_native(p['rows']) for name,p in payloads.items()}
    result=dict(scope='Saved E24 outputs; paired detection and offline real EKF states, not GT error/live ROS certification.',
        frames=first['frames'], gt_identity=first['gt_identity'],
        checkpoint_sha256=first['checkpoint_sha256'],
        units='Positions in metres, velocities m/s, angles degrees; score absolute probability difference.',
        population='Each matched object-frame has equal weight; supplemental tracker-balanced statistics give each reference track equal weight.',
        matching_note='Class-preserving nearest Hungarian matching; unmatched counts kept. 0.5m gate can censor state tails; 2m is diagnostic sensitivity, not certified object identity.',
        tracker_identity_note='Numeric IDs can be reused. Balanced units separate reappearances after absence; they are tracker lifecycle segments, not physical GT objects.',
        source_sha256={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in paths.items()},
        binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        tracker_source_sha256=hashlib.sha256((ROOT/'base_ws/src/ekf-multi-object-tracking/src/algorithm/ekf_multi_object_tracking.cpp').read_bytes()).hexdigest(),
        optimization={name:p['inference_optimization'] for name,p in payloads.items()}, ranges={})
    for range_name,maximum in (('radial30',30.),('full_model_roi',None)):
        print('Processing',range_name,flush=True)
        frames={}
        for name,p in payloads.items():
            frames[name]={}
            for case in ego:
                rows=sorted([r for r in p['rows'] if r['token'].startswith(case+'_')],key=lambda r:times[r['token']])
                frames[name][case]=core_replay(args.binary,rows,times,ego[case],extrinsic,POLICIES['current'],maximum)
        with gzip.open(args.output/(range_name+'_tracker_states.json.gz'),'wt') as f:json.dump(frames,f,allow_nan=False)
        comparisons={}
        for label,a,b in (('compile_isolated','parallel','compiled_parallel'),
                          ('combined_vs_baseline','baseline','compiled_parallel'),
                          ('parallel_control','baseline','parallel')):
            det,det_rows=detection_summary(native[a],native[b],maximum)
            trackers={}
            for gate in (.5,2.):
                report,rows=tracker_summary(frames[a],frames[b],gate)
                trackers[str(gate)]=report
                with gzip.open(args.output/f'{range_name}_{label}_tracker_gate{gate}_rows.json.gz','wt') as f:
                    json.dump(rows,f,allow_nan=False)
            comparisons[label]=dict(reference=a,candidate=b,detection=det,tracker=trackers)
            with gzip.open(args.output/f'{range_name}_{label}_detection_rows.json.gz','wt') as f:json.dump(det_rows,f,allow_nan=False)
        result['ranges'][range_name]=comparisons
        dump(args.output/'summary.json',result)
    # Unordered val scenes supplement bag class coverage, not temporal replay.
    result['native_validation']={}
    for path in args.validation_reports:
        report=json.loads(path.read_text());cases=report['cases']
        if not report.get('preserve_learned_config') or report['model_geometry']['classes']!=list(CLASSES):
            raise ValueError('Unchanged learned three-class native validation required')
        parallel=[i for i,c in enumerate(cases,1) if c['variant']=='parallel_camera'][0]
        compiled=[i for i,c in enumerate(cases,1) if c['variant']=='camera_neck_parallel'][0]
        def read(index):return json.loads((path.parent/f'{index}_{cases[index-1]["variant"]}_predictions.json').read_text())
        a,b=read(parallel),read(compiled)
        if set(a)!=set(b):raise ValueError('Different native val scenes')
        value,rows=detection_summary(a,b)
        value.update(frames=len(a),source_report_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        result['native_validation'][path.parent.name]=value
        with gzip.open(args.output/(path.parent.name+'_native_rows.json.gz'),'wt') as f:json.dump(rows,f,allow_nan=False)
    dump(args.output/'summary.json',result)
    print('Saved',args.output/'summary.json',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','baseline','parallel','compiled','binary','sensor-config','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--validation-reports',type=Path,nargs='*',default=[])
    analyze(parser.parse_args())
