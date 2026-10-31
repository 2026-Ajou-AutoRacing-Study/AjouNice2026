#!/usr/bin/env python3
"""CPU actual-EKF replay of frozen eager/TRT ROS-bag detections.

Fresh core per case and per mode; no installed code, live ROS, GPU, Map-CV or
planner. IDs are compared as cross-run associations, never ground-truth IDF1.
Current production core settings are explicit, including pedestrian Gaussian
ranking. Recorded prepared Ego yaw already includes -1.347deg: do not repeat.
"""
import argparse
from copy import deepcopy
import gzip
import json
import math
from pathlib import Path
import shlex
import subprocess
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'perception/detection3d/bevfusion/scripts'))
from qualify_base5_camera_tensorrt import distribution,sha256,write_new
from analyze_base5_runtime_effects import match,drift
sys.path.insert(0,str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
from bevfusion_ros.rotated_bev_nms import rotated_bev_iou
CLASSES=('vehicle','pedestrian','obstacle')
TRACKER=ROOT/'base_ws/src/ekf-multi-object-tracking'


def world_detection(detection,ego,extrinsic):
    """Current bottom->gravity boundary, then original sensor/Ego transform."""
    item=deepcopy(detection)
    local=np.asarray([*item['center'],1.],float);local[2]+=item['size'][2]/2
    base=extrinsic@local;c,s=math.cos(ego['yaw']),math.sin(ego['yaw'])
    item['center']=[ego['x']+c*base[0]-s*base[1],ego['y']+s*base[0]+c*base[1],ego['z']+base[2]]
    item['yaw']+=ego['yaw']+math.atan2(extrinsic[1,0],extrinsic[0,0])
    return item


def parse_core(text,expected_stamps):
    frames=[];remaining=0;current=None
    for line in text.splitlines():
        fields=line.split()
        if not fields:continue
        if fields[0]=='FRAME':
            if remaining:raise ValueError('Incomplete previous tracker frame')
            current=dict(stamp=float(fields[1]),tracks=[]);frames.append(current);remaining=int(fields[2])
        elif fields[0]=='TRACK':
            if current is None or remaining<=0:raise ValueError('Unexpected TRACK')
            tracker_label=int(fields[2]);label=0 if tracker_label in (1,2) else 1 if tracker_label in (3,4) else 2
            current['tracks'].append(dict(id=int(fields[1]),tracker_label=tracker_label,label=label,
                class_name=CLASSES[label],confirmed=bool(int(fields[3])),associated=bool(int(fields[4])),
                center=list(map(float,fields[5:8])),yaw=float(fields[8]),velocity=list(map(float,fields[9:11])),
                yaw_rate=float(fields[11]),size=list(map(float,fields[12:15])),score=float(fields[15]),
                age_frames=int(fields[16]),last_measurement_stamp=float(fields[17]),birth_stamp=float(fields[18])))
            remaining-=1
    if remaining or [r['stamp'] for r in frames]!=expected_stamps:raise ValueError('Missing or reordered tracker frames')
    return frames


def replay(binary,frames,context,extrinsic):
    lines=[]
    for frame in frames:
        source=context[frame['token']]
        if frame['stamp']!=source['stamp']:raise ValueError('Saved frame stamp changed')
        detections=[world_detection(d,source['ego'],extrinsic) for d in frame['detections']]
        lines.append('%.17g %d'%(frame['stamp'],len(detections)))
        for d in detections:
            lines.append(' '.join('%.17g'%value for value in [d['label'],*d['center'],d['yaw'],*d['size'],d['score']]))
    output=subprocess.run([str(binary)],input='\n'.join(lines)+'\n',capture_output=True,text=True,check=True)
    return parse_core(output.stdout,[f['stamp'] for f in frames])


def within_run(frames):
    seen=set();coasting=[];near_pairs=[];counts={name:0 for name in CLASSES}
    for frame in frames:
        tracks=[t for t in frame['tracks'] if t['confirmed']]
        for track in tracks:
            seen.add((track['label'],track['id']));counts[track['class_name']]+=1
            if not track['associated']:
                coasting.append(dict(stamp=frame['stamp'],id=track['id'],label=track['label'],
                    age_sec=frame['stamp']-track['last_measurement_stamp']))
        for i,first in enumerate(tracks):
            for second in tracks[i+1:]:
                distance=float(np.linalg.norm(np.asarray(first['center'][:2])-second['center'][:2]))
                if distance<.75:
                    iou=rotated_bev_iou(first,second)
                    if iou>.5:near_pairs.append(dict(stamp=frame['stamp'],first_id=first['id'],second_id=second['id'],
                        same_class=first['label']==second['label'],distance_m=distance,iou=iou))
    return dict(frames=len(frames),confirmed_observations_by_class=counts,
        unique_confirmed_ids_by_class={name:len({ident for label,ident in seen if label==i}) for i,name in enumerate(CLASSES)},
        coasting_observations=len(coasting),coasting_age_sec=distribution([r['age_sec'] for r in coasting]),
        near_overlapping_confirmed_pairs=near_pairs,
        duplicate_note='Center<.75m and rotated IoU>.5 diagnostic only; not oracle duplicate truth or an applied suppression policy')


def compare(reference,candidate,gate):
    if [r['stamp'] for r in reference]!=[r['stamp'] for r in candidate]:raise ValueError('Tracker time grids differ')
    rows=[];changes=[];mapping={};mapping_changes=[]
    for old,new in zip(reference,candidate):
        for label,name in enumerate(CLASSES):
            a=[t for t in old['tracks'] if t['confirmed'] and t['label']==label]
            b=[t for t in new['tracks'] if t['confirmed'] and t['label']==label]
            pairs,missing,extra=match(a,b,gate)
            if missing or extra:changes.append(dict(stamp=old['stamp'],class_name=name,missing=len(missing),extra=len(extra)))
            for i,j in pairs:
                x,y=a[i],b[j];key=(label,x['id'])
                if key in mapping and mapping[key]!=y['id']:
                    mapping_changes.append(dict(stamp=old['stamp'],class_name=name,reference_id=x['id'],
                        previous_candidate_id=mapping[key],candidate_id=y['id']))
                mapping[key]=y['id'];angle=x['yaw']-y['yaw'];footprint=drift(x,y)
                rows.append(dict(stamp=old['stamp'],class_name=name,reference_id=x['id'],candidate_id=y['id'],
                    position_m=float(np.linalg.norm(np.asarray(x['center'][:2])-y['center'][:2])),
                    velocity_2d_mps=float(np.linalg.norm(np.asarray(x['velocity'])-y['velocity'])),
                    speed_mps=abs(float(np.linalg.norm(x['velocity'])-np.linalg.norm(y['velocity']))),
                    yaw_deg=abs(math.degrees(math.atan2(math.sin(angle),math.cos(angle)))),
                    size_linf_m=float(np.max(np.abs(np.asarray(x['size'])-y['size']))),
                    footprint_corner_m=footprint['footprint_corner_m'],rotated_bev_iou=footprint['rotated_bev_iou'],
                    associated_flag_flip=x['associated']!=y['associated']))
    metrics=('position_m','velocity_2d_mps','speed_mps','yaw_deg','size_linf_m','footprint_corner_m','rotated_bev_iou')
    return dict(matched_observations=len(rows),confirmed_set_changes=changes,id_mapping_changes=mapping_changes,
        distributions={key:distribution([r[key] for r in rows]) for key in metrics},
        by_class={name:dict(matched=sum(r['class_name']==name for r in rows),
            distributions={key:distribution([r[key] for r in rows if r['class_name']==name]) for key in metrics},
            associated_flag_flips=sum(r['associated_flag_flip'] for r in rows if r['class_name']==name)) for name in CLASSES},
        largest_position_changes=sorted(rows,key=lambda r:-r['position_m'])[:10],
        largest_footprint_changes=sorted(rows,key=lambda r:-r['footprint_corner_m'])[:10],
        note='Numeric ID mapping differences between runs are diagnostics, not GT ID switches or IDF1')


def compare_delivered(reference,candidate):
    """Retain yaw/footprint outliers even when temporal core gates tolerate them."""
    if [(r['token'],r['stamp']) for r in reference]!=[(r['token'],r['stamp']) for r in candidate]:
        raise ValueError('Detector time grids differ')
    classes={}
    for label,name in enumerate(CLASSES):
        rows=[];missing=extra=0
        for old,new in zip(reference,candidate):
            a=[d for d in old['detections'] if d['label']==label]
            b=[d for d in new['detections'] if d['label']==label]
            pairs,lost,added=match(a,b,1.)
            missing+=len(lost);extra+=len(added)
            rows.extend(dict(token=old['token'],**drift(a[i],b[j])) for i,j in pairs)
        metrics=('center_xy_m','size_linf_m','yaw_deg','score_abs','footprint_corner_m','rotated_bev_iou')
        classes[name]=dict(matched=len(rows),unmatched_reference=missing,unmatched_candidate=extra,
            distributions={key:distribution([r[key] for r in rows]) for key in metrics},
            confidence_half_regime_flips=sum(r['tracker_confidence_regime_flip'] for r in rows),
            largest_footprint_changes=sorted(rows,key=lambda r:-r['footprint_corner_m'])[:10])
    return classes


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--context',type=Path,required=True)
    p.add_argument('--sensor-config',type=Path,required=True,help='Original archived sensor JSON; SHA must match the frozen context')
    p.add_argument('--detections',type=Path,required=True,help='Directory with eager/camera_trt/eager_repeat_delivered.json.gz')
    p.add_argument('--reference-mode',default='eager')
    p.add_argument('--candidate-mode',default='camera_trt')
    p.add_argument('--restore-mode',default='eager_repeat')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    modes=(a.reference_mode,a.candidate_mode,a.restore_mode)
    if len(set(modes))!=3 or any(not mode.replace('_','').isalnum() for mode in modes):
        p.error('Three distinct simple mode names required')
    if a.output.exists():p.error('Fresh tracker output required')
    a.output.mkdir(parents=True)
    harness=Path(__file__).with_name('replay_base5_continuous_tracker_core.cpp')
    source=TRACKER/'src/algorithm/ekf_multi_object_tracking.cpp';header=TRACKER/'include/algorithm/ekf_multi_object_tracking.hpp'
    calibration_source=ROOT/'docker/bevfusion/app/sensor4_calibration.py'
    files=[Path(__file__),harness,source,header,a.context,a.sensor_config,calibration_source]
    files += [a.detections/(mode+'_delivered.json.gz') for mode in modes]
    pins={str(path.resolve()):sha256(path) for path in files}
    context_payload=json.loads(a.context.read_text());context={r['token']:r for r in context_payload['rows']}
    if sha256(a.sensor_config)!=context_payload['calibration']['config_sha256']:
        raise ValueError('Original recorded sensor calibration SHA differs')
    sys.path.insert(0,str(ROOT/'docker/bevfusion'))
    from app.sensor4_calibration import load_sensor4_calibration
    extrinsic=np.asarray(load_sensor4_calibration(a.sensor_config)['base_from_lidar'],float)
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','roscpp'],text=True))
    binary=(a.output/'tracker_core').resolve()
    command=['g++','-std=c++17','-O1','-I/usr/include/eigen3','-I'+str(TRACKER/'include'),str(harness),str(source),'-o',str(binary),*flags]
    subprocess.run(command,check=True)
    payloads={mode:json.load(gzip.open(a.detections/(mode+'_delivered.json.gz'),'rt')) for mode in modes}
    reference=payloads[a.reference_mode]
    for payload in payloads.values():
        for key in ('input_manifest_sha256','checkpoint_sha256','box_origin','thresholds','nms'):
            if payload[key]!=reference[key]:raise ValueError('Different detector delivery contract: '+key)
        if [r['token'] for r in payload['frames']]!=[r['token'] for r in reference['frames']]:raise ValueError('Different chronological detections')
    reports={};all_frames={}
    for case in context_payload['cases']:
        frames={mode:[r for r in p['frames'] if r['case']==case] for mode,p in payloads.items()}
        if not frames[a.reference_mode]:raise ValueError('Empty selected tracker case')
        outputs={mode:replay(binary,rows,context,extrinsic) for mode,rows in frames.items()}
        control=replay(binary,frames[a.reference_mode],context,extrinsic)
        if outputs[a.reference_mode]!=control:raise ValueError('Same detections did not reproduce exact fresh-core state')
        reports[case]=dict(within_run={mode:within_run(rows) for mode,rows in outputs.items()},
            detector_comparisons={mode:compare_delivered(frames[a.reference_mode],frames[mode]) for mode in modes[1:]},
            comparisons={mode:{str(gate):compare(outputs[a.reference_mode],outputs[mode],gate) for gate in (.5,2.)}
                         for mode in modes[1:]},
            same_input_fresh_core_exact=True)
        all_frames[case]=outputs
    if pins!={path:sha256(path) for path in pins}:raise RuntimeError('Tracker source/input changed')
    result=dict(complete=True,schema='base5_continuous_trt_actual_core_v1',cases=reports,
        reference_mode=a.reference_mode,candidate_mode=a.candidate_mode,restore_mode=a.restore_mode,
        input_source_sha256=pins,binary_sha256=sha256(binary),build_command=command,
        scope='Unmodified actual EKF core with explicit production core settings; saved detections and prepared Ego transform. Not actual ROS-node TF, yaw canonicalizer,30Hz output scheduler,UNKNOWN policy,Map-CV/planner or GT accuracy certification.',
        configuration=dict(pedestrian_gaussian_ranking=True,CTRV=True,position_noise_m=.4,association_gate_m=5.,
            coasting_moving_stationary_sec=[.5,1.],detector_velocity_fusion=False,duplicate_birth_guard=True),
        UNKNOWN_note='Velocity differences shown are raw core states. Actual production UNKNOWN stationary output policy would set speed/yaw-rate0 but would not erase position/shape/ID differences.',
        ego_note=context_payload['ego_yaw'],production_changed=False,GT_IDF1_computed=False)
    with (a.output/'frames.json.gz').open('xb') as raw:
        with gzip.open(raw,'wt') as stream:json.dump(all_frames,stream,allow_nan=False)
    write_new(a.output/'report.json',result)
    print(json.dumps({case:{mode:{gate:dict(matched=value['matched_observations'],
        changes=len(value['confirmed_set_changes']),mapping_changes=len(value['id_mapping_changes']),
        position=value['distributions']['position_m']) for gate,value in gates.items()}
        for mode,gates in report['comparisons'].items()} for case,report in reports.items()},indent=2),flush=True)


if __name__=='__main__':main()
