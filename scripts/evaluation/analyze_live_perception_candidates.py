#!/usr/bin/env python3
"""CPU live-probe repeatability, operating correspondence and EKF regression.

No AP/FP oracle is claimed from simulator actor geometry. Compound components
are not one-to-one with actor records. All modes use frozen identical inputs.
"""
import argparse
from collections import Counter,defaultdict
import gzip
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment
import yaml

from experiment_base5_proposal_peaks import ROOT,digest
from analyze_base5_proposal_peaks import distribution,CLASSES,THRESHOLDS
from audit_tracking_continuity_events import build_original,replay,core_summary


def motion_summary(output,gt):
    """Bracketed center-difference diagnostic, never ObjectStatus velocity=0."""
    from audit_tracking_continuity import match,sample_series
    series=gt['gt'];values=defaultdict(list);counts=Counter()
    for frame in output:
        t=frame['stamp'];truth=[];motion={}
        for stamps,objects in series:
            index=np.searchsorted(stamps,t)
            current=sample_series((stamps,objects),t,.2)
            if current is None:continue
            truth.append(current)
            left=max(0,index-2);right=min(len(stamps)-1,index+2)
            if right<=left or any(b-a>.2 for a,b in zip(stamps[left:right],stamps[left+1:right+1])):
                counts['unbracketed_velocity']+=1;continue
            speed=(np.asarray(objects[right]['center'][:2])-objects[left]['center'][:2])/(stamps[right]-stamps[left])
            motion[current['key']]=speed
        tracks=[dict(track,class_name='pedestrian' if track['core_class'] in (3,4) else 'vehicle' if track['core_class'] in (1,2) else 'obstacle')
                for track in frame['tracks'] if track['confirmed']]
        for g,p,_ in match(truth,tracks,2.)[0]:
            if truth[g]['key'] not in motion:continue
            target=motion[truth[g]['key']]
            mode='moving' if np.linalg.norm(target)>.5 else 'slow_or_stopped'
            key=truth[g]['class_name']+'/'+mode
            counts[key+'/matched']+=1
            values[key+'/velocity_vector_error_mps'].append(float(np.linalg.norm(np.asarray(tracks[p]['velocity'])-target)))
            values[key+'/speed_abs_error_mps'].append(float(abs(np.linalg.norm(tracks[p]['velocity'])-np.linalg.norm(target))))
    return dict(counts=dict(counts),distributions={k:distribution(v) for k,v in values.items()},
                scope='Prepared catalog center finite difference with strict <=.2s source gaps, not synchronized UDP velocity GT.')


def shape_summary(output):
    small,tall=[],[];histories=defaultdict(list)
    for frame in output:
        people=[t for t in frame['tracks'] if t['confirmed'] and t['core_class'] in (3,4)]
        small.append(sum(t['size'][2]<1.1 for t in people));tall.append(sum(t['size'][2]>=1.1 for t in people))
        for t in people:histories[t['id']].append(float(t['size'][2]))
    return dict(small_height_count=distribution(small),tall_height_count=distribution(tall),
        per_id_height={str(k):distribution(v) for k,v in histories.items()},
        scope='Height strata are diagnostics, not true dog/human identity or component GT.')


def matches(a,b,labels_a,labels_b,gate=1.):
    if not len(a) or not len(b):
        return []
    costs=np.linalg.norm(np.asarray(a)[:,None,:2]-np.asarray(b)[None,:,:2],axis=2)
    permitted=(np.asarray(labels_a)[:,None]==np.asarray(labels_b)[None,:]) & (costs<gate)
    penalty=float(max(len(a),len(b))+1)*max(1.,gate)
    left,right=linear_sum_assignment(np.where(permitted,costs,penalty))
    return [(int(i),int(j),float(costs[i,j])) for i,j in zip(left,right) if permitted[i,j]]


def final(row):
    sys.path.insert(0,str(ROOT/'perception/detection3d/bevfusion/tools'))
    from summarize_base5_rosbag_quick import bridge_nms
    value=bridge_nms(row)
    indices=[i for i,(label,score) in enumerate(zip(value['pred_labels'],value['pred_scores'])) if score>=THRESHOLDS[label]]
    return {**value,**{k:[value[k][i] for i in indices] for k in ('pred_boxes','pred_labels','pred_scores')}}


def paired(left,right):
    numbers=defaultdict(list);counts=Counter()
    for a,b in zip(left,right):
        if a['token']!=b['token'] or a['point_sha256']!=b['point_sha256'] or a['image_sha256']!=b['image_sha256']:
            raise ValueError('A/B frozen input identities differ')
        counts['frames']+=1
        counts['raw_bitwise_changed_frames']+=any(a[k]!=b[k] for k in ('pred_boxes','pred_labels','pred_scores'))
        x,y=final(a),final(b)
        counts['operating_bitwise_changed_frames']+=any(x[k]!=y[k] for k in ('pred_boxes','pred_labels','pred_scores'))
        pairs=matches(x['pred_boxes'],y['pred_boxes'],x['pred_labels'],y['pred_labels'],2.)
        counts['matched_operating_boxes']+=len(pairs)
        counts['removed_or_displaced_operating']+=len(x['pred_boxes'])-len(pairs)
        counts['added_or_displaced_operating']+=len(y['pred_boxes'])-len(pairs)
        for i,j,d in pairs:
            prefix=CLASSES[x['pred_labels'][i]]
            numbers[prefix+'/xy_m'].append(d)
            numbers[prefix+'/score'].append(abs(x['pred_scores'][i]-y['pred_scores'][j]))
            yaw=x['pred_boxes'][i][6]-y['pred_boxes'][j][6]
            numbers[prefix+'/axis_yaw_deg'].append(math.degrees(abs(math.atan2(math.sin(2*yaw),math.cos(2*yaw)))/2))
    return dict(counts=dict(counts),distributions={k:distribution(v) for k,v in numbers.items()})


def correspondence(rows):
    counts=Counter();errors=defaultdict(list)
    for raw in rows:
        if not raw.get('actor_geometry_resolved',True):
            counts['unresolved_actor_frames_not_scored']+=1
            continue
        row=final(raw)
        for label,name in enumerate(CLASSES):
            gi=[i for i,l in enumerate(row['gt_labels']) if l==label]
            pi=[i for i,l in enumerate(row['pred_labels']) if l==label]
            gt=[row['gt_boxes'][i] for i in gi];pred=[row['pred_boxes'][i] for i in pi]
            pairs=matches(gt,pred,[label]*len(gt),[label]*len(pred),1.)
            counts[name+'/gt_observations']+=len(gt);counts[name+'/operating_boxes']+=len(pred)
            counts[name+'/matched_1m']+=len(pairs)
            counts[name+'/unmatched_operating']+=len(pred)-len(pairs)
            for g,p,d in pairs:
                counts[name+'/matched_20plus_points']+=row['gt_points'][gi[g]]>=20
                errors[name+'/xy_m'].append(d)
                # Both arrays are exported bottom center; assess gravity semantics separately.
                physical_z=gt[g][2]+gt[g][5]/2
                errors[name+'/legacy_ros_z_abs_m'].append(abs(pred[p][2]-physical_z))
                errors[name+'/gravity_ros_z_abs_m'].append(abs(pred[p][2]+pred[p][5]/2-physical_z))
    return dict(counts=dict(counts),errors={k:distribution(v) for k,v in errors.items()},
                scope='Catalog GT center diagnostics. Unmatched != proven false positive. No visibility filtering, AP or compound-component annotation.')


def world(box,row,gravity):
    b=np.asarray(box,dtype=float).copy()
    if gravity: b[2]+=b[5]/2
    b[:3]+=row['lidar_translation']
    ego=row['ego'];c,s=math.cos(ego['yaw']),math.sin(ego['yaw'])
    return [c*b[0]-s*b[1]+ego['x'],s*b[0]+c*b[1]+ego['y'],b[2]+ego['z'],*b[3:6],b[6]+ego['yaw']]


def tracker_frames(rows):
    frames=[];series=defaultdict(dict);egos=[];times=[]
    for raw in rows:
        row=final(raw);t=row['stamp'];times.append(t)
        objects=[]
        for b,l,score in zip(row['pred_boxes'],row['pred_labels'],row['pred_scores']):
            b=world(b,row,True)
            objects.append(dict(center=b[:3],size=b[3:6],yaw=b[6],score=score,class_name=CLASSES[l]))
        frames.append(dict(stamp=t,world_objects=objects))
        for b,l,key,asset in zip(row['gt_boxes'],row['gt_labels'],row['parent_uids'],row['gt_assets']):
            b=world(b,row,True)
            series[key][t]=dict(key=key,asset=asset,center=b[:3],size=b[3:6],yaw=b[6],class_name=CLASSES[l])
        ego=row['ego'];egos.append(dict(center=[ego['x'],ego['y'],ego['z']],yaw=ego['yaw']))
    return frames,dict(ego=(times,egos),gt=[(sorted(v),[v[t] for t in sorted(v)]) for v in series.values()],unresolved=[])


def main(args):
    if args.output.exists(): raise FileExistsError('Fresh analysis output required')
    args.output.mkdir(parents=True)
    buffers={p:json.load(gzip.open(args.predictions/(p+'.json.gz'),'rt'))['rows']
             for p in ('legacy_A','legacy_B','nearest_A','nearest_B')}
    if args.gt_inputs:
        corrected=json.loads(args.gt_inputs.read_text())
        lookup={r['token']:r for r in corrected['rows']}
        for mode,rows in buffers.items():
            if {r['token'] for r in rows}!=set(lookup):raise ValueError('Corrected GT cohort differs')
            for index,row in enumerate(rows):
                source=lookup[row['token']]
                for key in ('stamp','point_sha256','image_sha256'):
                    if row[key]!=source[key]:raise ValueError('GT-only correction changed sensor inputs')
                # Prediction payloads remain immutable. Derived analysis only.
                rows[index]={**row,**{k:v for k,v in source.items() if k.startswith('gt_') or k in
                    ('ego','parent_uids','heading_correction_deg','box_center_convention','actor_geometry_resolved')}}
    cases=sorted({r['case'] for r in buffers['legacy_A']})
    parameters=yaml.safe_load((ROOT/'base_ws/src/ekf-multi-object-tracking/config/config.yaml').read_text())['configure']
    binaries={name:build_original(args.output/('core_'+name),parameters,policy)[0]
              for name,policy in (('baseline',None),('ped_gaussian',dict(gaussian_xy_ranking='pedestrian')))}
    report=dict(cases={},input_receipt=json.loads((args.predictions/'receipt.json').read_text()),
        corrected_gt_inputs_sha256=digest(args.gt_inputs) if args.gt_inputs else None,
        scope='Full retained parked-Ego live inputs, exact A/A/A/B forwards, original C++ core. No actual 30Hz ROS/planner/closed-loop certification.')
    for case in cases:
        selected={k:[r for r in v if r['case']==case] for k,v in buffers.items()}
        row=dict(repeatability={p:paired(selected[p+'_A'],selected[p+'_B']) for p in ('legacy','nearest')},
            depth_change=paired(selected['legacy_A'],selected['nearest_A']),
            detector={p:correspondence(selected[p+'_A']) for p in ('legacy','nearest')},tracker={})
        for p in ('legacy','nearest'):
            frames,gt=tracker_frames(selected[p+'_A'])
            for name,binary in binaries.items():
                output=replay(binary,frames)
                row['tracker'][p+'/'+name]=core_summary(output,gt if all(r.get('actor_geometry_resolved',True) for r in selected[p+'_A']) else None)
                row['tracker'][p+'/'+name]['shape_diagnostic']=shape_summary(output)
                if all(r.get('actor_geometry_resolved',True) for r in selected[p+'_A']):
                    row['tracker'][p+'/'+name]['motion_diagnostic']=motion_summary(output,gt)
                with gzip.open(args.output/(case+'_'+p+'_'+name+'.json.gz'),'wt') as stream:json.dump(output,stream)
        report['cases'][case]=row
        print(json.dumps(dict(case=case,repeats={p:row['repeatability'][p]['counts'] for p in ('legacy','nearest')},
            ids={k:v['confirmed_unique_ids'] for k,v in row['tracker'].items()})),flush=True)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--predictions',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gt-inputs',type=Path)
    main(parser.parse_args())
