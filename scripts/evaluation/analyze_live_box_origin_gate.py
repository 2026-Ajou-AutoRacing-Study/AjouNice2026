#!/usr/bin/env python3
"""Read actual NN response, bridge output and two fresh installed EKF topics.

No edits/defaults. Primary uses the true adapter. Shadow is explicitly the
inverse representation of that same delivered message, not a second forward.
"""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment
from analyze_base5_proposal_peaks import distribution
from audit_tracking_continuity import object_row
from audit_tracking_continuity_events import sha


def analyze(args):
    import rosbag
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'base_ws/src/perception/bevfusion_ros/src'))
    from bevfusion_ros.box_center_origin import response_for_ros
    from bevfusion_ros.score_thresholds import filter_detections
    from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms
    source=json.loads((args.source/'report.json').read_text())
    raw={r['stamp']:r for r in source['responses']}
    counts=Counter();values=defaultdict(list);traces=[]
    bag=args.source/'live.bag'
    with rosbag.Bag(str(bag)) as handle:
        for topic,message,receipt in handle.read_messages(topics=['/origin_live/gravity/detection','/perception/bevfusion/frame_timing']):
            if topic.endswith('frame_timing'):
                timing=json.loads(message.data)
                for key,value in timing.items():
                    if key.endswith('_ms') and isinstance(value,(int,float)):values['stage/'+key].append(float(value))
                continue
            stamp=message.header.stamp.to_sec();key=min(raw,key=lambda t:abs(t-stamp)) if raw else None
            if key is None or abs(key-stamp)>1.e-6:
                counts['unmatched_response']+=1;continue
            expected=response_for_ros(raw[key]['response'],'gravity_center_v1')
            expected=class_aware_rotated_bev_nms(filter_detections(expected['detections'],.1,
                dict(vehicle=.4,pedestrian=.1,obstacle=.1)),.5)
            actual=[object_row(obj) for obj in message.objects]
            if len(expected)!=len(actual):raise ValueError('Actual bridge kept-box count differs from adapter oracle')
            for a,b in zip(actual,expected):
                if a['class_name']!=b['class_name']:raise ValueError('Actual bridge class order differs')
                error=float(np.max(np.abs(np.asarray(a['center'])-b['center'])))
                if error>1.e-6:raise ValueError('Actual bridge gravity center differs from tagged oracle')
                values['adapter/max_axis_center_error_m'].append(error)
                values['adapter/half_height_shift_m'].append(float(b['size'][2]/2))
                counts['adapter_boxes_verified']+=1
            counts['adapter_frames_verified']+=1
            values['detector_header_to_bag_receipt_ms'].append((receipt.to_sec()-stamp)*1000)
    gravity=source['tracks']['gravity'];legacy=source['tracks']['legacy']
    matched_frames=0
    for a in gravity:
        if not legacy:continue
        b=min(legacy,key=lambda r:abs(r['stamp']-a['stamp']))
        if abs(a['stamp']-b['stamp'])>.02:counts['tracker_scheduler_unmatched_frames']+=1;continue
        matched_frames+=1
        values['tracker/scheduler_pair_offset_ms'].append(abs(a['stamp']-b['stamp'])*1000)
        left,right=a['objects'],b['objects']
        if not left or not right:continue
        distances=np.linalg.norm(np.asarray([o['center'][:2] for o in left])[:,None,:]-np.asarray([o['center'][:2] for o in right])[None,:,:],axis=2)
        li,ri=linear_sum_assignment(distances)
        for i,j in zip(li,ri):
            if distances[i,j]>.1:counts['tracker_unmatched_or_displaced_boxes']+=1;continue
            x,y=left[i],right[j]
            values['tracker/xy_between_modes_m'].append(float(distances[i,j]))
            values['tracker/z_shift_minus_half_filtered_height_m'].append(float(abs(x['center'][2]-y['center'][2]-x['size'][2]/2)))
            values['tracker/height_between_modes_m'].append(float(abs(x['size'][2]-y['size'][2])))
            counts['tracker_paired_boxes']+=1
        counts['tracker_extra_gravity_boxes']+=len(left)-len(li)
        counts['tracker_extra_legacy_boxes']+=len(right)-len(ri)
    counts['tracker_scheduler_paired_frames']=matched_frames
    for mode,frames in source['tracks'].items():
        counts[mode+'/published_frames']=len(frames)
        counts[mode+'/distinct_ids']=len({tuple(o['id']) for r in frames for o in r['objects']})
        errors=[]
        log=(args.source/'stack_logs'/('tracker_'+mode+'.log')).read_text()
        counts[mode+'/transform_failure_log_lines']=sum('Transform failure' in line for line in log.splitlines())
    report=dict(counts=dict(counts),distributions={k:distribution(v) for k,v in values.items()},
        source_report_sha256=sha(args.source/'report.json'),bag_sha256=sha(bag),server_health=source['server_health'],
        scope='Actual tagged NN→gravity bridge adapter and inverse legacy shadow→two fresh installed EKFs with timestamped TF. Separate 30Hz schedulers are nearest-time paired, not byte-identical.',
        limitations=['No target4090/LAN/planner or compound native component GT certification.',
                     'Distinct IDs from independent nodes are not cross-run ground-truth identity switches.',
                     'Map-CV future vertical offset and live old-planner-cache migration are not certified.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(counts=report['counts'],adapter=report['distributions'].get('adapter/max_axis_center_error_m'),
                          tracker_z=report['distributions'].get('tracker/z_shift_minus_half_filtered_height_m')),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    analyze(parser.parse_args())
