#!/usr/bin/env python3
"""Offline natural-signal trial audit. No ROS/RPC/control and no raw-data edits."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import yaml

from observe_traffic_light_compliance import evaluate_crossing, native_bracket, segment_crossing


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(1024*1024),b''):h.update(data)
    return h.hexdigest()


def distribution(values):
    a=np.asarray([v for v in values if v is not None and np.isfinite(v)],dtype=float)
    if not len(a):return dict(count=0)
    return dict(count=len(a),mean=float(a.mean()),p50=float(np.percentile(a,50)),
                p90=float(np.percentile(a,90)),p95=float(np.percentile(a,95)),
                p99=float(np.percentile(a,99)),maximum=float(a.max()))


def signal_class(group,elements):
    if not elements or any(e['color']==0 or e['shape']==0 or e['status']!=2 for e in elements):return 'unknown'
    if any(e['color']==2 for e in elements):
        return 'unknown' if any(e['color']==3 for e in elements) else 'yellow'
    shape=2 if group==201326 else 1
    return 'authorized' if any(e['color']==3 and e['shape']==shape for e in elements) else 'stop'


def audit(run,campaign):
    observer=run/'observer';raw_path=observer/'observations.jsonl'
    metadata=json.loads((observer/'metadata.json').read_text())
    groups={g['group_id']:g for g in metadata['geometry']['document']['groups']}
    frames=[];native=[];signals=[];controls=[];velocity=[];selections={};counts=Counter()
    for line in raw_path.open():
        r=json.loads(line);topic=r.get('topic',r.get('event'));counts[topic]+=1
        if topic=='native_read_only_sample':native.append(r)
        if topic=='/ctrl_cmd':controls.append(r)
        if topic=='/localization/kinematic_state':velocity.append((r['receipt_ros_ns'],r['message']['twist']['twist']['linear']['x']))
        if topic=='/traffic_light_node/debug_selection':
            d=r['message'].get('parsed_json') or json.loads(r['message']['data'])
            if d.get('source_stamp_ns'):selections[d['source_stamp_ns']]=d
        if topic=='/traffic_light_node/debug_runtime':
            d=r['message'].get('parsed_json') or json.loads(r['message']['data'])
            if d.get('frame_decision'):frames.append(dict(record=r,data=d))
        if topic=='/perception/traffic_light_recognition/traffic_signals':
            m=r['message'];st=m['stamp'];stamp=st['sec']*10**9+st['nanosec']
            for g in m['traffic_light_groups']:
                if g['traffic_light_group_id'] in groups:
                    signals.append(dict(group=g['traffic_light_group_id'],source=stamp,
                                        receipt=r['receipt_ros_ns'],record_id=r['record_id'],
                                        phase=signal_class(g['traffic_light_group_id'],g['elements'])))
    compliance=[];timed=[];go_out=[]
    pattern=re.compile(r'\[(\d+\.\d+)\].*\[signal_compliance\] reg=(\d+) class=(\w+) source_age_sec=([-\d.]+) green_count=(\d+) require_stop=(\d+)')
    for line in (campaign/'stack_logs/planning.log').open(errors='replace'):
        m=pattern.search(line)
        if m:compliance.append(dict(time=float(m[1]),group=int(m[2]),phase=m[3],source_age=float(m[4]),count=int(m[5]),stop=bool(int(m[6]))))
        if '[signal_timed_yellow]'in line:
            m=re.search(r'\[(\d+\.\d+)\].*reg=(\d+) allow=(\d+) reason=(\w+)',line)
            if m:timed.append(dict(time=float(m[1]),group=int(m[2]),allow=bool(int(m[3])),reason=m[4]))
        if '[signal_compliance]'in line and 'state=GO_OUT'in line:go_out.append(line.strip())
    crossings=json.loads((observer/'crossings.json').read_text())['crossings']
    crossing_results=[]
    for c in crossings:
        g=groups[c['group_id']];line=g['stop_line_xy'];a=c['previous_ego']['front_point_map_xyz_m'][:2];b=c['current_ego']['front_point_map_xyz_m'][:2]
        hit=segment_crossing(a,b,line[0],line[1]);point_error=float(np.linalg.norm(np.subtract(hit['point_xy_m'],c['point_xy_m']))) if hit else None
        native_result=evaluate_crossing(c,native,g)
        brackets=c['crossing_jpeg_bracket'];jpeg=[]
        for side in ['before','after']:
            x=brackets[side];jpeg.append(dict(side=side,path=x['path'],sha256_matches=sha(Path(x['path']))==x['sha256'],delta_sec=x['source_delta_to_crossing_sec']))
        crossing_results.append(dict(mission=g['mission'],group=g['group_id'],
                                    crossing_source_ns=c['estimated_source_stamp_ns'],record_id=c['record_id'],
                                    finite_line_crossing_recomputed=hit is not None,
                                    crossing_point_error_m=point_error,native_recomputed=native_result,
                                    original_jpegs=jpeg))
    phase_rows=[]
    for gid,g in groups.items():
        gs=[s for s in signals if s['group']==gid]
        auth=[s for s in gs if s['phase']=='authorized']
        streak=[];first_pair=None;last_source=None
        for s in gs:
            fresh=0<=(s['receipt']-s['source'])/1e9<=.5
            if (not fresh or s['phase']!='authorized' or
                (last_source is not None and (s['source']<last_source or (s['receipt']-last_source)/1e9>.5))):streak=[]
            if fresh and s['phase']=='authorized':
                if not streak or s['source']>streak[-1]['source']:streak.append(s)
                if len(streak)>=2 and first_pair is None:first_pair=streak[-2:]
            last_source=s['source']
        granted=[s for s in compliance if s['group']==gid and s['phase']=='direction_authorized_green' and not s['stop']]
        group_frames=[f for f in frames if f['data']['frame_decision'].get('group_id')==gid]
        matched=[f for f in group_frames if f['data']['frame_decision'].get('selection_status')=='matched' and f['data']['frame_decision'].get('classifier_known')]
        first_source=auth[0]['source'] if auth else None
        cross_time=next(c['estimated_source_stamp_ns'] for c in crossings if c['group_id']==gid)
        pre_velocity=[v for t,v in velocity if first_source is not None and t<=first_source]
        moving_before=bool(pre_velocity and pre_velocity[-1]>.2)
        departure=[r for r in controls if first_source is not None and first_source<=r['receipt_ros_ns']<=cross_time
                   and r['message'].get('accel',0.)>.02 and r['message'].get('brake',1.)<.05]
        movement=[(t,v)for t,v in velocity if first_source is not None and first_source<=t<=cross_time and v>.2]
        phase_rows.append(dict(mission=g['mission'],group=gid,
                               first_canonical_phase=gs[0]['phase'] if gs else None,
                               canonical_phase_counts=dict(Counter(s['phase'] for s in gs)),
                               first_authorized_source_ns=auth[0]['source'] if auth else None,
                               first_two_distinct_canonical_authorized_sources=first_pair,
                               first_cpp_authorized_log=granted[0] if granted else None,
                               first_source_to_first_logged_cpp_authorization_sec=(granted[0]['time']-auth[0]['source']/1e9) if granted and auth else None,
                               already_moving_at_first_authorized_source=moving_before,
                               first_source_to_positive_control_sec=(departure[0]['receipt_ros_ns']-first_source)/1e9 if departure and not moving_before else None,
                               first_source_to_movement_over_point2mps_sec=(movement[0][0]-first_source)/1e9 if movement and not moving_before else None,
                               departure_caveat='Control/movement timestamps are observed after authorization, not causal proof that all delay is the signal guard.',
                               timing_caveat='C++ logs are throttled; this is first logged grant, not exact actual grant time. Canonical pair proves available publications, not actual consumption.',
                               matched_known_frames=len(matched),
                               callback_ms=distribution([1000*f['data']['frame_timing']['callback_to_output_sec'] for f in matched])))
    nearline_stop_vs_native=[]
    for f in frames:
        d=f['data'];dec=d['frame_decision'];gid=dec.get('group_id')
        if gid not in groups:continue
        c=next(c for c in crossings if c['group_id']==gid)
        st=d['source_stamp_ns']
        if not c['estimated_source_stamp_ns']-3*10**9<st<c['estimated_source_stamp_ns']:continue
        final=dec.get('final_tuples')or[]
        if not final or any(x[0]=='green' and (gid!=201326 or x[1]=='arrow' and x[2] is True) for x in final):continue
        r=f['record'];mono=r['receipt_monotonic']-(r['receipt_ros_ns']-st)/1e9
        brackets=[native_bracket(native,nid,mono,.25) for nid in groups[gid]['native_ids']]
        bit=32 if gid==201326 else 16
        if all(b['verified'] and b['color']&bit for b in brackets):
            nearline_stop_vs_native.append(dict(mission=groups[gid]['mission'],record_id=r['record_id'],source_ns=st,
                                                seconds_before_crossing=(c['estimated_source_stamp_ns']-st)/1e9,
                                                selection_status=dec.get('selection_status'),classifier_known=dec.get('classifier_known'),
                                                selection_reason=selections.get(st,{}).get('reason'),
                                                near_rescue=selections.get(st,{}).get('near_rescue'),
                                                observed=dec.get('observed_tuples'),final=final,origin=dec.get('observation_timing',{}).get('output_origin'),
                                                native=brackets))
    matched=[f for f in frames if f['data']['frame_decision'].get('selection_status')=='matched' and f['data']['frame_decision'].get('classifier_known')]
    selection=json.loads((run/'selection.json').read_text())
    config=yaml.safe_load((campaign/'config_snapshot.yaml').read_text())
    return dict(scope='One completed natural local MORAI full route, actual ROS1 planner/controller and od_gt; not LAN/4090/BEVFusion/official penalty certification',
                native_SET_enabled=selection['native_signal_SET_enabled'],synthetic_signal_proxy=selection['synthetic_canonical_signal_proxy'],
                configured_candidate={k:v for k,v in config['stack'].items() if k.startswith('signal_compliance_') or k=='traffic_light_yellow_passthrough'},
                configured_two_green_proof='Config snapshot and C++ logs; B/C grant log absent due throttle, canonical distinct-source pair only',
                inputs={str(p):sha(p) for p in [raw_path,observer/'crossings.json',campaign/'stack_logs/planning.log',campaign/'config_snapshot.yaml']},
                source_topic_counts=dict(counts),crossings=crossing_results,missions=phase_rows,
                CPP_logged_phase_counts=dict(Counter(s['phase'] for s in compliance)),
                CPP_logged_authorized_green_counts=dict(Counter(str(s['count']) for s in compliance if s['phase']=='direction_authorized_green' and not s['stop'])),
                CPP_invalid_authorization_logs=[s for s in compliance if not s['stop'] and (s['phase']!='direction_authorized_green' or s['count']<2)],
                CPP_timed_decisions=timed,CPP_timed_allow_count=sum(s['allow']for s in timed),
                no_natural_timed_yellow_exercise=sum(s['allow']for s in timed)==0 and not any(s['phase']=='yellow'for s in compliance),
                CPP_GO_OUT_logs=go_out,
                matched_known_callback_ms=distribution([1000*f['data']['frame_timing']['callback_to_output_sec'] for f in matched]),
                matched_known_header_to_callback_ms=distribution([1000*f['data']['frame_timing']['header_to_callback_sec']for f in matched]),
                near_crossing_canonical_STOP_native_permit_frames=nearline_stop_vs_native,
                limitations=['Counts of adjacent frames/log lines are not independent event rates.',
                             'Native permits do not certify rendered pixel truth; canonical STOP/native permit does not alone locate detector vs classifier vs gate cause.',
                             'No real first-yellow/GO stress evidence in this natural run; controlled trials remain separate.'])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--campaign',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=audit(a.run,a.campaign)
    with a.output.open('x')as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps({k:result[k]for k in ['matched_known_callback_ms','CPP_logged_phase_counts','CPP_logged_authorized_green_counts','CPP_invalid_authorization_logs','CPP_timed_allow_count']},ensure_ascii=False,indent=2))
    print('near-crossing stop/native-permit',len(result['near_crossing_canonical_STOP_native_permit_frames']))
    for r in result['missions']:print(r['mission'],r['first_canonical_phase'],'first source->first logged grant',r['first_source_to_first_logged_cpp_authorization_sec'],'canonical pair',bool(r['first_two_distinct_canonical_authorized_sources']))


if __name__=='__main__':main()
