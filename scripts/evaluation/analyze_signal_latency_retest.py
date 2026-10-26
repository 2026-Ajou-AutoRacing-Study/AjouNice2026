#!/usr/bin/env python3
"""Read-only comparison of closed signal trials: cadence, freshness and host load.

Header age is NOT pure processing time or MORAI generation age. No causal claim
is made from one run per mission. No commands, model inference or ROS graph edits.
"""
import argparse
from collections import Counter, defaultdict
import csv
import datetime
import json
import math
from pathlib import Path
import statistics

import yaml

TOPICS = ('/Ego_topic', '/sensor/gps', '/sensor/imu',
    '/localization/kinematic_state', '/localization/control_kinematic_state',
    '/planning/trajectory', '/planning/validation_status', '/control/command/control_cmd',
    '/control/command/steering', '/ctrl_cmd', '/purepursuit_autoware/debug/lateral_error',
    '/traffic_light_node/debug_runtime')


def stats(values):
    values = sorted(v for v in values if type(v) in (int,float) and math.isfinite(v))
    if not values:
        return dict(count=0)
    def q(p):
        index=(len(values)-1)*p; low=int(index); high=min(low+1,len(values)-1)
        return values[low]+(values[high]-values[low])*(index-low)
    return dict(count=len(values), mean=statistics.mean(values), p50=q(.5), p95=q(.95),
                p99=q(.99), max=values[-1], min=values[0])


def stamp_ns(message):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is None:
        stamp=getattr(message,'stamp',None)
    if stamp is None:
        return None
    if hasattr(stamp,'to_nsec'):
        value=stamp.to_nsec()
    else:
        sec=getattr(stamp,'sec',getattr(stamp,'secs',None))
        ns=getattr(stamp,'nanosec',getattr(stamp,'nsecs',None))
        value=sec*10**9+ns if sec is not None and ns is not None else None
    return value if value is not None and value > 0 else None


def phase_window(events):
    def ns(value):
        return round(datetime.datetime.fromisoformat(value.replace('Z','+00:00')).timestamp()*1e9)
    start=next((ns(r['time']) for r in events if r.get('event')=='CONTROLLER_START'),None)
    end=next((ns(r['time']) for r in events if r.get('event') in ('RUN_FINISHED','CASE_RESULT')
              and (start is None or ns(r['time']) >= start)),None)
    return start,end


def analyze_trial(root, mission, host):
    import rosbag
    path=root/(mission+'.log')
    if not path.exists():
        return dict(mission=mission, status='not_run')
    log=path.read_text()
    paths=[line.split('캠페인:',1)[1].split('|',1)[0].strip() for line in log.splitlines() if '캠페인:' in line]
    if len(paths)!=1:
        return dict(mission=mission, status='no_closed_campaign_result')
    campaign=Path(paths[0])
    result=list(csv.DictReader((campaign/'summary.csv').open()))[0]
    cases=list(campaign.glob('*/*/events.jsonl'))
    if len(cases)!=1:
        raise ValueError('One case required')
    case=cases[0].parent
    events=[json.loads(s) for s in cases[0].read_text().splitlines()]
    start,end=phase_window(events)
    output=dict(mission=mission, campaign=str(campaign), result=result,
                controller_window_ns=[start,end], scope='controller-running only, excludes preparation and parking')
    if start is None or end is None:
        output['timing_status']='no_valid_controller_window'
        return output
    bags=list(case.glob('*.bag'))
    if len(bags)!=1: raise ValueError('One closed bag required')
    receipts=defaultdict(list); ages=defaultdict(list); source_stamps=defaultdict(list)
    traffic=defaultdict(list); labels=Counter(); violations=Counter(); diag=Counter(); lateral=[]
    processing=defaultdict(lambda:defaultdict(list))
    with rosbag.Bag(str(bags[0])) as bag:
        for topic,msg,t in bag.read_messages():
            now=t.to_nsec()
            if not start <= now <= end: continue
            if topic in TOPICS:
                receipts[topic].append(now)
                stamp=stamp_ns(msg)
                if stamp is not None:
                    ages[topic].append((now-stamp)/1e6); source_stamps[topic].append(stamp)
            if topic.endswith('/debug/processing_time_ms_diag'):
                diag[topic]+=1
                for row in getattr(msg,'status',[msg]):
                    for value in getattr(row,'values',[]):
                        try:
                            number=float(value.value)
                        except (ValueError,TypeError):
                            continue
                        if math.isfinite(number):processing[topic][value.key].append(number)
            if topic=='/traffic_light_node/debug_runtime':
                body=json.loads(msg.data); decision=body.get('frame_decision') or {}
                labels[(decision.get('selection_status'),decision.get('classifier_known'))]+=1
                for key,value in (body.get('frame_timing') or {}).items():
                    if key.endswith('_sec') and type(value) in (int,float):traffic[key].append(value*1000)
            elif topic=='/rosout' and any(text in msg.msg for text in (
                    'out of route','too far from ego','stale or unavailable','timeout','stale')):
                violations[msg.msg]+=1
            if topic=='/purepursuit_autoware/debug/lateral_error':
                lateral.append(abs(msg.data))
    summaries={}
    for topic in TOPICS:
        times=receipts[topic]
        gaps=[(b-a)/1e6 for a,b in zip(times,times[1:])]
        summaries[topic]=dict(messages=len(times), effective_hz=(len(times)-1)/((times[-1]-times[0])/1e9)
            if len(times)>1 and times[-1]>times[0] else None,
            receipt_gap_ms=stats(gaps), header_age_ms=stats(ages[topic]),
            receipt_gaps_over_500ms=sum(v>500 for v in gaps),
            negative_header_age_count=sum(v<0 for v in ages[topic]),
            header_age_over_500ms=sum(v>500 for v in ages[topic]),
            repeated_source_stamp_count=len(source_stamps[topic])-len(set(source_stamps[topic])))
    window=[r for r in host if start <= r['wall_time_ns'] <= end]
    cpu=[r.get('cpu',{}).get('cpus',{}).get('cpu',{}).get('utilization_percent') for r in window]
    load=[r.get('load',{}).get('load_1m') for r in window]
    memory=[r.get('memory',{}).get('MemAvailable_bytes') for r in window]
    swap=[r.get('memory',{}).get('SwapFree_bytes') for r in window]
    sim=[v.get('cpu_percent') for r in window for v in r.get('processes',{}).values()]
    udp={}
    for key in ('InErrors','RcvbufErrors','SndbufErrors','InDatagrams'):
        values=[r.get('udp',{}).get('delta',{}).get(key) for r in window]
        values=[v for v in values if type(v) is int]
        udp[key]=dict(samples=len(values), cumulative_delta=sum(values) if values else None)
    params=yaml.safe_load((case/'ros_parameters_after_control.yaml').read_text())
    controllers={name:dict(control_rate_hz=row.get('control_rate_hz'),safety=row.get('safety'))
                 for name,row in params.items() if name in ('purepursuit_autoware','longi_pi_autoware')}
    output.update(timing_status='measured', topics=summaries,
        traffic_frame_timing_ms={k:stats(v) for k,v in traffic.items()},
        recorded_traffic_selection_counts={str(k):v for k,v in labels.items()},
        processing_diagnostic_messages=dict(diag), diagnostic_logs=dict(violations),
        published_processing_time_ms={topic:{key:stats(values) for key,values in rows.items()}
                                      for topic,rows in processing.items()},
        absolute_lateral_error_m=stats(lateral), controller_parameters=controllers,
        host=dict(samples=len(window), cpu_busy_percent=stats(cpu), load_1m=stats(load),
                  memory_available_bytes=stats(memory), swap_free_bytes=stats(swap),
                  simulator_cpu_percent_one_core=stats(sim), global_udp_deltas=udp),
        safety=json.loads((case/'safety_receipt.json').read_text()).get('shutdown'),
        trajectory_freshness_thresholds_note='Compare source/receipt ages with actual recorded controller params; not a pure computation benchmark')
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    host=[json.loads(s) for s in (a.input/'host.jsonl').read_text().splitlines()]
    metadata=next(r for r in host if r['record_type']=='metadata')
    host=[r for r in host if r['record_type']=='sample']
    report=dict(scope='one local natural trial per mission, production DEBUG off; no causal certification',
        limitations=['Ego UDP stamps are receiver-side, not MORAI physical generation timestamps.',
                     'Header age includes timestamp semantics/transport/queueing, not pure node compute.',
                     'Unpublished processing diagnostics remain missing, never zero.',
                     'One run per group cannot separate group from order/signal phase/host load.'],
        host_metadata=metadata, trials=[analyze_trial(a.input,m,host) for m in 'ABCD'])
    with a.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False)
    print(json.dumps([dict(mission=r['mission'],status=r.get('result',{}).get('status'),
        timing=r.get('timing_status')) for r in report['trials']]))


if __name__=='__main__':main()
