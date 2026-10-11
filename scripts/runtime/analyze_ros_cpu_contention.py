#!/usr/bin/env python3
"""Summarize bounded CPU probes and audit recorded input gaps. No GPU writes."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    names=('idle_mixed','stack_mixed','traffic_mixed','stress_mixed','idle_reverse',
           'cpu_only','obstacle_probe','roundabout_probe','roundabout_probe_v2')
    cases={}
    for name in names:
        path=args.root/name/'report.json'
        if not path.exists():continue
        report=json.loads(path.read_text())
        if not report['valid']:raise ValueError('Inactive stack: '+name)
        timing=report['inference']['timing']['all']['timing'] if report['inference'] else None
        plan_log=args.root/name/'planning.log'
        log=plan_log.read_text(errors='replace') if plan_log.exists() else ''
        cases[name]=dict(cpu_mean=report['resource']['total_cpu_percent']['mean'],
            cpu_p95=report['resource']['total_cpu_percent']['p95'],timing=timing,
            topic_rates={key:row['rate_hz'] for key,row in report['topics'].items()},
            control_interval=report['topics'].get('control',{}).get('wall_interval_ms'),
            trajectory_interval=report['topics'].get('trajectory',{}).get('wall_interval_ms'),
            prediction_actor_count=report['topics'].get('prediction',{}).get('object_or_point_count'),
            planning_invalid_log_count=log.count('Caution! Invalid Trajectory published.'),
            planning_latency_log_count=log.count('planning component latency is larger than threshold'),
            resource_groups=report['resource']['groups'],
            report_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    selected=[];baseline=[]
    for name in ('idle_mixed','stack_mixed','traffic_mixed','stress_mixed','idle_reverse'):
        path=args.root/name/'gpu/report.json'
        if not path.exists():raise ValueError('Missing matched case: '+name)
        report=json.loads(path.read_text());selected.append(report['source']['selection_sha256'])
        if name.startswith('idle'):
            baseline.extend(row['rtt_ms'] for row in report['cases'][0]['per_sample'])
    if len(set(selected))!=1:raise ValueError('Matched probe inputs differ')
    input_audit={}
    highway=json.loads((args.root/'stack_mixed/report.json').read_text())['bag']
    with rosbag.Bag(highway) as bag:
        start=bag.get_start_time()
        topics=[topic for topic in bag.get_type_and_topic_info().topics if topic=='/Object_topic'
                or 'lidar' in topic or 'camera/front/compressed' in topic]
        for topic in topics:
            times=[stamp.to_sec()-start for _,_,stamp in bag.read_messages(topics=[topic])]
            window=[stamp for stamp in times if 25<=stamp<=41]
            intervals=np.diff(window)*1000
            input_audit[topic]=dict(count=len(window),p95_interval_ms=float(np.percentile(intervals,95)),
                maximum_interval_ms=float(max(intervals)),
                gaps_over_300ms=[dict(at_bag_seconds=x,interval_ms=1000*(y-x))
                                for x,y in zip(times,times[1:]) if y-x>.3])
    failures={path.parent.name:json.loads(path.read_text())
              for path in args.root.glob('*/failure.json')}
    result=dict(cases=cases,matched_selection_sha256=selected[0],
        baseline=dict(requests=len(baseline),distinct_frames_per_repeat=96,
            rtt_mean_ms=float(np.mean(baseline)),rtt_p95_ms=float(np.percentile(baseline,95))),
        recorded_highway_input_audit=input_audit,excluded_failed_probes=failures,
        scope='CPU contention only, not detector AP, closed-loop safety or a 4090 13Hz deployment certificate')
    (args.root/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print('case | CPU mean % | RTT mean / p95 ms | control / planning Hz')
    for name,row in cases.items():
        timing=row['timing']['rtt_ms'] if row['timing'] else None
        rates=row['topic_rates']
        print(name,round(row['cpu_mean'],2),
              (round(timing['mean'],2),round(timing['p95'],2)) if timing else None,
              (round(rates.get('control',0),2),round(rates.get('trajectory',0),2)))
    print('Saved',args.root/'summary.json')


if __name__=='__main__':main()
