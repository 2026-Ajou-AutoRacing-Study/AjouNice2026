#!/usr/bin/env python3
"""Read-only signal/brake correlation audit, not a causal or safety certificate.

Records receipt and source clocks separately. It never publishes ROS data,
starts a simulator, alters the input bag, or infers a map version from a filename.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics


MISSIONS = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}
SIGNAL_TOPIC = '/perception/traffic_light_recognition/traffic_signals'


def permission(lamps, group):
    """Direction permission proxy; UNKNOWN is not actual planner STOP proof."""
    if not lamps or any(l['color'] == 'UNKNOWN' or l['shape'] == 'UNKNOWN' or
                        l['status'] != 'SOLID_ON' for l in lamps):
        return 'UNKNOWN'
    if any(l['color'] == 'AMBER' for l in lamps):
        return 'YELLOW'
    if group == 201326:
        return 'GO' if any(l['color'] == 'GREEN' and l['shape'] == 'LEFT_ARROW'
                           for l in lamps) else 'STOP'
    if group not in MISSIONS:
        return 'NONMISSION'
    return 'GO' if any(l['color'] == 'GREEN' and l['shape'] == 'CIRCLE'
                       for l in lamps) else 'STOP'


def closest(rows, t, tolerance=.15):
    if not rows:
        return None
    # Python3.8 ROS Noetic has no bisect key= argument.
    low, high = 0, len(rows)
    while low < high:
        mid=(low+high)//2
        if rows[mid]['t'] < t:low=mid+1
        else:high=mid
    index=low
    candidates = rows[max(0, index-1):index+1]
    row = min(candidates, key=lambda row: abs(row['t']-t))
    return row if abs(row['t']-t) <= tolerance else None


def unknown_gaps(signals, max_stream_gap=.5):
    """Group consecutive UNKNOWN receipts; a missing stream is not UNKNOWN."""
    by_group = {}
    for row in signals:
        if row['group'] in MISSIONS:
            by_group.setdefault(row['group'], []).append(row)
    result = []
    for group, rows in by_group.items():
        index = 0
        while index < len(rows):
            if rows[index]['phase'] != 'UNKNOWN':
                index += 1
                continue
            first = index
            while (index+1 < len(rows) and rows[index+1]['phase'] == 'UNKNOWN'
                   and rows[index+1]['t']-rows[index]['t'] <= max_stream_gap):
                index += 1
            last = index
            before = rows[first-1] if first else None
            after = rows[last+1] if last+1 < len(rows) else None
            bracketed = bool(before and after and before['phase'] == after['phase'] == 'GO'
                             and rows[first]['t']-before['t'] <= max_stream_gap
                             and after['t']-rows[last]['t'] <= max_stream_gap)
            result.append(dict(group=group, mission=MISSIONS[group], start=rows[first]['t'],
                               end=rows[last]['t'], frames=last-first+1,
                               unknown_receipt_span_sec=rows[last]['t']-rows[first]['t'],
                               go_to_go_receipt_gap_sec=after['t']-before['t'] if bracketed else None,
                               bracketed_by_go=bracketed))
            index += 1
    return sorted(result, key=lambda row: row['start'])


def brake_episodes(brakes, speeds, threshold=.05, min_speed=.3, max_gap=.3):
    active = []
    for row in brakes:
        speed = closest(speeds, row['t'])
        if (not math.isfinite(row['brake']) or row['brake'] <= threshold or speed is None
                or not math.isfinite(speed['speed']) or speed['speed'] < min_speed):
            continue
        if not active or row['t']-active[-1][-1]['t'] > max_gap:
            active.append([])
        active[-1].append(dict(row, speed=speed['speed']))
    return [dict(start=rows[0]['t'], end=rows[-1]['t'], frames=len(rows),
                 peak_brake=max(row['brake'] for row in rows),
                 start_speed=rows[0]['speed'], end_speed=rows[-1]['speed']) for rows in active]


def enum_label(item, field, labels):
    value = getattr(item, field, None)
    return next((name for name in labels if getattr(item, name, object()) == value), str(value))


def tuple_lamps(tuples):
    colors={'red':'RED','yellow':'AMBER','green':'GREEN'}
    return [dict(color=colors.get(color,'UNKNOWN'),
                 shape='CIRCLE' if shape=='circle' else 'LEFT_ARROW' if left is True
                       else 'RIGHT_ARROW' if left is False else 'UNKNOWN',status='SOLID_ON')
            for color,shape,left in tuples]


def stamp_seconds(message):
    return message.to_sec() if hasattr(message, 'to_sec') else message.sec+message.nanosec/1e9


def distribution(values):
    values=sorted(v for v in values if math.isfinite(v))
    if not values:return dict(n=0)
    return dict(n=len(values),mean=statistics.mean(values),p50=statistics.median(values),
                p95=values[math.ceil(.95*len(values))-1],max=values[-1])


def audit(path):
    import rosbag
    signals, brakes, speeds, selected, runtime, observations = [], [], [], [], [], []
    targets,factors,controller_invalid=[],[],[]
    map_metadata, rosout_clues, parse_errors = [], [], Counter()
    with rosbag.Bag(str(path)) as bag:
        begin, end = bag.get_start_time(), bag.get_end_time()
        info = bag.get_type_and_topic_info().topics
        topics = {name: dict(type=row.msg_type, messages=row.message_count,
                            frequency=row.frequency) for name, row in info.items()}
        stage_topics=('/planning/trajectory',
            '/planning/scenario_planning/lane_driving/behavior_planning/path',
            '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory',
            '/planning/scenario_planning/lane_driving/trajectory')
        wanted = [name for name in info if name in (
            SIGNAL_TOPIC, '/ctrl_cmd', '/localization/kinematic_state',
            '/traffic_light_node/debug_runtime', '/traffic_light_node/debug_selection',
            '/rosout', '/rosout_agg', '/map/vector_map','/control/autoware/failure_reason')
            or name in stage_topics or name.startswith('/planning/planning_factors/')]
        for topic, message, receipt in bag.read_messages(topics=wanted):
            t = receipt.to_sec()-begin
            if topic == SIGNAL_TOPIC:
                source = stamp_seconds(message.stamp)
                for group in message.traffic_light_groups:
                    lamps = [dict(color=enum_label(item, 'color', ('RED','AMBER','GREEN','WHITE','UNKNOWN')),
                                  shape=enum_label(item, 'shape', ('CIRCLE','LEFT_ARROW','RIGHT_ARROW','UNKNOWN')),
                                  status=enum_label(item, 'status', ('SOLID_OFF','SOLID_ON','FLASHING','UNKNOWN')))
                             for item in group.elements]
                    group_id = int(group.traffic_light_group_id)
                    signals.append(dict(t=t, source=source-begin, age_sec=receipt.to_sec()-source,
                                        group=group_id, lamps=lamps, phase=permission(lamps,group_id)))
            elif topic == '/ctrl_cmd':
                brakes.append(dict(t=t, brake=float(message.brake), accel=float(message.accel)))
            elif topic == '/localization/kinematic_state':
                v=message.twist.twist.linear
                p=message.pose.pose.position;c=message.pose.covariance
                speeds.append(dict(t=t,source=message.header.stamp.to_sec()-begin,
                    speed=math.hypot(v.x,v.y),xy=[p.x,p.y],z=p.z,
                    covariance_xyz_yaw=[c[i] for i in (0,7,14,35)],
                    position_sigma_m=math.sqrt(max(c[0],c[7])) if min(c[0],c[7])>=0 else None,
                    yaw_sigma_deg=math.degrees(math.sqrt(c[35])) if c[35]>=0 else None))
            elif topic in stage_topics:
                if speeds and t-speeds[-1]['t']<=.15 and message.points:
                    ego=speeds[-1]['xy'];points=message.points
                    index=min(range(len(points)),key=lambda i:(points[i].pose.position.x-ego[0])**2+
                              (points[i].pose.position.y-ego[1])**2)
                    targets.append(dict(t=t,topic=topic,speed=points[index].longitudinal_velocity_mps))
            elif topic.startswith('/planning/planning_factors/'):
                for factor in message.factors:
                    factors.append(dict(t=t,topic=topic,module=factor.module,behavior=factor.behavior,
                                        detail=factor.detail))
            elif topic=='/control/autoware/failure_reason':
                if message.data:controller_invalid.append(dict(t=t,reason=message.data))
            elif topic == '/map/vector_map':
                map_metadata.append(dict(t=t, fields={name:str(getattr(message,name))[:500]
                    for name in ('version_map_format','version_map','name_map') if hasattr(message,name)},
                    binary_sha256=hashlib.sha256(bytes(message.data)).hexdigest()
                    if hasattr(message,'data') else None))
            elif topic in ('/rosout','/rosout_agg'):
                text=message.msg
                if any(key in text.lower() for key in ('signal_compliance','lanelet2_map_v','traffic_light','fail-open')):
                    rosout_clues.append(dict(t=t,node=message.name,text=text))
            else:
                try:
                    body=json.loads(message.data)
                    if topic.endswith('/debug_selection'):
                        selected.append(dict(t=t,group=body.get('group_id'),status=body.get('status'),
                                             reason=body.get('reason'),head=body.get('head_id')))
                    else:
                        if not runtime:
                            runtime.append(dict(t=t,metadata={k:v for k,v in body.items()
                                if k not in ('frame_timing','frame_decision','cuda_memory')}))
                        decision=body.get('frame_decision') or {}
                        if decision:
                            group=decision.get('group_id')
                            observed=decision.get('observed_tuples')
                            if observed is not None:
                                observations.append(dict(t=t,group=group,
                                    phase=permission(tuple_lamps(observed),group),
                                    selection=decision.get('selection_status'),
                                    final=decision.get('final_tuples'),
                                    output_origin=(decision.get('observation_timing') or {}).get('output_origin')))
                except (ValueError, TypeError):
                    parse_errors[topic] += 1
    speeds.sort(key=lambda row:row['t'])
    episodes=brake_episodes(brakes,speeds)
    gaps=unknown_gaps(signals)
    unobserved=unknown_gaps(observations)
    for row in gaps+unobserved:
        row['nearby_moving_brake_episodes']=[e for e in episodes
            if e['start']<=row['end']+.8 and e['end']>=row['start']-.1]
        row['nearby_selection_reasons']=dict(Counter(s['reason'] for s in selected
            if s['group']==row['group'] and row['start']-.15<=s['t']<=row['end']+.15))
    for episode in episodes:
        position=closest(speeds,episode['start'])
        episode['start_xy']=position['xy'] if position else None
        episode['nearby_rosout']=[r for r in rosout_clues
            if episode['start']-.5<=r['t']<=episode['start']+.8]
        episode['nearby_factors']=dict(Counter(r['topic'].rsplit('/',1)[-1]+':'+r['module']
            for r in factors if episode['start']-.2<=r['t']<=episode['start']+.3))
    signal_timing={}
    for group in MISSIONS:
        rows=[row for row in signals if row['group']==group]
        signal_timing[str(group)]=dict(source_age_sec=distribution([r['age_sec'] for r in rows]),
            repeated_source_pairs=sum(a['source']==b['source'] for a,b in zip(rows,rows[1:])),
            source_age_gt_1sec=sum(r['age_sec']>1. for r in rows),
            receipt_interval_sec=distribution([b['t']-a['t'] for a,b in zip(rows,rows[1:])]))
    return dict(schema='traffic_light_hesitation_readonly_v1', bag=str(path),
                duration_sec=end-begin, topics=topics, parse_errors=dict(parse_errors),
                signal_phase_counts=dict(Counter('%s:%s'%(r['group'],r['phase']) for r in signals)),
                signal_timing=signal_timing,
                raw_observation_phase_counts=dict(Counter('%s:%s'%(r['group'],r['phase']) for r in observations)),
                unknown_gaps=gaps,unobserved_gaps=unobserved,moving_brake_episodes=episodes,
                speed_samples=speeds,target_speed_samples=targets,planning_factors=factors,
                controller_invalid=controller_invalid,
                runtime_metadata=runtime,map_metadata=map_metadata,rosout_clues=rosout_clues,
                limitations=['Map version is not inferred from bag filename or current local defaults.',
                    'GO/STOP here is a direction-permission proxy, not proof of planner state.',
                    'Temporal overlap does not prove UNKNOWN caused braking; other modules can brake.',
                    'No independent lamp/native GT or image inference is created by this audit.',
                    'Sparse or absent debug/command topics prevent full root-cause attribution.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    args.output.resolve().relative_to(root/'planner_logs/analysis')
    if args.output.exists():parser.error('Fresh output required; existing evidence is preserved')
    report=audit(args.bag.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2,ensure_ascii=False)
    print(json.dumps(dict(output=str(args.output),duration_sec=report['duration_sec'],
        unknown_gaps=len(report['unknown_gaps']),moving_brake_episodes=len(report['moving_brake_episodes']))))


if __name__=='__main__':main()
