#!/usr/bin/env python3
"""Read-only CLOSED-bag near-retry observations, not accuracy or driving PASS.

Selection/runtime/classifier joins require the exact source-image nanosecond
stamp AND mission group. Pose and route context use explicitly labelled bag-time
as-of samples, never a substitute source timestamp. No ROS nodes, GPU/model calls,
simulator control or external processes are started. Only a fresh artifact JSON
is written; the input bag is opened read-only and must remain unchanged.
"""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]
MISSION_NAMES = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}
DEFAULT_NAMESPACE = '/traffic_light_node'
DEFAULT_ROUTE = '/perception/traffic_light_recognition/route_group_id'
DEFAULT_LOCALIZATION = '/localization/kinematic_state'
DEFAULT_EGO = '/Ego_topic'


def finite(value):
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


def distribution(values):
    values = sorted(v for v in values if finite(v) is not None)
    if not values:
        return {'n': 0}
    return dict(n=len(values), mean=statistics.mean(values), p50=statistics.median(values),
                p95=values[math.ceil(.95 * len(values))-1], min=values[0], max=values[-1])


def frame_key(payload, runtime=False):
    if not isinstance(payload, dict):
        return None
    decision = payload.get('frame_decision')
    group = decision.get('group_id') if runtime and isinstance(decision, dict) else (
        None if runtime else payload.get('group_id'))
    stamp = payload.get('source_stamp_ns')
    if type(stamp) is int and stamp > 0 and type(group) is int and group >= 0:
        return stamp, group
    return None


def _asof(rows, elapsed, max_age=None):
    if not rows:
        return None
    index = bisect_right(rows['times'], elapsed) - 1
    if index < 0:
        return None
    row = rows['rows'][index]
    if max_age is not None and elapsed-row['t'] > max_age:
        return None
    return row


def _timeline(rows):
    ordered = sorted(rows, key=lambda row: row['t'])
    return {'rows': ordered, 'times': [row['t'] for row in ordered]}


def _gaps(rows, clock):
    values = [row.get(clock) for row in rows]
    valid = [v for v in values if finite(v) is not None]
    deltas = [(b-a)/(1e9 if clock == 'source_stamp_ns' else 1.) for a, b in zip(valid, valid[1:])]
    return dict(interval_sec=distribution([d for d in deltas if d >= 0]),
                negative_intervals=sum(d < 0 for d in deltas), zero_intervals=sum(d == 0 for d in deltas),
                missing_clock_samples=len(values)-len(valid))


def analyze_records(records, long_missing_sec=1., max_source_gap_sec=.75,
                    stop_speed_mps=.3, restart_speed_mps=1., min_stop_sec=1.,
                    route_max_age_sec=2., trace_step_sec=.5, max_trace_points=120):
    """Pure analysis entry point for normalized events; safe for synthetic tests.

    Each event has ``kind`` and bag-relative ``t``. Debug events additionally have
    ``payload``; pose events have speed_mps/x/y/source_stamp_ns, route has group_id.
    Missing-run lengths describe recorded observations only, not unseen images.
    """
    records = sorted(records, key=lambda row: row['t'])
    by_kind = defaultdict(list)
    for row in records:
        by_kind[row['kind']].append(row)
    route = _timeline(by_kind['route'])
    route_transitions, previous = [], object()
    for row in by_kind['route']:
        if row.get('group_id') != previous:
            route_transitions.append(dict(t=row['t'], group_id=row.get('group_id')))
            previous = row.get('group_id')

    indexes = {}
    join_quality = {}
    for kind in ('selection', 'runtime', 'classifier'):
        index = defaultdict(list)
        invalid = 0
        for row in by_kind[kind]:
            key = frame_key(row.get('payload'), runtime=kind == 'runtime')
            if key is None:
                invalid += 1
            else:
                index[key].append(row)
        indexes[kind] = index
        join_quality[kind] = dict(messages=len(by_kind[kind]), invalid_source_or_group=invalid,
            distinct_valid_keys=len(index), duplicate_keys=sum(len(rows) > 1 for rows in index.values()),
            duplicate_messages=sum(max(0, len(rows)-1) for rows in index.values()))
    camera_stamps = {row.get('source_stamp_ns') for row in by_kind['camera']
                     if type(row.get('source_stamp_ns')) is int and row['source_stamp_ns'] > 0}
    frames, group_counts = [], defaultdict(Counter)
    first_reasons, final_reasons, heads, classifier_reasons = (defaultdict(Counter) for _ in range(4))
    timing_values = defaultdict(lambda: defaultdict(list))
    timing_by_group = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    invalid_timing = Counter()
    for key, selections in indexes['selection'].items():
        # Do not silently choose among duplicate frame diagnostics.
        if len(selections) != 1:
            continue
        row = selections[0]
        payload = row['payload']
        group = key[1]
        rescue = payload.get('near_rescue')
        rescue = rescue if isinstance(rescue, dict) else {}
        attempted, accepted = rescue.get('attempted') is True, rescue.get('accepted') is True
        count = group_counts[group]
        count['selection_frames'] += 1
        count['matched' if payload.get('status') == 'matched' else 'missing_or_other'] += 1
        count['retry_attempted'] += attempted
        count['retry_accepted'] += attempted and accepted
        count['retry_rejected_or_error'] += attempted and not accepted
        count['retry_errors'] += attempted and bool(rescue.get('error_type'))
        first_reasons[group][str(rescue.get('primary_reason', payload.get('reason')))] += 1
        final_reasons[group][str(payload.get('reason'))] += 1
        if payload.get('status') == 'matched':
            heads[group][str(payload.get('head_id'))] += 1
        joined = {}
        for kind in ('runtime', 'classifier'):
            matches = indexes[kind].get(key, [])
            joined[kind] = matches[0] if len(matches) == 1 else None
            count[kind+'_exact_unique_joined'] += joined[kind] is not None
        camera_present = key[0] in camera_stamps
        count['camera_header_exact_joined'] += camera_present
        if joined['classifier']:
            classification = joined['classifier']['payload']
            count['classifier_known'] += classification.get('known') is True
            classifier_reasons[group][str(classification.get('reason'))] += 1
        frame = dict(t=row['t'], source_stamp_ns=key[0], group_id=group,
            status=payload.get('status'), reason=payload.get('reason'), head_id=payload.get('head_id'),
            retry_attempted=attempted, retry_accepted=attempted and accepted,
            runtime_exact_joined=joined['runtime'] is not None,
            classifier_exact_joined=joined['classifier'] is not None,
            camera_header_exact_joined=camera_present,
            classifier_known=(joined['classifier']['payload'].get('known') is True) if joined['classifier'] else None)
        route_row = _asof(route, row['t'], route_max_age_sec)
        frame['asof_route_group_id'] = route_row.get('group_id') if route_row else None
        frames.append(frame)
        if joined['runtime']:
            runtime = joined['runtime']['payload']
            category = ('retry_frames' if attempted else 'primary_matched_frames'
                        if payload.get('status') == 'matched' else 'primary_missing_or_other_frames')
            timing = runtime.get('frame_timing')
            for field, value in (timing if isinstance(timing, dict) else {}).items():
                if field.endswith('_sec'):
                    value = finite(value)
                    if value is None or value < 0:
                        invalid_timing[field] += 1
                    else:
                        timing_values[category][field[:-4]+'_ms'].append(value*1000)
                        timing_by_group[group][category][field[:-4]+'_ms'].append(value*1000)
    frames.sort(key=lambda row: row['t'])
    for kind in ('runtime', 'classifier'):
        join_quality[kind]['valid_keys_without_selection'] = sum(
            key not in indexes['selection'] for key in indexes[kind])

    runs, active = [], None
    def close_run(reason, recovery=None):
        nonlocal active
        if active is None:
            return
        first, last = active['frames'][0], active['frames'][-1]
        item = dict(group_id=first['group_id'], first_t=first['t'], last_t=last['t'],
            first_source_stamp_ns=first['source_stamp_ns'], last_source_stamp_ns=last['source_stamp_ns'],
            frames=len(active['frames']), observed_missing_source_span_sec=(last['source_stamp_ns']-first['source_stamp_ns'])/1e9,
            close_reason=reason, reasons=dict(Counter(frame['reason'] for frame in active['frames'])),
            exact_joins={name: sum(frame[name] is True for frame in active['frames']) for name in
                         ('runtime_exact_joined', 'classifier_exact_joined', 'camera_header_exact_joined')},
            retry_attempted=sum(frame['retry_attempted'] for frame in active['frames']),
            retry_accepted=sum(frame['retry_accepted'] for frame in active['frames']))
        if recovery is not None:
            item.update(recovered_t=recovery['t'], recovered_source_stamp_ns=recovery['source_stamp_ns'],
                missing_start_to_recovery_source_sec=(recovery['source_stamp_ns']-first['source_stamp_ns'])/1e9)
        runs.append(item)
        active = None
    previous = None
    for frame in frames:
        continuous = (previous is not None and frame['group_id'] == previous['group_id']
            and 0 < (frame['source_stamp_ns']-previous['source_stamp_ns'])/1e9 <= max_source_gap_sec)
        if not continuous:
            close_run('group_change_or_source_gap')
        if frame['status'] == 'missing' and frame['group_id'] in MISSION_NAMES:
            if active is None:
                active = {'frames': []}
            active['frames'].append(frame)
        else:
            close_run('recovered_or_nonmissing', recovery=frame if continuous else None)
        previous = frame
    close_run('bag_end')
    long_runs = [run for run in runs if run['observed_missing_source_span_sec'] >= long_missing_sec]

    frame_timeline = _timeline(frames)
    stop_observations = {}
    for kind in ('localization', 'ego'):
        pose_rows = by_kind[kind]
        stops, active = [], []
        def close_stop():
            if active and active[-1]['t']-active[0]['t'] >= min_stop_sec:
                stops.append(list(active))
            active.clear()
        previous_t = None
        for row in pose_rows:
            current_route = _asof(route, row['t'], route_max_age_sec)
            speed = finite(row.get('speed_mps'))
            is_d_stop = (current_route is not None and current_route.get('group_id') == 201326
                         and speed is not None and speed <= stop_speed_mps)
            if previous_t is not None and row['t']-previous_t > max_source_gap_sec:
                close_stop()
            if is_d_stop:
                active.append(row)
            else:
                close_stop()
            previous_t = row['t']
        close_stop()
        observations = []
        for stopped in stops:
            first, last = stopped[0], stopped[-1]
            restart = next((row for row in pose_rows if last['t'] < row['t'] <= last['t']+15.
                            and finite(row.get('speed_mps')) is not None
                            and row['speed_mps'] >= restart_speed_mps), None)
            end = restart['t']+3. if restart else last['t']+5.
            candidates = [row for row in pose_rows if first['t']-2. <= row['t'] <= end]
            trace, previous_trace_t = [], None
            for row in candidates:
                if previous_trace_t is not None and row['t']-previous_trace_t < trace_step_sec:
                    continue
                context = _asof(frame_timeline, row['t'], .5)
                route_row = _asof(route, row['t'], route_max_age_sec)
                trace.append(dict(t=row['t'], source_stamp_ns=row.get('source_stamp_ns'),
                    speed_mps=row.get('speed_mps'), x=row.get('x'), y=row.get('y'),
                    asof_route_group_id=route_row.get('group_id') if route_row else None,
                    asof_selection=context))
                previous_trace_t = row['t']
            if len(trace) > max_trace_points:
                trace = [trace[round(i*(len(trace)-1)/(max_trace_points-1))] for i in range(max_trace_points)]
            interval_frames = [frame for frame in frames if frame['group_id'] == 201326
                               and first['t'] <= frame['t'] <= last['t']]
            observations.append(dict(first_stopped_t=first['t'], last_stopped_t=last['t'],
                observed_stop_span_sec=last['t']-first['t'], stopped_pose_samples=len(stopped),
                first_restart_speed_t=restart['t'] if restart else None,
                last_stop_to_restart_sec=restart['t']-last['t'] if restart else None,
                stopped_selection_frames=len(interval_frames),
                stopped_missing_frames=sum(frame['status'] == 'missing' for frame in interval_frames),
                stopped_retry_attempts=sum(frame['retry_attempted'] for frame in interval_frames),
                stopped_retry_accepted=sum(frame['retry_accepted'] for frame in interval_frames),
                trace=trace))
        stop_observations[kind] = observations

    gap_summary = {}
    for kind, rows in by_kind.items():
        normalized = [dict(row, source_stamp_ns=(row.get('payload') or {}).get('source_stamp_ns',
                           row.get('source_stamp_ns'))) for row in rows]
        gap_summary[kind] = dict(bag_clock=_gaps(normalized, 't'),
                                 source_clock=_gaps(normalized, 'source_stamp_ns'))
    return dict(schema='traffic_light_near_retry_observations_v1',
        groups={str(group): dict(mission=MISSION_NAMES.get(group), counts=dict(count),
            retry_acceptance_percent=100*count['retry_accepted']/count['retry_attempted'] if count['retry_attempted'] else None,
            final_matched_percent=100*count['matched']/count['selection_frames'] if count['selection_frames'] else None,
            first_selection_reasons=dict(first_reasons[group]), final_selection_reasons=dict(final_reasons[group]),
            selected_heads=dict(heads[group]), classifier_reasons=dict(classifier_reasons[group]))
            for group, count in sorted(group_counts.items())},
        exact_source_and_group_join_quality=join_quality, route_transitions=route_transitions,
        timing_ms={category: {field: distribution(values) for field, values in fields.items()}
                   for category, fields in timing_values.items()},
        timing_ms_by_group={str(group): {category: {field: distribution(values) for field, values in fields.items()}
                           for category, fields in categories.items()} for group, categories in timing_by_group.items()},
        invalid_or_negative_timing_counts=dict(invalid_timing), gaps=gap_summary,
        missing_runs=dict(total=len(runs), long_threshold_sec=long_missing_sec,
            continuity_max_source_gap_sec=max_source_gap_sec, long_count=len(long_runs),
            longest_runs=sorted(runs, key=lambda row: row['observed_missing_source_span_sec'], reverse=True)[:30]),
        D_stop_and_restart_observations=dict(stop_speed_mps=stop_speed_mps,
            restart_speed_mps=restart_speed_mps, min_observed_stop_sec=min_stop_sec,
            route_context='bag-time as-of, max age {} sec'.format(route_max_age_sec),
            selection_trace_context='bag-time as-of, max age 0.5 sec; not an exact source join',
            by_pose_source=stop_observations),
        limitations=[
            'No native signal state, independent visibility/identity GT or signal compliance assessment.',
            'Retry accepted means ordinary association accepted a head, not a correct signal or a driving PASS.',
            'All route-active observations include distant/out-of-view approaches and legitimate stops.',
            'Source joins require exact image stamp AND group; duplicate keys are reported and excluded from joins.',
            'Missing-run spans cover recorded selection observations; gaps are split, not filled with assumed failures.',
            'No camera messages means camera-header crossjoins are unavailable, not camera dropout.',
            'D dwell/restart uses measured speeds and as-of route context; it does not infer why a vehicle stopped or permission to GO.',
            'Near-retry time is included in selection time, not an extra additive stage; callback wall time is not full-stack latency.'])


def _stamp_ns(message):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    return int(stamp.to_nsec()) if stamp is not None else None


def _pose_event(message, kind, elapsed):
    if kind == 'localization':
        position, velocity = message.pose.pose.position, message.twist.twist.linear
    else:
        position, velocity = message.position, message.velocity
    vx, vy = finite(velocity.x), finite(velocity.y)
    return dict(kind=kind, t=elapsed, source_stamp_ns=_stamp_ns(message),
        x=finite(position.x), y=finite(position.y),
        speed_mps=math.hypot(vx, vy) if vx is not None and vy is not None else None)


def summarize(path, namespace=DEFAULT_NAMESPACE, route_topic=DEFAULT_ROUTE,
              localization_topic=DEFAULT_LOCALIZATION, ego_topic=DEFAULT_EGO, camera_topic=None, **kwargs):
    """Open an indexed, completed .bag read-only; reject active/changed evidence."""
    import rosbag
    path = Path(path).resolve(strict=True)
    if not path.is_file() or path.suffix != '.bag' or Path(str(path)+'.active').exists():
        raise ValueError('Use a completed .bag, never a live/.active bag')
    before = path.stat()
    debug_topics = {namespace+'/'+suffix: kind for suffix, kind in (
        ('debug_selection', 'selection'), ('debug_runtime', 'runtime'), ('debug_classifier', 'classifier'))}
    topics = list(debug_topics) + [route_topic, localization_topic, ego_topic]
    if camera_topic:
        topics.append(camera_topic)
    records, malformed = [], Counter()
    with rosbag.Bag(str(path), mode='r', allow_unindexed=False) as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        available = set(bag.get_type_and_topic_info().topics)
        for topic, message, stamp in bag.read_messages(topics=topics):
            elapsed = stamp.to_sec()-start
            try:
                if topic in debug_topics:
                    payload = json.loads(message.data)
                    if not isinstance(payload, dict):
                        raise ValueError('Debug payload is not an object')
                    records.append(dict(kind=debug_topics[topic], t=elapsed, payload=payload))
                elif topic == route_topic:
                    records.append(dict(kind='route', t=elapsed, group_id=int(message.data)))
                elif topic in (localization_topic, ego_topic):
                    records.append(_pose_event(message, 'localization' if topic == localization_topic else 'ego', elapsed))
                else:
                    records.append(dict(kind='camera', t=elapsed, source_stamp_ns=_stamp_ns(message)))
            except (AttributeError, TypeError, ValueError, OverflowError):
                malformed[topic] += 1
    after = path.stat()
    identity = lambda stat: (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)
    if identity(before) != identity(after):
        raise ValueError('Input bag changed during analysis; no result may be published')
    result = analyze_records(records, **kwargs)
    result.update(bag=str(path), bag_start_ros_sec=start, bag_duration_sec=end-start,
        bag_bytes=after.st_size, bag_mtime_ns=after.st_mtime_ns,
        analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_topics=dict(requested=topics, present=sorted(set(topics) & available),
                          absent=sorted(set(topics)-available)), malformed_messages=dict(malformed))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirm-closed', action='store_true', required=True,
                        help='Explicitly confirm collection has ended; never inspect the live trial bag')
    parser.add_argument('--namespace', default=DEFAULT_NAMESPACE)
    parser.add_argument('--route-topic', default=DEFAULT_ROUTE)
    parser.add_argument('--localization-topic', default=DEFAULT_LOCALIZATION)
    parser.add_argument('--ego-topic', default=DEFAULT_EGO)
    parser.add_argument('--camera-topic', help='Optional header-only camera join; no image decoding')
    parser.add_argument('--long-missing-sec', type=float, default=1.)
    args = parser.parse_args()
    output = args.output.resolve()
    try:
        output.relative_to(ROOT / 'artifacts/evaluation')
    except ValueError:
        parser.error('Output must be a fresh JSON under artifacts/evaluation')
    if output.exists() or output.suffix != '.json':
        parser.error('Use a fresh .json output; prior evidence is preserved')
    if not math.isfinite(args.long_missing_sec) or args.long_missing_sec <= 0:
        parser.error('--long-missing-sec must be finite and positive')
    result = summarize(args.bag, namespace=args.namespace, route_topic=args.route_topic,
        localization_topic=args.localization_topic, ego_topic=args.ego_topic,
        camera_topic=args.camera_topic, long_missing_sec=args.long_missing_sec)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(str(output))


if __name__ == '__main__':
    main()
