#!/usr/bin/env python3
"""Read-only, bounded-topic summaries of completed natural traffic-light drives.

Inputs may be wrapper, campaign, execution, observer directories or a .bag.
Use repeated --observer DIR (one per input) to pair separately stored observers.
Example: analyze_traffic_light_driving_comparison.py RUN1 RUN2 --output
artifacts/evaluation/traffic_comparison.json

Raw means runtime.frame_decision.observed_tuples (NOT debug_raw_label, which is
post-normalization). Counts describe these drives, not accuracy or causal effect.
No camera, point cloud or other raw sensor payload is decoded by this tool.
"""
import argparse
from bisect import bisect_left
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from analyze_traffic_light_trial_timing import number, payload
from observe_traffic_light_compliance import primitive
from summarize_traffic_light_policy_trial import statistics_of, summarize_safety

ROOT = Path(__file__).resolve().parents[2]
MISSIONS = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}
TOPICS = ['/traffic_light_node/debug_runtime', '/traffic_light_node/debug_selection',
          '/Ego_topic', '/ctrl_cmd', '/planning/trajectory']


def unchanged(path, before):
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('Evidence changed during analysis; close the run first: '+str(path))


def read_json(path, evidence):
    path = Path(path)
    if not path.is_file():
        return {}
    before = path.stat()
    raw = path.read_bytes()
    unchanged(path, before)
    evidence.append(dict(path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest()))
    return json.loads(raw)


def jsonl(path, evidence, warnings):
    """Stream JSONL and pin immutable bytes; never load full observer rows."""
    path = Path(path)
    if not path.is_file():
        return
    before, digest, invalid, lines = path.stat(), hashlib.sha256(), 0, 0
    with path.open('rb') as stream:
        for lines, raw in enumerate(stream, 1):
            digest.update(raw)
            try:
                row = json.loads(raw)
                if not isinstance(row, dict):
                    raise ValueError('Object required')
                yield row
            except (ValueError, UnicodeDecodeError):
                invalid += 1
    unchanged(path, before)
    evidence.append(dict(path=str(path), bytes=before.st_size, sha256=digest.hexdigest(),
                         lines=lines, invalid_lines=invalid))
    if invalid:
        warnings.append('{} malformed JSONL lines in {}'.format(invalid, path))


def find_campaign(path):
    for candidate in (path, *path.parents):
        if (candidate/'execution.json').is_file() and (candidate/'campaign_state.json').is_file():
            return candidate
    return None


def discover(path, evidence, warnings):
    path = Path(path).resolve(strict=True)
    directory = path.parent if path.is_file() else path
    wrapper = directory if (directory/'selection.json').is_file() else None
    if directory.name == 'observer' and (directory.parent/'selection.json').is_file():
        wrapper = directory.parent
    observer = (wrapper/'observer' if wrapper else
                directory if (directory/'observations.jsonl').is_file() else None)
    selection = read_json(wrapper/'selection.json', evidence) if wrapper else {}
    events = list(jsonl(wrapper/'wrapper_events.jsonl', evidence, warnings)) if wrapper else []
    campaigns = set()
    campaign = find_campaign(directory)
    if campaign:
        campaigns.add(campaign)
    for event in events:
        for arg in event.get('command', []):
            if not isinstance(arg, str):
                continue
            value = arg.split(':=', 1)[-1]
            if not value.startswith('/'):
                continue
            candidate = find_campaign(Path(value).parent)
            if candidate:
                campaigns.add(candidate)
    if len(campaigns) > 1:
        raise ValueError('Ambiguous campaign references: '+repr(sorted(map(str, campaigns))))
    campaign = next(iter(campaigns), None)
    if path.is_file() and path.suffix == '.bag':
        bags = [path]
    elif campaign:
        bags = sorted(campaign.glob('*/*/*.bag'))
    else:
        bags = sorted(directory.glob('*.bag'))
    if len(bags) > 1:
        raise ValueError('Input names several bags; pass each explicit .bag instead: '+str(path))
    if any(directory.glob('*.bag.active')) or (campaign and any(campaign.glob('*/*/*.bag.active'))):
        raise ValueError('Active bag exists; completed evidence only')
    if wrapper and not any(row.get('event') == 'validator_returned' for row in events):
        raise ValueError('Wrapper has no validator_returned event; completed evidence only: '+str(wrapper))
    execution = bags[0].parent if bags else directory if (directory/'events.jsonl').is_file() else None
    if not bags:
        warnings.append('No completed bag: planner target analysis unavailable; observer runtime fallback only.')
    return dict(input=path, wrapper=wrapper, observer=observer, campaign=campaign,
                bag=bags[0] if bags else None, execution=execution,
                selection=selection, wrapper_events=events)


def lamp_label(tuples):
    if not isinstance(tuples, list):
        return 'MISSING'
    labels = set()
    for item in tuples:
        if not isinstance(item, (list, tuple)) or len(item) != 3:
            labels.add('UNSUPPORTED')
            continue
        color, shape, left = item
        if color not in ('red', 'yellow', 'green') or shape not in ('circle', 'arrow'):
            labels.add('UNSUPPORTED')
        else:
            label = {'red': 'R', 'yellow': 'Y', 'green': 'G'}[color]
            if shape == 'arrow':
                label += '_LEFT' if left is not False else '_RIGHT'
            labels.add(label)
    return '+'.join(sorted(labels)) if labels else 'UNKNOWN'


def permission(label, group):
    """Declared route-direction lamp proxy, not recovered C++ planner state."""
    lamps = set(label.split('+'))
    if label in ('UNKNOWN', 'MISSING') or 'UNSUPPORTED' in lamps:
        return 'UNKNOWN'
    if ('R' in lamps and 'G' in lamps) or ('Y' in lamps and ('G' in lamps or 'G_LEFT' in lamps)):
        return 'UNKNOWN'
    if group == 201326:
        return 'GO' if 'G_LEFT' in lamps and lamps <= {'R', 'G_LEFT'} else 'STOP'
    if lamps <= {'G', 'G_LEFT'} and 'G' in lamps:
        return 'GO'
    return 'STOP'


def normalize_runtime(body, when, origin):
    decision = body.get('frame_decision')
    source = body.get('source_stamp_ns')
    if not isinstance(decision, dict) or type(source) is not int or source <= 0:
        return None
    group = decision.get('group_id')
    # Keep group 0/non-mission packets as segment boundaries. Dropping them
    # here would silently join two different approaches to the same mission.
    if type(group) is not int or group < 0:
        return None
    raw, final = lamp_label(decision.get('observed_tuples')), lamp_label(decision.get('final_tuples'))
    timing = decision.get('observation_timing') or {}
    return dict(source_ns=source, receipt_ns=when, group=group, raw=raw, final=final,
                raw_permission=permission(raw, group), final_permission=permission(final, group),
                output_origin=timing.get('output_origin', 'missing'), output_is_held=timing.get('output_is_held'),
                adopted_source_ns=timing.get('adopted_observation_source_stamp_ns'),
                policy_reason=decision.get('policy_reason'), transition_reason=decision.get('transition_reason'),
                selection_status=decision.get('selection_status'), head_id=decision.get('head_id'),
                evidence_origin=origin)


def runtime_config(body):
    """Small actual-node snapshot, excluding per-frame/model-loader telemetry."""
    keys = ('selection_mode', 'classifier_mode', 'detector_weights_sha256', 'detector_imgsz',
            'planner_compliance_observations', 'low_confidence_hold_sec', 'no_lamps_hold_sec',
            'visual_continuation_sec', 'detector_actual_device', 'preserve_unreadable_color_evidence',
            'stamped_vote_window_size', 'stamped_vote_max_age_sec', 'stamped_vote_startup_passthrough')
    return {key: value for key, value in body.items()
            if key in keys or 'temporal' in key or 'smoothing' in key}


def point_distance(point, line):
    best = float('inf')
    for a, b in zip(line, line[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        norm = dx*dx+dy*dy
        if norm <= 0:
            continue
        ratio = min(1., max(0., ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/norm))
        best = min(best, math.hypot(point[0]-a[0]-ratio*dx, point[1]-a[1]-ratio*dy))
    return best


def zero_target(points):
    """First positive->zero boundary; all-zero/terminal-only paths are ambiguous."""
    moving = False
    for point in points:
        velocity = number(getattr(point, 'longitudinal_velocity_mps', None))
        if velocity is None:
            continue
        if abs(velocity) > .01:
            moving = True
        elif moving:
            p = point.pose.position
            return [p.x, p.y]
    return None


def line_target_states(points, lines, target):
    """Absence needs positive path coverage ACROSS this same line's zone.

    Looking only at the first zero cannot prove that a downstream signal stop
    was removed: an earlier stop may hide it in an already-zero plateau.
    """
    states = {}
    for group, line in lines.items():
        if target is not None and point_distance(target, line) <= 5.:
            states[group] = 'present'
            continue
        xmin, xmax = min(p[0] for p in line)-5., max(p[0] for p in line)+5.
        ymin, ymax = min(p[1] for p in line)-5., max(p[1] for p in line)+5.
        zone = []
        a, b = line[0], line[-1]
        for point in points:
            p = point.pose.position
            if xmin <= p.x <= xmax and ymin <= p.y <= ymax and point_distance([p.x, p.y], line) <= 5.:
                speed = number(point.longitudinal_velocity_mps)
                side = (b[0]-a[0])*(p.y-a[1])-(b[1]-a[1])*(p.x-a[0])
                zone.append((speed, side))
        crosses = bool(zone) and min(side for _, side in zone) < 0 < max(side for _, side in zone)
        positive = bool(zone) and all(speed is not None and abs(speed) > .01 for speed, _ in zone)
        states[group] = 'absent_positive_coverage_across_same_line_zone' if crosses and positive else 'ambiguous'
    return states


def measured_motion(message, when, origin):
    velocity = message.get('velocity', {})
    vector = [number(velocity.get(axis)) for axis in ('x', 'y', 'z')]
    if any(value is None for value in vector):
        return None
    speed = math.sqrt(sum(value*value for value in vector))
    acceleration = message.get('acceleration', {})
    acceleration = [number(acceleration.get(axis)) for axis in ('x', 'y', 'z')]
    signed = (sum(a*v for a, v in zip(acceleration, vector))/speed
              if speed > .2 and all(a is not None for a in acceleration) else None)
    return dict(receipt_ns=when, speed_mps=speed, accel_pedal=number(message.get('accel')),
                brake_pedal=number(message.get('brake')), acceleration_along_velocity_mps2=signed,
                evidence_origin=origin)


def planner_geometry(paths, evidence, warnings, projector):
    candidate = paths['selection'].get('map_candidate') or {}
    map_path = candidate.get('path')
    if not map_path and paths['campaign']:
        commands = read_json(paths['campaign']/'stack_logs/commands.json', evidence)
        text = json.dumps(commands)
        import re
        values = re.findall(r'map_file:=([^"\s]+)', text)
        if len(set(values)) == 1:
            map_path = values[0]
    if not map_path or not Path(map_path).is_file():
        warnings.append('Selected map path missing: same-line planner targets unavailable.')
        return {}, {}
    # Hash exact selected map and projector; source/installed map equivalence is
    # only claimed if the wrapper SHA agrees, never inferred by filename/date.
    raw = Path(map_path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    evidence.append(dict(path=map_path, bytes=len(raw), sha256=digest))
    if candidate.get('sha256') and candidate['sha256'] != digest:
        warnings.append('Selected map changed since wrapper snapshot: planner targets withheld.')
        return {}, {}
    try:
        from prepare_traffic_light_compliance_trials import stopline_groups
        projector_doc = Path(projector).read_bytes()
        evidence.append(dict(path=str(projector), bytes=len(projector_doc),
                             sha256=hashlib.sha256(projector_doc).hexdigest()))
        groups = stopline_groups(Path(map_path), Path(projector))
        return {row['group_id']: row['stop_line_xy'] for row in groups}, dict(
            map_path=map_path, map_sha256=digest, projector_path=str(projector),
            target_match_radius_m=5., front_axle_stop_offset_is_within_radius=True)
    except Exception as error:
        warnings.append('Selected-map projection unavailable: '+type(error).__name__+': '+str(error))
        return {}, {}


def collect_bag(path, lines, evidence, warnings):
    import rosbag
    before = path.stat()
    frames, selections, motion, control, targets = [], {}, [], [], []
    counts, configs = Counter(), Counter()
    with rosbag.Bag(str(path)) as bag:
        info = bag.get_type_and_topic_info().topics
        start, end = round(bag.get_start_time()*1e9), round(bag.get_end_time()*1e9)
        sensor_counts = {topic: row.message_count for topic, row in info.items()
                         if topic.startswith('/sensor/')}
        for topic, message, timestamp in bag.read_messages(topics=TOPICS):
            when = timestamp.to_nsec()
            counts[topic] += 1
            if topic.endswith('/debug_runtime') or topic.endswith('/debug_selection'):
                try:
                    body = json.loads(message.data)
                except (ValueError, TypeError):
                    counts['malformed_debug_json'] += 1
                    continue
                if topic.endswith('/debug_runtime'):
                    configs[json.dumps(runtime_config(body), sort_keys=True)] += 1
                    frame = normalize_runtime(body, when, 'bag')
                    if frame:
                        frames.append(frame)
                else:
                    key = body.get('source_stamp_ns'), body.get('group_id')
                    selections[key] = {key: body.get(key) for key in (
                        'status', 'reason', 'projection_reason', 'head_id')}
            elif topic == '/Ego_topic':
                row = measured_motion(primitive(message), when, 'bag/Ego_topic')
                if row:
                    motion.append(row)
            elif topic == '/ctrl_cmd':
                control.append(dict(receipt_ns=when, accel=number(message.accel),
                                    brake=number(message.brake), longlCmdType=message.longlCmdType))
            elif topic == '/planning/trajectory' and lines:
                if message.header.frame_id.strip('/') != 'map':
                    counts['non_map_trajectory_ignored'] += 1
                    continue
                target = zero_target(message.points)
                positive_path = any(number(point.longitudinal_velocity_mps) is not None and
                                    abs(point.longitudinal_velocity_mps) > .01 for point in message.points)
                distances = {group: point_distance(target, line) for group, line in lines.items()} if target else {}
                targets.append(dict(receipt_ns=when, target_xy_m=target,
                                    target_evidence_unambiguous=positive_path,
                                    group_states=line_target_states(message.points, lines, target),
                                    groups=[group for group, dist in distances.items() if dist <= 5.],
                                    target_line_distance_m=distances))
    unchanged(path, before)
    evidence.append(dict(path=str(path), bytes=before.st_size, mtime_ns=before.st_mtime_ns,
                         integrity='size_and_mtime_stable_not_full_bag_hash'))
    if not counts['/traffic_light_node/debug_runtime']:
        warnings.append('Bag debug_runtime missing; fallback to observer if available.')
    return dict(frames=frames, selections=selections, motion=motion, control=control, targets=targets,
                bag_start_ns=start, bag_end_ns=end, counts=dict(counts), raw_sensor_message_counts=sensor_counts,
                actual_node_runtime_config_counts=[dict(config=json.loads(config), frames=count)
                                                   for config, count in configs.items()],
                decoded_raw_sensor_messages=0)


def collect_observer(directory, data, evidence, warnings):
    if directory is None or not (directory/'observations.jsonl').is_file():
        warnings.append('Observer unavailable: native crossing verification missing.')
        return {}, {}
    metadata = read_json(directory/'metadata.json', evidence)
    crossings = read_json(directory/'crossings.json', evidence)
    validate_crossing_window(crossings, data['bag_start_ns'], data.get('bag_end_ns'))
    fallback_frames, fallback_selections, fallback_motion, fallback_control = [], {}, [], []
    need_frames, need_motion, need_control = not data['frames'], not data['motion'], not data['control']
    for row in jsonl(directory/'observations.jsonl', evidence, warnings):
        topic, when = row.get('topic', ''), row.get('receipt_ros_ns')
        if type(when) is not int:
            continue
        if need_frames and topic.endswith('/debug_runtime'):
            body = payload(row)
            if body:
                config = runtime_config(body)
                existing = next((entry for entry in data['actual_node_runtime_config_counts']
                                 if entry['config'] == config), None)
                if existing:
                    existing['frames'] += 1
                else:
                    data['actual_node_runtime_config_counts'].append(dict(config=config, frames=1))
                frame = normalize_runtime(body, when, 'observer')
                if frame:
                    fallback_frames.append(frame)
        elif need_frames and topic.endswith('/debug_selection'):
            body = payload(row)
            if body:
                fallback_selections[(body.get('source_stamp_ns'), body.get('group_id'))] = {
                    key: body.get(key) for key in ('status', 'reason', 'projection_reason', 'head_id')}
        elif need_motion and topic == '/Ego_topic':
            item = measured_motion(row.get('message', {}), when, 'observer/Ego_topic')
            if item:
                fallback_motion.append(item)
        elif need_control and topic == '/ctrl_cmd':
            message = row.get('message', {})
            fallback_control.append(dict(receipt_ns=when, accel=number(message.get('accel')),
                                         brake=number(message.get('brake')), longlCmdType=message.get('longlCmdType')))
    if need_frames:
        data['frames'], data['selections'] = fallback_frames, fallback_selections
    if need_motion:
        data['motion'] = fallback_motion
    if need_control:
        data['control'] = fallback_control
    return crossings, metadata


def validate_crossing_window(crossings, start, end):
    """Never attach another drive's native crossing result to this bag."""
    if start is None or end is None:
        return
    for crossing in crossings.get('crossings', []):
        stamp = crossing.get('estimated_source_stamp_ns')
        if type(stamp) is int and not start <= stamp <= end:
            raise ValueError('Observer crossing source stamp is outside the selected bag; '
                             'observer/run pairing is unverified: '+str(stamp))


def time_reference(source, receipt, start):
    return dict(source_stamp_ns=source, receipt_ros_stamp_ns=receipt,
                bag_elapsed_sec=(receipt-start)/1e9 if start is not None and receipt is not None else None,
                source_ROS_ISO_UTC=datetime.fromtimestamp(source/1e9, timezone.utc).isoformat() if source else None,
                ISO_is_ROS_clock_not_independently_verified_wall_clock=True)


def sequence(frames, key):
    segments = []
    for frame in frames:
        if not segments or segments[-1]['value'] != frame[key] or frame['segment'] != segments[-1]['segment']:
            segments.append(dict(value=frame[key], segment=frame['segment'], frames=0,
                                 first_source_ns=frame['source_ns'], first_receipt_ns=frame['receipt_ns']))
        segments[-1].update(last_source_ns=frame['source_ns'], last_receipt_ns=frame['receipt_ns'])
        segments[-1]['frames'] += 1
    for a, b in zip(segments, segments[1:]):
        if a['segment'] == b['segment']:
            a['next_value_source_ns'] = b['first_source_ns']
            a['duration_until_next_sample_sec'] = (b['first_source_ns']-a['first_source_ns'])/1e9
    return segments


def around(rows, timestamp, keys, radius=1.5):
    if timestamp is None:
        return dict(samples=0, reason='missing_incident_receipt_clock')
    # Arrays are bounded to selected scalar topics, not full ROS messages.
    selected = [row for row in rows if abs(row['receipt_ns']-timestamp) <= radius*1e9]
    timeline = []
    for offset in (-1., 0., .5, 1.):
        nearest = min(selected, key=lambda row: abs(row['receipt_ns']-(timestamp+offset*1e9)), default=None)
        if nearest and abs(nearest['receipt_ns']-(timestamp+offset*1e9)) <= .25*1e9:
            timeline.append(dict(offset_from_incident_sec=(nearest['receipt_ns']-timestamp)/1e9,
                                 **{key: nearest.get(key) for key in keys}))
    return dict(samples=len(selected), window_sec=[-radius, radius],
                fields={key: statistics_of([row.get(key) for row in selected]) for key in keys},
                samples_near_requested_offsets_sec=timeline,
                missing_or_sparse=not selected or min(row['receipt_ns'] for row in selected) > timestamp-1e9
                or max(row['receipt_ns'] for row in selected) < timestamp+1e9)


def first_permission_to_motion(frames, data, key):
    first = next((frame for frame in frames if frame[key] == 'GO'), None)
    if first is None:
        return dict(applicable=False, reason='no_direction_permitted_lamp_in_recording')
    timestamp = first['receipt_ns']
    before = [row for row in data['motion'] if 0 <= timestamp-row['receipt_ns'] <= .25e9]
    latest = max(before, key=lambda row: row['receipt_ns'], default=None)
    result = dict(first_GO_time=time_reference(first['source_ns'], timestamp, data['bag_start_ns']),
                  first_GO_lamps=first['raw'] if key == 'raw_permission' else first['final'],
                  measured_speed_at_first_GO_mps=latest['speed_mps'] if latest else None,
                  motion_threshold_mps=.3, correlation_not_causal_confirmation_delay=True)
    if latest is None or latest['speed_mps'] > .3:
        return dict(result, applicable=False,
                    reason='already_moving_at_first_GO' if latest else 'fresh_pre_GO_Ego_sample_missing')
    end = min(timestamp+15e9, frames[-1]['receipt_ns'])
    moving = next((row for row in data['motion'] if timestamp <= row['receipt_ns'] <= end and row['speed_mps'] > .3), None)
    brake_release = next((row for row in data['control'] if timestamp <= row['receipt_ns'] <= end
                          and row.get('brake') is not None and row['brake'] < .05), None)
    acceleration = next((row for row in data['control'] if timestamp <= row['receipt_ns'] <= end
                         and row.get('accel') is not None and row['accel'] > .05), None)
    def delay(row):
        return (row['receipt_ns']-timestamp)/1e9 if row else None
    return dict(result, applicable=True, search_window_sec=(end-timestamp)/1e9,
        first_measured_motion_receipt_ns=moving['receipt_ns'] if moving else None,
        first_measured_motion_time=time_reference(None, moving['receipt_ns'], data['bag_start_ns']) if moving else None,
        GO_receipt_to_measured_motion_sec=delay(moving),
        GO_source_to_measured_motion_sec=(moving['receipt_ns']-first['source_ns'])/1e9 if moving else None,
        GO_receipt_to_command_brake_below_0_05_sec=delay(brake_release),
        GO_receipt_to_command_accel_above_0_05_sec=delay(acceleration),
        missing_motion_or_command_is_not_proof_of_no_departure=moving is None or brake_release is None)


def load_video_index(wrapper, evidence, warnings):
    if wrapper is None:
        return None
    path = wrapper/'front_video/frames.jsonl'
    if not path.is_file():
        return None
    indexed = []
    for row in jsonl(path, evidence, warnings):
        image = row.get('image') or {}
        indexed.append(dict(source_ns=image.get('source_stamp_ns'), write_ns=row.get('write_ros_ns'),
                            frame_index=row.get('frame_index'), pts_sec=row.get('pts_sec'),
                            repeated=row.get('image_repeated'), exact_runtime_image_match=row.get('runtime_exact_image_stamp_match')))
    source_rows = sorted((row for row in indexed if type(row['source_ns']) is int),
                         key=lambda row: (row['source_ns'], row['frame_index']))
    receipt_rows = sorted((row for row in indexed if type(row['write_ns']) is int), key=lambda row: row['write_ns'])
    return dict(source_rows=source_rows, source_keys=[row['source_ns'] for row in source_rows],
                receipt_rows=receipt_rows, receipt_keys=[row['write_ns'] for row in receipt_rows])


def annotate_video_times(value, index, simulator_start):
    """Add review anchors only; no image decode or inference is performed."""
    if isinstance(value, list):
        for item in value:
            annotate_video_times(item, index, simulator_start)
        return
    if not isinstance(value, dict):
        return
    if 'bag_elapsed_sec' in value and 'receipt_ros_stamp_ns' in value:
        source, receipt = value.get('source_stamp_ns'), value.get('receipt_ros_stamp_ns')
        if index:
            clock = 'source' if type(source) is int else 'receipt'
            stamp = source if clock == 'source' else receipt
            keys, rows = index[clock+'_keys'], index[clock+'_rows']
            if type(stamp) is int and keys:
                position = bisect_left(keys, stamp)
                candidates = [rows[i] for i in (position-1, position) if 0 <= i < len(rows)]
                chosen = min(candidates, key=lambda row: abs(row['source_ns' if clock=='source' else 'write_ns']-stamp))
                delta = (chosen['source_ns' if clock=='source' else 'write_ns']-stamp)/1e9
                if abs(delta) <= .25:
                    value['front_video_review'] = dict(frame_index=chosen['frame_index'], pts_sec=chosen['pts_sec'],
                        join_clock=clock+'_ROS', signed_clock_delta_sec=delta, exact_source_match=clock=='source' and delta==0,
                        image_repeated=chosen['repeated'], runtime_exact_image_stamp_match=chosen['exact_runtime_image_match'])
                else:
                    value['front_video_review'] = dict(available=False, reason='no_video_clock_sample_within_0_25s')
        if simulator_start is not None and type(receipt) is int:
            value['simulator_video_elapsed_sec_proxy'] = receipt/1e9-simulator_start
            value['simulator_video_clock_basis'] = 'wrapper_child_started_wall_time_not_first_encoded_frame_ROS_wall_alignment_assumed'
        return
    for item in list(value.values()):
        annotate_video_times(item, index, simulator_start)


def target_reinsertions(targets, group, frames):
    if not targets or not frames:
        return []
    start, end = frames[0]['receipt_ns'], frames[-1]['receipt_ns']
    events, previous, had_target = [], None, False
    for row in targets:
        if not start <= row['receipt_ns'] <= end:
            continue
        if row.get('target_evidence_unambiguous') is False:
            # A stopped all-zero path does NOT prove that a signal target was
            # removed. Never turn its later identifiable boundary into a
            # false reinsertion event.
            previous = None
            continue
        state = row.get('group_states', {}).get(group)
        if state == 'ambiguous':
            previous = None
            continue
        present = state == 'present' if state else group in row['groups']
        if present and previous and not previous['present'] and had_target:
            gap = (row['receipt_ns']-previous['receipt_ns'])/1e9
            if 0 < gap <= .5:
                events.append(dict(receipt_ns=row['receipt_ns'], target_xy_m=row['target_xy_m'],
                                   distance_to_same_signal_ref_line_m=row['target_line_distance_m'][group],
                                   preceding_no_target_sample_ns=previous['receipt_ns'], sample_gap_sec=gap,
                                   absence_proof='strictly_positive_trajectory_coverage_across_same_signal_line_zone',
                                   provenance='first_positive_to_zero_boundary_near_same_selected_map_ref_line',
                                   attribution='spatial_temporal_proxy_not_planner_module_causality'))
        had_target |= present
        previous = dict(present=present, receipt_ns=row['receipt_ns'])
    return events


def summarize_mission(group, frames, data, crossings, short_sec):
    raw, final = sequence(frames, 'raw'), sequence(frames, 'final')
    raw_permissions = sequence(frames, 'raw_permission')
    final_permissions = sequence(frames, 'final_permission')
    interior, exclusions = [], Counter()
    for segment in sorted({frame['segment'] for frame in frames}):
        own = [frame for frame in frames if frame['segment'] == segment]
        known = [i for i, frame in enumerate(own) if frame['raw'] not in ('UNKNOWN', 'MISSING')]
        if not known:
            exclusions['all_unseen_segment_frames'] += len(own)
            continue
        exclusions['initial_unseen_frames'] += known[0]
        exclusions['trailing_no_recovery_frames'] += len(own)-known[-1]-1
        interior.extend(own[known[0]:known[-1]+1])
    incidents = []
    for key, rows in (('raw_permission', raw_permissions), ('final_permission', final_permissions)):
        for previous, current, following in zip(rows, rows[1:], rows[2:]):
            if len({previous['segment'], current['segment'], following['segment']}) != 1:
                continue
            kind = ('GO_UNKNOWN_GO' if previous['value'] == following['value'] == 'GO' and current['value'] == 'UNKNOWN'
                    else 'GO_STOP_GO' if previous['value'] == following['value'] == 'GO' and current['value'] == 'STOP'
                    else 'STOP_GO_STOP' if previous['value'] == following['value'] == 'STOP' and current['value'] == 'GO' else None)
            duration = current.get('duration_until_next_sample_sec')
            if kind and duration is not None and 0 <= duration <= short_sec:
                incidents.append(dict(stream=key, kind=kind, frames=current['frames'], duration_sec=duration,
                    time=time_reference(current['first_source_ns'], current['first_receipt_ns'], data['bag_start_ns']),
                    measured_Ego=around(data['motion'], current['first_receipt_ns'],
                        ('speed_mps', 'brake_pedal', 'accel_pedal', 'acceleration_along_velocity_mps2')),
                    control_command=around(data['control'], current['first_receipt_ns'], ('brake', 'accel'))))
    reinsertion = target_reinsertions(data['targets'], group, frames)
    for event in reinsertion:
        event['time'] = time_reference(None, event['receipt_ns'], data['bag_start_ns'])
        event['measured_Ego'] = around(data['motion'], event['receipt_ns'],
            ('speed_mps', 'brake_pedal', 'accel_pedal', 'acceleration_along_velocity_mps2'))
        event['control_command'] = around(data['control'], event['receipt_ns'], ('brake', 'accel'))
        event['nearby_short_islands_within_1s'] = [index for index, incident in enumerate(incidents)
            if abs(incident['time']['receipt_ros_stamp_ns']-event['receipt_ns']) <= 1e9]
    interior_seq = sequence(interior, 'raw')
    unknown_islands = [row for row in interior_seq if row['value'] == 'UNKNOWN']
    for row in unknown_islands:
        row['time'] = time_reference(row['first_source_ns'], row['first_receipt_ns'], data['bag_start_ns'])
        row['measured_Ego'] = around(data['motion'], row['first_receipt_ns'],
            ('speed_mps', 'brake_pedal', 'accel_pedal', 'acceleration_along_velocity_mps2'))
        row['control_command'] = around(data['control'], row['first_receipt_ns'], ('brake', 'accel'))
    mission_crossings = [row for row in crossings.get('crossings', []) if row.get('group_id') == group]
    for row in mission_crossings:
        row['video_review_time'] = time_reference(row.get('estimated_source_stamp_ns'),
            row.get('estimated_source_stamp_ns'), data['bag_start_ns'])
    return dict(mission=MISSIONS[group], group_id=group, runtime_frames=len(frames),
        first_time=time_reference(frames[0]['source_ns'], frames[0]['receipt_ns'], data['bag_start_ns']) if frames else None,
        last_time=time_reference(frames[-1]['source_ns'], frames[-1]['receipt_ns'], data['bag_start_ns']) if frames else None,
        raw_lamp_counts=dict(Counter(frame['raw'] for frame in frames)),
        final_lamp_counts=dict(Counter(frame['final'] for frame in frames)),
        raw_lamp_sequence=raw, final_lamp_sequence=final,
        raw_route_permission_sequence=raw_permissions, final_route_permission_sequence=final_permissions,
        output_origin_counts=dict(Counter(frame['output_origin'] for frame in frames)),
        selection_status_counts=dict(Counter(str(frame['selection_status']) for frame in frames)),
        selection_reason_counts=dict(Counter(str(frame.get('selection', {}).get('reason')) for frame in frames)),
        first_raw_permission_to_motion=first_permission_to_motion(frames, data, 'raw_permission'),
        first_final_permission_to_motion=first_permission_to_motion(frames, data, 'final_permission'),
        interior_raw_unknown=dict(denominator_frames=len(interior), unknown_frames=sum(frame['raw']=='UNKNOWN' for frame in interior),
            unknown_islands=len(unknown_islands), unknown_island_sequence=unknown_islands, excluded_frames=dict(exclusions),
            denominator='inclusive_first_to_last_nonempty_raw_lamp_frame_per_contiguous_mission_segment',
            trailing_exclusion='no_later_lamp_recovery_not_independent_proof_of_FOV_exit',
            rate_is_accuracy=False),
        short_islands=incidents, planner_same_line_stop_target_reinsertions=reinsertion,
        planner_evidence_available=bool(data['targets']),
        planner_all_zero_or_unreadable_target_samples=sum(row.get('target_evidence_unambiguous') is False
            for row in data['targets'] if frames and frames[0]['receipt_ns'] <= row['receipt_ns'] <= frames[-1]['receipt_ns']),
        native_crossings=mission_crossings,
        native_crossing_evidence_available=bool(crossings),
        concise=dict(raw_sequence=' > '.join(row['value'] for row in raw),
            final_sequence=' > '.join(row['value'] for row in final),
            interior_unknown_frames=sum(frame['raw']=='UNKNOWN' for frame in interior),
            interior_denominator_frames=len(interior),
            raw_short_islands=sum(row['stream']=='raw_permission' for row in incidents),
            final_short_islands=sum(row['stream']=='final_permission' for row in incidents),
            same_line_target_reinsertions=len(reinsertion), crossings=len(mission_crossings)))


def analyze(path, short_sec=1., projector=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml', observer=None):
    evidence, warnings = [], []
    paths = discover(path, evidence, warnings)
    if observer is not None:
        observer = Path(observer).resolve(strict=True)
        paths['observer'] = observer/'observer' if (observer/'observer').is_dir() else observer
    lines, geometry = planner_geometry(paths, evidence, warnings, projector)
    data = collect_bag(paths['bag'], lines, evidence, warnings) if paths['bag'] else dict(
        frames=[], selections={}, motion=[], control=[], targets=[], bag_start_ns=None, bag_end_ns=None,
        counts={}, raw_sensor_message_counts={}, actual_node_runtime_config_counts=[], decoded_raw_sensor_messages=0)
    crossings, _ = collect_observer(paths['observer'], data, evidence, warnings)
    unique, duplicate = {}, Counter()
    for frame in data['frames']:
        key = frame['source_ns'], frame['group']
        if key in unique:
            duplicate['duplicate_exact_source_group_frames'] += 1
            if (unique[key]['raw'], unique[key]['final']) != (frame['raw'], frame['final']):
                duplicate['conflicting_duplicate_frames'] += 1
        else:
            unique[key] = frame
    frames = sorted(unique.values(), key=lambda row: (row['source_ns'], row['receipt_ns']))
    segment, previous = 0, None
    for frame in frames:
        # A long callback gap cannot create an interpolated short island.
        if previous is None or frame['group'] != previous['group'] or frame['source_ns']-previous['source_ns'] > 1e9:
            segment += 1
        frame['segment'] = segment
        frame['selection'] = data['selections'].get((frame['source_ns'], frame['group']), {})
        previous = frame
    validator = {}
    if paths['execution']:
        results = list(paths['execution'].glob('planbag_*.json'))
        if len(results) == 1:
            validator['result'] = read_json(results[0], evidence)
        else:
            warnings.append('Missing/ambiguous validator final result; final status unverified.')
        validator['events'] = list(jsonl(paths['execution']/'events.jsonl', evidence, warnings))
        validator['safety'] = read_json(paths['execution']/'safety_receipt.json', evidence)
    safety = summarize_safety(paths['wrapper_events'], validator)
    missions = [summarize_mission(group, [f for f in frames if f['group']==group], data, crossings, short_sec)
                for group in MISSIONS]
    video = load_video_index(paths['wrapper'], evidence, warnings)
    simulator_start = next((row.get('wall_time') for row in paths['wrapper_events']
        if row.get('event') == 'child_started' and row.get('name') == 'simulator_video'), None)
    annotate_video_times(missions, video, simulator_start)
    return dict(input=str(paths['input']), wrapper=str(paths['wrapper']) if paths['wrapper'] else None,
        campaign=str(paths['campaign']) if paths['campaign'] else None,
        bag=str(paths['bag']) if paths['bag'] else None, selection=paths['selection'], evidence=evidence,
        observer=str(paths['observer']) if paths['observer'] else None,
        selected_launch_commands=[row for row in paths['wrapper_events'] if row.get('event') == 'launch'
            and row.get('package') in ('traffic_light_detection', 'ajounice_planning_integration')],
        actual_node_runtime_config_counts=data['actual_node_runtime_config_counts'],
        bag_start_ns=data['bag_start_ns'], bag_end_ns=data['bag_end_ns'], topic_message_counts=data['counts'],
        raw_sensor_message_counts=data['raw_sensor_message_counts'], decoded_raw_sensor_messages=0,
        runtime_duplicate_counts=dict(duplicate), planner_geometry=geometry, safety=safety,
        missions=missions, warnings=warnings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--short-island-sec', type=float, default=1.)
    parser.add_argument('--observer', type=Path, action='append',
                        help='Optional explicit observer directories, exactly one per positional input')
    parser.add_argument('--projector', type=Path, default=ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    args = parser.parse_args()
    if not math.isfinite(args.short_island_sec) or not 0 < args.short_island_sec <= 5:
        parser.error('--short-island-sec requires 0 < seconds <= 5')
    if args.observer and len(args.observer) != len(args.runs):
        parser.error('--observer must be repeated exactly once per positional input')
    output = args.output.resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():
        parser.error('Fresh output required; existing evidence is never overwritten')
    report = dict(schema='traffic_light_driving_comparison_v1', read_only=True,
        created_UTC=datetime.now(timezone.utc).isoformat(), short_island_max_sec=args.short_island_sec,
        limitations=[
            'Natural phases differ across runs: raw counts and island differences are descriptive, not accuracy or causal treatment effects.',
            'Raw lamps are observed_tuples; debug_raw_label is post-normalization and is deliberately not used as raw evidence.',
            'GO/STOP/UNKNOWN sequences are direction-aware lamp proxies, not the C++ planner internal decision; D green-circle alone is STOP.',
            'Interior denominator excludes initial unseen and trailing no-recovery frames per contiguous mission segment; trailing exclusion does not prove physical FOV exit.',
            'A >1s source gap splits a segment. Missing recordings and omitted frames cannot be classified as correct or incorrect.',
            'Stop target requires a positive-to-zero trajectory boundary within 5m of the same selected-map ref_line; all-zero paths are ambiguous and omitted; goal zero elsewhere is excluded.',
            'Reinsertion is a spatial/temporal proxy, not proof of traffic-module causality. Actual speed, pedal feedback and commands are separate evidence.',
            'Native crossing brackets and goal/stop-P completion remain separate. Development proxies are not official competition penalty certification.',
            'Video-review elapsed timestamps are relative to bag start; independent screen-recording offset must be established from its own metadata.',
            'No raw sensor payload decode; bag counts for recorded sensors are metadata only.'],
        runs=[analyze(path, args.short_island_sec, args.projector, observer)
              for path, observer in zip(args.runs, args.observer or [None]*len(args.runs))])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(output), runs=[dict(input=run['input'], safety=run['safety'],
        missions=[dict(mission=m['mission'], **m['concise']) for m in run['missions']], warnings=run['warnings'])
        for run in report['runs']]), ensure_ascii=False))


if __name__ == '__main__':
    main()
