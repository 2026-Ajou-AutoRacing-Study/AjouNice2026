#!/usr/bin/env python3
"""Read-only trial JSONL timing analysis. No ROS/GPU/model/accuracy evaluation.

Exact joins are (source image ROS stamp, mission group ID), never receipt-time
nearest neighbors. Missing sampled log packets do not prove consumer loss.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def number(value):
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


def positive_ns(value):
    return value if type(value) is int and value > 0 else None


def quantiles(values):
    values = sorted(value for value in values if number(value) is not None)
    if not values:
        return {'count': 0, 'p50': None, 'p95': None, 'p99': None, 'min': None, 'max': None}
    def percentile(p):
        index = (len(values)-1)*p
        low = int(math.floor(index)); high = int(math.ceil(index))
        return values[low]+(values[high]-values[low])*(index-low)
    return {'count': len(values), 'p50': percentile(.5), 'p95': percentile(.95),
            'p99': percentile(.99), 'min': values[0], 'max': values[-1]}


def latency_summary(values):
    """Negative source ages are reported, never clamped into valid latencies."""
    valid = [value for value in values if number(value) is not None]
    return {'nonnegative_sec': quantiles([v for v in valid if v >= 0]),
            'negative_count': sum(v < 0 for v in valid),
            'negative_sec': quantiles([v for v in valid if v < 0])}


def payload(row):
    parsed = row.get('parsed_debug')
    if isinstance(parsed, dict):
        return parsed
    message = row.get('message', {})
    raw = message.get('data') if isinstance(message, dict) else None
    if isinstance(raw, str):
        try:
            result = json.loads(raw)
            return result if isinstance(result, dict) else None
        except (ValueError, TypeError):
            pass
    return None


def key_for(stamp, group):
    if positive_ns(stamp) is None or type(group) is not int or group < 0:
        return None
    return stamp, group


def receipt(row):
    return {'ros_ns': row.get('receipt_ros_ns'), 'monotonic': number(row.get('receipt_monotonic'))}


def delta_sec(first, second):
    if number(first) is None or number(second) is None:
        return None
    # Integer subtraction before conversion retains source nanosecond precision.
    return (first-second)/1e9


def signal_stamp(row):
    top = positive_ns(row.get('source_message_stamp_ns'))
    if top is not None:
        return top
    # v1 recorder did not normalize top-level builtin_interfaces/Time, but
    # preserved it in the raw typed message. Do not substitute receipt stamps.
    stamp = row.get('message', {}).get('stamp', {})
    sec, ns = stamp.get('sec'), stamp.get('nanosec')
    if type(sec) is int and type(ns) is int and sec >= 0 and 0 <= ns < 1000000000:
        return positive_ns(sec*1000000000+ns)
    return None


def capture_header_stamp(record):
    """Only the captured camera header is a window clock, never native/receipt time."""
    stamp = record.get('header_stamp')
    if not isinstance(stamp, dict):
        return None
    sec = stamp.get('secs', stamp.get('sec'))
    ns = stamp.get('nsecs', stamp.get('nanosec'))
    if type(sec) is int and type(ns) is int and sec >= 0 and 0 <= ns < 1000000000:
        return positive_ns(sec*1000000000+ns)
    return None


def load_capture_windows(path):
    """Pin immutable manifest bytes; inspect capture identity fields only, not GT."""
    path = Path(path).resolve(strict=True)
    before = path.stat(); raw = path.read_bytes(); after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('Capture manifest changed during read')
    document = json.loads(raw)
    records = document.get('records') if isinstance(document, dict) else None
    if not isinstance(records, list) or not records:
        raise ValueError('Capture manifest requires a nonempty records array')
    groups = {}
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise ValueError('Capture record {} is not an object'.format(index))
        env = record.get('environment')
        group = record.get('group_regulatory_id')
        if group is None and type(record.get('group_id')) is int:
            group = record['group_id']
        stamp = capture_header_stamp(record)
        if not isinstance(env, str) or not env.strip() or type(group) is not int or group <= 0 or stamp is None:
            raise ValueError('Capture record {} requires environment, positive integer group_regulatory_id, and valid positive header_stamp'.format(index))
        item = groups.setdefault((env, group), {'stamps': [], 'pose_ids': set()})
        item['stamps'].append(stamp)
        if isinstance(record.get('group_id'), str):
            item['pose_ids'].add(record['group_id'])
    windows = []
    for (env, group), item in sorted(groups.items()):
        windows.append({'environment': env, 'group_id': group,
            'first_source_header_stamp_ns': min(item['stamps']),
            'last_source_header_stamp_ns': max(item['stamps']),
            'capture_record_count': len(item['stamps']),
            'unique_capture_header_stamps': len(set(item['stamps'])),
            'pose_group_ids': sorted(item['pose_ids'])})
    # One source/group event cannot be assigned to two environments. Do not
    # silently pick a window by ordering or by inference/GT success.
    for index, first in enumerate(windows):
        for second in windows[index+1:]:
            if first['group_id'] == second['group_id'] and max(first['first_source_header_stamp_ns'], second['first_source_header_stamp_ns']) <= min(first['last_source_header_stamp_ns'], second['last_source_header_stamp_ns']):
                raise ValueError('Overlapping capture source windows for the same mission group')
    return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(),
            'bytes': len(raw), 'record_count': len(records)}, windows


class CaptureWindowAnalysis:
    """Fixed inclusive source-time windows, independently of success/phase."""
    def __init__(self, manifest_path, **kwargs):
        self.provenance, self.windows = load_capture_windows(manifest_path)
        self.analyses = [TimingAnalysis(**kwargs) for _ in self.windows]
        self.aggregate = TimingAnalysis(**kwargs)
        self.filter_counts = defaultdict(Counter)

    def add(self, row):
        if not isinstance(row, dict):
            return
        topic = row.get('topic')
        a = self.aggregate
        if topic not in (a.map_topic, a.signals_topic, a.namespace+'/debug_selection', a.namespace+'/debug_runtime'):
            return
        if topic == a.signals_topic:
            stamp = signal_stamp(row)
            message = row.get('message', {})
            groups = message.get('traffic_light_groups', []) if isinstance(message, dict) else []
            groups = [g for g in groups if isinstance(g, dict)] if isinstance(groups, list) else []
            keys = [key_for(stamp, g.get('traffic_light_group_id')) for g in groups]
        else:
            body = payload(row)
            if body is None:
                self.filter_counts[topic]['unkeyed_relevant_events'] += 1
                return
            stamp = positive_ns(body.get('source_stamp_ns'))
            group = body.get('group_id')
            if topic.endswith('/debug_runtime'):
                decision = body.get('frame_decision', {})
                group = decision.get('group_id') if isinstance(decision, dict) else None
            keys = [key_for(stamp, group)]
        keys = [key for key in keys if key is not None]
        if not keys:
            self.filter_counts[topic]['unkeyed_relevant_events'] += 1
            return
        indexes = [index for index, window in enumerate(self.windows)
                   if any(window['first_source_header_stamp_ns'] <= key[0] <= window['last_source_header_stamp_ns']
                          and window['group_id'] == key[1] for key in keys)]
        if not indexes:
            inside = any(window['first_source_header_stamp_ns'] <= stamp <= window['last_source_header_stamp_ns'] for window in self.windows)
            self.filter_counts[topic]['source_in_window_but_group_mismatch' if inside else 'outside_source_windows'] += 1
            return
        self.filter_counts[topic]['included_events'] += 1
        def filtered_for(indexes):
            if topic != a.signals_topic:
                return row
            # An array may contain other mission groups. Only exact matching
            # groups contribute to the window signal join, not the whole array.
            allowed = {self.windows[index]['group_id'] for index in indexes}
            selected = [g for g in groups if g.get('traffic_light_group_id') in allowed]
            return dict(row, message=dict(message, traffic_light_groups=selected))
        for index in indexes:
            self.analyses[index].add(filtered_for([index]))
        self.aggregate.add(filtered_for(indexes))

    def report(self, metadata=None, logger_summary=None):
        return {'manifest': self.provenance,
            'window_clock': 'captured_camera_header_ROS_source_stamp_ns',
            'bounds': 'inclusive_first_to_last_capture_header_stamp_per_environment_and_mission_group',
            'selection_rule': 'all_recorded_events_with_matching_group_and_source_stamp_inside_fixed_bounds_regardless_of_ready_missing_or_success',
            'capture_protocol': {'scene_load_settle_seconds': 8.0,
                'settle_provenance': 'requested_v2_trial_protocol_not_independently_verified_from_camera_header_stamps',
                'window_is_after_declared_settle': True,
                'closed_loop_certification': False},
            'native_color_or_model_correctness_used_for_filtering': False,
            'filter_counts_recorded_events_by_topic': {topic: dict(counts) for topic, counts in self.filter_counts.items()},
            'aggregate': self.aggregate.report(metadata, logger_summary),
            'by_environment_and_group': [dict(window, analysis=analysis.report(metadata, logger_summary))
                                         for window, analysis in zip(self.windows, self.analyses)],
            'warnings': [
                'Windows are fixed from every manifest capture header, not successful model/native/TF frames; all matching-source/group events between bounds are included, not only saved JPEG stamps.',
                'Wrong-group and unkeyed events are excluded from same-group metrics and separately counted; absence is not evidence of a correct route feed.',
                'Manifest first/last header stamps do not independently verify SceneLoad settling, renderer weather, stopped ego, or closed-loop driving safety.',
                'Observer throttle counts and clock metadata come from the full monitor session, not reconstructed per-window logger losses.',
            ]}


class TimingAnalysis:
    def __init__(self, namespace='/traffic_light_trial/traffic_light_node', map_topic='/traffic_light_trial/map_rois',
                 signals_topic='/traffic_light_trial/traffic_signals'):
        self.namespace, self.map_topic, self.signals_topic = namespace.rstrip('/'), map_topic, signals_topic
        self.packet = defaultdict(list); self.selection = defaultdict(list)
        self.runtime = defaultdict(list); self.signals = defaultdict(list)
        self.counts = Counter(); self.ignored_unkeyed = Counter()
        self.selection_reasons = Counter(); self.packet_reasons = Counter(); self.packet_status = Counter()
        self.pose_delta = []; self.generated_age = []; self.packet_transport = []; self.selection_age = []; self.signal_age = []
        self.node_timings = defaultdict(list)

    def add(self, row):
        if not isinstance(row, dict):
            self.counts['nonobject_records'] += 1; return
        topic = row.get('topic', '')
        if topic not in (self.map_topic, self.signals_topic, self.namespace+'/debug_selection', self.namespace+'/debug_runtime'):
            return
        self.counts[topic] += 1
        if topic == self.signals_topic:
            stamp = signal_stamp(row)
            message = row.get('message', {})
            for group in message.get('traffic_light_groups', []):
                key = key_for(stamp, group.get('traffic_light_group_id'))
                if key is not None:self.signals[key].append(receipt(row))
                else:self.ignored_unkeyed[topic] += 1
            value = delta_sec(row.get('receipt_ros_ns'), stamp)
            if value is not None:self.signal_age.append(value)
            return
        body = payload(row)
        if body is None:
            self.ignored_unkeyed[topic] += 1; return
        group = body.get('group_id')
        if topic.endswith('/debug_runtime'):
            # Inspect timing/config identity only; never probabilities, lamps,
            # labels, correctness, native-state or phase accuracy.
            decision = body.get('frame_decision', {})
            group = decision.get('group_id') if isinstance(decision, dict) else None
            timing = body.get('frame_timing')
            if isinstance(timing, dict):
                for name in ('decode_sec','detector_and_box_readback_sec','selection_including_map_wait_sec',
                             'crop_and_classifier_sec','policy_and_publish_sec','callback_to_output_sec','header_to_callback_sec'):
                    value = number(timing.get(name))
                    if value is not None:self.node_timings[name].append(value)
        stamp = positive_ns(body.get('source_stamp_ns'))
        key = key_for(stamp, group)
        if key is None:
            self.ignored_unkeyed[topic] += 1; return
        item = receipt(row)
        if topic == self.map_topic:
            item.update(status=body.get('status'), reason=body.get('reason'),
                        generated_stamp_ns=body.get('generated_stamp_ns'),
                        eligible_head_count=sum(r.get('eligible') is True for r in body.get('rois', []) if isinstance(r,dict)))
            self.packet[key].append(item)
            self.packet_status[str(item['status'])] += 1;self.packet_reasons[str(item['reason'])] += 1
            value = delta_sec(item['generated_stamp_ns'], stamp)
            if value is not None:self.generated_age.append(value)
            value = delta_sec(item['ros_ns'], item['generated_stamp_ns'])
            if value is not None:self.packet_transport.append(value)
            value = number(body.get('pose_image_delta_sec'))
            if value is not None:self.pose_delta.append(value)
        elif topic.endswith('/debug_selection'):
            if body.get('mode') != 'map_association':
                self.ignored_unkeyed['non_map_selection_mode'] += 1;return
            item.update(status=body.get('status'),reason=body.get('reason'),
                        projection_source_stamp_ns=body.get('projection_source_stamp_ns'))
            self.selection[key].append(item);self.selection_reasons[str(item['reason'])] += 1
            value = delta_sec(item['ros_ns'], stamp)
            if value is not None:self.selection_age.append(value)
        else:
            self.runtime[key].append(item)

    def report(self, metadata=None, logger_summary=None):
        metadata, logger_summary = metadata or {}, logger_summary or {}
        joined=Counter();arrival=[];selection_unique_status=Counter();conflicts=Counter()
        for key, rows in self.selection.items():
            statuses={(row['status'],str(row['reason'])) for row in rows}
            if len(statuses)>1:conflicts['selection_keys_with_conflicting_status_or_reason']+=1
            if len(statuses)==1:selection_unique_status[str(rows[0]['status'])]+=1
            else:selection_unique_status['conflicting_duplicates']+=1
            packets=self.packet.get(key,[])
            if packets:
                joined['exact_stamp_and_group_packet_key']+=1
                if any(row['status']=='ready' for row in packets):joined['has_recorded_ready_packet']+=1
                if any(row['status']=='ready' and row['eligible_head_count']>0 for row in packets):joined['has_recorded_ready_eligible_packet']+=1
                pstatuses={(r['status'],str(r['reason'])) for r in packets}
                if len(pstatuses)>1:conflicts['joined_packet_keys_with_conflicting_status_or_reason']+=1
                pmono=[row['monotonic'] for row in packets if row['monotonic'] is not None]
                smono=[row['monotonic'] for row in rows if row['monotonic'] is not None]
                if pmono and smono:
                    difference=min(pmono)-min(smono);arrival.append(difference)
                    joined['helper_observed_after_selection']+=difference>0
                    joined['helper_observed_not_after_selection']+=difference<=0
            else:joined['no_exact_packet_key_in_sampled_log']+=1
            if key in self.runtime:joined['exact_runtime_key']+=1
            if key in self.signals:joined['exact_signal_output_key']+=1
        denominator=len(self.selection)
        by_group={}
        for key,rows in self.selection.items():
            item=by_group.setdefault(str(key[1]),{'unique_source_keys':0,'status_counts':Counter(),'reason_counts_recorded_events':Counter()})
            item['unique_source_keys']+=1
            statuses={row['status'] for row in rows}
            item['status_counts'][str(next(iter(statuses))) if len(statuses)==1 else 'conflicting_duplicates']+=1
            item['reason_counts_recorded_events'].update(str(row['reason']) for row in rows)
        by_group={group:{key:dict(value) if isinstance(value,Counter) else value for key,value in item.items()}
                  for group,item in by_group.items()}
        packet_events=sum(self.packet_status.values())
        throttle=logger_summary.get('throttled_by_topic',{})
        relevant_throttle={topic:value for topic,value in throttle.items() if topic in self.counts}
        warnings=[
            'No classifier/native-state correctness or traffic permission accuracy is scored.',
            'Same-source joins are exact (source_stamp_ns,group_id). No nearest receipt/adjacent camera frame join is used.',
            'A missing exact packet in independently throttled observer logs is NOT proof the inference node missed it.',
            'Helper generated-source latency includes image transport/callback scheduling/TF work; it is NOT isolated TF lookup compute duration.',
            'ROS source/receipt differences assume a common configured ROS clock, not independently asserted UTC or monotonic alignment.',
            'Monotonic arrival order is observer callback order, not proof of when the inference consumer received a packet.',
            'Logged ready means projection computation ready; visibility, detection correctness and mission identity accuracy are not proved.',
        ]
        if any(relevant_throttle.values()):warnings.append('Observer throttling occurred; exact join coverage denominators describe recorded samples only.')
        if not self.node_timings:warnings.append('debug_runtime node timing samples absent: compute/wait percentiles cannot be reconstructed and remain unavailable.')
        return {'schema_version':1,'scope':'read_only_source_stamp_TF_pipeline_timing_not_model_accuracy',
            'counts':dict(self.counts),'unkeyed_or_wrong_mode':dict(self.ignored_unkeyed),
            'selection_unique_source_group_keys':denominator,'selection_status_counts':dict(selection_unique_status),
            'selection_by_group_id':by_group,'group_zero_is_missing_trial_route_not_a_mission':True,
            'selection_reason_counts_recorded_events':dict(self.selection_reasons),
            'helper_status_counts_recorded_events':dict(self.packet_status),'helper_reason_counts_recorded_events':dict(self.packet_reasons),
            'helper_ready_rate_recorded_packet_events':self.packet_status.get('ready',0)/packet_events if packet_events else None,
            'exact_join_counts':dict(joined),
            'exact_join_coverage_recorded_selection_keys':joined['exact_stamp_and_group_packet_key']/denominator if denominator else None,
            'duplicate_key_counts':{name:sum(len(v)-1 for v in stream.values()) for name,stream in
                (('helper',self.packet),('selection',self.selection),('runtime',self.runtime),('signal',self.signals))},
            'conflicting_duplicates':dict(conflicts),
            'latency_sec':{'helper_source_to_generated_ROS_clock_assumption':latency_summary(self.generated_age),
                'helper_generated_to_observer_ROS_clock_assumption':latency_summary(self.packet_transport),
                'image_source_to_selection_observer_ROS_clock_assumption':latency_summary(self.selection_age),
                'image_source_to_signal_output_observer_ROS_clock_assumption':latency_summary(self.signal_age),
                'nearest_odom_image_stamp_delta_sec':quantiles(self.pose_delta),
                'helper_observer_arrival_minus_selection_observer_arrival_signed_monotonic':quantiles(arrival)},
            'node_reported_callback_wall_timing':{'available':bool(self.node_timings),
                'not_CUDA_event_or_isolated_TF_time':True,'fields':{key:latency_summary(values) for key,values in self.node_timings.items()}},
            'logger_throttled_by_relevant_topic':relevant_throttle,
            'clock_manifest':metadata.get('timestamp_contracts',{'common_source_and_receipt_ros_clock_independently_verified':False,'reason':'older_monitor_metadata_no_clock_manifest'}),
            'warnings':warnings}


def analyze(input_path, metadata=None, logger_summary=None, capture_manifest=None, **kwargs):
    path=Path(input_path).resolve(strict=True);before=path.stat();digest=hashlib.sha256();analysis=TimingAnalysis(**kwargs)
    captured=CaptureWindowAnalysis(capture_manifest,**kwargs) if capture_manifest is not None else None
    lines=invalid=0
    with path.open('rb') as stream:
        for line in stream:
            digest.update(line);lines+=1
            try:
                row=json.loads(line);analysis.add(row)
                if captured is not None:captured.add(row)
            except (ValueError,TypeError,KeyError,AttributeError):invalid+=1
    after=path.stat()
    if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):
        raise ValueError('Input log changed during analysis; use a closed immutable log')
    report=analysis.report(metadata,logger_summary)
    report['input']={'path':str(path),'sha256':digest.hexdigest(),'bytes':before.st_size,'lines':lines,'invalid_records':invalid}
    if captured is not None:report['capture_window_analysis']=captured.report(metadata,logger_summary)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--node-namespace',default='/traffic_light_trial/traffic_light_node')
    parser.add_argument('--map-topic',default='/traffic_light_trial/map_rois')
    parser.add_argument('--signals-topic',default='/traffic_light_trial/traffic_signals')
    parser.add_argument('--capture-manifest',type=Path,help='Optional fixed source-stamp capture windows; retains full-session report and adds aggregate/per-environment/group window reports')
    args=parser.parse_args(argv)
    output=args.output.expanduser().absolute()
    output.resolve().relative_to((ROOT/'artifacts/evaluation').resolve())
    if output.exists() or output.is_symlink():raise FileExistsError('Fresh output JSON required')
    sidecars={};pins={}
    for name,key in (('metadata.json','metadata'),('summary.json','logger_summary')):
        path=args.input.resolve().parent/name
        if path.exists():
            raw=path.read_bytes();sidecars[key]=json.loads(raw);pins[str(path)]=hashlib.sha256(raw).hexdigest()
    report=analyze(args.input,**sidecars,capture_manifest=args.capture_manifest,namespace=args.node_namespace,map_topic=args.map_topic,signals_topic=args.signals_topic)
    report['sidecar_sha256']=pins;report['analysis_source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as stream:json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({'output':str(output),'input_sha256':report['input']['sha256'],
                      'selection_keys':report['selection_unique_source_group_keys'],
                      'exact_join_coverage_recorded_samples':report['exact_join_coverage_recorded_selection_keys']}))


if __name__=='__main__':main()
