#!/usr/bin/env python3
"""Read-only SOURCE C++ Guard replay, not a vehicle/planner counterfactual.

Preserves canonical wire source/observer receipt ns. Receipt-event and five
20-Hz clock phases are separate assumptions, not measured planner callbacks.
No bags, images, ROS, native APIs, install or process-control operations.
"""
import argparse
from collections import Counter, defaultdict
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / ('planning_ws/src/autoware_behavior_velocity_traffic_light_module/include/'
                 'autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp')
CPP = Path(__file__).with_suffix('.cpp')
MISSIONS = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}
STREAMS = ('wire', 'counterfactual_partial_marker')


def jsonl(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_ns(stamp):
    if not isinstance(stamp, dict):
        return None
    sec, nano = stamp.get('sec'), stamp.get('nanosec')
    if type(sec) is not int or type(nano) is not int or not 0 <= nano < 10**9:
        return None
    return sec * 10**9 + nano


def classify(elements, group):
    """Recorded AW enums, explicit mission route contract; never native GT.

    D is protected-left, A/B/C straight. No group aliases are accepted. The
    pure Guard input does not reproduce Lanelet module lifetime or GO_OUT.
    """
    if group not in MISSIONS or not isinstance(elements, list):
        return dict(available=False, phase='unknown', has_color_evidence=True,
                    reason='unsupported_group_or_wire_elements')
    if any(not isinstance(e, dict) or type(e.get('color')) is not int or
           e['color'] not in (0, 1, 2, 3) or type(e.get('shape')) is not int or
           type(e.get('status')) is not int for e in elements):
        return dict(available=False, phase='unknown', has_color_evidence=True,
                    reason='unrecognized_wire_enum_schema')
    color = any(e['color'] != 0 for e in elements)
    if not elements or any(e['color'] == 0 or e['shape'] == 0 or e['status'] != 2
                           for e in elements):
        phase = 'unknown'
    elif any(e['color'] == 2 for e in elements):
        phase = 'unknown' if any(e['color'] == 3 for e in elements) else 'yellow'
    else:
        shape = 2 if group == 201326 else 1
        phase = 'green' if any(e['color'] == 3 and e['shape'] == shape
                               for e in elements) else 'stop'
    return dict(available=True, phase=phase, has_color_evidence=color,
                reason='exact_canonical_wire_AW_enums_and_explicit_route_contract')


def partial_evidence(runtime, classifier):
    """Only expose proven positive bits; absence of exact evidence is unavailable."""
    if not runtime or not classifier:
        return dict(available=False, reason='exact_source_group_raw_debug_missing')
    decision = runtime['frame_decision']
    if decision.get('planner_compliance_observations') is not True:
        return dict(available=False, reason='observed_only_normalization_not_proven')
    if (classifier.get('source_stamp_ns') != runtime['source_stamp_ns'] or
            classifier.get('group_id') != decision['group_id'] or
            classifier.get('head_id') != decision.get('head_id')):
        return dict(available=False, reason='raw_classifier_identity_mismatch')
    observed, normalized = decision.get('observed_tuples'), decision.get('normalized_tuples')
    if not isinstance(observed, list) or not isinstance(normalized, list):
        return dict(available=False, reason='raw_tuple_evidence_missing')
    colors = {lamp[0] for lamp in observed if isinstance(lamp, list) and len(lamp) == 3
              and lamp[0] in ('red', 'yellow', 'green')}
    bits = classifier.get('bits')
    valid_bits = (isinstance(bits, dict) and set(bits) == {'red', 'yellow', 'green', 'left_arrow'}
                  and all(value is None or type(value) is int and value in (0, 1)
                          for value in bits.values()))
    if valid_bits:
        colors.update('green' if key == 'left_arrow' else key
                      for key, value in bits.items() if value == 1)
    elif not colors and classifier.get('reason') != 'no_selected_crop':
        return dict(available=False, reason='positive_bit_presence_unprovable')
    return dict(available=True, has_color_evidence=bool(colors),
                reason='original_observed_tuples_and_exact_source_classifier_positive_bits',
                observed_tuples=observed, normalized_tuples=normalized, bits=bits,
                classifier_reason=classifier.get('reason'),
                reconstructed_unknown_color_markers=[dict(color={'red': 1, 'yellow': 2, 'green': 3}[c],
                    shape=0, status=0) for c in sorted(colors)])


def load_wrapper(directory):
    directory = directory.resolve(strict=True)
    events = list(jsonl(directory / 'wrapper_events.jsonl'))
    if not any(e.get('event') == 'validator_returned' for e in events):
        raise ValueError('Completed wrapper required: ' + str(directory))
    selection = json.loads((directory / 'selection.json').read_text())
    runtime, classifier, records = {}, {}, []
    observed_only = Counter()
    for row in jsonl(directory / 'observer/observations.jsonl'):
        topic = row.get('topic', '')
        if topic.endswith(('/debug_runtime', '/debug_classifier')):
            body = json.loads(row['message']['data'])
            if topic.endswith('/debug_classifier'):
                classifier[(body.get('source_stamp_ns'), body.get('group_id'))] = body
            elif isinstance(body.get('frame_decision'), dict):
                decision = body['frame_decision']
                key = body.get('source_stamp_ns'), decision.get('group_id')
                if key not in runtime:
                    runtime[key] = body
                    observed_only[str(decision.get('planner_compliance_observations'))] += 1
        elif topic.endswith('/traffic_signals'):
            message = row.get('message') or {}
            stamp = source_ns(message.get('stamp'))
            for group in message.get('traffic_light_groups', []):
                records.append(dict(record_id=row['record_id'], receipt_ros_ns=row['receipt_ros_ns'],
                    source_stamp_ns=stamp, group_id=group.get('traffic_light_group_id'),
                    elements=group.get('elements')))
    videos = {}
    for row in jsonl(directory / 'front_video/frames.jsonl'):
        stamp = (row.get('image') or {}).get('source_stamp_ns')
        if stamp:
            videos.setdefault(stamp, dict(frame_index=row['frame_index'], pts_sec=row['pts_sec'],
                runtime_exact_image_stamp_match=row.get('runtime_exact_image_stamp_match')))
    prior_group, segment = None, 0
    for row in records:
        group = row['group_id']
        if group != prior_group:
            segment += 1
            prior_group = group
        row['segment'] = segment
        row['front_video'] = videos.get(row['source_stamp_ns'])
        row['streams'] = {'wire': classify(row['elements'], group)}
        marker = dict(row['streams']['wire'])
        if marker['available'] and marker['phase'] == 'unknown' and not marker['has_color_evidence']:
            key = row['source_stamp_ns'], group
            evidence = partial_evidence(runtime.get(key), classifier.get(key))
            marker['partial_evidence'] = evidence
            marker['available'] = evidence['available']
            if evidence['available']:
                marker['has_color_evidence'] = evidence['has_color_evidence']
        row['streams']['counterfactual_partial_marker'] = marker
    required = selection.get('required_green_observations') or 1
    timeout = selection.get('signal_observation_timeout_sec') or 1.
    return dict(wrapper=str(directory), required_green=required, timeout_sec=timeout,
        required_green_basis=('explicit_recorded_wrapper_selection' if selection.get('required_green_observations')
                              else 'explicit_analysis_assumption_production_confirmation1'),
        observed_only_runtime_counts=dict(observed_only),
        input_hashes={name: digest(directory / name) for name in
                      ('selection.json', 'observer/observations.jsonl', 'front_video/frames.jsonl')},
        records=records)


def schedules(records, phases_ms):
    yield 'receipt_events', [(row['receipt_ros_ns'], i) for i, row in enumerate(records)]
    if not records:
        return
    step = 50_000_000
    for phase in phases_ms:
        index, ticks = 0, []
        for tick in range(records[0]['receipt_ros_ns'] + phase * 1_000_000,
                          records[-1]['receipt_ros_ns'] + 1, step):
            while index + 1 < len(records) and records[index + 1]['receipt_ros_ns'] <= tick:
                index += 1
            ticks.append((tick, index))
        yield '20Hz_phase_' + str(phase) + 'ms', ticks


def cpp_replay(binary, run, phases_ms, required_counts=(1, 2), graces=(0., .2, .3)):
    records = run['records']
    epoch = (records[0]['receipt_ros_ns'] // 10**9 - 1000) * 10**9
    requests, lines, last_segment = [], [], {}
    for scenario, ticks in schedules(records, phases_ms):
        for tick, index in ticks:
            row = records[index]
            if row['group_id'] not in MISSIONS:
                continue
            for stream in STREAMS:
                evidence = row['streams'][stream]
                for required in required_counts:
                    for grace in graces:
                        key = f'{scenario}_{stream}_{required}_{grace}_{row["group_id"]}'
                        reset = last_segment.get(key) != row['segment'] or not evidence['available']
                        last_segment[key] = row['segment']
                        phase = evidence['phase'] if evidence['available'] else 'unknown'
                        color = evidence['has_color_evidence'] if evidence['available'] else True
                        request = dict(scenario=scenario, stream=stream, grace_sec=grace,
                            required_green=required, wire_index=index, tick_ros_ns=tick,
                            group_id=row['group_id'], available=evidence['available'], phase=phase)
                        lines.append(f'{len(requests)} {key} {required} {grace} '
                            f'{phase} {row["source_stamp_ns"] or 0} {tick} {epoch} {int(color)} '
                            f'{int(reset)} {run["timeout_sec"]}')
                        requests.append(request)
    result = subprocess.run([str(binary)], input='\n'.join(lines) + '\n',
                            capture_output=True, text=True, check=True)
    rows = list(csv.DictReader(result.stdout.splitlines()))
    if len(rows) != len(requests):
        raise ValueError('C++ output row count mismatch')
    for result in rows:
        row = requests[int(result['id'])]
        row.update(require_stop=bool(int(result['require_stop'])) if row['available'] else None,
                   green_count=int(result['green_count']), grace_active=bool(int(result['grace_active'])))
    return requests


def summarize(run, results):
    grouped, paired = defaultdict(list), defaultdict(dict)
    for result in results:
        grouped[(result['scenario'], result['stream'], result['required_green'], result['grace_sec'], result['group_id'])].append(result)
        paired[(result['scenario'], result['stream'], result['required_green'], result['tick_ros_ns'], result['wire_index'])][result['grace_sec']] = result
    summaries = []
    for (scenario, stream, required, grace, group), rows in grouped.items():
        counts = Counter()
        transitions, previous = [], None
        for row in rows:
            counts['ticks'] += 1
            counts['unavailable' if row['require_stop'] is None else 'STOP' if row['require_stop'] else 'GO'] += 1
            counts['grace_active_ticks'] += row['grace_active']
            wire = run['records'][row['wire_index']]
            if previous is None or row['require_stop'] != previous['require_stop']:
                transitions.append(dict(tick_ros_ns=row['tick_ros_ns'], require_stop=row['require_stop'],
                    wire_record_id=wire['record_id'], wire_source_stamp_ns=wire['source_stamp_ns'],
                    front_video=wire['front_video'], phase=row['phase']))
            previous = row
        summaries.append(dict(scenario=scenario, stream=stream, required_green=required, grace_sec=grace,
            group_id=group, section=MISSIONS[group], counts=dict(counts), transitions=transitions))
    differences, invariance, per_setting = [], Counter(), defaultdict(Counter)
    for (scenario, stream, required, tick, index), pair in paired.items():
        a = pair[0.]
        for grace, b in pair.items():
            if grace == 0.:
                continue
            setting = per_setting[(required, grace)]
            if a['require_stop'] is None or b['require_stop'] is None:
                invariance['unavailable_pairs'] += 1
                setting['unavailable_pairs'] += 1
                continue
            equal = a['require_stop'] == b['require_stop']
            for counter in (invariance, setting):
                counter['available_pairs'] += 1
                counter['equal_stop_request_pairs' if equal else 'changed_stop_request_pairs'] += 1
                if a['phase'] in ('stop', 'yellow'):
                    counter['explicit_stop_or_yellow_pairs'] += 1
                    counter['explicit_stop_or_yellow_delay_violations'] += not (a['require_stop'] and b['require_stop'])
            if not equal:
                wire = run['records'][index]
                differences.append(dict(scenario=scenario, stream=stream, required_green=required,
                    grace_sec=grace, tick_ros_ns=tick, group_id=wire['group_id'], phase=a['phase'],
                    source_stamp_ns=wire['source_stamp_ns'], wire_receipt_ros_ns=wire['receipt_ros_ns'],
                    front_video=wire['front_video'], baseline_require_stop=a['require_stop'],
                    grace_require_stop=b['require_stop'], grace_active=b['grace_active']))
    for row in run['records']:
        row['receipt_event_guard'] = {}
    for row in results:
        if row['scenario'] == 'receipt_events':
            key = row['stream'] + ':green' + str(row['required_green']) + ':grace' + str(row['grace_sec'])
            run['records'][row['wire_index']]['receipt_event_guard'][key] = {
                key: row[key] for key in ('available', 'require_stop', 'green_count', 'grace_active')}
    return dict(per_mission=summaries, full_input_negative_invariance=dict(invariance),
        negative_invariance_per_setting=[dict(required_green=required, grace_sec=grace, counts=dict(counts))
            for (required, grace), counts in per_setting.items()], changed_ticks=differences)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wrappers', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--grid-phase-ms', nargs='+', type=int, default=[0, 10, 20, 30, 40])
    parser.add_argument('--required-green', nargs='+', type=int, default=[1, 2])
    parser.add_argument('--grace-sec', nargs='+', type=float, default=[0., .2, .3])
    args = parser.parse_args()
    if any(not 0 <= phase < 50 for phase in args.grid_phase_ms):
        parser.error('20-Hz phases must be integer milliseconds in0..49')
    if any(value not in (1, 2, 3) for value in args.required_green):
        parser.error('required-green must be1..3')
    if 0. not in args.grace_sec or any(not 0. <= value <= .3 for value in args.grace_sec):
        parser.error('grace-sec must include baseline0 and remain in0...3')
    output = args.output.resolve()
    output.relative_to(ROOT / 'artifacts/evaluation')
    if output.exists():
        raise FileExistsError('Fresh analysis output required')
    source_hash = digest(HEADER)
    runs = []
    with tempfile.TemporaryDirectory(prefix='traffic-guard-read-only-') as temp:
        binary = Path(temp) / 'guard'
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
            '-I' + str(HEADER.parents[2]), str(CPP), '-o', str(binary)], check=True)
        for wrapper in args.wrappers:
            run = load_wrapper(wrapper)
            results = cpp_replay(binary, run, args.grid_phase_ms, args.required_green, args.grace_sec)
            run['analysis'] = summarize(run, results)
            runs.append(run)
    if digest(HEADER) != source_hash:
        raise RuntimeError('Guard source changed during replay; refuse mixed revision')
    report = dict(schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
        source_guard_header=str(HEADER), source_guard_sha256=source_hash,
        cpp_sha256=digest(CPP), adapter_sha256=digest(Path(__file__)), runs=runs,
        analysis_settings=dict(required_green=args.required_green, grace_sec=args.grace_sec,
                               grid_phase_ms=args.grid_phase_ms),
        assumptions=dict(timeout_sec='recorded selection else explicit1.0 assumption',
            clock='original observer ROS receipt; integer ns subtraction before double seconds',
            planner_cadence='20-Hz synthetic grids with five phases, not recorded planner calls',
            route_direction_contract='exact regulatory201351/365/290 straight;201326 protected-left; no aliases/native GT',
            outer_planner_policy='Not simulated: unseen deferral, competition fail-open, stop-line/GO_OUT, path insertion, vehicle/controller.'),
        limitations=[__doc__, 'Counterfactual partial markers only expose proven raw positive bits as known-color UNKNOWN; they do not create a color authorization.',
            'Unavailable partial evidence resets replay history conservatively and is reported UNAVAILABLE, never an invented observation.',
            'Wire stream is actual recorded canonical input. Partial-marker stream is not the wire received by the historical planner.',
            'No latency/STOP-request change is a causal driving, jerk, safety, native accuracy or official penalty result.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(output), guard_sha256=source_hash,
        runs=[dict(wrapper=run['wrapper'], invariance=run['analysis']['full_input_negative_invariance']) for run in runs])))


if __name__ == '__main__':
    main()
