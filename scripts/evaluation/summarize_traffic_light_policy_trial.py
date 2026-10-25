#!/usr/bin/env python3
"""Summarize CLOSED development policy trials without editing their evidence.

Separates phase execution, observed motion, native crossing proxies, fault
coverage, observation age and validator stop/P cleanup. Goal SUCCESS is never
promoted to a signal-policy pass. No ROS/model/GPU/native API dependencies.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parents[2]


def numbers(values):
    return [float(value) for value in values if isinstance(value, (int, float))
            and not isinstance(value, bool) and math.isfinite(value)]


def statistics_of(values):
    values = sorted(numbers(values))
    if not values: return {'count': 0, 'min': None, 'mean': None, 'p50': None, 'p95': None, 'max': None}
    return dict(count=len(values), min=values[0], mean=statistics.mean(values),
                p50=statistics.median(values), p95=values[math.ceil(.95*len(values))-1], max=values[-1])


def stamp_ns(stamp):
    if not isinstance(stamp, dict): return None
    for second, nano in (('sec', 'nanosec'), ('secs', 'nsecs')):
        if type(stamp.get(second)) is int and type(stamp.get(nano)) is int:
            return stamp[second]*1000000000+stamp[nano]
    return None


def stage_name(value):
    return value.get('stage') if isinstance(value, dict) else value if isinstance(value, str) else None


def motion_samples(observer_rows, phase_rows):
    local, ego, fallback = [], [], []
    for row in observer_rows:
        if row.get('event') != 'ROS_observation': continue
        message, topic, when = row.get('message', {}), row.get('topic'), row.get('receipt_monotonic')
        try:
            if topic == '/localization/kinematic_state':
                position, velocity = message['pose']['pose']['position'], message['twist']['twist']['linear']
                target = local
            elif topic == '/Ego_topic':
                position, velocity, target = message['position'], message['velocity'], ego
            else: continue
            xyz = [float(position[axis]) for axis in ('x', 'y', 'z')]
            vector = [float(velocity[axis]) for axis in ('x', 'y', 'z')]
            if len(numbers(xyz+vector+[when])) != 7: continue
            target.append(dict(monotonic=when, position=xyz, speed=math.sqrt(sum(value*value for value in vector))))
        except (KeyError, TypeError, ValueError): continue
    for row in phase_rows:
        if row.get('event') == 'input_observation' and row.get('input_name') == 'odom':
            try:
                position, vector, when = row['position_xyz'], row['velocity_xyz'], row['receipt_monotonic']
                if len(position) != 3 or len(vector) != 3 or len(numbers(position+vector+[when])) != 7: continue
                fallback.append(dict(monotonic=when, position=position,
                                     speed=math.sqrt(sum(value*value for value in vector))))
            except (KeyError, TypeError, ValueError): continue
    return (local if local else fallback), ego, ('observer/localization' if local else
                                               'phase_controller/odom' if fallback else 'missing')


def summarize_motion(samples, start, end):
    if start is None or end is None or end < start:
        return {'sample_count': 0, 'coverage_verified': False, 'reason': 'stage_interval_unverified'}
    rows = sorted((row for row in samples if start <= row['monotonic'] <= end), key=lambda row: row['monotonic'])
    if not rows:
        return {'sample_count': 0, 'coverage_verified': False, 'reason': 'no_samples_in_stage'}
    first = rows[0]['position']
    displacement = [math.dist(row['position'][:2], first[:2]) for row in rows]
    gaps = [b['monotonic']-a['monotonic'] for a, b in zip(rows, rows[1:])]
    coverage = (len(rows) >= 2 and rows[0]['monotonic']-start <= .5 and end-rows[-1]['monotonic'] <= .5
                and max(gaps, default=float('inf')) <= .5)
    speed_over = max(row['speed'] for row in rows) > .3
    distance_over = max(displacement) > .25
    return dict(sample_count=len(rows), speed_mps=statistics_of([row['speed'] for row in rows]),
                maximum_XY_displacement_from_first_sample_m=max(displacement),
                final_XY_displacement_from_first_sample_m=displacement[-1],
                sample_span_sec=rows[-1]['monotonic']-rows[0]['monotonic'],
                first_sample_delay_sec=rows[0]['monotonic']-start, last_sample_gap_sec=end-rows[-1]['monotonic'],
                maximum_sample_gap_sec=max(gaps) if gaps else None, coverage_verified=coverage,
                speed_gt_0_3_mps=speed_over, displacement_gt_0_25_m=distance_over,
                departure_proxy_any=speed_over or distance_over, departure_proxy_both=speed_over and distance_over,
                reason='sampled_motion_proxy_not_official_vehicle_departure')


def fault_coverage(wrapper_rows, phase_rows, start, end, mode, group_id):
    if start is None or end is None: return {'source_frames': 0, 'coverage_verified': False}
    rows = [row for row in wrapper_rows if row.get('event') == 'signal_proxy'
            and start <= row.get('monotonic', -float('inf')) <= end]
    commands = [row for row in phase_rows if row.get('event') == 'fault_control_published'
                and start <= row.get('receipt_monotonic', -float('inf')) <= end]
    eligible, applied, effects = 0, 0, 0
    for row in rows:
        applied += row.get('mode') == mode
        input_ids, output_ids = row.get('input_ids') or [], row.get('output_ids')
        if group_id not in input_ids: continue
        eligible += 1
        if mode == 'drop': effects += row.get('mode') == mode and output_ids is None
        elif mode == 'wrong_id':
            effects += row.get('mode') == mode and output_ids is not None and group_id not in output_ids and 0 in output_ids
        else: effects += row.get('mode') == 'none' and input_ids == output_ids
    return dict(source_frames=len(rows), eligible_target_input_frames=eligible, expected_mode=mode,
                actual_mode_counts=dict(Counter(row.get('mode', 'unknown') for row in rows)),
                requested_command_mode_counts=dict(Counter(row.get('payload', {}).get('mode', 'unknown') for row in commands)),
                mode_applied_frames=applied, effect_confirmed_on_target_frames=effects,
                effect_fraction_of_eligible_target_frames=effects/eligible if eligible else None,
                coverage_verified=bool(rows and eligible and effects == eligible and applied == len(rows)),
                note='Proxy observations prove mode/identity/drop effect only, not native color or road safety.')


def summarize_stages(phase_rows, wrapper_rows, observer_rows, metadata, group_id):
    local, ego, local_source = motion_samples(observer_rows, phase_rows)
    expected = metadata.get('stages', [])
    expected_source = 'frozen phase_controller/metadata.json'
    if not expected or not all(isinstance(stage, dict) and stage_name(stage) for stage in expected):
        expected = [row['stage'] for row in phase_rows if row.get('event') == 'stage_requested' and isinstance(row.get('stage'), dict)]
        expected_source = 'observed request dictionaries only; absent/descriptive metadata cannot enumerate unexecuted stages'
    teardown = numbers([row.get('monotonic') for row in wrapper_rows if row.get('event') == 'validator_safety_teardown_started'])
    phase_end = max(numbers([row.get('monotonic', row.get('receipt_monotonic')) for row in phase_rows]), default=None)
    stop_candidates = [value for value in [min(teardown) if teardown else None, phase_end] if value is not None]
    observation_end = min(stop_candidates) if stop_candidates else None
    output = []
    for index, planned in enumerate(expected):
        name = stage_name(planned)
        own = [row for row in phase_rows if stage_name(row.get('stage')) == name]
        requested = next((row for row in own if row.get('event') == 'stage_requested'), None)
        confirmed = next((row for row in own if row.get('event') == 'stage_native_confirmed'), None)
        finished = next((row for row in own if row.get('event') == 'stage_complete'), None)
        start = confirmed.get('native_confirmed_monotonic', confirmed.get('monotonic')) if confirmed else None
        confirmation_evidence = 'stage_native_confirmed' if start is not None else None
        if start is None and finished and numbers([finished.get('native_confirmed_monotonic')]):
            # Some closed green-loss helpers recorded the actual confirmed
            # clock only in stage_complete. This is explicit evidence, not an
            # inferred start from requested duration or today's source code.
            start = finished['native_confirmed_monotonic']
            confirmation_evidence = 'stage_complete.native_confirmed_monotonic'
        release = planned.get('action') == 'reset_to_natural'
        terminal = planned.get('action') == 'controlled_go_until_owner_teardown'
        attempted = next((row for row in own if row.get('event') == 'natural_release_attempted'), None)
        if release and requested: start = requested.get('monotonic')
        end = finished.get('monotonic') if finished else observation_end if start is not None else None
        owner_stop = min(teardown) if teardown else None
        terminal_chronology_invalid = bool(terminal and start is not None and owner_stop is not None and owner_stop < start)
        # Native confirmation, owner teardown and the later child cleanup are
        # different times. Do not include stop/P cleanup in a terminal GO span.
        if terminal and end is not None and owner_stop is not None:
            end = min(end, owner_stop) if not terminal_chronology_invalid else None
        next_requests = numbers([row.get('monotonic') for row in phase_rows if row.get('event') == 'stage_requested'
                                 and requested and row.get('monotonic', 0) > requested.get('monotonic', float('inf'))])
        if not finished and next_requests and end is not None: end = min(end, min(next_requests))
        duration = planned.get('seconds')
        terminal_basis = bool(terminal and finished and finished.get('completion_basis') ==
                              'SIGTERM_after_verified_terminal_controlled_GO')
        terminal_confirmed = bool(terminal and type(planned.get('color')) is int and planned['color'] in (16, 33)
                                  and start is not None)
        complete = bool(finished and ((release and attempted) or terminal_basis and terminal_confirmed
                        and end is not None and end >= start and not terminal_chronology_invalid
                        or not terminal and not release and start is not None and end is not None
                        and duration is not None and end-start+1e-6 >= duration))
        status = ('controlled_GO_completed_at_owner_teardown' if complete and terminal else
                  'natural_release_attempted_not_mode_certified' if complete and release else 'completed' if complete else
                  'partial' if start is not None else 'requested_but_native_unconfirmed' if requested else 'unexecuted')
        native = [row for row in own if row.get('event') == 'native_signal_verified_actual']
        native_times = numbers([row.get('response_monotonic', row.get('monotonic'))
                                for row in native if row.get('verified')])
        must_stop = planned.get('color') in (1, 4, 5) and not release
        motion = {'localization': summarize_motion(local, start, end), 'raw_Ego': summarize_motion(ego, start, end)}
        output.append(dict(stage=name, planned=planned, execution_status=status, executed_complete=complete,
            completion_label_present=finished is not None, requested_monotonic=requested.get('monotonic') if requested else None,
            native_confirmation_start_evidence=confirmation_evidence,
            native_confirmed_start_monotonic=start if not release else None, observed_interval_start_monotonic=start,
            observed_interval_end_monotonic=end, native_or_action_complete_monotonic=finished.get('monotonic') if finished else None,
            actual_native_state_counts=dict(Counter(str(row.get('actual_state')) for row in native if row.get('verified'))),
            native_verified_samples=sum(bool(row.get('verified')) for row in native),
            native_requested_match_samples=sum(bool(row.get('matches_requested')) for row in native),
            actual_native_first_verified_response_monotonic=min(native_times) if native_times else None,
            actual_native_last_verified_response_monotonic=max(native_times) if native_times else None,
            terminal_controlled_GO=terminal, terminal_positive_GO_confirmed=terminal_confirmed,
            terminal_completion_basis_valid=terminal_basis,
            owner_teardown_context_verified=bool(terminal_basis and owner_stop is not None and start is not None
                                                 and start <= owner_stop <= finished['monotonic']),
            owner_teardown_precedes_native_confirmation=terminal_chronology_invalid,
            stationary_expected_proxy=must_stop, motion=motion, localization_motion_source=local_source,
            departure_proxy_observed={source: (values.get('departure_proxy_any') if must_stop else None)
                                      for source, values in motion.items()},
            fault_actual_coverage=fault_coverage(wrapper_rows, phase_rows, start, end, planned.get('fault', 'none'), group_id),
            natural_mode_restoration_certified=False if release or terminal else None))
    return output, expected_source


def summarize_crossings(document):
    events, counts = [], {bound: Counter() for bound in ('0.25', '0.5')}
    for row in document.get('crossings', []):
        evaluation = row.get('native_bracket_evaluation', {})
        for bound in counts:
            value = evaluation.get(bound, {})
            if not value.get('verified'): counts[bound]['unverified'] += 1
            elif value.get('route_permission_proxy') is True: counts[bound]['verified_permission_proxy'] += 1
            elif value.get('route_permission_proxy') is False: counts[bound]['verified_no_permission_proxy'] += 1
            else: counts[bound]['unverified'] += 1
        events.append({key: row.get(key) for key in ('record_id', 'group_id', 'segment_index', 'direction',
            'direction_configured', 'point_xy_m', 'estimated_source_monotonic', 'estimated_source_stamp_ns',
            'pose_time_verified', 'native_bracket_evaluation', 'crossing_jpeg_bracket')})
    return dict(events=events, counts_by_native_age_bound_sec={key: dict(value) for key, value in counts.items()},
                official_violation_certified=False, note='Finite front-point/native-state proxy only; unknown is not permission or violation.')


def summarize_ages_and_runtime(rows, signal_topic):
    signal_ages, reported_ages, debug_ages, times = [], [], [], {}
    synthetic, observed, held, invalid_stamps, debug_count = 0, 0, 0, 0, 0
    for row in rows:
        message, receipt_ns = row.get('message', {}), row.get('receipt_ros_ns')
        if row.get('event') != 'ROS_observation': continue
        if row.get('topic') == signal_topic:
            source = stamp_ns(message.get('stamp'))
            if source and source > 0 and isinstance(receipt_ns, int): signal_ages.append((receipt_ns-source)/1e9)
            else: invalid_stamps += 1
        if not str(row.get('topic', '')).endswith('/debug_runtime'): continue
        report = message.get('parsed_json')
        if not isinstance(report, dict):
            try: report = json.loads(message.get('data', ''))
            except (ValueError, TypeError): continue
        decision = report.get('frame_decision')
        if not isinstance(decision, dict): continue
        debug_count += 1
        observation = decision.get('observation_timing', {})
        if observation:
            synthetic += observation.get('output_is_observed') is False
            observed += observation.get('output_is_observed') is True
            held += observation.get('output_is_held') is True
            reported_ages += numbers([observation.get('output_observation_age_sec')])
            source = observation.get('adopted_observation_source_stamp_ns')
            if type(source) is int and source > 0 and type(receipt_ns) is int:
                debug_ages.append((receipt_ns-source)/1e9)
        for key, value in (report.get('frame_timing') or {}).items():
            if key.endswith('_sec') and numbers([value]): times.setdefault(key, []).append(value*1000.)
    return dict(standard_signal_stamp_age_at_observer_sec=statistics_of(signal_ages),
                invalid_or_zero_standard_signal_stamps=invalid_stamps,
                debug_reported_adopted_observation_age_sec=statistics_of(reported_ages),
                debug_adopted_source_age_at_observer_sec=statistics_of(debug_ages),
                runtime_frames=debug_count, observed_output_frames=observed, synthetic_fallback_frames=synthetic,
                held_observation_frames=held, runtime_ms={key: statistics_of(value) for key, value in times.items()},
                limitations=['Independent aggregates, not a nearest-receipt frame join.',
                    'Synthetic RED can have a fresh wire stamp but no adopted lamp observation.',
                    'near_rescue_detector_and_association_sec is a subset of selection; do not add them twice.'])


def summarize_safety(wrapper_rows, validator):
    result, safety, events = validator.get('result', {}), validator.get('safety', {}), validator.get('events', [])
    if not safety: safety = result.get('safety', {})
    shutdown = safety.get('shutdown', {})
    stop = shutdown.get('stop', {})
    parked = (shutdown.get('status') == 'VERIFIED' and stop.get('park_verified') is True and stop.get('gear') == 1)
    preliminary = [row for row in events if row.get('event') == 'CASE_RESULT']
    goal = result.get('drive_reason') == 'SUCCESS' or any(row.get('reason') == 'SUCCESS' for row in preliminary)
    final_success = result.get('status') == 'PASS' and result.get('reason') == 'SUCCESS'
    wrapper_returns = [row.get('returncode') for row in wrapper_rows if row.get('event') == 'validator_returned']
    return dict(preliminary_CASE_RESULT=preliminary, goal_success_observed=goal, validator_final_status=result.get('status'),
                validator_final_reason=result.get('reason'), validator_final_success=final_success,
                safety_shutdown_status=shutdown.get('status'), stop_P_verified=bool(parked), stop_P_receipt=stop,
                wrapper_validator_returncodes=wrapper_returns,
                wrapper_error_events=[row for row in wrapper_rows if row.get('event') in
                    ('child_error_abort', 'proxy_fatal_abort', 'owned_child_timeout', 'signal_proxy_error', 'invalid_fault_control')],
                separate_later_P_recovery_is_not_this_trial_completion=True,
                goal_SUCCESS_is_signal_policy_pass=False)


def summarize(phase_rows, wrapper_rows, observer_rows, crossings, metadata=None, phase_summary=None,
              selection=None, validator=None):
    metadata, selection, validator = metadata or {}, selection or {}, validator or {}
    group = metadata.get('group', {}).get('group_id', selection.get('group_id'))
    stages, source = summarize_stages(phase_rows, wrapper_rows, observer_rows, metadata, group)
    errors = [row for row in phase_rows if row.get('event') in ('experiment_error', 'native_read_error', 'natural_reset_error')]
    stages_stop = [stage for stage in stages if stage['stationary_expected_proxy']]
    safety = summarize_safety(wrapper_rows, validator)
    return dict(schema='traffic_light_policy_trial_summary_v1', development_only=True,
        experiment=metadata.get('experiment', selection.get('experiment')), group_id=group, stage_plan_source=source,
        stages=stages, controlled_phase_summary=phase_summary or {}, controlled_phase_errors=errors,
        start_gate_confirmed=any(row.get('event') == 'experiment_start_gate_confirmed' for row in phase_rows),
        stationary_stress_stages_complete=bool(stages_stop) and all(stage['executed_complete'] for stage in stages_stop),
        full_phase_program_complete=bool(stages) and source == 'frozen phase_controller/metadata.json'
                                    and all(stage['executed_complete'] for stage in stages),
        full_phase_plan_enumeration_verified=source == 'frozen phase_controller/metadata.json',
        observed_departure_proxy_stage_names=[stage['stage'] for stage in stages_stop
            if any(value is True for value in stage['departure_proxy_observed'].values())],
        crossings=summarize_crossings(crossings),
        timing=summarize_ages_and_runtime(observer_rows, selection.get('signal_topic', '/perception/traffic_light_recognition/traffic_signals')),
        safety=safety, automatic_overall_pass_issued=False,
        phase_child_exit_events=[row for row in wrapper_rows if row.get('event') == 'child_finished'
                                 and row.get('name') == 'phase_controller'],
        validator_teardown_events=[row for row in wrapper_rows if row.get('event') == 'validator_safety_teardown_started'],
        native_cleanup_diagnostics=dict(
            manual_natural_restore_required=(phase_summary or {}).get('manual_natural_restore_required',
                metadata.get('manual_natural_restore_required')),
            native_SET_in_cleanup=(phase_summary or {}).get('native_SET_in_cleanup',
                bool((phase_summary or {})['native_writes_in_cleanup'])
                if type((phase_summary or {}).get('native_writes_in_cleanup')) is int else None),
            last_requested_positive=(phase_summary or {}).get('last_requested_positive'),
            last_verified_positive=(phase_summary or {}).get('last_verified_positive'),
            last_native_observation=(phase_summary or {}).get('last_native_observation'),
            current_native_state_certified=False,
            natural_mode_restoration_certified=False),
        limitations=['No retraining, runtime change or native/ROS calls; original evidence is not rewritten.',
            'Reported phases are artifact-specific: use recorded metadata, not today\'s controller program.',
            'Partial/absent green or natural-release stages do not erase completed Y/R stages; no missing label is inferred complete.',
            'Controlled terminal GO with no fixed duration is complete only with native confirmation and its owner-SIGTERM completion label.',
            'Controlled GO is not a restored natural signal program; the recorded last positive override is not current-state certification.',
            'Native override/handler failures are experiment-control artifacts, not proof of perception/planner violations.',
            'Motion thresholds >0.3m/s or >0.25m are separate diagnostic proxies, not official departure/penalty rules.',
            'Localization and raw Ego are separate; sparse/missing motion or native data never certifies success.',
            'CASE_RESULT SUCCESS precedes stop/P cleanup; a later manual recovery does not certify this run.',
            'Native bracket permission and model output are not substituted for each other.'])


def load_closed_artifacts(directory, validator_execution=None):
    if validator_execution is not None and not (validator_execution.is_dir() or validator_execution.is_file()):
        raise FileNotFoundError('Explicit validator execution directory/result JSON does not exist: '+str(validator_execution))
    evidence, warnings = [], []
    def read(path, jsonl=False):
        if not path.is_file():
            warnings.append('Missing optional/evidence file: '+str(path))
            return [] if jsonl else {}
        before = path.stat()
        payload = path.read_bytes()
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError('Evidence changed while reading; close the trial first')
        evidence.append(dict(path=str(path.resolve()), sha256=hashlib.sha256(payload).hexdigest(), bytes=len(payload)))
        if not jsonl: return json.loads(payload)
        rows = []
        for line, text in enumerate(payload.decode().splitlines(), 1):
            if not text.strip(): continue
            try:
                row = json.loads(text)
                if not isinstance(row, dict): raise ValueError('JSONL event must be an object')
                rows.append(row)
            except ValueError: warnings.append('Malformed/truncated/non-object JSONL record '+str(path)+':'+str(line))
        return rows
    phase = read(directory/'phase_controller/events.jsonl', True)
    wrapper = read(directory/'wrapper_events.jsonl', True)
    observer = read(directory/'observer/observations.jsonl', True)
    crossings = read(directory/'observer/crossings.json')
    metadata = read(directory/'phase_controller/metadata.json')
    phase_summary = read(directory/'phase_controller/summary.json')
    selection = read(directory/'selection.json')
    validator = {}
    if validator_execution is not None:
        folder = validator_execution if validator_execution.is_dir() else validator_execution.parent
        candidates = [validator_execution] if validator_execution.is_file() else list(folder.glob('planbag_*.json'))
        if not candidates and (folder/'result.json').is_file(): candidates = [folder/'result.json']
        if len(candidates) == 1: validator['result'] = read(candidates[0])
        else: warnings.append('Validator result absent/ambiguous; do not infer final PASS')
        validator['safety'] = read(folder/'safety_receipt.json')
        validator['events'] = read(folder/'events.jsonl', True)
    return summarize(phase, wrapper, observer, crossings, metadata, phase_summary, selection, validator), evidence, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact_dir', type=Path)
    parser.add_argument('--validator-execution', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirm-closed', action='store_true', required=True)
    args = parser.parse_args()
    if not args.artifact_dir.is_dir(): parser.error('Explicit closed wrapper artifact directory required')
    if args.validator_execution is not None:
        args.validator_execution = args.validator_execution.expanduser()
        if not (args.validator_execution.is_dir() or args.validator_execution.is_file()):
            parser.error('--validator-execution must name an existing execution directory or explicit result JSON')
    output = args.output.expanduser().resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh summary output required')
    report, evidence, warnings = load_closed_artifacts(args.artifact_dir, args.validator_execution)
    report.update(input_evidence=evidence, warnings=warnings, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream: json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
    print(json.dumps(dict(output=str(output), stages=[(stage['stage'], stage['execution_status']) for stage in report['stages']],
                         stop_P_verified=report['safety']['stop_P_verified'], automatic_pass=False)))


if __name__ == '__main__': main()
