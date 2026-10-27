#!/usr/bin/env python3
"""Read-only same-input 3/2 versus 7/4 replay of COMPLETED wrapper evidence.

Uses recorded observed-only normalized inputs, never held production outputs as
votes. No ROS/native API, model inference, image decode, stack or control calls.
Each result is conditional on recorded/reconstructed current-crop identity; an
unprovable qualification is unavailable, not an invented UNKNOWN or success.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'base_ws/src/perception/traffic_light_detection/scripts'))
from traffic_light_stamped_vote import StampedPhaseVote
from analyze_traffic_light_driving_comparison import jsonl, read_json, lamp_label, permission
from observe_traffic_light_compliance import GROUP_IDS, native_bracket

MISSIONS = {201351: 'A', 201365: 'B', 201290: 'C', 201326: 'D'}
CASES = {'vote3_strict': dict(window_size=3, max_age_sec=.5, startup_passthrough=False),
         'vote7_passthrough': dict(window_size=7, max_age_sec=.9, startup_passthrough=True)}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def box(value):
    return (isinstance(value, list) and len(value) == 4 and all(finite(x) for x in value)
            and value[2] > value[0] and value[3] > value[1])


def identity_for(frame, selection, classifier):
    """Do not equate a retained head, raw lamp or nearby timestamp with a crop."""
    decision, group, source = frame['decision'], frame['group_id'], frame['source_stamp_ns']
    logged = decision.get('stamped_vote') or {}
    if logged.get('enabled') and type(logged.get('identity_valid')) is bool:
        return dict(value=logged['identity_valid'], basis='original_logged_stamped_vote_identity_valid',
                    independently_recomputed=False)
    if group not in MISSIONS:
        return dict(value=False, basis='not_a_mission_regulatory_group')
    if selection is None:
        return dict(value=None, basis='exact_source_group_selection_missing')
    if selection.get('status') != 'matched':
        return dict(value=False, basis='current_selection_not_matched_hard_identity_loss')
    head = decision.get('head_id')
    if head not in GROUP_IDS[group] or selection.get('head_id') != head:
        return dict(value=False, basis='current_head_not_qualified_for_this_mission')
    if selection.get('reason') not in ('map_head_association', 'visual_continuation_match'):
        return dict(value=False, basis='selection_reason_not_current_qualified_association')
    if classifier is None:
        return dict(value=None, basis='exact_source_group_current_crop_evidence_missing')
    if not box(classifier.get('detector_bbox_xyxy')) or not box(classifier.get('crop_xyxy')):
        return dict(value=None, basis='matched_selection_without_provable_current_detector_crop')
    if (classifier.get('source_stamp_ns') != source or classifier.get('group_id') != group
            or classifier.get('head_id') != head):
        return dict(value=None, basis='current_crop_identity_or_source_does_not_match_selection')
    roi = selection.get('roi') or {}
    if (selection.get('mode') != 'map_association' or selection.get('source_stamp_ns') != source
            or selection.get('group_id') != group or selection.get('projection_source_stamp_ns') != source
            or roi.get('eligible') is not True or roi.get('head_id') != head or roi.get('group_id') != group
            or type(selection.get('box_index')) is not int or selection['box_index'] < 0):
        return dict(value=None, basis='exact_current_map_ROI_qualification_not_provable')
    clock = frame.get('guard_clock_ns')
    if clock is None or not 0 <= clock-source <= 500000000:
        return dict(value=None, basis='current_image_freshness_for_legacy_identity_not_provable')
    # These are the recorded selector's current eligible map association and
    # actual crop, not its remembered previous head. The legacy run did not log
    # the future vote caller's final route/packet recheck; expose that limitation.
    return dict(value=True, basis='reconstructed_exact_source_group_eligible_map_ROI_and_current_crop',
                final_vote_instant_route_and_packet_recheck_not_independently_logged=True)


def make_frame(body, row):
    decision = body.get('frame_decision')
    source = body.get('source_stamp_ns')
    if not isinstance(decision, dict) or type(source) is not int or source <= 0:
        return None
    group = decision.get('group_id')
    if type(group) is not int or group < 0:
        return None
    vote = decision.get('stamped_vote') or {}
    clock = (decision.get('observation_timing') or {}).get('guard_clock_sec')
    confidence = (decision.get('raw_observation_confidence') if 'raw_observation_confidence' in decision
                  else vote.get('input_confidence'))
    return dict(source_stamp_ns=source, group_id=group, head_id=decision.get('head_id'),
        receipt_ros_ns=row.get('receipt_ros_ns'), receipt_monotonic=row.get('receipt_monotonic'),
        record_id=row.get('record_id'), guard_clock_sec_as_logged=clock,
        guard_clock_ns=round(clock*1e9) if finite(clock) and clock > 0 else None,
        raw_observation_confidence=confidence, normalized_tuples=decision.get('normalized_tuples'),
        observed_tuples=decision.get('observed_tuples'), policy_reason=decision.get('policy_reason'),
        planner_compliance_observations=decision.get('planner_compliance_observations'),
        decision=decision)


def native_refs(frame, samples):
    group = frame['group_id']
    if group not in GROUP_IDS or not finite(frame.get('receipt_monotonic')) or type(frame.get('receipt_ros_ns')) is not int:
        return dict(available=False, reason='native_source_clock_mapping_unavailable')
    moment = frame['receipt_monotonic']-(frame['receipt_ros_ns']-frame['source_stamp_ns'])/1e9
    return dict(source_monotonic_estimate=moment,
        clock_basis='observer_receipt_monotonic_minus_receipt_ROS_to_original_image_ROS_age_assumption',
        physical_render_GT_or_accuracy=False, required_route_native_bit=32 if group == 201326 else 16,
        brackets={str(bound): [native_bracket(samples, head, moment, bound) for head in GROUP_IDS[group]]
                  for bound in (.25, .5)})


def output_record(result, source):
    timing, debug = result['observation_timing'], result['debug']
    adopted = timing.get('adopted_observation_source_stamp_ns')
    return dict(available=True, lamps=result['lamps'], label=lamp_label(result['lamps']),
        confidence=result['confidence'], reason=result['reason'],
        adopted_observation_source_stamp_ns=adopted,
        wire_source_stamp_ns=adopted if timing['output_is_observed'] else source,
        wire_stamp_basis='adopted_original_observation' if timing['output_is_observed'] else 'current_image_UNKNOWN_decision_not_observation',
        adopted_observation_head_id=timing.get('adopted_observation_head_id'),
        output_is_held=timing['output_is_held'], output_origin=timing['output_origin'],
        reset_reason=debug['reset_reason'], winner_vote_count=debug['winner_vote_count'],
        vote_source_stamps_ns=debug['vote_source_stamps_ns'], vote_head_ids=debug['vote_head_ids'],
        vote_tuples=debug['vote_tuples'], available_vote_count=debug['available_vote_count'],
        window_full=debug['window_full'], startup_passthrough_active=debug['startup_passthrough_active'],
        accepted_count=debug.get('accepted_count'), startup_complete=debug.get('startup_complete'),
        startup_policy=debug.get('startup_policy'), candidate_version=debug.get('candidate_version'))


def replay_frames(frames, selections, classifiers):
    votes = {name: StampedPhaseVote(**config) for name, config in CASES.items()}
    counts, previous, segment = Counter(), None, 0
    for frame in frames:
        key = frame['source_stamp_ns'], frame['group_id']
        selection, classifier = selections.get(key), classifiers.get(key)
        identity = identity_for(frame, selection, classifier)
        frame['identity_qualification'] = identity
        counts['identity:'+str(identity['value'])+':'+identity['basis']] += 1
        decision = frame['decision']
        timing = decision.get('observation_timing') or {}
        frame['raw_label'] = lamp_label(frame['normalized_tuples'])
        frame['observed_raw_label'] = lamp_label(frame['observed_tuples'])
        frame['original_output'] = dict(label=lamp_label(decision.get('final_tuples')),
            lamps=decision.get('final_tuples'), confidence=decision.get('final_confidence'),
            adopted_observation_source_stamp_ns=timing.get('adopted_observation_source_stamp_ns'),
            wire_source_stamp_ns=timing.get('output_source_stamp_ns'),
            output_is_held=timing.get('output_is_held'), output_origin=timing.get('output_origin'),
            transition_reason=decision.get('transition_reason'))
        frame['selection_evidence'] = dict(status=selection.get('status'), reason=selection.get('reason'),
            head_id=selection.get('head_id'), projection_source_stamp_ns=selection.get('projection_source_stamp_ns')) if selection else None
        frame['classifier_evidence'] = dict(reason=classifier.get('reason'), known=classifier.get('known'),
            crop_xyxy=classifier.get('crop_xyxy'), detector_bbox_xyxy=classifier.get('detector_bbox_xyxy')) if classifier else None
        if (previous is None or frame['group_id'] != previous['group_id']
                or frame['source_stamp_ns']-previous['source_stamp_ns'] > 1000000000
                or frame['source_stamp_ns'] < previous['source_stamp_ns']):
            segment += 1
        frame['segment'] = segment
        previous = frame
        error = ('qualification_unprovable' if identity['value'] is None else
                 'input_not_logged_observed_only' if frame['planner_compliance_observations'] is not True else
                 'original_guard_clock_missing' if frame['guard_clock_ns'] is None else
                 'original_normalized_tuples_missing' if not isinstance(frame['normalized_tuples'], list) else
                 'original_normalized_policy_reason_missing' if not isinstance(frame['policy_reason'], str) else
                 'original_raw_confidence_missing_or_nonfinite' if not finite(frame['raw_observation_confidence']) else None)
        if error:
            counts['unreplayable_frames:'+error] += 1
        frame['replays'] = {}
        for name, vote in votes.items():
            if error:
                vote.reset()
                frame['replays'][name] = dict(available=False, reason=error,
                    conservative_replay_history_reset_not_observed_identity_loss=True)
                continue
            result = vote.update(frame['normalized_tuples'], normalized_reason=frame['policy_reason'],
                group_id=frame['group_id'], head_id=frame['head_id'], source_stamp_ns=frame['source_stamp_ns'],
                now_ns=frame['guard_clock_ns'], confidence=frame['raw_observation_confidence'],
                identity_valid=identity['value'])
            frame['replays'][name] = output_record(result, frame['source_stamp_ns'])
        frame.pop('decision')
    return dict(counts)


def label_for(frame, stream):
    if stream == 'raw':
        return frame['raw_label']
    if stream == 'original_output':
        return frame['original_output']['label']
    result = frame['replays'][stream]
    return result['label'] if result['available'] else 'UNAVAILABLE'


def case_activity(frames, name):
    counts, resets, reasons = Counter(), Counter(), Counter()
    for frame in frames:
        result = frame['replays'][name]
        if not result['available']:
            counts['unavailable_evidence_and_conservative_history_reset'] += 1
            continue
        counts['available_frames'] += 1
        reasons[result['reason']] += 1
        if result['window_full']:
            counts['window_full_frames'] += 1
        elif frame['identity_qualification']['value'] is True:
            counts['qualified_underfull_window_frames'] += 1
            if result['startup_complete']:
                counts['post_startup_underfull_window_frames'] += 1
        if result['startup_complete']:
            counts['startup_complete_frames'] += 1
        if result['startup_passthrough_active']:
            counts['startup_passthrough_frames'] += 1
            counts['startup_passthrough_UNKNOWN_frames' if result['label']=='UNKNOWN'
                   else 'startup_passthrough_known_phase_frames'] += 1
        if result['reason'] == 'stamped_vote:unconfirmed':
            counts['unconfirmed_full_window_frames' if result['window_full'] else 'unconfirmed_underfull_window_frames'] += 1
        if (not result['startup_passthrough_active'] and result['reason'] in
                ('stamped_vote:winner','stamped_vote:unknown_majority','stamped_vote:unconfirmed')):
            counts['majority_evaluation_frames'] += 1
            if not result['window_full']:
                counts['majority_evaluation_underfull_window_frames'] += 1
        if result['reset_reason']:
            resets[result['reset_reason']] += 1
    return dict(counts=dict(counts), reason_counts=dict(reasons), reset_reason_counts=dict(resets),
        source_reset_count=sum(resets[reason] for reason in ('source_reversal','invalid_future_or_stale_source')),
        clock_reset_count=sum(resets[reason] for reason in ('clock_reversal','invalid_clock')),
        identity_loss_reset_count=resets['identity_loss'], group_change_reset_count=resets['group_change'])


def sequences(frames, stream):
    sequence = []
    for frame in frames:
        label = label_for(frame, stream)
        state = permission(label, frame['group_id']) if label not in ('UNAVAILABLE','MISSING') else 'UNAVAILABLE'
        if not sequence or sequence[-1]['label'] != label or sequence[-1]['segment'] != frame['segment']:
            sequence.append(dict(label=label, route_permission_proxy=state, segment=frame['segment'],
                first_source_stamp_ns=frame['source_stamp_ns'], first_record_id=frame['record_id'],
                first_video=frame.get('front_video'), frames=0))
        sequence[-1]['last_source_stamp_ns'] = frame['source_stamp_ns']
        sequence[-1]['frames'] += 1
    for first, following in zip(sequence, sequence[1:]):
        if first['segment'] == following['segment']:
            first['duration_until_next_source_sec'] = (following['first_source_stamp_ns']-first['first_source_stamp_ns'])/1e9
    return sequence


def short_islands(sequence, frame_index, native_samples, maximum=1.):
    # Merge lamp-label changes that leave route permission unchanged.
    runs = []
    for row in sequence:
        if runs and runs[-1]['route_permission_proxy'] == row['route_permission_proxy'] and runs[-1]['segment'] == row['segment']:
            runs[-1]['frames'] += row['frames']
            runs[-1]['last_source_stamp_ns'] = row['last_source_stamp_ns']
        else:
            runs.append(dict(row))
    results = []
    for first, middle, last in zip(runs, runs[1:], runs[2:]):
        if len({r['segment'] for r in (first, middle, last)}) != 1:
            continue
        states = tuple(r['route_permission_proxy'] for r in (first, middle, last))
        if states not in (('GO','UNKNOWN','GO'), ('GO','STOP','GO'), ('STOP','GO','STOP')):
            continue
        duration = (last['first_source_stamp_ns']-middle['first_source_stamp_ns'])/1e9
        if not 0 <= duration <= maximum:
            continue
        frame = frame_index[middle['first_source_stamp_ns']]
        results.append(dict(kind='_'.join(states), first_source_stamp_ns=middle['first_source_stamp_ns'],
            recovery_source_stamp_ns=last['first_source_stamp_ns'], duration_sec=duration,
            frames=middle['frames'], front_video=middle['first_video'],
            native_conditional_refs_at_start=native_refs(frame, native_samples)))
    return results


def transition_lags(frames, native_samples):
    events, previous_known, previous_segment = [], None, None
    for index, frame in enumerate(frames):
        if frame['segment'] != previous_segment:
            previous_known = None
            previous_segment = frame['segment']
        label = frame['raw_label']
        if (label in ('UNKNOWN', 'MISSING') or not isinstance(frame['policy_reason'], str)
                or not frame['policy_reason'].endswith(':observed')):
            continue
        if previous_known == label:
            continue
        prior, previous_known = previous_known, label
        end = next((i for i in range(index+1, len(frames))
                    if frames[i]['segment'] != frame['segment'] or
                    frames[i]['raw_label'] not in ('UNKNOWN', 'MISSING', label)), len(frames))
        own = frames[index:end]
        results = {}
        for stream in ('original_output', *CASES):
            first = next((candidate for candidate in own if label_for(candidate, stream) == label), None)
            if first is None:
                results[stream] = dict(emitted_before_next_distinct_known_input=False,
                    unavailable_frames=sum(label_for(candidate, stream) == 'UNAVAILABLE' for candidate in own))
                continue
            out = first['original_output'] if stream == 'original_output' else first['replays'][stream]
            adopted = out.get('adopted_observation_source_stamp_ns')
            results[stream] = dict(emitted_before_next_distinct_known_input=True,
                first_output_current_source_stamp_ns=first['source_stamp_ns'],
                winner_original_source_stamp_ns=adopted,
                first_input_to_output_frame_source_sec=(first['source_stamp_ns']-frame['source_stamp_ns'])/1e9,
                first_input_to_winner_original_source_sec=(adopted-frame['source_stamp_ns'])/1e9 if adopted is not None else None,
                first_input_to_output_guard_sec=(first['guard_clock_ns']-frame['source_stamp_ns'])/1e9 if first['guard_clock_ns'] else None,
                first_input_to_original_observer_receipt_of_output_frame_sec=(first['receipt_ros_ns']-frame['source_stamp_ns'])/1e9
                    if type(first['receipt_ros_ns']) is int else None,
                replay_receipt_is_original_callback_clock_proxy_not_new_measured_runtime=True,
                output_video=first.get('front_video'), startup_passthrough_active=out.get('startup_passthrough_active'))
        events.append(dict(previous_known_raw_phase=prior, new_normalized_raw_phase=label,
            first_source_stamp_ns=frame['source_stamp_ns'], front_video=frame.get('front_video'),
            yellow_onset_caution=(dict(first_raw_Y_is_not_independent_physical_onset=True,
                latest_winner_source_is_not_first_Y_onset=True, timed_yellow_ON_compatibility_verified=False)
                if 'Y' in label.split('+') else None),
            native_conditional_refs_at_input=native_refs(frame, native_samples), streams=results))
    return events


def summarize(frames, native_samples):
    missions = []
    for group, mission in MISSIONS.items():
        own = [frame for frame in frames if frame['group_id'] == group]
        index = {frame['source_stamp_ns']: frame for frame in own}
        streams = {}
        for stream in ('raw', 'original_output', *CASES):
            sequence = sequences(own, stream)
            streams[stream] = dict(label_counts=dict(Counter(label_for(frame, stream) for frame in own)),
                sequence=sequence, short_islands=short_islands(sequence, index, native_samples))
            if stream in CASES:
                streams[stream]['mode_activity'] = case_activity(own, stream)
        interior = []
        for segment in sorted({frame['segment'] for frame in own}):
            part = [frame for frame in own if frame['segment'] == segment]
            known = [i for i, frame in enumerate(part) if frame['raw_label'] not in ('UNKNOWN','MISSING')]
            if known:
                interior.extend(part[known[0]:known[-1]+1])
        missions.append(dict(mission=mission, group_id=group, frames=len(own),
            interior_raw_unknown_frames=sum(frame['raw_label']=='UNKNOWN' for frame in interior),
            interior_raw_denominator_frames=len(interior),
            denominator='inclusive_first_to_last_nonempty_normalized_raw_phase_per_contiguous_mission_segment_initial_unseen_and_trailing_no_recovery_excluded_not_proven_FOV',
            identity_counts=dict(Counter(str(frame['identity_qualification']['value']) for frame in own)),
            streams=streams, new_raw_phase_transition_lags=transition_lags(own, native_samples)))
    return missions


def analyze(directory):
    directory = Path(directory).resolve(strict=True)
    evidence, warnings = [], []
    if not (directory/'selection.json').is_file() or not (directory/'observer/observations.jsonl').is_file():
        raise ValueError('Input must be a wrapper with selection.json and observer/observations.jsonl: '+str(directory))
    wrapper = list(jsonl(directory/'wrapper_events.jsonl', evidence, warnings))
    if not any(row.get('event') == 'validator_returned' for row in wrapper):
        raise ValueError('Completed wrapper required (validator_returned missing): '+str(directory))
    selection_receipt = read_json(directory/'selection.json', evidence)
    frames, selected, classified, native, seen, counts = [], {}, {}, [], {}, Counter()
    for row in jsonl(directory/'observer/observations.jsonl', evidence, warnings):
        if row.get('event') == 'native_read_only_sample':
            native.append({key: row.get(key) for key in ('record_id','native_id','valid','color',
                'request_monotonic','response_monotonic','request_ros_ns','response_ros_ns')})
            continue
        topic = row.get('topic', '')
        if not any(topic.endswith('/'+name) for name in ('debug_runtime','debug_selection','debug_classifier')):
            continue
        try:
            body = json.loads(row.get('message', {}).get('data', ''))
            if not isinstance(body, dict):
                raise ValueError('Debug payload must be an object')
        except (TypeError, ValueError):
            counts['invalid_debug_json'] += 1
            continue
        if topic.endswith('/debug_runtime'):
            frame = make_frame(body, row)
            if frame is None:
                counts['runtime_without_usable_source_or_decision'] += 1
                continue
            key = frame['source_stamp_ns'], frame['group_id']
            if key in seen:
                counts['duplicate_source_group_runtime'] += 1
                if seen[key] != frame['decision']:
                    counts['conflicting_duplicate_runtime'] += 1
                continue
            seen[key] = frame['decision']
            frames.append(frame)
        else:
            key = body.get('source_stamp_ns'), body.get('group_id')
            target = selected if topic.endswith('/debug_selection') else classified
            if key in target and target[key] != body:
                counts['conflicting_duplicate_auxiliary'] += 1
            target.setdefault(key, body)
    if counts['conflicting_duplicate_runtime'] or counts['conflicting_duplicate_auxiliary']:
        raise ValueError('Conflicting exact-source debug evidence; replay refuses to choose a winner')
    videos = {}
    for row in jsonl(directory/'front_video/frames.jsonl', evidence, warnings):
        source = (row.get('image') or {}).get('source_stamp_ns')
        if type(source) is int and source > 0:
            videos.setdefault(source, dict(frame_index=row.get('frame_index'), pts_sec=row.get('pts_sec'),
                exact_image_source_join=True, runtime_exact_image_stamp_match=row.get('runtime_exact_image_stamp_match')))
    for frame in frames:
        frame['front_video'] = videos.get(frame['source_stamp_ns'])
    counts.update(replay_frames(frames, selected, classified))
    if any(frame['identity_qualification']['value'] is None for frame in frames):
        warnings.append('Unprovable qualification is UNAVAILABLE and resets replay history conservatively; it is not an UNKNOWN vote or evidence of actual identity loss.')
    if any(frame['identity_qualification']['basis'].startswith('reconstructed_') for frame in frames):
        warnings.append('Legacy direct-run identity reconstructed from exact-source current eligible map ROI + detector/classifier crop; the final vote-instant route/packet recheck was not independently logged.')
    if any(frame['guard_clock_ns'] is None for frame in frames):
        warnings.append('Missing original guard clocks: affected replay frames unavailable.')
    if any(frame['planner_compliance_observations'] is not True for frame in frames):
        warnings.append('Observed-only normalization not explicitly logged on some frames: those replay inputs are unavailable; synthetic/legacy corrected phases are never voted.')
    return dict(wrapper=str(directory), selection=selection_receipt, evidence=evidence,
        selected_launch_commands=[row for row in wrapper if row.get('event') == 'launch'
            and row.get('package') in ('traffic_light_detection','ajounice_planning_integration')],
        counts=dict(counts), unique_source_group_frames=len(frames), native_read_only_samples=len(native),
        replay_case_activity={name:case_activity(frames,name) for name in CASES},
        crossings=read_json(directory/'observer/crossings.json', evidence),
        missions=summarize(frames, native), frames=frames, warnings=warnings)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('wrappers', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists():
        parser.error('Fresh output required; source and prior evidence are never overwritten')
    helper = ROOT/'base_ws/src/perception/traffic_light_detection/scripts/traffic_light_stamped_vote.py'
    probe = StampedPhaseVote().update([], normalized_reason='metadata:observation_missing', group_id=0,
        head_id=None, source_stamp_ns=0, now_ns=0, confidence=0., identity_valid=False)
    report = dict(schema='traffic_light_stamped_vote_same_input_replay_v2', read_only=True,
        created_UTC=datetime.now(timezone.utc).isoformat(), cases=CASES,
        helper=dict(path=str(helper), sha256=hashlib.sha256(helper.read_bytes()).hexdigest(),
                    candidate_version=probe['debug'].get('candidate_version')),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=[
            'Same recorded perception inputs only: not a closed-loop driving counterfactual, classifier accuracy, causal treatment effect or official safety certification.',
            'Original production/stamped-vote outputs are comparators only; replay votes use current observed-only normalized tuples and original raw confidence, never held final tuples.',
            '3/2 strict has max original age0.5s. Helperv4 7/4 age0.9s passes through only its first6 accepted DISTINCT inputs per qualified-group epoch, including UNKNOWN. accepted_count caps7; age pruning or source errors do not reopen cold startup. Full group/identity/clock reset starts a new epoch.',
            'window_full describes retained queue length, not startup completion. After7 accepted inputs, an underfull age-pruned queue still needs4 agreeing votes; post-startup underfull majority evaluation is counted separately from cold passthrough.',
            'Three matched UNKNOWN frames are bridgeable only with sufficient prior agreeing votes; hard identity loss resets all history and a long window cannot recover missing qualification.',
            'Reset counts count callbacks, including repeated initial-unseen/nonmission/post-FOV identity-loss callbacks, not distinct incidents or error rates. Debug group_change also marks requalification after cleared state, not necessarily a physical route-group change.',
            'Missing current-crop qualification remains unavailable. Reconstructed legacy qualifications are explicitly distinguished from logged vote identity; no retained housing or nearest-time join is accepted.',
            'Original guard clock was logged as epoch float seconds; round(seconds*1e9) reconstructs it with sub-microsecond quantization, not exact integer-clock recovery.',
            'Native references retain request/response record IDs and strict0.25/0.5s brackets; ROS-to-monotonic source mapping is conditional and native state is not rendered-pixel GT.',
            'R/Y are delayed or suppressed just like GO once majority voting applies; new raw phase means classifier input, not independently established physical lamp onset.',
            'Timed-yellow ON compatibility is NOT established: a winning Y adopts the latest actual agreeing frame (e.g. fourth Y), not first raw Y onset. Treating that later stamp as phase onset could extend assumed remaining yellow. Recorded tests keep yellow overrides OFF; this replay does not authorize enabling them unchanged.',
            'Source-based lags and short islands use recorded callback/image samples; initial unseen/trailing no-recovery exclusions do not prove visibility/FOV.',
            'No ROS graph, native RPC, model/GPU/image decoding, bag rewriting, install, control or live-process interaction.'],
        runs=[analyze(path) for path in args.wrappers])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(output), runs=[dict(wrapper=run['wrapper'], frames=run['unique_source_group_frames'],
        warnings=run['warnings'], missions=[dict(mission=m['mission'],
            sequence={name:' > '.join(s['label'] for s in value['sequence']) for name,value in m['streams'].items()},
            short_islands={name:len(value['short_islands']) for name,value in m['streams'].items()}) for m in run['missions']])
        for run in report['runs']]), ensure_ascii=False))


if __name__ == '__main__':
    main()
