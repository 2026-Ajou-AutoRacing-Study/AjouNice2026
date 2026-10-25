#!/usr/bin/env python3
"""Offline derived summary; never controls ROS, traffic lamps or a vehicle.

Native GET reports API state, not visible pixel truth. Duration bounds below
describe explicit SET-to-readback transitions, not continuous render timing.
No violation/safety PASS is inferred from a completed wrapper alone.
"""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transition_bounds(events, native_id, yellow):
    def first(event, stage, color_key=None, color=None):
        return next((r for r in events if r.get('event') == event and r.get('stage') == stage
                     and r.get('native_id') == native_id and
                     (color_key is None or r.get(color_key) == color) and
                     (event != 'native_signal_verified_actual' or r.get('verified'))), None)
    ys = first('native_signal_requested', 'moving_yellow', 'requested_state', yellow)
    yg = first('native_signal_verified_actual', 'moving_yellow', 'actual_state', yellow)
    rs = first('native_signal_requested', 'red_after_moving_yellow', 'requested_state', 1)
    rg = first('native_signal_verified_actual', 'red_after_moving_yellow', 'actual_state', 1)
    result = dict(native_id=native_id, all_transition_records_present=all((ys, yg, rs, rg)),
                  yellow_first_SET_monotonic=ys.get('request_monotonic') if ys else None,
                  yellow_first_GET_response_monotonic=yg.get('response_monotonic') if yg else None,
                  red_first_SET_monotonic=rs.get('request_monotonic') if rs else None,
                  red_first_GET_response_monotonic=rg.get('response_monotonic') if rg else None,
                  continuous_pixel_state_certified=False)
    if result['all_transition_records_present']:
        result['explicit_transition_duration_bounds_sec'] = [
            max(0., rs['request_monotonic']-yg['response_monotonic']),
            rg['response_monotonic']-ys['request_monotonic']]
    return result


def analyze(folder):
    selection_path = folder/'selection.json'
    selection = json.loads(selection_path.read_text())
    if not selection.get('real_native_moving_yellow') or selection.get('synthetic_canonical_signal_proxy'):
        raise ValueError('Require explicitly recorded real native moving yellow trial without a canonical proxy')
    metadata_path = folder/'phase_controller/metadata.json'
    metadata = json.loads(metadata_path.read_text())
    event_path = folder/'phase_controller/events.jsonl'
    events = [json.loads(line) for line in event_path.open() if line.strip()]
    group = metadata['group'];gid = group['group_id'];yellow = 5 if gid == 201326 else 4
    trigger = next((r for r in events if r.get('event') == 'moving_native_yellow_triggered'), None)
    completed_stages = [r['stage']['stage'] for r in events if r.get('event') == 'stage_complete']
    qualified = trigger is not None and trigger.get('distinct_authorized_observations', 0) >= 3
    # Early moving wrappers left their stopped-only receipt field null. The
    # controller's immutable metadata still records the actual requested list.
    duration = selection.get('yellow_durations_sec') or metadata.get('yellow_durations_sec')
    result = dict(development_only=True, group_id=gid, configured_duration_sec=duration,
                  configured_trigger_distance_m=selection.get('trigger_front_distance_m'),
                  real_native_vision_trial=True, native_GT_in_planner=False,
                  moving_gate_qualified=qualified, trigger=trigger,
                  completed_stages=completed_stages,
                  full_yellow_and_red_stage_complete=all(s in completed_stages for s in ('moving_yellow', 'red_after_moving_yellow')),
                  per_head=[transition_bounds(events, native_id, yellow) for native_id in group['native_ids']],
                  official_penalty_certified=False, runtime_safety_PASS_inferred=False,
                  natural_cycle_restored=False,
                  limitations=['Native GET is not independent visible-pixel truth.',
                      'Bounds refer to explicit SET-to-GET transitions; natural/intermediate state is not continuously certified.',
                      'Source timestamps and receipt times are separate; no fresh timestamp manufactured.',
                      'A completed goal can truncate stress stages; missing phase completion is not a full-duration test.'])
    crosses_path = folder/'observer/crossings.json'
    if crosses_path.exists():
        result['crossings'] = [c for c in json.loads(crosses_path.read_text()).get('crossings', []) if c['group_id'] == gid]
    observations_path = folder/'observer/observations.jsonl'
    yellow_frames = []
    if trigger and observations_path.exists():
        for line in observations_path.open():
            r = json.loads(line)
            if r.get('topic') != '/traffic_light_node/debug_runtime' or r['receipt_monotonic'] < trigger['monotonic']:
                continue
            d = r['message'].get('parsed_json') or json.loads(r['message']['data'])
            f = d.get('frame_decision') or {}
            if (f.get('group_id') == gid and f.get('selection_status') == 'matched' and
                    f.get('classifier_known') and any(v[0] == 'yellow' for v in f.get('observed_tuples', []))):
                yellow_frames.append(dict(record_id=r['record_id'], source_stamp_ns=d.get('source_stamp_ns'),
                                          receipt_monotonic=r['receipt_monotonic'], raw=f['observed_tuples'],
                                          output=f.get('final_tuples'), timing=f.get('observation_timing')))
        result['raw_CNN_yellow_frames'] = dict(count=len(yellow_frames),
                     first=yellow_frames[0] if yellow_frames else None, last=yellow_frames[-1] if yellow_frames else None,
                     independent_pixel_GT=False)
    sources = [selection_path, metadata_path, event_path]
    sources += [p for p in (crosses_path, observations_path) if p.exists()]
    result['source_sha256_at_analysis'] = {str(p): digest(p) for p in sources}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, help='Optional fresh derived JSON; defaults to stdout only')
    args = parser.parse_args();result = analyze(args.trial_dir)
    text = json.dumps(result, ensure_ascii=False, indent=2)+'\n'
    if args.output:
        if args.output.exists():
            raise FileExistsError('Never overwrite previous derived evidence')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
