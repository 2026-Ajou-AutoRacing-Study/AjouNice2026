#!/usr/bin/env python3
"""Read-only measured braking audit from source-stamped video evidence.

Distances from velocity integration are observations, not maximum braking
capability certification. An early STOP can deliberately use gentler braking.
"""
import argparse
import json
import math
from pathlib import Path


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values)-1, round((len(values)-1)*fraction))] if values else None


def audit(frames, group):
    samples = []
    for frame in frames:
        telemetry = frame['telemetry']
        planner = telemetry.get('planner_decision') or {}
        value = planner.get('value') or {}
        ego = telemetry.get('ego') or {}
        speed = (ego.get('value') or {}).get('speed_kph')
        if value.get('group_id') != group or speed is None:
            continue
        if any(not isinstance(row.get('receipt_age_sec'), (int, float)) or
               not 0 <= row['receipt_age_sec'] <= .5 for row in (planner, ego)):
            continue
        runtime = (telemetry.get('runtime', {}).get('value') or {}).get('frame_decision') or {}
        samples.append(dict(t=frame['write_ros_ns']/1e9, speed=speed/3.6,
            decision=value, lamps=runtime.get('observed_tuples'),
            brake=(telemetry.get('command', {}).get('value') or {}).get('brake'),
            image_age=frame['image'].get('source_age_sec'),
            planner_age=planner.get('source_age_sec')))
    episodes, previous = [], None
    for index, sample in enumerate(samples):
        if (previous and not previous['decision']['require_stop'] and
            sample['decision']['require_stop'] and sample['speed'] > 1.):
            end = next((j for j in range(index+1, len(samples)-5) if
                all(s['speed'] < .1 for s in samples[j:j+6]) and
                samples[j+5]['t']-samples[j]['t'] >= .4), None)
            if end is not None:
                path = samples[index:end+1]
                gaps = [b['t']-a['t'] for a, b in zip(path, path[1:])]
                distance = sum((a['speed']+b['speed'])/2*(b['t']-a['t'])
                               for a, b in zip(path, path[1:]))
                decels = [(path[j]['speed']-a['speed'])/(path[j]['t']-a['t'])
                    for k, a in enumerate(path) for j in [next((n for n in range(k+1,len(path))
                        if path[n]['t']-a['t'] >= .3), k)] if j > k]
                response = next((s['t']-sample['t'] for s in path
                    if isinstance(s['brake'], (int,float)) and s['brake'] > .01), None)
                episodes.append(dict(start_ros_sec=sample['t'], speed_mps=sample['speed'],
                    observation=sample['lamps'], calculated_brake_m=sample['decision'].get('brake_m'),
                    target_m_at_stop_request=sample['decision'].get('target_m'),
                    measured_distance_velocity_integral_m=distance,
                    stopped_target_m=samples[end]['decision'].get('target_m'),
                    elapsed_to_stop_sec=samples[end]['t']-sample['t'],
                    first_brake_command_delay_sec=response, max_sampling_gap_sec=max(gaps, default=None),
                    peak_deceleration_300ms_mps2=min(decels,default=None),
                    uninterrupted_stop=all(s['decision']['require_stop'] for s in path),
                    image_age_p95_sec=percentile([s['image_age'] for s in path
                        if isinstance(s['image_age'],(int,float))],.95)))
        previous = sample
    return dict(group_id=group, valid_samples=len(samples), stop_episodes=episodes,
        committed_go_samples=sum(bool(s['decision'].get('committed')) for s in samples),
        limitation='Normal planner/controller braking, not actuator-limit calibration. '
            '10Hz source-stamped video evidence; inspect sampling gaps and uninterrupted_stop.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--group-id', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.frames.open() as stream:
        result = audit([json.loads(line) for line in stream], args.group_id)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
