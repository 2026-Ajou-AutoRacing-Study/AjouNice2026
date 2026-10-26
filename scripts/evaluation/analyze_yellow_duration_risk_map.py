#!/usr/bin/env python3
"""Counterfactual yellow-duration geometry, never a driving/violation replay.

Recorded front-axle motion provides local speed and approach positions. Freeze
that speed for crossing, assume zero initial acceleration for stopping distance,
and test ONLY the physical-distance gate, not the full yellow state machine.
The resulting bands are conditional screening candidates, not observed offences,
not FOV-exit boundaries, and not statistical frequencies.
"""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

import numpy as np

from analyze_signal_last_observation_gap import BASE, RUNS, load_run, map_roads, sha


def stop_distance(v):
    """Current b=6, J=10, delay=.5 model, with explicit a0=0 assumption."""
    if v <= 0:
        return 0.
    if v > 1.8:
        return v*v/12.+.8*v-.09
    t = math.sqrt(2*v/10.)
    return .5*v+v*t-10*t**3/6.


def risk_gate(distance, velocity, duration, onset_to_decision_delay=0.):
    return (velocity >= 1. and distance > 0.
            and distance <= stop_distance(velocity)
            and distance/velocity >= duration-onset_to_decision_delay)


def recorded_speeds(run):
    """Use filtered body-forward odometry, not receipt-jittered pose differences."""
    speeds = []
    for line in (BASE/run/'observer/observations.jsonl').open():
        r = json.loads(line)
        if r.get('topic') != '/localization/kinematic_state':
            continue
        m = r['message']; stamp = m['header']['stamp']
        t = stamp['secs']*1000000000+stamp['nsecs']
        v = float(m['twist']['twist']['linear']['x'])
        if math.isfinite(v):
            speeds.append((t, v))
    return sorted(speeds)


def extract(rows, poses, speeds):
    speed_stamps = [p[0] for p in speeds]
    result = []
    for row in rows:
        tc = row['crossing_source_stamp_ns']
        cross = np.array(row['crossing_xy_m'])
        points = []
        for p in poses:
            t = p['source_stamp_ns']
            if not tc-90e9 < t < tc:
                continue
            xy = np.array(p['front_point_map_xyz_m'][:2])
            d = float(np.linalg.norm(xy-cross))
            if not .2 < d < 60:
                continue
            vi = bisect_right(speed_stamps, t)-1
            if vi < 0 or t-speed_stamps[vi] > 150000000:
                continue
            v = speeds[vi][1]
            if v < 1.:
                continue
            points.append(dict(xy=xy.tolist(), d_m=d, v_mps=v, source_stamp_ns=t,
                               stopping_model_a0_zero_m=stop_distance(v),
                               frozen_speed_arrival_sec=d/v,
                               speed_source_age_sec=(t-speed_stamps[vi])/1e9))
        stats = {}
        for duration in (1, 2, 3):
            for latency in (0., .2):
                selected = [p for p in points if risk_gate(p['d_m'], p['v_mps'], duration, latency)]
                stats[f'{duration}s_delay{latency:.1f}s'] = dict(
                    sampled_point_count=len(selected),
                    distance_range_m=[min(p['d_m'] for p in selected), max(p['d_m'] for p in selected)] if selected else None)
        result.append(dict(mission=row['mission'], run=row['run'], points=points, stats=stats))
    return result


def draw(results, groups, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    plt.rcParams['font.family'] = 'Noto Sans CJK JP'
    plt.rcParams['axes.unicode_minus'] = False
    roads, _ = map_roads(groups)
    fig, axes = plt.subplots(2, 2, figsize=(15, 14))
    for ax, group in zip(axes.flat, groups.values()):
        ax.add_collection(LineCollection(roads, colors='#c4cbd1', linewidths=.8))
        line = np.array(group['stop_line_xy']); center = line.mean(axis=0)
        ax.plot(*line.T, color='black', lw=5, label='정지선', zorder=7)
        selected = [r for r in results if r['mission'] == group['mission']]
        for n, r in enumerate(selected):
            p = r['points']
            xy = np.array([q['xy'] for q in p])
            ax.scatter(*xy.T, s=3, color='#64748b', alpha=.3, label='관측된 접근 위치' if n == 0 else None)
        for duration, color, size in ((1, '#e14646', 26), (2, '#eea31a', 12), (3, '#7c3ab7', 6)):
            p = [p for r in selected for p in r['points'] if risk_gate(p['d_m'], p['v_mps'], duration)]
            if p:
                xy = np.array([q['xy'] for q in p])
                ax.scatter(*xy.T, color=color, s=size, zorder=3+duration,
                           label=f'황색 {duration}초: 적색 도달 후보')
        texts = []
        for duration in (1, 2, 3):
            intervals = [r['stats'][f'{duration}s_delay0.0s']['distance_range_m'] for r in selected]
            intervals = [i for i in intervals if i]
            texts.append(f'{duration}초: '+(f"선 앞 {min(i[0] for i in intervals):.1f}~{max(i[1] for i in intervals):.1f}m" if intervals else '해당 후보 없음'))
        ax.text(.025, .975, '\n'.join(texts), transform=ax.transAxes, va='top', fontsize=12,
                bbox=dict(facecolor='white', alpha=.95, edgecolor='none'), zorder=10)
        ax.set_title(group['mission']+(' · 좌회전' if group['mission'] == 'D' else ' · 직진'), fontsize=18)
        # D enters from the west; A/B/C approach from the south.
        if group['mission'] == 'D':
            ax.set_xlim(center[0]-45, center[0]+10);ax.set_ylim(center[1]-20, center[1]+20)
        else:
            ax.set_xlim(center[0]-25, center[0]+20);ax.set_ylim(center[1]-50, center[1]+10)
        ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
        ax.legend(loc='lower right', fontsize=9)
    fig.suptitle('황색 1·2·3초 · 정지거리 조건과 적색 도달 조건의 겹침', fontsize=20)
    fig.text(.5, .035, '조건부 계산: 기록 위치·속도, 이후 등속 통과, 초기 가속도 0, 황색 시작 즉시 인지\n'
             '전체 GO 조건을 재현한 결과 아님 · 실제 위반/카메라 사각지대 지도 아님\n'
             '검출·판단 지연 0.2초 가정의 민감도는 별도 JSON에 보존', ha='center', fontsize=12)
    fig.tight_layout(rect=(0, .09, 1, .95));fig.savefig(output/'yellow_duration_conditional_risk_map.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();args.output.mkdir(parents=True, exist_ok=False)
    group_path = BASE/'production_trials_abcd_261026_v4/groups.json'
    groups = {g['group_id']: g for g in json.loads(group_path.read_text())['groups']}
    results, inputs = [], {}
    for run in RUNS:
        rows, poses, inputs[run] = load_run(run, groups)
        results.extend(extract(rows, poses, recorded_speeds(run)))
    draw(results, groups, args.output)
    document = dict(assumptions=dict(stopping_acceleration=-6., stopping_jerk=-10., stopping_delay=.5,
                    initial_acceleration=0., future_speed='frozen at latest recorded localization/kinematic_state body-x velocity within 150ms',
                    range='Euclidean to recorded finite-segment front-axle crossing point; not planner path-arc replay',
                    eligibility='distance gate only; does not simulate previous-stop, three greens, freshness, actual GO',
                    current_yellow_passthrough_default=False,
                    physical_FOV_blind_zone_measured=False,
                    official_penalty_or_safety_guarantee=False,
                    counts='correlated sampled positions, never violation frequency'),
                    inputs=inputs, groups_sha256=sha(group_path), results=results)
    (args.output/'conditional_measurements.json').write_text(json.dumps(document, ensure_ascii=False, indent=2)+'\n')
    for r in results:
        print(r['mission'], 'sunny' if 'sunny' in r['run'] else 'foggy',
              {k: v['distance_range_m'] for k, v in r['stats'].items()})


if __name__ == '__main__':
    main()
