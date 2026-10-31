#!/usr/bin/env python3
"""Explicit sequential dev trials; each child owns Validator safety/graph lock.

No direct vehicle publisher. Stop on the first failed child. Ctrl-C/SIGTERM is
forwarded to the owner for normal stop/P teardown, never SIGKILL.
"""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'artifacts/evaluation/traffic_light_robustness_261025'
CASES = [
    ('B_off2', 'B', False, '2', 'moving-native-yellow'),
    ('C_on2', 'C', True, '2', 'moving-native-yellow'),
    ('C_off2', 'C', False, '2', 'moving-native-yellow'),
    ('D_on2', 'D', True, '2', 'moving-native-yellow'),
    ('D_off2', 'D', False, '2', 'moving-native-yellow'),
    ('B_on1_stress', 'B', True, '1', 'moving-native-yellow'),
    ('B_on3', 'B', True, '3', 'moving-native-yellow'),
    ('D_stopped_yellow', 'D', True, '2,3', 'stopped-yellow'),
]
GROUPS = dict(A=201351, B=201365, C=201290, D=201326)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--configs', type=Path, default=BASE/'timed_yellow_campaigns_261026_v2')
    parser.add_argument('--cases', help='Explicit comma-separated subset; fresh output only, never resumes old runs')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    cases = CASES
    if args.cases:
        labels = args.cases.split(',')
        known = {case[0]: case for case in CASES}
        if len(set(labels)) != len(labels) or any(label not in known for label in labels):
            parser.error('Unknown or duplicate case label')
        cases = [known[label] for label in labels]
    plan = []
    for label, mission, on, durations, experiment in cases:
        cfg = args.configs/(mission+'_focused_'+('timed' if on else 'off')+'.yaml')
        cmd = [sys.executable, str(ROOT/'scripts/evaluation/run_traffic_light_compliance_trial.py'), str(cfg),
               '--output', str(args.output/label), '--geometry', str(BASE/'production_trials_abcd_261026_v4/groups.json'),
               '--mode', 'guarded', '--experiment', experiment, '--yellow-durations-sec', durations,
               '--group-id', str(GROUPS[mission]), '--temporal-direct', 'true', '--signal-timeout-sec', '.5',
               '--yellow-passthrough', str(on).lower(), '--compliance-debug']
        if experiment == 'moving-native-yellow':
            cmd.extend(['--trigger-front-distance-m', '8'])
        plan.append(dict(label=label, command=cmd, short_yellow_violates_candidate_assumption=durations == '1'))
    if not args.run:
        print(json.dumps(plan, indent=2));return 0
    args.output.resolve().relative_to(ROOT/'artifacts/evaluation')
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    active = [None]
    def interrupted(signum, _frame):
        if active[0] is not None and active[0].poll() is None:
            active[0].send_signal(signum)
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):signal.signal(sig, interrupted)
    try:
        for row in plan:
            print('START '+row['label'], flush=True)
            with (args.output/(row['label']+'.log')).open('x') as stream:
                active[0] = subprocess.Popen(row['command'], cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
                code = active[0].wait()
            with (args.output/'results.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(label=row['label'], returncode=code))+'\n')
            print('END '+row['label']+' returncode='+str(code), flush=True)
            if code:return code
    except KeyboardInterrupt:
        if active[0] is not None:
            active[0].wait()  # Owner completes verified stop/P; no forced kill.
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
