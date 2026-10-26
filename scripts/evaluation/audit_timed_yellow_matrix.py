#!/usr/bin/env python3
"""Read-only independent closed-wrapper audit of real native yellow trials.

Never touches live ROS/native controls. Printed/optional output is derived only.
Correlated frames and controlled dwell times are not natural-cycle statistics.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re

from summarize_moving_native_yellow_trial import analyze


def hash_file(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def finite_number(value):
    number = float(value)
    return number if math.isfinite(number) else None


def completed(folder):
    path = folder/'wrapper_events.jsonl'
    if not path.is_file():
        return False
    return any(json.loads(line).get('event') == 'validator_returned' for line in path.open() if line.strip())


def audit(folder):
    if not completed(folder):
        raise ValueError('Only closed wrapper runs may be audited')
    result = analyze(folder)
    selection = json.loads((folder/'selection.json').read_text())
    wrapper_events = [json.loads(line) for line in (folder/'wrapper_events.jsonl').open() if line.strip()]
    result['wrapper_returncode'] = next(r['returncode'] for r in reversed(wrapper_events) if r.get('event') == 'validator_returned')
    result['yellow_master'] = selection.get('enable_yellow_passthrough')
    result['installed_plugin_sha256'] = selection.get('installed_traffic_plugin_sha256')
    result['wrapper'] = str(folder)
    if not result['trigger']:
        return result
    trigger = result['trigger'];start = trigger['monotonic']; start_ros = trigger['ros_ns']/1e9
    crossings = result.get('crossings', [])
    cross = min(crossings, key=lambda c: c['estimated_source_monotonic']) if crossings else None
    end = cross['estimated_source_monotonic'] if cross else math.inf
    end_ros = cross['estimated_source_stamp_ns']/1e9 if cross else math.inf
    if cross:
        result['trigger_to_front_cross_sec'] = end-start
    group = json.loads((folder/'phase_controller/metadata.json').read_text())['group']
    a, b = group['stop_line_xy'][0], group['stop_line_xy'][-1]
    dx, dy = b[0]-a[0], b[1]-a[1];length = math.hypot(dx, dy)
    before_sign = -1 if cross and cross['direction'] == 'negative_to_positive' else 1
    raw_by_stamp, poses, commands = {}, [], []
    for line in (folder/'observer/observations.jsonl').open():
        r = json.loads(line)
        if r.get('topic') == '/Ego_topic':
            m = r['message'];st = m['header']['stamp'];ns = st['secs']*10**9+st['nsecs']
            raw_by_stamp[ns] = math.sqrt(sum(m['velocity'][k]**2 for k in 'xyz'))
        if start <= r.get('receipt_monotonic', -1) <= end:
            if r.get('event') == 'ego_front_GT_proxy':
                poses.append(r)
            elif r.get('topic') == '/ctrl_cmd':
                commands.append(r['message'])
    speeds, stationary = [], []
    for p in poses:
        v = raw_by_stamp.get(p['source_stamp_ns'])
        if v is None:
            continue
        speeds.append(v)
        if v < .1:
            x, y = p['front_point_map_xyz_m'][:2]
            stationary.append(dict(distance_to_line_m=before_sign*(dx*(y-a[1])-dy*(x-a[0]))/length,
                                   trigger_relative_sec=p['receipt_monotonic']-start))
    result['motion_before_crossing'] = dict(
        raw_speed_range_mps=[min(speeds), max(speeds)] if speeds else None,
        max_brake=max((r.get('brake', 0.) for r in commands), default=None),
        stationary_samples=len(stationary),
        stationary_front_line_range_m=[min(r['distance_to_line_m'] for r in stationary),
                                      max(r['distance_to_line_m'] for r in stationary)] if stationary else None,
        stationary_first_last_sec=[stationary[0]['trigger_relative_sec'], stationary[-1]['trigger_relative_sec']] if stationary else None)
    # Recover actual campaign from the frozen launch argv rather than names.
    campaign = None
    for r in wrapper_events:
        for arg in r.get('command', []):
            if arg.startswith('network_config:='):
                candidate = Path(arg.split(':=', 1)[1]).parent.parent
                if (candidate/'stack_logs/planning.log').is_file():
                    campaign = candidate
                    break
        if campaign:
            break
    logs = []
    if campaign:
        path = campaign/'stack_logs/planning.log';text = path.read_text(errors='replace')
        regex = r'\[(\d+\.\d+)\].*\[signal_timed_yellow\].*allow=(\d).*reason=(\S+).*remaining_sec=([\w.+-]+).*arrival_sec=([\w.+-]+).*front_line_m=([\w.+-]+)'
        for line in text.splitlines():
            m = re.search(regex, line)
            if m and start_ros <= float(m[1]) <= end_ros:
                logs.append(dict(stamp_sec=float(m[1]), allow=int(m[2]), reason=m[3],
                                 remaining_sec=finite_number(m[4]), arrival_sec=finite_number(m[5]), front_line_m=finite_number(m[6]),
                                 actual_until_front_cross_sec=end_ros-float(m[1]) if cross else None))
        result['campaign'] = str(campaign)
        result['planning_log_sha256'] = hash_file(path)
        summary = campaign/'summary.csv'
        if summary.is_file():
            with summary.open() as stream:
                cases = list(csv.DictReader(stream))
            result['case_summary'] = [{k: row.get(k) for k in ('scenario', 'status', 'reason', 'collision_count', 'result_path')}
                                      for row in cases]
            result['case_summary_sha256'] = hash_file(summary)
            safety = []
            for case in cases:
                receipt = campaign/Path(case['result_path']).parent/'safety_receipt.json'
                if receipt.is_file():
                    body = json.loads(receipt.read_text())
                    safety.append(dict(path=str(receipt), sha256=hash_file(receipt),
                                       startup_park_verified=body.get('startup', {}).get('park_verified'),
                                       shutdown_status=body.get('shutdown', {}).get('status'),
                                       shutdown_park_verified=body.get('shutdown', {}).get('stop', {}).get('park_verified')))
            result['case_safety_receipts'] = safety
    result['timed_decisions_before_crossing'] = logs
    result['logged_allow_to_deny_reversals_before_crossing'] = sum(a['allow'] == 1 and b['allow'] == 0 for a, b in zip(logs, logs[1:]))
    result['comparison_caveat'] = 'Trigger kinematics differ; RED is forced3sec thenGO. No natural-cycle benefit or official penalty inferred.'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = dict(development_only=True, results=[audit(p) for p in args.trials])
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    if args.output:
        if args.output.exists():
            raise FileExistsError('Fresh derived output required')
        args.output.parent.mkdir(parents=True, exist_ok=True);args.output.write_text(text)
    else:
        print(text, end='')


if __name__ == '__main__':
    main()
