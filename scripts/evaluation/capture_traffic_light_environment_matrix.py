#!/usr/bin/env python3
"""Stationary development-only six-environment signal probe.

Requires an independently owned Park/brake harness and a sourced ROS environment.
Uses the official environment API, records successful Set/Get responses, then
runs the subscriber-only phase sampler. No planning/controller launch or Ego
movement. Environment API readback is not renderer verification or lamp GT.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ENVIRONMENTS = [('Sunny',13),('Sunny',11),('Sunny',15),
                ('Foggy',13),('Foggy',11),('Foggy',15)]


def select_environments(value):
    if value is None:
        return list(ENVIRONMENTS)
    available = {weather.lower()+str(hour):(weather,hour) for weather,hour in ENVIRONMENTS}
    keys = value.split(',')
    if not keys or len(keys)!=len(set(keys)) or any(key not in available for key in keys):
        raise ValueError('Use unique comma-separated conditions such as foggy11,foggy15')
    return [available[key] for key in keys]


def write_new(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)


def check_environment(report, weather, hour):
    actual = report.get('actual', {})
    if (report.get('status') != 'READBACK_MATCH' or not report.get('readback_match')
            or actual.get('weather') != weather or actual.get('hour') != hour):
        raise RuntimeError('Environment request/readback did not match')


def api(weather=None, hour=None):
    command = [sys.executable,str(HERE/'morai_traffic_light_weather_api.py')]
    if weather is None:
        command += ['--get']
    else:
        command += ['--set-weather',weather,'--set-hour',str(hour),'--apply-development-weather']
    completed = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, timeout=18)
    report = json.loads(completed.stdout)
    if completed.returncode:
        raise RuntimeError(json.dumps(report, ensure_ascii=False))
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--native-id',required=True)
    p.add_argument('--states',default='1,4,16,48')
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--samples-per-state',default=3,type=int)
    p.add_argument('--verify-native-gt',action='store_true',
                   help='Bracket each fresh JPEG with independent native signal reads')
    p.add_argument('--environments',default=None,
                   help='Optional explicit subset; old successful cells are not modified')
    p.add_argument('--apply-development-weather',required=True,action='store_true')
    args = p.parse_args()
    environments = select_environments(args.environments)
    if args.output.exists():
        p.error('Output must be a fresh directory')
    args.output.mkdir(parents=True)
    write_new(args.output/'initial_environment.json',api())
    rows = []
    try:
        for weather,hour in environments:
            cell = args.output/('{}{:02d}'.format(weather.lower(),hour))
            before = api(weather,hour)
            write_new(args.output/(cell.name+'_environment_before.json'),before)
            check_environment(before,weather,hour)
            command = [sys.executable,str(HERE/'capture_traffic_light_phase_sequence.py'),
                '--native-id',args.native_id,'--states',args.states,'--output',str(cell),
                '--weather-annotation',weather,'--time-annotation',str(hour),
                '--samples-per-state',str(args.samples_per_state),'--timeout-sec','45']
            if args.verify_native_gt:
                command += ['--verify-native-gt','--native-mismatch-retries','2','--native-rpc-retries','2']
            done = subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=52)
            write_new(args.output/(cell.name+'_capture_process.json'),
                      {'command':command,'returncode':done.returncode,'stdout':done.stdout,'stderr':done.stderr})
            if done.returncode:
                raise RuntimeError('Capture failed: '+done.stderr[-1500:])
            after = api()
            write_new(args.output/(cell.name+'_environment_after.json'),after)
            if after['actual']['weather']!=weather or after['actual']['hour']!=hour:
                raise RuntimeError('Environment changed during cell capture')
            rows.append({'cell':cell.name,'weather':weather,'hour':hour,
                         'api_pre_post_match':True,'renderer_verified':False,
                         'native_gt_verification_requested':args.verify_native_gt,
                         'lamp_render_ground_truth_verified':False})
            write_new(args.output/(cell.name+'_qualification.json'),rows[-1])
            print(json.dumps(rows[-1]),flush=True)
        write_new(args.output/'matrix.json',{'status':'complete_api_verified_render_and_lamp_review_required',
                                            'selected_environments':environments,'cells':rows})
    finally:
        # A fresh protocol response is still not proof of what the renderer shows.
        restored = api('Sunny',13)
        write_new(args.output/'restore_sunny13.json',restored)


if __name__=='__main__':
    main()
