#!/usr/bin/env python3
"""Owned stationary MORAI campaign with natural synchronized traffic programs.

Development-only. Starts the existing P/brake probe, loads explicit stationary
poses, saves untouched JPEGs, and brackets each image with native state queries.
Never sends a traffic-light override, planning launch or driving command.
Finally restores the measured initial pose, Sunny13 and measured P, then stops
only the probe processes it owns. Original scenario and old artifacts are kept.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time

from capture_traffic_light_environment_matrix import api, check_environment
from capture_traffic_light_phase_sequence import read_native_rpc_with_retries
from probe_morai_traffic_light_gt import query_states, MODERN_SERVICE


class Probe:
    def __init__(self, output, scenario_dir):
        self.process = subprocess.Popen(['/usr/bin/python3', str(Path(__file__).with_name('traffic_light_stationary_probe.py')),
            '--output', str(output), '--scenario-dir', str(scenario_dir)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        self.buffer = b''
        self.transcript = (output.parent / (output.name+'_controller.jsonl')).open('x')
    def wait(self, event, timeout=30):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if b'\n' not in self.buffer:
                ready, _, _ = select.select([self.process.stdout], [], [], min(.5, deadline-time.monotonic()))
                if not ready:
                    if self.process.poll() is not None:
                        raise RuntimeError('Probe exited: '+self.process.stderr.read().decode()[-1500:])
                    continue
                data = os.read(self.process.stdout.fileno(), 65536)
                if not data:
                    raise RuntimeError('Probe output closed')
                self.buffer += data
            line, self.buffer = self.buffer.split(b'\n', 1)
            row = json.loads(line)
            self.transcript.write(json.dumps(row)+'\n'); self.transcript.flush()
            if row['event'] in ('command_failed', 'probe_failed', 'cleanup_failed'):
                raise RuntimeError(str(row))
            if row['event'] == event:
                return row
        raise TimeoutError('Probe event timeout: '+event)
    def command(self, row, event, timeout=30):
        self.process.stdin.write((json.dumps(row)+'\n').encode()); self.process.stdin.flush()
        return self.wait(event, timeout)
    def close(self):
        if self.process.poll() is None:
            self.command({'action':'stop'}, 'shutdown_stationary', 20)
            self.process.stdin.close()
            self.process.wait(timeout=15)
        if self.process.returncode:
            raise RuntimeError('Probe cleanup failed')
        self.transcript.close()


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)


def native_brackets_stable(before, after):
    if not before or len(before) != len(after):
        return False
    for a, b in zip(before, after):
        if (not a.get('readback_success') or not b.get('readback_success')
                or a.get('requested_id') != b.get('requested_id')
                or len(a.get('observations', [])) != 1 or len(b.get('observations', [])) != 1
                or a['observations'][0].get('tl_color') != b['observations'][0].get('tl_color')):
            return False
    return True


def absolute_output(path):
    return Path(path).expanduser().absolute()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--scenario-dir', required=True, type=Path)
    parser.add_argument('--source-scene', required=True, type=Path)
    parser.add_argument('--samples-per-pose', type=int, default=3)
    parser.add_argument('--sequence-duration-sec', type=float, default=0.,
                        help='Optional 20..180 second natural sequence without another scene load')
    parser.add_argument('--settle-after-load-sec', type=float, default=.35)
    args=parser.parse_args()
    args.output=absolute_output(args.output)
    if (args.output.exists() or not 1 <= args.samples_per_pose <= 5
            or not (args.sequence_duration_sec == 0 or 20 <= args.sequence_duration_sec <= 180)
            or not 0 <= args.settle_after_load_sec <= 30):
        parser.error('Fresh output, samples1..5, optional duration20..180, settle0..30 required')
    plan=json.loads(args.plan.read_text())
    if not plan.get('poses') or not plan.get('environments'):
        parser.error('Explicit poses and environments required')
    args.output.mkdir(parents=True)
    write(args.output/'plan.json', plan)
    source=Path(__file__).read_bytes()
    with (args.output/'capture_source_at_launch.py').open('xb') as stream:
        stream.write(source)
    write(args.output/'capture_contract.json',{'source_sha256':hashlib.sha256(source).hexdigest(),
        'plan_sha256':hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        'sequence_duration_sec':args.sequence_duration_sec,'settle_after_load_sec':args.settle_after_load_sec,
        'samples_per_pose':args.samples_per_pose,'no_signal_override':True})
    records=[]; errors=[]
    import grpc
    probe=Probe(args.output/'owned_probe',args.scenario_dir)
    initial=None
    with grpc.insecure_channel('127.0.0.1:7789') as channel:
        def call(path, request):
            if path != '/'+MODERN_SERVICE+'/GetTrafficLightInfo':
                raise ValueError('Only traffic-light Get allowed')
            method=channel.unary_unary(path,request_serializer=lambda x:x,response_deserializer=lambda x:x)
            return read_native_rpc_with_retries(lambda timeout:method(request,timeout=timeout),
                retries=2,rpc_error_type=grpc.RpcError,per_call_timeout=2.)
        def states(ids):
            return query_states([('traffic_light',key) for key in ids],call,'modern')
        try:
            ready=probe.wait('ready',25)
            measured=ready['status']; pos=measured['position']
            initial={**pos,'yaw':measured['heading']-1.347}
            write(args.output/'initial_pose.json', {'pose':initial,'status':measured})
            for weather,hour in plan['environments']:
                before=api(weather,hour);check_environment(before,weather,hour)
                env=weather+str(hour)
                write(args.output/(env+'_environment_before.json'),before)
                for scene in plan['poses']:
                    probe.command({'action':'load','source':str(args.source_scene),'pose':scene['pose']},'loaded',30)
                    time.sleep(args.settle_after_load_sec)
                    sample=0; sequence_started=time.monotonic()
                    while (time.monotonic()-sequence_started < args.sequence_duration_sec
                           if args.sequence_duration_sec else sample < args.samples_per_pose):
                        pre=states(scene['native_ids'])
                        name=env+'_'+scene['group_id']+'_'+str(sample)
                        capture=probe.command({'action':'capture','name':name,'weather_annotation':env},'captured',15)
                        post=states(scene['native_ids'])
                        matching=native_brackets_stable(pre,post)
                        records.append({'image_path':str(Path(capture['image_path']).relative_to(args.output)),
                            'image_sha256':capture['sha256'],'group_id':scene['group_id'],'split':scene['split'],
                            'environment':env,'width':1280,'height':720,'ego':capture['status'],
                            'header_stamp':capture['header_stamp'],'native_before':pre,'native_after':post,
                            'native_bracket_stable':matching,'requested_signal_override':False,
                            'natural_program_capture':True,'group_regulatory_id':scene['group_regulatory_id']})
                        if not args.sequence_duration_sec or sample%10==0:
                            print(json.dumps({'event':'captured_natural','environment':env,'group':scene['group_id'],'count':len(records),'stable':matching}),flush=True)
                        write(args.output/(name+'_record.json'),records[-1])
                        sample+=1
                after=api();write(args.output/(env+'_environment_after.json'),after)
                if after['actual']['weather']!=weather or after['actual']['hour']!=hour:
                    raise RuntimeError('Environment changed during collection')
        except BaseException as error:
            errors.append(str(error)); raise
        finally:
            try:
                if initial is not None:
                    restoration=probe.command({'action':'load','source':str(args.source_scene),'pose':initial},'loaded',30)
                    write(args.output/'restore_pose.json',restoration)
                write(args.output/'restore_environment.json',api('Sunny',13))
            except BaseException as error:
                errors.append('restoration_failed: '+str(error)); raise
            finally:
                try: probe.close()
                except BaseException as error:
                    errors.append('probe_cleanup_failed: '+str(error)); raise
                finally: write(args.output/'manifest.json',{'status':'complete' if not errors else 'failed','records':records,'errors':errors,'development_only':True,'state_is_not_pixel_visibility_GT':True})


if __name__=='__main__':
    main()
