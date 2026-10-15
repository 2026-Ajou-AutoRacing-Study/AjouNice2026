#!/usr/bin/env python3
"""Bounded GT-controlled live grouping comparison; no validator source edits.

Research-only wrapper injects a named grouping into the existing launch
command in this process. Existing validator safety/load/stop remain intact.
Neither detector weights nor a production sensor default is changed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import yaml
from run_lidar_packet_probe_auto import ROOT, VALIDATOR, prepare, write_json
from run_perception_candidate_probe import POLICIES
import record_lidar_packet_probe as probe


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--run',action='store_true')
    parser.add_argument('--child-config',type=Path)
    parser.add_argument('--policy',choices=POLICIES)
    args=parser.parse_args()
    if args.child_config:
        if not args.policy:
            parser.error('Child requires explicit policy')
        from planning_scenario_validator import runner
        original=runner._launch
        def research_launch(package,filename,**kwargs):
            if package=='bringup' and filename=='maptf_morai.launch':
                filename='localization_morai_gt.launch'
                kwargs['use_heading_correction']=True
            command=original(package,filename,**kwargs)
            if package=='sensor_interface' and filename=='morai_udp_interface.launch' and kwargs.get('enable_lidar'):
                command += POLICIES[args.policy]
            return command
        runner._launch=research_launch
        return runner.main([str(args.child_config)])
    if not args.output:
        parser.error('Parent requires output')
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    source=VALIDATOR/'src/planning_scenario_validator/runner.py'
    before=hashlib.sha256(source.read_bytes()).hexdigest()
    completed=[]
    for policy in POLICIES:
        configs=prepare(args.output/policy)
        for mission,path in configs.items():
            path=Path(path)
            document=yaml.safe_load(path.read_text())
            document['campaign_name']=f'lidar_{policy}_{mission}'
            path.write_text(yaml.safe_dump(document,allow_unicode=True,sort_keys=False))
            if not args.run:
                continue
            command=[sys.executable,str(Path(__file__).resolve()),'--child-config',str(path),'--policy',policy]
            with (args.output/(policy+'_'+mission+'.log')).open('w') as log:
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    code=child.wait(timeout=420)
                except (subprocess.TimeoutExpired,KeyboardInterrupt):
                    probe.stop_recorder(child)
                    raise
            if code:
                raise RuntimeError(f'Bounded {policy}/{mission} failed with {code}; evidence preserved')
            states=list((args.output/policy/'validator').glob(f'lidar_{policy}_{mission}_*/campaign_state.json'))
            if len(states)!=1:
                raise RuntimeError('Ambiguous campaign results')
            state=json.loads(states[0].read_text())
            if not state.get('results') or any(row['status']!='PASS' for row in state['results'].values()):
                raise RuntimeError('Controller validation failed; no inference/default promotion')
            bags=list(states[0].parent.rglob('*.bag'))
            if len(bags)!=1:
                raise RuntimeError('Expected one closed raw bag')
            import rosbag
            with rosbag.Bag(str(bags[0])) as bag:
                sizes=sorted({len(m.packets) for _,m,_ in bag.read_messages(topics=['/sensor/lidar1/packets'])})
            if sizes!=([51] if policy=='fixed51' else [50]):
                raise RuntimeError(f'Live policy mismatch: {policy} observed {sizes}')
            completed.append(dict(policy=policy,mission=mission,bag=str(bags[0].resolve()),packet_counts=sizes))
            write_json(args.output/'progress.json',dict(completed=completed,validator_source_sha256=before))
            print(json.dumps(completed[-1]),flush=True)
    if before!=hashlib.sha256(source.read_bytes()).hexdigest():
        raise RuntimeError('Validator source changed during trial; inspect before interpreting evidence')
    return 0


if __name__=='__main__':
    sys.exit(main())
