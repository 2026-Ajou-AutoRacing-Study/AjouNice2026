#!/usr/bin/env python3
"""Diagnostic-only one-shot scheduler delay after service discovery (no ROS command change)."""
import argparse
from pathlib import Path
import runpy
import sys
import time
import rospy


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe-delay-sec',type=float,required=True)
    args,remaining=p.parse_known_args()
    if not 0<args.probe_delay_sec<=5:raise ValueError('Bounded diagnostic delay required')
    import json
    manifest=Path(remaining[remaining.index('--manifest')+1])
    deck=json.loads(manifest.read_text())
    if len(deck['scenes'])!=1 or not json.loads(Path(deck['scenes'][0]['metadata']).read_text()).get('calibration_only'):
        raise ValueError('Delay probe requires one calibration-only scene')
    original=rospy.wait_for_service;injected=False
    def wait(name,*values,**options):
        nonlocal injected
        result=original(name,*values,**options)
        if name=='/Service_MoraiEventCmd' and not injected:
            injected=True
            print('DIAGNOSTIC: extra preparation delay',args.probe_delay_sec,flush=True)
            time.sleep(args.probe_delay_sec)
        return result
    rospy.wait_for_service=wait
    runner=Path(__file__).with_name('run_base5_capture.py')
    sys.argv=[str(runner)]+remaining
    runpy.run_path(str(runner),run_name='__main__')


if __name__=='__main__':main()
