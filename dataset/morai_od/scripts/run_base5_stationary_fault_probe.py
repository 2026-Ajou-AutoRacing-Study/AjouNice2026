#!/usr/bin/env python3
"""Pilot-only injection: reject the first calibration scene and exercise real tail recovery."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import run_base5_batch_campaign as batch


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();data=json.loads(a.manifest.read_text())
    if len(data['scenes'])!=2 or not json.loads(Path(data['scenes'][0]['metadata']).read_text()).get('calibration_only'):
        raise ValueError('Fault probe requires exactly two cells and a calibration-only first cell')
    original=batch.run_child
    def child(manifest,root,stop,heartbeat):
        root.mkdir(parents=True,exist_ok=True)
        code="""
import sys
import run_base5_capture as capture
original=capture.bounded_stationary_recheck
def forced(observe,guard,static,**kwargs):
    def rejected():
        row=observe();row.update(verified=False,test_injected=True);return row
    return original(rejected,guard,static,**kwargs)
capture.bounded_stationary_recheck=forced
capture.main()
"""
        command=[sys.executable,'-c',code,'--manifest',str(manifest),'--output',str(root),'--shared-ros-batch','--limit','2']
        import os
        environment=dict(os.environ,PYTHONPATH=str(Path(batch.__file__).parent)+':'+os.environ.get('PYTHONPATH',''))
        with (root.parent/'batch.log').open('a') as log:
            process=subprocess.Popen(command,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            return batch.wait_child(process,root,stop,heartbeat,100)
    batch.run_child=child
    try:return batch.execute(a.manifest,[0,1],a.output,2)
    finally:batch.run_child=original


if __name__=='__main__':raise SystemExit(main())
