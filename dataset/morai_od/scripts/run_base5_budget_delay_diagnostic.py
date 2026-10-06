#!/usr/bin/env python3
"""One calibration-only live fault probe: missed 6s target -> verified 9s retry."""
import argparse
import copy
from pathlib import Path
import sys
import run_base5_quality_loop as loop


def execute(source,index,output,delay=4.):
    if output.exists():raise FileExistsError('New diagnostic output required')
    if not 0<delay<=5:raise ValueError('Diagnostic delay bounded to five seconds')
    original=loop.gen.geo.load_json(source);entry=copy.deepcopy(original['scenes'][index])
    if original['config']['capture'].get('natural_setup_budget_sec')!=6:
        raise ValueError('Diagnostic requires a six-second candidate')
    output.mkdir(parents=True)
    meta=loop.gen.geo.load_json(Path(entry['metadata']))
    if not any(a.get('motion_path',{}).get('policy')=='natural-target-v1' for a in meta['runtime_actors']):
        raise ValueError('Natural-motion scene required')
    harness=Path(__file__);delay_runner=Path(__file__).with_name('run_base5_service_delay_probe.py')
    def tag(metadata):
        metadata.update(calibration_only=True,budget_delay_diagnostic=dict(
            delay_sec=delay,harness_sha256=loop.sha256(harness),delay_runner_sha256=loop.sha256(delay_runner),
            policy='Not a training quota; scheduler delay injected only into first attempt'))
    tag(meta);mp=output/'source_metadata.json';loop.atomic_json(mp,meta);entry['metadata']=str(mp.resolve())
    deck=copy.deepcopy({k:v for k,v in original.items() if k!='scenes'});deck.update(scenes=[entry],count=1)
    manifest=output/'manifest.json';loop.atomic_json(manifest,deck)
    real_popen=loop.subprocess.Popen;real_retry=loop.budget_retry
    def popen(command,**options):
        root=Path(command[command.index('--output')+1])
        if root.name=='initial':
            command=[sys.executable,str(delay_runner),'--probe-delay-sec',str(delay)]+command[2:]
        return real_popen(command,**options)
    def retry(*args,**kwargs):
        path=real_retry(*args,**kwargs)
        generated=loop.gen.geo.load_json(path)
        for row in generated['scenes']:
            mp=Path(row['metadata']);data=loop.gen.geo.load_json(mp);tag(data);loop.atomic_json(mp,data)
        return path
    loop.subprocess.Popen=popen;loop.budget_retry=retry
    try:return loop.execute(manifest,[0],output/'quality')
    finally:loop.subprocess.Popen=real_popen;loop.budget_retry=real_retry


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--index',type=int,default=0)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--delay-sec',type=float,default=4)
    a=p.parse_args();raise SystemExit(execute(a.manifest,a.index,a.output,a.delay_sec))
