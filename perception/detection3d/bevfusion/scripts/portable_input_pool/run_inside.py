#!/usr/bin/env python3
"""Direct process launcher inside the compatible team/Vast image."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile


def main():
    p=argparse.ArgumentParser();p.add_argument('--iterations',type=int,default=42)
    p.add_argument('--skip-training',action='store_true')
    p.add_argument('--shapes',nargs='+',default=['384x896','448x896','512x1056'])
    p.add_argument('--verify-only',action='store_true');a=p.parse_args()
    here=Path(__file__).resolve().parent;root=here/'payload'
    manifest=json.loads((here/'manifest.json').read_text())
    for rel,sha in manifest['sha256'].items():
        if hashlib.sha256((here/rel).read_bytes()).hexdigest()!=sha:raise ValueError('Checksum mismatch: '+rel)
    if a.verify_only:
        print('PAYLOAD CHECKSUMS PASS');return
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S');out=here/('results_'+stamp);out.mkdir()
    env=os.environ.copy();env['PYTHONPATH']=':'.join([str(root),str(root/'docker/bevfusion'),
        '/opt/mmdetection3d',env.get('PYTHONPATH','')]);env.setdefault('MAX_JOBS','2')
    scripts=root/'perception/detection3d/bevfusion/scripts';common=['--root',str(root),'--samples',str(root/'samples')]
    cases=[['test_pool_optimization.py','--output',str(out/'pool_units.json')],
        ['benchmark_base5_input_optimization.py',*common,'--output',str(out/'jpeg_cost.json'),
         '--iterations',str(a.iterations),'--codec','jpeg90','--shapes',*a.shapes,
         '--modes','baseline','cache','separated'],
        ['validate_base5_pool_model.py',*common,'--infos',str(root/'infos'),
         '--output',str(out/'full_model_validation.json'),'--shapes',*a.shapes,'--steps','3']]
    if a.skip_training:cases[-1].append('--skip-training')
    statuses=[]
    with (out/'console.log').open('w') as log:
        def run(cmd):
            proc=subprocess.Popen(cmd,env=env,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            for line in proc.stdout:print(line,end='',flush=True);log.write(line);log.flush()
            return proc.wait()
        run(['nvidia-smi'])
        for arguments in cases:
            rc=run([sys.executable,'-u',str(scripts/arguments[0]),*arguments[1:]])
            statuses.append(dict(script=arguments[0],exit_code=rc))
            if rc:break
    (out/'execution.json').write_text(json.dumps(dict(statuses=statuses,python=sys.version,
        source_manifest=manifest,cost_only=True,fullstack_measured=False),indent=2)+'\n')
    archive=here/('SEND_THIS_'+stamp+'.zip')
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=3) as z:
        for f in out.rglob('*'):
            if f.is_file():z.write(f,str(f.relative_to(out)))
    print('SEND THIS ZIP: '+str(archive),flush=True)
    if any(v['exit_code'] for v in statuses):raise SystemExit(1)


if __name__=='__main__':main()
