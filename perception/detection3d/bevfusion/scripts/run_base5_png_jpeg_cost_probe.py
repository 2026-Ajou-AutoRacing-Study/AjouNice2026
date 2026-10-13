#!/usr/bin/env python3
"""Very short PNG vs preencoded Q90 real-training cost comparison, not AP.

Four fresh runs in reversed order use the same initialization, sampler seed,
batch, augmentation and AMP optimizer. No full epoch or model checkpoint.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import torch


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--png-data',type=Path,required=True)
    p.add_argument('--jpeg-data',type=Path,required=True)
    p.add_argument('--init-checkpoint',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--batch',type=int,default=2)
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--steps',type=int,default=32)
    p.add_argument('--warmup',type=int,default=8)
    a=p.parse_args()
    if a.output.exists() or not 16<a.steps<=64 or not 0<a.warmup<a.steps:
        p.error('Fresh output and bounded 17..64 steps required')
    if not torch.cuda.is_available():p.error('CUDA GPU required')
    root=Path(__file__).resolve().parents[1]
    a.output.mkdir(parents=True)
    rows=[]
    for i,mode in enumerate(('png','jpeg','jpeg','png')):
        cfg=root/'configs'/('bevfusion_base5_dense_512_separated_candidate.py' if mode=='png'
                            else 'bevfusion_base5_dense_512_jpeg90_train.py')
        work=a.output/('%d_%s'%(i,mode))
        command=[sys.executable,str(root/'scripts/benchmark_base5_real_loader.py'),
            '--data',str(a.png_data if mode=='png' else a.jpeg_data),
            '--config',str(cfg),'--profile','512x1056','--work-dir',str(work),
            '--init-checkpoint',str(a.init_checkpoint),'--batch',str(a.batch),
            '--workers',str(a.workers),'--steps',str(a.steps),'--warmup',str(a.warmup),
            '--pixel-io-comparison']
        print('START %d/4: %s'%(i+1,mode),flush=True)
        log=a.output/('%d_%s.log'%(i,mode))
        with log.open('w') as handle:
            result=subprocess.run(command,stdout=handle,stderr=subprocess.STDOUT)
        if result.returncode:
            print(log.read_text()[-6000:],flush=True)
            raise RuntimeError('Bounded training probe failed: '+mode)
        cost=json.loads((work/'cost.json').read_text())
        if not cost['valid']:raise ValueError('Invalid cost result')
        rows.append(dict(mode=mode,run=i,cost=cost))
        print(json.dumps(dict(mode=mode,timing=cost['timing'],peak_allocated_mib=cost['peak_allocated_mib'])),flush=True)
    reference=rows[0]['cost']['per_step']
    for row in rows[1:]:
        for left,right in zip(reference,row['cost']['per_step']):
            if (left['scene_ids'],left['gt_geometry_sha256']) != (right['scene_ids'],right['gt_geometry_sha256']):
                raise ValueError('Sampler/GT/BEV augmentation differs; not a paired cost comparison')
    aggregate={}
    for mode in ('png','jpeg'):
        samples=[r for row in rows if row['mode']==mode for r in row['cost']['per_step'][a.warmup:]]
        aggregate[mode]={key:dict(mean=float(np.mean([r[key] for r in samples])),
                                 p50=float(np.median([r[key] for r in samples])))
                         for key in ('loader_wait_ms','train_ms','total_ms')}
    report=dict(valid=True,gpu=torch.cuda.get_device_name(),batch=a.batch,workers=a.workers,
        steps_per_run=a.steps,warmup_per_run=a.warmup,same_scene_order_gt_and_geometry=True,
        aggregate=aggregate,rows=rows,
        total_time_reduction_percent=100*(1-aggregate['jpeg']['total_ms']['mean']/aggregate['png']['total_ms']['mean']),
        limitations=['Very short warm-cache local GPU comparison; not convergence or AP',
                     'Not a 4090/full-epoch time guarantee; excludes validation and checkpoint overhead',
                     'Original PNG has no JPEG transform, JPEG is preencoded once; image tensors intentionally differ'])
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2),flush=True)


if __name__=='__main__':main()
