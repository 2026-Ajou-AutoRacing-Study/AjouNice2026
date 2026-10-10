#!/usr/bin/env python3
"""Bounded actual 5K loader/training cost qualification; never a full epoch."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

import numpy as np
import torch
from mmengine.hooks import Hook
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import HOOKS

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from run_base5_direct_training import prepare
from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner


class PlannedStop(Exception): pass


def summary(values):
    return dict(mean=float(np.mean(values)), p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)), p99=float(np.percentile(values,99)))


@HOOKS.register_module(force=True)
class Base5BoundedCost(Hook):
    # LoggerHook resets CUDA peak statistics; collect BEFORE it, and keep our
    # own maxima across iterations instead of trusting the final reset window.
    priority='ABOVE_NORMAL'
    def __init__(self, steps, warmup, output, batch):
        self.steps=steps; self.warmup=warmup; self.output=Path(output); self.batch=batch; self.rows=[]
        self.peak_allocated=0; self.peak_reserved=0; self.warm_peak=0
        self.seen=set(); self.measured_seen=set(); self.label_counts=Counter()
    def before_train(self, runner):
        self.started=time.perf_counter(); self.previous=self.started
    def before_train_iter(self, runner, batch_idx, data_batch=None):
        for sample in data_batch['data_samples']:
            identity=str(sample.metainfo.get('scene_id',sample.metainfo.get('sample_idx','')))
            if not identity: raise ValueError('Missing actual loader sample identity')
            self.seen.add(identity)
            if batch_idx>=self.warmup: self.measured_seen.add(identity)
            self.label_counts.update(map(int,sample.gt_instances_3d.labels_3d.tolist()))
        torch.cuda.synchronize(); self.begin=time.perf_counter()
        self.wait=(self.begin-self.previous)*1000
        if batch_idx==self.warmup:
            torch.cuda.empty_cache()
            torch.cuda.reset_peak_memory_stats()
    def after_train_iter(self, runner, batch_idx, data_batch=None, outputs=None):
        torch.cuda.synchronize(); end=time.perf_counter()
        allocated=torch.cuda.max_memory_allocated()/2**20
        reserved=torch.cuda.max_memory_reserved()/2**20
        if batch_idx < self.warmup:
            self.warm_peak=max(self.warm_peak,allocated)
        else:
            self.peak_allocated=max(self.peak_allocated,allocated)
            self.peak_reserved=max(self.peak_reserved,reserved)
        values={k:float(v.detach().cpu()) if torch.is_tensor(v) else float(v)
                for k,v in outputs.items() if 'loss' in k}
        if not values or not all(np.isfinite(v) for v in values.values()):
            raise ValueError('Missing/nonfinite training losses')
        self.rows.append(dict(step=batch_idx, loader_wait_ms=self.wait,
            train_ms=(end-self.begin)*1000, total_ms=(end-self.previous)*1000, losses=values))
        self.previous=end
        if (batch_idx+1)%16==0: print('COST STEP %d/%d'%(batch_idx+1,self.steps),flush=True)
        if batch_idx+1==self.steps:
            optimizer_steps=sorted({float(v['step']) for v in runner.optim_wrapper.optimizer.state.values() if 'step' in v})
            if optimizer_steps != [float(self.steps)]:
                raise ValueError('Optimizer did not advance on every bounded probe step')
            measured=self.rows[self.warmup:]
            result=dict(valid=True, batch=self.batch, measured_steps=len(measured),
                distinct_measured_examples_upper_bound=len(measured)*self.batch,
                actual_distinct_examples=len(self.seen), actual_distinct_measured_examples=len(self.measured_seen),
                labels_after_training_augmentation=dict(self.label_counts),
                timing={k:summary([r[k] for r in measured]) for k in ('loader_wait_ms','train_ms','total_ms')},
                peak_allocated_mib=self.peak_allocated,
                peak_reserved_mib=self.peak_reserved,
                warmup_peak_allocated_mib=self.warm_peak,
                memory_measurement='before LoggerHook reset, aggregate measured-window maxima; release autotune cache at warmup boundary',
                optimizer_steps=optimizer_steps,
                scaler=runner.optim_wrapper.loss_scaler.state_dict(),
                elapsed_seconds=end-self.started, per_step=self.rows,
                scope='bounded actual train loader + augmentation + AMP AdamW; no full epoch, validation or checkpoint cost')
            self.output.write_text(json.dumps(result,indent=2)+'\n')
            raise PlannedStop('Bounded cost probe complete')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True); p.add_argument('--config',type=Path,required=True)
    p.add_argument('--profile',choices=['448x896','512x1056'],required=True)
    p.add_argument('--work-dir',type=Path,required=True); p.add_argument('--init-checkpoint',type=Path,required=True)
    p.add_argument('--batch',type=int,required=True); p.add_argument('--workers',type=int,default=4)
    p.add_argument('--steps',type=int,default=80); p.add_argument('--warmup',type=int,default=16)
    a=p.parse_args()
    if not 16<a.steps<=160 or not 0<a.warmup<a.steps or a.work_dir.exists():
        p.error('Require fresh output and bounded budget')
    torch.set_num_threads(4); torch.backends.cudnn.benchmark=True
    args=argparse.Namespace(data=a.data,profile=a.profile,config=a.config,work_dir=a.work_dir,
        epochs=2,batch=a.batch,accumulate=1,workers=a.workers,lr=1e-4,warmup_updates=50,
        seed=261010,init_checkpoint=a.init_checkpoint,allow_random_init=False,resume=None,
        disable_val=True,wandb=False,run=False)
    cfg,manifest=prepare(args); import_modules_from_strings(**cfg.custom_imports)
    cfg.default_hooks.checkpoint=None
    cfg.default_hooks.logger.interval=16
    cfg.custom_hooks.append(dict(type='Base5BoundedCost',steps=a.steps,warmup=a.warmup,
        output=str(a.work_dir/'cost.json'),batch=a.batch))
    try: Base5AtomicRunner.from_cfg(cfg).train()
    except PlannedStop:
        result=json.loads((a.work_dir/'cost.json').read_text())
        result.update(profile=a.profile,workers=a.workers,contract_sha256=manifest['contract_sha256'],
            full_epoch_training_started=False)
        (a.work_dir/'cost.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='per_step'},indent=2))
    except torch.cuda.OutOfMemoryError:
        (a.work_dir/'cost.json').write_text(json.dumps(dict(valid=False,oom=True,profile=a.profile,batch=a.batch)))
        raise SystemExit(2)
    else: raise RuntimeError('Probe unexpectedly completed an epoch')


if __name__=='__main__': main()
