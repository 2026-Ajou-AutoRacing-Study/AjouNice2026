#!/usr/bin/env python3
"""Bounded actual 5K loader/training cost qualification; never a full epoch."""
import argparse
from collections import Counter
import hashlib
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
from base5_grid_candidates import candidate_config, SPATIAL_CASES
from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner


class PlannedStop(Exception): pass


def summary(values):
    return dict(mean=float(np.mean(values)), p50=float(np.percentile(values,50)),
                p95=float(np.percentile(values,95)), p99=float(np.percentile(values,99)))


@HOOKS.register_module(force=True)
class Base5TrainingProfile(Hook):
    """Bounded profiler window, excluded from ordinary cost statistics."""
    priority='HIGH'

    def __init__(self, warmup, steps, output):
        self.warmup=warmup; self.steps=steps; self.output=Path(output)
        self.profiler=None; self.originals=[]; self.spans=[]

    def before_train(self, runner):
        methods=[(runner.model.data_preprocessor,'forward','input_transfer'),
                 (runner.model.img_backbone,'forward','camera_backbone'),
                 (runner.model.img_neck,'forward','camera_neck'),
                 (runner.model.view_transform,'get_geometry','geometry_and_plan'),
                 (runner.model.view_transform,'get_cam_feats','depth_feature'),
                 (runner.model.view_transform,'bev_pool','pool_forward'),
                 (runner.model.pts_middle_encoder,'forward','sparse_lidar'),
                 (runner.model.bbox_head,'loss','head_and_targets'),
                 (runner.model,'parse_losses','scalar_logging'),
                 (runner.optim_wrapper,'backward','backward'),
                 (runner.optim_wrapper,'step','optimizer_and_clip')]
        for obj,name,label in methods:
            original=getattr(obj,name)
            def wrapped(*args,_fn=original,_label=label,**kwargs):
                if self.profiler is None:return _fn(*args,**kwargs)
                begin,end=torch.cuda.Event(enable_timing=True),torch.cuda.Event(enable_timing=True)
                begin.record(); started=time.perf_counter()
                with torch.profiler.record_function('BASE5::'+_label):
                    value=_fn(*args,**kwargs)
                end.record()
                self.spans.append((_label,begin,end,(time.perf_counter()-started)*1000))
                return value
            self.originals.append((obj,name,original)); setattr(obj,name,wrapped)

    def before_train_iter(self, runner, batch_idx, data_batch=None):
        active=self.warmup<=batch_idx<self.warmup+self.steps
        runner._base5_profiled_step=active
        if batch_idx==self.warmup:
            self.profiler=torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,
                torch.profiler.ProfilerActivity.CUDA],record_shapes=True)
            self.profiler.__enter__()

    def after_train_iter(self, runner, batch_idx, data_batch=None, outputs=None):
        if self.profiler is None:return
        self.profiler.step()
        if batch_idx+1==self.warmup+self.steps:
            self.profiler.__exit__(None,None,None)
            torch.cuda.synchronize()
            self.profiler.export_chrome_trace(str(self.output/'training_trace.json'))
            events=[]
            for event in self.profiler.key_averages():
                events.append(dict(name=event.key,calls=event.count,
                    self_cpu_ms=event.self_cpu_time_total/1000,
                    self_device_ms=getattr(event,'self_device_time_total',0)/1000,
                    total_cpu_ms=event.cpu_time_total/1000,
                    total_device_ms=getattr(event,'device_time_total',0)/1000))
            stages={}
            for label,begin,end,cpu_ms in self.spans:
                row=stages.setdefault(label,dict(stream_span_ms=[],host_call_ms=[]))
                row['stream_span_ms'].append(begin.elapsed_time(end));row['host_call_ms'].append(cpu_ms)
            (self.output/'training_profile.json').write_text(json.dumps(dict(
                steps=self.steps,events=sorted(events,key=lambda r:r['self_device_ms'],reverse=True),
                stages={label:{k:summary(v) for k,v in row.items()} for label,row in stages.items()},
                device_kernel_profile_available=any(e['self_device_ms']>0 for e in events),
                scope='Instrumented warm steps excluded from latency; CUDA event spans include host idle, not kernel self-time'),indent=2)+'\n')
            self.profiler=None
            for obj,name,original in self.originals:setattr(obj,name,original)
            self.originals=[]


@HOOKS.register_module(force=True)
class Base5BoundedCost(Hook):
    # LoggerHook resets CUDA peak statistics; collect BEFORE it, and keep our
    # own maxima across iterations instead of trusting the final reset window.
    # Install gradient audit BEFORE the profiler wraps optimizer.step, so its
    # method restoration keeps the audit alive after the profiling window.
    priority='VERY_HIGH'
    def __init__(self, steps, warmup, output, batch, audit_gradients=False):
        self.steps=steps; self.warmup=warmup; self.output=Path(output); self.batch=batch; self.rows=[]
        self.peak_allocated=0; self.peak_reserved=0; self.warm_peak=0
        self.seen=set(); self.measured_seen=set(); self.label_counts=Counter()
        self.audit_gradients=audit_gradients; self.gradient_audits=[]
    def before_train(self, runner):
        digest=hashlib.sha256()
        for name,value in runner.model.named_parameters():
            digest.update(name.encode());digest.update(value.detach().cpu().contiguous().numpy().tobytes())
        self.initial_parameters_sha256=digest.hexdigest()
        if self.audit_gradients:
            original_step=runner.optim_wrapper.step
            def audited_step(*args, **kwargs):
                if runner.iter in (0, self.warmup, self.steps//2, self.steps-1):
                    runner._base5_audited_step=True
                    names=[]; gradients=[]; missing=[]
                    for name,parameter in runner.model.named_parameters():
                        if not parameter.requires_grad:continue
                        if parameter.grad is None:missing.append(name)
                        else:names.append(name);gradients.append(parameter.grad)
                    if not gradients:raise ValueError('No training gradients')
                    finite=bool(torch.stack([torch.isfinite(g).all() for g in gradients]).all().item())
                    norms=torch.stack([g.detach().float().norm() for g in gradients])
                    scale=float(runner.optim_wrapper.loss_scaler.get_scale())
                    row=dict(step=runner.iter,finite=finite,gradient_names=names,
                        missing_gradient_names=missing,amp_scale=scale,
                        unscaled_global_l2=float(norms.norm().item())/scale,
                        nonzero_gradient_parameters=int((norms>0).sum().item()))
                    self.gradient_audits.append(row)
                    if not finite:raise ValueError('Nonfinite training gradients')
                    if missing:raise ValueError('Missing trainable-parameter gradients: '+str(missing))
                return original_step(*args, **kwargs)
            runner.optim_wrapper.step=audited_step
        self.started=time.perf_counter(); self.previous=self.started
    def before_train_iter(self, runner, batch_idx, data_batch=None):
        runner._base5_audited_step=False
        if batch_idx==0:
            digest=hashlib.sha256()
            for key in ('imgs','img','points'):
                values=data_batch['inputs'].get(key,[])
                for value in values if isinstance(values,list) else [values]:
                    digest.update(value.contiguous().numpy().tobytes())
            self.first_input_sha256=digest.hexdigest()
        self.batch_identities=[]; self.batch_gt_hashes=[]
        for sample in data_batch['data_samples']:
            identity=str(sample.metainfo.get('scene_id',sample.metainfo.get('sample_idx','')))
            if not identity: raise ValueError('Missing actual loader sample identity')
            self.seen.add(identity)
            self.batch_identities.append(identity)
            digest=hashlib.sha256(sample.gt_instances_3d.bboxes_3d.tensor.cpu().numpy().tobytes())
            digest.update(sample.gt_instances_3d.labels_3d.cpu().numpy().tobytes())
            digest.update(np.asarray(sample.metainfo['lidar_aug_matrix']).tobytes())
            self.batch_gt_hashes.append(digest.hexdigest())
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
            train_ms=(end-self.begin)*1000, total_ms=(end-self.previous)*1000, losses=values,
            scene_ids=self.batch_identities, gt_geometry_sha256=self.batch_gt_hashes,
            gradient_audited=bool(getattr(runner,'_base5_audited_step',False)),
            profiled=bool(getattr(runner,'_base5_profiled_step',False))))
        self.previous=end
        if (batch_idx+1)%16==0: print('COST STEP %d/%d'%(batch_idx+1,self.steps),flush=True)
        if batch_idx+1==self.steps:
            optimizer_steps=sorted({float(v['step']) for v in runner.optim_wrapper.optimizer.state.values() if 'step' in v})
            if optimizer_steps != [float(self.steps)]:
                raise ValueError('Optimizer did not advance on every bounded probe step')
            measured=[r for r in self.rows[self.warmup:] if not r['profiled'] and not r['gradient_audited']]
            parameter_finite=None
            if self.audit_gradients:
                parameter_finite=bool(torch.stack([torch.isfinite(p.detach()).all()
                    for p in runner.model.parameters()]).all().item())
                if not parameter_finite:raise ValueError('Nonfinite updated parameters')
            result=dict(valid=True, batch=self.batch, measured_steps=len(measured),
                initial_parameters_sha256=self.initial_parameters_sha256,
                first_input_sha256=self.first_input_sha256,
                pool_runtime_counters={k:runner.model.view_transform._pool_optimization.get(k,0)
                    for k in ('fast_geometry_calls','legacy_geometry_calls','boundary_rays_recomputed','bypasses','hits','geometry_guard')},
                distinct_measured_examples_upper_bound=len(measured)*self.batch,
                actual_distinct_examples=len(self.seen), actual_distinct_measured_examples=len(self.measured_seen),
                labels_after_training_augmentation=dict(self.label_counts),
                timing={k:summary([r[k] for r in measured]) for k in ('loader_wait_ms','train_ms','total_ms')},
                peak_allocated_mib=self.peak_allocated,
                peak_reserved_mib=self.peak_reserved,
                warmup_peak_allocated_mib=self.warm_peak,
                memory_measurement='before LoggerHook reset, aggregate measured-window maxima; release autotune cache at warmup boundary',
                optimizer_steps=optimizer_steps,
                gradient_audits=self.gradient_audits,final_parameters_finite=parameter_finite,
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
    p.add_argument('--pool-backward',choices=['legacy','warp','grouped'],default='legacy')
    p.add_argument('--compact-indices',action='store_true')
    p.add_argument('--geometry-mode',choices=['legacy','batched_checked'],default='legacy')
    p.add_argument('--batched-loss-log',action='store_true')
    p.add_argument('--fused-adamw',action='store_true')
    p.add_argument('--cudnn-benchmark',action='store_true')
    p.add_argument('--compile-camera',action='store_true',
                   help='Experimental bounded torch.compile of backbone forward only; parameter names unchanged')
    p.add_argument('--depth-step',type=float,choices=[.25,.5,1.])
    p.add_argument('--spatial-case',choices=SPATIAL_CASES)
    p.add_argument('--profile-steps',type=int,default=0,
                   help='Bounded torch profiler window, excluded from cost timing')
    p.add_argument('--audit-gradients',action='store_true',
                   help='Audit four real backward updates; exclude diagnostic steps from cost statistics')
    p.add_argument('--pixel-io-comparison',action='store_true',
                   help='Remove disabled pixel transform so PNG/JPEG consume the same geometry RNG')
    a=p.parse_args()
    if not 16<a.steps<=160 or not 0<a.warmup<a.steps or a.work_dir.exists():
        p.error('Require fresh output and bounded budget')
    if not 0<=a.profile_steps<=8 or a.warmup+a.profile_steps>=a.steps:
        p.error('Profile at most eight steps and leave uninstrumented measurements')
    torch.set_num_threads(4); torch.backends.cudnn.benchmark=True
    args=argparse.Namespace(data=a.data,profile=a.profile,config=a.config,work_dir=a.work_dir,
        epochs=2,batch=a.batch,accumulate=1,workers=a.workers,lr=1e-4,warmup_updates=50,
        seed=261010,init_checkpoint=a.init_checkpoint,allow_random_init=False,resume=None,
        disable_val=True,wandb=False,run=False)
    cfg,manifest=prepare(args); import_modules_from_strings(**cfg.custom_imports)
    if a.spatial_case:
        cfg=candidate_config(cfg,a.spatial_case,depth_step=a.depth_step or .5)
    if a.depth_step is not None:
        cfg.model.view_transform.dbound=[1.,100.,a.depth_step]
        if a.depth_step==.25:cfg.model.view_transform.pool_cache_limit_bytes=512*2**20
    cfg.model.view_transform.pool_backward_mode=a.pool_backward
    cfg.model.view_transform.pool_compact_indices=a.compact_indices
    cfg.model.view_transform.pool_geometry_mode=a.geometry_mode
    # Cost flags own these options; a preconfigured candidate must not turn a
    # purported baseline into a silently optimized run.
    cfg.model.type='BEVFusion'
    cfg.model.pop('compile_camera',None)
    cfg.optim_wrapper.optimizer.pop('fused',None)
    cfg.optim_wrapper.optimizer.pop('foreach',None)
    if a.batched_loss_log:
        import perception.detection3d.bevfusion.plugin.morai_training_runtime
        cfg.custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_training_runtime')
        cfg.model.type='MoraiTrainingRuntimeBEVFusion'
    if a.fused_adamw:
        cfg.optim_wrapper.optimizer.update(fused=True,foreach=False)
    cfg.env_cfg.cudnn_benchmark=a.cudnn_benchmark
    cfg.base5_bounded_runtime=dict(compile_camera=a.compile_camera,spatial_case=a.spatial_case)
    if a.pixel_io_comparison:
        pipeline=cfg.train_dataloader.dataset.pipeline
        for item in pipeline:
            if item['type']=='MoraiOnlineImageDomainAug3D' and any(item[k] for k in
                    ('jpeg_prob','blur_prob','exposure_prob','noise_prob')):
                raise ValueError('I/O comparison requires original PNG without augmentation or preencoded JPEG')
        cfg.train_dataloader.dataset.pipeline=[t for t in pipeline if t['type']!='MoraiOnlineImageDomainAug3D']
    cfg.default_hooks.checkpoint=None
    cfg.default_hooks.logger.interval=16
    cfg.custom_hooks.append(dict(type='Base5BoundedCost',steps=a.steps,warmup=a.warmup,
        output=str(a.work_dir/'cost.json'),batch=a.batch,audit_gradients=a.audit_gradients))
    if a.profile_steps:
        cfg.custom_hooks.append(dict(type='Base5TrainingProfile',warmup=a.warmup,
            steps=a.profile_steps,output=str(a.work_dir)))
    cfg.dump(str(a.work_dir/'bounded_cost_resolved.py'))
    runner=Base5AtomicRunner.from_cfg(cfg)
    if a.compile_camera:
        # Compile the bound callable, NOT a wrapping module with _orig_mod
        # parameter names. The audited load and checkpoint format stay intact.
        runner.model.img_backbone.forward=torch.compile(runner.model.img_backbone.forward,dynamic=False)
    try: runner.train()
    except PlannedStop:
        result=json.loads((a.work_dir/'cost.json').read_text())
        result.update(profile=a.profile,workers=a.workers,contract_sha256=manifest['contract_sha256'],
            pixel_io_comparison=a.pixel_io_comparison, image_policy=json.loads((a.data/'manifest.json').read_text()).get('image_policy'),
            bounded_config_sha256=hashlib.sha256((a.work_dir/'bounded_cost_resolved.py').read_bytes()).hexdigest(),
            runtime_options=dict(pool_backward=a.pool_backward,compact_indices=a.compact_indices,
                geometry_mode=a.geometry_mode,
                batched_loss_log=a.batched_loss_log,depth_step=a.depth_step,spatial_case=a.spatial_case,
                fused_adamw=a.fused_adamw,cudnn_benchmark=a.cudnn_benchmark,
                compile_camera=a.compile_camera,profile_steps=a.profile_steps,
                audit_gradients=a.audit_gradients),
            full_epoch_training_started=False)
        (a.work_dir/'cost.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='per_step'},indent=2))
    except torch.cuda.OutOfMemoryError:
        (a.work_dir/'cost.json').write_text(json.dumps(dict(valid=False,oom=True,profile=a.profile,batch=a.batch)))
        raise SystemExit(2)
    else: raise RuntimeError('Probe unexpectedly completed an epoch')


if __name__=='__main__': main()
