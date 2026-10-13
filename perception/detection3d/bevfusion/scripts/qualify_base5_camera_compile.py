#!/usr/bin/env python3
"""Real-image camera-only eval output/gradient and warmed latency qualification.

Dropout is disabled for the differential math check. This is NOT an AP test,
full-service inference latency or bitwise replay of stochastic training.
"""
import argparse
import json
from pathlib import Path
import time

import torch
from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmengine.runner import Runner,set_random_seed
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import MODELS
from mmdet.models.backbones import SwinTransformer


def timing(fn,images):
    with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
        for _ in range(8):fn(images)
        torch.cuda.synchronize();values=[]
        for _ in range(24):
            start=time.perf_counter();fn(images);torch.cuda.synchronize()
            values.append((time.perf_counter()-start)*1000)
    return dict(mean_ms=sum(values)/len(values),min_ms=min(values),max_ms=max(values))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--config',type=Path,required=True)
    p.add_argument('--init-checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    torch.set_num_threads(4);set_random_seed(261010);init_default_scope('mmdet3d')
    cfg=Config.fromfile(str(a.config));import_modules_from_strings(**cfg.custom_imports)
    loader=cfg.train_dataloader;loader.batch_size=1;loader.num_workers=0;loader.persistent_workers=False
    loader.dataset.data_root=str(a.data.resolve());loader.dataset.data_prefix=dict(pts='',img='')
    loader.dataset.ann_file=str(a.data.resolve()/'infos/512x1056/morai_base5_infos_train.pkl')
    batch=next(iter(Runner.build_dataloader(loader,seed=261010)))
    # BEVFusion consumes voxelize_cfg itself, before building this preprocessor.
    preprocess_cfg=cfg.model.data_preprocessor.copy()
    preprocess_cfg.pop('voxelize_cfg',None)
    preprocess=MODELS.build(preprocess_cfg).cuda()
    normalized=preprocess(batch,training=True)
    images=normalized['inputs']['imgs'];images=images.reshape(-1,*images.shape[2:]).contiguous()
    backbone_cfg=cfg.model.img_backbone.copy()
    if backbone_cfg.pop('type')!='mmdet.SwinTransformer':
        raise ValueError('This differential probe qualifies the current Swin backbone only')
    backbone=SwinTransformer(**backbone_cfg).cuda()
    # This MMDetection Swin overrides train() without returning self.
    backbone.eval()
    checkpoint=torch.load(a.init_checkpoint,map_location='cpu',weights_only=False)
    state=checkpoint.get('state_dict',checkpoint)
    backbone.load_state_dict({k.removeprefix('img_backbone.'):v for k,v in state.items()
                              if k.startswith('img_backbone.')},strict=True)
    del checkpoint,state
    original=backbone.forward;compiled=torch.compile(original,dynamic=False)
    with torch.autocast('cuda',dtype=torch.float16):
        reference=original(images)
    # LayerNorm makes squared feature energy nearly constant: its tiny,
    # cancellation-dominated derivative is not a representative head gradient.
    # Use the SAME fixed upstream sensitivity for both Jacobian-vector products.
    generator=torch.Generator(device=images.device).manual_seed(261011)
    sensitivity=[torch.randn(x.shape,device=x.device,dtype=x.dtype,generator=generator)*.001
                 for x in reference]
    torch.autograd.backward(reference,sensitivity)
    reference=[x.detach().cpu() for x in reference]
    gradients={name:p.grad.detach().cpu().clone() for name,p in backbone.named_parameters() if p.grad is not None}
    backbone.zero_grad(set_to_none=True)
    with torch.autocast('cuda',dtype=torch.float16):
        actual=compiled(images)
    torch.autograd.backward(actual,sensitivity)
    forward=[]
    for x,y in zip(actual,reference):
        x=x.detach().float().cpu();y=y.float()
        forward.append(dict(shape=list(x.shape),max_abs=float((x-y).abs().max()),
                            relative_l2=float((x-y).norm()/y.norm().clamp_min(1e-12))))
    worst_gradient=0.;missing=[];error2=0.;reference2=0.;finite=True
    for name,p in backbone.named_parameters():
        if name not in gradients and p.grad is None:continue
        if name not in gradients or p.grad is None:missing.append(name);continue
        x=p.grad.detach().cpu();y=gradients[name]
        finite=finite and bool(torch.isfinite(x).all())
        error2+=float((x.double()-y.double()).square().sum())
        reference2+=float(y.double().square().sum())
        worst_gradient=max(worst_gradient,float((x-y).norm()/y.norm().clamp_min(1e-12)))
    global_gradient=(error2/max(reference2,1e-24))**.5
    valid=finite and len(actual)==len(reference) and not missing and global_gradient<.003 and all(
        x['relative_l2']<.005 for x in forward)
    backbone.zero_grad(set_to_none=True);del actual,gradients
    timings=dict(eager=timing(original,images),compiled=timing(compiled,images))
    result=dict(valid=valid,scene=str(batch['data_samples'][0].metainfo.get('scene_id','')),
        image_shape=list(images.shape),forward=forward,worst_parameter_gradient_relative_l2=worst_gradient,
        all_parameter_gradient_relative_l2=global_gradient,finite_gradients=finite,
        missing_gradient_names=missing,timing=timings,
        gradient_objective='identical fixed random upstream sensitivities, seed 261011, scale .001',
        scope='camera backbone only; eval/dropout off; AMP output and gradient, not AP or full-service latency')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not valid:raise SystemExit(1)


if __name__=='__main__':main()
