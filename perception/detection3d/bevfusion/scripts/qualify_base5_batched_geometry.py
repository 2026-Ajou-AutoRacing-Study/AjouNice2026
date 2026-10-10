#!/usr/bin/env python3
"""Reject a faster geometry layout if real augmented BEV cell indices change."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmengine.runner import Runner, set_random_seed
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import MODELS
from perception.detection3d.bevfusion.tools.base5_grid_candidates import candidate_config,SPATIAL_CASES
from perception.detection3d.bevfusion.plugin.morai_pool_optimization import (
    batched_geometry,build_plan,SeparatedPool)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=32)
    p.add_argument('--depth-step',type=float,choices=[.25,.5],default=.5)
    p.add_argument('--repair-grid-boundaries',action='store_true')
    p.add_argument('--spatial-case',choices=SPATIAL_CASES)
    a=p.parse_args()
    if a.output.exists() or not 2<=a.samples<=64:p.error('Fresh output, 2..64 samples')
    torch.set_num_threads(4);set_random_seed(261010)
    cfg=Config.fromfile(str(a.config));import_modules_from_strings(**cfg.custom_imports)
    if a.spatial_case:cfg=candidate_config(cfg,a.spatial_case,depth_step=a.depth_step)
    init_default_scope('mmdet3d')
    loader=cfg.train_dataloader
    loader.batch_size=2;loader.num_workers=0;loader.persistent_workers=False
    loader.dataset.data_root=str(a.data.resolve());loader.dataset.data_prefix=dict(pts='',img='')
    loader.dataset.ann_file=str(a.data.resolve()/'infos/512x1056/morai_base5_infos_train.pkl')
    cfg.model.view_transform.dbound=[1.,100.,a.depth_step]
    view=MODELS.build(cfg.model.view_transform).cuda().eval()
    original=view._pool_optimization['originals']['get_geometry']
    rows=[];first=None
    with torch.no_grad():
        for batch in Runner.build_dataloader(loader,seed=261010):
            metas=[sample.metainfo for sample in batch['data_samples']]
            values={key:torch.tensor(np.asarray([m[key] for m in metas]),device='cuda',dtype=torch.float32)
                    for key in ('cam2lidar','cam2img','img_aug_matrix','lidar_aug_matrix')}
            c,k,i,l=(values[key] for key in values)
            args=(c[...,:3,:3],c[...,:3,3],k[...,:3,:3],i[...,:3,:3],i[...,:3,3])
            kwargs=dict(extra_rots=l[...,:3,:3],extra_trans=l[...,:3,3])
            baseline=original(*args,**kwargs);candidate=batched_geometry(
                view,*args,repair_grid_boundaries=a.repair_grid_boundaries,**kwargs)
            xyz_a=((baseline-(view.bx-view.dx/2))/view.dx).long()
            xyz_b=((candidate-(view.bx-view.dx/2))/view.dx).long()
            mismatch=(xyz_a!=xyz_b).any(dim=-1)
            valid_a=((xyz_a>=0)&(xyz_a<view.nx)).all(dim=-1)
            valid_b=((xyz_b>=0)&(xyz_b<view.nx)).all(dim=-1)
            rows.append(dict(samples=[str(m.get('scene_id',m.get('sample_idx',''))) for m in metas],
                max_xyz_error=float((baseline-candidate).abs().max()),
                all_cell_mismatches=int(mismatch.sum()),
                relevant_cell_mismatches=int((mismatch&(valid_a|valid_b)).sum()),
                boundary_rays_recomputed=view._pool_optimization.get('boundary_rays_recomputed',0),
                frustum_samples=mismatch.numel()))
            if first is None:first=(args,kwargs)
            if len(rows)*2>=a.samples:break
            del baseline,candidate,xyz_a,xyz_b,mismatch
        args,kwargs=first;timings={}
        for label,fn in [('original',lambda:original(*args,**kwargs)),
                         ('batched',lambda:batched_geometry(view,*args,
                             repair_grid_boundaries=a.repair_grid_boundaries,**kwargs))]:
            for _ in range(8):fn()
            torch.cuda.synchronize();samples=[]
            for _ in range(16):
                start,end=torch.cuda.Event(enable_timing=True),torch.cuda.Event(enable_timing=True)
                start.record();fn();end.record();end.synchronize();samples.append(start.elapsed_time(end))
            timings[label]=dict(mean_ms=float(np.mean(samples)),p95_ms=float(np.percentile(samples,95)))
    accepted=all(row['relevant_cell_mismatches']==0 for row in rows)
    del baseline,candidate,xyz_a,xyz_b,mismatch
    # Real full-size frustum/depth plan, not just tiny channel-tail fixtures.
    plan=build_plan(view,original(*args,**kwargs),compact_indices=True)
    B,N,D,H,W=plan['input_shape'];channels=view.C
    depth=torch.rand(B,N,D,H,W,device='cuda',requires_grad=True)
    feat=torch.randn(B,N,H,W,channels,device='cuda',requires_grad=True)
    ref=SeparatedPool.apply(depth,feat,plan,'legacy')
    gradient=torch.randn_like(ref)
    expected=torch.autograd.grad(ref,(depth,feat),gradient);gradient_evidence={}
    for mode in ('warp','grouped'):
        actual=SeparatedPool.apply(depth,feat,plan,mode)
        torch.testing.assert_close(actual,ref,rtol=0,atol=0)
        observed=torch.autograd.grad(actual,(depth,feat),gradient)
        errors=[]
        for x,y in zip(observed,expected):
            torch.testing.assert_close(x,y,rtol=1e-4,atol=1e-4)
            errors.append(dict(max_abs_error=float((x-y).abs().max()),
                               relative_l2=float((x-y).norm()/y.norm().clamp_min(1e-12))))
        gradient_evidence[mode]=dict(valid=True,errors=errors,valid_samples=plan['rd'].numel())
    result=dict(candidate_accepted=accepted,timings=timings,rows=rows,depth_step=a.depth_step,
                repaired_grid_boundaries=a.repair_grid_boundaries,
                spatial_case=a.spatial_case,
                pool_full_frustum_gradient=gradient_evidence,
                scope='real training augmentations, geometry only; no default change or AP claim')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
