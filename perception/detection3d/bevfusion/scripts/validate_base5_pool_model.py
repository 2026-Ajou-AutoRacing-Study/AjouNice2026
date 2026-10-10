#!/usr/bin/env python3
"""Real-frame equivalence + bounded real-GT optimizer smoke, not AP/training.

No checkpoints are loaded/saved. This does not establish convergence or final
initialization. Each shape/mode starts with identical freshly initialized weights.
"""
import argparse
from copy import deepcopy
import gc
import json
from pathlib import Path
import pickle
import time

import numpy as np
import torch
from mmengine.structures import InstanceData
from mmdet3d.apis import init_model
from mmdet3d.structures import LiDARInstance3DBoxes
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from app.view_geometry import restore_view_geometry
from benchmark_base5_input_optimization import config_for,load_samples,snapshot_prediction
from perception.detection3d.bevfusion.plugin.morai_pool_optimization import (
    enable_pool_optimization,disable_pool_optimization,extension)


def training_batch(pre,samples,infos,offset,size):
    result=dict(inputs=dict(points=[],img=[]),data_samples=[])
    for j in range(size):
        sample=samples[(offset+j)%len(samples)]
        item=infos[sample['meta']['id']]
        batch=pre.prepare(sample['points'],sample['images'])
        inst=item['instances']
        boxes=torch.tensor([v['bbox_3d'] for v in inst],dtype=torch.float32).reshape(-1,7)
        labels=torch.tensor([v['bbox_label_3d'] for v in inst],dtype=torch.long)
        batch['data_samples'][0].gt_instances_3d=InstanceData(
            bboxes_3d=LiDARInstance3DBoxes(boxes,box_dim=7,origin=(.5,.5,.5)),labels_3d=labels)
        for key in result['inputs']:result['inputs'][key].extend(batch['inputs'][key])
        result['data_samples'].extend(batch['data_samples'])
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,default=Path('/workspace/AjouNice2026'))
    p.add_argument('--samples',type=Path,required=True);p.add_argument('--infos',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--shapes',nargs='+',default=['448x896','512x1056'])
    p.add_argument('--steps',type=int,default=3);p.add_argument('--batches',nargs='+',type=int,default=[1,2])
    p.add_argument('--skip-training',action='store_true');a=p.parse_args()
    if a.output.exists():raise ValueError('Use a new result file')
    torch.set_num_threads(4);torch.backends.cudnn.benchmark=True
    torch.backends.cuda.matmul.allow_tf32=True;extension()
    manifest,samples=load_samples(a.samples)
    infos={}
    for file in a.infos.glob('*infos_*.pkl'):
        for item in pickle.loads(file.read_bytes())['data_list']:
            infos[item['scene_id']]=item
    missing=[v['meta']['id'] for v in samples if v['meta']['id'] not in infos]
    if missing:raise ValueError('Missing GT: '+str(missing))
    calibration=load_sensor4_calibration(a.root/'configs/sensors/Sensor.json')
    result=dict(gpu=torch.cuda.get_device_name(),source=manifest,cost_only=True,
                equivalence=[],training=[],contract='Fresh 4-class real-GT wiring only; no convergence/AP; no checkpoints saved.')
    def save():
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2)+'\n')
    for shape in a.shapes:
        cfg=config_for(a.root,tuple(map(int,shape.split('x'))))
        torch.manual_seed(261010);model=init_model(deepcopy(cfg),None,device='cuda:0')
        restore_view_geometry(model.view_transform);model.eval()
        pre=Base3OnlinePreprocessor(calibration,camera_workers=4,**preprocessing_options_from_config(cfg))
        values=[];hook=model.view_transform.register_forward_hook(lambda m,i,o:values.append(o.detach().cpu()))
        reference=[]
        with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
            for s in samples:
                pred=snapshot_prediction(model.test_step(pre.prepare(s['points'],s['images']))[0])
                reference.append((values.pop(),pred))
        for mode in ('cache','separated'):
            state=enable_pool_optimization(model.view_transform,mode)
            errors=[]
            with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
                for s,(view,expected) in zip(samples,reference):
                    pred=snapshot_prediction(model.test_step(pre.prepare(s['points'],s['images']))[0])
                    actual=values.pop();torch.testing.assert_close(actual,view,atol=3e-5,rtol=3e-5)
                    for v,e in zip(pred,expected):torch.testing.assert_close(v,e,atol=1e-3,rtol=1e-3)
                    errors.append(float((actual-view).abs().max()))
            result['equivalence'].append(dict(shape=shape,mode=mode,samples=len(samples),
                view_max_abs_error=max(errors),predictions_close=True,passed=True))
            disable_pool_optimization(model.view_transform);save()
        hook.remove();pre.close();del model,pre,reference,values,hook,state
        gc.collect();torch.cuda.empty_cache()
        if a.skip_training:continue
        for mode in ('baseline','separated'):
            for size in a.batches:
                torch.manual_seed(261010);model=init_model(deepcopy(cfg),None,device='cuda:0')
                restore_view_geometry(model.view_transform);model.train()
                pre=Base3OnlinePreprocessor(calibration,camera_workers=4,**preprocessing_options_from_config(cfg))
                state=enable_pool_optimization(model.view_transform,mode) if mode!='baseline' else None
                opt=torch.optim.AdamW(model.parameters(),lr=1e-4)
                # Match the existing Base3/Base5 MMEngine AMP contract, not
                # PyTorch's unrelated default 65536 (which overflows here).
                init_scale=float(cfg.optim_wrapper.loss_scale['init_scale'])
                scaler=torch.amp.GradScaler('cuda',init_scale=init_scale)
                case=dict(shape=shape,mode=mode,batch=size,steps=[],initial_loss_scale=init_scale)
                torch.cuda.reset_peak_memory_stats()
                before_step=model.view_transform.depthnet[-1].weight.detach().clone()
                try:
                    for step in range(a.steps):
                        opt.zero_grad(set_to_none=True);torch.cuda.synchronize();start=time.perf_counter()
                        data=model.data_preprocessor(training_batch(pre,samples,infos,step*size,size),training=True)
                        with torch.autocast('cuda',dtype=torch.float16):
                            losses=model._run_forward(data,mode='loss')
                            loss,_=model.parse_losses(losses)
                        if not torch.isfinite(loss):raise AssertionError('Nonfinite loss')
                        scaler.scale(loss).backward();scaler.unscale_(opt)
                        gradients=[v.grad for v in model.view_transform.depthnet.parameters() if v.grad is not None]
                        if not gradients or not all(torch.isfinite(v).all() for v in gradients):
                            raise AssertionError('Missing or nonfinite camera-depth gradients: '+str([
                                (n,v.grad is not None,bool(torch.isfinite(v.grad).all()) if v.grad is not None else None)
                                for n,v in model.view_transform.depthnet.named_parameters()]))
                        torch.nn.utils.clip_grad_norm_(model.parameters(),35.)
                        scale_before=scaler.get_scale();scaler.step(opt);scaler.update();torch.cuda.synchronize()
                        case['steps'].append(dict(loss=float(loss),elapsed_ms=(time.perf_counter()-start)*1000,
                            scale_before=scale_before,scale_after=scaler.get_scale(),depth_gradient_finite=True,
                            optimizer_step_not_skipped=scaler.get_scale()>=scale_before))
                        if step==0:
                            case['warmup_peak_allocated_mib']=torch.cuda.max_memory_allocated()/2**20
                            torch.cuda.reset_peak_memory_stats()
                        del data,loss,losses,gradients
                    if not all(v['optimizer_step_not_skipped'] for v in case['steps']):
                        raise AssertionError('AMP skipped an optimizer step')
                    case['passed']=True
                    case['depth_weights_changed']=not torch.equal(before_step,model.view_transform.depthnet[-1].weight)
                    if not case['depth_weights_changed']:raise AssertionError('Depth weights did not update')
                except Exception as exc:
                    case.update(passed=False,error=repr(exc),oom=isinstance(exc,torch.cuda.OutOfMemoryError))
                case['steady_peak_allocated_mib']=torch.cuda.max_memory_allocated()/2**20
                if state:case['cache_bypasses']=state['bypasses']
                result['training'].append(case);save();print(json.dumps(case),flush=True)
                pre.close()
                if state:disable_pool_optimization(model.view_transform)
                del model,pre,opt,scaler,state,before_step;gc.collect();torch.cuda.empty_cache()
    save()
    if any(not v['passed'] and not v.get('oom') for v in result['training']):raise SystemExit(1)


if __name__=='__main__':main()
