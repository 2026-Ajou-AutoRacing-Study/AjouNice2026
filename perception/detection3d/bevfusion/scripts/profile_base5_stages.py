#!/usr/bin/env python3
"""Batch1 CUDA-event stage spans on unchanged real bag samples.

Events do not synchronize individual stages. Nested view sub-stages must not
be added to their parent total. Stream spans include host-induced idle gaps;
these are critical-path spans, not CUDA kernel self-time or FLOP estimates.
Fine-grid variant is an untrained cost control, not a production proposal.
"""
import argparse
from copy import deepcopy
import gc
import json
from pathlib import Path
import time
import numpy as np
import torch
from mmengine.config import Config
from mmdet3d.apis import init_model
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from app.view_geometry import restore_view_geometry
from benchmark_base5_capacity import stats


def configuration(folder,case):
    name={'e25':'bevfusion_base3_b0_4cam_lidar.py','highres':'bevfusion_base5_highres_320x896_pilot.py',
          'wide_lowres':'bevfusion_base5_independent_100m.py','wide_highres':'bevfusion_base5_independent_100m.py',
          'wide_finegrid':'bevfusion_base5_independent_100m.py',
          'wide_highres_half_depth':'bevfusion_base5_independent_100m.py',
          'wide_dense_highres':'bevfusion_base5_independent_100m.py'}[case]
    cfg=Config.fromfile(folder/name);cfg.model.img_backbone.init_cfg=None
    if case in ('wide_lowres','wide_finegrid'):
        cfg.model.view_transform.image_size=[256,704];cfg.model.view_transform.feature_size=[32,88]
        for pipeline in (cfg.train_pipeline,cfg.test_pipeline,cfg.test_dataloader.dataset.pipeline,cfg.val_dataloader.dataset.pipeline):
            for item in pipeline:
                if item['type']=='MoraiImageAug3D':item['final_dim']=[256,704];item['resize_lim']=[.55,.55]
    if case in ('wide_finegrid','wide_dense_highres'):
        # Same semantic +/-100x30 ROI/depth/images/classes as wide_lowres.
        # Pad voxel extent to stride16 while restoring legacy metric spacing.
        vox=[.075,.075,.2];grid=[2688,864,41];origin=[-100.8,-32.4]
        cfg.model.data_preprocessor.voxelize_cfg.voxel_size=vox
        cfg.model.data_preprocessor.voxelize_cfg.point_cloud_range=[*origin,-5.,100.8,32.4,3.]
        cfg.model.pts_middle_encoder.sparse_shape=[864,2688,41]
        cfg.model.view_transform.xbound=[-100.8,100.8,.3]
        cfg.model.view_transform.ybound=[-32.4,32.4,.3]
        head=cfg.model.bbox_head
        head.train_cfg.update(grid_size=grid,voxel_size=vox,point_cloud_range=[*origin,-3.,100.8,32.4,2.])
        head.test_cfg.update(grid_size=grid,voxel_size=vox[:2],pc_range=origin)
        head.bbox_coder.update(voxel_size=vox[:2],pc_range=origin)
    if case in ('wide_highres_half_depth','wide_dense_highres'):
        cfg.model.view_transform.dbound=[1.,100.,.5]
    return cfg


class StageRecorder:
    def __init__(self):self.enabled=False;self.records=[];self.shapes={};self.voxels=[]
    def wrap(self,obj,method,name,shape=False):
        old=getattr(obj,method)
        def timed(*args,**kwargs):
            if not self.enabled:return old(*args,**kwargs)
            start=torch.cuda.Event(enable_timing=True);end=torch.cuda.Event(enable_timing=True)
            wall=time.perf_counter();start.record();value=old(*args,**kwargs);end.record()
            self.records.append((name,start,end,(time.perf_counter()-wall)*1000))
            if shape:
                items=value if isinstance(value,(tuple,list)) else [value]
                self.shapes[name]=[list(x.shape) for x in items if torch.is_tensor(x)]
            if name=='lidar_voxelize':self.voxels.append(int(value[0].shape[0]))
            return value
        setattr(obj,method,timed)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--iterations',type=int,default=30)
    p.add_argument('--cases',nargs='+',default=['e25','highres','wide_lowres','wide_highres','wide_finegrid'])
    p.add_argument('--root',type=Path,default=Path('/workspace/AjouNice2026'));a=p.parse_args()
    samples=[]
    for file in sorted(a.samples.glob('sample_*.npz')):
        with np.load(file) as data:samples.append((data['points'].copy(),{'image/'+k:data[k].tobytes() for k in data.files if k!='points'}))
    if not samples or a.iterations<10:raise ValueError('Samples and >=10 iterations required')
    calibration=load_sensor4_calibration(a.root/'configs/sensors/Sensor.json')
    checkpoint=a.root/'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth'
    report=dict(gpu=torch.cuda.get_device_name(),iterations=a.iterations,cases=[],
        contract='CUDA-event critical-path spans incl host idle; parent/substage overlap; hooks perturb timing. Same raw bag samples. Wide variants fresh/untrained.')
    torch.backends.cudnn.benchmark=True;torch.backends.cuda.matmul.allow_tf32=True
    for case in a.cases:
        model=None;pre=None;rec=None;result=dict(case=case)
        try:
            cfg=configuration(a.root/'perception/detection3d/bevfusion/configs',case);torch.manual_seed(261006)
            # BEVFusion constructor pops voxelize_cfg; preserve audit config.
            model=init_model(deepcopy(cfg),str(checkpoint) if case in ('e25','highres') else None,device='cuda:0')
            restore_view_geometry(model.view_transform);model.eval()
            pre=Base3OnlinePreprocessor(calibration,camera_workers=4,point_dim=4,**preprocessing_options_from_config(cfg))
            rec=StageRecorder()
            for name in ('data_preprocessor','img_backbone','img_neck','pts_middle_encoder','fusion_layer','pts_backbone','pts_neck'):
                rec.wrap(getattr(model,name),'forward',name,shape=True)
            rec.wrap(model,'voxelize','lidar_voxelize',shape=True)
            rec.wrap(model.view_transform,'forward','view_transform_total',shape=True)
            for method in ('get_geometry','get_cam_feats','bev_pool'):
                rec.wrap(model.view_transform,method,'view_'+method)
            rec.wrap(model.view_transform.downsample,'forward','view_downsample',shape=True)
            rec.wrap(model.bbox_head,'predict','head_predict',shape=False)
            spans={};host={};total=[];cpu=[];residual=[]
            with torch.inference_mode():
                for i in range(a.iterations+6):
                    rec.enabled=i>=6;rec.records=[]
                    pts,images=samples[i%len(samples)];t0=time.perf_counter();batch=pre.prepare(pts,images,i)
                    cpu_elapsed=(time.perf_counter()-t0)*1000;torch.cuda.synchronize();start=time.perf_counter()
                    with torch.autocast('cuda',dtype=torch.float16):model.test_step(batch)
                    torch.cuda.synchronize();elapsed=(time.perf_counter()-start)*1000
                    if rec.enabled:
                        total.append(elapsed);cpu.append(cpu_elapsed);top_sum=0.
                        for name,begin,end,submitted in rec.records:
                            ms=begin.elapsed_time(end);spans.setdefault(name,[]).append(ms);host.setdefault(name,[]).append(submitted)
                            if name!='view_downsample' and not name.startswith('view_get_') and name!='view_bev_pool':top_sum+=ms
                        residual.append(elapsed-top_sum)
            vt=model.view_transform
            result.update(valid=True,forward_wall_ms=stats(total),cpu_prepare_ms=stats(cpu),
                stage_stream_ms={k:stats(v) for k,v in spans.items()},stage_host_call_ms={k:stats(v) for k,v in host.items()},
                outside_instrumented_spans_wall_ms=stats(residual),feature_shapes=rec.shapes,
                occupied_voxels=sorted(set(rec.voxels)),
                image_hw=list(pre.final_dim),frustum_shape=list(vt.frustum.shape),view_nx=vt.nx.detach().cpu().tolist(),
                voxel_size=cfg.model.data_preprocessor.voxelize_cfg.voxel_size,
                sparse_shape=cfg.model.pts_middle_encoder.sparse_shape,
                classes=list(cfg.classes),retained_points=[len(pre._filter_points(s[0])) for s in samples])
        except Exception as exc:result.update(valid=False,error=repr(exc))
        finally:
            if pre:pre.close()
            # Wrappers form reference cycles with modules; collect before next case.
            del model,pre,rec;gc.collect();torch.cuda.empty_cache()
        report['cases'].append(result);a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
