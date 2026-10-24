#!/usr/bin/env python3
"""Saturated batch-one AMP timing on actual bag samples, independent of LiDAR Hz.

Wide cases are fresh 4-class geometry/cost probes, NOT detection accuracy tests.
No point subsampling, no temporal history, no simulator, no production changes.
"""
import argparse
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


def stats(values):
    return dict(mean=float(np.mean(values)),p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--samples',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--iterations',type=int,default=30)
    p.add_argument('--cases',nargs='+',default=['e25','highres','wide_lowres','wide_highres'])
    p.add_argument('--root',type=Path,default=Path('/workspace/AjouNice2026'));a=p.parse_args()
    if a.iterations<10:raise ValueError('At least 10 measurements per case')
    samples=[]
    for path in sorted(a.samples.glob('sample_*.npz')):
        with np.load(path) as data:
            samples.append((data['points'].copy(),{'image/'+k:data[k].tobytes() for k in data.files if k!='points'}))
    if not samples:raise ValueError('No samples')
    calibration=load_sensor4_calibration(a.root/'configs/sensors/Sensor_competition_local.json')
    folder=a.root/'perception/detection3d/bevfusion/configs'
    checkpoint=a.root/'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth'
    report=dict(gpu=torch.cuda.get_device_name(),batch=1,amp=True,deskew=False,
        points=[len(x[0]) for x in samples],iterations=a.iterations,cases=[],
        contract='Same untouched bag samples; saturated serial batch1. Wide=random four-class cost probe, not accuracy. Forward includes GPU normalization/voxelization/decode; service includes CPU compressed decode/resize/pack, excludes ROS/TCP/publication.')
    torch.backends.cudnn.benchmark=True;torch.backends.cuda.matmul.allow_tf32=True
    for case in a.cases:
        model=None;pre=None;torch.cuda.empty_cache();torch.cuda.reset_peak_memory_stats()
        result=dict(case=case)
        try:
            config_name={'e25':'bevfusion_base3_b0_4cam_lidar.py',
                         'highres':'bevfusion_base5_highres_320x896_pilot.py',
                         'wide_lowres':'bevfusion_base5_independent_100m.py',
                         'wide_highres':'bevfusion_base5_independent_100m.py'}[case]
            cfg=Config.fromfile(folder/config_name);cfg.model.img_backbone.init_cfg=None
            if case=='wide_lowres':
                cfg.model.view_transform.image_size=[256,704];cfg.model.view_transform.feature_size=[32,88]
                for pipeline in (cfg.train_pipeline,cfg.test_pipeline,
                        cfg.test_dataloader.dataset.pipeline,cfg.val_dataloader.dataset.pipeline):
                    for t in pipeline:
                        if t['type']=='MoraiImageAug3D':t['final_dim']=[256,704];t['resize_lim']=[.55,.55]
            torch.manual_seed(261006)
            model=init_model(cfg,str(checkpoint) if case in ('e25','highres') else None,device='cuda:0')
            restore_view_geometry(model.view_transform);model.eval()
            pre=Base3OnlinePreprocessor(calibration,camera_workers=4,point_dim=4,**preprocessing_options_from_config(cfg))
            service=[];forward=[];cpu=[];counts=[];start_run=None
            with torch.inference_mode():
                for i in range(a.iterations+6):
                    if i==6:
                        result['warmup_peak_allocated_mib']=torch.cuda.max_memory_allocated()/2**20
                        torch.cuda.reset_peak_memory_stats();start_run=time.perf_counter()
                    pts,images=samples[i%len(samples)];torch.cuda.synchronize();start=time.perf_counter()
                    batch=pre.prepare(pts,images,i);prepared=time.perf_counter()
                    with torch.autocast('cuda',dtype=torch.float16):prediction=model.test_step(batch)[0]
                    torch.cuda.synchronize();end=time.perf_counter()
                    # Materialize predictions just as the runtime server does.
                    prediction.pred_instances_3d.bboxes_3d.tensor.detach().cpu()
                    prediction.pred_instances_3d.scores_3d.detach().cpu()
                    prediction.pred_instances_3d.labels_3d.detach().cpu()
                    total=time.perf_counter()-start
                    if i>=6:
                        service.append(total*1000);forward.append((end-prepared)*1000);cpu.append((prepared-start)*1000)
                        counts.append(len(batch['inputs']['points'][0]))
                elapsed=time.perf_counter()-start_run
            result.update(valid=True,image_hw=list(pre.final_dim),roi=pre.point_cloud_range.tolist(),
                checkpoint='E25' if case in ('e25','highres') else 'fresh geometry only',
                service_ms=stats(service),forward_ms=stats(forward),cpu_prepare_ms=stats(cpu),
                serial_fps=a.iterations/elapsed,service_mean_inverse_hz=1000/np.mean(service),
                retained_points=sorted(set(counts)),peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20)
        except Exception as exc:
            result.update(valid=False,error=repr(exc),peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20)
        finally:
            if pre:pre.close()
            del model,pre;gc.collect();torch.cuda.empty_cache()
        report['cases'].append(result);a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
