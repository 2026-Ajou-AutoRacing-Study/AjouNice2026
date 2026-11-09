#!/usr/bin/env python3
"""One isolated E12 runtime ablation case. No ROS, training or production writes."""
import argparse
import gzip
import json
from pathlib import Path
import sys
import time
import subprocess

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT),str(ROOT/'docker/bevfusion'),
               str(ROOT/'perception/detection3d/bevfusion/scripts')]
from qualify_base5_camera_tensorrt import load_inputs, distribution, sha256, config_closure

CASES = ('original','cache','separated','parallel','trt','trt_head')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('inputs','config','checkpoint','sensor','output'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--case',choices=CASES,required=True)
    p.add_argument('--samples',type=int,default=96)
    p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--warmup',type=int,default=16)
    p.add_argument('--engine',type=Path)
    p.add_argument('--metadata',type=Path)
    a=p.parse_args()
    if a.output.exists(): raise ValueError('Fresh output required')
    a.output.mkdir(parents=True)
    samples,contract,pins=load_inputs(a.inputs,a.samples)
    source_paths=config_closure(a.config)|{Path(__file__),a.checkpoint,a.sensor}
    for folder in ('docker/bevfusion/app','perception/detection3d/bevfusion/plugin'):
        source_paths.update(f for f in (ROOT/folder).rglob('*') if f.suffix in ('.py','.cu','.cpp','.h'))
    if a.case in ('trt','trt_head'):
        if not a.engine or not a.metadata: raise ValueError('Local engine/metadata required')
        source_paths.update((a.engine,a.metadata))
    pins.update({str(f):sha256(f) for f in source_paths})
    import cv2
    import torch
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.plugin.morai_pool_optimization import (
        disable_pool_optimization,enable_pool_optimization)
    torch.set_num_threads(4);cv2.setNumThreads(1);torch.manual_seed(261109)
    torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False
    runtime=None
    result=dict(case=a.case,complete=False,checkpoint_sha256=sha256(a.checkpoint),
        config_sha256=sha256(a.config),input_sha256=pins,selected_ids=[s['id'] for s in samples],
        scope='In-memory original JPEG -> prepare -> GPU model -> CPU raw boxes; excludes disk, LAN, ROS, tracker.',
        amp=True,parallel_camera=a.case in ('parallel','trt','trt_head'),
        precision='AMP FP16; TF32 OFF; cudnn benchmark OFF',raw_floor=0.,raw_topk=200,
        unique_inputs=len(samples),repeats=a.repeats,warmup=a.warmup,
        gpu=torch.cuda.get_device_name(),torch_version=str(torch.__version__),
        production_promoted=False,ap_evaluated=False)
    result['nvidia_before']=subprocess.run(['nvidia-smi','-q'],capture_output=True,text=True).stdout
    (a.output/'STARTED.json').write_text(json.dumps(result,indent=2)+'\n')
    try:
        runtime=BevFusionModel(a.config,a.checkpoint,load_sensor4_calibration(a.sensor),
            base5_runtime=True,use_amp=True,parallel_camera=result['parallel_camera'],
            preprocess_workers=4,warmup_iterations=1,
            camera_tensorrt_engine=a.engine if a.case in ('trt','trt_head') else None,
            camera_tensorrt_metadata=a.metadata if a.case in ('trt','trt_head') else None,
            head_attention_sdpa=a.case=='trt_head',head_cache_bev_pos=a.case=='trt_head')
        torch.backends.cudnn.benchmark=False
        torch.backends.cuda.matmul.allow_tf32=False
        torch.backends.cudnn.allow_tf32=False
        state=runtime.model.view_transform._pool_optimization
        state['entries'].clear();state['bytes']=0;state['plan']=None
        if a.case in ('original','cache'):
            disable_pool_optimization(runtime.model.view_transform)
            if a.case=='cache': enable_pool_optimization(runtime.model.view_transform,'cache')
        runtime.warmed_up=True
        with torch.inference_mode():
            for i in range(a.warmup):
                sample=samples[i%len(samples)]
                runtime.infer(sample['points'],sample['images'],i,0.,200)
            torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats()
            times=[];predictions={}
            for repeat in range(a.repeats):
                for i,sample in enumerate(samples):
                    torch.cuda.synchronize();at=time.perf_counter()
                    pred,prepare,model=runtime.infer(sample['points'],sample['images'],i,0.,200)
                    torch.cuda.synchronize()
                    times.append(dict(id=sample['id'],repeat=repeat,service_ms=(time.perf_counter()-at)*1000,
                        preprocess_ms=prepare,model_ms=model,postprocess_ms=runtime.last_runtime_timings['postprocess_ms']))
                    if repeat==0: predictions[sample['id']]=pred
                print(json.dumps(dict(case=a.case,repeat=repeat+1,frames=len(times))),flush=True)
        result.update(complete=True,timings={key:distribution([r[key] for r in times])
            for key in ('service_ms','preprocess_ms','model_ms','postprocess_ms')},
            peak_allocated_bytes=torch.cuda.max_memory_allocated(),
            peak_reserved_bytes=torch.cuda.max_memory_reserved(),raw_times=times,
            head_report=runtime.head_runtime_report)
        pool_state=getattr(runtime.model.view_transform,'_pool_optimization',{})
        result['pool_state']={key:value for key,value in pool_state.items()
                              if isinstance(value,(str,int,float,bool))}
        result['nvidia_after']=subprocess.run(['nvidia-smi','-q'],capture_output=True,text=True).stdout
        if pins!={f:sha256(f) for f in pins}: raise ValueError('Inputs/source/checkpoint changed')
        with gzip.open(a.output/'predictions.json.gz','wt') as f: json.dump(predictions,f,allow_nan=False)
        (a.output/'RESULT.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(case=a.case,complete=True,timing=result['timings'],
            peak_allocated_bytes=result['peak_allocated_bytes'])),flush=True)
    except Exception as error:
        (a.output/'FAILED.json').write_text(json.dumps(dict(case=a.case,error=repr(error),
            complete=False,oom=isinstance(error,torch.cuda.OutOfMemoryError)),indent=2)+'\n')
        raise
    finally:
        if runtime: runtime.close()


if __name__=='__main__':main()
