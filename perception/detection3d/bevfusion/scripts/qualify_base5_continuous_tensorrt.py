#!/usr/bin/env python3
"""E12 eager/TRT differential inference on frozen original-ROS bag bytes.

No live ROS, sensors, training, threshold tuning or production replacement.
Uses original JPEG/XYZI255 through the real online preprocessor, with exactly
one intensity /255. A saved chronological detection stream feeds later fresh
tracker replays. Native validation arrays are not accepted as a substitute.
"""
import argparse
import ast
from copy import deepcopy
import gzip
import inspect
import json
from pathlib import Path
import sys
import time
import textwrap
import hashlib

from qualify_base5_camera_tensorrt import (
    ROOT, E12_SHA256, CAMERAS, CLASSES, aggregate_comparisons, compare_detections,
    config_closure, distribution, ros_delivery, sha256, validate_detections, write_new)


def load_ros_inputs(path):
    import numpy as np
    path=Path(path).resolve(strict=True);manifest=json.loads(path.read_text())
    if (manifest.get('schema')!='base5_rosbag_tensorrt_inputs_v1' or
            manifest.get('image_policy')!='original_ROS_compressed_JPEG_bytes' or
            manifest.get('point_intensity_source')!='ros255' or
            manifest.get('model_outputs_inspected_for_selection') is not False):
        raise ValueError('Frozen prediction-independent original ROS byte manifest required')
    rows=manifest['samples']
    if not 3<=len(rows)<=600 or len({r['id'] for r in rows})!=len(rows):
        raise ValueError('Bounded distinct continuous sample identities required')
    pins={str(path):sha256(path)};samples=[];cases={}
    for row in rows:
        if set(row['images'])!={'image/'+name for name in CAMERAS}:raise ValueError('Four ROS cameras required')
        blobs={}
        for name,item in [('points',row['points']),*row['images'].items()]:
            source=Path(item['path'])
            if source.is_absolute() or '..' in source.parts:raise ValueError('Portable relative input paths required')
            source=(path.parent/source).resolve(strict=True)
            source.relative_to(path.parent)
            if sha256(source)!=item['sha256']:raise ValueError('Frozen ROS payload SHA differs')
            pins[str(source)]=item['sha256'];blob=source.read_bytes()
            if name!='points' and blob[:2]!=b'\xff\xd8':raise ValueError('Original compressed JPEG bytes required')
            blobs[name]=blob
        points=np.frombuffer(blobs.pop('points'),dtype='<f4').reshape(-1,4).copy()
        if not len(points) or not np.isfinite(points).all() or np.any(points[:,3]<0) or np.any(points[:,3]>255):
            raise ValueError('Finite original ROS XYZI255 required')
        previous=cases.get(row['group'])
        if previous and (row['raw_lidar_index']!=previous['raw_lidar_index']+1 or
                         not 0<row['stamp']-previous['stamp']<=.2):
            raise ValueError('Do not skip/resample original chronological frames')
        cases[row['group']]=row
        samples.append(dict(row,points=points,images=blobs))
    if len(cases)<3:raise ValueError('At least three independent bag windows required')
    sensor=path.parent/manifest['sensor_config']['path']
    if sha256(sensor)!=manifest['sensor_config']['sha256']:raise ValueError('Original sensor calibration SHA differs')
    pins[str(sensor.resolve())]=sha256(sensor)
    context=path.parent/'context.json';pins[str(context)]=sha256(context)
    return samples,manifest,sensor,pins


def save_gzip_new(path,value):
    with path.open('xb') as raw:
        with gzip.open(raw,'wt') as stream:json.dump(value,stream,allow_nan=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','config','checkpoint','engine','metadata','output'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--warmup',type=int,default=8)
    p.add_argument('--capture-head',action='store_true',help='Save one actual eager head input/metas in a separate untimed call')
    p.add_argument('--comparison',choices=('camera_trt','head_sdpa_position'),default='camera_trt',
        help='Explicit isolated camera comparison or TRT-only vs TRT+SDPA/cache vs TRT-only restore; topk never enabled')
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    if not 1<=a.warmup<=16:p.error('Bounded1..16 warmups required')
    if sha256(a.checkpoint)!=E12_SHA256:p.error('Frozen full-data E12 weights required')
    samples,manifest,sensor,pins=load_ros_inputs(a.inputs)
    sys.path.insert(0,str(ROOT/'docker/bevfusion'))
    from app.camera_tensorrt import CameraTensorRTAdapter,install_camera_tensorrt
    expected=dict(checkpoint_sha256=E12_SHA256,config_sha256=sha256(a.config))
    adapter=CameraTensorRTAdapter(a.engine,a.metadata,expected);engine_identity=deepcopy(adapter.metadata);adapter.close()
    for path in (a.config,a.checkpoint,a.engine,a.metadata):pins[str(path.resolve())]=sha256(path)
    import cv2
    import torch
    from mmengine.config import Config
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    cfg=deepcopy(Config.fromfile(str(a.config),lazy_import=False))
    if (tuple(cfg.metainfo.classes)!=CLASSES or tuple(cfg.model.view_transform.image_size)!=(512,1056) or
            list(cfg.model.view_transform.dbound)!=[1.,100.,.25]):raise ValueError('Preserve current E12 geometry')
    cfg.model.compile_camera=False;cfg.model.img_backbone.init_cfg=None
    a.output.mkdir(parents=True);resolved=a.output/'resolved_config.py';cfg.dump(str(resolved))
    sources=config_closure(a.config)|{Path(__file__).resolve(),Path(__file__).with_name('qualify_base5_camera_tensorrt.py'),resolved.resolve()}
    for folder in ('docker/bevfusion/app','perception/detection3d/bevfusion/plugin',
                   'third_party/mmdetection3d/projects/BEVFusion'):
        sources.update(path for path in (ROOT/folder).rglob('*') if path.suffix in ('.py','.cu','.cpp','.h'))
    source_pins={str(path):sha256(path) for path in sorted(sources)}
    result=dict(schema='base5_continuous_ros_tensorrt_qualification_v1',complete=False,started_unix=time.time(),
        input_sha256=pins,source_sha256=source_pins,checkpoint_sha256=E12_SHA256,
        frames=len(samples),cases=manifest['cases'],engine_identity=engine_identity,
        contract=dict(amp=True,parallel=True,compile=False,tf32=True,cudnn_benchmark=True,
            point_intensity_source='ros255',point_scaling='Existing online preprocessor divides once by255',
            raw='<=200 decoded native ROI outputs; floor0, no ROS NMS',
            delivered='floor.1 then top100 then class-aware rotated NMS.5 then V/P/O.4/.1/.1',
            camera_calibration_sha256=sha256(sensor),JPEG_reencoded=False,GT_used_for_prediction_filter=False),
        scope='Original recorded ROS bytes through actual online preprocessing. Not live ROS/TCP/queue/LAN or physical24GB full-stack latency.',
        comparison=a.comparison,production_changed=False,cases_results=[])
    write_new(a.output/'STARTED.json',result)
    torch.set_num_threads(4);cv2.setNumThreads(1)
    runtime=trt_handle=head_handle=None;baseline=None
    try:
        runtime=BevFusionModel(resolved,a.checkpoint,load_sensor4_calibration(sensor,list(CAMERAS)),
            use_amp=True,parallel_camera=True,base5_runtime=True,point_intensity_source='ros255',warmup_iterations=2)
        torch.backends.cuda.matmul.allow_tf32=True;torch.backends.cudnn.allow_tf32=True;torch.backends.cudnn.benchmark=True
        result.update(gpu=torch.cuda.get_device_name(),torch_version=str(torch.__version__),torch_cuda=torch.version.cuda)
        head=runtime.model.bbox_head;forward=getattr(head.forward_single,'__func__',head.forward_single)
        head_source=Path(inspect.getsourcefile(forward)).resolve(strict=True)
        source_pins[str(head_source)]=sha256(head_source)
        forward_ast=ast.parse(textwrap.dedent(inspect.getsource(forward))).body[0]
        result['actual_head_source']=dict(class_name=type(head).__name__,class_module=type(head).__module__,
            forward_qualname=forward.__qualname__,forward_module=forward.__module__,file=str(head_source),
            file_sha256=sha256(head_source),forward_AST_sha256=hashlib.sha256(ast.dump(forward_ast,include_attributes=False).encode()).hexdigest())
        write_new(a.output/'LOADED_HEAD_SOURCE.json',result['actual_head_source'])
        modes=('eager','camera_trt','eager_repeat') if a.comparison=='camera_trt' else (
            'camera_trt','camera_trt_sdpa_position','camera_trt_restore')
        if a.comparison=='head_sdpa_position':
            trt_handle=install_camera_tensorrt(runtime.model,a.engine,a.metadata,expected)
            from app.head_runtime_optimization import install_head_runtime_optimization
        result['mode_contracts']={mode:dict(camera_trt=mode!='eager' and mode!='eager_repeat',
            head_sdpa=mode=='camera_trt_sdpa_position',head_position_cache=mode=='camera_trt_sdpa_position',
            head_topk=False) for mode in modes}
        for mode in modes:
            if a.comparison=='camera_trt' and mode=='camera_trt':
                trt_handle=install_camera_tensorrt(runtime.model,a.engine,a.metadata,expected)
            if mode=='eager_repeat':trt_handle.close();trt_handle=None
            if mode=='camera_trt_sdpa_position':
                head_handle=install_head_runtime_optimization(runtime.model.bbox_head,
                    proposal_topk=False,attention_sdpa=True,cache_bev_pos=True)
            if mode=='camera_trt_restore':head_handle.close();head_handle=None
            for i in range(a.warmup):
                sample=samples[(i*len(samples))//a.warmup]
                runtime.infer(sample['points'],sample['images'],i,0.,200,timestamp_sec=sample['stamp'])
            if mode=='eager' and a.capture_head:
                captured=[]
                def cpu_copy(value):
                    if torch.is_tensor(value):return value.detach().cpu().clone()
                    if isinstance(value,dict):return {key:cpu_copy(item) for key,item in value.items()}
                    if isinstance(value,list):return [cpu_copy(item) for item in value]
                    if isinstance(value,tuple):return tuple(cpu_copy(item) for item in value)
                    return deepcopy(value)
                hook=runtime.model.bbox_head.register_forward_pre_hook(
                    lambda module,args:captured.append(dict(features=cpu_copy(args[0]),metas=cpu_copy(args[1]))))
                sample=samples[0]
                try:runtime.infer(sample['points'],sample['images'],0,0.,200,timestamp_sec=sample['stamp'])
                finally:hook.remove()
                if len(captured)!=1:raise ValueError('Exactly one actual head entry required')
                head_path=a.output/'head_activation.pt'
                with head_path.open('xb') as stream:torch.save(dict(schema='base5_head_activation_v1',
                    **captured[0],metadata=dict(sample_id=sample['id'],checkpoint_sha256=E12_SHA256,
                        config_sha256=sha256(a.config),source_sha256=source_pins,amp=True,parallel=True,
                        tf32=True,scope='Actual original-ROS-frame eager head input. Untimed diagnostic; no synthetic feature.')),stream)
                result['head_capture']=dict(path=str(head_path),sha256=sha256(head_path),sample_id=sample['id'],
                    features=[dict(shape=list(value.shape),dtype=str(value.dtype)) for value in
                        (captured[0]['features'] if isinstance(captured[0]['features'],(tuple,list)) else [captured[0]['features']])],
                    trusted_pickle_note='Locally generated trusted metadata includes bbox type class; consumers may need weights_only=False')
                del captured
            torch.cuda.synchronize();torch.cuda.reset_peak_memory_stats()
            predictions={};times=[];frames=[]
            for i,sample in enumerate(samples):
                begin=time.perf_counter()
                prediction,prepare_ms,model_ms=runtime.infer(sample['points'],sample['images'],i,0.,200,timestamp_sec=sample['stamp'])
                elapsed=(time.perf_counter()-begin)*1000
                validate_detections(prediction);predictions[sample['id']]=prediction
                delivered=ros_delivery(prediction,True)
                frames.append(dict(token=sample['id'],case=sample['group'],stamp=sample['stamp'],detections=delivered))
                times.append(dict(token=sample['id'],case=sample['group'],service_ms=elapsed,
                    preprocess_ms=prepare_ms,model_ms=model_ms,postprocess_ms=runtime.last_runtime_timings['postprocess_ms']))
                if (i+1)%100==0:print(json.dumps(dict(event='continuous_trt_progress',mode=mode,frames=i+1,total=len(samples))),flush=True)
            if baseline is None:baseline=predictions
            policies=dict(raw=lambda x:x,ros_ap_candidates=lambda x:ros_delivery(x,False),delivered=lambda x:ros_delivery(x,True))
            comparisons={}
            for name,policy in policies.items():
                checks=[dict(id=s['id'],**compare_detections(policy(baseline[s['id']]),policy(predictions[s['id']]))) for s in samples]
                comparisons[name]=dict(pooled=aggregate_comparisons(checks),per_frame=checks)
            save_gzip_new(a.output/(mode+'_raw_predictions.json.gz'),dict(schema=result['schema'],mode=mode,
                input_manifest_sha256=sha256(a.inputs),checkpoint_sha256=E12_SHA256,
                frames=[dict(token=s['id'],case=s['group'],stamp=s['stamp'],detections=predictions[s['id']]) for s in samples]))
            save_gzip_new(a.output/(mode+'_delivered.json.gz'),dict(schema=result['schema'],mode=mode,
                input_manifest_sha256=sha256(a.inputs),checkpoint_sha256=E12_SHA256,frames=frames,
                box_origin='lidar_bottom_v1',thresholds=[.4,.1,.1],nms=.5))
            timing={key:distribution([r[key] for r in times]) for key in ('service_ms','preprocess_ms','model_ms','postprocess_ms')}
            case=dict(mode=mode,timing=timing,per_case_timing={name:{key:distribution([r[key] for r in times if r['case']==name])
                for key in timing} for name in manifest['cases']},timing_rows=times,
                memory=dict(peak_pytorch_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                    peak_pytorch_reserved_mib=torch.cuda.max_memory_reserved()/2**20,device_free_mib=torch.cuda.mem_get_info()[0]/2**20,
                    note='TensorRT external allocations excluded from PyTorch allocator; devicefree also reflects any other processes'),
                delivered_objects=sum(len(f['detections']) for f in frames),comparisons=comparisons,
                reference_mode=modes[0],exact_raw_prediction_values_equal_to_reference=predictions==baseline,
                delivered_sha256=sha256(a.output/(mode+'_delivered.json.gz')))
            if a.comparison=='camera_trt':case['exact_raw_prediction_values_equal_to_eager']=predictions==baseline
            result['cases_results'].append(case);write_new(a.output/(mode+'_RESULT.json'),case)
            write_new(a.output/(mode+'_SUMMARY.json'),{key:value for key,value in case.items()
                if key not in ('timing_rows','comparisons')} | dict(
                    comparisons={name:value['pooled'] for name,value in comparisons.items()}))
            print(json.dumps(dict(event='continuous_trt_case_complete',mode=mode,timing=timing,delivered_objects=case['delivered_objects'])),flush=True)
        if pins!={path:sha256(path) for path in pins} or source_pins!={path:sha256(path) for path in source_pins}:
            raise RuntimeError('Original input, weights, engine or source changed during qualification')
        result.update(complete=True,finished_unix=time.time(),source_and_inputs_unchanged=True,
            numerical_exactness_is_not_automatic_acceptance=True,tracker_evaluated_here=False,
            note='Follow saved chronological detections with fresh actual tracker replay; AP/numerical parity alone does not certify temporal identity.')
        write_new(a.output/'report.json',result)
        write_new(a.output/'summary.json',{key:value for key,value in result.items()
            if key not in ('input_sha256','source_sha256','engine_identity','cases_results')} | dict(
                source_pin_count=len(source_pins),input_pin_count=len(pins),
                report_sha256=sha256(a.output/'report.json'),
                results=[json.loads((a.output/(case['mode']+'_SUMMARY.json')).read_text()) for case in result['cases_results']]))
        write_new(a.output/'COMPLETE.json',dict(complete=True,report_sha256=sha256(a.output/'report.json'),
            source_and_inputs_unchanged=True,production_changed=False))
    except Exception as error:
        write_new(a.output/'FAILED.json',dict(complete=False,error=repr(error),production_changed=False))
        raise
    finally:
        if head_handle is not None:head_handle.close()
        if trt_handle is not None:trt_handle.close()
        if runtime is not None:
            if runtime.inference_optimization:runtime.inference_optimization.disable()
            runtime.preprocessor.close()


if __name__=='__main__':main()
