#!/usr/bin/env python3
"""Actual native-JPEG inputs: spatial/depth COST probes, never learned AP.

Run on one GPU without simulator/training competition. Batch 1 AMP, serial
service latency, warmed model, CPU prediction transfer included. Separate
instrumented stage spans from uninstrumented latency. Fresh output required.
"""
import argparse
from copy import deepcopy
import gc
import hashlib
import json
from pathlib import Path
import pickle
import sys
import time

import cv2
import numpy as np
import torch
from mmengine.config import Config
from mmdet3d.apis import init_model
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from app.view_geometry import restore_view_geometry
from profile_base5_stages import StageRecorder
from benchmark_base5_input_optimization import named_model_parameters, snapshot_prediction, summary

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from base5_grid_candidates import SPATIAL_CASES, candidate_config, geometry
from prepare_base5_portable_dataset import sha256, validate


def samples_from_selection(data, selection):
    manifest=validate(data)
    if manifest.get('box_geometry_policy')!='native_rpy_fixed_yaw_enclosure_v1':
        raise ValueError('Use current upright native-RPY data')
    if manifest.get('image_policy')!=dict.fromkeys(('train','val','test'),'native_jpeg90'):
        raise ValueError('Native JPEG in every split required')
    source=json.loads(selection.read_text())
    rows={}
    for split in ('train','val'):
        info=pickle.loads((data/f'infos/512x1056/morai_base5_infos_{split}.pkl').read_bytes())
        rows.update({r['token']:r for r in info['data_list']})
    samples=[];frames=('camera_front','camera_front_left','camera_front_right','camera_back')
    for old in source['samples']:
        row=rows[old['id']]
        # Freeze the earlier, non-AP selection and prove sensor paths unchanged.
        assert row['lidar_points']['lidar_path']==old['lidar_path']
        paths=[row['images'][frame.replace('camera_', 'Camera_', 1)]['img_path'] for frame in frames]
        assert paths==old['camera_paths']
        payloads={path:(data/path).read_bytes() for path in [old['lidar_path'],*paths]}
        for path,blob in payloads.items():
            if hashlib.sha256(blob).hexdigest()!=manifest['files'][path]['sha256']:
                raise ValueError('Selected sensor checksum differs: '+path)
        samples.append(dict(id=row['token'],group=old['sample_group'],
            points=np.frombuffer(payloads[old['lidar_path']],np.float32).reshape(-1,4).copy(),
            images={'image/'+frame:payloads[path] for frame,path in zip(frames,paths)}))
    if len({s['id'] for s in samples})!=len(samples):raise ValueError('Repeated sample identity')
    return samples,dict(source_manifest_sha256=sha256(data/'manifest.json'),
        selection_sha256=sha256(selection),selection_scope=source['scope'],
        counts={group:sum(s['group']==group for s in samples) for group in ('representative','stress')})


def run_case(cfg, samples, shared, iterations, stage_iterations, memory_fraction):
    # Instrumentation stores bound methods; collect the previous case only
    # after its stack has returned, before measuring this case's VRAM.
    gc.collect()
    torch.cuda.empty_cache()
    torch.cuda.reset_peak_memory_stats()
    torch.manual_seed(261011)
    model=init_model(deepcopy(cfg),None,device='cuda:0')
    parameters=named_model_parameters(model)
    matched=[];different=[]
    if shared is None:shared={k:v.detach().cpu().clone() for k,v in parameters.items()}
    with torch.no_grad():
        for name,value in parameters.items():
            if name in shared and value.shape==shared[name].shape:
                value.copy_(shared[name]);matched.append(name)
            else:different.append(name)
    digest=hashlib.sha256()
    for name,value in parameters.items():
        digest.update(name.encode());digest.update(value.detach().cpu().contiguous().numpy().tobytes())
    restore_view_geometry(model.view_transform);model.eval()
    calibration=load_sensor4_calibration(Path(__file__).resolve().parents[4]/'configs/sensors/Sensor.json')
    pre=Base3OnlinePreprocessor(calibration,camera_workers=4,**preprocessing_options_from_config(cfg))
    rows=[];shapes={};handles=[]
    for name in ('pts_middle_encoder','view_transform','fusion_layer','pts_backbone','pts_neck'):
        def hook(module,inp,out,name=name):
            values=out if isinstance(out,(tuple,list)) else [out]
            shapes[name]=[list(v.shape) for v in values if torch.is_tensor(v)]
        handles.append(getattr(model,name).register_forward_hook(hook))
    try:
        with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
            for i in range(16):model.test_step(pre.prepare(samples[i%len(samples)]['points'],samples[i%len(samples)]['images']))
            torch.cuda.synchronize()
            warm_peak=torch.cuda.max_memory_allocated()/2**20
            for handle in handles:handle.remove()
            handles=[]
            expected=[cfg.model.bbox_head.train_cfg.grid_size[1]//8,cfg.model.bbox_head.train_cfg.grid_size[0]//8]
            assert shapes['pts_middle_encoder'][0][-2:]==shapes['view_transform'][0][-2:]==expected
            plan=model.view_transform._pool_optimization['plan']
            assert list(plan['shape'][2:])==model.view_transform.nx.detach().cpu().tolist()[:2]
            torch.cuda.reset_peak_memory_stats()
            for i in range(iterations):
                sample=samples[i%len(samples)];torch.cuda.synchronize();start=time.perf_counter()
                batch=pre.prepare(sample['points'],sample['images'],i);prepared=time.perf_counter()
                pred=model.test_step(batch)[0];torch.cuda.synchronize();forward=time.perf_counter()
                prediction=snapshot_prediction(pred)
                assert all(torch.isfinite(t).all() for t in prediction[:2])
                transferred=time.perf_counter()
                rows.append(dict(id=sample['id'],group=sample['group'],prepare_ms=(prepared-start)*1000,
                    model_ms=(forward-prepared)*1000,service_ms=(transferred-start)*1000,
                    retained_points=len(batch['inputs']['points'][0])))
                if (i+1)%50==0:print('INFERENCE %d/%d'%(i+1,iterations),flush=True)
            peak=torch.cuda.max_memory_allocated()/2**20
            rec=StageRecorder();original={};stages={};voxels=[]
            methods=[(model.data_preprocessor,'forward','transfer_normalize'),
                (model.img_backbone,'forward','camera_backbone'),(model.img_neck,'forward','camera_neck'),
                (model,'voxelize','voxelize'),(model.pts_middle_encoder,'forward','lidar_sparse'),
                (model.view_transform,'forward','view_total'),(model.view_transform,'get_geometry','geometry'),
                (model.view_transform,'get_cam_feats','depth_feature'),(model.view_transform,'bev_pool','pool'),
                (model.view_transform.downsample,'forward','camera_bev_downsample'),
                (model.fusion_layer,'forward','fusion'),(model.pts_backbone,'forward','bev_backbone'),
                (model.pts_neck,'forward','bev_neck'),(model.bbox_head,'predict','head_decode')]
            try:
                for obj,method,name in methods:
                    original[(obj,method)]=getattr(obj,method);rec.wrap(obj,method,name)
                rec.enabled=True
                for i in range(stage_iterations):
                    sample=samples[(i*7)%len(samples)];rec.records=[]
                    model.test_step(pre.prepare(sample['points'],sample['images']));torch.cuda.synchronize()
                    for name,begin,end,_ in rec.records:stages.setdefault(name,[]).append(begin.elapsed_time(end))
                # Read the actual voxel count separately; not a new timing span.
                old=model.voxelize
                for i in range(min(16,len(samples))):
                    batch=pre.prepare(samples[i]['points'],samples[i]['images'])
                    normalized=model.data_preprocessor(batch,False)
                    result=old(normalized['inputs']['points']);voxels.append(int(result[0].shape[0]))
            finally:
                for (obj,method),fn in original.items():setattr(obj,method,fn)
        groups={}
        for group in ('all','representative','stress'):
            selected=rows if group=='all' else [r for r in rows if r['group']==group]
            if selected:groups[group]=dict(count=len(selected),distinct=len({r['id'] for r in selected}),
                timing={k:summary([r[k] for r in selected]) for k in ('prepare_ms','model_ms','service_ms')})
        return dict(valid=True,timing=groups,per_sample=rows,stage_stream_ms={k:summary(v) for k,v in stages.items()},
            stage_note='Separate instrumented CUDA-event critical-path spans; nested view spans overlap and include host idle.',
            feature_shapes=shapes,occupied_voxels_first16=voxels,warmup_peak_allocated_mib=warm_peak,
            steady_peak_allocated_mib=peak,parameter_sha256=digest.hexdigest(),
            matched_parameters=len(matched),different_shape_parameters=different,
            view_nx=model.view_transform.nx.detach().cpu().tolist(),depth_bins=model.view_transform.D,
            pooling_cache={k:model.view_transform._pool_optimization[k]
                           for k in ('hits','misses','bypasses','bytes','limit')},
            allocator_fraction=memory_fraction),shared
    finally:
        for handle in handles:handle.remove()
        pre.close();del model,pre,parameters;gc.collect();torch.cuda.empty_cache()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True);p.add_argument('--selection',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--cases',nargs='+',choices=SPATIAL_CASES,default=list(SPATIAL_CASES))
    p.add_argument('--depth-steps',type=float,nargs='+',default=[.5])
    p.add_argument('--pool-cache-mib',type=int,choices=[256,512])
    p.add_argument('--iterations',type=int,default=300);p.add_argument('--stage-iterations',type=int,default=32)
    a=p.parse_args()
    if a.output.exists():p.error('Fresh output required')
    if not 24<=a.iterations<=600 or not 8<=a.stage_iterations<=64:p.error('Bounded iteration budget required')
    a.output.mkdir(parents=True)
    repo=Path(__file__).resolve().parents[4]
    code_paths=[Path(__file__),repo/'perception/detection3d/bevfusion/tools/base5_grid_candidates.py',
                repo/'perception/detection3d/bevfusion/plugin/morai_pool_optimization.py',
                repo/'perception/detection3d/bevfusion/plugin/morai_rectangular_bev.py',a.config]
    code_identity={str(path):sha256(path) for path in code_paths}
    torch.set_num_threads(4);cv2.setNumThreads(1)
    torch.backends.cudnn.benchmark=True;torch.backends.cuda.matmul.allow_tf32=True
    torch.cuda.set_per_process_memory_fraction(.85)
    base=Config.fromfile(str(a.config));samples,source=samples_from_selection(a.data,a.selection)
    result=dict(gpu=torch.cuda.get_device_name(),torch=torch.__version__,source=source,code_identity=code_identity,cases=[],
        scope='Fresh/shared compatible parameters, 4 classes, batch1 AMP; cost/finite output only, not learned accuracy or ROS/network latency.',
        fixed=dict(image=[512,1056],semantic_roi=list(base.point_cloud_range),pooling='separated',queries=200),
        measurement='300 distinct inputs by default; warmed uninstrumented serial service includes CPU prep + test_step + decoded CPU transfer; disk/network excluded')
    shared=None
    for case in a.cases:
        for step in a.depth_steps:
            name=case+'_depth'+str(step)
            cfg=candidate_config(base,case,step,a.pool_cache_mib);cfg.dump(str(a.output/(name+'.py')))
            record=dict(name=name,grid_case=case,depth_step=step,geometry=geometry(case))
            try:
                torch.cuda.reset_peak_memory_stats()
                values,shared=run_case(cfg,samples,shared,a.iterations,a.stage_iterations,.85)
                record.update(values)
            except Exception as exc:
                record.update(valid=False,error=repr(exc));gc.collect();torch.cuda.empty_cache()
            result['cases'].append(record)
            (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            print('CASE '+json.dumps({k:v for k,v in record.items() if k not in ('per_sample','stage_stream_ms')}),flush=True)
    assert code_identity=={str(path):sha256(path) for path in code_paths},'Probe source changed during execution'
    if any(not c['valid'] for c in result['cases']):raise SystemExit(1)


if __name__=='__main__':main()
