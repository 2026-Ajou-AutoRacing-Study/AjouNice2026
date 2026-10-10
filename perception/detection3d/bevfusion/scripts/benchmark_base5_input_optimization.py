#!/usr/bin/env python3
"""Actual 5K raw: crop/path correctness and isolated dense-grid batch1 costs.

Fresh FOUR-class cost probes, NOT learned AP. No production configuration changes.
Outputs include calibration/geometry hashes, stage spans and per-sample timing.
"""
import argparse
import ast
from copy import deepcopy
import gc
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
from mmengine.config import Config
from mmengine.structures import InstanceData
from mmdet3d.apis import init_model
from mmdet3d.structures import LiDARInstance3DBoxes
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from app.view_geometry import restore_view_geometry
from perception.detection3d.bevfusion.plugin.morai_image_aug import MoraiImageAug3D
from perception.detection3d.bevfusion.plugin.morai_pool_optimization import (
    enable_pool_optimization,disable_pool_optimization,extension)
from profile_base5_stages import configuration,StageRecorder

CLASSES=('vehicle','pedestrian','obstacle','animal')
FRAMES=('camera_front','camera_front_left','camera_front_right','camera_back')
GEOMETRY_PARAMETERS={'view_transform.dx','view_transform.bx',
                     'view_transform.nx','view_transform.frustum'}


def named_model_parameters(model):
    # Upstream stores geometry as requires_grad=False nn.Parameters, not
    # buffers. These must vary with ROI and must never be copied as weights.
    return {name:value for name,value in model.named_parameters()
            if name not in GEOMETRY_PARAMETERS}


def summary(values):
    return {k:float(v) for k,v in zip(('mean','p50','p95','p99'),
                    [np.mean(values),*np.percentile(values,[50,95,99])])}


def config_for(root,hw,lateral_range=30):
    folder=root/'perception/detection3d/bevfusion/configs'
    if lateral_range==40:
        cfg=Config.fromfile(folder/'bevfusion_base5_dense_512_lateral40_separated_candidate.py')
        cfg.model.img_backbone.init_cfg=None
        # The harness installs each mode itself and compares it against an
        # unoptimized reference; avoid enabling separated pooling twice.
        cfg.model.view_transform.type='MoraiRectangularDepthLSSTransform'
    elif lateral_range==30:
        cfg=configuration(folder,'wide_dense_highres')
    else:
        raise ValueError('Only qualified lateral ranges 30/40 are supported')
    for pipeline in (cfg.train_pipeline,cfg.test_pipeline,cfg.train_dataloader.dataset.pipeline,
                     cfg.val_dataloader.dataset.pipeline,cfg.test_dataloader.dataset.pipeline):
        for aug in pipeline:
            if aug['type']=='MoraiImageAug3D':
                aug['final_dim']=list(hw);aug['resize_lim']=[hw[1]/1280]*2
    cfg.model.view_transform.image_size=list(hw)
    cfg.model.view_transform.feature_size=[v//8 for v in hw]
    cfg.model.pts_middle_encoder.type='MoraiRectangularSparseEncoder'
    assert cfg.model.bbox_head.num_classes==4
    assert cfg.model.view_transform.dbound[2]==.5
    assert cfg.model.data_preprocessor.voxelize_cfg.voxel_size==[.075,.075,.2]
    return cfg


def load_samples(folder):
    meta=json.loads((folder/'manifest.json').read_text())
    samples=[]
    if 'portable_data_root' in meta:
        root=Path(meta['portable_data_root']).resolve(strict=True)
        if hashlib.sha256((root/'manifest.json').read_bytes()).hexdigest()!=meta['source_sha256']:
            raise ValueError('Portable source identity changed')
        for item in meta['samples']:
            def path(value):
                candidate=(root/value).resolve(strict=True)
                candidate.relative_to(root)
                return candidate
            points=np.fromfile(path(item['lidar_path']),dtype='<f4').reshape(-1,4)
            samples.append(dict(meta=item,points=points,
                images={'image/'+frame:path(value).read_bytes()
                    for frame,value in zip(FRAMES,item['camera_paths'])}))
        return meta,samples
    for item in meta['samples']:
        path=folder/item['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=item['sha256']:
            raise ValueError('Sample checksum mismatch')
        with np.load(path) as data:
            samples.append(dict(meta=item,points=data['points'].copy(),
                images={'image/'+frame:data['camera'+str(i+1)].tobytes()
                        for i,frame in enumerate(FRAMES)}))
    return meta,samples


def host_e25_function(root):
    """Execute only the actual pure image method, no ROS node/master required."""
    source=root/'base_ws/src/perception/bevfusion_ros/scripts/bevfusion_ros_bridge.py'
    tree=ast.parse(source.read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and
             any(isinstance(y,ast.FunctionDef) and y.name=='_preprocess_camera' for y in x.body))
    method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_preprocess_camera')
    scope={'cv2':cv2,'np':np}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
    return scope['_preprocess_camera']


def preprocessing_check(root,samples,calibration,hw):
    pre=Base3OnlinePreprocessor(calibration,camera_workers=4,final_dim=hw,resize=hw[1]/1280,
                               point_cloud_range=(-100,-30,-3,100,30,2))
    aug=MoraiImageAug3D(final_dim=hw,resize_lim=[hw[1]/1280]*2,bot_pct_lim=[0,0],
                       rot_lim=[0,0],rand_flip=False,is_train=False)
    normalized=sys.modules.get('dynamic_capture_metadata')
    if normalized is None:
        sys.path.insert(0,str(root/'dataset/morai_od/scripts'))
        import dynamic_capture_metadata as normalized
    reference=normalized.normalized_sensor_calibration(root/'configs/sensors/Sensor.json')
    tests=[];host_method=host_e25_function(root)
    for sample in samples:
        native=[];metas={};blobs=sample['images']
        for camera in calibration['cameras']:
            rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(blobs['image/'+camera['frame_id']],np.uint8),1),cv2.COLOR_BGR2RGB)
            padded=np.zeros((720,1280,3),np.float32);padded[:rgb.shape[0],:rgb.shape[1]]=rgb
            native.append(padded);metas[camera['frame_id']]=dict(height=rgb.shape[0],width=rgb.shape[1])
        transformed=aug.transform(dict(img=native,images=metas))
        online=pre.prepare(sample['points'],blobs)
        prepared_blobs={}
        for i,camera in enumerate(calibration['cameras']):
            pixels=online['inputs']['img'][0][i].permute(1,2,0).numpy()
            np.testing.assert_array_equal(pixels,transformed['img'][i])
            np.testing.assert_allclose(pre.img_aug_matrix[i],transformed['img_aug_matrix'][i],atol=1e-5)
            prepared_blobs['image/'+camera['frame_id']]=pixels.tobytes()
            uid=i+1;ref=next(v for v in reference['cameras'].values() if v['sensor_unique_id']==uid)
            matrix=np.asarray(ref['ego_to_sensor'])@np.asarray(reference['lidar']['lidar_to_ego'])
            np.testing.assert_allclose(matrix,camera['lidar2cam'],atol=1e-6)
            np.testing.assert_allclose(ref['intrinsic'],camera['cam2img'],atol=1e-6)
            pts=sample['points'][:,:3]
            cam=pts@camera['lidar2cam'][:3,:3].T+camera['lidar2cam'][:3,3]
            valid=cam[:,2]>1e-3;cam=cam[valid]
            raw=cam@camera['cam2img'].T;raw/=raw[:,2:3]
            full=cam@camera['cam2img'].T
            composed=full@pre.img_aug_matrix[i,:3,:3].T
            # img_aug translation is after perspective division, not in K*XYZ.
            composed=composed/composed[:,2:3]+pre.img_aug_matrix[i,:3,3]
            aug_matrix=transformed['img_aug_matrix'][i]
            direct=raw@aug_matrix[:3,:3].T+aug_matrix[:3,3]
            np.testing.assert_allclose(composed,direct,atol=1e-5,rtol=1e-6)
            if hw==(256,704):
                obj=type('Host',(),{'camera_shapes':{camera['frame_id']:rgb_shape for rgb_shape in [(camera['height'],camera['width'])]}})()
                msg=type('Msg',(),{'data':blobs['image/'+camera['frame_id']]})()
                _,host=host_method(obj,(camera['frame_id'],msg))
                np.testing.assert_array_equal(host,pixels)
        prepared=pre.prepare(sample['points'],prepared_blobs,images_preprocessed=True)
        torch.testing.assert_close(prepared['inputs']['img'][0],online['inputs']['img'][0],rtol=0,atol=0)
        tests.append(sample['meta']['id'])
    pre.close()
    return dict(hw=hw,samples=len(tests),pixel_exact=True,homography_and_projection=True,
                sensor_source_sha256=reference['source_sha256'],
                host_prepared_E25_verified=hw==(256,704),
                note='Host ROS prepared path remains E25-only; enlarged models use raw Docker preprocessing until explicitly wired.')


def snapshot_prediction(pred):
    p=pred.pred_instances_3d
    return [p.bboxes_3d.tensor.detach().cpu(),p.scores_3d.detach().cpu(),p.labels_3d.detach().cpu()]


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path('/workspace/AjouNice2026'))
    p.add_argument('--samples',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--iterations',type=int,default=28);p.add_argument('--shapes',nargs='+',default=['320x896','384x896','448x896','512x1056'])
    p.add_argument('--modes',nargs='+',default=['baseline','cache','separated'])
    p.add_argument('--codec',choices=['png','jpeg90'],default='png')
    p.add_argument('--lateral-range',type=int,choices=[30,40],default=30,
                   help='Semantic lateral half-range; metric intervals remain unchanged')
    p.add_argument('--shared-parameter-state',type=Path,
                   help='Create/reuse random parameters for controlled range comparisons; excludes geometry buffers')
    p.add_argument('--no-stages',action='store_true',help='Measure service latency without stage instrumentation')
    p.add_argument('--skip-preprocess',action='store_true');a=p.parse_args()
    if a.output.exists():raise ValueError('Use a new output file')
    torch.set_num_threads(4);cv2.setNumThreads(1)
    torch.backends.cudnn.benchmark=True;torch.backends.cuda.matmul.allow_tf32=True
    calibration=load_sensor4_calibration(a.root/'configs/sensors/Sensor.json')
    manifest,samples=load_samples(a.samples)
    if a.codec=='jpeg90':
        for sample in samples:
            for key,blob in sample['images'].items():
                image=cv2.imdecode(np.frombuffer(blob,np.uint8),1)
                ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,90])
                if not ok:raise RuntimeError('JPEG cost sample encoding failed')
                sample['images'][key]=jpeg.tobytes()
    result=dict(gpu=torch.cuda.get_device_name(),torch=torch.__version__,amp=True,batch=1,codec=a.codec,
                source=manifest,cost_only=True,preprocessing=[],cases=[],
                lateral_range=a.lateral_range,
                contract='Fresh4-class; 200queries; 0.075m voxel,0.30m cameraBEV,0.50m depth; +/-100m longitudinal, +/-%dm lateral label ROI. Same weights within each shape. No ROS/network/AP.'%a.lateral_range)
    def save():
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(result,indent=2)+'\n')
    if not a.skip_preprocess:
        for hw in [(256,704)]+[tuple(map(int,v.split('x'))) for v in a.shapes]:
            result['preprocessing'].append(preprocessing_check(a.root,samples,calibration,hw));save()
            print('PREPROCESS PASS '+str(hw),flush=True)
    if 'separated' in a.modes:extension()
    for shape in a.shapes:
        hw=tuple(map(int,shape.split('x')));cfg=config_for(a.root,hw,a.lateral_range)
        torch.manual_seed(261010);model=init_model(deepcopy(cfg),None,device='cuda:0')
        restore_view_geometry(model.view_transform);model.eval()
        if a.shared_parameter_state:
            if a.shared_parameter_state.exists():
                shared=torch.load(a.shared_parameter_state,map_location='cpu',weights_only=True)
                parameters=named_model_parameters(model)
                if set(shared)!=set(parameters):
                    raise ValueError('Shared parameter names differ')
                with torch.no_grad():
                    for name,value in shared.items():
                        if value.shape!=parameters[name].shape or value.dtype!=parameters[name].dtype:
                            raise ValueError('Shared parameter shape/dtype differs: '+name)
                        parameters[name].copy_(value)
                del shared,parameters
            else:
                a.shared_parameter_state.parent.mkdir(parents=True,exist_ok=True)
                torch.save({name:value.detach().cpu().clone()
                            for name,value in named_model_parameters(model).items()},a.shared_parameter_state)
        restore_view_geometry(model.view_transform)
        expected_nx=[round((bound[1]-bound[0])/bound[2])
                     for bound in (cfg.model.view_transform.xbound,
                                   cfg.model.view_transform.ybound,
                                   cfg.model.view_transform.zbound)]
        if model.view_transform.nx.detach().cpu().tolist()!=expected_nx:
            raise ValueError('Model geometry differs from the selected ROI')
        parameter_hash=hashlib.sha256()
        for name,parameter in named_model_parameters(model).items():
            parameter_hash.update(name.encode())
            parameter_hash.update(parameter.detach().cpu().contiguous().numpy().tobytes())
        pre=Base3OnlinePreprocessor(calibration,camera_workers=4,**preprocessing_options_from_config(cfg))
        baseline_view=None;baseline_pred=None
        captured=[]
        handle=model.view_transform.register_forward_hook(lambda m,inp,out:captured.append(out.detach().cpu()))
        with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
            batch=pre.prepare(samples[0]['points'],samples[0]['images'])
            baseline_pred=snapshot_prediction(model.test_step(batch)[0]);baseline_view=captured[-1];captured.clear()
        handle.remove()
        for mode in a.modes:
            case=dict(shape=shape,mode=mode,parameter_sha256=parameter_hash.hexdigest(),
                      parameter_hash_scope='all parameters except dx/bx/nx/frustum geometry',
                      view_nx=expected_nx);state=None
            if mode!='baseline':state=enable_pool_optimization(model.view_transform,mode)
            rec=StageRecorder()
            original_methods={}
            try:
                captured=[]
                handle=model.view_transform.register_forward_hook(lambda m,inp,out:captured.append(out.detach().cpu()))
                with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
                    pred=snapshot_prediction(model.test_step(pre.prepare(samples[0]['points'],samples[0]['images']))[0])
                    view=captured[-1]
                handle.remove()
                torch.testing.assert_close(view,baseline_view,atol=3e-5,rtol=3e-5)
                case['view_max_abs_error']=float((view-baseline_view).abs().max())
                case['prediction_shapes_equal']=[list(v.shape) for v in pred]==[list(v.shape) for v in baseline_pred]
                case['prediction_close']=case['prediction_shapes_equal'] and all(
                    torch.allclose(v,b,atol=1e-3,rtol=1e-3) for v,b in zip(pred,baseline_pred))
                if not case['prediction_close']:
                    raise AssertionError('Decoded predictions differ from the baseline')
                case['correctness']=True
                # Instrumentation restored explicitly before switching runtime modes.
                stage_methods=[(model.view_transform,'forward','view_total'),
                    (model.view_transform,'get_geometry','geometry'),
                    (model.view_transform,'get_cam_feats','depth_feature'),
                    (model.view_transform,'bev_pool','pool'),(model.img_backbone,'forward','camera_backbone'),
                    (model.pts_middle_encoder,'forward','lidar_sparse'),(model.bbox_head,'predict','head')]
                for obj,method,name in ([] if a.no_stages else stage_methods):
                    original_methods[(obj,method)]=getattr(obj,method);rec.wrap(obj,method,name)
                values=[];spans={};torch.cuda.reset_peak_memory_stats()
                with torch.inference_mode():
                    for i in range(a.iterations+8):
                        s=samples[i%len(samples)];rec.enabled=i>=8;rec.records=[]
                        torch.cuda.synchronize();t=time.perf_counter();batch=pre.prepare(s['points'],s['images'],i)
                        prep=(time.perf_counter()-t)*1000;t1=time.perf_counter()
                        with torch.autocast('cuda',dtype=torch.float16):prediction=model.test_step(batch)[0]
                        torch.cuda.synchronize();t2=time.perf_counter()
                        snapshot_prediction(prediction);t3=time.perf_counter()
                        if i==7:
                            case['warmup_peak_allocated_mib']=torch.cuda.max_memory_allocated()/2**20
                            torch.cuda.reset_peak_memory_stats()
                        if i>=8:
                            values.append(dict(id=s['meta']['id'],prepare_ms=prep,model_ms=(t2-t1)*1000,
                                               service_ms=(t3-t)*1000,retained_points=len(batch['inputs']['points'][0])))
                            for name,begin,end,_ in rec.records:
                                spans.setdefault(name,[]).append(begin.elapsed_time(end))
                case.update(valid=True,stage_instrumented=not a.no_stages,iterations=a.iterations,timing={k:summary([v[k] for v in values])
                    for k in ('prepare_ms','model_ms','service_ms')},per_sample=values,
                    stages={k:summary(v) for k,v in spans.items()},
                    steady_peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                    depth=list(cfg.model.view_transform.dbound),voxel=list(cfg.model.data_preprocessor.voxelize_cfg.voxel_size),
                    camera_bev=list(cfg.model.view_transform.xbound),
                    camera_bev_ybound=list(cfg.model.view_transform.ybound),
                    semantic_roi=list(cfg.point_cloud_range),
                    sparse_shape=list(cfg.model.pts_middle_encoder.sparse_shape),
                    classes=list(cfg.classes))
                if state:case['cache']={k:state[k] for k in ('hits','misses','bypasses','bytes','limit')}
            except Exception as exc:
                case.update(valid=False,error=repr(exc));print('FAILED '+str(case),flush=True)
            finally:
                for (obj,method),fn in original_methods.items():setattr(obj,method,fn)
                if state:disable_pool_optimization(model.view_transform)
            result['cases'].append(case);save()
            print(json.dumps({k:v for k,v in case.items() if k not in ('per_sample','stages')}),flush=True)
        pre.close();original_methods.clear();state=None;captured.clear()
        del model,pre,rec,handle;gc.collect();torch.cuda.empty_cache()
    save()
    if any(not v.get('valid') for v in result['cases']):raise SystemExit(1)


if __name__=='__main__':main()
