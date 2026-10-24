#!/usr/bin/env python3
"""Read-only actual JPEG pipeline equivalence and CPU stage-cost qualification.

Compare legacy floats/placeholders, filtered floats and filtered RGB8 using
identical captures and random draws. Does not train, rewrite data or claim AP.
"""
import argparse
from collections import defaultdict
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from mmengine.config import Config
from mmengine.dataset import Compose
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import DATASETS, MODELS
from mmdet3d.utils import register_all_modules
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration


def summary(values):
    return dict(mean_ms=float(np.mean(values)),p50_ms=float(np.percentile(values,50)),
                p95_ms=float(np.percentile(values,95)),n=len(values))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--samples-per-split',type=int,default=64)
    parser.add_argument('--online-only',action='store_true',
                        help='Verify actual packed test input against online CPU preprocessing, without cost profiling')
    args=parser.parse_args()
    args.data=args.data.resolve()
    args.config=args.config.resolve()
    if args.output.exists(): parser.error('Use fresh output')
    if not 16 <= args.samples_per_split <= 256: parser.error('Bounded samples required')
    torch.set_num_threads(1);cv2.setNumThreads(0)
    register_all_modules(init_default_scope=True)
    cfg=Config.fromfile(str(args.config));import_modules_from_strings(**cfg.custom_imports)
    # BEVFusion itself consumes voxelize_cfg before building this preprocessor.
    # Here exercise only its real image cast/normalization on CPU.
    dp_cfg=deepcopy(cfg.model.data_preprocessor)
    dp_cfg.pop('voxelize_cfg',None)
    normalizer=MODELS.build(dp_cfg)
    sensor=Path(__file__).resolve().parents[4]/'configs/sensors/Sensor_competition_local.json'
    calibration=load_sensor4_calibration(sensor)
    saved_cal=json.loads((args.data/'infos/512x1056/calibration.json').read_text())
    if calibration['config_sha256']!=saved_cal['source_sha256']:
        raise ValueError('Sensor calibration differs from portable source')
    online=Base3OnlinePreprocessor(calibration,camera_workers=1,**preprocessing_options_from_config(cfg))
    test_pipeline=Compose(cfg.test_dataloader.dataset.pipeline)
    stats=defaultdict(list);point_stats=dict(records=0,zero_xyz=0,zero_xyz_nonzero_intensity=0,nonfinite=0,scenes=0,empty_after_filter=0)
    checks=0;online_checks=0;bytes_per_image_batch={};manifest_hash=hashlib.sha256((args.data/'manifest.json').read_bytes()).hexdigest()
    selected={}
    for split in ('train','val'):
        ds_cfg=deepcopy(cfg[split+'_dataloader'].dataset)
        ds_cfg.update(data_root=str(args.data),data_prefix=dict(pts='',img=''),
            ann_file=str(args.data/'infos/512x1056'/('morai_base5_infos_'+split+'.pkl')),pipeline=[])
        dataset=DATASETS.build(ds_cfg)
        # Scan the full 5K raw returns, without decoding any additional images.
        for i in range(len(dataset)):
            record=dataset.get_data_info(i)
            raw=np.fromfile(record['lidar_points']['lidar_path'],np.float32).reshape(-1,4)
            zero=(raw[:,:3]==0).all(axis=1);finite=np.isfinite(raw).all(axis=1)
            point_stats['records']+=len(raw);point_stats['zero_xyz']+=int(zero.sum())
            point_stats['zero_xyz_nonzero_intensity']+=int((zero & (raw[:,3]!=0)).sum())
            point_stats['nonfinite']+=int((~finite).sum());point_stats['scenes']+=1
            point_stats['empty_after_filter']+=int(not np.any(finite & ~zero))
        indices=sorted(range(len(dataset)),key=lambda i:hashlib.sha256(('input-review:'+split+':'+str(i)).encode()).digest())[:args.samples_per_split]
        selected[split]=[dataset.get_data_info(i)['token'] for i in indices]
        pipelines={}
        for name in ('legacy','filtered_float','filtered_uint8'):
            spec=deepcopy(cfg[split+'_dataloader'].dataset.pipeline)
            if name!='filtered_uint8':
                for item in spec:
                    if item['type']=='BEVLoadMultiViewImageFromFiles': item['to_float32']=True
                    if item['type']=='MoraiImageAug3D': item['output_dtype']='float32'
            if name=='legacy':spec=[t for t in spec if t['type']!='MoraiFilterInvalidPoints']
            pipelines[name]=(spec,Compose(spec).transforms)
        if args.online_only:
            for index in indices:
                record=dataset.get_data_info(index)
                offline=test_pipeline(deepcopy(record))
                raw=np.fromfile(record['lidar_points']['lidar_path'],np.float32).reshape(-1,4)
                # Infos use Camera_* asset keys; transport uses camera_* frame IDs.
                # The matrix comparison below verifies the actual view order too.
                blobs={'image/'+live_camera['frame_id']:Path(camera['img_path']).read_bytes()
                       for live_camera,camera in zip(calibration['cameras'],record['images'].values())}
                live=online.prepare(raw,blobs)
                torch.testing.assert_close(offline['inputs']['img'],live['inputs']['img'][0],rtol=0,atol=0)
                torch.testing.assert_close(offline['inputs']['points'],live['inputs']['points'][0],rtol=0,atol=0)
                for key in ('img_aug_matrix','lidar_aug_matrix','lidar2img','cam2img','lidar2cam','cam2lidar'):
                    reference=offline['data_samples'].metainfo.get(key,np.eye(4) if key=='lidar_aug_matrix' else None)
                    np.testing.assert_allclose(reference,live['data_samples'][0].metainfo[key],atol=1e-5,rtol=0)
                batch=dict(inputs={k:[v] for k,v in offline['inputs'].items()},data_samples=[offline['data_samples']])
                torch.testing.assert_close(normalizer(batch,training=False)['inputs']['imgs'],
                                           normalizer(live,training=False)['inputs']['imgs'],rtol=0,atol=0)
                online_checks+=1
            print('ONLINE',split,len(indices),flush=True)
            continue
        for epoch in range(3):
            for j,index in enumerate(indices):
                results={}
                # Rotate order to reduce warm-cache and scheduling bias.
                order=list(pipelines);offset=(j+epoch)%3;order=order[offset:]+order[:offset]
                for name in order:
                    spec,transforms=pipelines[name]
                    data=dataset.get_data_info(index)
                    seed=261011+index+epoch*10000
                    np.random.seed(seed);torch.manual_seed(seed)
                    total=0.
                    for item,transform in zip(spec,transforms):
                        start=time.perf_counter();data=transform(data)
                        elapsed=(time.perf_counter()-start)*1000;total+=elapsed
                        if j>=8:stats[split+':'+name+':'+item['type']].append(elapsed)
                    if j>=8:stats[split+':'+name+':TOTAL'].append(total)
                    results[name]=data
                    bytes_per_image_batch[name]=data['inputs']['img'].numel()*data['inputs']['img'].element_size()
                a,b=results['filtered_float'],results['filtered_uint8']
                torch.testing.assert_close(a['inputs']['img'],b['inputs']['img'].float(),rtol=0,atol=0)
                torch.testing.assert_close(a['inputs']['points'],b['inputs']['points'],rtol=0,atol=0)
                for key in ('img_aug_matrix','lidar_aug_matrix','lidar2img','cam2img','lidar2cam'):
                    aa=a['data_samples'].metainfo;bb=b['data_samples'].metainfo
                    assert (key in aa)==(key in bb)
                    if key in aa:np.testing.assert_array_equal(aa[key],bb[key])
                if split=='train':
                    for key in ('labels_3d',):
                        torch.testing.assert_close(a['data_samples'].gt_instances_3d[key],b['data_samples'].gt_instances_3d[key],rtol=0,atol=0)
                    torch.testing.assert_close(a['data_samples'].gt_instances_3d.bboxes_3d.tensor,
                                               b['data_samples'].gt_instances_3d.bboxes_3d.tensor,rtol=0,atol=0)
                # The real MMDetection3D cast + normalization, not a handwritten approximation.
                aa=normalizer(dict(inputs=dict(img=[a['inputs']['img']]),data_samples=[deepcopy(a['data_samples'])]),training=False)
                bb=normalizer(dict(inputs=dict(img=[b['inputs']['img']]),data_samples=[deepcopy(b['data_samples'])]),training=False)
                torch.testing.assert_close(aa['inputs']['imgs'],bb['inputs']['imgs'],rtol=0,atol=0)
                checks+=1
            print('QUALIFY',split,epoch+1,len(indices),flush=True)
    assert manifest_hash==hashlib.sha256((args.data/'manifest.json').read_bytes()).hexdigest()
    online.close()
    report=dict(valid=True,scope='CPU input qualification only; not GPU latency or AP',
        timing_scope='3 repeats, first 8 excluded each split/repeat' if not args.online_only else 'not measured',
        config=str(args.config),manifest_sha256=manifest_hash,selected_tokens=selected,point_stats=point_stats,
        exact_equality_checks=checks,online_equality_checks=online_checks,image_bytes_per_frame=bytes_per_image_batch,
        timings={k:summary(v) for k,v in stats.items()})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('selected_tokens','timings')},indent=2))
    print(json.dumps({k:v for k,v in report['timings'].items() if k.endswith(':TOTAL')},indent=2))


if __name__=='__main__': main()
