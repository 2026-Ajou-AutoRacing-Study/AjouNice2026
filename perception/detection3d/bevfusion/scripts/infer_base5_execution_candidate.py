#!/usr/bin/env python3
"""Export raw full-native E12 predictions for one explicit execution backend.

Uses the original Runner validation pipeline and canonical GT contract. No GT
matching, support filtering, ROS score floor/top100/NMS, online preprocessing,
training, package installation, networking or production replacement occurs.
Run each backend into a fresh directory, then score the identical ordered GT.
"""
import argparse
from copy import deepcopy
import gzip
import inspect
import json
from pathlib import Path
import pickle
import sys
import time
import types

from qualify_base5_camera_tensorrt import (
    ROOT, E12_SHA256, CLASSES, config_closure, distribution, sha256, write_new)

sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'docker/bevfusion'))
sys.path.insert(0, str(ROOT/'perception/detection3d/bevfusion/tools'))
from compare_base5_common30 import adapt_intensity, gt_identity, validate_payload


def sensor_paths(authored, data):
    """Deduplicate actual targets but also bind every original path alias."""
    aliases, targets = {}, set()
    for row in authored:
        names = [row['lidar_points']['lidar_path']] + [value['img_path'] for value in row['images'].values()]
        if len(row['images']) != 4: raise ValueError('Exactly four authored native cameras required')
        for name in names:
            path = Path(name)
            path = path if path.is_absolute() else data/path
            path = path.absolute()
            resolved = path.resolve(strict=True)
            if not resolved.is_file() or not resolved.stat().st_size:
                raise ValueError('Missing or empty native sensor: '+str(path))
            aliases[str(path)] = str(resolved)
            targets.add(str(resolved))
    return aliases, sorted(targets)


def sensor_snapshot(paths, mode):
    result = {}
    for name in paths:
        path = Path(name); stat = path.stat()
        record = dict(size=stat.st_size, mtime_ns=stat.st_mtime_ns,
                      inode=stat.st_ino, device=stat.st_dev)
        if mode == 'sha256': record['sha256'] = sha256(path)
        result[name] = record
    return result


def source_paths(config):
    files = config_closure(config) | {Path(__file__).resolve(),
        Path(__file__).with_name('qualify_base5_camera_tensorrt.py'),
        ROOT/'perception/detection3d/bevfusion/tools/compare_base5_common30.py',
        Path(__file__).with_name('base5_pool_pack4_candidate.py')}
    for folder in ('docker/bevfusion/app', 'perception/detection3d/bevfusion/plugin',
                   'third_party/mmdetection3d/projects/BEVFusion'):
        files.update(path for path in (ROOT/folder).rglob('*')
                     if path.suffix in ('.py', '.cu', '.cpp', '.h'))
    return files


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'info', 'config', 'checkpoint', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--expected-frames', type=int, required=True)
    p.add_argument('--backend', choices=['eager', 'camera_trt', 'camera_trt_head_sdpa_position',
                                        'pool_fused_layout', 'pool_pack4_fused'], required=True)
    p.add_argument('--engine', type=Path)
    p.add_argument('--metadata', type=Path)
    p.add_argument('--tf32-off', action='store_true')
    p.add_argument('--workers', type=int, choices=[0, 2, 4], default=2)
    p.add_argument('--sensor-integrity', choices=['sha256', 'stat'], default='sha256',
        help='sha256 hashes each distinct sensor once at start/end; stat only checks path target,size,mtime_ns,inode/device, NOT content identity')
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output required; prior evidence is immutable')
    if not 1 <= a.expected_frames <= 5000: p.error('Explicit bounded 1..5000 frame count required')
    camera_backend = a.backend in ('camera_trt', 'camera_trt_head_sdpa_position')
    head_candidate = a.backend == 'camera_trt_head_sdpa_position'
    if camera_backend != bool(a.engine and a.metadata):
        p.error('camera_trt requires both --engine and --metadata; other backends accept neither')
    if not camera_backend and (a.engine or a.metadata): p.error('Unused engine arguments are forbidden')
    for name in ('data', 'info', 'config', 'checkpoint'):
        setattr(a, name, getattr(a, name).resolve(strict=True))
    if sha256(a.checkpoint) != E12_SHA256: p.error('Frozen full-data E12 checkpoint SHA required')
    controls = {str(path): sha256(path) for path in (a.info, a.config, a.checkpoint)}
    with a.info.open('rb') as stream: authored_info = pickle.load(stream)
    authored = authored_info['data_list']
    if (len(authored) != a.expected_frames or len({str(row['token']) for row in authored}) != len(authored) or
            tuple(authored_info['metainfo']['classes']) != CLASSES):
        p.error('Authored frame/token/class contract differs')
    aliases, payload_paths = sensor_paths(authored, a.data)
    sensor_before = sensor_snapshot(payload_paths, a.sensor_integrity)
    if camera_backend:
        from app.camera_tensorrt import CameraTensorRTAdapter
        expected_engine = dict(checkpoint_sha256=E12_SHA256, config_sha256=controls[str(a.config)])
        adapter = CameraTensorRTAdapter(a.engine, a.metadata, expected_engine)
        engine_identity = deepcopy(adapter.metadata); adapter.close()
        controls.update({str(path.resolve()): sha256(path) for path in (a.engine, a.metadata)})
    else:
        expected_engine, engine_identity = None, None
    import cv2
    import numpy as np
    import torch
    from mmengine.config import Config
    from mmengine.runner import Runner
    from mmengine.utils import import_modules_from_strings
    from mmdet3d.apis import init_model
    from mmdet3d.utils import register_all_modules
    from app.inference_optimization import InferenceOptimization, validate_base5_runtime
    from app.view_geometry import restore_view_geometry

    torch.set_num_threads(4); cv2.setNumThreads(1); torch.manual_seed(261012)
    torch.backends.cuda.matmul.allow_tf32 = not a.tf32_off
    torch.backends.cudnn.allow_tf32 = not a.tf32_off
    torch.backends.cudnn.benchmark = True
    cfg = Config.fromfile(str(a.config), lazy_import=False)
    cfg.model.img_backbone.init_cfg = None
    cfg.model.compile_camera = False
    validate_base5_runtime(cfg)
    if (tuple(cfg.metainfo.classes) != CLASSES or tuple(cfg.model.view_transform.image_size) != (512,1056) or
            list(cfg.model.view_transform.dbound) != [1.,100.,.25] or
            list(cfg.model.data_preprocessor.voxelize_cfg.voxel_size)[:2] != [.05,.05] or
            cfg.model.bbox_head.num_proposals != 200 or cfg.model.bbox_head.test_cfg.get('nms_type') is not None):
        raise ValueError('Preserve learned E12 geometry, 200 raw queries, and no decoded-box NMS')
    register_all_modules(init_default_scope=True)
    if cfg.get('custom_imports'): import_modules_from_strings(**cfg.custom_imports)
    import perception.detection3d.bevfusion.plugin.morai_base5_dataset  # noqa: F401
    loader_cfg = deepcopy(cfg.val_dataloader)
    loader_cfg.batch_size = 1; loader_cfg.num_workers = a.workers
    loader_cfg.persistent_workers = bool(a.workers)
    if not a.workers:
        loader_cfg.pop('prefetch_factor', None)
        loader_cfg.pop('multiprocessing_context', None)
    loader_cfg.sampler = dict(type='DefaultSampler', shuffle=False)
    loader_cfg.dataset.update(type='MoraiBase5PedestrianDataset', data_root=str(a.data),
        ann_file=str(a.info), data_prefix=dict(pts='',img=''), test_mode=True,
        load_eval_anns=True, filter_empty_gt=False)
    loader = Runner.build_dataloader(loader_cfg)
    if len(loader.dataset) != len(authored): raise ValueError('Loader dropped or duplicated validation frames')
    a.output.mkdir(parents=True)
    resolved = a.output/'resolved_config.py'; cfg.dump(str(resolved))
    sources = source_paths(a.config) | {resolved.resolve()}
    initial_source_pins = {str(path): sha256(path) for path in sorted(sources)}
    start_receipt = dict(schema='base5_execution_candidate_native_v1', complete=False,
        backend=a.backend, started_unix=time.time(), expected_frames=a.expected_frames,
        source_sha256=initial_source_pins, control_sha256=controls,
        sensor_integrity=dict(mode=a.sensor_integrity, unique_files=len(payload_paths),
            snapshots='Once at start and once at finish, never per frame or per recovery',
            limitation=('stat-only is not a sensor content-SHA certificate' if a.sensor_integrity == 'stat' else None)),
        source_sensor_files=sensor_before, sensor_path_aliases=aliases,
        contract=dict(image='Original native Q90 JPEG; original Runner validation pipeline',
            native_intensity_scale=1., amp=True, parallel_camera=True, compile_camera=False,
            tf32=not a.tf32_off, cudnn_benchmark=True, raw_proposals=200,
            ros_score_floor=False, ros_top100=False, ros_rotated_nms=False,
            gt_support_filter=False, empty_gt_frames_preserved=True),
        engine_identity=engine_identity,
        mode_head_runtime=dict(proposal_topk=False, attention_sdpa=head_candidate, cache_bev_pos=head_candidate),
        production_changed=False)
    write_new(a.output/'STARTED.json', start_receipt)
    model = optimization = trt_handle = head_handle = original_pool = None
    try:
        checkpoint = torch.load(a.checkpoint, map_location='cpu', weights_only=False)
        saved_classes = checkpoint.get('meta', {}).get('dataset_meta', {}).get('classes')
        if saved_classes is not None and tuple(saved_classes) != CLASSES:
            raise ValueError('Checkpoint class order differs')
        model = init_model(deepcopy(cfg), None, device='cuda:0')
        model.load_state_dict(checkpoint['state_dict'], strict=True)
        del checkpoint
        restore_view_geometry(model.view_transform); model.eval()
        source_unavailable_module_types = set()
        for module in model.modules():
            try:
                path = inspect.getsourcefile(type(module))
            except (TypeError, OSError):
                path = None
            if path is None:
                source_unavailable_module_types.add(type(module).__module__+'.'+type(module).__qualname__)
            if path and Path(path).is_file(): sources.add(Path(path).resolve())
        actual_source_pins = {str(path): sha256(path) for path in sorted(sources)}
        # Prove pre-model pinned files did not change during model loading.
        if any(actual_source_pins[path] != digest for path,digest in initial_source_pins.items()):
            raise RuntimeError('Source changed during model initialization')
        if camera_backend:
            from app.camera_tensorrt import install_camera_tensorrt
            trt_handle = install_camera_tensorrt(model, a.engine, a.metadata, expected_engine)
        optimization = InferenceOptimization(model, (), parallel_camera=True)
        if head_candidate:
            from app.head_runtime_optimization import install_head_runtime_optimization
            # Research export is deliberately strict: unavailable hooks must
            # not silently produce a second reference and claim a speedup.
            head_handle = install_head_runtime_optimization(model.bbox_head,
                proposal_topk=False, attention_sdpa=True, cache_bev_pos=True)
        if a.backend.startswith('pool_'):
            from perception.detection3d.bevfusion.plugin import morai_pool_optimization as pool_module
            view = model.view_transform
            if view._pool_optimization['mode'] != 'separated': raise ValueError('Original separated pool required')
            original_pool = view.bev_pool
            pack4 = None
            if a.backend == 'pool_pack4_fused':
                from base5_pool_pack4_candidate import forward as pack4
            def candidate(self, geom, features):
                depth, feat = features; plan = self._pool_optimization['plan']
                if plan is None or tuple(depth.shape) != tuple(plan['input_shape']):
                    raise RuntimeError('Unchanged geometry plan missing or mismatched')
                output = (pack4(depth.contiguous().float(), feat.contiguous().float(), plan)
                          if pack4 else pool_module.SeparatedPool.apply(
                              depth, feat, plan, self._pool_optimization['backward_mode']))
                batch,z_cells,x_cells,y_cells,channels = output.shape
                return output.permute(0,1,4,3,2).contiguous().reshape(batch,z_cells*channels,y_cells,x_cells)
            view.bev_pool = types.MethodType(candidate, view)
        rows, elapsed = [], []
        began = time.monotonic()
        with torch.inference_mode():
            for i, batch in enumerate(loader):
                if int(batch['data_samples'][0].sample_idx) != i:
                    raise ValueError('Validation sampler changed original frame order')
                source = authored[i]; ann = batch['data_samples'][0].eval_ann_info
                gt = ann['gt_bboxes_3d'].tensor.cpu().numpy()
                expected = np.asarray([r['bbox_3d'] for r in source['instances']], np.float32).reshape(-1,7)
                expected[:,2] -= expected[:,5]/2
                if gt.shape != expected.shape or not np.allclose(gt,expected,rtol=0,atol=1e-5):
                    raise ValueError('Original Runner pipeline changed canonical authored GT')
                expected_labels = np.asarray([r['bbox_label_3d'] for r in source['instances']], dtype=np.int64)
                if not np.array_equal(ann['gt_labels_3d'], expected_labels):
                    raise ValueError('Original Runner pipeline changed authored GT labels')
                # Explicit identity adapter; raw XYZI/GT are never rewritten.
                for points in batch['inputs']['points']:
                    if (points.ndim != 2 or points.shape[1] != 4 or not bool(torch.isfinite(points).all()) or
                            (len(points) and (bool((points[:,3]<0).any()) or bool((points[:,3]>1.0001).any())))):
                        raise ValueError('Native E12 input must be finite XYZI with intensity 0..1; do not pass ROS255')
                batch['inputs']['points'] = [adapt_intensity(points,1.) for points in batch['inputs']['points']]
                torch.cuda.synchronize(); begin = time.perf_counter()
                with torch.autocast('cuda',enabled=True,dtype=torch.float16):
                    pred = model.test_step(batch)[0].pred_instances_3d
                boxes = pred.bboxes_3d.tensor.detach().cpu().numpy()
                scores = pred.scores_3d.detach().cpu().numpy()
                labels = pred.labels_3d.detach().cpu().numpy()
                torch.cuda.synchronize(); elapsed.append((time.perf_counter()-begin)*1000)
                if len(boxes) > 200: raise ValueError('Raw decoder exceeded learned query count')
                rows.append(dict(token=str(source['token']), gt_boxes=gt.tolist(),
                    gt_labels=ann['gt_labels_3d'].tolist(),
                    gt_points=[r['num_lidar_pts'] for r in source['instances']],
                    gt_assets=[r.get('asset','legacy_unknown') for r in source['instances']],
                    pred_boxes=boxes[:,:7].tolist(),pred_labels=labels.tolist(),pred_scores=scores.tolist()))
                if (i+1)%100 == 0:
                    print(json.dumps(dict(event='candidate_native_progress',backend=a.backend,
                        frames=i+1,total=a.expected_frames,seconds=round(time.monotonic()-began,1))),flush=True)
        payload = dict(name='E12_'+a.backend+('_tf32off' if a.tf32_off else '_runtime_tf32'),
            checkpoint_sha256=E12_SHA256, info_sha256=controls[str(a.info)],
            config_sha256=controls[str(a.config)], script_sha256=sha256(__file__),
            gt_identity=gt_identity(rows),frames=len(rows),classes=CLASSES,
            gpu=torch.cuda.get_device_name(),torch_version=str(torch.__version__),torch_cuda=torch.version.cuda,
            strict_state_load=True,inference_precision='AMP; TF32 '+('OFF' if a.tf32_off else 'ON')+'; cuDNN benchmark ON',
            inference_optimization=dict(optimization.report, backend=a.backend,
                head_runtime=deepcopy(head_handle.report) if head_handle is not None else None),
            input_pipeline=loader_cfg.dataset.pipeline,point_intensity_scale=1.,
            point_intensity_adapter='Identity native 0..1; no ROS255 conversion',
            inference_ms=distribution(elapsed[10:] if len(elapsed)>10 else elapsed),
            inference_ms_p50=distribution(elapsed[10:] if len(elapsed)>10 else elapsed)['p50'],
            inference_timing_warmup_excluded=10 if len(elapsed)>10 else 0,
            inference_timing_scope='Original Runner test_step plus decoded CPU transfer; excludes file decoding/loader, not live ROS latency',
            raw_prediction_contract='Original decoded ROI-filtered <=200 queries; no ROS floor/top100/NMS or GT filtering', rows=rows)
        validate_payload(payload)
        if payload['frames'] != a.expected_frames: raise ValueError('Missing original validation frames')
        if aliases != {name:str(Path(name).resolve(strict=True)) for name in aliases}:
            raise RuntimeError('A native sensor symlink/path target changed')
        if sensor_before != sensor_snapshot(payload_paths,a.sensor_integrity):
            raise RuntimeError('Native sensor inputs changed during export')
        if controls != {path:sha256(path) for path in controls} or actual_source_pins != {path:sha256(path) for path in actual_source_pins}:
            raise RuntimeError('Source, config, info, checkpoint or engine changed during export')
        with (a.output/'predictions.json.gz').open('xb') as raw:
            with gzip.open(raw,'wt') as stream: json.dump(payload,stream,allow_nan=False)
        receipt = {key:value for key,value in payload.items() if key != 'rows'}
        receipt.update(complete=True, backend=a.backend, finished_unix=time.time(),
            actual_source_sha256=actual_source_pins, control_sha256=controls,
            source_unavailable_module_types=sorted(source_unavailable_module_types),
            environment_binary_note='Actual Python module source files are pinned; framework/native library binaries are identified by the caller experiment image, not content-hashed here',
            sensor_integrity=start_receipt['sensor_integrity'], source_and_inputs_unchanged=True,
            predictions_sha256=sha256(a.output/'predictions.json.gz'), production_changed=False,
            gt_matching_performed=False, ap_evaluated=False)
        write_new(a.output/'receipt.json',receipt)
        write_new(a.output/'COMPLETE.json',dict(complete=True,frames=len(rows),gt_identity=payload['gt_identity'],
            receipt_sha256=sha256(a.output/'receipt.json'), predictions_sha256=receipt['predictions_sha256']))
        print(json.dumps(dict(event='candidate_native_complete',backend=a.backend,frames=len(rows),
                             gt_identity=payload['gt_identity'])),flush=True)
    except Exception as error:
        write_new(a.output/'FAILED.json',dict(complete=False,backend=a.backend,error=repr(error),production_changed=False))
        raise
    finally:
        if model is not None and original_pool is not None: model.view_transform.bev_pool = original_pool
        if optimization is not None: optimization.disable()
        if head_handle is not None: head_handle.close()
        if trt_handle is not None: trt_handle.close()


if __name__ == '__main__':
    main()
