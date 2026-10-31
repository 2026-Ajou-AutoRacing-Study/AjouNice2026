#!/usr/bin/env python3
"""Frozen E12 AMP/parallel profiling and reversible pooling-only A/B.

No training, production edit, simulator, network access or package installation.
The caller opts into local CUDA execution by running this CLI. Whole-service
timings, instrumented stream/host spans, and the optional one-frame activation
capture are distinct phases. Nested CUDA-event spans MUST NOT be summed.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
import types

from qualify_base5_camera_tensorrt import (
    ROOT, E12_SHA256, CLASSES, aggregate_comparisons, compare_detections,
    config_closure, distribution, load_inputs, ros_delivery, sha256,
    validate_detections, write_new)


def fused_layout(output):
    batch, z_cells, x_cells, y_cells, channels = output.shape
    return output.permute(0, 1, 4, 3, 2).contiguous().reshape(
        batch, z_cells*channels, y_cells, x_cells)


def instrumentation(runtime, samples, torch):
    """Separate critical-path spans and CPU submission timeline; no sums."""
    from profile_base5_stages import StageRecorder
    from perception.detection3d.bevfusion.plugin import morai_pool_optimization as pool_module
    model, rec, originals, marker_originals = runtime.model, StageRecorder(), {}, {}
    stages = [(model.data_preprocessor, 'forward', 'transfer_normalize'),
        (model.img_backbone, 'forward', 'camera_backbone'),
        (model.img_neck, 'forward', 'camera_neck'),
        (model, 'voxelize', 'voxelize'),
        (model.pts_middle_encoder, 'forward', 'lidar_sparse'),
        (model.view_transform, 'forward', 'view_total'),
        (model.view_transform, 'get_geometry', 'geometry'),
        (model.view_transform, 'get_cam_feats', 'depth_feature'),
        (model.view_transform, 'bev_pool', 'pool'),
        (model.view_transform.downsample, 'forward', 'camera_bev_downsample'),
        (model.fusion_layer, 'forward', 'fusion'),
        (model.pts_backbone, 'forward', 'bev_backbone'),
        (model.pts_neck, 'forward', 'bev_neck'),
        (model.bbox_head, 'predict', 'head_decode')]
    current, rows, spans, host = {}, [], {}, {}
    signature_original = pool_module.signature

    def marked(old, name):
        def call(*args, **kwargs):
            current.setdefault('markers', []).append(dict(name=name+'_enter', at=time.perf_counter()))
            try:
                return old(*args, **kwargs)
            finally:
                current['markers'].append(dict(name=name+'_exit', at=time.perf_counter()))
        return call

    def signature(view, args, kwargs):
        if view is not model.view_transform:
            return signature_original(view, args, kwargs)
        return marked(signature_original, 'signature')(view, args, kwargs)

    try:
        for obj, method, name in stages:
            originals[(obj, method)] = getattr(obj, method)
            rec.wrap(obj, method, name)
        for obj, method, name in ((model.img_backbone, 'forward', 'camera_backbone'),
                                  (model.pts_middle_encoder, 'forward', 'lidar_sparse')):
            marker_originals[(obj, method)] = getattr(obj, method)
            setattr(obj, method, marked(getattr(obj, method), name))
        pool_module.signature = signature
        rec.enabled = True
        for i, sample in enumerate(samples):
            current.clear(); rec.records = []
            origin = time.perf_counter()
            runtime.infer(sample['points'], sample['images'], i, 0., 200)
            torch.cuda.synchronize()
            for name, begin, end, submitted in rec.records:
                spans.setdefault(name, []).append(begin.elapsed_time(end))
                host.setdefault(name, []).append(submitted)
            markers = current.get('markers', [])
            by_name = {}
            for marker in markers:
                by_name.setdefault(marker['name'], []).append(marker['at'])
            gaps = {}
            for label, first, last in (
                ('signature_host_call_ms', 'signature_enter', 'signature_exit'),
                ('camera_exit_to_signature_exit_ms', 'camera_backbone_exit', 'signature_exit'),
                ('signature_exit_to_sparse_enter_ms', 'signature_exit', 'lidar_sparse_enter'),
                ('camera_exit_to_sparse_enter_ms', 'camera_backbone_exit', 'lidar_sparse_enter')):
                if by_name.get(first) and by_name.get(last):
                    gaps[label] = (by_name[last][0]-by_name[first][0])*1000
            rows.append(dict(id=sample['id'], markers=[dict(name=r['name'],
                offset_ms=(r['at']-origin)*1000) for r in markers], host_gaps=gaps))
        keys = {k for row in rows for k in row['host_gaps']}
        return dict(stage_stream_ms={k: distribution(v) for k, v in spans.items()},
            stage_host_call_ms={k: distribution(v) for k, v in host.items()},
            host_marker_rows=rows,
            host_marker_distributions={key: distribution([r['host_gaps'][key] for r in rows
                if key in r['host_gaps']]) for key in sorted(keys)},
            note='Separate instrumented phase. CUDA event spans include host idle and parent/child overlap; never add them. Host markers measure CPU submission order, not GPU completion.')
    finally:
        rec.enabled = False
        pool_module.signature = signature_original
        for (obj, method), old in marker_originals.items(): setattr(obj, method, old)
        for (obj, method), old in originals.items(): setattr(obj, method, old)


def capture_pool(runtime, sample, output, metadata, torch):
    """Exactly one untimed original-pool entry; save original dtypes/indices."""
    from benchmark_base5_pool_variants import PLAN_KEYS, SCHEMA, validate_payload
    view, captured = runtime.model.view_transform, []
    original = view.bev_pool
    def capture(geom, features):
        if captured:
            raise RuntimeError('One inference unexpectedly called pooling more than once')
        depth, feat = features
        plan = view._pool_optimization['plan']
        if plan is None: raise RuntimeError('Original geometry did not prepare a pool plan')
        payload = dict(schema=SCHEMA, depth=depth.detach().contiguous().cpu().clone(),
            feat=feat.detach().contiguous().cpu().clone(),
            plan={key: plan[key].detach().contiguous().cpu().clone() for key in PLAN_KEYS},
            metadata=dict(metadata, sample_id=sample['id'], sample_group=sample['group']))
        payload['plan'].update(shape=tuple(map(int, plan['shape'])),
                               input_shape=tuple(map(int, plan['input_shape'])))
        captured.append(payload)
        return original(geom, features)
    try:
        view.bev_pool = capture
        runtime.infer(sample['points'], sample['images'], 0, 0., 200)
        torch.cuda.synchronize()
    finally:
        view.bev_pool = original
    if len(captured) != 1: raise RuntimeError('Exactly one original pool activation required')
    description = validate_payload(captured[0], torch)
    with output.open('xb') as stream: torch.save(captured[0], stream)
    return dict(path=str(output), sha256=sha256(output), description=description,
                note='Untimed original pool entry; no dtype conversion, synthetic features or GT modification')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs-manifest', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--samples', type=int, default=12)
    p.add_argument('--timing-repeats', type=int, default=3)
    p.add_argument('--variants', nargs='+', choices=['baseline', 'fused_layout', 'pack4_fused'], default=['baseline'])
    p.add_argument('--capture-pool', action='store_true')
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output required')
    if not 1 <= a.samples <= 64 or not 1 <= a.timing_repeats <= 10:
        p.error('Bounded 1..64 actual samples and 1..10 timing repeats required')
    if a.variants[0] != 'baseline' or len(set(a.variants)) != len(a.variants):
        p.error('Baseline first and no repeated variants required')
    if sha256(a.checkpoint) != E12_SHA256: p.error('Frozen full-data E12 checkpoint required')
    samples, input_contract, inputs = load_inputs(a.inputs_manifest, a.samples)
    import cv2
    import torch
    from mmengine.config import Config
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.plugin import morai_pool_optimization as pool_module
    base = Config.fromfile(str(a.config), lazy_import=False); cfg = deepcopy(base)
    if (tuple(cfg.metainfo.classes) != CLASSES or list(cfg.model.view_transform.image_size) != [512, 1056] or
            list(cfg.model.view_transform.dbound) != [1., 100., .25] or
            list(cfg.model.data_preprocessor.voxelize_cfg.voxel_size)[:2] != [.05, .05]):
        p.error('Unchanged current E12 learned geometry required')
    cfg.model.compile_camera = False; cfg.model.img_backbone.init_cfg = None
    a.output.mkdir(parents=True)
    resolved = a.output/'resolved_config.py'; cfg.dump(str(resolved))
    paths = config_closure(a.config) | {Path(__file__).resolve(), resolved.resolve(), a.sensor_config.resolve(),
        Path(__file__).with_name('qualify_base5_camera_tensorrt.py'),
        Path(__file__).with_name('profile_base5_stages.py'),
        Path(__file__).with_name('benchmark_base5_pool_variants.py'),
        Path(__file__).with_name('base5_pool_pack4_candidate.py')}
    for folder in ('docker/bevfusion/app', 'perception/detection3d/bevfusion/plugin',
                   'third_party/mmdetection3d/projects/BEVFusion'):
        paths.update(path for path in (ROOT/folder).rglob('*') if path.suffix in ('.py', '.cu', '.cpp', '.h'))
    source_pins = {str(path): sha256(path) for path in sorted(paths)}
    inputs = dict(inputs, **{str(a.checkpoint.resolve()): E12_SHA256})
    result = dict(schema='base5_e12_execution_profile_v1', complete=False, started_unix=time.time(),
        source_sha256=source_pins, input_sha256=inputs, checkpoint_sha256=E12_SHA256,
        input_contract=dict(image='original Q90 JPEG', size=[512,1056],
            intensity_source=input_contract['point_intensity_source'], amp=True, parallel_camera=True,
            compile_camera=False, temporal=False), selected_ids=[s['id'] for s in samples],
        output_contract=dict(raw_floor=0., raw_topk=200, ros_floor=.1, ros_topk=100,
            ros_class_aware_rotated_nms=.5, ros_delivery_thresholds=[.4,.1,.1],
            box_origin='lidar_bottom_v1', gt_used=False),
        scope='In-memory serial requests; disk/LAN/ROS/tracker excluded. No AP or production approval.', cases=[])
    write_new(a.output/'STARTED.json', result)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    runtime, original_pool = None, None
    try:
        runtime = BevFusionModel(resolved, a.checkpoint, load_sensor4_calibration(a.sensor_config),
            base5_runtime=True, use_amp=True, parallel_camera=True, warmup_iterations=2,
            point_intensity_source=input_contract['point_intensity_source'])
        result.update(gpu=torch.cuda.get_device_name(), torch_version=str(torch.__version__),
                      cuda_version=torch.version.cuda,
                      runtime_numerics=dict(tf32=torch.backends.cuda.matmul.allow_tf32,
                                           cudnn_benchmark=torch.backends.cudnn.benchmark))
        view = runtime.model.view_transform
        if view._pool_optimization['mode'] != 'separated': raise ValueError('Original separated pool required')
        original_pool = view.bev_pool
        baseline = None
        modes = a.variants+(['baseline_restore'] if len(a.variants) > 1 else [])
        for mode in modes:
            view.bev_pool = original_pool
            if mode in ('fused_layout', 'pack4_fused'):
                pack4 = None
                if mode == 'pack4_fused':
                    from base5_pool_pack4_candidate import forward as pack4
                def candidate(self, geom, features, pack4=pack4):
                    depth, feat = features; plan = self._pool_optimization['plan']
                    if plan is None: raise RuntimeError('Original geometry plan missing')
                    if tuple(depth.shape) != tuple(plan['input_shape']): raise ValueError('Pool input/plan mismatch')
                    # Same explicit FP32 cast as SeparatedPool.forward; no
                    # quantization or reduction-order change is introduced.
                    output = (pack4(depth.contiguous().float(), feat.contiguous().float(), plan)
                              if pack4 else pool_module.SeparatedPool.apply(
                                  depth, feat, plan, self._pool_optimization['backward_mode']))
                    return fused_layout(output)
                view.bev_pool = types.MethodType(candidate, view)
            for i in range(max(4, len(samples))):
                sample = samples[i % len(samples)]
                runtime.infer(sample['points'], sample['images'], i, 0., 200)
            torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats()
            predictions, timings, repeats, repeat_exact = {}, [], [], []
            for repeat in range(a.timing_repeats):
                for i, sample in enumerate(samples):
                    begin = time.perf_counter()
                    prediction, prepare_ms, model_ms = runtime.infer(sample['points'], sample['images'], i, 0., 200)
                    elapsed = (time.perf_counter()-begin)*1000
                    validate_detections(prediction)
                    if repeat == 0: predictions[sample['id']] = prediction
                    else:
                        repeat_exact.append(predictions[sample['id']] == prediction)
                        repeats.append(compare_detections(predictions[sample['id']], prediction))
                    timings.append(dict(id=sample['id'], repeat=repeat, service_ms=elapsed,
                        preprocess_ms=prepare_ms, model_ms=model_ms,
                        postprocess_ms=runtime.last_runtime_timings['postprocess_ms']))
            memory = dict(peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
                device_free_mib=torch.cuda.mem_get_info()[0]/2**20)
            if baseline is None: baseline = predictions
            policies = dict(raw=lambda x:x, ros_ap_candidates=lambda x:ros_delivery(x,False),
                            ros_delivery=lambda x:ros_delivery(x,True))
            checks = {name:[dict(id=s['id'], **compare_detections(policy(baseline[s['id']]),
                         policy(predictions[s['id']]))) for s in samples] for name,policy in policies.items()}
            case = dict(variant=mode, timing={key:distribution([r[key] for r in timings]) for key in
                ('service_ms','preprocess_ms','model_ms','postprocess_ms')}, timing_rows=timings, vram=memory,
                comparisons={name:aggregate_comparisons(rows) for name,rows in checks.items()},
                per_frame_comparisons=checks,
                all_raw_prediction_values_equal_to_baseline=predictions == baseline,
                within_mode_repeats_exact_values=(all(repeat_exact) if repeat_exact else None),
                within_mode_raw_repeats=aggregate_comparisons(repeats) if repeats else None)
            for name, policy in policies.items():
                write_new(a.output/(mode+'_'+name+'_predictions.json'), {key:policy(value) for key,value in predictions.items()})
            case['instrumentation'] = instrumentation(runtime, samples, torch)
            if a.capture_pool and mode == 'baseline':
                case['pool_capture'] = capture_pool(runtime, samples[0], a.output/'pool_activation.pt',
                    dict(checkpoint_sha256=E12_SHA256, config_sha256=sha256(a.config),
                         sensor_config_sha256=sha256(a.sensor_config), amp=True, parallel=True,
                         intensity_source=input_contract['point_intensity_source']), torch)
            result['cases'].append(case)
            write_new(a.output/(mode+'_RESULT.json'), case)
            print(json.dumps(dict(event='e12_execution_profile_case_complete', variant=mode,
                service_ms=case['timing']['service_ms'],
                raw_engineering_passed=case['comparisons']['raw']['engineering_passed'])), flush=True)
        if source_pins != {path:sha256(path) for path in source_pins} or inputs != {path:sha256(path) for path in inputs}:
            raise RuntimeError('Source, weights or sensor/control input changed during profile')
        result.update(complete=True, finished_unix=time.time(), source_and_inputs_unchanged=True,
            engineering_passed=all(case['comparisons']['raw']['engineering_passed'] and
                (case['within_mode_raw_repeats'] is None or case['within_mode_raw_repeats']['engineering_passed'])
                for case in result['cases']), production_promoted=False,
            note='Engineering differences are reported, not silently accepted. No AP or continuous tracker evaluation.')
        write_new(a.output/'report.json', result)
        write_new(a.output/'COMPLETE.json', dict(complete=True, report_sha256=sha256(a.output/'report.json'),
                   engineering_passed=result['engineering_passed'], production_promoted=False))
    except Exception as error:
        write_new(a.output/'FAILED.json', dict(complete=False, error=repr(error),
            completed_cases=[case['variant'] for case in result['cases']], production_promoted=False))
        raise
    finally:
        if runtime is not None:
            if original_pool is not None: runtime.model.view_transform.bev_pool = original_pool
            if runtime.inference_optimization: runtime.inference_optimization.disable()
            runtime.preprocessor.close()


if __name__ == '__main__':
    main()
