#!/usr/bin/env python3
"""Bounded E12 head-only A/B on captured real sensor activations.

Run only when no other GPU benchmark is active. This is not a full-service
latency/AP/tracker certificate and does not modify production or training.
Capturing uses the existing online runtime once per distinct sensor sample;
all timed head variants then receive exactly the same captured activation.
"""
import argparse
from copy import deepcopy
from pathlib import Path
import sys
import time

from qualify_base5_camera_tensorrt import (
    ROOT, E12_SHA256, CLASSES, load_inputs, sha256, config_closure, distribution,
    compare_detections, aggregate_comparisons, ros_delivery, write_new)


def predictions(value):
    boxes = value.bboxes_3d.tensor.detach().float().cpu().tolist()
    scores = value.scores_3d.detach().float().cpu().tolist()
    labels = value.labels_3d.detach().cpu().tolist()
    return [dict(center=box[:3], size=box[3:6], yaw=box[6], label=int(label),
                 class_name=CLASSES[int(label)], score=score)
            for box, score, label in zip(boxes, scores, labels)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('inputs-manifest', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--samples', type=int, default=2)
    p.add_argument('--iterations', type=int, default=20)
    p.add_argument('--warmup', type=int, default=4)
    p.add_argument('--engine', type=Path)
    p.add_argument('--metadata', type=Path)
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output directory required')
    if not 1 <= a.samples <= 8 or not 1 <= a.iterations <= 100 or not 1 <= a.warmup <= 16:
        p.error('Bounded samples1..8 / iterations1..100 / warmup1..16 required')
    if bool(a.engine) != bool(a.metadata): p.error('Engine and metadata must be supplied together')
    if sha256(a.checkpoint) != E12_SHA256: p.error('Frozen full-data E12 checkpoint SHA required')
    samples, contract, pins = load_inputs(a.inputs_manifest, a.samples)
    sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT/'docker/bevfusion'))
    import cv2
    import torch
    from mmengine.config import Config
    from mmdet3d.structures import LiDARInstance3DBoxes
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    from app.head_runtime_optimization import install_head_runtime_optimization
    cfg = deepcopy(Config.fromfile(str(a.config), lazy_import=False))
    cfg.model.compile_camera = False
    cfg.model.img_backbone.init_cfg = None
    if (tuple(cfg.metainfo.classes) != CLASSES or
            list(cfg.model.view_transform.image_size) != [512, 1056] or
            list(cfg.model.view_transform.dbound) != [1., 100., .25]):
        p.error('Frozen E12 geometry/classes required')
    a.output.mkdir(parents=True)
    resolved = a.output/'resolved_config.py'; cfg.dump(str(resolved))
    sources = config_closure(a.config) | {Path(__file__).resolve(), resolved.resolve(),
        a.sensor_config.resolve(), ROOT/'docker/bevfusion/app/head_runtime_optimization.py',
        ROOT/'docker/bevfusion/app/bevfusion_model.py',
        ROOT/'third_party/mmdetection3d/projects/BEVFusion/bevfusion/transfusion_head.py'}
    pins.update({str(path): sha256(path) for path in sources})
    pins[str(a.checkpoint)] = E12_SHA256
    if a.engine:
        pins.update({str(path): sha256(path) for path in (a.engine, a.metadata)})
    report = dict(schema='base5_head_runtime_benchmark_v1', complete=False,
        scope='Head predict/decode only; no full-service/AP/ROS/tracker promotion',
        production_changed=False, checkpoint_sha256=E12_SHA256,
        source_and_inputs_sha256=pins, selected_ids=[s['id'] for s in samples], cases=[])
    write_new(a.output/'STARTED.json', report)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    torch.backends.cuda.matmul.allow_tf32 = True
    torch.backends.cudnn.allow_tf32 = True
    torch.cuda.set_per_process_memory_fraction(.8)
    runtime, handle, capture_hook, camera_handle = None, None, None, None
    captured = []
    try:
        # Optional camera TRT is explicit; it only selects which real features
        # are captured, never switches the tensor between head A/B cases.
        options = dict(base5_runtime=True, use_amp=True, parallel_camera=True,
                       point_intensity_source=contract['point_intensity_source'], warmup_iterations=2)
        runtime = BevFusionModel(resolved, a.checkpoint, load_sensor4_calibration(a.sensor_config), **options)
        if a.engine:
            from app.camera_tensorrt import install_camera_tensorrt
            camera_handle = install_camera_tensorrt(runtime.model, a.engine, a.metadata,
                expected_contract=dict(checkpoint_sha256=E12_SHA256, config_sha256=sha256(a.config)))
        head = runtime.model.bbox_head
        sample = samples[0]
        runtime.infer(sample['points'], sample['images'], 0, 0., 200)
        def capture(_module, args):
            captured.append(args[0].detach().contiguous().cpu().clone())
        capture_hook = head.shared_conv.register_forward_pre_hook(capture)
        for i, sample in enumerate(samples):
            runtime.infer(sample['points'], sample['images'], i, 0., 200)
        capture_hook.remove(); capture_hook = None
        if len(captured) != len(samples): raise RuntimeError('Exactly one head activation per sample required')
        if any(tuple(value.shape) != (1, 512, 168, 504) for value in captured):
            raise RuntimeError('Unexpected E12 fine-grid head activation shape')
        with (a.output/'head_activations.pt').open('xb') as stream:
            torch.save(dict(schema='base5_head_activations_v1', features=captured,
                checkpoint_sha256=E12_SHA256, ids=report['selected_ids'],
                config_sha256=sha256(a.config), camera_trt=bool(a.engine)), stream)
        features = [value.cuda() for value in captured]
        metas = [dict(box_type_3d=LiDARInstance3DBoxes)]
        report['gpu'] = torch.cuda.get_device_name()
        report['activation_dtypes'] = [str(value.dtype) for value in features]
        report['runtime_numerics'] = dict(amp=True, matmul_allow_tf32=torch.backends.cuda.matmul.allow_tf32,
                                         cudnn_allow_tf32=torch.backends.cudnn.allow_tf32)
        variants = [('baseline', {}), ('topk', dict(proposal_topk=True)),
                    ('sdpa', dict(attention_sdpa=True)),
                    ('topk_sdpa', dict(proposal_topk=True, attention_sdpa=True)),
                    ('topk_sdpa_position', dict(proposal_topk=True, attention_sdpa=True, cache_bev_pos=True)),
                    ('baseline_restore', {})]
        reference = None
        for name, flags in variants:
            if flags: handle = install_head_runtime_optimization(head, **flags)
            wall, device, results = [], [], []
            with torch.inference_mode(), torch.autocast('cuda', dtype=torch.float16):
                for i in range(a.warmup): head.predict(features[i % len(features)], metas)
                for _ in range(a.iterations):
                    for feature in features:
                        torch.cuda.synchronize()
                        begin, end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
                        at = time.perf_counter(); begin.record()
                        result = head.predict(feature, metas)[0]
                        end.record(); end.synchronize()
                        wall.append((time.perf_counter()-at)*1000); device.append(begin.elapsed_time(end))
                # Readback/comparison is excluded from the timed phase.
                for feature in features: results.append(predictions(head.predict(feature, metas)[0]))
            if reference is None: reference = results
            checks = {}
            for scope, transform in [('raw', lambda x: x),
                                     ('ros_delivery', lambda x: ros_delivery(x, True))]:
                checks[scope] = aggregate_comparisons([compare_detections(transform(left), transform(right))
                                                      for left, right in zip(reference, results)])
            case = dict(name=name, flags=flags, head_wall_ms=distribution(wall),
                        head_cuda_event_ms=distribution(device), comparisons=checks,
                        helper_report=handle.report if handle else None)
            write_new(a.output/(name+'_predictions.json'), results)
            write_new(a.output/(name+'_RESULT.json'), case)
            report['cases'].append(case)
            print(dict(event='head_case_complete', name=name, wall_ms=case['head_wall_ms']), flush=True)
            if handle: handle.close(); handle = None
        if pins != {path: sha256(path) for path in pins}:
            raise RuntimeError('Inputs/source/checkpoint changed during benchmark')
        report['complete'] = True
        write_new(a.output/'report.json', report)
        write_new(a.output/'COMPLETE.json', dict(report_sha256=sha256(a.output/'report.json'), production_promoted=False))
    except Exception as error:
        write_new(a.output/'FAILED.json', dict(error=repr(error), complete=False))
        raise
    finally:
        if capture_hook: capture_hook.remove()
        if handle: handle.close()
        if camera_handle: camera_handle.disable(); camera_handle.close()
        if runtime:
            runtime.close()


if __name__ == '__main__':
    main()
