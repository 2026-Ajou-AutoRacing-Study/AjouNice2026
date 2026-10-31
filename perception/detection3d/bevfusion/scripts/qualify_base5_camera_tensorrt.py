#!/usr/bin/env python3
"""Opt-in, same-E12-model camera TensorRT probe; never a production/AP gate.

No simulator, training, GT filtering, engine build, package installation or
network access. The caller supplies a locally built engine and immutable
inputs. Run in the dedicated compatible experiment environment. Captured
activations are measured separately from uninstrumented service timings.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
E12_SHA256 = 'f3ade1bf17375c061ea6c0284d636cfc647c98b745f4534cb365499e1394dba8'
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
CAMERAS = ('camera_front', 'camera_front_left', 'camera_front_right', 'camera_back')
ENGINEERING_LIMITS = dict(center_xy_m=.05, center_z_m=.05, size_linf_m=.05,
                          yaw_rad=.02, score_abs=.002)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def distribution(values):
    import numpy as np
    value = np.asarray(values, dtype=float).reshape(-1)
    if not np.isfinite(value).all():
        raise ValueError('Nonfinite numerical comparison')
    if not len(value):
        return dict(count=0, mean=None, p50=None, p90=None, p95=None, p99=None,
                    minimum=None, maximum=None)
    return dict(count=int(value.size), mean=float(value.mean()),
                **{'p'+str(p): float(np.percentile(value, p)) for p in (50, 90, 95, 99)},
                minimum=float(value.min()), maximum=float(value.max()))


def tensor_difference(reference, candidate):
    """Exact per-frame activation quantiles, not a sampled global percentile."""
    import numpy as np
    old, new = np.asarray(reference, dtype=np.float32), np.asarray(candidate, dtype=np.float32)
    if old.shape != new.shape or not np.isfinite(old).all() or not np.isfinite(new).all():
        raise ValueError('Activation shape or finite-value contract differs')
    delta = new.astype(np.float64) - old.astype(np.float64)
    return dict(shape=list(old.shape), absolute_error=distribution(np.abs(delta).reshape(-1)),
                relative_l2=float(np.linalg.norm(delta.reshape(-1)) /
                                  max(np.linalg.norm(old.astype(np.float64).reshape(-1)), 1e-12)))


def validate_detections(detections):
    import numpy as np
    for row in detections:
        label = row['label']
        if (not isinstance(label, int) or label not in range(3) or row['class_name'] != CLASSES[label] or
                len(row['center']) != 3 or len(row['size']) != 3 or
                not 0 <= row['score'] <= 1 or any(v <= 0 for v in row['size']) or
                not np.isfinite([*row['center'], *row['size'], row['yaw'], row['score']]).all()):
            raise ValueError('Finite three-class bottom-center detections required')


def ros_delivery(detections, final_thresholds=True):
    """Preserve real floor -> top100 -> NMS -> optional class cutoff order."""
    sys.path.insert(0, str(ROOT/'base_ws/src/perception/bevfusion_ros/src'))
    from bevfusion_ros.rotated_bev_nms import class_aware_rotated_bev_nms
    validate_detections(detections)
    selected = sorted((d for d in detections if d['score'] >= .1), key=lambda d: -d['score'])[:100]
    selected = class_aware_rotated_bev_nms(selected, .5)
    thresholds = (.4, .1, .1)
    return [d for d in selected if not final_thresholds or d['score'] >= thresholds[d['label']]]


def compare_detections(reference, candidate, gate_m=1.):
    """All-object class-preserving XY match; removed/added boxes stay visible.

    This is prediction-to-prediction association, not matching against GT and
    not temporal tracker identity. The 1m gate is an association aid, not an
    accuracy tolerance; tighter engineering limits are reported independently.
    """
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    validate_detections(reference)
    validate_detections(candidate)
    classes = {}
    for label, name in enumerate(CLASSES):
        old = [d for d in reference if d['label'] == label]
        new = [d for d in candidate if d['label'] == label]
        pairs, missing, extra = [], set(range(len(old))), set(range(len(new)))
        if old and new:
            costs = np.linalg.norm(np.array([d['center'][:2] for d in old])[:, None] -
                                   np.array([d['center'][:2] for d in new])[None], axis=-1)
            rows, cols = linear_sum_assignment(np.where(costs < gate_m, costs, 1e9))
            for i, j in zip(rows, cols):
                if costs[i, j] >= gate_m:
                    continue
                missing.remove(int(i)); extra.remove(int(j))
                x, y = old[i], new[j]
                angle = x['yaw'] - y['yaw']
                pairs.append(dict(center_xy_m=float(costs[i, j]),
                    center_z_m=abs(float(x['center'][2]-y['center'][2])),
                    size_linf_m=float(np.max(np.abs(np.array(x['size'])-y['size']))),
                    yaw_rad=abs(float(np.arctan2(np.sin(angle), np.cos(angle)))),
                    score_abs=abs(float(x['score']-y['score']))))
        classes[name] = dict(reference=len(old), candidate=len(new), matched=len(pairs),
            unmatched_reference=[old[i] for i in sorted(missing)],
            unmatched_candidate=[new[i] for i in sorted(extra)], pairs=pairs)
    all_pairs = [p for row in classes.values() for p in row['pairs']]
    passed = (bool(reference) and all(not row['unmatched_reference'] and not row['unmatched_candidate']
                                    for row in classes.values()) and
              all(p[name] <= bound for p in all_pairs for name, bound in ENGINEERING_LIMITS.items()))
    return dict(classes=classes, matched=len(all_pairs),
                distributions={key: distribution([p[key] for p in all_pairs]) for key in ENGINEERING_LIMITS},
                engineering_limits=ENGINEERING_LIMITS, engineering_passed=passed,
                nonvacuous=bool(reference), association_xy_gate_m=gate_m)


def aggregate_comparisons(rows):
    result = {}
    for name in CLASSES:
        parts = [row['classes'][name] for row in rows]
        pairs = [pair for part in parts for pair in part['pairs']]
        result[name] = dict(reference=sum(p['reference'] for p in parts),
            candidate=sum(p['candidate'] for p in parts), matched=len(pairs),
            unmatched_reference=sum(len(p['unmatched_reference']) for p in parts),
            unmatched_candidate=sum(len(p['unmatched_candidate']) for p in parts),
            distributions={key: distribution([p[key] for p in pairs]) for key in ENGINEERING_LIMITS})
    # Empty individual frames do not make the whole experiment fail, but an
    # entirely empty model must never be called a passed qualification.
    nonempty = [r for r in rows if r['nonvacuous']]
    return dict(classes=result, compared_frames=len(rows),
                nonvacuous=bool(nonempty), engineering_passed=bool(nonempty) and all(
                    r['engineering_passed'] if r['nonvacuous'] else not any(
                        v['candidate'] for v in r['classes'].values()) for r in rows))


def config_closure(path):
    path = Path(path).resolve()
    found = {path}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_base_' for t in node.targets):
            bases = ast.literal_eval(node.value)
            for base in [bases] if isinstance(bases, str) else bases:
                found.update(config_closure(path.parent/base))
    return found


def load_inputs(manifest_path, count):
    """Load explicit Q90 sensor-only selection; never inspect label/GT fields.

    Manifest schema: {schema:'base5_camera_trt_inputs_v1',
      image_policy:'native_jpeg90', point_intensity_source:'native'|'ros255',
      samples:[{id,group,points:{path,sha256},
        images:{'image/camera_front':{path,sha256}, ...all four...}}]}.
    Relative paths are relative to the manifest, not the invoking shell.
    """
    import numpy as np
    manifest_path = Path(manifest_path).resolve()
    source = json.loads(manifest_path.read_text())
    if (source.get('schema') != 'base5_camera_trt_inputs_v1' or
            source.get('image_policy') != 'native_jpeg90' or
            source.get('point_intensity_source') not in ('native', 'ros255')):
        raise ValueError('Explicit Q90/native-or-ROS255 sensor manifest required')
    rows = source['samples']
    if len(rows) < count or len({r['id'] for r in rows}) != len(rows):
        raise ValueError('Enough distinct input identities required; no repeated-frame inflation')
    selected, pins = [], {str(manifest_path): sha256(manifest_path)}
    for row in rows[:count]:
        images = row['images']
        if set(images) != {'image/'+name for name in CAMERAS}:
            raise ValueError('Four current cameras in explicit runtime names required')
        payloads = {}
        for name, item in [('points', row['points']), *images.items()]:
            path = Path(item['path'])
            path = (manifest_path.parent/path).resolve() if not path.is_absolute() else path.resolve()
            if not path.is_file() or not path.stat().st_size or sha256(path) != item['sha256']:
                raise ValueError('Input missing or checksum differs: '+str(path))
            pins[str(path)] = item['sha256']
            payload = path.read_bytes()
            if name != 'points' and payload[:2] != b'\xff\xd8':
                raise ValueError('Original JPEG bytes required, not on-the-fly PNG conversion')
            payloads[name] = payload
        points = np.frombuffer(payloads.pop('points'), dtype=np.float32).reshape(-1, 4).copy()
        ceiling = 1.0001 if source['point_intensity_source'] == 'native' else 255.0001
        if not np.isfinite(points).all() or np.any(points[:, 3] < 0) or np.any(points[:, 3] > ceiling):
            raise ValueError('Native 0..1 / ROS255 0..255 intensity contract differs')
        selected.append(dict(id=row['id'], group=row.get('group', 'unspecified'), points=points, images=payloads))
    return selected, source, pins


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs-manifest', 'config', 'checkpoint', 'sensor-config', 'engine', 'metadata', 'output'):
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--samples', type=int, default=8)
    p.add_argument('--warmup', type=int, default=4)
    p.add_argument('--timing-repeats', type=int, default=3)
    p.add_argument('--association-gate-m', type=float, default=1.)
    p.add_argument('--memory-budget-gib', type=float)
    p.add_argument('--require-decoded-parity', action='store_true')
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output directory required; never overwrite earlier evidence')
    if not 1 <= a.samples <= 64 or not 1 <= a.warmup <= 16 or not 1 <= a.timing_repeats <= 10:
        p.error('Bounded 1..64 samples, 1..16 warmups, 1..10 timing repeats required')
    if not 0 < a.association_gate_m <= 4: p.error('Finite association gate in (0,4] required')
    if sha256(a.checkpoint) != E12_SHA256: p.error('This first probe requires the frozen full-data E12 weights')
    samples, input_contract, inputs_before = load_inputs(a.inputs_manifest, a.samples)
    sys.path.insert(0, str(ROOT/'docker/bevfusion'))
    from app.camera_tensorrt import CameraTensorRTAdapter, install_camera_tensorrt
    expected_engine_contract = dict(checkpoint_sha256=E12_SHA256, config_sha256=sha256(a.config))
    # Reject an ONNX-only receipt, wrong frozen identity or changed binary
    # before allocating a model or importing CUDA libraries. The installer
    # additionally checks actual loaded camera parameters later.
    preflight_adapter = CameraTensorRTAdapter(a.engine, a.metadata, expected_engine_contract)
    preflight_adapter.close()
    # Heavy libraries and CUDA are imported only after CPU input preflight.
    import cv2
    import numpy as np
    import torch
    from mmengine.config import Config
    from app.bevfusion_model import BevFusionModel
    from app.sensor4_calibration import load_sensor4_calibration
    cfg = deepcopy(Config.fromfile(str(a.config), lazy_import=False))
    if (tuple(cfg.metainfo.classes) != CLASSES or list(cfg.model.view_transform.image_size) != [512, 1056] or
            list(cfg.model.view_transform.dbound) != [1., 100., .25] or
            list(cfg.model.data_preprocessor.voxelize_cfg.voxel_size)[:2] != [.05, .05]):
        p.error('Preserve current learned E12 classes/image/depth/voxel geometry')
    cfg.model.compile_camera = False
    cfg.model.img_backbone.init_cfg = None
    a.output.mkdir(parents=True)
    resolved = a.output/'resolved_config.py'; cfg.dump(str(resolved))
    sources = config_closure(a.config) | {Path(__file__).resolve(), a.sensor_config.resolve(), resolved.resolve()}
    for folder in ('docker/bevfusion/app', 'perception/detection3d/bevfusion/plugin',
                   'third_party/mmdetection3d/projects/BEVFusion'):
        sources.update((ROOT/folder).rglob('*.py'))
    pins = {str(path): sha256(path) for path in sorted(sources)}
    originals = dict(inputs_before, **{str(path.resolve()): sha256(path) for path in
                    (a.checkpoint, a.engine, a.metadata)})
    result = dict(schema='base5_camera_tensorrt_qualification_v1', complete=False,
        started_unix=time.time(), production_promoted=False, ap_evaluated=False, tracker_evaluated=False,
        source_sha256=pins, input_and_binary_sha256=originals,
        checkpoint_sha256=E12_SHA256, selected_ids=[s['id'] for s in samples],
        engine_build_identity=deepcopy(preflight_adapter.metadata['engine']),
        engine_io_contract=deepcopy(preflight_adapter.metadata['contract']),
        input_contract=dict(image_policy='native_jpeg90', image_size=[512, 1056],
            point_intensity_source=input_contract['point_intensity_source'],
            normalization='native unchanged; ros255 divided exactly once by existing preprocessor',
            amp=True, parallel_camera=True, compile_camera=False, temporal=False),
        output_contract=dict(raw=dict(score_floor=0., max_detections=200, nms=False),
            ros_ap_candidates=dict(score_floor=.1, max_detections=100, class_aware_rotated_nms=.5),
            ros_delivery=dict(score_thresholds=[.4, .1, .1], box_origin='lidar_bottom_v1'),
            note='No GT or model-distance filtering; preserve raw predictions for later AP and bag analysis'),
        scope='Local in-memory service; disk/LAN/ROS/tracker excluded. Engineering parity is not an accuracy or production approval.',
        cases=[])
    write_new(a.output/'STARTED.json', result)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    fraction = .85 if a.memory_budget_gib is None else a.memory_budget_gib*2**30/torch.cuda.get_device_properties(0).total_memory
    if not 0 < fraction <= .85: raise ValueError('Allocator cap must fit within 85% of this device')
    torch.cuda.set_per_process_memory_fraction(fraction)
    runtime = None; trt_handle = None; handles = []; observed = {}
    try:
        runtime = BevFusionModel(resolved, a.checkpoint, load_sensor4_calibration(a.sensor_config),
            base5_runtime=True, use_amp=True, parallel_camera=True, warmup_iterations=2,
            point_intensity_source=input_contract['point_intensity_source'])
        result.update(gpu=torch.cuda.get_device_name(), torch_version=str(torch.__version__),
                      torch_cuda_version=torch.version.cuda,
                      runtime_numerics=dict(matmul_allow_tf32=torch.backends.cuda.matmul.allow_tf32,
                          cudnn_allow_tf32=torch.backends.cudnn.allow_tf32,
                          cudnn_benchmark=torch.backends.cudnn.benchmark),
                      allocator_budget_gib=fraction*torch.cuda.get_device_properties(0).total_memory/2**30)
        reference_predictions = None; reference_activations = None
        for mode in ('eager', 'tensorrt', 'eager_repeat'):
            if mode == 'tensorrt':
                trt_handle = install_camera_tensorrt(runtime.model, a.engine, metadata_path=a.metadata,
                    expected_contract=expected_engine_contract)
            if mode == 'eager_repeat':
                trt_handle.disable(); trt_handle.close(); trt_handle = None
            for i in range(a.warmup):
                sample = samples[i % len(samples)]
                runtime.infer(sample['points'], sample['images'], i, 0., 200)
            torch.cuda.synchronize(); torch.cuda.reset_peak_memory_stats()
            times, predictions, within_mode_repeats = [], {}, []
            for repeat in range(a.timing_repeats):
                for i, sample in enumerate(samples):
                    begin = time.perf_counter()
                    prediction, prepare_ms, model_ms = runtime.infer(sample['points'], sample['images'], i, 0., 200)
                    total_ms = (time.perf_counter()-begin)*1000
                    validate_detections(prediction)
                    if repeat == 0:
                        predictions[sample['id']] = prediction
                    else:
                        # Matching runs after the stopwatch; repeat drift is
                        # visible without contaminating service latency.
                        within_mode_repeats.append(compare_detections(
                            predictions[sample['id']], prediction, a.association_gate_m))
                    times.append(dict(id=sample['id'], repeat=repeat, service_ms=total_ms,
                        preprocess_ms=prepare_ms, model_ms=model_ms,
                        postprocess_ms=runtime.last_runtime_timings['postprocess_ms']))
            torch.cuda.synchronize()
            peak = dict(allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                        reserved_mib=torch.cuda.max_memory_reserved()/2**20,
                        device_free_mib=torch.cuda.mem_get_info()[0]/2**20,
                        note='PyTorch allocator excludes TensorRT allocations; device-free snapshot also includes other processes')
            # GPU cloning and CPU activation readback occur ONLY in this
            # separate diagnostic phase, never in the timings just collected.
            activations, camera_spans = {}, []
            camera_events = {}
            def begin_camera(module, args):
                camera_events['begin'].record(torch.cuda.current_stream())
            def capture_camera(module, args):
                camera_events['end'].record(torch.cuda.current_stream())
                observed['camera_neck_output'] = args[0].detach().clone()
            def capture_heatmap(module, args, output):
                observed['heatmap'] = output.detach().clone()
            def capture_queries(module, args, kwargs):
                observed['query_positions'] = kwargs['query_pos'].detach().clone()
                observed['query_labels'] = runtime.model.bbox_head.query_labels.detach().clone()
            handles = [runtime.model.img_backbone.register_forward_pre_hook(begin_camera),
                runtime.model.view_transform.register_forward_pre_hook(capture_camera),
                runtime.model.bbox_head.heatmap_head.register_forward_hook(capture_heatmap),
                runtime.model.bbox_head.decoder[0].register_forward_pre_hook(capture_queries, with_kwargs=True)]
            for i, sample in enumerate(samples):
                observed.clear()
                camera_events.update(begin=torch.cuda.Event(enable_timing=True),
                                     end=torch.cuda.Event(enable_timing=True))
                runtime.infer(sample['points'], sample['images'], i, 0., 200)
                torch.cuda.synchronize()
                camera_spans.append(camera_events['begin'].elapsed_time(camera_events['end']))
                activations[sample['id']] = {k: v.float().cpu().numpy() for k, v in observed.items()}
            for handle in handles: handle.remove()
            handles = []
            if reference_predictions is None:
                reference_predictions, reference_activations = predictions, activations
            activation_checks = []
            for sample in samples:
                token = sample['id']; old = reference_activations[token]; new = activations[token]
                old_queries = {(int(l), *map(float, xy)) for l, xy in zip(old['query_labels'].reshape(-1), old['query_positions'].reshape(-1, 2))}
                new_queries = {(int(l), *map(float, xy)) for l, xy in zip(new['query_labels'].reshape(-1), new['query_positions'].reshape(-1, 2))}
                activation_checks.append(dict(id=token,
                    camera_neck_output=tensor_difference(old['camera_neck_output'], new['camera_neck_output']),
                    heatmap=tensor_difference(old['heatmap'], new['heatmap']),
                    reference_queries=len(old_queries), candidate_queries=len(new_queries),
                    shared_queries=len(old_queries & new_queries)))
            policies = dict(raw=lambda x: x, ros_ap_candidates=lambda x: ros_delivery(x, False),
                            ros_delivery=lambda x: ros_delivery(x, True))
            checks = {name: [dict(id=s['id'], **compare_detections(policy(reference_predictions[s['id']]),
                        policy(predictions[s['id']]), a.association_gate_m)) for s in samples]
                      for name, policy in policies.items()}
            for name, policy in policies.items():
                write_new(a.output/(mode+'_'+name+'_predictions.json'), {k: policy(v) for k, v in predictions.items()})
            case = dict(mode=mode, timing={key: distribution([r[key] for r in times]) for key in
                ('service_ms', 'preprocess_ms', 'model_ms', 'postprocess_ms')}, timing_rows=times,
                vram=peak, activation_checks=activation_checks,
                camera_instrumented_stream_span_ms=distribution(camera_spans),
                camera_span_note='Separate CUDA-event span from backbone entry to view-transform entry; includes neck, reshape and host idle. Not kernel self-time or uninstrumented service timing.',
                within_mode_repeated_raw_outputs=(aggregate_comparisons(within_mode_repeats)
                    if within_mode_repeats else dict(compared_frames=0, engineering_passed=None,
                                                     note='Only one timing repeat requested')),
                prediction_comparison={name: aggregate_comparisons(rows) for name, rows in checks.items()},
                per_frame_comparison=checks)
            result['cases'].append(case)
            write_new(a.output/(mode+'_RESULT.json'), case)
            print(json.dumps(dict(event='camera_trt_case_complete', mode=mode, service_ms=case['timing']['service_ms'],
                                  engineering_passed=case['prediction_comparison']['ros_delivery']['engineering_passed'])), flush=True)
        if pins != {path: sha256(path) for path in pins} or originals != {path: sha256(path) for path in originals}:
            raise RuntimeError('Source, checkpoint, engine, selection or input changed during qualification')
        result.update(complete=True, source_and_inputs_unchanged=True, finished_unix=time.time(),
            engineering_passed=all(
                case['prediction_comparison']['raw']['engineering_passed'] and
                case['prediction_comparison']['ros_delivery']['engineering_passed'] and
                case['within_mode_repeated_raw_outputs']['engineering_passed'] is not False
                for case in result['cases']),
            engineering_gate_scope='All decoded raw and delivered outputs, timing repeats, and eager restoration; no AP/tracker claim',
            qualification='Diagnostic only; no AP, continuous tracker or physical 24GB full-stack certification')
        write_new(a.output/'report.json', result)
        write_new(a.output/'COMPLETE.json', dict(complete=True, report_sha256=sha256(a.output/'report.json'),
                   engineering_passed=result['engineering_passed'], production_promoted=False))
        if a.require_decoded_parity and not result['engineering_passed']: raise SystemExit(2)
    except Exception as error:
        write_new(a.output/'FAILED.json', dict(complete=False, error=repr(error),
                  production_promoted=False, completed_cases=[c['mode'] for c in result['cases']]))
        raise
    finally:
        for handle in handles: handle.remove()
        if trt_handle is not None:
            trt_handle.disable(); trt_handle.close()
        if runtime is not None:
            if runtime.inference_optimization: runtime.inference_optimization.disable()
            runtime.preprocessor.close()


if __name__ == '__main__':
    main()
