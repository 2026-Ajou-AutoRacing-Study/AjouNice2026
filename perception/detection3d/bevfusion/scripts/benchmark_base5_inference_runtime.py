#!/usr/bin/env python3
"""Matched native-JPEG Base5 localhost inference cost and prediction parity.

No training, simulator, remote access, AP claim or production promotion. All
variants load one trusted local FULL checkpoint. Fresh output directories only.
"""
import argparse
from copy import deepcopy
import gc
import hashlib
import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
import torch
from mmengine.config import Config

from app.inference_server import InferenceServer
from app.protocol import TYPE_REQUEST, TYPE_RESPONSE, recv_message, send_message
from benchmark_base5_grid_depth import samples_from_selection
from benchmark_base5_input_optimization import summary
from profile_base5_stages import StageRecorder
from base5_grid_candidates import candidate_config


VARIANTS = {"baseline": ((), False), "camera": (("camera",), False),
            "camera_neck": (("camera", "neck"), False),
            "camera_bev": (("camera", "bev"), False),
            "fold": ((), True), "camera_fold": (("camera",), True),
            "graph": ((), False), "graph_neck": (("neck",), False)}
VARIANTS['camera_guarded'] = (("camera",), False)
VARIANTS['rgb'] = ((), False)
VARIANTS['camera_guarded_rgb'] = (("camera",), False)
VARIANTS['camera_layout'] = (("camera",), False)
VARIANTS['parallel_camera'] = ((), False)
VARIANTS['camera_neck_parallel'] = (("camera", "neck"), False)


def parity(reference, candidate):
    """Class-preserving bijection; do not hide lost or additional predictions.

    Fixed engineering gates for AMP execution equivalence, NOT accuracy gates:
    center/size 5cm, yaw .02rad, score .002. All emitted boxes are compared.
    """
    if len(reference) != len(candidate):
        return dict(passed=False, reason="prediction count differs")
    if any(d['label'] not in range(4) for d in (*reference, *candidate)):
        return dict(passed=False, reason="unknown class label")
    maxima = dict(center_m=0., size_m=0., yaw_rad=0., score=0.)
    for label in range(4):
        old = [d for d in reference if d['label'] == label]
        new = [d for d in candidate if d['label'] == label]
        if len(old) != len(new):
            return dict(passed=False, reason="class count differs")
        if not old:
            continue
        a = np.asarray([d['center'] for d in old]); b = np.asarray([d['center'] for d in new])
        rows, cols = linear_sum_assignment(np.linalg.norm(a[:, None] - b[None], axis=-1))
        for i, j in zip(rows, cols):
            x, y = old[i], new[j]
            maxima['center_m'] = max(maxima['center_m'], float(np.linalg.norm(a[i] - b[j])))
            maxima['size_m'] = max(maxima['size_m'], float(np.max(np.abs(np.asarray(x['size']) - y['size']))))
            maxima['yaw_rad'] = max(maxima['yaw_rad'], abs(float(np.arctan2(
                np.sin(x['yaw'] - y['yaw']), np.cos(x['yaw'] - y['yaw'])))))
            maxima['score'] = max(maxima['score'], abs(x['score'] - y['score']))
            assert x['class_name'] == y['class_name']
    limits = dict(center_m=.05, size_m=.05, yaw_rad=.02, score=.002)
    return dict(passed=all(maxima[k] <= limits[k] for k in limits), maxima=maxima, limits=limits)


def run_variant(args, config, samples, variant, reference):
    gc.collect(); torch.cuda.empty_cache(); torch.cuda.reset_peak_memory_stats()
    parts, fold = VARIANTS[variant]
    runtime_args = SimpleNamespace(sensor_config=str(args.sensor_config), camera_order=None,
        intrinsic_source="horizontal_fov", projection_only=False, config=str(config),
        checkpoint=str(args.checkpoint), device="cuda:0", amp=True, preprocess_workers=args.camera_workers,
        warmup_iterations=2, temporal_feature_cache=False, temporal_maximum_age_sec=.3,
        base5_runtime=True, compile_parts=parts, fuse_dense_bn=fold,
        camera_cuda_graph=variant.startswith('graph'),
        eager_camera_numerics=variant.startswith('camera_guarded'),
        rgb_after_crop=variant.endswith('rgb'),
        compile_reference_layout=variant == 'camera_layout')
    runtime_args.parallel_camera = variant in ('parallel_camera', 'camera_neck_parallel')
    start = time.perf_counter(); server = InferenceServer(runtime_args)
    cold_sec = time.perf_counter() - start
    ready = threading.Event(); address = {}; errors = []

    def worker():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.bind(('127.0.0.1', 0)); listener.listen(1)
                address['port'] = listener.getsockname()[1]; ready.set()
                connection, _ = listener.accept()
                with connection:
                    connection.settimeout(300)
                    server._serve_client(connection)
        except Exception as exc:
            errors.append(repr(exc)); ready.set()

    thread = threading.Thread(target=worker, daemon=True); thread.start()
    if not ready.wait(10) or errors:
        raise RuntimeError("localhost worker failed: " + repr(errors))
    predictions = {}; rows = []; checks = []; startup_peak = None
    calibration = server.calibration
    try:
        with socket.create_connection(('127.0.0.1', address['port']), timeout=300) as connection:
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            metadata = dict(operation='inference', sensor_config_sha256=calibration['config_sha256'],
                camera_order=[c['frame_id'] for c in calibration['cameras']],
                intrinsic_source=calibration['intrinsic_source'], point_dim=4,
                score_threshold=0., max_detections=200)
            for i in range(16 + args.iterations):
                sample = samples[(i - 16) % len(samples)] if i >= 16 else samples[i % len(samples)]
                blobs = dict(sample['images'], points=sample['points'].tobytes())
                request = dict(metadata, sequence=i, point_count=len(sample['points']))
                begin = time.perf_counter(); send_message(connection, TYPE_REQUEST, request, blobs)
                kind, response, _ = recv_message(connection); elapsed = (time.perf_counter() - begin) * 1000
                if kind != TYPE_RESPONSE:
                    raise RuntimeError(repr(response))
                detections = response['detections']
                for detection in detections:
                    assert 0 <= detection['label'] < 4
                    assert detection['class_name'] == ('vehicle', 'pedestrian', 'obstacle', 'animal')[detection['label']]
                    assert len(detection['center']) == 3 and len(detection['size']) == 3
                    assert detection['velocity'] == [0., 0.]
                    assert np.isfinite([*detection['center'], *detection['size'], detection['yaw'], detection['score']]).all()
                if i < 16:
                    if i == 15:
                        startup_peak = torch.cuda.max_memory_allocated() / 2**20
                        torch.cuda.reset_peak_memory_stats()
                    continue
                predictions[sample['id']] = detections
                rows.append(dict(id=sample['id'], group=sample['group'], rtt_ms=elapsed,
                    **{k:response[k] for k in ('preprocess_ms', 'inference_ms', 'postprocess_ms', 'server_ms')}))
                if reference is not None:
                    checks.append(dict(id=sample['id'], **parity(reference[sample['id']], detections)))
                if len(rows) % 50 == 0:
                    print(f'{variant} {len(rows)}/{args.iterations}', flush=True)
        thread.join(10)
        if thread.is_alive() or errors:
            raise RuntimeError('worker did not finish: ' + repr(errors))
        peak = torch.cuda.max_memory_allocated() / 2**20
        # Separate instrumentation from uninstrumented serial request latency.
        rec = StageRecorder(); original = {}; spans = {}
        model = server.model.model
        stages = [(model.img_backbone, 'forward', 'camera'), (model.img_neck, 'forward', 'camera_neck'),
            (model.pts_middle_encoder, 'forward', 'lidar_sparse'), (model, 'voxelize', 'voxelize'),
            (model.data_preprocessor, 'forward', 'transfer_normalize'),
            (model.view_transform, 'forward', 'view_total'),
            (model.view_transform, 'get_cam_feats', 'depth_feature'),
            (model.view_transform, 'get_geometry', 'geometry'),
            (model.view_transform, 'bev_pool', 'pool'),
            (model.fusion_layer, 'forward', 'fusion'), (model.pts_backbone, 'forward', 'bev_backbone'),
            (model.pts_neck, 'forward', 'bev_neck'), (model.bbox_head, 'predict', 'head_decode')]
        try:
            for module, method, label in stages:
                original[(module, method)] = getattr(module, method); rec.wrap(module, method, label)
            rec.enabled = True
            for i in range(args.stage_iterations):
                sample = samples[(i * 7) % len(samples)]; rec.records = []
                server.model.infer(sample['points'], sample['images'], i, 0., 200)
                for name, begin, end, _ in rec.records:
                    spans.setdefault(name, []).append(begin.elapsed_time(end))
        finally:
            for (module, method), fn in original.items():
                setattr(module, method, fn)
        groups = {}
        for group in ('all', 'representative', 'stress'):
            selected = rows if group == 'all' else [r for r in rows if r['group'] == group]
            if selected:
                groups[group] = dict(count=len(selected), distinct=len({r['id'] for r in selected}),
                    timing={k:summary([r[k] for r in selected]) for k in
                            ('preprocess_ms', 'inference_ms', 'postprocess_ms', 'server_ms', 'rtt_ms')})
        optimization = server.model.inference_optimization
        report = dict(variant=variant, cold_load_warmup_sec=cold_sec, timing=groups,
            prediction_counts=dict(total=sum(map(len,predictions.values())),
                nonempty_frames=sum(bool(v) for v in predictions.values()),
                minimum=min(map(len,predictions.values())), maximum=max(map(len,predictions.values()))),
            peak_allocated_mib=peak, optimization=optimization.report if optimization else {},
            startup_peak_allocated_mib=startup_peak,
            stage_stream_ms={k:summary(v) for k, v in spans.items()},
            stage_note='Separate CUDA-event critical path spans include host idle; not kernel self-time.',
            parity=dict(compared=len(checks), passed=all(c['passed'] for c in checks), checks=checks,
                        nonvacuous=bool(sum(map(len,predictions.values())))),
            per_sample=rows)
        return report, predictions
    finally:
        server.model.preprocessor.close()
        if server.model.inference_optimization:
            server.model.inference_optimization.disable()
        del server; gc.collect(); torch.cuda.empty_cache()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('data', 'selection', 'config', 'checkpoint', 'sensor-config', 'output'):
        parser.add_argument('--' + key, type=Path, required=True)
    parser.add_argument('--depth-step', type=float, default=.25)
    parser.add_argument('--grid', choices=('current075', 'common050'), default='current075')
    parser.add_argument('--variants', nargs='+', choices=tuple(VARIANTS), default=['baseline', 'camera'])
    parser.add_argument('--iterations', type=int, default=300)
    parser.add_argument('--stage-iterations', type=int, default=16)
    parser.add_argument('--require-parity', action='store_true', help='Fail if any decoded prediction gate fails')
    parser.add_argument('--camera-workers', type=int, choices=[1,2,4], default=4)
    parser.add_argument('--opencv-threads', type=int, choices=[1,2,4], default=1)
    parser.add_argument('--cpu-threads', type=int, choices=[1,2,4,8], default=4)
    parser.add_argument('--memory-budget-gib', type=float,
                        help='Optional allocator cap for a 24GB deployment compatibility probe')
    args = parser.parse_args()
    if args.output.exists(): parser.error('Fresh output required')
    if not 24 <= args.iterations <= 300: parser.error('Bounded 24..300 requests required')
    if not 8 <= args.stage_iterations <= 32: parser.error('Bounded 8..32 stage probes required')
    if args.variants[0] != 'baseline': parser.error('Baseline must run first for matched parity')
    args.output.mkdir(parents=True)
    torch.set_num_threads(args.cpu_threads); cv2.setNumThreads(args.opencv_threads)
    torch.backends.cudnn.benchmark = True; torch.backends.cuda.matmul.allow_tf32 = True
    fraction = .85
    if args.memory_budget_gib is not None:
        fraction = args.memory_budget_gib * 2**30 / torch.cuda.get_device_properties(0).total_memory
        if not 0 < fraction <= .85: parser.error('Memory budget must fit within the 85% safety limit')
    torch.cuda.set_per_process_memory_fraction(fraction)
    base = Config.fromfile(str(args.config))
    cfg = candidate_config(base, args.grid, args.depth_step, 512)
    config = args.output / 'runtime_config.py'; cfg.dump(str(config))
    samples, source = samples_from_selection(args.data, args.selection)
    if len(samples) < args.iterations: parser.error('Do not repeat inputs to inflate count')
    repo = Path(__file__).resolve().parents[4]
    paths = [Path(__file__), *[repo / 'docker/bevfusion/app' / name for name in
             ('inference_optimization.py', 'inference_parallel.py', 'bevfusion_model.py', 'inference_server.py', 'online_preprocessor.py')],
             repo / 'perception/detection3d/bevfusion/plugin/morai_pool_optimization.py', config, args.checkpoint]
    identity = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    result = dict(gpu=torch.cuda.get_device_name(), torch=str(torch.__version__), source=source,
        runtime_threads=dict(camera_workers=args.camera_workers, opencv=args.opencv_threads,
                             torch=args.cpu_threads),
        allocator_budget_gib=fraction*torch.cuda.get_device_properties(0).total_memory/2**30,
        identity=identity, cases=[], scope='Matched trusted full-model checkpoint, NOT an AP evaluation. Batch1 AMP native JPEG; localhost TCP RTT is not ROS/LAN full-stack latency.')
    reference = None
    for variant in args.variants:
        report, predictions = run_variant(args, config, samples, variant, reference)
        if reference is None: reference = deepcopy(predictions)
        result['cases'].append(report)
        (args.output / (str(len(result['cases'])) + '_' + variant + '_predictions.json')).write_text(json.dumps(predictions) + '\n')
        (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        print('RESULT ' + json.dumps({k:v for k,v in report.items() if k not in ('per_sample', 'parity')}), flush=True)
    assert identity == {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}, 'Source changed mid-probe'
    print('PARITY ' + json.dumps([dict(variant=c['variant'], prediction_counts=c['prediction_counts'],
        **{k:v for k,v in c['parity'].items() if k != 'checks'})
        for c in result['cases']]), flush=True)
    if args.require_parity and any(not c['parity']['passed'] or not c['parity']['nonvacuous'] for c in result['cases']):
        raise SystemExit(2)


if __name__ == '__main__': main()
