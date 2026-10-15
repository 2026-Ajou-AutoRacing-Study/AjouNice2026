#!/usr/bin/env python3
"""Same-loaded-model parallel branch correctness and uninstrumented cost.

No training or production promotion. Optional frozen depth raster is a
numerical control, never a cross-frame inference cache or deployment gate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import torch
from mmengine.config import Config
from app.bevfusion_model import BevFusionModel
from app.inference_optimization import InferenceOptimization
from app.inference_parallel import ParallelBranches
from app.sensor4_calibration import load_sensor4_calibration
from base5_grid_candidates import candidate_config
from benchmark_base5_grid_depth import samples_from_selection
from benchmark_base5_inference_runtime import parity
from benchmark_base5_input_optimization import summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('data', 'selection', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--samples', type=int, default=64)
    p.add_argument('--mode', choices=['legacy', 'voxel_main', 'camera_side'], default='camera_side')
    p.add_argument('--compile-camera', action='store_true')
    p.add_argument('--grid', choices=['current075','common050'],default='current075')
    p.add_argument('--frozen-raster-control', action='store_true')
    p.add_argument('--sync-hash-read-diagnostic', action='store_true',
                   help='Diagnosis only: compare spconv count with a synchronized GPU read')
    a = p.parse_args()
    if a.output.exists() or not 8 <= a.samples <= 300:
        p.error('Fresh output and bounded 8..300 distinct inputs required')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    torch.cuda.set_per_process_memory_fraction(min(.85,20*2**30/torch.cuda.get_device_properties(0).total_memory))
    cfg = candidate_config(Config.fromfile(str(a.config)), a.grid, .25, 512)
    config = a.output/'config.py'
    cfg.dump(str(config))
    root = Path(__file__).resolve().parents[4]
    paths = [Path(__file__), config, *[root/'docker/bevfusion/app'/name for name in
             ('bevfusion_model.py','inference_parallel.py','inference_optimization.py')]]
    identity = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    result = dict(mode=a.mode, compile_camera=a.compile_camera, frozen_raster_control=a.frozen_raster_control,
                  grid=a.grid,
                  source_identity=identity,
                  gpu=torch.cuda.get_device_name(), torch=str(torch.__version__), cases=[],
                  checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
                  scope='Same model, native JPEG batch1, cost fixture; not AP or ROS/LAN latency')
    hash_original = None
    if a.sync_hash_read_diagnostic:
        from spconv.pytorch import ops
        hash_original = ops.SpconvOps
        result['hash_count_mismatches'] = []
        class CountReadControl:
            def __getattr__(self, name):
                return getattr(hash_original, name)
            def unique_hash(self, *args, **kwargs):
                count = hash_original.unique_hash(*args, **kwargs)
                torch.cuda.current_stream().synchronize()
                actual = int(args[2].cpu().numpy()[0])
                bound = int(args[4])
                corrected = min(actual, bound if bound > 0 else args[3].dim(0))
                if count != corrected:
                    row = dict(original=int(count), synchronized=corrected)
                    result['hash_count_mismatches'].append(row)
                    print('HASH_COUNT '+json.dumps(row),flush=True)
                return corrected
        ops.SpconvOps = CountReadControl()
    runtime = BevFusionModel(config, a.checkpoint, load_sensor4_calibration(a.sensor_config),
                             base5_runtime=True, warmup_iterations=0)
    branch = ParallelBranches(runtime.model, a.mode)
    compiler = None
    try:
        # Reproduce the historical failure condition BEFORE a serial sparse call.
        runtime.warmup_iterations = 2
        runtime._warm_up_before_ready()
        result['cold_parallel_completed'] = True
        branch.disable()
        samples, source = samples_from_selection(a.data, a.selection)
        if len(samples) < a.samples:
            raise ValueError('Do not repeat inputs to inflate the sample count')
        result['source'] = source
        selected = [samples[i*(len(samples)-1)//(a.samples-1)] for i in range(a.samples)]
        if a.compile_camera:
            compiler = InferenceOptimization(runtime.model, ('camera','neck'))
        rasters, capture = [], []
        if a.frozen_raster_control:
            handle = runtime.model.view_transform.dtransform.register_forward_pre_hook(
                lambda module, args: capture.append(args[0].detach().clone()))
            for i, sample in enumerate(selected):
                capture.clear()
                runtime.infer(sample['points'],sample['images'],i,0.,200)
                rasters.append(capture[0])
            handle.remove()
        baseline = None
        for mode in ('serial', 'parallel', 'serial_repeat', 'parallel_repeat'):
            if mode.startswith('parallel'):
                branch = ParallelBranches(runtime.model,a.mode)
            timings, predictions = [], []
            for i in range(8):
                sample = selected[i % len(selected)]
                runtime.infer(sample['points'],sample['images'],i,0.,200)
            for i, sample in enumerate(selected):
                handle = None
                if rasters:
                    handle = runtime.model.view_transform.dtransform.register_forward_pre_hook(
                        lambda module, args, i=i: (rasters[i],))
                start = time.perf_counter()
                try:
                    prediction, *_ = runtime.infer(sample['points'],sample['images'],i,0.,200)
                finally:
                    if handle is not None:
                        handle.remove()
                timings.append((time.perf_counter()-start)*1000)
                predictions.append(prediction)
            checks = [parity(x,y) for x,y in zip(baseline or predictions,predictions)]
            if baseline is None:
                baseline = predictions
            row = dict(mode=mode, count=len(predictions), timing_ms=summary(timings),
                       total_predictions=sum(map(len,predictions)),
                       nonempty_frames=sum(bool(v) for v in predictions),
                       decoded_gate_passed=bool(sum(map(len,predictions))) and all(c['passed'] for c in checks),
                       parity_passed=sum(c['passed'] for c in checks), checks=checks)
            result['cases'].append(row)
            (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
            print('CASE '+json.dumps({k:v for k,v in row.items() if k != 'checks'}),flush=True)
            if mode.startswith('parallel'):
                branch.disable()
        assert identity == {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}, 'Source changed during probe'
    except Exception as error:
        result['error'] = repr(error)
        (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        raise
    finally:
        branch.disable()
        if compiler:
            compiler.disable()
        if hash_original is not None:
            ops.SpconvOps = hash_original
        runtime.preprocessor.close()


if __name__ == '__main__':
    main()
