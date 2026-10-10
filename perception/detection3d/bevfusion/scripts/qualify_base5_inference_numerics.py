#!/usr/bin/env python3
"""Isolate actual sparse-depth raster races from compiler arithmetic changes.

Frozen raster is a numerical CONTROL only, never an inference cache across
frames. Uses one loaded model and unchanged actual native-JPEG sensor inputs.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import torch
from mmengine.config import Config
from app.bevfusion_model import BevFusionModel
from app.inference_optimization import InferenceOptimization
from app.sensor4_calibration import load_sensor4_calibration
from base5_grid_candidates import candidate_config
from benchmark_base5_grid_depth import samples_from_selection
from benchmark_base5_inference_runtime import parity


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'selection', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--samples', type=int, default=32)
    p.add_argument('--parts', nargs='+', choices=['camera', 'neck', 'bev'], default=['camera'])
    p.add_argument('--eager-camera-numerics', action='store_true')
    p.add_argument('--reference-layout', action='store_true')
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output required')
    if not 8 <= a.samples <= 64: p.error('Bounded 8..64 actual samples')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    torch.cuda.set_per_process_memory_fraction(.85)
    cfg = candidate_config(Config.fromfile(str(a.config)), 'current075', .25, 512)
    path = a.output / 'config.py'; cfg.dump(str(path))
    runtime = BevFusionModel(path, a.checkpoint, load_sensor4_calibration(a.sensor_config),
                             base5_runtime=True)
    samples, source = samples_from_selection(a.data, a.selection)
    selected = [samples[i * (len(samples)-1) // (a.samples-1)] for i in range(a.samples)]
    captured = []; handle = runtime.model.view_transform.dtransform.register_forward_pre_hook(
        lambda module, inputs: captured.append(inputs[0].detach().clone()))
    result = dict(source=source, scope='One full checkpoint, frozen raster control only; not a deployment policy or AP test',
                  checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
                  compile_parts=a.parts, eager_camera_numerics=a.eager_camera_numerics,
                  reference_layout=a.reference_layout,
                  repeated_raster=[], cases=[])
    try:
        sample = selected[0]
        repeats = []
        for i in range(8):
            captured.clear()
            repeats.append(runtime.infer(sample['points'], sample['images'], i, 0., 200)[0])
            depth = captured[0].cpu()
            if i == 0: reference_depth = depth
            result['repeated_raster'].append(dict(iteration=i,
                changed_pixels=int((reference_depth != depth).sum()),
                maximum_depth_difference_m=float((reference_depth-depth).abs().max()),
                prediction_parity=parity(repeats[0], repeats[-1])))
        baselines = []; rasters = []
        for i, sample in enumerate(selected):
            captured.clear()
            baselines.append(runtime.infer(sample['points'], sample['images'], i, 0., 200)[0])
            rasters.append(captured[0].cpu())
        handle.remove(); handle = None
        for name, parts, fold in [('repeat_eager', (), False), ('compiled', a.parts, False), ('folded', (), True)]:
            optimization = InferenceOptimization(runtime.model, parts, fold,
                eager_camera_numerics=(a.eager_camera_numerics and name == 'compiled'),
                reference_layout=a.reference_layout and name == 'compiled') if parts or fold else None
            checks = []
            try:
                for i, sample in enumerate(selected):
                    raster = rasters[i].cuda()
                    # Same saved LiDAR raster for both paths. Never changes
                    # image pixels, LiDAR backbone input, weights or metadata.
                    hook = runtime.model.view_transform.dtransform.register_forward_pre_hook(
                        lambda module, inputs, raster=raster: (raster,))
                    try:
                        prediction = runtime.infer(sample['points'], sample['images'], i, 0., 200)[0]
                    finally: hook.remove()
                    checks.append(dict(id=sample['id'], **parity(baselines[i], prediction)))
                result['cases'].append(dict(name=name, passed=all(c['passed'] for c in checks), checks=checks))
                (a.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
                print(name, sum(not c['passed'] for c in checks), 'failed /', len(checks), flush=True)
            finally:
                if optimization: optimization.disable()
    finally:
        if handle: handle.remove()
        runtime.preprocessor.close()


if __name__ == '__main__': main()
