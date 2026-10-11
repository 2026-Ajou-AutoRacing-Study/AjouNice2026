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


def tensor_difference(before, after):
    a, b = before.float(), after.float()
    return dict(max_abs=float((a-b).abs().max()),
                relative_l2=float((a-b).norm()/a.norm().clamp_min(1e-12)))


def query_set(snapshot):
    labels, positions = snapshot['query_labels'], snapshot['query_positions']
    return {(int(label), float(x), float(y)) for label, (x,y) in
            zip(labels.flatten().tolist(), positions.reshape(-1,2).tolist())}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'selection', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--samples', type=int, default=32)
    p.add_argument('--parts', nargs='+', choices=['camera', 'neck', 'bev'], default=['camera'])
    p.add_argument('--eager-camera-numerics', action='store_true')
    p.add_argument('--reference-layout', action='store_true')
    p.add_argument('--depth-step', type=float, choices=[.25, .5], default=.25)
    p.add_argument('--grid', choices=['current075', 'common050'], default='current075')
    p.add_argument('--allow-tf32', action='store_true')
    p.add_argument('--cudnn-benchmark', action='store_true')
    p.add_argument('--memory-budget-gib', type=float)
    p.add_argument('--query-control', action='store_true',
                   help='Diagnostic only: force the reference query heatmap, never a deployment gate')
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output required')
    if not 8 <= a.samples <= 64: p.error('Bounded 8..64 actual samples')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    torch.backends.cuda.matmul.allow_tf32 = a.allow_tf32
    torch.backends.cudnn.benchmark = a.cudnn_benchmark
    fraction = .85
    if a.memory_budget_gib is not None:
        fraction = a.memory_budget_gib * 2**30 / torch.cuda.get_device_properties(0).total_memory
        if not 0 < fraction <= .85: p.error('Memory budget must fit within the 85% safety limit')
    torch.cuda.set_per_process_memory_fraction(fraction)
    cfg = candidate_config(Config.fromfile(str(a.config)), a.grid, a.depth_step, 512)
    path = a.output / 'config.py'; cfg.dump(str(path))
    runtime = BevFusionModel(path, a.checkpoint, load_sensor4_calibration(a.sensor_config),
                             base5_runtime=True)
    samples, source = samples_from_selection(a.data, a.selection)
    selected = [samples[i * (len(samples)-1) // (a.samples-1)] for i in range(a.samples)]
    captured = []; handle = runtime.model.view_transform.dtransform.register_forward_pre_hook(
        lambda module, inputs: captured.append(inputs[0].detach().clone()))
    observed = {}
    camera_hook = runtime.model.img_backbone.register_forward_hook(
        lambda module, inputs, output: observed.update(camera=[v.detach().cpu().clone() for v in output]))
    heatmap_hook = runtime.model.bbox_head.heatmap_head.register_forward_hook(
        lambda module, inputs, output: observed.update(heatmap=output.detach().cpu().clone()))
    def capture_queries(module, inputs, kwargs):
        observed.update(query_positions=kwargs['query_pos'].detach().cpu().clone(),
                        query_labels=runtime.model.bbox_head.query_labels.detach().cpu().clone())
    query_hook = runtime.model.bbox_head.decoder[0].register_forward_pre_hook(
        capture_queries, with_kwargs=True)
    result = dict(source=source, scope='One full checkpoint, frozen raster control only; not a deployment policy or AP test',
                  checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
                  compile_parts=a.parts, eager_camera_numerics=a.eager_camera_numerics,
                  reference_layout=a.reference_layout, depth_step=a.depth_step, grid=a.grid,
                  allow_tf32=a.allow_tf32, cudnn_benchmark=a.cudnn_benchmark,
                  gpu=torch.cuda.get_device_name(), torch_version=str(torch.__version__),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  resolved_config_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  allocator_budget_gib=fraction*torch.cuda.get_device_properties(0).total_memory/2**30,
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
        baselines = []; rasters = []; intermediates = []
        for i, sample in enumerate(selected):
            captured.clear()
            baselines.append(runtime.infer(sample['points'], sample['images'], i, 0., 200)[0])
            rasters.append(captured[0].cpu())
            intermediates.append(dict(observed))
        handle.remove(); handle = None
        cases = [('repeat_eager', (), False), ('compiled', a.parts, False), ('folded', (), True)]
        if a.query_control: cases.append(('compiled_query_control', a.parts, False))
        for name, parts, fold in cases:
            optimization = InferenceOptimization(runtime.model, parts, fold,
                eager_camera_numerics=(a.eager_camera_numerics and name.startswith('compiled')),
                reference_layout=a.reference_layout and name.startswith('compiled')) if parts or fold else None
            checks = []
            try:
                for i, sample in enumerate(selected):
                    raster = rasters[i].cuda()
                    # Same saved LiDAR raster for both paths. Never changes
                    # image pixels, LiDAR backbone input, weights or metadata.
                    hook = runtime.model.view_transform.dtransform.register_forward_pre_hook(
                        lambda module, inputs, raster=raster: (raster,))
                    query_override = None
                    if name == 'compiled_query_control':
                        frozen_heatmap = intermediates[i]['heatmap'].cuda()
                        query_override = runtime.model.bbox_head.heatmap_head.register_forward_hook(
                            lambda module, inputs, output, value=frozen_heatmap: value)
                    try:
                        prediction = runtime.infer(sample['points'], sample['images'], i, 0., 200)[0]
                    finally:
                        hook.remove()
                        if query_override: query_override.remove()
                    before = intermediates[i]
                    old_query, new_query = query_set(before), query_set(observed)
                    query_common = len(old_query & new_query)
                    checks.append(dict(id=sample['id'], **parity(baselines[i], prediction),
                        camera_features=[tensor_difference(x,y) for x,y in zip(before['camera'], observed['camera'])],
                        dense_heatmap=tensor_difference(before['heatmap'], observed['heatmap']),
                        original_query_count=len(old_query), shared_query_count=query_common))
                result['cases'].append(dict(name=name, passed=all(c['passed'] for c in checks), checks=checks,
                    nonvacuous=bool(sum(map(len,baselines))),
                    diagnostic_query_override=name == 'compiled_query_control',
                    baseline_prediction_count=sum(map(len,baselines))))
                (a.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
                print(name, sum(not c['passed'] for c in checks), 'failed /', len(checks), flush=True)
            finally:
                if optimization: optimization.disable()
    finally:
        if handle: handle.remove()
        camera_hook.remove(); heatmap_hook.remove(); query_hook.remove()
        runtime.preprocessor.close()


if __name__ == '__main__': main()
