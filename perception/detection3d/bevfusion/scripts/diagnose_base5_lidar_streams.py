#!/usr/bin/env python3
"""Bounded, inference-only stream isolation. Never change the model or weights."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import torch
from mmengine.config import Config
from app.bevfusion_model import BevFusionModel, build_warmup_points
from app.sensor4_calibration import load_sensor4_calibration
from base5_grid_candidates import candidate_config
from benchmark_base5_grid_depth import samples_from_selection


def difference(a, b):
    a, b = a.float(), b.float()
    return dict(equal=torch.equal(a, b), max_abs=float((a-b).abs().max()),
                relative_l2=float((a-b).norm()/a.norm().clamp_min(1e-12)))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('data', 'selection', 'config', 'checkpoint', 'sensor-config', 'output'):
        p.add_argument('--'+key, type=Path, required=True)
    p.add_argument('--samples', type=int, default=8)
    p.add_argument('--cold-side', action='store_true', help='Run sparse encoder first on the side stream, without hooks')
    a = p.parse_args()
    if a.output.exists() or not 2 <= a.samples <= 16:
        p.error('Fresh output and bounded 2..16 distinct inputs required')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4)
    cv2.setNumThreads(1)
    torch.cuda.set_per_process_memory_fraction(min(.85,20*2**30/torch.cuda.get_device_properties(0).total_memory))
    cfg = candidate_config(Config.fromfile(str(a.config)), 'current075', .25, 512)
    config = a.output/'config.py'
    cfg.dump(str(config))
    runtime = BevFusionModel(config, a.checkpoint, load_sensor4_calibration(a.sensor_config),
                             base5_runtime=True, warmup_iterations=0)
    samples, source = samples_from_selection(a.data, a.selection)
    if len(samples) < a.samples:
        runtime.preprocessor.close()
        raise ValueError('Do not repeat inputs to inflate the sample count')
    inputs = [('startup_synthetic', torch.from_numpy(build_warmup_points(4, 1)).cuda())]
    for i in range(a.samples):
        sample = samples[i*(len(samples)-1)//(a.samples-1)]
        batch = runtime.preprocessor.prepare(sample['points'], sample['images'], sample['id'])
        with torch.inference_mode():
            processed = runtime.model.data_preprocessor(batch, training=False)
        inputs.append((sample['id'], processed['inputs']['points'][0]))
    main_stream, side = torch.cuda.current_stream(), torch.cuda.Stream()
    result = dict(source=source, torch=str(torch.__version__), gpu=torch.cuda.get_device_name(),
                  checkpoint_sha256=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), samples=[])
    try:
        with torch.inference_mode():
            if a.cold_side:
                side.wait_stream(main_stream)
                with torch.cuda.stream(side), torch.autocast('cuda', dtype=torch.float16):
                    cold = runtime.model.extract_pts_feat(dict(points=[inputs[0][1]]))
                main_stream.wait_stream(side)
                torch.cuda.synchronize()
                result['cold_side'] = dict(completed=True, shape=list(cold.shape), finite=bool(cold.isfinite().all()))
                (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
                print('COLD_SIDE '+json.dumps(result['cold_side']),flush=True)
            for sample_id, points in inputs:
                torch.cuda.synchronize()
                with torch.autocast('cuda', enabled=False):
                    voxels = runtime.model.voxelize([points.float()])
                # A main-stream reference first removes lazy initialization as a confounder.
                with torch.autocast('cuda', dtype=torch.float16):
                    ref = runtime.model.extract_pts_feat(dict(points=[points])).clone()
                torch.cuda.synchronize()
                row = dict(id=sample_id, raw_points=len(points), voxel_count=len(voxels[1]),
                           coordinate_min=voxels[1].amin(0).cpu().tolist(),
                           coordinate_max=voxels[1].amax(0).cpu().tolist(), cases=[])
                for name in ('main_repeat', 'side_full', 'side_full_sync', 'side_sparse_only'):
                    captured = []
                    hook = runtime.model.pts_middle_encoder.register_forward_pre_hook(
                        lambda module, args: captured.append(args[1].detach().clone()))
                    start = time.perf_counter()
                    side.wait_stream(main_stream)
                    stream = main_stream if name == 'main_repeat' else side
                    try:
                        with torch.cuda.stream(stream), torch.autocast('cuda', dtype=torch.float16):
                            if name in ('main_repeat', 'side_full'):
                                out = runtime.model.extract_pts_feat(dict(points=[points]))
                            else:
                                if name == 'side_full_sync':
                                    with torch.autocast('cuda', enabled=False):
                                        feats, coords, sizes = runtime.model.voxelize([points.float()])
                                    torch.cuda.synchronize()
                                else:
                                    feats, coords, sizes = voxels
                                out = runtime.model.pts_middle_encoder(feats, coords, coords[-1,0]+1)
                        main_stream.wait_stream(stream)
                        torch.cuda.synchronize()
                        case = dict(name=name, completed=True, difference=difference(ref,out),
                                    elapsed_ms=(time.perf_counter()-start)*1000)
                    except Exception as error:
                        torch.cuda.synchronize()
                        case = dict(name=name, completed=False, error=repr(error))
                    finally:
                        hook.remove()
                    if captured:
                        case['coords_equal_to_reference'] = torch.equal(captured[0],voxels[1])
                        case['coords_min'] = captured[0].amin(0).cpu().tolist()
                        case['coords_max'] = captured[0].amax(0).cpu().tolist()
                    row['cases'].append(case)
                    print(json.dumps(dict(id=sample_id,**case)),flush=True)
                result['samples'].append(row)
                (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        runtime.preprocessor.close()


if __name__ == '__main__':
    main()
