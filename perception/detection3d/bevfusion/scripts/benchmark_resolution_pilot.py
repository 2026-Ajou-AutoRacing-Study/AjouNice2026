#!/usr/bin/env python3
"""Resolution wiring/latency pilot on one existing sample; NOT an AP test.

Run inside the team Docker with repository and morai_cap mounts. No training,
network server, ROS master, or simulator is started. E25 is kept unchanged.
"""
import argparse
import gc
import json
import pickle
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from app.bevfusion_model import BevFusionModel
from app.sensor4_calibration import load_sensor4_calibration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/workspace/AjouNice2026'))
    parser.add_argument('--iterations', type=int, default=12)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--configs', nargs='+', default=[
        'bevfusion_base3_b0_4cam_lidar.py', 'bevfusion_base5_highres_320x896_pilot.py'])
    parser.add_argument('--warmup-iterations', type=int, default=2)
    parser.add_argument('--max-points', type=int, default=0,
                        help='Reduced workload wiring smoke only, NOT production latency')
    args = parser.parse_args()
    if args.iterations < 3:
        parser.error('at least three iterations required')
    root = args.root
    payload = pickle.loads((root / 'artifacts/datasets/morai_base3_bevfusion_smoke_v1/morai_base3_infos_val.pkl').read_bytes())
    row = payload['data_list'][0]
    points = np.fromfile(row['lidar_points']['lidar_path'], dtype=np.float32).reshape(-1, 4)
    if args.max_points:
        points = points[::max(1, len(points) // args.max_points)][:args.max_points]
    images = {}
    for frame, key in [('camera_front', 'front'), ('camera_front_left', 'front_left'),
                       ('camera_front_right', 'front_right'), ('camera_back', 'back')]:
        image = cv2.imread(row['images'][key]['img_path'])
        if image is None:
            raise FileNotFoundError(row['images'][key]['img_path'])
        ok, blob = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        assert ok
        images['image/' + frame] = blob.tobytes()
    calibration = load_sensor4_calibration(root / 'configs/sensors/Sensor_competition_local.json')
    checkpoint = root / 'artifacts/detection3d/bevfusion/releases/base3_bf0_f40_e25/base3_bf0_f40_e25_inference.pth'
    report = dict(contract='Single existing sample, warmed up batch-1 AMP; no trained highres accuracy claim',
                  gpu=torch.cuda.get_device_name(), raw_points=len(points),
                  reduced_workload=bool(args.max_points), cases=[])
    for name in args.configs:
        torch.cuda.reset_peak_memory_stats()
        start = time.perf_counter()
        model = BevFusionModel(str(root / 'perception/detection3d/bevfusion/configs' / name),
                               str(checkpoint), calibration, warmup_iterations=args.warmup_iterations)
        times = []
        for i in range(args.iterations + 2):
            start_iter = time.perf_counter()
            model.infer(points, images, i, 0.1, 100)
            torch.cuda.synchronize()
            if i >= 2:
                times.append((time.perf_counter() - start_iter) * 1000)
        result = dict(config=name, input_hw=model.preprocessor.final_dim,
            point_cloud_range=model.preprocessor.point_cloud_range.tolist(),
            view_nx=model.model.view_transform.nx.detach().cpu().tolist(),
            view_bx=model.model.view_transform.bx.detach().cpu().tolist(),
            view_depth_bound=list(model.model.view_transform.dbound),
            elapsed_sec=time.perf_counter() - start,
            service_ms_mean=float(np.mean(times)), service_ms_p50=float(np.percentile(times, 50)),
            service_ms_p95=float(np.percentile(times, 95)),
            peak_allocated_mib=torch.cuda.max_memory_allocated() / 2**20,
            peak_reserved_mib=torch.cuda.max_memory_reserved() / 2**20)
        report['cases'].append(result)
        print(json.dumps(result), flush=True)
        model.preprocessor.close()
        del model
        gc.collect()
        torch.cuda.empty_cache()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
