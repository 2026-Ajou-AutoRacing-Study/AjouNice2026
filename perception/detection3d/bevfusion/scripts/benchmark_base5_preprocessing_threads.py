#!/usr/bin/env python3
"""Bounded CPU thread/layout cost probes with actual pixel/projection parity.

Run without a competing inference or training process. This never changes the
default preprocessor or dataset. Reversed second round reduces order bias.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from mmengine.config import Config
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from benchmark_base5_grid_depth import samples_from_selection
from benchmark_base5_input_optimization import summary


CASES = [(4,1,False), (1,1,False), (1,4,False), (2,1,False),
         (2,2,False), (4,2,False), (4,4,False), (4,1,True)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'selection', 'config', 'sensor-config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--samples', type=int, default=128)
    a = p.parse_args()
    if a.output.exists() or not 32 <= a.samples <= 300:
        p.error('Fresh output and 32..300 distinct inputs required')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4)
    samples, source = samples_from_selection(a.data, a.selection)
    if len(samples) < a.samples:
        p.error('Do not repeat inputs to inflate the sample count')
    selected = [samples[i*(len(samples)-1)//(a.samples-1)] for i in range(a.samples)]
    cfg = Config.fromfile(str(a.config)); calibration = load_sensor4_calibration(a.sensor_config)
    options = preprocessing_options_from_config(cfg)
    reference = []
    cv2.setNumThreads(1)
    pre = Base3OnlinePreprocessor(calibration, camera_workers=4, **options)
    try:
        for sample in selected:
            reference.append(pre.prepare(sample['points'], sample['images']))
    finally:
        pre.close()
    result = dict(source=source, source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  distinct_inputs=len(selected), cpu_only=True, rounds=[])
    for order in (CASES, list(reversed(CASES))):
        for workers, threads, late in order:
            cv2.setNumThreads(threads)
            pre = Base3OnlinePreprocessor(calibration, camera_workers=workers,
                                         rgb_after_crop=late, **options)
            try:
                mismatch = None
                for sample, old in zip(selected, reference):
                    new = pre.prepare(sample['points'], sample['images'])
                    for key in ('img','points'):
                        if not torch.equal(old['inputs'][key][0], new['inputs'][key][0]):
                            x,y = old['inputs'][key][0],new['inputs'][key][0]
                            mismatch = dict(id=sample['id'], kind=key,
                                changed_values=int((x!=y).sum()),
                                max_abs=float((x.float()-y.float()).abs().max()))
                    for key in ('cam2img','lidar2cam','cam2lidar','lidar2img','img_aug_matrix'):
                        if not np.array_equal(old['data_samples'][0].metainfo[key], new['data_samples'][0].metainfo[key]):
                            mismatch = dict(id=sample['id'],kind=key)
                    if mismatch: break
                if mismatch:
                    case = dict(camera_workers=workers, opencv_threads=threads, rgb_after_crop=late,
                                pixel_points_projection_equal=False, mismatch=mismatch)
                    result['rounds'].append(case)
                    (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
                    print('REJECT '+json.dumps(case),flush=True)
                    continue
                for sample in selected[:16]: pre.prepare(sample['points'], sample['images'])
                values = []
                for sample in selected:
                    start = time.perf_counter(); pre.prepare(sample['points'], sample['images'])
                    values.append((time.perf_counter()-start)*1000)
                case = dict(camera_workers=workers, opencv_threads=threads, rgb_after_crop=late,
                            pixel_points_projection_equal=True, timing_ms=summary(values))
                result['rounds'].append(case)
                (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
                print(json.dumps(case),flush=True)
            finally:
                pre.close()


if __name__=='__main__': main()
