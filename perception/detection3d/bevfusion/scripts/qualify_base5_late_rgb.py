#!/usr/bin/env python3
"""CPU-only actual JPEG pixel parity and matched warmed preprocessing cost."""
import argparse
import json
from pathlib import Path
import time

import cv2
import torch
from mmengine.config import Config
from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from app.sensor4_calibration import load_sensor4_calibration
from benchmark_base5_grid_depth import samples_from_selection
from benchmark_base5_input_optimization import summary


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('data', 'selection', 'config', 'sensor-config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    a = p.parse_args()
    if a.output.exists(): p.error('Fresh output required')
    a.output.mkdir(parents=True)
    torch.set_num_threads(4); cv2.setNumThreads(1)
    samples, source = samples_from_selection(a.data, a.selection)
    calibration = load_sensor4_calibration(a.sensor_config)
    cfg = Config.fromfile(str(a.config))
    preprocessors = [Base3OnlinePreprocessor(calibration, rgb_after_crop=late,
        **preprocessing_options_from_config(cfg)) for late in (False, True)]
    result = dict(source=source, pixel_equal=True, compared_cameras=0, rounds=[])
    try:
        for i, sample in enumerate(samples):
            old, new = [pre.prepare(sample['points'], sample['images'], i) for pre in preprocessors]
            assert torch.equal(old['inputs']['img'][0], new['inputs']['img'][0]), 'Pixel mismatch: ' + sample['id']
            assert torch.equal(old['inputs']['points'][0], new['inputs']['points'][0])
            result['compared_cameras'] += 4
        for order in ((0, 1), (1, 0)):
            for index in order:
                pre = preprocessors[index]
                for sample in samples[:16]: pre.prepare(sample['points'], sample['images'])
                values = []
                for sample in samples:
                    start = time.perf_counter(); pre.prepare(sample['points'], sample['images'])
                    values.append((time.perf_counter() - start) * 1000)
                result['rounds'].append(dict(late_rgb=bool(index), count=len(samples), timing_ms=summary(values)))
        (a.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        for pre in preprocessors: pre.close()


if __name__ == '__main__': main()
