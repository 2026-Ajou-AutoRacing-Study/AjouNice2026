#!/usr/bin/env python3
"""CPU same-photo packet grouping depth collision audit, no cloud rewrite.

Within each pair all policies retain baseline JPEGs and host reference stamp.
cut0_footer is the repeated cut0 geometry control, not a clock policy change.
Projection float32 CPU is not a CUDA depth-write repeatability certification.
"""

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
from experiment_base5_proposal_peaks import CAMERAS, bootstrap_source, digest, uniform_indices
from analyze_base5_proposal_peaks import distribution


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('inputs', 'output', 'sensor', 'config'):
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--pairs-per-case', type=int, default=20)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    bootstrap_source()
    import cv2
    import torch
    from mmengine.config import Config
    from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
    from app.sensor4_calibration import load_sensor4_calibration
    from perception.detection3d.bevfusion.depth_raster import project_positive_camera_z, reduce_depth_pixels
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True)
    manifest = json.loads(args.inputs.read_text())
    calibration = load_sensor4_calibration(args.sensor, ['camera_' + c for c in CAMERAS])
    cfg = Config.fromfile(str(args.config))
    preprocessor = Base3OnlinePreprocessor(calibration, camera_workers=4, point_intensity_source='ros255',
                                          **preprocessing_options_from_config(cfg))
    pairs = defaultdict(dict)
    case_pairs = defaultdict(list)
    for row in manifest['rows']:
        pairs[row['pair']][row['policy']] = row
        if row['policy'] == 'fixed51':
            case_pairs[row['case']].append(row['pair'])
    chosen = [values[i] for values in case_pairs.values() for i in uniform_indices(len(values), args.pairs_per_case)]
    results = []
    try:
        for sequence, pair in enumerate(chosen):
            group = pairs[pair]
            reference = group['fixed51']
            images = {key: Path(path).read_bytes() for key,path in reference['images'].items()}
            photo_hashes = {key: hashlib.sha256(blob).hexdigest() for key,blob in images.items()}
            baseline_meta = None
            for policy in ('fixed51', 'fixed50', 'cut0', 'cut180', 'cut0_footer'):
                source = group[policy]
                if source['images'] != reference['images'] or source['stamp'] != reference['stamp']:
                    raise ValueError('Paired photo or host reference was not frozen')
                with np.load(source['npz']) as cloud:
                    raw = np.column_stack([cloud[k] for k in ('x', 'y', 'z', 'intensity')]).astype(np.float32)
                if baseline_meta is None:
                    batch = preprocessor.prepare(raw, images, sequence, timestamp_sec=reference['stamp'])
                    baseline_meta = batch['data_samples'][0].metainfo
                points = torch.from_numpy(preprocessor._filter_points(raw))
                def matrix(key):
                    return torch.tensor(np.asarray(baseline_meta[key]), dtype=torch.float32)[None]
                projected = project_positive_camera_z([points], matrix('lidar2img'), matrix('img_aug_matrix'),
                                                      matrix('lidar_aug_matrix'), preprocessor.final_dim)
                cameras = []
                h, w = preprocessor.final_dim
                for ci, (xy, z) in enumerate(projected[0]):
                    index = xy[:,0]*w + xy[:,1]
                    distinct, counts = torch.unique(index, return_counts=True)
                    nearest = reduce_depth_pixels(xy,z,(h,w))
                    farthest = reduce_depth_pixels(xy,z,(h,w),'farthest_positive')
                    changed = farthest > nearest
                    cameras.append(dict(camera=CAMERAS[ci], valid_projected_points=len(index),
                        occupied_pixels=len(distinct), duplicate_pixels=int((counts>1).sum()),
                        excess_samples=len(index)-len(distinct), different_z_pixels=int(changed.sum()),
                        different_z_fraction_of_occupied=float(changed.sum())/len(distinct) if len(distinct) else None,
                        conflicting_camera_z_gap_m=distribution((farthest-nearest)[changed].tolist())))
                results.append(dict(pair=pair, case=source['case'], policy=policy, raw_points=len(raw),
                                    filtered_points=len(points), source_npz_sha256=digest(source['npz']),
                                    frozen_photo_sha256=photo_hashes, frozen_host_stamp=source['stamp'], cameras=cameras))
        keys = ('raw_points','filtered_points')
        summary = {}
        for case in case_pairs:
            summary[str(case)] = {}
            for policy in ('fixed51','fixed50','cut0','cut180','cut0_footer'):
                rows = [r for r in results if r['case']==case and r['policy']==policy]
                summary[str(case)][policy] = dict(pairs=len(rows), **{k: distribution([r[k] for r in rows]) for k in keys},
                    cameras={camera: {k: distribution([c[k] for r in rows for c in r['cameras'] if c['camera']==camera])
                                      for k in ('valid_projected_points','occupied_pixels','duplicate_pixels','excess_samples','different_z_pixels')}
                             for camera in CAMERAS})
        matched = defaultdict(dict)
        for row in results:
            matched[row['pair']][row['policy']] = row
        repeat_ok = all(v['cut0']['cameras'] == v['cut0_footer']['cameras'] and
                        v['cut0']['source_npz_sha256'] == v['cut0_footer']['source_npz_sha256'] for v in matched.values())
        report = dict(scope=__doc__, pairs=len(chosen), rows=results, summary=summary,
                      cut0_geometry_repeat_bitwise=repeat_ok, input_sha256=digest(args.inputs),
                      sensor_sha256=digest(args.sensor), config_sha256=digest(args.config))
        (args.output/'collisions.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(pairs=len(chosen), samples=len(results), cut0_geometry_repeat_bitwise=repeat_ok)),flush=True)
    finally:
        preprocessor.close()


if __name__ == '__main__':
    main()
