#!/usr/bin/env python3
"""Small diagnostic-only native-point ablation, never a deployable ego filter.

Uniformly select up to eight frames per dominant GT class/empty cohort. Remove
only the measured near-roof volume and require EVERY original GT point count
to remain unchanged. Source PNGs, GT, calibration, checkpoint and raw files
stay unchanged. The removed volume is not a general safe exclusion region.
"""
import argparse
import copy
import gzip
import json
from pathlib import Path
import pickle

import numpy as np

from build_base3_bevfusion_infos import points_in_box_count
from compare_base5_common30 import digest, gt_identity, validate_payload

LOWER = np.array([-1.2, -.75, -.35])
UPPER = np.array([1., .75, .1])
E24_SHA256 = '84c7a849406a37f05250f872d4d0dfa3ebcffdb66b48e47bdfcb3209efecc746'


def mask_roof(points):
    valid = np.isfinite(points).all(axis=1) & (points[:, :3] != 0).any(axis=1)
    result = np.zeros(len(points), dtype=bool)
    result[valid] = ((points[valid, :3] >= LOWER) & (points[valid, :3] <= UPPER)).all(axis=1)
    return result


def selection(rows):
    groups = {i: [] for i in range(4)}
    for i, src in enumerate(rows):
        labels = [g['bbox_label_3d'] for g in src['instances']]
        label = int(np.argmax(np.bincount(labels, minlength=3))) if labels else 3
        groups[label].append(i)
    chosen = []
    for indices in groups.values():
        chosen.extend(indices[int(j)] for j in np.linspace(0, len(indices)-1, min(8,len(indices)), dtype=int))
    return sorted(set(chosen))


def prepare(args):
    if args.output.exists():
        raise ValueError('Fresh probe output required')
    info = pickle.loads(args.info.read_bytes())
    with gzip.open(args.predictions, 'rt') as f:
        baseline = json.load(f)
    validate_payload(baseline)
    if (baseline['checkpoint_sha256'] != E24_SHA256
            or not baseline['inference_precision'].startswith('FP32')
            or baseline.get('inference_optimization')
            or baseline.get('point_intensity_scale') != 1. / 255.):
        raise ValueError('Probe requires the frozen E24 FP32 native-intensity baseline without optimizations')
    rows = info['data_list']
    if baseline['info_sha256'] != digest(args.info) or baseline['frames'] != len(rows):
        raise ValueError('Original complete Base3 val predictions required')
    by_token = {r['token']: r for r in baseline['rows']}
    indices = selection(rows)
    receipts = []
    updated = []
    clouds = []
    for index in indices:
        src = rows[index]
        host = args.capture / Path(src['lidar_points']['lidar_path']).relative_to('/datasets/base3')
        points = np.fromfile(host, '<f4').reshape(-1, 4)
        removed = mask_roof(points)
        filtered = points[~removed]
        for gt in src['instances']:
            before = points_in_box_count(points, gt['bbox_3d'], .03)
            after = points_in_box_count(filtered, gt['bbox_3d'], .03)
            if before != after or before != gt['num_lidar_pts']:
                raise ValueError('Roof probe changes original GT support; reject rather than reinterpret GT')
        target = args.output.resolve() / 'points' / (src['token'] + '.bin')
        row = copy.deepcopy(src)
        row['lidar_points']['lidar_path'] = str(target)
        updated.append(row)
        clouds.append((target, filtered))
        receipts.append(dict(token=src['token'], source_index=index, source_cloud_sha256=digest(host),
            before_points=len(points), removed_points=int(removed.sum()), after_points=len(filtered),
            all_original_gt_support_unchanged=True))
    args.output.mkdir(parents=True)
    (args.output / 'points').mkdir()
    for target, points in clouds:
        points.astype('<f4', copy=False).tofile(target)
    info['data_list'] = updated
    (args.output / 'masked_infos_val.pkl').write_bytes(pickle.dumps(info, protocol=4))
    baseline['rows'] = [by_token[r['token']] for r in updated]
    baseline.update(name='E24_original32', frames=len(updated), gt_identity=gt_identity(baseline['rows']))
    validate_payload(baseline)
    with gzip.open(args.output / 'baseline_selected.json.gz', 'wt') as f:
        json.dump(baseline, f, allow_nan=False)
    (args.output / 'manifest.json').write_text(json.dumps(dict(
        scope='Diagnostic input ablation only; no GT, training or production self filter change.',
        source_info_sha256=digest(args.info), source_predictions_sha256=digest(args.predictions),
        frames=len(updated), selection='Uniform per dominant GT label and empty cohort, not score-ranked.',
        lower=LOWER.tolist(), upper=UPPER.tolist(), frames_audit=receipts), indent=2) + '\n')
    print('Prepared diagnostic frames', len(updated), 'GT support unchanged')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for key in ('info','capture','predictions','output'):
        p.add_argument('--'+key, type=Path, required=True)
    prepare(p.parse_args())
