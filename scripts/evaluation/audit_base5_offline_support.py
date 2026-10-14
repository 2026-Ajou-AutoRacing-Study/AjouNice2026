#!/usr/bin/env python3
"""Read-only support/diversity audit of trusted local Base5 derived labels.

Counts are component observations, not independent actors or model accuracy.
The script neither changes labels nor runs ROS, MORAI, GPU inference or training.
Camera rich/gate flags are inherited actual instance-mask support proxies.
"""

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import pickle

import numpy as np

RANGES = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 80.0, 100.0, math.inf)


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def range_bin(value):
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError('Range must be finite and nonnegative')
    for lo, hi in zip(RANGES, RANGES[1:]):
        if lo <= value < hi:
            return '100plus_corner' if math.isinf(hi) else f'{int(lo)}_{int(hi)}m'
    raise AssertionError('Unreachable range')


def point_bin(value):
    value = int(value)
    if value < 0:
        raise ValueError('Negative LiDAR point count')
    return '0' if value == 0 else '1_4' if value < 5 else '5_19' if value < 20 else '20plus'


def sector(value, sectors=12):
    if not math.isfinite(float(value)) or sectors <= 0:
        raise ValueError('Invalid angle/sector count')
    # Half-open sectors, invariant under complete revolutions.
    return int(((float(value) + math.pi) % (2 * math.pi)) / (2 * math.pi) * sectors) % sectors


def percentiles(values):
    return dict(zip(('p10', 'p50', 'p90', 'p95'), map(float, np.percentile(values, (10, 50, 90, 95))))) if values else None


def support_summary(rows):
    if not rows:
        return {'gt_observations': 0}
    rich = sum(r['points'] >= 20 or r['camera_rich_proxy'] is True for r in rows)
    camera_unknown = sum(r['camera_gate'] is None for r in rows)
    weak_known = sum(r['points'] < 5 and r['camera_gate'] is False for r in rows)
    footprint = [float(r['size_m'][0] * r['size_m'][1]) for r in rows]
    return {
        'gt_observations': len(rows),
        'frames': len({r['token'] for r in rows}),
        'parent_observations': len({(r['token'], r['parent_uid']) for r in rows}),
        'primary_gt': sum(r['primary'] for r in rows),
        'secondary_gt': sum(not r['primary'] for r in rows),
        'point_bins': dict(Counter(point_bin(r['points']) for r in rows)),
        'rich_support_proxy': rich,
        'rich_support_pct': 100 * rich / len(rows),
        'weak_support_known': weak_known,
        'camera_unknown': camera_unknown,
        'camera_gate': sum(r['camera_gate'] is True for r in rows),
        'camera_rich_proxy': sum(r['camera_rich_proxy'] is True for r in rows),
        'camera_only': sum(r['points'] == 0 for r in rows),
        'crop_loss': sum(r['any_extra_crop_loss'] is True for r in rows),
        'native_points': percentiles([r['points'] for r in rows]),
        'mask_pixels': percentiles([r['best_visible_mask_pixels'] for r in rows
                                    if r['best_visible_mask_pixels'] is not None]),
        'footprint_m2': percentiles(footprint),
        'heading_30deg_sectors': dict(sorted(Counter(sector(r['yaw_lidar_rad']) for r in rows).items())),
        'aspect_30deg_sectors': dict(sorted(Counter(sector(r['yaw_lidar_rad'] -
            math.atan2(r['center_lidar_m'][1], r['center_lidar_m'][0])) for r in rows).items())),
        'directions': dict(Counter(r['direction'] for r in rows)),
        'families': dict(Counter(r['family'] for r in rows)),
        'original_semantics': dict(Counter(r['original_semantic'] for r in rows)),
    }


def audit(objects, infos):
    """Join immutable component IDs, fail rather than silently drop unmatched GT."""
    instance_index, frame_splits, groups, mapped_classes = {}, {}, {}, {}
    for split, source in infos.items():
        class_names = source.get('metainfo', {}).get('classes')
        for frame in source['data_list']:
            token = frame['token']
            if token in frame_splits:
                raise ValueError('Duplicate frame or split overlap: ' + token)
            frame_splits[token] = split
            groups[token] = frame.get('scene_group_id')
            for inst in frame['instances']:
                key = (token, inst['component_id'])
                if key in instance_index:
                    raise ValueError('Duplicate component in infos: ' + str(key))
                instance_index[key] = inst
                if class_names is not None:
                    original = class_names[int(inst['bbox_label_3d'])]
                    mapped_classes[key] = 'pedestrian' if original == 'animal' else original
    rows, seen = [], set()
    for item in objects:
        row = dict(item)
        key = (row['token'], row['component_id'])
        if key in seen:
            raise ValueError('Duplicate support object: ' + str(key))
        seen.add(key)
        inst = instance_index.get(key)
        if inst is None or frame_splits[row['token']] != row['split']:
            raise ValueError('Support/infos component or split mismatch: ' + str(key))
        if key in mapped_classes and row['cls'] != mapped_classes[key]:
            raise ValueError('Support/infos merged training class mismatch: ' + str(key))
        if (inst['asset'] != row['asset'] or
                not 0 <= int(row['points']) <= int(row['native_points']) <= int(inst['num_lidar_pts'])):
            raise ValueError('Support/infos asset/point mismatch: ' + str(key))
        if not np.allclose(inst['bbox_3d'][:3], row['center_lidar_m'], atol=1e-5, rtol=0):
            raise ValueError('Support/infos center mismatch: ' + str(key))
        row['parent_uid'] = str(inst['parent_uid'])
        row['annotation_points_total'] = int(inst['num_lidar_pts'])
        row['yaw_lidar_rad'] = float(inst['bbox_3d'][6])
        row['distance_bin_refined'] = range_bin(row['distance_m'])
        rows.append(row)
    if seen != set(instance_index):
        raise ValueError('Infos has components absent from support evidence')
    by_group = defaultdict(list)
    by_frame = defaultdict(list)
    for row in rows:
        by_frame[row['token']].append(row)
        keys = ['all', row['split'], f"{row['split']}/{row['cls']}",
                f"{row['split']}/{row['cls']}/{row['distance_bin_refined']}",
                f"{row['split']}/{row['cls']}/{row['distance_bin_refined']}/{row['direction']}",
                f"{row['split']}/asset/{row['asset']}",
                f"{row['split']}/asset/{row['asset']}/{row['distance_bin_refined']}",
                f"{row['split']}/family/{row['family']}"]
        for key in keys:
            by_group[key].append(row)
    train = [r for r in rows if r['split'] == 'train']
    assets = sorted({r['asset'] for r in train})
    asset_review = []
    for asset in assets:
        values = [r for r in train if r['asset'] == asset]
        supported = [r for r in values if r['points'] >= 20 or r['camera_rich_proxy'] is True]
        near_mid = [r for r in values if r['distance_m'] < 30]
        asset_review.append({
            'asset': asset, 'class_semantics': sorted({r['cls'] for r in values}),
            'train_gt': len(values), 'primary_gt': sum(r['primary'] for r in values),
            'secondary_gt': sum(not r['primary'] for r in values),
            'near30_gt': len(near_mid), 'rich_support_gt': len(supported),
            'rich_support_pct': 100 * len(supported) / len(values),
            'rich_support_near30': sum(r['points'] >= 20 or r['camera_rich_proxy'] is True for r in near_mid),
            'heading_sectors_near30': len({sector(r['yaw_lidar_rad']) for r in near_mid}),
            'rich_heading_sectors': len({sector(r['yaw_lidar_rad']) for r in supported}),
            'directions': dict(Counter(r['direction'] for r in values)),
            'point_bins': dict(Counter(point_bin(r['points']) for r in values)),
            'range_bins': dict(Counter(r['distance_bin_refined'] for r in values)),
            'footprint_m2_median': float(np.median([r['size_m'][0] * r['size_m'][1] for r in values])),
        })
    frame_summary = {}
    for split in infos:
        tokens = [t for t, s in frame_splits.items() if s == split]
        component_counts = [len(by_frame[t]) for t in tokens]
        parent_counts = [len({r['parent_uid'] for r in by_frame[t]}) for t in tokens]
        rich_counts = [sum(r['points'] >= 20 or r['camera_rich_proxy'] is True for r in by_frame[t]) for t in tokens]
        frame_summary[split] = {
            'frames': len(tokens), 'gt_observations': sum(component_counts),
            'component_count_histogram': dict(Counter(component_counts)),
            'visible_parent_count_histogram': dict(Counter(parent_counts)),
            'rich_component_count_histogram': dict(Counter(rich_counts)),
            'component_count': percentiles(component_counts),
            'visible_parent_count': percentiles(parent_counts),
        }
    asset_review.sort(key=lambda r: (r['rich_support_gt'], r['train_gt'], r['asset']))
    return {
        'schema': 'base5_offline_support_diversity_v1',
        'scope': 'Immutable derived GT + actual-mask/native-point evidence; not detector accuracy or independent test',
        'join_checks': {
            'support_instances': len(rows), 'infos_instances': len(instance_index), 'unmatched': 0,
            'xy_roi_clipped_components': sum(r['native_points'] != r['annotation_points_total'] for r in rows),
            'vertical_roi_clipped_components': sum(r['points'] != r['native_points'] for r in rows),
        },
        'frames': frame_summary,
        'groups': {k: support_summary(v) for k, v in sorted(by_group.items())},
        'train_asset_review': asset_review,
        'train_rare_support_20_or_fewer': [r for r in asset_review if r['rich_support_gt'] <= 20],
        'train_far_nonvehicle': support_summary([r for r in train if r['cls'] != 'vehicle' and r['distance_m'] >= 50]),
        'train_far_vehicle': support_summary([r for r in train if r['cls'] == 'vehicle' and r['distance_m'] >= 50]),
        'limitations': [
            'Rich support: LiDAR >=20 OR unambiguous mask >=64 model pixels and minimum bbox extent >=8 pixels; not correct shape/heading/depth.',
            'Camera ambiguity is unknown, never counted as invisibility.',
            'Yaw distributions include symmetric assets; heading diversity does not certify observable heading.',
            'Native size footprint describes retained components, not selection reward or mesh footprint.',
            'Annotation point count is total native BBox support; evidence native_points applies XY ROI, points also applies model Z ROI.',
            'Radial >100m can be a corner inside rectangular X±100/Y±30m ROI.',
            'Small-asset frequency cannot establish sampling bias without parent/primary quota and generator controls.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--objects', type=Path, required=True)
    parser.add_argument('--train-infos', type=Path, required=True)
    parser.add_argument('--val-infos', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sources = [args.objects, args.train_infos, args.val_infos]
    before = {str(p): sha256(p) for p in sources}
    with args.train_infos.open('rb') as stream:
        train = pickle.load(stream)
    with args.val_infos.open('rb') as stream:
        val = pickle.load(stream)
    report = audit(json.loads(args.objects.read_text()), {'train': train, 'val': val})
    if before != {str(p): sha256(p) for p in sources}:
        raise RuntimeError('Input changed during read-only audit')
    report['inputs_sha256'] = before
    report['generated_at_utc'] = datetime.now(timezone.utc).isoformat()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise FileExistsError('Use a new output path to preserve prior audit')
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'output': str(args.output), 'join_checks': report['join_checks'],
                      'frames': report['frames'], 'rare_assets': len(report['train_rare_support_20_or_fewer'])}, indent=2))


if __name__ == '__main__':
    main()
