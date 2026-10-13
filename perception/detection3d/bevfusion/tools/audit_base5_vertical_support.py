#!/usr/bin/env python3
"""Read-only native-box LiDAR Z support audit; never change labels or sensors.

Counts use the native full-RPY box, not the enclosing upright training box.
Point support is not class accuracy, visibility, or physical ground contact.
Only trusted local portable pickle files may be loaded.
"""
import argparse
from collections import Counter, defaultdict
import itertools
import json
from pathlib import Path
import pickle
import time

import numpy as np

from prepare_base5_portable_dataset import sha256, validate

CLASSES = ('vehicle', 'pedestrian', 'obstacle', 'animal')
Z_RANGES = {'current': (-3., 2.), 'internal': (-5., 3.)}


def native_rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def supported_points(xyz, component):
    center = np.asarray(component['center_lidar_m'])
    local = (xyz - center) @ native_rotation(component['rotation_rpy'])
    return xyz[(np.abs(local) <= np.asarray(component['size_m']) / 2 + 1e-5).all(1)]


def range_count(points, bounds):
    return int(((points[:, 2] > bounds[0]) & (points[:, 2] < bounds[1])).sum())


def augmented_z_counts(points, scale, translation, bounds=(-3., 2.)):
    # BEVFusionGlobalRotScaleTrans applies rotation -> translation -> scale.
    # Yaw rotation does not change Z. Isolate Z filtering, not XY/crop effects.
    z = (points[:, 2, None] + translation) * scale
    return ((z > bounds[0]) & (z < bounds[1])).sum(0)


def crop_center_depths(center, images):
    depths = []
    for camera in images.values():
        matrix = np.asarray(camera['lidar2cam'])
        cam = (matrix @ np.r_[center, 1.])[:3]
        if cam[2] <= 0:
            continue
        pixel = np.asarray(camera['cam2img'])[:3, :3] @ cam
        scale = .825 * 1280 / camera['width']
        uv = pixel[:2] / pixel[2] * scale - [0., int(camera['height'] * scale) - 512]
        if ((uv >= [0., 0.]) & (uv < [1056., 512.])).all():
            depths.append(float(cam[2]))
    return depths


def summarize(rows):
    support = sum(r['native_xy_support'] for r in rows)
    kept = sum(r['current_support'] for r in rows)
    return dict(objects=len(rows), native_xy_points=support, current_points=kept,
                removed_points=support - kept,
                removed_point_fraction=(support - kept) / support if support else None,
                affected_objects=sum(r['current_support'] < r['native_xy_support'] for r in rows),
                zero_before=sum(r['native_xy_support'] == 0 for r in rows),
                lost_all_support=sum(r['native_xy_support'] > 0 and r['current_support'] == 0 for r in rows),
                recovered_points_internal=sum(r['internal_support'] - r['current_support'] for r in rows),
                recovered_objects_internal=sum(r['internal_support'] > r['current_support'] for r in rows),
                lost_at_least_half=sum(r['native_xy_support'] > 0 and 2 * r['current_support'] <= r['native_xy_support'] for r in rows),
                augmented_lost_all_observations=sum(r.get('augmented_lost_all_observations', 0) for r in rows),
                augmented_lost_half_observations=sum(r.get('augmented_lost_half_observations', 0) for r in rows))


def audit(data, output):
    data, output = Path(data), Path(output)
    if output.exists():
        raise FileExistsError('Fresh output required')
    manifest = validate(data)
    if manifest.get('box_geometry_policy') != 'native_rpy_fixed_yaw_enclosure_v1':
        raise ValueError('Current upright portable data required')
    before = sha256(data / 'manifest.json')
    rows, groups, cloud_counts = [], defaultdict(list), Counter()
    rng = np.random.RandomState(261011)
    z_hist = np.zeros(80, dtype=np.int64)
    voxel_caps = Counter()
    started = time.monotonic()
    frames = 0
    for split in ('train', 'val'):
        records = pickle.loads((data / 'infos/512x1056' / ('morai_base5_infos_' + split + '.pkl')).read_bytes())['data_list']
        for record in records:
            raw = np.fromfile(data / record['lidar_points']['lidar_path'], np.float32).reshape(-1, 4)
            valid = np.isfinite(raw).all(1) & (raw[:, :3] != 0).any(1)
            xyz = raw[valid, :3]
            xyz = xyz[(np.abs(xyz[:, 0]) < 100.) & (np.abs(xyz[:, 1]) < 30.)]
            cloud_counts['valid_xy'] += len(xyz)
            cloud_counts['current'] += range_count(xyz, Z_RANGES['current'])
            cloud_counts['internal'] += range_count(xyz, Z_RANGES['internal'])
            filtered = xyz[(xyz[:, 2] > -3.) & (xyz[:, 2] < 2.)]
            coors = np.floor((filtered - [-100.8, -32.4, -5.]) / [.075, .075, .2]).astype(np.int64)
            counts = np.unique(coors, axis=0, return_counts=True)[1] if len(coors) else np.array([], dtype=int)
            voxel_caps['max_cloud_points'] = max(voxel_caps['max_cloud_points'], len(raw))
            voxel_caps['max_occupied_voxels'] = max(voxel_caps['max_occupied_voxels'], len(counts))
            voxel_caps['max_points_per_voxel'] = max(voxel_caps['max_points_per_voxel'], int(counts.max()) if len(counts) else 0)
            voxel_caps['voxel_overflow_points_10'] += int(np.maximum(counts - 10, 0).sum())
            z_hist += np.histogram(xyz[:, 2], bins=80, range=(-10., 10.))[0]
            source = json.loads((data / 'data' / record['token'] / 'audit.json').read_text())
            components = {c['component_id']: c for c in source['components']}
            scales = rng.uniform(.95, 1.05, 16)
            translations = rng.normal(0., .2, 16)
            for inst in record['instances']:
                component = components[inst['component_id']]
                pts = supported_points(xyz, component)
                cls = CLASSES[inst['bbox_label_3d']]
                distance = float(np.linalg.norm(component['center_lidar_m'][:2]))
                signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
                corners = (signs * np.asarray(component['size_m']) / 2) @ native_rotation(component['rotation_rpy']).T + component['center_lidar_m']
                voxels = {}
                for dz in (.1, .2, .4):
                    keep = pts[(pts[:, 2] > -3.) & (pts[:, 2] < 2.)]
                    coors = np.floor((keep - [-100.8, -32.4, -5.]) / [.075, .075, dz]).astype(np.int64)
                    voxels[str(dz)] = len(np.unique(coors, axis=0)) if len(coors) else 0
                augmented = augmented_z_counts(pts, scales, translations)
                camera_box_depths = []
                for camera in record['images'].values():
                    if crop_center_depths(component['center_lidar_m'], {'camera': camera}):
                        matrix = np.asarray(camera['lidar2cam'])
                        depths = (corners @ matrix[:3, :3].T + matrix[:3, 3])[:, 2]
                        camera_box_depths.append([float(depths.min()), float(depths.max())])
                row = dict(token=record['token'], split=split, component_id=inst['component_id'],
                           asset=inst['asset'], cls=cls, distance_m=distance,
                           native_xy_support=len(pts), current_support=range_count(pts, Z_RANGES['current']),
                           internal_support=range_count(pts, Z_RANGES['internal']),
                           below_current=int((pts[:, 2] <= -3.).sum()), above_current=int((pts[:, 2] >= 2.).sum()),
                           bbox_z_min=float(corners[:, 2].min()), bbox_z_max=float(corners[:, 2].max()),
                           occupied_voxels_by_z_pitch=voxels,
                           crop_projectable_center_depths=crop_center_depths(component['center_lidar_m'], record['images']),
                           crop_center_native_box_depth_intervals=camera_box_depths,
                           augmented_lost_all_observations=int((augmented == 0).sum()) if len(pts) else 0,
                           augmented_lost_half_observations=int((augmented * 2 <= len(pts)).sum()) if len(pts) else 0,
                           augmented_min_support=int(augmented.min()))
                rows.append(row)
                for key in ('all', cls, 'asset:' + inst['asset'], cls + (':near10' if distance < 10 else ':10to60' if distance < 60 else ':60plus')):
                    groups[key].append(row)
            frames += 1
            if frames % 500 == 0:
                print('Z AUDIT %d/5000' % frames, flush=True)
    camera_only = [r for r in rows if r['native_xy_support'] == 0]
    depth_values = [d for r in rows for d in r['crop_projectable_center_depths']]
    depth_report = dict(scope='Positive optical-depth centers geometrically project inside current crop; does not prove visibility or learned depth accuracy',
                        observations=len(depth_values), camera_only_objects=len(camera_only),
                        optical_depth_m=dict(min=float(min(depth_values)), max=float(max(depth_values)),
                                             p95=float(np.percentile(depth_values, 95)), p99=float(np.percentile(depth_values, 99))),
                        camera_only_without_projectable_center=sum(not r['crop_projectable_center_depths'] for r in camera_only),
                        outside_current_depth_all_views=sum(bool(r['crop_projectable_center_depths']) and not any(1. <= d < 100. for d in r['crop_projectable_center_depths']) for r in rows),
                        camera_only_outside_current_depth_all_views=sum(bool(r['crop_projectable_center_depths']) and not any(1. <= d < 100. for d in r['crop_projectable_center_depths']) for r in camera_only),
                        camera_only_entire_box_below_1_all_center_projectable_views=sum(bool(r['crop_center_native_box_depth_intervals']) and all(hi < 1. for lo, hi in r['crop_center_native_box_depth_intervals']) for r in camera_only),
                        camera_only_beyond_upper_range={str(limit): sum(bool(r['crop_projectable_center_depths']) and all(d >= limit for d in r['crop_projectable_center_depths']) for r in camera_only) for limit in (60., 80., 100.)})
    report = dict(valid=True, frames=frames, gt=len(rows), source_manifest_sha256=before,
                  scope='Native full-RPY support inside fixed XY ROI, before/after open Z filters; not AP or ground-contact certification',
                  z_ranges=Z_RANGES, cloud_points=dict(cloud_counts),
                  voxel_caps=dict(voxel_caps), camera_depth_range=depth_report,
                  augmentation=dict(draws_per_frame=16, seed=261011, scale=[.95, 1.05], translation_z_std=.2,
                                    scope='Z-only RTS filtering probe, native XY support fixed; not actual full-pipeline replay or AP'),
                  groups={key: summarize(values) for key, values in groups.items()},
                  z_voxel_information={str(dz): dict(occupied_object_voxels=sum(r['occupied_voxels_by_z_pitch'][str(dz)] for r in rows),
                                                    note='Quantization information only; no backbone redesign or learned AP comparison') for dz in (.1, .2, .4)},
                  z_histogram=dict(edges_m=np.linspace(-10., 10., 81).tolist(), counts=z_hist.tolist()),
                  elapsed_seconds=time.monotonic() - started)
    assert before == sha256(data / 'manifest.json'), 'Source manifest changed'
    assert frames == 5000 and len({r['token'] for r in rows}) <= frames
    output.mkdir(parents=True)
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / 'objects.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('groups', 'z_histogram')}, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    audit(args.data, args.output)


if __name__ == '__main__':
    main()
