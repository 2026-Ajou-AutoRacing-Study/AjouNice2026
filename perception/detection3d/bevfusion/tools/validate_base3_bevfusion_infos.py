#!/usr/bin/env python3
"""Validate the complete Base3 BEVFusion info contract without training."""

import argparse
import json
import pickle
from collections import Counter
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--info-dir', type=Path, required=True)
    parser.add_argument(
        '--capture-root', type=Path,
        help='host capture root used to validate portable /datasets/base3 paths')
    parser.add_argument(
        '--serialized-capture-root', type=Path, default=Path('/datasets/base3'))
    args = parser.parse_args()

    def host_path(value):
        path = Path(value)
        if args.capture_root is None:
            return path
        try:
            relative = path.relative_to(args.serialized_capture_root)
        except ValueError as exc:
            raise AssertionError(
                f'{path} is outside {args.serialized_capture_root}') from exc
        return args.capture_root.expanduser().resolve() / relative
    seen, class_counts, split_counts = set(), Counter(), {}
    scene_splits = {}
    for split in ('train', 'val', 'test'):
        path = args.info_dir / f'morai_base3_infos_{split}.pkl'
        payload = pickle.loads(path.read_bytes())
        assert tuple(payload['metainfo']['classes']) == (
            'vehicle', 'pedestrian', 'obstacle')
        box_dim = int(payload['metainfo'].get('box_dim', 7))
        causal_sweep_count = int(
            payload['metainfo'].get('causal_sweep_count', 1))
        rows = payload['data_list']
        split_counts[split] = len(rows)
        for row in rows:
            sample = row['sample_idx']
            assert sample not in seen, f'duplicate sample: {sample}'
            seen.add(sample)
            scene = str(row['scene_id'])
            previous_split = scene_splits.setdefault(scene, split)
            assert previous_split == split, (
                f'scene leakage: {scene} is in {previous_split} and {split}')
            lidar = host_path(row['lidar_points']['lidar_path'])
            assert lidar.is_file() and lidar.stat().st_size % 16 == 0, lidar
            sweeps = row.get('lidar_sweeps', [])
            assert len(sweeps) == max(causal_sweep_count - 1, 0)
            lags = [float(sweep['time_lag_sec']) for sweep in sweeps]
            assert lags == sorted(lags) and all(lag > 0.0 for lag in lags)
            for sweep in sweeps:
                sweep_lidar = host_path(sweep['lidar_points']['lidar_path'])
                assert (
                    sweep_lidar.is_file()
                    and sweep_lidar.stat().st_size % 16 == 0
                ), sweep_lidar
                transform = np.asarray(
                    sweep['lidar_points']['sweep2lidar'], dtype=np.float64)
                assert transform.shape == (4, 4) and np.isfinite(transform).all()
                rotation = transform[:3, :3]
                assert np.allclose(
                    rotation.T @ rotation, np.eye(3), atol=1.0e-5)
                assert np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-5)
                assert np.allclose(
                    transform[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8)
                assert float(sweep['timestamp']) < float(row['timestamp'])
                assert np.isclose(
                    float(row['timestamp']) - float(sweep['timestamp']),
                    float(sweep['time_lag_sec']), atol=1.0e-6)
            assert list(row['images']) == [
                'front', 'front_left', 'front_right', 'back']
            for camera in row['images'].values():
                assert host_path(camera['img_path']).is_file(), camera['img_path']
                assert np.asarray(camera['cam2img']).shape == (3, 3)
                assert np.asarray(camera['lidar2cam']).shape == (4, 4)
                assert np.asarray(camera['lidar2img']).shape == (3, 4)
                assert np.isfinite(np.asarray(camera['lidar2img'])).all()
                assert camera['height'] > 0 and camera['width'] > 0
            for instance in row['instances']:
                label = int(instance['bbox_label_3d'])
                assert 0 <= label < 3
                assert int(instance['num_lidar_pts']) >= 5
                box = np.asarray(instance['bbox_3d'], dtype=np.float64)
                assert box.shape == (box_dim,) and np.isfinite(box).all()
                assert np.all(box[3:6] > 0)
                if box_dim == 9:
                    velocity = np.asarray(
                        instance['velocity_target_ego_mps'], dtype=np.float64)
                    assert velocity.shape == (2,) and np.isfinite(velocity).all()
                    assert np.allclose(box[7:9], velocity)
                class_counts[('vehicle', 'pedestrian', 'obstacle')[label]] += 1
    report = {
        'valid': True,
        'frames': len(seen),
        'scenes': len(scene_splits),
        'splits': split_counts,
        'kept_gt': dict(class_counts),
    }
    target = args.info_dir / 'validation_audit.json'
    target.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
