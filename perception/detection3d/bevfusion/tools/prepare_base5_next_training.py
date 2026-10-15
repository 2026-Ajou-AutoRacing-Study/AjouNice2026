#!/usr/bin/env python3
"""CPU-only next-run audit. Never imports model registries or starts training.

Only read trusted local portable infos (pickle). This produces an audit plan,
not the frozen runner's run_manifest or a qualified remote launch contract.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import pickle
import shlex

import numpy as np
from mmengine.config import Config

from prepare_base5_portable_dataset import safe_relative, sha256, validate

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_CONFIG = ROOT / 'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py'
POLICY = 'base5_eager_nearest_v1'
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
ROI = [-100., -30., -3., 100., 30., 2.]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def check_config(path=DEFAULT_CONFIG):
    cfg = Config.fromfile(str(path), import_custom_modules=False)
    require(cfg.get('required_next_training_policy') == POLICY, 'Use the new next-training config')
    model = cfg.model
    require(model.type == 'MoraiTrainingRuntimeBEVFusion' and model.get('compile_camera') is False,
            'Next training requires explicit compile OFF')
    require(not cfg.get('compile', False), 'Runner-wide compile must also be OFF')
    require(model.pts_middle_encoder.type == 'MoraiRectangularSparseEncoder'
            and model.bbox_head.type == 'MoraiRectangularTransFusionHead',
            'Use the metric-axis-corrected encoder/head implementations')
    view = model.view_transform
    require(view.type == 'MoraiDepthRasterSeparatedDepthLSSTransform'
            and view.depth_raster_policy == 'nearest_positive' and view.depth_raster_backend == 'sorted',
            'Train/val must share nearest_positive/sorted depth hints')
    require(tuple(cfg.classes) == CLASSES and model.bbox_head.num_classes == 3,
            'Expected vehicle/pedestrian/obstacle classes')
    require(cfg.required_training_label_policy == 'animal_to_pedestrian_v1', 'Animal merge policy differs')
    require(cfg.required_camera_input_policy == 'native_jpeg90', 'Native JPEG policy differs')
    require(cfg.required_box_geometry_policy == 'native_rpy_fixed_yaw_enclosure_v1', 'Upright GT policy differs')
    require('vel' not in model.bbox_head.common_heads and model.bbox_head.bbox_coder.code_size == 8,
            'Expected single-frame 7D detector without OD velocity')
    require(list(cfg.point_cloud_range) == list(model.bbox_head.bbox_coder.post_center_range) == ROI,
            'Semantic ROI differs')
    voxel = model.data_preprocessor.voxelize_cfg
    require(list(voxel.voxel_size) == [.05, .05, .2]
            and list(voxel.point_cloud_range) == [-100.8, -33.6, -5., 100.8, 33.6, 3.],
            'LiDAR voxel/padding contract differs')
    require(list(model.pts_middle_encoder.sparse_shape) == [1344, 4032, 41], 'Sparse Y/X axes differ')
    require(list(view.image_size) == [512, 1056] and list(view.dbound) == [1., 100., .25],
            'Image/depth contract differs')
    require(list(view.xbound) == [-100.8, 100.8, .2]
            and list(view.ybound) == [-33.6, 33.6, .2] and view.downsample == 2,
            'Camera BEV contract differs')
    head = model.bbox_head
    for part in (head.train_cfg, head.test_cfg, head.bbox_coder):
        require(part.out_size_factor == 8, 'Head stride differs')
        require(list(part.voxel_size)[:2] == [.05, .05], 'Head voxel units differ')
    require(list(head.train_cfg.grid_size) == list(head.test_cfg.grid_size) == [4032, 1344, 41],
            'Head X/Y grid differs')
    require(list(head.train_cfg.point_cloud_range) == [-100.8, -33.6, -3., 100.8, 33.6, 2.]
            and list(head.test_cfg.pc_range) == list(head.bbox_coder.pc_range) == [-100.8, -33.6],
            'Camera/LiDAR/head physical origins differ')
    require(cfg.load_from is None and not cfg.get('resume', False), 'New run must not inherit a checkpoint/resume')
    for split in ('train', 'val', 'test'):
        dataset = cfg[split + '_dataloader'].dataset
        require(dataset.type == 'MoraiBase5PedestrianDataset', 'Use the upright three-class dataset adapter')
        pipeline = dataset.pipeline
        kinds = [t['type'] for t in pipeline]
        require(not any('Sweep' in t or 'Temporal' in t for t in kinds), 'Unexpected temporal input')
        require('MoraiOnlineImageDomainAug3D' not in kinds and 'MoraiUseNativeJPEGCache' not in kinds,
                'Preencoded JPEG must not be encoded again')
        require(kinds.count('MoraiFilterInvalidPoints') == 1, 'Expected one invalid-return filter')
        image = next(t for t in pipeline if t['type'] == 'BEVLoadMultiViewImageFromFiles')
        points = next(t for t in pipeline if t['type'] == 'LoadPointsFromFile')
        augment = next(t for t in pipeline if t['type'] == 'MoraiImageAug3D')
        require(image.to_float32 is False and image.num_views == 4 and augment.output_dtype == 'uint8',
                'Native camera loading contract differs')
        require(list(augment.final_dim) == [512, 1056] and list(augment.resize_lim) == [.825, .825],
                'Image crop/resize differs')
        require(points.coord_type == 'LIDAR' and points.load_dim == points.use_dim == 4,
                'Expected native four-feature LiDAR input')
        require(not points.get('norm_intensity', False) and not points.get('shift_height', False)
                and not points.get('use_color', False), 'Native XYZI must not be normalized or extended')
    return cfg, dict(policy=POLICY, compile_camera=False, depth_hint='nearest_positive',
                     classes=list(CLASSES), image_size=[512, 1056], roi=ROI,
                     voxel_xy_m=.05, camera_bev_pitch_m=.2, fusion_pitch_m=.4,
                     fusion_shape_yx=[168, 504], depth_step_m=.25,
                     training_intensity='native_0_1_no_divide_255', box_dim=7)


def config_hashes(path, root=ROOT):
    """Fingerprint all inherited files, without importing any plugin code."""
    result = {}
    def visit(source):
        source = source.resolve(strict=True)
        name = source.relative_to(root).as_posix()
        if name in result:
            return
        result[name] = sha256(source)
        for node in ast.parse(source.read_text()).body:
            if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == '_base_' for t in node.targets):
                bases = ast.literal_eval(node.value)
                for base in ([bases] if isinstance(bases, str) else bases):
                    visit(source.parent / base)
    visit(path)
    return result


def check_data(data, cfg, *, sample_clouds=16, verify_hashes=False):
    require(sample_clouds > 0, 'sample-clouds must be positive')
    inventory = validate(data, verify_hashes=verify_hashes)
    require('512x1056' in inventory['profiles'], '512x1056 infos not exported')
    require(inventory.get('image_policy') == dict.fromkeys(('train', 'val', 'test'), 'native_jpeg90'),
            'All splits must use native JPEG')
    require(inventory.get('training_label_policy') == cfg.required_training_label_policy,
            'Dataset/config class policy differs')
    counts, info_hashes, point_samples = {}, {}, []
    train_tokens, train_groups = set(), set()
    for split in ('train', 'val', 'test'):
        path = data / 'infos/512x1056' / ('morai_base5_infos_' + split + '.pkl')
        info = pickle.loads(path.read_bytes())
        meta, rows = info['metainfo'], info['data_list']
        expected = dict(box_dim=7, single_frame=True, velocity_target=False, range_frame='lidar',
                        box_geometry_policy=cfg.required_box_geometry_policy,
                        training_label_policy=cfg.required_training_label_policy)
        require(all(meta.get(k) == v for k, v in expected.items())
                and tuple(meta['classes']) == CLASSES and list(meta['point_cloud_range']) == ROI,
                'Infos coordinate/box/class contract differs: ' + split)
        tokens = [r['token'] for r in rows]
        require(len(tokens) == len(set(tokens)), 'Duplicate scene tokens: ' + split)
        if split == 'train':
            train_tokens, train_groups = set(tokens), {r['scene_group_id'] for r in rows}
        elif split == 'val':
            require(not train_tokens.intersection(tokens)
                    and not train_groups.intersection(r['scene_group_id'] for r in rows),
                    'Train/val scene-group leakage')
        boxes = 0
        for row in rows:
            require(not row.get('lidar_sweeps') and len(row['images']) == 4
                    and row['lidar_points']['num_pts_feats'] == 4, 'Unexpected sensor inputs')
            for instance in row.get('instances', []):
                box = np.asarray(instance['bbox_3d'], dtype=float)
                require(box.shape == (7,) and np.isfinite(box).all() and (box[3:6] > 0).all(),
                        'Invalid 7D upright GT')
                original = instance.get('original_bbox_label_3d')
                require(original in range(4)
                        and instance.get('original_semantic_class') == ('vehicle', 'pedestrian', 'obstacle', 'animal')[original]
                        and instance['bbox_label_3d'] == (1 if original == 3 else original),
                        'Original animal/class provenance lost')
                boxes += 1
        counts[split] = dict(scenes=len(rows), boxes=boxes)
        info_hashes[split] = sha256(path)
        for index in np.linspace(0, len(rows)-1, min(sample_clouds, len(rows)), dtype=int) if rows else []:
            path = (data / safe_relative(rows[index]['lidar_points']['lidar_path'])).resolve(strict=True)
            require(path.is_relative_to(data), 'Point cloud resolves outside portable data')
            require(path.stat().st_size > 0 and path.stat().st_size % 16 == 0, 'Invalid float32 XYZI size')
            points = np.fromfile(path, dtype=np.float32).reshape(-1, 4)
            # Match MoraiFilterInvalidPoints: exact zero placeholders only,
            # not a new near-distance cutoff or an intensity-zero filter.
            valid = np.isfinite(points).all(axis=1) & (points[:, :3] != 0).any(axis=1)
            require(valid.any(), 'Sample cloud has no valid returns')
            intensity = points[valid, 3]
            require(intensity.min() >= -1e-6 and intensity.max() <= 1.000001,
                    'Expected native intensity 0..1; never divide training data by 255')
            point_samples.append(dict(split=split, token=rows[index]['token'], points=len(points),
                                      filtered_returns=int((~valid).sum()),
                                      intensity_min=float(intensity.min()), intensity_max=float(intensity.max())))
    require(counts['train']['scenes'] > 0 and counts['val']['scenes'] > 0, 'Train and val must be nonempty')
    require({k: counts[k]['scenes'] for k in ('train', 'val')} == inventory['split_counts'],
            'Split counts differ from manifest')
    require(sum(counts[k]['scenes'] for k in ('train', 'val')) == inventory['scenes'], 'Scene total differs')
    return dict(counts=counts, info_sha256=info_hashes, point_cloud_samples=point_samples,
                all_payload_hashes_verified=verify_hashes)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--check-config', action='store_true', help='Read-only config audit; no data/work needed')
    p.add_argument('--data', type=Path, help='Trusted portable dataset (never unpickle untrusted downloads)')
    p.add_argument('--work-dir', type=Path, help='New empty directory only; no resume')
    p.add_argument('--init-checkpoint', type=Path, help='Explicit compatible initializer; CPU records SHA only')
    p.add_argument('--sample-clouds', type=int, default=16, help='Uniformly sample this many clouds PER split')
    p.add_argument('--verify-payload-hashes', action='store_true')
    p.add_argument('--epochs', type=int, default=24)
    p.add_argument('--batch', type=int, default=4)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--lr', type=float, default=1e-4)
    p.add_argument('--seed', type=int, default=261015)
    p.add_argument('--wandb', action='store_true', help='Plan W&B logging; no login or network here')
    return p


def prepare(args):
    cfg, settings = check_config(args.config)
    if args.check_config:
        return dict(event='next_training_config_checked', settings=settings, training_started=False)
    require(args.data is not None and args.work_dir is not None and args.init_checkpoint is not None,
            'Provide --data, --work-dir and explicit --init-checkpoint')
    require(args.epochs > 0 and args.batch > 0 and args.workers >= 0 and np.isfinite(args.lr) and args.lr > 0,
            'Invalid training budget')
    data, work = args.data.resolve(strict=True), args.work_dir.resolve()
    require(not work.is_relative_to(data) and not data.is_relative_to(work), 'Work must be separate from data')
    if work.exists() and (not work.is_dir() or any(work.iterdir())):
        raise FileExistsError('Use a NEW empty work-dir; old E24/resume contracts are preserved')
    initializer = args.init_checkpoint.resolve(strict=True)
    require(initializer.is_file() and initializer.stat().st_size > 0, 'Initializer must be a nonempty file')
    dataset = check_data(data, cfg, sample_clouds=args.sample_clouds, verify_hashes=args.verify_payload_hashes)
    diagnostics = ROOT / 'scripts/runtime/base5_distance_diagnostics.py'
    command = ['python3', str(diagnostics), '--frozen-entrypoint',
               str(ROOT / 'perception/detection3d/bevfusion/tools/run_base5_direct_training.py'),
               '--config', str(args.config.resolve()), '--data', str(data), '--profile', '512x1056',
               '--work-dir', str(work), '--init-checkpoint', str(initializer),
               '--epochs', str(args.epochs), '--batch', str(args.batch), '--accumulate', '1',
               '--workers', str(args.workers), '--lr', str(args.lr), '--seed', str(args.seed)]
    if args.wandb:
        command.append('--wandb')
    report = dict(schema='base5_next_training_cpu_plan_v1', settings=settings, data=dataset,
                  dataset_manifest_sha256=sha256(data / 'manifest.json'),
                  config_files_sha256=config_hashes(args.config),
                  preparer_sha256=sha256(Path(__file__)), initializer_sha256=sha256(initializer),
                  distance_diagnostics_sha256=sha256(diagnostics),
                  initializer_shape_qualified=False, resume_allowed=False, training_started=False,
                  direct_prepare_command=shlex.join(command),
                  qualification='CPU audit only; target-GPU initialization/cost/resume/backup qualification remains')
    report['plan_sha256'] = hashlib.sha256(json.dumps(report, sort_keys=True).encode()).hexdigest()
    work.mkdir(parents=True, exist_ok=True)
    cfg.dump(str(work / 'checked_model_config.py'))
    (work / 'next_training_preflight.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return dict(event='next_training_cpu_prepared', settings=settings, counts=dataset['counts'],
                point_cloud_samples=len(dataset['point_cloud_samples']),
                plan_sha256=report['plan_sha256'], work_dir=str(work), training_started=False)


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        result = prepare(args)
    except (ValueError, FileExistsError, FileNotFoundError) as error:
        p.exit(2, str(error) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
