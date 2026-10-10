#!/usr/bin/env python3
"""Prepare/run Base5 training INSIDE the existing environment (no docker run).

Explicit --run required; final shape/batch/epochs are an operator choice, not a
production promotion. Do not expose secrets in arguments or run manifests.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

from mmengine.config import Config
from mmengine.utils import import_modules_from_strings

from prepare_base5_portable_dataset import validate, sha256


def git_revision(root):
    result = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True, capture_output=True)
    return result.stdout.strip() if result.returncode == 0 else 'source-snapshot-without-git'


def prepare(args):
    root = args.data.resolve(strict=True)
    # Preparation can be cheap; an actual start/resume must also check payload
    # contents, not just filenames and byte counts from a stale manifest.
    inventory = validate(root, verify_hashes=getattr(args, 'run', False))
    if args.profile not in inventory['profiles']:
        raise ValueError('Profile not exported')
    if args.batch < 1 or args.accumulate < 1 or args.epochs < 1 or args.workers < 0 or args.lr <= 0:
        raise ValueError('Invalid training budget')
    import pickle
    train_info = root / 'infos' / args.profile / 'morai_base5_infos_train.pkl'
    count = len(pickle.loads(train_info.read_bytes())['data_list'])
    iterations = math.ceil(count / args.batch)
    # An epoch-boundary checkpoint cannot serialize in-flight accumulation
    # gradients. MMEngine accumulation can straddle epochs: prohibit that.
    if iterations % args.accumulate:
        raise ValueError('Epoch iterations must divide accumulation for epoch-boundary resume; use accumulate=1 or choose a compatible batch')
    cfg = Config.fromfile(str(args.config.resolve(strict=True)))
    if tuple(cfg.classes) != ('vehicle', 'pedestrian', 'obstacle', 'animal'):
        raise ValueError('Four-class contract required')
    height, width = map(int, args.profile.split('x'))
    if list(cfg.model.view_transform.image_size) != [height, width]:
        raise ValueError('Config/crop infos differ')
    # Regression code has eight channels (yaw is sin/cos), decoded box has 7D.
    if cfg.model.type != 'BEVFusion' or 'vel' in cfg.model.bbox_head.common_heads or cfg.model.bbox_head.bbox_coder.code_size != 8:
        raise ValueError('Expected single-frame 7D detector without OD velocity')
    for split in ('train', 'val'):
        loader = cfg[split + '_dataloader']
        loader.batch_size = args.batch if split == 'train' else 1
        loader.num_workers = args.workers
        loader.persistent_workers = args.workers > 0
        loader.dataset.data_root = str(root)
        loader.dataset.data_prefix = dict(pts='', img='')
        loader.dataset.ann_file = str(root / 'infos' / args.profile / ('morai_base5_infos_' + split + '.pkl'))
        # These are intentionally single-frame, even if a parent supports sweeps.
        if any('Sweep' in item['type'] or 'Temporal' in item['type'] for item in loader.dataset.pipeline):
            raise ValueError('Unexpected temporal transform')
    jpeg_cache = getattr(args, 'jpeg_cache', None)
    jpeg_cache_identity = None
    image_policy = inventory.get('image_policy')
    if image_policy:
        if image_policy != dict(train='native_jpeg90', val='original_png', test='original_png'):
            raise ValueError('Unsupported portable image policy')
        if jpeg_cache:
            raise ValueError('Preencoded portable data must not use an additional JPEG cache')
        from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import preencoded_train_pipeline
        cfg.train_dataloader.dataset.pipeline = preencoded_train_pipeline(cfg.train_dataloader.dataset.pipeline)
    if jpeg_cache:
        from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import (
            cache_train_pipeline, validate_jpeg_cache)
        train_rows = pickle.loads(train_info.read_bytes())['data_list']
        required = [root / camera['img_path'] for row in train_rows for camera in row['images'].values()]
        validate_jpeg_cache(root, jpeg_cache, required, verify_hashes=getattr(args, 'run', False))
        cfg.train_dataloader.dataset.pipeline = cache_train_pipeline(
            cfg.train_dataloader.dataset.pipeline, root, jpeg_cache)
        jpeg_cache_identity = sha256(Path(jpeg_cache) / 'manifest.json')
    cfg.test_dataloader = None; cfg.test_evaluator = None; cfg.test_cfg = None
    cfg.train_cfg = dict(type='EpochBasedTrainLoop', max_epochs=args.epochs, val_interval=1)
    cfg.optim_wrapper.accumulative_counts = args.accumulate
    cfg.optim_wrapper.optimizer.lr = args.lr
    total_iterations = iterations * args.epochs
    warmup = min(args.warmup_updates * args.accumulate, max(1, total_iterations // 10))
    cfg.param_scheduler = [dict(type='LinearLR', start_factor=.1, by_epoch=False, begin=0, end=warmup),
        dict(type='CosineAnnealingLR', by_epoch=False, begin=warmup, end=total_iterations,
             T_max=max(1, total_iterations - warmup), eta_min=args.lr / 100)]
    cfg.default_hooks.checkpoint.update(interval=1, by_epoch=True, max_keep_ckpts=3,
        save_optimizer=True, save_param_scheduler=True)
    if args.disable_val:
        cfg.val_dataloader = None; cfg.val_evaluator = None; cfg.val_cfg = None
        cfg.default_hooks.checkpoint.pop('save_best', None)
        cfg.default_hooks.checkpoint.pop('rule', None)
    cfg.randomness = dict(seed=args.seed, deterministic=False)
    cfg.work_dir = str(args.work_dir.absolute())
    cfg.launcher = 'none'
    cfg.load_from = None
    cfg.resume = False
    backends = [dict(type='LocalVisBackend')]
    if args.wandb:
        # Entity/key are supplied by environment/account, not a teammate's
        # hardcoded account inherited from Base3. No credentials in config.
        backends.append(dict(type='MoraiWandbVisBackend', init_kwargs=dict(
            project='morai-3d-object-detection', name=args.work_dir.name,
            group='base5-sunny13', job_type='train', id=args.work_dir.name, resume='allow')))
    cfg.visualizer.vis_backends = backends
    cfg.vis_backends = backends
    cfg.custom_imports.imports.append('perception.detection3d.bevfusion.plugin.morai_remote_checkpoint')
    if args.init_checkpoint:
        cfg.load_from = str(args.init_checkpoint.resolve(strict=True))
    elif not args.resume and not args.allow_random_init:
        raise ValueError('Provide audited --init-checkpoint; --allow-random-init is for explicit probes only')
    # Hash portable identities, not host-specific roots. Include every code
    # input under the detector/plugin + fork/ops: resume cannot change code.
    repo = Path(__file__).resolve().parents[4]
    folders = ('perception/detection3d/bevfusion', 'perception/detection3d/common',
               'third_party/mmdetection3d')
    source_files = sorted({path for folder in folders for path in (repo / folder).rglob('*')
                           if path.is_file() and path.suffix in ('.py', '.cu', '.cpp', '.h', '.cuh')
                           and not set(path.parts) & {'build', '__pycache__', '.git', '.mim'}
                           and not any(part.endswith('.egg-info') for part in path.parts)})
    source_hashes = {str(path.relative_to(repo)): sha256(path) for path in source_files}
    from importlib.metadata import version
    packages = ('torch', 'mmengine', 'mmcv', 'mmdet', 'mmdet3d', 'spconv-cu120', 'numpy')
    identity = dict(dataset_manifest_sha256=sha256(root / 'manifest.json'), profile=args.profile,
        config_sha256=sha256(args.config), source_hashes=source_hashes,
        package_versions={name: version(name) for name in packages}, model=cfg.model.to_dict(),
        train_pipeline=list(cfg.train_dataloader.dataset.pipeline),
        epochs=args.epochs, batch=args.batch, accumulate=args.accumulate, workers=args.workers,
        lr=args.lr, warmup_iterations=warmup, seed=args.seed, val_enabled=not args.disable_val,
        train_count=count, iterations_per_epoch=iterations,
        initial_checkpoint_sha256=sha256(args.init_checkpoint) if args.init_checkpoint else None)
    if jpeg_cache_identity:
        identity['jpeg_cache_manifest_sha256'] = jpeg_cache_identity
    previous_path = args.work_dir / 'run_manifest.json'
    if args.resume:
        if not previous_path.is_file():
            raise ValueError('Resume needs the original work_dir/run_manifest.json')
        previous = json.loads(previous_path.read_text())
        if not args.init_checkpoint:
            identity['initial_checkpoint_sha256'] = previous['identity']['initial_checkpoint_sha256']
    contract = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    for backend in cfg.visualizer.vis_backends:
        if backend['type'] == 'MoraiWandbVisBackend':
            backend.init_kwargs.id = args.work_dir.name + '-' + contract[:12]
    if args.resume:
        if previous['contract_sha256'] != contract:
            raise ValueError('Resume contract changed; do not silently change dataset/schedule/code')
        last = args.work_dir / 'last_checkpoint'
        checkpoint = Path(args.resume) if args.resume != 'auto' else Path(last.read_text().strip())
        cfg.resume = True; cfg.load_from = str(checkpoint.resolve(strict=True))
    elif previous_path.exists():
        previous = json.loads(previous_path.read_text())
        started = (args.work_dir / 'fresh_start.json').exists() or (args.work_dir / 'last_checkpoint').exists() or any(args.work_dir.glob('*.pth'))
        if started or previous.get('contract_sha256') != contract:
            raise FileExistsError('Existing started/changed run: use --resume or a fresh work-dir')
    cfg.custom_hooks.append(dict(type='MoraiResumeEvidenceHook', contract_sha256=contract))
    manifest = dict(version=1, identity=identity, contract_sha256=contract,
        git_revision=git_revision(repo), effective_batch=args.batch * args.accumulate,
        optimizer_updates_per_epoch=iterations // args.accumulate,
        resume_scope='epoch-boundary only; no exact worker/prefetch/mid-epoch replay claim',
        paths=dict(data=str(root), config=str(args.config.absolute())))
    args.work_dir.mkdir(parents=True, exist_ok=True)
    if not args.resume and not previous_path.exists():
        previous_path.write_text(json.dumps(manifest, indent=2) + '\n')
    cfg.dump(str(args.work_dir / ('resolved_resume.py' if args.resume else 'resolved_train.py')))
    return cfg, manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--jpeg-cache', type=Path,
                   help='Optional verified native Q90 train cache; original PNG is preserved and val stays clean')
    p.add_argument('--profile', choices=['448x896', '512x1056'], required=True)
    p.add_argument('--work-dir', type=Path, required=True)
    p.add_argument('--epochs', type=int, required=True)
    p.add_argument('--batch', type=int, default=1)
    p.add_argument('--accumulate', type=int, default=1)
    p.add_argument('--workers', type=int, default=4)
    p.add_argument('--lr', type=float, default=1e-4)
    p.add_argument('--warmup-updates', type=int, default=500)
    p.add_argument('--seed', type=int, default=261010)
    p.add_argument('--init-checkpoint', type=Path)
    p.add_argument('--allow-random-init', action='store_true')
    p.add_argument('--resume', nargs='?', const='auto')
    p.add_argument('--disable-val', action='store_true', help='For bounded wiring probes only')
    p.add_argument('--wandb', action='store_true')
    p.add_argument('--run', action='store_true')
    a = p.parse_args()
    cfg, manifest = prepare(a)
    print(json.dumps({k: manifest[k] for k in ('contract_sha256', 'effective_batch', 'optimizer_updates_per_epoch', 'resume_scope')}, indent=2))
    if a.run:
        import_modules_from_strings(**cfg.custom_imports)
        from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
        Base5AtomicRunner.from_cfg(cfg).train()
    else:
        print('Prepared only; add --run after initialization/cost/resume qualification.')


if __name__ == '__main__':
    main()
