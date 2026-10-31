#!/usr/bin/env python3
"""Bounded train/val profiling of the current full-cohort Base5 configuration.

Reconstructs configuration through the original, ordered source prepare function
and reviewed full-data adapter. Never constructs a model from a sorted resolved
dump, writes a training checkpoint, contacts W&B, or edits the original run.
Only trusted local checkpoints are supported (full MMEngine pickle state).
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.dont_write_bytecode = True
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
QUALIFIED_CONTRACT = '7de3d916b0080074bbdbe62b97f714c398579ad2f9bf638c60719a8b18a0960f'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 2**20), b''):
            value.update(chunk)
    return value.hexdigest()


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def summary(values):
    values = np.asarray(values, dtype=np.float64)
    return {name: float(value) for name, value in (
        ('mean', np.mean(values)), ('p50', np.percentile(values, 50)),
        ('p90', np.percentile(values, 90)), ('p95', np.percentile(values, 95)),
        ('max', np.max(values)))} if values.size else {}


def open_fd_count():
    path = Path('/proc/self/fd')
    return len(list(path.iterdir())) if path.is_dir() else None


def evenly_spaced_indices(length, count):
    require(length > 0 and 0 < count <= length, 'Invalid bounded subset size')
    return np.linspace(0, length - 1, count, dtype=np.int64).tolist()


def cohort_indices(dataset, count):
    """Round-robin uniformly within every nonempty original Concat child."""
    sizes = [len(child) for child in getattr(dataset, 'datasets', [dataset])]
    require(0 < count <= sum(sizes), 'Subset exceeds complete dataset')
    allocations = [0] * len(sizes)
    while sum(allocations) < count:
        for i, size in enumerate(sizes):
            if allocations[i] < size and sum(allocations) < count:
                allocations[i] += 1
    result = []
    offset = 0
    for size, n in zip(sizes, allocations):
        if n:
            result.extend(offset + i for i in evenly_spaced_indices(size, n))
        offset += size
    return result


def compare_predictions(reference, candidate, match_radius=1.):
    """Same-token/class one-to-one XY matching; never hide unmatched queries."""
    require([r['token'] for r in reference] == [r['token'] for r in candidate],
            'Validation tokens/order differ')
    deltas = {name: [] for name in ('xy_m', 'z_m', 'size_m', 'yaw_rad', 'score')}
    unmatched_reference = unmatched_candidate = exact_rows = matched = 0
    supported = {name: {'reference': 0, 'candidate': 0} for name in CLASSES}
    for left, right in zip(reference, candidate):
        require(np.array_equal(left['gt_boxes'], right['gt_boxes'])
                and np.array_equal(left['gt_labels'], right['gt_labels']), 'GT changed')
        exact_rows += int(all(np.array_equal(left[key], right[key]) for key in
                              ('pred_boxes', 'pred_labels', 'pred_scores')))
        for label, name in enumerate(CLASSES):
            li = np.flatnonzero(left['pred_labels'] == label)
            ri = np.flatnonzero(right['pred_labels'] == label)
            supported[name]['reference'] += len(li)
            supported[name]['candidate'] += len(ri)
            taken = set()
            for index in li[np.argsort(-left['pred_scores'][li], kind='stable')]:
                available = [j for j in ri if int(j) not in taken]
                if not available:
                    unmatched_reference += 1
                    continue
                distance = np.linalg.norm(right['pred_boxes'][available, :2]
                                          - left['pred_boxes'][index, :2], axis=1)
                best = int(np.argmin(distance))
                if distance[best] >= match_radius:
                    unmatched_reference += 1
                    continue
                other = int(available[best]); taken.add(other); matched += 1
                a, b = left['pred_boxes'][index], right['pred_boxes'][other]
                deltas['xy_m'].append(float(distance[best]))
                deltas['z_m'].append(abs(float(a[2] - b[2])))
                deltas['size_m'].append(float(np.max(np.abs(a[3:6] - b[3:6]))))
                deltas['yaw_rad'].append(abs(float((a[6] - b[6] + np.pi) % (2*np.pi) - np.pi)))
                deltas['score'].append(abs(float(left['pred_scores'][index]
                                                       - right['pred_scores'][other])))
            unmatched_candidate += len(ri) - len(taken)
    return dict(frames=len(reference), exact_frames=exact_rows, matched=matched,
                unmatched_reference=unmatched_reference, unmatched_candidate=unmatched_candidate,
                class_prediction_counts=supported, match_radius_m=match_radius,
                deltas={name: summary(values) for name, values in deltas.items()})


def qualified_metadata(args, adapter, config_path):
    """Reuse a pinned completed admission for profiling only; no payload walk."""
    import pickle
    require(digest(args.reuse_qualified_manifest) == args.qualified_manifest_sha256,
            'Previously qualified manifest SHA differs')
    previous = json.loads(args.reuse_qualified_manifest.read_text())
    identity = previous['identity']
    calculated = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    require(previous['contract_sha256'] == calculated == QUALIFIED_CONTRACT,
            'Only the explicit completed full30750 contract may reuse admission')
    require(digest(args.data_index) == identity['dataset_manifest_sha256']
            and digest(config_path) == identity['config_sha256']
            and digest(args.initializer) == identity['initial_checkpoint_sha256'],
            'Original index/config/initializer differs')
    for relative, expected in identity['source_hashes'].items():
        require(digest(args.source/relative) == expected, 'Original source differs: '+relative)
    index = json.loads(args.data_index.read_text())
    require(index.get('complete') is True
            and {r['name']: r['expected_scenes'] for r in index['cohorts']} == adapter.EXPECTED,
            'Complete qualified cohort set differs')
    indexed = {row['name']: row for row in index['cohorts']}
    tokens = {}; products = copy.deepcopy(identity['dataset_cohorts'])
    counts = dict(train=0, val=0, test=0)
    for cohort in products:
        item = indexed[cohort['name']]; root = adapter.safe(cohort['root'], True)
        require(str(root) == item['root'] and digest(root/'manifest.json')
                == cohort['manifest_sha256'] == item['manifest_sha256'],
                'Qualified cohort root/manifest differs')
        for split in counts:
            path = root/'infos/512x1056'/f'morai_base5_infos_{split}.pkl'
            require(digest(path) == cohort['info_sha256'][split], 'Qualified infos changed')
            counts[split] += cohort['split_counts'][split]
            if split != 'val': continue
            rows = pickle.loads(path.read_bytes())['data_list']
            require(len(rows) == cohort['split_counts']['val'], 'Qualified val count differs')
            for row in rows:
                token = row['token']
                require(token not in tokens and row.get('sample_idx') == token
                        and Path(row['lidar_points']['lidar_path']).parts.count(token) == 1,
                        'Qualified token identity differs')
                tokens[token] = cohort['name']
    require(counts == adapter.EXPECTED_COUNTS and len(tokens) == counts['val'],
            'Qualified split/frame count differs')
    return dict(index=str(args.data_index), index_sha256=identity['dataset_manifest_sha256'],
        cohorts=products, counts=counts, train_count=counts['train'],
        payload_hashes_verified=False, global_tokens=sum(counts.values()),
        group_split_leakage=0, original_payloads_copied=0, val_token_to_cohort=tokens)


def reconstruct(args):
    """Do not read resolved_train.py: SeparateHead insertion order is semantic."""
    source = args.source.resolve(strict=True)
    adapter = load_module(args.adapter.resolve(strict=True), '_profile_full_data_adapter')
    core_path = source / adapter.CORE_REL
    require(digest(core_path) == adapter.CORE_SHA, 'Original core SHA differs')
    sys.path[:0] = [str(source), str(source/'third_party/mmdetection3d'), str(core_path.parent)]
    from scripts.runtime import base5_distance_diagnostics
    base5_distance_diagnostics.install()
    extension = load_module(args.adapter.with_name('base5_val_logging.py'), '_profile_val_logging')
    require(digest(args.adapter.with_name('base5_val_logging.py')) == adapter.VAL_LOGGING_SHA,
            'Actual validation extension SHA differs')
    extension.install()
    if args.owned_storage_extension is not None:
        require(digest(args.owned_storage_extension) == args.owned_storage_sha256,
                'Owned-storage extension SHA differs from explicit qualification')
        load_module(args.owned_storage_extension, '_profile_owned_storage').install()
    core = load_module(core_path, '_profile_original_core')
    setup = argparse.Namespace(source=source, data_index=args.data_index,
        data=None, work_dir=args.output/'prepared_contract', init_checkpoint=args.initializer,
        epochs=args.epochs, seed=args.seed, wandb=False, resume=None, run=False,
        config=source/'perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py',
        profile='512x1056', batch=4, accumulate=1, workers=4, lr=1e-4,
        warmup_updates=500, lr_schedule='cosine', disable_val=False,
        jpeg_cache=None, allow_random_init=False, wandb_group=adapter.GROUP)
    if args.reuse_qualified_manifest is not None:
        state = qualified_metadata(args, adapter, setup.config)
        def inspected(path, validate, *, verify_hashes=False):
            require(not verify_hashes and Path(path) == args.data_index,
                    'Benchmark-only metadata reuse must not admit an actual start')
            return copy.deepcopy(state)
        adapter.inspect_index = inspected
    cfg, manifest = adapter.adapted_prepare(core_path.read_text(), vars(core))(setup)
    if args.reuse_qualified_manifest is not None:
        require(manifest['contract_sha256'] == QUALIFIED_CONTRACT,
                'Reconstructed ordered config differs from qualified contract')
    require(cfg.model.get('compile_camera') is False and not cfg.get('compile', False),
            'Compile must remain OFF')
    # Profiles are deliberately isolated: no original work-file or checkpoint write.
    cfg.work_dir = str(args.output/'isolated_runner')
    cfg.visualizer.vis_backends = [dict(type='LocalVisBackend')]
    cfg.vis_backends = [dict(type='LocalVisBackend')]
    cfg.default_hooks.checkpoint = None
    cfg.default_hooks.logger.interval = max(args.steps + 1, 1000)
    cfg.load_from = str(args.checkpoint)
    cfg.resume = args.mode == 'train'
    cfg.test_cfg = cfg.test_dataloader = cfg.test_evaluator = None
    loader = cfg.train_dataloader if args.mode == 'train' else cfg.val_dataloader
    loader.update(num_workers=args.workers, persistent_workers=args.workers > 0,
                  pin_memory=args.pin_memory)
    if args.workers > 0: loader['prefetch_factor'] = args.prefetch
    else: loader.pop('prefetch_factor', None)
    return cfg, manifest


def samples_from_outputs(outputs):
    result = []
    for output in outputs:
        row = output.to_dict() if hasattr(output, 'to_dict') else output
        annotation, pred = row['eval_ann_info'], row['pred_instances_3d']
        def array(value):
            if hasattr(value, 'tensor'): value = value.tensor
            if hasattr(value, 'detach'): value = value.detach().cpu().numpy()
            return np.asarray(value).copy()
        result.append(dict(token=str(row['sample_idx']),
            gt_boxes=array(annotation['gt_bboxes_3d']), gt_labels=array(annotation['gt_labels_3d']),
            pred_boxes=array(pred['bboxes_3d']), pred_labels=array(pred['labels_3d']),
            pred_scores=array(pred['scores_3d'])))
    return result


def center_metrics(samples, point_cloud_range):
    from perception.detection3d.common.evaluation.morai_detection import (
        _evaluate_center, _filter_center_box_range, DEFAULT_CENTER_THRESHOLDS)
    simple = [{key: value for key, value in row.items() if key != 'token'} for row in samples]
    simple = _filter_center_box_range(simple, point_cloud_range)
    return _evaluate_center(simple, list(CLASSES), DEFAULT_CENTER_THRESHOLDS,
                            2., .1, .1, 'center', include_tp=False, include_operating_points=False)


class PlannedStop(Exception):
    pass


def train_profile(runner, args):
    import torch
    from mmengine.hooks import Hook

    class Bounded(Hook):
        priority = 'VERY_HIGH'
        def before_train(self, runner):
            self.rows = []; self.stage_rows = []; self.originals = []
            self.active = False; self.start_iter = runner.iter
            self.before_steps = sorted({float(value['step']) for value in
                runner.optim_wrapper.optimizer.state.values() if 'step' in value})
            require(self.before_steps, 'Train profiling requires a full resumable checkpoint')
            methods = [(runner.model.data_preprocessor, 'forward', 'input_transfer'),
                (runner.model.img_backbone, 'forward', 'camera_backbone'),
                (runner.model.img_neck, 'forward', 'camera_neck'),
                (runner.model.view_transform, 'get_geometry', 'geometry'),
                (runner.model.view_transform, 'get_cam_feats', 'depth_feature'),
                (runner.model.view_transform, 'bev_pool', 'pool_forward'),
                (runner.model.pts_middle_encoder, 'forward', 'sparse_lidar'),
                (runner.model.bbox_head, 'loss', 'head_and_targets'),
                (runner.model, 'parse_losses', 'scalar_logging'),
                (runner.optim_wrapper, 'backward', 'backward'),
                (runner.optim_wrapper, 'step', 'optimizer_and_clip')]
            for obj, name, label in methods:
                original = getattr(obj, name)
                def timed(*a, _fn=original, _label=label, **kw):
                    if not self.active: return _fn(*a, **kw)
                    begin, end = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
                    begin.record(); start = time.perf_counter()
                    with torch.profiler.record_function('BASE5::'+_label): value = _fn(*a, **kw)
                    end.record()
                    self.stage_rows.append((_label, begin, end, (time.perf_counter()-start)*1000))
                    return value
                self.originals.append((obj, name, original)); setattr(obj, name, timed)
            torch.cuda.synchronize(); self.previous = time.perf_counter()

        def before_train_iter(self, runner, batch_idx, data_batch=None):
            step = runner.iter - self.start_iter
            self.active = args.warmup <= step < args.warmup + args.stage_steps
            torch.cuda.synchronize(); self.begin = time.perf_counter()
            self.wait = (self.begin-self.previous)*1000
            if step == args.warmup: torch.cuda.reset_peak_memory_stats()

        def after_train_iter(self, runner, batch_idx, data_batch=None, outputs=None):
            torch.cuda.synchronize(); end = time.perf_counter()
            step = runner.iter-self.start_iter
            values = {key: float(value.detach().cpu()) if torch.is_tensor(value) else float(value)
                      for key, value in outputs.items()}
            require(all(np.isfinite(value) for value in values.values()), 'Nonfinite training logs')
            self.rows.append(dict(step=step, loader_wait_ms=self.wait,
                train_ms=(end-self.begin)*1000, total_ms=(end-self.previous)*1000,
                instrumented=self.active, logs=values,
                open_fds=open_fd_count(),
                allocated_mib=torch.cuda.max_memory_allocated()/2**20,
                reserved_mib=torch.cuda.max_memory_reserved()/2**20))
            self.previous = end
            if step+1 == args.steps:
                stages = {}
                for name, begin, finish, host in self.stage_rows:
                    target = stages.setdefault(name, dict(stream_span_ms=[], host_call_ms=[]))
                    target['stream_span_ms'].append(begin.elapsed_time(finish))
                    target['host_call_ms'].append(host)
                measured = [r for r in self.rows[args.warmup:] if not r['instrumented']]
                updates = sorted({float(value['step']) for value in runner.optim_wrapper.optimizer.state.values()
                                  if 'step' in value})
                require(updates == [value+args.steps for value in self.before_steps],
                        'Optimizer skipped or reordered a bounded update')
                self.report = dict(mode='train', valid=True, initial_iteration=self.start_iter,
                    steps=args.steps, warmup=args.warmup, measured_steps=len(measured),
                    optimizer_steps_before=self.before_steps, optimizer_steps_after=updates,
                    timing={key: summary([r[key] for r in measured]) for key in
                            ('loader_wait_ms', 'train_ms', 'total_ms')},
                    peak_allocated_mib=max(r['allocated_mib'] for r in self.rows[args.warmup:]),
                    peak_reserved_mib=max(r['reserved_mib'] for r in self.rows[args.warmup:]),
                    stages={name: {key: summary(values) for key, values in row.items()}
                            for name, row in stages.items()}, per_step=self.rows,
                    measurement='synchronized wall time; stage events include host idle and nested overlap',
                    original_checkpoint_modified=False, checkpoint_saved=False)
                for obj, name, original in self.originals: setattr(obj, name, original)
                raise PlannedStop()
    hook = Bounded(); runner.register_hook(hook)
    try: runner.train()
    except PlannedStop: return hook.report
    raise RuntimeError('Bounded train unexpectedly completed its epoch')


def val_profile(runner, cfg, args):
    import torch
    from torch.utils.data import Subset
    from mmengine.runner import Runner
    from mmdet3d.registry import METRICS
    runner._init_model_weights(); runner.load_or_resume(); runner.model.eval()
    # Build the identical full ordered test dataset once, then reuse it for cases.
    initial = Runner.build_dataloader(copy.deepcopy(cfg.val_dataloader), seed=args.seed)
    dataset = initial.dataset
    indices = cohort_indices(dataset, args.frames)
    subset = Subset(dataset, indices)
    del initial
    reference = None; cases = []
    options = [(batch, False) for batch in args.val_batches]
    if args.val_amp: options.append((1, True))
    for batch, amp in options:
        loader_cfg = copy.deepcopy(cfg.val_dataloader)
        loader_cfg.update(dataset=subset, batch_size=batch, num_workers=args.workers,
            persistent_workers=args.workers > 0, pin_memory=args.pin_memory)
        if args.workers > 0: loader_cfg['prefetch_factor'] = args.prefetch
        else: loader_cfg.pop('prefetch_factor', None)
        loader = Runner.build_dataloader(loader_cfg, seed=args.seed)
        metric = METRICS.build(copy.deepcopy(cfg.val_evaluator))
        metric.dataset_meta = dataset.metainfo
        rows = []; samples = []
        initial_fds = open_fd_count()
        torch.cuda.synchronize(); previous = time.perf_counter()
        torch.cuda.reset_peak_memory_stats()
        with torch.no_grad():
            for index, data in enumerate(loader):
                begin = time.perf_counter(); wait = (begin-previous)*1000
                with torch.autocast('cuda', enabled=amp, dtype=torch.float16):
                    predictions = runner.model.val_step(data)
                torch.cuda.synchronize(); computed = time.perf_counter()
                metric.process(data, [sample.to_dict() if hasattr(sample, 'to_dict') else sample
                                      for sample in predictions])
                outputs = samples_from_outputs(predictions)
                samples.extend(outputs); end = time.perf_counter()
                rows.append(dict(batch_index=index, frames=len(outputs), loader_wait_ms=wait,
                    compute_ms=(computed-begin)*1000, collect_ms=(end-computed)*1000,
                    total_ms=(end-previous)*1000, open_fds=open_fd_count(),
                    retained_metric_rows=len(metric.results)))
                previous = end
        require(len(samples) == args.frames, 'Validation subset incomplete')
        values = center_metrics(samples, cfg.val_evaluator.get('point_cloud_range'))
        measured = rows[min(args.val_warmup_batches, len(rows)-1):]
        case = dict(batch=batch, fp16=amp, frames=len(samples), workers=args.workers,
            prefetch=args.prefetch, pin_memory=args.pin_memory,
            timing_ms_per_frame={key: summary([r[key]/r['frames'] for r in measured])
                for key in ('loader_wait_ms', 'compute_ms', 'collect_ms', 'total_ms')},
            window_frames=sum(r['frames'] for r in measured),
            window_seconds=sum(r['total_ms'] for r in measured)/1000,
            peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
            peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
            initial_open_fds=initial_fds, final_open_fds=open_fd_count(),
            fd_scope='real metric.process retains all subset results until case end',
            metrics=values, per_batch=rows)
        if reference is None:
            reference = samples; baseline_metrics = values
        case['comparison_to_batch1_fp32'] = compare_predictions(reference, samples)
        case['metric_delta_to_batch1_fp32'] = {key: float(value-baseline_metrics[key])
                                              for key, value in values.items()}
        np.savez_compressed(args.output/f'val_b{batch}_{"amp" if amp else "fp32"}.npz',
                            samples=np.asarray(samples, dtype=object))
        cases.append(case)
        print(json.dumps({key: value for key, value in case.items() if key != 'per_batch'}), flush=True)
        iterator = getattr(loader, '_iterator', None)
        if iterator is not None: iterator._shutdown_workers()
        del loader, metric
    return dict(mode='val', valid=True, indices=indices, cases=cases,
                subset_cost_only=True, full_validation_AP_certified=False)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ('source', 'adapter', 'data-index', 'initializer', 'checkpoint', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--mode', choices=('train', 'val'), required=True)
    parser.add_argument('--epochs', type=int, default=12)
    parser.add_argument('--seed', type=int, default=261015)
    parser.add_argument('--steps', type=int, default=32)
    parser.add_argument('--warmup', type=int, default=8)
    parser.add_argument('--stage-steps', type=int, default=4)
    parser.add_argument('--frames', type=int, default=64)
    parser.add_argument('--val-batches', type=int, nargs='+', default=[1, 2, 4])
    parser.add_argument('--val-warmup-batches', type=int, default=2)
    parser.add_argument('--val-amp', action='store_true', help='Explicit approximate candidate only')
    parser.add_argument('--owned-storage-extension', type=Path,
                        help='Explicit separately audited result-storage-only metric patch')
    parser.add_argument('--owned-storage-sha256')
    parser.add_argument('--reuse-qualified-manifest', type=Path,
                        help='Explicit completed admission, profiling only; training gates unchanged')
    parser.add_argument('--qualified-manifest-sha256')
    parser.add_argument('--torch-threads', type=int, default=4)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--prefetch', type=int, default=2)
    parser.add_argument('--pin-memory', action='store_true')
    args = parser.parse_args(argv)
    require(16 <= args.steps <= 64 and 0 < args.warmup < args.steps,
            'Train budget must be 16..64 steps with a warmup')
    require(0 <= args.stage_steps <= 4 and args.warmup+args.stage_steps < args.steps,
            'At most four instrumented steps; leave ordinary measurements')
    require(16 <= args.frames <= 192 and args.val_batches[0] == 1
            and len(set(args.val_batches)) == len(args.val_batches)
            and set(args.val_batches) <= {1, 2, 4}, 'Bounded val starts at batch1 FP32')
    require(0 <= args.workers <= 8 and 1 <= args.prefetch <= 4
            and 0 <= args.val_warmup_batches <= 4, 'Bounded loader configuration required')
    require(1 <= args.torch_threads <= 16, 'Bounded CPU thread count required')
    require((args.owned_storage_extension is None) == (args.owned_storage_sha256 is None),
            'Owned-storage extension and exact SHA must be supplied together')
    if args.owned_storage_extension is not None:
        require(args.owned_storage_extension.is_absolute()
                and len(args.owned_storage_sha256) == 64,
                'Explicit absolute owned-storage extension and SHA required')
    require((args.reuse_qualified_manifest is None) == (args.qualified_manifest_sha256 is None),
            'Qualified manifest and exact SHA must be supplied together')
    if args.reuse_qualified_manifest is not None:
        require(args.reuse_qualified_manifest.is_absolute()
                and len(args.qualified_manifest_sha256) == 64,
                'Explicit absolute qualified manifest and SHA required')
    for name in ('source', 'adapter', 'data_index', 'initializer', 'checkpoint', 'output'):
        require(getattr(args, name).is_absolute(), 'Explicit absolute paths required')
    require(not args.output.exists() and args.checkpoint.is_file(), 'Fresh output/trusted checkpoint required')
    return args


def main(argv=None):
    args = parse_args(argv); args.output.mkdir(parents=True)
    import torch
    torch.set_num_threads(args.torch_threads)
    cfg, manifest = reconstruct(args)
    from mmengine.utils import import_modules_from_strings
    import_modules_from_strings(**cfg.custom_imports)
    from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
    if args.mode == 'train':
        cfg.val_cfg = cfg.val_dataloader = cfg.val_evaluator = None
    runner = Base5AtomicRunner.from_cfg(cfg)
    report = train_profile(runner, args) if args.mode == 'train' else val_profile(runner, cfg, args)
    report.update(contract_sha256=manifest['contract_sha256'],
        source=str(args.source), checkpoint=str(args.checkpoint),
        compile_camera=False, wandb_enabled=False, original_run_modified=False,
        torch_num_threads=torch.get_num_threads(),
        owned_storage_extension_sha256=args.owned_storage_sha256,
        qualified_manifest_sha256=args.qualified_manifest_sha256,
        arguments={key: str(value) if isinstance(value, Path) else value
                   for key, value in vars(args).items()})
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ('per_step', 'cases')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
