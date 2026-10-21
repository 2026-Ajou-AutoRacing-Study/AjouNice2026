#!/usr/bin/env python3
"""Bounded, CPU-only actual Base5 Concat input profile; never builds a model.

Use a separate output path. Source, resolved config, datasets and train work are
read-only. Per-global-index RNG makes augmented samples comparable across loader
worker/batch variants; this is a diagnostic sampler, not a training replacement.
"""
import argparse
import copy
import gc
import hashlib
import json
import os
from pathlib import Path
import random
import resource
import sys
import time


def select_indices(dataset, count):
    """Stable, unique, evenly spread indices, with every nonempty child covered."""
    total = len(dataset)
    lengths = [len(child) for child in getattr(dataset, 'datasets', [dataset])]
    if not 1 <= count <= min(total, 96) or count < sum(n > 0 for n in lengths):
        raise ValueError('Require child coverage and at most 96 actual samples')
    chosen = set()
    offset = 0
    for length in lengths:
        if length:
            chosen.add(offset + (length - 1) // 2)
        offset += length
    # Fill from an increasingly fine regular grid; no random scene substitution.
    grid = count
    while len(chosen) < count:
        for i in range(grid):
            chosen.add(round(i * (total - 1) / max(1, grid - 1)))
            if len(chosen) == count:
                break
        grid *= 2
    return sorted(chosen)


def payload_digest(value):
    """Hash exact CPU payload, including all GT and geometric metadata."""
    import numpy as np
    digest = hashlib.sha256()

    def visit(item):
        if hasattr(item, 'detach') and hasattr(item, 'numpy'):
            array = item.detach().cpu().contiguous().numpy()
            visit(array)
        elif isinstance(item, np.ndarray):
            digest.update(str(item.dtype).encode())
            digest.update(str(item.shape).encode())
            if item.dtype.hasobject:
                visit(item.tolist())
            else:
                digest.update(np.ascontiguousarray(item).tobytes())
        elif isinstance(item, dict):
            for key in sorted(item, key=str):
                digest.update(str(key).encode()); visit(item[key])
        elif isinstance(item, (list, tuple)):
            digest.update(str(len(item)).encode())
            for child in item:
                visit(child)
        elif isinstance(item, type):
            digest.update((item.__module__ + '.' + item.__qualname__).encode())
        elif hasattr(item, 'to_dict'):
            visit(item.to_dict())
        elif hasattr(item, 'tensor'):
            visit(item.tensor)
        elif item is None or isinstance(item, (str, bool, int, float, np.generic)):
            digest.update(repr(item).encode())
        else:
            raise TypeError('Unsupported exact payload hash type: ' + str(type(item)))

    visit(value)
    return digest.hexdigest()


class SeededSubset:
    """Keep exact data/GT; make augmentation identical for each global index."""
    def __init__(self, dataset, indices, seed):
        self.dataset, self.indices, self.seed = dataset, list(indices), seed
        self.getitem_times_ms = []

    def __len__(self):
        return len(self.indices)

    def __getitem__(self, position):
        import numpy as np
        import torch
        index = self.indices[position]
        states = random.getstate(), np.random.get_state(), torch.get_rng_state()
        try:
            seed = (self.seed + index * 1000003) % (2 ** 32)
            random.seed(seed); np.random.seed(seed)
            # CPU generator only: never initialize CUDA inside a worker.
            torch.random.default_generator.manual_seed(seed)
            started = time.perf_counter()
            value = self.dataset[index]
            self.getitem_times_ms.append((time.perf_counter()-started)*1000)
            if value is None:
                raise ValueError('Original pipeline returned None; no substitution allowed')
            return value
        finally:
            random.setstate(states[0]); np.random.set_state(states[1]); torch.set_rng_state(states[2])


class TimedCallable:
    """Diagnostic wrapper: forward the exact same objects, return unmodified."""
    def __init__(self, original, records, label):
        self.original, self.records, self.label = original, records, label

    def __call__(self, *args, **kwargs):
        wall, cpu = time.perf_counter(), time.process_time()
        value = self.original(*args, **kwargs)
        self.records.setdefault(self.label, []).append(dict(
            wall_ms=(time.perf_counter()-wall)*1000,
            cpu_ms=(time.process_time()-cpu)*1000))
        return value


def instrument_cpu_pipeline(dataset, records):
    """Only use in workers=0: retain image/point/calibration and RNG semantics."""
    for child in getattr(dataset, 'datasets', [dataset]):
        child.get_data_info = TimedCallable(child.get_data_info, records, 'get_data_info')
        child.pipeline.transforms = [TimedCallable(transform, records, type(transform).__name__)
                                     for transform in child.pipeline.transforms]


def process_memory():
    """PSS avoids double counting fork-shared serialized dataset buffers."""
    pids = [os.getpid()]
    for pid in pids:
        children = Path('/proc') / str(pid) / 'task' / str(pid) / 'children'
        try:
            pids.extend(int(x) for x in children.read_text().split())
        except OSError:
            pass
    records = []
    for pid in pids:
        try:
            fields = dict(line.split(':', 1) for line in (Path('/proc') / str(pid) / 'smaps_rollup').read_text().splitlines() if ':' in line)
            records.append(dict(pid=pid, **{key + '_kib': int(fields[key].split()[0])
                                           for key in ('Rss', 'Pss', 'Swap') if key in fields}))
        except OSError:
            pass
    return records


def resource_snapshot():
    result = dict(time=time.time(), processes=process_memory())
    result['main_fd_count'] = len(os.listdir('/proc/self/fd'))
    result['nofile_soft_hard'] = list(resource.getrlimit(resource.RLIMIT_NOFILE))
    for process in result['processes']:
        try:
            process['fd_count'] = len(os.listdir('/proc/' + str(process['pid']) + '/fd'))
        except OSError:
            process['fd_count'] = None
    result['total_pss_bytes'] = 1024 * sum(row.get('Pss_kib', 0) for row in result['processes'])
    result['total_swap_bytes'] = 1024 * sum(row.get('Swap_kib', 0) for row in result['processes'])
    shm = os.statvfs('/dev/shm')
    result['shm_free_bytes'] = shm.f_bavail * shm.f_frsize
    for name, path in [('cpu_pressure', '/proc/pressure/cpu'), ('memory_pressure', '/proc/pressure/memory'),
                       ('io_pressure', '/proc/pressure/io'), ('cpu_stat', '/sys/fs/cgroup/cpu.stat'),
                       ('cpu_max', '/sys/fs/cgroup/cpu.max'), ('memory_current', '/sys/fs/cgroup/memory.current'),
                       ('memory_max', '/sys/fs/cgroup/memory.max'), ('memory_events', '/sys/fs/cgroup/memory.events')]:
        try:
            result[name] = Path(path).read_text().strip()
        except OSError:
            result[name] = None
    return result


def guard(max_ram_gib):
    snapshot = resource_snapshot()
    if snapshot['total_pss_bytes'] > max_ram_gib * 2 ** 30:
        raise RuntimeError('Diagnostic PSS exceeds bounded RAM budget')
    if snapshot['shm_free_bytes'] < 512 * 2 ** 20:
        raise RuntimeError('Less than 512 MiB shared memory free')
    return snapshot


def summary(values):
    import numpy as np
    return {key: float(fn(values)) for key, fn in [('mean', np.mean), ('p50', np.median),
                ('p90', lambda x: np.percentile(x, 90)), ('p95', lambda x: np.percentile(x, 95)),
                ('max', np.max)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--split', choices=['train', 'val'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=64)
    parser.add_argument('--workers', type=int, choices=[0, 2, 4, 8], default=4)
    parser.add_argument('--batch', type=int, choices=[1, 2, 4], default=1)
    parser.add_argument('--prefetch', type=int, choices=[1, 2], default=2)
    parser.add_argument('--pin-memory', action='store_true')
    parser.add_argument('--seed', type=int, default=261015)
    parser.add_argument('--max-ram-gib', type=float, default=16)
    args = parser.parse_args(argv)
    if args.output.exists() or not 1 <= args.samples <= 96 or not 1 <= args.max_ram_gib <= 16:
        parser.error('Require fresh output, <=96 samples and <=16 GiB RAM budget')
    args.source = args.source.resolve(strict=True); args.config = args.config.resolve(strict=True)
    sys.path[:0] = [str(args.source), str(args.source / 'third_party/mmdetection3d')]
    import numpy as np
    import torch
    import cv2
    from mmengine.config import Config
    from mmengine.dataset import BaseDataset
    from mmengine.runner import Runner
    from mmengine.utils import import_modules_from_strings
    from mmdet3d.utils import register_all_modules
    torch.set_num_threads(4); cv2.setNumThreads(0)
    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(str(args.config)); import_modules_from_strings(**cfg.custom_imports)
    baseline_resources = guard(args.max_ram_gib)
    initializations = []
    original_init = BaseDataset.full_init

    def measured_init(dataset):
        before = time.perf_counter(); was_initialized = getattr(dataset, '_fully_initialized', False)
        value = original_init(dataset)
        if not was_initialized:
            initializations.append(dict(type=type(dataset).__name__, seconds=time.perf_counter()-before,
                samples=len(dataset), serialize_data=bool(dataset.serialize_data),
                serialized_bytes=int(getattr(dataset, 'data_bytes', np.array([])).nbytes),
                resources=guard(args.max_ram_gib)))
        return value

    BaseDataset.full_init = measured_init
    loader_cfg = copy.deepcopy(cfg[args.split + '_dataloader'])
    loader_cfg.update(batch_size=args.batch, num_workers=args.workers,
                      persistent_workers=False, pin_memory=args.pin_memory)
    loader_cfg.pop('prefetch_factor', None)
    if args.workers:
        loader_cfg['prefetch_factor'] = args.prefetch
    started = time.perf_counter()
    try:
        from mmdet3d.registry import DATASETS
        dataset = DATASETS.build(loader_cfg.pop('dataset'))
    finally:
        BaseDataset.full_init = original_init
    build_seconds = time.perf_counter() - started
    indices = select_indices(dataset, min(args.samples, len(dataset)))
    transform_records = {}
    if not args.workers:
        instrument_cpu_pipeline(dataset, transform_records)
    subset = SeededSubset(dataset, indices, args.seed)
    loader_cfg['dataset'] = subset
    # Explicit deterministic diagnostic order, not the original train shuffle.
    loader_cfg['sampler'] = dict(type='DefaultSampler', shuffle=False)
    loader = Runner.build_dataloader(loader_cfg, seed=args.seed)
    rows, fingerprints, retained_batches = [], [], []
    before_iteration = guard(args.max_ram_gib)
    iterator = iter(loader)
    try:
        for step in range(len(loader)):
            before = time.perf_counter(); batch = next(iterator); loaded = time.perf_counter()
            retained_batches.append(batch)
            # Keep <=96 uint8 examples until after the measured drain. Hashing
            # during next() timing would let prefetch hide the actual wait.
            resources = guard(args.max_ram_gib) if step % 8 == 0 else None
            rows.append(dict(step=step, samples=len(batch['data_samples']), loader_wait_ms=1000*(loaded-before),
                             resource_guard_ms=1000*(time.perf_counter()-loaded), resources=resources,
                             main_fd_count=len(os.listdir('/proc/self/fd'))))
    finally:
        if hasattr(iterator, '_shutdown_workers'):
            iterator._shutdown_workers()
    hash_start = time.perf_counter()
    for batch in retained_batches:
        for i in range(len(batch['data_samples'])):
            fingerprints.append(payload_digest(dict(
                inputs={key: value[i] for key, value in batch['inputs'].items()},
                data_samples=batch['data_samples'][i])))
    hash_seconds = time.perf_counter() - hash_start
    retained_resources = guard(args.max_ram_gib)
    retained_batches.clear()
    del batch
    gc.collect()
    result = dict(schema='base5_bounded_input_profile_v1', split=args.split,
        source=str(args.source), config=str(args.config), config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
        samples=len(indices), indices=indices, sample_payload_sha256=fingerprints,
        workers=args.workers, batch=args.batch, prefetch=args.prefetch if args.workers else None,
        pin_memory=args.pin_memory, persistent_workers=False, seed=args.seed,
        diagnostic_hash_seconds=hash_seconds,
        single_process_getitem_ms=summary(subset.getitem_times_ms) if not args.workers else None,
        single_process_stages={name: {key: summary([row[key] for row in records])
                               for key in ('wall_ms', 'cpu_ms')}
                               for name, records in transform_records.items()},
        dataset_build_seconds=build_seconds, child_initializations=initializations,
        loader_wait_ms=summary([row['loader_wait_ms'] for row in rows]),
        baseline_resources=baseline_resources, before_iteration=before_iteration,
        retained_resources=retained_resources,
        after_resources=guard(args.max_ram_gib), per_batch=rows,
        scope='CPU-only sampled input diagnostic; no model, optimizer, training or source edits; hashes are excluded from loader wait; <=96 batches intentionally retained until hashing, so their temporary FD growth is NOT a metric leak; after_resources follows release')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ['split','samples','workers','batch','dataset_build_seconds','loader_wait_ms']}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
