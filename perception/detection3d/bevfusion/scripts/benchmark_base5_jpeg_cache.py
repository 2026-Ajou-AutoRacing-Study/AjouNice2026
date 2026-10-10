#!/usr/bin/env python3
"""Bounded CPU comparison: PNG + native Q90 vs one-time Q90 file cache.

No model, GPU, remote connection, original edits, or training accuracy claim.
Use actual MMDetection3D file loading, full train pipeline, and batch packing.
Cache generation and paired pixel/GT/geometry checks precede timing. OS cache
is deliberately warm; do not flush global host caches or call this disk speed.
"""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import gc
import hashlib
import json
from pathlib import Path
import pickle
import time

import cv2
from mmengine.config import Config
from mmengine.dataset import Compose, pseudo_collate
from mmengine.utils import import_modules_from_strings
from mmdet3d.registry import DATASETS
from mmdet3d.utils import register_all_modules
import numpy as np
from PIL import Image
import torch
from torch.utils.data import DataLoader, Dataset


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def stats(values):
    return dict(mean=float(np.mean(values)), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)))


def choose_indices(dataset, ledger, count):
    pools = defaultdict(list)
    for index in range(len(dataset)):
        token = dataset.get_data_info(index)['token']
        pools[ledger[token]['family']].append(index)
    for values in pools.values():
        values.sort(key=lambda i: sha_bytes(dataset.get_data_info(i)['token'].encode()))
    selected = []
    while len(selected) < min(count, len(dataset)):
        for family in sorted(pools):
            if pools[family] and len(selected) < count:
                selected.append(pools[family].pop(0))
        if not any(pools.values()):
            break
    return selected


def build_cache(data, records, output):
    if output.exists():
        raise FileExistsError('Cache must use a fresh directory')
    source_manifest_bytes = (data / 'manifest.json').read_bytes()
    source_manifest = json.loads(source_manifest_bytes)
    output.mkdir(parents=True)
    files = {}
    elapsed = []
    total_start = time.perf_counter()
    for record in records:
        for camera in record['images'].values():
            source = Path(camera['img_path']).resolve(strict=True)
            relative = str(source.relative_to(data))
            if relative in files:
                continue
            start = time.perf_counter()
            raw = source.read_bytes()
            if sha_bytes(raw) != source_manifest['files'][relative]['sha256']:
                raise ValueError('Source PNG hash differs from portable manifest')
            # Exactly the upstream PNG loader's RGB pixel source, then explicit
            # RGB -> BGR for OpenCV encode. No resize and no padded area.
            with Image.open(source) as image:
                if image.format != 'PNG':
                    raise ValueError('Only original lossless PNG input is allowed')
                rgb = np.array(image.convert('RGB'))
            if rgb.shape[:2] != (camera['height'], camera['width']):
                raise ValueError('Native camera dimensions changed')
            ok, blob = cv2.imencode('.jpg', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                raise RuntimeError('JPEG cache encoding failed')
            destination = output / 'images' / (sha_bytes(relative.encode()) + '.jpg')
            destination.parent.mkdir(exist_ok=True)
            encoded = blob.tobytes()
            destination.write_bytes(encoded)
            elapsed.append((time.perf_counter() - start) * 1000)
            files[relative] = dict(path=str(destination.relative_to(output)),
                bytes=len(encoded), sha256=sha_bytes(encoded), source_bytes=len(raw),
                source_sha256=sha_bytes(raw), height=int(rgb.shape[0]), width=int(rgb.shape[1]))
            if len(files) % 1000 == 0:
                print('Encoded %d native Q90 images' % len(files), flush=True)
    report = dict(version=1, complete=True, quality=90, native_resolution=True,
        source_manifest_sha256=sha_bytes(source_manifest_bytes),
        cv2_version=cv2.__version__, source_color_order='rgb', files=files,
        generation_seconds=time.perf_counter() - total_start,
        per_image_generation_ms=stats(elapsed), image_count=len(files),
        source_image_bytes=sum(r['source_bytes'] for r in files.values()),
        cache_image_bytes=sum(r['bytes'] for r in files.values()))
    (output / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


class ProbeDataset(Dataset):
    def __init__(self, records, pipeline, data, cache=None, freeze_geometry=False):
        self.records = records
        self.data = data
        self.cache = cache
        self.cache_manifest = json.loads((cache / 'manifest.json').read_text()) if cache else None
        transforms = deepcopy(pipeline)
        if cache:
            # Cached files have exactly one JPEG encoding. Never re-encode.
            transforms = [t for t in transforms if t['type'] != 'MoraiOnlineImageDomainAug3D']
        if freeze_geometry:
            transforms = [t for t in transforms if t['type'] != 'PointShuffle']
            for transform in transforms:
                if transform['type'] == 'BEVFusionGlobalRotScaleTrans':
                    transform.update(scale_ratio_range=[1., 1.], rot_range=[0., 0.], translation_std=0.)
        self.pipeline = Compose(transforms)

    def __len__(self):
        return len(self.records)

    def __getitem__(self, index):
        # Deterministic within each timed path. Removing JPEG RNG draws shifts
        # later geometry draws; paired checks separately freeze geometry.
        np.random.seed(261010 + index)
        data = deepcopy(self.records[index])
        if self.cache:
            for camera in data['images'].values():
                relative = str(Path(camera['img_path']).resolve().relative_to(self.data))
                camera['img_path'] = str(self.cache / self.cache_manifest['files'][relative]['path'])
        return self.pipeline(data)


def paired_checks(online, cached):
    labels = Counter()
    for index in range(len(online)):
        left, right = online[index], cached[index]
        torch.testing.assert_close(left['inputs']['img'], right['inputs']['img'], rtol=0, atol=0)
        torch.testing.assert_close(left['inputs']['points'], right['inputs']['points'], rtol=0, atol=0)
        a, b = left['data_samples'], right['data_samples']
        torch.testing.assert_close(a.gt_instances_3d.bboxes_3d.tensor,
                                   b.gt_instances_3d.bboxes_3d.tensor, rtol=0, atol=0)
        torch.testing.assert_close(a.gt_instances_3d.labels_3d,
                                   b.gt_instances_3d.labels_3d, rtol=0, atol=0)
        for key in ('img_aug_matrix', 'lidar_aug_matrix', 'lidar2img', 'cam2img', 'lidar2cam'):
            np.testing.assert_array_equal(a.metainfo[key], b.metainfo[key])
        labels.update(map(int, a.gt_instances_3d.labels_3d.tolist()))
    return dict(valid=True, scenes=len(online), camera_images=len(online) * 4,
                exact_pixels_points_gt_and_calibration=True, labels=dict(labels),
                scope='Paired comparison freezes BEV geometry and point shuffle only')


def time_loader(dataset, workers, batch, gpu_step_ms):
    loader = DataLoader(dataset, batch_size=batch, num_workers=workers,
        persistent_workers=workers > 0, collate_fn=pseudo_collate,
        **(dict(prefetch_factor=2) if workers else {}))
    results = []
    # Pass 0 is worker launch/cache warmup. Persistent workers live across the
    # two measured epochs. There is no inference or actual GPU execution.
    for repeat in range(3):
        begin = time.perf_counter()
        iterator = iter(loader)
        waits = []
        examples = 0
        for _ in range(len(loader)):
            start = time.perf_counter()
            item = next(iterator)
            waits.append((time.perf_counter() - start) * 1000)
            examples += len(item['data_samples'])
            if gpu_step_ms:
                # Artificial consumer delay allows CPU prefetch to overlap.
                # It is NOT a GPU benchmark or an updated train-time estimate.
                time.sleep(gpu_step_ms / 1000)
        elapsed = time.perf_counter() - begin
        if examples != len(dataset):
            raise ValueError('Loader dropped examples')
        results.append(dict(repeat=repeat, warmup=repeat == 0, examples=examples,
                            seconds=elapsed, scenes_per_second=examples / elapsed,
                            wait_ms=stats(waits), steady_wait_ms=stats(waits[2:]),
                            per_batch_wait_ms=waits))
    del iterator, loader
    gc.collect()
    measured = results[1:]
    return dict(workers=workers, batch=batch, consumer_delay_ms=gpu_step_ms,
        scenes_per_second=float(np.mean([r['scenes_per_second'] for r in measured])),
        seconds_mean=float(np.mean([r['seconds'] for r in measured])),
        loader_wait_ms_mean=float(np.mean([r['wait_ms']['mean'] for r in measured])),
        steady_loader_wait_ms_mean=float(np.mean([r['steady_wait_ms']['mean'] for r in measured])),
        repeats=results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes', type=int, default=256)
    parser.add_argument('--paced-scenes', type=int, default=64)
    parser.add_argument('--consumer-ms', type=float, default=609.57)
    parser.add_argument('--cache', type=Path, help='Reuse an existing verified pilot cache without regenerating')
    parser.add_argument('--skip-unpaced', action='store_true', help='Measure paced prefetch only')
    args = parser.parse_args()
    if args.output.exists() or not 32 <= args.scenes <= 512:
        raise ValueError('Fresh output and bounded 32..512 scenes required')
    if not 16 <= args.paced_scenes <= args.scenes or not 0 <= args.consumer_ms <= 1000:
        raise ValueError('Invalid paced budget')
    args.data = args.data.resolve(strict=True)
    args.output.mkdir(parents=True)
    torch.set_num_threads(1)
    cv2.setNumThreads(1)
    register_all_modules(init_default_scope=True)
    cfg = Config.fromfile(str(Path(__file__).resolve().parents[1] /
        'configs/bevfusion_base5_dense_512_jpeg90_train.py'))
    import_modules_from_strings(**cfg.custom_imports)
    dataset_cfg = deepcopy(cfg.train_dataloader.dataset)
    dataset_cfg.update(data_root=str(args.data), data_prefix=dict(pts='', img=''),
        ann_file=str(args.data / 'infos/512x1056/morai_base5_infos_train.pkl'), pipeline=[])
    dataset = DATASETS.build(dataset_cfg)
    ledger = {r['id']: r for r in map(json.loads,
        (args.data / 'provenance/original_selected_ledger.jsonl').read_text().splitlines())}
    indices = choose_indices(dataset, ledger, args.scenes)
    records = [dataset.get_data_info(i) for i in indices]
    report = dict(valid=False, scope='Local CPU/warm-cache actual loader; no training or AP comparison',
        scenes=len(records), families=dict(Counter(ledger[r['token']]['family'] for r in records)),
        tokens=[r['token'] for r in records], cpu_threads=1, cv2_threads=1,
        config_sha256=sha_bytes(Path(cfg.filename).read_bytes()), trials=[])
    cache = args.cache.resolve(strict=True) if args.cache else args.output / 'cache'
    if args.cache:
        from perception.detection3d.bevfusion.plugin.morai_jpeg_cache import validate_jpeg_cache
        required = [c['img_path'] for r in records for c in r['images'].values()]
        report['cache'] = validate_jpeg_cache(args.data, cache, required, verify_hashes=True)
        report['reused_cache'] = str(cache)
    else:
        report['cache'] = build_cache(args.data, records, cache)
    online = ProbeDataset(records, cfg.train_pipeline, args.data, freeze_geometry=True)
    cached = ProbeDataset(records, cfg.train_pipeline, args.data, cache, freeze_geometry=True)
    report['paired'] = paired_checks(online, cached)
    del online, cached
    population = Counter(ledger[dataset.get_data_info(i)['token']]['family'] for i in range(len(dataset)))
    sampled_bytes = defaultdict(list)
    for record in records:
        family = ledger[record['token']]['family']
        sampled_bytes[family].append(sum(report['cache']['files'][
            str(Path(c['img_path']).resolve().relative_to(args.data))]['bytes']
            for c in record['images'].values()))
    expected_bytes = sum(population[f] * np.mean(sampled_bytes[f]) for f in population)
    report['estimated_full_train'] = dict(scenes=len(dataset), camera_images=len(dataset) * 4,
        cache_image_bytes=float(expected_bytes), cache_gib=float(expected_bytes / 2**30),
        generation_seconds=report['cache']['generation_seconds'] * len(dataset) / len(records),
        population_families=dict(population),
        scope='Family-weighted cache image bytes; generation time is unweighted local extrapolation. Not measured full-cache build or other-weather size.')
    def save():
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    save()
    # Reverse order for the second round to limit systematic cache/order bias.
    for workers in (() if args.skip_unpaced else (0, 4)):
        for round_id, modes in enumerate((('online', 'cached'), ('cached', 'online'))):
            for mode in modes:
                probe = ProbeDataset(records, cfg.train_pipeline, args.data,
                                     cache if mode == 'cached' else None)
                row = time_loader(probe, workers, 4, 0)
                row.update(mode=mode, round=round_id, paced=False)
                report['trials'].append(row)
                save()
                print(json.dumps({k:v for k,v in row.items() if k != 'repeats'}), flush=True)
    for round_id, modes in enumerate((('online', 'cached'), ('cached', 'online'))):
        for mode in modes:
            probe = ProbeDataset(records[:args.paced_scenes], cfg.train_pipeline, args.data,
                                 cache if mode == 'cached' else None)
            row = time_loader(probe, 4, 4, args.consumer_ms)
            row.update(mode=mode, round=round_id, paced=True)
            report['trials'].append(row)
            save()
            print(json.dumps({k:v for k,v in row.items() if k != 'repeats'}), flush=True)
    report['valid'] = True
    report['limitations'] = ['PNG and JPEG sources are OS-cache warm; not network/storage cold performance',
        'Paced consumer uses artificial delay, not real CUDA or measured end-to-end training',
        'Local CPU/disk differ from Vast; no remote cost or accuracy guarantee',
        'Geometry RNG sequence changes with JPEG removal; paired correctness freezes geometry',
        'Cache is sampled pilot only; cannot use for complete training']
    save()
    print('PASS: ' + str(args.output / 'report.json'), flush=True)


if __name__ == '__main__':
    main()
