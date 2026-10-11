#!/usr/bin/env python3
"""Audited evaluation-only extension of an immutable Base5 training source.

No model, optimizer, scheduler, dataset, training pipeline or run ID changes.
The frozen training contract is retained; this separately checksummed extension
is recorded in the work directory and logs only additional validation scalars.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np

FAR_BINS = ((50., 80.), (80., 100.))
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
VERSION = 'base5_distance_diagnostics_v2'


def range_map_metrics(metrics, prefix, classes=CLASSES):
    """Fixed-class macro AP; absent GT is unavailable, not a zero AP."""
    supported = sum(metrics[f'{prefix}/{name}_GT'] > 0 for name in classes)
    result = {f'{prefix}/mAP_available': float(supported == len(classes)),
              f'{prefix}/mAP_GT_classes': float(supported)}
    if supported == len(classes):
        values = [float(metrics[f'{prefix}/{name}_mean_AP']) for name in classes]
        # The canonical AP integrator may return 1 + floating-point epsilon.
        if not all(np.isfinite(value) and -1e-10 <= value <= 1 + 1e-10 for value in values):
            raise ValueError('Invalid class AP in ' + prefix)
        result[f'{prefix}/mAP'] = float(np.mean(values))
    return result


def far_map_backfill_rows(receipts):
    """Derive only missing aggregate AP from exact saved validation scalars."""
    rows = []
    epochs = set()
    for receipt in sorted(receipts, key=lambda r: r['epoch']):
        epoch = receipt['epoch']
        if epoch in epochs:
            raise ValueError('Duplicate validation epoch')
        epochs.add(epoch)
        row = {'epoch': epoch}
        for lower, upper in FAR_BINS:
            prefix = f'val/range/{lower:g}_{upper:g}m'
            row.update(range_map_metrics(receipt['metrics'], prefix))
        rows.append(row)
    return rows


def enable_resume_metadata_updates(cfg):
    # MMEngine sends the resolved config again on resume. load_from necessarily
    # changes from epoch_1 to epoch_N; the frozen contract already guards every
    # actual training setting. This permission is only for W&B metadata.
    for backends in (cfg.visualizer.vis_backends, cfg.vis_backends):
        for backend in backends:
            if backend['type'] == 'MoraiWandbVisBackend':
                backend['init_kwargs']['allow_val_change'] = True


def diagnostic_metrics(samples, classes=CLASSES):
    from perception.detection3d.common.evaluation.morai_detection import (
        DEFAULT_CENTER_THRESHOLDS, DEFAULT_DISTANCE_BINS, _evaluate_center, _filter_range)
    extra = {}
    for lower, upper in (*DEFAULT_DISTANCE_BINS, *FAR_BINS):
        ranged = _filter_range(samples, (lower, upper))
        prefix = f'range/{lower:g}_{upper:g}m'
        if (lower, upper) in FAR_BINS:
            ap = _evaluate_center(ranged, list(classes), DEFAULT_CENTER_THRESHOLDS,
                                  2., .1, .1, prefix, include_tp=False,
                                  include_operating_points=False)
            for i, name in enumerate(classes):
                count = sum(int(np.count_nonzero(s['gt_labels'] == i)) for s in ranged)
                if count:
                    extra[f'{prefix}/{name}_mean_AP'] = ap[f'{prefix}/{name}_mean_AP']
        for i, name in enumerate(classes):
            count = sum(int(np.count_nonzero(s['gt_labels'] == i)) for s in ranged)
            extra[f'{prefix}/{name}_GT'] = float(count)
            extra[f'{prefix}/{name}_available'] = float(count > 0)
            extra[f'{prefix}/{name}_small_sample'] = float(0 < count < 20)
        if (lower, upper) in FAR_BINS:
            extra.update(range_map_metrics(extra, prefix, classes))
    for direction in ('front', 'rear'):
        directional = []
        for s in samples:
            gmask = (s['gt_boxes'][:, 0] >= 0) if direction == 'front' else (s['gt_boxes'][:, 0] < 0)
            pmask = (s['pred_boxes'][:, 0] >= 0) if direction == 'front' else (s['pred_boxes'][:, 0] < 0)
            directional.append({k: v[gmask if k.startswith('gt_') else pmask] for k, v in s.items()})
        for lower, upper in FAR_BINS:
            ranged = _filter_range(directional, (lower, upper))
            prefix = f'range_direction/{lower:g}_{upper:g}m/{direction}'
            count = sum(int(np.count_nonzero(s['gt_labels'] == 0)) for s in ranged)
            extra[f'{prefix}/vehicle_GT'] = float(count)
            extra[f'{prefix}/vehicle_available'] = float(count > 0)
            if count:
                ap = _evaluate_center(ranged, list(classes), DEFAULT_CENTER_THRESHOLDS,
                                      2., .1, .1, prefix, include_tp=False,
                                      include_operating_points=False)
                extra[f'{prefix}/vehicle_mean_AP'] = ap[f'{prefix}/vehicle_mean_AP']
    corners = _filter_range(samples, (100., float('inf')))
    for i, name in enumerate(classes):
        extra[f'range_support/radial_ge100m/{name}_GT'] = float(sum(
            np.count_nonzero(s['gt_labels'] == i) for s in corners))
    return extra


def install():
    from perception.detection3d.common.evaluation.morai_detection import (
        _extract_samples, _filter_center_box_range)
    from perception.detection3d.bevfusion.plugin.morai_metric import MoraiBase5MergedCenterMetric
    from perception.detection3d.bevfusion.plugin.morai_wandb import MoraiWandbVisBackend
    from mmengine.hooks import Hook
    from mmengine.registry import HOOKS
    if getattr(MoraiBase5MergedCenterMetric, '_distance_extension', None) == VERSION:
        return
    original_compute = MoraiBase5MergedCenterMetric.compute_metrics

    def compute(self, results):
        original = original_compute(self, results)
        classes = list(self.dataset_meta['classes'])
        if classes != list(CLASSES):
            raise ValueError('Distance extension is qualified only for merged Base5 three-class evaluation')
        samples = _extract_samples([r['info'] for r in results],
                                   [r['prediction'] for r in results], classes, True)
        samples = _filter_center_box_range(samples, self.point_cloud_range)
        extra = diagnostic_metrics(samples, classes)
        for key in original.keys() & extra.keys():
            if original[key] != extra[key]:
                raise AssertionError('Diagnostic extension changed an existing scalar: ' + key)
        return {**original, **extra}

    MoraiBase5MergedCenterMetric.compute_metrics = compute
    MoraiBase5MergedCenterMetric._distance_extension = VERSION
    original_init = MoraiWandbVisBackend._init_env

    def init(self):
        original_init(self)
        from perception.detection3d.common.evaluation.morai_detection import DEFAULT_DISTANCE_BINS
        for lower, upper in (*DEFAULT_DISTANCE_BINS, *FAR_BINS):
            self._wandb.define_metric(f'val/range/{lower:g}_{upper:g}m/mAP',
                                     step_metric='epoch', hidden=False, summary='max', overwrite=True)
        for lower, upper in FAR_BINS:
            for name in CLASSES:
                self._wandb.define_metric(f'val/range/{lower:g}_{upper:g}m/{name}_mean_AP',
                                         step_metric='epoch', hidden=False, summary='max', overwrite=True)
        self._wandb.define_metric('val/range_direction/*', step_metric='epoch', hidden=True)
        self._wandb.define_metric('val/range_support/*', step_metric='epoch', hidden=True)

    MoraiWandbVisBackend._init_env = init

    @HOOKS.register_module()
    class Base5DistanceDiagnosticReceiptHook(Hook):
        priority = 'LOWEST'

        def before_save_checkpoint(self, runner, checkpoint):
            checkpoint.setdefault('meta', {})['base5_distance_diagnostics'] = dict(
                version=VERSION, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

        def after_val_epoch(self, runner, metrics=None):
            destination = Path(runner.work_dir) / 'distance_diagnostics'
            destination.mkdir(exist_ok=True)
            row = dict(version=VERSION, epoch=runner.epoch,
                       iter=runner.iter, time=time.time(), metrics=metrics)
            (destination / f'epoch_{runner.epoch}.json').write_text(json.dumps(row, indent=2)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frozen-entrypoint', type=Path, required=True)
    a, remaining = p.parse_known_args()
    install()
    # Preserve the original entrypoint's sibling-tool import resolution.
    sys.path.insert(0, str(a.frozen_entrypoint.resolve().parent))
    spec = importlib.util.spec_from_file_location('frozen_base5_direct_training', a.frozen_entrypoint)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_prepare = module.prepare

    def prepare(args):
        cfg, manifest = original_prepare(args)
        enable_resume_metadata_updates(cfg)
        cfg.custom_hooks.append(dict(type='Base5DistanceDiagnosticReceiptHook'))
        receipt = dict(version=VERSION, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                       training_contract_sha256=manifest['contract_sha256'],
                       frozen_entrypoint=str(a.frozen_entrypoint),
                       scope='validation-only; model/optimizer/schedule/data/run_id unchanged')
        (args.work_dir / 'distance_diagnostics_extension.json').write_text(json.dumps(receipt, indent=2)+'\n')
        cfg.dump(str(args.work_dir / 'resolved_distance_diagnostics.py'))
        return cfg, manifest

    module.prepare = prepare
    sys.argv = [str(a.frozen_entrypoint), *remaining]
    module.main()


if __name__ == '__main__':
    main()
