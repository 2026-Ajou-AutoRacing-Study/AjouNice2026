#!/usr/bin/env python3
"""Opt-in, CPU-only validation scalars for the complete Base5 cohort run.

The immutable metric and distance extension run first.  This module consumes
their same predictions without altering AP, best-checkpoint selection, the
dataset, model, optimizer, scheduler, or training batches.  A token-to-cohort
map in the resolved evaluator config, not token spelling, assigns environments.
"""
import hashlib
import json
from pathlib import Path
import time

import numpy as np

VERSION = 'base5_val_logging_v1'
CLASSES = ('vehicle', 'pedestrian', 'obstacle')
ENVIRONMENTS = ('sunny11', 'sunny13', 'sunny15', 'foggy11', 'foggy13', 'foggy15')
SUPPLEMENTS = ('supp_sunny11', 'supp_sunny13', 'supp_sunny15')
TEN_METER_BINS = tuple((float(lower), float(lower + 10)) for lower in range(0, 100, 10))
FIXED_THRESHOLDS = dict(vehicle=0.40, pedestrian=0.10, obstacle=0.10)
SCHEMA = dict(
    version=VERSION, ap_unit='fraction_0_to_1', x_axis='epoch',
    environments=list(ENVIRONMENTS), supplement='sunny_obstacle',
    ten_meter_bins=[list(bounds) for bounds in TEN_METER_BINS],
    fixed_score_thresholds=FIXED_THRESHOLDS,
    fixed_match='same class, XY distance <1m, score >= threshold, greedy one-to-one',
    center_AP='mean AP over XY distance thresholds 0.5, 1, 2, 4m',
    zero_GT='omit new class AP; publish support; preserve existing legacy scalars',
    source='existing validation predictions; no additional GPU inference',
)


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


SCHEMA_SHA256 = canonical_sha(SCHEMA)


def _class_ap(samples, prefix):
    """Reuse the canonical nuScenes-style XY AP integrator, not a new AP."""
    from perception.detection3d.common.evaluation.morai_detection import (
        DEFAULT_CENTER_THRESHOLDS, _evaluate_center)
    return _evaluate_center(samples, list(CLASSES), DEFAULT_CENTER_THRESHOLDS,
                            2., .1, .1, prefix, include_tp=False,
                            include_operating_points=False)


def _support_ap(samples, prefix, *, small_sample=False, availability=True,
                aggregate=True, classes=CLASSES, precomputed=None):
    counts = {name: sum(int(np.count_nonzero(sample['gt_labels'] == index))
                        for sample in samples)
              for index, name in enumerate(CLASSES)}
    result = {}
    supported = [name for name in classes if counts[name] > 0]
    if precomputed is not None:
        for name in CLASSES:
            if precomputed.get(f'{prefix}/{name}_GT') != float(counts[name]):
                raise AssertionError('Reused legacy range support differs: ' + prefix)
        ap = precomputed
    else:
        ap = _class_ap(samples, prefix) if supported else {}
    for name in CLASSES:
        result[f'{prefix}/{name}_GT'] = float(counts[name])
        if availability:
            result[f'{prefix}/{name}_available'] = float(counts[name] > 0)
        if small_sample:
            result[f'{prefix}/{name}_small_sample'] = float(0 < counts[name] < 20)
        if name in supported:
            result[f'{prefix}/{name}_mean_AP'] = ap[f'{prefix}/{name}_mean_AP']
    if aggregate:
        result[f'{prefix}/mAP_GT_classes'] = float(len(supported))
        result[f'{prefix}/mAP_available'] = float(len(supported) == len(classes))
        if len(supported) == len(classes):
            result[f'{prefix}/mAP'] = float(np.mean([
                result[f'{prefix}/{name}_mean_AP'] for name in classes]))
    return result


def ten_meter_metrics(samples, legacy_metrics=None):
    """Radial [lower, upper) XY bins; do not replace the broad legacy bins."""
    from perception.detection3d.common.evaluation.morai_detection import _filter_range
    result = {}
    for lower, upper in TEN_METER_BINS:
        prefix = f'range/{lower:g}_{upper:g}m'
        result.update(_support_ap(_filter_range(samples, (lower, upper)), prefix,
                                  small_sample=True,
                                  precomputed=legacy_metrics if lower < 30 else None))
    return result


def environment_metrics(samples, cohorts):
    """Six base environments only; combine all three supplement cohorts apart."""
    if len(samples) != len(cohorts):
        raise ValueError('One explicit cohort is required for every validation frame')
    if any(name not in (*ENVIRONMENTS, *SUPPLEMENTS) for name in cohorts):
        raise ValueError('Unknown validation cohort; token spelling is not a weather label')
    result = {}
    for environment in ENVIRONMENTS:
        selected = [sample for sample, name in zip(samples, cohorts) if name == environment]
        prefix = f'environment/{environment}'
        result[f'{prefix}/frames'] = float(len(selected))
        # The panel schema deliberately has just mAP, three class APs and
        # support counts.  Never manufacture a zero AP for an absent class.
        values = _support_ap(selected, prefix, availability=False)
        result.update({key: value for key, value in values.items()
                       if not key.endswith(('/mAP_available', '/mAP_GT_classes'))})
    selected = [sample for sample, name in zip(samples, cohorts) if name in SUPPLEMENTS]
    prefix = 'supplement/sunny_obstacle'
    result[f'{prefix}/frames'] = float(len(selected))
    # This subset is obstacle-only: do not promote a one-class mean to mAP.
    values = _support_ap(selected, prefix, aggregate=False, classes=('obstacle',))
    result.update(values)
    result[f'{prefix}/mAP_GT_classes'] = float(sum(
        values[f'{prefix}/{name}_GT'] > 0 for name in CLASSES))
    result[f'{prefix}/mAP_available'] = 0.0
    return result


def fixed_operating_metrics(samples):
    """Score-sorted same-class one-to-one XY matching over all val frames.

    P/R/F1 use raw integer counts (not interpolated best-F1 curves).  Precision
    with no predictions and recall with no GT are zero by convention; the full
    configured val has GT in all three classes.  FP/frame includes empty frames.
    """
    result = {}
    frames = len(samples)
    for index, name in enumerate(CLASSES):
        threshold = FIXED_THRESHOLDS[name]
        positives = true_positives = false_positives = 0
        for sample in samples:
            gt = sample['gt_boxes'][sample['gt_labels'] == index, :2]
            positives += len(gt)
            candidates = np.flatnonzero((sample['pred_labels'] == index)
                                        & (sample['pred_scores'] >= threshold))
            # Stable ties match the canonical AP accumulator's input order.
            candidates = candidates[np.argsort(-sample['pred_scores'][candidates], kind='stable')]
            taken = np.zeros(len(gt), dtype=bool)
            for candidate in candidates:
                distances = np.linalg.norm(gt - sample['pred_boxes'][candidate, :2], axis=1)
                distances[taken] = np.inf
                nearest = int(np.argmin(distances)) if len(distances) else None
                if nearest is not None and distances[nearest] < 1.0:
                    taken[nearest] = True
                    true_positives += 1
                else:
                    false_positives += 1
        predictions = true_positives + false_positives
        precision = true_positives / predictions if predictions else 0.0
        recall = true_positives / positives if positives else 0.0
        prefix = f'fixed/{name}'
        result.update({f'{prefix}/score_threshold': float(threshold),
                       f'{prefix}/precision': float(precision),
                       f'{prefix}/recall': float(recall),
                       f'{prefix}/F1': float(2 * precision * recall / (precision + recall)
                                           if precision + recall else 0.0),
                       f'{prefix}/fp_per_frame': float(false_positives / frames if frames else 0.0)})
    return result


def extension_metrics(samples, cohorts, legacy_metrics=None):
    """Return unprefixed scalars; MMEngine's evaluator supplies ``val/`` once."""
    return {**ten_meter_metrics(samples, legacy_metrics), **environment_metrics(samples, cohorts),
            **fixed_operating_metrics(samples)}


def merge_preserving_legacy(original, extra):
    """Shared populated-bin values must be bit-identical; old GT0 stays old."""
    for key in original.keys() & extra.keys():
        if original[key] != extra[key]:
            raise AssertionError('Validation extension changed an existing scalar: ' + key)
    return {**original, **extra}


def _configuration(value):
    if value is None:
        return None
    if set(value) != {'token_to_cohort', 'expected_cohort_frames'}:
        raise ValueError('Explicit token map and cohort counts are required')
    mapping = dict(value['token_to_cohort'])
    counts = dict(value['expected_cohort_frames'])
    if (not mapping or set(counts) != set((*ENVIRONMENTS, *SUPPLEMENTS))
            or any(not isinstance(token, str) or not token or name not in counts
                   for token, name in mapping.items())
            or any(type(count) is not int or count <= 0 for count in counts.values())
            or any(sum(name == cohort for name in mapping.values()) != count
                   for cohort, count in counts.items())):
        raise ValueError('Validation token/cohort counts do not agree')
    if (any(counts[name] != 750 for name in ENVIRONMENTS)
            or any(counts[name] != 38 for name in SUPPLEMENTS)):
        raise ValueError('Expected six base val sets of 750 and three supplements of 38')
    return dict(token_to_cohort=mapping, expected_cohort_frames=counts)


def _sample_token(sample, mapping):
    meta = sample.metainfo if hasattr(sample, 'metainfo') else sample.get('metainfo', sample)
    sample_idx = meta.get('sample_idx')
    lidar_path = meta.get('lidar_path')
    # Original portable paths are data/<token>/LIDAR_5/<native>.bin; the
    # immediate parent is a sensor name, not the observation token.
    path_matches = [part for part in Path(str(lidar_path)).parts if part in mapping] if lidar_path else []
    if lidar_path and len(path_matches) != 1:
        raise ValueError('LiDAR path must identify exactly one known validation token')
    path_token = path_matches[0] if path_matches else None
    if isinstance(sample_idx, str):
        if sample_idx not in mapping:
            raise ValueError('Validation sample_idx is absent from the explicit token map')
        if path_token and path_token != sample_idx:
            raise ValueError('Validation sample_idx and LiDAR token disagree')
        return sample_idx
    if path_token in mapping:
        return path_token
    raise ValueError('Validation frame is absent from the explicit cohort token map')


def install():
    """Install after the immutable distance diagnostics, before Runner creation.

    Set ``cfg.val_evaluator.base5_val_logging`` to the two-field configuration
    documented in ``_configuration``.  Metrics with no opt-in config are no-ops.
    """
    from perception.detection3d.bevfusion.plugin.morai_metric import MoraiBase5MergedCenterMetric
    from perception.detection3d.bevfusion.plugin.morai_wandb import MoraiWandbVisBackend
    from perception.detection3d.common.evaluation.morai_detection import (
        _extract_samples, _filter_center_box_range)
    from mmengine.hooks import Hook
    from mmengine.registry import HOOKS
    if getattr(MoraiBase5MergedCenterMetric, '_val_logging_extension', None) == VERSION:
        return
    if not getattr(MoraiBase5MergedCenterMetric, '_distance_extension', None):
        raise RuntimeError('Install the qualified distance diagnostics before the val extension')
    original_init = MoraiBase5MergedCenterMetric.__init__
    original_process = MoraiBase5MergedCenterMetric.process
    original_compute = MoraiBase5MergedCenterMetric.compute_metrics

    def metric_init(self, *args, base5_val_logging=None, **kwargs):
        self._base5_val_logging = _configuration(base5_val_logging)
        original_init(self, *args, **kwargs)

    def process(self, data_batch, data_samples):
        config = getattr(self, '_base5_val_logging', None)
        if config is None:
            return original_process(self, data_batch, data_samples)
        for sample in data_samples:
            token = _sample_token(sample, config['token_to_cohort'])
            original_process(self, data_batch, [sample])
            self.results[-1]['_base5_val_token'] = token
            self.results[-1]['_base5_val_cohort'] = config['token_to_cohort'][token]

    def compute(self, results):
        original = original_compute(self, results)
        config = getattr(self, '_base5_val_logging', None)
        if config is None:
            return original
        if tuple(self.dataset_meta['classes']) != CLASSES:
            raise ValueError('Validation extension requires merged Base5 three-class evaluation')
        tokens = [row['_base5_val_token'] for row in results]
        if len(tokens) != len(set(tokens)) or set(tokens) != set(config['token_to_cohort']):
            raise ValueError('Validation result tokens are duplicated, missing, or unexpected')
        cohorts = [config['token_to_cohort'][token] for token in tokens]
        if any(row['_base5_val_cohort'] != cohort for row, cohort in zip(results, cohorts)):
            raise ValueError('Validation result cohort provenance differs')
        samples = _extract_samples([row['info'] for row in results],
                                   [row['prediction'] for row in results], list(CLASSES), True)
        samples = _filter_center_box_range(samples, self.point_cloud_range)
        return merge_preserving_legacy(original, extension_metrics(
            samples, cohorts, legacy_metrics=original))

    MoraiBase5MergedCenterMetric.__init__ = metric_init
    MoraiBase5MergedCenterMetric.process = process
    MoraiBase5MergedCenterMetric.compute_metrics = compute
    MoraiBase5MergedCenterMetric._val_logging_extension = VERSION
    original_wandb_init = MoraiWandbVisBackend._init_env

    def wandb_init(self):
        original_wandb_init(self)
        for namespace in ('range', 'environment', 'supplement', 'fixed'):
            self._wandb.define_metric(f'val/{namespace}/*', step_metric='epoch', hidden=True)
        visible = []
        for lower, upper in TEN_METER_BINS:
            visible.extend(f'val/range/{lower:g}_{upper:g}m/{suffix}'
                           for suffix in ('mAP', *(f'{name}_mean_AP' for name in CLASSES)))
        for environment in ENVIRONMENTS:
            visible.extend(f'val/environment/{environment}/{suffix}'
                           for suffix in ('mAP', *(f'{name}_mean_AP' for name in CLASSES)))
        visible.append('val/supplement/sunny_obstacle/obstacle_mean_AP')
        for name in CLASSES:
            visible.extend(f'val/fixed/{name}/{suffix}' for suffix in ('precision', 'recall', 'F1'))
            self._wandb.define_metric(f'val/fixed/{name}/fp_per_frame', step_metric='epoch',
                                     hidden=False, summary='min', overwrite=True)
        for key in visible:
            self._wandb.define_metric(key, step_metric='epoch', hidden=False,
                                     summary='max', overwrite=True)

    MoraiWandbVisBackend._init_env = wandb_init

    @HOOKS.register_module()
    class Base5ValLoggingReceiptHook(Hook):
        priority = 'LOWEST'

        def before_save_checkpoint(self, runner, checkpoint):
            checkpoint.setdefault('meta', {})['base5_val_logging'] = dict(
                version=VERSION, schema_sha256=SCHEMA_SHA256,
                script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

        def after_val_epoch(self, runner, metrics=None):
            destination = Path(runner.work_dir) / 'val_logging'
            destination.mkdir(exist_ok=True)
            row = dict(version=VERSION, schema_sha256=SCHEMA_SHA256, epoch=runner.epoch,
                       iter=runner.iter, time=time.time(), metrics=metrics)
            (destination / f'epoch_{runner.epoch}.json').write_text(
                json.dumps(row, indent=2, allow_nan=False) + '\n')
