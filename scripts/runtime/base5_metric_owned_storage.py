"""Release DataLoader shared-memory ownership after retaining val results.

The frozen MORAI metric's ``tensor.cpu().numpy()`` may be zero-copy when GT
already lives on CPU.  Keeping that NumPy view for a complete validation epoch
also keeps each worker shared-memory storage/file descriptor alive.  This
opt-in wrapper owns the small retained result arrays instead, without changing
any value, sample order, metric computation, or public logging key.
"""
import copy
import hashlib
from pathlib import Path

import numpy as np


VERSION = 'base5_metric_owned_storage_v1'


def _own_array(value):
    if isinstance(value, np.ndarray):
        return value.copy(order='K')
    return copy.deepcopy(value)


def own_result_storage(row):
    """Detach a newly retained metric row, not its input batch/DataSample."""
    info = row['info']
    for key in ('gt_boxes', 'gt_names', 'num_lidar_pts', 'valid_flag'):
        if key in info:
            info[key] = _own_array(info[key])
    prediction = row['prediction']
    # Frozen process already moved these to CPU; clone prevents a shared
    # storage alias even when a CPU-only test/producer supplies predictions.
    boxes = prediction['boxes_3d']
    boxes.tensor = boxes.tensor.detach().clone()
    for key in ('scores_3d', 'labels_3d'):
        prediction[key] = prediction[key].detach().clone()
    for key in ('original_semantic_class', 'assets'):
        if key in row:
            row[key] = _own_array(row[key])
    return row


def install():
    """Install after ordinary metric imports; affect merged Base5 only."""
    from perception.detection3d.bevfusion.plugin.morai_metric import MoraiBase5MergedCenterMetric
    from mmengine.hooks import Hook
    from mmdet3d.registry import HOOKS
    if getattr(MoraiBase5MergedCenterMetric, '_base5_owned_storage', None) == VERSION:
        return
    original_process = MoraiBase5MergedCenterMetric.process

    def process(self, data_batch, data_samples):
        start = len(self.results)
        original_process(self, data_batch, data_samples)
        for row in self.results[start:]:
            own_result_storage(row)

    MoraiBase5MergedCenterMetric.process = process
    MoraiBase5MergedCenterMetric._base5_owned_storage = VERSION

    @HOOKS.register_module()
    class Base5MetricOwnedStorageReceiptHook(Hook):
        priority = 'LOWEST'

        def before_save_checkpoint(self, runner, checkpoint):
            checkpoint.setdefault('meta', {})['base5_metric_owned_storage'] = dict(
                version=VERSION,
                script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
