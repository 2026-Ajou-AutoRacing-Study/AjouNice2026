import gc
import importlib.util
from pathlib import Path
import unittest

import numpy as np
import torch

from scripts.runtime.base5_metric_owned_storage import own_result_storage, install


class Boxes:
    def __init__(self, tensor):
        self.tensor = tensor


def make_row(shared=True):
    boxes = torch.tensor([[1., 2., 3., 4., 5., 6., 0.25]], dtype=torch.float32)
    labels = torch.tensor([2], dtype=torch.int64)
    scores = torch.tensor([.375], dtype=torch.float32)
    if shared:
        boxes.share_memory_()
        labels.share_memory_()
        scores.share_memory_()
    return dict(info=dict(gt_boxes=boxes.numpy(), gt_names=np.array(['obstacle']),
                         num_lidar_pts=np.array([8]), valid_flag=np.array([True])),
                prediction=dict(boxes_3d=Boxes(boxes), scores_3d=scores, labels_3d=labels),
                original_semantic_class=np.array(['obstacle']), assets=np.array(['WoodBox']))


def fd_count():
    return len(list(Path('/proc/self/fd').iterdir()))


class OwnedStorageTests(unittest.TestCase):
    def test_all_retained_values_are_identical_without_shared_ownership(self):
        row = make_row()
        info_before = {key: value.copy() for key, value in row['info'].items()}
        source_boxes = row['prediction']['boxes_3d'].tensor
        source_scores = row['prediction']['scores_3d']
        source_labels = row['prediction']['labels_3d']
        self.assertTrue(source_boxes.is_shared())
        result = own_result_storage(row)
        self.assertIs(result, row)
        for key, value in info_before.items():
            np.testing.assert_array_equal(result['info'][key], value)
        self.assertIsNone(result['info']['gt_boxes'].base)
        for key, previous in (('scores_3d', source_scores), ('labels_3d', source_labels)):
            current = result['prediction'][key]
            self.assertTrue(torch.equal(current, previous))
            self.assertFalse(current.is_shared())
            self.assertNotEqual(current.data_ptr(), previous.data_ptr())
        current_boxes = result['prediction']['boxes_3d'].tensor
        self.assertTrue(torch.equal(current_boxes, source_boxes))
        self.assertFalse(current_boxes.is_shared())
        self.assertFalse(np.shares_memory(result['info']['gt_boxes'], source_boxes.numpy()))

    @unittest.skipUnless(Path('/proc/self/fd').is_dir(), 'Linux fd accounting required')
    def test_unfixed_zero_copy_accumulates_descriptors_but_5100_fixed_rows_do_not(self):
        gc.collect()
        baseline = fd_count()
        # A bounded counterexample proves the exact retention mechanism without
        # deliberately exhausting the process fd limit.
        unfixed = [make_row() for _ in range(24)]
        leaked = fd_count() - baseline
        self.assertGreaterEqual(leaked, 24 * 3)
        del unfixed
        gc.collect()
        recovered = fd_count()
        self.assertLessEqual(recovered, baseline + 2)
        retained = []
        for _ in range(5100):
            retained.append(own_result_storage(make_row()))
        gc.collect()
        final = fd_count()
        self.assertEqual(len(retained), 5100)
        self.assertLessEqual(final, recovered + 3,
                             f'File descriptors grew from {recovered} to {final}')
        self.assertTrue(all(row['info']['gt_boxes'].base is None for row in retained))

    def test_owning_results_does_not_mutate_original_input_tensors(self):
        source = make_row()
        boxes = source['prediction']['boxes_3d'].tensor
        old = boxes.clone()
        fixed = own_result_storage(source)
        fixed['info']['gt_boxes'][0, 0] = 999
        fixed['prediction']['boxes_3d'].tensor[0, 1] = 888
        self.assertTrue(torch.equal(boxes, old))

    @unittest.skipUnless(Path('/proc/self/fd').is_dir(), 'Linux fd accounting required')
    def test_actual_two_worker_dataloader_5100_ipc_batches_keep_descriptors_bounded(self):
        from torch.utils.data import DataLoader, TensorDataset
        gc.collect()
        baseline = fd_count()
        data = TensorDataset(torch.ones((5100, 1, 7)), torch.full((5100, 1), 2),
                             torch.full((5100, 1), .375))
        loader = DataLoader(data, batch_size=1, num_workers=2, prefetch_factor=2)
        retained = []
        maximum = baseline
        for boxes, labels, scores in loader:
            self.assertTrue(boxes.is_shared())
            row = dict(info=dict(gt_boxes=boxes[0].numpy(), gt_names=np.array(['obstacle']),
                                 num_lidar_pts=np.array([8]), valid_flag=np.array([True])),
                       prediction=dict(boxes_3d=Boxes(boxes[0]), scores_3d=scores[0],
                                       labels_3d=labels[0]))
            retained.append(own_result_storage(row))
            if len(retained) % 100 == 0:
                maximum = max(maximum, fd_count())
        del loader, boxes, labels, scores, row
        gc.collect()
        self.assertEqual(len(retained), 5100)
        self.assertLessEqual(maximum, baseline + 40,
                             f'FD count accumulated across batches: {baseline} -> {maximum}')
        self.assertLessEqual(fd_count(), baseline + 5)


@unittest.skipUnless(importlib.util.find_spec('mmengine') is not None,
                     'The real metric runs in the prepared training environment')
class RuntimeIntegrationTests(unittest.TestCase):
    def test_real_process_keeps_5100_shared_memory_frames_without_descriptor_growth(self):
        from perception.detection3d.bevfusion.plugin.morai_metric import MoraiBase5MergedCenterMetric
        install()
        metric = MoraiBase5MergedCenterMetric(point_cloud_range=[-100, -30, -3, 100, 30, 2])
        metric.dataset_meta = dict(classes=('vehicle', 'pedestrian', 'obstacle'))
        gc.collect()
        baseline = fd_count()
        for _ in range(5100):
            row = make_row()
            prediction = row['prediction']
            sample = dict(eval_ann_info=dict(
                gt_bboxes_3d=prediction['boxes_3d'],
                gt_labels_3d=prediction['labels_3d'],
                original_semantic_class=row['original_semantic_class'],
                original_asset=row['assets']), pred_instances_3d=dict(
                bboxes_3d=prediction['boxes_3d'], scores_3d=prediction['scores_3d'],
                labels_3d=prediction['labels_3d']))
            metric.process({}, [sample])
        del sample, prediction, row
        gc.collect()
        self.assertEqual(len(metric.results), 5100)
        self.assertLessEqual(fd_count(), baseline + 3)
        self.assertTrue(all(result['info']['gt_boxes'].base is None for result in metric.results))
        np.testing.assert_array_equal(metric.results[-1]['info']['gt_boxes'],
                                      [[1., 2., 3., 4., 5., 6., .25]])


if __name__ == '__main__':
    unittest.main()
