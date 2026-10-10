"""CPU regression tests in the team image; no full model or GPU required."""
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

import torch
from mmengine.config import Config
from mmengine.structures import InstanceData
from mmdet.models.task_modules import AssignResult
from mmdet3d.registry import MODELS
from mmdet3d.structures import LiDARInstance3DBoxes
from mmdet3d.utils import register_all_modules
from projects.BEVFusion.bevfusion.transfusion_head import TransFusionHead
from perception.detection3d.bevfusion.plugin.morai_rectangular_bev import (
    MoraiRectangularTransFusionHead)
from perception.detection3d.bevfusion.plugin.morai_transfusion_head import MoraiTransFusionHead

CONFIG = Path(__file__).resolve().parents[1] / 'configs/bevfusion_base5_dense_512_native_jpeg90.py'


def instances(rows, labels):
    return InstanceData(
        bboxes_3d=LiDARInstance3DBoxes(torch.tensor(rows, dtype=torch.float32).reshape(-1, 7)),
        labels_3d=torch.tensor(labels, dtype=torch.long))


def target_only_head():
    head = object.__new__(MoraiRectangularTransFusionHead)
    torch.nn.Module.__init__(head)
    head.train_cfg = Config.fromfile(str(CONFIG)).model.bbox_head.train_cfg
    head.num_classes = 4
    return head


class HeatmapTargetsTest(unittest.TestCase):
    def setUp(self):
        self.head = target_only_head()

    def test_quadrants_and_all_classes_have_expected_peak(self):
        for label in range(4):
            for x, y in [(0, 0), (40, 0), (-40, 0), (0, 10), (0, -10),
                         (90, 29), (-90, -29), (45, -20), (-45, 20)]:
                with self.subTest(label=label, x=x, y=y):
                    gt = instances([[x, y, -1, 4.5, 1.8, 1.5, .7]], [label])
                    heatmap = self.head.build_dense_heatmap_targets(gt)
                    self.assertEqual(tuple(heatmap.shape), (1, 4, 108, 336))
                    xi, yi = ((gt.bboxes_3d.tensor[0, :2] - torch.tensor([-100.8, -32.4])) / .6).int()
                    self.assertEqual(float(heatmap[0, label, yi, xi]), 1.)
                    self.assertEqual(int((heatmap[0, label] == 1).sum()), 1)
                    self.assertEqual(float(heatmap[0].sum() - heatmap[0, label].sum()), 0.)
                    # The flattened cell queried by the head is the same cell.
                    grid = self.head.create_2D_grid(336, 108)
                    torch.testing.assert_close(grid[0, yi * 336 + xi], torch.tensor([xi + .5, yi + .5]))

    def test_semantic_and_padded_grid_boundaries(self):
        for x, y in [(-100, -30), (100, 30), (-100.8, -32.4), (100.79, 32.39)]:
            gt = instances([[x, y, 0, .2, .2, .2, 0]], [2])
            heatmap = self.head.build_dense_heatmap_targets(gt)
            self.assertEqual(int((heatmap == 1).sum()), 1)
        for x, y in [(100.8, 0), (0, 32.4), (-100.81, 0), (0, -32.41)]:
            with self.subTest(x=x, y=y), self.assertRaisesRegex(ValueError, 'outside padded BEV'):
                self.head.build_dense_heatmap_targets(instances([[x, y, 0, 1, 1, 1, 0]], [0]))

    def test_empty_target(self):
        heatmap = self.head.build_dense_heatmap_targets(instances([], []))
        self.assertEqual(tuple(heatmap.shape), (1, 4, 108, 336))
        self.assertEqual(float(heatmap.sum()), 0.)

    def test_invalid_inputs_fail_instead_of_silently_repositioning(self):
        for rows, labels in [([[0, 0, 0, 1, 1, 1, 0]], [4]),
                             ([[0, 0, 0, 1, 1, 1, 0]], [-1]),
                             ([[0, 0, 0, 0, 1, 1, 0]], [0]),
                             ([[float('nan'), 0, 0, 1, 1, 1, 0]], [0])]:
            with self.assertRaises(ValueError):
                self.head.build_dense_heatmap_targets(instances(rows, labels))
        self.head.train_cfg = deepcopy(self.head.train_cfg)
        self.head.train_cfg.grid_size = [2687, 864, 41]
        with self.assertRaisesRegex(ValueError, 'divisible'):
            self.head.build_dense_heatmap_targets(instances([], []))

    def test_override_replaces_only_last_return_value(self):
        gt = instances([[40, 0, 0, 4, 2, 1, 0]], [0])
        old = (torch.tensor([i]) for i in range(7))
        old = (*old, torch.zeros(1, 4, 108, 336))
        with patch.object(MoraiTransFusionHead, 'get_targets_single', return_value=old):
            new = self.head.get_targets_single(gt, {}, 0)
        for before, after in zip(old[:-1], new[:-1]):
            self.assertIs(before, after)
        self.assertEqual(float(new[-1].max()), 1.)

    def test_legacy_e25_method_is_not_overridden(self):
        self.assertIs(MoraiTransFusionHead.get_targets_single, TransFusionHead.get_targets_single)

    def test_actual_parent_assignment_and_regression_are_preserved(self):
        register_all_modules(init_default_scope=True)
        cfg = Config.fromfile(str(CONFIG)).model.bbox_head
        head = MODELS.build(cfg)
        gt = instances([[40, 0, -1, 4.5, 1.8, 1.5, .5]], [0])

        class FixedAssigner:
            def assign(self, predictions, *args):
                count = len(predictions)
                indices = torch.zeros(count, dtype=torch.long)
                indices[0] = 1
                overlaps = torch.zeros(count)
                overlaps[0] = .9
                labels = torch.full((count,), -1, dtype=torch.long)
                labels[0] = 0
                return AssignResult(1, indices, overlaps, labels=labels)

        # Only IoU/Hungarian is stubbed to avoid CUDA-only IoU. Real parent
        # decoding, PseudoSampler, encoding and target assembly are exercised.
        head.bbox_assigner = FixedAssigner()
        count = head.num_proposals
        predictions = dict(heatmap=torch.full((1, 4, count), .1),
                           center=torch.zeros(1, 2, count), height=torch.zeros(1, 1, count),
                           dim=torch.zeros(1, 3, count), rot=torch.zeros(1, 2, count))
        predictions['rot'][:, 1] = 1
        old = TransFusionHead.get_targets_single(head, gt, predictions, 0)
        new = head.get_targets_single(gt, predictions, 0)
        for before, after in zip(old[:-1], new[:-1]):
            if torch.is_tensor(before):
                torch.testing.assert_close(before, after, rtol=0, atol=0)
            else:
                self.assertEqual(before, after)
        self.assertEqual(float(old[-1].max()), 0.)
        self.assertEqual(float(new[-1][0, 0, 54, 234]), 1.)
        encoded = new[2][0, 0]
        torch.testing.assert_close(encoded[:2], torch.tensor([140.8 / .6, 32.4 / .6]))
        decoded = head.bbox_coder.decode(predictions['heatmap'][:, :, :1],
            torch.tensor([[[torch.sin(torch.tensor(.5))], [torch.cos(torch.tensor(.5))]]]),
            encoded[3:6].reshape(1, 3, 1), encoded[:2].reshape(1, 2, 1),
            encoded[2:3].reshape(1, 1, 1), None)[0]['bboxes'][0]
        torch.testing.assert_close(decoded, gt.bboxes_3d.tensor[0], rtol=0, atol=1e-5)

    def test_loss_gradient_teaches_the_correct_cell_not_the_swapped_cell(self):
        register_all_modules(init_default_scope=True)
        head = MODELS.build(Config.fromfile(str(CONFIG)).model.bbox_head)
        gt = instances([[-40, 0, 0, .2, .2, .2, 0]], [3])
        target = head.build_dense_heatmap_targets(gt)
        logits = torch.zeros_like(target, requires_grad=True)
        loss = head.loss_heatmap(logits.sigmoid(), target, avg_factor=1)
        loss.backward()
        self.assertTrue(torch.isfinite(loss))
        self.assertTrue(torch.isfinite(logits.grad).all())
        self.assertLess(float(logits.grad[0, 3, 54, 101]), 0.)
        self.assertGreater(float(logits.grad[0, 3, 101, 54]), 0.)


if __name__ == '__main__':
    torch.set_num_threads(1)
    unittest.main()
