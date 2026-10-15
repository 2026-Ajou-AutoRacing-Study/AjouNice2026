#!/usr/bin/env python3
"""CPU-only peak-policy tests; load upstream method with no MMDetection import."""

import ast
import copy
import importlib.util
import os
from pathlib import Path
import sys
import unittest

import torch
from torch import nn
from torch.nn import functional as F

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
from perception.detection3d.bevfusion.proposal_peak import (
    explicit_class_peak_mask,
    forward_single_with_explicit_peaks,
    validate_peak_kernels,
)


def vendored_forward_single():
    path = ROOT / 'third_party/mmdetection3d/projects/BEVFusion/bevfusion/transfusion_head.py'
    parsed = ast.parse(path.read_text())
    cls = next(x for x in parsed.body if isinstance(x, ast.ClassDef) and x.name == 'TransFusionHead')
    method = next(x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name == 'forward_single')
    module = ast.Module(body=[method], type_ignores=[])
    namespace = dict(torch=torch, F=F)
    exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), namespace)
    return namespace['forward_single']


class Decoder(nn.Module):
    def forward(self, query, key, query_pos, key_pos):
        return query + key.mean(dim=-1, keepdim=True) * .1


class Prediction(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.center = nn.Conv1d(channels, 2, 1)
        self.heatmap = nn.Conv1d(channels, 3, 1)
        self.dim = nn.Conv1d(channels, 3, 1)

    def forward(self, query):
        return dict(center=self.center(query), heatmap=self.heatmap(query), dim=self.dim(query))


class Fixture(nn.Module):
    def __init__(self, auxiliary=True, layers=2):
        super().__init__()
        self.num_classes, self.num_proposals, self.nms_kernel_size = 3, 20, 3
        self.auxiliary, self.num_decoder_layers = auxiliary, layers
        self.test_cfg = dict(dataset='Waymo')
        self.shared_conv = nn.Conv2d(4, 8, 1)
        self.heatmap_head = nn.Conv2d(8, 3, 1)
        self.class_encoding = nn.Conv1d(3, 8, 1)
        self.decoder = nn.ModuleList([Decoder() for _ in range(layers)])
        self.prediction_heads = nn.ModuleList([Prediction(8) for _ in range(layers)])
        y, x = torch.meshgrid(torch.arange(9.), torch.arange(10.), indexing='ij')
        self.bev_pos = torch.stack((x+.5, y+.5), dim=-1).reshape(1, -1, 2)


class ProposalPeakTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)

    def test_validation(self):
        self.assertEqual(validate_peak_kernels([3, 1, 3], 3), (3, 1, 3))
        for bad in ([], [1, 3], [1, 3, 3, 1], [1, 2, 3], [1, True, 3], [1., 3, 3], None):
            with self.assertRaises(ValueError):
                validate_peak_kernels(bad, 3)

    def test_no_mutation_and_gradients(self):
        heatmap = torch.rand(2, 3, 9, 10, requires_grad=True)
        before = heatmap.detach().clone()
        result = explicit_class_peak_mask(heatmap, [3, 1, 3])
        self.assertTrue(torch.equal(heatmap.detach(), before))
        result.sum().backward()
        self.assertTrue(torch.isfinite(heatmap.grad).all())

    def test_kernel1_preserves_pedestrian(self):
        heatmap = torch.rand(2, 3, 9, 10)
        result = explicit_class_peak_mask(heatmap, [3, 1, 3])
        self.assertTrue(torch.equal(result[:, 1], heatmap[:, 1]))

    def test_neighbor_query_suppressed_not_box_nms(self):
        h = torch.zeros(1, 3, 9, 10)
        h[0, 2, 4, 4], h[0, 2, 4, 5] = .9, .8
        legacy = explicit_class_peak_mask(h, [3, 1, 1])
        trial = explicit_class_peak_mask(h, [3, 1, 3])
        self.assertEqual(torch.count_nonzero(legacy[:, 2]), 2)
        self.assertEqual(torch.count_nonzero(trial[:, 2]), 1)

    def test_two_close_distinct_objects_risk(self):
        h = torch.zeros(1, 3, 9, 10)
        h[0, 2, 4, 4], h[0, 2, 4, 6] = .9, .8
        self.assertEqual(torch.count_nonzero(explicit_class_peak_mask(h, [3, 1, 3])[:, 2]), 2)
        self.assertEqual(torch.count_nonzero(explicit_class_peak_mask(h, [3, 1, 5])[:, 2]), 1)

    def test_equal_peak_ties_retained(self):
        h = torch.zeros(1, 3, 9, 10)
        h[0, 2, 4, 4:6] = .9
        self.assertEqual(torch.count_nonzero(explicit_class_peak_mask(h, [3, 1, 3])[:, 2]), 2)

    def test_original_zero_border_and_too_small(self):
        h = torch.ones(1, 3, 9, 10)
        result = explicit_class_peak_mask(h, [3, 1, 5])
        self.assertEqual(float(result[0, 0, 0, 0]), 0.)
        self.assertEqual(float(result[0, 1, 0, 0]), 1.)
        self.assertEqual(float(result[0, 2, 1, 1]), 0.)
        with self.assertRaises(ValueError):
            explicit_class_peak_mask(torch.ones(1, 3, 3, 3), [3, 1, 5])

    def test_vendored_waymo_explicit_bitwise(self):
        original = vendored_forward_single()
        for auxiliary in (False, True):
            for layers in (1, 2):
                torch.manual_seed(21)
                source = Fixture(auxiliary=auxiliary, layers=layers).eval()
                candidate = copy.deepcopy(source)
                inputs = torch.randn(2, 4, 9, 10)
                expected = original(source, inputs, [{}, {}])
                actual = forward_single_with_explicit_peaks(candidate, inputs, [{}, {}], [3, 1, 1])
                self.assertEqual(expected[0].keys(), actual[0].keys())
                for key in expected[0]:
                    self.assertTrue(torch.equal(expected[0][key], actual[0][key]), key)
                self.assertTrue(torch.equal(source.query_labels, candidate.query_labels))
                self.assertEqual(source.state_dict().keys(), candidate.state_dict().keys())

    def test_default_subclass_delegates_without_helper(self):
        # Inspect the actual lightweight subclass without importing mmdet3d or
        # extensions: the method's default branch must call the inherited one.
        path = ROOT / 'perception/detection3d/bevfusion/plugin/morai_proposal_peak_head.py'
        tree = ast.parse(path.read_text())
        cls = next(x for x in tree.body if isinstance(x, ast.ClassDef))
        namespace = dict(MoraiRectangularTransFusionHead=type('Base', (), {
            'forward_single': lambda self, inputs, metas: ('legacy', inputs, metas)}),
            forward_single_with_explicit_peaks=lambda *_: (_ for _ in ()).throw(AssertionError('Called opt-in helper')),
            validate_peak_kernels=validate_peak_kernels)
        cls.decorator_list = []
        exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), 'exec'), namespace)
        head = object.__new__(namespace['MoraiProposalPeakTransFusionHead'])
        head.proposal_peak_kernels = None
        self.assertEqual(head.forward_single('input', 'meta'), ('legacy', 'input', 'meta'))

    @unittest.skipUnless(importlib.util.find_spec('mmengine'), 'Full MMDetection CPU environment is optional')
    def test_actual_registered_head_state_and_forward(self):
        from mmengine.config import Config
        from mmengine.registry import init_default_scope
        from mmdet3d.registry import MODELS
        from mmengine.utils import import_modules_from_strings
        config_dir = ROOT / 'perception/detection3d/bevfusion/configs'
        cfg = Config.fromfile(str(config_dir / 'bevfusion_base5_fine_512_depth25_proposal_peak3_trial.py'))
        init_default_scope('mmdet3d')
        import_modules_from_strings(**cfg.custom_imports)
        # CPU wiring test only: small grid, identical trainable parameter sizes.
        # This does not certify full-ROI model output/cost or sensor accuracy.
        config = copy.deepcopy(cfg.model.bbox_head)
        config.train_cfg.grid_size = [80, 72, 41]
        config.test_cfg.grid_size = [80, 72, 41]
        config.type = 'MoraiRectangularTransFusionHead'
        config.pop('proposal_peak_kernels')
        torch.manual_seed(42)
        legacy = MODELS.build(config).eval()
        config.type = 'MoraiProposalPeakTransFusionHead'
        default = MODELS.build(config).eval()
        config.proposal_peak_kernels = [3, 1, 1]
        explicit_legacy = MODELS.build(config).eval()
        default.load_state_dict(legacy.state_dict(), strict=True)
        explicit_legacy.load_state_dict(legacy.state_dict(), strict=True)
        self.assertEqual(legacy.state_dict().keys(), default.state_dict().keys())
        self.assertEqual(legacy.state_dict().keys(), explicit_legacy.state_dict().keys())
        with torch.inference_mode():
            inputs = torch.randn(1, 512, 9, 10)
            a = legacy.forward_single(inputs, [{}])
            b = default.forward_single(inputs, [{}])
            c = explicit_legacy.forward_single(inputs, [{}])
        for key in a[0]:
            self.assertTrue(torch.equal(a[0][key], b[0][key]), key)
            self.assertTrue(torch.equal(a[0][key], c[0][key]), key)
        self.assertTrue(torch.equal(legacy.query_labels, explicit_legacy.query_labels))

    @unittest.skipUnless(importlib.util.find_spec('mmengine'), 'Full MMDetection CPU environment is optional')
    def test_actual_registered_candidates_resolve(self):
        from mmengine.config import Config
        from mmdet3d.registry import MODELS
        from mmengine.utils import import_modules_from_strings
        directory = ROOT / 'perception/detection3d/bevfusion/configs'
        for kernel in (3, 5):
            cfg = Config.fromfile(str(directory /
                f'bevfusion_base5_fine_512_depth25_proposal_peak{kernel}_trial.py'))
            import_modules_from_strings(**cfg.custom_imports)
            self.assertEqual(cfg.model.bbox_head.proposal_peak_kernels, [3, 1, kernel])
            self.assertEqual(cfg.model.bbox_head.type, 'MoraiProposalPeakTransFusionHead')

    @unittest.skipUnless(importlib.util.find_spec('mmengine') and os.environ.get('BASE5_PEAK_E24_CHECKPOINT'),
                         'Explicit trusted E24 checkpoint and full MMDetection environment required')
    def test_actual_e24_head_weights_strictly_compatible(self):
        from mmengine.config import Config
        from mmengine.registry import init_default_scope
        from mmengine.utils import import_modules_from_strings
        from mmdet3d.registry import MODELS
        cfg = Config.fromfile(str(ROOT / 'perception/detection3d/bevfusion/configs/'
            'bevfusion_base5_fine_512_depth25_proposal_peak3_trial.py'))
        init_default_scope('mmdet3d')
        import_modules_from_strings(**cfg.custom_imports)
        checkpoint = torch.load(os.environ['BASE5_PEAK_E24_CHECKPOINT'], map_location='cpu', weights_only=False)
        state = checkpoint['state_dict']
        head_state = {k[len('bbox_head.'):]: v for k, v in state.items() if k.startswith('bbox_head.')}
        self.assertTrue(head_state, 'No exact bbox_head prefix in checkpoint')
        for kernels in (None, [3, 1, 1], [3, 1, 3], [3, 1, 5]):
            config = copy.deepcopy(cfg.model.bbox_head)
            config.proposal_peak_kernels = kernels
            head = MODELS.build(config).eval()
            head.load_state_dict(head_state, strict=True)
            self.assertEqual(head.state_dict().keys(), head_state.keys())


if __name__ == '__main__':
    unittest.main()
