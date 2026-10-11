"""Initializer conversion contracts; no main training or dataset changes."""
from collections import OrderedDict
import unittest

import torch
from torch.nn import functional as F

from prepare_base5_full_initializer import adapt_position, build_delta, compose


class FullInitializerTest(unittest.TestCase):
    def fixture(self):
        torch.manual_seed(123)
        source = OrderedDict({
            'bbox_head.heatmap_head.1.bias': torch.randn(3),
            'fusion_layer.0.weight': torch.randn(3, 2, 3, 3),
            'view_transform.depthnet.6.weight': torch.randn(198, 4, 1, 1),
            'view_transform.depthnet.6.bias': torch.randn(198),
            'bbox_head.decoder.0.self_posembed.position_embedding_head.0.weight': torch.randn(4, 2, 1),
            'bbox_head.decoder.0.self_posembed.position_embedding_head.0.bias': torch.randn(4),
            'bbox_head.prediction_heads.0.center.1.weight': torch.randn(2, 4, 1),
            'bbox_head.prediction_heads.0.center.1.bias': torch.randn(2),
            'view_transform.dx': torch.tensor([.3, .3, 20.]),
        })
        source._metadata = {'fusion_layer.1': {'version': 2}}
        target = OrderedDict((key, value.clone()) for key, value in source.items())
        target['view_transform.depthnet.6.weight'] = torch.randn(476, 4, 1, 1)
        target['view_transform.depthnet.6.bias'] = torch.randn(476)
        return dict(state_dict=source), target

    def test_conv_xy_equivalence_on_rectangle(self):
        source, target = self.fixture()
        delta = build_delta(source, target)
        x = torch.randn(1, 2, 10, 14)
        old = F.conv2d(x, source['state_dict']['fusion_layer.0.weight'], padding=1)
        new = F.conv2d(x.transpose(-1, -2), delta['state_dict']['fusion_layer.0.weight'], padding=1)
        torch.testing.assert_close(new, old.transpose(-1, -2), atol=1e-5, rtol=1e-5)

    def test_position_affine_keeps_physical_encoding(self):
        weight, bias = torch.randn(7, 2, 1), torch.randn(7)
        converted, shifted = adapt_position(weight, bias, [-54., -54.], [.6, .6],
                                             [-100.8, -33.6], [.4, .4])
        xy = torch.tensor([[20., 10.], [-70., -20.], [0., 0.]])
        old = (xy + 54.) / .6
        new = (xy - torch.tensor([-100.8, -33.6])) / .4
        torch.testing.assert_close(F.linear(old, weight[..., 0], bias),
                                   F.linear(new, converted[..., 0], shifted), atol=1e-4, rtol=1e-5)

    def test_depth_logits_reset_context_copy_center_scale_geometry_reset(self):
        source, target = self.fixture()
        delta = build_delta(source, target)
        for suffix in ('weight', 'bias'):
            key = 'view_transform.depthnet.6.' + suffix
            torch.testing.assert_close(delta['state_dict'][key][:396], target[key][:396])
            torch.testing.assert_close(delta['state_dict'][key][396:], source['state_dict'][key][118:])
            center = 'bbox_head.prediction_heads.0.center.1.' + suffix
            torch.testing.assert_close(delta['state_dict'][center] * .4, source['state_dict'][center] * .6)
        self.assertNotIn('view_transform.dx', delta['state_dict'])
        self.assertEqual(delta['state_dict']._metadata['fusion_layer.1']['version'], 2)

    def test_reject_unknown_missing_shape_nonfinite(self):
        source, target = self.fixture()
        target['bbox_head.unexpected'] = torch.ones(2)
        with self.assertRaisesRegex(ValueError, 'Incomplete'):
            build_delta(source, target)
        target.pop('bbox_head.unexpected')
        source['state_dict']['fusion_layer.0.weight'][0, 0, 0, 0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            build_delta(source, target)

    def test_compose_preserves_baseline_and_metadata(self):
        source, target = self.fixture()
        delta = build_delta(source, target)
        base = OrderedDict({'pts_middle_encoder.weight': torch.ones(3)})
        base._metadata = {'pts_middle_encoder': {'version': 2}}
        full = compose(dict(state_dict=base), delta)
        torch.testing.assert_close(full['state_dict']['pts_middle_encoder.weight'], base['pts_middle_encoder.weight'])
        self.assertEqual(full['state_dict']._metadata['pts_middle_encoder']['version'], 2)
        with self.assertRaisesRegex(ValueError, 'Baseline'):
            compose(dict(state_dict=delta['state_dict']), delta)


if __name__ == '__main__':
    unittest.main()
