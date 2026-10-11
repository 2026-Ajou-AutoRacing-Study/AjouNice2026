"""Sparse initialization safety and actual coordinate-transpose qualification."""
import copy
from collections import OrderedDict
import os
import unittest

import torch

from prepare_base5_lidar_initializer import build_initializer


class InitializerTest(unittest.TestCase):
    def fixture(self):
        camera = dict(state_dict=OrderedDict({'img_backbone.weight': torch.ones(2)}))
        sparse = OrderedDict({'pts_middle_encoder.conv_input.0.weight':
            torch.arange(16*3*3*3*4).reshape(16,3,3,3,4).float(),
            'bbox_head.unused': torch.ones(1)})
        sparse._metadata = {'pts_middle_encoder.conv_input.0': {'version': 2}}
        return camera, dict(state_dict=sparse)

    def test_only_camera_and_sparse_with_xy_permutation_and_metadata(self):
        camera, sparse = self.fixture()
        result = build_initializer(camera, sparse)
        expected = sparse['state_dict']['pts_middle_encoder.conv_input.0.weight'].transpose(1,2)
        torch.testing.assert_close(result['state_dict']['pts_middle_encoder.conv_input.0.weight'], expected)
        self.assertNotIn('bbox_head.unused', result['state_dict'])
        self.assertEqual(result['state_dict']._metadata['pts_middle_encoder.conv_input.0']['version'], 2)

    def test_unknown_layout_and_non_camera_baseline_are_rejected(self):
        camera, sparse = self.fixture()
        sparse['state_dict']._metadata.clear()
        with self.assertRaisesRegex(ValueError, 'layout'):
            build_initializer(camera, sparse)
        camera['state_dict']['bbox_head.weight'] = torch.ones(1)
        with self.assertRaisesRegex(ValueError, 'only audited camera'):
            build_initializer(camera, sparse)

    @unittest.skipUnless(torch.cuda.is_available() and os.environ.get('BASE5_SPARSE_SOURCE'),
                         'Requires trusted sparse checkpoint and CUDA')
    def test_cuda_sparse_encoder_is_xy_transpose_equivariant(self):
        from mmengine.config import Config
        from mmengine.registry import init_default_scope
        from mmengine.utils import import_modules_from_strings
        from mmdet3d.registry import MODELS
        config=Config.fromfile(os.environ['BASE5_SPARSE_CONFIG'])
        import_modules_from_strings(**config.custom_imports)
        init_default_scope('mmdet3d')
        source=torch.load(os.environ['BASE5_SPARSE_SOURCE'],map_location='cpu',weights_only=False)
        adapted=build_initializer(dict(state_dict=OrderedDict({'img_backbone.dummy':torch.ones(1)})),source)
        def encoder(kind, shape, weights):
            cfg=copy.deepcopy(config.model.pts_middle_encoder)
            cfg.type=kind;cfg.sparse_shape=shape
            module=MODELS.build(cfg)
            prefix='pts_middle_encoder.'
            state=OrderedDict((k[len(prefix):],v) for k,v in weights.items() if k.startswith(prefix))
            state._metadata={k[len(prefix):]:v for k,v in weights._metadata.items() if k.startswith(prefix)}
            module.load_state_dict(state,strict=True)
            return module.cuda().eval()
        old=encoder('BEVFusionSparseEncoder',[96,80,41],source['state_dict'])
        new=encoder('MoraiRectangularSparseEncoder',[80,96,41],adapted['state_dict'])
        torch.manual_seed(12345)
        coors=torch.stack((torch.zeros(4000,dtype=torch.long),torch.randint(96,(4000,)),
                           torch.randint(80,(4000,)),torch.randint(41,(4000,))),dim=1)
        coors=torch.unique(coors,dim=0).int().cuda()
        features=torch.randn(len(coors),4,device='cuda')
        with torch.no_grad():
            expected=old(features,coors,1).transpose(-1,-2).contiguous()
            actual=new(features,coors,1)
        torch.testing.assert_close(actual,expected,rtol=1e-4,atol=1e-4)
        print('Sparse XY equivalence:',tuple(actual.shape),
              'max_abs_error=',float((actual-expected).abs().max()),flush=True)


if __name__ == '__main__':
    unittest.main()
