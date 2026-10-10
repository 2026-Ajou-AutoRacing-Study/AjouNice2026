"""Base5-only valid returns and RGB8 transport; no accuracy assertions."""
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

from mmengine.config import Config
from mmengine.dataset import Compose
from mmengine.utils import import_modules_from_strings
from mmdet3d.models.data_preprocessors import Det3DDataPreprocessor
from mmdet3d.structures.points import LiDARPoints
from mmdet3d.utils import register_all_modules
from projects.BEVFusion.bevfusion.transforms_3d import BEVFusionGlobalRotScaleTrans
import numpy as np
import torch

from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config
from perception.detection3d.bevfusion.plugin.morai_image_aug import MoraiImageAug3D
from perception.detection3d.bevfusion.plugin.morai_point_filter import MoraiFilterInvalidPoints


ROOT = Path(__file__).resolve().parents[1]


class Base5InputPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        register_all_modules(init_default_scope=True)
        cls.cfg = Config.fromfile(str(ROOT/'configs/bevfusion_base5_dense_512_native_jpeg90.py'))
        import_modules_from_strings(**cls.cfg.custom_imports)

    def test_filter_preserves_real_near_zero_intensity_and_gt(self):
        raw = torch.tensor([[0,0,0,0], [0,0,0,10], [.001,0,0,0],
                            [1,2,3,0], [float('nan'),1,1,1],
                            [1,1,1,float('inf')]])
        boxes = object()
        result = MoraiFilterInvalidPoints().transform(dict(
            points=LiDARPoints(raw.clone(), points_dim=4), gt_bboxes_3d=boxes))
        torch.testing.assert_close(result['points'].tensor, raw[2:4])
        self.assertIs(result['gt_bboxes_3d'], boxes)
        self.assertEqual(raw.shape, (6,4))

    def test_empty_remains_empty(self):
        for raw in (torch.zeros((0,4)), torch.zeros((3,4))):
            result = MoraiFilterInvalidPoints().transform(dict(points=LiDARPoints(raw,points_dim=4)))
            self.assertEqual(tuple(result['points'].tensor.shape), (0,4))

    def test_filter_before_translation_prevents_phantom_cluster(self):
        raw=torch.tensor([[0,0,0,0],[0,0,0,0],[4,1,0,0]],dtype=torch.float32)
        aug=BEVFusionGlobalRotScaleTrans(rot_range=[0,0],scale_ratio_range=[1,1],translation_std=.2)
        with patch.object(np.random,'normal',return_value=np.asarray([.2,.1,0],np.float32)):
            legacy=aug.transform(dict(points=LiDARPoints(raw.clone(),points_dim=4)))
            filtered=MoraiFilterInvalidPoints().transform(dict(points=LiDARPoints(raw.clone(),points_dim=4)))
            result=aug.transform(filtered)
        torch.testing.assert_close(legacy['points'].tensor[:2,:3],torch.tensor([[.2,.1,0],[.2,.1,0]]))
        torch.testing.assert_close(result['points'].tensor,legacy['points'].tensor[2:])
        # Filtering AFTER translation would leave all three rows: wrong order.
        late=MoraiFilterInvalidPoints().transform(legacy)
        self.assertEqual(len(late['points']),3)

    def test_config_all_splits_and_legacy_unchanged(self):
        for split in ('train', 'val', 'test'):
            p = self.cfg[split+'_dataloader'].dataset.pipeline
            names = [t.type for t in p]
            self.assertEqual(names.count('MoraiFilterInvalidPoints'),1)
            self.assertEqual(names.index('MoraiFilterInvalidPoints'),names.index('LoadPointsFromFile')+1)
            self.assertFalse(next(t for t in p if t.type=='BEVLoadMultiViewImageFromFiles').to_float32)
            self.assertEqual(next(t for t in p if t.type=='MoraiImageAug3D').output_dtype,'uint8')
            self.assertNotIn('MoraiOnlineImageDomainAug3D',names)
            Compose(p)  # all registered and valid constructor arguments
        legacy = Config.fromfile(str(ROOT/'configs/bevfusion_base3_b0_4cam_lidar.py'))
        for split in ('train','val','test'):
            p = legacy[split+'_dataloader'].dataset.pipeline
            self.assertNotIn('MoraiFilterInvalidPoints',[t.type for t in p])
            self.assertTrue(next(t for t in p if t.type=='BEVLoadMultiViewImageFromFiles').to_float32)
        before = Config.fromfile(str(ROOT/'configs/bevfusion_base5_dense_512_separated_candidate.py'))
        self.assertEqual(self.cfg.model.to_dict(),before.model.to_dict())
        self.assertEqual(next(t for t in self.cfg.train_pipeline if t.type=='BEVFusionGlobalRotScaleTrans'),
                         next(t for t in before.train_pipeline if t.type=='BEVFusionGlobalRotScaleTrans'))

    def test_runtime_uses_same_filter_only_when_opted_in(self):
        raw = np.asarray([[0,0,0,0], [.001,0,0,0], [1,2,0,0],
                          [1,1,0,np.nan], [0,31,0,1],
                          [100,0,0,1],[-100,0,0,1],[0,30,0,1],
                          [0,-30,0,1],[1,0,-3,1],[1,0,2,1]],np.float32)
        options = preprocessing_options_from_config(self.cfg)
        self.assertTrue(options['remove_zero_returns'])
        calibration=dict(cameras=[dict(width=1280,height=720,frame_id='front',
            cam2img=np.eye(3),lidar2cam=np.eye(4),cam2lidar=np.eye(4))])
        runtime = Base3OnlinePreprocessor(calibration,camera_workers=1,**options)
        legacy = Base3OnlinePreprocessor(calibration,camera_workers=1)
        try:
            np.testing.assert_array_equal(runtime._filter_points(raw),raw[1:3])
            np.testing.assert_array_equal(legacy._filter_points(raw),raw[[0,1,2,7,8,9,10]])
            valid=MoraiFilterInvalidPoints().transform(dict(points=LiDARPoints(raw,points_dim=4)))['points'].tensor.numpy()
            np.testing.assert_array_equal(runtime._filter_points(valid),runtime._filter_points(raw))
            with self.assertRaisesRegex(ValueError,'no finite LiDAR points'):
                runtime._filter_points(np.zeros((3,4),np.float32))
        finally:
            runtime.close(); legacy.close()

    def test_uint8_mixed_views_exact_pixels_geometry_and_normalization(self):
        rng=np.random.RandomState(9)
        imgs=[rng.randint(0,256,(720,1280,3),np.uint8),
              rng.randint(0,256,(480,640,3),np.uint8)]
        meta=OrderedDict((str(i),dict(height=im.shape[0],width=im.shape[1])) for i,im in enumerate(imgs))
        data=dict(img=[np.pad(im,((0,720-im.shape[0]),(0,1280-im.shape[1]),(0,0))) for im in imgs],images=meta)
        kwargs=dict(final_dim=[512,1056],resize_lim=[.825,.825],bot_pct_lim=[0,0],rot_lim=[0,0],rand_flip=False,is_train=False)
        old=MoraiImageAug3D(**kwargs).transform(deepcopy(data))
        new=MoraiImageAug3D(**kwargs,output_dtype='uint8').transform(deepcopy(data))
        np.testing.assert_array_equal(old['img_aug_matrix'],new['img_aug_matrix'])
        dp=Det3DDataPreprocessor(mean=[123.675,116.28,103.53],std=[58.395,57.12,57.375],bgr_to_rgb=False)
        for left,right in zip(old['img'],new['img']):
            np.testing.assert_array_equal(left,right)
            self.assertEqual(right.dtype,np.uint8)
            self.assertTrue(right.flags.c_contiguous)
            a=torch.from_numpy(left).permute(2,0,1)
            b=torch.from_numpy(right).permute(2,0,1)
            torch.testing.assert_close(dp.preprocess_img(a),dp.preprocess_img(b),rtol=0,atol=0)

    def test_bad_dtype_and_insufficient_crop_rejected(self):
        kwargs=dict(final_dim=[512,1056],resize_lim=[.825,.825],bot_pct_lim=[0,0],rot_lim=[0,0],rand_flip=False,is_train=False)
        with self.assertRaisesRegex(ValueError,'output_dtype'):
            MoraiImageAug3D(**kwargs,output_dtype='float16')
        aug=MoraiImageAug3D(**dict(kwargs,resize_lim=[.1,.1]))
        with self.assertRaisesRegex(ValueError,'final_dim'):
            aug.transform(dict(img=[np.zeros((720,1280,3),np.uint8)],images=OrderedDict(front=dict(height=720,width=1280))))


if __name__=='__main__': unittest.main()
