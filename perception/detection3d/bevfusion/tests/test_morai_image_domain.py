"""CPU tests in the team image; no GPU inference or training required."""
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
from mmengine.config import Config
import numpy as np

from perception.detection3d.bevfusion.plugin.morai_image_aug import (
    MoraiImageAug3D, MoraiOnlineImageDomainAug3D)


ROOT = Path(__file__).resolve().parents[1]


class ImageDomainTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.RandomState(7)
        self.rgb = rng.randint(0, 256, (64, 96, 3), dtype=np.uint8)

    def transform(self, **kwargs):
        return MoraiOnlineImageDomainAug3D(
            jpeg_prob=kwargs.pop('jpeg_prob', 1), jpeg_quality=(90, 90),
            blur_prob=0, exposure_prob=0, noise_prob=0,
            color_order='rgb', **kwargs)

    def test_rgb_matches_native_online_bgr_roundtrip(self):
        ok, encoded = cv2.imencode('.jpg', self.rgb[:, :, ::-1],
                                   [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.assertTrue(ok)
        expected = cv2.imdecode(encoded, cv2.IMREAD_COLOR)[:, :, ::-1]
        result = self.transform().transform(dict(img=[self.rgb]))['img'][0]
        np.testing.assert_array_equal(result, expected.astype(np.float32))

    def test_native_padding_is_not_encoded_or_changed(self):
        padded = np.zeros((80, 120, 3), np.float32)
        padded[:64, :96] = self.rgb
        data = dict(img=[padded], images=OrderedDict(
            side=dict(height=64, width=96)), lidar2img=np.eye(4)[None],
            gt_bboxes_3d='sentinel')
        before = padded.copy()
        calibration = data['lidar2img']
        result = self.transform(native_resolution=True).transform(data)
        expected = self.transform().transform(dict(img=[self.rgb]))['img'][0]
        np.testing.assert_array_equal(result['img'][0][:64, :96], expected)
        np.testing.assert_array_equal(result['img'][0][64:], before[64:])
        np.testing.assert_array_equal(result['img'][0][:, 96:], before[:, 96:])
        np.testing.assert_array_equal(padded, before)
        self.assertIs(result['lidar2img'], calibration)
        self.assertEqual(result['gt_bboxes_3d'], 'sentinel')

    def test_disabled_is_pixel_identity(self):
        result = self.transform(jpeg_prob=0).transform(dict(img=[self.rgb]))
        np.testing.assert_array_equal(result['img'][0], self.rgb)

    def test_seed_reproducibility_and_80_percent_sampling(self):
        aug = self.transform(jpeg_prob=.8)
        np.random.seed(77)
        first = [aug.transform(dict(img=[self.rgb]))['img'][0] for _ in range(200)]
        np.random.seed(77)
        second = [aug.transform(dict(img=[self.rgb]))['img'][0] for _ in range(200)]
        for left, right in zip(first, second):
            np.testing.assert_array_equal(left, right)
        changed = sum(not np.array_equal(x, self.rgb) for x in first)
        self.assertTrue(140 <= changed <= 180, changed)

    def test_encode_failure_is_not_silently_skipped(self):
        with patch.object(cv2, 'imencode', return_value=(False, None)):
            with self.assertRaisesRegex(RuntimeError, 'encoder failed'):
                self.transform().transform(dict(img=[self.rgb]))

    def test_metadata_mismatch_is_rejected(self):
        for cameras in ({}, dict(front=dict(height=65, width=96))):
            with self.assertRaises(ValueError):
                self.transform(native_resolution=True).transform(dict(
                    img=[self.rgb], images=cameras))

    def test_invalid_settings_are_rejected(self):
        for settings in (dict(jpeg_prob=-.1), dict(blur_prob=1.1),
                         dict(jpeg_quality=(95, 90)), dict(color_order='unknown')):
            with self.assertRaises(ValueError):
                MoraiOnlineImageDomainAug3D(**settings)

    def test_train_only_order_and_geometry_contract(self):
        cfg = Config.fromfile(str(ROOT / 'configs/bevfusion_base5_dense_512_jpeg90_train.py'))
        original = Config.fromfile(str(ROOT / 'configs/bevfusion_base5_dense_512_separated_candidate.py'))
        self.assertEqual(cfg.model.to_dict(), original.model.to_dict())
        self.assertEqual(cfg.point_cloud_range, [-100., -30., -3., 100., 30., 2.])
        pipeline = cfg.train_dataloader.dataset.pipeline
        domain = [t for t in pipeline if t.type == 'MoraiOnlineImageDomainAug3D']
        self.assertEqual(len(domain), 1)
        self.assertEqual(domain[0].jpeg_quality, [90, 90])
        self.assertEqual(domain[0].jpeg_prob, 1.0)
        self.assertTrue(domain[0].native_resolution)
        self.assertEqual(domain[0].color_order, 'rgb')
        for key in ('blur_prob', 'exposure_prob', 'noise_prob'):
            self.assertEqual(domain[0][key], 0)
        self.assertLess(next(i for i,t in enumerate(pipeline) if t.type == 'MoraiOnlineImageDomainAug3D'),
                        next(i for i,t in enumerate(pipeline) if t.type == 'MoraiImageAug3D'))
        for split in ('val_dataloader', 'test_dataloader'):
            self.assertEqual(cfg[split].dataset.pipeline, original[split].dataset.pipeline)
            for item in cfg[split].dataset.pipeline:
                if item.type == 'MoraiOnlineImageDomainAug3D':
                    self.assertTrue(all(item[key] == 0 for key in
                        ('jpeg_prob','blur_prob','exposure_prob','noise_prob')))

    def test_geometry_and_pixels_match_online_preprocess_after_jpeg(self):
        # Native mixed-resolution padding and resize order, no labels changed.
        rng = np.random.RandomState(1)
        images = [rng.randint(0, 256, (720,1280,3), np.uint8),
                  rng.randint(0, 256, (480,640,3), np.uint8)]
        meta = OrderedDict((str(i),dict(height=im.shape[0],width=im.shape[1]))
                           for i,im in enumerate(images))
        padded = [np.pad(im,((0,720-im.shape[0]),(0,1280-im.shape[1]),(0,0))).astype(np.float32)
                  for im in images]
        aug = MoraiImageAug3D(final_dim=[512,1056], resize_lim=[.825,.825],
            bot_pct_lim=[0,0], rot_lim=[0,0], rand_flip=False, is_train=False)
        native_data = dict(img=padded, images=meta)
        reference = aug.transform(deepcopy(native_data))
        encoded = self.transform(native_resolution=True).transform(deepcopy(native_data))
        result = aug.transform(encoded)
        np.testing.assert_array_equal(reference['img_aug_matrix'], result['img_aug_matrix'])
        for before, actual in zip(images, result['img']):
            ok, blob = cv2.imencode('.jpg', before[:,:,::-1], [cv2.IMWRITE_JPEG_QUALITY,90])
            self.assertTrue(ok)
            decoded = cv2.imdecode(blob,cv2.IMREAD_COLOR)[:,:,::-1]
            scale = 1056 / decoded.shape[1]
            expected = cv2.resize(decoded,(1056,int(decoded.shape[0]*scale)),
                                  interpolation=cv2.INTER_CUBIC)[-512:]
            np.testing.assert_array_equal(actual,expected.astype(np.float32))


if __name__ == '__main__':
    unittest.main()
