"""Contract tests for the Base3 in-memory runtime preprocessor."""

import unittest

import cv2
import numpy as np

from app.online_preprocessor import Base3OnlinePreprocessor, preprocessing_options_from_config


def calibration():
    cameras = []
    for index, (name, width, height) in enumerate((
        ("camera_front", 1280, 720),
        ("camera_front_left", 640, 480),
        ("camera_front_right", 640, 480),
        ("camera_back", 1280, 720),
    )):
        lidar2cam = np.eye(4, dtype=np.float32)
        lidar2cam[0, 3] = float(index)
        cameras.append({
            "frame_id": name,
            "width": width,
            "height": height,
            "cam2img": np.eye(3, dtype=np.float32),
            "lidar2cam": lidar2cam,
            "cam2lidar": np.linalg.inv(lidar2cam),
        })
    return {"cameras": cameras}


class OnlinePreprocessorTest(unittest.TestCase):
    def test_explicit_ros255_adapter_preserves_source_and_native_default(self):
        raw=np.asarray([[1,2,0,255],[2,1,0,127],[0,0,0,0]],dtype=np.float32)
        before=raw.copy()
        ros=Base3OnlinePreprocessor(calibration(),camera_workers=1,
                                   point_intensity_source='ros255',remove_zero_returns=True)
        try:
            normalized=ros._filter_points(raw)
            np.testing.assert_array_equal(raw,before)
            np.testing.assert_array_equal(normalized[:,:3],raw[:2,:3])
            np.testing.assert_array_equal(normalized[:,3],raw[:2,3]*np.float32(1./255.))
            np.testing.assert_array_equal(self.preprocessor._filter_points(raw),raw)
            bad=raw.copy();bad[0,3]=256
            with self.assertRaisesRegex(ValueError,'0..255'):ros._filter_points(bad)
        finally:ros.close()

    def test_training_config_uses_deterministic_val_when_test_is_none(self):
        cfg=dict(point_cloud_range=[-100,-30,-3,100,30,2],
                 model=dict(view_transform=dict(image_size=[512,1056],feature_size=[64,132])),
                 test_dataloader=None,val_dataloader=dict(dataset=dict(pipeline=[
                     dict(type='MoraiImageAug3D',final_dim=[512,1056],resize_lim=[.825,.825],is_train=False)])))
        self.assertEqual(preprocessing_options_from_config(cfg)['final_dim'],(512,1056))
        cfg['val_dataloader']['dataset']['pipeline'][0]['is_train']=True
        with self.assertRaisesRegex(ValueError,'deterministic'):preprocessing_options_from_config(cfg)

    def test_late_rgb_preserves_pixels_and_projection(self):
        rng = np.random.default_rng(261011)
        late = Base3OnlinePreprocessor(calibration(), camera_workers=1, rgb_after_crop=True)
        try:
            for camera in calibration()['cameras']:
                image = rng.integers(0, 256, (camera['height'], camera['width'], 3), dtype=np.uint8)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 90])
                self.assertTrue(ok)
                argument = (camera, {'image/' + camera['frame_id']: encoded.tobytes()})
                np.testing.assert_array_equal(self.preprocessor._decode_resize(argument), late._decode_resize(argument))
                self.assertEqual(self.preprocessor._resize_crop(camera), late._resize_crop(camera))
                np.testing.assert_array_equal(self.preprocessor._augmentation_matrix(camera), late._augmentation_matrix(camera))
        finally:
            late.close()

    def test_config_drives_high_resolution_and_rejects_mismatched_projection(self):
        config = dict(point_cloud_range=[-30, -30, -3, 30, 30, 2],
            model=dict(view_transform=dict(image_size=[320, 896], feature_size=[40, 112])),
            test_dataloader=dict(dataset=dict(pipeline=[dict(type='MoraiImageAug3D',
                final_dim=[320, 896], resize_lim=[0.70, 0.70], is_train=False)])))
        options = preprocessing_options_from_config(config)
        self.assertEqual(options['final_dim'], (320, 896))
        highres = Base3OnlinePreprocessor(calibration(), camera_workers=1, **options)
        try:
            for camera in calibration()['cameras']:
                resized, crop = highres._resize_crop(camera)
                self.assertEqual((crop[3] - crop[1], crop[2] - crop[0]), (320, 896))
                self.assertGreaterEqual(resized[1], 320)
        finally:
            highres.close()
        config['model']['view_transform']['image_size'] = [256, 704]
        with self.assertRaisesRegex(ValueError, 'sizes differ'):
            preprocessing_options_from_config(config)
        config['test_dataloader']['dataset']['pipeline'][0]['final_dim'] = [320, 880]
        with self.assertRaisesRegex(ValueError, 'divisible by 32'):
            preprocessing_options_from_config(config)

    def setUp(self):
        self.preprocessor = Base3OnlinePreprocessor(
            calibration(), camera_workers=2
        )

    def tearDown(self):
        self.preprocessor.close()

    def test_produces_base3_batch_and_filters_points(self):
        blobs = {}
        for camera in calibration()["cameras"]:
            image = np.full(
                (camera["height"], camera["width"], 3), 127, dtype=np.uint8
            )
            ok, encoded = cv2.imencode(".jpg", image)
            self.assertTrue(ok)
            blobs["image/{}".format(camera["frame_id"])] = encoded.tobytes()
        points = np.asarray([
            [1.0, 2.0, 0.0, 30.0],
            [31.0, 0.0, 0.0, 20.0],
            [0.0, 0.0, np.nan, 10.0],
        ], dtype=np.float32)
        batch = self.preprocessor.prepare(
            points,
            blobs,
            sample_idx=7,
            timestamp_sec=12.5,
            lidar_pose_world=(3.0, 4.0, 0.5, np.pi / 2.0),
            scene_id="online-test",
        )
        self.assertEqual(tuple(batch["inputs"]["img"][0].shape), (4, 3, 256, 704))
        self.assertEqual(tuple(batch["inputs"]["points"][0].shape), (1, 4))
        meta = batch["data_samples"][0].metainfo
        self.assertEqual(meta["sample_idx"], "7")
        self.assertEqual(meta["scene_id"], "online-test")
        self.assertAlmostEqual(meta["timestamp"], 12.5)
        np.testing.assert_allclose(
            meta["lidar2global"],
            np.asarray([
                [0.0, -1.0, 0.0, 3.0],
                [1.0, 0.0, 0.0, 4.0],
                [0.0, 0.0, 1.0, 0.5],
                [0.0, 0.0, 0.0, 1.0],
            ], dtype=np.float32),
            atol=1.0e-6,
        )
        self.assertEqual(meta["cam2img"].shape, (4, 4, 4))
        self.assertEqual(meta["img_aug_matrix"].shape, (4, 4, 4))
        self.assertAlmostEqual(meta["img_aug_matrix"][0, 0, 0], 0.55)
        self.assertAlmostEqual(meta["img_aug_matrix"][1, 0, 0], 1.10)

    def test_rejects_old_five_feature_contract(self):
        with self.assertRaisesRegex(ValueError, "Nx4"):
            self.preprocessor._filter_points(np.zeros((3, 5), dtype=np.float32))

    def test_longitudinal_range60_retains_far_points_without_widening_lateral_roi(self):
        points = np.asarray([[45., 0., 0., 1.], [-45., 0., 0., 1.],
                             [61., 0., 0., 1.], [0., 31., 0., 1.]], dtype=np.float32)
        with self.assertRaisesRegex(ValueError, 'no finite LiDAR points'):
            self.preprocessor._filter_points(points)
        self.preprocessor.point_cloud_range = np.asarray([-60, -30, -3, 60, 30, 2])
        actual = self.preprocessor._filter_points(points)
        self.assertEqual(len(actual), 2)
        np.testing.assert_array_equal(actual[:, 0], [45., -45.])

    def test_rejects_invalid_lidar_pose(self):
        with self.assertRaisesRegex(ValueError, "finite x, y, z, yaw"):
            self.preprocessor._lidar2global((1.0, 2.0, np.nan, 0.0))


if __name__ == "__main__":
    unittest.main()
