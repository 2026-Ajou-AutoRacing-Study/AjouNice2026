"""Contract tests for the Base3 in-memory runtime preprocessor."""

import unittest

import cv2
import numpy as np

from app.online_preprocessor import Base3OnlinePreprocessor


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

    def test_rejects_invalid_lidar_pose(self):
        with self.assertRaisesRegex(ValueError, "finite x, y, z, yaw"):
            self.preprocessor._lidar2global((1.0, 2.0, np.nan, 0.0))


if __name__ == "__main__":
    unittest.main()
