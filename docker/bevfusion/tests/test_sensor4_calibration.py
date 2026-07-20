"""Unit tests for Sensor_4 camera order and rigid-transform consistency."""

import argparse
import unittest

import numpy as np

from app.sensor4_calibration import load_sensor4_calibration


SENSOR_CONFIG = None


class Sensor4CalibrationTest(unittest.TestCase):
    def test_four_cameras_and_inverse_consistency(self):
        calibration = load_sensor4_calibration(
            SENSOR_CONFIG,
            ["camera_front", "camera_front_left", "camera_front_right", "camera_back"],
        )
        self.assertEqual(calibration["lidar_frame"], "lidar1")
        self.assertEqual(len(calibration["cameras"]), 4)
        for camera in calibration["cameras"]:
            self.assertTrue(np.isfinite(camera["lidar2img"]).all())
            identity = camera["lidar2cam"] @ camera["cam2lidar"]
            np.testing.assert_allclose(identity, np.eye(4), atol=1e-9)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor-config", required=True)
    arguments, remaining = parser.parse_known_args()
    SENSOR_CONFIG = arguments.sensor_config
    unittest.main(argv=[__file__] + remaining)

