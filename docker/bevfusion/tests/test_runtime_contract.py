"""Tests for the server-side single-frame and temporal point contracts."""

import unittest

import numpy as np
from types import SimpleNamespace

from app.inference_server import parse_points, InferenceServer


class RuntimeContractTest(unittest.TestCase):
    def test_rgb_crop_size_mismatch_is_rejected_before_inference(self):
        server=InferenceServer.__new__(InferenceServer)
        server.calibration=dict(config_sha256='sensor',cameras=[dict(frame_id='front')],intrinsic_source='horizontal_fov')
        server.model=SimpleNamespace(preprocessor=SimpleNamespace(final_dim=(512,1056)))
        metadata=dict(sensor_config_sha256='sensor',camera_order=['front'],intrinsic_source='horizontal_fov',
                      point_count=1,point_dim=4,image_encoding='rgb8',image_height=256,image_width=2112)
        with self.assertRaisesRegex(ValueError,'crop size differs'):
            server._handle_request(None,metadata,{'points':np.zeros((1,4),dtype='<f4').tobytes()})

    def test_intensity_contract_mismatch_is_rejected_before_inference(self):
        server=InferenceServer.__new__(InferenceServer)
        server.calibration=dict(config_sha256='sensor',cameras=[dict(frame_id='front')],intrinsic_source='horizontal_fov')
        server.model=SimpleNamespace(preprocessor=SimpleNamespace(final_dim=(512,1056),point_intensity_source='native'))
        metadata=dict(sensor_config_sha256='sensor',camera_order=['front'],intrinsic_source='horizontal_fov',
                      point_count=1,point_dim=4,point_intensity_source='ros255')
        with self.assertRaisesRegex(ValueError,'intensity source contract differs'):
            server._handle_request(None,metadata,{'points':np.zeros((1,4),dtype='<f4').tobytes()})

    def test_parses_xyzi_payload(self):
        expected = np.arange(12, dtype="<f4").reshape(3, 4)
        actual = parse_points(
            {"points_blob": "points", "point_dim": 4},
            {"points": expected.tobytes()},
        )
        np.testing.assert_array_equal(actual, expected)

    def test_parses_xyzi_time_payload(self):
        expected = np.arange(15, dtype="<f4").reshape(3, 5)
        actual = parse_points(
            {"points_blob": "points", "point_dim": 5},
            {"points": expected.tobytes()},
        )
        np.testing.assert_array_equal(actual, expected)

    def test_rejects_unsupported_feature_width(self):
        with self.assertRaisesRegex(ValueError, "4 or 5"):
            parse_points(
                {"points_blob": "points", "point_dim": 6},
                {"points": np.zeros((2, 6), dtype="<f4").tobytes()},
            )


if __name__ == "__main__":
    unittest.main()
