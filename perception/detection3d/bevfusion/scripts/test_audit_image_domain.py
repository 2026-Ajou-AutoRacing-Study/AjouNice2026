"""Host CPU tests for codec evidence, independent of model dependencies."""
from pathlib import Path
import unittest

import cv2
import numpy as np

from audit_base5_image_domain import audit_bag, codec_metadata, model_crop, quality_reference


class DomainAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = quality_reference()

    def test_exact_quantization_match_not_file_size_guess(self):
        image = np.zeros((32, 32, 3), np.uint8)
        for quality in (75, 90, 95):
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            self.assertTrue(ok)
            info = codec_metadata(jpeg.tobytes(), self.reference)
            self.assertEqual(info['opencv_quality_matches'], [quality])
            self.assertEqual(info['sampling'], 2)

    def test_png_does_not_receive_a_jpeg_quality(self):
        ok, png = cv2.imencode('.png', np.zeros((16,16,3), np.uint8))
        self.assertTrue(ok)
        info = codec_metadata(png.tobytes(), self.reference)
        self.assertEqual(info['codec'], 'PNG')
        self.assertEqual(info['opencv_quality_matches'], [])
        self.assertIsNone(info['sampling'])

    def test_common_width_bottom_crop_for_mixed_native_sizes(self):
        for shape in ((720,1280,3), (480,640,3)):
            image = np.zeros(shape, np.uint8)
            self.assertEqual(model_crop(image).shape, (512,1056,3))

    def test_locked_bag_is_rejected_before_any_read(self):
        with self.assertRaisesRegex(ValueError, 'Locked evaluation'):
            audit_bag(Path('/missing/evaluation_locked/never_read.bag'), 32, self.reference)


if __name__ == '__main__':
    unittest.main()
