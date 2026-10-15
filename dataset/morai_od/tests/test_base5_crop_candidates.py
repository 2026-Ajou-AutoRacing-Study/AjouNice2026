import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from analyze_base5_crop_candidates import PROFILES, geometry, native_row_interval, mask_statistics, summarize, inside_analysis_roi


class CropCandidatesTest(unittest.TestCase):
    def test_candidate_roi_boundary_contract(self):
        self.assertTrue(inside_analysis_roi([-100, -30, -3]))
        self.assertTrue(inside_analysis_roi([99.9, 29.9, 1.9]))
        for center in ([100, 0, 0], [0, 30, 0], [0, 0, 2], [-100.1, 0, 0]):
            self.assertFalse(inside_analysis_roi(center))

    def test_all_profiles_fit_every_camera(self):
        for camera in ('CAMERA_1', 'CAMERA_2', 'CAMERA_3', 'CAMERA_4'):
            for profile in PROFILES:
                g = geometry(camera, profile)
                self.assertEqual(g['crop_xyxy'][2] - g['crop_xyxy'][0], profile['w'])
                self.assertEqual(g['crop_xyxy'][3] - g['crop_xyxy'][1], profile['h'])
                self.assertLessEqual(g['crop_xyxy'][3], g['resize_wh'][1])

    def test_front_keeps_bottom_for_shifted_profile(self):
        self.assertEqual(geometry('CAMERA_1', PROFILES[0]), geometry('CAMERA_1', PROFILES[1]))
        self.assertNotEqual(geometry('CAMERA_2', PROFILES[0]), geometry('CAMERA_2', PROFILES[1]))

    def test_matrix_round_trip_and_crop_origin(self):
        for camera in ('CAMERA_1', 'CAMERA_2'):
            for profile in PROFILES:
                g = geometry(camera, profile)
                matrix = np.asarray(g['augmentation_matrix'])
                point = np.asarray([100., 200., 1.])
                np.testing.assert_allclose(np.linalg.inv(matrix) @ (matrix @ point), point)
                native = g['native_window_xyxy']
                np.testing.assert_allclose(matrix @ [native[0], native[1], 1.], [0., 0., 1.], atol=1e-10)

    def test_pixel_center_interval(self):
        self.assertEqual(native_row_interval(10, 1, 2, 5), (2, 7))
        self.assertEqual(native_row_interval(10, 2, 4, 10), (2, 7))

    def test_native_and_model_mask_counts(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[220:230, 100:110] = 1
        prefix = np.r_[0, np.cumsum(mask.sum(axis=1), dtype=np.int64)]
        current = mask_statistics(mask, prefix, [100, 220, 110, 230], geometry('CAMERA_2', PROFILES[0]))
        shifted = mask_statistics(mask, prefix, [100, 220, 110, 230], geometry('CAMERA_2', PROFILES[1]))
        self.assertEqual(current['native_fraction'], 0)
        self.assertEqual(shifted['native_fraction'], 1)
        self.assertTrue(shifted['candidate_camera_gate'])

    def test_shift_can_remove_lower_foreground(self):
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[440:460, 100:120] = 1
        prefix = np.r_[0, np.cumsum(mask.sum(axis=1), dtype=np.int64)]
        current = mask_statistics(mask, prefix, [100, 440, 120, 460], geometry('CAMERA_2', PROFILES[0]))
        shifted = mask_statistics(mask, prefix, [100, 440, 120, 460], geometry('CAMERA_2', PROFILES[1]))
        self.assertEqual(current['native_fraction'], 1)
        self.assertEqual(shifted['native_fraction'], 0)
        self.assertEqual(shifted['lost_bottom'], 400)

    def test_counts_have_explicit_denominator(self):
        metrics = {i: {'test': dict(native_fraction=f, candidate_camera_gate=f>0,
                    model_mask_pixels_nearest=10, model_bbox_extent_px=[5, 5],
                    lost_top=f<1, lost_bottom=0)} for i, f in enumerate((0, .5, 1))}
        r = summarize([0, 1, 2], metrics, 'test')
        self.assertEqual(r['n'], 3)
        self.assertAlmostEqual(r['full_pct'], 100/3)
        self.assertAlmostEqual(r['mean_native_retention_pct'], 50)


if __name__ == '__main__':
    unittest.main()
