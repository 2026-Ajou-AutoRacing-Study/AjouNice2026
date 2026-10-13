import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from report_base5_camera_crop import crop_start, height_state, window_state, observations, pixel_rate


class CameraCropTest(unittest.TestCase):
    def test_front_and_side_cutoffs(self):
        self.assertAlmostEqual(crop_start('CAMERA_1'), 1 - 320 / 504)
        self.assertAlmostEqual(crop_start('CAMERA_2'), 1 - 320 / 672)

    def test_proportional_resolution_keeps_view(self):
        for camera in ('CAMERA_1', 'CAMERA_2'):
            self.assertAlmostEqual(crop_start(camera), crop_start(camera, 480, 1344))
            self.assertLess(crop_start(camera, 448, 896), crop_start(camera))

    def test_bbox_classification(self):
        self.assertEqual(height_state(.1, .2, .3), 'removed')
        self.assertEqual(height_state(.1, .4, .3), 'partial')
        self.assertEqual(height_state(.3, .4, .3), 'full')

    def test_pixel_counts(self):
        result = pixel_rate([dict(native_pixels=100, kept_native_pixels=k)
                             for k in (0, 40, 100)])
        self.assertEqual([result[k] for k in ('full', 'partial', 'removed')], [1, 1, 1])
        self.assertAlmostEqual(result['mean_instance_pixel_retention'], 1.4 / 3)

    def test_shifted_window_can_cut_the_bottom(self):
        self.assertEqual(window_state(.4, .9, .35, .82), 'partial')
        self.assertEqual(window_state(.85, .95, .35, .82), 'removed')
        self.assertEqual(window_state(.4, .6, .35, .82), 'full')

    def test_invalid_pixel_count(self):
        with self.assertRaises(ValueError):
            pixel_rate([dict(native_pixels=10, kept_native_pixels=11)])

    def test_shared_component_camera_is_counted_once(self):
        camera = dict(camera='CAMERA_2', native_instance_pixels=40,
                      native_bottom_crop_instance_pixels=10, crop_320x896_instance_pixels=20,
                      native_rect=[1, 200, 20, 300], camera_instance_id=7,
                      annotation_class='Pedestrian')
        component = dict(asset='Test', training_class='pedestrian',
                         range_xy_m=10, lidar_points=0, cameras=[camera])
        entry = dict(id='test', status='accepted_candidate', capture_root='unused',
                     audit=dict(components=[component, component.copy()]))
        rows, conflicts = observations([entry])
        self.assertEqual(len(rows), 1)
        self.assertEqual(conflicts, 0)
        self.assertIsNone(rows[0]['lidar_points'])


if __name__ == '__main__':
    unittest.main()
