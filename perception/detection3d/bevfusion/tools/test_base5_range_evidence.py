"""CPU tests for diagnostic semantics; never require a GPU or source capture."""
import unittest
from unittest.mock import patch
import json

from audit_base5_range_evidence import audit, camera_key, camera_summary, compare_metrics, distance_bin, summarize


def observation(pixels=100, extent=(10, 10), fraction=1):
    return {'candidates': {'bottom512w1056': dict(model_mask_pixels_nearest=pixels,
            model_bbox_extent_px=extent, native_fraction=fraction,
            candidate_camera_gate=pixels >= 16 and min(extent) >= 4)}}


def row(points=0, obs=None, ambiguous=False, token='one'):
    return dict(token=token, points=points,
                **camera_summary([] if obs is None else [obs], ambiguous))


class RangeEvidenceTests(unittest.TestCase):
    def test_half_open_distance_bins(self):
        for value, name in [(0, '0_10m'), (9.999, '0_10m'), (10, '10_20m'),
                            (30, '30_50m'), (50, '50_80m'), (80, '80_100m'),
                            (100, '100plus_corner'), (104, '100plus_corner')]:
            self.assertEqual(distance_bin(value), name)
        for bad in (-1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                distance_bin(bad)

    def test_actual_foreground_not_bbox_size(self):
        info = camera_summary([observation(1, (200, 200))], False)
        self.assertFalse(info['camera_gate'])
        self.assertFalse(info['camera_rich_proxy'])

    def test_small_extent_not_rich(self):
        self.assertFalse(camera_summary([observation(200, (2, 100))], False)['camera_rich_proxy'])

    def test_ambiguous_components_are_not_invisible(self):
        info = camera_summary([observation()], True)
        self.assertIsNone(info['camera_gate'])
        self.assertIsNone(info['best_visible_mask_pixels'])
        result = summarize([row(1, observation(), True)])
        self.assertEqual(result['camera_unknown'], 1)
        self.assertEqual(result['weak_support_proxy'], 0)

    def test_camera_only_can_have_rich_support(self):
        result = summarize([row(0, observation())])
        self.assertEqual(result['camera_only'], 1)
        self.assertEqual(result['rich_support_proxy'], 1)
        self.assertEqual(result['weak_support_proxy'], 0)

    def test_weak_rich_and_zero_camera_separate(self):
        result = summarize([row(1), row(5), row(20), row(0, observation())])
        self.assertEqual(result['weak_support_proxy'], 1)
        self.assertEqual(result['rich_support_proxy'], 2)
        self.assertEqual(result['gt'], 4)
        self.assertEqual(result['frames'], 1)

    def test_gate_and_rich_thresholds(self):
        self.assertTrue(camera_summary([observation(16, (4, 4))], False)['camera_gate'])
        self.assertFalse(camera_summary([observation(63, (8, 8))], False)['camera_rich_proxy'])
        self.assertTrue(camera_summary([observation(64, (8, 8))], False)['camera_rich_proxy'])

    def test_best_view_and_crop_loss(self):
        info = camera_summary([observation(80), observation(100, fraction=.5)], False)
        self.assertEqual(info['best_visible_mask_pixels'], 100)
        self.assertTrue(info['any_extra_crop_loss'])

    def test_saved_and_cached_camera_keys_match(self):
        common = dict(camera='CAMERA_1', camera_instance_id=-850002, native_rect=[1, 2, 3, 4])
        self.assertEqual(camera_key('scene', dict(common, raw_class='Pedestrian')),
                         camera_key('scene', dict(common, annotation_class='Pedestrian')))

    def test_empty_groups(self):
        result = summarize([])
        self.assertIsNone(result['weak_support_pct'])
        self.assertIsNone(result['mask_pixels_p10_p50_p90'])

    def test_ap_receipt_denominators_verified(self):
        groups, metrics = {}, {}
        for cls in ('vehicle', 'pedestrian', 'obstacle'):
            for name in ('0_10m', '10_20m', '20_30m', '30_50m', '50_80m', '80_100m'):
                groups[f'val/{cls}/{name}'] = {'gt': 3}
                metrics[f'val/range/{name}/{cls}_GT'] = 3
                metrics[f'val/range/{name}/{cls}_mean_AP'] = .5
        payload = json.dumps(dict(epoch=15, metrics=metrics))
        with patch('audit_base5_range_evidence.Path.read_text', return_value=payload), \
                patch('audit_base5_range_evidence.sha256', return_value='receipt_hash'):
            self.assertEqual(len(compare_metrics(groups, 'receipt')['comparisons']), 18)
            groups['val/obstacle/80_100m']['gt'] = 4
            with self.assertRaisesRegex(ValueError, 'denominator mismatch'):
                compare_metrics(groups, 'receipt')

    def test_source_dataset_cannot_be_analysis_output(self):
        with self.assertRaisesRegex(ValueError, 'outside source dataset'):
            audit('/tmp/range_audit_input_guard', '/tmp/unused', '/tmp/unused',
                  '/tmp/range_audit_input_guard/analysis')


if __name__ == '__main__':
    unittest.main()
