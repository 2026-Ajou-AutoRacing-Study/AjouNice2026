"""CPU-only support semantics; no ROS, MORAI, GPU or real capture dependency."""
import copy
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import audit_base5_far_vehicle_support as audit


def fixtures():
    inst = dict(bbox_3d=[90., 4., 0., 4., 2., 1.5, 0.], bbox_label_3d=0,
                num_lidar_pts=2, component_id='8500:Vehicle:0', asset='Sedan')
    frame = dict(token='scene', scene_group_id='physical-link-A', instances=[inst])
    doc = dict(metainfo=dict(classes=['vehicle', 'pedestrian']), data_list=[frame])
    observation = dict(token='scene', component_id=inst['component_id'], cls='vehicle',
        split='val', asset='Sedan', points=2, native_points=2, center_lidar_m=[90., 4., 0.],
        camera_gate=True, best_visible_mask_pixels=100,
        best_visible_bbox_extent_px=[14., 10.], family='highway')
    return doc, observation


class SupportAuditTest(unittest.TestCase):
    def test_range_boundary_and_corner(self):
        self.assertEqual(audit.range_band(80), '80_100m')
        self.assertEqual(audit.range_band(100), '100plus_corner')
        with self.assertRaises(ValueError):
            audit.range_band(float('nan'))

    def test_direction_policies_are_distinct(self):
        self.assertEqual(audit.direction([2, 4]), 'front')
        self.assertEqual(audit.direction([2, 4], 'sector'), 'left')
        self.assertEqual(audit.direction([-2, -4], 'sector'), 'right')

    def test_exact_join_and_small_val_warning(self):
        doc, observation = fixtures()
        rows = audit.join_infos({'val': doc}, [observation])
        report = audit.captured_report(rows)
        self.assertFalse(report['split_modified'])
        far = report['groups']['val/80_100m']
        self.assertEqual((far['gt'], far['scenes'], far['scene_groups'], far['assets']), (1, 1, 1, 1))
        self.assertEqual(far['point_fraction']['1_4'], 1.)
        self.assertEqual(far['bbox_short_side_pixels']['p50'], 10.)
        self.assertTrue(any(w['band'] == '80_100m' for w in report['warnings']))
        self.assertEqual(doc['data_list'][0]['scene_group_id'], 'physical-link-A')

    def test_mismatched_mask_cache_rejected(self):
        doc, observation = fixtures()
        for key, value in (('points', 3), ('split', 'train'), ('asset', 'SUV'),
                           ('center_lidar_m', [90.1, 4., 0.])):
            wrong = dict(observation, **{key: value})
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'mismatch'):
                audit.join_infos({'val': doc}, [wrong])

    def test_annotation_and_filtered_counts_are_distinct(self):
        doc, observation = fixtures()
        doc['data_list'][0]['instances'][0]['num_lidar_pts'] = 5
        rows = audit.join_infos({'val': doc}, [observation])
        self.assertEqual(rows[0]['raw_annotation_points'], 5)
        self.assertEqual(rows[0]['native_points'], 2)
        self.assertEqual(rows[0]['points'], 2)

    def test_duplicate_and_extra_cache_gt_rejected(self):
        doc, observation = fixtures()
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            audit.join_infos({'val': doc}, [observation, observation])
        extra = dict(observation, token='extra')
        with self.assertRaisesRegex(ValueError, 'denominator'):
            audit.join_infos({'val': doc}, [observation, extra])

    def test_component_count_is_not_scene_count(self):
        doc, observation = fixtures()
        other = copy.deepcopy(observation)
        other['component_id'] = '8501:Vehicle:0'
        rows = [dict(r, scene_group_id='same-group') for r in (observation, other)]
        summary = audit.summarize(rows)
        self.assertEqual(summary['gt'], 2)
        self.assertEqual(summary['scenes'], 1)
        self.assertEqual(summary['scene_groups'], 1)
        self.assertEqual(summary['assets'], 1)

    def test_unknown_camera_does_not_become_failed_visibility(self):
        _, observation = fixtures()
        row = dict(observation, scene_group_id='group', camera_gate=None,
                   best_visible_mask_pixels=None, best_visible_bbox_extent_px=None)
        summary = audit.summarize([row])
        self.assertEqual(summary['camera_known'], 0)
        self.assertIsNone(summary['camera_gate_fraction_known'])
        self.assertIsNone(summary['mask_pixels'])

    def test_lateral_bins_are_explicit_proxy(self):
        self.assertEqual(audit.lateral_proxy([90, 1.99]), '0_2m')
        self.assertEqual(audit.lateral_proxy([90, -2]), '2_6m')
        self.assertEqual(audit.lateral_proxy([90, 6]), '6plus_m')

    def test_planned_link_imbalance_warns_without_reassigning(self):
        groups = {'val/80_100m': {'planned_parent_scenes': 100}}
        links = {'80_100m': {'val': {'one-link'}}}
        before = copy.deepcopy(links)
        warnings = audit.planned_warnings(groups, links)
        self.assertTrue(any(w['band'] == '80_100m' and w['planned_val_ego_links'] == 1 for w in warnings))
        self.assertEqual(links, before)


if __name__ == '__main__':
    unittest.main()
