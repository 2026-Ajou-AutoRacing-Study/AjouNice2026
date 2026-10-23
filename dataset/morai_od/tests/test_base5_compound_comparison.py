import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import compare_base5_compound_diagnostic as compare


def fixture(root='baseline', points=4, exclusive=0):
    components = [dict(component_id='1:Pedestrian:0', native_raw_class='Pedestrian',
        size_m=[.4, .3, .6], range_xy_m=6., bbox_point_count=points,
        sibling_exclusive_bbox_points=exclusive, shared_sibling_bbox_points=points-exclusive,
        camera_support_state='unknown_component_correspondence',
        diagnostic_sensor_state='lidar_bbox_supported_camera_unknown' if points else
            'camera_correspondence_unknown_zero_lidar', saved_positive=bool(points))]
    return dict(diagnostic_only=True, labels_and_acceptance_unchanged=True, profile={'h': 512},
        counts={'captures': 1}, captures=[dict(capture_root=root, scene_id=root,
            parents=[dict(parent_uid=1, asset='BabyStroller1', components=components,
                same_parent_pairs=[dict(component_ids=['1:Pedestrian:0', '1:Obstacle:0'],
                    shared_bbox_point_count=points, same_class_identical_geometry_review=False)])])])


class CompoundComparisonTest(unittest.TestCase):
    def test_dataset_and_targeted_diagnostic_are_explicitly_distinct(self):
        result = compare.compare(fixture(), fixture('diagnostic'), {}, {})
        self.assertEqual(result['baseline']['source_type'], 'dataset_composition')
        self.assertEqual(result['diagnostic']['source_type'], 'targeted_static_diagnostic')
        self.assertEqual(result['baseline']['compound_capture_fraction'], 1/5000)
        self.assertNotIn('compound_capture_fraction', result['diagnostic'])

    def test_zero_exclusive_does_not_turn_into_duplicate_geometry(self):
        result = compare.summarize_report(fixture(), {}, 'test')
        self.assertEqual(result['observations']['zero_exclusive_among_lidar_positive'], 1)
        self.assertEqual(result['identical_geometry_review_pairs'], 0)
        self.assertTrue(result['source_report_counts'])

    def test_small_leaf_is_geometry_stratum_not_native_child_mapping(self):
        result = compare.summarize_report(fixture(), {}, 'test')
        self.assertIn('stroller_small_pedestrian_leaf', result['by_component_stratum'])
        self.assertEqual(result['observations']['camera_correspondence_unknown'], 1)

    def test_zero_points_are_not_divided_or_fully_shared_positive(self):
        result = compare.summarize_report(fixture(points=0), {}, 'test')['observations']
        self.assertEqual(result['lidar_positive'], 0)
        self.assertEqual(result['zero_exclusive_among_lidar_positive'], 0)
        self.assertEqual(result['shared_fraction_among_lidar_positive'], {'n': 0})
        self.assertEqual(result['camera_unknown_zero_lidar'], 1)

    def test_body_overlap_requires_a_real_body_component(self):
        source = fixture()
        self.assertFalse(compare.records(source, {})[0]['fully_shared_with_body'])
        parent = source['captures'][0]['parents'][0]
        body = copy.deepcopy(parent['components'][0])
        body.update(component_id='1:Obstacle:0', native_raw_class='Obstacle', size_m=[1., 1., 1.])
        parent['components'].append(body)
        self.assertTrue(compare.records(source, {})[0]['fully_shared_with_body'])

    def test_observer_view_is_parent_axis_not_ego_bearing(self):
        metadata = dict(ego=dict(x=10., y=0.), runtime_actors=[dict(unique_id=1,
            geometric_center_estimate=[0., 0., 0.], placement_yaw_deg=0.)])
        self.assertEqual(compare.parent_view(metadata, 1)['view'], 'front')
        metadata['runtime_actors'][0]['placement_yaw_deg'] = 180.
        self.assertEqual(compare.parent_view(metadata, 1)['view'], 'rear')
        metadata['runtime_actors'][0]['placement_yaw_deg'] = 90.
        self.assertEqual(compare.parent_view(metadata, 1)['view'], 'side')

    def test_range_bin_boundaries(self):
        self.assertEqual(compare.distance_band(9.99), '0_10m')
        self.assertEqual(compare.distance_band(10.), '10_20m')
        self.assertEqual(compare.distance_band(None), 'unknown')
        with self.assertRaises(ValueError):
            compare.distance_band(-1)

    def test_partition_disagreement_fails_closed(self):
        source = fixture()
        source['captures'][0]['parents'][0]['components'][0]['shared_sibling_bbox_points'] = 3
        with self.assertRaisesRegex(ValueError, 'partition'):
            compare.records(source, {})

    def test_overlap_profiles_and_duplicate_sources_fail(self):
        with self.assertRaisesRegex(ValueError, 'overlap'):
            compare.compare(fixture(), fixture(), {}, {})
        diagnostic = fixture('diagnostic')
        diagnostic['profile']['h'] = 320
        with self.assertRaisesRegex(ValueError, 'profiles'):
            compare.compare(fixture(), diagnostic, {}, {})
        source = fixture()
        source['captures'] *= 2
        with self.assertRaisesRegex(ValueError, 'Duplicate captures'):
            compare.summarize_report(source, {}, 'test')

    def test_comparison_does_not_modify_labels_or_source_reports(self):
        baseline, diagnostic = fixture(), fixture('diagnostic')
        before = copy.deepcopy((baseline, diagnostic))
        compare.compare(baseline, diagnostic, {}, {})
        self.assertEqual((baseline, diagnostic), before)

    def test_unknown_or_invalid_evidence_is_not_treated_as_valid_source(self):
        source = fixture()
        source['labels_and_acceptance_unchanged'] = False
        with self.assertRaisesRegex(ValueError, 'read-only'):
            compare.summarize_report(source, {}, 'test')
        with self.assertRaisesRegex(ValueError, 'Dataset capture count'):
            compare.summarize_report(fixture(), {}, 'test', 0)


if __name__ == '__main__':
    unittest.main()
