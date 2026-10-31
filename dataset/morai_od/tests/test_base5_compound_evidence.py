import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report_base5_compound_evidence as report


class CompoundEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'capture'
        self.root.mkdir()

    def fixture(self, raw_classes=('Pedestrian', 'Pedestrian'), camera_records=None,
                points=None, centers=None, missing_camera=None, rpy=None):
        if points is None:
            points = [[4., 0., 0., 1.], [4., .4, 0., 1.], [4., .7, 0., 1.], [0., 0., 0., 0.]]
        points = np.asarray(points, dtype=np.float32).reshape(-1, 4)
        centers = centers or [[4., 0., 0.], [4., .8, 0.]]
        rpy = rpy or [0., 0., 0.]
        counts = {}
        components = []
        for index, raw_class in enumerate(raw_classes):
            ordinal = counts.get(raw_class, 0)
            counts[raw_class] = ordinal + 1
            components.append(dict(parent_uid=8500, component_id=f'8500:{raw_class}:{ordinal}',
                asset='BabyStroller2', raw_class=raw_class,
                training_class='pedestrian' if raw_class == 'Pedestrian' else 'obstacle',
                center_lidar_m=centers[index], size_m=[1., 1., 2.], rotation_rpy=rpy,
                camera_component_ambiguous=raw_classes.count(raw_class) > 1,
                range_xy_m=4., positive_lidar1_camera0=False, ignore_reason='no_visible_sensor_evidence'))
        valid_points = points[(points[:, :3] != 0).any(axis=1)]
        memberships = report.point_memberships(valid_points, components)
        for c, membership in zip(components, memberships):
            c['lidar_points'] = int(membership.sum())
            c['positive_lidar1_camera0'] = bool(c['lidar_points'])
        metadata = dict(scene_id='test', spec=dict(family='pedestrian', mission=False), runtime_actors=[
            dict(unique_id=8500, name='BabyStroller2', data_id=40200012, source_list='objectList', motion='static')])
        audit = dict(components=components, accepted_candidate=True, review_reasons=[],
                     ground_support=dict(actors=[dict(uid=8500, verified=True, gap_m=.01)]))
        (self.root / 'metadata.json').write_text(json.dumps(metadata))
        (self.root / 'audit.json').write_text(json.dumps(audit))
        lidar = self.root / 'LIDAR_5'
        lidar.mkdir()
        points.tofile(lidar / '1_Intensity.bin')
        lines = []
        for c in components:
            values = c['center_lidar_m'] + c['rotation_rpy'] + c['size_m'] + [0.] * 4
            lines.append(c['raw_class'] + ' ' + ' '.join(map(str, values)) + ' 8500')
        (lidar / '1_Instance.txt').write_text('\n'.join(lines))
        camera_records = camera_records if camera_records is not None else [
            ('Pedestrian', 8500, (11, 12, 13, 255), (600, 400, 640, 440))]
        for camera, (_, width, height) in report.CAMERAS.items():
            if camera == missing_camera:
                continue
            folder = self.root / camera
            folder.mkdir()
            pixels = np.zeros((height, width, 4), dtype=np.uint8)
            camera_lines = []
            if camera == 'CAMERA_1':
                for raw_class, ident, colour, rect in camera_records:
                    x1, y1, x2, y2 = rect
                    pixels[max(0, y1):min(height, y2), max(0, x1):min(width, x2)] = colour
                    camera_lines.append(f'{raw_class} {x1} {y1} {x2} {y2} {ident} {list(colour)}')
            Image.fromarray(pixels, mode='RGBA').save(folder / '1_Instance.png')
            (folder / '1_BBox2D.txt').write_text('\n'.join(camera_lines))
        return components

    def test_native_parent_encoding_is_not_a_prefix_or_lidar_ordinal(self):
        self.assertEqual(report.camera_parent(-850002), 8500)
        self.assertEqual(report.camera_parent(8500), 8500)
        self.assertNotEqual(report.camera_parent(-850010), 85001)
        self.assertNotEqual(report.camera_parent(85001), 8500)

    def test_same_class_correspondence_is_unknown_despite_visible_parent_pixels(self):
        self.fixture()
        result = report.capture_report(self.root)
        parent = result['parents'][0]
        group = parent['camera_class_groups'][0]
        self.assertEqual(group['parent_class_camera_observation_state'], 'supported')
        self.assertEqual(group['component_camera_support_state'], 'unknown_component_correspondence')
        self.assertIsNone(group['component_camera_gate'])
        self.assertEqual(len(group['observations']), 1)
        for c in parent['components']:
            self.assertIsNone(c['camera_gate512_component'])
            self.assertTrue(c['saved_camera_ambiguous'])

    def test_zero_lidar_unknown_is_not_counted_as_invisible(self):
        self.fixture(points=[[10., 10., 10., 1.]])
        result = report.capture_report(self.root)
        self.assertTrue(all(c['diagnostic_sensor_state'] == 'camera_correspondence_unknown_zero_lidar'
                            for c in result['parents'][0]['components']))
        counts = report.summarize([result])['counts']
        self.assertEqual(counts['camera_correspondence_unknown_zero_lidar'], 2)
        self.assertNotIn('no_observed_sensor_evidence', counts)

    def test_unique_raw_class_matches_camera_without_sharing_other_class_pixels(self):
        self.fixture(raw_classes=('Pedestrian', 'Obstacle'), points=[[10., 10., 10., 1.]])
        cs = report.capture_report(self.root)['parents'][0]['components']
        self.assertEqual(cs[0]['diagnostic_sensor_state'], 'camera_supported_zero_lidar')
        self.assertTrue(cs[0]['camera_gate512_component'])
        self.assertEqual(cs[1]['diagnostic_sensor_state'], 'no_observed_sensor_evidence')
        self.assertFalse(cs[1]['camera_gate512_component'])

    def test_missing_camera_is_incomplete_not_invisible(self):
        self.fixture(raw_classes=('Pedestrian', 'Obstacle'), camera_records=[],
                     points=[[10., 10., 10., 1.]], missing_camera='CAMERA_4')
        cs = report.capture_report(self.root)['parents'][0]['components']
        self.assertTrue(all(c['diagnostic_sensor_state'] == 'camera_incomplete_zero_lidar' for c in cs))
        self.assertTrue(all(c['camera_gate512_component'] is None for c in cs))

    def test_visible_tiny_mask_is_below_gate_not_no_evidence(self):
        self.fixture(raw_classes=('Pedestrian', 'Obstacle'), points=[[10., 10., 10., 1.]],
            camera_records=[('Pedestrian', 8500, (11, 12, 13, 255), (600, 400, 601, 401))])
        cs = report.capture_report(self.root)['parents'][0]['components']
        self.assertEqual(cs[0]['diagnostic_sensor_state'], 'camera_pixels_below_gate_zero_lidar')
        self.assertFalse(cs[0]['camera_gate512_component'])

    def test_mask_above_crop_is_not_physical_invisibility(self):
        self.fixture(raw_classes=('Pedestrian', 'Obstacle'), points=[[10., 10., 10., 1.]],
            camera_records=[('Pedestrian', 8500, (11, 12, 13, 255), (600, 5, 640, 45))])
        parent = report.capture_report(self.root)['parents'][0]
        c = parent['components'][0]
        self.assertEqual(c['camera_support_state'], 'below_gate')
        obs = next(x for x in parent['camera_class_groups'] if x['raw_class'] == 'Pedestrian')['observations'][0]
        self.assertGreater(obs['native_instance_pixels'], 0)
        self.assertEqual(obs['model_512x1056_pixels'], 0)

    def test_sibling_point_union_prevents_double_count_and_overlap_is_not_duplicate(self):
        self.fixture()
        parent = report.capture_report(self.root)['parents'][0]
        self.assertEqual(parent['sum_component_bbox_points'], 4)
        self.assertEqual(parent['unique_parent_bbox_returns'], 3)
        self.assertEqual(parent['shared_bbox_returns'], 1)
        self.assertEqual([x['sibling_exclusive_bbox_points'] for x in parent['components']], [1, 1])
        self.assertFalse(parent['same_parent_pairs'][0]['same_class_identical_geometry_review'])

    def test_same_class_identical_geometry_is_review_only_not_merged(self):
        self.fixture(centers=[[4., 0., 0.], [4., 0., 0.]])
        parent = report.capture_report(self.root)['parents'][0]
        self.assertTrue(parent['same_parent_pairs'][0]['same_class_identical_geometry_review'])
        self.assertEqual(len(parent['components']), 2)

    def test_different_class_nested_box_is_not_duplicate_candidate(self):
        self.fixture(raw_classes=('Pedestrian', 'Obstacle'), centers=[[4., 0., 0.], [4., 0., 0.]])
        parent = report.capture_report(self.root)['parents'][0]
        self.assertFalse(parent['same_parent_pairs'][0]['same_class_identical_geometry_review'])

    def test_full_rpy_membership_rotates_vertical_axis(self):
        c = dict(center_lidar_m=[4., 0., 0.], size_m=[.2, .2, 2.],
                 rotation_rpy=[0., np.pi/2, 0.])
        points = np.array([[4.8, 0., 0.], [4., 0., .8]])
        np.testing.assert_array_equal(report.point_memberships(points, [c]), [[True, False]])

    def test_source_files_and_labels_are_unchanged(self):
        self.fixture()
        before = {str(p): report.sha256(p) for p in self.root.rglob('*') if p.is_file()}
        result = report.capture_report(self.root)
        after = {str(p): report.sha256(p) for p in self.root.rglob('*') if p.is_file()}
        self.assertEqual(before, after)
        self.assertEqual(result['zero_xyz_returns_excluded'], 1)
        self.assertEqual(result['parents'][0]['saved_ground_support'], [dict(uid=8500, verified=True, gap_m=.01)])

    def test_native_audit_geometry_disagreement_fails_closed(self):
        self.fixture()
        audit = json.loads((self.root / 'audit.json').read_text())
        audit['components'][0]['center_lidar_m'][0] += .1
        (self.root / 'audit.json').write_text(json.dumps(audit))
        with self.assertRaisesRegex(ValueError, 'geometry mismatch'):
            report.capture_report(self.root)

    def test_saved_point_count_disagreement_fails_closed(self):
        self.fixture()
        audit = json.loads((self.root / 'audit.json').read_text())
        audit['components'][0]['lidar_points'] += 1
        (self.root / 'audit.json').write_text(json.dumps(audit))
        with self.assertRaisesRegex(ValueError, 'count disagree'):
            report.capture_report(self.root)

    def test_nonfinite_raw_points_fail_closed(self):
        self.fixture()
        np.array([[np.nan, 0., 0., 1.]], dtype=np.float32).tofile(self.root / 'LIDAR_5/1_Intensity.bin')
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            report.capture_report(self.root)

    def test_camera_child_suffix_does_not_resolve_stroller_person_vs_baby(self):
        self.fixture(camera_records=[('Pedestrian', -850002, (11, 12, 13, 255), (600, 400, 640, 440))])
        parent = report.capture_report(self.root)['parents'][0]
        self.assertEqual(parent['camera_class_groups'][0]['observations'][0]['native_child_suffix'], 2)
        self.assertTrue(all(c['camera_gate512_component'] is None for c in parent['components']))

    def test_cli_deduplicates_same_capture_from_two_ledgers(self):
        self.fixture()
        ledger = self.root.parent / 'ledger.jsonl'
        ledger.write_text(json.dumps(dict(capture_root=str(self.root), status='accepted_candidate')) + '\n')
        output = self.root.parent / 'report.json'
        argv = ['report', '--ledger', str(ledger), '--ledger', str(ledger), '--output', str(output)]
        with patch.object(sys, 'argv', argv), contextlib.redirect_stdout(io.StringIO()):
            report.main()
        result = json.loads(output.read_text())
        self.assertEqual(result['counts']['captures'], 1)
        self.assertEqual(result['counts']['compound_parents'], 1)
        self.assertTrue(result['labels_and_acceptance_unchanged'])

    def test_cli_does_not_write_inside_native_capture(self):
        self.fixture()
        argv = ['report', '--capture-root', str(self.root), '--output', str(self.root / 'report.json')]
        with patch.object(sys, 'argv', argv), self.assertRaisesRegex(ValueError, 'inside a source capture'):
            report.main()
        self.assertFalse((self.root / 'report.json').exists())


if __name__ == '__main__':
    unittest.main()
