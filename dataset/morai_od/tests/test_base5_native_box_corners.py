import copy
import itertools
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_native_box_corners as native
from audit_morai_lidar_gt_visibility import parse_instances


class NativeCornerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.annotation = self.root / 'timestamp_Instance.txt'
        self.corner = self.root / 'timestamp_Instance_8Points.txt'
        self.center = np.asarray([5.1, -1.2, 2.3])
        self.rpy = np.asarray([.02, -.03, .7])
        self.size = np.asarray([1., 2., .5])

    def fixture(self, extent=None, center=None, rpy=None, corners=None, raw_class='Obstacle', uid=8500):
        extent = self.size if extent is None else np.asarray(extent)
        center = self.center if center is None else np.asarray(center)
        rpy = self.rpy if rpy is None else np.asarray(rpy)
        if corners is None:
            signs = np.asarray(list(itertools.product((-1, 1), repeat=3)))
            corners = signs * extent / 2 @ native._rotation(rpy).T + center
        self.annotation.write_text(' '.join([raw_class, *map(str, self.center), *map(str, self.rpy),
                                             *map(str, self.size), '5', '0', '0', '0', str(uid)]) + '\n')
        self.corner.write_text(' '.join([raw_class, *map(str, np.asarray(corners).reshape(-1)),
                                         *map(str, rpy), '5', '0', '0', '0', str(uid)]) + '\n')
        return parse_instances(self.annotation)[0], native.load_matching_corners(self.annotation)[(uid, raw_class, 0)]

    def test_unit_box_is_exact_numeric_noop_with_native_center_rotation_preserved(self):
        box, record = self.fixture()
        before = self.annotation.read_bytes(), self.corner.read_bytes()
        result = native.derive_geometry(box, record)
        self.assertEqual(result['size'], box['size'].tolist())
        self.assertEqual(result['center'], box['center'].tolist())
        self.assertEqual(result['rotation_rpy'], box['rotation_rpy'])
        self.assertFalse(result['evidence']['dimensions_changed'])
        self.assertTrue(result['evidence']['original_matching_size_preserved'])
        self.assertFalse(result['evidence']['scenario_scale_multiplied'])
        self.assertEqual(before, (self.annotation.read_bytes(), self.corner.read_bytes()))

    def test_variant_requires_explicit_call_policy_then_uses_corners_not_factor(self):
        box, record = self.fixture([3., 2.5, 1.5])
        with self.assertRaisesRegex(native.BoxCornerContractError, 'explicit bound variant'):
            native.derive_geometry(box, record)
        result = native.derive_geometry(box, record, allow_dimension_change=True)
        np.testing.assert_allclose(result['size'], [3., 2.5, 1.5], atol=1.e-7)
        self.assertEqual(result['center'], box['center'].tolist())
        self.assertEqual(result['rotation_rpy'], box['rotation_rpy'])
        self.assertTrue(result['evidence']['dimensions_changed'])

    def test_already_scaled_size_is_idempotent_and_never_multiplied_again(self):
        box, record = self.fixture([3., 2.5, 1.5])
        result = native.derive_geometry(box, record, allow_dimension_change=True)
        already = dict(box, size=result['size'])
        second = native.derive_geometry(already, record)
        self.assertEqual(second['size'], result['size'])
        self.assertFalse(second['evidence']['dimensions_changed'])

    def test_exact_companion_name_required_not_nearest_time_or_any_glob(self):
        self.fixture()
        self.corner.rename(self.root / 'other_timestamp_Instance_8Points.txt')
        with self.assertRaises(native.BoxCornerContractError):
            native.load_matching_corners(self.annotation)
        with self.assertRaises(native.BoxCornerContractError):
            native.load_matching_corners(self.root / 'timestamp.txt')

    def test_source_hash_change_after_parse_fails_closed(self):
        box, record = self.fixture()
        self.corner.write_text(self.corner.read_text() + '\n')
        with self.assertRaisesRegex(native.BoxCornerContractError, 'checksum changed'):
            native.derive_geometry(box, record)

    def test_record_tampering_fails_even_if_geometry_would_be_valid(self):
        box, record = self.fixture()
        changed = copy.deepcopy(record)
        changed['corners_lidar_m'] = (2 * (np.asarray(changed['corners_lidar_m']) - self.center) + self.center).tolist()
        with self.assertRaisesRegex(native.BoxCornerContractError, 'record changed'):
            native.derive_geometry(box, changed, allow_dimension_change=True)

    def test_center_mismatch_and_rotation_mismatch_rejected(self):
        for change in (dict(center=self.center + [1., 0., 0.]), dict(rpy=self.rpy + [0., 0., .01])):
            with self.subTest(change=change):
                box, record = self.fixture(**change)
                with self.assertRaises(native.BoxCornerContractError):
                    native.derive_geometry(box, record, allow_dimension_change=True)
        box, record = self.fixture()
        for changed in (dict(box, center=box['center'] + [1., 0., 0.]),
                        dict(box, rotation_rpy=[0., 0., 0.])):
            with self.assertRaises(native.BoxCornerContractError):
                native.derive_geometry(changed, record)

    def test_sheared_or_missing_box_vertices_rejected(self):
        signs = np.asarray(list(itertools.product((-1, 1), repeat=3)))
        original = signs * self.size / 2 @ native._rotation(self.rpy).T + self.center
        changed = original.copy()
        changed[0] += [.1, 0., 0.]
        changed[7] -= [.1, 0., 0.]
        box, record = self.fixture(corners=changed)
        with self.assertRaises(native.BoxCornerContractError):
            native.derive_geometry(box, record)
        changed = original.copy()
        changed[0] = changed[1]
        box, record = self.fixture(corners=changed)
        with self.assertRaises(native.BoxCornerContractError):
            native.derive_geometry(box, record)

    def test_degenerate_box_rejected(self):
        box, record = self.fixture([1., 2., .001])
        with self.assertRaisesRegex(native.BoxCornerContractError, 'Degenerate'):
            native.derive_geometry(box, record, allow_dimension_change=True)

    def test_only_single_obstacle_supported_not_vehicle_or_composite(self):
        for raw_class in ('Vehicle', 'Pedestrian'):
            box, record = self.fixture(raw_class=raw_class)
            with self.assertRaises(native.BoxCornerContractError):
                native.derive_geometry(box, record)
        self.fixture()
        self.annotation.write_text(self.annotation.read_text() * 2)
        self.corner.write_text(self.corner.read_text() * 2)
        records = native.load_matching_corners(self.annotation)
        self.assertEqual(len(records), 2)
        box = parse_instances(self.annotation)[0]
        with self.assertRaises(native.BoxCornerContractError):
            native.derive_geometry(box, records[(8500, 'Obstacle', 0)])

    def test_class_uid_and_component_count_mismatch_between_files_rejected(self):
        for token, value in ((0, 'Vehicle'), (-1, '8501')):
            self.fixture()
            fields = self.corner.read_text().split()
            fields[token] = value
            self.corner.write_text(' '.join(fields) + '\n')
            with self.assertRaises(native.BoxCornerContractError):
                native.load_matching_corners(self.annotation)
        self.fixture()
        self.corner.write_text(self.corner.read_text() * 2)
        with self.assertRaises(native.BoxCornerContractError):
            native.load_matching_corners(self.annotation)

    def test_malformed_nonfinite_and_fractional_uid_rejected(self):
        for path_name, token, replacement in (('corner', 1, 'nan'), ('corner', -1, '8500.5'),
                                               ('annotation', 7, 'inf')):
            self.fixture()
            path = getattr(self, path_name)
            fields = path.read_text().split()
            fields[token] = replacement
            path.write_text(' '.join(fields) + '\n')
            with self.assertRaises(native.BoxCornerContractError):
                native.load_matching_corners(self.annotation)
        self.fixture()
        self.corner.write_text(self.corner.read_text().rstrip() + ' extra\n')
        with self.assertRaises(native.BoxCornerContractError):
            native.load_matching_corners(self.annotation)

    def test_symlink_source_rejected_and_uid_binding_checked(self):
        box, record = self.fixture()
        with self.assertRaises(native.BoxCornerContractError):
            native.derive_geometry(dict(box, unique_id=9999), record)
        actual = self.root / 'actual.txt'
        self.corner.rename(actual)
        self.corner.symlink_to(actual)
        with self.assertRaises(native.BoxCornerContractError):
            native.load_matching_corners(self.annotation)


if __name__ == '__main__':
    unittest.main()
