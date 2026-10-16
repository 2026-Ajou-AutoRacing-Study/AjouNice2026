"""Opt-in measured-corner audit checks, preserving legacy scale-1 validation."""
import copy
import itertools
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_morai_lidar_gt_visibility import parse_instances
from base5_capture_audit import rotation_matrix
from base5_native_box_corners import load_matching_corners, derive_geometry
from report_base5_compound_evidence import _validate_native_geometry


class NativeCornerReportValidationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.lidar = self.root / 'LIDAR_5'
        self.lidar.mkdir()

    def fixture(self, *, expanded=True):
        center, rpy, summary = np.array([10., 0., -1.]), [.1, .2, .3], [2., 1., .5]
        size = np.array([3., 2., 1.]) if expanded else np.array(summary)
        signs = np.array(list(itertools.product((-1., 1.), repeat=3)))
        corners = signs * size / 2 @ rotation_matrix(rpy).T + center
        annotation = self.lidar / 'stamp_Instance.txt'
        annotation.write_text('Obstacle ' + ' '.join(map(str, [*center, *rpy, *summary, 0, 0, 0, 0])) + ' 8500\n')
        companion = self.lidar / 'stamp_Instance_8Points.txt'
        companion.write_text('Obstacle ' + ' '.join(map(str, [*corners.reshape(-1), *rpy, 0, 0, 0, 0, 8500])) + '\n')
        box = parse_instances(annotation)[0]
        derived = derive_geometry(box, load_matching_corners(annotation)[(8500, 'Obstacle', 0)], allow_dimension_change=True)
        component = dict(component_id='8500:Obstacle:0', parent_uid=8500, raw_class='Obstacle',
            center_lidar_m=derived['center'], rotation_rpy=derived['rotation_rpy'], size_m=derived['size'],
            native_corner_geometry=derived['evidence'], native_summary_size_m=box['size'].tolist())
        return annotation, companion, component

    def test_measured_expanded_size_and_native_source_are_accepted_without_scale_multiplication(self):
        annotation, _, component = self.fixture()
        self.assertEqual(_validate_native_geometry(self.root, [component]), annotation)
        np.testing.assert_allclose(component['size_m'], [3., 2., 1.], atol=1e-7)

    def test_legacy_summary_validation_stays_unchanged_without_markers(self):
        _, _, component = self.fixture()
        component['size_m'] = component.pop('native_summary_size_m')
        component.pop('native_corner_geometry')
        _validate_native_geometry(self.root, [component])

    def test_partial_or_forged_provenance_is_fatal(self):
        _, _, component = self.fixture()
        for changed in (dict(component, native_corner_geometry={}),
                        {k:v for k, v in component.items() if k != 'native_summary_size_m'},
                        dict(component, native_summary_size_m=[9., 9., 9.])):
            with self.subTest(changed=changed), self.assertRaises((ValueError, RuntimeError)):
                _validate_native_geometry(self.root, [changed])

    def test_wrong_corrected_size_does_not_pass_on_source_hash_alone(self):
        _, _, component = self.fixture()
        changed = copy.deepcopy(component)
        changed['size_m'][0] *= 2.
        with self.assertRaisesRegex(ValueError, 'geometry mismatch'):
            _validate_native_geometry(self.root, [changed])

    def test_changed_corner_raw_or_uid_invalidates_saved_evidence(self):
        _, companion, component = self.fixture()
        companion.write_text(companion.read_text().replace('8500', '8501'))
        with self.assertRaises((ValueError, RuntimeError)):
            _validate_native_geometry(self.root, [component])

    def test_scale1_native_geometry_is_exact_noop(self):
        _, _, component = self.fixture(expanded=False)
        self.assertFalse(component['native_corner_geometry']['dimensions_changed'])
        self.assertTrue(component['native_corner_geometry']['original_matching_size_preserved'])
        self.assertEqual(component['size_m'], component['native_summary_size_m'])
        _validate_native_geometry(self.root, [component])


if __name__ == '__main__':
    unittest.main()
