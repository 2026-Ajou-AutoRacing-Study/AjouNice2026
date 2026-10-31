"""CPU-only native far-secondary QA and current-input crop contracts.

The placement-preservation verifier is isolated here; its own tests must cover
the immutable scene contract. These tests exercise native evidence, not learned
detector accuracy, live safety, measured ground contact, or capture acceptance.
"""
import ast
import copy
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from PIL import Image


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import base5_far_vehicle_native as native
from base5_capture_audit import camera_evidence
from analyze_base5_crop_candidates import geometry


def metadata(role='front'):
    return dict(spec=dict(secondary_far_vehicle=dict(
        target_ordinal=1, target_uid=8501, distance_bounds_m=[80., 90.], role=role)),
        runtime_actors=[dict(unique_id=8500), dict(unique_id=8501)])


def component(**overrides):
    value = dict(parent_uid=8501, component_id='8501:Sedan:0',
        raw_class='Sedan', training_class='vehicle',
        center_ego_m=[85., 0., -.5], range_ego_xy_m=85.,
        center_lidar_m=[84., 0., -.5],
        size_m=[4.5, 2., 1.5], lidar_points=1, cameras=[],
        camera_component_ambiguous=False)
    value.update(overrides)
    return value


def camera(pixels=16, extent=(4., 4.)):
    return dict(crop_512x1056_instance_pixels=pixels,
                model512_extent_px=list(extent))


def runtime_resize_crop(camera_meta):
    """Execute the actual pure runtime geometry method without MMDet imports."""
    path = ROOT / 'docker/bevfusion/app/online_preprocessor.py'
    tree = ast.parse(path.read_text())
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef)
               and node.name == 'Base3OnlinePreprocessor')
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef)
                  and node.name == '_resize_crop')
    namespace = {}
    module = ast.Module(body=[copy.deepcopy(method)], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(path), 'exec'), namespace)
    preprocessor = types.SimpleNamespace(
        final_dim=(512, 1056), resize=.825, reference_width=1280.)
    return namespace['_resize_crop'](preprocessor, camera_meta)


class FarVehicleNativeReviewTest(unittest.TestCase):
    def setUp(self):
        verifier = types.ModuleType('base5_secondary_far_vehicle')
        self.verify_preserved = Mock()
        verifier.verify_preserved = self.verify_preserved
        self.modules = patch.dict(sys.modules,
                                  base5_secondary_far_vehicle=verifier)
        self.modules.start()
        self.addCleanup(self.modules.stop)

    def review(self, components, meta=None):
        return native.review(meta if meta is not None else metadata(), components)

    def test_population_uses512_only_for_allocated_target_without_editing_raw(self):
        primary = component(parent_uid=8500, positive_lidar1_camera0=False, lidar_points=0)
        far = component(lidar_points=0, cameras=[camera()], positive_lidar1_camera0=False)
        rows = [primary, far]
        before = copy.deepcopy(rows)
        result = native.population_components(metadata(), rows)
        self.assertFalse(result[0]['positive_lidar1_camera0'])
        self.assertTrue(result[1]['positive_lidar1_camera0'])
        self.assertEqual(rows, before)
        self.assertIs(native.population_components({}, rows), rows)

    def test_population_camera_gate_does_not_accept_tiny_far_mask(self):
        far = component(lidar_points=0, cameras=[camera(15)], positive_lidar1_camera0=False)
        result = native.population_components(metadata(), [far])
        self.assertFalse(result[0]['positive_lidar1_camera0'])

    def test_far_target_outside_actual_training_center_roi_is_not_approved(self):
        for center in ([100., 0., 0.], [-100.01, 0., 0.], [90., 30., 0.],
                       [90., 0., 2.], [90., 0., -3.01], [90., 0., math.nan]):
            with self.subTest(center=center):
                reasons, _ = self.review([component(center_lidar_m=center)])
                self.assertIn('far_vehicle_outside_model_center_roi', reasons)

    def test_nonplanned_is_exact_noop_without_importing_verifier(self):
        with patch.dict(sys.modules, base5_secondary_far_vehicle=None):
            self.assertEqual(native.review({}, []), ([], None))
        self.verify_preserved.assert_not_called()

    def test_point_one_is_accepted_for_all_native_vehicle_raw_classes(self):
        for raw_class in ('Vehicle', 'Sedan', 'SUV', 'Truck', 'Bus', 'MPV',
                          'Wagon', 'Car'):
            with self.subTest(raw_class=raw_class):
                reasons, proof = self.review([component(raw_class=raw_class)])
                self.assertEqual(reasons, [])
                self.assertTrue(proof['components'][0]['sensor_supported'])
                self.assertFalse(proof['components'][0]['camera512_gate'])

    def test_camera_only_accepted_at_exact_threshold(self):
        reasons, proof = self.review([component(
            lidar_points=0, cameras=[camera()])])
        self.assertEqual(reasons, [])
        self.assertEqual(proof['target_uid'], 8501)
        self.assertTrue(proof['components'][0]['camera512_gate'])

    def test_no_current_input_evidence_rejected(self):
        for cameras in ([], [camera(15)], [camera(16, (3.99, 10))],
                        [dict(crop_320x896_instance_pixels=100,
                              model_extent_px=[10, 10])]):
            with self.subTest(cameras=cameras):
                reasons, proof = self.review([component(
                    lidar_points=0, cameras=cameras)])
                self.assertIn('far_vehicle_no_current_input_sensor_evidence', reasons)
                self.assertFalse(proof['components'][0]['sensor_supported'])

    def test_missing_or_wrong_parent_or_semantic_box_rejected(self):
        for components in ([], [component(parent_uid=8500)],
                           [component(training_class='pedestrian')]):
            with self.subTest(components=components):
                reasons, _ = self.review(components)
                self.assertIn('far_vehicle_missing_or_ambiguous_native_vehicle_box', reasons)

    def test_multiple_vehicle_components_rejected_even_with_sensor_support(self):
        rows = [component(), component(component_id='8501:Vehicle:1')]
        reasons, proof = self.review(rows)
        self.assertIn('far_vehicle_missing_or_ambiguous_native_vehicle_box', reasons)
        self.assertEqual(len(proof['components']), 2)

    def test_wrong_distance_rejected_and_half_open_tolerance_preserved(self):
        for distance, accepted in ((79.499, False), (79.5, True),
                                   (90.499, True), (90.5, False)):
            with self.subTest(distance=distance):
                reasons, _ = self.review([component(
                    center_ego_m=[distance, 0., -.5], range_ego_xy_m=distance)])
                mismatch = 'far_vehicle_actual_distance_direction_or_roi_mismatch'
                self.assertEqual(mismatch not in reasons, accepted)

    def test_rear_direction_is_checked_independently_of_positive_range(self):
        reasons, _ = self.review([component(
            center_ego_m=[-85., 0., -.5])], metadata(role='rear'))
        self.assertEqual(reasons, [])
        for role, x in (('rear', 85.), ('front', -85.)):
            with self.subTest(role=role, x=x):
                reasons, _ = self.review([component(
                    center_ego_m=[x, 0., -.5])], metadata(role=role))
                self.assertIn('far_vehicle_actual_distance_direction_or_roi_mismatch', reasons)

    def test_explicit_tail_window_cannot_be_filled_by_lower_end_of_band(self):
        meta=metadata()
        meta['spec']['secondary_far_vehicle'].update(distance_bounds_m=[95.,99.],
            sampling_bounds_m=[97.,99.])
        for distance, accepted in ((95.5,False),(96.499,False),(96.5,True),(98.8,True)):
            with self.subTest(distance=distance):
                reasons,proof=self.review([component(center_ego_m=[distance,0.,-.5],
                    range_ego_xy_m=distance,center_lidar_m=[distance-1.,0.,-.5])],meta)
                self.assertEqual(not reasons,accepted)
                self.assertEqual(proof['sampling_bounds_m'],[97.,99.])

    def test_roi_lateral_guard_does_not_accept_far_side_target(self):
        reasons, _ = self.review([component(center_ego_m=[80., 31., -.5])])
        self.assertIn('far_vehicle_actual_distance_direction_or_roi_mismatch', reasons)

    def test_nonfinite_and_missing_calibrated_geometry_rejected(self):
        cases = [dict(center_ego_m=None), dict(range_ego_xy_m=None),
                 dict(center_ego_m=[85., math.nan, 0.]),
                 dict(center_ego_m=[85., 0., math.inf]),
                 dict(range_ego_xy_m=math.nan),
                 dict(range_ego_xy_m=math.inf)]
        for change in cases:
            with self.subTest(change=change):
                reasons, _ = self.review([component(**change)])
                self.assertIn('far_vehicle_missing_calibrated_native_geometry', reasons)

    def test_center_requires_exactly_three_dimensions(self):
        for center in ([], [85.], [85., 0.], [85., 0., 0., 1.]):
            with self.subTest(center=center):
                reasons, _ = self.review([component(center_ego_m=center)])
                self.assertIn('far_vehicle_missing_calibrated_native_geometry', reasons)

    def test_primary_visibility_cannot_substitute_for_allocated_far_target(self):
        reasons, _ = self.review([
            component(parent_uid=8500, lidar_points=2000),
            component(lidar_points=0)])
        self.assertIn('far_vehicle_no_current_input_sensor_evidence', reasons)

    def test_metadata_and_components_are_not_modified(self):
        meta, rows = metadata(), [component()]
        before = copy.deepcopy((meta, rows))
        native.review(meta, rows)
        self.assertEqual((meta, rows), before)
        self.verify_preserved.assert_called_once_with(meta, meta)

    def test_preservation_error_is_not_silently_accepted(self):
        self.verify_preserved.side_effect = ValueError('frozen actor mismatch')
        with self.assertRaisesRegex(ValueError, 'frozen actor mismatch'):
            self.review([component()])


class FarVehicleCamera512Test(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def fixture(self, *, width=64, height=48, raw_class='Sedan',
                annotation_uid=8501, rect=None):
        folder = self.root / 'CAMERA_1'
        folder.mkdir()
        rgba = np.zeros((height, width, 4), dtype=np.uint8)
        rgba[:, :, 3] = 255
        colour = [7, 11, 13, 255]
        # Small source image; the same mixed-resolution runtime geometry still
        # produces an actual 512x1056 mask and exercises non-integer resizing.
        y, x = height // 2, width // 3
        rgba[y:y + 3, x:x + 2] = colour
        Image.fromarray(rgba).save(folder / 'frame_Instance.png')
        if rect is None:
            rect = [x, y, x + 2, y + 3]
        annotation = '{} {} {} {} {} {} {}\n'.format(
            raw_class, *rect, annotation_uid, colour)
        (folder / 'frame_BBox2D.txt').write_text(annotation)
        mask = (rgba == np.asarray(colour)).all(axis=2).astype(np.uint8)
        return mask, rect

    def expected(self, mask, rect):
        height, width = mask.shape
        resize_wh, crop = runtime_resize_crop(dict(
            width=width, height=height, frame_id='synthetic'))
        left, top, right, bottom = crop
        resized = cv2.resize(mask, resize_wh, interpolation=cv2.INTER_NEAREST)
        scale = .825 * 1280 / width
        x1, y1, x2, y2 = rect
        x1, x2 = max(0., x1), min(float(width), x2)
        y1, y2 = max(0., y1), min(float(height), y2)
        extent = [max(0., min(right, x2 * scale) - max(left, x1 * scale)),
                  max(0., min(bottom, y2 * scale) - max(top, y1 * scale))]
        return int(resized[top:bottom, left:right].sum()), extent

    def test_camera512_exact_synthetic_mask_matches_runtime_crop(self):
        mask, rect = self.fixture()
        expected_pixels, expected_extent = self.expected(mask, rect)
        row, = camera_evidence(self.root, 8501, 'Sedan', model512=True)
        self.assertEqual(row['crop_512x1056_instance_pixels'], expected_pixels)
        np.testing.assert_allclose(row['model512_extent_px'], expected_extent,
                                   rtol=0, atol=1e-10)

    def test_camera512_uses_runtime_int_resize_not_round(self):
        mask, rect = self.fixture(height=51)
        expected_pixels, expected_extent = self.expected(mask, rect)
        row, = camera_evidence(self.root, 8501, 'Sedan', model512=True)
        self.assertEqual(row['crop_512x1056_instance_pixels'], expected_pixels)
        np.testing.assert_allclose(row['model512_extent_px'], expected_extent,
                                   rtol=0, atol=1e-10)

    def test_model_bbox_extent_clips_to_sensor_and_model_crop(self):
        mask, rect = self.fixture(rect=[-3., -4., 80., 60.])
        _, expected_extent = self.expected(mask, rect)
        row, = camera_evidence(self.root, 8501, 'Sedan', model512=True)
        np.testing.assert_allclose(row['model512_extent_px'], expected_extent,
                                   rtol=0, atol=1e-10)
        self.assertLessEqual(row['model512_extent_px'][0], 1056.)
        self.assertLessEqual(row['model512_extent_px'][1], 512.)

    def test_historical320_evidence_is_unchanged_when512_enabled(self):
        self.fixture()
        old, = camera_evidence(self.root, 8501, 'Sedan', model512=False)
        new, = camera_evidence(self.root, 8501, 'Sedan', model512=True)
        self.assertNotIn('crop_512x1056_instance_pixels', old)
        self.assertEqual(old, {k: v for k, v in new.items() if k in old})

    def test_child_instance_id_retains_parent_mapping(self):
        self.fixture(annotation_uid=-(8501 * 100 + 2))
        row, = camera_evidence(self.root, 8501, 'Sedan', model512=True)
        self.assertEqual(row['camera_instance_id'], -(8501 * 100 + 2))
        self.assertEqual(camera_evidence(self.root, 8500, 'Sedan', model512=True), [])
        self.assertEqual(camera_evidence(self.root, 8501, 'SUV', model512=True), [])

    def test_nominal_four_camera_geometry_matches_existing_offline_audit(self):
        profile = dict(h=512, w=1056, side_start=None)
        for name, width, height in (('CAMERA_1', 1280, 720),
                                    ('CAMERA_2', 640, 480),
                                    ('CAMERA_3', 640, 480),
                                    ('CAMERA_4', 1280, 720)):
            with self.subTest(camera=name):
                expected = geometry(name, profile)
                dims, crop = runtime_resize_crop(dict(
                    width=width, height=height, frame_id=name))
                self.assertEqual(list(dims), expected['resize_wh'])
                self.assertEqual(list(crop), expected['crop_xyxy'])


if __name__ == '__main__':
    unittest.main()
