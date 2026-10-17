"""CPU qualification regression; temporary synthetic files only, no live nodes."""
import copy
import hashlib
import json
from pathlib import Path
import pickle
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import qualify_base5_training_inputs as q
import qualify_base5_training_loader_cpu as loader
from base5_capture_audit import camera_evidence
from dynamic_capture_metadata import normalized_sensor_calibration


class QualificationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value) + '\n')
        return path

    def component(self, ident='8500:Obstacle:0', **kwargs):
        result = dict(component_id=ident, parent_uid=8500, asset='WoodBox',
            data_id=40100002, raw_class='Obstacle', training_class='obstacle',
            center_lidar_m=[10., 0., -1.], size_m=[1., 1., 1.],
            rotation_rpy=[.1, .2, .3], lidar_points=1, range_xy_m=10.,
            camera_component_ambiguous=False, cameras=[], positive_lidar1_camera0=True)
        result.update(kwargs)
        return result

    def instance(self, component):
        return dict(component_id=component['component_id'], parent_uid=component['parent_uid'],
            asset=component['asset'], bbox_3d=q.component_bbox(component),
            bbox_label_3d=q.CLASSES.index(component['training_class']),
            num_lidar_pts=component['lidar_points'], bbox_3d_isvalid=True, instance_id=0)

    def object_fixture(self, components):
        root = self.root / 'native'
        meta = dict(split='train', scene_group_id='group',
            spec=dict(family='obstacle'), runtime_actors=[dict(unique_id=8500)])
        self.write('native/metadata.json', meta)
        self.write('native/audit.json', dict(components=components))
        row = dict(id='slot', capture_root=str(root))
        support = [dict(component_id=c['component_id'],
            camera_gate512=c.get('test_camera', False),
            cameras=[dict(model_mask_pixels_nearest=25)] if c.get('test_camera') else [])
            for c in components]
        measured = dict(captures=[dict(id='slot', components=support)])
        records = [self.instance(c) for c in components if (
            c['lidar_points'] or c.get('test_camera')) and q.in_roi(c['center_lidar_m'])]
        derived = dict(train=dict(data_list=[dict(token='slot', scene_group_id='group',
                                                 instances=records)]))
        return [row], measured, derived

    def test_unterminated_or_duplicate_selection_fails_without_source_edit(self):
        path = self.root / 'selected.jsonl'
        value = dict(id='slot', status='accepted_candidate', capture_root='/native/a',
                     scenario_sha256='a')
        for payload in (json.dumps(value).encode(),
                        (json.dumps(value) + '\n') .encode() * 2):
            path.write_bytes(payload)
            with self.assertRaises(ValueError):
                q.ledger(path)
            self.assertEqual(path.read_bytes(), payload)

    def test_reviewed_native_attempt_does_not_fill_selected_cohort(self):
        path = self.root / 'ledger.jsonl'
        path.write_text(json.dumps(dict(status='review')) + '\n')
        with self.assertRaisesRegex(ValueError, 'approved selections only'):
            q.ledger(path)

    def test_upright_full_rpy_and_identity_class_guards(self):
        component = self.component()
        instance = self.instance(component)
        q.assert_upright(component, instance)
        for key, value in (('component_id', 'different'), ('bbox_label_3d', 0),
                           ('bbox_3d', component['center_lidar_m'] + component['size_m'] + [.3]),
                           ('num_lidar_pts', 2)):
            changed = copy.deepcopy(instance)
            changed[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                q.assert_upright(component, changed)

    def test_zero_lidar_camera_supported_component_is_retained(self):
        rows, measured, derived = self.object_fixture([
            self.component(lidar_points=0, test_camera=True)])
        objects, _ = q.object_rows(rows, measured, derived)
        self.assertTrue(objects[0]['retained'])
        self.assertEqual(objects[0]['lidar_points'], 0)
        self.assertTrue(objects[0]['camera512_supported'])

    def test_ambiguous_component_remains_unknown_without_invented_pixels(self):
        # Primary has independent LiDAR; sibling zero-point component stays raw
        # and ambiguous, never converted by borrowing its sibling's mask.
        components = [self.component(), self.component('8500:Pedestrian:0',
            training_class='pedestrian', raw_class='Pedestrian', lidar_points=0,
            camera_component_ambiguous=True)]
        rows, measured, derived = self.object_fixture(components)
        objects, _ = q.object_rows(rows, measured, derived)
        self.assertEqual(len(objects), 2)
        self.assertTrue(objects[1]['compound_parent'])
        self.assertFalse(objects[1]['retained'])
        self.assertEqual(objects[1]['camera_state'], 'unknown_component_correspondence')
        measured['captures'][0]['components'][1]['camera_gate512'] = True
        with self.assertRaisesRegex(ValueError, 'invented camera support'):
            q.object_rows(rows, measured, derived)

    def test_missing_and_extra_retained_components_are_fatal(self):
        rows, measured, derived = self.object_fixture([self.component()])
        derived['train']['data_list'][0]['instances'] = []
        with self.assertRaisesRegex(ValueError, 'Supported component missing'):
            q.object_rows(rows, measured, derived)
        rows, measured, derived = self.object_fixture([self.component()])
        extra = self.instance(self.component('8501:Obstacle:0'))
        derived['train']['data_list'][0]['instances'].append(extra)
        with self.assertRaisesRegex(ValueError, 'Extra or missing converted'):
            q.object_rows(rows, measured, derived)

    def test_model_center_roi_and_radial_band_have_different_meaning(self):
        center = [99., 29., 0.]
        self.assertTrue(q.in_roi(center))
        self.assertEqual(q.distance_band(np.hypot(*center[:2])), 'outside_radial_100m')
        rows, measured, derived = self.object_fixture([
            self.component(center_lidar_m=[100., 0., 0.])])
        objects, _ = q.object_rows(rows, measured, derived)
        self.assertFalse(objects[0]['retained'])
        self.assertEqual(objects[0]['exclusion_reason'], 'outside_roi')

    def test_summary_distinguishes_20plus_bucket_from_zero_points(self):
        objects = [dict(training_class='obstacle', asset='a', retained=True,
            distance_lidar_xy_m=25., direction='front', lidar_points=points,
            camera512_supported=True, camera_component_ambiguous=False)
            for points in (0, 1, 4, 5, 19)]
        totals = q.summarize(objects, True)['totals']
        self.assertEqual(totals['lidar_0'], 1)
        self.assertEqual(totals['lidar_1_4'], 2)
        self.assertEqual(totals['lidar_5_19'], 2)
        self.assertEqual(totals.get('lidar_20plus', 0), 0)
        self.assertEqual(totals['camera_only'], 1)
        unknown = q.summarize([dict(objects[0], camera512_supported=None)], True)['totals']
        self.assertEqual(unknown['camera512_supported'], 0)
        self.assertEqual(unknown['camera512_unknown'], 1)

    def test_input_gap_uses_model_points_not_native_count_and_preserves_unknown(self):
        base = dict(training_class='pedestrian', distance_lidar_xy_m=25.,
            split='train', retained=True, lidar_points=2, model_roi_lidar_points=0,
            camera512_supported=None, camera_component_ambiguous=True,
            compound_shared_bbox_points=2)
        rows = [base, dict(base, camera512_supported=True),
                dict(base, model_roi_lidar_points=1)]
        result = q.input_support_diagnostics(rows)
        self.assertEqual(result['native_positive_but_current_model_zero_and_no_resolved_camera'], 1)
        self.assertEqual(result['ambiguous_with_shared_bbox_points'], 3)
        self.assertIsNone(result['gap_examples'][0]['camera512_supported'])

    def test_compound_point_overlap_is_geometric_and_parent_local(self):
        points = np.array([[10., 0., -1., 1.], [30., 0., -1., 1.]])
        components = [self.component(rotation_rpy=[0., 0., 0.]),
            self.component('8500:Pedestrian:0', rotation_rpy=[0., 0., 0.]),
            self.component('8501:Obstacle:0', parent_uid=8501, rotation_rpy=[0., 0., 0.])]
        result = q.shared_compound_points(points, components)
        self.assertEqual(result, {'8500:Obstacle:0': 1, '8500:Pedestrian:0': 1})

    def test_cpu_loader_expected_augmented_roi_removes_only_transformed_boundary(self):
        instances = [dict(bbox_3d=[99., 0., -1., 2., 3., 4., .1], bbox_label_3d=2),
                     dict(bbox_3d=[20., 0., -1., 1., 2., 3., .2], bbox_label_3d=1)]
        matrix = np.diag([1.05, 1.05, 1.05, 1.])
        matrix[:3, 3] = [.2, .1, -.2]
        centers, dims, labels = loader.expected_after_bev(instances, matrix,
                                                         [-100., -30., -3., 100., 30., 2.])
        np.testing.assert_allclose(centers, [[21.2, .1, -1.25]])
        np.testing.assert_allclose(dims, [[1.05, 2.1, 3.15]])
        np.testing.assert_array_equal(labels, [1])

    def native_fixture(self):
        capture = self.root / 'raw'
        camera_sizes = ((1280, 720), (640, 480), (640, 480), (1280, 720))
        for index, (width, height) in enumerate(camera_sizes, 1):
            folder = capture / f'CAMERA_{index}'
            folder.mkdir(parents=True)
            rgb = np.zeros((height, width, 3), np.uint8)
            rgb[..., 1] = 80
            Image.fromarray(rgb).save(folder / 'frame_Intensity.png')
            rgba = np.zeros((height, width, 4), np.uint8)
            rgba[..., 3] = 255
            text = ''
            if index == 1:
                rgba[500:550, 600:650] = [7, 11, 13, 255]
                text = 'Obstacle 600 500 650 550 8500 [7, 11, 13, 255]\n'
            Image.fromarray(rgba).save(folder / 'frame_Instance.png')
            (folder / 'frame_BBox2D.txt').write_text(text)
        lidar = capture / 'LIDAR_5'
        lidar.mkdir()
        np.asarray([[10., 0., -1., 100.]], np.float32).tofile(lidar / 'frame_Intensity.bin')
        component = self.component()
        component['cameras'] = camera_evidence(capture, 8500, 'Obstacle')
        (lidar / 'frame_Instance.txt').write_text('Obstacle ' + ' '.join(map(str,
            component['center_lidar_m'] + component['rotation_rpy'] + component['size_m']
            + [10., 0., 0., 0.])) + ' 8500\n')
        self.write('raw/audit.json', dict(components=[component], review_reasons=[],
                                        accepted_candidate=True))
        scenario = self.write('raw/scenario.json', dict(example='frozen'))
        calibration = normalized_sensor_calibration(q.REPO / 'configs/sensors/Sensor_competition_local.json')
        self.write('raw/metadata.json', dict(spec=dict(family='obstacle', mission=False), split='train',
            ego=dict(x=0., y=0., z=0., yaw=0.),
            scene_group_id='fixed-group', sensor_source_sha256=calibration['source_sha256'],
            lidar_to_ego=calibration['lidar']['lidar_to_ego'],
            runtime_actors=[dict(unique_id=8500, planned_center_distance_m=10.,
                                source_list='objectList', population_pool='obstacle')]))
        ledger = self.root / 'selected_ledger.jsonl'
        ledger.write_text(json.dumps(dict(id='slot', capture_root=str(capture),
            status='accepted_candidate', scenario_sha256=q.sha256(scenario))) + '\n')
        return ledger, capture

    def test_full_cpu_q90_upright_training3_derivation_preserves_raw(self):
        ledger, capture = self.native_fixture()
        original = {p: p.read_bytes() for p in capture.rglob('*') if p.is_file()}
        result = q.qualify_cohort('test', ledger, self.root / 'derived',
                                  q.REPO / 'configs/sensors/Sensor_competition_local.json')
        self.assertEqual(result['frames'], 1)
        self.assertEqual(result['retained_gt'], 1)
        self.assertEqual(result['rgb']['images_checked'], 4)
        self.assertEqual(result['rgb']['actual_rgb_crop_shape'], [512, 1056, 3])
        self.assertTrue(result['source_unchanged'])
        self.assertEqual(original, {p: p.read_bytes() for p in capture.rglob('*') if p.is_file()})
        package = Path(result['training3_package'])
        info = pickle.loads((package / 'infos/512x1056/morai_base5_infos_train.pkl').read_bytes())
        self.assertEqual(tuple(info['metainfo']['classes']), ('vehicle', 'pedestrian', 'obstacle'))
        self.assertEqual(info['data_list'][0]['instances'][0]['original_semantic_class'], 'obstacle')
        self.assertTrue(all(camera['img_path'].endswith('.jpg')
            for camera in info['data_list'][0]['images'].values()))

    def test_native_cpu_derivation_uses_tagged_layout_overlay_not_raw_split(self):
        import base5_layout_split as layout
        ledger, capture = self.native_fixture()
        records = []
        for index in range(30):
            meta = q.read(capture/'metadata.json')
            meta['scene_group_id'] = 'clip-'+str(index)
            meta['ego'].update(x=float(index), link_idx='same-known-map-link')
            meta['spec'].update(role='front', yaw_limit=3, distance_bin=0, topology='route_general')
            meta['runtime_actors'][0].update(data_id=40100002, name='WoodBox', motion='static',
                position=dict(x=float(index)+10, y=0., z=0.), placement_yaw_deg=0., estimated_size=[1.,1.,1.])
            records.append(dict(id='slot-'+str(index), meta=meta,
                scenario_sha256=q.sha256(capture/'scenario.json')))
        overlay = layout.allocate(records)
        ident, assignment = next((key, value) for key, value in overlay['assignments'].items() if value['split']=='val')
        overlay['overlay_sha256'] = layout.digest(overlay)
        overlay_path = self.write('split_overlay.json', overlay)
        meta = next(row['meta'] for row in records if row['id']==ident)
        meta['training_split_overlay'] = dict(path=str(overlay_path), sha256=q.sha256(overlay_path),
            overlay_sha256=overlay['overlay_sha256'])
        self.write('raw/metadata.json', meta)
        # A finalized cohort can authorize immutable RAW tags from its parent
        # overlays while the derived ledger selects one merged overlay.
        consolidated = copy.deepcopy(overlay)
        consolidated.pop('overlay_sha256')
        consolidated['authorized_source_overlays'] = {overlay['overlay_sha256']:
            dict(path=str(overlay_path),sha256=q.sha256(overlay_path))}
        consolidated['overlay_sha256'] = layout.digest(consolidated)
        consolidated_path = self.write('consolidated_overlay.json',consolidated)
        row, = q.ledger(ledger);row.update(id=ident,training_split_overlay=dict(
            path=str(consolidated_path),sha256=q.sha256(consolidated_path),
            overlay_sha256=consolidated['overlay_sha256']))
        ledger.write_text(json.dumps(row)+'\n')
        original = {p:p.read_bytes() for p in capture.rglob('*') if p.is_file()}
        result = q.qualify_cohort('overlay', ledger, self.root/'overlay_derived',
            q.REPO/'configs/sensors/Sensor_competition_local.json')
        self.assertEqual(result['split_frames'], dict(val=1))
        self.assertEqual(result['source_split_frames'], dict(train=1))
        self.assertEqual(result['retained_gt'], 1)
        package = Path(result['training3_package'])
        info = pickle.loads((package/'infos/512x1056/morai_base5_infos_val.pkl').read_bytes())
        self.assertEqual(info['data_list'][0]['scene_group_id'], assignment['group_id'])
        self.assertEqual(original, {p:p.read_bytes() for p in capture.rglob('*') if p.is_file()})

    def test_changed_native_geometry_and_point_support_are_fatal(self):
        ledger, capture = self.native_fixture()
        row, = q.ledger(ledger)
        audit = q.read(capture / 'audit.json')
        altered = copy.deepcopy(audit)
        altered['components'][0]['size_m'][0] = 2.
        with self.assertRaisesRegex(ValueError, 'geometry mismatch'):
            q.assert_native(capture, altered, row['scenario_sha256'])
        altered = copy.deepcopy(audit)
        altered['components'][0]['lidar_points'] = 0
        with self.assertRaisesRegex(ValueError, 'point support differs'):
            q.assert_native(capture, altered, row['scenario_sha256'])

    def test_empty_camera_annotations_stay_unknown_but_existing_evidence_cannot_disappear(self):
        ledger, capture = self.native_fixture()
        row, = q.ledger(ledger)
        image = capture / 'CAMERA_1/frame_Instance.png'
        image.unlink()
        with self.assertRaisesRegex(ValueError, 'Saved camera evidence is missing'):
            q.measure_selected(ledger, [row])
        audit = q.read(capture / 'audit.json')
        audit['components'][0]['cameras'] = []
        (capture / 'audit.json').write_text(json.dumps(audit))
        for path in capture.glob('CAMERA_*/*_Instance.png'):
            path.unlink()
        measured = q.measure_selected(ledger, [row])
        component, = measured['captures'][0]['components']
        self.assertEqual(component['camera_state'], 'incomplete')
        self.assertFalse(component['camera_gate512'])
        self.assertTrue(component['kept512'])  # The independent LiDAR point remains.

    def test_existing_output_and_duplicate_cohort_raw_are_rejected(self):
        ledger, _ = self.native_fixture()
        output = self.root / 'output'
        output.mkdir()
        with self.assertRaisesRegex(ValueError, 'Never overwrite'):
            q.run({'a': ledger}, output, 'absent')
        with patch.object(q, 'qualify_cohort', return_value={}):
            with self.assertRaisesRegex(ValueError, 'multiple cohorts'):
                q.run({'a': ledger, 'b': ledger}, self.root / 'new-output', 'absent')


if __name__ == '__main__':
    unittest.main()
