"""Synthetic CPU-only native scale diagnostics; no simulator or GPU access."""
import copy
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report_base5_scale_probe as probe


class ScaleProbeTest(unittest.TestCase):
    def test_lazy_runtime_import_is_pinned_before_execution(self):
        with patch.object(probe, 'runtime_sources', return_value={'a.py':'a','lazy.py':'b'}):
            self.assertEqual(probe.verify_runtime_sources({'a.py':'a','lazy.py':'b','unused.py':'c'}),
                             {'a.py':'a','lazy.py':'b'})

    def test_lazy_runtime_mutation_or_unpinned_import_is_rejected(self):
        for used in ({'a.py':'changed'}, {'new.py':'b'}):
            with patch.object(probe, 'runtime_sources', return_value=used):
                with self.assertRaisesRegex(ValueError, 'runtime source changed'):
                    probe.verify_runtime_sources({'a.py':'a'})

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, path, value):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value) + '\n')
        return path

    def fixture(self, name, scale=(1., 1., 1.), yaw=0., size=(2., 1., 1.),
                asset='CargoBox', training_class='obstacle', condition='control'):
        root = self.root / name
        center, rotation = [10., 0., -1.], [0., 0., yaw]
        lidar = root / 'LIDAR_5'
        lidar.mkdir(parents=True)
        np.asarray([[10., 0., -1., 100.], [0., 0., 0., 0.]], np.float32).tofile(lidar / 'frame_Intensity.bin')
        (lidar / 'frame_Instance.txt').write_text('Obstacle ' + ' '.join(map(str,
            center + rotation + list(size) + [10., 0., 0., 0.])) + ' 8500\n')
        components = [dict(parent_uid=8500, component_id='8500:Obstacle:0', asset=asset,
            raw_class='Obstacle', training_class=training_class, center_lidar_m=center,
            size_m=list(size), rotation_rpy=rotation, lidar_points=1)]
        if list(scale)!=[1.,1.,1.]:
            from base5_capture_audit import rotation_matrix
            from audit_morai_lidar_gt_visibility import parse_instances
            from base5_native_box_corners import load_matching_corners,derive_geometry
            corners=np.asarray(list(itertools.product((-1.,1.),repeat=3)))*np.asarray(size)/2
            corners=corners@rotation_matrix(rotation).T+np.asarray(center)
            (lidar/'frame_Instance_8Points.txt').write_text('Obstacle '+' '.join(map(str,
                [*corners.reshape(-1),*rotation,0.,0.,0.,0.,8500]))+'\n')
            annotation=lidar/'frame_Instance.txt'
            box=parse_instances(annotation)[0]
            derived=derive_geometry(box,load_matching_corners(annotation)[(8500,'Obstacle',0)],allow_dimension_change=True)
            components[0].update(native_corner_geometry=derived['evidence'],native_summary_size_m=list(size))
        audit = dict(components=components, ground_warnings=['annotation_plane_not_contact'],
                     review_reasons=[], accepted_candidate=True,
                     capture_safety=dict(collision=dict(observed=True, no_collision_verified=True)))
        self.write(root / 'audit.json', audit)
        scenario = self.write(root / 'scenario.json', dict(objectList=[dict(UNIQUEID=8500,DataID=40100001,
            scale=dict(zip('xyz', scale)), pos=dict(x=10., y=0., z=0.))]))
        self.write(root / 'metadata.json', dict(runtime_actors=[dict(unique_id=8500, name=asset, data_id=40100001,
            source_list='objectList',motion='static', training_class=training_class, estimated_size=list(size))],
            lidar_to_ego=np.eye(4).tolist(),
            scale_probe=dict(pair_key='pair0', condition=condition, asset=asset, parent_uid=8500,
                             requested_scale=list(scale), requested_yaw_deg=float(np.degrees(yaw)))))
        self.write(root / 'shutdown_safety.json', dict(park_verified=True))
        for index, (_, width, height) in enumerate(probe.CAMERAS.values(), 1):
            camera = root / f'CAMERA_{index}'
            camera.mkdir()
            rgb = np.zeros((height, width, 3), np.uint8)
            rgb[..., 1] = 60
            Image.fromarray(rgb).save(camera / 'frame_Intensity.png')
            mask = np.zeros((height, width, 4), np.uint8)
            mask[..., 3] = 255
            text = ''
            if index == 1:
                mask[500:550, 600:650] = [7, 11, 13, 255]
                text = 'Obstacle 600 500 650 550 8500 [7, 11, 13, 255]\n'
            Image.fromarray(mask).save(camera / 'frame_Instance.png')
            (camera / 'frame_BBox2D.txt').write_text(text)
        return dict(id=name, capture_root=str(root), status='accepted_candidate',
                    scenario_sha256=probe.sha256(scenario), audit=audit, source_ledger='test')

    def test_scale_vector_retains_unity_axis_order_and_rejects_invalid(self):
        self.assertEqual(probe.scale_xyz(dict(x=2., y=3., z=4.)), [2., 3., 4.])
        for value in ([1, 2], [1, -1, 1], [1, float('nan'), 1]):
            with self.subTest(value=value), self.assertRaises(ValueError):
                probe.scale_xyz(value)

    def test_registration_and_payload_scale_must_agree(self):
        row = self.fixture('control')
        root = Path(row['capture_root'])
        metadata = probe.read(root / 'metadata.json')
        scene = probe.read(root / 'scenario.json')
        probe.resolve_probe(metadata, scene)
        scene['objectList'][0]['scale']['x'] = 2
        with self.assertRaisesRegex(ValueError, 'actual scenario scale'):
            probe.resolve_probe(metadata, scene)

    def test_no_scale_control_and_forbidden_vehicle_composites(self):
        row = self.fixture('scaled', scale=(2., 2., 2.), condition='scaled',
                           asset='OBJ_G80', training_class='vehicle')
        root = Path(row['capture_root'])
        metadata = probe.read(root / 'metadata.json')
        scene = probe.read(root / 'scenario.json')
        self.assertTrue(probe.resolve_probe(metadata, scene)[0]['scale_eligibility_violation'])
        metadata['scale_probe']['condition'] = 'control'
        with self.assertRaisesRegex(ValueError, 'unit scale'):
            probe.resolve_probe(metadata, scene)

    def test_anisotropic_native_axis_hypotheses_independent_of_yaw(self):
        for yaw in (0., np.pi / 2):
            # Unity xyz [2,3,4] might map native lwh <- z,x,y. That mapping
            # is diagnosed, never assumed in native GT or changed by yaw.
            a = dict(id='unit', probe=dict(pair_key='p', condition='control', asset='CargoBox',
                parent_uid=8500, requested_yaw_deg=yaw, actual_scenario_scale_xyz=[1, 1, 1]),
                components=[dict(raw_class='Obstacle', native_size_lwh_m=[2, 1, 1],
                                 native_center_lidar_m=[10, 0, -1])])
            b = copy.deepcopy(a)
            b['id'] = 'scaled'
            b['probe'].update(condition='scaled', actual_scenario_scale_xyz=[2, 3, 4])
            b['components'][0]['native_size_lwh_m'] = [8, 2, 3]
            comparison, = probe.pair_reports([a, b])
            self.assertTrue(comparison['comparable'])
            self.assertEqual(comparison['hypotheses'][0]['native_lwh_from_unity_xyz'], ['z', 'x', 'y'])
            self.assertEqual(comparison['hypotheses'][0]['maximum_absolute_ratio_error'], 0.)

    def test_no_control_is_not_guessed_from_nearby_asset(self):
        value = dict(id='scaled', probe=dict(pair_key='p', condition='scaled'))
        comparison, = probe.pair_reports([value])
        self.assertFalse(comparison['comparable'])

    def test_actual_pivot_transform_uses_raw_ego_heading_correction_once(self):
        transform = np.eye(4)
        transform[:3, 3] = [.5, 0., 1.8]
        metadata = dict(ego=dict(yaw=90.), lidar_to_ego=transform.tolist())
        status = dict(obstacle_list=[dict(unique_id=8500, position=dict(x=0., y=10., z=0.))])
        telemetry = dict(schema_version=3, ego_before=dict(x=0., y=0., z=0., heading=91.347),
                         objects_before=status, objects_after=status)
        component = dict(parent_uid=8500, center_lidar_m=[10., 0., -1.], rotation_rpy=[0., 0., 0.])
        result = probe.pivot_diagnostic(component, metadata, telemetry)
        self.assertTrue(result['available'])
        self.assertEqual(result['heading_correction_applied_once_deg'], -1.347)
        np.testing.assert_allclose(result['native_bbox_center_world_m'], [0., 10.5, .8], atol=1e-12)
        np.testing.assert_allclose(result['pivot_to_native_bbox_center_local_m'], [.5, 0., .8], atol=1e-12)
        component['rotation_rpy'][2] = np.pi / 2
        result = probe.pivot_diagnostic(component, metadata, telemetry)
        np.testing.assert_allclose(result['pivot_to_native_bbox_center_local_m'], [0., -.5, .8], atol=1e-12)

    def test_corrected_heading_and_unknown_pose_do_not_get_recorrected(self):
        component = dict(parent_uid=8500, center_lidar_m=[10., 0., -1.], rotation_rpy=[0., 0., 0.])
        metadata = dict(ego=dict(yaw=0.), lidar_to_ego=np.eye(4).tolist())
        telemetry = dict(schema_version=3, ego_before=dict(x=0., y=0., z=0., heading=1.347,
                                                         heading_corrected=True))
        self.assertFalse(probe.pivot_diagnostic(component, metadata, telemetry)['available'])
        telemetry['ego_before'].pop('heading_corrected')
        telemetry['ego_before']['heading'] = 181.347
        self.assertIn('conflicts', probe.pivot_diagnostic(component, metadata, telemetry)['reason'])

    def test_native_dimensions_are_not_multiplied_twice(self):
        row = self.fixture('scaled', scale=(2., 3., 2.), size=(4., 3., 2.), condition='scaled')
        output = self.root / 'output'
        output.mkdir()
        result = probe.capture_row(row, output, 0)
        self.assertEqual(result['components'][0]['native_size_lwh_m'], [4., 3., 2.])
        self.assertEqual(result['components'][0]['lidar_bbox_points'], 1)
        self.assertTrue(result['components'][0]['camera512_supported'])
        self.assertEqual(result['human_semantic_review']['status'], 'pending')
        self.assertFalse(result['human_semantic_review']['automatic_approval'])
        self.assertEqual(Image.open(result['montage']).size, (1280, 1020))
        self.assertEqual(Image.open(result['lidar_projection']).size, (1080, 400))
        self.assertEqual(result['ground_warnings'], ['annotation_plane_not_contact'])
        self.assertTrue(result['shutdown']['park_verified'])

    def test_report_native_inventory_unchanged_and_does_not_change_approval(self):
        rows = [self.fixture('unit'), self.fixture('scaled', scale=(2, 3, 2),
                    size=(4., 3., 2.), condition='scaled')]
        ledger = self.root / 'ledger.jsonl'
        ledger.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        old = {str(p): p.read_bytes() for row in rows for p in Path(row['capture_root']).rglob('*') if p.is_file()}
        result = probe.report([ledger], self.root / 'report')
        self.assertTrue(result['native_unchanged'])
        self.assertEqual(result['summary']['comparable_scale_pairs'], 1)
        self.assertEqual(result['captures'][1]['status'], 'accepted_candidate')
        self.assertEqual(old, {str(p): p.read_bytes() for row in rows for p in Path(row['capture_root']).rglob('*') if p.is_file()})
        with self.assertRaisesRegex(ValueError, 'Fresh diagnostic output'):
            probe.report([ledger], self.root / 'report')

    def test_source_tamper_is_fatal_instead_of_changing_gt(self):
        row = self.fixture('unit')
        root = Path(row['capture_root'])
        audit = probe.read(root / 'audit.json')
        audit['components'][0]['size_m'][0] = 20.
        self.write(root / 'audit.json', audit)
        row['audit'] = audit
        output = self.root / 'bad'
        output.mkdir()
        with self.assertRaisesRegex(ValueError, 'geometry mismatch'):
            probe.capture_row(row, output, 0)

    def test_component_annotation_metadata_is_not_a_semantic_class(self):
        row = self.fixture('scaled', scale=(1.5, 1.5, 1.5), condition='scaled')
        root = Path(row['capture_root'])
        metadata = probe.read(root / 'metadata.json')
        metadata['runtime_actors'][0]['training_class'] = 'component_annotation'
        self.write(root / 'metadata.json', metadata)
        output = self.root / 'semantic'
        output.mkdir()
        result = probe.capture_row(row, output, 0)
        self.assertFalse(result['probe']['scale_eligibility_violation'])

    def test_saved_mask_support_cannot_silently_disappear(self):
        row = self.fixture('unit')
        root = Path(row['capture_root'])
        audit = probe.read(root / 'audit.json')
        audit['components'][0]['cameras'] = [dict(camera='CAMERA_1', camera_instance_id=8500,
            native_rect=[600., 500., 650., 550.], native_instance_pixels=1)]
        self.write(root / 'audit.json', audit)
        row['audit'] = audit
        output = self.root / 'mismatch'
        output.mkdir()
        with self.assertRaisesRegex(ValueError, 'current native mask'):
            probe.capture_row(row, output, 0)

    def test_duplicate_attempts_and_changed_inventory_fail(self):
        row = self.fixture('unit')
        ledger = self.root / 'ledger.jsonl'
        ledger.write_text(json.dumps(row) + '\n')
        with self.assertRaisesRegex(ValueError, 'supplied twice'):
            probe.report([ledger, ledger], self.root / 'duplicate')
        with self.assertRaisesRegex(ValueError, 'inside immutable native'):
            probe.report([ledger], Path(row['capture_root']) / 'report')
        initial = probe.inventory([ledger], [row])
        changed = dict(initial, invented=dict(sha256='bad'))
        with patch.object(probe, 'inventory', side_effect=[initial, changed]):
            with self.assertRaisesRegex(ValueError, 'changed during'):
                probe.report([ledger], self.root / 'mutation')


if __name__ == '__main__':
    unittest.main()
