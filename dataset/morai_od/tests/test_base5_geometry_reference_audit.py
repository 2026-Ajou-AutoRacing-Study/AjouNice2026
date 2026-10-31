import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from audit_base5_geometry_reference import (Sources, audit, candidate_reference,
    classify, distribution, native_observation, size_modes, vector)
import base5_native_box_corners as corners


class GeometryReferenceAuditTest(unittest.TestCase):
    def test_classification_keeps_nonunit_compound_and_motion_separate(self):
        actor = dict(motion='static')
        payload = dict(scale=dict(x=1., y=1., z=1.))
        self.assertEqual(classify(actor, payload, 1), 'static_unit_single')
        self.assertEqual(classify(actor, payload, 2), 'compound_parent')
        self.assertEqual(classify(dict(motion='walking'), payload, 1), 'moving_unit_single')
        self.assertEqual(classify(dict(actor, motion_path={'points': []}), payload, 1),
                         'moving_unit_single')
        payload['scale']['y'] = 1.25
        self.assertEqual(classify(actor, payload, 1), 'nonunit_single')
        self.assertEqual(classify(actor, payload, 2), 'compound_parent')

    def test_bad_scales_fail_closed(self):
        for invalid in (True, 0., float('nan'), float('inf')):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                classify(dict(motion='static'), dict(scale=dict(x=invalid, y=1, z=1)), 1)

    def test_modes_do_not_hide_redbarrel_states_in_one_median(self):
        rows = [dict(scene_id=str(i), parent_uid=i, corner_local_size_m=size)
                for i, size in enumerate(([.57, .57, .92], [.57001, .57, .92],
                                          [.7125, .7125, 1.15]))]
        result = size_modes(rows)
        self.assertEqual([item['distribution']['n'] for item in result], [2, 1])
        self.assertEqual(size_modes([]), [])

    def test_mode_tolerance_cannot_chain_into_broad_variation(self):
        rows = [dict(scene_id=str(i), parent_uid=i, corner_local_size_m=[1 + i * .004, 1, 1])
                for i in range(3)]
        self.assertEqual(len(size_modes(rows)), 2)

    def test_reference_is_diagnostic_and_preserves_envelope(self):
        envelope = dict(bbox_size_m=[2., 2., 2.], pivot_to_bbox_center_local_m=[0., 0., 1.])
        original = copy.deepcopy(envelope)
        rows = [dict(scene_id=str(i), parent_uid=i, classification='static_unit_single',
                     corner_local_size_m=[1., 1., 1.], summary_corner_dimensions_differ=False)
                for i in range(2)]
        candidate, reasons = candidate_reference('CargoBox', dict(data_id=1), rows, envelope)
        self.assertEqual(reasons, [])
        self.assertFalse(candidate['production_approved'])
        self.assertFalse(candidate['height_contact_certified'])
        self.assertEqual(candidate['pivot_to_bbox_center_local_m'], [0., 0., .5])
        self.assertEqual(envelope, original)
        candidate, reasons = candidate_reference('RedBarrel', dict(data_id=3), rows,
                                                 envelope, known_variation=True)
        self.assertIsNone(candidate)
        self.assertIn('independent_native_size_variation_evidence', reasons)
        rows[-1]['summary_corner_dimensions_differ'] = True
        self.assertIsNone(candidate_reference('Box', dict(data_id=1), rows, envelope)[0])

    def test_nonunit_rows_are_not_divided_by_scale_into_unit_reference(self):
        rows = [dict(classification='nonunit_single', corner_local_size_m=[1., 1., 1.])]
        candidate, reasons = candidate_reference('Box', dict(data_id=1), rows, {})
        self.assertIsNone(candidate)
        self.assertIn('fewer_than_two_unit_observations', reasons)

    def test_vector_and_distribution(self):
        np.testing.assert_array_equal(vector([1, 2, 3], True), [1, 2, 3])
        self.assertIsNone(distribution([]))
        self.assertEqual(distribution([[1, 2, 3], [3, 4, 5]])['q50_m'], [2., 3., 4.])
        for value in ([1, True, 3], [1, float('nan'), 3], [1, 2]):
            with self.assertRaises(ValueError):
                vector(value)

    def test_source_mutation_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'source.json'
            path.write_text('{}')
            sources = Sources()
            self.assertEqual(sources.json(path), {})
            path.write_text('{"changed": true}')
            with self.assertRaises(ValueError):
                sources.verify()
            link = Path(temp) / 'link.json'
            link.symlink_to(path)
            with self.assertRaises(ValueError):
                Sources().data(link)

    def fixture(self, root):
        def write(name, value):
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(value))
            return path
        capture = root / 'capture'
        component = dict(parent_uid=8500, component_id='8500:Obstacle:0',
            asset='CargoBox', data_id=40100001, raw_class='Obstacle', training_class='obstacle',
            center_lidar_m=[0., 0., 0.], size_m=[1., 1., 1.], rotation_rpy=[0., 0., 0.],
            lidar_points=0, positive_lidar1_camera0=True)
        actor = dict(unique_id=8500, name='CargoBox', data_id=40100001,
                     source_list='objectList', motion='static')
        payload = dict(UNIQUEID=8500, DataID=40100001, scale=dict(x=1., y=1., z=1.))
        scenario = write('capture/scenario.json', dict(objectList=[payload]))
        write('capture/metadata.json', dict(split='train', sensor_source_sha256='sensor',
                                          runtime_actors=[actor]))
        saved = dict(components=[component], accepted_candidate=True)
        write('capture/audit.json', saved)
        entry = dict(id='scene1', capture_root=str(capture), status='accepted_candidate',
                     scenario_sha256=hashlib.sha256(scenario.read_bytes()).hexdigest(), audit=saved)
        ledger = root / 'selected_ledger.jsonl'
        ledger.write_text(json.dumps(entry) + '\n')
        points = [[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)]
        annotation = capture / 'LIDAR_5/t_Instance.txt'
        annotation.parent.mkdir()
        annotation.write_text('Obstacle 0 0 0 0 0 0 1 1 1 0 0 0 0 8500\n')
        companion = annotation.with_name('t_Instance_8Points.txt')
        companion.write_text('Obstacle ' + ' '.join(map(str,
            [v for point in points for v in point] + [0] * 7)) + ' 8500\n')
        assets = [dict(name='CargoBox' if i == 0 else 'Unused' + str(i), data_id=40100001 + i,
                       semantic_class='obstacle', collection_role='positive') for i in range(19)]
        params = dict(catalog=write('catalog.json', dict(assets=assets)),
            historical=write('old.json', dict(assets={'40100001': dict(name='CargoBox', observations=1,
                size_q50_m=[1., 1., 1.])})),
            source_manifest=write('manifest.json', dict(config={})),
            unit_references=[write('unit.json', dict(assets={}))],
            variation=write('variation.json', dict(passed=True, asset=dict(name='RedBarrel'),
                source_files={}, summary=dict(unit_observations=0), observations=[])),
            qualified=write('qualified.json', dict(assets={})), expected_frames=1)
        return ledger, params, annotation, component, actor, payload

    def test_full_read_only_cli_path_and_zero_lidar_camera_positive_retention(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            ledger, params, annotation, _, _, _ = self.fixture(root)
            before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
            report = audit(ledger, root / 'output', **params)
            self.assertTrue(report['passed'])
            self.assertEqual(report['selected_scenes'], 1)
            self.assertEqual(report['counts']['static_unit_single'], 1)
            rows = json.loads((root / 'output/observations.json').read_text())
            self.assertTrue(rows[0]['audit_positive_lidar1_camera0'])
            self.assertEqual(rows[0]['lidar_points'], 0)
            self.assertFalse(rows[0]['gt_adoption'])
            for path, content in before.items():
                self.assertEqual(path.read_bytes(), content)
            with self.assertRaises(ValueError):
                audit(ledger, root / 'output', **params)

    def test_native_annotation_mismatch_is_not_silently_repaired(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            ledger, params, annotation, _, _, _ = self.fixture(root)
            annotation.write_text(annotation.read_text().replace('1 1 1', '2 1 1'))
            with self.assertRaisesRegex(ValueError, 'summary size differs'):
                audit(ledger, root / 'output', **params)
            self.assertFalse((root / 'output').exists())

    def test_compound_does_not_force_single_geometry(self):
        component = dict(component_id='8500:Obstacle:0', parent_uid=8500, asset='Box', data_id=1,
            size_m=[1., 1., 1.], rotation_rpy=[0., 0., 0.], center_lidar_m=[0., 0., 0.],
            lidar_points=1, positive_lidar1_camera0=True)
        result = native_observation(component, {}, dict(scale=dict(x=1., y=1., z=1.)),
                                    Path('/tmp/capture'), {}, 'compound_parent')
        self.assertNotIn('corner_local_size_m', result)


if __name__ == '__main__':
    unittest.main()
