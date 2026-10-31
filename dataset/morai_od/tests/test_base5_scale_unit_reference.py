import copy
import itertools
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import measure_base5_scale_unit_reference as measure


class UnitReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manifest = self.write(self.root/'source.json', dict(config={}))
        self.rows = []
        self.size = [1., 2., 1.]
        self.capture(0.)
        self.capture(90.)
        self.persist()

    def write(self, path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value)+'\n')
        return path

    def capture(self, yaw):
        ident = 'control_'+str(int(yaw))
        root = self.root/'captures'/ident
        uid, name, data_id = 8500, 'CargoBox', 40100001
        actor = dict(unique_id=uid, name=name, data_id=data_id, source_list='objectList', motion='static',
            placement_yaw_deg=yaw, position=dict(x=5., y=0., z=.1),
            map_ground_initializer=dict(usable=True, height_m=0.))
        metadata = dict(runtime_actors=[actor], scale_probe=dict(asset=name, profile='original', condition='control',
            parent_uid=uid, requested_scale=[1., 1., 1.], requested_yaw_deg=yaw))
        scenario = dict(objectList=[dict(UNIQUEID=uid, DataID=data_id, scale=dict(x=1., y=1., z=1.),
            pos=actor['position'], rot=dict(yaw=yaw))])
        components = [dict(parent_uid=uid, raw_class='Obstacle', asset=name, size_m=self.size)]
        audit = dict(accepted_candidate=True, review_reasons=[], components=components,
            capture_safety=dict(collision=dict(observed=True, no_collision_verified=True)))
        self.write(root/'metadata.json', metadata)
        scene = self.write(root/'scenario.json', scenario)
        self.write(root/'audit.json', audit)
        lidar = root/'LIDAR_5'
        lidar.mkdir()
        center, rpy = [5., 0., 0.], [0., 0., 0.]
        corners = np.asarray(list(itertools.product((-1, 1), repeat=3)))*np.asarray(self.size)/2+center
        (lidar/'frame_Instance.txt').write_text(' '.join(['Obstacle', *map(str, center), *map(str, rpy),
            *map(str, self.size), '5', '0', '0', '0', str(uid)])+'\n')
        (lidar/'frame_Instance_8Points.txt').write_text(' '.join(['Obstacle', *map(str, corners.reshape(-1)),
            *map(str, rpy), '5', '0', '0', '0', str(uid)])+'\n')
        self.rows.append(dict(id=ident, capture_root=str(root), status='accepted_candidate',
            scenario_sha256=measure.sha256(scene), audit=audit))

    def persist(self):
        self.ledger = self.root/'ledger.jsonl'
        self.ledger.write_text(''.join(json.dumps(row)+'\n' for row in self.rows))
        self.write(self.root/'status.json', dict(stage='complete', error=None, finished=len(self.rows)))
        self.write(self.root/'shutdown_safety.json', dict(park_verified=True))

    def run_measure(self, path=None):
        old = dict(CargoBox=dict(bbox_size_m=[2., 4., 2.], pivot_to_bbox_center_local_m=[.2, .4, 1.]))
        with patch.object(measure, 'unit_geometry', return_value=old):
            return measure.measure(self.manifest, self.ledger, ['CargoBox'], path or self.root/'result.json')

    def mutate_metadata(self, change):
        path = Path(self.rows[0]['capture_root'])/'metadata.json'
        data = measure.read(path)
        change(data)
        self.write(path, data)

    def test_two_original_yaws_measure_native_size_and_explicit_proxy_without_raw_edits(self):
        before = {str(path):measure.sha256(path) for path in self.root.rglob('*') if path.is_file()}
        output = self.run_measure()
        result = measure.read(output)
        self.assertEqual(result['schema'], measure.SCHEMA)
        self.assertFalse(result['training_gt_modified'])
        self.assertFalse(result['production_approved'])
        asset = result['assets']['CargoBox']
        self.assertEqual(asset['observations'], 2)
        np.testing.assert_allclose(asset['bbox_size_m'], self.size)
        np.testing.assert_allclose(asset['pivot_to_bbox_center_local_m'], [.1, .2, .4])
        self.assertFalse(asset['method']['object_topic_used'])
        self.assertFalse(asset['method']['world_pivot_transform_used'])
        self.assertFalse(asset['method']['automatic_profile_approval'])
        self.assertIn('placement proxy', asset['method']['xy'])
        self.assertTrue(any(path.endswith('_Instance_8Points.txt') for path in asset['source_pins']))
        self.assertEqual(before, {path:measure.sha256(path) for path in before})

    def test_missing_second_yaw_fails_before_output(self):
        self.rows.pop()
        self.persist()
        with self.assertRaisesRegex(ValueError, 'Two accepted original yaws'):
            self.run_measure()
        self.assertFalse((self.root/'result.json').exists())

    def test_review_control_cannot_be_relabelled_as_a_reference(self):
        self.rows[0]['status'] = 'review'
        self.persist()
        with self.assertRaisesRegex(ValueError, 'must be accepted'):
            self.run_measure()

    def test_nonunit_original_is_rejected(self):
        self.mutate_metadata(lambda m:m['scale_probe'].update(requested_scale=[2., 2., 2.]))
        with self.assertRaisesRegex(ValueError, 'scale-1'):
            self.run_measure()

    def test_duplicate_yaw_is_not_two_observations(self):
        path = Path(self.rows[1]['capture_root'])
        meta = measure.read(path/'metadata.json')
        meta['scale_probe']['requested_yaw_deg'] = 0.
        meta['runtime_actors'][0]['placement_yaw_deg'] = 0.
        self.write(path/'metadata.json', meta)
        scene = measure.read(path/'scenario.json')
        scene['objectList'][0]['rot']['yaw'] = 0.
        self.write(path/'scenario.json', scene)
        self.rows[1]['scenario_sha256'] = measure.sha256(path/'scenario.json')
        self.persist()
        with self.assertRaisesRegex(ValueError, 'Duplicate original'):
            self.run_measure()

    def test_missing_exact_companion_fails_closed(self):
        (Path(self.rows[0]['capture_root'])/'LIDAR_5/frame_Instance_8Points.txt').unlink()
        with self.assertRaises(measure.native.BoxCornerContractError):
            self.run_measure()

    def test_changed_scene_and_ground_or_wrong_identity_are_rejected(self):
        self.mutate_metadata(lambda m:m['runtime_actors'][0]['map_ground_initializer'].update(usable=False))
        with self.assertRaisesRegex(ValueError, 'ground initializer'):
            self.run_measure()
        self.mutate_metadata(lambda m:m['runtime_actors'][0]['map_ground_initializer'].update(usable=True))
        self.mutate_metadata(lambda m:m['runtime_actors'][0].update(data_id=999))
        with self.assertRaisesRegex(ValueError, 'identity|asset/motion'):
            self.run_measure()

    def test_summary_dimensions_cannot_override_measured_corners(self):
        path = Path(self.rows[0]['capture_root'])/'LIDAR_5/frame_Instance.txt'
        fields = path.read_text().split()
        fields[7] = '2.'
        path.write_text(' '.join(fields)+'\n')
        with self.assertRaises(measure.native.BoxCornerContractError):
            self.run_measure()

    def test_accepted_audit_must_match_saved_audit(self):
        path = Path(self.rows[0]['capture_root'])/'audit.json'
        saved = measure.read(path)
        saved['components'][0]['size_m'][0] = 1.2
        self.write(path, saved)
        with self.assertRaisesRegex(ValueError, 'Saved control audit'):
            self.run_measure()

    def test_incomplete_session_or_unverified_park_rejected(self):
        self.write(self.root/'status.json', dict(stage='running', error=None, finished=2))
        with self.assertRaisesRegex(ValueError, 'Completed'):
            self.run_measure()
        self.persist()
        self.write(self.root/'shutdown_safety.json', dict(park_verified=False))
        with self.assertRaisesRegex(ValueError, 'Completed'):
            self.run_measure()

    def test_existing_output_and_duplicate_or_excluded_assets_rejected(self):
        with self.assertRaisesRegex(ValueError, 'New unit'):
            self.run_measure(self.manifest)
        with self.assertRaisesRegex(ValueError, 'Unique whitelisted'):
            measure.measure(self.manifest, self.ledger, ['OBJ_Hyundai_G80'], self.root/'out.json')


if __name__ == '__main__':
    unittest.main()
