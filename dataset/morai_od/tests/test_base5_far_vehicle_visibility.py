import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_far_vehicle_visibility as far


def actor(uid, x, size, offset_z):
    return dict(unique_id=uid, data_id=uid, name='car_'+str(uid), source_list='vehicleList',
        motion='static', geometry_reference_known=True, estimated_size=list(size),
        position=dict(z=0.), longitudinal_m=x, lateral_m=0., placement_yaw_deg=0.,
        static_height_initializer=dict(usable=True), map_ground_initializer=dict(
            usable=True, footprint=dict(all_supported=True)))


def camera(y=0., z=1.5):
    rotation = np.asarray([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation@[0., y, z]
    return dict(ego_to_sensor=transform.tolist(), intrinsic=[[528., 0., 528.],
        [0., 528., 320.], [0., 0., 1.]], resolution_px=[1056, 640])


def fixture():
    blocker = actor(1, 10., [4., 4., 4.], 1.5)
    target = actor(2, 20., [4., 2., 1.5], .75)
    meta = dict(spec=dict(secondary_far_vehicle=dict(target_uid=2, target_data_id=2,
        target_name='car_2', target_ordinal=1)), runtime_actors=[blocker, target],
        ego=dict(z=0., yaw=0.))
    reference = {a['name']: dict(**{'class': 'vehicle'}, observations=10,
        bbox_size_m=a['estimated_size'], pivot_to_bbox_center_local_m=[0., 0., z])
        for a, z in ((blocker, 1.5), (target, .75))}
    lidar = np.eye(4); lidar[2, 3] = -1.5
    return meta, dict(cameras=dict(front=camera()), lidar=dict(ego_to_lidar=lidar.tolist())), reference


class FarVisibilityTest(unittest.TestCase):
    def setUp(self):
        self.meta, self.cal, self.ref = fixture()

    def evaluate(self):
        return far.evaluate(self.meta, self.cal, self.ref)

    def test_full_single_vehicle_blocker_and_nonmutating_score(self):
        snapshot = copy.deepcopy((self.meta, self.cal, self.ref))
        original = dict(score=5., primary=dict(lidar_beam_proxy=1))
        result = far.score(self.meta, self.cal, 'all_actor_visibility', lambda *a: original, self.ref)
        self.assertEqual(result['score'], -math.inf)
        self.assertEqual(result['far_vehicle_visibility']['foreground']['blocked_samples_by_sensor'],
                         dict(lidar=27, front=27))
        self.assertEqual((self.meta, self.cal, self.ref), snapshot)
        self.assertEqual(original['score'], 5.)

    def test_uses_reference_z_offset_instead_of_half_height(self):
        self.ref['car_2']['pivot_to_bbox_center_local_m'][2] = .2
        self.assertAlmostEqual(self.evaluate()['target_center_ego_m'][2], .2)

    def test_target_identity_survives_target_first_ranking_view(self):
        proof = self.evaluate()
        self.meta['runtime_actors'].reverse()
        self.assertEqual(self.evaluate(), proof)

    def test_legacy_source_returns_same_score_object(self):
        self.meta['spec'].clear()
        original = dict(score=1.)
        self.assertIs(far.score(self.meta, {}, '', lambda *a: original), original)
        self.assertFalse(self.evaluate()['applicable'])

    def test_partial_lateral_and_tall_target_are_kept(self):
        self.meta['runtime_actors'][0]['lateral_m'] = 2.5
        self.assertFalse(self.evaluate()['rejected'])
        self.meta, self.cal, self.ref = fixture()
        self.meta['runtime_actors'][1]['estimated_size'][2] = 8.
        self.ref['car_2']['bbox_size_m'][2] = 8.
        self.ref['car_2']['pivot_to_bbox_center_local_m'][2] = 4.
        self.assertFalse(self.evaluate()['rejected'])

    def test_sensor_parallax_keeps_target(self):
        self.cal['cameras']['offset'] = camera(y=8.)
        proof = self.evaluate()
        self.assertFalse(proof['rejected'])
        self.assertIn('offset', proof['sensor_names'])
        self.assertEqual(proof['foreground_candidates'][0]['blocked_samples_by_sensor']['front'], 27)
        self.assertLess(proof['foreground_candidates'][0]['blocked_samples_by_sensor']['offset'], 27)

    def test_unknown_height_size_or_compound_target_stays_eligible(self):
        for mutation in ('missing_ref', 'missing_z', 'different_size', 'unknown', 'height', 'compound', 'moving'):
            self.meta, self.cal, self.ref = fixture()
            target = self.meta['runtime_actors'][1]
            if mutation == 'missing_ref': self.ref.pop('car_2')
            elif mutation == 'missing_z': self.ref['car_2'].pop('pivot_to_bbox_center_local_m')
            elif mutation == 'different_size': self.ref['car_2']['bbox_size_m'] = [8., 2., 1.5]
            elif mutation == 'unknown': target['geometry_reference_known'] = False
            elif mutation == 'height': target['static_height_initializer']['usable'] = False
            elif mutation == 'compound': target['component_count'] = 2
            else: target['motion'] = 'walking'
            self.assertFalse(self.evaluate()['rejected'])

    def test_nonvehicle_unknown_or_compound_foreground_is_not_proof(self):
        for mutation in ('pedestrian', 'compound', 'unknown', 'small'):
            self.meta, self.cal, self.ref = fixture()
            blocker = self.meta['runtime_actors'][0]
            if mutation == 'pedestrian': blocker['source_list'] = 'pedestrianList'
            elif mutation == 'compound': blocker['is_compound'] = True
            elif mutation == 'unknown': blocker['geometry_reference_known'] = False
            else:
                blocker['estimated_size'][0] = 2.9
                self.ref['car_1']['bbox_size_m'][0] = 2.9
            self.assertFalse(self.evaluate()['rejected'])

    def test_current_512_crop_includes_a_box_above_old_320_crop(self):
        corners = np.asarray([[20.+x, y, 3.+z] for x in (-1, 1)
                              for y in (-1, 1) for z in (-.1, .1)])
        origins, names, uncertain = far._origins512(corners, self.cal)
        self.assertEqual(names, ['front'])
        self.assertFalse(uncertain)

    def test_genuine_near_plane_ambiguity_is_kept(self):
        self.meta['runtime_actors'][1]['longitudinal_m'] = .1
        proof = self.evaluate()
        self.assertFalse(proof['rejected'])
        self.assertEqual(proof['reason'], 'uncertain_camera_near_plane')

    def test_camera_that_cannot_see_target_does_not_create_near_plane_ambiguity(self):
        side = camera()
        rotation = np.asarray([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        transform = np.eye(4); transform[:3, :3] = rotation
        transform[:3, 3] = -rotation@[0., 0., 1.5]
        side['ego_to_sensor'] = transform.tolist()
        self.cal['cameras']['left'] = side
        self.assertTrue(self.evaluate()['rejected'])
        self.assertNotIn('left', self.evaluate()['sensor_names'])

    def test_invalid_finite_geometry_and_calibration_halt(self):
        self.ref['car_2']['pivot_to_bbox_center_local_m'][2] = math.nan
        with self.assertRaises(RuntimeError): self.evaluate()
        self.meta, self.cal, self.ref = fixture()
        self.cal['cameras']['front']['ego_to_sensor'][0][0] = math.nan
        with self.assertRaises(RuntimeError): self.evaluate()

    def test_policy_pins_each_dependency_and_normalized_calibration(self):
        policy = far.configuration()
        self.assertEqual(far.validate_policy(policy)['source_sha256'], policy['sensor']['sha256'])
        for key in ('implementation', 'ray_helper', 'geometry_reference', 'sensor', 'normalizer'):
            modified = copy.deepcopy(policy)
            modified[key]['sha256'] = 'changed'
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                far.validate_policy(modified)
        with self.assertRaises(RuntimeError): far.validate_policy(policy, {})

    def test_policy_detects_reference_file_tamper_and_caches_normalization_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            reference = Path(tmp)/'geometry.json'
            reference.write_bytes(far.DEFAULT_GEOMETRY.read_bytes())
            policy = far.configuration(geometry_reference=reference)
            far.validate_policy(policy)
            reference.write_text('{}')
            with self.assertRaises(RuntimeError): far.validate_policy(policy)
        far._calibration.cache_clear()
        with patch.object(far.sensor_metadata, 'normalized_sensor_calibration',
                          wraps=far.sensor_metadata.normalized_sensor_calibration) as normalize:
            policy = far.configuration()
            far.validate_policy(policy)
            far.validate_policy(policy)
        self.assertEqual(normalize.call_count, 1)


if __name__ == '__main__':
    unittest.main()
