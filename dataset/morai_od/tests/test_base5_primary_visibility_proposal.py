import copy
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_primary_visibility_proposal as proposal


def actor(x, y=0., size=(1., 1., .5), z=0., yaw=0., uid=1):
    return dict(unique_id=uid, data_id=uid, name='asset_' + str(uid), source_list='objectList',
        motion='static', geometry_reference_known=True, estimated_size=list(size),
        longitudinal_m=x, lateral_m=y, position=dict(z=z), placement_yaw_deg=yaw,
        map_ground_initializer=dict(usable=True))


def camera(y=0., z=1.5):
    # Camera optical axes: right=-Ego y, down=-Ego z, forward=Ego x.
    rotation = np.asarray([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = -rotation @ [0., y, z]
    return dict(ego_to_sensor=transform.tolist(), resolution_px=[896, 640],
        intrinsic=[[448., 0., 448.], [0., 448., 320.], [0., 0., 1.]])


def inputs():
    primary = actor(20.)
    foreground = actor(10., size=(3., 3., 3.), uid=2)
    foreground['source_list'] = 'vehicleList'
    meta = dict(ego=dict(z=0., yaw=0.), spec=dict(primary_obstacle_range=dict(
        semantic_class='obstacle', source_list='objectList', primary_data_id=1)),
        runtime_actors=[primary, foreground])
    lidar = np.eye(4)
    lidar[2, 3] = -1.5
    return meta, dict(cameras=dict(front=camera()), lidar=dict(ego_to_lidar=lidar.tolist()))


class ProposalTests(unittest.TestCase):
    def setUp(self):
        self.meta, self.cal = inputs()
        self.original = dict(score=3.25, primary=dict(lidar_beam_proxy=0,
            angular_occlusion_proxy=1., camera_min_extent_proxy_px=12.), marker='retained')
        self.calls = []

    def run_score(self):
        def original(meta, cal, kind):
            self.calls.append((meta, cal, kind))
            return self.original
        return proposal.score(self.meta, self.cal, 'primary_visibility', original)

    def test_complete_single_foreground_rejects_without_mutation(self):
        snapshots = copy.deepcopy((self.meta, self.cal, self.original))
        result = self.run_score()
        self.assertEqual(result['score'], -math.inf)
        proof = result['primary_visibility_proposal']
        self.assertTrue(proof['rejected'])
        self.assertEqual(proof['foreground']['blocked_samples_by_sensor'], dict(lidar=27, front=27))
        self.assertEqual(len(self.calls), 1)
        self.assertEqual((self.meta, self.cal, self.original), snapshots)

    def test_unplanned_and_nonphysical_rows_are_exact_legacy_object(self):
        for mutation in ('no_plan', 'pedestrian', 'moving', 'wrong_identity'):
            self.meta, self.cal = inputs()
            if mutation == 'no_plan': self.meta['spec'].clear()
            elif mutation == 'pedestrian': self.meta['runtime_actors'][0]['source_list'] = 'pedestrianList'
            elif mutation == 'moving': self.meta['runtime_actors'][0]['motion'] = 'dynamic'
            else: self.meta['spec']['primary_obstacle_range']['primary_data_id'] = 22
            self.assertIs(self.run_score(), self.original)

    def test_predicted_lidar_beam_retains_even_extreme_angular_overlap(self):
        self.original['primary']['lidar_beam_proxy'] = 1
        result = self.run_score()
        self.assertEqual(result['score'], 3.25)
        self.assertFalse(result['primary_visibility_proposal']['rejected'])

    def test_partial_lateral_occlusion_is_retained(self):
        self.meta['runtime_actors'][1]['lateral_m'] = 1.5
        result = self.run_score()
        self.assertTrue(math.isfinite(result['score']))
        self.assertFalse(result['primary_visibility_proposal']['rejected'])

    def test_tall_primary_visible_above_low_foreground_is_retained(self):
        self.meta['runtime_actors'][0]['estimated_size'][2] = 5.
        self.meta['runtime_actors'][1]['estimated_size'][2] = 1.
        result = self.run_score()
        self.assertEqual(result['score'], 3.25)
        self.assertFalse(result['primary_visibility_proposal']['rejected'])

    def test_camera_parallax_prevents_rejection(self):
        self.cal['cameras']['offset'] = camera(y=5.)
        result = self.run_score()
        proof = result['primary_visibility_proposal']
        self.assertFalse(proof['rejected'])
        self.assertEqual(proof['sensor_names'], ['lidar', 'front', 'offset'])
        counts = proof['foreground_candidates'][0]['blocked_samples_by_sensor']
        self.assertEqual(counts['front'], 27)
        self.assertLess(counts['offset'], 27)

    def test_no_union_of_different_foregrounds(self):
        self.meta['runtime_actors'][0]['estimated_size'][1] = 4.
        self.meta['runtime_actors'][1].update(lateral_m=-.6, estimated_size=[3., 1.4, 3.])
        second = actor(10., y=.6, size=(3., 1.4, 3.), uid=3)
        second['source_list'] = 'vehicleList'
        self.meta['runtime_actors'].append(second)
        proof = self.run_score()['primary_visibility_proposal']
        self.assertFalse(proof['rejected'])
        self.assertEqual(len(proof['foreground_candidates']), 2)
        for candidate in proof['foreground_candidates']:
            self.assertLess(candidate['blocked_samples_by_sensor']['front'], 27)

    def test_compound_or_uncertain_primary_height_is_retained(self):
        for mutation in ('compound', 'unknown', 'ground', 'height', 'missing'):
            self.meta, self.cal = inputs()
            primary = self.meta['runtime_actors'][0]
            if mutation == 'compound': primary['component_count'] = 2
            elif mutation == 'unknown': primary['geometry_reference_known'] = False
            elif mutation == 'ground': primary['map_ground_initializer']['usable'] = False
            elif mutation == 'height': primary['static_height_initializer'] = dict(usable=False)
            else: primary.pop('estimated_size')
            result = self.run_score()
            self.assertEqual(result['score'], 3.25)
            self.assertFalse(result['primary_visibility_proposal']['rejected'])

    def test_uncertain_foreground_is_not_a_blocker(self):
        for mutation in ('compound', 'unknown', 'moving', 'pedestrian', 'ground', 'object', 'small'):
            self.meta, self.cal = inputs()
            foreground = self.meta['runtime_actors'][1]
            if mutation == 'compound': foreground['is_compound'] = True
            elif mutation == 'unknown': foreground['geometry_reference_known'] = False
            elif mutation == 'moving': foreground['motion'] = 'dynamic'
            elif mutation == 'pedestrian': foreground['source_list'] = 'pedestrianList'
            elif mutation == 'object': foreground['source_list'] = 'objectList'
            elif mutation == 'small': foreground['estimated_size'] = [2.9, 3., 3.]
            else: foreground['map_ground_initializer']['usable'] = False
            self.assertFalse(self.run_score()['primary_visibility_proposal']['rejected'])

    def test_foreground_behind_primary_is_not_a_blocker(self):
        self.meta['runtime_actors'][1]['longitudinal_m'] = 30.
        self.assertFalse(self.run_score()['primary_visibility_proposal']['rejected'])

    def test_lidar_parallax_also_prevents_rejection(self):
        self.cal['lidar']['ego_to_lidar'][1][3] = -5.
        proof = self.run_score()['primary_visibility_proposal']
        self.assertFalse(proof['rejected'])
        self.assertEqual(proof['foreground_candidates'][0]['blocked_samples_by_sensor']['front'], 27)
        self.assertLess(proof['foreground_candidates'][0]['blocked_samples_by_sensor']['lidar'], 27)

    def test_relative_yaw_and_ego_height_are_used(self):
        self.meta['ego'].update(z=28., yaw=90.)
        for actor in self.meta['runtime_actors']:
            actor['position']['z'] += 28.
            actor['placement_yaw_deg'] += 90.
        self.assertTrue(self.run_score()['primary_visibility_proposal']['rejected'])

    def test_shrunk_foreground_does_not_cover_margin_only(self):
        self.meta['runtime_actors'][0]['estimated_size'][1] = 6.62
        self.assertFalse(self.run_score()['primary_visibility_proposal']['rejected'])

    def test_no_camera_or_crop_witness_does_not_certify_occlusion(self):
        self.meta['runtime_actors'][0]['position']['z'] = 30.
        result = self.run_score()
        self.assertFalse(result['primary_visibility_proposal']['rejected'])
        self.assertEqual(result['primary_visibility_proposal']['reason'], 'no_in_crop_camera_witness')

    def test_invalid_geometry_and_calibration_fail_closed(self):
        for mutation in ('size', 'yaw', 'matrix', 'fov', 'score'):
            self.meta, self.cal = inputs()
            self.original['score'] = 3.25
            if mutation == 'size': self.meta['runtime_actors'][0]['estimated_size'][0] = math.nan
            elif mutation == 'yaw': self.meta['runtime_actors'][0]['placement_yaw_deg'] = math.inf
            elif mutation == 'matrix': self.cal['cameras']['front']['ego_to_sensor'][0][0] = math.nan
            elif mutation == 'fov': self.cal['cameras']['front']['resolution_px'][0] = 0
            else: self.original['score'] = math.nan
            with self.assertRaises(RuntimeError): self.run_score()

    def test_slab_parallel_and_sensor_inside_are_safe(self):
        self.meta['runtime_actors'][1].update(longitudinal_m=0., position=dict(z=0.),
            estimated_size=[4., 4., 4.])
        self.assertFalse(self.run_score()['primary_visibility_proposal']['rejected'])

    def test_near_plane_crossing_retains_uncertainty(self):
        self.meta['runtime_actors'][0].update(longitudinal_m=.1, estimated_size=[1., 1., .5])
        result = self.run_score()
        self.assertFalse(result['primary_visibility_proposal']['rejected'])
        self.assertEqual(result['primary_visibility_proposal']['reason'], 'uncertain_camera_near_plane')


if __name__ == '__main__':
    unittest.main()
