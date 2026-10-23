"""CPU contracts only; never construct ROS, load scenarios or use a GPU."""
import copy
from collections import Counter
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_compound_diagnostic as probe


class CompoundDiagnosticTest(unittest.TestCase):
    def base(self):
        return dict(family='pedestrian_related', static_yaw_policy=dict(
            policy='static-free-sector-yaw-v130', scope='generic',
            eligible_data_ids=[42], primary_mode='uniform', primary_sector=1),
            spatial_sampling_plan=dict(split='train', policy='original-allowed-plan'),
            avoidance_phase='passing', dynamic_observation_phase='yielding',
            dynamic_mission_pool='person', stationary_mission_pool='stroller',
            frozen_population_data_ids=[1, 2], population_budget_bounds=[8, 20],
            required_secondary_pool='vehicle', original_family='obstacle',
            shadow_moving=True, crossing_direction_sign=-1)

    def test_factorial_and_contrast_coverage_stays_small(self):
        conditions = probe.specifications()
        self.assertEqual(len(conditions), 22)
        strollers = [c for c in conditions if c['pool'] == 'stroller']
        self.assertEqual(len(strollers), 18)
        self.assertEqual(Counter(c['view'] for c in strollers), dict(front=6, side=6, rear=6))
        for asset in probe.STROLLERS:
            self.assertEqual({(c['distance_m'], c['view']) for c in strollers if c['asset'] == asset},
                {(d, v) for d in (6, 12) for v in ('front', 'side', 'rear')})
        self.assertEqual(Counter(c['pool'] for c in conditions), dict(stroller=18, pet_person=2, rider=2))

    def test_new_single_static_contract_does_not_mutate_source(self):
        source = self.base()
        before = copy.deepcopy(source)
        row = probe.make_row(source, probe.specifications()[0], 42, 0)
        self.assertEqual(source, before)
        self.assertEqual(row['actors'], 1)
        self.assertEqual(row['population_max_actors'], 1)
        self.assertEqual(row['population_mode'], 'generic_single')
        self.assertEqual(row['motion'], 'static')
        self.assertFalse(row['mission'])
        self.assertEqual(row['primary_pool_override'], 'stroller')
        self.assertEqual(row['primary_identity'], 42)
        self.assertEqual(row['distance_bounds'], [5.25, 6.75])
        self.assertEqual(row['spatial_sampling_plan'], before['spatial_sampling_plan'])
        for key in ('avoidance_phase', 'dynamic_observation_phase', 'dynamic_mission_pool',
                    'stationary_mission_pool', 'frozen_population_data_ids',
                    'population_budget_bounds', 'required_secondary_pool', 'original_family',
                    'shadow_moving', 'crossing_direction_sign'):
            self.assertNotIn(key, row)

    def test_nominal_angle_and_actual_sector_are_distinct(self):
        for index, view in enumerate(('front', 'side', 'rear')):
            condition = dict(asset='BabyStroller1', pool='stroller', distance_m=6, view=view)
            row = probe.make_row(self.base(), condition, 42, index)
            contract = row['compound_diagnostic_contract']
            self.assertEqual(contract['nominal_road_relative_yaw_deg'], {'front': 180., 'side': 90., 'rear': 0.}[view])
            self.assertEqual(row['static_yaw_policy']['primary_sector'], {'front': 11, 'side': 8, 'rear': 6}[view])
            self.assertFalse(contract['training'])

    def test_ineligible_asset_or_legacy_yaw_policy_rejected(self):
        with self.assertRaisesRegex(ValueError, 'current compound'):
            probe.make_row(self.base(), probe.specifications()[0], 43, 0)
        base = self.base()
        base['static_yaw_policy']['policy'] = 'legacy'
        with self.assertRaisesRegex(ValueError, 'current compound'):
            probe.make_row(base, probe.specifications()[0], 42, 0)

    def meta(self, yaw):
        return dict(ego=dict(x=0., y=0., yaw=0.), runtime_actors=[dict(
            geometric_center_estimate=[6., 0.], placement_yaw_deg=yaw,
            static_yaw_policy=dict(road_relative_yaw_deg=yaw))])

    def test_actual_view_uses_observer_geometry_not_road_yaw_label(self):
        for view, yaw in [('front', 180.), ('side', 90.), ('rear', 0.)]:
            proof = probe.view_proof(self.meta(yaw), dict(view=view))
            self.assertAlmostEqual(proof['nominal_view_error_deg'], 0.)
            self.assertTrue(proof['within_nominal_view'])
            self.assertAlmostEqual(proof['actual_center_distance_m'], 6.)
            self.assertAlmostEqual(proof['actual_bearing_from_ego_deg'], 0.)
        self.assertFalse(probe.view_proof(self.meta(0.), dict(view='front'))['within_nominal_view'])

    def test_view_angle_is_rotation_invariant(self):
        meta = self.meta(180.)
        meta['runtime_actors'][0]['geometric_center_estimate'] = [0., 6.]
        meta['runtime_actors'][0]['placement_yaw_deg'] = -90.
        meta['ego']['yaw'] = 90.
        proof = probe.view_proof(meta, dict(view='front'))
        self.assertAlmostEqual(proof['nominal_view_error_deg'], 0.)
        self.assertAlmostEqual(proof['actual_bearing_from_ego_deg'], 0.)

    def test_bad_observation_geometry_fails_closed(self):
        meta = self.meta(0.)
        meta['runtime_actors'][0]['geometric_center_estimate'] = [0., 0.]
        with self.assertRaisesRegex(ValueError, 'Coincident'):
            probe.view_proof(meta, dict(view='front'))
        with self.assertRaisesRegex(ValueError, 'Unknown diagnostic view'):
            probe.view_proof(self.meta(0.), dict(view='invalid'))

    def test_existing_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                probe.prepare(Path(directory)/'not-read.json', Path(directory))


if __name__ == '__main__':
    unittest.main()
