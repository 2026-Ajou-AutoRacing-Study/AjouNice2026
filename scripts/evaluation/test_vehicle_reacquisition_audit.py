#!/usr/bin/env python3
"""Small deterministic checks for native-replay diagnostic compatibility."""
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from audit_tracking_continuity_events import replay
from audit_vehicle_reacquisition_core import pair_gate
from qualify_vehicle_innovation_gate import noisy_cases, synthetic_cases


class NativeGateEvidenceTests(unittest.TestCase):
    def test_recorded_sonata_rejected_by_covariance_not_cartesian_limit(self):
        prior = dict(center=[198.64391246002828, 664.2178083890016],
                     covariance=[.20744295789667605, -.0006738967329588231, .2174247222360896])
        observation = dict(center=[198.39507261273104, 661.692541488753])
        result = pair_gate(prior, observation)
        self.assertTrue(result['euclidean_admitted'])
        self.assertFalse(result['covariance_admitted'])
        self.assertAlmostEqual(result['distance_m'], 2.537497623071552)
        self.assertAlmostEqual(result['mahalanobis'], 5.44491881138787)

    def test_hard_distance_gate_is_not_overruled_by_large_covariance(self):
        result = pair_gate(dict(center=[0., 0.], covariance=[100., 0., 100.]),
                           dict(center=[5.01, 0.]))
        self.assertTrue(result['covariance_admitted'])
        self.assertFalse(result['native_eligible'])

    def test_exact_five_meter_limit_retains_existing_boundary_contract(self):
        result = pair_gate(dict(center=[0., 0.], covariance=[100., 0., 100.]),
                           dict(center=[5., 0.]))
        self.assertTrue(result['native_eligible'])

    def test_proposed_innovation_covariance_rescues_same_sonata_measurement(self):
        prior = dict(center=[198.64391246002828, 664.2178083890016],
                     covariance=[.20744295789667605, -.0006738967329588231, .2174247222360896])
        observation = dict(center=[198.39507261273104, 661.692541488753], score=.8407)
        result = pair_gate(prior, observation, measurement_variance=.16)
        self.assertTrue(result['native_eligible'])
        self.assertLess(result['mahalanobis'], 4.2)

    def test_proposed_covariance_cannot_admit_separate_vehicle_outside_limit(self):
        result = pair_gate(dict(center=[0., 0.], covariance=[100., 0., 100.]),
                           dict(center=[7., 0.], score=.45), measurement_variance=.16)
        self.assertFalse(result['native_eligible'])
        self.assertFalse(result['euclidean_admitted'])

    def test_invalid_covariance_or_variance_does_not_fabricate_eligibility(self):
        with self.assertRaises(ValueError):
            pair_gate(dict(center=[0., 0.], covariance=[0., 0., 0.]), dict(center=[0., 0.]))
        with self.assertRaises(ValueError):
            pair_gate(dict(center=[0., 0.], covariance=[1., 0., 1.]), dict(center=[0., 0.]),
                      measurement_variance=float('nan'))


class SyntheticCounterexampleTests(unittest.TestCase):
    def test_clean_parallel_and_opposite_lanes_are_not_one_actor(self):
        cases = synthetic_cases()
        self.assertEqual(len(cases), 9)
        for frames, gt in cases.values():
            self.assertEqual(len(gt['gt']), 2)
            for series in gt['gt']:
                self.assertEqual(len(series[0]), len(series[1]))
            self.assertEqual(len(frames), 65)

    def test_noisy_measurements_do_not_mutate_clean_gt(self):
        cases = noisy_cases()
        self.assertEqual(len(cases), 13)
        frames, gt = cases['noisy_0.65_drop0.25_seed23']
        self.assertEqual(gt['gt'][0][1][0]['center'], [0., 0., .8])
        self.assertNotEqual(frames[0]['world_objects'][0]['center'], gt['gt'][0][1][0]['center'])

    def test_noise_seed_and_dropout_are_reproducible(self):
        self.assertEqual(noisy_cases(), noisy_cases())

    def test_outside_limit_new_birth_has_its_own_short_gt_timeline(self):
        _, gt = noisy_cases()['independent_birth_outside_5m']
        self.assertEqual(len(gt['gt'][0][0]), 45)
        self.assertEqual(len(gt['gt'][1][0]), 20)
        self.assertEqual(gt['gt'][1][0][0], 102.5)
        self.assertEqual(gt['gt'][1][1][0]['key'], 'new')


class NativeTraceCompatibilityTests(unittest.TestCase):
    OLD_TRACK = 'TRACK 0 1 1 1 2 3 0.8 4 5 0.1 4.5 1.8 1.6 0.8 100 3\n'

    def test_old_frozen_binary_keeps_old_rows_exactly(self):
        with patch('audit_tracking_continuity_events.subprocess.run',
                   return_value=SimpleNamespace(stdout='FRAME 100 1\n' + self.OLD_TRACK)) as run:
            result = replay(Path('/tmp/frozen'), [dict(stamp=100., world_objects=[])])
        self.assertNotIn('yaw_rate', result[0]['tracks'][0])
        self.assertEqual(run.call_args[0][0], ['/tmp/frozen'])

    def test_extended_diagnostics_and_optional_binary_args(self):
        line = self.OLD_TRACK.rstrip() + ' 0.2 99.5\n'
        with patch('audit_tracking_continuity_events.subprocess.run',
                   return_value=SimpleNamespace(stdout='FRAME 100 1\n' + line)) as run:
            result = replay(Path('/tmp/probe'), [dict(stamp=100., world_objects=[])], extra_args=(1, 'on'))
        self.assertEqual(result[0]['tracks'][0]['yaw_rate'], .2)
        self.assertEqual(result[0]['tracks'][0]['initialization_time'], 99.5)
        self.assertEqual(run.call_args[0][0], ['/tmp/probe', '1', 'on'])


if __name__ == '__main__':
    unittest.main()
