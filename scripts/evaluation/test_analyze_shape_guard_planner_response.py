#!/usr/bin/env python3
"""Pure-Python guardrails for fixed-input actual-Cruise qualification."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import hashlib

SPEC = importlib.util.spec_from_file_location('planner_response', Path(__file__).with_name(
    'analyze_shape_guard_planner_response.py'))
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def row(stamp=1., target='a', velocity=5.):
    return dict(index=0, decision_stamp=stamp,
                selection=dict(candidate_uuids=[target], selected_uuids=[target]),
                limit=dict(max_velocity=velocity), clear_limit=False)


class FixedInputContracts(unittest.TestCase):
    def test_latest_never_looks_ahead(self):
        self.assertIsNone(analysis.latest_index([2, 4], 1))
        self.assertEqual(analysis.latest_index([2, 4], 3), 0)

    def test_exact_receipt_is_legal(self):
        self.assertEqual(analysis.latest_index([2, 4], 4), 1)

    def test_duplicate_receipt_takes_last_received(self):
        self.assertEqual(analysis.latest_index([2, 2, 4], 2), 1)

    def test_empty_history(self):
        self.assertIsNone(analysis.latest_index([], 1))

    def test_aa_requires_entire_rows(self):
        a, b = row(), row()
        self.assertTrue(analysis.compare_rows([a], [b])['exact_rows'])
        b['selection']['diagnostic_only'] = True
        result = analysis.compare_rows([a], [b])
        self.assertFalse(result['exact_rows'])
        self.assertEqual(result['changed'], {})

    def test_target_change_without_velocity_change_is_counted(self):
        result = analysis.compare_rows([row()], [row(target='b')])
        self.assertEqual(result['changed'], {'candidate_set': 1, 'selected_target': 1})

    def test_velocity_change_without_target_change_is_counted(self):
        self.assertEqual(analysis.compare_rows([row()], [row(velocity=4.)])['changed'],
                         {'velocity_limit': 1})

    def test_cohort_size_mismatch_fails(self):
        with self.assertRaises(ValueError):
            analysis.compare_rows([row()], [])

    def test_cohort_clock_mismatch_fails(self):
        with self.assertRaises(ValueError):
            analysis.compare_rows([row()], [row(stamp=2.)])

    def test_write_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'report.json'
            analysis.write_json(path, {'value': 1})
            with self.assertRaises(FileExistsError):
                analysis.write_json(path, {'value': 2})

    def test_nan_report_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaises(ValueError):
                analysis.write_json(Path(temp) / 'report.json', {'value': float('nan')})

    def test_master_is_isolated(self):
        port, env = analysis.private_environment(Path('/tmp/shape_guard_test'))
        self.assertNotEqual(port, 11311)
        self.assertEqual(env['ROS_MASTER_URI'], 'http://127.0.0.1:%d' % port)
        self.assertEqual(env['ROS_HOME'], '/tmp/shape_guard_test/ros_home')

    def test_native_harness_calls_real_plan(self):
        self.assertIn('module.plan(trajectory->points,trajectory->points,data)', analysis.HARNESS)
        self.assertNotIn('/ctrl_cmd', analysis.HARNESS)

    def test_aa_gate_rejects_different_prediction_payload(self):
        report = dict(modes=dict(a=dict(predicted_input_sha256='x', fixed_input_sha256='p'),
                                 b=dict(predicted_input_sha256='y', fixed_input_sha256='p')))
        with self.assertRaisesRegex(ValueError, 'prediction payload'):
            analysis.validate_aa(report, {'exact_rows': True}, 'a', 'b')

    def test_aa_gate_rejects_nonexact_actual_response(self):
        mode = dict(predicted_input_sha256='x', fixed_input_sha256='p')
        with self.assertRaisesRegex(ValueError, 'actual Cruise response'):
            analysis.validate_aa(dict(modes=dict(a=mode, b=mode)), {'exact_rows': False}, 'a', 'b')

    def test_aa_gate_rejects_missing_mode(self):
        with self.assertRaisesRegex(ValueError, 'absent'):
            analysis.validate_aa(dict(modes={}), {'exact_rows': True}, 'a', 'b')

    def test_aa_gate_accepts_exact_actual_response(self):
        mode = dict(predicted_input_sha256='x', fixed_input_sha256='p')
        self.assertTrue(analysis.validate_aa(dict(modes=dict(a=mode, b=mode)),
            {'exact_rows': True}, 'a', 'b')['actual_cruise_response_exact'])

    def test_build_rejects_binary_tampering(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / 'cruise_harness').write_bytes(b'changed')
            analysis.write_json(directory / 'build_receipt.json', {'binary_sha256': 'original'})
            with self.assertRaisesRegex(ValueError, 'binary SHA'):
                analysis.verify_build(directory)

    def test_build_rejects_stale_source(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / 'cruise_harness').write_bytes(b'binary')
            (directory / 'cruise_harness.cpp').write_text(analysis.HARNESS)
            analysis.write_json(directory / 'build_receipt.json', dict(
                binary_sha256=analysis.sha(directory / 'cruise_harness'),
                harness_sha256=hashlib.sha256(analysis.HARNESS.encode()).hexdigest(),
                source_sha256={'source': 'old'}, dependency_sha256={}))
            with mock.patch.object(analysis, 'source_receipt', return_value={'source': 'new'}):
                with self.assertRaisesRegex(ValueError, 'different planner sources'):
                    analysis.verify_build(directory)

    def test_candidate_set_ignores_order_but_aa_does_not(self):
        a, b = row(), row()
        a['selection']['candidate_uuids'] = ['a', 'b']
        b['selection']['candidate_uuids'] = ['b', 'a']
        result = analysis.compare_rows([a], [b])
        self.assertEqual(result['changed'], {})
        self.assertFalse(result['exact_rows'])

    def test_distribution_linear_quantiles(self):
        result = analysis.distribution([3., 0., 2., 1.])
        self.assertEqual(result['mean'], 1.5)
        self.assertEqual(result['p50'], 1.5)
        self.assertAlmostEqual(result['p90'], 2.7)
        self.assertAlmostEqual(result['p95'], 2.85)

    def test_absent_limits_are_zero_change_not_zero_speed(self):
        a, b = row(), row()
        a['limit'] = b['limit'] = None
        result = analysis.velocity_delta_distributions([a], [b])
        self.assertEqual(result['all_decision_cycles']['signed_mps']['count'], 1)
        self.assertEqual(result['all_decision_cycles']['signed_mps']['mean'], 0.)
        self.assertEqual(result['both_limited_cycles']['signed_mps']['count'], 0)

    def test_limit_presence_transition_has_no_fabricated_delta(self):
        a, b = row(), row()
        b['limit'] = None
        result = analysis.velocity_delta_distributions([a], [b])
        self.assertEqual(result['limit_presence_change_cycles'], 1)
        self.assertEqual(result['all_decision_cycles']['signed_mps']['count'], 0)

    def test_changed_cycle_signed_and_absolute_are_separate(self):
        result = analysis.velocity_delta_distributions([row(velocity=5.)], [row(velocity=4.)])
        self.assertEqual(result['changed_velocity_limit_cycles']['signed_mps']['mean'], -1.)
        self.assertEqual(result['changed_velocity_limit_cycles']['absolute_mps']['mean'], 1.)


if __name__ == '__main__':
    unittest.main()
