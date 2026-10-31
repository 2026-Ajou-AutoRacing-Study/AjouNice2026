import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_far_vehicle_spread as spread


class FarSpreadTest(unittest.TestCase):
    def rows(self):
        return {str(i): dict(spec=dict(secondary_far_vehicle=dict(
            role='front' if i < 7 else 'rear', sampling_bounds_m=[97, 99]))) for i in range(14)}

    def test_full_stratum_covers_both_halves(self):
        result = spread.assignments(self.rows())
        for role in ('front', 'rear'):
            values = sorted(r['target_distance_ego_xy_m'] for r in result.values() if r['role'] == role)
            self.assertEqual(len(values), 7)
            self.assertLess(values[0], 97.2)
            self.assertGreater(values[-1], 98.8)
            self.assertTrue(all(97 <= v < 99 for v in values))

    def test_assignments_are_order_independent_without_input_mutation(self):
        rows = self.rows()
        before = copy.deepcopy(rows)
        self.assertEqual(spread.assignments(rows), spread.assignments(dict(reversed(list(rows.items())))))
        self.assertEqual(rows, before)

    def test_non_far_and_other_windows_do_not_change_existing_group(self):
        rows = self.rows()
        expected = spread.assignments(rows)
        rows['legacy'] = dict(spec={})
        rows['other'] = dict(spec=dict(secondary_far_vehicle=dict(role='front', sampling_bounds_m=[60, 80])))
        actual = spread.assignments(rows)
        self.assertEqual({k:actual[k] for k in expected}, expected)

    def test_nearest_distance_not_nearest_range_wins(self):
        candidates = [(None, None, None, None, d) for d in (97.1, 97.6, 98.1, 98.6)]
        with patch.object(spread.far, '_draw_index', side_effect=lambda p, a, n:a%n):
            ordered = spread.proposal_order({}, candidates, 98.8)
        self.assertEqual([i for i, _ in ordered], [3, 2, 1, 0])

    def test_repeated_draws_are_deduplicated_and_bounded(self):
        candidates = [(None, None, None, None, d) for d in (97.1, 98.6)]
        with patch.object(spread.far, '_draw_index', return_value=0) as draw:
            result = spread.proposal_order({}, candidates, 98.8, maximum=9)
        self.assertEqual(result, [(0, 0)])
        self.assertEqual(draw.call_count, 9)

    def test_empty_candidate_set_is_explicit_geometry_exhaustion(self):
        with self.assertRaises(spread.far.FarGeometryExhausted):
            spread.proposal_order({}, [], 98.)

    def transition(self):
        assignment = dict(role='front', sampling_bounds_m=[97, 99],
                          target_distance_ego_xy_m=98.5, rank=3, group_count=7, quantile=.5)
        before = dict(spec=dict(seed=1), ego=dict(x=0), runtime_actors=[dict(id=8500),
            dict(planned_center_distance_m=97.)], split='train')
        after = copy.deepcopy(before)
        after['runtime_actors'][1]['planned_center_distance_m'] = 98.4
        after['secondary_far_vehicle_generation'] = dict(initial_distance_spread=dict(
            policy=spread.POLICY, **assignment, actual_distance_ego_xy_m=98.4,
            target_error_m=abs(98.4-98.5)))
        return before, after, assignment

    def test_serialized_output_and_frozen_primary_are_verified(self):
        before, after, assignment = self.transition()
        with patch.object(spread.far, 'verify_preserved', return_value={'target_ordinal':1}), \
                patch.object(spread.far, '_read_source', return_value=({'frozen':True}, {})), \
                patch.object(spread.far, 'verify_payload') as payload:
            spread.verify_transition(before, after, {'serialized':True}, assignment)
            payload.assert_called_once_with({'serialized':True}, {'frozen':True}, {'target_ordinal':1}, after)
            after['runtime_actors'][0]['id'] = 999
            with self.assertRaises(spread.far.FarContractError):
                spread.verify_transition(before, after, {}, assignment)

    def test_foreign_assignment_or_output_distance_proof_is_rejected(self):
        for key, value in (('rank', 2), ('actual_distance_ego_xy_m', 98.), ('target_error_m', 0.)):
            before, after, assignment = self.transition()
            after['secondary_far_vehicle_generation']['initial_distance_spread'][key] = value
            with self.subTest(key=key), \
                    patch.object(spread.far, 'verify_preserved', return_value={'target_ordinal':1}), \
                    patch.object(spread.far, '_read_source', return_value=({}, {})), \
                    patch.object(spread.far, 'verify_payload'), self.assertRaises(spread.far.FarContractError):
                spread.verify_transition(before, after, {}, assignment)

    def test_source_spec_tamper_is_rejected_before_any_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/'source.json'
            output = Path(directory)/'fresh'
            entry = dict(id='source', scenario='/unused/scene.json', metadata='/unused/meta.json',
                         scenario_sha256='same')
            deck = dict(count=5000, scenes=[entry]*5000,
                        config=dict(frozen_specs_path='/unused/frozen.json', frozen_specs_sha256='same'))
            with patch.object(spread, 'read', side_effect=[deck, [dict(seed=1)]*5000, dict(spec=dict(seed=2))]), \
                    patch.object(spread, 'sha256', return_value='same'), \
                    patch('prepare_base5_far_vehicle_qualification.source_audit_pins', return_value={}), \
                    patch.object(spread.gen, 'context') as context:
                with self.assertRaisesRegex(spread.far.FarContractError, 'frozen spec'):
                    spread.prepare(source, output)
                context.assert_not_called()
                self.assertFalse(output.exists())

    def test_source_frozen_checksum_is_not_repaired(self):
        with tempfile.TemporaryDirectory() as directory:
            deck = dict(count=5000, scenes=[{}]*5000,
                        config=dict(frozen_specs_path='/unused/frozen.json', frozen_specs_sha256='expected'))
            with patch.object(spread, 'read', return_value=deck), \
                    patch.object(spread, 'sha256', return_value='changed'), \
                    patch('prepare_base5_far_vehicle_qualification.source_audit_pins', return_value={}):
                with self.assertRaisesRegex(spread.far.FarContractError, 'checksum'):
                    spread.prepare(Path(directory)/'source.json', Path(directory)/'fresh')


if __name__ == '__main__':
    unittest.main()
