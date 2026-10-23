import copy
from collections import Counter
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
import unittest
from argparse import Namespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_base5_fresh_completion as completion
import base5_primary_obstacle_range as policy


def source(planned=False, front=False):
    spec = dict(family='obstacle', mission=False, seed=17, distance_bounds=[10, 20],
                role='front', actors=1)
    if planned:
        spec['primary_obstacle_range'] = {'test': True}
    if front:
        spec['secondary_obstacle_front_support'] = {'test': True}
    return dict(spec=spec, split='train', scene_group_id='fresh:source',
                runtime_actors=[dict(data_id=1, name='Box', motion='static')])


class PrimaryRangeIntegrationTests(unittest.TestCase):
    def test_capped_source_never_borrows_another_asset(self):
        original, donor = source(True), source()
        donor['runtime_actors'][0]['data_id'] = 2
        self.assertEqual(completion.eligible_floor_donors(
            original, [donor], Counter({1: 100}), 4, {1: 'obstacle', 2: 'obstacle'}), [original])

    def test_uncapped_source_cannot_gain_a_foreign_range_or_front_plan(self):
        original = source()
        candidates = [source(True), source(front=True), source()]
        with patch.object(completion, 'floor_donors', side_effect=lambda s, d, c, f: d):
            self.assertEqual(completion.eligible_floor_donors(
                original, candidates, Counter(), 4, {1: 'obstacle'}), [candidates[2]])

    def prepare(self, original):
        with tempfile.TemporaryDirectory() as temporary, ExitStack() as stack:
            actual = copy.deepcopy(original)
            candidate = ({'ego': 1}, actual)
            front = stack.enter_context(patch.object(completion.front_completion, 'choose', return_value=candidate))
            ranged = stack.enter_context(patch.object(completion.primary_range, 'choose', return_value=candidate))
            legacy = stack.enter_context(patch.object(completion.flex, 'choose', return_value=candidate))
            stack.enter_context(patch.object(completion.front_completion, 'verify_preserved'))
            stack.enter_context(patch.object(completion.primary_range, 'validate_plan'))
            checked = stack.enter_context(patch.object(completion.primary_range, 'verify_preserved'))
            folder = Path(temporary) / 'placement'
            result = completion.prepare(dict(id='slot', scenario_sha256='old'), original, [],
                dict(config={'environment_tag': 'sunny13'}, scenes=[]), {}, {}, 1,
                folder, set(), {1: 'obstacle'})
            self.assertEqual(json.loads(Path(result['entry']['metadata']).read_text())['spec'], original['spec'])
            checked.assert_called_once_with(original, actual)
            return front.call_count, ranged.call_count, legacy.call_count

    def test_range_only_source_uses_frozen_seed_geometry_adapter(self):
        self.assertEqual(self.prepare(source(True)), (0, 1, 0))

    def test_dual_plan_uses_front_geometry_and_both_preservation_guards(self):
        self.assertEqual(self.prepare(source(True, front=True)), (1, 0, 0))

    def test_old_unplanned_source_retains_legacy_dispatch(self):
        self.assertEqual(self.prepare(source()), (0, 0, 1))

    def test_frozen_callback_uses_the_scoped_visibility_filter(self):
        meta, calibration, result = source(True), {}, {'score': 1.0}
        with patch.object(completion.primary_visibility, 'score', return_value=result) as scorer:
            self.assertEqual(completion.frozen_proposal_score(meta, calibration, 'all_actor_visibility'), result)
            scorer.assert_called_once_with(meta, calibration, 'all_actor_visibility', completion.adaptive.proposal_score)

    def test_range_config_mismatch_stops_before_generator_or_child(self):
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(completion.primary_range, 'validate_plan',
                    side_effect=policy.RangeContractError('Range config changed')), \
                patch.object(completion.spatial, 'installed') as geometry:
            with self.assertRaises(policy.RangeContractError):
                completion.prepare({'id': 'slot'}, source(True, front=True), [], {'config': {}},
                    {}, {}, 1, Path(directory) / 'placement', set(), {1: 'obstacle'})
            geometry.assert_not_called()

    def test_accepted_native_cannot_silently_drop_range_contract(self):
        original = source(True)
        actual = source()
        row = dict(status='accepted_candidate', capture_root='/raw')
        attempt = dict(moving=False, manifest='/manifest', manifest_sha256='manifest',
                       entry={'metadata': '/entry_metadata'}, capture='/capture')
        with patch.object(completion, 'sha256', return_value='manifest'), \
                patch.object(completion.adaptive.fa, 'check_placement'), \
                patch.object(completion.adaptive.fa.batch, 'inspect_batch', return_value=[row]), \
                patch.object(completion, 'read', return_value=actual), \
                patch.object(completion.front_completion, 'verify_preserved'), \
                patch.object(completion.primary_range, 'verify_preserved',
                             side_effect=RuntimeError('Frozen range plan lost')):
            with self.assertRaisesRegex(RuntimeError, 'range plan lost'):
                completion.verify(attempt, {'config': {}}, original, {1: 'obstacle'})

    def execute_fixture(self, root, outcome, fatal=False, no_deficit=False):
        """Synthetic CPU journal exercise; not native deficit/capture evidence."""
        parent, output = root / 'parent', root / 'completion'
        parent.mkdir()
        for name in ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'):
            (parent / name).write_text('{}')
        old, capped = source(), source(True)
        raw = root / 'baseline_raw'
        completion.atomic_json(raw / 'metadata.json', old)
        completion.atomic_json(root / 'source.json', capped)
        baseline = dict(source_cell_id='kept', status='accepted_candidate',
                        capture_root=str(raw), scenario_sha256='baseline')
        manifest = root / 'manifest.json'
        entries = [dict(id='kept', metadata=str(raw / 'metadata.json'), scenario_sha256='baseline')]
        if not no_deficit:
            entries.append(dict(id='missing', metadata=str(root / 'source.json'), scenario_sha256='old'))
        completion.atomic_json(manifest, dict(config=dict(environment_tag='sunny13',
            fresh_geometry_policy={'synthetic_test': True}, primary_obstacle_range_policy={'synthetic_test': True}),
            scenes=entries))
        args = Namespace(manifest=manifest, baseline=parent, output=output,
                         resume=False, run=True, attempts=3, time_budget_sec=100)
        def prepare(entry, original, donors, deck, ctx, cal, ordinal, folder, known, pools):
            return dict(id='missing', ordinal=ordinal, group=capped['scene_group_id'],
                        entry={'scenario_sha256': 'new' + str(ordinal)},
                        manifest='synthetic', moving=False, capture='synthetic', delta={})
        replacement = root / 'replacement_raw'
        completion.atomic_json(replacement / 'metadata.json', capped)
        result = dict(source_cell_id='missing_fc01', status=outcome,
                      capture_root=str(replacement), scenario_sha256='new1')
        with ExitStack() as stack:
            stack.enter_context(patch.object(completion, 'verify_parent', return_value=(
                [baseline], [] if no_deficit else ['missing'], 1.)))
            stack.enter_context(patch.object(completion.adaptive.fa.batch.campaign, 'contract', return_value={}))
            stack.enter_context(patch.object(completion.spatial.gen, 'context', return_value={}))
            stack.enter_context(patch.object(completion.adaptive, 'prewarm', side_effect=lambda c, _: c))
            stack.enter_context(patch.object(completion, 'pool_map', return_value={1: 'obstacle'}))
            stack.enter_context(patch.object(completion.adaptive.fa, 'normalized_sensor_calibration', return_value={}))
            stack.enter_context(patch.object(completion.primary_range, 'verify_preserved',
                side_effect=RuntimeError('Range source contract changed') if fatal else None))
            proposed = stack.enter_context(patch.object(completion, 'prepare', side_effect=prepare))
            child = stack.enter_context(patch.object(completion, 'run_child', return_value=0 if outcome == 'accepted_candidate' else 1))
            stack.enter_context(patch.object(completion, 'verify', return_value=result))
            stack.enter_context(patch.object(completion, 'adopted_entry',
                side_effect=lambda row, lookup: dict(id=row['source_cell_id'])))
            code = completion.execute(args)
            if fatal:
                self.assertEqual(code, 1)
                self.assertEqual(completion.read(output / 'state.json')['stage'], 'halted')
                self.assertEqual(proposed.call_count, 0)
                self.assertEqual(child.call_count, 0)
                return
            self.assertEqual(code, 0)
            count = 0 if no_deficit else 1 if outcome == 'accepted_candidate' else 3
            self.assertEqual(child.call_count, count)
            state = completion.read(output / 'state.json')
            self.assertEqual(state['baseline_selected'], [baseline])
            expected_stage = 'complete' if no_deficit or outcome == 'accepted_candidate' else 'complete_with_deficit'
            self.assertEqual(state['stage'], expected_stage)
            self.assertEqual(len(state['attempts']), count)
            self.assertEqual(completion.read(output / 'status.json')['replenished'], int(count == 1))
            before = {name: (output / name).read_bytes()
                      for name in ('state.json', 'selected_ledger.jsonl', 'status.json')}
            args.resume = True
            self.assertEqual(completion.execute(args), 0)
            self.assertEqual(child.call_count, count)
            self.assertEqual(before, {name: (output / name).read_bytes() for name in before})

    def test_cpu_orchestration_success_preserves_parent_and_terminal_resume(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.execute_fixture(Path(temporary), 'accepted_candidate')

    def test_cpu_orchestration_reviews_are_finite_and_never_restarted(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.execute_fixture(Path(temporary), 'review')

    def test_cpu_orchestration_contract_failure_halts_before_any_child(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.execute_fixture(Path(temporary), 'accepted_candidate', fatal=True)

    def test_cpu_orchestration_no_deficit_is_honest_and_does_not_capture(self):
        with tempfile.TemporaryDirectory() as temporary:
            self.execute_fixture(Path(temporary), 'accepted_candidate', no_deficit=True)


if __name__ == '__main__':
    unittest.main()
