import copy
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from argparse import Namespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_front_completion_adapter as adapter
import base5_obstacle_front_support as front
import base5_obstacle_rich_floor as rich
import run_base5_fresh_completion as completion


def examples():
    items = []
    for index in range(4):
        actors = [dict(data_id=data_id, name='Box' + str(data_id), source_list='objectList',
                       semantic_class='obstacle', motion='static', population_pool='obstacle_box',
                       estimated_size=[.5, .5, 1.]) for data_id in (1, 2, 10)]
        row = dict(family='obstacle', mission=False, population_mode='generic_regular',
                   actors=3, population_max_actors=8, population_budget_bounds=[12, 16],
                   seed=index, role='rear', distance_bounds=[15, 30],
                   frozen_population_data_ids=[1, 2, 10],
                   secondary_obstacle_support=dict(policy=rich.POLICY, variant='richfloor',
                       ordinals=[2], new_actor_ordinals=[2], protected_near_ordinals=[1],
                       protected_mid_ordinals=[], primary_preserved=True,
                       mission_and_moving_excluded=True, original_generation_attempt=0))
        items.append(('slot' + str(index), row, actors))
    planned = front.assign_plans(items)
    result = []
    for slot, _, actors in items:
        row = planned[slot]
        for ordinal, actor in enumerate(actors):
            actor.update(planned_center_distance_m=20. if ordinal == 0 else 12.,
                         actual_ego_role='rear' if ordinal == 0 else 'left')
        legacy = rich.sample(row, 2, actors[2], [.5, .5, 1.], 'rear', 2)
        proof = front.apply(row, 2, actors[2], [.5, .5, 1.], 2, legacy)
        actors[2].update(planned_center_distance_m=proof['target_center_distance_m'],
                         actual_ego_role=proof['role'], secondary_obstacle_support=proof)
        result.append(dict(spec=row, runtime_actors=actors, retry=2, split='train',
                           scene_group_id='fresh:' + slot))
    return result


def selected():
    return next(m for m in examples() if m['spec'][front.PLAN_KEY]['selected'])


def unplanned():
    m = selected()
    del m['spec'][front.PLAN_KEY]
    del m['runtime_actors'][2]['secondary_obstacle_support']['front_support']
    return m


def digest(scene):
    return hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()


class AdapterTests(unittest.TestCase):
    def choose(self, source, ordinal=1, known=None, novelty=None, build=None, score=None):
        def generated(row, config, ctx, retry):
            self.assertEqual(row, source['spec'])
            m = copy.deepcopy(source)
            m['retry'] = retry
            return dict(retry=retry), m
        return adapter.choose(source, {}, {}, {}, ordinal, 'slot', known or set(),
            build or generated, score or (lambda m, cal, kind: dict(score=m['retry'])),
            novelty or (lambda m, history: 1.))

    def test_selected_and_unselected_preserve_every_frozen_field(self):
        for source in examples():
            before = copy.deepcopy(source)
            scene, actual = self.choose(source)
            self.assertEqual(source, before)
            self.assertEqual(actual['spec'], before['spec'])
            self.assertEqual(actual['runtime_actors'], before['runtime_actors'])
            self.assertEqual(scene['retry'], source['retry'] + 8)
            self.assertEqual(actual['flexible_completion']['valid_candidates'], 8)
            self.assertEqual(adapter.verify_preserved(source, actual), source['spec'][front.PLAN_KEY])

    def test_attempt_ordinal_is_durable_disjoint_and_repeatable(self):
        source = selected()
        a, b = self.choose(source, 2), self.choose(source, 2)
        self.assertEqual(a, b)
        self.assertEqual(a[1]['flexible_completion']['retry_block_start'], source['retry'] + 1 + 4096)
        self.assertNotEqual(self.choose(source, 1)[0], a[0])

    def test_known_payload_and_insufficient_novelty_are_not_ranked(self):
        source = selected()
        first = source['retry'] + 1
        scene, m = self.choose(source, known={digest(dict(retry=first))},
            novelty=lambda m, history: .1 if m['retry'] == first + 1 else 1.)
        self.assertEqual(scene['retry'], first + 9)
        self.assertEqual(m['flexible_completion']['proposals_tried'], 10)

    def test_all_failed_proposals_stop_after_512_and_never_reset_seed(self):
        source = selected()
        attempts = []
        def fails(row, config, ctx, retry):
            self.assertEqual(row, source['spec'])
            attempts.append(retry)
            raise ValueError('No safe layout')
        with self.assertRaisesRegex(adapter.FrontGeometryExhausted, 'bounded search'):
            self.choose(source, build=fails)
        self.assertEqual(attempts, list(range(3, 515)))

    def test_only_eight_candidates_are_ranked_and_ties_use_first_retry(self):
        source = selected()
        scene, m = self.choose(source, score=lambda *args: dict(score=1.))
        self.assertEqual(scene['retry'], 3)
        self.assertEqual(m['flexible_completion']['proposals_tried'], 8)

    def test_source_and_candidate_mutations_fail_closed(self):
        source = selected()
        for kind in ('seed', 'plan', 'actor_id', 'actor_count', 'motion', 'split',
                     'front_role', 'front_band', 'front_proof', 'protected_near',
                     'unique_id', 'name', 'source_list', 'population_pool'):
            changed = copy.deepcopy(source)
            if kind == 'seed': changed['spec']['seed'] += 1
            elif kind == 'plan': changed['spec'][front.PLAN_KEY]['logical_slot_id'] = 'other'
            elif kind == 'actor_id': changed['runtime_actors'][1]['data_id'] = 999
            elif kind == 'actor_count': changed['runtime_actors'].pop()
            elif kind == 'motion': changed['runtime_actors'][1]['motion'] = 'walking'
            elif kind == 'split': changed['split'] = 'val'
            elif kind == 'front_role': changed['runtime_actors'][2]['actual_ego_role'] = 'rear'
            elif kind == 'front_band': changed['runtime_actors'][2]['planned_center_distance_m'] = 20.
            elif kind == 'front_proof': changed['runtime_actors'][2]['secondary_obstacle_support'].pop('front_support')
            elif kind in ('unique_id', 'name', 'source_list', 'population_pool'):
                changed['runtime_actors'][1][kind] = 'changed'
            else: changed['runtime_actors'][1]['planned_center_distance_m'] = 25.
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                adapter.verify_preserved(source, changed)

    def test_unselected_cannot_gain_front_proof(self):
        source = next(m for m in examples() if not m['spec'][front.PLAN_KEY]['selected'])
        changed = copy.deepcopy(source)
        changed['runtime_actors'][2]['secondary_obstacle_support']['front_support'] = source['spec'][front.PLAN_KEY]
        with self.assertRaises(ValueError): adapter.verify_preserved(source, changed)

    def test_unplanned_legacy_noop_but_foreign_plan_is_rejected(self):
        source = unplanned()
        changed = copy.deepcopy(source)
        changed['spec']['seed'] = 99
        changed['runtime_actors'] = changed['runtime_actors'][:1]
        self.assertIsNone(adapter.verify_preserved(source, changed))
        with self.assertRaises(ValueError): adapter.verify_preserved(source, selected())

    def test_attempt_and_missing_plan_guards(self):
        for ordinal in (0, 6, True, 1.5):
            with self.assertRaises(ValueError): self.choose(selected(), ordinal)
        with self.assertRaises(ValueError): self.choose(unplanned())
        source = selected()
        source['spec'][front.PLAN_KEY] = None
        with self.assertRaises(ValueError): self.choose(source)

    def test_source_helper_hash_is_still_enforced(self):
        source = selected()
        with patch.object(front, 'implementation_sha256', return_value='different'):
            with self.assertRaises(adapter.FrozenFrontContractError): self.choose(source)

    def test_helper_change_during_search_is_not_geometry_exhaustion(self):
        source = selected()
        original = front.implementation_sha256()
        with patch.object(front, 'implementation_sha256', return_value=original) as implementation:
            attempts = []
            def changed(row, config, ctx, retry):
                attempts.append(retry)
                implementation.return_value = 'changed'
                raise ValueError('Placement failed while source changed')
            with self.assertRaises(adapter.FrozenFrontContractError):
                self.choose(source, build=changed)
        self.assertEqual(len(attempts), 1)

    def test_builder_identity_violation_halts_instead_of_spending_512_layouts(self):
        source = selected()
        attempts = []
        def changed(row, config, ctx, retry):
            attempts.append(retry)
            actual = copy.deepcopy(source)
            actual['runtime_actors'][1]['data_id'] = 999
            return dict(retry=retry), actual
        with self.assertRaises(adapter.FrozenFrontContractError):
            self.choose(source, build=changed)
        self.assertEqual(len(attempts), 1)


class FreshCompletionIntegrationTests(unittest.TestCase):
    def execution_fixture(self, root, source):
        parent, output = root / 'parent', root / 'output'
        parent.mkdir()
        for name in ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'):
            (parent / name).write_text('{}')
        completion.atomic_json(root / 'metadata.json', source)
        ident = source['spec'].get(front.PLAN_KEY, {}).get('logical_slot_id', 'legacy')
        completion.atomic_json(root / 'manifest.json', dict(config=dict(
            fresh_geometry_policy={'policy': 'test'}, environment_tag='sunny13',
            obstacle_front_support_policy={'policy': front.POLICY}), scenes=[dict(
                id=ident, metadata=str(root / 'metadata.json'), scenario_sha256='old')]))
        args = Namespace(attempts=3, time_budget_sec=100, output=output, baseline=parent,
            manifest=root / 'manifest.json', resume=False, run=True)
        return args, ident

    def execution_mocks(self, stack, ident):
        stack.enter_context(patch.object(completion, 'verify_parent', return_value=([], [ident], 0.)))
        stack.enter_context(patch.object(completion.adaptive.fa.batch.campaign, 'contract', return_value={}))
        stack.enter_context(patch.object(completion.spatial.gen, 'context', return_value={}))
        stack.enter_context(patch.object(completion.adaptive, 'prewarm', side_effect=lambda ctx, config: ctx))
        stack.enter_context(patch.object(completion, 'pool_map', return_value={1: 'obstacle', 2: 'obstacle', 10: 'obstacle'}))
        stack.enter_context(patch.object(completion.adaptive.fa, 'normalized_sensor_calibration', return_value={}))
        return stack.enter_context(patch.object(completion, 'run_child'))

    def test_bad_source_pin_halts_before_candidate_search_or_native_capture(self):
        for run in (False, True):
            with self.subTest(run=run), tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
                source = selected()
                source['spec'][front.PLAN_KEY]['implementation_sha256'] = 'changed'
                args, ident = self.execution_fixture(Path(folder), source)
                args.run = run
                launched = self.execution_mocks(stack, ident)
                prepare = stack.enter_context(patch.object(completion, 'prepare'))
                self.assertEqual(completion.execute(args), 1)
                prepare.assert_not_called()
                launched.assert_not_called()
                self.assertEqual(completion.read(args.output / 'state.json')['stage'], 'halted')
                self.assertEqual(completion.read(args.output / 'state.json')['unavailable'], {})

    def test_only_bounded_front_geometry_exhaustion_is_an_ordinary_deficit(self):
        for error, stage, unavailable in ((adapter.FrontGeometryExhausted('bounded'), 'complete_with_deficit', True),
                                           (ValueError('policy mismatch'), 'halted', False),
                                           (adapter.FrozenFrontContractError('contract'), 'halted', False)):
            with self.subTest(error=type(error)), tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
                args, ident = self.execution_fixture(Path(folder), selected())
                launched = self.execution_mocks(stack, ident)
                prepare = stack.enter_context(patch.object(completion, 'prepare', side_effect=error))
                self.assertEqual(completion.execute(args), 0 if unavailable else 1)
                self.assertEqual(prepare.call_count, 1)
                launched.assert_not_called()
                state = completion.read(args.output / 'state.json')
                self.assertEqual(state['stage'], stage)
                self.assertEqual(bool(state['unavailable']), unavailable)

    def test_unplanned_legacy_value_error_remains_an_ordinary_deficit(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            args, ident = self.execution_fixture(Path(folder), unplanned())
            launched = self.execution_mocks(stack, ident)
            stack.enter_context(patch.object(completion, 'prepare', side_effect=ValueError('No legacy layout')))
            self.assertEqual(completion.execute(args), 0)
            launched.assert_not_called()
            self.assertEqual(completion.read(args.output / 'state.json')['stage'], 'complete_with_deficit')

    def test_floor_filter_cannot_borrow_a_plan_and_keeps_source_fallback(self):
        source, donor = unplanned(), selected()
        pools = {1: 'obstacle', 2: 'obstacle', 10: 'obstacle'}
        self.assertEqual(completion.eligible_floor_donors(source, [donor], Counter({1: 0}), 4, pools), [source])
        self.assertEqual(completion.eligible_floor_donors(donor, [source], Counter({1: 99}), 4, pools), [donor])

    def test_unplanned_prepare_is_exact_legacy_choose_and_filters_plan_donors(self):
        source, donor = unplanned(), selected()
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(completion.flex, 'choose', return_value=({'scene': 1}, copy.deepcopy(source))) as choose:
                completion.prepare(dict(id='s', scenario_sha256='old'), source, [source, donor],
                    dict(config=dict(environment_tag='sunny13'), scenes=[]), {}, {}, 1,
                    Path(folder) / 'placement', set(), {1: 'obstacle', 2: 'obstacle', 10: 'obstacle'})
            self.assertEqual(choose.call_args.args[1], [source])
            self.assertEqual(choose.call_args.args[0], source)

    def test_selected_prepare_uses_adapter_and_retains_source_group(self):
        source = selected()
        def build(row, config, ctx, retry):
            actual = copy.deepcopy(source)
            actual['retry'] = retry
            return dict(retry=retry), actual
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(completion.flex, 'choose') as legacy, \
                 patch.object(completion.spatial, 'build', side_effect=build), \
                 patch.object(completion.adaptive, 'proposal_score', return_value=dict(score=1.)), \
                 patch.object(completion.adaptive.fa, 'novelty', return_value=1.):
                result = completion.prepare(dict(id='s', scenario_sha256='old'), source, [],
                    dict(config=dict(environment_tag='sunny13'), scenes=[]), {}, {}, 1,
                    Path(folder) / 'placement', set(), {1: 'obstacle', 2: 'obstacle', 10: 'obstacle'})
            legacy.assert_not_called()
            actual = completion.read(result['entry']['metadata'])
            self.assertEqual(actual['spec'], source['spec'])
            self.assertEqual(actual['scene_group_id'], source['scene_group_id'])
            self.assertEqual(actual['flexible_completion']['policy'], adapter.POLICY)

    def test_native_accepted_but_changed_front_role_is_rejected(self):
        source = selected()
        changed = copy.deepcopy(source)
        changed['runtime_actors'][2]['actual_ego_role'] = 'rear'
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            completion.atomic_json(root / 'metadata.json', changed)
            completion.atomic_json(root / 'manifest.json', {})
            attempt = dict(moving=False, manifest=str(root / 'manifest.json'),
                manifest_sha256=completion.sha256(root / 'manifest.json'), capture=str(root),
                entry=dict(metadata=str(root / 'metadata.json')))
            with patch.object(completion.adaptive.fa, 'check_placement'), \
                 patch.object(completion.adaptive.fa.batch, 'inspect_batch', return_value=[
                     dict(status='accepted_candidate', capture_root=str(root))]):
                with self.assertRaises(ValueError): completion.verify(attempt, {'config': {}}, source, {1: 'obstacle'})

    def test_resume_hashes_are_conditional_and_pin_adapter_source(self):
        for front_enabled in (False, True):
            with self.subTest(front_enabled=front_enabled), tempfile.TemporaryDirectory() as folder:
                root = Path(folder)
                parent, output = root / 'parent', root / 'output'
                parent.mkdir()
                for name in ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json'):
                    (parent / name).write_text('{}')
                config = dict(fresh_geometry_policy={'policy': 'test'}, environment_tag='sunny13')
                if front_enabled: config['obstacle_front_support_policy'] = {'policy': front.POLICY}
                completion.atomic_json(root / 'manifest.json', dict(config=config, scenes=[]))
                args = Namespace(attempts=3, time_budget_sec=100, output=output, baseline=parent,
                    manifest=root / 'manifest.json', resume=False, run=False)
                with patch.object(completion, 'verify_parent', return_value=([], [], 0.)), \
                     patch.object(completion.adaptive.fa.batch.campaign, 'contract', return_value={}):
                    self.assertEqual(completion.execute(args), 0)
                hashes = completion.read(output / 'state.json')['contract']['code_hashes']
                for name in ('base5_front_completion_adapter.py', 'base5_obstacle_front_support.py',
                             'base5_obstacle_rich_floor.py', 'base5_obstacle_support_policy.py'):
                    self.assertEqual(name in hashes, front_enabled)


if __name__ == '__main__':
    unittest.main()
