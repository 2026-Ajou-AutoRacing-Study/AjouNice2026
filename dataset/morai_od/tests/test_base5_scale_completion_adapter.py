"""CPU-only frozen scale completion; no simulator, capture or learned model."""
import copy
from contextlib import ExitStack, nullcontext
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_asset_scale as scale
import base5_scale_completion_adapter as adapter
import run_base5_fresh_completion as completion
import run_base5_flexible_completion as flex


def payload_sha(scene):
    return hashlib.sha256((json.dumps(scene, ensure_ascii=False, indent=2) + '\n').encode()).hexdigest()


class ScaleCompletionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        evidence = self.root / 'native_visual_evidence.json'
        evidence.write_text('{}\n')
        self.policy = scale.candidate_policy([2, 0, 1])
        self.policy['assets'] = {'CargoBox': self.policy['assets']['CargoBox']}
        record = self.policy['assets']['CargoBox']
        record['profiles'] = {'iso125': record['profiles']['iso125']}
        profile = record['profiles']['iso125']
        profile.update(status='qualified', native_geometry=dict(bbox_size_m=[1.25, 1.25, .625],
            pivot_to_bbox_center_local_m=[0., 0., .3125], pivot_height_above_ground_m=0.),
            native_qualification=dict(observations=2, semantic_visual_review='accepted',
                contact_review='passed', sensor_gt_review='passed', axis_mapping_verified=True,
                single_component_obstacle_verified=True,
                evidence=[dict(path=str(evidence), sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())]))
        self.config = {scale.CONFIG_KEY: self.policy, 'environment_tag': 'sunny13'}
        unit_actor = dict(name='CargoBox', data_id=40100001, unique_id=8500,
            source_list='objectList', semantic_class='obstacle', motion='static',
            population_pool='obstacle', estimated_size=[1., 1., .5], position=dict(x=12., y=0., z=0.))
        original = dict(seed=31, actors=1, family='obstacle', mission=True, motion='static',
                        role='front', distance_bounds=[10, 20])
        spec = scale.attach_plan(original, [unit_actor], self.policy, ['iso125'])
        assignment = scale.select(unit_actor, spec, 0, 'static', self.policy, profile_id='iso125')
        actor = dict(unit_actor, estimated_size=[1.25, 1.25, .625], asset_scale=assignment,
            static_height_initializer=scale.height_initializer(assignment, self.policy,
                [1.25, 1.25, .625], [0., 0., .3125]))
        self.source = dict(spec=spec, runtime_actors=[actor], retry=2, split='train', scene_group_id='group')
        self.scene = dict(objectList=[scale.apply_actor(dict(DataID=40100001, UNIQUEID=8500),
                                                     assignment, self.policy)])
        self.pools = {40100001: 'obstacle'}

    def unscaled(self):
        value = copy.deepcopy(self.source)
        value['spec'] = copy.deepcopy(self.source['spec'][scale.PLAN_KEY]['source_spec'])
        value['runtime_actors'][0].pop('asset_scale')
        return value

    def generated(self, row, config, ctx, retry):
        self.assertEqual(row, self.source['spec'])
        actual = copy.deepcopy(self.source)
        actual['retry'] = retry
        return dict(self.scene, retry=retry), actual

    def choose(self, ordinal=1, build=None, known=None, novelty=None):
        return adapter.choose(self.source, self.config, {}, {}, ordinal, 'slot', known or set(),
            build or self.generated, lambda meta, cal, kind: dict(score=meta['retry']),
            novelty or (lambda meta, history: 1.))

    def test_seed_profiles_and_actor_population_remain_exact(self):
        before = copy.deepcopy(self.source)
        scene, actual = self.choose()
        self.assertEqual(self.source, before)
        self.assertEqual(actual['spec'], before['spec'])
        self.assertEqual(actual['runtime_actors'], before['runtime_actors'])
        self.assertEqual(scene['retry'], self.source['retry'] + 8)
        self.assertFalse(actual['flexible_completion']['scale_profile_fallback'])
        self.assertEqual(actual['flexible_completion']['source_profile_ids'], ['iso125'])

    def test_resume_ordinals_have_disjoint_deterministic_retry_blocks(self):
        a, b = self.choose(2), self.choose(2)
        self.assertEqual(a, b)
        self.assertEqual(a[1]['flexible_completion']['retry_block_start'], self.source['retry'] + 1 + 4096)
        self.assertNotEqual(self.choose(1)[0], a[0])

    def test_unit_donor_cannot_replace_scaled_source_or_gain_scale_plan(self):
        unit = self.unscaled()
        self.assertFalse(flex.compatible(self.source, unit, self.pools))
        self.assertFalse(flex.compatible(unit, self.source, self.pools))
        self.assertFalse(completion.compatible(self.source, unit, self.pools))
        self.assertFalse(completion.compatible(unit, self.source, self.pools))
        self.assertTrue(completion.compatible(self.source, copy.deepcopy(self.source), self.pools))
        self.assertEqual(completion.eligible_floor_donors(self.source, [unit], {}, 4, self.pools), [self.source])
        self.assertNotIn(self.source,
            completion.eligible_floor_donors(unit, [self.source, unit], {40100001: 0}, 0, self.pools))

    def test_bounded_geometry_exhaustion_only_is_ordinary_deficit(self):
        seen = []
        def fails(row, config, ctx, retry):
            seen.append(retry)
            raise ValueError('No safe placement')
        with self.assertRaises(adapter.ScaleGeometryExhausted):
            self.choose(build=fails)
        self.assertEqual(len(seen), 512)
        self.assertEqual(seen[0], self.source['retry'] + 1)
        self.assertEqual(seen[-1], self.source['retry'] + 512)
        with self.assertRaises(scale.ScaleContractError):
            self.choose(build=lambda *args: (_ for _ in ()).throw(scale.ScaleContractError('Profile changed')))

    def test_seed_profile_or_actor_tamper_is_fatal(self):
        for key in ('seed', 'profile', 'identity', 'size'):
            def altered(row, config, ctx, retry):
                scene, actual = self.generated(row, config, ctx, retry)
                if key == 'seed':
                    actual['spec']['seed'] += 1
                elif key == 'profile':
                    actual['runtime_actors'][0]['asset_scale']['profile_id'] = scale.BASE
                elif key == 'identity':
                    actual['runtime_actors'][0]['unique_id'] += 1
                else:
                    actual['runtime_actors'][0]['estimated_size'] = [1., 1., .5]
                return scene, actual
            with self.subTest(key=key), self.assertRaises(scale.ScaleContractError):
                self.choose(build=altered)

    def test_native_serialized_scale_is_verified_before_ranking(self):
        def altered(row, config, ctx, retry):
            scene, actual = self.generated(row, config, ctx, retry)
            scene = copy.deepcopy(scene)
            scene['objectList'][0]['scale']['x'] = 1.
            return scene, actual
        with self.assertRaises(scale.ScaleContractError):
            self.choose(build=altered)

    def test_existing_front_range_far_contracts_still_run(self):
        with ExitStack() as stack:
            probes = [stack.enter_context(patch.object(module, 'verify_preserved'))
                      for module in (adapter.front, adapter.primary_range, adapter.secondary_far)]
            self.choose()
            self.assertTrue(all(p.call_count > 1 for p in probes))

    def test_scale_adapter_owns_exhaustion_with_front_range_or_far_overlay(self):
        with ExitStack() as stack:
            for module in (completion.front_completion, completion.primary_range, completion.secondary_far):
                stack.enter_context(patch.object(module, 'has_plan', return_value=True))
            self.assertTrue(completion.recoverable_geometry_error(
                self.source, adapter.ScaleGeometryExhausted('Finite geometry search')))
            self.assertFalse(completion.recoverable_geometry_error(self.source, ValueError('Contract data')))
            self.assertFalse(completion.recoverable_geometry_error(self.source, scale.ScaleContractError('Mismatch')))
            self.assertFalse(completion.recoverable_geometry_error(
                self.source, completion.front_completion.FrontGeometryExhausted('Wrong adapter')))

    def test_known_payload_and_low_novelty_are_never_ranked(self):
        first = self.source['retry'] + 1
        known = {payload_sha(dict(self.scene, retry=first))}
        scene, actual = self.choose(known=known,
            novelty=lambda meta, history: .1 if meta['retry'] == first + 1 else 1.)
        self.assertEqual(scene['retry'], first + 9)
        self.assertEqual(actual['flexible_completion']['proposals_tried'], 10)

    def test_fresh_prepare_prioritizes_scale_over_front_and_does_not_call_flex(self):
        entry = dict(id='slot', scenario_sha256='old', role='front', distance_bounds=[10, 20])
        deck = dict(config=self.config)
        scene, actual = self.generated(self.source['spec'], self.config, {}, 5)
        with ExitStack() as stack:
            choose = stack.enter_context(patch.object(completion.scale_completion, 'choose', return_value=(scene, actual)))
            stack.enter_context(patch.object(completion.front_completion, 'has_plan', return_value=True))
            stack.enter_context(patch.object(completion.front_completion, 'verify_preserved'))
            stack.enter_context(patch.object(completion.front_completion, 'choose', side_effect=AssertionError('Front adapter used')))
            stack.enter_context(patch.object(completion.flex, 'choose', side_effect=AssertionError('Unit donor used')))
            stack.enter_context(patch.object(completion.spatial, 'installed', return_value=nullcontext()))
            stack.enter_context(patch.object(completion.adaptive.fa.batch.campaign, 'subset_deck',
                                             side_effect=lambda deck, rows: dict(deck, scenes=rows)))
            value = completion.prepare(entry, self.source, [self.unscaled()], deck, {}, {}, 1,
                self.root / 'proposal', set(), self.pools)
            self.assertTrue(choose.called)
            self.assertEqual(value['entry']['id'], 'slot_fc01')


if __name__ == '__main__':
    unittest.main()
