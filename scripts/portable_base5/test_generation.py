import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'))
import generation as common
from base5_capture_safety import atomic_json, sha256


class CommonGenerationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root/'source'/'manifest.json'
        self.baseline = self.root/'baseline'/'manifest.json'
        range_reference = self.root/'range.json'
        atomic_json(range_reference, dict(example=True))
        atomic_json(self.source, dict(count=5000, scenes=[{}]*5000,
            config=dict(obstacle_support_policy=dict(variant='richfloor'))))
        atomic_json(self.baseline, dict(count=5000, scenes=[{}]*5000,
            split_overlay=dict(path='unused'), config=dict(environment_tag='sunny13',
                native_unit_geometry_reference=dict(path='unused'),
                primary_obstacle_range_policy=dict(reference=dict(path=str(range_reference),
                    sha256=sha256(range_reference))),
                static_mission_extent=dict(counts=dict(before=300, intersection=120, after=180)))))
        self.runtime = {str(Path(common.__file__).resolve()):sha256(common.__file__)}
        self.unit = patch.object(common.assembly.unit, 'load', return_value=dict(
            sources=[dict(path='reference.json', sha256='a'*64, assets=['Box'])]))
        self.unit.start()
        self.addCleanup(self.unit.stop)
        self.runtime_patch = patch.object(common.inputs, 'collect_runtime', return_value=self.runtime)
        self.runtime_patch.start()
        self.addCleanup(self.runtime_patch.stop)

    def plan(self, environment='sunny15', seed=123, opt_in=False, output=None):
        return common.plan(self.source, self.baseline, output or self.root/'new',
                           environment, seed, opt_in)

    def snapshot(self):
        return {str(path):path.read_bytes() for path in self.root.rglob('*') if path.is_file()}

    def test_all_six_environments_share_exact_policy_and_implementation_fingerprint(self):
        before = self.snapshot()
        results = [self.plan(environment, seed=index, opt_in=environment == 'sunny13')
                   for index, environment in enumerate(common.ENVIRONMENTS)]
        self.assertEqual(len({row['shared_policy_sha256'] for row in results}), 1)
        self.assertEqual(len({row['implementation_sha256'] for row in results}), 1)
        for row in results:
            self.assertEqual(row['shared_policy']['stages'], list(common.STAGES))
            self.assertEqual(row['shared_policy']['adjacent_lane_policy'], common.assembly.ADJACENT_POLICY)
            self.assertEqual(row['shared_policy']['far_spread_proposal_policy'], common.assembly.SPREAD_POLICY)
            self.assertTrue(row['shared_policy']['known_map_layout_split'])
            self.assertTrue(row['shared_policy']['capture_split_frozen_before_regeneration'])
            self.assertFalse(row['shared_policy']['environment_specific_policy_branch'])
            self.assertFalse(row['shared_policy']['native_qa_relaxed'])
            self.assertFalse(row['native_qualified'])
            self.assertFalse(row['bulk_ready'])
            self.assertFalse(row['bulk_started'])
            self.assertFalse(row['simulator_access'])
            self.assertFalse(row['remote_access'])
            self.assertEqual(row['shared_policy']['static_extent_counts'],
                             dict(before=300, intersection=120, after=180))
        self.assertEqual(self.snapshot(), before)
        self.assertFalse((self.root/'new').exists())

    def test_environment_settings_only_select_weather_hour_and_seed(self):
        for environment in common.ENVIRONMENTS:
            with self.subTest(environment=environment):
                result = common.environment_settings(environment, 99, environment == 'sunny13')
                self.assertEqual(result, dict(environment=environment,
                    weather=environment[:-2], hour=int(environment[-2:]), seed=99))
        with self.assertRaisesRegex(ValueError, 'requires --allow-new-sunny13'):
            self.plan('sunny13')
        for environment, seed in (('rainy15', 1), ('sunny15', True), ('sunny15', -1),
                                  ('sunny15', 2**63), ('sunny15', '123')):
            with self.subTest(environment=environment, seed=seed), self.assertRaises(ValueError):
                self.plan(environment, seed)
        with self.assertRaisesRegex(ValueError, 'boolean'):
            self.plan(opt_in='yes')

    def test_existing_output_and_frozen_source_destinations_remain_forbidden(self):
        before = self.snapshot()
        for output in (self.source.parent/'nested', self.baseline.parent/'nested', self.root):
            with self.subTest(output=output), self.assertRaises(ValueError):
                self.plan('sunny13', opt_in=True, output=output)
        self.assertEqual(self.snapshot(), before)

    def test_original_sunny13_safety_gate_is_not_permanently_changed(self):
        original_environments = common.assembly.ENVIRONMENTS
        original_generate = common.assembly.fresh.generate
        self.plan('sunny13', opt_in=True)
        self.assertIs(common.assembly.ENVIRONMENTS, original_environments)
        self.assertIs(common.assembly.fresh.generate, original_generate)
        with self.assertRaisesRegex(ValueError, 'remaining five'):
            common.assembly.plan(self.source, self.baseline, self.root/'new', 'sunny13', 123)

    def test_adapter_restores_functions_on_exception_and_rejects_nested_use(self):
        environments = common.assembly.ENVIRONMENTS
        original = common.assembly.fresh.generate
        with self.assertRaisesRegex(RuntimeError, 'intentional'):
            with common.common_assembly_adapter(True):
                self.assertEqual(common.assembly.ENVIRONMENTS, common.ENVIRONMENTS)
                with self.assertRaisesRegex(ValueError, 'Concurrent or nested'):
                    with common.common_assembly_adapter(True):
                        pass
                raise RuntimeError('intentional')
        self.assertIs(common.assembly.fresh.generate, original)
        self.assertIs(common.assembly.ENVIRONMENTS, environments)
        with common.common_assembly_adapter():
            pass

    def test_every_environment_uses_same_generator_and_only_sunny13_adds_safety_opt_in(self):
        def sentinel(*args, **kwargs):
            return args, kwargs
        with patch.object(common.assembly.fresh, 'generate', side_effect=sentinel) as original:
            with common.common_assembly_adapter(True):
                for environment in common.ENVIRONMENTS:
                    args, kwargs = common.assembly.fresh.generate('source', 'output', environment,
                        123, known_map_layout_split=True)
                    self.assertEqual(args, ('source', 'output', environment, 123))
                    expected = dict(known_map_layout_split=True)
                    if environment == 'sunny13':
                        expected['allow_new_sunny13'] = True
                    self.assertEqual(kwargs, expected)
            self.assertEqual(original.call_count, 6)
        with common.common_assembly_adapter():
            with self.assertRaisesRegex(ValueError, 'not explicitly authorized'):
                common.assembly.fresh.generate('source', 'output', 'sunny13')

    def test_policy_drift_or_stage_skipping_requires_review(self):
        request = self.plan()['assembly_request']
        for field, value in (('stages', list(common.STAGES)[1:]),
                             ('policy', 'unreviewed-new-policy'),
                             ('continue_from_unqualified_far_stage', dict(path='historical.json'))):
            changed = copy.deepcopy(request)
            changed[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                common.policy_contract(changed)

    def test_underlying_policy_upgrade_cannot_silently_keep_same_common_version(self):
        for field in ('POLICY', 'ADJACENT_POLICY', 'SPREAD_POLICY'):
            with self.subTest(field=field), patch.object(common.assembly, field, 'unreviewed-policy'):
                with self.assertRaisesRegex(ValueError, 'review and version'):
                    self.plan()

    def fake_assembly(self, source, baseline, output, environment, seed):
        output = Path(output)
        output.mkdir()
        request = common.assembly.plan(source, baseline, output/'nested', environment, seed)
        request['output'] = str(output)
        final = output/'candidate'/'manifest.json'
        atomic_json(final, dict(config=dict(environment_tag=environment,
            collection_environment=dict(weather=environment[:-2], hour=int(environment[-2:])))))
        atomic_json(output/'build_receipt.json', dict(request,
            final_manifest=str(final), final_manifest_sha256=sha256(final),
            stages=[dict(stage=name) for name in common.STAGES[:-1]],
            cross_environment_audit=dict(passed=True)))
        return final

    def test_prepare_records_separate_contract_without_rewriting_final_or_assembly_receipt(self):
        before = self.snapshot()
        for environment in common.ENVIRONMENTS:
            output = self.root/environment
            with self.subTest(environment=environment), \
                    patch.object(common.assembly, 'prepare', side_effect=self.fake_assembly):
                final = common.prepare(self.source, self.baseline, output, environment, 123,
                                       environment == 'sunny13')
            receipt = common.assembly.read(output/'common_generation_receipt.json')
            self.assertEqual(receipt['final_manifest_sha256'], sha256(final))
            self.assertEqual(receipt['assembly_receipt']['sha256'], sha256(output/'build_receipt.json'))
            self.assertEqual(receipt['settings']['environment'], environment)
            self.assertFalse(receipt['completion_live_qualified'])
            self.assertFalse(receipt['existing_frozen_releases_rewritten'])
            self.assertFalse(receipt['training_started'])
        for path, value in before.items():
            self.assertEqual(Path(path).read_bytes(), value)

    def test_prepare_rejects_wrong_environment_seed_changed_manifest_or_skipped_stage(self):
        for kind in ('environment', 'seed', 'payload', 'stages', 'cross_audit', 'final_environment'):
            output = self.root/kind
            def bad(source, baseline, output, environment, seed):
                final = self.fake_assembly(source, baseline, output, environment, seed)
                path = Path(output)/'build_receipt.json'
                build = common.assembly.read(path)
                if kind == 'environment':
                    build['environment'] = 'foggy11'
                elif kind == 'seed':
                    build['seed'] += 1
                elif kind == 'payload':
                    atomic_json(final, dict(changed=True))
                elif kind == 'stages':
                    build['stages'].pop(0)
                elif kind == 'cross_audit':
                    build['cross_environment_audit']['passed'] = False
                else:
                    deck = common.assembly.read(final)
                    deck['config']['collection_environment']['hour'] = 11
                    atomic_json(final, deck)
                    build['final_manifest_sha256'] = sha256(final)
                atomic_json(path, build)
                return final
            with self.subTest(kind=kind), patch.object(common.assembly, 'prepare', side_effect=bad):
                with self.assertRaises(ValueError):
                    common.prepare(self.source, self.baseline, output, 'sunny15', 123)
            self.assertFalse((output/'common_generation_receipt.json').exists())

    def test_prepare_rejects_changed_fixed_source_without_repairing_or_rewriting_it(self):
        output = self.root/'changed_source'
        def altered(*args):
            final = self.fake_assembly(*args)
            atomic_json(self.source, dict(external_change=True))
            return final
        with patch.object(common.assembly, 'prepare', side_effect=altered):
            with self.assertRaisesRegex(ValueError, 'Immutable generation input changed'):
                common.prepare(self.source, self.baseline, output, 'sunny15', 123)
        self.assertEqual(common.assembly.read(self.source), dict(external_change=True))
        self.assertFalse((output/'common_generation_receipt.json').exists())

    def test_prepare_rejects_changed_runtime_at_the_boundary(self):
        runtime = self.root/'implementation.py'
        atomic_json(runtime, dict(original=True))
        pins = {str(runtime.resolve()):sha256(runtime)}
        output = self.root/'changed_runtime'
        def altered(*args):
            final = self.fake_assembly(*args)
            atomic_json(runtime, dict(changed=True))
            return final
        with patch.object(common.inputs, 'collect_runtime', return_value=pins), \
                patch.object(common.assembly, 'prepare', side_effect=altered):
            with self.assertRaisesRegex(ValueError, 'Common generation implementation changed'):
                common.prepare(self.source, self.baseline, output, 'sunny15', 123)
        self.assertFalse((output/'common_generation_receipt.json').exists())


if __name__ == '__main__':
    unittest.main()
