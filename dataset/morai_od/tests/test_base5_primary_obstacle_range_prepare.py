import copy
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_primary_obstacle_range as policy
import prepare_base5_primary_obstacle_range as prepare
from base5_capture_safety import atomic_json, sha256
from test_base5_primary_obstacle_range import source_meta


class PreparationTests(unittest.TestCase):
    def fixture(self, root):
        entries, metadata = {}, {}
        for index, kind in enumerate(('far', 'retained', 'near', 'vehicle', 'avoidance')):
            meta = source_meta()
            meta['spec']['seed'] += index
            meta['scene_group_id'] = 'fresh:' + kind
            if kind == 'retained':
                meta['spec']['distance_bounds'] = [10, 30]
                meta['runtime_actors'][0]['planned_center_distance_m'] = 15.
            elif kind == 'near':
                meta['spec']['distance_bounds'] = [3, 10]
                meta['runtime_actors'][0]['planned_center_distance_m'] = 5.
            elif kind == 'vehicle':
                meta['runtime_actors'][0].update(data_id=30, name='OBJ_Car', population_pool='static_vehicle')
            elif kind == 'avoidance':
                meta['spec'].update(avoidance_phase='alongside', distance_bounds=[0, 30])
                meta['runtime_actors'][0]['planned_center_distance_m'] = 5.
            scenario, mp = root / 'source' / (kind + '.json'), root / 'source' / (kind + '_metadata.json')
            atomic_json(scenario, dict(original=kind))
            atomic_json(mp, meta)
            entry = dict(id=kind, scenario=str(scenario), scenario_sha256=sha256(scenario), metadata=str(mp),
                role=meta['spec']['role'], distance_bounds=meta['spec']['distance_bounds'], split=meta['split'])
            entries[kind], metadata[kind] = entry, meta
        source = root / 'source/manifest.json'
        atomic_json(source, dict(config=dict(environment_tag='sunny13'), scenes=list(entries.values()), count=5))
        reference = root / 'reference.json'
        atomic_json(reference, dict(schema=policy.REFERENCE_SCHEMA, assets={
            '10': dict(name='Thin', size_q50_m=[2., .2, 1.], observations=10)}))
        catalog = root / 'catalog.json'
        atomic_json(catalog, dict(assets=[dict(data_id=data_id, name=name, source_list='objectList', semantic_class=semantic)
            for data_id, name, semantic in ((10, 'Thin', 'obstacle'), (20, 'Box', 'obstacle'), (30, 'OBJ_Car', 'vehicle'))]))
        return source, reference, root / 'candidate', catalog, entries, metadata

    def mocks(self, stack, catalog, entries, metadata):
        stack.enter_context(patch.object(prepare.qualification, 'validate_source', return_value=(entries, metadata, {})))
        stack.enter_context(patch.object(prepare.spatial.gen.catalog, 'DEFAULT_CATALOG', catalog))
        stack.enter_context(patch.object(prepare.spatial.gen, 'context', return_value={}))
        stack.enter_context(patch.object(prepare, 'prewarm', side_effect=lambda ctx, config: ctx))
        audit = stack.enter_context(patch.object(prepare.audit_base5_scenarios, 'audit',
            return_value=dict(status='OFFLINE_PASS', scenes=5, errors=[])))
        by_seed = {meta['spec']['seed']: meta for meta in metadata.values()}
        def generated(row, config, ctx, retry):
            actual = copy.deepcopy(by_seed[row['seed']])
            actual['spec'] = copy.deepcopy(row)
            actual['runtime_actors'][0]['planned_center_distance_m'] = sum(row['distance_bounds']) / 2
            actual['retry'] = retry
            return dict(new_seed=row['seed'], retry=retry), actual
        generated_mock = stack.enter_context(patch.object(prepare.spatial, 'build', side_effect=generated))
        return generated_mock, audit

    def test_new_explicit_bounds_keep_already_in_bounds_payload_and_all_out_of_scope_bytes(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            source, reference, output, catalog, entries, metadata = self.fixture(Path(folder))
            build, audit = self.mocks(stack, catalog, entries, metadata)
            before = {path: path.read_bytes() for path in (Path(folder) / 'source').glob('*.json')}
            manifest = prepare.prepare(source, reference, output)
            candidate = prepare.read(manifest)
            proof = prepare.read(output / 'preservation_audit.json')
            self.assertTrue(proof['passed'])
            self.assertEqual(proof['declared_plan_count'], 2)
            self.assertEqual(proof['geometry_changed_scenes'], 1)
            self.assertEqual(proof['declared_only_payloads'], 1)
            self.assertEqual(proof['unchanged_payloads'], 4)
            self.assertEqual(proof['far_vehicle_primary_50m_payloads_preserved'], 1)
            self.assertEqual(build.call_count, 1)
            audit.assert_called_once_with(manifest)
            for row in candidate['scenes']:
                original = entries[row['id']]
                after = prepare.read(row['metadata'])
                if row['id'] in ('far', 'retained'):
                    self.assertIn(policy.PLAN_KEY, after['spec'])
                    self.assertEqual(row['distance_bounds'], [10, 20])
                    self.assertEqual(after['spec']['frozen_population_data_ids'], [10, 20])
                    policy.verify_preserved(after, after)
                else:
                    self.assertEqual(Path(row['metadata']).read_bytes(), Path(original['metadata']).read_bytes())
                if row['id'] != 'far':
                    self.assertEqual(Path(row['scenario']).read_bytes(), Path(original['scenario']).read_bytes())
                self.assertEqual(policy.identity(after['runtime_actors']), policy.identity(metadata[row['id']]['runtime_actors']))
                self.assertEqual(after['spec']['static_yaw_policy'], metadata[row['id']]['spec']['static_yaw_policy'])
                self.assertEqual(after['spec']['spatial_sampling_plan'], metadata[row['id']]['spec']['spatial_sampling_plan'])
            self.assertEqual(before, {path: path.read_bytes() for path in before})
            self.assertEqual(prepare.read(candidate['config']['frozen_specs_path']),
                [prepare.read(row['metadata'])['spec'] for row in candidate['scenes']])

    def test_actual_at_upper_bound_requires_new_geometry(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            source, reference, output, catalog, entries, metadata = self.fixture(Path(folder))
            metadata['retained']['runtime_actors'][0]['planned_center_distance_m'] = 20.
            atomic_json(entries['retained']['metadata'], metadata['retained'])
            build, _ = self.mocks(stack, catalog, entries, metadata)
            prepare.prepare(source, reference, output)
            self.assertEqual(build.call_count, 2)
            self.assertEqual(prepare.read(output / 'preservation_audit.json')['geometry_changed_scenes'], 2)

    def test_offline_geometry_budget_is_finite_and_does_not_overwrite_source(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            source, reference, output, catalog, entries, metadata = self.fixture(Path(folder))
            build, _ = self.mocks(stack, catalog, entries, metadata)
            build.side_effect = ValueError('No road-supported pose')
            original = sha256(source)
            with self.assertRaises(policy.RangeGeometryExhausted):
                prepare.prepare(source, reference, output, maximum_attempts=3)
            self.assertEqual(build.call_count, 3)
            self.assertEqual(sha256(source), original)
            self.assertTrue((output / 'failure.json').exists())
            self.assertFalse((output / 'manifest.json').exists())

    def test_contract_error_is_not_treated_as_another_geometry_draw(self):
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            source, reference, output, catalog, entries, metadata = self.fixture(Path(folder))
            build, _ = self.mocks(stack, catalog, entries, metadata)
            build.side_effect = policy.RangeContractError('Changed reference hash')
            with self.assertRaises(policy.RangeContractError):
                prepare.prepare(source, reference, output, maximum_attempts=3)
            self.assertEqual(build.call_count, 1)

    def test_empty_or_changed_existing_output_is_not_reused(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / 'existing'
            output.mkdir()
            with self.assertRaises(FileExistsError):
                prepare.prepare(Path(folder) / 'missing_source', Path(folder) / 'missing_ref', output)
        for maximum in (0, 4097):
            with tempfile.TemporaryDirectory() as folder, self.assertRaises(ValueError):
                prepare.prepare(Path(folder) / 'source', Path(folder) / 'ref', Path(folder) / 'output', maximum)


if __name__ == '__main__':
    unittest.main()
