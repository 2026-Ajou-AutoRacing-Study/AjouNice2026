import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_obstacle_front_support as front
import base5_obstacle_rich_floor as rich
import base5_obstacle_support_policy as support
from base5_capture_safety import atomic_json, sha256
import prepare_base5_obstacle_front_support as prepare


class FrontSupportPrepareTests(unittest.TestCase):
    def source(self, root):
        entries, originals = [], {}
        for index in range(4):
            ident = 'slot' + str(index)
            row = dict(family='obstacle', mission=False, role='front', seed=index,
                population_mode='generic_regular', actors=3, distance_bounds=[15, 30],
                frozen_population_data_ids=[1, 2, 10],
                secondary_obstacle_support=dict(policy=rich.POLICY, variant='richfloor',
                    ordinals=[2], new_actor_ordinals=[2], protected_near_ordinals=[1],
                    protected_mid_ordinals=[], primary_preserved=True,
                    mission_and_moving_excluded=True, original_generation_attempt=0))
            actors = [dict(name='asset' + str(data_id), data_id=data_id,
                source_list='objectList', motion='static', population_pool='obstacle_box',
                estimated_size=[.5, .5, 1.], actual_ego_role=role,
                planned_center_distance_m=distance)
                for data_id, role, distance in ((1, 'front', 22.), (2, 'left', 12.),
                                                 (10, 'rear', 15.))]
            metadata = dict(spec=row, runtime_actors=actors, split='train', retry=0,
                            scene_group_id='group:' + ident)
            path, meta = root / (ident + '.json'), root / (ident + '_metadata.json')
            atomic_json(path, dict(unchanged_native_payload=index))
            atomic_json(meta, metadata)
            entries.append(dict(id=ident, scenario=str(path), metadata=str(meta),
                                scenario_sha256=sha256(path), split='train'))
            originals[index] = metadata
        manifest = root / 'manifest.json'
        atomic_json(manifest, dict(config={'obstacle_support_policy': {'policy': rich.POLICY}},
                                   count=4, scenes=entries))
        return manifest, originals

    def fake_build(self, originals):
        def build(row, config, context, attempt):
            meta = copy.deepcopy(originals[row['seed']])
            meta.update(spec=copy.deepcopy(row), retry=attempt)
            asset = dict(data_id=10, source_list='objectList', semantic_class='obstacle')
            proof = support.sample(row, 2, asset, [.5, .5, 1.], 'rear', attempt)
            meta['runtime_actors'][2].update(planned_center_distance_m=proof['target_center_distance_m'],
                                             actual_ego_role=proof['role'],
                                             secondary_obstacle_support=proof)
            return dict(seed=row['seed'], attempt=attempt, support=proof), meta
        return build

    def test_prepare_rebuilds_selected_only_and_preserves_payloads_and_frozen_specs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, originals = self.source(root)
            before = {str(p): sha256(p) for p in root.glob('*.json')}
            output = root / 'candidate'
            with mock.patch.object(prepare.spatial.gen, 'context', return_value={}), \
                 mock.patch.object(prepare, 'prewarm', side_effect=lambda ctx, config: ctx), \
                 mock.patch.object(prepare.spatial, 'build', side_effect=self.fake_build(originals)) as build:
                manifest = prepare.prepare(source, output)
            candidate = json.loads(manifest.read_text())
            delta = json.loads((output / 'front_support_delta.json').read_text())
            self.assertEqual(build.call_count, 2)
            self.assertEqual(delta['changed_scenes'], 2)
            self.assertEqual(delta['untouched_payloads'], 2)
            self.assertFalse(delta['live_qualified'])
            self.assertFalse(delta['bulk_started'])
            self.assertEqual(candidate['config']['obstacle_front_support_policy']['policy'], front.POLICY)
            self.assertEqual(candidate['config']['frozen_specs_sha256'],
                             sha256(candidate['config']['frozen_specs_path']))
            frozen = json.loads(Path(candidate['config']['frozen_specs_path']).read_text())
            self.assertEqual(frozen, [json.loads(Path(e['metadata']).read_text())['spec']
                                      for e in candidate['scenes']])
            old = {e['id']: e for e in json.loads(source.read_text())['scenes']}
            for entry in candidate['scenes']:
                meta = json.loads(Path(entry['metadata']).read_text())
                if not meta['spec'][front.PLAN_KEY]['selected']:
                    self.assertEqual(Path(entry['scenario']).read_bytes(),
                                     Path(old[entry['id']]['scenario']).read_bytes())
                spec = copy.deepcopy(meta['spec'])
                spec.pop(front.PLAN_KEY)
                self.assertEqual(spec, originals[spec['seed']]['spec'])
            self.assertEqual(before, {path: sha256(path) for path in before})
            with self.assertRaises(FileExistsError):
                prepare.prepare(source, output)

    def test_bounded_rejection_does_not_relax_contract_or_overwrite_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, _ = self.source(root)
            digest = sha256(source)
            with mock.patch.object(prepare.spatial.gen, 'context', return_value={}), \
                 mock.patch.object(prepare, 'prewarm', side_effect=lambda ctx, config: ctx), \
                 mock.patch.object(prepare.spatial, 'build', side_effect=ValueError('No safe pose')) as build:
                with self.assertRaisesRegex(ValueError, 'Bounded front-support proposal exhausted'):
                    prepare.prepare(source, root / 'failed', maximum_attempts=2)
            self.assertEqual(build.call_count, 2)
            failure = json.loads((root / 'failed' / 'failure.json').read_text())
            self.assertEqual(failure['maximum_attempts'], 2)
            self.assertFalse((root / 'failed' / 'manifest.json').exists())
            self.assertEqual(sha256(source), digest)

    def test_preservation_refuses_identity_seed_split_and_protected_band_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, originals = self.source(Path(tmp))
            old = originals[0]
            for change in ('identity', 'seed', 'split', 'near'):
                new = copy.deepcopy(old)
                if change == 'identity': new['runtime_actors'][0]['data_id'] = 99
                if change == 'seed': new['spec']['seed'] = 99
                if change == 'split': new['split'] = 'val'
                if change == 'near': new['runtime_actors'][1]['planned_center_distance_m'] = 22.
                with self.subTest(change=change), self.assertRaises(ValueError):
                    prepare.preservation(old, new, selected=False)


if __name__ == '__main__':
    unittest.main()
