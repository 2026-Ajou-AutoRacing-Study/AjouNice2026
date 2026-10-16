import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_scale_extent_smoke as smoke
from base5_capture_safety import sha256


class ScaleExtentSmokeTests(unittest.TestCase):
    def setUp(self):
        self.pairs = {('CargoBox', 'iso125'), ('CargoBox', 'iso150'),
                      ('PE_Drum', 'iso125'), ('PE_Drum', 'iso150')}
        self.metadata = {}
        self.entries = []
        self.unchanged = set()

        def add(ident, family, actors, **row):
            split = row.pop('split', 'train')
            self.entries.append(dict(id=ident, scenario='/' + ident + '.json'))
            self.metadata[ident] = dict(spec=dict(family=family, mission=False, **row),
                                       split=split, runtime_actors=actors)
            return self.metadata[ident]

        self.add = add
        for index in range(2):
            add('shadow' + str(index), 'shadow', [dict(motion='walking')], shadow_moving=True)
        for crossing in smoke.CROSSWALKS:
            add(crossing, 'pedestrian_related', [dict(motion='walking', mission_crossing_id=crossing)])
        for lower, upper in ((50, 80), (80, 100)):
            for role in ('front', 'rear'):
                add(f'{lower}_{role}', 'highway', [dict(source_list='vehicleList',
                    actual_ego_role=role, planned_center_distance_m=(lower + upper) / 2)])
        self.unchanged = set(self.metadata)
        for index, (name, profile) in enumerate(sorted(self.pairs)):
            primary = index % 2 == 0
            actors = [self.scaled(name, profile, 0 if primary else 1)]
            if not primary:
                actors.insert(0, dict(name='Woman2'))
            family = ('obstacle', 'shadow', 'pedestrian_related', 'obstacle')[index]
            meta = add('scale' + str(index), family, actors)
            if index == 0:
                meta['spec'].update(mission=True, static_mission_extent=dict(stratum='before'),
                                    avoidance_phase='approach')
        for index, phase in enumerate(smoke.PHASES):
            meta = add('extent' + str(index), 'obstacle', [dict(name='Dog2')],
                       split='val' if index % 2 else 'train',
                       static_mission_extent=dict(stratum=('before', 'intersection', 'after', 'after')[index]),
                       avoidance_phase=phase)
            meta['spec']['mission'] = True

    @staticmethod
    def scaled(name, profile, ordinal):
        return dict(name=name, asset_scale=dict(profile_id=profile, candidate_only=False,
            profile_status='qualified', name=name, ordinal=ordinal))

    def choose(self, maximum=32):
        return smoke.select(dict(scenes=self.entries), self.metadata, self.pairs,
                            ['shadow0', 'shadow1'], self.unchanged, maximum)

    def test_all_profiles_extent_contexts_and_eight_original_controls_covered(self):
        rows, proof = self.choose()
        self.assertTrue(proof['coverage_passed'])
        self.assertEqual(proof['protected_frames'], 8)
        self.assertLessEqual(len(rows), 32)
        self.assertEqual(len({row['id'] for row in rows}), len(rows))
        self.assertEqual(proof['unavailable_scale_context_or_roles'], [])
        self.assertTrue(set(proof['required']) <= set(proof['covered']))
        self.assertFalse(proof['native_outcomes_used_for_selection'])
        self.assertEqual({row['id'] for row in rows[:8]}, self.unchanged)

    def test_manifest_and_dictionary_order_do_not_change_selection(self):
        before = copy.deepcopy(self.entries), copy.deepcopy(self.metadata)
        expected = self.choose()
        self.entries.reverse()
        self.metadata = dict(reversed(list(self.metadata.items())))
        self.assertEqual(expected, self.choose())
        self.assertEqual(sorted(self.entries, key=lambda row: row['id']),
                         sorted(before[0], key=lambda row: row['id']))
        self.assertEqual(self.metadata, before[1])

    def test_missing_profile_is_explicit_failure_not_original_fallback(self):
        self.metadata['scale0']['runtime_actors'][0].pop('asset_scale')
        with self.assertRaisesRegex(smoke.SmokeContractError, 'profile/CargoBox/iso125'):
            self.choose()

    def test_missing_far_original_is_not_replaced_with_changed_entry(self):
        self.unchanged.remove('80_rear')
        with self.assertRaisesRegex(smoke.SmokeContractError, '80_100m/rear'):
            self.choose()

    def test_two_crosswalk_paths_require_actual_moving_actors(self):
        self.metadata['CW000002']['runtime_actors'][0]['motion'] = 'standing'
        with self.assertRaisesRegex(smoke.SmokeContractError, 'CW000002'):
            self.choose()

    def test_exact_shadow_controls_must_be_moving_original_and_unique(self):
        self.metadata['shadow0']['spec']['shadow_moving'] = False
        with self.assertRaisesRegex(smoke.SmokeContractError, 'actual moving'):
            self.choose()
        self.metadata['shadow0']['spec']['shadow_moving'] = True
        self.unchanged.remove('shadow0')
        with self.assertRaisesRegex(smoke.SmokeContractError, 'changed'):
            self.choose()

    def test_profile_candidate_or_wrong_actor_ordinal_is_rejected(self):
        assignment = self.metadata['scale0']['runtime_actors'][0]['asset_scale']
        for key, value in (('candidate_only', True), ('ordinal', 2), ('name', 'Vehicle')):
            old = assignment[key]
            assignment[key] = value
            with self.subTest(key=key), self.assertRaises(smoke.SmokeContractError):
                self.choose()
            assignment[key] = old

    def test_finite_cap_and_no_automatic_cap_growth(self):
        with self.assertRaisesRegex(smoke.SmokeContractError, 'cap cannot cover'):
            self.choose(8)
        for value in (0, 33, True):
            with self.subTest(value=value), self.assertRaises(smoke.SmokeContractError):
                self.choose(value)

    def test_available_context_absence_is_reported_not_fabricated(self):
        self.metadata['scale1']['spec']['family'] = 'obstacle'
        _, proof = self.choose()
        self.assertEqual(proof['unavailable_scale_context_or_roles'], ['scaled_context/shadow'])

    def test_seven_asset_two_qualified_profile_contract_excludes_deferred_redbarrel(self):
        policy = dict(assets={name: dict(profiles={'a': {}, 'b': {}})
                             for name in smoke.APPROVED_ASSETS})
        with patch.object(smoke.scale, '_qualified', return_value=True):
            self.assertEqual(len(smoke.approved_pairs(policy)), 14)
            policy['assets']['CargoBox']['profiles']['unexpected'] = {}
            with self.assertRaises(smoke.SmokeContractError):
                smoke.approved_pairs(policy)

    def test_fixed_native_count_adds_deterministic_preoutcome_scale_slots(self):
        for index in range(4):
            self.add('extra' + str(index), 'obstacle', [self.scaled('CargoBox', 'iso125', 0)])
        expected = smoke.select(dict(scenes=self.entries), self.metadata, self.pairs,
            ['shadow0', 'shadow1'], self.unchanged, target=17)
        rows, proof = expected
        self.assertEqual(len(rows), 17)
        self.assertEqual(proof['target_frames'], 17)
        self.assertGreater(proof['extra_fixed_count_frames'], 0)
        self.assertFalse(proof['native_outcomes_used_for_selection'])
        self.entries.reverse()
        self.metadata = dict(reversed(list(self.metadata.items())))
        self.assertEqual(expected, smoke.select(dict(scenes=self.entries), self.metadata, self.pairs,
            ['shadow0', 'shadow1'], self.unchanged, target=17))

    def test_fixed_count_shortage_never_repeats_or_substitutes_unscaled_slots(self):
        with self.assertRaisesRegex(smoke.SmokeContractError, 'additional qualified-scale'):
            smoke.select(dict(scenes=self.entries), self.metadata, self.pairs,
                         ['shadow0', 'shadow1'], self.unchanged, target=17)
        for value in (0, 33, True):
            with self.subTest(value=value), self.assertRaisesRegex(smoke.SmokeContractError, 'Fixed native target'):
                smoke.select(dict(scenes=self.entries), self.metadata, self.pairs,
                             ['shadow0', 'shadow1'], self.unchanged, target=value)

    def test_scale_source_audit_requires_full_pass_and_each_present_sha(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / 'manifest.json'
            manifest.write_text('{}\n')
            preservation = dict(schema='base5_asset_scale_preparation_v1', passed=True,
                errors=[], frames=5000, manifest_sha256=sha256(manifest))
            scenario = dict(status='OFFLINE_PASS', errors=[], scenes=5000, unique_payloads=5000)
            (root / 'preservation_audit.json').write_text(json.dumps(preservation))
            (root / 'scenario_audit.json').write_text(json.dumps(scenario))
            self.assertEqual(len(smoke.source_checks(manifest)), 2)
            preservation['candidate_manifest_sha256'] = '0' * 64
            (root / 'preservation_audit.json').write_text(json.dumps(preservation))
            with self.assertRaisesRegex(smoke.SmokeContractError, 'stale'):
                smoke.source_checks(manifest)
            preservation.pop('candidate_manifest_sha256')
            (root / 'preservation_audit.json').write_text(json.dumps(preservation))
            scenario['unique_payloads'] = 4999
            (root / 'scenario_audit.json').write_text(json.dumps(scenario))
            with self.assertRaisesRegex(smoke.SmokeContractError, 'incomplete'):
                smoke.source_checks(manifest)

    def test_final_shadow_registry_overlay_requires_exact_audited_parent(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent_dir, child_dir = root / 'parent', root / 'registered'
            parent_dir.mkdir()
            child_dir.mkdir()
            parent = dict(count=5000, config=dict(asset_scale_policy='fixture'),
                          scenes=[dict(id=str(index)) for index in range(5000)])
            parent_path = parent_dir / 'manifest.json'
            parent_path.write_text(json.dumps(parent))
            preservation = dict(schema='base5_asset_scale_preparation_v1', passed=True,
                errors=[], frames=5000, manifest_sha256=sha256(parent_path))
            (parent_dir / 'preservation_audit.json').write_text(json.dumps(preservation))
            (parent_dir / 'scenario_audit.json').write_text(json.dumps(dict(
                status='OFFLINE_PASS', errors=[], scenes=5000, unique_payloads=5000)))
            registry_path = child_dir / 'registry.json'
            registry = dict(policy=smoke.reuse.POLICY,
                source_manifest=dict(path=str(parent_path), sha256=sha256(parent_path)))
            registry_path.write_text(json.dumps(registry))
            child = copy.deepcopy(parent)
            child['config'][smoke.reuse.KEY] = dict(policy=smoke.reuse.POLICY,
                registration=dict(path=str(registry_path), sha256=sha256(registry_path)))
            manifest = child_dir / 'manifest.json'
            manifest.write_text(json.dumps(child))
            proof = smoke.source_checks(manifest)
            self.assertEqual(len(proof), 4)
            self.assertEqual(proof[str(parent_path)], sha256(parent_path))
            self.assertEqual(proof[str(registry_path)], sha256(registry_path))
            child['scenes'][0]['id'] = 'silently-replaced'
            manifest.write_text(json.dumps(child))
            with self.assertRaisesRegex(smoke.SmokeContractError, 'outside its one'):
                smoke.source_checks(manifest)
            child['scenes'][0]['id'] = '0'
            manifest.write_text(json.dumps(child))
            registry['source_manifest']['sha256'] = '0' * 64
            registry_path.write_text(json.dumps(registry))
            child['config'][smoke.reuse.KEY]['registration']['sha256'] = sha256(registry_path)
            manifest.write_text(json.dumps(child))
            with self.assertRaisesRegex(smoke.SmokeContractError, 'parent manifest pin'):
                smoke.source_checks(manifest)

    def test_missing_full_audit_does_not_authorize_an_arbitrary_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = Path(directory) / 'manifest.json'
            manifest.write_text(json.dumps(dict(count=5000, config={}, scenes=[])))
            with self.assertRaisesRegex(smoke.SmokeContractError, 'explicit shadow registry'):
                smoke.source_checks(manifest)


if __name__ == '__main__':
    unittest.main()
