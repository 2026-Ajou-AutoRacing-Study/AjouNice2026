"""CPU-only deterministic native-qualification selection contracts."""
from collections import Counter
import copy
import sys
from pathlib import Path
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import prepare_base5_far_vehicle_qualification as qualification


def fixtures():
    result = []
    for band in qualification.BANDS:
        for role in qualification.ROLES:
            for lane in qualification.LANES:
                for index, name in enumerate(('DEFAULT_SEDAN', '2021_Hyundai_G80', 'DEFAULT_SUV',
                                               '2021_Hyundai_GV70', 'DEFAULT_TRUCK')):
                    result.append(dict(id=f'{band[0]}_{role}_{lane}_{index}', stratum=(band,role,lane),
                        asset=name, body_group=qualification.body_group(name),
                        sampling_bounds_m=list((97,99) if index in (1,3) else (95,97))
                            if band==(95,99) else list(band)))
    return result


class FarQualificationTest(unittest.TestCase):
    def test_reproducible_balanced_selection_without_mutation(self):
        rows = fixtures()
        before = copy.deepcopy(rows)
        first = qualification.select(rows)
        second = qualification.select(list(reversed(rows)))
        self.assertEqual(first, second)
        self.assertEqual(rows, before)
        self.assertEqual(len(first), 32)
        self.assertEqual(set(Counter(r['stratum'] for r in first).values()), {2})
        for band in qualification.BANDS:
            for role in qualification.ROLES:
                chosen = [r for r in first if r['stratum'][:2] == (band, role)]
                self.assertGreaterEqual(len({r['asset'] for r in chosen}), 3)
                self.assertTrue({'passenger','suv_crossover'} <= {r['body_group'] for r in chosen})

    def test_missing_lane_stratum_fails_explicitly(self):
        rows = [r for r in fixtures() if r['stratum'] != ((95,99),'rear','adjacent')]
        with self.assertRaisesRegex(ValueError, 'stratum shortage'):
            qualification.select(rows)

    def test_truck_only_is_not_a_good_average(self):
        rows = [dict(r, asset='DEFAULT_TRUCK', body_group='other') for r in fixtures()]
        with self.assertRaisesRegex(ValueError, 'asset coverage unavailable'):
            qualification.select(rows)

    def test_duplicate_candidate_rejected(self):
        rows = fixtures()
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            qualification.select(rows + rows[:1])

    def test_unrecognized_asset_is_not_mislabeled_suv(self):
        self.assertEqual(qualification.body_group('unknown_tall_van'), 'other')
        self.assertEqual(qualification.body_group('2021_Hyundai_G80'), 'passenger')
        self.assertEqual(qualification.body_group('DEFAULT_SUV'), 'suv_crossover')

    def test_candidate_targets_secondary_not_visible_primary(self):
        target = dict(unique_id=8501, data_id=30100010, name='2021_Hyundai_G80',
            source_list='vehicleList', motion='static', actual_ego_role='rear',
            planned_center_distance_m=95.)
        plan = dict(target_uid=8501, target_data_id=30100010, target_name='2021_Hyundai_G80',
            target_ordinal=1, distance_bounds_m=[95,99], sampling_bounds_m=[95,97],
            role='rear', lane_relation='same')
        meta = dict(spec=dict(secondary_far_vehicle=plan, family='highway', mission=False,
            vehicle_only_population=True, seed=3), split='train',
            runtime_actors=[dict(target, unique_id=8500), target])
        row = qualification.candidate_row(dict(id='far'), meta)
        self.assertEqual(row['target_uid'], 8501)
        self.assertEqual(row['stratum'], ((95,99),'rear','same'))
        plan['target_ordinal'] = 0
        with self.assertRaisesRegex(ValueError, 'secondary ordinal'):
            qualification.candidate_row(dict(id='far'), meta)

    def test_tail_selection_has_both_windows_upper_front_car_and_preferred_lanes(self):
        chosen = qualification.select(fixtures())
        for role in qualification.ROLES:
            tail = [r for r in chosen if r['stratum'][:2]==((95,99),role)]
            self.assertEqual({tuple(r['sampling_bounds_m']) for r in tail}, set(qualification.TAIL_WINDOWS))
            upper = [r for r in tail if r['sampling_bounds_m']==[97,99]]
            self.assertEqual({r['stratum'][2] for r in upper}, {'same','adjacent'})
            if role == 'front':
                self.assertTrue(any(r['body_group']=='passenger' for r in upper))

    def test_tail_missing_lower_window_is_not_silently_reselected(self):
        rows = fixtures()
        for row in rows:
            if row['stratum'][0]==(95,99):
                row['sampling_bounds_m']=[97,99]
        with self.assertRaisesRegex(ValueError, 'both sampling windows'):
            qualification.select(rows)

    def test_tail_requires_upper_front_passenger_not_just_upper_trucks(self):
        rows = fixtures()
        for row in rows:
            if (row['stratum'][:2]==((95,99),'front') and row['sampling_bounds_m']==[97,99]
                    and row['body_group']=='passenger'):
                row.update(asset='DEFAULT_TRUCK',body_group='other')
        with self.assertRaisesRegex(ValueError, 'front passenger'):
            qualification.select(rows)

    def test_upper_both_lane_preference_does_not_remove_hard_valid_fallback(self):
        rows = fixtures()
        for row in rows:
            if row['stratum']==((95,99),'front','adjacent'):
                row['sampling_bounds_m']=[95,97]
        chosen = qualification.select(rows)
        upper = [r for r in chosen if r['stratum'][:2]==((95,99),'front')
                 and r['sampling_bounds_m']==[97,99]]
        self.assertEqual({r['stratum'][2] for r in upper}, {'same'})
        self.assertTrue(any(r['body_group']=='passenger' for r in upper))

    def test_candidate_must_actually_be_inside_its_declared_sampling_window(self):
        actor = dict(unique_id=8501,data_id=30100010,name='2021_Hyundai_G80',
            source_list='vehicleList',motion='static',actual_ego_role='front',
            planned_center_distance_m=95.5)
        plan = dict(target_uid=8501,target_data_id=30100010,target_name='2021_Hyundai_G80',
            target_ordinal=1,distance_bounds_m=[95,99],sampling_bounds_m=[97,99],
            role='front',lane_relation='same')
        meta = dict(spec=dict(secondary_far_vehicle=plan,family='highway',mission=False,
            vehicle_only_population=True,seed=3),split='train',runtime_actors=[dict(actor,unique_id=8500),actor])
        with self.assertRaisesRegex(ValueError, 'sampling window mismatch'):
            qualification.candidate_row(dict(id='wrong_window'),meta)

    def test_local_import_closure_includes_function_local_transitive_imports(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            entry, middle, leaf = (directory / (name + '.py') for name in ('entry','middle','leaf'))
            entry.write_text('def function():\n    import middle\n')
            middle.write_text('from leaf import value\nimport json\n')
            leaf.write_text('value = 1\n')
            closure = qualification.collect_runtime(scripts_dir=directory, entrypoints=('entry.py',))
            self.assertEqual(set(closure), {str(p.resolve()) for p in (entry,middle,leaf)})
            self.assertEqual(closure[str(leaf.resolve())], qualification.sha256(leaf))

    def test_runtime_closure_includes_literal_subprocess_scripts(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            entry, child = directory/'entry.py', directory/'run_child.py'
            entry.write_text("def run():\n    return __file__.with_name('run_child.py')\n")
            child.write_text('value = 1\n')
            closure = qualification.collect_runtime(scripts_dir=directory, entrypoints=('entry.py',))
            self.assertEqual(set(closure), {str(entry.resolve()), str(child.resolve())})

    def test_runtime_closure_rejects_missing_dynamic_script(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            entry = directory/'entry.py'
            entry.write_text("value = __file__.with_name('run_missing.py')\n")
            with self.assertRaisesRegex(ValueError, 'Missing frozen release file'):
                qualification.collect_runtime(scripts_dir=directory, entrypoints=('entry.py',))

    def test_visibility_helper_and_function_local_dependencies_are_pinned(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory/'base5_secondary_far_vehicle.py'
            helper = directory/'base5_far_vehicle_visibility.py'
            rays = directory/'base5_primary_visibility_proposal.py'
            normalizer = directory/'dynamic_capture_metadata.py'
            source.write_text('def choose():\n    from base5_far_vehicle_visibility import score\n')
            helper.write_text('import base5_primary_visibility_proposal as rays\n'
                              'def check():\n    import dynamic_capture_metadata\n')
            rays.write_text('value = 1\n')
            normalizer.write_text('value = 2\n')
            closure = qualification.collect_runtime(scripts_dir=directory,
                entrypoints=('base5_secondary_far_vehicle.py',))
            self.assertEqual(set(closure), {str(p.resolve()) for p in
                (source, helper, rays, normalizer)})

    def test_frozen_config_reference_is_verified_not_just_recorded(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            data = directory / 'reference.json'
            qualification.atomic_json(data, dict(value=1))
            digest = qualification.sha256(data)
            config = dict(nested=dict(path='reference.json',sha256=digest),
                          frozen_specs_path='reference.json',frozen_specs_sha256=digest,
                          source_manifest='reference.json',source_manifest_sha256=digest)
            self.assertEqual(qualification.config_references(config,directory), {str(data.resolve()):digest})
            qualification.atomic_json(data, dict(value=2))
            with self.assertRaisesRegex(ValueError, 'dependency changed'):
                qualification.config_references(config,directory)

    def test_visibility_config_pins_every_file_not_derived_digest_as_a_path(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            visibility = dict(policy='far-secondary-single-foreground-3d-proposal-v1',
                              normalized_calibration_sha256='0'*64)
            expected = {}
            for key in ('implementation', 'ray_helper', 'geometry_reference', 'sensor', 'normalizer'):
                path = directory/(key+'.json')
                qualification.atomic_json(path, dict(reference=key))
                visibility[key] = dict(path=str(path), sha256=qualification.sha256(path))
                expected[str(path.resolve())] = qualification.sha256(path)
            config = dict(secondary_far_vehicle_policy=dict(proposal_visibility=visibility))
            self.assertEqual(qualification.config_references(config, directory), expected)
            qualification.atomic_json(directory/'sensor.json', dict(changed=True))
            with self.assertRaisesRegex(ValueError, 'dependency changed'):
                qualification.config_references(config, directory)

    def test_existing_output_refused_before_reading_source(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileExistsError):
                qualification.prepare(Path(directory)/'missing_manifest.json',Path(directory))

    def source_audit_fixture(self, directory):
        directory = Path(directory)
        manifest = directory / 'manifest.json'
        qualification.atomic_json(manifest, dict(count=5000))
        qualification.atomic_json(directory / 'preservation_audit.json', dict(passed=True, errors=[],
            frames=5000, candidate_manifest_sha256=qualification.sha256(manifest)))
        qualification.atomic_json(directory / 'scenario_audit.json', dict(status='OFFLINE_PASS',
            errors=[], scenes=5000, unique_payloads=5000))
        return manifest

    def test_full_candidate_audits_are_verified_and_pinned(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            pins = qualification.source_audit_pins(manifest)
            self.assertEqual(len(pins), 2)
            self.assertTrue(all(qualification.sha256(p) == digest for p, digest in pins.items()))

    def test_failed_preservation_audit_prevents_subset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            path = manifest.parent / 'preservation_audit.json'
            payload = qualification.read(path)
            payload['passed'] = False
            qualification.atomic_json(path, payload)
            with self.assertRaisesRegex(ValueError, 'preservation audit'):
                qualification.source_audit_pins(manifest)

    def test_failed_scenario_audit_prevents_subset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            path = manifest.parent / 'scenario_audit.json'
            payload = qualification.read(path)
            payload.update(status='OFFLINE_FAIL', errors=[['04978', 'primary angular-occluded']])
            qualification.atomic_json(path, payload)
            with self.assertRaisesRegex(ValueError, 'scenario audit'):
                qualification.source_audit_pins(manifest)

    def test_stale_candidate_hash_prevents_subset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            qualification.atomic_json(manifest, dict(count=5000, changed=True))
            with self.assertRaisesRegex(ValueError, 'preservation audit'):
                qualification.source_audit_pins(manifest)

    def test_incomplete_scenario_audit_prevents_subset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            path = manifest.parent / 'scenario_audit.json'
            payload = qualification.read(path)
            payload['scenes'] = 24
            qualification.atomic_json(path, payload)
            with self.assertRaisesRegex(ValueError, 'scenario audit'):
                qualification.source_audit_pins(manifest)

    def test_missing_full_audit_prevents_subset_registration(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = self.source_audit_fixture(directory)
            (manifest.parent / 'scenario_audit.json').unlink()
            with self.assertRaises(FileNotFoundError):
                qualification.source_audit_pins(manifest)


if __name__ == '__main__':
    unittest.main()
