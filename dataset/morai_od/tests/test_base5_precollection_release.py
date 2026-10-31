"""CPU-only release evidence rejection tests; no collector, ROS or GPU calls.

Small temporary inventories exercise the finalizer's own checks. Expensive
official parent reconstruction and portable validation are mocked at their API
boundary, with explicit assertions that the finalizer invokes them. Their
native/geometry implementations have separate regression suites.
"""
import copy
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import finalize_base5_precollection_release as release
import prepare_base5_precollection_smoke as smoke
import qualify_base5_training_inputs as training
import report_base5_far_vehicle_qualification as far_report
import run_base5_fresh_completion as completion
import verify_base5_fresh_terminal_resume as terminal
from base5_capture_safety import sha256


class TemporaryEvidenceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, relative, value):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value) + '\n')
        return path

    def blob(self, relative, value=b'immutable fixture'):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value)
        return path

    def jsonl(self, relative, rows):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
        return path


class SmokeEvidenceTest(TemporaryEvidenceTest):
    def fixture(self):
        labels = ['low_obstacle_near', 'wide_obstacle_mid', 'stroller_components',
                  'pet_components', 'intersection_walking', 'intersection_running',
                  'shadow_reuse_Dog1', 'shadow_reuse_Dog2']
        entries, selected, registered = [], [], []
        config = dict(environment_tag='sunny13')
        for i, label in enumerate(labels):
            ident = 'source_' + str(i)
            meta = dict(spec=dict(family='obstacle', mission=False), runtime_actors=[])
            if label.startswith('intersection_'):
                meta.update(spec=dict(family='pedestrian_related', mission=True),
                    runtime_actors=[dict(motion_path=dict(ground_path_evidence=dict(
                        policy='editor-intersection-path-v116')))])
            scenario = self.write('source/' + ident + '/scenario.json', dict(id=ident))
            metadata = self.write('source/' + ident + '/metadata.json', meta)
            entry = dict(id=ident, scenario=str(scenario), metadata=str(metadata),
                         scenario_sha256=sha256(scenario))
            entries.append(entry)
            raw = self.root / 'capture' / ('cell_' + str(i)) / 'initial/captures' / ident
            self.blob(raw / 'scenario.json', scenario.read_bytes())
            self.write(raw / 'metadata.json', meta)
            if label.startswith('shadow_'):
                # Real reuse preserves the original source entry but launches
                # its independently grounded sibling deck, not that original.
                adopted_meta = copy.deepcopy(meta)
                adopted_meta[release.reuse.KEY] = dict(source_id=ident)
                folder = raw.parents[2] / 'motion_reuse_deck'
                adopted_scenario = self.write(folder / 'scenario.json',
                    dict(id=ident, independently_ground_prepared=True))
                adopted_metadata = self.write(folder / 'metadata.json', adopted_meta)
                adopted_entry = dict(entry, scenario=str(adopted_scenario),
                    metadata=str(adopted_metadata), scenario_sha256=sha256(adopted_scenario))
                adoption = self.write(folder / 'manifest.json',
                    dict(count=1, config=copy.deepcopy(config), scenes=[adopted_entry]))
                self.write(raw.parents[1] / 'session_contract.json',
                           dict(manifest_sha256=sha256(adoption)))
                self.blob(raw / 'scenario.json', adopted_scenario.read_bytes())
                self.write(raw / 'metadata.json', adopted_meta)
            audit = dict(accepted_candidate=True, review_reasons=[])
            self.write(raw / 'audit.json', audit)
            self.write(raw / 'collision_history.json', dict(summary=dict(
                observed=True, no_collision_verified=True, non_ego_uids=[])))
            self.write(raw.parents[1] / 'shutdown_safety.json', dict(park_verified=True))
            selected.append(dict(id=ident, source_cell_id=ident, capture_root=str(raw),
                status='accepted_candidate', scenario_sha256=sha256(raw / 'scenario.json'), audit=audit))
            registered.append(dict(id=ident, stratum=label,
                scenario_sha256=sha256(scenario), metadata_sha256=sha256(metadata)))
        deck = dict(count=8, config=config, scenes=entries)
        full = self.write('full/manifest.json', deck)
        subset = self.write('smoke/manifest.json', deck)
        dependency = self.blob('terrain.bin')
        runtime = {str(self.blob('runtime.py')): sha256(self.root / 'runtime.py')}
        doc = dict(policy=smoke.POLICY, source_manifest_sha256=sha256(full),
            subset_manifest_sha256=sha256(subset), runtime_hashes=runtime,
            helper_sha256=sha256(smoke.__file__),
            reuse_input_hashes={str(dependency): sha256(dependency)}, selected=registered)
        registration = self.write('smoke/registration.json', doc)
        capture = self.root / 'capture'
        self.jsonl(capture / 'selected_ledger.jsonl', selected)
        self.write(capture / 'status.json', dict(stage='complete', requested=8,
                                               accepted_candidate=8, review=0))
        return SimpleNamespace(**locals())

    def verify(self, f, official=None, missing=None):
        with patch.object(completion, 'verify_parent', return_value=(
                copy.deepcopy(f.selected if official is None else official), missing or [], 1.)) as parent, \
                patch.object(release.reuse, 'verify_capture') as reuse_check:
            result = release.verify_smoke(f.registration, f.capture, f.full, f.runtime)
            parent.assert_called_once_with(f.subset, f.capture)
            self.assertEqual(reuse_check.call_count, 8)
            for call, row, entry in zip(reuse_check.call_args_list, f.selected, f.entries):
                raw = Path(row['capture_root'])
                if row['source_cell_id'] in ('source_6', 'source_7'):
                    adopted = release.read(raw.parents[2] / 'motion_reuse_deck/manifest.json')
                    self.assertEqual(call.args, (adopted['scenes'][0], raw, adopted))
                    self.assertNotEqual(adopted['scenes'][0]['scenario_sha256'], entry['scenario_sha256'])
                else:
                    self.assertEqual(call.args, (entry, raw, release.read(f.full)))
            return result

    def test_valid_evidence_invokes_official_parent_and_every_raw_verifier(self):
        f = self.fixture()
        self.assertEqual(self.verify(f), dict(accepted=8, shadow_preparation_reused=2,
                                             old_raw_used_as_new_training=False))

    def test_same_source_id_cannot_substitute_a_different_selected_raw(self):
        f = self.fixture()
        forged = copy.deepcopy(f.selected)
        forged[0]['capture_root'] = '/foreign/approved/raw'
        self.jsonl(f.capture / 'selected_ledger.jsonl', forged)
        with self.assertRaisesRegex(ValueError, 'Official smoke reconstruction'):
            self.verify(f)

    def test_matching_raw_and_ledger_hash_cannot_replace_exact_source_geometry(self):
        f = self.fixture()
        raw = Path(f.selected[0]['capture_root'])
        self.write(raw / 'scenario.json', dict(id='source_0', foreign_geometry=True))
        f.selected[0]['scenario_sha256'] = sha256(raw / 'scenario.json')
        self.jsonl(f.capture / 'selected_ledger.jsonl', f.selected)
        # Even an upstream reconstruction accepting the same ledger must not
        # turn source_cell_id alone into an exact-input smoke qualification.
        self.assertNotEqual(f.selected[0]['scenario_sha256'], f.entries[0]['scenario_sha256'])
        with self.assertRaisesRegex(ValueError, 'exact registered source'):
            self.verify(f)

    def test_non_editor_raw_spec_change_cannot_reuse_source_identity(self):
        f = self.fixture()
        raw = Path(f.selected[0]['capture_root'])
        meta = release.read(raw / 'metadata.json')
        meta['spec']['mission'] = True
        self.write(raw / 'metadata.json', meta)
        self.assertEqual(f.selected[0]['scenario_sha256'], f.entries[0]['scenario_sha256'])
        with self.assertRaisesRegex(ValueError, 'exact registered source'):
            self.verify(f)

    def test_shadow_adoption_must_match_the_actual_child_session_manifest(self):
        f = self.fixture()
        raw = Path(f.selected[6]['capture_root'])
        self.write(raw.parents[1] / 'session_contract.json', dict(manifest_sha256='0' * 64))
        with self.assertRaisesRegex(ValueError, 'adoption differs from its actual session/source'):
            self.verify(f)

    def test_shadow_adoption_cannot_mix_another_config_even_if_session_rehashed(self):
        f = self.fixture()
        raw = Path(f.selected[6]['capture_root'])
        manifest = raw.parents[2] / 'motion_reuse_deck/manifest.json'
        adopted = release.read(manifest)
        adopted['config']['environment_tag'] = 'foggy13'
        self.write(manifest, adopted)
        self.write(raw.parents[1] / 'session_contract.json', dict(manifest_sha256=sha256(manifest)))
        with self.assertRaisesRegex(ValueError, 'adoption differs from its actual session/source'):
            self.verify(f)

    def test_official_parent_contract_failure_is_not_swallowed(self):
        f = self.fixture()
        with patch.object(completion, 'verify_parent', side_effect=RuntimeError('foreign source')):
            with self.assertRaisesRegex(RuntimeError, 'foreign source'):
                release.verify_smoke(f.registration, f.capture, f.full, f.runtime)

    def test_official_parent_missing_slot_is_not_approval(self):
        f = self.fixture()
        with self.assertRaisesRegex(ValueError, 'Official smoke reconstruction'):
            self.verify(f, missing=['source_0'])

    def test_audit_must_equal_selected_ledger_and_be_explicitly_accepted(self):
        for change in ('ledger_disagrees', 'accepted_missing', 'review_reason'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as folder:
                old_root, self.root = self.root, Path(folder)
                try:
                    f = self.fixture()
                    audit = release.read(Path(f.selected[0]['capture_root']) / 'audit.json')
                    if change == 'ledger_disagrees':
                        audit['extra'] = 'not in selected row'
                    elif change == 'accepted_missing':
                        audit.pop('accepted_candidate')
                        f.selected[0]['audit'] = copy.deepcopy(audit)
                    else:
                        audit['review_reasons'] = ['invisible_primary']
                        f.selected[0]['audit'] = copy.deepcopy(audit)
                    self.write(Path(f.selected[0]['capture_root']) / 'audit.json', audit)
                    self.jsonl(f.capture / 'selected_ledger.jsonl', f.selected)
                    with self.assertRaisesRegex(ValueError, 'native QA/contact'):
                        self.verify(f)
                finally:
                    self.root = old_root

    def test_changed_subset_or_helper_cannot_reuse_registration(self):
        f = self.fixture()
        doc = release.read(f.registration)
        doc['helper_sha256'] = '0' * 64
        self.write(f.registration, doc)
        with self.assertRaisesRegex(ValueError, 'helper/subset'):
            self.verify(f)

    def test_rehashed_subset_must_still_be_unchanged_full_source_entries(self):
        f = self.fixture()
        subset = release.read(f.subset)
        subset['scenes'][0]['scenario_sha256'] = '0' * 64
        self.write(f.subset, subset)
        doc = release.read(f.registration)
        doc['subset_manifest_sha256'] = sha256(f.subset)
        self.write(f.registration, doc)
        with self.assertRaisesRegex(ValueError, 'helper/subset'):
            self.verify(f)

    def test_changed_source_metadata_is_not_covered_by_native_success(self):
        f = self.fixture()
        self.write(f.entries[0]['metadata'], dict(foreign=True))
        with self.assertRaisesRegex(ValueError, 'Smoke source changed'):
            self.verify(f)

    def test_intersection_label_requires_actual_editor_policy(self):
        f = self.fixture()
        raw = Path(f.selected[4]['capture_root'])
        meta = release.read(raw / 'metadata.json')
        meta['runtime_actors'][0]['motion_path']['ground_path_evidence']['policy'] = 'unverified'
        self.write(raw / 'metadata.json', meta)
        with self.assertRaisesRegex(ValueError, 'actual editor path contract'):
            self.verify(f)

    def test_missing_collision_observation_is_not_approval(self):
        f = self.fixture()
        raw = Path(f.selected[0]['capture_root'])
        self.write(raw / 'collision_history.json', dict(summary=dict(
            observed=False, no_collision_verified=True, non_ego_uids=[])))
        with self.assertRaisesRegex(ValueError, 'native QA/contact'):
            self.verify(f)

    def test_missing_session_final_park_is_not_approval(self):
        f = self.fixture()
        raw = Path(f.selected[0]['capture_root'])
        self.write(raw.parents[1] / 'shutdown_safety.json', dict(park_verified=False))
        with self.assertRaisesRegex(ValueError, 'session final Park'):
            self.verify(f)

    def test_no_actual_new_shadow_observations_is_not_reuse_qualification(self):
        f = self.fixture()
        for row in f.selected[-2:]:
            self.write(Path(row['capture_root']) / 'metadata.json', dict(spec={}))
        with self.assertRaisesRegex(ValueError, 'Both registered ground reuses'):
            self.verify(f)


class ResumeEvidenceTest(TemporaryEvidenceTest):
    def fixture(self):
        manifest = self.write('manifest.json', dict(scenes=[dict(id='source')]))
        capture = self.root / 'capture'
        self.write(capture / 'status.json', dict(stage='complete', elapsed_sec=12.5))
        self.jsonl(capture / 'selected_ledger.jsonl', [dict(source_cell_id='source')])
        self.blob(capture / 'session/captures/source/CAMERA_1/raw_Intensity.png', b'pixels')
        self.jsonl(capture / 'session/event_cmd_requests.jsonl', [dict(event='capture')])
        snapshot = release.snapshot(capture)
        doc = dict(schema='base5_fresh_terminal_resume_v1', passed=True,
            additional_native_captures=0, raw_unchanged=True, selected_unchanged=True,
            elapsed_unchanged=True, manifest_sha256=sha256(manifest), capture_root=str(capture),
            verifier_sha256=sha256(terminal.__file__), before=snapshot, after=copy.deepcopy(snapshot))
        receipt = self.write('resume.json', doc)
        return SimpleNamespace(**locals())

    def test_current_inventory_is_rechecked_without_replaying_capture(self):
        f = self.fixture()
        with patch.object(terminal.fresh, 'run') as run:
            self.assertEqual(release.verify_resume(f.receipt, f.capture, f.manifest), f.doc)
        run.assert_not_called()

    def test_boolean_success_cannot_cover_changed_raw(self):
        f = self.fixture()
        self.blob(f.capture / 'session/captures/source/CAMERA_1/raw_Intensity.png', b'new pixels')
        with self.assertRaisesRegex(ValueError, 'inventory/raw/ledger/elapsed'):
            release.verify_resume(f.receipt, f.capture, f.manifest)

    def test_boolean_success_cannot_cover_changed_selection_or_elapsed(self):
        for key in ('selected', 'elapsed'):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder:
                old_root, self.root = self.root, Path(folder)
                try:
                    f = self.fixture()
                    if key == 'selected':
                        self.jsonl(f.capture / 'selected_ledger.jsonl', [dict(source_cell_id='foreign')])
                    else:
                        self.write(f.capture / 'status.json', dict(stage='complete', elapsed_sec=13.))
                    with self.assertRaisesRegex(ValueError, 'inventory/raw/ledger/elapsed'):
                        release.verify_resume(f.receipt, f.capture, f.manifest)
                finally:
                    self.root = old_root

    def test_foreign_manifest_and_verifier_receipts_are_rejected(self):
        f = self.fixture()
        f.doc['verifier_sha256'] = '0' * 64
        self.write(f.receipt, f.doc)
        with self.assertRaisesRegex(ValueError, 'inventory/raw/ledger/elapsed'):
            release.verify_resume(f.receipt, f.capture, f.manifest)
        f.doc['manifest_sha256'] = '0' * 64
        self.write(f.receipt, f.doc)
        with self.assertRaisesRegex(ValueError, 'Current-source terminal'):
            release.verify_resume(f.receipt, f.capture, f.manifest)

    def test_different_before_after_inventory_is_rejected(self):
        f = self.fixture()
        f.doc['before']['cumulative_elapsed_sec'] = 0
        self.write(f.receipt, f.doc)
        with self.assertRaisesRegex(ValueError, 'inventory/raw/ledger/elapsed'):
            release.verify_resume(f.receipt, f.capture, f.manifest)

    def test_empty_service_inventory_cannot_certify_no_action_replay(self):
        f = self.fixture()
        (f.capture / 'session/event_cmd_requests.jsonl').unlink()
        f.doc['before'] = f.doc['after'] = release.snapshot(f.capture)
        self.write(f.receipt, f.doc)
        with self.assertRaisesRegex(ValueError, 'inventory/raw/ledger/elapsed'):
            release.verify_resume(f.receipt, f.capture, f.manifest)


class TrainingEvidenceTest(TemporaryEvidenceTest):
    def fixture(self):
        raw = self.root / 'native/raw'
        self.blob(raw / 'cloud.bin', b'native xyzi')
        rows = [dict(id='source', status='accepted_candidate', scenario_sha256='a' * 64,
                     capture_root=str(raw))]
        source_ledger = self.jsonl('native/selected_ledger.jsonl', rows)
        source_snapshot = self.write('qualification/cohort/source_snapshot.json',
                                     training.snapshot(source_ledger, rows))
        package = self.root / 'portable'
        package_manifest = self.write(package / 'manifest.json', dict(kind='fixture'))
        helper = self.blob('conversion_helper.py')
        cohort = dict(frames=1, source_unchanged=True, lost_retained_component_ids=0,
            extra_retained_component_ids=0, full_rpy_points_recounted=True,
            upright_corner_enclosure_verified=True, source_ledger=str(source_ledger),
            source_ledger_sha256=sha256(source_ledger),
            source_snapshot_sha256=sha256(source_snapshot), training3_package=str(package),
            training3_manifest_sha256=sha256(package_manifest))
        doc = dict(schema=training.SCHEMA, report_script_sha256=sha256(training.__file__),
            training_started=False, capture_started=False, cohorts=dict(cohort=cohort),
            helper_sha256={str(helper): sha256(helper)})
        report = self.write('qualification/report.json', doc)
        return SimpleNamespace(**locals())

    def test_exact_cohort_inventory_and_portable_hash_validation_are_required(self):
        f = self.fixture()
        with patch.object(training.portable, 'validate') as validate:
            self.assertEqual(release.verify_training_inputs(f.report, dict(cohort=1)), f.doc)
        validate.assert_called_once_with(f.package, verify_hashes=True)

    def test_known_cli_relative_qualification_self_pin_is_normalized(self):
        f = self.fixture()
        script = Path(training.__file__).resolve()
        relative = str(script.relative_to(release.pins.ROOT))
        f.doc['helper_sha256'][relative] = sha256(script)
        self.write(f.report, f.doc)
        before = f.report.read_bytes()
        with patch.object(training.portable, 'validate') as validate:
            self.assertEqual(release.verify_training_inputs(f.report, dict(cohort=1)), f.doc)
        validate.assert_called_once_with(f.package, verify_hashes=True)
        self.assertEqual(f.report.read_bytes(), before)

    def test_conflicting_absolute_and_relative_qualification_self_pins_are_rejected(self):
        f = self.fixture()
        script = Path(training.__file__).resolve()
        relative = str(script.relative_to(release.pins.ROOT))
        f.doc['helper_sha256'][str(script)] = sha256(script)
        f.doc['helper_sha256'][relative] = '0' * 64
        self.write(f.report, f.doc)
        with self.assertRaisesRegex(ValueError, 'Conflicting qualification script pins'):
            release.verify_training_inputs(f.report, dict(cohort=1))

    def test_small_or_missing_cohort_cannot_claim_required_frame_counts(self):
        f = self.fixture()
        for expected in (dict(cohort=179), dict(cohort=1, absent=8)):
            with self.subTest(expected=expected):
                with self.assertRaisesRegex(ValueError, 'Exact native/Q90/upright'):
                    release.verify_training_inputs(f.report, expected)

    def test_changed_native_file_cannot_be_covered_by_source_unchanged_flag(self):
        f = self.fixture()
        self.blob(f.raw / 'cloud.bin', b'different native xyzi')
        with self.assertRaisesRegex(ValueError, 'Native training source inventory'):
            release.verify_training_inputs(f.report, dict(cohort=1))

    def test_changed_snapshot_even_with_matching_report_hash_is_recomputed(self):
        f = self.fixture()
        self.write(f.source_snapshot, {})
        f.doc['cohorts']['cohort']['source_snapshot_sha256'] = sha256(f.source_snapshot)
        self.write(f.report, f.doc)
        with self.assertRaisesRegex(ValueError, 'Native training source inventory'):
            release.verify_training_inputs(f.report, dict(cohort=1))

    def test_changed_training_manifest_or_payload_is_rejected(self):
        f = self.fixture()
        self.write(f.package_manifest, dict(changed=True))
        with self.assertRaisesRegex(ValueError, 'Training package manifest'):
            release.verify_training_inputs(f.report, dict(cohort=1))
        f.doc['cohorts']['cohort']['training3_manifest_sha256'] = sha256(f.package_manifest)
        self.write(f.report, f.doc)
        with patch.object(training.portable, 'validate', side_effect=ValueError('payload checksum')):
            with self.assertRaisesRegex(ValueError, 'payload checksum'):
                release.verify_training_inputs(f.report, dict(cohort=1))

    def test_changed_converter_is_rejected(self):
        f = self.fixture()
        self.blob(f.helper, b'changed conversion')
        with self.assertRaisesRegex(ValueError, 'Training input qualification helper'):
            release.verify_training_inputs(f.report, dict(cohort=1))

    def test_missing_geometry_certificate_is_rejected(self):
        f = self.fixture()
        f.doc['cohorts']['cohort']['upright_corner_enclosure_verified'] = False
        self.write(f.report, f.doc)
        with self.assertRaisesRegex(ValueError, 'provenance/geometry qualification'):
            release.verify_training_inputs(f.report, dict(cohort=1))


class FarAndJUnitEvidenceTest(TemporaryEvidenceTest):
    def fixture(self):
        runtime_file = self.blob('runtime.py')
        runtime = {str(runtime_file): sha256(runtime_file)}
        receipt = self.write('runtime_receipt.json', dict(runtime_hashes=runtime))
        results = self.blob('tests.xml', b'<testsuites><testsuite tests="1240" failures="0" errors="0" skipped="0"/></testsuites>')
        geometry = self.write('geometry.json', dict(config={}, scenes=[]))
        manifest = self.write('manifest.json', dict(config={release.reuse.KEY: {}}, scenes=[]))
        registration = self.write('far_registration.json', dict(source_candidate=str(geometry)))
        capture = self.root / 'far_capture'
        far_manifest = self.write('far_manifest.json', dict(scenes=[]))
        doc = dict(baseline=str(capture), manifest=str(far_manifest), registration=str(registration),
                   observations=[dict(id='source', native_lidar_points=1)], final_accepted=32, remaining=[])
        report = self.write('far_report.json', doc)
        args = SimpleNamespace(output=self.root / 'release', runtime_receipt=receipt,
            test_results=results, manifest=manifest, far_report=report,
            far_registration=registration, far_capture=capture, far_manifest=far_manifest)
        return SimpleNamespace(**locals())

    def complete_fixture(self):
        f = self.fixture()
        historical_sha = 'a' * 64
        meta = self.write('source_metadata.json', dict(spec={release.far.PLAN_KEY: dict(
            source_manifest=dict(sha256=historical_sha))}))
        entries = [dict(id='source_' + str(i), metadata=str(meta)) for i in range(160)]
        config = dict(obstacle_support_policy='fixture historical policy')
        self.write(f.geometry, dict(config=config, scenes=entries))
        self.write(f.manifest, dict(config=dict(config, **{release.reuse.KEY: {}}), scenes=entries))
        self.write(f.registration, dict(source_candidate=str(f.geometry),
            source_candidate_sha256=sha256(f.geometry)))
        f.doc.update(native_support_qualified_all_registered_targets=True, requested=32,
            registration_sha256=sha256(f.registration), source_input_sha256=f.runtime)
        self.write(f.report, f.doc)
        f.args.historical_release = self.write('historical_release.json', dict(
            bulk_ready=True, completion_live_qualified=True,
            environments=dict(sunny13=dict(manifest_sha256=historical_sha)),
            estimated_5k_hours=9., estimated_5k_gib=30.))
        for name in ('smoke_registration', 'smoke_manifest', 'smoke_resume', 'far_resume',
                     'training_inputs', 'current_training_inputs'):
            setattr(f.args, name, self.write(name + '.json', {}))
        f.args.smoke_capture = self.root / 'smoke_capture'
        f.args.capture_output = self.root / 'new_5k_capture'
        f.historical_training = dict(cohorts=dict(
            static64=dict(retained_gt=250), moving8=dict(retained_gt=30),
            range53=dict(retained_gt=200), far32=dict(retained_gt=90),
            compound22=dict(retained_gt=31)))
        f.current_training = dict(cohorts=dict(
            current_far32=dict(far_targets=32,
                source_ledger=str(f.args.far_capture / 'selected_ledger.jsonl')),
            current_smoke8=dict(far_targets=0,
                source_ledger=str(f.args.smoke_capture / 'selected_ledger.jsonl'))))
        return f

    def freeze_with_qualified_dependencies(self, f):
        # This fixture isolates release assembly/count gates. The smoke,
        # inventory, parent and package implementations are exercised above.
        with ExitStack() as stack:
            stack.enter_context(patch.object(release.pins, 'collect_runtime', return_value=f.runtime))
            stack.enter_context(patch.object(far_report, 'report', return_value=copy.deepcopy(f.doc)))
            stack.enter_context(patch.object(release.reuse, 'validate_manifest', return_value=f.runtime))
            stack.enter_context(patch.object(release, 'verify_smoke', return_value=dict(
                accepted=8, shadow_preparation_reused=2)))
            stack.enter_context(patch.object(release, 'verify_resume'))
            verify_training = stack.enter_context(patch.object(release, 'verify_training_inputs',
                side_effect=[f.historical_training, f.current_training]))
            stack.enter_context(patch.object(release.pins, 'collect_inputs',
                return_value={str(f.manifest): sha256(f.manifest)}))
            stack.enter_context(patch.object(release.audit_base5_scenarios, 'audit',
                return_value=dict(status='OFFLINE_PASS', unique_payloads=5000)))
            output = release.freeze(f.args)
            return output, verify_training.call_args_list

    def test_release_keeps_old_601_separate_from_current40_and_freezes_only_sunny13(self):
        f = self.complete_fixture()
        output, calls = self.freeze_with_qualified_dependencies(f)
        self.assertEqual(calls[0].args, (f.args.training_inputs, dict(
            static64=64, moving8=8, range53=53, far32=32, compound22=22)))
        self.assertEqual(calls[1].args, (f.args.current_training_inputs,
                                        dict(current_far32=32, current_smoke8=8)))
        doc = release.read(output)
        self.assertEqual(doc['qualified_environments'], ['sunny13'])
        self.assertTrue(doc['bulk_ready'])
        self.assertFalse(doc['bulk_started'])
        self.assertTrue(doc['qualification_basis']['historical_evidence_is_not_new_runtime_native'])
        self.assertNotEqual(doc['qualification_basis']['training_input_qualification'],
                            doc['qualification_basis']['current_training_input_qualification'])

    def test_wrong_old_gt_total_cannot_claim_historical_601_qualification(self):
        f = self.complete_fixture()
        f.historical_training['cohorts']['static64']['retained_gt'] -= 1
        with self.assertRaisesRegex(ValueError, '601 GT conversion differs'):
            self.freeze_with_qualified_dependencies(f)
        self.assertFalse(f.args.output.exists())

    def test_one_lost_current_far_target_blocks_release(self):
        f = self.complete_fixture()
        f.current_training['cohorts']['current_far32']['far_targets'] = 31
        with self.assertRaisesRegex(ValueError, 'Every current far target'):
            self.freeze_with_qualified_dependencies(f)
        self.assertFalse(f.args.output.exists())

    def test_current_conversion_from_another_capture_is_not_accepted(self):
        f = self.complete_fixture()
        f.current_training['cohorts']['current_far32']['source_ledger'] = '/foreign/selected_ledger.jsonl'
        with self.assertRaisesRegex(ValueError, 'bound to another capture'):
            self.freeze_with_qualified_dependencies(f)
        self.assertFalse(f.args.output.exists())

    def test_far_report_is_bound_to_cli_capture_manifest_and_registration(self):
        f = self.fixture()
        f.doc['baseline'] = str(self.root / 'foreign_capture')
        self.write(f.report, f.doc)
        with patch.object(release.pins, 'collect_runtime', return_value=f.runtime):
            with self.assertRaisesRegex(ValueError, 'another capture/manifest/registration'):
                release.freeze(f.args)
        self.assertFalse(f.args.output.exists())

    def test_far_native_observations_are_reconstructed_not_trusted(self):
        f = self.fixture()
        regenerated = copy.deepcopy(f.doc)
        regenerated['observations'][0]['native_lidar_points'] = 0
        with patch.object(release.pins, 'collect_runtime', return_value=f.runtime), \
                patch.object(far_report, 'report', return_value=regenerated) as reconstruct:
            with self.assertRaisesRegex(ValueError, 'Far native reconstruction differs'):
                release.freeze(f.args)
        reconstruct.assert_called_once_with(f.registration, f.far_manifest, f.capture,
            f.args.output.parent / (f.args.output.name + '_far_reconstructed.json'))
        self.assertFalse(f.args.output.exists())

    def test_small_failing_or_empty_junit_is_not_full_cpu_regression(self):
        for attributes in ('tests="5"', 'tests="1240" failures="1"', 'tests="1240" errors="1"'):
            with self.subTest(attributes=attributes):
                path = self.blob('tests.xml', ('<testsuite ' + attributes + '/>').encode())
                with self.assertRaisesRegex(ValueError, 'Complete passing'):
                    release.test_results(path)
        path = self.blob('tests.xml', b'<testsuites/>')
        with self.assertRaisesRegex(ValueError, 'no test suites'):
            release.test_results(path)

    def test_all_skipped_junit_is_not_executed_cpu_regression(self):
        path = self.blob('tests.xml', b'<testsuites><testsuite tests="1240" '
            b'failures="0" errors="0" skipped="1240"/></testsuites>')
        with self.assertRaisesRegex(ValueError, 'Complete passing'):
            release.test_results(path)


if __name__ == '__main__':
    unittest.main()
