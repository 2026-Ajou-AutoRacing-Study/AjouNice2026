"""Read-only report contracts; no ROS, native capture or learned AP claims."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report_base5_far_vehicle_qualification as report


class ReportTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, name, value):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2) + '\n')
        return path

    def row(self, ident='slot', status='accepted_candidate'):
        return dict(id=ident, source_cell_id=ident, status=status,
                    capture_root=str(self.root / 'capture' / ident),
                    scenario_sha256=ident + '-unique-payload', audit=dict(review_reasons=[]))

    def observation(self, ident, band=(95, 99), role='front', lane='same', points=0):
        return dict(id=ident, distance_bounds_m=list(band), role=role, lane_relation=lane,
            asset='ordinary-car', native_lidar_points=points,
            filtered_model_lidar_points=points, native_points_bucket='0' if points == 0
            else '1_4' if points < 5 else '5plus', camera512_gate=True,
            camera_only=points == 0, center_inside_current_model_roi=True,
            best_model512_pixels=80, best_model512_extent_px=[10., 8.],
            actual_distance_ego_xy_m=97., actual_distance_lidar_xy_m=96.145)

    def case(self, band=(95, 99), role='front', lane='same'):
        return dict(registered=dict(stratum=dict(distance_bounds_m=list(band),
                                                 role=role, lane_relation=lane)))

    def test_output_refused_before_reading_any_missing_input(self):
        output = self.write('existing.json', {})
        with self.assertRaises(FileExistsError):
            report.report('missing-reg', 'missing-manifest', 'missing-parent', output)

    def test_unterminated_and_corrupt_ledgers_are_rejected_without_repair(self):
        path = self.root / 'ledger.jsonl'
        for payload in (b'{"x":1}', b'{"x":1}\ntruncated\n'):
            path.write_bytes(payload)
            with self.assertRaises(ValueError):
                report.ledger(path)
            self.assertEqual(path.read_bytes(), payload)

    def test_snapshot_includes_failed_raw_and_parent_status_content(self):
        self.write('parent/status.json', dict(stage='complete', elapsed_sec=1.))
        raw = self.write('parent/failed/captures/a/audit.json', dict(review=True))
        before = report.snapshot(self.root / 'parent')
        self.assertIn('failed/captures/a/audit.json', before)
        self.assertIn('status.json', before)
        raw.write_text('{"review":false}\n')
        self.assertNotEqual(report.snapshot(self.root / 'parent'), before)

    def test_native_selection_rejects_duplicate_slot_raw_and_review(self):
        first = self.row('a')
        for second in (copy.deepcopy(first), self.row('foreign'),
                       dict(self.row('b'), capture_root=first['capture_root']),
                       dict(self.row('b'), scenario_sha256=first['scenario_sha256']),
                       self.row('b', status='review')):
            with self.subTest(second=second):
                with self.assertRaisesRegex(ValueError, 'native selection'):
                    report.check_selection([first, second], {'a', 'b'})

    def test_summary_uses_registered_dynamic_bands_and_keeps_camera_only(self):
        cases, observations = {}, []
        bands = ((60, 80), (80, 90), (90, 95), (95, 99))
        for index, band in enumerate(bands):
            ident = 'case' + str(index)
            cases[ident] = self.case(band)
            observations.append(self.observation(ident, band, points=(0, 1, 4, 5)[index]))
        rows = report.summarize(observations, cases)
        self.assertEqual(len(rows), 16)
        self.assertEqual(rows['95_99/front/same']['accepted'], 1)
        self.assertEqual(rows['95_99/front/same']['native_lidar_buckets'], {'5plus': 1})
        self.assertEqual(rows['60_80/front/same']['camera_only'], 1)
        self.assertEqual(rows['80_90/front/same']['native_lidar_buckets'], {'1_4': 1})
        self.assertEqual(rows['90_95/front/same']['native_lidar_buckets'], {'1_4': 1})

    def test_summary_reports_missing_targets_without_promoting_another_primary(self):
        cases = {'a': self.case(), 'b': self.case()}
        rows = report.summarize([self.observation('a')], cases)
        self.assertEqual(rows['95_99/front/same']['requested'], 2)
        self.assertEqual(rows['95_99/front/same']['accepted'], 1)
        self.assertEqual(rows['95_99/front/same']['remaining'], ['b'])
        with self.assertRaisesRegex(ValueError, 'Duplicate or foreign'):
            report.summarize([self.observation('a'), self.observation('a')], cases)

    def registration_fixture(self):
        manifest = self.write('manifest.json', dict(count=1, config={}, scenes=[dict(id='a')]))
        dependency = self.write('dependency.json', {'frozen': True})
        registered = dict(policy=report.qualification.POLICY,
            subset_manifest_sha256=report.sha256(manifest), source_count=5000,
            selected=[dict(id='a')], live_qualified=False, bulk_started=False,
            helper_sha256=report.sha256(report.qualification.__file__),
            core_local_import_closure_sha256={str(dependency): report.sha256(dependency)},
            frozen_source_dependency_sha256={str(dependency): report.sha256(dependency)},
            full_source_offline_audit_sha256={str(dependency): report.sha256(dependency)})
        registration = self.write('registration.json', registered)
        return registration, manifest, dependency, registered

    def test_registration_subset_sha_drift_is_rejected(self):
        registration, manifest, _, _ = self.registration_fixture()
        manifest.write_text('{"changed":true}\n')
        with self.assertRaisesRegex(ValueError, 'source/subset/helper'):
            report.source_inputs(registration, manifest)

    def test_registered_runtime_and_source_dependency_drift_is_rejected(self):
        registration, manifest, dependency, _ = self.registration_fixture()
        dependency.write_text('{"frozen":false}\n')
        with self.assertRaisesRegex(ValueError, 'changed'):
            report.source_inputs(registration, manifest)

    def native_fixture(self, points=1):
        capture = self.root / 'native'
        plan = dict(target_uid=8501, target_ordinal=1, target_name='ordinary-car',
            target_data_id=30100001, distance_bounds_m=[80., 90.], role='front',
            lane_relation='same', source={})
        frozen_scene = self.write('frozen/scenario.json', {'frozen': True})
        frozen_meta = self.write('frozen/metadata.json', dict(runtime_actors=[{},
            dict(planned_center_distance_m=35.)]))
        plan['source'] = dict(scenario=dict(path=str(frozen_scene)),
                              metadata=dict(path=str(frozen_meta)))
        actual = dict(spec={report.far.PLAN_KEY: plan},
            runtime_actors=[{}, dict(unique_id=8501, planned_center_distance_m=85.)],
            lidar_to_ego=np.eye(4).tolist())
        self.write('native/metadata.json', actual)
        scene = self.write('native/scenario.json', {'captured': True})
        folder = capture / 'LIDAR_5'
        folder.mkdir(parents=True)
        cloud = np.asarray([[0., 0., 0., 0.], [1., 0., 0., 0.]]
                           + [[85., 0., 0., 1.] for _ in range(points)], np.float32)
        (folder / 'frame_Intensity.bin').write_bytes(cloud.tobytes())
        camera_folder = capture / 'CAMERA_1'
        camera_folder.mkdir()
        image = np.zeros((48, 64, 4), np.uint8)
        image[:, :, 3] = 255
        image[24:27, 20:22] = [7, 11, 13, 255]
        Image.fromarray(image).save(camera_folder / 'frame_Instance.png')
        (camera_folder / 'frame_BBox2D.txt').write_text(
            'Sedan 20 24 22 27 8501 [7, 11, 13, 255]\n')
        cameras = report.camera_evidence(capture, 8501, 'Sedan', model512=True)
        component = dict(parent_uid=8501, component_id='8501:Sedan:0',
            raw_class='Sedan', training_class='vehicle', center_lidar_m=[85., 0., 0.],
            center_ego_m=[85., 0., 0.], range_ego_xy_m=85., size_m=[4., 2., 1.5],
            rotation_rpy=[0., 0., 0.], lidar_points=points, cameras=cameras)
        with patch.object(report.far, 'verify_preserved', return_value=plan):
            reasons, proof = report.native.review(actual, [component])
        self.assertEqual(reasons, [])
        audit = dict(components=[component], accepted_candidate=True,
                     review_reasons=[], far_vehicle_native=proof)
        self.write('native/audit.json', audit)
        case = dict(entry=dict(id='slot'), metadata=actual, registered=dict(plan=plan))
        selected = dict(id='native-slot', capture_root=str(capture),
                        scenario_sha256=report.sha256(scene), audit=audit)
        return case, selected, plan

    def test_target_observation_recomputes_points_and_current512_mask(self):
        case, selected, plan = self.native_fixture(points=1)
        before = report.snapshot(self.root / 'native')
        with patch.object(report.far, 'verify_preserved', return_value=plan), patch.object(
                report.far, 'verify_payload'):
            row = report.target_observation(case, selected)
        self.assertEqual(row['native_lidar_points'], 1)
        self.assertEqual(row['filtered_model_lidar_points'], 1)
        self.assertTrue(row['camera512_gate'])
        self.assertTrue(row['frozen_primary_and_all_non_target_preserved'])
        self.assertEqual(before, report.snapshot(self.root / 'native'))

    def test_target_observation_keeps_genuinely_camera_only_far_vehicle(self):
        case, selected, plan = self.native_fixture(points=0)
        with patch.object(report.far, 'verify_preserved', return_value=plan), patch.object(
                report.far, 'verify_payload'):
            row = report.target_observation(case, selected)
        self.assertTrue(row['camera_only'])
        self.assertEqual(row['native_points_bucket'], '0')
        self.assertGreaterEqual(row['best_model512_pixels'], 16)

    def test_target_observation_rejects_changed_raw_point_support(self):
        case, selected, plan = self.native_fixture(points=1)
        path = self.root / 'native/LIDAR_5/frame_Intensity.bin'
        path.write_bytes(np.asarray([[1., 0., 0., 1.]], np.float32).tobytes())
        with patch.object(report.far, 'verify_preserved', return_value=plan), patch.object(
                report.far, 'verify_payload'):
            with self.assertRaisesRegex(ValueError, 'point support'):
                report.target_observation(case, selected)

    def test_target_observation_rejects_ego_lidar_calibration_disagreement(self):
        case, selected, plan = self.native_fixture(points=1)
        path = self.root / 'native/metadata.json'
        actual = json.loads(path.read_text())
        actual['lidar_to_ego'][0][3] = .855
        path.write_text(json.dumps(actual))
        with patch.object(report.far, 'verify_preserved', return_value=plan), patch.object(
                report.far, 'verify_payload'):
            with self.assertRaisesRegex(ValueError, 'LiDAR calibration'):
                report.target_observation(case, selected)

    def report_fixture(self):
        baseline = self.root / 'parent'
        self.write('parent/status.json', dict(stage='complete', elapsed_sec=1.))
        self.write('parent/rejected/captures/old/audit.json', {'review': True})
        rows = [self.row('a'), self.row('b')]
        for row in rows:
            Path(row['capture_root']).mkdir(parents=True)
        (baseline / 'selected_ledger.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in rows))
        manifest = self.write('manifest.json', {})
        registration = self.write('registration.json', {})
        cases = {ident: self.case() for ident in ('a', 'b')}
        sources = {str(manifest): report.sha256(manifest)}
        return baseline, rows, manifest, registration, cases, sources

    def test_end_to_end_parent_verification_writes_only_new_report(self):
        baseline, rows, manifest, registration, cases, sources = self.report_fixture()
        before = report.snapshot(baseline)
        output = self.root / 'new_report.json'
        document = dict(source_candidate=str(self.root / 'full/manifest.json'))
        with patch.object(report, 'source_inputs', return_value=(
                registration, document, {}, cases, sources)), patch.object(
                report.completion, 'verify_parent', return_value=(rows, [], 1.)) as verify, patch.object(
                report, 'target_observation', side_effect=lambda case, row:
                    self.observation(row['source_cell_id'])):
            result = report.report(registration, manifest, baseline, output)
        verify.assert_called_once_with(manifest, baseline)
        self.assertTrue(output.is_file())
        self.assertEqual(result['final_accepted'], 2)
        self.assertFalse(result['detector_ap_improvement_claim'])
        limitations = ' '.join(result['limitations'])
        self.assertIn('explicit native acceptance gate', limitations)
        self.assertIn('does not certify actual LiDAR 95+ m coverage', limitations)
        self.assertNotIn('not a GT rewrite or a new capture acceptance rule', limitations)
        self.assertTrue(result['immutable_evidence']['after_equal'])
        self.assertEqual(before, report.snapshot(baseline))

    def test_report_refuses_verifier_that_changes_existing_status(self):
        baseline, rows, manifest, registration, cases, sources = self.report_fixture()
        output = self.root / 'must_not_exist.json'
        def mutating_verifier(*unused):
            self.write('parent/status.json', dict(stage='complete', elapsed_sec=2.))
            return rows, [], 1.
        with patch.object(report, 'source_inputs', return_value=(
                registration, dict(source_candidate=str(self.root / 'full/manifest.json')),
                {}, cases, sources)), patch.object(report.completion, 'verify_parent',
                side_effect=mutating_verifier), patch.object(report, 'target_observation',
                side_effect=lambda case, row: self.observation(row['source_cell_id'])):
            with self.assertRaisesRegex(ValueError, 'changed capture RAW/journal/status'):
                report.report(registration, manifest, baseline, output)
        self.assertFalse(output.exists())

    def completion_fixture(self, recover=False):
        baseline, rows, manifest, _, cases, _ = self.report_fixture()
        parent_rows = rows[:1] if recover else rows
        (baseline / 'selected_ledger.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in parent_rows))
        for name in ('quota', 'batch_contract', 'fresh_runtime_contract'):
            self.write('parent/' + name + '.json', {})
        names = ('run_base5_fresh_completion.py', 'base5_spatial_diversity.py',
            'run_base5_flexible_completion.py', 'run_base5_adaptive_completion.py',
            'run_base5_capture_optimized.py', 'run_base5_capture_campaign_spatial.py',
            'base5_secondary_far_vehicle.py', 'base5_far_vehicle_native.py')
        contract = dict(policy=report.completion.POLICY,
            source_sha256=report.sha256(manifest), baseline=str(baseline), attempts=3,
            parent_hashes={name: report.sha256(baseline / name) for name in
                ('selected_ledger.jsonl', 'quota.json', 'batch_contract.json', 'fresh_runtime_contract.json')},
            code_hashes={name: report.sha256(Path(report.__file__).with_name(name)) for name in names},
            capture_contract={'test': 'capture-contract'})
        attempts = [dict(id='b', ordinal=1, result=rows[1], group='group',
                         manifest='prepared.json', delta={})] if recover else []
        state = dict(stage='complete', contract=contract, baseline_selected=parent_rows,
            deficits=['b'] if recover else [], source_order=['a', 'b'], attempts=attempts)
        selected, missing = report.completion.merged(state)
        completed = self.root / 'completed'
        self.write('completed/state.json', state)
        self.write('completed/status.json', dict(stage='complete', requested=2,
            accepted_candidate=2, remaining_deficit=0, review=0))
        self.write('completed/quota.json', dict(target=2, accepted=2, deficit=0,
                                               pending=0, cells=[]))
        (completed / 'selected_ledger.jsonl').write_text(
            ''.join(json.dumps(row) + '\n' for row in selected))
        for case in cases.values():
            case['metadata'] = {}
        return completed, baseline, manifest, cases, parent_rows, state, selected

    def test_zero_deficit_terminal_completion_is_read_only_and_does_not_verify_new_attempt(self):
        completed, baseline, manifest, cases, parent_rows, _, selected = self.completion_fixture()
        before = report.snapshot(completed)
        pools = {'pools': {name: [] for name in report.completion.POOLS}}
        with patch.object(report.completion.adaptive.fa.batch.campaign, 'contract',
                return_value={'test': 'capture-contract'}), patch.object(
                report.completion.spatial.gen, 'context', return_value=pools), patch.object(
                report.completion, 'verify') as verify:
            rows, missing, _ = report.verify_completion(completed, baseline, manifest,
                {}, parent_rows, [], cases)
        self.assertEqual(rows, selected)
        self.assertEqual(missing, [])
        verify.assert_not_called()
        self.assertEqual(before, report.snapshot(completed))

    def test_real_completion_attempt_is_officially_verified_before_selection(self):
        completed, baseline, manifest, cases, parent_rows, state, selected = self.completion_fixture(True)
        pools = {'pools': {name: [] for name in report.completion.POOLS}}
        with patch.object(report.completion.adaptive.fa.batch.campaign, 'contract',
                return_value={'test': 'capture-contract'}), patch.object(
                report.completion.spatial.gen, 'context', return_value=pools), patch.object(
                report.completion, 'verify', return_value=state['attempts'][0]['result']) as verify:
            rows, missing, _ = report.verify_completion(completed, baseline, manifest,
                {}, parent_rows, ['b'], cases)
        self.assertEqual(rows, selected)
        self.assertEqual(missing, [])
        verify.assert_called_once()
        self.assertEqual(verify.call_args.args[0], state['attempts'][0])

    def test_completion_result_disagreement_is_rejected(self):
        completed, baseline, manifest, cases, parent_rows, _, _ = self.completion_fixture(True)
        pools = {'pools': {name: [] for name in report.completion.POOLS}}
        with patch.object(report.completion.adaptive.fa.batch.campaign, 'contract',
                return_value={'test': 'capture-contract'}), patch.object(
                report.completion.spatial.gen, 'context', return_value=pools), patch.object(
                report.completion, 'verify', return_value={'status': 'review'}):
            with self.assertRaisesRegex(ValueError, 'official raw verification differs'):
                report.verify_completion(completed, baseline, manifest,
                    {}, parent_rows, ['b'], cases)


if __name__ == '__main__':
    unittest.main()
