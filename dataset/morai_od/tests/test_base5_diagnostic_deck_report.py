"""CPU semantics for paired-ledger reports, without raw sensors or ROS."""
import copy
import json
import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import report_base5_diagnostic_deck as report


class DiagnosticReportTest(unittest.TestCase):
    def provenance_fixture(self, root):
        source, capture = root / 'deck', root / 'capture'
        source.mkdir()
        capture.mkdir()
        scenario = dict(egoVehicle=dict(initPosition=dict(pos=dict(x=1., y=2., z=28.))))
        metadata = dict(spec=dict(seed=7, family='highway'), ego=dict(x=1., y=2., yaw=3.),
            runtime_actors=[dict(unique_id=8500, data_id=4, motion='static',
                                position=dict(x=90., y=2., z=28.))],
            far_vehicle_diagnostic=dict(distance_m=85, role='front', lane='same', not_training=True))
        for folder, name in ((source, 'probe.json'), (capture, 'scenario.json')):
            (folder / name).write_text(json.dumps(scenario))
        (source / 'probe_metadata.json').write_text(json.dumps(metadata))
        (capture / 'metadata.json').write_text(json.dumps(metadata))
        entry = dict(id='probe', scenario='probe.json', metadata='probe_metadata.json',
                     scenario_sha256=report.sha256(source / 'probe.json'))
        return source, capture, entry, metadata

    def test_capture_provenance_accepts_only_additional_capture_fields(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, capture, entry, metadata = self.provenance_fixture(Path(temporary))
            before = {str(path): report.sha256(path) for path in source.iterdir()}
            expanded = copy.deepcopy(metadata)
            expanded['capture_safety'] = dict(park_verified=True)
            expanded['runtime_actors'][0]['capture_diagnostic'] = dict(observed=True)
            (capture / 'metadata.json').write_text(json.dumps(expanded))
            observed, _ = report.capture_provenance(capture, entry, source)
            self.assertEqual(observed, expanded)
            self.assertEqual(before, {str(path): report.sha256(path) for path in source.iterdir()})

    def test_swapped_raw_scenario_rejected_despite_unchanged_ledger_sha(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, capture, entry, _ = self.provenance_fixture(Path(temporary))
            (capture / 'scenario.json').write_text(json.dumps(dict(swapped_native_scenario=True)))
            with self.assertRaisesRegex(ValueError, 'Native capture scenario checksum mismatch'):
                report.capture_provenance(capture, entry, source)

    def test_changed_source_scenario_is_not_adopted_as_a_new_reference(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, capture, entry, _ = self.provenance_fixture(Path(temporary))
            (source / 'probe.json').write_text(json.dumps(dict(changed_source=True)))
            with self.assertRaisesRegex(ValueError, 'deck scenario checksum mismatch'):
                report.capture_provenance(capture, entry, source)

    def test_swapped_capture_metadata_conditions_pose_and_population_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            source, capture, entry, metadata = self.provenance_fixture(Path(temporary))
            for change in ('condition', 'ego', 'actor_identity', 'actor_count', 'missing_source_field'):
                altered = copy.deepcopy(metadata)
                if change == 'condition': altered['far_vehicle_diagnostic']['lane'] = 'adjacent'
                if change == 'ego': altered['ego']['yaw'] += 180.
                if change == 'actor_identity': altered['runtime_actors'][0]['data_id'] += 1
                if change == 'actor_count': altered['runtime_actors'].append(copy.deepcopy(altered['runtime_actors'][0]))
                if change == 'missing_source_field': altered.pop('spec')
                (capture / 'metadata.json').write_text(json.dumps(altered))
                with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'Capture/deck metadata'):
                    report.capture_provenance(capture, entry, source)

    def test_serialized_pivot_not_initializer_is_z_reference(self):
        actor = dict(unique_id=8500, source_list='vehicleList', position=dict(z=28.))
        scene = dict(vehicleList=[dict(UNIQUEID=8500, initPosition=dict(pos=dict(z=28.36)))])
        telemetry = {key: dict(npc_list=[dict(unique_id=8500, position=dict(z=z))])
                     for key, z in (('objects_before', 28.32), ('objects_after', 28.321))}
        summary = report.pivot_z_summary(scene, actor, telemetry)
        self.assertAlmostEqual(summary['serialized_minus_generator_reference_z_m'], .36)
        self.assertAlmostEqual(summary['runtime_minus_serialized_pivot_z_m'], -.04)
        self.assertAlmostEqual(summary['capture_window_pivot_z_change_m'], .001)

    def test_no_runtime_z_stays_unknown(self):
        actor = dict(unique_id=8500, source_list='objectList', position=dict(z=28.))
        scene = dict(objectList=[dict(UNIQUEID=8500, pos=dict(z=28.))])
        summary = report.pivot_z_summary(scene, actor, {})
        self.assertIsNone(summary['runtime_minus_serialized_pivot_z_m'])

    def test_ego_and_lidar_ranges_remain_distinct(self):
        summary = report.range_summary(dict(range_xy_m=7.145, range_ego_xy_m=8.),
                                       dict(planned_center_distance_m=8.))
        self.assertEqual(summary['range_lidar_xy_m'], 7.145)
        self.assertEqual(summary['range_ego_xy_m'], 8.)
        self.assertEqual(summary['planned_center_distance_ego_xy_m'], 8.)

    def test_unfinished_ledger_is_not_silently_accepted(self):
        with patch.object(Path, 'read_bytes', return_value=b'{"id":"example"}'):
            with self.assertRaisesRegex(ValueError, 'unfinished trailing row'):
                report.completed_lines(Path('unused-ledger'))
        with patch.object(Path, 'read_bytes', return_value=b'{"id":"example"}\n'):
            self.assertEqual(report.completed_lines(Path('unused-ledger')), [dict(id='example')])

    def test_unknown_points_do_not_imply_bad_camera(self):
        rows = [dict(id='a', asset='Sedan', native_lidar_points=0,
                     camera_gate512=True, best_model_mask_pixels=100, best_bbox_short_side_px=10)]
        summary = report.support_summary(rows)
        self.assertEqual(summary['camera_only'], 1)
        self.assertEqual(summary['gt'], 1)
        self.assertEqual(summary['scenes'], 1)


if __name__ == '__main__':
    unittest.main()
