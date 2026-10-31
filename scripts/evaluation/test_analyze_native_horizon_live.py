"""Native horizon audit protocols: no ROS master, publishers or simulator."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import tempfile
import unittest

from analyze_native_horizon_live import (
    class_horizons, compare_native, comparison_summary, drive_window, gap_summary,
    held_time, static_wire,
)
from qualify_e12_mixed_horizon import NODE_PATH, build_message, wire
import test_qualify_e12_mixed_horizon as fixtures


class NativeHorizonAuditTest(unittest.TestCase):
    def output(self, count):
        from autoware_perception_msgs.msg import ObjectClassification
        value = fixtures.MixedHorizonQualificationTest().output(count)
        value.objects[0].classification = [ObjectClassification(label=1, probability=1.)]
        return value

    def test_native_exact_extension_is_measured_not_assumed(self):
        baseline, longer = self.output(31), self.output(61)
        before3, before6 = wire(baseline), wire(longer)
        rows = compare_native(baseline, longer)
        self.assertTrue(rows[0]['whole_ordered_prefix_exact'])
        self.assertEqual(rows[0]['nearest_prefix_representatives'][0]['mean_xy_m'], 0.)
        self.assertEqual(wire(baseline), before3)
        self.assertEqual(wire(longer), before6)

    def test_changed_prefix_is_reported_without_false_rejection(self):
        baseline, longer = self.output(31), self.output(61)
        longer.objects[0].kinematics.predicted_paths[0].path[30].position.x += 3.
        rows = compare_native(baseline, longer)
        self.assertFalse(rows[0]['ordered_prefix_geometry_exact'])
        self.assertAlmostEqual(rows[0]['top1_same_index_mean_xy_m'], .1)
        self.assertEqual(rows[0]['top1_same_index_terminal3_xy_m'], 3.)

    def test_confidence_change_is_separate_from_geometry(self):
        baseline, longer = self.output(31), self.output(61)
        longer.objects[0].kinematics.predicted_paths[0].confidence = .3
        row = compare_native(baseline, longer)[0]
        self.assertTrue(row['ordered_prefix_geometry_exact'])
        self.assertFalse(row['ordered_confidence_exact'])
        self.assertFalse(row['whole_ordered_prefix_exact'])

    def test_route_representative_reordering_is_not_top1_position_error(self):
        baseline, longer = self.output(31), self.output(61)
        for msg in (baseline, longer):
            first = msg.objects[0].kinematics.predicted_paths[0]
            second = deepcopy(first)
            for pose in second.path:
                pose.position.y += 10.
            msg.objects[0].kinematics.predicted_paths.append(second)
        longer.objects[0].kinematics.predicted_paths.reverse()
        row = compare_native(baseline, longer)[0]
        self.assertFalse(row['ordered_prefix_geometry_exact'])
        self.assertEqual(row['prefix_geometry_index_changes'], 2)
        self.assertEqual(row['top1_same_index_mean_xy_m'], 10.)
        self.assertTrue(all(p['exact_geometry'] for p in row['nearest_prefix_representatives']))
        self.assertTrue(all(p['mean_xy_m'] == 0. for p in row['nearest_prefix_representatives']))

    def test_native_topk_count_can_change(self):
        baseline, longer = self.output(31), self.output(61)
        longer.objects[0].kinematics.predicted_paths.append(
            deepcopy(longer.objects[0].kinematics.predicted_paths[0]))
        row = compare_native(baseline, longer)[0]
        self.assertTrue(row['path_count_changed'])
        self.assertEqual((row['topk3'], row['topk6']), (1, 2))

    def test_reverse_prefix_reveals_added_branch_and_confidence_gate_changes(self):
        baseline, longer = self.output(31), self.output(61)
        new = deepcopy(longer.objects[0].kinematics.predicted_paths[0])
        for pose in new.path[1:]:
            pose.position.y = 5.
        new.confidence = .4
        longer.objects[0].kinematics.predicted_paths.append(new)
        summary = comparison_summary(compare_native(baseline, longer))['CAR']
        self.assertEqual(summary['native6_net_additional_path_observations'], 1)
        self.assertEqual(summary['native6_prefix_not_exactly_present_in_native3'], 1)
        self.assertEqual(summary['native6_eligible_prefix_without_eligible_exact_native3'], 1)
        self.assertEqual(summary['native3_eligible_paths_at_0_05'], 1)
        self.assertEqual(summary['native6_eligible_paths_at_0_05'], 2)

    def test_reverse_xy_tie_does_not_hide_exact_yaw_match_in_another_path(self):
        baseline, longer = self.output(31), self.output(61)
        yaw_changed = deepcopy(baseline.objects[0].kinematics.predicted_paths[0])
        for pose in yaw_changed.path:
            pose.orientation.z, pose.orientation.w = .6, .8
        baseline.objects[0].kinematics.predicted_paths.insert(0, yaw_changed)
        row = compare_native(baseline, longer)[0]
        # Pure XY ties deliberately choose the first index. It has wrong yaw,
        # while the second path is a complete pose+time-step exact prefix.
        reverse = row['native6_to_native3_nearest_prefix'][0]
        self.assertFalse(reverse['exact_geometry'])
        self.assertTrue(reverse['has_exact_native3_prefix'])
        summary = comparison_summary([row])['CAR']
        self.assertEqual(summary['native6_to_native3_prefix_exact_geometry'], 0)
        self.assertEqual(summary['native6_to_native3_any_exact_prefix'], 1)
        self.assertEqual(summary['native6_prefix_not_exactly_present_in_native3'], 0)

    def test_current_state_or_uuid_changes_are_not_tolerated(self):
        baseline, longer = self.output(31), self.output(61)
        longer.objects[0].shape.dimensions.x += .1
        with self.assertRaisesRegex(ValueError, 'Current-state'):
            compare_native(baseline, longer)
        longer = self.output(61)
        longer.objects[0].object_id.uuid[1] ^= 1
        with self.assertRaisesRegex(ValueError, 'Current-state'):
            compare_native(baseline, longer)

    def test_partial_or_truncated_native_output_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '31/61'):
            compare_native(self.output(31), self.output(60))

    def test_class_horizons_report_actual_lengths_not_requested_horizon(self):
        message = self.output(31)
        second = deepcopy(message.objects[0])
        second.classification[0].label = 7
        second.kinematics.predicted_paths = self.output(61).objects[0].kinematics.predicted_paths
        message.objects.append(second)
        stats = class_horizons(message)
        self.assertEqual(stats['CAR']['objects_all_31'], 1)
        self.assertEqual(stats['CAR']['objects_all_61'], 0)
        self.assertEqual(stats['PEDESTRIAN']['paths_61_poses'], 1)
        self.assertEqual(stats['PEDESTRIAN']['objects_all_61'], 1)

    def test_static_wire_omits_only_prediction_and_publisher_sequence(self):
        baseline, longer = self.output(31), self.output(61)
        longer.header.seq += 20
        self.assertEqual(static_wire(baseline), static_wire(longer))
        longer.header.frame_id = 'wrong'
        self.assertNotEqual(static_wire(baseline), static_wire(longer))

    def test_comparison_summary_preserves_zero_and_changed_fractions(self):
        baseline, longer = self.output(31), self.output(61)
        rows = compare_native(baseline, longer)
        longer.objects[0].kinematics.predicted_paths[0].confidence = .3
        rows += compare_native(baseline, longer)
        summary = comparison_summary(rows)['CAR']
        self.assertEqual(summary['object_observations'], 2)
        self.assertEqual(summary['whole_ordered_prefix_exact_fraction'], .5)
        self.assertEqual(summary['ordered_confidence_changed'], 1)

    def test_stopped_time_does_not_interpolate_across_long_missing_gaps(self):
        result = held_time([(0., 0.), (.1, 1.), (10., 0.), (10.1, 0.)], 0., 10.1,
                           lambda speed: speed < .1)
        self.assertAlmostEqual(result['matching_sec'], .2)
        self.assertAlmostEqual(result['covered_sec'], .2)
        self.assertAlmostEqual(result['uncovered_sec'], 9.9)

    def test_gaps_use_actual_message_times_and_expose_backwards_steps(self):
        result = gap_summary([1., 1.1, 1.5, 1.4])
        self.assertEqual(result['messages'], 4)
        self.assertEqual(result['backwards_steps'], 1)
        self.assertEqual(result['gaps_over_250ms'], 1)

    def test_drive_window_requires_explicit_controller_start_and_end(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/'events.jsonl').write_text(
                '{"time":"2026-10-23T16:25:36+00:00","event":"CONTROLLER_START"}\n'
                '{"time":"2026-10-23T16:26:06+00:00","event":"RUN_FINISHED"}\n')
            window = drive_window(run)
            self.assertEqual(window['end_sec']-window['start_sec'], 30.)
            (run/'events.jsonl').write_text('{}\n')
            self.assertFalse(drive_window(run)['available'])

    def test_drive_window_uses_labeled_case_result_fallback_on_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/'events.jsonl').write_text(
                '{"time":"2026-10-23T16:25:36+00:00","event":"CONTROLLER_START"}\n'
                '{"time":"2026-10-23T16:26:06+00:00","event":"CASE_RESULT"}\n')
            window = drive_window(run)
            self.assertTrue(window['available'])
            self.assertEqual(window['end_event'], 'CASE_RESULT')

    def test_actual_native_public_converter_extends_nonvehicles_too_without_mutating_input(self):
        from perception_object_pipeline.map_based_prediction import LaneletNetwork, LaneletRecord
        from perception_object_pipeline.kinematic_prediction import build_prediction_horizons
        spec = importlib.util.spec_from_file_location('native_horizon_test_actual_node', NODE_PATH)
        node = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(node)
        fixture = fixtures.MixedHorizonQualificationTest()
        tracks = [fixture.track(1, 1), fixture.track(2, 0), fixture.track(3, 3)]
        raw = dict(stamp=1., native_index=9, tracks=tracks)
        normalized = dict(objects=[dict(id='1', center=[10., 0., 0.], yaw=0.,
            body_velocity=[5., 0.], yaw_rate=.1)])
        transform = dict(parent='map', child='morai_map', xyz=[0., 0., 0.], rpy=[0., 0., 0.])
        message = build_message('test', 0, raw, normalized, transform)
        before = wire(message)
        network = LaneletNetwork((LaneletRecord(1, ((0., 0., 0.), (100., 0., 0.)), (),
            (('subtype', 'road'), ('speed_limit', '150'))),))
        results = [node.make_map_based_predicted_objects(message,
            build_prediction_horizons(.1, value), network) for value in (3, 6)]
        self.assertEqual(wire(message), before)
        self.assertEqual(results[0][1:3], (1, 2))
        self.assertEqual(results[1][1:3], (1, 2))
        rows = compare_native(results[0][0], results[1][0])
        self.assertEqual({row['class_name'] for row in rows}, {'CAR', 'UNKNOWN', 'PEDESTRIAN'})
        self.assertTrue(all(row['whole_ordered_prefix_exact'] for row in rows))
        self.assertTrue(all(v['objects_all_61'] == 1 for v in class_horizons(results[1][0]).values()))


if __name__ == '__main__':
    unittest.main()
