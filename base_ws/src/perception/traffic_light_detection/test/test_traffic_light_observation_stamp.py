#!/usr/bin/env python3
"""Original-observation timestamps; fake ROS/model only, no live processes.

These are provenance and existing phase-policy tests, not classifier accuracy,
physical yellow remaining-time estimation, or official stop-line qualification.
"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from signal_phase_policy import SignalPhasePolicy
from signal_smoother import SignalSmoother
import test_traffic_light_candidate_node as fixture

R, Y, G, L = (("red", "circle", None), ("yellow", "circle", None),
              ("green", "circle", None), ("green", "arrow", True))


class ObservationMetadataTest(unittest.TestCase):
    def setUp(self):
        self.policy = SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))

    def step(self, raw, clock, source, smoother, group=201326):
        normalized, reason = self.policy.apply(raw, group)
        result = self.policy.process_normalized(normalized, reason, group, clock, .9, smoother,
                                                source_stamp_ns=source)
        return result, self.policy.output_observation_metadata(group)

    def test_existing_six_argument_api_still_returns_three_values(self):
        result = self.policy.process_normalized([R], 'simple_3color:valid', 201351, 10., .9,
                                                SignalSmoother(1))
        self.assertEqual(len(result), 3)
        self.assertEqual(result[0], [R])
        # No source image time was supplied: never synthesize it from clock=10.
        self.assertIsNone(self.policy.output_observation_metadata(201351)[
            'adopted_observation_source_stamp_ns'])

    def test_unguarded_vote_keeps_winning_green_original_exact_nanoseconds(self):
        smoother = SignalSmoother(7, 4)
        base = 1792918944123456789
        for index in range(7):
            self.step([G], 10+index*.1, base+index*100000000, smoother, 201351)
        original_green = base+600000000
        for index in range(3):
            result, metadata = self.step([R], 10.7+index*.1, base+700000000+index*100000000,
                                         smoother, 201351)
            self.assertEqual(result[0], [G])
            self.assertEqual(metadata['adopted_observation_source_stamp_ns'], original_green)
            self.assertTrue(metadata['output_is_held'])
            self.assertEqual(metadata['output_origin'], 'smoothed_observation')
        result, metadata = self.step([R], 11., base+1000000000, smoother, 201351)
        self.assertEqual(result[0], [R])
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], base+1000000000)

    def test_guard_hold_and_synthetic_expiry_do_not_refresh_observed_source(self):
        smoother = SignalSmoother(1)
        self.step([R, L], 10., 9000000000, smoother)
        result, metadata = self.step([], 11., 10000000000, smoother)
        self.assertEqual(set(result[0]), {R, L})
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 9000000000)
        self.assertEqual(metadata['current_image_source_stamp_ns'], 10000000000)
        self.assertEqual(metadata['output_origin'], 'bounded_observation_hold')
        result, metadata = self.step([], 11.51, 10510000000, smoother)
        self.assertEqual(result[0:2], ([R], 0.))
        self.assertFalse(metadata['output_is_observed'])
        self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])
        self.assertEqual(metadata['last_trusted_observation_source_stamp_ns'], 9000000000)

    def test_missing_fallback_red_vote_is_not_observed_red(self):
        smoother = SignalSmoother(7, 4)
        for index in range(7):
            self.step([G], 10+index*.1, 9000000000+index*100000000, smoother, 201351)
        for index in range(4):
            result, metadata = self.step([], 10.7+index*.1, 9700000000+index*100000000,
                                         smoother, 201351)
        self.assertEqual(result[0], [R])
        self.assertFalse(metadata['output_is_observed'])
        self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])
        self.assertEqual(metadata['last_trusted_observation_source_stamp_ns'], 9600000000)

    def test_split_vote_fallback_is_not_latest_raw_observation(self):
        smoother = SignalSmoother(3, 3)
        for index, lamps in enumerate(([R], [G], [Y])):
            result, metadata = self.step(lamps, 10+index*.1, 9000000000+index*100000000,
                                         smoother, 201351)
        self.assertEqual(result[0], [R])
        self.assertEqual(metadata['output_origin'], 'policy_fallback')
        self.assertFalse(metadata['output_is_observed'])
        self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])

    def test_source_clock_and_guard_clock_are_not_interchangeable(self):
        smoother = SignalSmoother(1)
        self.step([R, L], 10., 1000000000, smoother)
        result, metadata = self.step([], 11., 1100000000, smoother)
        self.assertEqual(set(result[0]), {R, L})
        self.assertEqual(metadata['adopted_observation_clock_sec'], 10.)
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 1000000000)
        self.assertEqual(metadata['guard_clock_sec'], 11.)
        self.assertEqual(self.step([], 11.51, 1200000000, smoother)[0][0], [R])

    def test_reset_and_clock_rollback_do_not_carry_other_lamp_source(self):
        smoother = SignalSmoother(1)
        self.step([R, L], 10., 9000000000, smoother)
        result, metadata = self.step([], 1., 100000000, smoother)
        self.assertEqual(result[0], [R])
        self.assertIsNone(metadata['last_trusted_observation_source_stamp_ns'])
        self.policy.reset_transition()
        self.assertEqual(self.policy.output_observation_metadata(201326), {})

    def test_yellow_guard_is_not_a_three_second_physical_remaining_time_estimate(self):
        # Existing guard is a policy elapsed bound, not the sample scenario's
        # 3 s yellow duration or evidence that first observation began a phase.
        smoother = SignalSmoother(1)
        for index in range(8):
            result, metadata = self.step([R, Y], index*.5, 1000000000+index*500000000, smoother)
            self.assertEqual(set(result[0]), {R, Y})
            self.assertTrue(metadata['output_is_observed'])
        result, metadata = self.step([R, Y], 4.01, 5010000000, smoother)
        self.assertEqual(result[0], [R])
        self.assertFalse(metadata['output_is_observed'])

    def test_snapshot_mutation_cannot_rewrite_policy_history(self):
        self.step([R, L], 10., 9000000000, SignalSmoother(1))
        self.policy.output_observation_metadata(201326)['adopted_observation_source_stamp_ns'] = 0
        self.assertEqual(self.policy.output_observation_metadata(201326)[
            'adopted_observation_source_stamp_ns'], 9000000000)

    def test_missing_invalid_source_stamp_is_not_created_from_guard_clock(self):
        for invalid_source in (None, 0, -1, True, 1.2):
            with self.subTest(source=invalid_source):
                self.policy.reset_transition()
                _, metadata = self.step([R, L], 10., invalid_source, SignalSmoother(1))
                self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])
                self.assertIsNone(metadata['last_trusted_observation_source_stamp_ns'])


class PlannerObservationHandoffTest(unittest.TestCase):
    """No new permission filter here; C++ owns distinct-fresh grant confirmation."""
    def setUp(self):
        self.policy = SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))

    def step(self, raw, clock, source, smoother, group=201326, use_smoothing=False,
             head='C1256W000052', eligible=False):
        normalized, reason = self.policy.apply(raw, group, observed_only=not use_smoothing)
        result = self.policy.process_normalized(normalized, reason, group, clock, .9, smoother,
            source_stamp_ns=source, use_smoothing=use_smoothing,
            observation_head_id=head, detection_miss_hold_eligible=eligible)
        return result, self.policy.output_observation_metadata(group)

    def test_opt_in_valid_red_cancels_existing_green_on_first_current_observation(self):
        from unittest.mock import Mock
        for group in (201351, 201365, 201290):
            with self.subTest(group=group):
                self.policy.reset_transition()
                smoother = SignalSmoother(7, 4)
                smoother.update = Mock(side_effect=AssertionError('Opt-in must not add another vote'))
                self.step([G], 10., 9000000000, smoother, group)
                result, metadata = self.step([R], 10.1, 9100000000, smoother, group)
                self.assertEqual(result[0], [R])
                self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 9100000000)
                self.assertFalse(metadata['output_is_held'])

    def test_opt_in_current_yellow_reaches_downstream_without_node_majority_delay(self):
        smoother = SignalSmoother(7, 4)
        self.step([G], 10., 9000000000, smoother, 201351)
        result, metadata = self.step([Y], 10.1, 9100000000, smoother, 201351)
        self.assertEqual(result[0], [Y])
        self.assertEqual(metadata['output_origin'], 'current_observation')
        # D's observed yellow remains yellow; route permission belongs to C++.
        result, metadata = self.step([Y], 10.2, 9200000000, smoother, 201326)
        self.assertEqual(result[0], [Y])
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 9200000000)

    def test_opt_in_fresh_G_is_observation_not_a_self_contained_permission_grant(self):
        smoother = SignalSmoother(7, 4)
        result, metadata = self.step([G], 10., 9000000000, smoother, 201351)
        self.assertEqual(result[0], [G])
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 9000000000)
        # The node does not invent distinct source stamps for repeated frames.
        _, repeated = self.step([G], 10.1, 9000000000, smoother, 201351)
        self.assertEqual(repeated['adopted_observation_source_stamp_ns'], 9000000000)

    def test_opt_in_D_detection_gap_keeps_source_only_for_200ms_then_unknown(self):
        smoother = SignalSmoother(7, 4)
        self.step([R, L], 10., 10000000000, smoother)
        result, metadata = self.step([], 10.1, 10100000000, smoother, eligible=True)
        self.assertEqual(set(result[0]), {R, L})
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 10000000000)
        self.assertTrue(metadata['output_is_held'])
        result, metadata = self.step([], 10.201, 10201000000, smoother, eligible=True)
        self.assertEqual(result[:2], ([], 0.))
        self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])
        self.assertFalse(metadata['output_is_observed'])

    def test_opt_in_D_real_red_revokes_held_left_on_current_frame(self):
        smoother = SignalSmoother(7, 4)
        self.step([R, L], 10., 9000000000, smoother)
        self.step([], 10.2, 9200000000, smoother)
        result, metadata = self.step([R], 10.3, 9300000000, smoother)
        self.assertEqual(result[0], [R])
        self.assertEqual(metadata['adopted_observation_source_stamp_ns'], 9300000000)

    def test_opt_in_simple_unknown_is_unknown_not_synthetic_red_or_old_green(self):
        smoother = SignalSmoother(7, 4)
        self.step([G], 10., 9000000000, smoother, 201351)
        result, metadata = self.step([], 10.1, 9100000000, smoother, 201351)
        self.assertEqual(result[0], [])
        self.assertFalse(metadata['output_is_observed'])
        self.assertIsNone(metadata['adopted_observation_source_stamp_ns'])

    def test_opt_in_fresh_D_yellow_is_not_ended_by_legacy_four_second_cap(self):
        smoother = SignalSmoother(7, 4)
        for index in range(8):
            self.assertEqual(set(self.step([R, Y], 10+index*.5, 10000000000+index*500000000, smoother)[0][0]), {R, Y})
        result, metadata = self.step([R, Y], 14.01, 14010000000, smoother)
        self.assertEqual(set(result[0]), {R, Y})
        self.assertTrue(metadata['output_is_observed'])

    def test_default_smoothing_still_delays_switch_until_four_of_seven(self):
        smoother = SignalSmoother(7, 4)
        for index in range(7):
            self.step([G], 10+index*.1, 9000000000+index*100000000, smoother, 201351, True)
        for index in range(3):
            self.assertEqual(self.step([R], 10.7+index*.1, 9700000000+index*100000000,
                                       smoother, 201351, True)[0][0], [G])
        self.assertEqual(self.step([R], 11., 10000000000, smoother, 201351, True)[0][0], [R])

    def test_smoothing_option_requires_bool_not_truthy_string(self):
        with self.assertRaises(ValueError):
            self.step([G], 10., 9000000000, SignalSmoother(7, 4), 201351, 'false')


class NodeObservationStampTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    def node(self, group=201326, window=1):
        node = fixture.CandidateNodeTest.node(self, mode='legacy_center')
        node.signal_phase_policy = SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))
        node.signal_smoother = SignalSmoother(window)
        node._resolve_group_id = lambda: group
        node._last_policy_group_id = group
        return node

    def image(self):
        return fixture.CandidateNodeTest.image(self)

    def debug(self, node, publisher):
        return fixture.CandidateNodeTest.debug(self, node, publisher)

    def step(self, node, raw, now, source):
        message = self.image()
        message.header.stamp = source
        with patch.object(self.module.rospy.Time, 'now', return_value=fixture.Stamp(now)), \
                patch.object(self.module, 'classify_crop', return_value=(raw, {})):
            node.image_callback(message)
        return node.pub.publish.call_args[0][0], self.debug(node, 'runtime_debug_pub')['frame_decision']

    def test_hold_output_wire_stamp_and_age_refer_to_original_observation(self):
        node = self.node()
        source = fixture.Stamp(9, 123456789)
        output, _ = self.step(node, [R, L], 10., source)
        self.assertIs(output.stamp, source)
        output, decision = self.step(node, [], 11., fixture.Stamp(10, 876543210))
        self.assertEqual(output.stamp.to_nsec(), source.to_nsec())
        timing = decision['observation_timing']
        self.assertEqual(timing['adopted_observation_source_stamp_ns'], 9123456789)
        self.assertEqual(timing['output_stamp_basis'], 'adopted_lamp_observation')
        self.assertAlmostEqual(timing['output_observation_age_sec'], 1.876543211)
        self.assertTrue(timing['output_is_held'])

    def test_expired_synthetic_red_has_new_wire_stamp_but_no_new_color_observation(self):
        node = self.node()
        self.step(node, [R, L], 10., fixture.Stamp(9.))
        current = fixture.Stamp(11.01)
        output, decision = self.step(node, [], 11.51, current)
        self.assertIs(output.stamp, current)
        self.assertEqual(decision['final_tuples'], [['red', 'circle', None]])
        self.assertEqual(decision['final_confidence'], 0.)
        timing = decision['observation_timing']
        self.assertFalse(timing['output_is_observed'])
        self.assertIsNone(timing['adopted_observation_source_stamp_ns'])
        self.assertIsNone(timing['output_observation_age_sec'])
        self.assertEqual(timing['last_trusted_observation_source_stamp_ns'], 9000000000)
        self.assertEqual(timing['output_stamp_basis'], 'current_image_policy_fallback')

    def test_held_epoch_nanoseconds_are_not_rounded_through_float_seconds(self):
        node = self.node()
        original = fixture.Stamp(1792918944, 123456789)
        self.step(node, [R, L], 1792918944.5, original)
        output, decision = self.step(node, [], 1792918945., fixture.Stamp(1792918944, 923456789))
        self.assertEqual(output.stamp.to_nsec(), 1792918944123456789)
        self.assertEqual(decision['observation_timing']['output_source_stamp_ns'], 1792918944123456789)

    def test_unstamped_observed_lamp_does_not_claim_ros_now_as_image_time(self):
        node = self.node()
        output, decision = self.step(node, [R, L], 10., fixture.Stamp(0))
        self.assertEqual(output.stamp.to_nsec(), 0)
        self.assertIsNone(decision['observation_timing']['adopted_observation_source_stamp_ns'])

    def test_node_opt_in_current_red_is_immediate_with_its_original_source_stamp(self):
        node = self.node(group=201351, window=7)
        node.planner_compliance_observations = True
        self.step(node, [G], 10., fixture.Stamp(9.))
        current = fixture.Stamp(9.1)
        output, decision = self.step(node, [R], 10.1, current)
        self.assertEqual(decision['final_tuples'], [['red', 'circle', None]])
        self.assertIs(output.stamp, current)
        self.assertTrue(decision['planner_compliance_observations'])
        self.assertFalse(decision['observation_timing']['output_is_held'])

    def test_node_opt_in_D_unknown_is_not_legacy_unqualified_hold(self):
        node = self.node(window=7)
        node.planner_compliance_observations = True
        self.step(node, [R, L], 10., fixture.Stamp(9.))
        output, decision = self.step(node, [], 10.5, fixture.Stamp(9.5))
        self.assertEqual(output.stamp.to_nsec(), 9500000000)
        self.assertTrue(decision['planner_compliance_observations'])
        self.assertFalse(decision['observation_timing']['output_is_held'])
        self.assertFalse(decision['observation_timing']['output_is_observed'])


if __name__ == '__main__':
    unittest.main()
