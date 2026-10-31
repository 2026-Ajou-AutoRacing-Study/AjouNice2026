"""Pure experimental vote contracts; no ROS, model, GPU, or live process."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from traffic_light_stamped_vote import StampedPhaseVote

R, Y, G, L = (("red", "circle", None), ("yellow", "circle", None),
              ("green", "circle", None), ("green", "arrow", True))


class StampedPhaseVoteTest(unittest.TestCase):
    def step(self, vote, source, lamps=(G,), now=None, **overrides):
        args = dict(normalized_reason='test:observed' if lamps else 'test:observation_missing',
                    group_id=201351, head_id='head', source_stamp_ns=source,
                    now_ns=source if now is None else now, confidence=.9, identity_valid=True)
        args.update(overrides)
        return vote.update(lamps, **args)

    def test_startup_requires_two_original_frames_not_three_frame_passthrough(self):
        vote = StampedPhaseVote()
        self.assertEqual(self.step(vote, 1_000_000_000)['lamps'], [])
        output = self.step(vote, 1_100_000_000, confidence=.7)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['confidence'], .7)
        self.assertEqual(output['debug']['winner_vote_count'], 2)
        self.assertEqual(output['observation_timing']['adopted_observation_source_stamp_ns'], 1_100_000_000)

    def test_single_real_red_or_yellow_is_delayed_by_existing_green_majority(self):
        for changed in (R, Y):
            with self.subTest(changed=changed):
                vote = StampedPhaseVote()
                self.step(vote, 1_000_000_000)
                self.step(vote, 1_100_000_000, confidence=.72)
                output = self.step(vote, 1_200_000_000, lamps=[changed], confidence=.99)
                self.assertEqual(output['lamps'], [G])
                self.assertEqual(output['confidence'], .72)
                self.assertEqual(output['observation_timing']['adopted_observation_source_stamp_ns'], 1_100_000_000)
                self.assertTrue(output['observation_timing']['output_is_held'])
                output = self.step(vote, 1_300_000_000, lamps=[changed])
                self.assertEqual(output['lamps'], [changed])

    def test_unknown_votes_count_without_inventing_red(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000)
        self.step(vote, 1_100_000_000)
        output = self.step(vote, 1_200_000_000, lamps=[])
        self.assertEqual(output['lamps'], [G])
        output = self.step(vote, 1_300_000_000, lamps=[])
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['reason'], 'stamped_vote:unknown_majority')
        self.assertEqual(output['confidence'], 0.)
        self.assertFalse(output['observation_timing']['output_is_observed'])

    def test_duplicate_sources_never_add_vote_or_renew_confidence_clock(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000)
        for now in (1_010_000_000, 1_100_000_000):
            output = self.step(vote, 1_000_000_000, now=now)
            self.assertEqual(output['lamps'], [])
            self.assertEqual(output['debug']['vote_source_stamps_ns'], [1_000_000_000])
            self.assertTrue(output['debug']['duplicate_source'])
        self.step(vote, 1_200_000_000, confidence=.7)
        output = self.step(vote, 1_200_000_000, now=1_250_000_000, confidence=.99)
        self.assertEqual(output['confidence'], .7)
        self.assertEqual(output['observation_timing']['adopted_observation_clock_sec'], 1.2)
        self.assertTrue(output['observation_timing']['output_is_held'])

    def test_sparse_sources_prune_oldest_vote_by_elapsed_original_age(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000)
        self.step(vote, 1_100_000_000)
        output = self.step(vote, 1_600_000_001, lamps=[R])
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['vote_source_stamps_ns'], [1_600_000_001])

    def test_age_boundary_uses_integer_ns_and_old_frames_never_refresh(self):
        source = 1_790_000_000_123_456_789
        vote = StampedPhaseVote()
        self.step(vote, source)
        output = self.step(vote, source + 100_000_000, now=source + 500_000_000)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['observation_timing']['adopted_observation_source_stamp_ns'], source + 100_000_000)
        output = self.step(vote, source + 100_000_000, now=source + 500_000_001)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['vote_source_stamps_ns'], [source + 100_000_000])

    def test_custom_age_window_is_not_a_frame_count_lease(self):
        vote = StampedPhaseVote(max_age_sec=.2)
        self.step(vote, 1_000_000_000)
        self.step(vote, 1_100_000_000)
        self.assertEqual(self.step(vote, 1_300_000_001)['lamps'], [])

    def test_group_change_or_identity_loss_resets_old_permission(self):
        changes = ({'group_id': 201365}, {'identity_valid': False},
                   {'head_id': None}, {'group_id': 0})
        for change in changes:
            with self.subTest(change=change):
                vote = StampedPhaseVote()
                self.step(vote, 1_000_000_000)
                self.step(vote, 1_100_000_000)
                output = self.step(vote, 1_200_000_000, **change)
                self.assertEqual(output['lamps'], [])
                self.assertLessEqual(len(output['debug']['vote_source_stamps_ns']), 1)

    def test_same_group_qualified_heads_share_votes_without_section_specific_exception(self):
        for group in (201351, 201365, 201290, 201326):
            with self.subTest(group=group):
                vote = StampedPhaseVote()
                first = self.step(vote, 1_000_000_000, group_id=group, head_id='head_A')
                self.assertEqual(first['lamps'], [])
                for index, head in enumerate(('head_B', 'head_A', 'head_B'), 1):
                    source = 1_000_000_000 + index * 100_000_000
                    output = self.step(vote, source, group_id=group, head_id=head)
                    self.assertEqual(output['lamps'], [G])
                    self.assertIsNone(output['debug']['reset_reason'])
                    self.assertEqual(output['debug']['winner_source_stamp_ns'], source)
                    self.assertEqual(output['debug']['winner_head_id'], head)
                    self.assertEqual(output['observation_timing']['adopted_observation_head_id'], head)
                self.assertEqual(output['debug']['vote_head_ids'], ['head_B', 'head_A', 'head_B'])
                self.assertEqual(output['debug']['identity_scope'], 'regulatory_group')

    def test_winner_preserves_original_head_confidence_and_stamp_across_new_input_head(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000, head_id='first_green_head')
        self.step(vote, 1_100_000_000, head_id='winning_green_head', confidence=.72)
        output = self.step(vote, 1_200_000_000, head_id='current_red_head', lamps=[R], confidence=.99)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['confidence'], .72)
        self.assertEqual(output['debug']['head_id'], 'current_red_head')
        self.assertEqual(output['debug']['winner_head_id'], 'winning_green_head')
        self.assertEqual(output['debug']['winner_source_stamp_ns'], 1_100_000_000)
        self.assertEqual(output['observation_timing']['adopted_observation_head_id'], 'winning_green_head')

    def test_same_original_frame_on_another_head_cannot_add_a_vote(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000, head_id='head_A')
        output = self.step(vote, 1_000_000_000, now=1_100_000_000, head_id='head_B')
        self.assertEqual(output['lamps'], [])
        self.assertTrue(output['debug']['duplicate_source'])
        self.assertEqual(output['debug']['vote_head_ids'], ['head_A'])

    def test_different_head_still_requires_current_map_qualification(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000, head_id='head_A')
        self.step(vote, 1_100_000_000, head_id='head_B')
        output = self.step(vote, 1_200_000_000, head_id='unqualified_head', identity_valid=False)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['reset_reason'], 'identity_loss')
        self.assertEqual(output['debug']['vote_head_ids'], [])
        self.assertEqual(self.step(vote, 1_300_000_000, head_id='head_A')['lamps'], [])

    def test_clock_reversal_starts_new_epoch_but_does_not_keep_previous_votes(self):
        vote = StampedPhaseVote()
        self.step(vote, 2_000_000_000)
        self.step(vote, 2_100_000_000)
        output = self.step(vote, 1_000_000_000)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['reset_reason'], 'clock_reversal')
        self.assertEqual(self.step(vote, 1_100_000_000)['lamps'], [G])

    def test_out_of_order_source_resets_and_does_not_lower_source_highwater(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000)
        self.step(vote, 1_100_000_000)
        output = self.step(vote, 1_050_000_000, now=1_200_000_000)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['reset_reason'], 'source_reversal')
        self.assertEqual(self.step(vote, 1_100_000_000, now=1_250_000_000)['lamps'], [])
        self.assertEqual(self.step(vote, 1_300_000_000)['lamps'], [])
        self.assertEqual(self.step(vote, 1_400_000_000)['lamps'], [G])

    def test_invalid_future_stale_source_or_clock_invalidates_votes(self):
        for source, now in ((0, 1_200_000_000), (1_200_000_001, 1_200_000_000),
                            (1_000_000_000, 1_500_000_001), (True, 1_200_000_000),
                            (1_200_000_000, True)):
            with self.subTest(source=source, now=now):
                vote = StampedPhaseVote()
                self.step(vote, 1_000_000_000)
                self.step(vote, 1_100_000_000)
                self.assertEqual(self.step(vote, source, now=now)['lamps'], [])

    def test_legacy_corrections_and_invalid_confidence_cannot_supply_lamp_votes(self):
        for kwargs in ({'normalized_reason': 'D:corrected'}, {'normalized_reason': 'D:fail_open'},
                       {'confidence': 0.}, {'confidence': float('nan')}, {'confidence': True}):
            with self.subTest(kwargs=kwargs):
                vote = StampedPhaseVote()
                self.step(vote, 1_000_000_000, lamps=[R], **kwargs)
                output = self.step(vote, 1_100_000_000, lamps=[R], **kwargs)
                self.assertEqual(output['lamps'], [])

    def test_exact_combination_votes_never_add_or_remove_an_observed_lamp(self):
        vote = StampedPhaseVote()
        self.step(vote, 1_000_000_000, lamps=[R, L])
        output = self.step(vote, 1_100_000_000, lamps=[L, R])
        self.assertEqual(output['lamps'], [L, R])
        self.assertNotIn(G, output['lamps'])

    def test_configuration_rejects_unbounded_windows(self):
        for window in (0, 2, 4, True, 3.):
            with self.assertRaises(ValueError):
                StampedPhaseVote(window)
        for age in (0, -.1, .5000001, float('nan'), float('inf'), True, '0.5'):
            with self.assertRaises(ValueError):
                StampedPhaseVote(max_age_sec=age)

    def test_seven_frame_configuration_and_startup_bool_are_explicit_and_bounded(self):
        vote = StampedPhaseVote(7, .9)
        self.assertEqual(vote.window_size, 7)
        self.assertEqual(vote.majority_count, 4)
        self.assertFalse(vote.startup_passthrough)
        for age in (0, -.1, .9000001, float('nan'), float('inf'), True, '0.9', 1e-12):
            with self.subTest(age=age), self.assertRaises(ValueError):
                StampedPhaseVote(7, age)
        for startup in (0, 1, None, 'false', 'true'):
            with self.subTest(startup=startup), self.assertRaises(ValueError):
                StampedPhaseVote(7, .9, startup_passthrough=startup)

    def test_seven_strict_default_requires_four_agreeing_original_frames(self):
        vote = StampedPhaseVote(7, .9)
        for index in range(3):
            self.assertEqual(self.step(vote, 1_000_000_000 + index * 100_000_000)['lamps'], [])
        output = self.step(vote, 1_300_000_000)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['debug']['majority_count'], 4)
        self.assertEqual(output['debug']['candidate_version'], 4)
        self.assertEqual(output['debug']['startup_policy'], 'strict_majority')

    def test_seven_stable_green_bridges_three_unknown_frames_but_not_four(self):
        for startup in (False, True):
            with self.subTest(startup=startup):
                vote = StampedPhaseVote(7, .9, startup_passthrough=startup)
                for index in range(7):
                    self.step(vote, 1_000_000_000 + index * 100_000_000, head_id='original_green_head', confidence=.72)
                for index in range(7, 10):
                    output = self.step(vote, 1_000_000_000 + index * 100_000_000,
                                       lamps=[], head_id='current_unknown_head')
                    self.assertEqual(output['lamps'], [G])
                    self.assertEqual(output['confidence'], .72)
                    self.assertEqual(output['debug']['winner_source_stamp_ns'], 1_600_000_000)
                    self.assertEqual(output['debug']['winner_head_id'], 'original_green_head')
                    self.assertFalse(output['debug']['startup_passthrough_active'])
                output = self.step(vote, 2_000_000_000, lamps=[])
                self.assertEqual(output['lamps'], [])
                self.assertEqual(output['reason'], 'stamped_vote:unknown_majority')

    def test_seven_true_red_and_yellow_wait_three_processed_intervals_after_first_change(self):
        for changed in (R, Y):
            with self.subTest(changed=changed):
                vote = StampedPhaseVote(7, .9, startup_passthrough=True)
                for index in range(7):
                    self.step(vote, 1_000_000_000 + index * 100_000_000)
                for index in range(7, 10):
                    output = self.step(vote, 1_000_000_000 + index * 100_000_000, lamps=[changed])
                    self.assertEqual(output['lamps'], [G])
                output = self.step(vote, 2_000_000_000, lamps=[changed])
                self.assertEqual(output['lamps'], [changed])
                self.assertEqual(output['debug']['winner_vote_count'], 4)
                self.assertEqual(output['debug']['winner_source_stamp_ns'], 2_000_000_000)

    def test_seven_legacy_style_first_six_frames_passthrough_current_original_input(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        for index, lamp in enumerate((G, G, G, G, R, Y)):
            source = 1_000_000_000 + index * 100_000_000
            output = self.step(vote, source, lamps=[lamp], head_id=f'head_{index}', confidence=.7 + index * .01)
            self.assertEqual(output['lamps'], [lamp])
            self.assertEqual(output['reason'], 'stamped_vote:startup_passthrough')
            self.assertTrue(output['debug']['startup_passthrough_active'])
            self.assertFalse(output['debug']['window_full'])
            self.assertEqual(output['debug']['available_vote_count'], index + 1)
            self.assertEqual(output['debug']['accepted_count'], index + 1)
            self.assertFalse(output['debug']['startup_complete'])
            self.assertEqual(output['debug']['winner_source_stamp_ns'], source)
            self.assertEqual(output['debug']['winner_head_id'], f'head_{index}')
            self.assertEqual(output['confidence'], .7 + index * .01)
            self.assertEqual(output['observation_timing']['adopted_observation_clock_sec'], source / 1e9)
            self.assertEqual(output['observation_timing']['output_origin'], 'stamped_vote_startup_passthrough')
        output = self.step(vote, 1_600_000_000, lamps=[G])
        self.assertEqual(output['lamps'], [G])
        self.assertFalse(output['debug']['startup_passthrough_active'])
        self.assertTrue(output['debug']['window_full'])
        self.assertTrue(output['debug']['startup_complete'])
        self.assertEqual(output['reason'], 'stamped_vote:winner')

    def test_seven_warmup_unknown_passes_unknown_instead_of_reusing_single_green(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        self.assertEqual(self.step(vote, 1_000_000_000)['lamps'], [G])
        output = self.step(vote, 1_100_000_000, lamps=[])
        self.assertEqual(output['lamps'], [])
        self.assertTrue(output['debug']['startup_passthrough_active'])
        self.assertFalse(output['observation_timing']['output_is_observed'])

    def test_warmup_duplicate_does_not_refresh_original_phase_head_confidence_or_clock(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        self.step(vote, 1_000_000_000, head_id='original', confidence=.72)
        output = self.step(vote, 1_000_000_000, now=1_100_000_000, lamps=[R], head_id='duplicate', confidence=.99)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['confidence'], .72)
        self.assertEqual(output['debug']['vote_head_ids'], ['original'])
        self.assertEqual(output['debug']['winner_head_id'], 'original')
        self.assertEqual(output['observation_timing']['adopted_observation_clock_sec'], 1.)
        self.assertEqual(output['debug']['available_vote_count'], 1)
        self.assertEqual(output['debug']['accepted_count'], 1)
        self.assertTrue(output['debug']['duplicate_source'])
        self.assertTrue(output['observation_timing']['output_is_held'])
        output = self.step(vote, 1_000_000_000, now=1_900_000_001)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['reset_reason'], 'invalid_future_or_stale_source')
        self.assertFalse(output['debug']['startup_passthrough_active'])

    def test_seven_qualified_c_head_swaps_keep_full_window_and_winner_provenance(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        for index in range(10):
            head = 'C1256W000016' if index % 2 == 0 else 'C1256W000018'
            output = self.step(vote, 1_000_000_000 + index * 100_000_000,
                               lamps=[G, L], group_id=201290, head_id=head)
            self.assertEqual(output['lamps'], [G, L])
            self.assertEqual(output['debug']['winner_head_id'], head)
            if index:
                self.assertIsNone(output['debug']['reset_reason'])
        self.assertEqual(output['debug']['available_vote_count'], 7)

    def test_seven_reset_clears_majority_before_passthrough_of_current_input(self):
        for reset in ({'group_id': 201365}, {'identity_valid': False}, {'head_id': None}):
            with self.subTest(reset=reset):
                vote = StampedPhaseVote(7, .9, startup_passthrough=True)
                for index in range(7):
                    self.step(vote, 1_000_000_000 + index * 100_000_000)
                output = self.step(vote, 1_700_000_000, lamps=[R], **reset)
                if 'group_id' in reset:
                    self.assertEqual(output['lamps'], [R])
                    self.assertEqual(output['debug']['available_vote_count'], 1)
                    self.assertEqual(output['debug']['reset_reason'], 'group_change')
                else:
                    self.assertEqual(output['lamps'], [])
                    self.assertEqual(output['debug']['available_vote_count'], 0)
                    self.assertEqual(output['debug']['reset_reason'], 'identity_loss')

    def test_seven_sparse_age_pruning_does_not_reopen_cold_startup(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        for index in range(7):
            self.step(vote, 1_000_000_000 + index * 100_000_000)
        output = self.step(vote, 2_500_000_001, lamps=[R])
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['vote_source_stamps_ns'], [2_500_000_001])
        self.assertFalse(output['debug']['startup_passthrough_active'])
        self.assertTrue(output['debug']['startup_complete'])
        self.assertEqual(output['debug']['accepted_count'], 7)

    def test_seven_source_and_clock_reversals_cannot_reuse_old_majority(self):
        for reversal in ('source', 'clock'):
            with self.subTest(reversal=reversal):
                vote = StampedPhaseVote(7, .9, startup_passthrough=True)
                for index in range(7):
                    self.step(vote, 1_000_000_000 + index * 100_000_000)
                if reversal == 'source':
                    output = self.step(vote, 1_500_000_000, now=1_700_000_000, lamps=[R])
                    self.assertEqual(output['lamps'], [])
                    self.assertEqual(output['debug']['reset_reason'], 'source_reversal')
                    self.assertTrue(output['debug']['startup_complete'])
                    self.assertEqual(output['debug']['accepted_count'], 7)
                    output = self.step(vote, 1_800_000_000, lamps=[R])
                    self.assertEqual(output['lamps'], [])
                    self.assertFalse(output['debug']['startup_passthrough_active'])
                    self.assertTrue(output['debug']['startup_complete'])
                else:
                    output = self.step(vote, 500_000_000, lamps=[R])
                    self.assertEqual(output['debug']['reset_reason'], 'clock_reversal')
                    self.assertEqual(output['lamps'], [R])
                    self.assertTrue(output['debug']['startup_passthrough_active'])
                    self.assertFalse(output['debug']['startup_complete'])
                self.assertEqual(output['debug']['available_vote_count'], 1)

    def test_full_seven_then_400ms_gap_keeps_four_surviving_green_votes(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        for index in range(7):
            source = 1_000_000_000 + index * 100_000_000
            self.step(vote, source, now=source + 200_000_000, confidence=.72)
        output = self.step(vote, 2_000_000_000, now=2_200_000_000, lamps=[])
        self.assertEqual(output['debug']['available_vote_count'], 5)
        self.assertFalse(output['debug']['window_full'])
        self.assertTrue(output['debug']['startup_complete'])
        self.assertFalse(output['debug']['startup_passthrough_active'])
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['debug']['winner_vote_count'], 4)
        self.assertEqual(output['debug']['winner_source_stamp_ns'], 1_600_000_000)
        self.assertEqual(output['confidence'], .72)
        output = self.step(vote, 2_100_000_000, now=2_300_000_000)
        self.assertEqual(output['lamps'], [G])
        self.assertEqual(output['debug']['winner_vote_count'], 4)
        self.assertFalse(output['debug']['startup_passthrough_active'])

    def test_six_hz_ends_cold_startup_after_seventh_accepted_input_not_queue_fill(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        for index in range(20):
            source = 1_000_000_000 + index * 166_666_667
            output = self.step(vote, source)
            self.assertEqual(output['lamps'], [G])
            self.assertEqual(output['debug']['accepted_count'], min(index + 1, 7))
            self.assertEqual(output['debug']['startup_passthrough_active'], index < 6)
            self.assertEqual(output['debug']['startup_complete'], index >= 6)
            if index >= 6:
                self.assertEqual(output['debug']['available_vote_count'], 6)
                self.assertFalse(output['debug']['window_full'])
        output = self.step(vote, source + 1_000_000_000)
        self.assertEqual(output['lamps'], [])
        self.assertEqual(output['debug']['available_vote_count'], 1)
        self.assertTrue(output['debug']['startup_complete'])
        self.assertFalse(output['debug']['startup_passthrough_active'])

    def test_invalid_sources_do_not_advance_or_reopen_cold_startup_ledger(self):
        vote = StampedPhaseVote(7, .9, startup_passthrough=True)
        self.step(vote, 1_000_000_000)
        duplicate = self.step(vote, 1_000_000_000, now=1_100_000_000)
        self.assertEqual(duplicate['debug']['accepted_count'], 1)
        invalid = self.step(vote, 1_200_000_001, now=1_200_000_000)
        self.assertEqual(invalid['lamps'], [])
        self.assertEqual(invalid['debug']['accepted_count'], 1)
        for index in range(1, 7):
            source = 1_200_000_000 + index * 100_000_000
            output = self.step(vote, source)
        self.assertTrue(output['debug']['startup_complete'])
        self.assertEqual(output['debug']['accepted_count'], 7)
        invalid = self.step(vote, 1_900_000_001, now=1_900_000_000)
        self.assertEqual(invalid['debug']['accepted_count'], 7)
        self.assertTrue(invalid['debug']['startup_complete'])
        output = self.step(vote, 2_000_000_000)
        self.assertEqual(output['lamps'], [])
        self.assertFalse(output['debug']['startup_passthrough_active'])


if __name__ == '__main__':
    unittest.main()
