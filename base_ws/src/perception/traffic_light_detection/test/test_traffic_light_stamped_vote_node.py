"""Actual callback + real observed-only policy, with fake ROS/models only."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import test_traffic_light_candidate_node as fixture
import test_traffic_light_direct_node_handoff as direct
from traffic_light_stamped_vote import StampedPhaseVote
from traffic_light_cnn_classifier import observation_from_probabilities

R, Y, G, L = direct.R, direct.Y, direct.G, direct.L
PACKAGE = Path(__file__).resolve().parents[1]


class StampedVoteNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    image = direct.RealPolicyNodeHandoffTest.image
    debug = direct.RealPolicyNodeHandoffTest.debug
    detector_results = direct.RealPolicyNodeHandoffTest.detector_results
    step = direct.RealPolicyNodeHandoffTest.step

    def node(self, group=201351, enabled=True, window_size=3, max_age_sec=.5,
             startup_passthrough=False):
        node = direct.RealPolicyNodeHandoffTest.node(self, group)
        node._stamped_vote = (StampedPhaseVote(window_size, max_age_sec, startup_passthrough)
                              if enabled else None)
        return node

    def test_candidate_votes_raw_current_frames_and_preserves_winner_source_confidence(self):
        node = self.node()
        _, first = self.step(node, 10.01, 10.)
        self.assertEqual(first['final_tuples'], [])
        self.step(node, 10.11, 10.1)
        output, decision = self.step(node, 10.21, 10.2, raw=[R])
        self.assertEqual(decision['observed_tuples'], [list(R)])
        self.assertEqual(decision['normalized_tuples'], [list(R)])
        self.assertEqual(decision['final_tuples'], [list(G)])
        self.assertEqual(output.stamp.to_nsec(), 10_100_000_000)
        self.assertEqual(decision['raw_observation_source_stamp_ns'], 10_200_000_000)
        self.assertEqual(decision['stamped_vote']['vote_source_stamps_ns'],
                         [10_000_000_000, 10_100_000_000, 10_200_000_000])
        self.assertEqual(decision['observation_timing']['output_origin'], 'stamped_vote_winner')
        self.assertAlmostEqual(decision['final_confidence'], .9)

    def test_cnn_gap_is_unknown_vote_not_repeated_production_held_green(self):
        node = self.node()
        self.step(node, 10.01, 10.)
        self.step(node, 10.11, 10.1)
        dark = observation_from_probabilities((.01, .01, .01, .01))
        _, decision = self.step(node, 10.21, 10.2, observation=dark)
        self.assertTrue(decision['no_lamps_hold']['eligible'])
        self.assertEqual(decision['stamped_vote']['input_normalized_tuples'], [])
        self.assertEqual(decision['final_tuples'], [list(G)])
        _, decision = self.step(node, 10.31, 10.3, observation=dark)
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(decision['stamped_vote']['vote_tuples'], [[list(G)], [], []])
        self.assertEqual(decision['observation_timing']['output_stamp_basis'],
                         'current_image_stamped_vote_unknown')

    def test_detector_miss_resets_even_when_production_dropout_hold_is_eligible(self):
        node = self.node()
        self.step(node, 10.01, 10.)
        self.step(node, 10.11, 10.1)
        _, decision = self.step(node, 10.16, 10.15, detected=False)
        self.assertTrue(decision['detection_miss_hold']['eligible'])
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(decision['stamped_vote']['reset_reason'], 'identity_loss')
        _, decision = self.step(node, 10.21, 10.2)
        self.assertEqual(decision['final_tuples'], [])

    def test_duplicate_original_jpegs_cannot_complete_startup_or_refresh_winner(self):
        node = self.node()
        self.step(node, 10.01, 10.)
        _, decision = self.step(node, 10.08, 10.)
        self.assertEqual(decision['final_tuples'], [])
        self.assertTrue(decision['stamped_vote']['duplicate_source'])
        self.step(node, 10.11, 10.1)
        output, decision = self.step(node, 10.16, 10.1)
        self.assertEqual(output.stamp.to_nsec(), 10_100_000_000)
        self.assertTrue(decision['observation_timing']['output_is_held'])

    def test_group_change_and_missing_map_cannot_retain_other_identity_votes(self):
        for changes in ({'group': 201365}, {'ready': False}):
            with self.subTest(changes=changes):
                node = self.node()
                self.step(node, 10.01, 10.)
                self.step(node, 10.11, 10.1)
                if 'group' in changes:
                    node._resolve_group_id = lambda: changes['group']
                _, decision = self.step(node, 10.21, 10.2, **changes)
                self.assertEqual(decision['final_tuples'], [])

    def test_current_same_group_head_switches_do_not_create_artificial_unknown_gap(self):
        node = self.node(201290)
        raw = [G, L]
        self.step(node, 10.01, 10., raw=raw, head='C1256W000016')
        self.step(node, 10.11, 10.1, raw=raw, head='C1256W000016')
        for index, head in enumerate(('C1256W000018', 'C1256W000016', 'C1256W000018'), 2):
            source = 10. + index * .1
            output, decision = self.step(node, source + .01, source, raw=raw, head=head)
            self.assertEqual(decision['final_tuples'], [list(G), list(L)])
            self.assertIsNone(decision['stamped_vote']['reset_reason'])
            self.assertEqual(decision['stamped_vote']['winner_head_id'], head)
            self.assertEqual(decision['observation_timing']['adopted_observation_head_id'], head)
            self.assertEqual(output.stamp.to_nsec(), fixture.Stamp(source).to_nsec())

    def test_group_owned_vote_preserves_other_heads_original_winning_source(self):
        node = self.node()
        self.step(node, 10.01, 10., head='green_head_A')
        self.step(node, 10.11, 10.1, head='green_head_B')
        output, decision = self.step(node, 10.21, 10.2, raw=[R], head='current_red_head')
        self.assertEqual(decision['final_tuples'], [list(G)])
        self.assertEqual(output.stamp.to_nsec(), 10_100_000_000)
        self.assertEqual(decision['stamped_vote']['head_id'], 'current_red_head')
        self.assertEqual(decision['stamped_vote']['winner_head_id'], 'green_head_B')
        self.assertEqual(decision['stamped_vote']['vote_head_ids'],
                         ['green_head_A', 'green_head_B', 'current_red_head'])

    def test_current_observed_only_normalization_never_recreates_legacy_correction(self):
        for group, lamps in ((201290, [L]), (201326, [Y])):
            with self.subTest(group=group, lamps=lamps):
                node = self.node(group)
                self.step(node, 10.01, 10., raw=lamps)
                _, decision = self.step(node, 10.11, 10.1, raw=lamps)
                self.assertEqual(decision['final_tuples'], [list(lamp) for lamp in lamps])
                self.assertTrue(decision['policy_reason'].endswith(':observed'))

    def test_candidate_disabled_path_keeps_immediate_red_and_production_gap_hold(self):
        node = self.node(enabled=False)
        self.step(node, 10.01, 10.)
        output, decision = self.step(node, 10.08, 10.07, detected=False)
        self.assertEqual(decision['final_tuples'], [list(G)])
        self.assertEqual(output.stamp.to_nsec(), 10_000_000_000)
        self.assertFalse(decision['stamped_vote']['enabled'])
        _, decision = self.step(node, 10.11, 10.1, raw=[R])
        self.assertEqual(decision['final_tuples'], [list(R)])

    def test_seven_warmup_and_four_unknown_cutoff_use_current_raw_inputs(self):
        node = self.node(window_size=7, max_age_sec=.9, startup_passthrough=True)
        for index in range(7):
            source = 10. + index * .1
            _, decision = self.step(node, source + .01, source)
            self.assertEqual(decision['final_tuples'], [list(G)])
            self.assertEqual(decision['stamped_vote']['startup_passthrough_active'], index < 6)
        dark = observation_from_probabilities((.01, .01, .01, .01))
        for index in range(7, 11):
            source = 10. + index * .1
            output, decision = self.step(node, source + .01, source, observation=dark)
            self.assertEqual(decision['normalized_tuples'], [])
            self.assertEqual(decision['stamped_vote']['input_normalized_tuples'], [])
            self.assertEqual(decision['stamped_vote']['window_size'], 7)
            self.assertEqual(decision['stamped_vote']['majority_count'], 4)
            if index < 10:
                self.assertEqual(decision['final_tuples'], [list(G)])
                self.assertEqual(output.stamp.to_nsec(), fixture.Stamp(10.6).to_nsec())
            else:
                self.assertEqual(decision['final_tuples'], [])

    def test_seven_does_not_hold_green_across_detector_identity_loss(self):
        node = self.node(window_size=7, max_age_sec=.9, startup_passthrough=True)
        for index in range(7):
            source = 10. + index * .1
            self.step(node, source + .01, source)
        _, decision = self.step(node, 10.71, 10.7, detected=False)
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(decision['stamped_vote']['reset_reason'], 'identity_loss')
        _, decision = self.step(node, 10.81, 10.8, raw=[R])
        self.assertEqual(decision['final_tuples'], [list(R)])
        self.assertTrue(decision['stamped_vote']['startup_passthrough_active'])

    def test_launches_keep_candidate_disabled_and_forward_explicit_experiment(self):
        for filename in ('traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
            root = ET.parse(PACKAGE / 'launch' / filename).getroot()
            args = {row.attrib['name']: row.attrib.get('default') for row in root.findall('arg')}
            self.assertEqual(args['stamped_vote_window_size'], '0')
            self.assertEqual(args['stamped_vote_max_age_sec'], '0.5')
            self.assertEqual(args['stamped_vote_startup_passthrough'], 'false')
            self.assertEqual(args['planner_compliance_observations'], 'true')
            tag = 'param' if filename == 'traffic_light_node.launch' else 'arg'
            wired = {row.attrib['name']: row.attrib.get('value') for row in root.iter(tag)}
            for key in ('stamped_vote_window_size', 'stamped_vote_max_age_sec',
                        'stamped_vote_startup_passthrough'):
                self.assertEqual(wired[key], '$(arg ' + key + ')')


if __name__ == '__main__':
    unittest.main()
