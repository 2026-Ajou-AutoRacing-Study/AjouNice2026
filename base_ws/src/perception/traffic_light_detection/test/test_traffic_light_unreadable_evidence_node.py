"""Real callback/policy, fake model/ROS: opt-in unreadable wire provenance."""
from pathlib import Path
from unittest.mock import patch
import unittest
import xml.etree.ElementTree as ET

import test_traffic_light_candidate_node as fixture
import test_traffic_light_direct_node_handoff as direct
from traffic_light_cnn_classifier import observation_from_probabilities
from traffic_light_stamped_vote import StampedPhaseVote

PACKAGE = Path(__file__).resolve().parents[1]
R, Y, G = direct.R, direct.Y, direct.G


class UnreadableEvidenceNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    image = direct.RealPolicyNodeHandoffTest.image
    debug = direct.RealPolicyNodeHandoffTest.debug
    detector_results = direct.RealPolicyNodeHandoffTest.detector_results
    step = direct.RealPolicyNodeHandoffTest.step

    def node(self, enabled=True):
        node = direct.RealPolicyNodeHandoffTest.node(self)
        node.preserve_unreadable_color_evidence = enabled
        node.no_lamps_hold_sec = .2
        return node

    def elements(self, output):
        return [(e.color, e.shape, e.status, e.confidence)
                for e in output.traffic_light_groups[0].elements]

    def test_known_red_green_contradiction_is_unknown_shape_status_not_solid_lamps(self):
        node = self.node()
        output, decision = self.step(node, 10.01, 10., raw=[R, G])
        self.assertEqual(decision['normalized_tuples'], [])
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(output.stamp.to_nsec(), 10_000_000_000)
        unknown = self.module.TrafficLightElement.UNKNOWN
        self.assertEqual(self.elements(output), [(self.module.COLOR_MAP['red'], unknown, unknown, .9),
                                                (self.module.COLOR_MAP['green'], unknown, unknown, .9)])
        self.assertEqual([row['color'] for row in decision['wire_unreadable_color_evidence']], ['red', 'green'])
        self.assertTrue(decision['preserve_unreadable_color_evidence'])

    def test_partial_confident_red_or_yellow_cannot_become_empty_grace_input(self):
        for probabilities, color in (((.99, .01, .01, .5), 'red'),
                                     ((.01, .99, .01, .5), 'yellow')):
            with self.subTest(color=color):
                node = self.node()
                self.step(node, 10.01, 10.)
                soft = observation_from_probabilities(probabilities)
                output, decision = self.step(node, 10.11, 10.1, observation=soft)
                self.assertEqual(decision['observed_tuples'], [])
                self.assertEqual(decision['normalized_tuples'], [])
                self.assertEqual(decision['final_tuples'], [])
                self.assertEqual(output.stamp.to_nsec(), 10_100_000_000)
                self.assertEqual(self.elements(output), [(self.module.COLOR_MAP[color],
                    self.module.TrafficLightElement.UNKNOWN, self.module.TrafficLightElement.UNKNOWN, 0.)])
                self.assertEqual(decision['raw_observation_confidence'], 0.)

    def test_partial_green_remains_unknown_shape_not_canonical_green_permission(self):
        node = self.node()
        soft = observation_from_probabilities((.01, .01, .99, .5))
        output, decision = self.step(node, 10.01, 10., observation=soft)
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(self.elements(output), [(self.module.COLOR_MAP['green'],
            self.module.TrafficLightElement.UNKNOWN, self.module.TrafficLightElement.UNKNOWN, 0.)])

    def test_no_lamps_and_soft_without_positive_colors_stay_empty(self):
        for probabilities in ((.01, .01, .01, .01), (.01, .01, .5, .01)):
            node = self.node()
            output, decision = self.step(node, 10.01, 10., observation=observation_from_probabilities(probabilities))
            self.assertEqual(self.elements(output), [])
            self.assertEqual(decision['wire_unreadable_color_evidence'], [])

    def test_map_or_detector_missing_does_not_reuse_old_classifier_evidence(self):
        for changes in ({'detected': False}, {'ready': False}):
            node = self.node()
            self.step(node, 10.01, 10., raw=[R, G])
            output, decision = self.step(node, 10.11, 10.1, raw=[R, G], **changes)
            self.assertEqual(self.elements(output), [])
            self.assertEqual(decision['wire_unreadable_color_evidence'], [])

    def test_valid_solid_output_and_existing_held_output_are_not_modified(self):
        node = self.node()
        self.step(node, 10.01, 10.)
        soft = observation_from_probabilities((.01, .01, .99, .5))
        output, decision = self.step(node, 10.11, 10.1, observation=soft)
        self.assertEqual(decision['final_tuples'], [list(G)])
        self.assertEqual(decision['wire_unreadable_color_evidence'], [])
        self.assertEqual(self.elements(output)[0][1:3],
                         (self.module.TrafficLightElement.CIRCLE, self.module.TrafficLightElement.SOLID_ON))

    def test_vote_unconfirmed_normalized_known_input_does_not_acquire_extra_wire_evidence(self):
        node = self.node()
        node._stamped_vote = StampedPhaseVote()
        output, decision = self.step(node, 10.01, 10.)
        self.assertEqual(decision['normalized_tuples'], [list(G)])
        self.assertEqual(decision['final_tuples'], [])
        self.assertEqual(self.elements(output), [])
        self.assertEqual(decision['wire_unreadable_color_evidence'], [])

    def test_default_false_matches_existing_canonical_wire_fields_for_known_and_unknown(self):
        for kwargs in ({'raw': [R]}, {'raw': [R, G]}, {'raw': []},
                       {'observation': observation_from_probabilities((.01, .99, .01, .5))}):
            with self.subTest(kwargs=kwargs):
                old = self.node(False)
                del old.preserve_unreadable_color_evidence
                explicit = self.node(False)
                old_output, _ = self.step(old, 10.01, 10., **kwargs)
                new_output, decision = self.step(explicit, 10.01, 10., **kwargs)
                self.assertEqual(old_output.stamp.to_nsec(), new_output.stamp.to_nsec())
                self.assertEqual(self.elements(old_output), self.elements(new_output))
                self.assertEqual(decision['wire_unreadable_color_evidence'], [])

    def test_legacy_or_non_boolean_flag_is_rejected_before_model_loading(self):
        for flag, observed in ((True, False), ('true', True), (1, True)):
            params = {'~weights_path': '/unused/model.pt', '~classifier_mode': 'hsv',
                      '~preserve_unreadable_color_evidence': flag,
                      '~planner_compliance_observations': observed}
            with patch.object(self.module.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default), create=True):
                with self.assertRaisesRegex(ValueError, 'unreadable'):
                    self.module.TrafficLightNode()

    def test_production_launches_pair_evidence_with_direct_observations(self):
        key = 'preserve_unreadable_color_evidence'
        for name in ('traffic_light_node.launch', 'traffic_light_v5_candidate.launch'):
            root = ET.parse(PACKAGE / 'launch' / name).getroot()
            args = {row.attrib['name']: row.attrib.get('default') for row in root.findall('arg')}
            self.assertEqual(args[key], '$(arg planner_compliance_observations)')
            tag = 'param' if name == 'traffic_light_node.launch' else 'arg'
            values = {row.attrib['name']: row.attrib.get('value') for row in root.iter(tag)}
            self.assertEqual(values[key], '$(arg ' + key + ')')


if __name__ == '__main__':
    unittest.main()
