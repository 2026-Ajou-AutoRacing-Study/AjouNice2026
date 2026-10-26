"""Actual callback + real policy + fake model/ROS: identity-bound dropout."""
import types
from unittest.mock import Mock, patch
import unittest
from pathlib import Path
import sys

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
import test_traffic_light_candidate_node as fixture
from signal_phase_policy import SignalPhasePolicy
from signal_smoother import SignalSmoother

R, Y, G, L = (("red", "circle", None), ("yellow", "circle", None),
              ("green", "circle", None), ("green", "arrow", True))


class RealPolicyNodeHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    def node(self, group=201351):
        node = fixture.CandidateNodeTest.node(self, classifier='cnn')
        node.planner_compliance_observations = True
        node.signal_phase_policy = SignalPhasePolicy.from_yaml(str(PACKAGE/'config/traffic_signal_profiles.yaml'))
        node.signal_smoother = SignalSmoother(7, 4)
        node._resolve_group_id = lambda: group
        node._cnn_startup = types.SimpleNamespace(ready=True, classifier=types.SimpleNamespace())
        return node

    def image(self):
        return fixture.CandidateNodeTest.image(self)

    def debug(self, node, publisher):
        return fixture.CandidateNodeTest.debug(self, node, publisher)

    def detector_results(self, *bboxes):
        return fixture.CandidateNodeTest.detector_results(self, *bboxes)

    def step(self, node, now, source, raw=(G,), detected=True, ready=True,
             head='test_head', group=None, projection_head=None, unknown=False):
        group = node._resolve_group_id() if group is None else group
        message = self.image();message.header.stamp = fixture.Stamp(source)
        packet = fixture.CandidateNodeTest.packet(self, group=group)
        packet.update(group_id=group, source_stamp_ns=message.header.stamp.to_nsec(),
                      generated_stamp_ns=message.header.stamp.to_nsec(),
                      status='ready' if ready else 'missing', reason='projected' if ready else 'localization_missing')
        packet['rois'][0].update(group_id=group, eligible=True,
                                  head_id=head if projection_head is None else projection_head)
        node._map_packet_for_image.return_value = packet
        node.model.predict.return_value = self.detector_results([480,180,530,194]) if detected else self.detector_results()
        obs = types.SimpleNamespace(known=not unknown, tuples=list(raw) if not unknown else [], confidence=.95)
        obs.to_dict = lambda: dict(known=obs.known, reason='unknown' if unknown else 'observed_lamps',
                                  tuples=obs.tuples, confidence=obs.confidence)
        node._cnn_startup.classifier.classify_bgr = Mock(return_value=obs)
        with patch.object(self.module.rospy.Time, 'now', return_value=fixture.Stamp(now)):
            node.image_callback(message)
        output = node.pub.publish.call_args[0][0]
        return output, self.debug(node,'runtime_debug_pub')['frame_decision']

    def test_all_missions_hold_exact_stamp_then_expire_without_renewal(self):
        for group in (201351,201365,201290,201326):
            with self.subTest(group=group):
                node=self.node(group)
                first,_=self.step(node,10.01,10.)
                self.assertEqual(first.stamp.to_nsec(),10_000_000_000)
                for now,source in ((10.08,10.07),(10.15,10.14)):
                    output,decision=self.step(node,now,source,detected=False)
                    self.assertTrue(decision['detection_miss_hold']['eligible'])
                    self.assertEqual(decision['final_tuples'],[list(G)])
                    self.assertTrue(decision['observation_timing']['output_is_held'])
                    self.assertEqual(output.stamp.to_nsec(),10_000_000_000)
                    self.assertEqual(node._selected_head_id,'test_head')
                output,decision=self.step(node,10.201,10.20,detected=False)
                self.assertEqual(decision['final_tuples'],[])
                self.assertEqual(decision['final_confidence'],0.)
                self.assertFalse(decision['observation_timing']['output_is_observed'])
                self.assertEqual(output.stamp.to_nsec(),10_200_000_000)

    def test_explicit_red_or_yellow_replaces_green_without_majority_delay(self):
        for raw in ([R],[Y]):
            with self.subTest(raw=raw):
                node=self.node(201326)
                self.step(node,10.01,10.,raw=[L])
                self.step(node,10.08,10.07,detected=False)
                output,decision=self.step(node,10.11,10.10,raw=raw)
                self.assertEqual(decision['final_tuples'],[list(x) for x in raw])
                self.assertEqual(output.stamp.to_nsec(),10_100_000_000)
                self.assertFalse(decision['observation_timing']['output_is_held'])

    def test_cnn_unknown_or_all_off_never_uses_dropout_hold_and_invalidates_cache(self):
        for unknown,raw in ((True,[G]),(False,[])):
            with self.subTest(unknown=unknown,raw=raw):
                node=self.node()
                self.step(node,10.01,10.)
                _,decision=self.step(node,10.06,10.05,raw=raw,unknown=unknown)
                self.assertFalse(decision['detection_miss_hold']['eligible'])
                self.assertEqual(decision['final_tuples'],[])
                _,later=self.step(node,10.11,10.10,detected=False)
                self.assertEqual(later['final_tuples'],[])

    def test_map_missing_cannot_hold_old_signal_or_resurrect_on_next_miss(self):
        node=self.node()
        self.step(node,10.01,10.)
        _,decision=self.step(node,10.06,10.05,detected=False,ready=False)
        self.assertFalse(decision['detection_miss_hold']['eligible'])
        self.assertEqual(decision['final_tuples'],[])
        _,later=self.step(node,10.11,10.10,detected=False)
        self.assertEqual(later['final_tuples'],[])

    def test_different_projected_head_never_carries_another_heads_green(self):
        node=self.node()
        self.step(node,10.01,10.)
        _,decision=self.step(node,10.06,10.05,detected=False,projection_head='different_head')
        self.assertFalse(decision['detection_miss_hold']['eligible'])
        self.assertEqual(decision['final_tuples'],[])

    def test_contradiction_never_holds_or_fabricates_red(self):
        node=self.node()
        self.step(node,10.01,10.)
        _,decision=self.step(node,10.06,10.05,raw=[R,G])
        self.assertEqual(decision['observed_tuples'],[list(R),list(G)])
        self.assertEqual(decision['normalized_tuples'],[])
        self.assertEqual(decision['final_tuples'],[])


if __name__=='__main__':unittest.main()
