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
from traffic_light_cnn_classifier import observation_from_probabilities

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
             head='test_head', group=None, projection_head=None, unknown=False,
             observation=None, motion_speed=None, latest_motion_speed=None):
        group = node._resolve_group_id() if group is None else group
        message = self.image();message.header.stamp = fixture.Stamp(source)
        packet = fixture.CandidateNodeTest.packet(self, group=group)
        packet.update(group_id=group, source_stamp_ns=message.header.stamp.to_nsec(),
                      generated_stamp_ns=message.header.stamp.to_nsec(),
                      status='ready' if ready else 'missing', reason='projected' if ready else 'localization_missing')
        packet['rois'][0].update(group_id=group, eligible=True,
                                  head_id=head if projection_head is None else projection_head)
        if motion_speed is not None:
            packet['motion_evidence']=dict(schema_version=1,valid=True,
                aligned_speed_mps=motion_speed,
                latest_speed_mps=motion_speed if latest_motion_speed is None else latest_motion_speed,
                aligned_pose_stamp_ns=message.header.stamp.to_nsec(),
                latest_pose_stamp_ns=message.header.stamp.to_nsec(),
                latest_pose_receipt_stamp_ns=fixture.Stamp(now).to_nsec())
        node._map_packet_for_image.return_value = packet
        node.model.predict.return_value = self.detector_results([480,180,530,194]) if detected else self.detector_results()
        obs = types.SimpleNamespace(known=not unknown, tuples=list(raw) if not unknown else [], confidence=.95)
        obs.to_dict = lambda: dict(known=obs.known, reason='unknown' if unknown else 'observed_lamps',
                                  tuples=obs.tuples, confidence=obs.confidence)
        if observation is not None:
            obs=observation
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

    def test_unknown_stream_keeps_current_source_then_recovers_without_old_green(self):
        for group in (201351,201365,201290,201326):
            with self.subTest(group=group):
                node=self.node(group)
                self.step(node,10.01,10.)
                # Fresh UNKNOWN is not transport silence, even after >1s.
                for index in range(1,21):
                    source=10.+index*.1
                    output,decision=self.step(node,source+.01,source,unknown=True)
                    self.assertEqual(output.stamp.to_nsec(),fixture.Stamp(source).to_nsec())
                    self.assertEqual(decision['final_tuples'],[])
                    self.assertFalse(decision['observation_timing']['output_is_held'])
                output,decision=self.step(node,12.11,12.1,raw=[R])
                self.assertEqual(decision['final_tuples'],[list(R)])
                self.assertEqual(output.stamp.to_nsec(),12_100_000_000)

    def test_hold_budget_includes_original_input_age_not_only_gap_between_callbacks(self):
        node=self.node()
        self.step(node,10.15,10.)
        # A 60ms callback gap cannot reuse a source already 210ms old.
        _,decision=self.step(node,10.21,10.20,detected=False)
        self.assertTrue(decision['detection_miss_hold']['eligible'])
        self.assertEqual(decision['final_tuples'],[])
        self.assertFalse(decision['observation_timing']['output_is_held'])

    def test_mission_change_and_invalid_route_never_borrow_previous_green(self):
        node=self.node()
        self.step(node,10.01,10.)
        for group,now,source in ((0,10.06,10.05),(201351,10.11,10.10),
                                 (201326,10.16,10.15)):
            node._resolve_group_id=lambda group=group: group
            output,decision=self.step(node,now,source,detected=False)
            self.assertEqual(output.traffic_light_groups[0].traffic_light_group_id,group)
            self.assertEqual(decision['final_tuples'],[])
            self.assertFalse(decision['observation_timing']['output_is_held'])

    def test_two_soft_unknown_frames_keep_source_without_permission_reconfirmation(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        node=self.node()
        # Include 237ms input age rather than assuming zero transport delay.
        self.step(node,10.237,10.,motion_speed=2.)
        for now,source in ((10.35,10.1),(10.45,10.2)):
            output,decision=self.step(node,now,source,observation=soft,motion_speed=2.)
            self.assertTrue(decision['low_confidence_hold']['eligible'])
            self.assertEqual(decision['final_tuples'],[list(G)])
            self.assertEqual(output.stamp.to_nsec(),10_000_000_000)
            self.assertEqual(decision['observation_timing']['output_origin'],'low_confidence_hold')
        _,decision=self.step(node,10.501,10.3,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[]) # absolute source age >500ms

    def test_processing_deadline_expires_and_unknown_cannot_renew_it(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        node=self.node()
        self.step(node,10.01,10.,motion_speed=2.)
        _,decision=self.step(node,10.20,10.19,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[list(G)])
        _,decision=self.step(node,10.311,10.30,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])
        _,decision=self.step(node,10.32,10.31,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])

    def test_processing_budget_is_inclusive_at_300ms_not_float_subtraction_dependent(self):
        node=self.node();self.step(node,10.01,10.,motion_speed=2.)
        _,decision=self.step(node,10.31,10.3,
            observation=observation_from_probabilities((.01,.01,.5,.01)),motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[list(G)])
        _,decision=self.step(node,10.310001,10.300001,
            observation=observation_from_probabilities((.01,.01,.5,.01)),motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])

    def test_reprocessed_green_jpeg_does_not_renew_processing_deadline(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        node=self.node()
        self.step(node,10.01,10.,motion_speed=2.)
        self.step(node,10.20,10.,motion_speed=2.)
        _,decision=self.step(node,10.32,10.3,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])

    def test_reprocessed_green_after_cache_invalidation_cannot_recreate_a_new_lease(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        node=self.node();self.step(node,10.01,10.,motion_speed=2.)
        # Same source UNKNOWN invalidates the cache, but not original provenance.
        self.step(node,10.32,10.,observation=soft,motion_speed=2.)
        self.step(node,10.40,10.,motion_speed=2.)
        _,decision=self.step(node,10.45,10.1,observation=soft,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])

    def test_regressed_green_source_is_unknown_and_future_source_does_not_poison_highwater(self):
        node=self.node();self.step(node,10.01,10.,motion_speed=2.)
        _,decision=self.step(node,10.11,9.99,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])
        _,decision=self.step(node,10.12,10.1,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[list(G)])
        _,decision=self.step(node,10.13,10.3,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])
        _,decision=self.step(node,10.15,10.14,motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[list(G)])

    def test_stopped_latest_pose_or_missing_motion_never_holds_soft_unknown(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        for speed,latest in ((None,None),(0.,0.),(2.,0.),(.99,2.),(float('nan'),2.)):
            with self.subTest(speed=speed,latest=latest):
                node=self.node()
                self.step(node,10.01,10.,motion_speed=2.)
                _,decision=self.step(node,10.10,10.09,observation=soft,
                                     motion_speed=speed,latest_motion_speed=latest)
                self.assertEqual(decision['final_tuples'],[])

    def test_real_red_yellow_no_lamps_or_inference_error_end_soft_hold(self):
        soft=observation_from_probabilities((.01,.01,.5,.01))
        for observation in (observation_from_probabilities((.99,.01,.01,.01)),
                            observation_from_probabilities((.01,.99,.01,.01)),
                            observation_from_probabilities((.01,.01,.01,.01)),
                            observation_from_probabilities((float('nan'),.01,.01,.01))):
            node=self.node();self.step(node,10.01,10.,motion_speed=2.)
            self.step(node,10.08,10.07,observation=soft,motion_speed=2.)
            _,decision=self.step(node,10.11,10.10,observation=observation,motion_speed=2.)
            self.assertFalse(decision['observation_timing']['output_is_held'])
            self.assertNotEqual(decision['final_tuples'],[list(G)])

    def test_confident_new_red_or_yellow_bit_cannot_hide_inside_soft_abstention(self):
        for probabilities in ((.99,.01,.5,.01),(.01,.99,.5,.01)):
            node=self.node();self.step(node,10.01,10.,motion_speed=2.)
            _,decision=self.step(node,10.10,10.09,
                observation=observation_from_probabilities(probabilities),motion_speed=2.)
            self.assertEqual(decision['final_tuples'],[])

    def test_d_protected_arrow_soft_gap_keeps_observed_red_and_arrow_not_straight_green(self):
        node=self.node(201326);self.step(node,10.01,10.,raw=[R,L],motion_speed=2.)
        output,decision=self.step(node,10.10,10.09,
            observation=observation_from_probabilities((.99,.01,.01,.5)),motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[list(R),list(L)])
        self.assertEqual(output.stamp.to_nsec(),10_000_000_000)

    def test_zero_option_disables_soft_hold_without_disabling_detector_budget(self):
        node=self.node();node.low_confidence_hold_sec=0.
        self.step(node,10.01,10.,motion_speed=2.)
        _,decision=self.step(node,10.08,10.07,
            observation=observation_from_probabilities((.01,.01,.5,.01)),motion_speed=2.)
        self.assertEqual(decision['final_tuples'],[])


if __name__=='__main__':unittest.main()
