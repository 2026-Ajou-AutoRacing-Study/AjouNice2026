#!/usr/bin/env python3
"""Candidate node wiring with fake ROS/model objects; no node/process/GPU startup."""
import importlib.util
import json
import sys
import types
import unittest
import threading
from collections import deque
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from traffic_light_map_selector import DEFAULT_SELECTOR_CONFIG, SELECTOR_CONTRACT_VERSION
from traffic_light_near_rescue import NearRescueConfig


class Duration:
    def __init__(self, value=0): self.value = value
    def to_sec(self): return self.value
    def __lt__(self, other): return self.value < other.value


class Stamp:
    def __init__(self, value=0, nsecs=0):
        self._nsecs = int(value * 1e9) + int(nsecs)
        self.value = self._nsecs / 1e9
    def to_sec(self): return self.value
    def to_nsec(self): return self._nsecs
    def __sub__(self, other): return Duration(self.value-other.value)
    @staticmethod
    def now(): return Stamp(10.)


class Message:
    def __init__(self, data=None): self.data = data


class Group:
    def __init__(self): self.elements = []


class GroupArray:
    def __init__(self): self.traffic_light_groups = []


class Element:
    RED, AMBER, GREEN, UNKNOWN, CIRCLE, RIGHT_ARROW, LEFT_ARROW, SOLID_ON = range(8)


def load_node():
    modules = {}
    def module(name, **attributes):
        result = types.ModuleType(name)
        result.__dict__.update(attributes)
        modules[name] = result
    module("rospy", Time=Stamp, Duration=Duration, logwarn_throttle=Mock(), logdebug=Mock(),
           logerr=Mock(), logwarn=Mock(), loginfo=Mock())
    module("sensor_msgs.msg", CompressedImage=Message)
    module("std_msgs.msg", String=Message, UInt64=Message)
    module("ultralytics", YOLO=Mock())
    module("autoware_perception_msgs.msg", TrafficLightElement=Element, TrafficLightGroup=Group,
           TrafficLightGroupArray=GroupArray)
    module("ros_time", builtin_time_from_ros=lambda value: value)
    name = "traffic_light_candidate_node_fake"
    spec = importlib.util.spec_from_file_location(name, PACKAGE / "scripts/traffic_light_node.py")
    result = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(result)
    return result


class CandidateNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.module = load_node()

    def node(self, mode="map_association", classifier="hsv"):
        node = object.__new__(self.module.TrafficLightNode)
        node._last_stamp, node._min_period = Stamp(0), Duration(0)
        node.status = Mock()
        detection = types.SimpleNamespace(xyxy=np.array([[480, 180, 530, 194.]]), conf=np.array([.9]))
        node.model = types.SimpleNamespace(predict=Mock(return_value=[types.SimpleNamespace(boxes=[detection])]))
        node.conf_threshold, node.inference_device = .25, "cpu"
        node.detector_imgsz = 640
        node.near_rescue_config = NearRescueConfig()
        node.runtime_metadata = {"detector_imgsz": 640}
        node.publish_debug_image, node.debug_pub = False, None
        node._resolve_group_id = lambda: 201290
        node._last_policy_group_id = None
        node._selected_head_id = None
        node.selection_mode, node.classifier_mode = mode, classifier
        node.cnn_crop_margin_px = 12
        node.center_tolerance_ratio, node.y_weight = .2, .6
        node.signal_smoother = Mock()
        node.planner_compliance_observations = False
        node.map_packet_max_age_sec = .5
        node.detection_miss_hold_sec = .2
        node.signal_phase_policy = Mock()
        node.signal_phase_policy.apply.side_effect = lambda raw, group, **kwargs: (raw, "unchanged")
        node.signal_phase_policy.process_normalized.side_effect = lambda raw, why, group, now, conf, smoother, **kwargs: (raw, conf, "unchanged")
        node.signal_phase_policy.output_observation_metadata.side_effect = lambda group: {
            "output_is_observed": True,
            "adopted_observation_source_stamp_ns":
                node.signal_phase_policy.process_normalized.call_args.kwargs["source_stamp_ns"]}
        for name in ("pub", "selection_debug_pub", "classifier_debug_pub", "observed_debug_pub", "raw_debug_pub", "policy_debug_pub", "runtime_debug_pub"):
            setattr(node, name, Mock())
        node._maybe_save_conflict_crop = Mock()
        node._log_hz_if_needed = Mock()
        node._proc_time_sum, node._proc_time_max = 0., 0.
        node._n_conflict_fixed = 0
        node._map_packet_for_image = Mock(return_value=None)
        return node

    def image(self):
        ok, encoded = cv2.imencode(".jpg", np.zeros((720, 1280, 3), dtype=np.uint8))
        self.assertTrue(ok)
        return types.SimpleNamespace(data=encoded.tobytes(),
            header=types.SimpleNamespace(stamp=Stamp(9.99), frame_id="camera_front"))

    def packet(self, group=201290):
        return {"status": "ready", "source_stamp_ns": 9990000000,
                "selector_contract_version": SELECTOR_CONTRACT_VERSION,
                "selector_config": DEFAULT_SELECTOR_CONFIG.to_dict(), "rois": [{
            "head_id": "C1256W000016", "group_id": group, "eligible": group == 201290,
            "bbox_xyxy": [480., 180., 530., 194.], "gate_xyxy": [450., 150., 560., 224.],
            "padding_xy_px": [30., 30.]}]}

    def debug(self, node, publisher):
        return json.loads(getattr(node, publisher).publish.call_args[0][0].data)

    def detector_results(self, *bboxes):
        return [types.SimpleNamespace(boxes=[types.SimpleNamespace(
            xyxy=np.array([bbox], dtype=float), conf=np.array([.9])) for bbox in bboxes])]

    def near_node(self, group=201290):
        node = self.node()
        node.detector_imgsz = node.runtime_metadata["detector_imgsz"] = 960
        node.near_rescue_config = NearRescueConfig(enabled=True)
        node._resolve_group_id = lambda: group
        packet = self.packet(group=group)
        packet["generated_stamp_ns"] = 9990000000
        packet["reason"] = "projected"
        packet["rois"][0].update(group_id=group, eligible=True, depth_m=10.,
            bbox_xyxy=[480., 180., 560., 200.], gate_xyxy=[450., 150., 590., 230.])
        node._map_packet_for_image.return_value = packet
        return node

    def assert_missing_signal(self, node):
        node.pub.publish.assert_called_once()
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])
        self.assertEqual(self.debug(node, "selection_debug_pub")["status"], "missing")
        self.assertFalse(self.debug(node, "classifier_debug_pub")["known"])

    def test_near_retry_uses_same_image_and_identity_gate_for_all_mission_groups(self):
        for group in (201351, 201365, 201290, 201326):
            with self.subTest(group=group):
                node = self.near_node(group=group)
                node.model.predict.side_effect = [self.detector_results(),
                    self.detector_results([480., 180., 560., 200.])]
                message = self.image()
                with patch.object(self.module, "classify_crop", return_value=([("red", "circle", None)], {})) as classify:
                    node.image_callback(message)
                self.assertEqual(node.model.predict.call_count, 2)
                first, second = [call.kwargs for call in node.model.predict.call_args_list]
                self.assertEqual([first["imgsz"], second["imgsz"]], [960, 640])
                self.assertIs(first["source"], second["source"])
                self.assertEqual(first["conf"], second["conf"])
                self.assertEqual(first["device"], second["device"])
                classify.assert_called_once()
                selection = self.debug(node, "selection_debug_pub")
                self.assertEqual(selection["status"], "matched")
                self.assertEqual(selection["group_id"], group)
                self.assertEqual(selection["source_stamp_ns"], message.header.stamp.to_nsec())
                self.assertEqual(selection["projection_source_stamp_ns"], message.header.stamp.to_nsec())
                rescue = selection["near_rescue"]
                self.assertTrue(rescue["attempted"])
                self.assertTrue(rescue["accepted"])
                self.assertEqual(rescue["eligible_head_ids"], [selection["head_id"]])
                report = self.debug(node, "runtime_debug_pub")
                self.assertEqual(report["source_stamp_ns"], message.header.stamp.to_nsec())
                self.assertEqual(report["frame_decision"]["near_rescue"], rescue)
                timing = report["frame_timing"]
                self.assertGreater(timing["near_rescue_detector_and_association_sec"], 0)
                self.assertLessEqual(timing["near_rescue_detector_and_association_sec"],
                                     timing["selection_including_map_wait_sec"])
                # Rescue is a subset of selection, not an additional additive stage.
                self.assertAlmostEqual(timing["callback_to_output_sec"], sum(timing[key] for key in
                    ("decode_sec", "detector_and_box_readback_sec", "selection_including_map_wait_sec",
                     "crop_and_classifier_sec", "policy_and_publish_sec")))
                self.assertEqual(self.debug(node, "classifier_debug_pub")["source_stamp_ns"],
                                 message.header.stamp.to_nsec())
                self.assertIs(node.pub.publish.call_args[0][0].stamp, message.header.stamp)

    def test_near_retry_context_accepts_fresh_generation_stamp_at_inclusive_boundaries(self):
        # The fake ROS clock is fixed at 10 s. Retry may accept a packet
        # generated now or exactly max_age ago, but never future/expired data.
        node = self.near_node()
        node.map_packet_max_age_sec = .5
        packet = node._map_packet_for_image.return_value
        for generated_stamp_ns in (10000000000, 9990000000, 9500000000):
            with self.subTest(generated_stamp_ns=generated_stamp_ns):
                packet["generated_stamp_ns"] = generated_stamp_ns
                self.assertIsNone(node._near_retry_context_reason(packet, 201290))

    def test_context_change_during_secondary_inference_preserves_missing_publication(self):
        cases = (
            ("route_changed", "route_changed_during_retry"),
            ("route_became_unknown", "route_changed_during_retry"),
            ("expired", "map_packet_expired_during_retry"),
            ("future", "map_packet_expired_during_retry"),
            ("missing_generation", "invalid_retry_packet_generation_stamp"),
        )
        for condition, expected_reason in cases:
            with self.subTest(condition=condition):
                node = self.near_node()
                packet = node._map_packet_for_image.return_value
                calls = []

                def predict(**kwargs):
                    calls.append(kwargs)
                    if len(calls) == 1:
                        return self.detector_results()
                    # The first selection is valid. Change context only while
                    # the secondary detector runs to exercise the final guard.
                    if condition == "route_changed":
                        node._resolve_group_id = lambda: 201326
                    elif condition == "route_became_unknown":
                        node._resolve_group_id = lambda: 0
                    elif condition == "expired":
                        packet["generated_stamp_ns"] = 9499999999
                    elif condition == "future":
                        packet["generated_stamp_ns"] = 10000000001
                    else:
                        packet.pop("generated_stamp_ns")
                    return self.detector_results([480., 180., 560., 200.])

                node.model.predict.side_effect = predict
                message = self.image()
                with patch.object(self.module, "classify_crop") as classify:
                    node.image_callback(message)
                self.assertEqual(node.model.predict.call_count, 2)
                classify.assert_not_called()
                self.assert_missing_signal(node)
                selection = self.debug(node, "selection_debug_pub")
                self.assertEqual(selection["reason"], "no_unambiguous_route_detection")
                self.assertNotIn("head_id", selection)
                self.assertIsNone(node._selected_head_id)
                rescue = selection["near_rescue"]
                self.assertTrue(rescue["attempted"])
                self.assertFalse(rescue["accepted"])
                self.assertEqual(rescue["context_rejection"], expected_reason)
                # A geometrically valid retry detection must not bypass the
                # context guard or reach the classifier/permission policy.
                self.assertEqual(rescue["result"]["status"], "matched")
                self.assertEqual(selection["source_stamp_ns"], message.header.stamp.to_nsec())
                self.assertIs(node.pub.publish.call_args[0][0].stamp, message.header.stamp)
                report = self.debug(node, "runtime_debug_pub")
                self.assertEqual(report["frame_decision"]["near_rescue"], rescue)
                self.assertGreater(report["frame_timing"]["near_rescue_detector_and_association_sec"], 0)

    def test_near_first_match_does_not_run_secondary_detector(self):
        node = self.near_node()
        node.model.predict.return_value = self.detector_results([480., 180., 560., 200.])
        with patch.object(self.module, "classify_crop", return_value=([], {})) as classify:
            node.image_callback(self.image())
        node.model.predict.assert_called_once()
        classify.assert_called_once()
        rescue = self.debug(node, "selection_debug_pub")["near_rescue"]
        self.assertFalse(rescue["attempted"])
        self.assertEqual(rescue["reason"], "first_selection_not_missing")

    def test_near_retry_remains_off_without_explicit_enable(self):
        node = self.near_node()
        node.near_rescue_config = NearRescueConfig()
        node.model.predict.return_value = self.detector_results()
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        node.model.predict.assert_called_once()
        classify.assert_not_called()
        self.assert_missing_signal(node)
        self.assertEqual(self.debug(node, "selection_debug_pub")["near_rescue"]["reason"], "disabled")

    def test_far_missing_stale_and_invalid_packets_never_trigger_near_retry(self):
        for condition in ("far", "missing", "projection_missing", "old_contract", "invalid_rois"):
            with self.subTest(condition=condition):
                node = self.near_node()
                node.model.predict.return_value = self.detector_results()
                packet = node._map_packet_for_image.return_value
                if condition == "far":
                    packet["rois"][0]["depth_m"] = 15.01
                elif condition == "missing":
                    node._map_packet_for_image.return_value = None
                elif condition == "projection_missing":
                    packet.update(status="missing", reason="localization_stale")
                elif condition == "old_contract":
                    packet["selector_contract_version"] = SELECTOR_CONTRACT_VERSION - 1
                else:
                    packet["rois"] = None
                with patch.object(self.module, "classify_crop") as classify:
                    node.image_callback(self.image())
                node.model.predict.assert_called_once()
                classify.assert_not_called()
                self.assert_missing_signal(node)
                rescue = self.debug(node, "selection_debug_pub")["near_rescue"]
                self.assertFalse(rescue["attempted"])
                if condition in ("missing", "projection_missing"):
                    self.assertEqual(rescue["reason"], "map_packet_missing_or_stale"
                                     if condition == "missing" else "localization_stale")
                    self.assertEqual(self.debug(node, "runtime_debug_pub")["frame_decision"]["near_rescue"],
                                     rescue)

    def test_ambiguous_primary_never_retries_or_classifies(self):
        node = self.near_node()
        packet = node._map_packet_for_image.return_value
        packet["rois"].append(dict(packet["rois"][0], head_id="other_mission",
                                   group_id=201351, eligible=False))
        node.model.predict.return_value = self.detector_results([480., 180., 560., 200.])
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        node.model.predict.assert_called_once()
        classify.assert_not_called()
        self.assert_missing_signal(node)
        rescue = self.debug(node, "selection_debug_pub")["near_rescue"]
        self.assertFalse(rescue["attempted"])
        self.assertEqual(rescue["reason"], "other_head_identity_ambiguous")

    def test_neighbor_projection_cannot_authorize_retry_even_if_packet_lookup_returns_it(self):
        node = self.near_node()
        # A nonzero primary projection tolerance must not silently widen the
        # stricter same-image retry contract.
        node._map_packet_for_image.return_value["source_stamp_ns"] = 9980000000
        node.model.predict.return_value = self.detector_results()
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        node.model.predict.assert_called_once()
        classify.assert_not_called()
        self.assert_missing_signal(node)
        selection = self.debug(node, "selection_debug_pub")
        self.assertEqual(selection["source_stamp_ns"], 9990000000)
        self.assertEqual(selection["projection_source_stamp_ns"], 9980000000)
        self.assertFalse(selection["near_rescue"]["retry"])
        self.assertFalse(selection["near_rescue"]["attempted"])
        self.assertEqual(selection["near_rescue"]["reason"], "retry_requires_exact_image_stamp")

    def test_ambiguous_secondary_cannot_override_identity_gate(self):
        node = self.near_node()
        packet = node._map_packet_for_image.return_value
        packet["rois"].append(dict(packet["rois"][0], head_id="other_mission",
                                   group_id=201351, eligible=False))
        node.model.predict.side_effect = [self.detector_results(),
            self.detector_results([480., 180., 560., 200.])]
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        self.assertEqual(node.model.predict.call_count, 2)
        classify.assert_not_called()
        self.assert_missing_signal(node)
        rescue = self.debug(node, "selection_debug_pub")["near_rescue"]
        self.assertTrue(rescue["attempted"])
        self.assertFalse(rescue["accepted"])
        self.assertEqual(rescue["result"]["rejected"][0]["reason"], "other_head_identity_ambiguous")

    def test_retry_cannot_accept_another_head_even_if_it_is_same_mission(self):
        for other_group in (201290, 201351):
            with self.subTest(other_group=other_group):
                node = self.near_node()
                packet = node._map_packet_for_image.return_value
                packet["rois"].append(dict(packet["rois"][0], head_id="other_head",
                    group_id=other_group, eligible=other_group == 201290, depth_m=30.,
                    bbox_xyxy=[800., 180., 880., 200.], gate_xyxy=[770., 150., 910., 230.]))
                node.model.predict.side_effect = [self.detector_results(),
                    self.detector_results([800., 180., 880., 200.])]
                with patch.object(self.module, "classify_crop") as classify:
                    node.image_callback(self.image())
                self.assertEqual(node.model.predict.call_count, 2)
                classify.assert_not_called()
                self.assert_missing_signal(node)
                selection = self.debug(node, "selection_debug_pub")
                self.assertNotIn("head_id", selection)
                self.assertIsNone(node._selected_head_id)
                rescue = selection["near_rescue"]
                self.assertTrue(rescue["attempted"])
                self.assertFalse(rescue["accepted"])
                self.assertNotIn("other_head", rescue["eligible_head_ids"])
                if other_group == 201290:
                    self.assertEqual(rescue["result"]["head_id"], "other_head")

    def test_retry_error_preserves_missing_publication_and_source_stamp(self):
        node = self.near_node()
        node.model.predict.side_effect = [self.detector_results(), RuntimeError("secondary failed")]
        message = self.image()
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(message)
        self.assertEqual(node.model.predict.call_count, 2)
        classify.assert_not_called()
        self.assert_missing_signal(node)
        selection = self.debug(node, "selection_debug_pub")
        self.assertEqual(selection["reason"], "no_unambiguous_route_detection")
        self.assertEqual(selection["near_rescue"]["error_type"], "RuntimeError")
        self.assertFalse(selection["near_rescue"]["accepted"])
        self.assertEqual(selection["source_stamp_ns"], message.header.stamp.to_nsec())
        self.assertIs(node.pub.publish.call_args[0][0].stamp, message.header.stamp)
        self.assertEqual(self.debug(node, "runtime_debug_pub")["source_stamp_ns"],
                         message.header.stamp.to_nsec())

    def test_legacy_and_primary_640_never_run_duplicate_retry(self):
        for condition in ("legacy", "already_640"):
            with self.subTest(condition=condition):
                node = self.near_node()
                node.model.predict.return_value = self.detector_results()
                if condition == "legacy":
                    node.selection_mode = "legacy_center"
                else:
                    node.detector_imgsz = 640
                with patch.object(self.module, "classify_crop") as classify:
                    node.image_callback(self.image())
                node.model.predict.assert_called_once()
                classify.assert_not_called()
                rescue = self.debug(node, "selection_debug_pub")["near_rescue"]
                self.assertFalse(rescue["attempted"])
                self.assertEqual(rescue["reason"], "not_map_association" if condition == "legacy"
                                 else "primary_already_retry_scale")
                if condition == "legacy":
                    node._map_packet_for_image.assert_not_called()

    def test_matched_detector_with_unknown_cnn_is_not_a_retry_request(self):
        node = self.near_node()
        node.classifier_mode = "cnn"
        node.model.predict.return_value = self.detector_results([480., 180., 560., 200.])
        observation = types.SimpleNamespace(known=False, tuples=(), confidence=0.,
            to_dict=lambda: {"known": False, "reason": "low_confidence_bits:left_arrow"})
        node._cnn_startup = types.SimpleNamespace(ready=True,
            classifier=types.SimpleNamespace(classify_bgr=Mock(return_value=observation)))
        with patch.object(self.module, "classify_crop") as hsv:
            node.image_callback(self.image())
        hsv.assert_not_called()
        node.model.predict.assert_called_once()
        self.assertEqual(self.debug(node, "selection_debug_pub")["status"], "matched")
        self.assertFalse(self.debug(node, "selection_debug_pub")["near_rescue"]["attempted"])
        self.assertFalse(self.debug(node, "classifier_debug_pub")["known"])
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])

    def test_map_missing_does_not_classify_unrelated_center_box_or_abort(self):
        node = self.node()
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        classify.assert_not_called()
        node.pub.publish.assert_called_once()
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])
        self.assertEqual(self.debug(node, "selection_debug_pub")["status"], "missing")

    def test_wrong_route_head_does_not_inherit_current_route_id(self):
        node = self.node()
        node._map_packet_for_image.return_value = self.packet(group=201351)
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        classify.assert_not_called()
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])

    def test_old_D1_not_a_mission_even_if_route_helper_and_YOLO_report_it(self):
        node = self.node()
        node._external_route_group_id = 201397
        node._external_route_group_id_stamp = Stamp(9.99)
        node.route_group_id_stale_sec, node.fallback_group_id = 2., 201397
        node._resolve_group_id = types.MethodType(self.module.TrafficLightNode._resolve_group_id, node)
        self.assertEqual(node._resolve_group_id(), 0)
        node._map_packet_for_image.return_value = self.packet(group=201397)
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        classify.assert_not_called()
        output = node.pub.publish.call_args[0][0]
        self.assertNotIn(201397, [g.traffic_light_group_id for g in output.traffic_light_groups])

    def test_valid_map_head_classifies_exact_selected_detection(self):
        node = self.node()
        node._map_packet_for_image.return_value = self.packet()
        with patch.object(self.module, "classify_crop", return_value=([("red", "circle", None)], {"red": 6})) as classify:
            node.image_callback(self.image())
        self.assertEqual(classify.call_args[0][0].shape, (14, 50, 3))
        self.assertEqual(self.debug(node, "selection_debug_pub")["head_id"], "C1256W000016")
        classifier = self.debug(node, "classifier_debug_pub")
        self.assertEqual(classifier["source_stamp_ns"], 9990000000)
        self.assertEqual(classifier["group_id"], 201290)
        self.assertEqual(classifier["head_id"], "C1256W000016")
        decision = self.debug(node, "runtime_debug_pub")["frame_decision"]
        self.assertEqual(decision["head_id"], "C1256W000016")
        self.assertEqual(decision["observed_tuples"], [["red", "circle", None]])
        self.assertEqual(decision["final_tuples"], [["red", "circle", None]])
        timing = self.debug(node, "runtime_debug_pub")["frame_timing"]
        self.assertAlmostEqual(timing["header_to_callback_sec"], .01)
        self.assertGreaterEqual(timing["callback_to_output_sec"], 0)
        self.assertAlmostEqual(timing["callback_to_output_sec"], sum(timing[key] for key in
            ("decode_sec", "detector_and_box_readback_sec", "selection_including_map_wait_sec",
             "crop_and_classifier_sec", "policy_and_publish_sec")))
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [("red", "circle", None)])

    def test_mixed_old_helper_contract_is_missing_not_implicit_new_defaults(self):
        node = self.node()
        packet = self.packet(); packet["selector_contract_version"] = 1
        node._map_packet_for_image.return_value = packet
        with patch.object(self.module, "classify_crop") as classify:
            node.image_callback(self.image())
        classify.assert_not_called()
        self.assertIn("selector_contract_version_mismatch", self.debug(node, "selection_debug_pub")["reason"])

    def test_legacy_default_path_remains_independent_of_map_packets(self):
        node = self.node(mode="legacy_center")
        with patch.object(self.module, "classify_crop", return_value=([], {})) as classify:
            node.image_callback(self.image())
        classify.assert_called_once()
        node._map_packet_for_image.assert_not_called()
        self.assertEqual(node.model.predict.call_args[1]["imgsz"], 640)
        self.assertEqual(self.debug(node, "runtime_debug_pub")["detector_actual_device"], "cpu")

    def test_explicit_960_detector_resolution_is_forwarded_and_recorded(self):
        node = self.node()
        node.detector_imgsz = 960
        node.runtime_metadata["detector_imgsz"] = 960
        node.image_callback(self.image())
        self.assertEqual(node.model.predict.call_args[1]["imgsz"], 960)
        self.assertEqual(self.debug(node, "runtime_debug_pub")["detector_imgsz"], 960)

    def test_runtime_device_prefers_deepcopied_predictor_over_outer_cpu(self):
        node = self.node()
        node.inference_device = "0"
        node.model.device = "cpu"
        node.model.predictor = types.SimpleNamespace(device="cuda:0",
            model=types.SimpleNamespace(device="cuda:1"))
        fake_cuda = types.SimpleNamespace(memory_allocated=Mock(return_value=11),
            memory_reserved=Mock(return_value=33), max_memory_allocated=Mock(return_value=22))
        fake_torch = types.ModuleType("torch"); fake_torch.cuda = fake_cuda
        timing, decision = {"callback_to_output_sec": .025}, {"group_id": 201290}
        with patch.dict(sys.modules, {"torch": fake_torch}):
            node._publish_runtime_debug(Stamp(9.99), timing, decision)
        report = self.debug(node, "runtime_debug_pub")
        self.assertEqual(report["detector_actual_device"], "cuda:0")
        self.assertEqual(report["detector_actual_device_source"], "predictor.device")
        self.assertEqual(report["cuda_memory"], {"device": "cuda:0", "allocated_bytes": 11,
                                                "reserved_bytes": 33, "max_allocated_bytes": 22})
        for query in (fake_cuda.memory_allocated, fake_cuda.memory_reserved, fake_cuda.max_memory_allocated):
            query.assert_called_once_with("cuda:0")
        self.assertEqual(report["source_stamp_ns"], 9990000000)
        self.assertEqual(report["frame_timing"], timing)
        self.assertEqual(report["frame_decision"], decision)
        self.assertEqual(node.model.device, "cpu")
        node.model.predict.assert_not_called()
        node.pub.publish.assert_not_called()

    def test_runtime_device_backend_outer_and_requested_fallbacks(self):
        cases = (
            (types.SimpleNamespace(device="cpu", predictor=types.SimpleNamespace(
                device=None, model=types.SimpleNamespace(device="cuda:2"))),
                ("cuda:2", "predictor.model.device")),
            (types.SimpleNamespace(device="cuda:3", predictor=None), ("cuda:3", "model.device")),
            (types.SimpleNamespace(device="cpu"), ("cpu", "model.device")),
            (types.SimpleNamespace(device=None), ("cpu", "requested_device_fallback")),
            (types.SimpleNamespace(), ("cpu", "requested_device_fallback")),
            (types.SimpleNamespace(device="cuda:0", predictor=types.SimpleNamespace(
                device="cpu", model=types.SimpleNamespace(device="cuda:1"))),
                ("cpu", "predictor.device")),
        )
        for model, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(self.module._detector_runtime_device(model, "cpu"), expected)

    def test_cnn_unavailable_and_low_confidence_never_fall_back_to_hsv(self):
        for ready in (False, True):
            node = self.node(classifier="cnn")
            node._map_packet_for_image.return_value = self.packet()
            observation = types.SimpleNamespace(known=False, tuples=(), confidence=0.,
                to_dict=lambda: {"known": False, "reason": "low_confidence_bits:left_arrow"})
            classifier = types.SimpleNamespace(classify_bgr=Mock(return_value=observation))
            node._cnn_startup = types.SimpleNamespace(ready=ready, classifier=classifier, reason="checkpoint_missing")
            with patch.object(self.module, "classify_crop") as hsv:
                node.image_callback(self.image())
            hsv.assert_not_called()
            self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])
            self.assertFalse(self.debug(node, "classifier_debug_pub")["known"])

    def test_cnn_string_tuple_order_is_preserved_before_policy(self):
        node = self.node(classifier="cnn")
        node._map_packet_for_image.return_value = self.packet()
        raw = (("red", "circle", None), ("green", "arrow", True))
        observation = types.SimpleNamespace(known=True, tuples=raw, confidence=.8,
            to_dict=lambda: {"known": True, "reason": "observed_lamps"})
        node._cnn_startup = types.SimpleNamespace(ready=True,
            classifier=types.SimpleNamespace(classify_bgr=Mock(return_value=observation)))
        with patch.object(self.module, "classify_crop") as hsv:
            node.image_callback(self.image())
        hsv.assert_not_called()
        self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], list(raw))
        self.assertAlmostEqual(node.signal_phase_policy.process_normalized.call_args[0][4], .8)
        crop = node._cnn_startup.classifier.classify_bgr.call_args[0][0]
        self.assertEqual(crop.shape, (38, 74, 3))
        self.assertEqual(self.debug(node, "classifier_debug_pub")["crop_xyxy"], [468, 168, 542, 206])

    def test_exact_image_packet_callback_race_has_bounded_condition_wait(self):
        node = self.node()
        node._roi_condition = threading.Condition()
        node._roi_packets = deque()
        node.map_packet_wait_sec, node.map_packet_max_age_sec = .05, .5
        node.map_packet_max_image_delta_sec = 0.
        node._map_packet_for_image = types.MethodType(self.module.TrafficLightNode._map_packet_for_image, node)
        packet = dict(self.packet(), schema_version=1, generated_stamp_ns=9990000000,
                      group_id=201290, image_width=1280, image_height=720, camera_frame="camera_front")
        timer = threading.Timer(.005, lambda: node.map_roi_callback(Message(json.dumps(packet))))
        timer.start()
        try:
            selected = node._map_packet_for_image(self.image(), 201290, 1280, 720)
        finally:
            timer.join()
        self.assertIsNotNone(selected)
        self.assertEqual(selected["source_stamp_ns"], 9990000000)

    def test_exact_image_default_does_not_reuse_neighbor_projection(self):
        node = self.node()
        node._roi_condition, node._roi_packets = threading.Condition(), deque()
        node.map_packet_wait_sec, node.map_packet_max_age_sec = 0., .5
        node.map_packet_max_image_delta_sec = 0.
        node._map_packet_for_image = types.MethodType(self.module.TrafficLightNode._map_packet_for_image, node)
        node._roi_packets.append(dict(self.packet(), schema_version=1, source_stamp_ns=9980000000,
            generated_stamp_ns=9990000000, group_id=201290, image_width=1280, image_height=720,
            camera_frame="camera_front"))
        self.assertIsNone(node._map_packet_for_image(self.image(), 201290, 1280, 720))


if __name__ == "__main__":
    unittest.main()
