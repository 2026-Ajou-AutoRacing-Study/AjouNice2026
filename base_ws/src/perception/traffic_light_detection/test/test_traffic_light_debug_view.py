#!/usr/bin/env python3
"""Diagnostic-only overlay/viewer tests: no ROS graph, model, RPC, or vehicle."""
from pathlib import Path
import ast
import importlib.util
import os
import sys
import types
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
import traffic_light_debug_view as view
import test_traffic_light_candidate_node as fixture


class OverlayTest(unittest.TestCase):
    def metadata(self, **values):
        base = {"group_id": 201326, "head_id": "C1256W000052", "classifier_mode": "cnn",
                "classifier_known": True, "observed_lamps": "YC", "normalized_lamps": "RC",
                "final_lamps": "RC+GLA", "confidence": .8, "selection_status": "missing",
                "selection_reason": "no_unambiguous_route_detection", "source_stamp_ns": 2000000000,
                "observation_timing": {"output_is_observed": True, "output_is_held": True,
                    "output_origin": "bounded_observation_hold", "adopted_observation_source_stamp_ns": 1000000000,
                    "output_observation_age_sec": 1.},
                "planner_parameters": {"enable_signal_compliance_guard": True, "enable_yellow_passthrough": False, "yellow_light_is_go": True,
                    "enable_arrow_aware_yellow_passing": False, "tl_state_timeout": .5}}
        base.update(values)
        return base

    def test_raw_profile_output_and_held_missing_are_not_conflated(self):
        lines = "\n".join(view.overlay_lines(self.metadata()))
        for required in ("MISSION D", "C1256W000052", "RAW CNN: YC", "PROFILE/CORRECTION: RC",
                         "OUTPUT LAMPS: RC+GLA", "MISSING | HELD OBSERVATION", "adopted lamp age=1.000s",
                         "NOT a planner GO decision", "CONFIG ONLY", "yellow_passthrough=OFF", "flag is NOT GO permission"):
            self.assertIn(required, lines)
        self.assertIn("LAMP AGE EXCEEDS CONFIGURED TTL", lines)

    def test_new_yellow_master_not_legacy_ON_values_is_displayed(self):
        metadata = self.metadata()
        # Legacy compatibility values may remain ON with the actual new
        # yellow master OFF. They must not make the GUI look enabled.
        metadata["planner_parameters"].update(yellow_light_is_go=True,
            enable_arrow_aware_yellow_passing=True, enable_yellow_passthrough=False)
        lines = "\n".join(view.overlay_lines(metadata))
        self.assertIn("yellow_passthrough=OFF", lines)
        self.assertNotIn("yellow_is_go=ON", lines)
        self.assertNotIn("arrow-aware-yellow=ON", lines)
        metadata["planner_parameters"]["enable_yellow_passthrough"] = True
        self.assertIn("yellow_passthrough=ON", "\n".join(view.overlay_lines(metadata)))

    def test_synthetic_red_has_no_fresh_lamp_age(self):
        lines = "\n".join(view.overlay_lines(self.metadata(classifier_known=False, observation_timing={
            "output_is_observed": False, "output_origin": "policy_fallback",
            "output_observation_age_sec": .01, "adopted_observation_source_stamp_ns": 123})))
        self.assertIn("UNKNOWN (no trusted", lines)
        self.assertIn("NOT A LAMP OBSERVATION", lines)
        self.assertIn("adopted lamp age=NONE", lines)
        self.assertIn("adopted ns=NONE", lines)

    def test_all_four_missions_and_invalid_group_are_display_only(self):
        for group, expected in ((201351, "A"), (201365, "B"), (201290, "C (2 heads)"), (201326, "D"), (0, "NONE")):
            self.assertTrue(view.overlay_lines(self.metadata(group_id=group))[0].startswith("MISSION " + expected))

    def test_overlay_preserves_original_fov_input_and_distinguishes_boxes(self):
        image = np.zeros((720, 1280, 3), np.uint8)
        boxes = [(0, 0, 0, 0, (30, 40, 80, 60, .7)), (0, 0, 0, 0, (200, 200, 260, 230, .9))]
        output = view.render_node_overlay(image, boxes, [200, 200, 260, 230], [190, 190, 270, 240], self.metadata())
        self.assertEqual(output.shape[1], 1280)
        self.assertGreater(output.shape[0], 720)
        self.assertEqual(int(image.sum()), 0)
        self.assertEqual(tuple(output[50, 30]), view.ALL_BOX_COLOR)
        self.assertEqual(tuple(output[215, 200]), view.SELECTED_BOX_COLOR)
        self.assertEqual(tuple(output[220, 190]), view.CROP_BOX_COLOR)


class ViewerFreshnessTest(unittest.TestCase):
    def test_source_stamp_and_wall_receipt_both_have_to_be_fresh(self):
        self.assertFalse(view.frame_freshness(10., 9.9, 10000000000, 9900000000)["stale"])
        for values in ((10., 9., 10000000000, 9900000000),
                       (10., 9.9, 10000000000, 9000000000),
                       (10., 10.1, 10000000000, 9900000000),
                       (10., 9.9, 10000000000, 11000000000),
                       (10., 9.9, 10000000000, 0),
                       (10., None, 10000000000, 9900000000)):
            self.assertTrue(view.frame_freshness(*values)["stale"], values)

    def test_frozen_ros_clock_still_becomes_stale_by_wall_time(self):
        self.assertTrue(view.frame_freshness(11., 10., 10000000000, 10000000000)["stale"])

    def test_old_green_is_desaturated_and_decode_error_does_not_look_live(self):
        image = np.full((200, 300, 3), (0, 255, 0), np.uint8)
        stale = view.frame_freshness(10., 9., 10000000000, 9900000000)
        result = view.render_viewer_frame(image, stale)
        pixel = result[160, 150]
        self.assertEqual(pixel[0], pixel[1])
        self.assertEqual(pixel[1], pixel[2])
        fresh = view.frame_freshness(10., 9.99, 10000000000, 9990000000)
        error = view.render_viewer_frame(image, fresh, error="corrupt JPEG")
        self.assertEqual(error[160, 150, 0], error[160, 150, 1])
        self.assertEqual(tuple(image[100, 150]), (0, 255, 0))

    def test_latest_mailbox_is_bounded_and_preserves_integer_source_time(self):
        box = view.FrameMailbox()
        self.assertIsNone(box.snapshot())
        exact = 1792945307923871467
        box.accept(bytearray(b"first"), exact, 10.)
        box.accept(b"last", exact+1, 10.1)
        self.assertEqual(box.snapshot(), (2, b"last", exact+1, 10.1))

    def test_system_opencv_GTK_unsupported_visible_property_is_not_window_close(self):
        with patch.object(view.cv2, "getWindowProperty", side_effect=[-1., 0.]):
            self.assertFalse(view.window_closed())
        with patch.object(view.cv2, "getWindowProperty", side_effect=[-1., -1.]):
            self.assertTrue(view.window_closed())
        with patch.object(view.cv2, "getWindowProperty", return_value=0.):
            self.assertTrue(view.window_closed())

    def test_readonly_launch_and_source_never_start_detection_or_publish(self):
        from xml.etree import ElementTree as ET
        doc = ET.parse(PACKAGE / "launch/traffic_light_debug_view.launch")
        args = {x.attrib["name"]: x.attrib.get("default") for x in doc.findall("arg")}
        self.assertEqual(args["image_topic"], "/traffic_light_node/debug_image/compressed")
        self.assertIn("display", args)
        nodes = doc.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["name"], "traffic_light_debug_view")
        self.assertEqual(nodes[0].attrib["launch-prefix"], "/usr/bin/python3")
        tree = ast.parse((PACKAGE / "scripts/traffic_light_debug_view.py").read_text())
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        self.assertFalse(any(isinstance(n.func, ast.Attribute) and n.func.attr in ("Publisher", "ServiceProxy", "set_param") for n in calls))

    def test_mocked_viewer_cleans_up_only_its_subscriber_and_window(self):
        subscriber = Mock()
        rospy = types.SimpleNamespace(init_node=Mock(), get_param=lambda _key, default: default,
            Subscriber=Mock(return_value=subscriber), is_shutdown=Mock(side_effect=[False, True]),
            Time=types.SimpleNamespace(now=lambda: fixture.Stamp(10)), loginfo=Mock(), logerr=Mock(),
            logwarn_throttle=Mock(), signal_shutdown=Mock())
        sensor = types.ModuleType("sensor_msgs.msg")
        sensor.CompressedImage = object
        with patch.dict(sys.modules, {"rospy": rospy, "sensor_msgs.msg": sensor}), \
                patch.dict(os.environ, {"DISPLAY": ":99"}), \
                patch.object(view.cv2, "namedWindow"), patch.object(view.cv2, "resizeWindow"), \
                patch.object(view.cv2, "imshow"), patch.object(view.cv2, "waitKey", return_value=-1), \
                patch.object(view.cv2, "getWindowProperty", return_value=1), \
                patch.object(view.cv2, "destroyWindow") as destroy, patch.object(view.time, "sleep"):
            self.assertEqual(view.main(), 0)
        subscriber.unregister.assert_called_once()
        destroy.assert_called_once_with(view.WINDOW_TITLE)
        self.assertEqual(rospy.Subscriber.call_args.args[0], "/traffic_light_node/debug_image/compressed")


class NodeOverlayWiringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    def node(self, enabled):
        node = fixture.CandidateNodeTest.node(self, mode="legacy_center")
        node.publish_debug_image = enabled
        node.debug_pub = Mock() if enabled else None
        node.debug_jpeg_quality = 60
        node._debug_planner_parameters = Mock(return_value={})
        return node

    def image(self):
        return fixture.CandidateNodeTest.image(self)

    def test_debug_off_has_no_render_or_planner_config_query(self):
        node = self.node(False)
        with patch.object(self.module, "classify_crop", return_value=([("green", "circle", None)], {})), \
                patch.object(self.module, "render_node_overlay") as render:
            node.image_callback(self.image())
        render.assert_not_called()
        node._debug_planner_parameters.assert_not_called()
        node.pub.publish.assert_called_once()

    def test_overlay_runs_after_standard_output_and_uses_original_image_stamp(self):
        node, message = self.node(True), self.image()
        def draw(image, boxes, selected, crop, metadata):
            node.pub.publish.assert_called_once()
            node.runtime_debug_pub.publish.assert_called_once()
            self.assertEqual(metadata["source_stamp_ns"], message.header.stamp.to_nsec())
            self.assertEqual(metadata["observed_lamps"], "GC")
            return image.copy()
        with patch.object(self.module, "classify_crop", return_value=([("green", "circle", None)], {})), \
                patch.object(self.module, "render_node_overlay", side_effect=draw) as render:
            node.image_callback(message)
        render.assert_called_once()
        debug = node.debug_pub.publish.call_args.args[0]
        self.assertIs(debug.header, message.header)
        self.assertIs(debug.header.stamp, message.header.stamp)

    def test_broken_debug_renderer_cannot_suppress_standard_signal(self):
        node = self.node(True)
        with patch.object(self.module, "render_node_overlay", side_effect=ValueError("test rendering only")):
            node.image_callback(self.image())
        node.pub.publish.assert_called_once()
        node.runtime_debug_pub.publish.assert_called_once()
        node.debug_pub.publish.assert_not_called()

    def test_planner_settings_are_readonly_and_cached_for_two_seconds(self):
        node = object.__new__(self.module.TrafficLightNode)
        get = Mock(return_value=True)
        with patch.object(self.module.rospy, "get_param", get, create=True), \
                patch.object(self.module.time, "monotonic", side_effect=[10., 11., 12.]):
            self.module.TrafficLightNode._debug_planner_parameters(node)
            self.module.TrafficLightNode._debug_planner_parameters(node)
            self.assertEqual(get.call_count, 5)
            self.module.TrafficLightNode._debug_planner_parameters(node)
            self.assertEqual(get.call_count, 10)
        self.assertTrue(any(call.args[0].endswith("/traffic_light/enable_yellow_passthrough")
                            for call in get.call_args_list))


if __name__ == "__main__":
    unittest.main()
