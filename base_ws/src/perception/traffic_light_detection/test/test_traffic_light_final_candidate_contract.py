#!/usr/bin/env python3
"""Final wiring regressions with fake ROS/YOLO only; no model forward or master.

These check software contracts, not visual identity, model accuracy or driving
safety. Existing callback fixtures intentionally replace both ROS and inference.
"""
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import test_traffic_light_candidate_node as fixture


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from traffic_light_cnn_classifier import observation_from_probabilities
from traffic_light_map_selector import MISSION_GROUP_IDS, load_geometry


class LaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch = ET.parse(PACKAGE / "launch/traffic_light_node.launch").getroot()
        cls.args = {item.get("name"): item.get("default") for item in cls.launch.findall("arg")}

    def test_production_defaults_remain_legacy_hsv_640_and_existing_cadence(self):
        expected = {"selection_mode": "legacy_center", "classifier_mode": "hsv",
                    "detector_imgsz": "640", "conf": "0.25", "max_rate_hz": "10.0",
                    "smoothing_window_size": "7", "smoothing_majority_count": "4",
                    "cnn_checkpoint_path": "",
                    "output_topic": "/perception/traffic_light_recognition/traffic_signals"}
        for key, value in expected.items():
            with self.subTest(parameter=key):
                self.assertEqual(self.args[key], value)
        self.assertTrue(self.args["weights_path"].endswith("/weights/best.pt"))

    def test_map_helper_and_route_allowlist_are_opt_in_and_system_python(self):
        helper_group = next(group for group in self.launch.findall("group")
                            if any(node.get("type") == "traffic_light_map_roi_node.py"
                                   for node in group.findall("node")))
        self.assertEqual(helper_group.get("if"), "$(eval arg('selection_mode') == 'map_association')")
        helper = helper_group.find("node")
        self.assertEqual(helper.get("launch-prefix"), "/usr/bin/python3")
        route = next(node for node in self.launch.iter("node")
                     if node.get("type") == "traffic_light_route_group_id_node.py")
        self.assertEqual(route.get("launch-prefix"), "/usr/bin/python3")
        allowlist = next(item for item in route.findall("rosparam") if item.get("param") == "allowed_group_ids")
        self.assertEqual(allowlist.get("if"), helper_group.get("if"))
        self.assertEqual(set(json.loads(allowlist.text)), MISSION_GROUP_IDS)
        self.assertNotIn(201397, json.loads(allowlist.text))

    def test_inference_interpreter_and_output_parameters_are_forwarded(self):
        inference = next(node for node in self.launch.findall("node")
                         if node.get("type") == "traffic_light_node.py")
        self.assertEqual(inference.get("launch-prefix"), "$(arg inference_python)")
        params = {item.get("name"): item.get("value") for item in inference.findall("param")}
        for name in ("output_topic", "detector_imgsz", "selection_mode", "classifier_mode",
                     "cnn_checkpoint_path", "cnn_device", "cnn_crop_margin_px", "map_roi_topic"):
            self.assertEqual(params[name], "$(arg " + name + ")")
        self.assertEqual(params["enable_route_matching"], "false")

    def test_installer_includes_every_candidate_runtime_module_and_geometry(self):
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        for name in ("traffic_light_node.py", "traffic_light_route_group_id_node.py",
                     "traffic_light_map_selector.py", "traffic_light_map_roi_node.py",
                     "traffic_light_cnn_classifier.py", "signal_phase_policy.py", "signal_smoother.py"):
            self.assertIn("scripts/" + name, cmake)
        self.assertIn("install(DIRECTORY config launch weights", cmake)
        self.assertIn("<depend>tf2_ros</depend>", (PACKAGE / "package.xml").read_text())

    def test_checked_geometry_promotes_exactly_the_five_mission_heads(self):
        geometry = load_geometry(PACKAGE / "config/traffic_light_map_geometry.json")
        promoted = {head["id"]: head["group_id"] for head in geometry["heads"]
                    if head.get("group_id") is not None}
        self.assertEqual(promoted, {"C1256W000077": 201351, "C1256W000091": 201365,
                                    "C1256W000016": 201290, "C1256W000018": 201290,
                                    "C1256W000052": 201326})
        self.assertEqual(geometry["geometry_frame"], "morai_map")


class CallbackBoundaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = fixture.load_node()

    def node(self, **kwargs):
        return fixture.CandidateNodeTest.node(self, **kwargs)

    def image(self):
        return fixture.CandidateNodeTest.image(self)

    def packet(self):
        return fixture.CandidateNodeTest.packet(self)

    def debug(self, node, publisher):
        return fixture.CandidateNodeTest.debug(self, node, publisher)

    def test_standalone_hsv_startup_does_not_initialize_or_forward_cnn(self):
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / "fake.pt"
            weights.write_bytes(b"fake fixture; never passed to real YOLO")
            params = {"~weights_path": str(weights), "~signal_profile_path":
                      str(PACKAGE / "config/traffic_signal_profiles.yaml")}
            model = types.SimpleNamespace(device="cpu", predict=Mock())
            fake_cnn = types.ModuleType("traffic_light_cnn_classifier")
            fake_cnn.initialize_classifier = Mock(side_effect=AssertionError("HSV startup imported CNN"))
            with patch.object(self.module.rospy, "get_param", side_effect=lambda key, default=None:
                              params.get(key, default), create=True), \
                    patch.object(self.module.rospy, "Publisher", side_effect=lambda *a, **k: Mock(), create=True), \
                    patch.object(self.module.rospy, "Subscriber", create=True), \
                    patch.object(self.module, "YOLO", return_value=model), \
                    patch.dict(sys.modules, {"traffic_light_cnn_classifier": fake_cnn}):
                node = self.module.TrafficLightNode()
            self.assertEqual((node.selection_mode, node.classifier_mode), ("legacy_center", "hsv"))
            self.assertIsNone(node._cnn_startup)
            self.assertIsNone(node.sub_map_rois)
            fake_cnn.initialize_classifier.assert_not_called()
            model.predict.assert_not_called()

    def test_current_route_transition_resets_previous_head_and_temporal_state(self):
        node = self.node()
        node._last_policy_group_id, node._selected_head_id = 201351, "C1256W000077"
        node.image_callback(self.image())
        node.signal_smoother.reset.assert_called_once()
        node.signal_phase_policy.reset_transition.assert_called_once()
        self.assertIsNone(node._selected_head_id)
        self.assertEqual(node._last_policy_group_id, 201290)

    def test_route_zero_or_stale_never_uses_fallback_group_in_map_mode(self):
        node = self.node()
        node._resolve_group_id = types.MethodType(self.module.TrafficLightNode._resolve_group_id, node)
        node.route_group_id_stale_sec, node.fallback_group_id = 2., 201351
        node.route_group_id_callback(fixture.Message(201290))
        self.assertEqual(node._resolve_group_id(), 201290)
        node.route_group_id_callback(fixture.Message(0))
        self.assertEqual(node._resolve_group_id(), 0)
        node._external_route_group_id, node._external_route_group_id_stamp = 201290, fixture.Stamp(7.)
        self.assertEqual(node._resolve_group_id(), 0)
        node.selection_mode = "legacy_center"
        self.assertEqual(node._resolve_group_id(), 201351)

    def test_real_adapter_all_zero_vs_uncertain_survives_node_diagnostics(self):
        for probabilities, known, kind in (([.01] * 4, True, "no_lamps"),
                                            ([.5] * 4, False, "unknown")):
            with self.subTest(kind=kind):
                node = self.node(classifier="cnn")
                node._map_packet_for_image.return_value = self.packet()
                observation = observation_from_probabilities(probabilities)
                classifier = types.SimpleNamespace(classify_bgr=Mock(return_value=observation))
                node._cnn_startup = types.SimpleNamespace(ready=True, classifier=classifier)
                with patch.object(self.module, "classify_crop") as hsv:
                    node.image_callback(self.image())
                hsv.assert_not_called()
                debug = self.debug(node, "classifier_debug_pub")
                self.assertEqual(debug["known"], known)
                self.assertEqual(debug["empty_result_kind"], kind)
                self.assertEqual(node.signal_phase_policy.apply.call_args[0][0], [])
                node.pub.publish.assert_called_once()

    def test_invalid_roi_configuration_produces_missing_without_hsv_or_abort(self):
        for invalid in ({"unknown_parameter": 1}, {"max_padding_px": float("nan")},
                        {"min_aspect_ratio_factor": True}):
            with self.subTest(invalid=invalid):
                node = self.node()
                packet = self.packet()
                packet["selector_config"] = invalid
                node._map_packet_for_image.return_value = packet
                with patch.object(self.module, "classify_crop") as hsv:
                    node.image_callback(self.image())
                hsv.assert_not_called()
                self.assertEqual(self.debug(node, "selection_debug_pub")["status"], "missing")
                node.pub.publish.assert_called_once()


if __name__ == "__main__":
    unittest.main()
