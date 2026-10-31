#!/usr/bin/env python3
"""Exercise the read-only vehicle HUD without ROS, a master, or MORAI."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


SCRIPT = (Path(__file__).resolve().parents[1] / "validation" / "scripts" /
          "planning_vehicle_overlay.py")


class FakeOverlayText:
    LEFT = 0
    BOTTOM = 2


class FakeColorRGBA:
    def __init__(self, r, g, b, a):
        self.r, self.g, self.b, self.a = r, g, b, a


class VehicleOverlayTest(unittest.TestCase):
    def load_overlay(self, parameters=None):
        parameters = {} if parameters is None else parameters
        self.clock = 10.0
        self.publisher = Mock()
        rospy = ModuleType("rospy")
        rospy.get_param = Mock(side_effect=lambda key, default: parameters.get(key, default))
        rospy.Publisher = Mock(return_value=self.publisher)
        rospy.Subscriber = Mock()
        rospy.Duration = Mock(side_effect=lambda seconds: seconds)
        rospy.Timer = Mock()

        messages = {}
        for package, names in (
            ("morai_msgs", ("CtrlCmd", "EgoVehicleStatus")),
            ("rviz_2d_overlay_msgs", ("OverlayText",)),
            ("std_msgs", ("ColorRGBA",)),
        ):
            parent = ModuleType(package)
            child = ModuleType(package + ".msg")
            for name in names:
                cls = {"OverlayText": FakeOverlayText,
                       "ColorRGBA": FakeColorRGBA}.get(name, type(name, (), {}))
                setattr(child, name, cls)
            parent.msg = child
            messages[package], messages[package + ".msg"] = parent, child
        messages["rospy"] = rospy

        with patch.dict(sys.modules, messages):
            spec = importlib.util.spec_from_file_location("planning_vehicle_overlay_under_test", SCRIPT)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
        module.time = SimpleNamespace(monotonic=lambda: self.clock)
        self.module, self.rospy = module, rospy
        return module.VehicleOverlay()

    @staticmethod
    def status(vx, vy=0.0):
        return SimpleNamespace(velocity=SimpleNamespace(x=vx, y=vy))

    def latest_lines(self, overlay):
        overlay.publish(None)
        return dict((line[:10].strip(), line[10:])
                    for line in self.publisher.publish.call_args.args[0].text.splitlines())

    def test_default_status_subscription_uses_competition_not_ego(self):
        overlay = self.load_overlay()
        self.rospy.get_param.assert_called_with("~vehicle_status_topic", "/Competition_topic")
        subscriptions = self.rospy.Subscriber.call_args_list
        self.assertEqual([call.args[0] for call in subscriptions],
                         ["/Competition_topic", "/ctrl_cmd"])
        self.assertIs(subscriptions[0].args[1], self.module.EgoVehicleStatus)
        self.assertEqual(subscriptions[0].args[2], overlay.on_status)
        self.assertEqual(subscriptions[1].args[2], overlay.on_command)
        self.rospy.Timer.assert_called_once_with(0.1, overlay.publish)

    def test_explicit_status_topic_override_does_not_add_ego_subscription(self):
        overlay = self.load_overlay({"~vehicle_status_topic": "/custom/vehicle_status"})
        self.rospy.Subscriber.assert_any_call(
            "/custom/vehicle_status", self.module.EgoVehicleStatus,
            overlay.on_status, queue_size=1)
        self.assertNotIn("/Ego_topic", [call.args[0]
                                        for call in self.rospy.Subscriber.call_args_list])

    def test_bridge_meters_per_second_are_converted_to_kph_once(self):
        overlay = self.load_overlay()
        overlay.on_status(self.status(10.0))
        self.assertAlmostEqual(overlay.speed, 36.0)
        self.assertEqual(self.latest_lines(overlay)["Speed"], "36.0 km/h")

    def test_stale_status_displays_unknown_instead_of_cached_speed(self):
        overlay = self.load_overlay()
        overlay.on_status(self.status(10.0))
        self.clock += 1.0
        self.assertEqual(self.latest_lines(overlay)["Speed"], "-- km/h")

    def test_command_units_and_freshness_are_unchanged(self):
        overlay = self.load_overlay()
        overlay.on_command(SimpleNamespace(steering=math.radians(10.0), accel=0.4, brake=0.1))
        values = self.latest_lines(overlay)
        self.assertEqual(values["Steer"], "+10.0 °")
        self.assertEqual(values["Accel cmd"], "0.400")
        self.assertEqual(values["Brake cmd"], "0.100")
        self.clock += 1.0
        overlay.on_status(self.status(10.0))
        values = self.latest_lines(overlay)
        self.assertEqual(values["Speed"], "36.0 km/h")
        self.assertEqual(values["Steer"], "-- °")
        self.assertEqual(values["Accel cmd"], "--")
        self.assertEqual(values["Brake cmd"], "--")

    def test_missing_inputs_do_not_prevent_overlay_publication(self):
        overlay = self.load_overlay()
        values = self.latest_lines(overlay)
        self.assertEqual(values, {"Speed": "-- km/h", "Steer": "-- °",
                                  "Accel cmd": "--", "Brake cmd": "--"})


if __name__ == "__main__":
    unittest.main()
