#!/usr/bin/env python3

import io
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from autoware_planning_msgs.msg import RouteState
import lanelet_route_matcher as route_matcher_module
from lanelet_route_matcher import LaneletRouteGroupIdMatcher
from ros_time import builtin_time_from_ros


class PortedContractsTest(unittest.TestCase):
    def test_builtin_stamp_is_serializable(self):
        stamp = builtin_time_from_ros(SimpleNamespace(secs=12, nsecs=345))
        self.assertEqual(stamp.sec, 12)
        self.assertEqual(stamp.nanosec, 345)
        stamp.serialize(io.BytesIO())

    def test_unset_route_state_invalidates_latched_route(self):
        matcher = object.__new__(LaneletRouteGroupIdMatcher)
        matcher._route_active = False
        matcher._route_segments = [[101], [102]]

        matcher._route_state_callback(SimpleNamespace(state=RouteState.SET))
        self.assertTrue(matcher._route_active)
        self.assertEqual(matcher._route_segments, [[101], [102]])

        matcher._route_state_callback(SimpleNamespace(state=RouteState.UNSET))
        self.assertFalse(matcher._route_active)
        self.assertIsNone(matcher._route_segments)

    def test_robust_loader_tolerates_only_declared_roundabout_errors(self):
        osm = """<osm version="0.6">
          <relation id="201455">
            <tag k="type" v="regulatory_element"/>
            <tag k="subtype" v="roundabout"/>
          </relation>
        </osm>"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".osm") as stream:
            stream.write(osm)
            stream.flush()
            lanelet_map = SimpleNamespace(laneletLayer=[1, 2, 3])
            roundabout_errors = [
                "Errors ocurred while parsing Lanelet Map:",
                "Error parsing primitive 201455: unsupported roundabout",
                "Error parsing primitive 200480: Failed to get id 201455 from map",
            ]
            with patch.object(
                route_matcher_module.io,
                "loadRobust",
                return_value=(lanelet_map, roundabout_errors),
            ):
                loaded = route_matcher_module._load_lanelet_map_for_traffic_matching(
                    stream.name, object()
                )
            self.assertIs(loaded, lanelet_map)

            unrelated_errors = roundabout_errors + ["Error parsing primitive 999999: bad way"]
            with patch.object(
                route_matcher_module.io,
                "loadRobust",
                return_value=(lanelet_map, unrelated_errors),
            ):
                with self.assertRaises(RuntimeError):
                    route_matcher_module._load_lanelet_map_for_traffic_matching(
                        stream.name, object()
                    )


if __name__ == "__main__":
    unittest.main()
