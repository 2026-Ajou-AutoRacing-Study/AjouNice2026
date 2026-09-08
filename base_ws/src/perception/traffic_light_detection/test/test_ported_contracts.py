#!/usr/bin/env python3

import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from autoware_planning_msgs.msg import RouteState
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


if __name__ == "__main__":
    unittest.main()
