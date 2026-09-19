#!/usr/bin/env python3
import unittest

from competition_rule_monitor.rules import OrderedCheckpoints, PeriodicPenalty, is_speeding


class RulesTest(unittest.TestCase):
    def test_lane_seven_seconds_is_two_penalties(self):
        rule = PeriodicPenalty(3.0, 5)
        self.assertEqual(rule.begin(10.0), [])
        self.assertEqual(rule.update(12.99), [])
        self.assertEqual(rule.update(13.0), [(3.0, 5)])
        self.assertEqual(rule.update(16.0), [(6.0, 5)])
        total = rule.end(17.0)
        self.assertEqual(total["penalties"], 2)
        self.assertEqual(total["penalty_seconds"], 10)

    def test_speed_is_immediate_then_periodic(self):
        rule = PeriodicPenalty(3.0, 15, immediate=True)
        self.assertEqual(rule.begin(2.0), [(0.0, 15)])
        self.assertEqual(rule.update(4.99), [])
        self.assertEqual(rule.update(5.0), [(3.0, 15)])

    def test_checkpoints_are_ordered(self):
        checkpoints = OrderedCheckpoints(2)
        self.assertFalse(checkpoints.observe(2, True))
        self.assertTrue(checkpoints.observe(1, True))
        self.assertTrue(checkpoints.observe(2, True))
        self.assertIsNone(checkpoints.next_number)

    def test_speed_threshold_is_inclusive_except_in_official_zone(self):
        self.assertFalse(is_speeding(59.9999, 60.0))
        self.assertTrue(is_speeding(60.0, 60.0))
        self.assertFalse(is_speeding(80.0, 60.0, in_exception=True))


if __name__ == "__main__":
    unittest.main()
