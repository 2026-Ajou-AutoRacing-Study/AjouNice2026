"""Pooled MORAI trigger contacts must not hide a later real Ego contact."""
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from planning_scenario_validator.runtime import LiveMonitor


def contact(object_id=1, x=100.):
    return SimpleNamespace(collision_object=[SimpleNamespace(
        unique_id=object_id, position=SimpleNamespace(x=x, y=0.))])


class PrearmStaleContactsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.events = []
        m = self.monitor = LiveMonitor.__new__(LiveMonitor)
        m.ego = SimpleNamespace(position=SimpleNamespace(x=0., y=0.))
        m.armed = False
        m.events_path = Path(self.temp.name) / 'events.jsonl'
        m.collision_ids = set()
        m.seen_collision_ids = set()
        m.preparation_collisions = {}
        m.stamps = {}
        m.collision_count = 0
        m.far_collision = None
        m.evidence_lock = threading.Lock()
        m.event = lambda name, **fields: self.events.append((name, fields))

    def arm_after_far(self):
        with patch('planning_scenario_validator.runtime.time.monotonic', return_value=10.):
            self.monitor._collision(contact())
        with patch('planning_scenario_validator.runtime.time.monotonic', return_value=10.1):
            self.monitor.arm(self.monitor.events_path)

    def test_known_prearm_far_contact_is_recorded_once_without_aborting(self):
        self.arm_after_far()
        self.assertEqual(self.monitor.prearm_far_contact_ids, {1})
        self.monitor._collision(contact())
        self.monitor._collision(contact())
        self.assertIsNone(self.monitor.far_collision)
        self.assertEqual(self.monitor.collision_count, 0)
        self.assertEqual(self.monitor.seen_collision_ids, set())
        self.assertEqual(sum(n == 'COLLISION_FAR_FROM_EGO_IGNORED' for n, _ in self.events), 1)

    def test_same_pooled_id_still_counts_immediately_when_close(self):
        self.arm_after_far()
        self.monitor._collision(contact())
        self.monitor._collision(contact(x=5.))
        self.monitor._collision(contact(x=5.))
        self.assertEqual(self.monitor.collision_count, 1)
        self.assertIn(1, self.monitor.seen_collision_ids)

    def test_new_far_report_during_driving_is_ignored(self):
        self.arm_after_far()
        self.monitor._collision(contact(object_id=2))
        self.assertIsNone(self.monitor.far_collision)
        self.assertEqual(self.monitor.collision_count, 0)

    def test_unknown_far_then_close_is_not_hidden_by_seen_id(self):
        with patch('planning_scenario_validator.runtime.time.monotonic', return_value=10.):
            self.monitor.arm(self.monitor.events_path)
        self.monitor._collision(contact())
        self.monitor._collision(contact(x=5.))
        self.assertEqual(self.monitor.collision_count, 1)

    def test_old_or_no_longer_reported_far_contact_is_also_ignored(self):
        for removed in (False, True):
            with self.subTest(removed=removed):
                m = self.monitor
                m.armed = False
                with patch('planning_scenario_validator.runtime.time.monotonic', return_value=10.):
                    m._collision(contact())
                    if removed:
                        m._collision(SimpleNamespace(collision_object=[]))
                with patch('planning_scenario_validator.runtime.time.monotonic',
                           return_value=10.1 if removed else 12.):
                    m.arm(m.events_path)
                self.assertEqual(m.prearm_far_contact_ids, set())
                m._collision(contact())
                self.assertIsNone(m.far_collision)

    def test_close_prearm_contact_is_never_quarantined(self):
        with patch('planning_scenario_validator.runtime.time.monotonic', return_value=10.):
            self.monitor._collision(contact(x=5.))
            self.monitor.arm(self.monitor.events_path)
        self.assertEqual(self.monitor.prearm_far_contact_ids, set())
        self.monitor._collision(contact(x=5.))
        self.assertEqual(self.monitor.collision_count, 1)


    def test_ten_meter_boundary_is_ignored_but_just_inside_counts(self):
        self.monitor.arm(self.monitor.events_path)
        self.monitor._collision(contact(object_id=1, x=10.000001))
        self.assertEqual(self.monitor.collision_count, 0)
        self.assertNotIn(1, self.monitor.seen_collision_ids)
        self.monitor._collision(contact(object_id=1, x=10.))
        self.assertEqual(self.monitor.collision_count, 0)
        self.monitor._collision(contact(object_id=1, x=9.999999))
        self.assertEqual(self.monitor.collision_count, 1)
        self.assertIsNone(self.monitor.far_collision)

    def test_mixed_near_and_far_packet_keeps_near_collision(self):
        self.monitor.arm(self.monitor.events_path)
        far = contact(object_id=1).collision_object[0]
        near = contact(object_id=2, x=5.).collision_object[0]
        self.monitor._collision(SimpleNamespace(collision_object=[far, near]))
        self.assertEqual(self.monitor.collision_count, 1)
        self.assertEqual(self.monitor.seen_collision_ids, {2})
        self.assertIsNone(self.monitor.far_collision)

    def test_ignored_report_does_not_abort_run_loop(self):
        self.monitor.arm(self.monitor.events_path)
        self.monitor._collision(contact())
        self.monitor.rospy = SimpleNamespace(is_shutdown=lambda: False)
        reason, _ = self.monitor.run_until_done(
            SimpleNamespace(timeout_sec=0.), {}, started=0.)
        self.assertEqual(reason, 'TIMEOUT')

    def test_ignore_log_is_once_per_id_per_case(self):
        self.monitor.arm(self.monitor.events_path)
        self.monitor._collision(contact())
        self.monitor._collision(contact())
        self.assertEqual(sum(n == 'COLLISION_FAR_FROM_EGO_IGNORED'
                             for n, _ in self.events), 1)
        self.monitor.arm(self.monitor.events_path)
        self.monitor._collision(contact())
        self.assertEqual(sum(n == 'COLLISION_FAR_FROM_EGO_IGNORED'
                             for n, _ in self.events), 2)


if __name__ == '__main__':
    unittest.main()
