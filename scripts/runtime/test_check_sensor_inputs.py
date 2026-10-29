import unittest
from unittest.mock import patch
from types import SimpleNamespace
import check_sensor_inputs as check


class InputCheckTests(unittest.TestCase):
    def test_missing(self):
        self.assertEqual(check.evaluate([], 5, 30)[0], 'FAIL')

    def test_old_or_latched_single(self):
        self.assertEqual(check.evaluate([1], 5, 30)[0], 'FAIL')
        self.assertEqual(check.evaluate([1, 2], 5, 30)[0], 'FAIL')

    def test_stationary_receiving(self):
        # Counts/freshness do not depend on vehicle moving or D gear.
        self.assertEqual(check.evaluate([4.90, 4.95, 5], 5, 30)[0], 'PASS')

    def test_low_rate_warns(self):
        self.assertEqual(check.evaluate([4, 4.5, 5], 5, 30)[0], 'WARN')

    def test_type_mismatch(self):
        self.assertEqual(check.evaluate([4.9, 5], 5, 30, type_ok=False)[0], 'FAIL')

    def test_shadow_not_disconnection(self):
        self.assertEqual(check.evaluate([4.9, 5], 5, 15, invalid=True)[0], 'WARN')
        self.assertTrue(check.gps_invalid(SimpleNamespace(latitude=0, longitude=0)))
        self.assertFalse(check.gps_invalid(SimpleNamespace(latitude=37.2, longitude=126.8)))
        self.assertTrue(check.gps_invalid(SimpleNamespace(latitude=float('nan'), longitude=126.8)))

    def test_no_gt_or_collision_requirement(self):
        topics = [spec[1] for specs in check.TOPICS.values() for spec in specs]
        self.assertNotIn('/Ego_topic', topics)
        self.assertNotIn('/Object_topic', topics)
        self.assertNotIn('/CollisionData', topics)
        self.assertEqual(len(topics), 8)

    @patch.object(check.subprocess, 'run')
    def test_network_profile_explicit(self, run):
        run.return_value = SimpleNamespace(stdout='[{"addr_info":[{"family":"inet","local":"10.42.0.1"}]}]')
        self.assertEqual(check.network_rows('teamlead', lambda *args: '')[0][0], 'PASS')
        self.assertEqual(check.network_rows('competition', lambda *args: '')[0][0], 'FAIL')


if __name__=='__main__':
    unittest.main()
