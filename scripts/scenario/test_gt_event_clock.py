import unittest
from gt_event_clock import ObjectClockOverlaps


class Tests(unittest.TestCase):
    def test_object_event_before_latch_not_later_ego_event(self):
        stats = ObjectClockOverlaps(16.137594)
        for stamp, area, age in [(16.101412, .00063, .001004),
                                  (16.129085, .01603, .000573),
                                  (16.129649, .01603, .001138)]:
            self.assertTrue(stats.record(38, stamp, stamp, age, area))
        self.assertEqual(stats.by_phase['before_intervention'][38]['samples'], 3)
        self.assertEqual(stats.by_phase['after_intervention'], {})

    def test_boundary_and_totals_not_hidden(self):
        stats = ObjectClockOverlaps(10)
        stats.record(1, 9.9, 1, 0, .1)
        stats.record(1, 10, 1.1, .01, .2)
        self.assertEqual(stats.all[1]['samples'], 2)
        self.assertEqual(stats.by_phase['after_intervention'][1]['samples'], 1)

    def test_unknown_not_assumed_pre(self):
        stats = ObjectClockOverlaps()
        stats.record(1, 1, 0, .1, .1)
        self.assertEqual(stats.by_phase['before_intervention'], {})
        self.assertEqual(stats.by_phase['unknown_boundary'][1]['samples'], 1)

    def test_bad_age_or_area_rejected(self):
        stats = ObjectClockOverlaps()
        for age, area in [(-.001, .1), (.2, .1), (0, -1),
                          (float('nan'), .1), (0, float('inf'))]:
            self.assertFalse(stats.record(1, 1, 0, age, area))
        self.assertEqual(stats.all, {})

    def test_callback_duplicates_are_explicit_not_distinct_frame_claim(self):
        stats = ObjectClockOverlaps()
        for _ in range(2):
            stats.record(1, 1, 0, 0, 0)
        self.assertEqual(stats.all[1]['samples'], 2)


if __name__ == '__main__':
    unittest.main()
