#!/usr/bin/env python3
import unittest

from replay_tracker_shape_guard_ros import fingerprint, grid, stats


class DeterministicClockTests(unittest.TestCase):
    def test_grid_preserves_input_order_and_separate_current_clock(self):
        frames = [dict(stamp=1792781503.25 + i * .0667) for i in range(50)]
        schedule = grid(frames)
        seen = [(i, now) for now, indices in schedule for i in indices]
        self.assertEqual([i for i, _ in seen], list(range(50)))
        self.assertTrue(all(.0199 <= now-frames[i]['stamp'] < .0534 for i, now in seen))
        self.assertEqual(schedule, grid(frames))

    def test_refuses_latest_packet_overwrite(self):
        with self.assertRaisesRegex(ValueError, 'More than one input'):
            grid([dict(stamp=100.), dict(stamp=100.001), dict(stamp=100.002)])

    def test_fingerprint_is_not_dictionary_insertion_order_dependent(self):
        self.assertEqual(fingerprint(dict(a=1,b=2)), fingerprint(dict(b=2,a=1)))

    def test_receipt_grid_retains_detector_age(self):
        frames = [dict(stamp=100.+i*.1, receipt=100.12+i*.1) for i in range(10)]
        schedule = grid(frames, clock_mode='receipt')
        arrivals = [(i, now) for now, indices in schedule for i in indices]
        self.assertEqual([i for i, _ in arrivals], list(range(10)))
        self.assertTrue(all(0 <= now-frames[i]['receipt'] < 1./30.+1.e-6 for i, now in arrivals))
        self.assertTrue(all(now-frames[i]['stamp'] >= .119999 for i, now in arrivals))

    def test_nonfinite_measurement_rejected(self):
        with self.assertRaises(ValueError):
            stats([float('nan')])


if __name__ == '__main__':
    unittest.main()
