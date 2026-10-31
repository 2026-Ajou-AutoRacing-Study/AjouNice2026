import math
from types import SimpleNamespace
import unittest

import observe_traffic_light_runtime_timing as timing


def message(seconds=10, nanoseconds=0):
    return SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(secs=seconds, nsecs=nanoseconds)))


class RuntimeTimingTest(unittest.TestCase):
    def test_quantiles_interpolate_and_drop_nonfinite(self):
        stats = timing.quantiles([0., 10., 20., 30., float('nan'), float('inf')])
        self.assertEqual(stats['n'], 4)
        self.assertEqual(stats['p50'], 15.)
        self.assertEqual(stats['p90'], 27.)
        self.assertEqual(stats['mean'], 15.)
        self.assertIsNone(timing.quantiles([]))

    def test_zero_invalid_and_missing_headers_are_not_zero_latency(self):
        stats = timing.TopicStats()
        for index, msg in enumerate([message(0), message(-1), message(1, 10**9), SimpleNamespace()]):
            stats.observe(msg, 11., index)
        snapshot = stats.snapshot(5.)
        self.assertIsNone(snapshot['age_ms'])
        self.assertEqual(snapshot['counts']['invalid_source_stamp'], 3)
        self.assertEqual(snapshot['counts']['header_absent'], 1)

    def test_ctrl_cmd_has_no_header_and_does_not_claim_latency(self):
        stats = timing.TopicStats(expect_stamp=False)
        stats.observe(SimpleNamespace(), 100., 1.)
        snapshot = stats.snapshot(2.)
        self.assertEqual(snapshot['age_kind'], 'unavailable_no_header_not_latency')
        self.assertEqual(snapshot['counts']['unstamped_by_contract'], 1)
        self.assertNotIn('invalid_source_stamp', snapshot['counts'])
        self.assertIsNone(snapshot['age_ms'])

    def test_high_hz_callbacks_are_all_counted_not_latest_only_sampled(self):
        stats = timing.TopicStats(max_samples=25)
        for index in range(101):
            stats.observe(message(), 10.025, index * .01)
        snapshot = stats.snapshot(1.1)
        self.assertEqual(snapshot['counts']['messages'], 101)
        self.assertEqual(snapshot['counts']['valid_age_samples'], 101)
        self.assertEqual(snapshot['receipt_interval_ms']['n'], 25)
        self.assertAlmostEqual(snapshot['receipt_interval_ms']['mean'], 10.)
        self.assertAlmostEqual(snapshot['age_ms']['mean'], 25.)
        self.assertAlmostEqual(snapshot['observed_hz'], 100.)

    def test_future_and_invalid_receipt_are_explicit(self):
        stats = timing.TopicStats()
        stats.observe(message(), 9., 1.)
        stats.observe(message(), math.nan, 2.)
        self.assertEqual(stats.counts['future_source_stamp'], 1)
        self.assertEqual(stats.counts['invalid_receipt_ros_time'], 1)
        self.assertIsNone(stats.snapshot(3.)['age_ms'])

    def test_top_level_stamp_supported_without_synthetic_header(self):
        stamp = SimpleNamespace(stamp=SimpleNamespace(secs=10, nsecs=500000000))
        self.assertEqual(timing.source_seconds(stamp), (10.5, None))

    def test_compat_builtin_time_uses_sec_nanosec_fields(self):
        stamp = SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=250000000))
        stats = timing.TopicStats()
        stats.observe(stamp, 10.3, 1.)
        self.assertEqual(timing.source_seconds(stamp), (10.25, None))
        self.assertAlmostEqual(stats.snapshot(1.)['age_ms']['p95'], 50.)

    def test_concurrent_receipt_order_inversion_is_counted_not_negative_interval(self):
        stats = timing.TopicStats()
        for receipt in (1., 2., 1.9, 3.):
            stats.observe(message(), 10.1, receipt)
        snapshot = stats.snapshot(3.)
        self.assertEqual(snapshot['counts']['receipt_order_inversion'], 1)
        self.assertEqual(snapshot['receipt_interval_ms']['n'], 2)
        self.assertEqual(snapshot['receipt_interval_ms']['min'], 1000.)

    def test_duration_bounds_are_checked_before_ros_import_or_file_creation(self):
        for duration in ('0', '-1', '1801', 'nan', 'inf'):
            with self.subTest(duration=duration), self.assertRaises(SystemExit) as error:
                timing.main(['--duration-sec', duration])
            self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
