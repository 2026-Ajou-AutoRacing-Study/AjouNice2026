"""순수 테스트: 실제 RPC·ROS·시뮬레이터·대기 실행 없음."""

import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
SPEC = importlib.util.spec_from_file_location("cycle_observer", Path(__file__).with_name("observe_morai_traffic_light_cycles.py"))
observer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(observer)


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


def payload(unique_id="C1256W000077", color=1):
    identifier = unique_id.encode()
    nested = b"\x0a" + bytes((len(identifier),)) + identifier
    return b"\x12" + bytes((len(nested),)) + nested + b"\x18" + bytes((color,))


def sample(stamp, color, unique_id="x", scan=0):
    return {"requested_id": unique_id, "native_id": unique_id, "color": color,
            "color_name": observer.native.COLORS[color], "response_monotonic": stamp,
            "scan_index": scan, "event": "observation", "readback_success": True,
            "read_duration_sec": .01, "schedule_lateness_sec": .02}


class ObserverTests(unittest.TestCase):
    def test_only_modern_get_path_and_default_ids(self):
        clock, calls = Clock(), []
        def rpc(path, request, timeout):
            calls.append((path, request, timeout))
            return payload()
        rows, run = observer.observe(rpc_call=rpc, seconds=.41, hz=5,
                                     ids=["C1256W000077"], monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(len(rows), 3)
        self.assertTrue(all(path == observer.READ_PATH for path, _, _ in calls))
        self.assertEqual(rows[0]["response_hex"], payload().hex())
        self.assertTrue(all(row["readback_success"] for row in rows))
        self.assertEqual(run["max_attempts_per_id_per_scan"], 2)
        self.assertEqual(observer.DEFAULT_IDS[-2:], ("C1256W000016", "C1256W000052"))

    def test_failed_reads_are_logged_and_retry_is_bounded(self):
        clock, calls = Clock(), []
        def rpc(path, request, timeout):
            calls.append(path)
            raise RuntimeError("가짜 수신 오류")
        rows, _ = observer.observe(rpc_call=rpc, seconds=.1, hz=5,
                                   ids=["C1256W000077"], monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertEqual(len(calls), 2)
        self.assertEqual([row["attempt"] for row in rows], [1, 2])
        self.assertTrue(all(row["event"] == "error" for row in rows))
        report = observer.summarize(rows, ["C1256W000077"])
        self.assertEqual(report["per_id"]["C1256W000077"]["phases"], [])

    def test_wrong_native_identity_never_becomes_timeline_success(self):
        clock = Clock()
        rows, _ = observer.observe(rpc_call=lambda *args: payload("other"), seconds=.1, hz=5,
                                   ids=["C1256W000077"], monotonic=clock.monotonic, sleep=clock.sleep)
        self.assertTrue(all(row["event"] == "invalid_readback" for row in rows))
        self.assertEqual(observer.summarize(rows, ["C1256W000077"])["per_id"]["C1256W000077"]["successful_samples"], 0)

    def test_interrupted_read_has_event_and_can_be_summarized(self):
        clock, emitted = Clock(), []
        def interrupted(*args):
            raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            observer.observe(rpc_call=interrupted, seconds=.1, ids=["C1256W000077"],
                             monotonic=clock.monotonic, sleep=clock.sleep, emit=emitted.append)
        self.assertEqual(emitted[0]["event"], "interrupted")
        summary = observer.summarize(emitted, ["C1256W000077"])
        self.assertEqual(summary["per_id"]["C1256W000077"]["attempt_event_counts"], {"interrupted": 1})

    def test_first_and_last_phase_censored_internal_duration_has_bounds(self):
        phases, gaps = observer.timeline([sample(0, 1), sample(.2, 1), sample(.4, 16),
                                          sample(.6, 16), sample(.8, 4)], .3)
        self.assertFalse(gaps)
        self.assertTrue(phases[0]["left_censored"])
        self.assertTrue(phases[-1]["right_censored"])
        self.assertIsNone(phases[0]["duration_estimate_sec"])
        self.assertAlmostEqual(phases[1]["duration_estimate_sec"], .4)
        self.assertAlmostEqual(phases[1]["complete_duration_bounds_sec"][0], .2)
        self.assertAlmostEqual(phases[1]["complete_duration_bounds_sec"][1], .6)

    def test_large_gap_breaks_duration_and_cycle_chains(self):
        phases, gaps = observer.timeline([sample(0, 1), sample(.2, 16), sample(2, 4), sample(2.2, 1)], .5)
        self.assertEqual(len(gaps), 1)
        self.assertTrue(phases[1]["right_censored"])
        self.assertTrue(phases[2]["left_censored"])
        self.assertIsNone(phases[1]["duration_estimate_sec"])
        self.assertEqual(observer.cycle_estimates(phases), [])

    def test_boundary_includes_previous_rpc_read_window_not_only_receipt(self):
        previous, following = sample(1.0, 1), sample(1.2, 16)
        previous["request_monotonic"] = .7
        phases, _ = observer.timeline([previous, following], .6)
        self.assertEqual(phases[1]["start_boundary"]["after_monotonic"], .7)
        self.assertEqual(phases[1]["start_boundary"]["by_monotonic"], 1.2)

    def test_one_cycle_or_two_entries_not_enough_to_claim_period(self):
        phases, _ = observer.timeline([sample(i, color) for i, color in enumerate([16, 1, 16, 1])], 2)
        self.assertEqual(observer.cycle_estimates(phases), [])
        report = observer.summarize([sample(0, 1)], ["x"])
        self.assertIsNone(report["assumed_common_period_sec"])

    def test_two_complete_matching_cycles_need_three_entry_boundaries(self):
        phases, _ = observer.timeline([sample(i, color) for i, color in enumerate([16, 1, 16, 1, 16, 1])], 2)
        estimates = observer.cycle_estimates(phases)
        self.assertEqual(len(estimates), 1)
        self.assertEqual(estimates[0]["complete_repetitions"], 2)
        self.assertEqual(estimates[0]["ordered_color_sequence"], [1, 16])
        self.assertEqual(estimates[0]["period_estimates_sec"], [2.0, 2.0])

    def test_changed_order_does_not_make_cycle_estimate(self):
        phases, _ = observer.timeline([sample(i, color) for i, color in enumerate([4, 1, 16, 1, 4, 16, 1])], 2)
        self.assertEqual(observer.cycle_estimates(phases), [])

    def test_sampling_jitter_skew_and_frequency_are_reported(self):
        rows = [sample(0, 1, "a", 0), sample(.03, 4, "b", 0),
                sample(.22, 1, "a", 1), sample(.26, 16, "b", 1)]
        report = observer.summarize(rows, ["a", "b"], hz=5)
        self.assertAlmostEqual(report["inter_id_response_skew_sec"]["max"], .04)
        self.assertEqual(report["per_id"]["a"]["sample_color_frequency_counts"], {"1": 2})
        self.assertAlmostEqual(report["per_id"]["a"]["interval_jitter_from_requested_period_sec"]["mean"], .02)

    def test_invalid_configuration_and_nonmonotonic_receipts(self):
        for seconds, hz, ids in [(float("nan"), 5, ["x"]), (0, 5, ["x"]),
                                  (130, 0, ["x"]), (130, 5, ["x", "x"])]:
            with self.assertRaises(ValueError):
                observer.validate_options(seconds, hz, ids, .5)
        with self.assertRaises(ValueError):
            observer.timeline([sample(1, 1), sample(1, 4)], .6)


if __name__ == "__main__":
    unittest.main()
