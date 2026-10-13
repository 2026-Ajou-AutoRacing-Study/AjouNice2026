import pathlib
import ast
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from profile_pipeline_cpu_map_gc import PendingExpectations, PairedTimingLedger, HeaderPacer, overlap, GC_SHIM


class StreamingGcTests(unittest.TestCase):
    def test_parent_wire_heap_is_bounded_and_removes_matched_payload(self):
        pending = PendingExpectations(2)
        pending.put(1, b"one", 1.)
        pending.put(2, b"two", 2.)
        pending.put(3, b"three", 3.)
        self.assertEqual(pending.evicted, [1])
        self.assertEqual(pending.peak_count, 2)
        self.assertEqual(pending.pop(2), (b"two", 2.))
        self.assertEqual(pending.bytes, 5)
        self.assertIsNone(pending.pop(1))

    def test_overlap_requires_actual_callback_and_gc_time_intersection(self):
        events = [(2, 1., 2.), (0, 2., 2.1), (1, 3., 4.)]
        self.assertEqual(overlap(1.5, 2., events), [events[0]])
        self.assertEqual(overlap(2.2, 2.8, events), [])

    def test_shim_compiles_without_gc_control_or_input_output_capture(self):
        compile(GC_SHIM, "gc_shim", "exec")
        for forbidden in ("gc.disable(", "gc.collect(", "gc.set_threshold("):
            self.assertNotIn(forbidden, GC_SHIM)
        self.assertIn("deque(maxlen=10000)", GC_SHIM)
        self.assertIn("finally:", GC_SHIM)

    def test_initial_bag_index_oracle_time_cannot_create_catchup_burst(self):
        pacer = HeaderPacer(3.)
        self.assertEqual(pacer.target(100_000_000_000, 1234.), 1234.)
        self.assertAlmostEqual(pacer.target(100_100_000_000, 1234.001), 1234. + .1 / 3.)

    def test_paired_join_handles_any_callback_order_and_exact_component_sum(self):
        ledger = PairedTimingLedger(2)
        ledger.put(1, send_start=1.)
        ledger.put(1, observer_entry=1.007, wire_verify_end=1.009)
        ledger.put(1, callback_start=1.002, objects_publish_return=1.006, callback_end=1.008)
        self.assertEqual(ledger.completed, 0)
        ledger.put(1, publish_return=1.001)
        self.assertEqual(ledger.completed, 1)
        self.assertEqual(len(ledger.pending), 0)
        sample = ledger.tails[0]["paired_ms"]
        self.assertAlmostEqual(sum(sample[key] for key in ("send_to_callback_start",
            "callback_start_to_objects_publish_return", "objects_publish_return_to_observer_entry",
            "observer_entry_to_wire_verify")), sample["send_to_wire_verify"])
        self.assertLess(sample["callback_end_to_observer_entry"], 0.)

    def test_pair_pending_tokens_and_retained_tail_tokens_are_bounded(self):
        ledger = PairedTimingLedger(2, 4)
        for token in range(20):
            ledger.put(token, send_start=float(token))
        self.assertEqual(len(ledger.pending), 2)
        self.assertEqual(ledger.evicted, 18)
        for token in range(20, 40):
            ledger.put(token, send_start=float(token), publish_return=token+.001,
                callback_start=token+.002, objects_publish_return=token+.003,
                callback_end=token+.004, observer_entry=token+.005,
                wire_verify_end=token+.006 + token/1000.)
        self.assertEqual(len(ledger.tails), 4)
        self.assertEqual(len(ledger.pending), 1)
        self.assertEqual(sorted(row["token_ns"] for row in ledger.tails), [36, 37, 38, 39])

    def test_missing_token_samples_never_accumulate_all_evictions(self):
        pending = PendingExpectations(2)
        for token in range(1000):
            pending.put(token, b"x", 1.)
        self.assertEqual(pending.evicted_count, 998)
        self.assertEqual(len(pending.evicted), 64)

    def test_trial_subscriber_override_is_input_only_and_preserves_other_arguments(self):
        tree = ast.parse(GC_SHIM)
        function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                        and node.name == "make_trial_subscriber")
        namespace = {}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])), "shim_subscriber", "exec"), namespace)
        for enabled in (False, True):
            calls, requests = [], []
            def original(name, *args, **kwargs):
                result = (name, args, kwargs)
                calls.append(result)
                return result
            subscriber = namespace["make_trial_subscriber"](original, enabled, requests)
            result = subscriber("~input/objects", "TrackedObjects", "callback", queue_size=1)
            self.assertIs(result, calls[-1])
            self.assertEqual(result[2], dict(queue_size=1, tcp_nodelay=enabled))
            self.assertEqual(requests, [("~input/objects", enabled)])
            subscriber("/clock", "Clock", queue_size=10, tcp_nodelay=True)
            self.assertEqual(calls[-1][2], dict(queue_size=10, tcp_nodelay=True))
            self.assertEqual(len(requests), 1)


if __name__ == "__main__":
    unittest.main()
