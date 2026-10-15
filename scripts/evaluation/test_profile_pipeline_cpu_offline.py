import gc
import io
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from profile_pipeline_cpu_offline import observe_gc, serialize, stats


class ProfilePipelineCpuTest(unittest.TestCase):
    def test_stats_include_tail_without_trimming(self):
        result = stats([1., 2., 3., 100.])
        self.assertEqual(result["count"], 4)
        self.assertEqual(result["mean"], 26.5)
        self.assertEqual(result["median"], 2.5)
        self.assertEqual(result["max"], 100.)
        self.assertGreater(result["p95"], 80.)
        self.assertGreater(result["p99"], result["p95"])

    def test_empty_stats_have_no_invented_zero_ms(self):
        self.assertEqual(stats([]), {"count": 0})

    def test_gc_probe_keeps_normal_gc_and_restores_callback_on_error(self):
        before = list(gc.callbacks)
        enabled = gc.isenabled()
        with self.assertRaises(ValueError):
            with observe_gc():
                self.assertEqual(gc.isenabled(), enabled)
                self.assertEqual(len(gc.callbacks), len(before)+1)
                raise ValueError("diagnostic failure")
        self.assertEqual(gc.callbacks, before)
        self.assertEqual(gc.isenabled(), enabled)

    def test_wire_serialization_does_not_modify_message(self):
        class Message:
            def __init__(self):
                self.value = b"\x00\xfftest"
            def serialize(self, output):
                output.write(self.value)
        message = Message()
        self.assertEqual(serialize(message), message.value)
        self.assertEqual(serialize(message), b"\x00\xfftest")


if __name__ == "__main__":
    unittest.main()
