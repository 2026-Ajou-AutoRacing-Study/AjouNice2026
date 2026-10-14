import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from replay_pipeline_diagnostics_offline import select_window, select_balanced_cohort, validate_received


class PrivateReplayTests(unittest.TestCase):
    def test_window_is_contiguous_mixed_and_prefers_unknown(self):
        rows = [dict(stamp_ns=i, objects=int(i % 2 == 0), unknown=int(i == 7)) for i in range(20)]
        start, selected = select_window(rows, 6)
        self.assertEqual(selected, rows[start:start + 6])
        self.assertEqual(sum(row["objects"] for row in selected), 3)
        self.assertTrue(any(row["unknown"] for row in selected))

    def test_bad_or_unmixed_input_fails_before_private_nodes(self):
        rows = [dict(stamp_ns=i, objects=1, unknown=0) for i in range(10)]
        with self.assertRaises(ValueError):
            select_window(rows, 11)
        with self.assertRaises(ValueError):
            select_window(rows, 5)
        rows[2]["objects"] = 0
        rows[2]["stamp_ns"] = 1
        with self.assertRaises(ValueError):
            select_window(rows, 10)

    def test_drop_is_exact_subset_not_implied_output_success(self):
        expected = {1: b"one", 2: b"two", 3: b"three"}
        result = validate_received(expected, {1: b"one", 3: b"three"})
        self.assertTrue(result["exact_subset"])
        self.assertEqual(result["missing"], [2])
        self.assertFalse(validate_received(expected, {4: b"four"})["exact_subset"])
        self.assertFalse(validate_received(expected, {1: b"changed"})["exact_subset"])

    def test_balanced_cohort_exercises_all_present_categories_without_mutation(self):
        rows = [dict(stamp_ns=i, objects=int(i % 4 != 0), unknown=int(i % 4 == 1),
                     pedestrian=int(i % 4 == 2), vehicle=int(i % 4 == 3)) for i in range(100)]
        indices, cohort = select_balanced_cohort(rows, 20)
        self.assertEqual(len(indices), 20)
        self.assertEqual(indices, sorted(set(indices)))
        self.assertTrue(all(cohort[index] is rows[original] for index, original in enumerate(indices)))
        for name in ("unknown", "pedestrian", "vehicle"):
            self.assertTrue(any(row[name] for row in cohort))
        self.assertTrue(any(not row["objects"] for row in cohort))


if __name__ == "__main__":
    unittest.main()
