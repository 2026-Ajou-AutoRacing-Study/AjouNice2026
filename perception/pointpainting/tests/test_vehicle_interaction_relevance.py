import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "tools/audit_vehicle_interaction_relevance.py"
)
SPEC = importlib.util.spec_from_file_location("interaction_audit", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def interaction(current=20.0, synchronous=20.0, path=20.0, delta=0.0):
    return {
        "current_distance_m": current,
        "synchronous_min_distance_m": synchronous,
        "path_min_distance_m": path,
        "path_min_time_gap_sec": 3.0,
        "distance_delta_m": delta,
    }


class InteractionRelevanceTest(unittest.TestCase):
    def test_geometric_path_overlap_is_critical_despite_safe_time_gap(self):
        value = interaction(path=3.0)
        self.assertEqual(MODULE.relevance_label(value), "conflict_critical")

    def test_close_synchronous_actor_is_critical(self):
        value = interaction(synchronous=7.9)
        self.assertEqual(MODULE.relevance_label(value), "conflict_critical")

    def test_near_non_conflicting_actor_is_planning_relevant(self):
        value = interaction(current=14.0, synchronous=12.0, path=8.0)
        self.assertEqual(MODULE.relevance_label(value), "planning_relevant")

    def test_far_diverging_actor_is_peripheral(self):
        value = interaction(current=25.0, synchronous=22.0, path=10.0, delta=4.0)
        self.assertEqual(MODULE.relevance_label(value), "peripheral")
        self.assertEqual(MODULE.trend_label(value["distance_delta_m"]), "receding")

    def test_path_sampling_interpolates(self):
        points = ((0.0, 0.0), (1.0, 2.0), (2.0, 4.0))
        self.assertEqual(MODULE.sample_path(points, 1.0, 1.5), (1.5, 3.0))


if __name__ == "__main__":
    unittest.main()
