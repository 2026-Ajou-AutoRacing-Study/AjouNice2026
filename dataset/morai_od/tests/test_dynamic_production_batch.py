#!/usr/bin/env python3

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "dataset/morai_od/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


def load_module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


batch = load_module(
    "dynamic_production_batch",
    "dataset/morai_od/scripts/generate_dynamic_dataset_batch.py",
)
static_component = load_module(
    "static_component",
    "dataset/morai_od/scripts/generate_static_component_set.py",
)


class DynamicProductionBatchTest(unittest.TestCase):
    def test_primary_quota_is_exactly_1500(self):
        self.assertEqual(batch.PRIMARY_QUOTAS, {"mission": 744, "static": 525, "hard": 231})
        self.assertEqual(sum(batch.PRIMARY_QUOTAS.values()), 1500)

    def test_reserve_reflects_observed_personal_launcher_failure_rate(self):
        ratio = sum(batch.RESERVE_QUOTAS.values()) / sum(batch.PRIMARY_QUOTAS.values())
        self.assertGreaterEqual(ratio, 0.15)
        self.assertLessEqual(ratio, 0.20)

    def test_static_reserve_is_load_safe_and_obstacle_focused(self):
        values = batch.static_scene_types(primary=False)
        self.assertEqual(sum(item["count"] for item in values), 50)
        obstacle = next(item for item in values if item["name"] == "obstacle_centric")
        self.assertEqual(obstacle["count"], 35)

    def test_static_target_has_40_percent_obstacle_focused_scenes(self):
        values = batch.static_scene_types(primary=True)
        self.assertEqual(sum(item["count"] for item in values), 175)
        obstacle = next(item for item in values if item["name"] == "obstacle_centric")
        self.assertEqual(obstacle["count"], 70)

    def test_static_component_preserves_exact_split(self):
        spec = batch.static_spec("batch001", "static", 7, batch.static_scene_types(True))
        values = static_component.build_scene_specs(spec)
        self.assertEqual(sum(item["count"] for item in values), 175)
        observed = {key: 0 for key in ("train", "val", "test")}
        for value in values:
            for split in value["splits"]:
                observed[split] += 1
        self.assertEqual(observed, spec["split_counts"])


if __name__ == "__main__":
    unittest.main()
