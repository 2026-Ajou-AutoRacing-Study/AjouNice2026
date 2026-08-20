#!/usr/bin/env python3

import importlib
import sys
import unittest
from collections import Counter
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "dataset/morai_od/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))


class StaticRandomCapturePolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        wrapper = importlib.import_module("generate_static_random_1998_v1_set")
        cls.generator = wrapper.generator
        cls.config = yaml.safe_load(
            (
                ROOT
                / "dataset/morai_od/configs/od_static_random_1998_v1.yaml"
            ).read_text(encoding="utf-8")
        )

    def test_three_frames_cover_nine_metres(self):
        policy = self.generator.capture_policy(self.config)
        self.assertEqual(policy["captures_per_scene"], 3)
        self.assertEqual(policy["capture_spacing_m"], 3.0)
        required_motion = (
            policy["captures_per_scene"] * policy["capture_spacing_m"]
        )
        self.assertEqual(required_motion, 9.0)
        self.assertGreaterEqual(policy["maximum_motion_m"], required_motion)

    def test_scene_and_split_quotas(self):
        specs = self.generator.SCENE_TYPE_SPECS
        self.assertEqual(sum(spec["count"] for spec in specs), 666)
        splits = Counter(
            split for spec in specs for split in spec["splits"]
        )
        self.assertEqual(
            splits, Counter({"train": 466, "val": 100, "test": 100})
        )

    def test_current_baseline_uses_only_sunny_13(self):
        self.assertEqual(
            self.generator.ENVIRONMENTS,
            ({"id": "sunny_t13", "weather": "sunny", "time_hour": 13},),
        )
        self.assertEqual(666 * 3, 1998)


if __name__ == "__main__":
    unittest.main()
