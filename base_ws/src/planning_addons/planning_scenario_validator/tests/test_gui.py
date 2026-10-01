"""Headless tests for the GUI's YAML contract (no Tk display or ROS needed)."""
import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.gui import (EXAMPLES, build_document, campaign_output,
                                             form_values, is_within, load_document,
                                             personal_path, save_personal)


class GuiConfigTests(unittest.TestCase):
    def setUp(self):
        self.example = load_document(EXAMPLES / "static_pair_stress_gt.yaml")

    def test_form_preserves_advanced_and_case_settings(self):
        source = copy.deepcopy(self.example)
        source["default_case"]["waypoints"] = [{"x": 1, "y": 2}]
        source["custom_team_key"] = {"keep": True}
        values = form_values(source)
        values.update(campaign_name="soa_pair_gui_test", detector="bevfusion",
                      goal_x="42.5", video=False, timeout_sec="90")
        result = build_document(source, values)
        self.assertEqual(result["default_case"]["goal"]["x"], 42.5)
        self.assertEqual(result["default_case"]["waypoints"], [{"x": 1, "y": 2}])
        self.assertEqual(result["custom_team_key"], {"keep": True})
        self.assertEqual(result["stack"]["detector"], "bevfusion")
        self.assertFalse(result["recording"]["video"]["enabled"])
        self.assertEqual(source["stack"]["detector"], "od_gt")

    def test_save_resume_does_not_rewrite_hash_or_running_config(self):
        with tempfile.TemporaryDirectory() as temp:
            values = form_values(self.example)
            values["campaign_name"] = "gui_resume_test"
            document = build_document(self.example, values)
            document["output_root"] = temp
            path = Path(temp) / "config.yaml"
            save_personal(document, path)
            original = path.read_bytes()
            output = campaign_output(document)
            output.mkdir()
            (output / "campaign_state.json").write_text(json.dumps({"current": None}), encoding="utf-8")
            save_personal(document, path)
            self.assertEqual(original, path.read_bytes())
            changed = copy.deepcopy(document)
            changed["stack"]["detector"] = "bevfusion"
            with self.assertRaisesRegex(ValueError, "새 캠페인 이름"):
                save_personal(changed, path)

    def test_bad_name_and_relative_paths(self):
        values = form_values(self.example)
        values["campaign_name"] = "../wrong"
        with self.assertRaises(ValueError):
            build_document(self.example, values)
        self.assertTrue(is_within(EXAMPLES / "static_pair_stress_gt.yaml", EXAMPLES))
        self.assertFalse(is_within(Path("/tmp/elsewhere.yaml"), EXAMPLES))
        self.assertEqual(personal_path({"campaign_name": "test_gui"}, user="sw").name, "test_gui.yaml")


if __name__ == "__main__":
    unittest.main()
