"""One YAML, one common driving condition; no hidden per-scene settings."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from planning_scenario_validator import gui
from planning_scenario_validator.config import (ConfigError, EXECUTION_CONTRACT, load_campaign)
from planning_scenario_validator.runner import _load_state
from planning_scenario_validator.runtime import RuntimeFailure


class CommonCampaignTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.document = gui.load_document(gui.MISSIONS / "highway/base/example.yaml")
        self.scenes = self.root / "scenarios"
        self.scenes.mkdir()
        for name in ("one", "two"):
            (self.scenes / (name + ".json")).write_text(json.dumps({
                "mapInfo": {"mapName": "R_KR_PR_K-city_2025"},
                "egoVehicle": {"initPosition": {"pos": {"x": 0, "y": 0}}}}))
        self.document["scenarios"] = {"directory": str(self.scenes), "glob": "*.json"}
        self.document["output_root"] = str(self.root / "results")

    def save(self, name="campaign.yaml", document=None):
        path = self.root / name
        path.write_text(yaml.safe_dump(document or self.document), encoding="utf-8")
        return path

    def test_all_cases_use_gui_common_settings_even_with_broken_sidecars(self):
        (self.scenes / "one.case.yaml").write_text("goal: {x: -999, y: -999}\ntimeout_sec: 1\narrival_mode: pass_through\n")
        (self.scenes / "two.case.yaml").write_text("[broken YAML")
        values = gui.form_values(self.document)
        values.update(goal_x=42.5, goal_y=12.5, timeout_sec=23)
        self.document = gui.build_document(self.document, values)
        cfg = load_campaign(self.save(), check_install=False)
        self.assertEqual(len(cfg["ignored_case_yamls"]), 2)
        self.assertTrue(all(case.goal["x"] == 42.5 and case.goal["y"] == 12.5
                            and case.timeout_sec == 23 and case.arrival_mode == "stop" for case in cfg["cases"]))

    def test_different_goal_requires_another_yaml_and_does_not_change_inputs(self):
        first = self.save()
        original = first.read_bytes()
        other = copy.deepcopy(self.document)
        other["campaign_name"] = "other_goal"
        other["default_case"]["goal"]["x"] += 10
        second = self.save("other_goal.yaml", other)
        a = load_campaign(first, check_install=False)
        b = load_campaign(second, check_install=False)
        self.assertEqual([c.source_sha256 for c in a["cases"]], [c.source_sha256 for c in b["cases"]])
        self.assertEqual(b["cases"][0].goal["x"] - a["cases"][0].goal["x"], 10)
        self.assertNotEqual(a["output"], b["output"])
        self.assertEqual(first.read_bytes(), original)

    def test_gui_rejects_sidecar_and_incomplete_mapping_before_loading(self):
        for name, content in (("one.case.yaml", {"goal": {"x": 1, "y": 2}, "timeout_sec": 10}),
                              ("empty.yaml", {}), ("partial.yaml", {"schema_version": 2})):
            path = self.root / name
            path.write_text(yaml.safe_dump(content))
            with self.assertRaises(ValueError):
                gui.load_document(path)
        with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(self.root / "settings")}):
            gui.remember_yaml(self.root / "one.case.yaml")
            self.assertIsNone(gui.remembered_yaml())

    def test_legacy_yaml_is_inspectable_but_execution_needs_explicit_v2_save(self):
        legacy = copy.deepcopy(self.document)
        legacy["schema_version"] = 1
        path = self.save("old.yaml", legacy)
        before = path.read_bytes()
        loaded = gui.load_document(path)
        self.assertEqual(loaded["schema_version"], 1)
        with self.assertRaisesRegex(ConfigError, "schema_version: 2"):
            load_campaign(path, check_install=False)
        upgraded = gui.build_document(loaded, gui.form_values(loaded))
        self.assertEqual(upgraded["schema_version"], 2)
        self.assertEqual(upgraded["default_case"], loaded["default_case"])
        self.assertEqual(path.read_bytes(), before)

    @unittest.skipUnless(os.environ.get("DISPLAY"), "Tk integration requires X11/Xvfb")
    def test_gui_rejected_open_preserves_active_form(self):
        bad = self.root / "empty.yaml"
        bad.write_text("{}")
        captured = []

        def ready(app):
            def exercise():
                try:
                    good = gui.MISSIONS / "highway/base/example.yaml"
                    app["actions"]["open_yaml"](good)
                    before = {key: value.get() for key, value in app["fields"].items()}
                    source = app["active"]["source_path"]
                    self.assertEqual(float(before["goal_x"]), 197.5177)
                    self.assertEqual(float(before["timeout_sec"]), 90)
                    with self.assertRaises(ValueError):
                        app["actions"]["open_yaml"](bad)
                    self.assertEqual(before, {key: value.get() for key, value in app["fields"].items()})
                    self.assertEqual(source, app["active"]["source_path"])
                    captured.append(True)
                except BaseException as error:
                    captured.append(error)
                finally:
                    app["root"].destroy()
            app["root"].after(30, exercise)

        with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(self.root / "settings")}):
            gui.main(on_ready=ready)
        self.assertEqual(len(captured), 1)
        if isinstance(captured[0], BaseException):
            raise captured[0]

    def test_resume_uses_new_contract_and_preserves_legacy_results(self):
        cfg = load_campaign(self.save(), check_install=False)
        cfg["output"].mkdir(parents=True)
        state = _load_state(cfg, True)
        self.assertEqual(state["execution_contract"], EXECUTION_CONTRACT)
        state["results"] = {"one": {"status": "PASS"}}
        path = cfg["output"] / "campaign_state.json"
        path.write_text(json.dumps(state))
        self.assertEqual(_load_state(cfg, True)["results"], state["results"])
        legacy = copy.deepcopy(state)
        legacy.pop("execution_contract")
        path.write_text(json.dumps(legacy))
        before = path.read_bytes()
        with self.assertRaisesRegex(RuntimeFailure, "새 이름"):
            _load_state(cfg, True)
        self.assertEqual(path.read_bytes(), before)

    def test_changed_yaml_cannot_resume_and_does_not_erase_state(self):
        cfg = load_campaign(self.save(), check_install=False)
        cfg["output"].mkdir(parents=True)
        path = cfg["output"] / "campaign_state.json"
        path.write_text(json.dumps(_load_state(cfg, True)))
        before = path.read_bytes()
        cfg["path"].write_text(cfg["path"].read_text() + "# changed\n")
        with self.assertRaisesRegex(RuntimeFailure, "YAML 내용"):
            _load_state(cfg, True)
        self.assertEqual(path.read_bytes(), before)

    def test_no_per_case_override_or_non_json_input(self):
        self.document["cases"] = {"one": {"goal": {"x": 1, "y": 2}}}
        with self.assertRaisesRegex(ConfigError, "별도 YAML"):
            load_campaign(self.save(), check_install=False)
        self.document.pop("cases")
        self.document["scenarios"]["glob"] = "*"
        (self.scenes / "ignored.yaml").write_text("not a scenario")
        (self.scenes / "directory.json").mkdir()
        self.assertEqual(len(load_campaign(self.save(), check_install=False)["cases"]), 2)


if __name__ == "__main__":
    unittest.main()
