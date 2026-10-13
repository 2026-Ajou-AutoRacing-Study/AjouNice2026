"""Interpreter/UI contract and preflight guards; no GPU/ROS master required."""
import copy
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from planning_scenario_validator import gui, runner
from planning_scenario_validator.config import ConfigError, load_campaign
from planning_scenario_validator.traffic_light import DEFAULT_PYTHON, MODE_LABELS, runtime_mode, runtime_settings


class TrafficRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.example = gui.load_document(gui.EXAMPLES / "static_pair_stress_gt.yaml")

    def test_default_and_legacy_roundtrip(self):
        self.assertEqual(runtime_settings({}, self.root), ("/usr/bin/python3", "0"))
        for stack in ({}, {"traffic_light_python": "/my/conda/bin/python", "traffic_light_device": 0},
                      {"traffic_light_runtime": "system", "traffic_light_python": "/my/conda/bin/python"}):
            source = copy.deepcopy(self.example)
            source["stack"].update(stack)
            self.assertEqual(gui.build_document(source, gui.form_values(source)), source)
        self.assertEqual(runtime_mode({"traffic_light_python": "/my/conda/bin/python"}), "virtualenv")

    def test_gui_choices_persist_to_yaml(self):
        values = gui.form_values(self.example)
        values.update(traffic_light=True, traffic_light_runtime=MODE_LABELS["virtualenv"],
                      traffic_light_device="cpu")
        document = gui.build_document(self.example, values)
        stack = document["stack"]
        self.assertEqual(stack["traffic_light_runtime"], "virtualenv")
        self.assertEqual(stack["traffic_light_python"], DEFAULT_PYTHON)
        self.assertEqual(stack["traffic_light_device"], "cpu")
        values = gui.form_values(document)
        values["traffic_light_runtime"] = MODE_LABELS["system"]
        system = gui.build_document(document, values)
        self.assertEqual(runtime_settings(system["stack"], self.root)[0], "/usr/bin/python3")

    def test_paths_expand_without_resolving_venv_symlink(self):
        with patch.dict(os.environ, {"HOME": str(self.root)}):
            path, device = runtime_settings({"traffic_light_runtime": "virtualenv"}, self.root)
        self.assertEqual(path, str(self.root / ".local/share/ajounice/traffic_light_gpu/bin/python"))
        link = self.root / "python"
        link.symlink_to("/usr/bin/python3")
        self.assertEqual(runtime_settings({"traffic_light_python": "python"}, self.root, check_exists=True)[0], str(link))

    def test_off_does_not_require_an_environment_but_invalid_choices_fail(self):
        stack = {"traffic_light": False, "traffic_light_python": "/missing/bin/python"}
        runtime_settings(stack, self.root)
        with self.assertRaisesRegex(ValueError, "Python"):
            runtime_settings(stack, self.root, check_exists=True)
        for stack in ({"traffic_light_runtime": "docker"}, {"traffic_light_device": "-1"},
                      {"traffic_light_device": "0,1"}, {"traffic_light_device": "cuda"},
                      {"traffic_light_runtime": "virtualenv", "traffic_light_python": ""}):
            with self.assertRaises(ValueError):
                runtime_settings(stack, self.root)

    def test_campaign_on_checks_interpreter_but_off_does_not(self):
        import yaml
        document = copy.deepcopy(self.example)
        document["stack"].update(traffic_light_runtime="virtualenv", traffic_light_python="/missing/bin/python")
        path = self.root / "test.yaml"
        for enabled in (False, True):
            document["stack"]["traffic_light"] = enabled
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            if enabled:
                with self.assertRaisesRegex(ConfigError, "Python"):
                    load_campaign(path, check_install=False)
            else:
                load_campaign(path, check_install=False)

    def test_preflight_off_and_selected_python(self):
        cfg = {"output": self.root, "stack": {"traffic_light": False}}
        with patch.object(runner.subprocess, "run") as checked:
            runner._check_traffic_light(cfg)
            checked.assert_not_called()
        python = self.root / "custom python"
        python.symlink_to("/usr/bin/python3")
        cfg["stack"].update(traffic_light=True, traffic_light_python=str(python))
        with patch.object(runner.subprocess, "run", return_value=Mock(returncode=0, stdout="PASS\n")) as checked:
            runner._check_traffic_light(cfg)
            command = checked.call_args.args[0]
            self.assertEqual(command[:2], [str(python), "-s"])
            self.assertIn("--isolate-python", command)
        self.assertEqual((self.root / "stack_logs/traffic_light_preflight.log").read_text(), "PASS\n")

    def test_preflight_failure_and_timeout_abort(self):
        cfg = {"output": self.root, "stack": {"traffic_light": True}}
        with patch.object(runner.subprocess, "run", return_value=Mock(returncode=1, stdout="CUDA missing")):
            with self.assertRaisesRegex(runner.RuntimeFailure, "주행"):
                runner._check_traffic_light(cfg)
        with patch.object(runner.subprocess, "run", side_effect=subprocess.TimeoutExpired("check", 120)):
            with self.assertRaisesRegex(runner.RuntimeFailure, "실패"):
                runner._check_traffic_light(cfg)

    def test_launch_uses_selected_runtime_and_preserves_planning_toggle(self):
        python = self.root / "custom python"
        python.symlink_to("/usr/bin/python3")
        cfg = {"stack": {"pose_source": "maptf_morai", "detector": "od_gt", "traffic_light": True,
                         "traffic_light_python": str(python), "traffic_light_device": "cpu"},
               "network": {"sensor_json": "sensor.json"}, "document": {}}
        processes, monitor = Mock(), Mock()
        runner._start_case_stack(cfg, processes, monitor)
        calls = {call.args[0]: call.args[1] for call in processes.start.call_args_list}
        self.assertIn("inference_python:='{}'".format(python), calls["traffic_light"])
        self.assertIn("device:=cpu", calls["traffic_light"])
        self.assertIn("isolate_python:=true", calls["traffic_light"])
        self.assertIn("enable_traffic_light_planner:=true", calls["planning"])

    def test_installer_usage_and_existing_environment_protection(self):
        script = gui.ROOT / "scripts/runtime/setup_traffic_light_env.sh"
        result = subprocess.run(["bash", str(script), "--help"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn("--check", result.stdout)
        result = subprocess.run(["bash", str(script), "--device", "-1"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        with tempfile.TemporaryDirectory(prefix="ajounice-installer-test-", dir=Path.home()) as path:
            result = subprocess.run(["bash", str(script), "--prefix", path], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("기존 환경은 변경하지", result.stderr)
            self.assertEqual(list(Path(path).iterdir()), [])
            # Installer owns only this lock; no manager/env was created.
            Path(path + ".install.lock").unlink(missing_ok=True)

    @unittest.skipUnless(os.environ.get("DISPLAY"), "requires Xvfb or X11")
    def test_tk_traffic_environment_is_inspect_only(self):
        failure = []
        with patch.dict(os.environ, {"XDG_CONFIG_HOME": str(self.root)}):
            def ready(ui):
                def exercise():
                    try:
                        fields = ui["fields"]
                        def find_var(widget, var):
                            for child in widget.winfo_children():
                                if "textvariable" in child.keys() and str(child.cget("textvariable")) == str(var):
                                    return child
                                found = find_var(child, var)
                                if found is not None:
                                    return found
                            return None
                        mode = find_var(ui["root"], fields["traffic_light_runtime"])
                        python = find_var(ui["root"], fields["traffic_light_python"])
                        fields["traffic_light"].set(False)
                        self.assertEqual(str(mode.cget("state")), "disabled")
                        fields["traffic_light"].set(True)
                        fields["traffic_light_runtime"].set(MODE_LABELS["virtualenv"])
                        self.assertEqual(str(mode.cget("state")), "disabled")
                        self.assertEqual(str(python.cget("state")), "readonly")
                        fields["traffic_light_runtime"].set(MODE_LABELS["system"])
                        self.assertEqual(str(python.cget("state")), "readonly")
                    except Exception as error:
                        failure.append(error)
                    finally:
                        ui["root"].destroy()
                ui["root"].after(30, exercise)
            gui.main(on_ready=ready)
        if failure:
            raise failure[0]


if __name__ == "__main__":
    unittest.main()
