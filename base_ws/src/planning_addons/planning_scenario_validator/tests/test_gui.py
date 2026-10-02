"""Headless tests for the GUI's YAML contract (no Tk display or ROS needed)."""
import copy
import json
import os
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.gui import (EXAMPLES, build_document, campaign_output,
                                             completion_label, form_values, is_within,
                                             load_document, personal_path,
                                             render_result_table, result_counts,
                                             save_personal)
from planning_scenario_validator import gui as gui_module


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

    def test_nonfinite_or_empty_critical_values_are_rejected_before_save(self):
        cases = (("goal_x", "nan"), ("timeout_sec", "inf"),
                 ("goal_tolerance_m", "0"), ("scenario_load_host", " "))
        for field, bad in cases:
            with self.subTest(field=field):
                values = form_values(self.example)
                values["campaign_name"] = "safe_gui_input"
                values[field] = bad
                with self.assertRaises(ValueError):
                    build_document(self.example, values)

    def test_exit_zero_is_not_a_success_label_when_scenarios_failed(self):
        state = {"total": 3, "results": {
            "one": {"status": "PASS"},
            "two": {"status": "FAIL"},
            "three": {"status": "ERROR"},
        }}
        self.assertEqual(result_counts(state), {"PASS": 1, "FAIL": 1, "ERROR": 1})
        label = completion_label(0, state)
        self.assertIn("주행 실패 1", label)
        self.assertIn("시스템 오류 1", label)
        self.assertNotIn("전체 성공", label)
        self.assertIn("결과 파일 없음", completion_label(2, None))


@unittest.skipUnless(os.environ.get("DISPLAY"), "Tk display가 없어 화면 테스트를 건너뜁니다")
class GuiWindowTests(unittest.TestCase):
    def test_result_selection_survives_progress_refresh(self):
        import tkinter as tk
        from tkinter import ttk
        root = tk.Tk()
        root.withdraw()
        try:
            table = ttk.Treeview(root, columns=("scenario", "status", "reason", "category", "stage"))
            paths = {}
            first = {"results": {"fail_scene": {"status": "FAIL", "reason": "TIMEOUT",
                    "failure_category": "NO_DEPARTURE", "suspected_stage": "planning_output",
                    "result_path": "/tmp/fail.json"}}}
            render_result_table(table, first, paths)
            table.selection_set("fail_scene")
            second = {"results": {"fail_scene": {"status": "FAIL", "reason": "TIMEOUT",
                    "failure_category": "NO_DEPARTURE", "suspected_stage": "planning_output",
                    "result_path": "/tmp/fail.json"},
                "pass_scene": {"status": "PASS", "reason": "SUCCESS",
                    "failure_category": "SUCCESS", "suspected_stage": "undetermined",
                    "result_path": "/tmp/pass.json"}}}
            render_result_table(table, second, paths)
            self.assertEqual(table.selection(), ("fail_scene",))
            self.assertEqual(len(table.get_children()), 2)
            self.assertEqual(paths["fail_scene"], Path("/tmp/fail.json"))
            self.assertEqual(table.item("pass_scene", "values")[-1], "")
        finally:
            root.destroy()

    def test_buttons_check_start_stop_and_forward_rerun_option(self):
        with tempfile.TemporaryDirectory() as temp:
            executable = Path(temp) / "fake_validator"
            executable.write_text(
                "#!/usr/bin/python3\n"
                "import sys, time\n"
                "if '--dry-run' in sys.argv:\n"
                "    time.sleep(0.7)\n"
                "    print('DRY_RUN_OK', flush=True)\n"
                "else:\n"
                "    print('RUN_STARTED ' + ' '.join(sys.argv[2:]), flush=True)\n"
                "    time.sleep(30)\n", encoding="utf-8")
            executable.chmod(0o755)
            captured = {}

            def ready(app):
                root = app["root"]
                captured["app"] = app
                app["fields"]["campaign_name"].set("gui_process_smoke")
                app["rerun_failed"].set(True)
                root.after(50, app["actions"]["check"])
                root.after(250, app["actions"]["start"])  # Must not overlap dry-run.
                root.after(1150, app["actions"]["start"])
                root.after(1900, app["actions"]["stop"])

                def finish():
                    if app["active"]["process"] is None and "실행 종료" in app["status"].get():
                        captured["log"] = app["log"].get("1.0", "end")
                        captured["status"] = app["status"].get()
                        root.after_cancel(safety_timer)
                        root.destroy()
                    else:
                        root.after(100, finish)

                root.after(2200, finish)
                safety_timer = root.after(9000, root.destroy)  # A failing test must not hang CI.

            with patch.object(gui_module, "USERS", Path(temp) / "users"), \
                 patch("tkinter.messagebox.askokcancel", return_value=True), \
                 patch("tkinter.messagebox.askyesno", return_value=True), \
                 patch("tkinter.messagebox.showinfo") as info:
                gui_module.main(on_ready=ready, runner_executable=executable)
            process = captured["app"]["active"]["process"]
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
            self.assertIn("DRY_RUN_OK", captured.get("log", ""))
            self.assertIn("RUN_STARTED --rerun-failed", captured.get("log", ""))
            self.assertTrue(any(call.args[0] == "설정 검사 중" for call in info.call_args_list))
            self.assertIn("결과 파일 없음", captured.get("status", ""))

    def test_video_mode_hides_gui_and_terminal_interrupt_stops_runner(self):
        with tempfile.TemporaryDirectory() as temp:
            executable = Path(temp) / "fake_validator"
            executable.write_text("#!/usr/bin/python3\nimport time\n"
                                  "print('VIDEO_RUN_STARTED', flush=True)\ntime.sleep(30)\n",
                                  encoding="utf-8")
            executable.chmod(0o755)
            captured = {}

            def ready(app):
                root = app["root"]
                app["fields"]["campaign_name"].set("gui_video_interrupt")
                app["fields"]["video"].set(True)
                root.after(50, app["actions"]["start"])

                def interrupt():
                    captured["window_state"] = root.state()
                    signal.raise_signal(signal.SIGINT)

                root.after(1250, interrupt)

                def finish():
                    if app["active"]["process"] is None and "실행 종료" in app["status"].get():
                        captured["log"] = app["log"].get("1.0", "end")
                        root.after_cancel(safety_timer)
                        root.destroy()
                    else:
                        root.after(100, finish)

                root.after(1550, finish)
                safety_timer = root.after(8000, root.destroy)

            with patch.object(gui_module, "USERS", Path(temp) / "users"), \
                 patch("tkinter.messagebox.askokcancel", return_value=True):
                gui_module.main(on_ready=ready, runner_executable=executable)
            self.assertEqual(captured.get("window_state"), "withdrawn")
            self.assertIn("VIDEO_RUN_STARTED", captured.get("log", ""))


if __name__ == "__main__":
    unittest.main()
