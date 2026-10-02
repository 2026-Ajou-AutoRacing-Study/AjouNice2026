"""Headless tests for the GUI's YAML contract (no Tk display or ROS needed)."""
import copy
import hashlib
import json
import os
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.gui import (EXAMPLES, build_document, campaign_output,
                                             completion_label, form_values, is_within,
                                             load_document, personal_path,
                                             render_result_table, result_counts,
                                             save_personal)
from planning_scenario_validator import gui as gui_module
from planning_scenario_validator.config import DEFAULT_UPLOAD_REMOTE, EXECUTION_CONTRACT


class GuiConfigTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        config_patch = patch.dict(os.environ, {"XDG_CONFIG_HOME": directory.name})
        config_patch.start()
        self.addCleanup(config_patch.stop)
        self.example = load_document(EXAMPLES / "static_pair_stress_gt.yaml")

    def test_remember_existing_yaml_and_fallback_when_deleted(self):
        path = Path(os.environ["XDG_CONFIG_HOME"]) / "personal.yaml"
        path.write_text(yaml.safe_dump(self.example), encoding="utf-8")
        original = path.read_bytes()
        gui_module.remember_yaml(path)
        self.assertEqual(gui_module.remembered_yaml(), path.resolve())
        self.assertEqual(path.read_bytes(), original)
        path.unlink()
        self.assertIsNone(gui_module.remembered_yaml())

    def test_bad_gui_settings_and_bad_yaml_fall_back(self):
        settings = gui_module.gui_settings_path()
        settings.parent.mkdir(parents=True)
        settings.write_text("broken json", encoding="utf-8")
        self.assertIsNone(gui_module.remembered_yaml())
        path = settings.parent / "invalid.yaml"
        path.write_text("- not a campaign", encoding="utf-8")
        gui_module.remember_yaml(path)
        self.assertIsNone(gui_module.remembered_yaml())

    def test_mission_workspace_copy_preserves_edits_and_saves_named_campaigns(self):
        root = Path(os.environ['XDG_CONFIG_HOME'])
        missions = root / 'missions'
        base = missions / 'highway/base'
        base.mkdir(parents=True)
        (base / 'scenarios').mkdir()
        example = copy.deepcopy(self.example)
        example['output_root'] = str(root / 'results')
        (base / 'example.yaml').write_text(yaml.safe_dump(example))
        (base / 'README.md').write_text('base guide')
        (base / 'scenarios/a.json').write_text('{"base": true}')
        (base / 'scenarios/a.case.yaml').write_text('timeout_sec: 45\n')
        with patch.object(gui_module, 'MISSIONS', missions), patch.object(gui_module, 'ROOT', root):
            first = gui_module.copy_base_to_workspace(base / 'example.yaml')
            self.assertEqual(first.parent, missions / 'highway/workspace')
            document = load_document(first)
            self.assertEqual(document['scenarios']['directory'], 'missions/highway/workspace/scenarios')
            self.assertFalse((first.parent / 'scenarios/a.case.yaml').exists())
            self.assertTrue(gui_module.is_personal(first))
            notes = first.parent / 'README.md'
            self.assertIn('../base/README.md', notes.read_text())
            notes.write_text('내 실험 메모', encoding='utf-8')
            self.assertEqual(gui_module.personal_path({'campaign_name': 'new_name'}, source=first),
                             first.parent / 'new_name.yaml')
            changed = first.parent / 'scenarios/a.json'
            changed.write_text('{"my_edit": true}')
            second = gui_module.copy_base_to_workspace(first)
            self.assertNotEqual(first, second)
            self.assertEqual(changed.read_text(), '{"my_edit": true}')
            self.assertEqual(notes.read_text(encoding='utf-8'), '내 실험 메모')
            with self.assertRaisesRegex(ValueError, '공유 base'):
                save_personal(example, base / 'example.yaml')
    def test_old_campaign_roundtrip_keeps_resume_hash_compatible(self):
        self.assertEqual(build_document(self.example, form_values(self.example)), self.example)
        changed = form_values(self.example)
        changed["video_layout"] = "split"
        video = build_document(self.example, changed)["recording"]["video"]
        self.assertEqual(video["window_layout"], "split")
        self.assertFalse(video["focus_planviz"])

    def test_legacy_blank_drive_path_displays_default_without_changing_resume_yaml(self):
        legacy = copy.deepcopy(self.example)
        legacy["upload"]["rclone_remote"] = ""
        values = form_values(legacy)
        self.assertEqual(values["upload_remote"], DEFAULT_UPLOAD_REMOTE)
        self.assertEqual(build_document(legacy, values), legacy)
        values["upload_enabled"] = True
        self.assertEqual(build_document(legacy, values)["upload"]["rclone_remote"],
                         DEFAULT_UPLOAD_REMOTE)

    def test_monitor_bar_setting_roundtrip(self):
        values = form_values(self.example)
        values["bottom_bar_px"] = "18%"
        document = build_document(self.example, values)
        self.assertEqual(document["recording"]["video"]["bottom_bar_px"], "18%")
        self.assertEqual(form_values(document)["bottom_bar_px"], "18%")
        values["bottom_bar_px"] = "400%"
        with self.assertRaisesRegex(ValueError, "10~30%"):
            build_document(self.example, values)

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

    def test_upload_profile_roundtrip_and_remote_validation(self):
        values = form_values(self.example)
        values.update(upload_enabled=True, upload_profile=gui_module.UPLOAD_PROFILE_LABELS["bag"],
                      upload_remote="team:planning")
        result = build_document(self.example, values)
        self.assertEqual(result["upload"], {"enabled": True, "rclone_remote": "team:planning",
                                            "profile": "bag"})
        self.assertEqual(form_values(result)["upload_profile"], gui_module.UPLOAD_PROFILE_LABELS["bag"])
        values["upload_remote"] = ""
        with self.assertRaisesRegex(ValueError, "rclone 경로"):
            build_document(self.example, values)

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
            (output / "campaign_state.json").write_text(json.dumps({
                "current": None, "config_path": str(path.resolve()),
                "execution_contract": EXECUTION_CONTRACT,
                "config_sha256": hashlib.sha256(original).hexdigest()}), encoding="utf-8")
            save_personal(document, path)
            self.assertEqual(original, path.read_bytes())
            changed = copy.deepcopy(document)
            changed["stack"]["detector"] = "bevfusion"
            with self.assertRaisesRegex(ValueError, "새 캠페인 이름"):
                save_personal(changed, path)
            path.write_bytes(original + b"# edited comment\n")
            with self.assertRaisesRegex(ValueError, "새 캠페인 이름"):
                save_personal(document, path)

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
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        config_patch = patch.dict(os.environ, {"XDG_CONFIG_HOME": directory.name})
        config_patch.start()
        self.addCleanup(config_patch.stop)

    def test_workspace_button_and_save_use_selected_mission(self):
        root_path = Path(os.environ['XDG_CONFIG_HOME'])
        missions = root_path / 'missions'
        base = missions / 'roundabout/base'
        (base / 'scenarios').mkdir(parents=True)
        example = load_document(EXAMPLES / 'static_pair_stress_gt.yaml')
        example['output_root'] = str(root_path / 'results')
        (base / 'example.yaml').write_text(yaml.safe_dump(example))
        (base / 'scenarios/scene.json').write_text('{}')
        captured = {}

        def ready(app):
            try:
                app['actions']['open_yaml'](base / 'example.yaml', fresh_name=True)
                app['actions']['copy_workspace']()
                path = app['active']['config']
                self.assertEqual(path.parent, missions / 'roundabout/workspace')
                self.assertEqual(app['fields']['directory'].get(), 'missions/roundabout/workspace/scenarios')
                app['fields']['timeout_sec'].set('42')
                self.assertEqual(app['actions']['save'](), path)
                self.assertEqual(load_document(path)['default_case']['timeout_sec'], 42)
                captured['ok'] = True
            finally:
                app['root'].destroy()

        with patch.object(gui_module, 'MISSIONS', missions), patch.object(gui_module, 'ROOT', root_path):
            gui_module.main(on_ready=ready)
        self.assertTrue(captured.get('ok'))

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

    def test_video_mode_keeps_gui_visible_and_terminal_interrupt_stops_runner(self):
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
                app["fields"]["video_layout"].set("manual")
                root.after(50, app["actions"]["start"])

                def interrupt():
                    captured["window_state"] = root.state()
                    captured["compact"] = app["compact_state"]["enabled"]
                    captured["window_height"] = root.winfo_height()
                    captured["window_width"] = root.winfo_width()
                    captured["screen_width"] = root.winfo_screenwidth()
                    captured["expand_text"] = app["compact_button"].cget("text")
                    captured["expand_mapped"] = app["compact_button"].winfo_ismapped()
                    captured["saved_video"] = app["active"]["document"]["recording"]["video"]
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
                 patch("tkinter.messagebox.askokcancel", return_value=True), \
                 patch("tkinter.messagebox.askyesno", return_value=True):
                gui_module.main(on_ready=ready, runner_executable=executable)
            self.assertEqual(captured.get("window_state"), "normal")
            self.assertTrue(captured.get("compact"))
            self.assertLess(captured.get("window_height", 999), 200)
            self.assertLessEqual(captured.get("window_width", 9999),
                                 captured.get("screen_width", 0) * 0.6)
            self.assertEqual(captured.get("expand_text"), "전체 설정 보기")
            self.assertTrue(captured.get("expand_mapped"))
            self.assertEqual(captured.get("saved_video", {}).get("window_layout"), "split")
            self.assertFalse(captured.get("saved_video", {}).get("focus_planviz", True))
            self.assertIn("VIDEO_RUN_STARTED", captured.get("log", ""))

    def test_manual_monitor_mode_can_expand_back_to_full_settings(self):
        captured = {}

        def ready(app):
            root = app["root"]

            def compact():
                app["fields"]["video_layout"].set("manual")
                app["status"].set("주행 검증 실행 중: " + "very_long_campaign_name_" * 20)
                app["live_status"].set("현재 단계 / 시간 / 거리 / 속도 / 충돌 " * 25)
                app["compact_button"].invoke()
                root.after(350, inspect_compact)

            def inspect_compact():
                captured["compact_width"] = root.winfo_width()
                captured["expand_mapped"] = app["compact_button"].winfo_ismapped()
                captured["expand_button_bottom"] = (app["compact_button"].winfo_rooty()
                                                    + app["compact_button"].winfo_height()
                                                    - root.winfo_rooty())
                captured["compact_height"] = root.winfo_height()
                captured["video_layout"] = app["fields"]["video_layout"].get()
                app["compact_button"].invoke()
                root.after(150, inspect_expanded)

            def inspect_expanded():
                captured["expanded_width"] = root.winfo_width()
                captured["button_text"] = app["compact_button"].cget("text")
                root.destroy()

            root.after(50, compact)
            root.after(3000, root.destroy)

        gui_module.main(on_ready=ready)
        self.assertTrue(captured.get("expand_mapped"))
        self.assertLess(captured.get("expand_button_bottom", 9999),
                        captured.get("compact_height", 0))
        self.assertEqual(captured.get("video_layout"), "split")
        self.assertGreater(captured.get("expanded_width", 0),
                           captured.get("compact_width", 9999))
        self.assertEqual(captured.get("button_text"), "모니터 모드")

    def test_changed_executed_campaign_offers_new_name_and_preserves_old_yaml(self):
        with tempfile.TemporaryDirectory() as temp:
            captured = {}

            def ready(app):
                root = app["root"]

                def exercise():
                    app["active"]["template"]["output_root"] = temp
                    app["fields"]["campaign_name"].set("gui_clone_test")
                    first = app["actions"]["save"]()
                    original = first.read_bytes()
                    output = campaign_output(app["active"]["document"])
                    output.mkdir()
                    (output / "campaign_state.json").write_text("{}", encoding="utf-8")
                    app["fields"]["goal_x"].set("42.5")
                    second = app["actions"]["save"]()
                    captured.update(first=first, second=second, original=original,
                                    old_bytes=first.read_bytes(), new_doc=load_document(second),
                                    selected_name=app["fields"]["campaign_name"].get())
                    root.destroy()

                root.after(20, exercise)

            with patch.object(gui_module, "USERS", Path(temp) / "users"), \
                 patch("tkinter.messagebox.askyesno", return_value=True):
                gui_module.main(on_ready=ready)
            self.assertNotEqual(captured["first"], captured["second"])
            self.assertEqual(captured["old_bytes"], captured["original"])
            self.assertEqual(captured["new_doc"]["default_case"]["goal"]["x"], 42.5)
            self.assertEqual(captured["new_doc"]["campaign_name"], captured["selected_name"])

    def test_live_drive_metrics_are_visible_in_gui(self):
        with tempfile.TemporaryDirectory() as temp:
            captured = {}

            def ready(app):
                root = app["root"]

                def exercise():
                    self.assertEqual(app["active"]["source_path"].stem, "static_pair_mission_gt_split")
                    app["active"]["template"]["output_root"] = temp
                    app["fields"]["campaign_name"].set("gui_live_metrics")
                    config_path = app["actions"]["save"]()
                    state_path = campaign_output(app["active"]["document"]) / "campaign_state.json"
                    state_path.parent.mkdir()
                    state_path.write_text(json.dumps({
                        "config_path": str(config_path.resolve()), "total": 1,
                        "results": {}, "current": {"scenario": "test_scene", "phase": "DRIVING",
                            "metrics": {"elapsed_sec": 12.5, "goal_distance_m": 30.0,
                                        "speed_mps": 4.5, "collision_count": 0}}}), encoding="utf-8")
                    app["active"]["process"] = SimpleNamespace(poll=lambda: None)
                    app["actions"]["update_progress"]()
                    captured["status"] = app["status"].get()
                    captured["live"] = app["live_status"].get()
                    app["active"]["process"] = None
                    root.destroy()

                root.after(20, exercise)

            with patch.object(gui_module, "USERS", Path(temp) / "users"):
                gui_module.main(on_ready=ready)
            self.assertIn("test_scene / DRIVING", captured["status"])
            self.assertIn("12.5초", captured["live"])
            self.assertIn("30.0m", captured["live"])
            self.assertIn("4.5m/s", captured["live"])


if __name__ == "__main__":
    unittest.main()
