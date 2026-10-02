"""실제 MORAI를 건드리지 않고 config와 시나리오 사본 계약을 검증한다."""
import json
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from build_static_corridor_cases import build as build_static_corridor_cases
from planning_scenario_validator.config import (AJOU_DRIVE_ROOT_FOLDER_ID, ConfigError,
                                                DEFAULT_UPLOAD_REMOTE, ROOT, load_campaign,
                                                resolve_path)
from planning_scenario_validator.layout import normalize_bar_setting, resolve_bar_height
from planning_scenario_validator.runtime import (LiveMonitor, arrange_split_windows,
                                                 focus_planviz, prepare_scenario, video_size)
from arrange_split_windows import choose_windows, clear_maximized_axes, split_rectangles
from place_monitor_bar import find_gui_window
from planning_scenario_validator.runner import (_recover_interrupted, _upload,
                                                _upload_commands, _write_state)


EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_gt.yaml"
BEVFUSION_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_bevfusion.yaml"
BEVFUSION_CLEAR_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/bevfusion_clear_smoke.yaml"
SPLIT_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_gt_split.yaml"


class CampaignTest(unittest.TestCase):
    def test_example_paths_follow_each_users_repository(self):
        examples = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples"
        for path in examples.glob("*.yaml"):
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            paths = [document["scenarios"]["directory"], document["recording"]["topic_profile"],
                     document["network"]["sensor_json"], document["network"]["ego_network_json"],
                     document["output_root"]]
            with self.subTest(example=path.name):
                self.assertTrue(all(not Path(value).is_absolute() for value in paths))
                self.assertEqual(Path(document["recording"]["topic_profile"]).name, "basic.json")
                self.assertFalse(document["recording"]["include_sensors"])
                self.assertTrue(document["recording"]["video"]["enabled"])
        with tempfile.TemporaryDirectory() as directory:
            teammate_root = Path(directory) / "AjouNice2026"
            with patch("planning_scenario_validator.config.ROOT", teammate_root):
                self.assertEqual(resolve_path("configs/sensors/Sensor_competition_local.json"),
                                 teammate_root / "configs/sensors/Sensor_competition_local.json")

    def test_default_drive_destination_uses_shared_ajounice_folder(self):
        cfg = load_campaign(SPLIT_EXAMPLE, check_install=False)
        self.assertFalse(cfg["upload"]["enabled"])
        self.assertEqual(cfg["upload"]["rclone_remote"], DEFAULT_UPLOAD_REMOTE)
        self.assertEqual(cfg["upload"]["drive_root_folder_id"], AJOU_DRIVE_ROOT_FOLDER_ID)
        case = cfg["output"] / "success" / "case_01"
        case_command, metadata = _upload_commands(cfg, case)
        for command in [case_command] + metadata:
            self.assertIn("--drive-root-folder-id", command)
            self.assertEqual(command[command.index("--drive-root-folder-id") + 1],
                             AJOU_DRIVE_ROOT_FOLDER_ID)
        self.assertEqual(case_command[3],
                         "ajou_drive:planning_validator/{}/success/case_01".format(cfg["name"]))

    def test_invalid_upload_profile_is_rejected_before_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            document = yaml.safe_load(SPLIT_EXAMPLE.read_text(encoding="utf-8"))
            document["upload"]["profile"] = "surprise"
            path = Path(directory) / "invalid_upload.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "upload.profile"):
                load_campaign(path, check_install=False)

    def test_upload_profiles_keep_category_and_choose_large_files(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "success" / "case_01"
            for profile, has_bag, has_video in (("basic", False, False),
                                                 ("bag", True, False),
                                                 ("video", False, True),
                                                 ("full", True, True)):
                cfg = {"upload": {"rclone_remote": "team:planning", "profile": profile},
                       "name": "campaign", "output": Path(directory)}
                case_command, metadata = _upload_commands(cfg, folder)
                self.assertEqual(case_command[3], "team:planning/campaign/success/case_01")
                self.assertEqual("*.bag" not in case_command, has_bag)
                self.assertEqual("*.mp4" not in case_command, has_video)
                self.assertEqual([command[3].rsplit("/", 1)[-1] for command in metadata],
                                 ["summary.csv", "campaign_config.yaml"])

    @unittest.skipUnless(shutil.which("rclone"), "로컬 rclone 필터 테스트를 건너뜁니다")
    def test_rclone_upload_profiles_copy_only_selected_media(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root / "local" / "success" / "case_01"
            folder.mkdir(parents=True)
            for name in ("result.json", "events.jsonl", "drive.bag", "screen.mp4"):
                (folder / name).write_text(name, encoding="utf-8")
            for profile, expected in (
                    ("basic", {"result.json", "events.jsonl"}),
                    ("bag", {"result.json", "events.jsonl", "drive.bag"}),
                    ("video", {"result.json", "events.jsonl", "screen.mp4"}),
                    ("full", {"result.json", "events.jsonl", "drive.bag", "screen.mp4"})):
                cfg = {"upload": {"rclone_remote": str(root / profile), "profile": profile},
                       "name": "campaign", "output": root / "local"}
                case_command, _ = _upload_commands(cfg, folder)
                subprocess.run(case_command, check=True, capture_output=True, text=True)
                copied = root / profile / "campaign" / "success" / "case_01"
                self.assertEqual({path.name for path in copied.iterdir()}, expected)

    def test_upload_failure_keeps_local_result_and_attempts_csv_and_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = root / "failure" / "case_01"
            folder.mkdir(parents=True)
            cfg = {"upload": {"enabled": True, "profile": "basic",
                              "rclone_remote": "team:planning"},
                   "name": "campaign", "output": root}
            state = {"results": {}, "upload_errors": []}
            calls = []

            def fake_run(command, **_kwargs):
                calls.append(command)
                return SimpleNamespace(returncode=1 if len(calls) == 1 else 0,
                                       stderr="simulated upload failure")

            with patch("planning_scenario_validator.runner.shutil.which", return_value="/usr/bin/rclone"), \
                 patch("planning_scenario_validator.runner.subprocess.run", side_effect=fake_run):
                _upload(cfg, folder, state, SimpleNamespace(name="case_01"))
            self.assertEqual(len(calls), 4)
            self.assertTrue(folder.is_dir())
            self.assertEqual(len(state["upload_errors"]), 1)
            self.assertTrue((root / "summary.csv").is_file())
            self.assertEqual(calls[-1][3], "team:planning/campaign/campaign_state.json")

    def test_shared_summary_uses_relative_result_path(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            result = root / "success" / "case_01" / "result.json"
            result.parent.mkdir(parents=True)
            result.write_text("{}", encoding="utf-8")
            state = {"results": {"case_01": {"status": "PASS", "reason": "SUCCESS",
                                               "result_path": str(result), "attempts": 1}}}
            _write_state({"output": root}, state)
            self.assertIn("success/case_01/result.json", (root / "summary.csv").read_text(encoding="utf-8"))
            self.assertNotIn(str(root), (root / "summary.csv").read_text(encoding="utf-8"))

    def test_split_example_records_full_screen_without_planviz_maximize(self):
        cfg = load_campaign(SPLIT_EXAMPLE)
        video = cfg["recording"]["video"]
        self.assertEqual(len(cfg["cases"]), 1)
        self.assertEqual(video["window_layout"], "split")
        self.assertEqual(video["bottom_bar_px"], "auto")
        self.assertFalse(video["focus_planviz"])

    def test_monitor_bar_auto_percent_and_legacy_pixels(self):
        self.assertEqual(resolve_bar_height(741, "auto"), 170)
        self.assertEqual(resolve_bar_height(1053, "auto"), 179)
        self.assertEqual(resolve_bar_height(1413, "auto"), 240)
        self.assertEqual(resolve_bar_height(2133, "auto"), 275)
        self.assertEqual(resolve_bar_height(1053, "20%"), 211)
        self.assertEqual(resolve_bar_height(1053, 150), 150)
        self.assertEqual(normalize_bar_setting("14.0%"), "14%")
        for value in ("9%", "31%", "nan", "401", "140.5", "oops"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                normalize_bar_setting(value)

    def test_invalid_monitor_bar_setting_is_rejected_before_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            document = yaml.safe_load(SPLIT_EXAMPLE.read_text(encoding="utf-8"))
            document["recording"]["video"]["bottom_bar_px"] = "75%"
            path = Path(directory) / "invalid_bar.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ConfigError, "bottom_bar_px"):
                load_campaign(path, check_install=False)

    def test_split_window_picker_ignores_morai_launcher_terminal(self):
        def window(name, class_name):
            return SimpleNamespace(get_name=lambda: name,
                                   get_class_group_name=lambda: class_name)
        terminal = window("luv@pc: ~/MoraiLauncher_Lin", "Terminator")
        simulator = window("Simulator", "Simulator.x86_64")
        rviz = window("kcity_aw149_planning - RViz", "Rviz")
        self.assertEqual(choose_windows([terminal, simulator, rviz]), (rviz, simulator))

    def test_monitor_bar_uses_unique_gui_title_not_missing_tk_pid(self):
        first = SimpleNamespace(get_name=lambda: "AjouNice 주행 시나리오 검증기 [111]",
                                get_pid=lambda: 0)
        second = SimpleNamespace(get_name=lambda: "AjouNice 주행 시나리오 검증기 [222]",
                                 get_pid=lambda: 0)
        self.assertIs(find_gui_window([first, second], second.get_name()), second)

    def test_split_rectangles_respect_desktop_dock_and_bottom_bar(self):
        area = SimpleNamespace(x=72, y=27, width=1848, height=1053)
        self.assertEqual(split_rectangles(area, 150),
                         ((72, 27, 924, 903), (996, 27, 924, 1053)))

    def test_split_clears_vertical_only_maximize(self):
        calls = []
        window = SimpleNamespace(
            is_maximized_horizontally=lambda: False,
            is_maximized_vertically=lambda: True,
            is_fullscreen=lambda: False,
            unmaximize_horizontally=lambda: calls.append("horizontal"),
            unmaximize_vertically=lambda: calls.append("vertical"),
            set_fullscreen=lambda value: calls.append("fullscreen"))
        clear_maximized_axes(window)
        self.assertEqual(calls, ["vertical"])

    def test_split_layout_uses_video_display(self):
        with patch("planning_scenario_validator.runtime.subprocess.run",
                   return_value=SimpleNamespace(returncode=0, stderr="", stdout="")) as run:
            arrange_split_windows(bottom_bar_px=150, display=":9.0")
        self.assertEqual(run.call_args.kwargs["env"]["DISPLAY"], ":9.0")
        self.assertEqual(run.call_args.args[0][-2:], ["150", "0.0"])

    def test_split_layout_waits_for_first_rviz_config_restore(self):
        with patch("planning_scenario_validator.runtime.subprocess.run",
                   return_value=SimpleNamespace(returncode=0, stderr="", stdout="split layout")) as run:
            detail = arrange_split_windows(bottom_bar_px="auto", startup_grace_sec=6.0)
        self.assertEqual(run.call_args.args[0][-2:], ["auto", "6.0"])
        self.assertEqual(detail, "split layout")

    def test_example_selects_two_cases_and_local_gt_profile(self):
        cfg = load_campaign(EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 2)
        self.assertEqual(cfg["stack"]["detector"], "od_gt")
        self.assertIn("/CollisionData", cfg["recording"]["topics"])
        self.assertTrue(all(c.timeout_sec == 75 for c in cfg["cases"]))
        network = json.loads(cfg["network"]["ego_network_json"].read_text(encoding="utf-8"))
        entries = network["egoNetworkData"][0]["listEgoNetworkInfo"]
        self.assertEqual(sum(e.get("config", {}).get("udpConfig", {}).get("hostPort") == 9095 for e in entries), 1)

    def test_bevfusion_example_uses_four_cameras_lidar_without_sensor_recording(self):
        cfg = load_campaign(BEVFUSION_EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 1)
        self.assertEqual(cfg["stack"]["detector"], "bevfusion")
        self.assertEqual(cfg["network"]["camera_count"], 4)
        self.assertTrue(cfg["network"]["has_lidar"])
        self.assertNotIn("/sensor/lidar1", cfg["recording"]["topics"])

    def test_bevfusion_clear_smoke_omits_raw_sensors_from_bag(self):
        cfg = load_campaign(BEVFUSION_CLEAR_EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 1)
        self.assertEqual(cfg["stack"]["detector"], "bevfusion")
        self.assertFalse(cfg["recording"]["include_sensors"])
        self.assertNotIn("/sensor/lidar1", cfg["recording"]["topics"])

    def test_static_corridor_cases_cover_catalog_and_pair_controls(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            build_static_corridor_cases(output=output)
            for stem, count in (("soa_corridor_00_clear", 0),
                                ("soa_corridor_01_cargobox", 1),
                                ("soa_corridor_03_mixed", 3),
                                ("soa_corridor_06_mixed", 6),
                                ("soa_official_00_clear", 0),
                                ("soa_official_01_cargobox_native", 1),
                                ("soa_official_01_cargobox_scaled", 1)):
                scene = json.loads((output / (stem + ".json")).read_text())
                self.assertEqual(len(scene["objectList"]), count)
                self.assertEqual(scene["shadedAreaList"], [])
                self.assertFalse((output / (stem + ".case.yaml")).exists())
            sample = json.loads((output / "soa_official_01_cargobox_scaled.json").read_text())
            self.assertEqual(sample["objectList"][0]["scale"]["x"], 2.0)
            self.assertEqual(sample["objectList"][0]["scale"]["y"], 3.0)
            expected_assets = {"pe_drum": 40100027, "traffic_barrel": 40100005,
                               "concrete_barricade": 40100047, "pe_firewall_orange": 40100028}
            for name, data_id in expected_assets.items():
                for suffix, dx in (("sample_pose", 0.0), ("pathward_080", 0.8)):
                    path = output / ("soa_catalog_{}_{}.json".format(name, suffix))
                    catalog_scene = json.loads(path.read_text())
                    self.assertEqual(catalog_scene["shadedAreaList"], [])
                    self.assertEqual(len(catalog_scene["objectList"]), 1)
                    obj = catalog_scene["objectList"][0]
                    self.assertEqual(obj["DataID"], data_id)
                    self.assertAlmostEqual(obj["pos"]["x"], -60.61000061035156 + dx)
                    self.assertEqual(obj["scale"]["x"], 1.0)
                    self.assertEqual(catalog_scene["egoVehicle"]["initPosition"]["pos"]["x"], -63.195)
            for stem, ids in (("soa_pair_01_drum_barrel_zigzag", [40100027, 40100005]),
                              ("soa_pair_02_barrel_drum_zigzag", [40100005, 40100027]),
                              ("soa_pair_03_drum_barrel_separated", [40100027, 40100005]),
                              ("soa_pair_04_drum_barrel_valid_start", [40100027, 40100005]),
                              ("soa_pair_05_drum_barrel_alternate_side", [40100027, 40100005])):
                pair = json.loads((output / (stem + ".json")).read_text())
                self.assertEqual([item["DataID"] for item in pair["objectList"]], ids)
                self.assertEqual([item["UNIQUEID"] for item in pair["objectList"]], [2, 3])
                self.assertEqual(pair["shadedAreaList"], [])
                self.assertFalse((output / (stem + ".case.yaml")).exists())
            metadata = json.loads((output / "metadata/manifest.json").read_text())
            wide_case = metadata["soa_pair_03_drum_barrel_separated"]
            self.assertEqual(wide_case["reference_goal"]["x"], 71.704)
            self.assertEqual(wide_case["reference_timeout_sec"], 50)
            for index, (stem, data_id) in enumerate((("soa_pair_control_01_early_drum", 40100027),
                                  ("soa_pair_control_02_early_barrel", 40100005),
                                  ("soa_pair_control_03_late_barrel", 40100005),
                                  ("soa_pair_control_04_mid_drum_long_goal", 40100027),
                                  ("soa_pair_control_05_mid_barrel_long_goal", 40100005),
                                  ("soa_pair_control_06_mid_barrel_left", 40100005),
                                  ("soa_pair_control_07_mid_drum_right", 40100027)), 1):
                folder = output / "mid_controls" if index >= 4 else output
                control = json.loads((folder / (stem + ".json")).read_text())
                self.assertEqual(len(control["objectList"]), 1)
                self.assertEqual(control["objectList"][0]["DataID"], data_id)
                self.assertEqual(control["shadedAreaList"], [])

    def test_scenario_staging_preserves_source_and_installs_network_sidecar(self):
        cfg = load_campaign(EXAMPLE)
        case = cfg["cases"][0]
        original = case.source.read_bytes()
        with tempfile.TemporaryDirectory() as scratch:
            output, ego = prepare_scenario(case, Path(scratch), cfg["network"]["ego_network_json"])
            scene = json.loads(output.read_text(encoding="utf-8"))
            sidecar = Path(scratch) / "EgoNetwork" / (case.install_stem + "_MN.json")
            self.assertEqual(scene["egoVehicle"]["currentControlMode"], 2)
            self.assertEqual(scene["egoVehicle"]["currentVelocity"], 0)
            self.assertEqual(ego[0], scene["egoVehicle"]["initPosition"]["pos"]["x"])
            self.assertTrue(sidecar.is_file())
            self.assertEqual(case.source.read_bytes(), original)

    def test_missing_goal_is_rejected(self):
        from planning_scenario_validator.config import _load_case
        with self.assertRaises(ConfigError):
            _load_case(cfg_case(), {"timeout_sec": 60})

    def test_crashed_running_directory_is_preserved_for_resume(self):
        with tempfile.TemporaryDirectory() as scratch:
            output = Path(scratch)
            unfinished = output / "running" / "scene_20261002_001"
            unfinished.mkdir(parents=True)
            (unfinished / "video.mp4").write_bytes(b"partial")
            _recover_interrupted({"output": output})
            rescued = output / "error" / unfinished.name
            self.assertEqual((rescued / "video.mp4").read_bytes(), b"partial")
            self.assertEqual(json.loads((rescued / "interrupted.json").read_text())["reason"], "PROCESS_INTERRUPTED")

    def test_same_spawn_requires_observable_reload_not_just_fresh_ego(self):
        monitor = LiveMonitor.__new__(LiveMonitor)
        monitor.ego = SimpleNamespace(position=SimpleNamespace(x=53.08, y=-466.31))
        monitor.stamps = {"ego": 10.1}
        monitor.last_ego_gap_at = 0.0
        expected = (53.08, -466.31)
        self.assertFalse(monitor.scenario_confirmed(
            expected, after=10.0, baseline_xy=expected, tolerance=2.0))
        monitor.last_ego_gap_at = 10.2
        monitor.stamps["ego"] = 10.3
        self.assertTrue(monitor.scenario_confirmed(
            expected, after=10.0, baseline_xy=expected, tolerance=2.0))
        monitor.last_ego_gap_at = 0.0
        self.assertTrue(monitor.scenario_confirmed(
            expected, after=10.0, baseline_xy=(-144.0, -452.0), tolerance=2.0))

    def test_repeated_contact_with_one_object_counts_once(self):
        monitor = LiveMonitor.__new__(LiveMonitor)
        monitor.armed = True
        monitor.collision_ids = set()
        monitor.seen_collision_ids = set()
        monitor.collision_count = 0
        monitor.stamps = {}
        events = []
        monitor.event = lambda name, **fields: events.append((name, fields))
        contact = SimpleNamespace(collision_object=[SimpleNamespace(unique_id=4101)])
        empty = SimpleNamespace(collision_object=[])
        monitor._collision(contact)
        monitor._collision(empty)
        monitor._collision(contact)
        self.assertEqual(monitor.collision_count, 1)
        self.assertEqual(events, [("COLLISION", {"object_id": 4101})])

    def test_planviz_focus_retries_transient_wnck_crash(self):
        with patch("planning_scenario_validator.runtime.subprocess.run", side_effect=[
                SimpleNamespace(returncode=-11, stderr="", stdout=""),
                SimpleNamespace(returncode=0, stderr="", stdout="")]) as run:
            with patch("planning_scenario_validator.runtime.time.sleep"):
                focus_planviz(timeout=2)
        self.assertEqual(run.call_count, 2)

    def test_video_tools_use_configured_display_not_calling_shell_display(self):
        with patch("planning_scenario_validator.runtime.subprocess.check_output",
                   return_value="  dimensions:    1920x1080 pixels\n") as check:
            self.assertEqual(video_size(":9.0"), "1920x1080")
        self.assertEqual(check.call_args.args[0], ["xdpyinfo", "-display", ":9.0"])
        with patch("planning_scenario_validator.runtime.subprocess.run",
                   return_value=SimpleNamespace(returncode=0, stderr="", stdout="")) as run:
            focus_planviz(timeout=2, display=":9.0")
        self.assertEqual(run.call_args.kwargs["env"]["DISPLAY"], ":9.0")

    def test_live_evidence_records_motion_plan_and_control(self):
        monitor = LiveMonitor.__new__(LiveMonitor)
        monitor.evidence_lock = threading.Lock()
        monitor.stamps = {}
        monitor.odom = None
        monitor.recent_odom = deque(maxlen=300)
        monitor.start_driving(time.monotonic())

        def odom(x, speed):
            return SimpleNamespace(
                pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0))),
                twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=speed))))

        monitor._odom(odom(0.0, 0.0))
        monitor._trajectory(SimpleNamespace(points=[
            SimpleNamespace(longitudinal_velocity_mps=3.0),
            SimpleNamespace(longitudinal_velocity_mps=4.0)]))
        monitor._control(SimpleNamespace(accel=0.3, brake=0.0))
        monitor._detections(SimpleNamespace(objects=[]))
        monitor._detections(SimpleNamespace(objects=[object(), object()]))
        monitor._odom(odom(1.2, 2.0))
        evidence = monitor.drive_evidence()
        self.assertGreaterEqual(evidence["max_displacement_m"], 1.2)
        self.assertEqual(evidence["trajectory_near_max_speed_mps"], 4.0)
        self.assertEqual(evidence["control_command_count"], 1)
        self.assertEqual(evidence["max_accel_command"], 0.3)
        self.assertEqual(evidence["detection_message_count"], 2)
        self.assertEqual(evidence["detection_nonempty_message_count"], 1)
        self.assertEqual(evidence["detection_max_objects"], 2)
        self.assertEqual(evidence["latest_detection_objects"], 2)


def cfg_case():
    return ROOT / "configs/scenario/hidden_mission/static_3/s3_01_three_vehicles_gap10.json"


if __name__ == "__main__":
    unittest.main()
