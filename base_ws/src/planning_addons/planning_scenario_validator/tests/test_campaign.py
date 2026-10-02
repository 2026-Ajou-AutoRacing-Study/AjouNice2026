"""실제 MORAI를 건드리지 않고 config와 시나리오 사본 계약을 검증한다."""
import json
import sys
import tempfile
import threading
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from build_static_corridor_cases import build as build_static_corridor_cases
from planning_scenario_validator.config import ConfigError, ROOT, load_campaign
from planning_scenario_validator.runtime import (LiveMonitor, arrange_split_windows,
                                                 focus_planviz, prepare_scenario, video_size)
from arrange_split_windows import choose_windows
from planning_scenario_validator.runner import _recover_interrupted


EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_gt.yaml"
BEVFUSION_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_bevfusion.yaml"
BEVFUSION_CLEAR_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/bevfusion_clear_smoke.yaml"
SPLIT_EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_pair_mission_gt_split.yaml"


class CampaignTest(unittest.TestCase):
    def test_split_example_records_full_screen_without_planviz_maximize(self):
        cfg = load_campaign(SPLIT_EXAMPLE)
        video = cfg["recording"]["video"]
        self.assertEqual(len(cfg["cases"]), 1)
        self.assertEqual(video["window_layout"], "split")
        self.assertEqual(video["bottom_bar_px"], 150)
        self.assertFalse(video["focus_planviz"])

    def test_split_window_picker_ignores_morai_launcher_terminal(self):
        def window(name, class_name):
            return SimpleNamespace(get_name=lambda: name,
                                   get_class_group_name=lambda: class_name)
        terminal = window("luv@pc: ~/MoraiLauncher_Lin", "Terminator")
        simulator = window("Simulator", "Simulator.x86_64")
        rviz = window("kcity_aw149_planning - RViz", "Rviz")
        self.assertEqual(choose_windows([terminal, simulator, rviz]), (rviz, simulator))

    def test_split_layout_uses_video_display(self):
        with patch("planning_scenario_validator.runtime.subprocess.run",
                   return_value=SimpleNamespace(returncode=0, stderr="", stdout="")) as run:
            arrange_split_windows(bottom_bar_px=150, display=":9.0")
        self.assertEqual(run.call_args.kwargs["env"]["DISPLAY"], ":9.0")
        self.assertEqual(run.call_args.args[0][-1], "150")

    def test_example_selects_two_cases_and_local_gt_profile(self):
        cfg = load_campaign(EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 2)
        self.assertEqual(cfg["stack"]["detector"], "od_gt")
        self.assertIn("/CollisionData", cfg["recording"]["topics"])
        self.assertTrue(all(c.timeout_sec == 180 for c in cfg["cases"]))
        network = json.loads(cfg["network"]["ego_network_json"].read_text(encoding="utf-8"))
        entries = network["egoNetworkData"][0]["listEgoNetworkInfo"]
        self.assertEqual(sum(e.get("config", {}).get("udpConfig", {}).get("hostPort") == 9095 for e in entries), 1)

    def test_bevfusion_example_uses_four_cameras_lidar_and_sensor_recording(self):
        cfg = load_campaign(BEVFUSION_EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 1)
        self.assertEqual(cfg["stack"]["detector"], "bevfusion")
        self.assertEqual(cfg["network"]["camera_count"], 4)
        self.assertTrue(cfg["network"]["has_lidar"])
        self.assertIn("/sensor/lidar1", cfg["recording"]["topics"])

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
                self.assertTrue((output / (stem + ".case.yaml")).is_file())
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
                self.assertTrue((output / (stem + ".case.yaml")).is_file())
            wide_case = (output / "soa_pair_03_drum_barrel_separated.case.yaml").read_text()
            self.assertIn("x: 71.704", wide_case)
            self.assertIn("timeout_sec: 100", wide_case)
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
