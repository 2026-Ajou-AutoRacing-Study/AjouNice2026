#!/usr/bin/env python3
"""Keep the shared driving view wide without changing its map or data inputs."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "rviz" / "kcity_aw149_planning.rviz"


class PlanningRvizDefaultsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG_PATH.read_text())
        cls.manager = cls.config["Visualization Manager"]

    def test_shared_launch_reads_the_tracked_driving_config(self):
        launch = ET.parse(PACKAGE_ROOT / "launch" / "aw149_planning_rviz.launch")
        config_arg = launch.find("arg[@name='rviz_config']")
        self.assertIsNotNone(config_arg)
        self.assertEqual(
            config_arg.attrib["default"],
            "$(arg repo_root)/base_ws/src/bringup/rviz/kcity_aw149_planning.rviz",
        )

    def test_wide_view_preserves_vehicle_following_and_top_down_orientation(self):
        view = self.manager["Views"]["Current"]
        self.assertEqual(view["Class"], "rviz/ThirdPersonFollower")
        self.assertEqual(view["Target Frame"], "base_link")
        self.assertEqual(view["Distance"], 150)
        self.assertAlmostEqual(view["Pitch"], 1.5697963237762451)
        self.assertAlmostEqual(view["Yaw"], 3.100398302078247)
        self.assertAlmostEqual(view["Field of View"], 0.7853981852531433)
        self.assertEqual(self.manager["Global Options"]["Fixed Frame"], "map")

    def test_map_route_and_live_trajectory_stay_enabled(self):
        by_name = {display["Name"]: display for display in self.manager["Displays"]}
        expected = {
            "Mission Route": ("Marker Topic", "/planning/mission_planning/route_marker"),
            "Final Trajectory": ("Marker Topic", "/planning/debug/final_trajectory/markers"),
            "Final Trajectory Path": ("Topic", "/planning/debug/final_trajectory/path"),
        }
        for name, (field, topic) in expected.items():
            with self.subTest(display=name):
                self.assertTrue(by_name[name]["Enabled"])
                self.assertEqual(by_name[name][field], topic)
        maps = [
            display for display in self.manager["Displays"]
            if display.get("Marker Topic") == "/map/lanelet_map"
        ]
        self.assertEqual(len(maps), 1)
        self.assertTrue(maps[0]["Enabled"])

    def test_speed_overlay_uses_competition_without_ego_fallback(self):
        launch = ET.parse(PACKAGE_ROOT / "launch" / "aw149_planning_rviz.launch")
        self.assertEqual(launch.find("arg[@name='vehicle_status_topic']").get('default'), '/Competition_topic')
        overlay = launch.find("node[@name='planning_vehicle_overlay']")
        self.assertEqual(overlay.find("param[@name='vehicle_status_topic']").get('value'), '$(arg vehicle_status_topic)')

    def test_no_gt_recording_gate_is_forwarded_to_all_topic_recorder(self):
        launch = ET.parse(PACKAGE_ROOT / "launch" / "aw149_planning_rviz.launch")
        recorder = launch.find("node[@name='aw149_planning_rviz']")
        self.assertEqual(
            recorder.find("param[@name='record_morai_gt']").get('value'),
            '$(arg show_morai_gt)',
        )
        self.assertIsNone(recorder.find("param[@name='rosbag_topics']"))


if __name__ == "__main__":
    unittest.main()
