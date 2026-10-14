import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest

HERE = pathlib.Path(__file__).parent
sys.path.insert(0, str(HERE))
SPEC = importlib.util.spec_from_file_location("auto_probe", HERE / "run_lidar_packet_probe_auto.py")
auto = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(auto)


class AutoProbeTests(unittest.TestCase):
    def test_prepare_does_not_run_ros_and_preserves_sources(self):
        before = {path: path.read_bytes() for path in auto.BENCHMARK.glob("*.json")}
        with tempfile.TemporaryDirectory() as temp:
            output = pathlib.Path(temp) / "setup"
            configs = auto.prepare(output)
            self.assertEqual(set(configs), {"highway", "roundabout"})
            topics = json.loads((output / "topics.json").read_text())
            self.assertTrue(set(auto.probe.REQUIRED) <= set(topics))
            for path in configs.values():
                config = auto.yaml.safe_load(pathlib.Path(path).read_text())
                self.assertEqual(config["stack"]["detector"], "od_gt")
                self.assertEqual(config["stack"]["pose_source"], "maptf_morai")
                self.assertFalse(config["stack"]["traffic_light"])
                self.assertFalse(config["stack"]["planviz"])
                self.assertFalse(config["upload"]["enabled"])
                self.assertTrue(config["recording"]["include_sensors"])
                self.assertFalse(config["recording"]["video"]["enabled"])
                self.assertEqual(len(config["scenarios"]["include"]), 1)
            with self.assertRaises(FileExistsError):
                auto.prepare(output)
        self.assertEqual(before, {path: path.read_bytes() for path in auto.BENCHMARK.glob("*.json")})

    def test_validator_configuration_accepts_both_cases(self):
        from planning_scenario_validator.config import load_campaign
        with tempfile.TemporaryDirectory() as temp:
            configs = auto.prepare(pathlib.Path(temp) / "setup")
            for path in configs.values():
                config = load_campaign(path)
                self.assertEqual(len(config["cases"]), 1)
                self.assertIn("/sensor/lidar1/packets", config["recording"]["topics"])


if __name__ == "__main__":
    unittest.main()
