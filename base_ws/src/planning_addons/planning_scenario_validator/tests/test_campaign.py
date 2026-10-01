"""실제 MORAI를 건드리지 않고 config와 시나리오 사본 계약을 검증한다."""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from planning_scenario_validator.config import ConfigError, ROOT, load_campaign
from planning_scenario_validator.runtime import prepare_scenario
from planning_scenario_validator.runner import _recover_interrupted


EXAMPLE = ROOT / "base_ws/src/planning_addons/planning_scenario_validator/config/examples/static_obstacle_gt.yaml"


class CampaignTest(unittest.TestCase):
    def test_example_selects_two_cases_and_local_gt_profile(self):
        cfg = load_campaign(EXAMPLE)
        self.assertEqual(len(cfg["cases"]), 2)
        self.assertEqual(cfg["stack"]["detector"], "od_gt")
        self.assertIn("/CollisionData", cfg["recording"]["topics"])
        self.assertTrue(all(c.timeout_sec == 180 for c in cfg["cases"]))
        network = json.loads(cfg["network"]["ego_network_json"].read_text(encoding="utf-8"))
        entries = network["egoNetworkData"][0]["listEgoNetworkInfo"]
        self.assertEqual(sum(e.get("config", {}).get("udpConfig", {}).get("hostPort") == 9095 for e in entries), 1)

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


def cfg_case():
    return ROOT / "configs/scenario/hidden_mission/static_3/s3_01_three_vehicles_gap10.json"


if __name__ == "__main__":
    unittest.main()
