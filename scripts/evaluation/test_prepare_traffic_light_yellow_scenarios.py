import copy
import json
from pathlib import Path
import tempfile
import unittest

from prepare_traffic_light_yellow_scenarios import prepare, prepare_campaigns, shifted_scene, validate_scene


def scene():
    return {
        "egoVehicle": {"initPosition": {"pos": {"x": -131.486}, "gear": 1}},
        "tlList": [{"index": 0, "state": 2, "deltaTime": 7.044259071350098},
                   {"index": 2, "state": 0, "deltaTime": 7.044259071350098}],
        "mapInfo": {"mapName": "R_KR_PR_K-city_2025"},
        "vehicleList": [], "weather": {"hour": 13},
    }


class PreparationTest(unittest.TestCase):
    def test_campaigns_keep_full_route_and_pair_policy(self):
        import yaml
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenarios = root / 'artifacts' / 'scenarios'
            source = root / 'source.json'
            source.write_text(json.dumps(scene()))
            prepare(source, scenarios, allowed_root=root / 'artifacts')
            base = dict(campaign_name='original', scenarios={},
                        stack=dict(traffic_light_profile='legacy'),
                        default_case=dict(goal=dict(x=1), waypoints=[dict(x=2)], timeout_sec=600),
                        recording=dict(selected_topics=['/Ego_topic', '/Competition_topic', '/rosout',
                                        '/perception/traffic_light_recognition/map_rois',
                                        '/sensor/camera/front/compressed', '/evaluation/fixture'],
                                       video=dict(enabled=False)), upload=dict(enabled=True))
            base_path = root / 'base.yaml'
            base_path.write_text(yaml.safe_dump(base))
            original = base_path.read_bytes()
            output = root / 'artifacts' / 'configs'
            receipt = prepare_campaigns(base_path, scenarios, output, allowed_root=root / 'artifacts')
            self.assertEqual(base_path.read_bytes(), original)
            self.assertEqual(len(receipt['campaigns']), 3)
            for row in receipt['campaigns']:
                doc = yaml.safe_load((output / row['config']).read_text())
                self.assertEqual(doc['default_case'], base['default_case'])
                self.assertEqual(len(doc['scenarios']['include']), 1)
                self.assertTrue(doc['stack']['traffic_light_yellow_passthrough'])
                self.assertTrue(doc['stack']['preserve_unreadable_color_evidence'])
                self.assertEqual(doc['stack']['signal_compliance_unknown_grace_sec'], .3)
                self.assertEqual(doc['stack']['signal_compliance_yellow_deceleration_mps2'], 8)
                self.assertEqual(doc['recording']['selected_topics'], ['/Ego_topic', '/Competition_topic', '/rosout'])
                self.assertTrue(doc['recording']['video']['enabled'])
                self.assertEqual(doc['recording']['video']['window_layout'], 'split')
                self.assertFalse(doc['upload']['enabled'])
                native = json.loads((output / row['native_plan']).read_text())
                for group in native['groups']:
                    for phase in group['phases']:
                        if phase['color'] in (4, 5):
                            self.assertEqual(phase['seconds'], row['requested_yellow_seconds'])
            with self.assertRaises(FileExistsError):
                prepare_campaigns(base_path, scenarios, output, allowed_root=root / 'artifacts')

    def test_only_saved_clock_changes_and_source_is_not_mutated(self):
        source = scene()
        original = copy.deepcopy(source)
        shifted = shifted_scene(source, 8)
        self.assertEqual(source, original)
        self.assertEqual(shifted["tlList"][0]["deltaTime"], source["tlList"][0]["deltaTime"] + 8)
        for before, after in zip(source["tlList"], shifted["tlList"]):
            self.assertEqual(before["index"], after["index"])
            self.assertEqual(before["state"], after["state"])
        shifted["tlList"] = source["tlList"]
        self.assertEqual(shifted, source)

    def test_unknown_duration_schema_is_not_silently_accepted(self):
        source = scene()
        source["tlList"][0]["yellowDuration"] = 2
        with self.assertRaises(ValueError):
            validate_scene(source)

    def test_invalid_clocks_states_and_indices_are_rejected(self):
        for key, value in (("index", True), ("state", -1), ("state", 2.5),
                           ("deltaTime", float("nan")), ("deltaTime", -1),
                           ("deltaTime", True)):
            source = scene()
            source["tlList"][0][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_scene(source)
        source = scene()
        source["tlList"][1]["index"] = 0
        with self.assertRaises(ValueError):
            validate_scene(source)

    def test_invalid_offset_is_rejected(self):
        for offset in (-1, 21, float("inf"), True, "8"):
            with self.subTest(offset=offset), self.assertRaises(ValueError):
                shifted_scene(scene(), offset)

    def test_fresh_receipted_three_variants_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.json"
            original = json.dumps(scene()).encode()
            source.write_bytes(original)
            output = root / "artifacts" / "scenarios"
            receipt = prepare(source, output, allowed_root=root / "artifacts")
            self.assertEqual(source.read_bytes(), original)
            self.assertEqual(len(receipt["variants"]), 3)
            self.assertFalse(receipt["shorter_period_applied"])
            self.assertFalse(receipt["native_phase_durations_in_scene"])
            self.assertFalse(receipt["simulator_loaded"])
            self.assertEqual([row["delta_time_added_sec"] for row in receipt["variants"]], [0, 8, 16])
            for row in receipt["variants"]:
                self.assertTrue((output / row["file"]).is_file())
            with self.assertRaises(FileExistsError):
                prepare(source, output, allowed_root=root / "artifacts")

    def test_invalid_all_variants_and_escape_have_no_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.json"
            source.write_text(json.dumps(scene()))
            output = root / "artifacts" / "scenarios"
            for offsets in ((0, 8, 21), (0, 0), (), tuple(range(13))):
                with self.subTest(offsets=offsets), self.assertRaises(ValueError):
                    prepare(source, output, offsets, allowed_root=root / "artifacts")
                self.assertFalse(output.exists())
            with self.assertRaises(ValueError):
                prepare(source, root / "outside", allowed_root=root / "artifacts")
            self.assertFalse((root / "outside").exists())


if __name__ == "__main__":
    unittest.main()
