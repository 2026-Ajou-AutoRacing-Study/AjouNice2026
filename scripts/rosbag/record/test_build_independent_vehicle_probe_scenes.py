import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import build_independent_vehicle_probe_scenes as probe


class IndependentVehicleScenesTests(unittest.TestCase):
    def setUp(self):
        self.assets = probe.asset_names(json.loads(probe.CATALOG.read_text()))
        self.source = json.loads((probe.BENCHMARK / "bench_highway_merge_raw.json").read_text())

    def test_three_distinct_parameters_preserve_every_geometry_field(self):
        generated = []
        for profile in probe.PROFILES:
            original = json.loads((probe.BENCHMARK / profile["source"]).read_text())
            before = copy.deepcopy(original)
            scene = probe.build_scene(original, profile, self.assets)
            probe.validate_geometry_preserved(original, scene)
            self.assertEqual(original, before)
            self.assertEqual(scene["egoVehicle"], original["egoVehicle"])
            self.assertNotEqual(scene["spawnPointList"], original["spawnPointList"])
            self.assertTrue(all(not actor["isLaneChange"] for actor in scene["spawnPointList"]))
            self.assertTrue(all(actor["maximumSpawnVehicle"] == 12 for actor in scene["spawnPointList"]))
            generated.append(scene)
        self.assertNotEqual(generated[0], generated[1])

    def test_spawn_binding_is_by_id_not_source_order(self):
        forward = probe.build_scene(self.source, probe.PROFILES[0], self.assets)
        reversed_source = copy.deepcopy(self.source)
        reversed_source["spawnPointList"].reverse()
        reverse = probe.build_scene(reversed_source, probe.PROFILES[0], self.assets)
        self.assertEqual({row["UNIQUEID"]: row for row in forward["spawnPointList"]},
                         {row["UNIQUEID"]: row for row in reverse["spawnPointList"]})

    def test_generate_is_deterministic_and_has_provenance_without_live_claim(self):
        before = {path: path.read_bytes() for path in probe.BENCHMARK.glob("*.json")}
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            first = probe.prepare(root / "one")
            second = probe.prepare(root / "two")
            self.assertEqual(len(first["scenes"]), 3)
            for a, b in zip(first["scenes"], second["scenes"]):
                self.assertEqual(a["sha256"], b["sha256"])
                self.assertEqual(a["sha256"], probe.sha256(a["path"]))
                self.assertEqual(probe.sha256(a["source"]), a["source_sha256"])
                self.assertEqual(probe.sha256(a["mission_configuration_source"]), a["mission_configuration_sha256"])
                self.assertFalse(a["live_observed"])
                self.assertFalse(a["event_occurrence_guaranteed"])
                self.assertFalse(a["actual_reverse_npc"])
                self.assertFalse(a["safety_certified"])
            self.assertTrue(first["preparation_only"])
            self.assertFalse(first["production_changed"])
            self.assertFalse(first["bulk_dataset"])
            with self.assertRaises(FileExistsError):
                probe.prepare(root / "one")
        self.assertEqual(before, {path: path.read_bytes() for path in probe.BENCHMARK.glob("*.json")})

    def test_selective_scene_and_unknown_selection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            plan = probe.prepare(root / "curve", ["ind_roundabout_curve"])
            self.assertEqual([row["mission"] for row in plan["scenes"]], ["roundabout"])
            with self.assertRaises(ValueError):
                probe.prepare(root / "invalid", ["unknown"])
            self.assertFalse((root / "invalid").exists())

    def test_invalid_speed_model_and_period_rejected(self):
        for key, value in (("data_id", -1), ("initial_kmh", -1.), ("initial_kmh", 90.),
                           ("goal_kmh", 81.), ("period_sec", 2.), ("goal_kmh", float("nan"))):
            profile = copy.deepcopy(probe.PROFILES[0])
            profile["spawns"][6][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                probe.build_scene(self.source, profile, self.assets)

    def test_incomplete_extra_and_duplicate_spawns_rejected(self):
        for mode in ("missing", "extra", "duplicate"):
            source = copy.deepcopy(self.source)
            if mode == "missing":
                source["spawnPointList"].pop()
            elif mode == "extra":
                source["spawnPointList"].append(dict(source["spawnPointList"][0], UNIQUEID=999))
            else:
                source["spawnPointList"].append(copy.deepcopy(source["spawnPointList"][0]))
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                probe.build_scene(source, probe.PROFILES[0], self.assets)

    def test_geometry_safety_and_route_edits_rejected(self):
        scene = probe.build_scene(self.source, probe.PROFILES[0], self.assets)
        for field, value in (("pos", dict(x=0., y=0., z=29.)), ("SafetyDist", 0.),
                             ("startLinkInfo", dict(linkIdx="guessed", pointIdx=0)), ("isLaneChange", True)):
            modified = copy.deepcopy(scene)
            modified["spawnPointList"][0][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                probe.validate_geometry_preserved(self.source, modified)

    def test_saved_running_actor_input_rejected(self):
        source = copy.deepcopy(self.source)
        source["vehicleList"] = [dict(DataID=30100007, UNIQUEID=55)]
        with self.assertRaises(ValueError):
            probe.build_scene(source, probe.PROFILES[0], self.assets)


if __name__ == "__main__":
    unittest.main()
